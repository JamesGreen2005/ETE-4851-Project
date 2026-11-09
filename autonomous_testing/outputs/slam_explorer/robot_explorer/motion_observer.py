"""LiDAR scan odometry and optional calibrated ground-plane camera odometry."""
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from .fusion_math import scan_motion,rigid_fit,motion_twist


def camera_calibration(data):
    """H maps UNDISTORTED native pixels onto ground coordinates in base_link metres."""
    k=np.asarray(data['camera_matrix'],float);d=np.asarray(data['distortion'],float)
    h=np.asarray(data['ground_homography'],float);roi=np.asarray(data['ground_roi'],np.int32)
    if not data.get('verified') or k.shape!=(3,3) or h.shape!=(3,3) or roi.ndim!=2 or roi.shape[1]!=2 or len(roi)<3:
        raise ValueError('Verified intrinsics, metric ground homography and ground ROI required')
    if not all(np.isfinite(v).all() for v in (k,d,h)) or abs(np.linalg.det(h))<1e-12 or abs(np.linalg.det(k))<1e-12:
        raise ValueError('Invalid camera calibration')
    if int(data['width'])<=0 or int(data['height'])<=0:raise ValueError('Invalid camera size')
    return k,d,h,roi


class MotionObserver(Node):
    def __init__(self):
        super().__init__('motion_observer');self.declare_parameter('calibration','')
        self.cfg=json.loads(Path(self.get_parameter('calibration').value).read_text())
        self.cv=CvBridge();self.previous_scan=None;self.previous_images={};self.last_work={}
        self.health={'lidar':{'state':'waiting'},'front':{'state':'disabled: camera calibration required'},
                     'rear':{'state':'disabled: camera calibration required'}}
        self.motion_publishers={name:self.create_publisher(Odometry,name+'/odometry',5) for name in ('lidar','front','rear')}
        self.status_pub=self.create_publisher(String,'fusion/status',5)
        self.create_subscription(LaserScan,'scan',self.scan_cb,qos_profile_sensor_data)
        self.calibrations={}
        for role in ('front','rear'):
            path=self.cfg.get(role+'_odometry_calibration')
            if not path:continue
            try:
                data=json.loads(Path(path).expanduser().read_text());camera_calibration(data)
                self.calibrations[role]=data;self.health[role]={'state':'waiting'}
                self.create_subscription(Image,'/camera/image_raw' if role=='front' else '/camera/rear/image_raw',
                    lambda m,role=role:self.image_cb(m,role),qos_profile_sensor_data)
            except Exception as e:raise ValueError(role+' camera calibration: '+str(e)) from e
        self.create_timer(.5,self.status)

    def status(self):
        now=time.monotonic();data={}
        for name,h in self.health.items():
            data[name]={k:v for k,v in h.items() if k!='received'}
            age=now-h['received'] if 'received' in h else None
            data[name]['age_s']=age
            if age is not None and age>.6:data[name]['state']='stale: excluded from new updates'
        msg=String();msg.data=json.dumps(data);self.status_pub.publish(msg)

    def stamp(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not -.05<=self.get_clock().now().nanoseconds/1e9-stamp<=.35:raise ValueError('Stale sensor frame')
        return stamp

    def publish(self,name,msg,r,t,dt,variance,quality):
        vx,vy,w=motion_twist(r,t,dt)
        out=Odometry();out.header.stamp=msg.header.stamp;out.header.frame_id='odom';out.child_frame_id='base_link'
        out.pose.pose.orientation.w=1.;out.twist.twist.linear.x=vx;out.twist.twist.linear.y=vy;out.twist.twist.angular.z=w
        # Floors cover scan correlation / scene-model errors; not fitted precision alone.
        for i in (0,7):out.twist.covariance[i]=max(.04,variance/dt**2)
        out.twist.covariance[35]=max(.08,variance/dt**2)
        for i in (14,21,28):out.twist.covariance[i]=1e6
        self.motion_publishers[name].publish(out)
        self.health[name]=dict(state='accepted',quality=float(quality),received=time.monotonic())

    def scan_cb(self,msg):
        try:
            stamp=self.stamp(msg)
            if msg.header.frame_id!=self.cfg['laser_frame']:raise ValueError('Wrong LiDAR frame')
            if stamp-self.last_work.get('lidar',0)<.09:return
            self.last_work['lidar']=stamp
            ranges=np.asarray(msg.ranges);angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment+self.cfg['laser_yaw_rad']
            keep=np.isfinite(ranges)&(ranges>max(.15,msg.range_min))&(ranges<min(8.,msg.range_max))
            points=np.column_stack((ranges[keep]*np.cos(angles[keep])+self.cfg['laser_x_m'],
                                    ranges[keep]*np.sin(angles[keep])+self.cfg['laser_y_m']))
            points=points[::max(1,math.ceil(len(points)/240))]
            previous=self.previous_scan;self.previous_scan=(stamp,points)
            if previous is None:return
            dt=stamp-previous[0]
            if not .04<=dt<=.5:raise ValueError('LiDAR frame gap')
            r,t,variance,quality=scan_motion(points,previous[1]);self.publish('lidar',msg,r,t,dt,variance,quality)
        except Exception as e:self.health['lidar']={'state':'rejected: '+str(e)}

    def image_cb(self,msg,role):
        try:
            stamp=self.stamp(msg)
            if stamp-self.last_work.get(role,0)<.18:return
            self.last_work[role]=stamp;data=self.calibrations[role];k,d,h,roi=camera_calibration(data)
            if msg.width!=data['width'] or msg.height!=data['height']:raise ValueError('Image calibration size mismatch')
            gray=self.cv.imgmsg_to_cv2(msg,'mono8');old=self.previous_images.get(role)
            self.previous_images[role]=(stamp,gray)
            if old is None:return
            dt=stamp-old[0]
            if not .04<=dt<=.5:raise ValueError('Camera frame gap')
            mask=np.zeros_like(gray);cv2.fillPoly(mask,[roi],255)
            p=cv2.goodFeaturesToTrack(old[1],180,.02,8,mask=mask)
            if p is None or len(p)<20:raise ValueError('Too few ground features')
            q,ok,_=cv2.calcOpticalFlowPyrLK(old[1],gray,p,None)
            back,ok_back,_=cv2.calcOpticalFlowPyrLK(gray,old[1],q,None)
            good=(ok.ravel()>0)&(ok_back.ravel()>0)&(np.linalg.norm(back-p,axis=2).ravel()<1.)
            pixels=q.reshape(-1,2);ix=np.rint(pixels[:,0]).astype(int);iy=np.rint(pixels[:,1]).astype(int)
            inside=(ix>=0)&(ix<msg.width)&(iy>=0)&(iy<msg.height)
            good&=inside
            good[inside]&=mask[iy[inside],ix[inside]]>0
            if good.sum()<20:raise ValueError('Ground feature tracking failed')
            def ground(points):
                undistorted=cv2.undistortPoints(points,k,d,P=k)
                return cv2.perspectiveTransform(undistorted,h).reshape(-1,2)
            before=ground(p[good]);after=ground(q[good])
            affine,inliers=cv2.estimateAffinePartial2D(after,before,method=cv2.RANSAC,ransacReprojThreshold=.03)
            if affine is None or inliers.sum()<20 or inliers.mean()<.65:raise ValueError('Ground motion consensus failed')
            scale=np.linalg.norm(affine[:,0])
            if not .97<=scale<=1.03:raise ValueError('Nonrigid scene / ground model failed')
            selected=inliers.ravel()>0;r,t,residual=rigid_fit(after[selected],before[selected])
            if residual>.03:raise ValueError('Visual motion residual too high')
            self.publish(role,msg,r,t,dt,max(.001,residual**2),float(inliers.mean()))
        except Exception as e:self.health[role]={'state':'rejected: '+str(e)}


def main():
    rclpy.init();node=MotionObserver()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
