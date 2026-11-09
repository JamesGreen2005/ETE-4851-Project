"""Current-footprint overlay shared by exploration and Nav2; raw SLAM retained."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from .footprint import footprint_known_map,observed_rays


class NavigationMap(Node):
    def __init__(self):
        super().__init__('navigation_map');self.map=None;self.received=0.;self.scan=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.create_publisher(OccupancyGrid,'navigation/map',qos)
        self.create_subscription(OccupancyGrid,'map',self.receive,qos)
        self.create_subscription(LaserScan,'scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_timer(.5,self.publish_map)

    def receive(self,msg):self.map=msg;self.received=time.monotonic()

    def publish_map(self):
        if self.map is None or time.monotonic()-self.received>10:return
        try:
            msg=self.map
            if msg.header.frame_id!='map' or abs(msg.info.origin.orientation.z)>.001:return
            tr=self.tf.lookup_transform('map','base_link',Time())
            stamp=tr.header.stamp.sec+tr.header.stamp.nanosec/1e9
            if not -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=1.:return
            t=tr.transform;q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            data=msg.data
            if self.scan is not None:
                scan=self.scan;stamp=scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
                if -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=.4:
                    mount=self.tf.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
                    rot=mount.rotation;angle=math.atan2(2*(rot.w*rot.z+rot.x*rot.y),1-2*(rot.y*rot.y+rot.z*rot.z))
                    source=(mount.translation.x,mount.translation.y);ends=[]
                    for i,r in enumerate(scan.ranges):
                        if math.isfinite(r) and scan.range_min<r<min(scan.range_max,8.):
                            a=angle+scan.angle_min+i*scan.angle_increment
                            ends.append((source[0]+r*math.cos(a),source[1]+r*math.sin(a)))
                    data=observed_rays(data,msg.info.width,msg.info.height,msg.info.resolution,
                        (msg.info.origin.position.x,msg.info.origin.position.y),source,ends)
            data,_=footprint_known_map(data,msg.info.width,msg.info.height,msg.info.resolution,
                (msg.info.origin.position.x,msg.info.origin.position.y),(t.translation.x,t.translation.y,yaw))
            out=copy.deepcopy(msg);out.data=data;self.map_pub.publish(out)
        except Exception as e:self.get_logger().debug(str(e))


def main():
    rclpy.init();node=NavigationMap()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
