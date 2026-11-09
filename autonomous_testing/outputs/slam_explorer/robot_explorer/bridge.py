"""Single serial owner: telemetry -> ROS sensors; velocity -> vehicle commands."""
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String, Empty, Bool
from std_srvs.srv import SetBool
from .vehicle_control import Control
from .core import telemetry, wrap_delta, steering, require_calibration, encoder_scale,locked_steering_profile
from .retrace import rear_coverage,rear_coverage_details,BRIDGE_REVERSE_DISTANCE,BRIDGE_REVERSE_SECONDS
from .telemetry_view import MotorSensors
from .fusion_math import wheel_variance,combine_wheel_speeds


class VehicleBridge(Node):
    def __init__(self):
        super().__init__('vehicle_bridge')
        self.declare_parameter('calibration','')
        self.declare_parameter('allow_motion',False)
        self.cfg=require_calibration(locked_steering_profile(json.loads(Path(self.get_parameter('calibration').value).read_text())), require_verified=self.get_parameter('allow_motion').value)
        self.allowed=self.get_parameter('allow_motion').value
        import serial
        self.port=serial.Serial(self.cfg['serial_port'],115200,timeout=0,write_timeout=.05,exclusive=True)
        self.port.reset_input_buffer()
        self.c=Control();self.c.settings.update(motor1=self.cfg['motor1_sign'],motor2=self.cfg['motor2_sign'],
            front=self.cfg['front_sign'],rear=self.cfg['rear_sign'],travel=int(self.cfg['servo_travel_us']),
            front_center=int(self.cfg['front_center_us']),rear_center=int(self.cfg['rear_center_us']))
        self.enabled=False;self.buffer=bytearray();self.next_tx=0;self.started=time.monotonic()
        self.last_cmd=0;self.last_scan=0;self.last_ekf=0;self.last_heartbeat=0
        self.command=(0.,0.);self.ekf_speed=0.;self.previous=None;self.previous_time=None
        self.measured_speed=0.;self.integral=0.;self.last_motion=0.;self.motion_since=None
        self.fields={};self.reason='Disarmed';self.last_imu=0;self.scan=None
        self.motor_sensors=MotorSensors(float(self.cfg.get('ina_shunt_ohms',.1)))
        self.retrace_authorized_at=0;self.reverse_start=None;self.reverse_distance=0.;self.reverse_active_seconds=0.;self.reverse_driving=False;self.reverse_limited=False
        self.odom_pub=self.create_publisher(Odometry,'wheel/odometry',10)
        self.imu_pub=self.create_publisher(Imu,'imu/data',10)
        self.status_pub=self.create_publisher(String,'drive/status',10)
        self.create_subscription(TwistStamped,'cmd_vel',self.command_cb,10)
        self.create_subscription(LaserScan,'scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'odometry/filtered',self.ekf_cb,10)
        self.create_subscription(Empty,'operator_heartbeat',self.heartbeat_cb,10)
        self.create_subscription(Bool,'retrace/active',self.retrace_cb,10)
        self.create_service(SetBool,'drive/enable',self.enable_cb)
        self.create_timer(.02,self.tick)
        self.create_timer(.5,self.status)

    def heartbeat_cb(self,msg): self.last_heartbeat=time.monotonic()
    def retrace_cb(self,msg):
        if msg.data:self.retrace_authorized_at=time.monotonic()
        elif abs(self.command[0])<.005 and abs(self.measured_speed)<.02:
            # A completed route at a measured standstill ends this maneuver's
            # budget. Do not accumulate unrelated reverse routes forever.
            # False does not refresh authorization or reset a moving maneuver.
            self.reverse_start=None;self.reverse_distance=0.;self.reverse_active_seconds=0.;self.reverse_driving=False;self.reverse_limited=False
    def command_cb(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        if not -.1<=age<=.3:return
        self.command=(float(msg.twist.linear.x),float(msg.twist.angular.z));self.last_cmd=time.monotonic()
    def scan_cb(self,msg):
        now=self.get_clock().now().nanoseconds/1e9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not -.1<=now-stamp<=.5:return
        if msg.header.frame_id!=self.cfg['laser_frame']:return
        self.scan=msg;self.last_scan=time.monotonic()
    def ekf_cb(self,msg):
        self.last_ekf=time.monotonic();self.ekf_speed=msg.twist.twist.linear.x

    def problem(self):
        now=time.monotonic()
        if self.c.last_telemetry is None or now-self.c.last_telemetry>.4:return 'Serial telemetry stale'
        if int(self.fields.get('CMDERR','1')):return 'Board command fault'
        if now-self.last_imu>.4:return 'IMU invalid/stale'
        if now-self.last_scan>.4:return 'LiDAR stale or wrong frame'
        if now-self.last_ekf>.5:return 'EKF output stale'
        if now-self.last_heartbeat>3.:return 'Operator dashboard heartbeat expired'
        if not self.scan or sum(math.isfinite(v) and v>self.scan.range_min for v in self.scan.ranges)<30:
            return 'Insufficient LiDAR returns'
        return ''

    def stop(self,reason):
        self.c.stop();self.enabled=False;self.integral=0.;self.reason=reason
        self.command=(0.,0.);self.last_cmd=0;self.motion_since=None
        self.reverse_start=None;self.reverse_distance=0.;self.reverse_active_seconds=0.;self.reverse_driving=False;self.reverse_limited=False

    def enable_cb(self,request,response):
        if not request.data:
            if self.enabled or self.c.state!='disarmed':self.stop('Stop requested')
            response.success=True
        else:
            reason=self.problem()
            if not self.allowed:reason='Launch allow_motion:=true only after validation'
            if self.c.state!='disarmed':reason='Already arming/armed; stop first'
            if not reason:
                try:
                    self.c.request_arm(time.monotonic());self.enabled=True
                    self.reason='Arming at zero throttle';self.last_cmd=0;self.integral=0
                except ValueError as e:reason=str(e)
            response.success=not bool(reason)
            response.message=reason or 'Arm requested; wait for acknowledgment'
        return response

    def receive(self,line,now):
        try:f=telemetry(line)
        except (ValueError,KeyError):return
        self.fields=f;self.motor_sensors.update(f,now);self.c.receive(line,now)
        if self.enabled and self.c.state=='disarmed':self.stop('Board disarmed/faulted')
        stamp=self.get_clock().now().to_msg()
        if f.get('VALID')=='1' and int(f.get('ERR','1'))==0:
            self.last_imu=now
            yaw=self.cfg['imu_yaw_sign']*math.radians(float(f['H']))+self.cfg['imu_yaw_offset_rad']
            imu=Imu();imu.header.stamp=stamp;imu.header.frame_id='imu_link'
            imu.orientation.z=math.sin(yaw/2);imu.orientation.w=math.cos(yaw/2)
            imu.orientation_covariance=[.1,0.,0.,0.,.1,0.,0.,0.,.05]
            imu.angular_velocity_covariance[0]=-1.;imu.linear_acceleration_covariance[0]=-1.
            self.imu_pub.publish(imu)
        count=int(f['E'+str(self.cfg['encoder_index'])])
        if self.previous is not None:
            dt=now-self.previous_time
            if .02<=dt<=.4:
                delta=wrap_delta(count,self.previous)*self.cfg['encoder_sign']
                travel=delta/encoder_scale(self.cfg)
                # Counter-steer: mean axle speed = body speed / cos(delta).
                pulse=float(f.get('S1','0'))
                angle=0. if pulse==0 else (pulse-self.cfg['front_center_us'])/(
                    self.cfg['front_sign']*self.cfg['servo_travel_us'])*self.cfg['max_steer_rad']
                angle=max(-self.cfg['max_steer_rad'],min(self.cfg['max_steer_rad'],angle))
                speed=travel/dt*math.cos(angle)
                secondary=self.cfg.get('secondary_encoder');secondary_speed=None
                if secondary and getattr(self,'previous_secondary',None) is not None:
                    other=int(f['E'+str(secondary['index'])])
                    secondary_speed=wrap_delta(other,self.previous_secondary)*secondary['sign']/encoder_scale(secondary)/dt*math.cos(angle)
                speed,disagreement=combine_wheel_speeds(speed,secondary_speed)
                if max(abs(travel/dt*math.cos(angle)),abs(secondary_speed or 0.))>self.cfg['measured_speed_mps']*3:
                    if self.enabled:self.stop('Encoder jump or speed exceeds calibrated range')
                else:
                    self.measured_speed=speed
                    if getattr(self,'reverse_start',None) is not None and speed<0:
                        self.reverse_distance+=abs(speed)*dt
                    if abs(delta)>=1:self.last_motion=now
                    odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom'
                    odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
                    odom.twist.twist.linear.x=speed
                    variance,confidence=wheel_variance(self.motor_sensors.snapshot(self.fields,now),
                        self.cfg.get('load_reference_current_a'))
                    variance+=disagreement
                    odom.twist.covariance[0]=variance;odom.twist.covariance[7]=max(.02,variance*.5)
                    self.wheel_confidence=confidence
                    odom.twist.covariance[35]=.2
                    self.odom_pub.publish(odom)
        self.previous=count;self.previous_time=now
        if self.cfg.get('secondary_encoder'):
            self.previous_secondary=int(f['E'+str(self.cfg['secondary_encoder']['index'])])

    def obstacle_stop(self,v,w):
        self.obstacle_detail=None
        # Conservative swept-disk check in base_link. Scan mounting is calibrated.
        cfg=self.cfg;radius=math.hypot(.3048,.3048)/2+cfg['clearance_margin_m']
        # Braking follows the larger requested/observed speed, including slip
        # or downhill overspeed. Baseline braking scales quadratically.
        speed=max(abs(v),abs(self.measured_speed),abs(self.ekf_speed))
        reach=cfg['stopping_distance_m']*max(1.,(speed/cfg['measured_speed_mps'])**2)+speed*.7
        curvature=0 if abs(v)<.001 else w/v
        for i,r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or not self.scan.range_min<r<self.scan.range_max:continue
            a=self.scan.angle_min+i*self.scan.angle_increment+cfg['laser_yaw_rad']
            px=cfg['laser_x_m']+r*math.cos(a);py=cfg['laser_y_m']+r*math.sin(a)
            if abs(w)<1e-6 and ((v>0 and px<-.3048/2) or (v<0 and px>.3048/2)):
                continue  # Straight forward escape from a rear-only obstruction.
            samples=max(5,math.ceil(reach/.04))
            for j in range(samples+1):
                s=math.copysign(reach*j/samples,v)
                x=s if abs(curvature)<1e-6 else math.sin(curvature*s)/curvature
                y=0 if abs(curvature)<1e-6 else (1-math.cos(curvature*s))/curvature
                if math.hypot(px-x,py-y)<radius:
                    self.obstacle_detail=dict(x=round(px,3),y=round(py,3),range_m=round(r,3),
                        checked_speed=round(v,3),checked_yaw_rate=round(w,3))
                    return True
        return False

    def account_reverse(self,now):
        previous=getattr(self,'reverse_clock',now)
        if getattr(self,'reverse_driving',False):
            # MCU output expires after 250 ms if this process stalls.
            self.reverse_active_seconds=getattr(self,'reverse_active_seconds',0.)+min(.25,max(0.,now-previous))
        self.reverse_clock=now

    def tick(self):
        now=time.monotonic();self.account_reverse(now)
        try:
            self.buffer.extend(self.port.read(min(self.port.in_waiting,4096)))
            if len(self.buffer)>8192:raise RuntimeError('Serial framing overflow')
            latest=None
            while b'\n' in self.buffer:
                raw,_,self.buffer=self.buffer.partition(b'\n')
                line=raw.decode('ascii',errors='replace').strip()
                try:telemetry(line);latest=line
                except (ValueError,KeyError):pass
            if latest is not None:self.receive(latest,now)
            event=self.c.check_link(now)
            if event:self.stop(event)
            if self.enabled:
                reason=self.problem()
                if reason:self.stop(reason)
                elif self.c.state=='armed':
                    v,w=self.command
                    if now-self.last_cmd>.3:v=w=0.
                    if not math.isfinite(v+w) or abs(v)>self.cfg['measured_speed_mps']:
                        self.stop('Unsupported/nonfinite velocity command')
                    elif v<0 and now-getattr(self,'retrace_authorized_at',0)>.3:
                        self.stop('Reverse requires active bounded recovery authorization')
                    elif getattr(self,'reverse_limited',False):
                        self.c.drive(0);self.integral=0;self.motion_since=None
                        self.reason='Reverse maneuver limit reached; waiting for replan'
                    elif v<0 and not rear_coverage(self.scan,self.cfg,v,w):
                        self.c.drive(0);self.integral=0;self.motion_since=None
                        detail=rear_coverage_details(self.scan,self.cfg,v,w)
                        self.reason=('Reverse unavailable: need at least 1 of 20 rear sectors ('+
                            str(detail['known_bins'])+'/'+str(detail['required_bins'])+' sectors; missing '+
                            ','.join(str(a) for a in detail['missing_degrees'])+' deg)')
                    elif abs(v)<.005:
                        self.c.drive(0);self.c.steer(0);self.integral=0;self.motion_since=None
                        self.reason='Armed; waiting for navigation command'
                    else:
                        if v<0:
                            if getattr(self,'reverse_start',None) is None:self.reverse_start=now;self.reverse_distance=0.
                            if getattr(self,'reverse_active_seconds',0.)>=BRIDGE_REVERSE_SECONDS or self.reverse_distance>=BRIDGE_REVERSE_DISTANCE:
                                self.last_reverse_limit=dict(active_seconds=getattr(self,'reverse_active_seconds',0.),
                                    distance_m=self.reverse_distance,cause='distance' if self.reverse_distance>=BRIDGE_REVERSE_DISTANCE else 'active time')
                                self.reverse_limited=True;self.reverse_driving=False;self.c.drive(0);self.integral=0
                                self.command=(0.,0.);self.last_cmd=0
                                self.reason='Reverse maneuver limit reached; waiting for replan'
                                self.port.write(self.c.next_frame());self.next_tx=now+.1
                                return
                        angle=steering(v,w,self.cfg['wheelbase_m'],self.cfg['max_steer_rad'])
                        applied_w=2*v*math.tan(angle)/self.cfg['wheelbase_m']
                        limited=abs(applied_w-w)>.001
                        if self.obstacle_stop(v,applied_w):
                            self.c.drive(0);self.integral=0;self.motion_since=None
                            detail=getattr(self,'obstacle_detail',None)
                            self.reason=('Obstacle stop at x='+str(detail['x'])+' m, y='+str(detail['y'])+' m'
                                         if detail else 'Obstacle stop; waiting for Nav2 replan')
                        else:
                            if self.motion_since is None:self.motion_since=now
                            if now-max(self.motion_since,self.last_motion)>1.5:
                                self.stop('Working encoder has no motion; explicit restart required')
                            else:
                                direction=1 if v>0 else -1
                                error=abs(v)-direction*self.measured_speed
                                self.integral=max(-.5,min(.5,self.integral+error*.02))
                                duty=self.cfg['measured_duty_pct']*abs(v)/self.cfg['measured_speed_mps']
                                duty+=20*error+5*self.integral
                                self.c.steer(angle/self.cfg['max_steer_rad'])
                                self.c.drive(direction*max(0.,min(self.cfg.get('max_duty_pct',100.),duty)))
                                self.reason=('Steering capped at calibrated limit; following path'
                                             if limited else 'Following Nav2 path')
            self.reverse_driving=bool(self.enabled and self.c.state=='armed' and self.command[0]<-.005
                and self.c.duty*self.c.settings['motor1']<0)
            if now>=self.next_tx:
                packet=self.c.next_frame()
                if self.port.write(packet)!=len(packet):raise RuntimeError('Short serial write')
                self.next_tx=now+.1
        except Exception as e:
            self.stop(str(e))
            try:self.port.write(self.c.next_frame())
            except Exception:pass

    def status(self):
        now=time.monotonic()
        msg=String();msg.data=json.dumps(dict(enabled=self.enabled,state=self.c.state,
            reason=self.reason,health=self.problem(),speed=self.measured_speed,
            confirmed_disarmed=self.c.confirmed_disarmed,
            reverse_budget=dict(active_seconds=getattr(self,'reverse_active_seconds',0.),distance_m=self.reverse_distance,
                max_active_seconds=BRIDGE_REVERSE_SECONDS,max_distance_m=BRIDGE_REVERSE_DISTANCE,
                limited=getattr(self,'reverse_limited',False),last_limit=getattr(self,'last_reverse_limit',None)),
            encoder_index=self.cfg['encoder_index'],requested_speed=self.command[0],
            max_duty_pct=self.cfg.get('max_duty_pct',100.),
            wheel_confidence=getattr(self,'wheel_confidence','Waiting'),
            applied_duty=[self.c.duty,self.c.duty2],obstacle=getattr(self,'obstacle_detail',None),
            sensors=self.motor_sensors.snapshot(self.fields,now),
            steering_pulses=self.c.pulses(),steering_limits=dict(min_us=1000,straight_us=1500,max_us=2000),
            sensor_ages={name:(now-stamp if stamp else None) for name,stamp in
                         [('lidar',self.last_scan),('imu',self.last_imu),('ekf',self.last_ekf)]},
            rear_coverage=(rear_coverage_details(self.scan,self.cfg,self.command[0],self.command[1])
                           if self.scan else None),fields=self.fields))
        self.status_pub.publish(msg)

    def close(self):
        self.stop('Shutdown')
        for _ in range(3):
            try:self.port.write(self.c.next_frame());self.port.flush();time.sleep(.1)
            except Exception:break
        self.port.close()


def main():
    rclpy.init();node=None
    try:node=VehicleBridge();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
