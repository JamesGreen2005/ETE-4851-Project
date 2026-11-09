"""Frontier exploration with goal commitment and bounded failure memory."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from std_msgs.msg import String,Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer,TransformListener
from .core import frontier_goals,choose_goal
from .route_planner import viewpoint_goals
from .retrace import Trail,reverse_command,short_backup,MAX_RETRACE_SECONDS,RouteDeviation,straight_reverse_command


class Explorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('behavior_tree','')
        self.declare_parameter('clearance',.38)
        self.declare_parameter('retrace_speed',.15)
        self.declare_parameter('wheelbase',.2413)
        self.declare_parameter('max_steer',.261799)
        self.bt=self.get_parameter('behavior_tree').value
        self.map=None;self.map_time=0;self.enabled=False;self.goal_handle=None;self.pending=False;self.starting=False
        self.generation=0;self.target=None;self.started=0;self.memory=[];self.failures={}
        self.reason='Stopped';self.drive={};self.drive_time=0;self.finished=False
        self.last_stop=None
        self.waiting_for_arm=False;self.arm_requested_at=0;self.arm_deadline=0
        self.trail=Trail();self.odom_pose=None;self.odom_time=0;self.reversing=False
        self.reverse_path=[];self.reverse_started=0;self.want_retrace=False;self.blocked_since=None
        self.recovery_used=False;self.recovery_anchor=None
        self.reverse_straight=False;self.reverse_anchor=None;self.last_recovery_event=None
        self.replan_after=0.;self.forward_retry_once=False
        self.visits=[];self.planning_diagnostics={}
        self.limit_replan_pending=False;self.resetting=False;self.reset_requested_at=0.;self.action_requests=set();self.action_results=set()
        self.reverse_cmd=self.create_publisher(TwistStamped,'cmd_vel',10)
        self.reverse_active=self.create_publisher(Bool,'retrace/active',10)
        self.create_subscription(Odometry,'odometry/filtered',self.odom_cb,10)
        self.create_timer(.05,self.reverse_tick)
        self.create_timer(.1,self.navigation_reverse_authorization)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.action=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        self.plan_handle=None;self.waiting_plan=False
        self.motor=self.create_client(SetBool,'drive/enable')
        self.status_pub=self.create_publisher(String,'explore/status',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'navigation/map',self.map_cb,qos)
        self.create_subscription(String,'drive/status',self.drive_cb,10)
        self.create_service(SetBool,'explore/enable',self.enable_cb)
        self.create_service(SetBool,'explore/reset',self.reset_cb)
        self.create_timer(1.,self.tick)

    def map_cb(self,msg): self.map=msg;self.map_time=time.monotonic()
    def drive_cb(self,msg): self.drive=json.loads(msg.data);self.drive_time=time.monotonic()

    def odom_cb(self,msg):
        if msg.header.frame_id!='odom':return
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        now=time.monotonic();previous=self.odom_pose
        self.odom_pose=(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,now)
        self.odom_time=now
        if not getattr(self,'visits',[]) or math.dist(self.visits[-1],self.odom_pose[:2])>.15:
            self.visits=(getattr(self,'visits',[])+[self.odom_pose[:2]])[-200:]
        if (self.reversing and previous and now-previous[3]<=.4
            and math.dist(previous[:2],self.odom_pose[:2])>.35):
            self.halt('Odometry jumped during reverse');return
        if (self.recovery_anchor is not None and not self.reversing
            and msg.twist.twist.linear.x>.02
            and math.hypot(self.odom_pose[0]-self.recovery_anchor[0],
                           self.odom_pose[1]-self.recovery_anchor[1])>=.3):
            self.recovery_used=False;self.recovery_anchor=None
        if not self.reversing and msg.twist.twist.linear.x>.02:self.trail.add(*self.odom_pose)

    def start_retrace(self):
        if not self.enabled or self.pending or self.goal_handle or self.reversing:return False
        self.want_retrace=False
        if self.recovery_used:
            self.reason='Recovery used; waiting for a clear forward route';return False
        if not self.odom_pose or time.monotonic()-self.odom_time>.4:return False
        path=self.trail.reverse(self.odom_pose,time.monotonic())
        recorded=bool(path)
        if not path:path=short_backup(self.odom_pose)
        if not path:return False
        self.recovery_used=True;self.recovery_anchor=self.odom_pose
        self.reverse_path=path;self.reversing=True;self.reverse_started=time.monotonic()
        self.reverse_straight=not recorded;self.reverse_anchor=self.odom_pose
        self.want_retrace=False
        self.reason=('Retracing recent route, up to 2 m' if recorded else
                     'Straight reverse recovery, up to 1 m; rear LiDAR monitored')
        return True

    def navigation_reverse_authorization(self):
        # Reeds-Shepp routes may contain either direction. The bridge still
        # requires fresh authorization, rear coverage and collision clearance.
        if not self.reversing:
            active=Bool();active.data=bool(self.enabled and self.goal_handle)
            self.reverse_active.publish(active)

    def finish_recovery(self,reason):
        stop=TwistStamped();stop.header.stamp=self.get_clock().now().to_msg()
        stop.header.frame_id='base_link';self.reverse_cmd.publish(stop)
        active=Bool();active.data=False;self.reverse_active.publish(active)
        self.reversing=False;self.reverse_straight=False;self.reverse_path=[]
        self.reverse_anchor=None;self.want_retrace=False;self.blocked_since=None
        self.trail.points.clear();self.recovery_anchor=self.odom_pose
        self.replan_after=time.monotonic()+.6
        self.forward_retry_once=True
        self.last_recovery_event=reason+'; checking a forward route'
        self.reason=self.last_recovery_event

    def reverse_tick(self):
        if not self.reversing:return
        now=time.monotonic()
        try:
            if (not self.enabled or now-self.odom_time>.4 or now-self.drive_time>1.
                or self.drive.get('health') or self.drive.get('state')!='armed'):
                raise ValueError('Retrace stopped: stale sensors, fault or disarm')
            if self.drive.get('reverse_budget',{}).get('active_seconds',0.)>=MAX_RETRACE_SECONDS:
                self.finish_recovery('Reverse recovery time limit reached');return
            if str(self.drive.get('reason','')).startswith(('Reverse blocked','Reverse unavailable','Obstacle stop','Reverse maneuver limit')) and now-self.reverse_started>1.:
                self.finish_recovery('Reverse recovery blocked: '+str(self.drive.get('reason')));return
            if now-self.reverse_started<.6:v=w=0.;done=False
            elif now<getattr(self,'reverse_center_until',0.):v=w=0.;done=False
            else:
                speed=self.get_parameter('retrace_speed').value
                if self.reverse_straight:
                    v,w,done=straight_reverse_command(self.reverse_anchor,self.odom_pose,speed)
                else:
                    try:
                        v,w,done=reverse_command(self.reverse_path,self.odom_pose,speed,
                            self.get_parameter('wheelbase').value,self.get_parameter('max_steer').value)
                    except RouteDeviation as error:
                        self.reverse_straight=True;self.reverse_anchor=self.odom_pose
                        self.reverse_center_until=now+.4
                        self.last_recovery_event=str(error)+'; switching to straight reverse'
                        self.reason=self.last_recovery_event
                        # First center steering at zero throttle; the original
                        # recovery timer and bridge distance bound are not reset.
                        v=w=0.;done=False
            msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg()
            msg.header.frame_id='base_link';msg.twist.linear.x=v;msg.twist.angular.z=w
            active=Bool();active.data=not done;self.reverse_active.publish(active);self.reverse_cmd.publish(msg)
            if done:
                self.finish_recovery('Reverse route ended')
        except Exception as e:self.halt(str(e))

    def halt(self,reason):
        self.last_stop=dict(reason=reason,at_unix=time.time(),odom_pose=self.odom_pose,
                            reverse_points=len(self.reverse_path))
        self.generation+=1;self.enabled=False;self.starting=False;self.reason=reason;self.finished=False
        self.waiting_for_arm=False;self.pending=False;self.resetting=False;self.limit_replan_pending=False
        self.reversing=False;self.want_retrace=False;self.reverse_path=[];self.blocked_since=None
        if rclpy.ok():
            stop=TwistStamped();stop.header.stamp=self.get_clock().now().to_msg()
            self.reverse_cmd.publish(stop);active=Bool();active.data=False;self.reverse_active.publish(active)
        if self.goal_handle:self.goal_handle.cancel_goal_async()
        if self.plan_handle:self.plan_handle.cancel_goal_async()
        self.plan_handle=None;self.waiting_plan=False
        self.goal_handle=None;self.target=None
        if rclpy.ok() and self.motor.service_is_ready():
            req=SetBool.Request();req.data=False;self.motor.call_async(req)

    def enable_cb(self,req,res):
        if not req.data:
            self.halt('Operator stop');res.success=True;return res
        if getattr(self,'resetting',False):res.message='Reset in progress; wait for Ready';return res
        if getattr(self,'action_requests',set()) or getattr(self,'action_results',set()):
            res.message='Waiting for old navigation actions to finish cancelling';return res
        if self.enabled or self.pending or self.starting:res.message='Already active or awaiting action cancellation';return res
        if not self.map or time.monotonic()-self.map_time>10:res.message='No fresh SLAM map';return res
        if not self.action.server_is_ready() or not self.motor.service_is_ready():
            res.message='Navigation/drive service not ready';return res
        # Async service call: dashboard observes status rather than assuming arm succeeded.
        self.starting=True
        request=SetBool.Request();request.data=True;future=self.motor.call_async(request)
        generation=self.generation
        def armed(f):
            try:
                result=f.result()
                if generation!=self.generation:return
                self.starting=False
                self.enabled=result.success;self.reason=result.message or 'Exploring'
                self.waiting_for_arm=result.success
                self.arm_requested_at=time.monotonic();self.arm_deadline=self.arm_requested_at+3.
            except Exception as e:
                if generation==self.generation:self.halt(str(e))
        future.add_done_callback(armed)
        res.success=True;res.message='Start requested; inspect drive/exploration status'
        return res

    def reset_cb(self,req,res):
        if not req.data:res.message='Reset requires true';return res
        if getattr(self,'resetting',False):
            res.success=True;res.message='Reset already in progress';return res
        self.halt('Dashboard reset requested')
        self.memory.clear();self.failures.clear();self.visits.clear();self.trail.points.clear()
        self.recovery_used=False;self.recovery_anchor=None;self.reverse_straight=False
        self.reverse_anchor=None;self.reverse_started=0.;self.reverse_center_until=0.
        self.replan_after=0.;self.forward_retry_once=False;self.planning_diagnostics={}
        self.arm_requested_at=0.;self.arm_deadline=0.;self.started=0.
        self.reset_requested_at=time.monotonic();self.resetting=True
        self.reason='Resetting: cancelling navigation and waiting for confirmed motor stop'
        res.success=True;res.message=self.reason;self.publish_status();return res

    def reset_tick(self):
        if self.action_requests or self.action_results:
            self.reason='Resetting: waiting for old navigation actions to finish cancelling';return
        if self.drive_time<=self.reset_requested_at or time.monotonic()-self.drive_time>1.:
            self.reason='Resetting: waiting for fresh drive telemetry';return
        if not self.drive.get('confirmed_disarmed') or self.drive.get('state')!='disarmed' or any(abs(v)>.01 for v in self.drive.get('applied_duty',[1,1])):
            self.reason='Resetting: waiting for confirmed motor stop';return
        if abs(self.drive.get('speed',1.))>.02:
            self.reason='Resetting: waiting for vehicle to stop';return
        if self.drive.get('health'):
            self.reason='Reset stopped: '+str(self.drive['health'])+'; fix the cause, then wait';return
        if not self.map or time.monotonic()-self.map_time>10:
            self.reason='Resetting: waiting for a fresh SLAM map';return
        if not self.action.server_is_ready() or not self.planner.server_is_ready() or not self.motor.service_is_ready():
            self.reason='Resetting: waiting for navigation and drive services';return
        self.resetting=False;self.reason='Reset complete. Ready: press Start exploration'

    def failure(self,goal,reason):
        key=(round(goal['x']*5),round(goal['y']*5))
        n=self.failures.get(key,0)+1;self.failures[key]=n
        self.memory.append(dict(x=goal['x'],y=goal['y'],until=time.monotonic()+min(20,2*2**min(n,3))))
        self.reason=reason

    def send(self,goal):
        generation=self.generation;self.target=goal;self.pending=True;self.started=time.monotonic()
        req=NavigateToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=self.get_clock().now().to_msg()
        req.pose.pose.position.x=goal['x'];req.pose.pose.position.y=goal['y']
        req.pose.pose.orientation.z=math.sin(goal['yaw']/2);req.pose.pose.orientation.w=math.cos(goal['yaw']/2)
        req.behavior_tree=self.bt
        def accepted(f):
            self.action_requests.discard(f)
            try:
                h=f.result()
                if generation!=self.generation:
                    if h.accepted:self.retire_handle(h)
                    return
                if not h.accepted:
                    self.pending=False;self.failure(goal,'Goal rejected; trying another frontier');self.target=None
                    return
                self.goal_handle=h;self.reason='Following steering-feasible route'
                result_future=h.get_result_async();self.action_results.add(result_future)
                def done(result):
                    self.action_results.discard(result)
                    if generation!=self.generation:return
                    self.pending=False
                    if getattr(self,'limit_replan_pending',False):
                        self.limit_replan_pending=False;self.goal_handle=None;self.target=None
                        self.finish_recovery('Reverse maneuver limit reached');return
                    try:status=result.result().status
                    except Exception as e:self.halt('Navigation result failed: '+str(e));return
                    if status!=4:self.failure(goal,'Path failed; frontier temporarily avoided')
                    else:
                        self.memory.append(dict(x=goal['x'],y=goal['y'],until=time.monotonic()+20))
                        self.reason='Frontier reached; selecting next'
                    self.goal_handle=None;self.target=None
                    if status!=4 or self.want_retrace:self.start_retrace()
                result_future.add_done_callback(done)
            except Exception as e:
                if generation==self.generation:self.halt(str(e))
        try:request_future=self.action.send_goal_async(req)
        except Exception as e:self.halt('Navigation request failed: '+str(e));return
        self.action_requests.add(request_future)
        request_future.add_done_callback(accepted)

    def probe(self,goal):
        """Ask the kinematic planner before committing a frontier to control."""
        if not self.planner.server_is_ready():
            self.reason='Waiting for path planner';return
        generation=self.generation;self.pending=True;self.waiting_plan=True
        self.target=goal;self.started=time.monotonic();self.reason='Checking steering-feasible route'
        req=ComputePathToPose.Goal();req.goal.header.frame_id='map'
        req.goal.header.stamp=self.get_clock().now().to_msg()
        req.goal.pose.position.x=goal['x'];req.goal.pose.position.y=goal['y']
        req.goal.pose.orientation.z=math.sin(goal['yaw']/2);req.goal.pose.orientation.w=math.cos(goal['yaw']/2)
        req.planner_id='GridBased';req.use_start=False
        def accepted(f):
            self.action_requests.discard(f)
            try:
                h=f.result()
                if generation!=self.generation:
                    if h.accepted:self.retire_handle(h)
                    return
                if not h.accepted:
                    self.pending=False;self.waiting_plan=False;self.target=None
                    self.failure(goal,'Planner rejected frontier');return
                self.plan_handle=h
                result_future=h.get_result_async();self.action_results.add(result_future)
                def complete(future):
                    self.action_results.discard(future)
                    if generation!=self.generation:return
                    self.pending=False;self.waiting_plan=False;self.plan_handle=None
                    try:result=future.result()
                    except Exception as e:self.halt('Planner result failed: '+str(e));return
                    if result.status==4 and len(result.result.path.poses)>1:self.send(goal)
                    else:
                        self.failure(goal,'Frontier unreachable: '+str(result.result.error_msg))
                        self.target=None
                result_future.add_done_callback(complete)
            except Exception as e:
                if generation==self.generation:self.halt(str(e))
        try:request_future=self.planner.send_goal_async(req)
        except Exception as e:self.halt('Planner request failed: '+str(e));return
        self.action_requests.add(request_future)
        request_future.add_done_callback(accepted)

    def retire_handle(self,handle):
        # An old request may be accepted AFTER Stop/Reset. Cancel it and fence
        # Start until its terminal result arrives; it must not touch new state.
        future=handle.get_result_async();self.action_results.add(future)
        future.add_done_callback(lambda f:self.action_results.discard(f))
        handle.cancel_goal_async()

    def tick(self):
        if getattr(self,'resetting',False):
            self.reset_tick();self.publish_status();return
        now=time.monotonic()
        obstacle=self.drive.get('obstacle') or {}
        front_blocked=(str(self.drive.get('reason','')).startswith('Obstacle stop')
                       or obstacle.get('checked_speed',0)>0)
        if not front_blocked:self.blocked_since=None
        if self.enabled and self.waiting_for_arm:
            fresh=self.drive_time >= self.arm_requested_at
            if fresh and self.drive.get('state')=='armed':
                self.waiting_for_arm=False
            elif now>=self.arm_deadline:
                self.halt('Arm acknowledgment timeout: '+str(self.drive.get('reason','No drive status')))
            else:
                self.reason='Waiting for fresh arm acknowledgment'
                self.publish_status()
                return
        if self.enabled:
            if now-self.map_time>10:
                self.halt('SLAM map stale; restart explicitly')
            elif now-self.drive_time>1.5 or self.drive.get('health'):
                self.halt('Drive/sensors unavailable: '+str(self.drive.get('health','No drive status')))
            elif self.drive.get('state')=='disarmed':self.halt('Drive disarmed: '+str(self.drive.get('reason','Unknown cause')))
            elif str(self.drive.get('reason','')).startswith('Reverse maneuver limit'):
                if self.reversing:self.finish_recovery('Reverse maneuver limit reached')
                elif self.goal_handle:
                    if not getattr(self,'limit_replan_pending',False):
                        self.limit_replan_pending=True;self.goal_handle.cancel_goal_async()
                    self.reason='Ending reverse route before replanning'
                elif not self.pending:self.finish_recovery('Reverse maneuver limit reached')
            elif self.reversing:
                self.reason='Reverse recovery; rear LiDAR clearance monitored'
            elif now<getattr(self,'replan_after',0.):
                self.reason='Pausing at zero throttle before forward planning'
            elif self.goal_handle and front_blocked:
                if self.blocked_since is None:self.blocked_since=now
                if now-self.blocked_since>=1 and not self.want_retrace:
                    self.want_retrace=True;self.reason='Cancelling blocked goal before retracing'
                    self.goal_handle.cancel_goal_async()
            elif self.waiting_plan and now-self.started>10:
                self.halt('Planner response timeout')
            elif self.goal_handle and now-self.started>90:
                self.failure(self.target,'Goal timeout');self.halt('Goal timeout; inspect robot and restart')
            elif not self.pending:
                if now-self.map_time>10:self.halt('SLAM map stale')
                else:
                    try:
                        tr=self.tf.lookup_transform('map','base_link',Time())
                        age=self.get_clock().now().nanoseconds/1e9-(tr.header.stamp.sec+tr.header.stamp.nanosec/1e9)
                        if age>1.:raise RuntimeError('Map pose stale')
                        grid=self.map;origin=grid.info.origin
                        if abs(origin.orientation.z)>.001:raise RuntimeError('Rotated grid origin unsupported')
                        robot=(tr.transform.translation.x,tr.transform.translation.y)
                        self.memory=[m for m in self.memory if m['until']>now]
                        rotation=tr.transform.rotation
                        yaw=math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                            1-2*(rotation.y*rotation.y+rotation.z*rotation.z))
                        visits=[]
                        if self.odom_pose:
                            offset=yaw-self.odom_pose[2];co,si=math.cos(offset),math.sin(offset)
                            for vx,vy in getattr(self,'visits',[]):
                                dx,dy=vx-self.odom_pose[0],vy-self.odom_pose[1]
                                visits.append((robot[0]+co*dx-si*dy,robot[1]+si*dx+co*dy))
                        goals,self.planning_diagnostics=viewpoint_goals(grid.data,grid.info.width,grid.info.height,
                            grid.info.resolution,(origin.position.x,origin.position.y),(*robot,yaw),
                            self.get_parameter('wheelbase').value,self.get_parameter('max_steer').value,
                            max(.1,self.get_parameter('clearance').value-.216),visits)
                        goal=choose_goal(goals,self.memory,now,robot_yaw=yaw,exclusion_radius=.12)
                        if not goal and getattr(self,'forward_retry_once',False):
                            # A reverse maneuver changed the start pose. Probe
                            # one fresh path even if all frontier cooldowns are set.
                            goal=choose_goal(goals,[],now,robot_yaw=yaw)
                        self.forward_retry_once=False
                        if goal:self.probe(goal)
                        elif goals:
                            if not self.start_retrace():self.reason='Waiting for route retry cooldown or observed clearance'
                        elif not self.start_retrace():self.reason='No collision-free lattice route; waiting for observed clearance'
                    except Exception as e:self.reason=str(e)
        self.publish_status()

    def publish_status(self):
        msg=String();msg.data=json.dumps(dict(enabled=self.enabled,reason=self.reason,target=self.target,
            resetting=getattr(self,'resetting',False),
            cancelling_actions=len(getattr(self,'action_requests',set()))+len(getattr(self,'action_results',set())),
            blocked_targets=len(self.memory),pending=self.pending,reversing=self.reversing,
            recorded_points=len(self.trail.points),last_stop=self.last_stop,
            reverse_mode=('straight' if self.reverse_straight else 'recorded') if self.reversing else 'idle',
            last_recovery_event=self.last_recovery_event,planning=self.planning_diagnostics))
        self.status_pub.publish(msg)


def main():
    rclpy.init();node=Explorer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.halt('Shutdown');node.destroy_node();rclpy.shutdown()
