import rclpy,time,json,math
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from robot_explorer.route_planner import viewpoint_goals
rclpy.init();n=Node('footprint_path_probe');b=Buffer();l=TransformListener(b,n);state={};subs=[]
for key,topic in [('raw','map'),('nav','navigation/map')]:
 subs.append(n.create_subscription(OccupancyGrid,topic,lambda m,k=key:state.update({k:m}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
subs.append(n.create_subscription(String,'drive/status',lambda m:state.update(drive=json.loads(m.data)),10))
a=ActionClient(n,ComputePathToPose,'compute_path_to_pose');cfg=json.load(open('/home/swerve/robot_nav_calibration.json'));end=time.monotonic()+52;goals=[];diag={}
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if 'nav' not in state or not a.server_is_ready():continue
 try:
  t=b.lookup_transform('map','base_link',Time()).transform;q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));m=state['nav']
  goals,diag=viewpoint_goals(m.data,m.info.width,m.info.height,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),(t.translation.x,t.translation.y,yaw),cfg['wheelbase_m'],cfg['max_steer_rad'],cfg['clearance_margin_m'])
  if goals:break
 except Exception as e:diag={'error':str(e)}
print('CANDIDATES',len(goals),diag,flush=True)
assert state.get('drive',{}).get('enabled') is False,state.get('drive')
passed=False
for g in goals[:5]:
 req=ComputePathToPose.Goal();req.goal.header.frame_id='map';req.goal.header.stamp=n.get_clock().now().to_msg();req.planner_id='GridBased'
 req.goal.pose.position.x=g['x'];req.goal.pose.position.y=g['y'];req.goal.pose.orientation.z=math.sin(g['yaw']/2);req.goal.pose.orientation.w=math.cos(g['yaw']/2)
 f=a.send_goal_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=6)
 if not f.done():continue
 h=f.result()
 if not h.accepted:continue
 f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=8)
 if not f.done():h.cancel_goal_async();continue
 result=f.result();print('PATH',result.status,len(result.result.path.poses),result.result.error_msg,flush=True)
 if result.status==4 and len(result.result.path.poses)>1:passed=True;break
print('PASS_PATH',passed,'MOTORS_ENABLED',state.get('drive',{}).get('enabled'),flush=True)
json.dump({'candidates':len(goals),'diagnostics':diag,'path_passed':passed,'drive_enabled':state.get('drive',{}).get('enabled')},open('/tmp/footprint_017_result.json','w'))
n.destroy_node();rclpy.shutdown()
