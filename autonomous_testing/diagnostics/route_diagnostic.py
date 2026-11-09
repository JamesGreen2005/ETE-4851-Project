import rclpy,time,json,math
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from robot_explorer.route_planner import viewpoint_goals
rclpy.init();n=Node('readonly_route_diagnostic');b=Buffer();listener=TransformListener(b,n);data={}
subs=[]
for key,topic in [('drive','drive/status'),('explore','explore/status')]:subs.append(n.create_subscription(String,topic,lambda m,k=key:data.update({k:json.loads(m.data)}),10))
subs.append(n.create_subscription(OccupancyGrid,'map',lambda m:data.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
end=time.monotonic()+4
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
out={k:v for k,v in data.items() if k!='map'}
try:
 t=b.lookup_transform('map','base_link',Time()).transform;q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));m=data['map'];cfg=json.load(open('/home/swerve/robot_nav_calibration.json'))
 goals,diag=viewpoint_goals(m.data,m.info.width,m.info.height,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),(t.translation.x,t.translation.y,yaw),cfg['wheelbase_m'],cfg['max_steer_rad'],cfg['clearance_margin_m'])
 out['probe']={'pose':[t.translation.x,t.translation.y,yaw],'diagnostic':diag,'goals':len(goals)}
 counts={'unknown':0,'occupied':0,'free':0}
 for j,v in enumerate(m.data):
  x=m.info.origin.position.x+(j%m.info.width+.5)*m.info.resolution;y=m.info.origin.position.y+(j//m.info.width+.5)*m.info.resolution
  if math.hypot(x-t.translation.x,y-t.translation.y)<.5:counts['unknown' if v<0 else 'occupied' if v>=50 else 'free']+=1
 out['nearby_map_cells']=counts
except Exception as e:out['probe_error']=str(e)
print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
