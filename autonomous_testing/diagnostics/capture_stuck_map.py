import rclpy,json,time,math
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('capture_stuck_map');b=Buffer();l=TransformListener(b,n);maps=[]
s=n.create_subscription(OccupancyGrid,'map',maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
end=time.monotonic()+18
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
m=maps[-1];t=b.lookup_transform('map','base_link',Time()).transform;q=t.rotation
out=dict(data=list(m.data),width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],pose=[t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
json.dump(out,open('/tmp/stuck_map.json','w'));print('Captured',out['pose']);n.destroy_node();rclpy.shutdown()
