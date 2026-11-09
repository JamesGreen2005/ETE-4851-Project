import json,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
rclpy.init();node=Node('codex_stationary_probe');state={};counts={}
def receive(name,msg):
    counts[name]=counts.get(name,0)+1
    if name in ('drive','fusion'):state[name]=json.loads(msg.data)
    else:state[name]={'vx':msg.twist.twist.linear.x,'vy':msg.twist.twist.linear.y}
subs=[]
for name,topic,kind in [('drive','drive/status',String),('fusion','fusion/status',String),('ekf','odometry/filtered',Odometry),('lidar','lidar/odometry',Odometry)]:
    subs.append(node.create_subscription(kind,topic,lambda m,name=name:receive(name,m),10))
end=time.monotonic()+28
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.2)
print(json.dumps({'counts':counts,'state':state},indent=2))
node.destroy_node();rclpy.shutdown()
