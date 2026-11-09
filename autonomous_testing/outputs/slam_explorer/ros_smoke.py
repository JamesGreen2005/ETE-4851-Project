"""Real ROS constructor smoke check. Opens no serial port; commands no motors."""
import argparse
import rclpy
from robot_explorer.motion_observer import MotionObserver
p=argparse.ArgumentParser();p.add_argument('--calibration',required=True);args=p.parse_args()
rclpy.init(args=['--ros-args','-p','calibration:='+args.calibration]);node=None
try:
    node=MotionObserver()
    print('PASS: real rclpy MotionObserver constructed, including all publishers/subscribers')
finally:
    if node is not None:node.destroy_node()
    rclpy.shutdown()
