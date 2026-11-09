import time,rclpy
from robot_explorer.dashboard import Dashboard,PAGE
rclpy.init(args=['--ros-args','-r','__node:=dashboard_layout_probe'])
node=Dashboard()
# Navigation is stopped, so load its actual calibrated mount into this
# probe's LOCAL TF buffer. Do not publish a duplicate transform.
import json,math
from geometry_msgs.msg import TransformStamped
cfg=json.loads(open('/home/swerve/robot_nav_calibration.json').read())
t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id=cfg['laser_frame']
t.transform.translation.x=float(cfg['laser_x_m']);t.transform.translation.y=float(cfg['laser_y_m']);t.transform.translation.z=float(cfg['laser_z_m'])
t.transform.rotation.z=math.sin(cfg['laser_yaw_rad']/2);t.transform.rotation.w=math.cos(cfg['laser_yaw_rad']/2)
node.tf.set_transform_static(t,'stationary_probe_local_buffer')
end=time.monotonic()+4
try:
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    scan=node.lidar_state()
    assert scan['fresh'] and len(scan['points'])>20,scan
    assert 'mirrorRear' not in PAGE
    assert PAGE.index('id="camera"')<PAGE.index('Live motor and sensor readings')<PAGE.index('id="diagnostics"')
    print('PASS real dashboard constructor; live LiDAR display received',len(scan['points']),'base-frame returns; updated layout; no motor commands')
finally:node.destroy_node();rclpy.shutdown()
