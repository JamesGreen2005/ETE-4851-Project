"""Exercise actual bridge state/command code without opening ROS or hardware."""
import numpy as np
import importlib
import sys
import time
import types
import unittest
from unittest.mock import patch
from test_core import calibration


def load_bridge():
    modules={}
    for name in ('rclpy','rclpy.node','rclpy.qos','geometry_msgs','geometry_msgs.msg',
                 'nav_msgs','nav_msgs.msg','sensor_msgs','sensor_msgs.msg',
                 'std_msgs','std_msgs.msg','std_srvs','std_srvs.srv'):
        modules[name]=types.ModuleType(name)
    modules['rclpy.node'].Node=object
    modules['rclpy.qos'].qos_profile_sensor_data=object()
    for mod,names in [('geometry_msgs.msg',['Twist','TwistStamped']),('nav_msgs.msg',['Odometry']),
        ('sensor_msgs.msg',['Imu','LaserScan']),('std_msgs.msg',['String','Empty','Bool']),('std_srvs.srv',['SetBool'])]:
        for name in names:setattr(modules[mod],name,type(name,(),{}))
    with patch.dict(sys.modules,modules):
        return importlib.import_module('robot_explorer.bridge').VehicleBridge


Bridge=load_bridge()
from robot_explorer.vehicle_control import Control


class Port:
    in_waiting=0
    def __init__(self):self.packets=[]
    def read(self,n):return b''
    def write(self,data):self.packets.append(data);return len(data)


def bridge():
    b=Bridge.__new__(Bridge);now=time.monotonic()
    b.cfg=calibration();b.c=Control();b.c.state='armed';b.c.last_telemetry=now
    b.enabled=True;b.port=Port();b.buffer=bytearray();b.next_tx=0
    b.problem=lambda:'';b.command=(.1,0.);b.last_cmd=now;b.motion_since=now
    b.last_motion=now;b.integral=0.;b.measured_speed=.1;b.ekf_speed=.1;b.reason=''
    b.obstacle_stop=lambda v,w:False
    return b


class GateTests(unittest.TestCase):
    def test_zero_velocity_centers_steering_before_straight_recovery(self):
        b=bridge();b.c.steer(1.);b.command=(0.,0.)
        b.tick();self.assertEqual(b.c.pulses(),[1500,1500]);self.assertEqual(b.c.duty,0)
    def test_stale_command_sends_zero(self):
        b=bridge();b.c.drive(50);b.last_cmd=0;b.tick()
        self.assertEqual((b.c.duty,b.c.duty2),(0,0))

    def test_sensor_failure_disarms(self):
        b=bridge();b.problem=lambda:'LiDAR stale';b.tick()
        self.assertFalse(b.enabled);self.assertEqual(b.c.state,'disarmed')
        self.assertEqual(b.port.packets[-1][3],0)

    def test_operator_expiry_disarms(self):
        b=bridge();b.problem=lambda:'Operator dashboard heartbeat expired';b.tick()
        self.assertFalse(b.enabled)

    def test_obstacle_sends_zero_without_retargeting(self):
        b=bridge();b.obstacle_stop=lambda v,w:True;b.c.drive(50);b.tick()
        self.assertTrue(b.enabled);self.assertEqual(b.c.duty,0)

    def test_stopped_encoder_disarms(self):
        b=bridge();b.last_motion=b.motion_since=time.monotonic()-2;b.tick()
        self.assertFalse(b.enabled);self.assertIn('no motion',b.reason)

    def test_reverse_not_supported(self):
        b=bridge();b.command=(-.1,0);b.tick()
        self.assertFalse(b.enabled)

    def test_excess_curvature_caps_and_checks_actual_turn(self):
        b=bridge();b.command=(.01,1);checked=[]
        b.obstacle_stop=lambda v,w:checked.append((v,w)) or False
        b.tick()
        self.assertTrue(b.enabled)
        self.assertEqual(b.c.steering,1.)
        import math
        self.assertAlmostEqual(checked[0][1],2*.01*math.tan(b.cfg['max_steer_rad'])/b.cfg['wheelbase_m'])
        self.assertIn('capped',b.reason)

    def test_duty_is_bounded_and_same_polarity(self):
        b=bridge();b.measured_speed=-10;b.tick()
        self.assertGreaterEqual(b.c.duty,0)
        self.assertLessEqual(b.c.duty,100)
        self.assertEqual(b.c.duty,b.c.duty2)

    def test_full_duty_ceiling_and_configured_lower_ceiling(self):
        for cap in (50,100):
            b=bridge();b.cfg['max_duty_pct']=cap;b.measured_speed=-10;b.tick()
            self.assertEqual(b.c.duty,cap)

    def test_passive_launch_refuses_enable(self):
        b=bridge();b.c.stop();b.allowed=False
        req=types.SimpleNamespace(data=True);res=types.SimpleNamespace(success=None,message='')
        b.enable_cb(req,res)
        self.assertFalse(res.success)
        self.assertEqual(b.c.state,'disarmed')


if __name__=='__main__':unittest.main()
