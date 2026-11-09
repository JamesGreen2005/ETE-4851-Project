import copy
import json
import math
import struct
import unittest
from pathlib import Path
from robot_explorer.core import *
from robot_explorer.configuration import parameters
from robot_explorer.vehicle_control import Control


def calibration():
    c=json.loads((Path(__file__).parents[1]/'config/calibration.example.json').read_text())
    c.update(verified=True,counts_per_meter=10000.,wheelbase_m=.25,max_steer_rad=.3,
        measured_speed_mps=.15,stopping_distance_m=.15)
    return c


class CoreTests(unittest.TestCase):
    def test_counter_wrap_both_directions(self):
        self.assertEqual(wrap_delta(5,65530),11)
        self.assertEqual(wrap_delta(65530,5),-11)

    def test_accept_actual_fw7_diagnostics(self):
        line='H=+358.9 R=+001.6 P=-036.3 CAL=0/3/1/0 VALID=1 ERR=000 DRIVE=1 FW=7 S1=1500 S2=1500 T2=20000 INA=1 P2=3 A2=00000 B2=00000 X2=00000 G2=8 C2=127 E1=41258 E2=00000 ARM=1 D1=+050.0 D2=+050.0 CMDERR=000'
        self.assertEqual(telemetry(line)['E1'],'41258')
        with self.assertRaises(ValueError):telemetry(line[10:])
        with self.assertRaises(ValueError):telemetry(line.replace('41258','99999'))

    def test_measurements_required(self):
        c=calibration();self.assertEqual(encoder_scale(c),10000)
        for key in ('counts_per_meter','wheelbase_m','measured_speed_mps','stopping_distance_m'):
            bad=copy.deepcopy(c);bad[key]=None
            with self.subTest(key=key),self.assertRaises(ValueError):require_calibration(bad)
        c['verified']=False
        with self.assertRaises(ValueError):require_calibration(c)

    def test_counts_per_rev_alternative(self):
        c=calibration();c.update(counts_per_meter=None,wheel_diameter_m=.1,counts_per_wheel_rev=1000)
        self.assertAlmostEqual(encoder_scale(c),10000/math.pi)

    def test_nonfinite_and_servo_limits(self):
        for key,value in [('measured_speed_mps',float('nan')),('servo_travel_us',600),('front_center_us',1000)]:
            c=calibration();c[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):require_calibration(c)

    def test_curvature_and_no_spin(self):
        self.assertEqual(steering(0,.5,.25,.3),0)
        angle=steering(.2,.2,.25,.3)
        self.assertAlmostEqual(2*.2*math.tan(angle)/.25,.2)
        self.assertEqual(steering(.01,1.,.25,.3),.3)
        self.assertEqual(steering(.01,-1.,.25,.3),-.3)
        with self.assertRaises(ValueError):steering(.1,float('nan'),.25,.3)

    def test_parameter_sensor_masks(self):
        ekf,slam,nav=parameters(calibration(),'/tmp/committed.xml')
        e=ekf['ekf_filter_node']['ros__parameters']
        self.assertEqual(len(e['odom0_config']),15);self.assertEqual(len(e['imu0_config']),15)
        self.assertEqual([i for i,v in enumerate(e['odom0_config']) if v],[6,7])
        self.assertEqual([i for i,v in enumerate(e['imu0_config']) if v],[5])
        controller=nav['controller_server']['ros__parameters']['FollowPath']
        self.assertFalse(controller['use_rotate_to_heading'])
        self.assertTrue(controller['allow_reversing'])
        self.assertEqual(slam['slam_toolbox']['ros__parameters']['mode'],'mapping')

    def test_turn_radius_matches_bridge(self):
        c=calibration();_,_,nav=parameters(c,'/tmp/tree.xml')
        radius=nav['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
        self.assertAlmostEqual(steering(.1,.1/radius,c['wheelbase_m'],c['max_steer_rad']),c['max_steer_rad'])

    def test_positive_defaults_are_serialized(self):
        c=Control();c.state='armed';c.drive(50)
        self.assertEqual(struct.unpack_from('<hh',c.next_frame(),12),(500,500))

    def test_frontier_reachable_and_known(self):
        w=h=40;res=.1
        data=[-1]*(w*h)
        for y in range(3,37):
            for x in range(3,30):data[y*w+x]=0
        goals=frontier_goals(data,w,h,res,(0,0),(1.,2.))
        self.assertTrue(goals)
        for goal in goals:
            k=int(goal['y']/res)*w+int(goal['x']/res)
            self.assertEqual(data[k],0)

    def test_no_unknown_or_occupied_origin_goal(self):
        self.assertFalse(frontier_goals([-1]*100,10,10,.1,(0,0),(.5,.5)))
        self.assertFalse(frontier_goals([100]*100,10,10,.1,(0,0),(.5,.5)))

    def test_symmetric_open_map_has_frontier_goal_not_robot_centroid(self):
        w=h=50;data=[-1]*(w*h)
        for y in range(5,45):
            for x in range(5,45):data[y*w+x]=0
        goals=frontier_goals(data,w,h,.1,(0,0),(2.5,2.5))
        self.assertTrue(goals)
        for goal in goals:
            self.assertGreaterEqual(math.hypot(goal['x']-2.5,goal['y']-2.5),.5)
            ix,iy=int(goal['x']/.1),int(goal['y']/.1)
            self.assertEqual(data[iy*w+ix],0)
            self.assertGreaterEqual(min(ix-4,45-ix,iy-4,45-iy)*.1,.38)

    def test_unreachable_frontier_not_selected(self):
        w=h=30;data=[100]*(w*h)
        for y in range(3,10):
            for x in range(3,10):data[y*w+x]=0
        for y in range(15,25):
            for x in range(15,25):data[y*w+x]=0
        for y in range(15,25):data[y*w+25]=-1
        self.assertFalse(frontier_goals(data,w,h,.1,(0,0),(.6,.6),.1))

    def test_failure_memory_and_expiry(self):
        a=dict(x=1.,y=1.,score=10);b=dict(x=3.,y=2.,score=5)
        memory=[dict(x=1.,y=1.,until=100)]
        self.assertEqual(choose_goal([a,b],memory,50),b)
        self.assertEqual(choose_goal([a,b],memory,101),a)

    def test_heading_preference_and_alternative_frontier(self):
        rear=dict(x=-1.,y=0.,yaw=math.pi,score=1.1)
        front=dict(x=1.,y=0.,yaw=0.,score=1.)
        self.assertEqual(choose_goal([rear,front],[],0,robot_yaw=0),front)
        alternative=dict(x=1.4,y=0.,yaw=0.,score=.9)
        self.assertEqual(choose_goal([front,alternative],[dict(x=1.,y=0.,until=50)],0),alternative)

    def test_bt_has_no_periodic_retarget_or_spin(self):
        import xml.etree.ElementTree as ET
        root=ET.parse(Path(__file__).parents[1]/'behavior_trees/committed_path.xml').getroot()
        self.assertEqual(len(root.findall('.//ComputePathToPose')),1)
        self.assertEqual(len(root.findall('.//FollowPath')),1)
        self.assertFalse(root.findall('.//RateController'))
        self.assertFalse(root.findall('.//Spin'))


if __name__=='__main__':unittest.main()
