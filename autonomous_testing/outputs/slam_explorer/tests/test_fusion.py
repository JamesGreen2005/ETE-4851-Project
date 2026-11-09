import math
import unittest
import numpy as np
import cv2
from robot_explorer.fusion_math import rigid_fit,scan_motion,motion_twist,wheel_variance,combine_wheel_speeds


class FusionTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(4851)
        self.previous=rng.uniform(-2,2,(200,2))
        yaw=.025;self.r=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        self.t=np.array([.025,-.005])
        self.current=(self.previous-self.t)@self.r

    def test_scan_motion_sign_scale_and_rotation(self):
        r,t,variance,quality=scan_motion(self.current,self.previous)
        np.testing.assert_allclose(r,self.r,atol=.001)
        np.testing.assert_allclose(t,self.t,atol=.001)
        vx,vy,w=motion_twist(r,t,.1)
        self.assertGreater(vx,.2);self.assertLess(vy,0);self.assertAlmostEqual(w,.25,places=2)
        self.assertGreaterEqual(variance,.0004);self.assertGreater(quality,.55)

    def test_stationary_scan_is_accepted(self):
        r,t,_,_=scan_motion(self.previous,self.previous)
        np.testing.assert_allclose(t,0,atol=1e-8)

    def test_single_wall_and_sparse_scan_are_rejected(self):
        wall=np.column_stack((np.linspace(-2,2,100),np.ones(100)))
        with self.assertRaises(ValueError):scan_motion(wall,wall)
        with self.assertRaises(ValueError):scan_motion(wall[:5],wall[:5])

    def test_motion_outliers_and_time_gaps_are_rejected(self):
        for dt in (-.1,0,.01,1.):
            with self.assertRaises(ValueError):motion_twist(self.r,self.t,dt)
        with self.assertRaises(ValueError):motion_twist(self.r,np.array([1.,0]),.1)
        with self.assertRaises(ValueError):rigid_fit(np.full((30,2),np.nan),self.previous[:30])

    def test_load_and_missing_current_reduce_wheel_confidence(self):
        def sensors(a,b):return dict(fresh=True,motors=[dict(current_a=v,current_status='ok') for v in (a,b)])
        base,_=wheel_variance(sensors(.2,.2))
        imbalance,_=wheel_variance(sensors(.2,2.))
        load,_=wheel_variance(sensors(2.,2.),.5)
        missing,_=wheel_variance(dict(fresh=False))
        self.assertGreater(imbalance,base);self.assertGreater(load,base);self.assertGreater(missing,base)

    def test_ekf_fuses_only_independent_twists_from_motion_observers(self):
        from test_core import calibration
        from robot_explorer.configuration import parameters
        e=parameters(calibration(),'/tmp/path.xml')[0]['ekf_filter_node']['ros__parameters']
        for i in (1,2,3):
            self.assertEqual([j for j,v in enumerate(e[f'odom{i}_config']) if v],[6,7,11])
            self.assertFalse(e[f'odom{i}_differential'])

    def test_second_encoder_disagreement_increases_uncertainty(self):
        self.assertEqual(combine_wheel_speeds(.2,None),(.2,0))
        speed,variance=combine_wheel_speeds(.2,.4)
        self.assertAlmostEqual(speed,.3);self.assertAlmostEqual(variance,.04)
        from test_core import calibration
        from robot_explorer.core import require_calibration
        c=calibration();c['secondary_encoder']=dict(index=2,sign=-1,counts_per_meter=12000.)
        require_calibration(c)
        c['secondary_encoder']['index']=1
        with self.assertRaises(ValueError):require_calibration(c)


class CameraObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import sys,types,importlib
        from unittest.mock import patch
        modules={n:types.ModuleType(n) for n in ('rclpy','rclpy.node','rclpy.qos','sensor_msgs','sensor_msgs.msg',
            'nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg','cv_bridge')}
        modules['rclpy.node'].Node=object;modules['rclpy.qos'].qos_profile_sensor_data=object()
        for mod,names in [('sensor_msgs.msg',('LaserScan','Image')),('nav_msgs.msg',('Odometry',)),
                          ('std_msgs.msg',('String',)),('cv_bridge',('CvBridge',))]:
            for n in names:setattr(modules[mod],n,type(n,(),{}))
        with patch.dict(sys.modules,modules):cls.module=importlib.import_module('robot_explorer.motion_observer')

    def test_missing_unverified_camera_calibration_rejected(self):
        with self.assertRaises((ValueError,KeyError)):self.module.camera_calibration({})

    def test_calibrated_ground_flow_produces_metric_motion(self):
        import cv2,types
        rng=np.random.default_rng(12);image=np.zeros((240,320),np.uint8)
        for x,y in rng.integers([20,20],[300,220],size=(130,2)):cv2.circle(image,(int(x),int(y)),2,255,-1)
        shifted=cv2.warpAffine(image,np.float32([[1,0,-3],[0,1,0]]),(320,240))
        node=self.module.MotionObserver.__new__(self.module.MotionObserver)
        node.last_work={};node.previous_images={};node.health={};node.stamp=lambda m:m.timestamp
        node.cv=types.SimpleNamespace(imgmsg_to_cv2=lambda m,encoding:m.image)
        node.calibrations={'front':dict(verified=True,width=320,height=240,camera_matrix=np.eye(3).tolist(),
            distortion=[0.]*5,ground_homography=[[.005,0,-.8],[0,.005,-.6],[0,0,1]],
            ground_roi=[[10,10],[310,10],[310,230],[10,230]])}
        accepted=[];node.publish=lambda *args:accepted.append(args)
        for stamp,pixels in ((1.,image),(1.2,shifted)):
            node.image_cb(types.SimpleNamespace(width=320,height=240,timestamp=stamp,image=pixels),'front')
        self.assertEqual(len(accepted),1,node.health)
        _,_,r,t,dt,_,_=accepted[0]
        self.assertAlmostEqual(float(t[0]),.015,delta=.002)
        self.assertAlmostEqual(motion_twist(r,t,dt)[0],.075,delta=.01)
