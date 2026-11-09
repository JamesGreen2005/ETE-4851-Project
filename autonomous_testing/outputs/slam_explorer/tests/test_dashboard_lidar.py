import ast,math,time,types,unittest
from pathlib import Path

class DashboardLidarTests(unittest.TestCase):
    def node(self,yaw=0.):
        tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/dashboard.py').read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Dashboard')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('scan_cb','lidar_state')]
        ns=dict(math=math,time=time,Time=lambda:None)
        exec(compile(ast.Module(body=methods,type_ignores=[]),'<actual dashboard scan>','exec'),ns)
        mount=types.SimpleNamespace(rotation=types.SimpleNamespace(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)),translation=types.SimpleNamespace(x=.1,y=.2))
        b=types.SimpleNamespace(lidar_scan=None,get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=100e9)),
            tf=types.SimpleNamespace(lookup_transform=lambda *args:types.SimpleNamespace(transform=mount)))
        for n in ('scan_cb','lidar_state'):setattr(b,n,types.MethodType(ns[n],b))
        return b
    def scan(self,stamp=100):
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=stamp,nanosec=0),frame_id='laser'),
            angle_min=0.,angle_increment=math.pi/2,range_min=.1,range_max=10.,ranges=[1.,2.,float('nan'),float('inf'),0.])
    def test_calibrated_mount_transforms_display_into_robot_coordinates(self):
        b=self.node(math.pi);b.scan_cb(self.scan());points=b.lidar_state()['points']
        self.assertEqual(len(points),2);self.assertAlmostEqual(points[0][0],-.9);self.assertAlmostEqual(points[0][1],.2)
        self.assertAlmostEqual(points[1][1],-1.8)
    def test_expired_scans_are_removed_from_display(self):
        b=self.node();b.scan_cb(self.scan());b.lidar_scan['received']-=1.
        self.assertFalse(b.lidar_state()['fresh']);self.assertEqual(b.lidar_state()['points'],[])
    def test_stale_source_frame_is_rejected(self):
        b=self.node();b.scan_cb(self.scan(98));self.assertIsNone(b.lidar_scan)
    def test_missing_transform_does_not_display_misleading_points(self):
        b=self.node()
        def fail(*args):raise RuntimeError('Missing transform')
        b.tf.lookup_transform=fail;b.scan_cb(self.scan());self.assertFalse(b.lidar_state()['fresh'])
