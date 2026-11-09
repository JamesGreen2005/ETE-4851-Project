import ast
import math
from pathlib import Path
import types
import unittest
from robot_explorer.retrace import Trail,MAX_RETRACE_SECONDS


class ForwardAfterReverseTests(unittest.TestCase):
    def setup_robot(self,goals=True):
        tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/explore.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Explorer')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('tick','reverse_tick','finish_recovery')]
        self.now=100.;self.commands=[];self.authorizations=[];self.halts=[];self.probes=[]
        goal=dict(x=2.,y=0.,yaw=0.)
        def twist():
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=None,frame_id=''),
                twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.),angular=types.SimpleNamespace(z=0.)))
        ns=dict(time=types.SimpleNamespace(monotonic=lambda:self.now),math=math,Time=lambda:None,
            TwistStamped=twist,Bool=lambda:types.SimpleNamespace(data=False),MAX_RETRACE_SECONDS=MAX_RETRACE_SECONDS,
            viewpoint_goals=lambda *args:([goal] if goals else [],{}),
            choose_goal=lambda candidates,memory,*args,**kwargs:candidates[0] if candidates and not memory else None)
        exec(compile(ast.Module(body=methods,type_ignores=[]),'<actual explorer methods>','exec'),ns)
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.),orientation=types.SimpleNamespace(z=0.))
        tf=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100,nanosec=0)),
            transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=0.,y=0.),
                rotation=types.SimpleNamespace(w=1.,x=0.,y=0.,z=0.)))
        b=types.SimpleNamespace(enabled=True,reversing=True,waiting_for_arm=False,
            odom_time=100.,drive_time=100.,map_time=100.,odom_pose=(0.,0.,0.,100.),
            drive=dict(state='armed',health='',reason='Obstacle stop at x=-0.415 m, y=0.294 m',
                       obstacle={'checked_speed':-.15}),reverse_started=98.,
            trail=Trail(),reverse_path=[(0.,0.,0.,100.)],recovery_used=True,last_stop=None,
            goal_handle=None,waiting_plan=False,pending=False,memory=[dict(x=2.,y=0.,until=200.)],
            map=types.SimpleNamespace(data=[],info=types.SimpleNamespace(origin=pose,width=1,height=1,resolution=.05)),
            tf=types.SimpleNamespace(lookup_transform=lambda *args:tf),
            get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None,nanoseconds=self.now*1e9)),
            get_parameter=lambda name:types.SimpleNamespace(value=.3),publish_status=lambda:None,
            reverse_cmd=types.SimpleNamespace(publish=self.commands.append),
            reverse_active=types.SimpleNamespace(publish=self.authorizations.append),
            start_retrace=lambda:False)
        def halt(reason):self.halts.append(reason);b.enabled=False
        def probe(target):self.probes.append(target);b.pending=True
        b.halt=halt;b.probe=probe
        for name in ('tick','reverse_tick','finish_recovery'):setattr(b,name,types.MethodType(ns[name],b))
        return b

    def test_blocked_reverse_stays_enabled_then_probes_forward_after_pause(self):
        b=self.setup_robot();b.reverse_tick()
        self.assertTrue(b.enabled);self.assertFalse(b.reversing);self.assertFalse(self.halts)
        self.assertEqual(self.commands[-1].twist.linear.x,0.)
        self.assertFalse(self.authorizations[-1].data)
        b.tick();self.assertFalse(self.probes)
        self.now=100.7;b.drive['reason']='Armed; waiting for navigation command'
        b.tick();self.assertEqual(len(self.probes),1)
        self.assertFalse(b.forward_retry_once);self.assertTrue(b.enabled)

    def test_no_clear_forward_goal_waits_without_disarming_or_driving(self):
        b=self.setup_robot(goals=False);b.reverse_tick();self.now=100.7
        b.tick();self.assertTrue(b.enabled);self.assertFalse(self.probes);self.assertFalse(self.halts)
        self.assertIn('waiting',b.reason)
        self.assertTrue(all(msg.twist.linear.x==0. for msg in self.commands))

    def test_sensor_fault_still_disarms_instead_of_switching_direction(self):
        b=self.setup_robot();b.drive['health']='LiDAR stale';b.reverse_tick()
        self.assertFalse(b.enabled);self.assertTrue(self.halts);self.assertFalse(self.probes)

    def test_reverse_timeout_returns_to_forward_planning(self):
        b=self.setup_robot();b.reverse_started=80.;b.drive['reverse_budget']={'active_seconds':15.};b.reverse_tick()
        self.assertTrue(b.enabled);self.assertFalse(b.reversing);self.assertFalse(self.halts)
        self.assertIn('time limit',b.last_recovery_event)
