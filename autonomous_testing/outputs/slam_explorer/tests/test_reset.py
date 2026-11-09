"""Exercise actual explorer methods with delayed action replies."""
import ast
import math
import time
import types
import unittest
from concurrent.futures import Future
from pathlib import Path
from robot_explorer.retrace import Trail


def message():
    pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.),orientation=types.SimpleNamespace(z=0.,w=1.))
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=None,frame_id=''),pose=pose,
        twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.),angular=types.SimpleNamespace(z=0.)))


class Client:
    def __init__(self):self.requests=[];self.futures=[]
    def service_is_ready(self):return True
    def server_is_ready(self):return True
    def call_async(self,req):
        self.requests.append(req.data);f=Future();f.set_result(types.SimpleNamespace(success=True,message='accepted'));return f
    def send_goal_async(self,req):
        self.requests.append(req);f=Future();self.futures.append(f);return f


class Handle:
    accepted=True
    def __init__(self):self.result=Future();self.cancels=0
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.cancels+=1;return Future()


class ResetTests(unittest.TestCase):
    def robot(self):
        tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/explore.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Explorer');cls.bases=[]
        ns=dict(time=time,math=math,rclpy=types.SimpleNamespace(ok=lambda:True),TwistStamped=message,
            Bool=lambda:types.SimpleNamespace(data=False),SetBool=types.SimpleNamespace(Request=lambda:types.SimpleNamespace(data=False)),
            NavigateToPose=types.SimpleNamespace(Goal=lambda:types.SimpleNamespace(pose=message())),
            ComputePathToPose=types.SimpleNamespace(Goal=lambda:types.SimpleNamespace(goal=message())))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'<actual Explorer>','exec'),ns)
        b=ns['Explorer'].__new__(ns['Explorer']);now=time.monotonic()
        b.generation=0;b.enabled=False;b.pending=False;b.starting=False;b.resetting=False
        b.waiting_for_arm=False;b.action_requests=set();b.action_results=set();b.goal_handle=None;b.plan_handle=None
        b.odom_pose=(0.,0.,0.,now);b.reverse_path=[];b.reversing=False;b.want_retrace=False
        b.memory=[{'x':1}];b.failures={'x':1};b.visits=[(0,0)];b.trail=Trail();b.trail.points.append((0,0,0,now))
        b.recovery_used=True;b.recovery_anchor=b.odom_pose;b.reverse_straight=True;b.bt='';b.target=None
        b.map=object();b.map_time=now;b.drive_time=now
        b.drive={'state':'disarmed','confirmed_disarmed':True,'applied_duty':[0.,0.],'speed':0.,'health':''}
        b.action=Client();b.planner=Client();b.motor=Client();b.published=[]
        b.reverse_cmd=types.SimpleNamespace(publish=b.published.append);b.reverse_active=b.reverse_cmd
        b.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None))
        b.publish_status=lambda:None
        return b

    def reset(self,b):
        res=types.SimpleNamespace(success=False,message='')
        b.reset_cb(types.SimpleNamespace(data=True),res);self.assertTrue(res.success)

    def settle(self,b):b.drive_time=time.monotonic()+.01;b.reset_tick()

    def test_reset_clears_failure_state_and_never_arms(self):
        b=self.robot();b.pending=True;self.reset(b)
        self.assertFalse(b.pending);self.assertFalse(b.enabled);self.assertTrue(b.resetting)
        self.assertFalse(b.memory);self.assertFalse(b.failures);self.assertFalse(b.visits);self.assertFalse(b.trail.points)
        self.assertFalse(b.recovery_used);self.assertEqual(b.motor.requests,[False])
        self.settle(b);self.assertFalse(b.resetting);self.assertIn('Ready',b.reason)

    def test_reset_waits_for_fresh_stop_and_sensor_health(self):
        b=self.robot();self.reset(b);b.reset_tick();self.assertTrue(b.resetting)
        for patch in ({'state':'armed'},{'confirmed_disarmed':False},{'applied_duty':[50,50]}, {'speed':.1},{'health':'LiDAR stale'}):
            original=dict(b.drive);b.drive.update(patch);self.settle(b);self.assertTrue(b.resetting);b.drive=original
        self.settle(b);self.assertFalse(b.resetting)

    def test_stop_clears_pending_and_fences_unanswered_requests(self):
        b=self.robot();b.probe(dict(x=1.,y=0.,yaw=0.));self.assertTrue(b.pending)
        b.halt('Stop');self.assertFalse(b.pending)
        res=types.SimpleNamespace(success=False,message='');b.enable_cb(types.SimpleNamespace(data=True),res)
        self.assertFalse(res.success);self.assertIn('cancelling',res.message)

    def test_late_acceptance_after_reset_is_cancelled_and_fenced(self):
        for method,client in (('probe','planner'),('send','action')):
            b=self.robot();getattr(b,method)(dict(x=1.,y=0.,yaw=0.));self.reset(b)
            h=Handle();getattr(b,client).futures[0].set_result(h)
            self.assertEqual(h.cancels,1);self.assertFalse(b.pending);self.settle(b);self.assertTrue(b.resetting)
            h.result.set_result(types.SimpleNamespace(status=5));self.settle(b)
            self.assertFalse(b.resetting);self.assertFalse(b.action_requests);self.assertFalse(b.action_results)

    def test_old_result_cannot_clear_new_pending_or_target(self):
        for method,client in (('probe','planner'),('send','action')):
            b=self.robot();getattr(b,method)(dict(x=1.,y=0.,yaw=0.))
            h=Handle();getattr(b,client).futures[0].set_result(h);b.halt('Stop')
            b.pending=True;b.target={'new':True};b.reason='new attempt'
            h.result.set_result(types.SimpleNamespace(status=5))
            self.assertTrue(b.pending);self.assertEqual(b.target,{'new':True});self.assertEqual(b.reason,'new attempt')

    def test_repeated_reset_start_needs_explicit_start_each_time(self):
        b=self.robot()
        for _ in range(3):
            self.reset(b);self.settle(b);self.assertFalse(b.enabled)
            res=types.SimpleNamespace(success=False,message='');b.enable_cb(types.SimpleNamespace(data=True),res)
            self.assertTrue(res.success);self.assertTrue(b.enabled);self.assertTrue(b.waiting_for_arm)
        self.assertEqual(b.motor.requests,[False,True]*3)

    def test_planned_reverse_limit_cancels_then_replans_without_disarm(self):
        b=self.robot();b.enabled=True;b.send(dict(x=-1.,y=0.,yaw=0.))
        h=Handle();b.action.futures[0].set_result(h)
        b.drive.update(state='armed',reason='Reverse maneuver limit reached; waiting for replan')
        b.tick();self.assertEqual(h.cancels,1);self.assertTrue(b.limit_replan_pending)
        b.tick();self.assertEqual(h.cancels,1)
        h.result.set_result(types.SimpleNamespace(status=5))
        self.assertTrue(b.enabled);self.assertFalse(b.pending);self.assertIsNone(b.goal_handle)
        self.assertGreater(b.replan_after,time.monotonic());self.assertFalse(b.motor.requests)
