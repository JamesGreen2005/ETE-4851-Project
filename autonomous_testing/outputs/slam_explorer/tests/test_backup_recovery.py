import ast
import math
from pathlib import Path
import time
import types
import unittest
from robot_explorer.retrace import Trail, short_backup, reverse_command


def method(name):
    tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/explore.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Explorer')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    ns=dict(time=time,short_backup=short_backup,math=math)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual explorer method>','exec'),ns)
    return ns[name]


class BackupRecoveryTests(unittest.TestCase):
    def robot(self):
        now=time.monotonic()
        return types.SimpleNamespace(enabled=True,pending=False,goal_handle=None,reversing=False,
            recovery_used=False,recovery_anchor=None,odom_pose=(1.,2.,.7,now),odom_time=now,
            trail=Trail(),want_retrace=True)

    def test_straight_backup_is_bounded_and_behind_current_heading(self):
        current=(1.,2.,.7,100.)
        path=short_backup(current)
        self.assertAlmostEqual(math.dist(current[:2],path[-1][:2]),1.)
        v,w,done=reverse_command(path,current,.15,.2413,.26)
        self.assertLess(v,0);self.assertAlmostEqual(w,0);self.assertFalse(done)
        self.assertTrue(reverse_command(path,path[-1],.15,.2413,.26)[2])

    def test_no_forward_history_can_use_one_short_backup(self):
        b=self.robot()
        self.assertTrue(method('start_retrace')(b))
        self.assertTrue(b.reversing);self.assertTrue(b.recovery_used)
        self.assertIn('1 m',b.reason)
        b.reversing=False
        self.assertFalse(method('start_retrace')(b))

    def test_no_backup_until_navigation_cancellation_completes(self):
        for attr,value in [('pending',True),('goal_handle',object()),('enabled',False)]:
            b=self.robot();setattr(b,attr,value)
            self.assertFalse(method('start_retrace')(b))

    def test_stale_odometry_prevents_backup(self):
        b=self.robot();b.odom_time-=1
        self.assertFalse(method('start_retrace')(b))

    def test_budget_resets_after_forward_progress_not_reverse(self):
        b=self.robot();b.recovery_used=True;b.recovery_anchor=(0.,0.)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='odom'),
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=.31,y=0.),
                orientation=types.SimpleNamespace(w=1.,x=0.,y=0.,z=0.))),
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=-.1))))
        method('odom_cb')(b,msg);self.assertTrue(b.recovery_used)
        msg.twist.twist.linear.x=.1
        method('odom_cb')(b,msg);self.assertFalse(b.recovery_used)
