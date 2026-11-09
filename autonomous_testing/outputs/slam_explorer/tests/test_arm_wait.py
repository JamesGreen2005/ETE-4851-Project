import ast
from pathlib import Path
import types
import unittest


class ArmWaitTests(unittest.TestCase):
    def run_tick(self,state,stamp=99.,deadline=103.):
        tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/explore.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Explorer')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
        ns={'time':types.SimpleNamespace(monotonic=lambda:100.)}
        exec(compile(ast.Module(body=[tick],type_ignores=[]),'<actual explorer tick>','exec'),ns)
        b=types.SimpleNamespace(enabled=True,waiting_for_arm=True,drive_time=stamp,
            arm_requested_at=99.5,arm_deadline=deadline,drive={'state':state,'reason':'test','health':''},
            map_time=100.,goal_handle=None,pending=True,waiting_plan=False,reversing=False,reason='',publish_status=lambda:None)
        def halt(reason):b.enabled=False;b.reason=reason;b.waiting_for_arm=False
        b.halt=halt;ns['tick'](b);return b

    def test_stale_disarmed_status_does_not_cancel_start(self):
        b=self.run_tick('disarmed')
        self.assertTrue(b.enabled);self.assertTrue(b.waiting_for_arm)

    def test_fresh_armed_status_releases_wait(self):
        b=self.run_tick('armed',stamp=100.)
        self.assertTrue(b.enabled);self.assertFalse(b.waiting_for_arm)

    def test_no_arm_acknowledgment_times_out(self):
        b=self.run_tick('disarmed',deadline=99.)
        self.assertFalse(b.enabled);self.assertIn('timeout',b.reason)
