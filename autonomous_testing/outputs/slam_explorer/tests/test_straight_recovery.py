import ast
from pathlib import Path
import types
import unittest
from robot_explorer.retrace import RouteDeviation,reverse_command,straight_reverse_command,MAX_RETRACE_SECONDS
from robot_explorer.core import locked_steering_profile
from robot_explorer.vehicle_control import Control


class StraightRecoveryTests(unittest.TestCase):
    def test_pulse_limits_and_straight_are_exact(self):
        profile=locked_steering_profile(dict(servo_travel_us=25,front_center_us=1450,rear_center_us=1550,max_steer_rad=.26))
        self.assertEqual(profile['max_steer_rad'],.26)
        self.assertEqual([profile[k] for k in ('servo_travel_us','front_center_us','rear_center_us')],[500,1500,1500])
        c=Control();c.state='armed'
        for turn,pulses in [(-1,[1000,2000]),(0,[1500,1500]),(1,[2000,1000])]:
            c.steer(turn);self.assertEqual(c.pulses(),pulses)

    def test_reconfiguration_cannot_move_center_or_reduce_locked_span(self):
        c=Control();c.confirmed_disarmed=True
        for settings in [dict(travel=25),dict(front_center=1450),dict(rear_center=1550)]:
            with self.assertRaises(ValueError):c.configure(settings)

    def test_straight_recovery_does_not_depend_on_old_route_error(self):
        anchor=(0.,.153,0.,1.)
        self.assertEqual(straight_reverse_command(anchor,(-.5,.3,.1),.15),(-.15,0.,False))
        self.assertEqual(straight_reverse_command(anchor,(-1.01,.153,0.),.15),(0.,0.,True))
        with self.assertRaises(ValueError):straight_reverse_command(anchor,(float('nan'),0.,0.),.15)

    def test_actual_explorer_switches_without_disarm_and_keeps_original_timer(self):
        tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/explore.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Explorer')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='reverse_tick')
        def twist():
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=None,frame_id=''),
                twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.),angular=types.SimpleNamespace(z=0.)))
        ns=dict(time=types.SimpleNamespace(monotonic=lambda:100.),MAX_RETRACE_SECONDS=MAX_RETRACE_SECONDS,
            RouteDeviation=RouteDeviation,reverse_command=reverse_command,straight_reverse_command=straight_reverse_command,
            TwistStamped=twist,Bool=lambda:types.SimpleNamespace(data=False))
        exec(compile(ast.Module(body=[tick],type_ignores=[]),'<actual reverse tick>','exec'),ns)
        published=[];halted=[];finished=[]
        b=types.SimpleNamespace(reversing=True,enabled=True,odom_time=100.,drive_time=100.,
            drive={'state':'armed','health':'','reason':''},reverse_started=98.,reverse_straight=False,
            reverse_path=[(0.,0.,0.,1.),(-1.,0.,0.,1.)],odom_pose=(0.,.153,0.,100.),
            get_parameter=lambda name:types.SimpleNamespace(value={'retrace_speed':.15,'wheelbase':.2413,'max_steer':.26}[name]),
            get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:None)),
            reverse_active=types.SimpleNamespace(publish=lambda msg:None),
            reverse_cmd=types.SimpleNamespace(publish=published.append),halt=halted.append,finish_recovery=finished.append)
        ns['reverse_tick'](b)
        self.assertTrue(b.reverse_straight);self.assertFalse(halted)
        self.assertEqual(published[-1].twist.linear.x,0.)
        ns['reverse_tick'](b)
        self.assertEqual(published[-1].twist.linear.x,0.)
        ns['time'].monotonic=lambda:100.5
        b.odom_time=b.drive_time=100.5
        ns['reverse_tick'](b)
        self.assertEqual(published[-1].twist.linear.x,-.15)
        self.assertEqual(published[-1].twist.angular.z,0.)
        self.assertEqual(b.reverse_started,98.)
        b.reverse_started=80.
        b.drive['reverse_budget']={'active_seconds':15.};ns['reverse_tick'](b);self.assertIn('time limit',finished[-1]);self.assertFalse(halted)
