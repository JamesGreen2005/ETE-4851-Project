import math
import time
import types
import unittest
from robot_explorer.retrace import Trail,reverse_command,rear_coverage,rear_coverage_details,short_backup
from test_drive_gates import bridge


class RetraceTests(unittest.TestCase):
    def test_reverse_budget_resets_only_between_stopped_maneuvers(self):
        b=bridge();b.reverse_start=1.;b.reverse_distance=1.5;b.retrace_authorized_at=10.
        b.command=(-.1,0.);b.measured_speed=-.1
        b.retrace_cb(types.SimpleNamespace(data=False));self.assertEqual(b.reverse_distance,1.5)
        b.command=(0.,0.);b.measured_speed=0.
        b.retrace_cb(types.SimpleNamespace(data=False));self.assertIsNone(b.reverse_start)
        self.assertEqual(b.reverse_distance,0.);self.assertEqual(b.retrace_authorized_at,10.)
    def test_between_sparse_breadcrumbs_is_on_route(self):
        path=[(0.,0.,0.,1.),(-.34,0.,0.,2.),(-.68,0.,0.,3.)]
        v,w,done=reverse_command(path,(-.17,0.,0.),.15,.2413,.26)
        self.assertLess(v,0);self.assertAlmostEqual(w,0);self.assertFalse(done)

    def test_real_continuous_path_deviation_still_stops_with_measured_error(self):
        path=[(0.,0.,0.,1.),(-.34,0.,0.,2.)]
        with self.assertRaisesRegex(ValueError,'0.160 m exceeds 0.150 m'):
            reverse_command(path,(-.17,.16,0.),.15,.2413,.26)
    def test_recorded_recovery_can_use_two_metres(self):
        trail=Trail()
        for i in range(76):trail.add(i*.04,0.,0.,i*.1)
        path=trail.reverse((3.,0.,0.,8.),8.)
        self.assertAlmostEqual(sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])),2.)

    def test_endpoint_overshoot_finishes_instead_of_faulting(self):
        path=short_backup((0.,0.,0.,1.))
        self.assertEqual(reverse_command(path,(-1.08,0.,0.),.15,.2413,.26),(0.,0.,True))

    def test_no_remaining_point_behind_stops_without_reverse_toward_front(self):
        path=short_backup((0.,0.,0.,1.))
        self.assertEqual(reverse_command(path,(0.,0.,math.pi),.15,.2413,.26),(0.,0.,True))

    def test_one_metre_reverse_tracks_for_longer_than_old_timeout(self):
        path=short_backup((0.,0.,0.,1.));x=0.;elapsed=0.
        for _ in range(300):
            v,w,done=reverse_command(path,(x,0.,0.),.15,.2413,.26)
            if done:break
            self.assertLess(v,0);self.assertAlmostEqual(w,0)
            x+=v*.05;elapsed+=.05
        self.assertTrue(done);self.assertGreater(elapsed,5.);self.assertLess(x,-.94)

    def test_bridge_allows_reverse_beyond_old_distance_and_time_bounds(self):
        b=bridge();b.command=(-.1,0);b.measured_speed=-.1;b.retrace_authorized_at=time.monotonic()
        b.reverse_start=time.monotonic()-7;b.reverse_distance=.9
        b.scan=types.SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        b.tick();self.assertTrue(b.enabled);self.assertLess(b.c.duty,0)

    def test_extended_reverse_still_enforces_time_bound(self):
        b=bridge();b.command=(-.1,0);b.retrace_authorized_at=time.monotonic()
        b.reverse_start=time.monotonic()-17;b.reverse_active_seconds=16.;b.reverse_distance=.9
        b.scan=types.SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        b.tick();self.assertTrue(b.enabled);self.assertEqual(b.c.duty,0);self.assertTrue(b.reverse_limited)

    def test_paused_time_does_not_consume_reverse_budget(self):
        b=bridge();b.reverse_clock=100.;b.reverse_active_seconds=5.;b.reverse_driving=False
        b.account_reverse(130.);self.assertEqual(b.reverse_active_seconds,5.)
        b.reverse_driving=True;b.account_reverse(130.1);self.assertAlmostEqual(b.reverse_active_seconds,5.1)

    def test_limit_stays_zero_until_stopped_authorization_is_withdrawn(self):
        b=bridge();b.reverse_limited=True;b.command=(-.1,0);b.retrace_authorized_at=time.monotonic()
        b.tick();self.assertEqual(b.c.duty,0);self.assertTrue(b.enabled)
        b.command=(0,0);b.measured_speed=0;b.retrace_cb(types.SimpleNamespace(data=False))
        self.assertFalse(b.reverse_limited);self.assertEqual(b.reverse_active_seconds,0)

    def trail(self):
        t=Trail()
        for i in range(21):t.add(i*.04,0.,0.,i*.2)
        return t

    def test_only_recorded_route_and_distance_cap(self):
        p=self.trail().reverse((.8,0,0,5),5,limit=.5)
        self.assertTrue(p)
        self.assertAlmostEqual(sum(math.hypot(a[0]-b[0],a[1]-b[1]) for a,b in zip(p,p[1:])),.5)
        self.assertTrue(all(x[0]<=.8 for x in p))

    def test_stopped_history_still_valid_but_expired_or_offset_not(self):
        self.assertTrue(self.trail().reverse((.8,0,0,12),12))
        self.assertFalse(self.trail().reverse((.8,0,0,60),60))
        self.assertFalse(self.trail().reverse((1.2,0,0,5),5))

    def test_jump_clears_old_route(self):
        t=self.trail();t.add(4.,0.,0.,5)
        self.assertFalse(t.reverse((4.,0.,0.,5),5))

    def test_reverse_command_negative_and_completes(self):
        p=self.trail().reverse((.8,0,0,5),5)
        v,w,done=reverse_command(p,(.8,0,0),.15,.2413,.26)
        self.assertLess(v,0);self.assertAlmostEqual(w,0);self.assertFalse(done)
        self.assertTrue(reverse_command(p,p[-1],.15,.2413,.26)[2])

    def test_route_deviation_stops(self):
        p=self.trail().reverse((.8,0,0,5),5)
        with self.assertRaises(ValueError):reverse_command(p,(.7,.3,0),.15,.2413,.26)

    def test_rear_coverage_is_required(self):
        s=types.SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        self.assertTrue(rear_coverage(s,{'laser_yaw_rad':0}))
        s.ranges=[float('inf')]*360
        self.assertFalse(rear_coverage(s,{'laser_yaw_rad':0}))

    def test_side_gap_outside_straight_corridor_does_not_block_reverse(self):
        s=types.SimpleNamespace(angle_min=0.,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        for i in range(90,110):s.ranges[i]=float('inf')
        self.assertTrue(rear_coverage(s,{'laser_yaw_rad':0},-.15,0))
        self.assertFalse(rear_coverage_details(s,{'laser_yaw_rad':0},-.15,.1)['complete'])
        self.assertTrue(rear_coverage(s,{'laser_yaw_rad':0},-.15,.1))

    def test_small_corridor_gap_is_reported_and_allowed(self):
        s=types.SimpleNamespace(angle_min=0.,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        for i in range(170,190):s.ranges[i]=float('inf')
        detail=rear_coverage_details(s,{'laser_yaw_rad':0})
        self.assertFalse(detail['complete']);self.assertTrue(detail['missing_degrees'])
        self.assertLess(detail['known_bins'],detail['required_bins'])
        self.assertTrue(detail['sufficient'])

    def test_all_invalid_scan_is_unknown_and_mount_yaw_is_applied(self):
        s=types.SimpleNamespace(angle_min=0.,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[float('inf')]*360)
        self.assertFalse(rear_coverage(s,{'laser_yaw_rad':math.pi}))
        s.ranges=[3.]*360
        for i in list(range(0,15))+list(range(345,360)):s.ranges[i]=float('nan')
        self.assertFalse(rear_coverage_details(s,{'laser_yaw_rad':math.pi})['complete'])
        self.assertTrue(rear_coverage(s,{'laser_yaw_rad':math.pi}))
        self.assertTrue(rear_coverage(s,{'laser_yaw_rad':0.}))

    def test_exactly_twenty_sectors_and_half_threshold(self):
        for yaw in (0.,math.pi):
            for turn in (0.,.1):
                cfg={'laser_yaw_rad':yaw}
                base=types.SimpleNamespace(angle_min=0.,angle_increment=.01,
                    range_min=.1,range_max=10.,ranges=[])
                detail=rear_coverage_details(base,cfg,-.15,turn)
                self.assertEqual(detail['required_bins'],20)
                half=detail['half_angle_deg']
                # Use interior points well away from sector boundaries.
                for count,allowed in [(1,True),(9,True),(20,True),(0,False)]:
                    start=math.pi-math.radians(half)+math.radians(half)/20-yaw
                    s=types.SimpleNamespace(angle_min=start,angle_increment=math.radians(2*half/20),
                        range_min=.1,range_max=10.,ranges=[3.]*count+[float('inf')]*(20-count))
                    result=rear_coverage_details(s,cfg,-.15,turn)
                    self.assertEqual(result['known_bins'],count)
                    self.assertEqual(result['sufficient'],allowed)

    def test_detected_rear_obstacle_stops_even_with_sufficient_coverage(self):
        from test_drive_gates import Bridge as VehicleBridge
        b=bridge();b.command=(-.1,0);b.retrace_authorized_at=time.monotonic()
        b.cfg['laser_yaw_rad']=0
        b.scan=types.SimpleNamespace(angle_min=0.,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        for i in range(170,180):b.scan.ranges[i]=float('inf')
        b.scan.ranges[180]=.4
        b.obstacle_stop=lambda v,w:VehicleBridge.obstacle_stop(b,v,w)
        self.assertTrue(rear_coverage(b.scan,b.cfg))
        b.tick();self.assertEqual(b.c.duty,0);self.assertIn('Obstacle stop',b.reason)

    def test_bridge_reverses_only_with_authorization_and_rear_coverage(self):
        b=bridge();b.command=(-.1,0);b.measured_speed=-.1;b.retrace_authorized_at=time.monotonic()
        b.scan=types.SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        b.tick();self.assertTrue(b.enabled);self.assertLess(b.c.duty,0)

    def test_reverse_distance_bound_stops_without_disarming(self):
        b=bridge();b.command=(-.1,0);b.retrace_authorized_at=time.monotonic()
        b.reverse_start=time.monotonic();b.reverse_distance=2.16
        b.scan=types.SimpleNamespace(angle_min=-math.pi,angle_increment=math.pi/180,
            range_min=.1,range_max=10.,ranges=[3.]*360)
        b.tick();self.assertTrue(b.enabled);self.assertEqual(b.c.duty,0);self.assertTrue(b.reverse_limited)

    def test_reverse_checks_rear_not_forward_swept_direction(self):
        from test_drive_gates import Bridge as VehicleBridge
        b=bridge();b.cfg['laser_yaw_rad']=0
        b.scan=types.SimpleNamespace(angle_min=math.pi,angle_increment=.01,
            range_min=.1,range_max=10.,ranges=[.4])
        self.assertTrue(VehicleBridge.obstacle_stop(b,-.15,0))
        self.assertFalse(VehicleBridge.obstacle_stop(b,.15,0))
        b.scan.angle_min=0
        self.assertFalse(VehicleBridge.obstacle_stop(b,-.15,0))
        self.assertTrue(VehicleBridge.obstacle_stop(b,.15,0))
