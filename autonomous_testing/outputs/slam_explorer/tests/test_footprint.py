import unittest,math,json
from pathlib import Path
from robot_explorer.footprint import footprint_known_map,cell_intersects_body,observed_rays
from robot_explorer.route_planner import viewpoint_goals

class FootprintTests(unittest.TestCase):
    def test_unknown_under_body_is_known_but_occupied_is_preserved(self):
        raw=[-1]*1600;raw[20*40+20]=100
        changed,n=footprint_known_map(raw,40,40,.05,(-1,-1),(0,0,0))
        self.assertGreater(n,0);self.assertEqual(changed[820],100);self.assertEqual(raw[819],-1)
        self.assertEqual(changed[0],-1)
    def test_empty_unknown_world_never_authorizes_a_route(self):
        goals,_=viewpoint_goals([-1]*1600,40,40,.05,(-1,-1),(0,0,0),.2413,.26)
        self.assertFalse(goals)
    def test_rotation_changes_intersection(self):
        self.assertFalse(cell_intersects_body(.2,0,.01,(0,0,0)))
        self.assertTrue(cell_intersects_body(.2,0,.01,(0,0,math.pi/4)))
    def test_live_ray_clears_unknown_but_not_wall_or_unseen_side(self):
        data=[-1]*1600;data[20*40+25]=100
        out=observed_rays(data,40,40,.05,(-1,-1),(0.025,.025),[(.9,.025)])
        self.assertEqual(out[20*40+22],0);self.assertEqual(out[20*40+25],100)
        self.assertEqual(out[20*40+26],-1);self.assertEqual(out[21*40+22],-1)
        self.assertEqual(observed_rays(data,40,40,.05,(-1,-1),(.025,.025),[]),data)
    def test_ray_endpoint_is_occupied(self):
        out=observed_rays([-1]*1600,40,40,.05,(-1,-1),(.025,.025),[(.525,.025)])
        self.assertEqual(out[20*40+30],100)
    def test_captured_map_no_longer_rejects_current_body(self):
        m=json.loads((Path(__file__).parent/'fixtures/stuck_map.json').read_text())
        _,d=viewpoint_goals(m['data'],m['width'],m['height'],m['resolution'],m['origin'],m['pose'],.2413,.261799)
        self.assertNotIn('error',d)
