import json
import math
from pathlib import Path
import unittest
from robot_explorer.route_planner import viewpoint_goals


class ViewpointPlannerTests(unittest.TestCase):
    def test_open_map_produces_feasible_arcs_with_correct_goal_heading(self):
        goals,diagnostic=viewpoint_goals([0]*6400,80,80,.05,(-2.,-2.),(0.,0.,0.),.2413,.261799)
        self.assertTrue(goals);self.assertGreater(diagnostic['valid_viewpoints'],1)
        for goal in goals:
            expected=sum(2*math.tan(turn*.261799)/.2413*direction*distance for direction,turn,distance in goal['segments'])
            self.assertAlmostEqual(goal['yaw'],expected)
            self.assertGreaterEqual(math.hypot(goal['x'],goal['y']),.2)

    def test_unknown_or_occupied_routes_are_not_authorized(self):
        for value in (-1,100):
            goals,_=viewpoint_goals([value]*6400,80,80,.05,(-2.,-2.),(0.,0.,0.),.2413,.261799)
            self.assertEqual(goals,[])

    def test_reverse_is_available_when_front_is_a_dead_end(self):
        data=[0]*6400
        for y in range(80):
            for x in range(45,80):data[y*80+x]=100
        goals,_=viewpoint_goals(data,80,80,.05,(-2.,-2.),(0.,0.,0.),.2413,.261799)
        self.assertTrue(any(g['direction']==-1 and g['x']<0 for g in goals))
        self.assertFalse(any(g['segments'][0][0]==1 for g in goals))

    def test_visited_viewpoint_has_lower_score(self):
        args=([0]*6400,80,80,.05,(-2.,-2.),(0.,0.,0.),.2413,.261799)
        goals,_=viewpoint_goals(*args);best=goals[0]
        changed,_=viewpoint_goals(*args,visits=[(best['x'],best['y'])]*5)
        matching=next(g for g in changed if g['x']==best['x'] and g['y']==best['y'])
        self.assertLess(matching['score'],best['score'])

    def test_actual_stationary_robot_map_has_a_steering_feasible_viewpoint(self):
        m=json.loads((Path(__file__).parent/'fixtures/stationary_map.json').read_text())
        goals,diagnostic=viewpoint_goals(m['data'],m['width'],m['height'],m['resolution'],m['origin'],
                                       (0.,0.,0.),.2413,.261799)
        self.assertTrue(goals,diagnostic)
