"""Temporal candidate preference, fresh feasibility and real worker/MPC handoff."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import runpy
import subprocess
import sys
import unittest

api = runpy.run_path(str(Path(__file__).with_name('test_lidar_avoidance.py')))
s = api['shadow']


def job(center=0.):
    record = api['record'](())
    record['road'].update(centerline_xy=[(0.,center),(.5,center),(1.5,center)],
        left_boundary_xy=[(0.,.7),(2.,.7)],right_boundary_xy=[(0.,-.7),(2.,-.7)])
    record.update(shadow_pose=[0.,0.,0.],shadow_odometry_epoch=1)
    record['settings'].update(min_turn_radius_m=1.,obstacle_check_enabled=True,collision_coverage='generated')
    return record


def next_job(old, center):
    record = job(center)
    record['stamp_s'] = 100.5
    record['execution_reference'] = dict(path=deepcopy(old['selected']['path']),pose=[0.,0.,0.],
        epoch=1,stamp_s=100.,progress_m=0.,job_id=5)
    return record


def plan(record, **values):
    return s['plan'](record,weight_consistency=4.,switch_cost_margin=.05,**values)


class ContinuityChecks(unittest.TestCase):
    def test_disabled_or_missing_history_preserves_native_choice(self):
        old = s['plan'](job(-.1))
        record = next_job(old,.1)
        disabled = s['plan'](record)
        plain = job(.1)
        native = s['plan'](plain)
        missing = plan(plain)
        for result in (disabled,missing):
            self.assertEqual(result['selected']['path'],native['selected']['path'])
            self.assertEqual([c['cost'] for c in result['candidates']],[c['cost'] for c in native['candidates']])
        for values in ({'weight_consistency':-1.},{'switch_cost_margin':-1.},{'continuity_window_m':0.}):
            with self.assertRaises(ValueError):
                s['Settings'](**values)

    def test_small_road_jitter_reduces_path_change_and_uses_hysteresis(self):
        old = s['plan'](job(-.1))
        record = next_job(old,.1)
        native, actual = s['plan'](record),plan(record)
        ctx = s['ns']['_shadow_continuity_context'](record,s['Settings'](weight_consistency=4.))
        cost = s['ns']['_shadow_continuity_cost']
        self.assertLess(cost(actual['selected']['path'],ctx),cost(native['selected']['path'],ctx))
        self.assertTrue(actual['path_continuity']['hysteresis_applied'])
        self.assertEqual(actual['candidate_count'],48)
        self.assertIsNone(actual['selected']['collision'])
        # No new curves are invented or old curves admitted past fresh checks.
        self.assertEqual([c['path'] for c in actual['candidates']],[c['path'] for c in native['candidates']])
        self.assertEqual([c['collision'] for c in actual['candidates']],[c['collision'] for c in native['candidates']])

    def test_significant_road_change_can_switch_immediately(self):
        old = s['plan'](job(-.1))
        result = plan(next_job(old,.3))
        self.assertEqual(result['status'],'VALID')
        self.assertFalse(result['path_continuity']['hysteresis_applied'])
        self.assertGreater(result['selected']['path'][20][1],0.)

    def test_new_obstacle_overrides_preference_and_wall_still_stops(self):
        old = s['plan'](job(-.1))
        record = next_job(old,-.1)
        point = min(old['selected']['path'],key=lambda p:abs(p[0]-.9))
        record['obstacles'].update(timestamp_s=100.5,points_xyz_m=[(point[0],point[1],0.)])
        cfg = s['Settings'](**{**record['settings'],'weight_consistency':4.,'switch_cost_margin':.05})
        context = s['ns']['_shadow_obstacle_context'](record,cfg)
        self.assertIsNotNone(s['ns']['_shadow_obstacle_collision'](old['selected']['path'],context,cfg))
        result = plan(record)
        self.assertEqual(result['status'],'VALID')
        self.assertIsNone(s['ns']['_shadow_obstacle_collision'](result['selected']['path'],context,cfg))
        record['obstacles']['points_xyz_m'] = [( .75,-.6+i*.02,0.) for i in range(61)]
        result = plan(record)
        self.assertEqual(result['status'],'ALL_COLLIDE')
        self.assertTrue(result['stop_required'])

    def test_motion_alignment_epoch_age_and_remaining_support(self):
        old = s['plan'](job())
        record = next_job(old,0.)
        record['shadow_pose'] = [.2,0.,0.]
        cfg = s['Settings'](weight_consistency=4.)
        ctx = s['ns']['_shadow_continuity_context'](record,cfg)
        self.assertIsNotNone(ctx)
        point = ctx[0].at(ctx[1])[0]
        self.assertAlmostEqual(point[0],0.)
        self.assertAlmostEqual(point[1],0.)
        for key,value in [('epoch',2),('stamp_s',90.),('stamp_s',101.),('progress_m',1.3)]:
            altered = deepcopy(record)
            altered['execution_reference'][key] = value
            self.assertIsNone(s['ns']['_shadow_continuity_context'](altered,cfg))

    def test_real_worker_transports_continuity_and_selection(self):
        record = next_job(s['plan'](job(-.1)),.1)
        record['settings'].update(weight_consistency=4.,switch_cost_margin=.05)
        expected = plan(record)
        process = subprocess.run([sys.executable,str(s['source']),'--shadow-worker'],
            input=json.dumps(record)+'\n',capture_output=True,text=True,timeout=15)
        self.assertEqual(process.returncode,0,process.stderr)
        result = json.loads(process.stdout)
        self.assertEqual(result['status'],'VALID')
        self.assertEqual(result['path_continuity'],expected['path_continuity'])
        self.assertEqual(result['selected']['id'],expected['selected']['id'])

    def test_node_sends_executing_geometry_and_keeps_the_mpc_controller(self):
        m = runpy.run_path(str(Path(__file__).with_name('test_mpc_planner_control.py')))
        node = m['make_node'](shadow_enabled=True,planner=True,mode='mpc-exec-lattice21')
        m['run'](node,6)
        controller,active = node.mpc_controller,deepcopy(node.planner_execution.active)
        node.shadow_settings = replace(node.shadow_settings,weight_consistency=4.,switch_cost_margin=.05,
            min_turn_radius_m=1.,length_m=1.4,sample_count=48)
        node.shadow_last_submit_s = -math.inf
        node._shadow_offer()
        submitted,result = node.shadow_worker.jobs[-1],node.shadow_worker.results[-1]
        self.assertEqual(submitted['execution_reference']['path'],[list(p) for p in active['path']])
        self.assertEqual(submitted['execution_reference']['pose'],active['pose'])
        self.assertTrue(result['path_continuity']['available'])
        node._shadow_poll()
        after = m['run'](node,2)[-1]
        self.assertEqual(after['reference_selection'],'planner')
        self.assertEqual(after['branch'],'solved')
        self.assertGreater(node.action_publisher.messages[-1].drive,0.)
        self.assertIs(node.mpc_controller,controller)


if __name__ == '__main__':
    unittest.main()
