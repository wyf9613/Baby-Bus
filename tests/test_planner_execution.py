"""Opt-in lattice execution adapter geometry, without ROS.

Ported from origin/mvp2.1 3652e9b (tests/test_planner_execution.py) on
2026-10-16: GeometryExecutionChecks verbatim except where marked. Its
RuntimeExecutionChecks drive the MVP PID node; this branch's MPC consumer
is covered by test_mpc_planner_control.py.
"""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

shadow = runpy.run_path(str(Path(__file__).with_name('test_shadow_lattice.py')))
ns = shadow['ns']
Manager, Settings = ns['PlannerExecutionManager'], ns['PlannerControlSettings']
Cfg = ns['ShadowSettings']


def result(stamp=10., epoch=1, y=0., length=1., job_id=1, pose=(0.,0.,0.)):
    return dict(status='VALID', input_stamp_s=stamp, measurement_stamp_s=stamp-.1,
        job_id=job_id, execution_anchor=dict(pose=list(pose), odometry_epoch=epoch),
        selected=dict(path=[(length*i/40, y, 0.) for i in range(41)]))


def state(stamp=10., speed=.2):
    return dict(timestamp_s=stamp, frame_id='base_link', speed_valid=True,
                yaw_rate_valid=True, speed_mps=speed, yaw_rate_rps=0.)


def road(stamp=10.):
    return dict(timestamp_s=stamp, frame_id='base_link', time_aligned=True,
        boundary_source=dict(left='observed', right='observed'),
        left_boundary_xy=[(-.4,.5),(2.,.5)], right_boundary_xy=[(-.4,-.5),(2.,-.5)])


class GeometryExecutionChecks(unittest.TestCase):
    def select(self, manager, stamp=10., pose=(0.,0.,0.), epoch=1, boundaries=None, speed=.2):
        return manager.select(road(stamp) if boundaries is None else boundaries, state(stamp,speed),
                              pose, epoch, Cfg(), .35, .2, .05, .25)

    def test_default_is_baseline_and_switch_rejects_non_boolean(self):
        self.assertFalse(Settings().enabled)
        with self.assertRaises(ValueError):
            Settings(enabled='true')

    def test_actual_worker_retains_original_execution_anchor_and_mode(self):
        import json
        import subprocess
        import sys
        value=shadow['job']()
        value.update(shadow_pose=[0.,0.,0.],shadow_odometry_epoch=1,planner_control_enabled=True)
        process=subprocess.run([sys.executable,str(shadow['source']),'--shadow-worker'],
            input=json.dumps(value)+'\n',text=True,capture_output=True,timeout=15)
        self.assertEqual(process.returncode,0,process.stderr)
        response=json.loads(process.stdout)
        self.assertEqual(response['status'],'VALID')
        self.assertEqual(response['execution_anchor'],{'pose':[0.,0.,0.],'odometry_epoch':1})
        self.assertFalse(response['shadow_only'])
        manager=Manager();manager.accept(response)
        self.assertIsNotNone(self.select(manager,stamp=100.2,pose=(.04,0.,0.)))

    def test_delayed_result_aligns_and_retains_source_age_in_fresh_execution_reference(self):
        manager=Manager(); manager.accept(result())
        ref=self.select(manager, stamp=13.2, pose=(.3,0.,0.))
        self.assertTrue(ref['valid'])
        self.assertEqual(ref['timestamp_s'],13.2)
        self.assertEqual(ref['original_input_stamp_s'],10.)
        self.assertEqual(ref['valid_for_s'],.25)
        self.assertAlmostEqual(manager.diagnostics['source_age_s'],3.2)
        self.assertAlmostEqual(manager.diagnostics['remaining_length_m'],.7)
        self.assertAlmostEqual(ref['path']['samples_xy'][0][0],0.)
        self.assertEqual(manager.diagnostics['mode'],'planner')

    def test_alignment_rotation_and_passed_arc_crop_preserve_future_negative_x(self):
        manager=Manager()
        value=result(length=2.)
        value['selected']['path']=[(0.,0.,0.),(.5,0.,0.),(1.,.3,1.),(.5,.7,2.),(-.3,.8,3.)]
        manager.accept(value)
        ref=self.select(manager,pose=(.1,0.,0.))
        self.assertIsNotNone(ref)
        self.assertTrue(any(p[0]<0 for p in ref['path']['samples_xy'][1:]))
        ref=self.select(manager,stamp=10.05,pose=(.2,0.,.2))
        self.assertIsNotNone(ref)
        expected=ns['_shadow_pose_points']([(-.3,.8)],(0,0,0),(.2,0,.2))[0]
        for a,b in zip(ref['path']['samples_xy'][-1],expected): self.assertAlmostEqual(a,b)

    def test_rolling_check_covers_preview_beyond_original_planner_prefix(self):
        manager=Manager(); manager.accept(result())
        boundaries=road()
        # At x=.6 it is outside the initial 0.02 m CG execution check plus body
        # front .25 m, but inside the .35 m preview plus body front.
        boundaries['left_boundary_xy']=[(.6,-.1),(.6,.1)]
        self.assertIsNone(self.select(manager,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['reason'],'known_boundary_collision')
        self.assertGreaterEqual(manager.diagnostics['check_length_m'],.35)

    def test_new_known_wall_rechecked_every_tick_but_inferred_side_is_not_hard_wall(self):
        manager=Manager(); manager.accept(result())
        self.assertIsNotNone(self.select(manager))
        boundaries=road(10.05)
        boundaries.update(right_boundary_xy=[(.3,-.1),(.3,.1)],
                          boundary_source={'left':'absent','right':'width_inferred'})
        self.assertIsNotNone(self.select(manager,stamp=10.05,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['known_boundary_sides'],[])
        boundaries['boundary_source']['right']='observed'
        self.assertIsNone(self.select(manager,stamp=10.05,boundaries=boundaries))

    def test_coverage_failure_and_recovery_without_fixed_age_timeout(self):
        manager=Manager(); manager.accept(result())
        self.assertIsNotNone(self.select(manager,stamp=15.))
        self.assertIsNone(self.select(manager,stamp=15.05,pose=(.7,0.,0.)))
        self.assertEqual(manager.diagnostics['reason'],'preview_coverage_exhausted')
        manager.accept(result(stamp=15.05,job_id=2,pose=(.7,0.,0.)))
        self.assertIsNotNone(self.select(manager,stamp=15.1,pose=(.71,0.,0.)))

    def test_epoch_change_malformed_and_planning_failures_cannot_reuse_old_success(self):
        for status in ('NO_ROAD','ALL_COLLIDE','WORKER_ERROR','TRANSPORT_ERROR'):
            manager=Manager(); manager.accept(result())
            manager.accept(dict(status=status,job_id=2))
            self.assertIsNone(self.select(manager))
            self.assertEqual(manager.diagnostics['reason'],'planner_'+status.lower())
        manager=Manager(); manager.accept(result())
        self.assertIsNone(self.select(manager,epoch=2))
        self.assertEqual(manager.diagnostics['reason'],'motion_epoch_changed')
        manager.accept({**result(),'execution_anchor':None})
        self.assertIsNone(self.select(manager))
        self.assertEqual(manager.diagnostics['reason'],'planner_invalid_execution_payload')

    def test_unaligned_known_boundary_falls_back_without_using_wrong_frame(self):
        manager=Manager(); manager.accept(result())
        boundaries=road(9.9)
        self.assertIsNone(self.select(manager,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['reason'],'known_boundary_alignment_unavailable')

    def test_geometric_connection_checks_actual_body_to_offset_path(self):
        manager=Manager(); manager.accept(result(y=.5))
        boundaries=road()
        boundaries.update(left_boundary_xy=[(-.1,.25),(.1,.25)],right_boundary_xy=None)
        self.assertIsNone(self.select(manager,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['reason'],'known_boundary_collision')


if __name__=='__main__':
    unittest.main()
