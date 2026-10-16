"""Pinned module integration, sensor reset/fork and side-channel isolation."""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from offline.mvp21_sim.source import api,parameters,verify_preserved_files,SOURCE_SHA256,ROOT
from offline.mvp21_sim.simulation import MVP21SimulationEnv,recorded_environment,plant_action
from offline.mvp21_sim.__main__ import DEFAULT_ROAD,run
from offline.planner_sim import load_notebook


class MVP21IntegrationChecks(unittest.TestCase):
    def test_pinned_modules_exclude_policy_node_and_preserve_original_files(self):
        active=api()
        for name in ('EstimationPipeline','RoadCenterlinePlanner','PolicyController','ShadowLatticePlanner'):
            self.assertTrue(hasattr(active,name))
        self.assertFalse(hasattr(active,'PolicyNode'))
        self.assertFalse(hasattr(active,'calculate_policy_actions'))
        self.assertTrue(verify_preserved_files()['unchanged'])

    def test_new_estimator_survives_reset_and_snapshot_fork(self):
        with MVP21SimulationEnv(load_notebook(with_obstacles=False)) as env:
            env.reset(seed=7)
            self.assertEqual(env.estimator_sha256,SOURCE_SHA256)
            self.assertEqual(env.estimator.settings.road_geometry,'ordered_xy')
            env.step((.01,.05))
            snapshot=env.snapshot()
            with env.fork() as branch:
                left=env.step((.01,.05))[0]
                right=branch.step((.01,.05))[0]
                self.assertEqual(left['estimation']['state'],right['estimation']['state'])
                self.assertEqual(list(env._backend.car._state_values()),list(branch._backend.car._state_values()))
            env.restore(snapshot)
            self.assertEqual(env.estimator_sha256,SOURCE_SHA256)
            env.reset(seed=8)
            self.assertEqual(env.estimator_sha256,SOURCE_SHA256)

    def test_shadow_uses_actual_upstream_forward_geometry_offer(self):
        active=api();jobs=[]
        port=active.ShadowJobSource()
        port.shadow_settings=active.ShadowSettings(**parameters()['shadow'])
        port.planning_settings=active.PlanningSettings(**parameters()['planning'])
        port.shadow_generation,port.shadow_job_id=1,0
        port.shadow_last_submit_s,port.shadow_handoff_estimate_s=-math.inf,0.
        port._monotonic=lambda:10.
        port.shadow_worker=SimpleNamespace(submit=jobs.append)
        forward=dict(centerline_xy=[(.5,0.),(1.,0.)],left_boundary_xy=[(.5,.5),(1.,.5)],
            right_boundary_xy=[(.5,-.5),(1.,-.5)],boundary_source={'left':'observed','right':'observed'},
            lane_width_m=1.,status='near_field_unobserved')
        port.estimation_output=dict(road={'forward_geometry':forward,'valid':False},
            state={'speed_mps':.1,'timestamp_s':10.})
        port._shadow_offer()
        self.assertEqual(jobs[0]['road']['centerline_xy'],forward['centerline_xy'])
        result=active.ShadowLatticePlanner(port.shadow_settings).plan(jobs[0])
        self.assertEqual(result['status'],'VALID')
        port._shadow_offer()
        self.assertEqual(len(jobs),1)

    def test_actuator_conversion_changes_sign_without_rewriting_upstream_controller(self):
        self.assertEqual(plant_action(.3,-.4),(.3,.4))
        self.assertEqual(plant_action(.3,-.4,actuator_steering_sign=1.),(.3,-.4))
        with self.assertRaises(ValueError):plant_action(.3,float('nan'))
        self.assertEqual(parameters()['control']['mvp_drive_feedforward'],.3)
        self.assertEqual(parameters()['control']['mvp_steering_direction'],-1.)

    def test_measured_road_uses_new_estimation_and_same_recorded_station(self):
        road=json.loads(DEFAULT_ROAD.read_text())
        with recorded_environment(road) as env:
            self.assertEqual(env.estimator_sha256,SOURCE_SHA256)
            self.assertAlmostEqual(env._info['evaluation']['ground_truth']['reference_line_progress_m'],
                                   road['original_initial_progress_m'])
            self.assertEqual(len(env._backend._resolved_cones.records),42)

    def test_shadow_results_cannot_change_baseline_actions(self):
        road=json.loads(DEFAULT_ROAD.read_text())
        with tempfile.TemporaryDirectory(dir=ROOT/'.verification',prefix='mvp21-test-') as directory:
            directory=Path(directory)
            self.assertTrue(directory.resolve().is_relative_to((ROOT/'.verification').resolve()))
            on=run(road,directory/'on',max_steps=4,shadow_enabled=True,path_source='perception')
            off=run(road,directory/'off',max_steps=4,shadow_enabled=False,path_source='perception')
            left=[json.loads(line) for line in (directory/'on/trace.jsonl').read_text().splitlines()]
            right=[json.loads(line) for line in (directory/'off/trace.jsonl').read_text().splitlines()]
            self.assertEqual([r['plant_action'] for r in left],[r['plant_action'] for r in right])
            self.assertEqual(on['final_progress_m'],off['final_progress_m'])
            self.assertEqual(on['valid_reference_count'],4)
            self.assertEqual(on['shadow']['status_counts'],{'VALID':1})
            self.assertEqual(off['shadow']['calls'],0)




if __name__=='__main__':
    unittest.main()
