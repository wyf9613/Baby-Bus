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
from offline.mvp21_sim.planner_control import selected_reference
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
            on=run(road,directory/'on',max_steps=4,shadow_enabled=True)
            off=run(road,directory/'off',max_steps=4,shadow_enabled=False)
            left=[json.loads(line) for line in (directory/'on/trace.jsonl').read_text().splitlines()]
            right=[json.loads(line) for line in (directory/'off/trace.jsonl').read_text().splitlines()]
            self.assertEqual([r['plant_action'] for r in left],[r['plant_action'] for r in right])
            self.assertEqual(on['final_progress_m'],off['final_progress_m'])
            self.assertEqual(on['valid_reference_count'],4)
            self.assertEqual(on['shadow']['status_counts'],{'VALID':1})
            self.assertEqual(off['shadow']['calls'],0)

    def test_lattice_handoff_keeps_geometry_without_refitting_and_rejects_diagnostic_candidate(self):
        envelope=dict(valid=True,timestamp_s=4.,valid_for_s=.2,target_speed_mps=.2,
            stop_requested=False,path=dict(preview_m=.35,samples_xy=[(0.,0.),(1.,0.)]))
        result=dict(status='VALID',input_stamp_s=4.,check_length_m=.1,score_length_m=.2,
            generated_length_m=1.,selected=dict(id=7,family='halton',
                path=[(0.,0.,0.),(.2,.1,.5),(.1,.4,1.8)]))
        before=deepcopy((envelope,result))
        reference=selected_reference(envelope,result)
        self.assertEqual(reference['path']['samples_xy'],[(0.,0.),(.2,.1),(.1,.4)])
        self.assertAlmostEqual(reference['path']['range'][1],math.hypot(.2,.1)+math.hypot(-.1,.3))
        self.assertEqual(reference['valid_for_s'],.2)
        self.assertEqual((envelope,result),before)
        result['status']='ALL_COLLIDE'  # Still contains a selected diagnostic candidate.
        rejected=selected_reference(envelope,result)
        self.assertFalse(rejected['valid'])
        self.assertIsNone(rejected['path'])
        self.assertEqual(rejected['target_speed_mps'],0.)
        result['status']='VALID';result['input_stamp_s']=3.9
        self.assertEqual(selected_reference(envelope,result)['reason'],'lattice_input_time_mismatch')
        result['input_stamp_s']=4.;result['selected']['path']=[(0.,0.,0.),(0.,0.,0.)]
        self.assertEqual(selected_reference(envelope,result)['reason'],'lattice_selected_path_invalid')

    def test_fresh_lattice_cycles_and_literal_one_hz_handoff(self):
        road=json.loads(DEFAULT_ROAD.read_text())
        with tempfile.TemporaryDirectory(dir=ROOT/'.verification',prefix='mvp21-lattice-test-') as directory:
            directory=Path(directory)
            fresh=run(road,directory/'fresh',max_steps=2,path_source='lattice',lattice_cadence='control')
            stale=run(road,directory/'stale',max_steps=2,path_source='lattice',lattice_cadence='upstream')
            self.assertEqual(fresh['valid_reference_count'],2)
            self.assertEqual(fresh['shadow']['calls'],2)
            self.assertTrue(fresh['shadow']['influences_actions'])
            self.assertEqual(fresh['lattice_generation_rate_hz'],20.)
            self.assertEqual(fresh['shadow']['effective_settings']['rate_hz'],1.)
            self.assertEqual(stale['valid_reference_count'],1)
            self.assertEqual(stale['fault'],'control:control_frame_or_time_mismatch')


if __name__=='__main__':
    unittest.main()
