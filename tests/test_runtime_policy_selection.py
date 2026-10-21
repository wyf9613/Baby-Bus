"""Frozen policy selection, submission-frame inputs and shared PID execution."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from mvp21_policy.lattice import ShadowLatticePlanner,ShadowSettings
from mvp21_policy.control import PolicyController,ControlSettings
from offline.mvp21_sim.policy_selection import FrozenPolicySelector,DEFAULT_MODEL
from offline.mvp21_sim.delivery import SimulatedPlannerWorker
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.source import parameters
from offline.mvp21_sim.validate_roads import read_rows,timing_audit
from offline.lattice_training.validation_roads import validation_case


def job(identifier=1):
    return dict(settings=vars(ShadowSettings(enabled=True,sample_count=8)),generation=1,job_id=identifier,
        stamp_s=1.,submitted_monotonic_s=0.,preview_m=.35,state=dict(speed_mps=.2),
        shadow_pose=[0.,0.,0.],shadow_odometry_epoch=0,
        road=dict(centerline_xy=[(0.,0.),(2.,0.)],left_boundary_xy=[(0.,.6),(2.,.6)],
            right_boundary_xy=[(0.,-.6),(2.,-.6)],boundary_source=dict(left='observed',right='observed')))


def observation(speed=.2):
    return dict(frame_id='base_link',timestamp_s=1.,estimation=dict(road=job()['road'],
        state=dict(speed_mps=speed,yaw_rate_rps=0.,speed_valid=True,yaw_rate_valid=True),
        obstacles=dict(available=True,points_xyz_m=[[1.,.2,0.]])))


class RuntimePolicyChecks(unittest.TestCase):
    def test_ranking_changes_selection_without_native_cost_or_history_inputs(self):
        selector=FrozenPolicySelector(DEFAULT_MODEL)
        result=ShadowLatticePlanner(ShadowSettings(**job()['settings'])).plan(job())
        original=deepcopy(result);seen=[]
        def scorer(record):
            seen.append(deepcopy(record))
            return record['candidate_path']['samples'][-1][2]
        with patch.object(selector.model,'score',side_effect=scorer):
            first=selector.select(deepcopy(result),observation())
            changed=deepcopy(result)
            for candidate in changed['candidates']:candidate['cost']=1e9-candidate['id']
            other=observation();other['previous_path']=[[999,999]];other['ground_truth']={'x':999}
            second=selector.select(changed,other)
        self.assertEqual(first['policy_selection']['status'],'SELECTED')
        self.assertNotEqual(first['selected']['id'],original['selected']['id'])
        self.assertEqual(first['selected']['id'],second['selected']['id'])
        self.assertEqual(first['policy_selection']['scores'],second['policy_selection']['scores'])
        authored=next(c for c in original['candidates'] if c['id']==first['selected']['id'])
        self.assertEqual(first['selected']['path'],authored['path'])
        self.assertEqual(set(seen[0]),{'schema_version','current_state','current_road_and_obstacles','candidate_path'})
        self.assertEqual(seen[0]['current_road_and_obstacles']['road']['centerline']['points_xy'],[[0.,0.],[2.,0.]])

    def test_hard_rejections_are_never_scored_and_errors_clear_selection(self):
        selector=FrozenPolicySelector(DEFAULT_MODEL)
        result=ShadowLatticePlanner(ShadowSettings(**job()['settings'])).plan(job())
        keep=result['candidates'][-1]['id']
        for candidate in result['candidates']:
            candidate['collision']=None if candidate['id']==keep else {'side':'left'}
        with patch.object(selector.model,'score',return_value=.5) as score:
            selected=selector.select(deepcopy(result),observation())
            self.assertEqual(score.call_count,1)
        self.assertEqual(selected['selected']['id'],keep)
        with patch.object(selector.model,'score',return_value=float('nan')):
            failed=selector.select(deepcopy(result),observation())
        self.assertEqual(failed['status'],'POLICY_ERROR');self.assertIsNone(failed['selected'])
        with self.assertRaisesRegex(ValueError,'cruise-speed'):
            FrozenPolicySelector(DEFAULT_MODEL,cruise_speed_mps=.5)

    def test_waiting_job_keeps_submission_snapshot_and_delivery_is_causal(self):
        selector=FrozenPolicySelector(DEFAULT_MODEL)
        worker=SimulatedPlannerWorker(delay_s=.3,candidate_selector=selector)
        source=observation(.2);worker.policy_observation=source;worker.submit(job(1))
        source['estimation']['state']['speed_mps']=.4
        worker.now_s=.1;worker.submit(job(2))
        source['estimation']['state']['speed_mps']=.9
        self.assertIsNone(worker.poll(.25))
        first=worker.poll(.3);second=worker.poll(.6)
        self.assertEqual(first['job']['policy_observation']['estimation']['state']['speed_mps'],.2)
        self.assertEqual(second['job']['policy_observation']['estimation']['state']['speed_mps'],.4)
        self.assertEqual(second['result']['policy_selection']['status'],'SELECTED')

    def test_clock_driven_policy_uses_shared_pid_and_cold_start_is_not_bypassed(self):
        case=validation_case('validation_uturn_original_left')
        # This fixture deliberately uses the historical 0.2 m/s model; the
        # shipped RNN configuration now explicitly targets 0.5 m/s.
        config=parameters();config['planning']['cruise_speed_mps']=.2
        with tempfile.TemporaryDirectory() as temporary, patch(
                'offline.mvp21_sim.runtime.parameters',side_effect=lambda:deepcopy(config)):
            output=Path(temporary)/'run'
            summary=run_runtime(case,output,max_steps=40,latency_steps=1,seed=713,
                detailed_trace=True,stop_tail_s=.1,policy_model_path=DEFAULT_MODEL)
            self.assertTrue(timing_audit(output)['checks_passed'])
            self.assertTrue(summary['learned_scorer_used']);self.assertGreater(summary['policy_reference_count'],0)
            controller=PolicyController(ControlSettings(**parameters()['control']))
            for index,row in enumerate(r for r in read_rows(output/'trace.jsonl') if r['phase']=='tracking'):
                drive,steer,diag=controller.calculate(row['reference'],row['state'],
                    row['observation']['estimation']['vehicle_params'],row['timestamp_s'],0. if index==0 else .05)
                self.assertAlmostEqual(drive,row['raw_action'][0],places=12)
                self.assertAlmostEqual(steer,row['raw_action'][1],places=12)
            for packet in read_rows(output/'shadow.jsonl'):
                result=packet['result'];self.assertEqual(result['selected']['id'],result['policy_selection']['selected_id'])
            cold=run_runtime(case,Path(temporary)/'cold',max_steps=20,latency_steps=1,seed=713,
                progress=case['cold_start_progress_m'],detailed_trace=True,policy_model_path=DEFAULT_MODEL)
            self.assertTrue(cold['learned_scorer_enabled']);self.assertFalse(cold['learned_scorer_used'])
            self.assertEqual(cold['policy_candidate_calls'],0);self.assertEqual(cold['delivery']['submitted'],0)


if __name__=='__main__':unittest.main()
