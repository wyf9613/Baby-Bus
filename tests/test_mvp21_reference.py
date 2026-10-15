"""Lattice frame reuse, bounded lifetimes and causal delayed task delivery."""
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mvp21_policy.estimation import EstimationSettings
from mvp21_policy.lattice import ShadowLatticePlanner,ShadowSettings,_shadow_collision
from mvp21_policy.motion import EstimationMotionHistory
from mvp21_policy.planning import PlanningSettings
from mvp21_policy.reference import LatticeReferenceManager
from mvp21_policy.transport import ShadowJobSource
from offline.mvp21_sim.delivery import SimulatedPlannerWorker
from offline.mvp21_sim.__main__ import DEFAULT_ROAD
from offline.mvp21_sim.__main__ import run
from offline.mvp21_sim.simulation import recorded_environment,ideal_observation


def motion(speed=.5,rate=0.):
    history=EstimationMotionHistory(EstimationSettings(max_source_age_s=1.5,
        motion_max_interval_s=.5,motion_max_gap_s=.25))
    for i in range(41):
        t=i*.05
        history.observe('wheel_speed',speed,('w',i),t,t)
        history.observe('imu_angular_velocity',rate,('g',i),t,t)
    return history


def envelope(now):
    return dict(valid=True,timestamp_s=now,valid_for_s=.2,frame_id='base_link',
        vehicle_reference_point='cg_ground_projection',target_speed_mps=.2,
        stop_requested=False,simulation_only=False,
        path=dict(type='ORDERED_CENTERLINE',preview_m=.35,samples_xy=[(0.,0.),(2.,0.)]))


def state(now):
    return dict(valid=True,speed_valid=True,yaw_rate_valid=True,timestamp_s=now,frame_id='base_link')


def result(identifier=1,stamp=0.,hold=1.1,length=2.):
    return dict(generation=1,job_id=identifier,status='VALID',input_stamp_s=stamp,hold_time_s=hold,
        check_length_m=length,score_length_m=length,measurement_stamp_s=stamp,
        selected=dict(path=[(0.,0.,0.),(length,0.,0.)],id=0,family='follow',collision=None))


def job(identifier=1):
    return dict(settings=vars(ShadowSettings(enabled=True,sample_count=8)),generation=1,job_id=identifier,
        stamp_s=0.,submitted_monotonic_s=0.,preview_m=.35,state=dict(speed_mps=.2),
        road=dict(centerline_xy=[(0.,0.),(1.,0.)],left_boundary_xy=[(0.,.5),(1.,.5)],
            right_boundary_xy=[(0.,-.5),(1.,-.5)],boundary_source=dict(left='observed',right='observed')))


class LatticeReferenceChecks(unittest.TestCase):
    def test_segmented_history_matches_a_known_circular_arc(self):
        history=motion(.5,.2)
        self.assertEqual(history.pose_between(0.,1.2)[1],'motion_interval_too_long')
        pose,problem=history.pose_between_segments(0.,1.2)
        self.assertIsNone(problem)
        self.assertAlmostEqual(pose['dx_m'],2.5*math.sin(.24))
        self.assertAlmostEqual(pose['dy_m'],2.5*(1-math.cos(.24)))
        self.assertAlmostEqual(pose['yaw_rad'],.24)
        self.assertAlmostEqual(pose['travelled_distance_m'],.6)

    def test_delayed_path_is_transformed_and_control_ticks_cannot_extend_its_deadline(self):
        history=motion();manager=LatticeReferenceManager()
        self.assertTrue(manager.accept(result(),.3,history))
        reference=manager.select(envelope(.35),state(.35),history,.35)
        self.assertAlmostEqual(reference['path']['samples_xy'][0][0],-.175)
        self.assertEqual(reference['lattice_reference']['input_timestamp_s'],0.)
        self.assertAlmostEqual(reference['lattice_reference']['deadline_s'],1.1)
        reference=manager.select(envelope(1.05),state(1.05),history,1.05)
        self.assertAlmostEqual(reference['valid_for_s'],.05)
        with self.assertRaisesRegex(ValueError,'expired'):
            manager.select(envelope(1.1),state(1.1),history,1.1)

    def test_original_measurement_age_can_expire_before_hold_deadline(self):
        history=motion();manager=LatticeReferenceManager(max_source_age_s=.7)
        packet=result();packet['measurement_stamp_s']=-.2
        manager.accept(packet,.3,history)
        self.assertAlmostEqual(manager.deadline_s,.5)
        with self.assertRaisesRegex(ValueError,'expired'):
            manager.select(envelope(.5),state(.5),history,.5)

    def test_old_runs_and_jobs_cannot_replace_the_selected_path(self):
        history=motion();manager=LatticeReferenceManager()
        manager.accept(result(2,.1),.3,history)
        old=result(1);old['status']='ALL_COLLIDE'
        self.assertFalse(manager.accept(old,.4,history))
        old=result(3,.2);old['generation']=0
        self.assertFalse(manager.accept(old,.4,history))
        self.assertEqual(manager.job_id,2)
        self.assertTrue(manager.accept(result(3,.2),.4,history))

    def test_new_collision_result_does_not_fall_back_to_previous_path(self):
        history=motion();manager=LatticeReferenceManager()
        manager.accept(result(),.1,history)
        invalid=result(2,.2);invalid['status']='ALL_COLLIDE'
        with self.assertRaisesRegex(ValueError,'ALL_COLLIDE'):
            manager.accept(invalid,.3,history)
        self.assertIsNone(manager.points)

    def test_motion_reset_gap_and_exhausted_preview_reject(self):
        history=motion();manager=LatticeReferenceManager()
        manager.accept(result(length=.4),.1,history)
        with self.assertRaisesRegex(ValueError,'support_exhausted'):
            manager.select(envelope(.2),state(.2),history,.2)
        manager.accept(result(2,.2),.2,history)
        history.clear()
        with self.assertRaisesRegex(ValueError,'history_reset'):
            manager.select(envelope(.25),state(.25),history,.25)
        history=motion()
        history.samples['wheel_speed']=[(0.,.5)]
        self.assertEqual(history.pose_between_segments(0.,1.2)[1],'motion_history_gap')

    def test_future_expired_and_current_frame_mismatch_reject(self):
        history=motion();manager=LatticeReferenceManager()
        invalid=result();invalid['measurement_stamp_s']=.1
        with self.assertRaisesRegex(ValueError,'measurement_time_invalid'):
            manager.accept(invalid,.3,history)
        with self.assertRaisesRegex(ValueError,'expired_or_future'):
            manager.accept(result(stamp=.4),.3,history)
        with self.assertRaisesRegex(ValueError,'expired_or_future'):
            manager.accept(result(),1.2,history)
        manager.accept(result(),.3,history)
        with self.assertRaisesRegex(ValueError,'frame_or_time_mismatch'):
            manager.select(envelope(.35),state(.3),history,.35)

    def test_fixed_delay_hides_results_and_waiting_queue_keeps_latest_only(self):
        worker=SimulatedPlannerWorker(delay_s=.3)
        worker.submit(job(1));worker.now_s=.1;worker.submit(job(2))
        worker.now_s=.2;worker.submit(job(3))
        self.assertIsNone(worker.poll(.25))
        self.assertEqual(worker.poll(.3)['result']['job_id'],1)
        self.assertIsNone(worker.poll(.55))
        self.assertEqual(worker.poll(.6)['result']['job_id'],3)
        self.assertEqual([p['result']['job_id'] for p in worker.completed],[1,3])

    def test_one_hz_submission_does_not_skip_a_tick_due_to_roundoff(self):
        offer=ShadowJobSource();jobs=[]
        offer.shadow_settings=ShadowSettings(enabled=True)
        offer.planning_settings=PlanningSettings()
        offer.shadow_last_submit_s=-math.inf;offer.shadow_handoff_estimate_s=.1
        offer.shadow_generation,offer.shadow_job_id=1,0
        offer.shadow_worker=SimpleNamespace(submit=jobs.append)
        offer._monotonic=lambda:offer.now_s
        offer.estimation_output=dict(road={},state={})
        for timestamp in (1.3,1.35,2.3,3.3):
            offer.now_s=timestamp;offer._shadow_offer()
        self.assertEqual([p['submitted_monotonic_s'] for p in jobs],[1.3,2.3,3.3])

    def test_generated_coverage_checks_and_scores_the_controller_readable_tail(self):
        road=json.loads(DEFAULT_ROAD.read_text())
        with recorded_environment(road) as env:
            ideal=ideal_observation(env,env._frame)
            self.assertEqual(ideal['estimation']['road']['measurement_timestamp_s'],ideal['timestamp_s'])
            self.assertEqual(ideal['estimation']['road']['source_age_s'],0.)
            estimates=env._frame['estimation'];jobs=[]
            offer=ShadowJobSource()
            offer.shadow_settings=ShadowSettings(enabled=True)
            offer.planning_settings=PlanningSettings(preview_m=.35)
            offer.shadow_last_submit_s=-math.inf;offer.shadow_handoff_estimate_s=.1
            offer.shadow_generation,offer.shadow_job_id=1,0
            offer.shadow_worker=SimpleNamespace(submit=jobs.append)
            offer._monotonic=lambda:env._frame['timestamp_s'];offer.estimation_output=estimates
            offer._shadow_offer();packet=jobs[0]
            prefix=ShadowLatticePlanner(offer.shadow_settings).plan(packet)
            self.assertEqual(prefix['check_length_m'],0.)
            self.assertEqual(prefix['score_length_m'],.2)
            settings=ShadowSettings(enabled=True,coverage_mode='generated',length_m=1.3)
            full=ShadowLatticePlanner(settings).plan(packet)
            self.assertEqual(full['status'],'VALID')
            self.assertEqual(full['check_length_m'],full['generated_length_m'])
            self.assertEqual(full['score_length_m'],full['generated_length_m'])
            self.assertIsNone(_shadow_collision(full['selected']['path'],full['generated_length_m'],
                {side:packet['road'][side+'_boundary_xy'] for side in ('left','right')},settings))

    def test_late_delivery_cannot_extend_first_path_or_fall_back_to_centerline(self):
        road=json.loads(DEFAULT_ROAD.read_text())
        with tempfile.TemporaryDirectory(dir=ROOT/'.verification',prefix='mvp21-late-delivery-') as directory:
            output=Path(directory)/'run'
            summary=run(road,output,
                road_input='map',delivery_delay_s=.3,handoff_budget_s=.1,max_steps=60)
            self.assertEqual(summary['fault'],'lattice_reference_expired_or_clock_reversed')
            self.assertEqual(summary['delivery']['accepted'],1)
            self.assertEqual(summary['lattice_cadence'],'cached')
            failure=json.loads((output/'failure.json').read_text())
            self.assertTrue(failure['stop_requested'])
            self.assertEqual(failure['stop_action'],[0.,0.])
            trace=[json.loads(line) for line in (output/'trace.jsonl').read_text().splitlines()]
            tracking=[row for row in trace if row['reference'] is not None]
            self.assertTrue(tracking)
            self.assertTrue(all(row['reference']['planner_version']=='mvp21_lattice_cached' for row in tracking))


if __name__=='__main__':
    unittest.main()
