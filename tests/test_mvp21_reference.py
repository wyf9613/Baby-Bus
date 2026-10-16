"""Motion integration, causal delivery, exact-rate scheduling and explicit planner profiles."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mvp21_policy.estimation import EstimationSettings
from mvp21_policy.lattice import ShadowLatticePlanner,ShadowSettings,_shadow_collision
from mvp21_policy.motion import EstimationMotionHistory
from mvp21_policy.planning import PlanningSettings
from mvp21_policy.transport import ShadowJobSource
from offline.mvp21_sim.delivery import SimulatedPlannerWorker
from offline.mvp21_sim.__main__ import DEFAULT_ROAD
from offline.mvp21_sim.simulation import recorded_environment,ideal_observation


def motion(speed=.5,rate=0.):
    history=EstimationMotionHistory(EstimationSettings(max_source_age_s=1.5,
        motion_max_interval_s=.5,motion_max_gap_s=.25))
    for i in range(41):
        t=i*.05
        history.observe('wheel_speed',speed,('w',i),t,t)
        history.observe('imu_angular_velocity',rate,('g',i),t,t)
    return history


def job(identifier=1):
    return dict(settings=vars(ShadowSettings(enabled=True,sample_count=8)),generation=1,job_id=identifier,
        stamp_s=0.,submitted_monotonic_s=0.,preview_m=.35,state=dict(speed_mps=.2),
        road=dict(centerline_xy=[(0.,0.),(1.,0.)],left_boundary_xy=[(0.,.5),(1.,.5)],
            right_boundary_xy=[(0.,-.5),(1.,-.5)],boundary_source=dict(left='observed',right='observed')))


class PlannerDeliveryChecks(unittest.TestCase):
    def test_segmented_history_matches_a_known_circular_arc(self):
        history=motion(.5,.2)
        self.assertEqual(history.pose_between(0.,1.2)[1],'motion_interval_too_long')
        pose,problem=history.pose_between_segments(0.,1.2)
        self.assertIsNone(problem)
        self.assertAlmostEqual(pose['dx_m'],2.5*math.sin(.24))
        self.assertAlmostEqual(pose['dy_m'],2.5*(1-math.cos(.24)))
        self.assertAlmostEqual(pose['yaw_rad'],.24)
        self.assertAlmostEqual(pose['travelled_distance_m'],.6)


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
            settings=ShadowSettings(enabled=True,collision_coverage='generated',score_coverage='generated',length_m=1.3)
            full=ShadowLatticePlanner(settings).plan(packet)
            self.assertEqual(full['status'],'VALID')
            self.assertEqual(full['check_length_m'],full['generated_length_m'])
            self.assertEqual(full['score_length_m'],full['generated_length_m'])
            self.assertIsNone(_shadow_collision(full['selected']['path'],full['generated_length_m'],
                {side:packet['road'][side+'_boundary_xy'] for side in ('left','right')},settings))


if __name__=='__main__':
    unittest.main()
