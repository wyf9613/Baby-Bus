"""Observed one-sided geometry survives width failure, with motion gates intact."""
from copy import deepcopy
import math
from types import SimpleNamespace
import unittest

from tests import test_estimation as estimation_checks
from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings
from mvp21_policy.planning import PlanningSettings
from mvp21_policy.transport import ShadowJobSource


class SingleBoundaryOutputChecks(unittest.TestCase):
    def pipeline(self):
        return estimation_checks.Pipeline(estimation_checks.Settings(road_geometry='ordered_xy', known_lane_width_m=0.,
            single_side_from_learned_width=True, history_propagation='continuous',
            continuous_road_output=True))

    def step(self, pipeline, now, side='right', **changes):
        points = [(x, .5 if side == 'left' else -.5) for x in estimation_checks.RoadDropoutChecks.XS]
        return estimation_checks.RoadDropoutChecks.step(pipeline, now,
            left=points if side == 'left' else [], right=points if side == 'right' else [], **changes)

    def test_raw_single_side_retains_measurement_without_inferred_width_or_center(self):
        for side in ('left', 'right'):
            pipeline = self.pipeline()
            points = [(x, .5 if side == 'left' else -.5) for x in estimation_checks.RoadDropoutChecks.XS]
            road = pipeline.road(estimation_checks.cones(points if side == 'left' else [],
                                       points if side == 'right' else []), 10_000_000_000, 0.)
            with self.subTest(side=side):
                self.assertFalse(road['valid'])
                self.assertEqual(road['status'], 'single_side_width_unknown')
                self.assertEqual(road['centerline_xy'], [])
                self.assertIsNone(road['lane_width_m'])
                self.assertTrue(road[side+'_boundary_xy'])
                self.assertEqual(road['boundary_source'][side], 'observed')
                self.assertIsNone(road[('left' if side == 'right' else 'right')+'_boundary_xy'])

    def test_delayed_single_boundary_is_aligned_with_original_acquisition_time(self):
        pipeline = self.pipeline()
        first = self.step(pipeline, 10., speed=.2, yaw=.3)['road']
        points = deepcopy(first['forward_geometry']['right_boundary_xy'])
        road = self.step(pipeline, 10.1, stamp=10., speed=.2, yaw=.3)['road']
        forward = road['forward_geometry']
        self.assertFalse(road['valid'])
        self.assertFalse(road['time_aligned'])
        self.assertIsNone(road['timestamp_s'])
        self.assertFalse(road['near_field']['valid'])
        self.assertIsNone(road['right_boundary_xy'])
        self.assertFalse(forward['valid'])
        self.assertTrue(forward['time_aligned'])
        self.assertTrue(forward['geometry_valid'])
        self.assertEqual(forward['measurement_timestamp_s'], 10.)
        self.assertEqual(forward['timestamp_s'], 10.1)
        self.assertEqual(forward['centerline_xy'], [])
        self.assertIsNone(forward['lane_width_m'])
        angle = .3*.1
        dx, dy = .2/.3*math.sin(angle), .2/.3*(1-math.cos(angle))
        for (x, y), moved in zip(points, forward['right_boundary_xy']):
            self.assertAlmostEqual(moved[0], math.cos(angle)*(x-dx)+math.sin(angle)*(y-dy))
            self.assertAlmostEqual(moved[1], -math.sin(angle)*(x-dx)+math.cos(angle)*(y-dy))
        self.assertIsNone(pipeline._learned_lane_width_m)
        self.assertIsNone(pipeline._trusted_road)

    def test_native_handoff_uses_observed_side_while_baseline_corridor_stays_invalid(self):
        for side in ('left', 'right'):
            estimates = self.step(self.pipeline(), 10., side=side, speed=.2, yaw=0.)
            self.assertFalse(estimates['road']['valid'])
            jobs = []
            source = ShadowJobSource()
            source.shadow_settings = ShadowSettings(sample_count=8, rate_hz=20.,
                length_m=1.5, fusion_enabled=False, collision_coverage='generated', score_coverage='generated')
            source.planning_settings = PlanningSettings()
            source.estimation_output = estimates
            source.shadow_last_submit_s = -math.inf
            source.shadow_job_id, source.shadow_generation = 0, 1
            source.shadow_handoff_estimate_s = 0.
            source.shadow_worker = SimpleNamespace(submit=jobs.append)
            source._monotonic = lambda: 10.
            source._shadow_offer()
            self.assertEqual(len(jobs), 1)
            self.assertEqual(jobs[0]['road']['centerline_xy'], [])
            self.assertIsNone(jobs[0]['road']['lane_width_m'])
            result = ShadowLatticePlanner(source.shadow_settings).plan(jobs[0])
            with self.subTest(side=side):
                self.assertEqual(result['status'], 'VALID')
                self.assertEqual(result['known_boundary_sides'], [side])
                self.assertEqual(result['candidate_count'], 8)
                self.assertGreater(result['feasible_count'], 0)

    def test_missing_state_does_not_publish_unaligned_single_boundary(self):
        road = self.step(self.pipeline(), 10., yaw=None)['road']
        self.assertNotIn('forward_geometry', road)
        self.assertIsNone(road['left_boundary_xy'])
        self.assertIsNone(road['right_boundary_xy'])
        self.assertFalse(road['time_aligned'])
        self.assertFalse(road['valid'])

    def test_stale_out_of_order_and_missing_motion_do_not_leak_a_boundary(self):
        for fault in ('stale', 'out_of_order', 'motion_gap'):
            pipeline = self.pipeline()
            self.step(pipeline, 10.)
            if fault == 'motion_gap':
                pipeline.motion.clear()
            now, stamp = (11., 10.) if fault == 'stale' else (10.1, 9.9 if fault == 'out_of_order' else 10.)
            road = self.step(pipeline, now, stamp=stamp)['road']
            with self.subTest(fault=fault):
                self.assertNotIn('forward_geometry', road)
                self.assertIsNone(road['left_boundary_xy'])
                self.assertIsNone(road['right_boundary_xy'])
                self.assertFalse(road['valid'])

    def test_contradictory_two_sided_geometry_is_not_exposed_as_single_boundary(self):
        pipeline = self.pipeline()
        points = estimation_checks.RoadDropoutChecks.XS
        road = estimation_checks.RoadDropoutChecks.step(pipeline, 10.,
            left=[(x,-.5) for x in points], right=[(x,.5) for x in points])['road']
        self.assertFalse(road['valid'])
        self.assertNotIn('forward_geometry', road)
        self.assertIsNone(road['left_boundary_xy'])
        self.assertIsNone(road['right_boundary_xy'])


if __name__ == '__main__':
    unittest.main()
