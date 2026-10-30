"""Both track colour conventions, ambiguity, and run-scoped association."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mvp21_policy.estimation import EstimationPipeline, EstimationSettings
from mvp21_policy.execution import priming_motion_history


def settings(**changes):
    values = dict(colour_assignment="auto", road_geometry="ordered_xy",
                  road_min_coverage_m=0.3, single_side_from_learned_width=True)
    return EstimationSettings(**{**values, **changes})


def batch(left_colour=2, center=0.0, width=0.8):
    return dict(detections=[(x, center+offset, 0.0, colour, 0.95)
        for colour, offset in ((left_colour, width/2), (3-left_colour, -width/2))
        for x in (0.3, 0.6, 0.9, 1.2, 1.5, 1.8)])


class ColourAssignmentChecks(unittest.TestCase):
    def test_both_conventions_ignore_the_configured_fixed_preference(self):
        for geometry in ("ordered_xy", "y_of_x"):
            for physical_left in (1, 2):
                for configured_left in (1, 2):
                    with self.subTest(geometry=geometry, physical_left=physical_left,
                                      configured_left=configured_left):
                        pipeline = EstimationPipeline(settings(road_geometry=geometry,
                            left_colour=configured_left, right_colour=3-configured_left))
                        road = pipeline.road(batch(physical_left), 10_000_000_000, 0.0)
                        self.assertTrue(road["valid"], road["status"])
                        self.assertEqual(road["visibility"], "both")
                        self.assertEqual(road["colour_assignment"]["left_colour"], physical_left)
                        self.assertAlmostEqual(road["lane_width_m"], 0.8)
                        self.assertTrue(all(abs(p[1]) < 1e-6 for p in road["centerline_xy"]))

    def test_two_boundaries_on_same_body_side_use_geometry_not_y_sign(self):
        for left in (1, 2):
            pipeline = EstimationPipeline(settings())
            road = pipeline.road(batch(left, center=-1.0), 10_000_000_000, 0.0)
            self.assertTrue(road["valid"], road["status"])
            self.assertEqual(pipeline.left_colour, left)

    def test_single_side_cannot_lock_even_with_a_configured_width(self):
        pipeline = EstimationPipeline(settings(known_lane_width_m=0.8))
        one = batch()
        one["detections"] = [p for p in one["detections"] if p[3] == 2]
        road = pipeline.road(one, 10_000_000_000, 0.0)
        self.assertFalse(road["valid"])
        self.assertEqual(road["status"], "boundary_colour_assignment_pending")
        self.assertIsNone(road["colour_assignment"]["left_colour"])
        self.assertFalse(pipeline._colour_assignment_locked)
        road = pipeline.road(batch(1), 10_100_000_000, 0.0)
        self.assertTrue(road["valid"])
        self.assertEqual(pipeline.left_colour, 1)

    def test_assignment_survives_partial_frames_and_history_clears(self):
        for left in (1, 2):
            pipeline = EstimationPipeline(settings())
            road = pipeline._history_road(batch(left), 10_000_000_000, 0.0, 10.0)
            self.assertTrue(road["valid"])
            learned = pipeline._learned_lane_width_m
            one = batch(left)
            one["detections"] = [p for p in one["detections"] if p[3] == left]
            road = pipeline.road(one, 10_100_000_000, 0.0)
            self.assertTrue(road["single_side_inferred_from_learned_width"])
            self.assertEqual(road["visibility"], "left_only")
            self.assertAlmostEqual(road["lane_width_m"], learned)
            pipeline._clear_road_history()
            opposite = pipeline.road(batch(3-left), 10_200_000_000, 0.0)
            self.assertFalse(opposite["valid"])
            self.assertEqual(pipeline.left_colour, left)
            self.assertEqual(opposite["colour_assignment"]["status"], "locked")
            new_run = EstimationPipeline(settings())
            self.assertTrue(new_run.road(batch(3-left), 11_000_000_000, 0.0)["valid"])
            self.assertEqual(new_run.left_colour, 3-left)

    def test_width_gate_is_not_relaxed_to_force_assignment(self):
        road = EstimationPipeline(settings()).road(batch(width=0.1), 10_000_000_000, 0.0)
        self.assertFalse(road["valid"])
        self.assertEqual(road["colour_assignment"]["status"], "pending")

    def test_equally_plausible_two_sided_hypotheses_remain_unassigned(self):
        template = EstimationPipeline(settings(colour_assignment="fixed")).road(
            batch(), 10_000_000_000, 0.0)
        with patch.object(EstimationPipeline, "_road", return_value=deepcopy(template)):
            road = EstimationPipeline(settings()).road(batch(), 10_000_000_000, 0.0)
        self.assertEqual(road["status"], "boundary_colour_assignment_ambiguous")
        self.assertFalse(road["valid"])
        self.assertEqual(road["visibility"], "unassigned")
        self.assertFalse(road["centerline_xy"])
        self.assertIsNone(road["colour_assignment"]["left_colour"])

    def test_nearest_observed_corridor_wins_only_with_clear_distance_margin(self):
        template = EstimationPipeline(settings(colour_assignment="fixed")).road(
            batch(), 10_000_000_000, 0.0)
        def candidate(probe, *_):
            result = deepcopy(template)
            offset = 0.0 if probe.left_colour == 2 else 1.0
            result["centerline_xy"] = [(p[0], p[1]+offset) for p in result["centerline_xy"]]
            return result
        pipeline = EstimationPipeline(settings(left_colour=1, right_colour=2))
        with patch.object(EstimationPipeline, "_road", candidate):
            road = pipeline.road(batch(), 10_000_000_000, 0.0)
        self.assertTrue(road["valid"])
        self.assertEqual(pipeline.left_colour, 2)

    def test_neutral_acquisition_is_bounded_and_only_before_tracking(self):
        control = SimpleNamespace(startup_grace_s=1.0)
        for status in ("boundary_colour_assignment_pending", "boundary_colour_assignment_ambiguous"):
            estimates = dict(state=dict(valid=True), road=dict(status=status))
            baseline = dict(valid=False)
            self.assertTrue(priming_motion_history(baseline, estimates, False, 0.2, control))
            self.assertFalse(priming_motion_history(baseline, estimates, True, 0.2, control))
            self.assertFalse(priming_motion_history(baseline, estimates, False, 1.0, control))
            estimates["state"]["valid"] = False
            self.assertFalse(priming_motion_history(baseline, estimates, False, 0.2, control))

    def test_missing_or_stale_data_does_not_acquire_a_mapping(self):
        pipeline = EstimationPipeline(settings())
        for value, stamp, age in ((None, None, None), (batch(), 10_000_000_000, 0.6)):
            road = pipeline.road(value, stamp, age)
            self.assertEqual(road["status"], "unavailable")
            self.assertFalse(pipeline._colour_assignment_locked)
        with self.assertRaises(ValueError):
            replace(settings(), colour_assignment="unknown")


if __name__ == "__main__":
    unittest.main()
