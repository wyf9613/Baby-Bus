"""Ordered pairing regression cases from mpc-car-run d155a80; no MPC imports."""
import json
import math
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mvp21_policy.estimation import EstimationPipeline, EstimationSettings
from mvp21_policy.geometry import _path_lengths
FIXTURES = Path(__file__).parent/'fixtures'


def ordered_settings(**changes):
    values = dict(road_geometry="ordered_xy", road_min_coverage_m=0.3,
                  road_forward_max_m=4.0, road_rear_support_m=1.0,
                  road_width_min_m=0.15, road_width_max_m=2.5,
                  single_side_from_learned_width=True,
                  max_source_age_s=1.5, motion_max_gap_s=0.25, motion_max_interval_s=0.5,
                  history_propagation="continuous", continuous_road_output=True,
                  road_hold_s=0.8, road_prediction_max_distance_m=0.5)
    return EstimationSettings(**{**values, **changes})


def road_from_frames(filename, **setting_changes):
    """Replay a car05 ordered_xy fixture's frames through _history_road directly
    (main's own OrderedRoadChecks pattern), returning the final road dict."""
    record = json.loads((FIXTURES / filename).read_text())
    pipeline = EstimationPipeline(ordered_settings(**setting_changes))
    road = None
    for frame in record["frames"]:
        road = pipeline._history_road({"detections": frame["detections"]},
                                      round(frame["stamp_s"]*1e9), 0.0, frame["stamp_s"])
    return road, record


def synthetic_road(left, right, **changes):
    detections = [(x, y, 0.0, 1, 0.9) for x, y in left]+[(x, y, 0.0, 2, 0.9) for x, y in right]
    pipeline = EstimationPipeline(ordered_settings(left_colour=1, right_colour=2, min_cone_confidence=0.25,
                                                   road_max_gap_m=2.0, huber_delta_m=0.2,
                                                   road_max_abs_slope=3.0, **changes))
    return pipeline._history_road({"detections": detections}, 10_000_000_000, 0.0, 10.0)


def centerline_profile_value(section, key):
    import yaml
    text = (ROOT / "config" / "ai4r_policy.yaml").read_text(encoding="utf-8")
    return yaml.safe_load(text)["/**/ai4r_policy"]["ros__parameters"][section][key]


class OrderedPairingBacktrackChecks(unittest.TestCase):
    """estimation.ordered_pairing_backtrack_m (2026-10-15): main's 3 cm step-back
    limit splits the centerline on one noisy cone (Gym reference_too_short), but a
    larger limit pairs car 05's misclassified cones into a short road, so the
    centerline profile keeps 0.03. Both sides of that trade-off are pinned here."""
    def test_one_outward_cone_splits_the_run_only_at_the_default(self):
        left = [(0.3+0.25*i, 0.4) for i in range(12)]
        right = [(0.4+0.25*i, -0.4) for i in range(12)]
        left[4] = (left[4][0], 0.48)         # one yellow cone 8 cm outward
        lengths = {}
        for backtrack in (0.03, 0.10):
            road = synthetic_road(left, right, ordered_pairing_backtrack_m=backtrack)
            self.assertTrue(road["valid"], road["status"])
            lengths[backtrack] = _path_lengths(road["centerline_xy"])[-1]
        self.assertLess(lengths[0.03], 1.0)
        self.assertGreater(lengths[0.10], 2.4)

    def test_u_turn_never_pairs_the_entry_with_the_exit(self):
        radius, half_width = 0.8, 0.4       # entry and exit centerlines 1.6 m apart

        def centre(s):
            if s <= 1.2:
                return 0.3+s, 0.0, 0.0
            a = (s-1.2)/radius
            if a <= math.pi:
                return 1.5+radius*math.sin(a), radius*(1-math.cos(a)), a
            return 1.5-(s-1.2-math.pi*radius), 2*radius, math.pi

        def boundary(offset, spacing):
            points, s = [], 0.0
            while s < 1.2+math.pi*radius+1.0:
                x, y, heading = centre(s)
                points.append((x-math.sin(heading)*offset, y+math.cos(heading)*offset))
                s += spacing
            return points
        truth = [centre(0.01*i)[:2] for i in range(int((2.2+math.pi*radius)/0.01))]
        road = synthetic_road(boundary(half_width, 0.2), boundary(-half_width, 0.3), ordered_pairing_backtrack_m=0.10)
        self.assertTrue(road["valid"], road["status"])
        worst = max(min(math.dist(p, q) for q in truth) for p in road["centerline_xy"])
        self.assertLess(worst, 0.05)

    def fixture_road(self, filename, backtrack):
        road, _ = road_from_frames(filename, left_colour=1, right_colour=2, min_cone_confidence=0.25,
                                   road_max_gap_m=2.0, huber_delta_m=0.2, road_max_abs_slope=3.0,
                                   ordered_pairing_backtrack_m=backtrack)
        return road

    def test_car05_fixtures_keep_their_long_single_side_road_at_the_profile_value(self):
        backtrack = centerline_profile_value("estimation", "ordered_pairing_backtrack_m")
        for filename in ("car05_ordered_chain_false_seed.json", "car05_ordered_short_near_boundary.json"):
            with self.subTest(filename=filename):
                road = self.fixture_road(filename, backtrack)
                self.assertTrue(road["valid"], road["status"])
                self.assertEqual(road["visibility"], "right_only")
                self.assertGreater(_path_lengths(road["centerline_xy"])[-1], 2.0)

    def test_a_wider_backtrack_pairs_the_false_seed_into_a_short_road(self):
        road = self.fixture_road("car05_ordered_chain_false_seed.json", 0.10)
        self.assertEqual(road["visibility"], "both")
        self.assertLess(_path_lengths(road["centerline_xy"])[-1], 0.5)

    def test_out_of_range_values_are_refused(self):
        for value in (-0.01, 0.31):
            with self.assertRaises(ValueError):
                ordered_settings(ordered_pairing_backtrack_m=value)


if __name__ == "__main__":
    unittest.main()
