"""Real-car cone frames (car05/car15, ported from main 0c8593a) driven through
the actual Estimation -> road_centerline Planning -> MPC adapter chain.

Self-contained ast extraction directly from scripts/policy_node.py (no
runpy chain through test_estimation/test_planning/test_mpc), so this file
does not depend on -- or need to update -- those files' own NAMES sets.
It exercises exactly the new 2026-10-15 MPC port: estimation.road_geometry=
ordered_xy, planning.algorithm=road_centerline, and the MPC's ORDERED_CENTERLINE
adapter (reference_trajectory_from_planning) and anti-jump path projector
(mpc_path_projector), end to end on real car05/car15 cone batches.

car05 fixtures need estimation.road_geometry=ordered_xy set explicitly (the
captured estimation_parameters_at_capture predates that field); they are fed
through EstimationPipeline._history_road directly, as main's own
OrderedRoadChecks does. car15 fixtures were captured under y_of_x and are out
of scope here (ordered_xy has no car15 multi-frame fixture in this repo yet).
"""
import ast
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import json
import math
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "scripts" / "policy_node.py"
FIXTURES = Path(__file__).parent / "fixtures"

NAMES = {
    # Estimation.
    "finite_number", "wrap_angle", "EstimationSettings", "FirstOrderSampleFilter", "EstimationMotionHistory",
    "_estimation_median", "_estimation_solve", "_estimation_fit_boundary",
    "_path_points", "_path_lengths", "_path_sample", "_path_tangent", "_path_project",
    "_estimation_order_boundary", "EstimationPipeline",
    # Planning.
    "PlanningSettings", "planning_vehicle_limits", "evaluate_planning_path",
    "CenterlinePlanner", "RoadCenterlinePlanner",
    # MPC.
    "ControlReferenceManager", "MPCStop", "VehicleParamsSettings",
    "course_simulation_vehicle_params", "course_model_step", "MPCSettings",
    "mpc_vehicle_params", "reference_trajectory_from_planning", "_ordered_lead_in", "mpc_reference_errors",
    "mpc_path_projector", "VehicleModelMPC", "MPCController", "mpc_debug_json",
}
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
definitions = [item for item in tree.body if isinstance(item, (ast.FunctionDef, ast.ClassDef))
               and item.name in NAMES]
missing = NAMES - {item.name for item in definitions}
assert not missing, f"not found in {SOURCE}: {missing}"
namespace = {"dataclass": dataclass, "math": math, "deepcopy": deepcopy, "json": json,
             "time": time, "Real": __import__("numbers").Real}
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), namespace)

EstimationSettings = namespace["EstimationSettings"]
EstimationPipeline = namespace["EstimationPipeline"]
PlanningSettings = namespace["PlanningSettings"]
RoadCenterlinePlanner = namespace["RoadCenterlinePlanner"]
_path_lengths = namespace["_path_lengths"]
MPCSettings = namespace["MPCSettings"]
VehicleParamsSettings = namespace["VehicleParamsSettings"]
course_simulation_vehicle_params = namespace["course_simulation_vehicle_params"]
MPCController = namespace["MPCController"]
adapt = namespace["reference_trajectory_from_planning"]

try:
    import osqp  # noqa: F401
    QP_SOLVER = MPCSettings().qp_solver
except ImportError:
    QP_SOLVER = "dense"   # the car's Jetson has no osqp either; see car-jetson-no-osqp


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


def mpc_vehicle():
    # Mirrors test_mpc.py's own measured(): "course_simulation" values relabelled
    # as a measured record, since only a measured source may be valid=True.
    values = dict(vars(course_simulation_vehicle_params()), valid=True, source="offline_test_fixture")
    return VehicleParamsSettings(**values)


def mpc_controller(**changes):
    base = dict(enabled=True, shadow=True, qp_solver=QP_SOLVER, v_exec_max_mps=0.3)
    base.update(changes)
    return MPCController(MPCSettings(**base), mpc_vehicle())


class RealFrameOrderedRoadChecks(unittest.TestCase):
    """Reproduces main's OrderedRoadChecks (the estimation layer) first, so a
    failure here is clearly an estimation-port issue, not an MPC-adapter one.
    """
    def test_car05_false_nearest_cluster_does_not_hide_the_real_boundary(self):
        road, _ = road_from_frames("car05_ordered_chain_false_seed.json",
                                   left_colour=1, right_colour=2, min_cone_confidence=0.25,
                                   road_max_gap_m=2.0, huber_delta_m=0.2, road_max_abs_slope=3.0)
        self.assertTrue(road["valid"], road["status"])
        self.assertEqual(road["visibility"], "right_only")
        self.assertTrue(road["single_side_inferred_from_learned_width"])

    def test_car05_short_near_boundary_uses_the_longer_opposite_support(self):
        road, _ = road_from_frames("car05_ordered_short_near_boundary.json",
                                   left_colour=1, right_colour=2, min_cone_confidence=0.25,
                                   road_max_gap_m=2.0, huber_delta_m=0.2, road_max_abs_slope=3.0)
        self.assertTrue(road["valid"], road["status"])
        self.assertEqual(road["visibility"], "right_only")
        self.assertGreater(_path_lengths(road["centerline_xy"])[-1], 2.0)


class RealFrameMpcChecks(unittest.TestCase):
    """The new 2026-10-15 chain: the same real-car road -> RoadCenterlinePlanner
    -> MPC's ORDERED_CENTERLINE adapter -> one shadow MPC step."""
    def plan_and_adapt(self, filename, **estimation_changes):
        road, _ = road_from_frames(filename, left_colour=1, right_colour=2,
                                   min_cone_confidence=0.25, road_max_gap_m=2.0,
                                   huber_delta_m=0.2, road_max_abs_slope=3.0, **estimation_changes)
        # _history_road (called directly, like OrderedRoadChecks above) does not
        # run the later alignment step that sets time_aligned; this frame is
        # aligned by construction (state below shares its exact timestamp), so
        # set it explicitly rather than drive the full update() just for this.
        road["time_aligned"] = True
        now_s = road["timestamp_s"]
        state = {"valid": True, "speed_valid": True, "yaw_rate_valid": True,
                 "speed_mps": 0.0, "yaw_rate_rps": 0.0, "frame_id": "base_link",
                 "timestamp_s": now_s, "source_age_s": {"wheel_speed": 0.01, "imu_angular_velocity": 0.01}}
        planner = RoadCenterlinePlanner(PlanningSettings(
            algorithm="road_centerline", max_source_age_s=1.5, reference_lifetime_s=0.2,
            allow_forward_only=True))
        ref, diag = planner.plan(road, state, {}, {}, now_s,
                                 {"road": 1.5, "wheel_speed": 0.25, "imu_angular_velocity": 0.25}, mvp=True)
        self.assertTrue(ref["valid"], (ref["reason"], diag))
        self.assertEqual(ref["path"]["type"], "ORDERED_CENTERLINE")
        traj = adapt(ref, spacing_m=0.05)
        self.assertTrue(traj["valid"], traj["reason"])
        return road, ref, traj

    def assert_well_formed_trajectory(self, traj):
        points = traj["points"]
        self.assertGreaterEqual(len(points), 2)
        # s_m must be strictly increasing: a jump back onto an earlier part of
        # the chain (or onto a disconnected branch) would show up here first.
        for a, b in zip(points, points[1:]):
            self.assertGreater(b["s_m"], a["s_m"])
        for p in points:
            for key in ("x_m", "y_m", "yaw_rad", "curvature_1pm", "target_speed_mps"):
                self.assertTrue(math.isfinite(p[key]), (key, p))
            self.assertLessEqual(abs(p["curvature_1pm"]), 5.0)

    def test_car05_false_seed_adapts_to_a_well_formed_mpc_trajectory(self):
        _, _, traj = self.plan_and_adapt("car05_ordered_chain_false_seed.json")
        self.assert_well_formed_trajectory(traj)

    def test_car05_short_near_boundary_adapts_to_a_well_formed_mpc_trajectory(self):
        _, _, traj = self.plan_and_adapt("car05_ordered_short_near_boundary.json")
        self.assert_well_formed_trajectory(traj)

    def test_shadow_mpc_step_solves_within_budget_and_steers_toward_the_road(self):
        for filename in ("car05_ordered_chain_false_seed.json", "car05_ordered_short_near_boundary.json"):
            with self.subTest(filename=filename):
                road, ref, traj = self.plan_and_adapt(filename)
                ctrl = mpc_controller()
                result = ctrl.step(traj, 0.0, 0.0, traj["timestamp_s"], True)
                debug = result["debug"]
                self.assertIn(debug["branch"], ("shadow", "zero_target"), debug)
                self.assertLess(debug["step_s"] or 0.0, 0.05, "exceeds the 50 ms control budget")
                if debug["branch"] == "shadow":
                    self.assertTrue(math.isfinite(debug["candidate_steer_action"]))
                    self.assertTrue(math.isfinite(debug["candidate_drive"]))
                    # The road bends toward the single visible (right) boundary's
                    # side; steer sign just needs to be finite and bounded here,
                    # the actual direction is covered by test_mpc.py's sign tests.
                    self.assertLessEqual(abs(debug["candidate_steer_action"]), 1.0 + 1e-6)
                    self.assertIsNotNone(debug["pred_xy"])
                    self.assertEqual(len(debug["pred_xy"]), MPCSettings().horizon_n)


def ordered_centerline_reference(total_s, spacing_m=0.1, speed=0.2, n=41):
    """A straight ORDERED_CENTERLINE reference of exactly total_s length, for
    sweeping total_s against the adapter's resampling spacing."""
    points = [[i*total_s/(n-1), 0.0] for i in range(n)]
    path = {"type": "ORDERED_CENTERLINE", "independent_variable": "s_m",
            "samples_xy": points, "range": [0.0, total_s]}
    return {"frame_id": "base_link", "timestamp_s": 10.0, "valid_for_s": 0.2, "valid": True,
            "status": "TRACK", "reason": None, "path_id": 1, "simulation_only": False,
            "source_ages_s": {}, "path": path, "target_speed_mps": speed,
            "stop_requested": False, "planned_stop": False, "stop_at_path_m": None}


class OrderedCenterlineAdapterChecks(unittest.TestCase):
    """2026-10-15 regression: the forced final resample point (s=total_s) used to
    always be appended even when within 0.25*spacing_m of the previous point,
    producing a near-zero-length final segment that mpc_reference_errors
    rejects as "repeated_point" (first caught by a Gym run under
    car_readiness_centerline.py, not by a single-frame fixture)."""
    def test_lengths_near_a_spacing_multiple_do_not_create_a_repeated_point(self):
        spacing_m = 0.1
        for k in range(3, 20):
            for epsilon in (-1e-4, -1e-9, 0.0, 1e-9, 1e-4):
                total_s = k*spacing_m + epsilon
                with self.subTest(total_s=total_s):
                    traj = adapt(ordered_centerline_reference(total_s, spacing_m), spacing_m=spacing_m)
                    self.assertTrue(traj["valid"], traj["reason"])
                    points = traj["points"]
                    for a, b in zip(points, points[1:]):
                        self.assertGreater(math.hypot(b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]), 1e-6)
                        self.assertGreater(b["s_m"], a["s_m"])
                    self.assertAlmostEqual(points[-1]["s_m"], total_s, places=6)


def ordered_reference(points):
    reference = ordered_centerline_reference(1.0)
    reference["path"]["samples_xy"] = [list(p) for p in points]
    return reference


def errors_at_car(traj, max_extension=0.6):
    """(e_y, e_psi) of a car at the base_link origin, heading +x: car minus path."""
    px, py, heading, *_ = namespace["mpc_path_projector"](traj["points"], max_extension)(0.0, 0.0)
    return -(-px*math.sin(heading)+py*math.cos(heading)), -heading


def arc(start_x, radius, count=60, spacing=0.05):
    return [(start_x+radius*math.sin(i*spacing/radius), radius*(1-math.cos(i*spacing/radius)))
            for i in range(count)]


class OrderedLeadInChecks(unittest.TestCase):
    """2026-10-15, car 05 18:18 replay: the ordered centerline started 0.58 m
    ahead (near field unobserved) and the projector extended its first 5 cm
    segment to the car, multiplying that segment's heading noise by the gap.
    The adapter now adds a lead-in on the start tangent of a quadratic fit."""
    def test_one_noisy_first_sample_is_not_multiplied_by_the_gap(self):
        points = [(0.58+0.05*i, 0.0) for i in range(50)]
        points[0] = (0.58, -0.015)        # first sample 1.5 cm off: a 0.3 rad first segment
        old = errors_at_car(adapt(ordered_reference(points), 0.05, lead_in_fit_m=0.0, max_lead_in_m=0.6))
        new = errors_at_car(adapt(ordered_reference(points), 0.05, lead_in_fit_m=1.0, max_lead_in_m=0.6))
        self.assertGreater(abs(old[0]), 0.1)
        self.assertLess(abs(new[0]), 0.025)     # the fit still weighs the noisy sample
        self.assertLess(abs(new[1]), 0.03)

    def test_straight_offset_and_bends_of_any_radius_stay_exact(self):
        cases = {"offset_straight": ([(0.58+0.05*i, 0.05) for i in range(50)], (-0.05, 0.0))}
        for radius in (1.0, 1.5, 3.0, 5.0):
            for start_x in (0.1, 0.3, 0.58):
                cases[f"R{radius}_from_{start_x}"] = (arc(start_x, radius), (0.0, 0.0))
        for name, (points, (e_y, e_psi)) in cases.items():
            with self.subTest(name):
                traj = adapt(ordered_reference(points), 0.05, lead_in_fit_m=1.0, max_lead_in_m=0.6)
                self.assertTrue(traj["valid"], traj["reason"])
                got = errors_at_car(traj)
                self.assertAlmostEqual(got[0], e_y, delta=0.02)
                self.assertAlmostEqual(got[1], e_psi, delta=0.035)
                for a, b in zip(traj["points"], traj["points"][1:]):
                    self.assertGreater(b["s_m"], a["s_m"])
                    self.assertGreater(math.hypot(b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]), 1e-6)

    def test_paths_at_or_behind_the_car_or_beyond_the_cap_are_unchanged(self):
        for name, points, cap in (("behind", [(-0.2+0.05*i, 0.01*i) for i in range(40)], 0.6),
                                  ("at_body", [(0.03+0.05*i, 0.0) for i in range(40)], 0.6),
                                  ("beyond_cap", [(0.9+0.05*i, 0.0) for i in range(40)], 0.6)):
            with self.subTest(name):
                reference = ordered_reference(points)
                old = adapt(deepcopy(reference), 0.05, lead_in_fit_m=0.0, max_lead_in_m=cap)
                new = adapt(deepcopy(reference), 0.05, lead_in_fit_m=1.0, max_lead_in_m=cap)
                self.assertEqual(old["points"], new["points"])


def synthetic_road(left, right, **changes):
    detections = [(x, y, 0.0, 1, 0.9) for x, y in left]+[(x, y, 0.0, 2, 0.9) for x, y in right]
    pipeline = EstimationPipeline(ordered_settings(left_colour=1, right_colour=2, min_cone_confidence=0.25,
                                                   road_max_gap_m=2.0, huber_delta_m=0.2,
                                                   road_max_abs_slope=3.0, **changes))
    return pipeline._history_road({"detections": detections}, 10_000_000_000, 0.0, 10.0)


def centerline_profile_value(section, key):
    import yaml
    text = (ROOT / "config" / "ai4r_policy_mpc_centerline.yaml").read_text(encoding="utf-8")
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


class ImportShadowingChecks(unittest.TestCase):
    """2026-10-15 regression: `from nav_msgs.msg import Path` (ROS branch only)
    rebound the module's `from pathlib import Path`, so AsyncLatticePlanner's
    Path(__file__) crashed at startup. Only the ROS gate saw it; this checks the
    source offline: no imported name may be bound by two different imports."""
    def test_no_import_rebinds_a_name(self):
        bound = {}
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                origin = node.module if isinstance(node, ast.ImportFrom) else None
                for alias in node.names:
                    name = alias.asname or alias.name.split(".")[0]
                    source = f"{origin}.{alias.name}" if origin else alias.name
                    self.assertEqual(bound.setdefault(name, source), source,
                                     f"{name!r} imported from both {bound[name]} and {source}")


if __name__ == "__main__":
    unittest.main()
