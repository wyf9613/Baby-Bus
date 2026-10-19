"""V1 contracts against actual single-file source; no ROS or algorithm copies.

Uses the same installed-source selector as test_estimation in the ROS gate.
Vehicle assumptions are labelled and never treated as real-car calibration.
"""
from copy import deepcopy
from pathlib import Path
import runpy
import math
from types import SimpleNamespace
import unittest

api = runpy.run_path(str(Path(__file__).with_name("test_estimation.py")))
Settings = api["namespace"]["PlanningSettings"]
Planner = api["namespace"]["CenterlinePlanner"]


def fixture(offset=0.0, slope=0.0):
    xs = [i*0.1 for i in range(31)]
    road = {"valid": True, "frame_id": "base_link", "timestamp_s": 10.0,
            "source_age_s": 0.0, "visibility": "both", "local_curvature_1pm": 0.0,
            "centerline_xy": [(x, offset+slope*x) for x in xs],
            "left_boundary_xy": [(x, offset+slope*x+0.5) for x in xs],
            "right_boundary_xy": [(x, offset+slope*x-0.5) for x in xs]}
    state = {"speed_valid": True, "yaw_rate_valid": True, "frame_id": "base_link",
             "timestamp_s": 10.0, "speed_mps": 0.1, "yaw_rate_rps": 0.0,
             "source_age_s": {"wheel_speed": 0.01, "imu_angular_velocity": 0.01}}
    # Labelled simulation assumptions: these are not real-car calibration.
    limits = {"valid": True, "source": "offline_test_assumption",
              "vehicle_reference_point": "cg_ground_projection",
              "footprint_xy_m": [(-0.15, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.15, 0.1)],
              "speed_max_mps": 0.5, "braking_deceleration_mps2": 0.5,
              "actuation_delay_s": 0.1, "safety_margin_m": 0.05}
    return road, state, {"available": False, "source_age_s": None}, limits, 10.0


class PlanningChecks(unittest.TestCase):
    def test_degraded_road_advisory_and_prediction_lifetime_reach_v1(self):
        args = list(fixture())
        args[0].update(degraded=True, recommended_speed_scale=0.5, prediction_remaining_s=0.04)
        for mvp in (True, False):
            ref, _ = Planner(Settings()).plan(*args, mvp=mvp)
            self.assertTrue(ref["valid"], ref["reason"])
            self.assertLessEqual(ref["target_speed_mps"], 0.1)
            self.assertLessEqual(ref["valid_for_s"], 0.04)
        args[0]["prediction_remaining_s"] = 0
        self.assertFalse(Planner(Settings()).plan(*args, mvp=True)[0]["valid"])

    def test_mvp_reference_without_calibration_keeps_geometry_and_freshness_checks(self):
        args = list(fixture(0.1, 0.05))
        args[3] = {"valid": False, "source": "unmeasured"}
        ref, diag = Planner(Settings()).plan(*args, mvp=True)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertEqual(ref["operating_profile"], "mvp_low_speed")
        self.assertFalse(ref["simulation_only"])
        self.assertEqual(ref["target_speed_mps"], 0.2)
        self.assertFalse(diag["calibration_required"])
        self.assertFalse(diag["stopping_model_enabled"])
        args[0]["source_age_s"] = 0.2
        self.assertFalse(Planner(Settings()).plan(*args, mvp=True)[0]["valid"])
        args = list(fixture(0.3))
        args[3] = {"valid": False}
        self.assertFalse(Planner(Settings()).plan(*args, mvp=True)[0]["valid"])

    def test_offset_and_linear_coefficients_are_preserved(self):
        for offset in (-0.15, 0.0, 0.15):
            args = fixture(offset, 0.05)
            ref, diag = Planner(Settings()).plan(*args)
            self.assertTrue(ref["valid"], ref["reason"])
            self.assertAlmostEqual(ref["path"]["coeffs_low_to_high"][0], offset)
            self.assertAlmostEqual(ref["path"]["coeffs_low_to_high"][1], 0.05)
            self.assertLessEqual(ref["target_speed_mps"], 0.3)
            self.assertGreaterEqual(ref["path"]["range"][0], 0.0)
            self.assertLess(ref["path"]["range"][1], 3.0)
            self.assertFalse(diag["obstacle_response_enabled"])

    def test_failure_contract_for_missing_stale_or_wrong_frame(self):
        for change in ("missing", "stale", "frame", "single_side", "limits"):
            road, state, obs, limits, now = fixture()
            if change == "missing":
                state["speed_valid"] = False
            elif change == "stale":
                road["source_age_s"] = 0.2
            elif change == "frame":
                road["frame_id"] = "camera"
            elif change == "single_side":
                road["visibility"] = "left_only"
            else:
                limits["valid"] = False
            ref, _ = Planner(Settings()).plan(road, state, obs, limits, now)
            self.assertFalse(ref["valid"])
            self.assertIsNone(ref["path"])
            self.assertEqual(ref["target_speed_mps"], 0.0)
            self.assertTrue(ref["stop_requested"])
            self.assertEqual(ref["reason"], ref["stop_reason"])

    def test_geometry_and_operating_domain_rejections(self):
        for change in ("curve", "offset", "heading", "near", "gap", "width", "nan"):
            args = list(deepcopy(fixture()))
            road = args[0]
            if change == "curve":
                road["local_curvature_1pm"] = 0.2
            elif change == "offset":
                args = list(fixture(0.3))
            elif change == "heading":
                args = list(fixture(0.0, 0.3))
            elif change == "near":
                for key in ("centerline_xy", "left_boundary_xy", "right_boundary_xy"):
                    road[key] = road[key][10:]
            elif change == "gap":
                road["centerline_xy"] = road["centerline_xy"][::5]
            elif change == "width":
                road["left_boundary_xy"] = [(x, 0.1) for x, _ in road["left_boundary_xy"]]
            else:
                road["centerline_xy"][3] = (0.3, float("nan"))
            ref, _ = Planner(Settings()).plan(*args)
            self.assertFalse(ref["valid"], change)

    def test_old_frame_requires_opt_in_and_compensation_changes_path(self):
        args = list(fixture(0.0, 0.05))
        args[0]["timestamp_s"] = 9.95
        args[0]["source_age_s"] = 0.05
        ref, _ = Planner(Settings()).plan(*args)
        self.assertEqual(ref["reason"], "motion_alignment_required")
        ref, diag = Planner(Settings(compensate_constant_twist=True)).plan(*args)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertEqual(diag["alignment"], "constant_twist_approximation")
        self.assertAlmostEqual(ref["path"]["coeffs_low_to_high"][0], 0.00025)
        self.assertEqual(ref["timestamp_s"], 10.0)

    def test_lifetime_respects_remaining_framework_deadline(self):
        args = list(fixture())
        args[1]["source_age_s"]["wheel_speed"] = 0.09
        ref, _ = Planner(Settings()).plan(*args, source_timeout_s={
            "road": 0.2, "wheel_speed": 0.1, "imu_angular_velocity": 0.2})
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertAlmostEqual(ref["valid_for_s"], 0.01)

    def test_actual_speed_not_target_determines_stopping_feasibility(self):
        args = list(fixture())
        args[1]["speed_mps"] = 0.5
        args[3]["braking_deceleration_mps2"] = 0.01
        ref, _ = Planner(Settings()).plan(*args)
        self.assertEqual(ref["reason"], "current_speed_outside_stopping_domain")

    def test_input_not_mutated_and_ids_advance(self):
        args = fixture()
        before = deepcopy(args)
        planner = Planner(Settings())
        first, _ = planner.plan(*args)
        second, _ = planner.plan(*args)
        self.assertEqual(args, before)
        self.assertEqual(second["trajectory_id"], first["trajectory_id"]+1)

    def test_evaluator_obeys_scale_range_and_expiry(self):
        ref, _ = Planner(Settings()).plan(*fixture(0.1))
        evaluate = api["namespace"]["evaluate_planning_path"]
        self.assertAlmostEqual(evaluate(ref, 1.0, 10.0)["position_xy_m"][1], 0.1)
        self.assertIsNone(evaluate(ref, -1.0, 10.0))
        self.assertIsNone(evaluate(ref, 1.0, 10.0+ref["valid_for_s"]))
        ref["path"].update(origin=0.5, scale=2.0, coeffs_low_to_high=[0.1, 0.2])
        self.assertAlmostEqual(evaluate(ref, 1.0, 10.0)["position_xy_m"][1], 0.15)

    def test_history_aligned_estimator_output_is_not_compensated_twice(self):
        args = api["input_record"](speed=0.1, yaw=0.0)
        xs = [0.2+0.1*i for i in range(27)]
        args[0]["cone_detections"] = api["cones"](
            [(x, 0.5) for x in xs], [(x, -0.5) for x in xs])
        # Forward-only startup has no observed support at the body origin.
        # Reject it before testing same-epoch consumption of a valid corridor.
        front_only = api["Pipeline"](api["Settings"]()).update(*args, 10.05)
        self.assertFalse(front_only["road"]["valid"])
        self.assertEqual(front_only["road"]["status"], "near_field_unobserved")
        xs = [-0.3, -0.2, -0.1, 0.0] + xs
        args[0]["cone_detections"] = api["cones"](
            [(x, 0.5) for x in xs], [(x, -0.5) for x in xs])
        out = api["Pipeline"](api["Settings"]()).update(*args, 10.05)
        self.assertTrue(out["alignment"]["valid"])
        ref, diag = Planner(Settings()).plan(
            out["road"], out["state"], out["obstacles"], fixture()[3], 10.05)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertEqual(diag["alignment"], "same_timestamp")
        self.assertEqual(ref["source_ages_s"]["road"], 0.05)

    def test_simulation_profile_is_explicit_and_matches_course_geometry(self):
        resolve = api["namespace"]["planning_vehicle_limits"]
        unknown = {"valid": False, "source": "unmeasured", "footprint_xy_m": None}
        self.assertEqual(resolve(unknown, Settings()), unknown)
        limits = resolve(unknown, Settings(vehicle_limits_source="course_simulation"))
        self.assertTrue(limits["simulation_only"])
        self.assertAlmostEqual(limits["wheelbase_m"], 0.33)
        self.assertAlmostEqual(max(x for x, _ in limits["footprint_xy_m"]), 0.297)
        self.assertAlmostEqual(min(x for x, _ in limits["footprint_xy_m"]), -0.198)
        self.assertAlmostEqual(max(y for _, y in limits["footprint_xy_m"]), 0.125)
        self.assertFalse(unknown["valid"])
        self.assertIn("explicit_offline_assumptions", limits["parameter_sources"]["motion_limits"])

    def test_actual_entry_with_simulation_limits_exports_reference_and_zero_actions(self):
        obs, ages, stamps, receipts = api["input_record"](speed=0.1, yaw=0.0)
        for key in ("fiducial_detections", "lidar_scan", "imu_orientation", "imu_specific_force"):
            obs[key] = None
        node = SimpleNamespace(estimation_settings=api["Settings"](), policy_frame_id="base_link",
            planning_settings=Settings(vehicle_limits_source="course_simulation"),
            sensor_timeout_s={key: 0.5 for key in obs}, heading_reference=None,
            motion_history=api["Motion"](api["Settings"]()),
            observations={key: SimpleNamespace(received_at=receipts.get(key),
                                               received_ros_ns=10_000_000_000) for key in obs},
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_050_000_000)))
        method = api["namespace"]["calculate_policy_actions"]
        self.assertEqual(method(node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True),
                         (0.0, 0.0, None, None, None))
        self.assertTrue(node.planning_output["valid"], node.planning_output["reason"])
        self.assertTrue(node.planning_output["simulation_only"])
        self.assertFalse(node.estimation_output["vehicle_limits"]["valid"])
        self.assertEqual(node.planning_diagnostics["alignment"], "same_timestamp")
        # A stricter estimator motion deadline must shorten the planner output.
        node.estimation_settings = api["Settings"](motion_max_gap_s=0.08)
        method(node, obs, ages, stamps, 1.01, 0.01, False)
        self.assertTrue(node.planning_output["valid"])
        self.assertAlmostEqual(node.planning_output["valid_for_s"], 0.03)
        obs["wheel_speed"] = None
        method(node, obs, ages, stamps, 1.05, 0.05, False)
        self.assertFalse(node.planning_output["valid"])
        self.assertIsNone(node.planning_output["path"])

    def test_near_coverage_is_checked_before_body_support_trim(self):
        args = list(fixture())
        for key in ("centerline_xy", "left_boundary_xy", "right_boundary_xy"):
            args[0][key] = args[0][key][5:]
        args[3] = {"valid": False, "source": "unmeasured"}
        ref, _ = Planner(Settings(vehicle_limits_source="course_simulation")).plan(*args)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertGreater(ref["path"]["range"][0], 0.5)

    def test_vehicle_assumptions_reduce_speed_without_changing_geometry(self):
        cfg = Settings(vehicle_limits_source="course_simulation", simulation_speed_max_mps=0.15)
        ref, _ = Planner(cfg).plan(*fixture())
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertEqual(ref["target_speed_mps"], 0.15)
        args = list(fixture())
        args[1]["speed_mps"] = 0.5
        ref, _ = Planner(Settings(vehicle_limits_source="course_simulation",
                                 simulation_braking_deceleration_mps2=0.01)).plan(*args)
        self.assertFalse(ref["valid"])
        self.assertEqual(ref["reason"], "current_speed_outside_stopping_domain")

    def test_invalid_planning_configuration_is_rejected(self):
        for kwargs in ({"vehicle_limits_source": "hardware_guess"},
                       {"simulation_braking_deceleration_mps2": 0.0},
                       {"simulation_actuation_delay_s": float("nan")},
                       {"compensate_constant_twist": 1}, {"max_near_x_m": 2.0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Settings(**kwargs)

    def test_all_sampled_vehicle_corners_remain_in_straight_corridor(self):
        args = fixture(0.1, 0.08)
        cfg = Settings(vehicle_limits_source="course_simulation")
        ref, _ = Planner(cfg).plan(*args)
        self.assertTrue(ref["valid"], ref["reason"])
        limits = api["namespace"]["planning_vehicle_limits"]({}, cfg)
        heading = math.atan(0.08)
        for i in range(51):
            lo, hi = ref["path"]["range"]
            x = lo+(hi-lo)*i/50
            for bx, by in limits["footprint_xy_m"]:
                cx = x+math.cos(heading)*bx-math.sin(heading)*by
                cy = 0.1+0.08*x+math.sin(heading)*bx+math.cos(heading)*by
                self.assertGreaterEqual(cx, -1e-12)
                self.assertLessEqual(cx, 3.0+1e-12)
                normal_left = (0.1+0.08*cx+0.5-cy)/math.hypot(1, 0.08)
                normal_right = (cy-(0.1+0.08*cx-0.5))/math.hypot(1, 0.08)
                self.assertGreaterEqual(min(normal_left, normal_right), cfg.simulation_safety_margin_m)

    def test_shipped_planning_configuration_is_valid(self):
        # Validate the actual selected profile rather than freeze all defaults.
        import re
        config = Path(__file__).resolve().parents[1]/"config"/"ai4r_policy.yaml"
        inside, scalars = False, {}
        for line in config.read_text(encoding="utf-8").splitlines():
            if line == "    planning:":
                inside = True
                continue
            if not inside or not line.strip() or line.lstrip().startswith("#"):
                continue
            if not line.startswith("      "):
                break
            if line.startswith("        ") or line == "      lattice:":
                continue
            match = re.fullmatch(r"      ([a-z_][a-z_0-9]*): ([a-z_0-9]+|-?[0-9]+(?:\.[0-9]+)?)", line)
            self.assertIsNotNone(match, line)
            value = match[2]
            scalars[match[1]] = (value == "true" if value in ("true", "false") else
                                 float(value) if value[0] in "-0123456789" else value)
        settings = Settings(**scalars)
        self.assertIn(settings.algorithm, ("centerline", "lattice_v2"))
        self.assertGreater(settings.reference_lifetime_s, 1/20)


if __name__ == "__main__":
    unittest.main()
