"""Analytic geometry, failure contracts and integrated V2 checks, without ROS."""
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
import unittest

api = runpy.run_path(str(Path(__file__).with_name("test_planning.py")))
ns = api["api"]["namespace"]
Settings, Config, Planner = ns["PlanningSettings"], ns["LatticeSettings"], ns["FrenetLatticePlanner"]


def fixture(offset=0.0, near=0.0, curve=0.0, speed=0.1, points=()):
    args = list(api["fixture"](offset))
    road, state = args[:2]
    xs = [i*0.1 for i in range(31) if i*0.1 >= near-1e-9]
    for key, shift in (("centerline_xy", 0), ("left_boundary_xy", 0.5), ("right_boundary_xy", -0.5)):
        road[key] = [(x, offset+curve*x*x+shift) for x in xs]
    state["speed_mps"] = speed
    args[2] = {"available": True, "timestamp_s": 10.0, "source_age_s": 0.01,
               "frame_id": "base_link", "points_xyz_m": list(points)}
    return args


def planner(**kwargs):
    # Numerical correctness uses a nonbinding wall budget; deterministic clock
    # checks exercise timeout separately. Actual runtime is measured by study.
    return Planner(Settings(algorithm="lattice_v2", max_source_age_s=0.4, reference_lifetime_s=0.2),
                   Config(budget_s=5.0, clear_start_assumed=True, **kwargs))


class LatticeChecks(unittest.TestCase):
    def test_straight_projection_and_arc_are_analytic(self):
        curve = ns["LatticeCurve"]([(0, 0.2), (1, 0.2), (2, 0.2)], 0.5, 0.025)
        s0, d0 = curve.project_origin()
        self.assertAlmostEqual(s0, 0.5)
        self.assertAlmostEqual(d0, -0.2)
        x, y, tx, ty, k, dk = curve.at_s(s0+1)
        self.assertAlmostEqual(x, 1)
        self.assertEqual((y, tx, ty, k, dk), (0.2, 1, 0, 0, 0))

    def test_quintic_end_conditions_in_physical_units(self):
        length = 1.3
        c = ns["_lattice_lateral"](-0.1, 0.08, 0.03, 0.2, length)
        for u, target in ((0, (-0.1, 0.08, 0.03)), (1, (0.2, 0, 0))):
            values = [ns["_control_poly_eval"](ns["_lattice_derivative"](c, order), u)/length**order for order in range(3)]
            for actual, expected in zip(values, target):
                self.assertAlmostEqual(actual, expected, places=9)

    def test_cg_curvature_steering_round_trip(self):
        model = planner()._model({})
        for angle in (-0.5, 0, 0.5):
            tangent = math.tan(angle)
            k = tangent/model["wheelbase_m"]/math.sqrt(1+(model["rear_axle_from_cg_m"]*tangent/model["wheelbase_m"])**2)
            self.assertAlmostEqual(ns["_lattice_steering"](k, model), angle)

    def test_straight_cruise_output_and_serialization(self):
        ref, diag = planner().plan(*fixture())
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertFalse(ref["planned_stop"])
        decoded = json.loads(json.dumps(ref))
        start = decoded["frenet_path"]["lateral"]["origin"]
        g = ns["evaluate_frenet_path"](decoded, start+0.2, 10.01)
        self.assertAlmostEqual(g["x"], 0.2, places=5)
        self.assertAlmostEqual(g["y"], 0)
        self.assertIsNone(ns["evaluate_frenet_path"](decoded, start, 10.2))
        self.assertIsNone(ns["evaluate_frenet_path"](decoded, start+5, 10.01))
        self.assertAlmostEqual(ref["speed_profile"][0][1], 0.1)
        self.assertIn("costs", diag["selected"])

    def test_offset_returns_continuous_path_and_preview_turn(self):
        for offset in (-0.1, 0.1):
            ref, diag = planner().plan(*fixture(offset))
            self.assertTrue(ref["valid"], ref["reason"])
            at_start = ns["evaluate_planning_path"](ref, ref["path"]["range"][0], 10.01)
            self.assertAlmostEqual(at_start["position_xy_m"][1], 0, places=5)
            self.assertAlmostEqual(at_start["heading_rad"], 0, places=5)
            preview = ns["evaluate_planning_path"](ref, 0.35, 10.01)
            self.assertGreater(preview["position_xy_m"][1]*offset, 0)
            self.assertLessEqual(diag["output_errors"]["position_m"], 0.01)

    def test_near_gap_requires_explicit_assumption_and_preserves_offset(self):
        args = fixture(0.1, near=0.9)
        ref, diag = planner().plan(*args)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertAlmostEqual(diag["initialization"]["d0"], -0.1)
        cfg = Config(budget_s=5)
        rejected, _ = Planner(Settings(), cfg).plan(*args)
        self.assertEqual(rejected["reason"], "near_gap_requires_clear_start_assumption")

    def test_curve_is_supported_without_v1_straight_gate(self):
        ref, diag = planner().plan(*fixture(curve=0.08))
        self.assertTrue(ref["valid"], (ref["reason"], diag["candidates"]))
        self.assertGreater(ns["evaluate_planning_path"](ref, 0.35, 10.01)["heading_rad"], 0)

    def test_obstacle_wall_selects_stop_before_contact(self):
        points = [(1.5, y, 0) for y in (-0.45, -0.3, -0.15, 0, 0.15, 0.3, 0.45)]
        ref, diag = planner().plan(*fixture(points=points))
        self.assertTrue(ref["valid"], (ref["reason"], diag))
        self.assertTrue(ref["planned_stop"])
        self.assertEqual(diag["behavior"], "obstacle_stop")
        self.assertAlmostEqual(ref["speed_profile"][-1][1], 0, places=6)
        self.assertLess(ref["path"]["range"][1]+0.297+0.1, 1.5)
        self.assertLessEqual(max(v for _, v in ref["speed_profile"]), 0.1+1e-5)

    def test_small_obstacle_can_be_bypassed_and_roadside_point_does_not_force_stop(self):
        ref, diag = planner().plan(*fixture(points=[(1.5, 0, 0)]))
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertFalse(ref["planned_stop"])
        self.assertGreater(abs(diag["selected"]["offset_m"]), 0.2)
        ref, diag = planner().plan(*fixture(points=[(1.5, 0.45, 0)]))
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertFalse(ref["planned_stop"])
        self.assertEqual(diag["selected"]["offset_m"], 0)

    def test_stop_decision_persists_after_obstacle_disappears(self):
        p = planner()
        args = fixture(points=[(1.5, y, 0) for y in (-0.4, -0.2, 0, 0.2, 0.4)])
        ref, _ = p.plan(*args)
        self.assertTrue(ref["planned_stop"])
        args[2]["points_xyz_m"] = []
        args[4] = 10.05
        for obj in args[:3]: obj["timestamp_s"] = args[4]
        ref, _ = p.plan(*args)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertTrue(ref["planned_stop"])
        args[4] = 10.4
        for obj in args[:3]: obj["timestamp_s"] = args[4]
        ref, _ = p.plan(*args)
        self.assertFalse(ref["valid"])
        self.assertEqual(ref["reason"], "planned_stop_requires_explicit_restart")

    def test_measured_speed_overshoot_is_not_mistaken_for_invalid_input(self):
        ref, _ = planner().plan(*fixture(speed=0.21))
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertAlmostEqual(ref["speed_profile"][0][1], 0.21)
        self.assertLessEqual(ref["target_speed_mps"], 0.21)

    def test_standstill_can_start_and_blocked_standstill_stays_neutral(self):
        ref, _ = planner().plan(*fixture(speed=0))
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertEqual(ref["speed_profile"][0][1], 0)
        self.assertGreater(ref["target_speed_mps"], 0)
        ref, _ = planner().plan(*fixture(speed=0, points=[(1.5, y, 0) for y in (-0.4, -0.2, 0, 0.2, 0.4)]))
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertTrue(ref["planned_stop"])
        self.assertEqual(ref["target_speed_mps"], 0)

    def test_shipped_lattice_parameters_are_loadable_without_hidden_keys(self):
        import re
        config_path = Path(__file__).resolve().parents[1]/"config"/"ai4r_policy.yaml"
        values = {}
        for line in config_path.read_text(encoding="utf-8").splitlines():
            match = re.fullmatch(r"        ([a-z_0-9]+): (.+)", line)
            if match:
                key, raw = match.groups()
                try: value = json.loads(raw)
                except ValueError: value = raw
                self.assertNotIn(key, values)
                values[key] = value
        cfg = Config(**values)
        self.assertFalse(cfg.obstacle_check_enabled)
        self.assertTrue(cfg.clear_start_assumed)
        self.assertEqual(set(values), set(vars(cfg)))

    def test_obstacle_disabled_ignores_missing_stale_and_noisy_lidar(self):
        results = []
        for obstacle in (None, {"available": False}, fixture(points=[(0.1, 0, 0)])[2]):
            args = fixture()
            args[2] = obstacle
            ref, diag = planner(obstacle_check_enabled=False).plan(*args)
            self.assertTrue(ref["valid"], ref["reason"])
            self.assertFalse(diag["obstacle_response_enabled"])
            self.assertNotIn("obstacles", ref["source_ages_s"])
            results.append(ref["path"])
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0], results[2])
        args = fixture()
        args[0]["left_boundary_xy"] = [(x, 0.1) for x, _ in args[0]["left_boundary_xy"]]
        args[0]["right_boundary_xy"] = [(x, -0.1) for x, _ in args[0]["right_boundary_xy"]]
        ref, _ = planner(obstacle_check_enabled=False).plan(*args)
        self.assertFalse(ref["valid"])
        self.assertTrue(ref["stop_requested"])

    def test_actual_node_can_follow_road_without_optional_lidar(self):
        control_api = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
        node = control_api["make_node"](simulated=False, mvp=True)
        node.planning_settings = Settings(algorithm="lattice_v2", max_source_age_s=0.4, reference_lifetime_s=0.2)
        node.lattice_settings = Config(budget_s=5, clear_start_assumed=True, obstacle_check_enabled=False)
        self.assertNotIn("lidar_cartesian", node.required_sensors)
        control_api["feed"](node, center=0.1)
        control_api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.planning_output["valid"])
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)

    def test_actual_node_latches_lattice_failure_and_requires_explicit_resume(self):
        control_api = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
        node = control_api["make_node"](simulated=False, mvp=True)
        node.planning_settings = Settings(algorithm="lattice_v2", max_source_age_s=0.4, reference_lifetime_s=0.2)
        node.lattice_settings = Config(budget_s=5, clear_start_assumed=True)
        node.required_sensors.append("lidar_cartesian")
        control_api["feed"](node, center=0.1)
        node._store("lidar_scan", {"ranges": [], "intensities": []}, node.test_clock.ros_ns,
                    node.test_clock.monotonic, node.test_clock.ros_ns)
        node._store("lidar_cartesian", {"points_xyz": [], "scan_indices": []},
                    node.test_clock.ros_ns, node.test_clock.monotonic, node.test_clock.ros_ns)
        control_api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertGreater(node.action_publisher.messages[-1].steer, 0)
        node.test_clock.advance(0.05)
        control_api["feed"](node, center=0.1)
        node._store("lidar_scan", {"ranges": [], "intensities": []}, node.test_clock.ros_ns,
                    node.test_clock.monotonic, node.test_clock.ros_ns)
        node._store("lidar_cartesian", {"points_xyz": [(0.1, 0, 0)], "scan_indices": [0]},
                    node.test_clock.ros_ns, node.test_clock.monotonic, node.test_clock.ros_ns)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.test_clock.advance(0.05)
        control_api["feed"](node, center=0.1)
        node._store("lidar_scan", {"ranges": [], "intensities": []}, node.test_clock.ros_ns,
                    node.test_clock.monotonic, node.test_clock.ros_ns)
        node._store("lidar_cartesian", {"points_xyz": [], "scan_indices": []},
                    node.test_clock.ros_ns, node.test_clock.monotonic, node.test_clock.ros_ns)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_close_obstacle_or_narrow_road_never_outputs_drive_path(self):
        for args in (fixture(points=[(0.1, 0, 0)]), fixture()):
            if not args[2]["points_xyz_m"]:
                args[0]["left_boundary_xy"] = [(x, 0.12) for x, _ in args[0]["left_boundary_xy"]]
                args[0]["right_boundary_xy"] = [(x, -0.12) for x, _ in args[0]["right_boundary_xy"]]
            ref, _ = planner().plan(*args)
            self.assertFalse(ref["valid"])
            self.assertTrue(ref["stop_requested"])
            self.assertIsNone(ref["path"])

    def test_freshness_frame_point_cap_and_source_deadline(self):
        for change in ("road_age", "obstacle_age", "frame", "points", "missing"):
            args = fixture()
            if change == "road_age": args[0]["source_age_s"] = 0.4
            if change == "obstacle_age": args[2]["source_age_s"] = 0.4
            if change == "frame": args[2]["frame_id"] = "lidar"
            if change == "points": args[2]["points_xyz_m"] = [(1, math.nan, 0)]
            if change == "missing": args[2]["available"] = False
            ref, _ = planner().plan(*args)
            self.assertFalse(ref["valid"], change)
        ref, _ = planner().plan(*fixture(), source_timeout_s={"road": 0.12, "wheel_speed": 0.15, "imu_angular_velocity": 0.15, "obstacles": 0.08})
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertLessEqual(ref["valid_for_s"], 0.07+1e-9)

    def test_budget_timeout_is_deterministic_and_neutral(self):
        tick = [0.0]
        def clock():
            tick[0] += 0.01
            return tick[0]
        p = Planner(Settings(), Config(clear_start_assumed=True), clock=clock)
        ref, _ = p.plan(*fixture())
        self.assertEqual(ref["status"], "TIME_BUDGET_EXCEEDED")
        self.assertEqual(ref["target_speed_mps"], 0)
        self.assertIsNone(p.previous)

    def test_upstream_missing_parameters_do_not_fall_back(self):
        ref, _ = planner(model_source="upstream").plan(*fixture())
        self.assertFalse(ref["valid"])
        self.assertEqual(ref["reason"], "incomplete_lattice_vehicle_limits")

    def test_time_polynomials_meet_boundary_conditions(self):
        p = planner()
        for stop in (False, True):
            profile = p._longitudinal(0.5, 0.1, 1.5, 10, 0.2, stop)
            c, duration = profile["coeffs_low_to_high"], profile["duration_s"]
            self.assertAlmostEqual(c[0], 0.5)
            self.assertAlmostEqual(c[1]/duration, 0.1)
            final_rate = ns["_control_poly_eval"](ns["_lattice_derivative"](c), 1)/duration
            self.assertAlmostEqual(final_rate, 0 if stop else 0.2)
            self.assertAlmostEqual(ns["_control_poly_eval"](ns["_lattice_derivative"](c, 2), 1), 0)
            if stop: self.assertAlmostEqual(sum(c), 1.5)

    def test_invalid_settings_are_rejected(self):
        for kwargs in ({"geometry_step_m": 0.2}, {"terminal_offsets_m": "nan"},
                       {"durations_s": "1"}, {"max_candidates": 1}, {"clear_start_assumed": 1},
                       {"model_source": "guess"}):
            with self.assertRaises(ValueError): Config(**kwargs)

    def test_inputs_are_not_mutated(self):
        args = fixture(0.1)
        original = deepcopy(args)
        planner().plan(*args)
        self.assertEqual(args, original)

    def test_score_scales_and_peak_clearance_have_expected_meaning(self):
        p = planner()
        geometry = [{"s": i*0.1, "d": 0.125, "clearance": 0.125,
                     "support": "observed"} for i in range(11)]
        cost = p._spatial_score(geometry, [], None)
        self.assertAlmostEqual(cost["center"], 1)
        self.assertEqual(cost["clearance"], 0)
        geometry[5]["clearance"] = 0
        cost = p._spatial_score(geometry, [], None)
        self.assertGreaterEqual(cost["clearance"], 0.5)
        self.assertLess(cost["clearance"], 0.6)

    def test_near_extension_is_c2_at_observed_join(self):
        curve = ns["LatticeCurve"]([(i*0.1, 0.08*(i*0.1)**2) for i in range(9, 31)], 1.3, 0.025)
        a, b = curve.at_x(0.9-1e-8), curve.at_x(0.9+1e-8)
        for order in range(3): self.assertAlmostEqual(a[order], b[order], places=6)

    def test_cubic_reference_matches_quadratic_curvature(self):
        curve = ns["LatticeCurve"]([(i*0.1, 0.08*(i*0.1)**2) for i in range(31)], 0.5, 0.025)
        for x in (0, 0.4, 1.2, 2.8):
            s = ns["_lattice_interp"](curve.arc, x)
            actual = curve.at_s(s)
            self.assertAlmostEqual(actual[4], 0.16/(1+(0.16*x)**2)**1.5, places=6)


if __name__ == "__main__":
    unittest.main()
