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
    # Numerical/source-age contracts use a controlled clock; the explicit
    # advancing-clock test checks expiry. Wall runtime is measured by study.
    return Planner(Settings(algorithm="lattice_v2", max_source_age_s=0.4, reference_lifetime_s=0.2),
                   Config(budget_s=5.0, clear_start_assumed=True, **kwargs), clock=lambda: 0.0)


class LatticeChecks(unittest.TestCase):
    def test_observed_rear_support_needs_no_extension_and_roundtrips(self):
        for start in (-1.5, -0.5, -0.348):
            args = fixture()
            xs = [start]+[i*0.1 for i in range(-14, 31) if i*0.1 > start]
            for key, y in (("centerline_xy", 0), ("left_boundary_xy", 0.5), ("right_boundary_xy", -0.5)):
                args[0][key] = [(x, y) for x in xs]
            ref, diag = planner().plan(*args)
            self.assertTrue(ref["valid"], ref["reason"])
            self.assertEqual(diag["near_support"]["extension_m"], 0)
            self.assertEqual(diag["near_support"]["method"], "observed_support")
            origin = ref["frenet_path"]["lateral"]["origin"]
            g = ns["evaluate_frenet_path"](json.loads(json.dumps(ref)), origin, 10.0)
            self.assertAlmostEqual(g["x"], 0, places=6)
            self.assertAlmostEqual(g["y"], 0, places=6)

    def test_fused_forward_only_start_requires_explicit_planner_assumption(self):
        estimator_api = api["api"]
        out = estimator_api["NearFieldHistoryChecks"].step(
            estimator_api["Pipeline"](estimator_api["Settings"]()), 10.0,
            [0.9+0.3*i for i in range(8)], speed=0.0)
        road = out["road"]
        self.assertFalse(road["valid"])
        self.assertEqual(road["centerline_xy"], [])
        self.assertTrue(road["forward_geometry"]["geometry_valid"])
        before = deepcopy(road)
        enabled = planner(obstacle_check_enabled=False)
        ref, diag = enabled.plan(road, out["state"], out["obstacles"], {}, 10.0)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertTrue(diag["controlled_near_extension"])
        # Once observed origin support exists, loss must not revert to the
        # startup assumption, even when the forward shape remains plausible.
        observed = fixture()
        self.assertTrue(enabled.plan(*observed)[0]["valid"])
        self.assertFalse(enabled.plan(road, out["state"], None, {}, 10.0)[0]["valid"])
        enabled.cfg = Config(clear_start_assumed=False, budget_s=5, obstacle_check_enabled=False)
        self.assertFalse(enabled.plan(road, out["state"], out["obstacles"], {}, 10.0)[0]["valid"])
        bad = deepcopy(road)
        bad["status"] = "observed_history_conflict"
        self.assertFalse(planner(obstacle_check_enabled=False).plan(bad, out["state"], None, {}, 10.0)[0]["valid"])
        self.assertEqual(road, before)

    def test_degraded_fused_road_caps_speed_and_reference_deadline(self):
        args = fixture(speed=0.2)
        args[0].update(degraded=True, recommended_speed_scale=0.5, prediction_remaining_s=0.08)
        ref, diag = planner().plan(*args)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertLessEqual(ref["target_speed_mps"], 0.1)
        self.assertLessEqual(ref["valid_for_s"], 0.08)
        self.assertTrue(diag["road_degraded"])
        self.assertAlmostEqual(ref["speed_profile"][0][1], 0.2)
        self.assertLessEqual(diag["selected"]["terminal_speed_mps"], 0.1)
        for changes in ({"prediction_remaining_s": 0}, {"prediction_remaining_s": None},
                        {"recommended_speed_scale": 0}, {"recommended_speed_scale": float("nan")}):
            args[0].update(changes)
            self.assertFalse(planner().plan(*args)[0]["valid"])
            args[0].update(prediction_remaining_s=0.08, recommended_speed_scale=0.5)

    def test_actual_node_bridge_slows_then_expires_and_stays_stopped(self):
        control_api = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
        node = control_api["make_node"](simulated=False, mvp=True)
        node.planning_settings = Settings(algorithm="lattice_v2", max_source_age_s=0.4, reference_lifetime_s=0.2)
        node.lattice_settings = Config(budget_s=5, clear_start_assumed=True, obstacle_check_enabled=False)
        control_api["feed"](node, center=0.0, speed=0.2)
        control_api["start"](node)
        # This checks prediction ages/geometry on the synthetic node clock.
        # Real elapsed expiry and actual subprocess deadlines have separate tests.
        node.planner = Planner(node.planning_settings, node.lattice_settings, clock=lambda: 0.0)
        node.previous_policy_step_at = node.test_clock.monotonic
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        deadlines = []
        for _ in range(6):
            node.test_clock.advance(0.05)
            control_api["feed"](node, center=0.0, speed=0.2, empty=True)
            node.run_policy_step()
            if node.fsm_state == 2:
                break
            self.assertTrue(node.planning_diagnostics["road_degraded"])
            self.assertLessEqual(node.planning_output["target_speed_mps"], 0.1)
            prediction_deadline = node.planning_output["timestamp_s"]+node.planning_diagnostics["prediction_remaining_s"]
            self.assertLessEqual(node.last_control_reference_deadline_s, prediction_deadline+1e-8)
            deadlines.append(prediction_deadline)
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        self.assertTrue(deadlines)
        self.assertLessEqual(max(deadlines)-min(deadlines), 1e-8)
        node.test_clock.advance(0.05)
        control_api["feed"](node, center=0.0)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)

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


class DirectSampleChecks(unittest.TestCase):
    def test_checked_samples_match_raw_frenet_and_cover_full_horizon(self):
        for bend, offset in ((0, 0.1), (0.04, 0), (-0.04, -0.1), (0.12, 0)):
            args = fixture(curve=bend, offset=offset)
            before = deepcopy(args)
            ref, diag = planner(direct_sample_output=True).plan(*args)
            self.assertTrue(ref["valid"], (ref["reason"], diag))
            decoded = json.loads(json.dumps(ref))
            points = decoded["path"]["samples_xy_heading_curvature"]
            self.assertLessEqual(len(points), 201)
            lateral = decoded["frenet_path"]["lateral"]
            for i, point in enumerate(points):
                s = lateral["range"][0]+(lateral["range"][1]-lateral["range"][0])*i/(len(points)-1)
                raw = ns["evaluate_frenet_path"](decoded, s, 10.01)
                for value, key in zip(point, ("x", "y", "heading", "curvature")):
                    self.assertAlmostEqual(value, raw[key], places=8)
                query = ns["evaluate_planning_path"](decoded, point[0], 10.01)
                self.assertAlmostEqual(query["position_xy_m"][1], point[1])
            self.assertGreater(lateral["range"][1]-lateral["range"][0], diag["selected"]["length_m"])
            self.assertEqual(diag["output_errors"]["conversion"], "none")
            self.assertEqual(args, before)
            self.assertIsNone(ns["evaluate_planning_path"](decoded, points[-1][0]+0.01, 10.01))
            self.assertIsNone(ns["evaluate_planning_path"](decoded, points[0][0], 10.2))

    def test_fit_error_no_longer_rejects_checked_trajectory(self):
        args = fixture()
        for key, side in (("centerline_xy", 0), ("left_boundary_xy", 0.5), ("right_boundary_xy", -0.5)):
            args[0][key] = [(x, 0.15*math.sin(2*x)+side) for x, _ in args[0][key]]
        cfg = dict(geometry_step_m=0.05, time_step_s=0.1, transition_lengths_m="0.6,1.5",
                   terminal_offsets_m="0.0", max_candidates=16, obstacle_check_enabled=False)
        legacy, old_diag = planner(**cfg).plan(*args)
        self.assertFalse(legacy["valid"])
        self.assertEqual(legacy["reason"], "no_valid_cartesian_output")
        self.assertTrue(any(c["reason"] == "compatibility_fit_error" for c in old_diag["candidates"]))
        p = planner(direct_sample_output=True, **cfg)
        p._compatibility = lambda *args: self.fail("Direct output must not refit")
        ref, diag = p.plan(*args)
        self.assertTrue(ref["valid"], (ref["reason"], diag))
        self.assertEqual(ref["path"]["type"], "CARTESIAN_SAMPLES")

    def test_sample_preview_interpolates_tangent_and_retains_curve_direction(self):
        for bend in (-0.08, 0.08):
            args = fixture(curve=bend)
            ref, diag = planner(direct_sample_output=True).plan(*args)
            self.assertTrue(ref["valid"], ref["reason"])
            _, steer, result = ns["PolicyController"](ns["ControlSettings"](mode="mvp")).calculate(
                ref, args[1], {}, 10.01, 0.05)
            self.assertTrue(result["valid"], result)
            self.assertGreater(steer*bend, 0)
            self.assertGreater(result["curvature_1pm"]*bend, 0)
            self.assertAlmostEqual(result["control_preview_x_m"], 0.35)
            self.assertIsNone(result["path_degree"])

    def test_invalid_sample_contract_cannot_publish_driving_action(self):
        args = fixture()
        ref, _ = planner(direct_sample_output=True).plan(*args)
        for problem in ("nan", "order", "cap", "bounds", "row", "heading", "variable"):
            bad = deepcopy(ref)
            points = bad["path"]["samples_xy_heading_curvature"]
            points[:] = [list(p) for p in points]
            if problem == "nan": points[1][1] = math.nan
            if problem == "order": points[1][0] = points[0][0]
            if problem == "cap": points[:] = points*202
            if problem == "bounds": bad["path"]["range"][1] += 1
            if problem == "row": points[1] = None
            if problem == "heading": points[1][2] = math.pi
            if problem == "variable": bad["path"]["independent_variable"] = "time_s"
            self.assertIsNone(ns["evaluate_planning_path"](bad, 0.35, 10.01), problem)
            drive, steer, result = ns["PolicyController"](ns["ControlSettings"](mode="mvp")).calculate(
                bad, args[1], {}, 10.01, 0.05)
            self.assertFalse(result["valid"], problem)
            self.assertEqual((drive, steer), (0, 0))

    def test_delayed_and_repeated_transport_preserve_samples_and_original_expiry(self):
        ref, _ = planner(direct_sample_output=True).plan(*fixture(curve=0.08))
        before = deepcopy(ref)
        motion = ns["EstimationMotionHistory"](ns["EstimationSettings"]())
        for t in (10, 10.05, 10.1):
            motion.observe("wheel_speed", 0.2, t, t, t)
            motion.observe("imu_angular_velocity", 0.1, t, t, t)
        first = ns["transport_control_reference"](ref, motion, 10.05)
        twice = ns["transport_control_reference"](first, motion, 10.1)
        once = ns["transport_control_reference"](ref, motion, 10.1)
        original = ref["path"]["samples_xy_heading_curvature"]
        self.assertEqual(len(original), len(once["control_cached_path_samples"]))
        for a, b, raw in zip(twice["control_cached_path_samples"], once["control_cached_path_samples"], original):
            for x, y in zip(a, b): self.assertAlmostEqual(x, y, places=8)
            self.assertEqual(a[3], raw[3])
        self.assertAlmostEqual(twice["timestamp_s"]+twice["valid_for_s"], 10.2)
        self.assertEqual(ref, before)
        g = ns["evaluate_planning_path"](once, 0.35, 10.1)
        self.assertAlmostEqual(g["heading_rad"], ns["_planning_control_geometry"](once, 10.1)["path_heading_rad"])

    def test_raw_collision_stop_and_time_budget_still_apply(self):
        wall = [(1.5, y, 0) for y in (-0.45, -0.3, -0.15, 0, 0.15, 0.3, 0.45)]
        ref, _ = planner(direct_sample_output=True).plan(*fixture(points=wall))
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertTrue(ref["planned_stop"])
        self.assertAlmostEqual(ref["speed_profile"][-1][1], 0, places=6)
        args = fixture()
        args[0]["left_boundary_xy"] = [(x, 0.1) for x, _ in args[0]["left_boundary_xy"]]
        args[0]["right_boundary_xy"] = [(x, -0.1) for x, _ in args[0]["right_boundary_xy"]]
        self.assertFalse(planner(direct_sample_output=True).plan(*args)[0]["valid"])
        tick = iter(i*0.01 for i in range(1000))
        p = Planner(Settings(), Config(direct_sample_output=True, clear_start_assumed=True), clock=lambda: next(tick))
        timed, _ = p.plan(*fixture())
        self.assertFalse(timed["valid"])
        self.assertEqual(timed["target_speed_mps"], 0)
        self.assertEqual(timed["status"], "TIME_BUDGET_EXCEEDED")


class Mvp2Checks(unittest.TestCase):
    def test_actual_worker_starts_without_ros_and_accepts_only_one_job(self):
        import time
        worker = ns["AsyncLatticePlanner"]()
        try:
            deadline = time.monotonic()+2.0
            while not worker.ready and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(worker.ready)
            _, args, cfg = self.plan()
            job = {"generation": 42, "frame_id": "base_link", "planning": vars(Settings(algorithm="lattice_v2")),
                   "lattice": vars(Config(direct_sample_output=True, clear_start_assumed=True, obstacle_check_enabled=False)),
                   "control": vars(cfg), "args": args}
            self.assertTrue(worker.submit(job, time.monotonic()))
            self.assertFalse(worker.submit(job, time.monotonic()))
            result = None
            while result is None and time.monotonic() < deadline:
                result = worker.poll()
                time.sleep(0.005)
            self.assertIsNotNone(result)
            self.assertEqual(result["generation"], 42)
            self.assertTrue(result["reference"]["valid"], result)
            self.assertEqual(result["reference"]["path"]["type"], "CARTESIAN_SAMPLES")
        finally:
            worker.close()
        self.assertIsNotNone(worker.process.poll())

    def config(self):
        return Config(recovery_enabled=True, clear_start_assumed=True, obstacle_check_enabled=False,
                      geometry_step_m=0.05, time_step_s=0.1, transition_lengths_m="0.6,1.5",
                      terminal_offsets_m="0.0", max_candidates=16)

    def plan(self, angle=0, offset=0, bend=0, width=1.2):
        args = fixture(offset=offset, curve=bend, speed=0.0)
        slope = math.tan(math.radians(angle))
        for key, shift in (("centerline_xy", 0), ("left_boundary_xy", width/2), ("right_boundary_xy", -width/2)):
            args[0][key] = [(x, offset+slope*x+bend*x*x+shift*math.hypot(1, slope)) for x, _ in args[0][key]]
        args.append({"road": 0.4, "wheel_speed": 0.25, "imu_angular_velocity": 0.25})
        p = Planner(Settings(algorithm="lattice_v2", max_source_age_s=0.4), self.config())
        control = ns["ControlSettings"](mode="mvp", mvp_lateral_kp=1.5, mvp_heading_kp=1.0)
        return ns["_mvp_plan"](p, control, args), args, control

    def test_offset_and_thirty_to_fortyfive_degree_start_use_checked_centerline(self):
        for angle in (-45, -30, 0, 30, 45):
            for offset in (-0.15, 0, 0.15):
                with self.subTest(angle=angle, offset=offset):
                    (ref, diag), args, cfg = self.plan(angle, offset)
                    self.assertTrue(ref["valid"], (ref["reason"], diag))
                    self.assertEqual(ref["planner_version"], "mvp2_centerline")
                    self.assertLessEqual(diag["rollout_steps"], 120)
                    self.assertGreaterEqual(diag["minimum_clearance_m"], 0)
                    _, steer, result = ns["PolicyController"](cfg).calculate(ref, args[1], {}, 10, 0.05)
                    self.assertTrue(result["valid"], result)
                    if angle and abs(offset) < 0.01:
                        self.assertGreater(steer*angle, 0)

    def test_gentle_curves_both_directions_keep_curvature_and_preview(self):
        for bend in (-0.08, -0.03, 0.03, 0.08):
            (ref, diag), args, cfg = self.plan(bend=bend)
            self.assertTrue(ref["valid"], (ref["reason"], diag))
            point = ns["evaluate_planning_path"](ref, 0.35, 10)
            self.assertGreater(point["curvature_1pm"]*bend, 0)
            _, steer, result = ns["PolicyController"](cfg).calculate(ref, args[1], {}, 10, 0.05)
            self.assertTrue(result["valid"], result)
            self.assertGreater(steer*bend, 0)

    def test_actual_body_outside_corridor_is_not_hidden_by_recovery(self):
        (accepted, _), _, _ = self.plan(offset=0.5, width=1.8)
        self.assertTrue(accepted["valid"], accepted["reason"])
        (ref, diag), _, _ = self.plan(offset=0.6, width=1.0)
        self.assertFalse(ref["valid"])
        self.assertEqual(ref["reason"], "recovery_body_boundary_conflict")

    def test_backward_heading_and_short_support_do_not_make_a_recovery_path(self):
        (ref, _), _, _ = self.plan(angle=80)
        self.assertFalse(ref["valid"])
        args = fixture()
        for key in ("centerline_xy", "left_boundary_xy", "right_boundary_xy"):
            args[0][key] = args[0][key][:4]
        args.append({"road": 0.4, "wheel_speed": 0.25, "imu_angular_velocity": 0.25})
        p = Planner(Settings(algorithm="lattice_v2"), self.config())
        ref, _ = ns["_mvp_plan"](p, ns["ControlSettings"](mode="mvp"), args)
        self.assertFalse(ref["valid"])

    def test_low_speed_small_gyro_offset_no_longer_rejects_start(self):
        (ref, _), args, cfg = self.plan()
        args[1]["yaw_rate_rps"] = 0.07
        p = Planner(Settings(algorithm="lattice_v2"), self.config())
        self.assertTrue(ns["_mvp_plan"](p, cfg, args)[0]["valid"])
        args[1]["yaw_rate_rps"] = 0.5
        self.assertFalse(ns["_mvp_plan"](p, cfg, args)[0]["valid"])

    def test_delayed_curve_transport_retains_absolute_expiry_and_heading(self):
        (ref, _), _, _ = self.plan(bend=0.05)
        motion = ns["EstimationMotionHistory"](ns["EstimationSettings"]())
        for stamp in (10.0, 10.05, 10.1):
            motion.observe("wheel_speed", 0.2, stamp, stamp, stamp)
            motion.observe("imu_angular_velocity", 0.1, stamp, stamp, stamp)
        transported = ns["transport_control_reference"](ref, motion, 10.05)
        self.assertAlmostEqual(transported["timestamp_s"]+transported["valid_for_s"], ref["timestamp_s"]+ref["valid_for_s"])
        self.assertEqual(ref["timestamp_s"], 10)
        self.assertNotIn("control_cached_path_samples", ref)
        self.assertNotEqual(transported["control_cached_path_samples"][0][1], 0)
        with self.assertRaisesRegex(ValueError, "reference_expired"):
            ns["transport_control_reference"](ref, motion, 10.3)


if __name__ == "__main__":
    unittest.main()
