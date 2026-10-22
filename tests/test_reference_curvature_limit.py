"""MPC reference curvature limit (option B, hooks only), without ROS.

docs/MPC_REFERENCE_CURVATURE_LIMIT_PLAN_CN.md section 5. The pure functions come
from the whole policy module loaded the way tools/policy_offline.py does; the
calculate_policy_actions checks reuse test_mpc_planner_control's stand-in node
on the deployed car 05 layers. Geometry checks only: no closed-loop, timing or
physical clearance result.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from policy_offline import load_policy_namespace  # noqa: E402

ns = load_policy_namespace()
MPCSettings, Vehicle = ns["MPCSettings"], ns["VehicleParamsSettings"]
limit_reference_curvature = ns["limit_reference_curvature"]
peak_curvature = ns["reference_peak_curvature_1pm"]
curvature_limit = ns["reference_curvature_limit_1pm"]
NEW_FIELDS = ("reference_curvature_limit_mode", "reference_hook_curvature_1pm", "reference_hook_window_m",
              "reference_hook_trigger_length_m", "reference_curvature_max_1pm", "reference_curvature_clearance_m")
CAR05 = Vehicle(steering_min_rad=-0.37, steering_max_rad=0.37, wheelbase_m=0.33, body_width_m=0.25)
HALF_WIDTH, CHECK_M = 0.125, 0.5


def hooks(**values):
    return MPCSettings(**{"reference_curvature_limit_mode": "hooks", **values})


def trajectory(xy, speed=0.2):
    """v0.1 reference from a polyline, s_m/yaw_rad/curvature_1pm from its geometry."""
    points, s = [], 0.0
    for i, (x, y) in enumerate(xy):
        if i:
            s += math.hypot(x-xy[i-1][0], y-xy[i-1][1])
        a, b = xy[max(0, i-1)], xy[min(len(xy)-1, i+1)]
        points.append({"s_m": s, "x_m": x, "y_m": y, "yaw_rad": math.atan2(b[1]-a[1], b[0]-a[0]),
                       "curvature_1pm": 0.0, "target_speed_mps": speed})
    return {"schema": "reference_trajectory_v0.1", "frame_id": "base_link", "timestamp_s": 10.0,
            "valid_for_s": 0.25, "valid": True, "stop_required": False, "reason": None, "path_id": 7,
            "points": points, "reference_source": "planning", "planning_bypassed": False}


def straight(length=1.2, spacing=0.05):
    return [(spacing*i, 0.0) for i in range(int(round(length/spacing))+1)]


def arc(radius=2.0, length=1.2, spacing=0.05):
    return [(radius*math.sin(spacing*i/radius), radius*(1-math.cos(spacing*i/radius)))
            for i in range(int(round(length/spacing))+1)]


def hook(lead=0.3, turn_deg=150.0, turn_length=0.25, tail=0.5, spacing=0.05):
    """Straight lead, then a right turn of turn_deg within turn_length, then straight again."""
    radius = turn_length/math.radians(turn_deg)
    xy = [(spacing*i, 0.0) for i in range(int(round(lead/spacing))+1)]
    n = int(round(turn_length/spacing))
    for i in range(1, n+1):
        angle = math.radians(turn_deg)*i/n
        xy.append((lead+radius*math.sin(angle), -radius*(1-math.cos(angle))))
    heading = -math.radians(turn_deg)
    for i in range(1, int(round(tail/spacing))+1):
        x, y = xy[-1]
        xy.append((x+spacing*math.cos(heading), y+spacing*math.sin(heading)))
    return xy


def first_heading(points):
    return math.atan2(points[1]["y_m"]-points[0]["y_m"], points[1]["x_m"]-points[0]["x_m"])


class OffChecks(unittest.TestCase):
    def test_off_is_the_default_and_returns_the_same_trajectory(self):
        cfg = MPCSettings()
        self.assertEqual(cfg.reference_curvature_limit_mode, "off")
        traj = trajectory(hook())
        result, debug = limit_reference_curvature(traj, {}, cfg, CAR05, HALF_WIDTH, CHECK_M)
        self.assertIs(result, traj)
        self.assertIsNone(debug)

    def test_base_yaml_declares_every_new_setting_at_its_default(self):
        import yaml
        p = yaml.safe_load((ROOT/"config"/"ai4r_policy.yaml").read_text(encoding="utf-8"))
        mpc = p["/**/ai4r_policy"]["ros__parameters"]["mpc"]
        for name in NEW_FIELDS:
            self.assertEqual(mpc[name], getattr(MPCSettings(), name), name)
        self.assertEqual(mpc["reference_curvature_limit_mode"], "off")    # quoted: not YAML false

    def test_every_deploy_profile_is_off(self):
        deploy = runpy.run_path(str(ROOT/"tools"/"make_deploy_yaml.py"))
        modes = ("mvp", "mpc-shadow", "mpc-exec", "mpc-shadow-centerline", "mpc-exec-centerline",
                 "mpc-shadow-lattice21", "mpc-exec-lattice21", "steer-arc")
        built = 0
        for car in ("27", "05", "01"):
            for mode in modes:
                for longitudinal in ("joint", *deploy["FEEDBACK_VARIANTS"]):
                    with self.subTest(car=car, mode=mode, longitudinal=longitudinal):
                        try:
                            params = deploy["build"](mode, 3.0, cones="yellow-left", car=car,
                                                     longitudinal=longitudinal)[0]
                        except (AssertionError, ValueError):
                            continue    # mpc-exec* on car 27 (vehicle.valid false), feedback off car 05/01
                        mpc = params[deploy["KEY"]]["ros__parameters"]["mpc"]
                        self.assertEqual(mpc["reference_curvature_limit_mode"], "off")
                        MPCSettings(**mpc)
                        built += 1
        self.assertGreater(built, 30)


class TriggerChecks(unittest.TestCase):
    def test_straight_and_two_metre_arc_do_not_trigger(self):
        for name, xy in (("straight", straight()), ("arc", arc(2.0)), ("right_arc", [(x, -y) for x, y in arc(2.0)])):
            with self.subTest(name):
                traj = trajectory(xy)
                before = deepcopy(traj["points"])
                result, debug = limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)
                self.assertIs(result, traj)
                self.assertEqual(result["points"], before)
                self.assertFalse(debug["triggered"] or debug["applied"])
                self.assertLess(debug["peak_1pm"], 0.6)

    def test_trigger_uses_the_window_geometry_not_neighbouring_yaw(self):
        traj = trajectory(straight())
        for i, p in enumerate(traj["points"]):
            p["yaw_rad"] = 0.3 if i % 2 else -0.3        # noisy per-point yaw, straight geometry
        self.assertFalse(limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)[1]["triggered"])

    def test_hook_beyond_the_trigger_length_is_ignored(self):
        traj = trajectory(hook(lead=0.6))
        debug = limit_reference_curvature(traj, {}, hooks(reference_hook_trigger_length_m=0.4),
                                          CAR05, HALF_WIDTH, CHECK_M)[1]
        self.assertFalse(debug["triggered"])
        self.assertTrue(limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)[1]["triggered"])

    def test_invalid_stop_or_short_references_are_returned_unchanged(self):
        for change in ({"valid": False}, {"stop_required": True}):
            traj = {**trajectory(hook()), **change}
            result, debug = limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)
            self.assertIs(result, traj)
            self.assertFalse(debug["triggered"])
        traj = trajectory(hook())
        traj["points"] = traj["points"][:2]
        self.assertIs(limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)[0], traj)


class SmoothingChecks(unittest.TestCase):
    def test_hook_is_limited_from_the_reference_start(self):
        traj = trajectory(hook(), speed=0.17)
        traj["points"] = [{**p, "s_m": p["s_m"]+0.4} for p in traj["points"]]   # s_m need not start at 0
        result, debug = limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)
        limit = curvature_limit(MPCSettings(), CAR05)
        self.assertTrue(debug["triggered"] and debug["applied"])
        self.assertIsNone(debug["reason"])
        self.assertGreater(debug["peak_1pm"], 4.0)
        self.assertAlmostEqual(debug["limit_1pm"], limit)
        self.assertGreater(debug["max_shift_m"], 0.01)     # the hook starts at 0.3 m: little shift by 0.4 m
        self.assertLess(debug["max_shift_m"], 0.1)
        points = result["points"]
        self.assertGreaterEqual(len(points), 3)
        self.assertIsNot(result, traj)
        self.assertEqual({k: v for k, v in result.items() if k != "points"},
                         {k: v for k, v in traj.items() if k != "points"})
        xy = [(p["x_m"], p["y_m"]) for p in points]
        self.assertLessEqual(peak_curvature(xy, 0.15, 10.0), 1.05*limit)
        self.assertEqual((points[0]["x_m"], points[0]["y_m"]), (traj["points"][0]["x_m"], traj["points"][0]["y_m"]))
        self.assertLess(abs(first_heading(points)-first_heading(traj["points"])), math.radians(1.0))
        self.assertAlmostEqual(points[0]["s_m"], traj["points"][0]["s_m"])
        self.assertTrue(all(b["s_m"] > a["s_m"] for a, b in zip(points, points[1:])))
        self.assertTrue(all(abs(p["curvature_1pm"]) <= limit+1e-9 for p in points))
        self.assertTrue(all(p["target_speed_mps"] == 0.17 for p in points))
        # The MPC's own projection still accepts it (no repeated or nonfinite points).
        info, reason = ns["mpc_reference_errors"](result, 10, 0.1, 0.2, 0.6)
        self.assertIsNone(reason)
        self.assertLess(abs(info["e_y_m"]), 1e-6)

    def test_turning_left_too(self):
        traj = trajectory([(x, -y) for x, y in hook()])
        result, debug = limit_reference_curvature(traj, {}, hooks(), CAR05, HALF_WIDTH, CHECK_M)
        self.assertTrue(debug["applied"])
        self.assertGreater(result["points"][-1]["y_m"], 0.0)

    def test_limit_from_the_vehicle_record(self):
        self.assertAlmostEqual(curvature_limit(MPCSettings(), CAR05), math.tan(0.37)/0.33)
        self.assertAlmostEqual(curvature_limit(MPCSettings(), CAR05), 1.18, delta=0.01)
        asymmetric = Vehicle(steering_min_rad=-0.37, steering_max_rad=0.30, wheelbase_m=0.33)
        self.assertAlmostEqual(curvature_limit(MPCSettings(), asymmetric), math.tan(0.30)/0.33)
        asymmetric = Vehicle(steering_min_rad=-0.25, steering_max_rad=0.37, wheelbase_m=0.33)
        self.assertAlmostEqual(curvature_limit(MPCSettings(), asymmetric), math.tan(0.25)/0.33)
        self.assertEqual(curvature_limit(MPCSettings(reference_curvature_max_1pm=0.9), CAR05), 0.9)

    def test_unknown_vehicle_limit_keeps_the_reference(self):
        traj = trajectory(hook())
        result, debug = limit_reference_curvature(traj, {}, hooks(), Vehicle(), HALF_WIDTH, CHECK_M)
        self.assertIs(result, traj)
        self.assertEqual((debug["triggered"], debug["applied"], debug["reason"]),
                         (True, False, "no_curvature_limit"))


class ClearanceChecks(unittest.TestCase):
    def test_limited_path_into_the_outer_boundary_keeps_the_original(self):
        xy = hook()
        apex = max(x for x, _ in xy)
        outer = [(apex+0.12, 0.6-0.05*i) for i in range(31)]     # 0.12 m beyond the hook
        self.assertAlmostEqual(min(abs(x-outer[0][0]) for x, _ in xy), 0.12, places=6)
        traj = trajectory(xy)
        for road in ({"left_boundary_xy": outer, "right_boundary_xy": None},
                     {"centerline_xy": [], "forward_geometry": {"left_boundary_xy": outer}}):
            result, debug = limit_reference_curvature(traj, road, hooks(), CAR05, HALF_WIDTH, CHECK_M)
            self.assertIs(result, traj)
            self.assertEqual((debug["triggered"], debug["applied"], debug["reason"]),
                             (True, False, "rejected_clearance"))

    def test_boundary_with_room_and_one_sided_road_pass(self):
        traj = trajectory(hook())
        far = [(1.2, 0.6-0.05*i) for i in range(31)]
        right = [(-0.2+0.05*i, -0.9) for i in range(30)]
        for road in ({"left_boundary_xy": far, "right_boundary_xy": right}, {"right_boundary_xy": right}, {}):
            debug = limit_reference_curvature(traj, road, hooks(), CAR05, HALF_WIDTH, CHECK_M)[1]
            self.assertTrue(debug["applied"], road)

    def test_clearance_is_only_checked_over_the_check_length(self):
        xy = hook()
        apex = max(x for x, _ in xy)
        outer = [(apex+0.12, 0.6-0.05*i) for i in range(31)]
        debug = limit_reference_curvature(trajectory(xy), {"left_boundary_xy": outer}, hooks(),
                                          CAR05, HALF_WIDTH, 0.2)[1]
        self.assertTrue(debug["applied"])


class SettingsChecks(unittest.TestCase):
    def test_invalid_settings_raise(self):
        for name, value in (("reference_curvature_limit_mode", "on"), ("reference_curvature_limit_mode", False),
                            ("reference_hook_curvature_1pm", 0.0), ("reference_hook_curvature_1pm", -1.0),
                            ("reference_hook_window_m", 0.05), ("reference_hook_window_m", 0.6),
                            ("reference_hook_trigger_length_m", 0.0), ("reference_hook_trigger_length_m", 2.5),
                            ("reference_curvature_max_1pm", -0.1), ("reference_curvature_clearance_m", -0.01),
                            ("reference_hook_curvature_1pm", math.nan)):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                MPCSettings(**{name: value})
        hooks(reference_hook_window_m=0.5, reference_hook_trigger_length_m=2.0,
              reference_curvature_max_1pm=0.0, reference_curvature_clearance_m=0.0)


# calculate_policy_actions on test_mpc_planner_control's stand-in node.
planner = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
cns = planner["cns"]
for name in ("limit_reference_curvature", "reference_curvature_limit_1pm", "reference_peak_curvature_1pm",
             "_reference_segments", "_reference_curvature_limited", "_reference_polyline_distance",
             "REFERENCE_SMOOTH_STEP_M", "REFERENCE_SMOOTH_LOOKAHEAD_M", "REFERENCE_SHIFT_LENGTH_M"):
    cns.setdefault(name, ns[name])


def planner_node(mode):
    node = planner["make_node"](shadow_enabled=True, planner=True, worker=planner["SyncWorker"](deliver=False))
    node.mpc_settings = cns["MPCSettings"](**{**vars(node.mpc_settings), "reference_curvature_limit_mode": mode})
    return node


def hooked_reference(node):
    """A lattice execution reference (as PlannerExecutionManager.select returns) with a hook."""
    return {"schema_version": "planning_reference_v1", "planner_version": "lattice_execution",
            "trajectory_id": 41, "timestamp_s": node.estimation_output["state"]["timestamp_s"]+0.05,
            "valid_for_s": 0.25, "frame_id": "base_link", "vehicle_reference_point": "cg_ground_projection",
            "valid": True, "status": "VALID",
            "path": {"type": "ORDERED_CENTERLINE", "independent_variable": "s_m", "samples_xy": hook(),
                     "range": [0.0, 1.05]},
            "target_speed_mps": 0.2, "stop_requested": False, "simulation_only": False}


class PolicyIntegrationChecks(unittest.TestCase):
    def test_off_never_calls_the_limit_and_leaves_mpc_debug_unchanged(self):
        def refuse(*args, **kwargs):
            raise AssertionError("limit_reference_curvature called with mode off")
        saved, cns["limit_reference_curvature"] = cns["limit_reference_curvature"], refuse
        try:
            records = planner["run"](planner_node("off"), 12)
        finally:
            cns["limit_reference_curvature"] = saved
        self.assertTrue(records and all("reference_curvature" not in r for r in records))

    def test_hooks_on_a_straight_road_changes_nothing_but_the_added_field(self):
        without = planner["without_timing"]
        off = planner["run"](planner["make_node"](), 16)
        on = planner["run"](planner_node("hooks"), 16)
        self.assertEqual(len(off), len(on))
        for a, b in zip(off, on):
            curvature = b.pop("reference_curvature")
            self.assertFalse(curvature["triggered"] or curvature["applied"])
            b.pop("reference_selection", None), b.pop("reference_selection_reason", None)
            self.assertEqual(without(a), without(b))

    def test_hooked_lattice_path_is_limited_and_solved(self):
        records = {}
        for mode in ("off", "hooks"):
            node = planner_node(mode)
            planner["run"](node, 4)
            reference = hooked_reference(node)

            def select(*args, node=node, reference=reference):
                node.planner_execution.diagnostics = {"mode": "planner", "fallback": False, "reason": None}
                return reference
            node.planner_execution.select = select
            records[mode] = planner["run"](node, 1)[-1]
            self.assertEqual(records[mode]["reference_selection"], "planner")
            self.assertEqual(records[mode]["branch"], "shadow")         # the MPC solved this tick
            self.assertEqual(records[mode]["path_id"], 41)
            if mode == "hooks":
                xy = node._mpc_trajectory["points"]
                self.assertLessEqual(peak_curvature([(p["x_m"], p["y_m"]) for p in xy], 0.15, 10.0), 1.25)
        self.assertNotIn("reference_curvature", records["off"])
        curvature = records["hooks"]["reference_curvature"]
        self.assertTrue(curvature["triggered"] and curvature["applied"], curvature)
        self.assertIsNone(curvature["reason"])
        self.assertAlmostEqual(curvature["limit_1pm"], math.tan(0.37)/0.33)
        json.dumps(curvature, allow_nan=False)
        self.assertNotEqual(records["off"]["candidate_steer_action"], records["hooks"]["candidate_steer_action"])


if __name__ == "__main__":
    unittest.main()
