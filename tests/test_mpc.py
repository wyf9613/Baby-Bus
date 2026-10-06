"""Vehicle-model MPC contracts against the actual single-file source; no ROS needed.

The pure MPC definitions are extracted from scripts/policy_node.py like test_planning does.
The closed loops drive offline/mpc_prediction_model/vehicle_model.step (the prediction
model validated against Dream Gym) at the course step of 0.05 s, with parameters that
differ from the controller's: they show that this implementation is self-consistent under
the course model and modest mismatch, not that the real car behaves the same way.
"""
import ast
from pathlib import Path
import json
import math
import random
import runpy
import statistics
import sys
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "offline" / "mpc_prediction_model"))
from vehicle_model import CourseModelParams, step as plant_step  # noqa: E402

planning_api = runpy.run_path(str(Path(__file__).with_name("test_planning.py")))
namespace = planning_api["api"]["namespace"]
SOURCE = ROOT / "scripts" / "policy_node.py"
NAMES = {"wrap_angle", "MPCStop", "VehicleParamsSettings", "course_simulation_vehicle_params",
         "course_model_step", "MPCSettings", "mpc_vehicle_params", "reference_trajectory_from_planning",
         "reference_trajectory_from_road", "mpc_reference_errors", "mpc_path_projector",
         "VehicleModelMPC", "MPCController", "mpc_debug_json"}
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
definitions = [item for item in tree.body if isinstance(item, (ast.FunctionDef, ast.ClassDef))
               and item.name in NAMES]
assert {item.name for item in definitions} == NAMES
namespace.update(time=time, json=json, String=lambda data: SimpleNamespace(data=data))
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), namespace)
Settings = namespace["MPCSettings"]
Vehicle = namespace["VehicleParamsSettings"]
course_vehicle = namespace["course_simulation_vehicle_params"]
model_step = namespace["course_model_step"]
Stop = namespace["MPCStop"]
Controller = namespace["MPCController"]
Solver = namespace["VehicleModelMPC"]
adapt = namespace["reference_trajectory_from_planning"]
from_road = namespace["reference_trajectory_from_road"]
errors = namespace["mpc_reference_errors"]
debug_json = namespace["mpc_debug_json"]
Planner, PlanSettings, plan_fixture = (planning_api["Planner"], planning_api["Settings"],
                                       planning_api["fixture"])

# Closed-loop pass criteria, fixed before looking at results: 1.0 m road, 0.25 m car, 0.05 m margin.
E_LIMIT = (1.0-0.25)/2 - 0.05
PLANT_DT = 0.05


def measured(**changes):
    """Course values relabelled as a measured record: an offline TEST fixture only."""
    values = dict(vars(course_vehicle()), valid=True, source="offline_test_fixture")
    values.update(changes)
    return Vehicle(**values)


def verified(vehicle=None, **kwargs):
    base = dict(enabled=True, shadow=False, v_exec_max_mps=0.3)
    base.update(kwargs)
    return Controller(Settings(**base), measured() if vehicle is None else vehicle)


def plant_params(vehicle, gain_scale=1.0, offset=0.0, motor_scale=1.0, drag_scale=1.0):
    return CourseModelParams(
        front_axle_m=vehicle.wheelbase_m-vehicle.rear_axle_from_cg_m, rear_axle_m=vehicle.rear_axle_from_cg_m,
        mass_kg=vehicle.mass_kg, motor_gain_n=vehicle.motor_gain_n*motor_scale,
        drag_kg_per_m=vehicle.drag_kg_per_m*drag_scale, steering_gain_rad=vehicle.steering_gain_rad*gain_scale,
        steering_offset_rad=vehicle.steering_offset_rad+offset, steering_min_rad=vehicle.steering_min_rad,
        steering_max_rad=vehicle.steering_max_rad, steering_rate_lower_rad_s=-vehicle.steering_rate_limit_rad_s,
        steering_rate_upper_rad_s=vehicle.steering_rate_limit_rad_s)


def straight_trajectory(y=0.0, slope=0.0, speed=0.3, start_x=0.2, length=2.0, stamp=10.0,
                        lifetime=0.1, curvature=0.0):
    theta = math.atan(slope)
    points, s = [], 0.0
    for i in range(int(length/0.05)+1):
        s = i*0.05
        points.append({"s_m": s, "x_m": start_x+s*math.cos(theta), "y_m": y+start_x*slope+s*math.sin(theta),
                       "yaw_rad": theta, "curvature_1pm": curvature, "target_speed_mps": speed})
    return {"schema": "reference_trajectory_v0.1", "frame_id": "base_link", "timestamp_s": stamp,
            "valid_for_s": lifetime, "valid": True, "stop_required": False, "status": "TRACK",
            "reason": None, "path_id": 1, "simulation_only": True, "source_ages_s": {}, "points": points}


def arc_trajectory(kappa, speed=0.3, length=2.0, stamp=10.0):
    """Points on a circle of curvature kappa tangent to +x at the origin."""
    points = []
    for i in range(int(length/0.05)+1):
        s = i*0.05
        x, y = (s, 0.0) if kappa == 0 else (math.sin(kappa*s)/kappa, (1-math.cos(kappa*s))/kappa)
        points.append({"s_m": s, "x_m": x, "y_m": y, "yaw_rad": kappa*s, "curvature_1pm": kappa,
                       "target_speed_mps": speed})
    return dict(straight_trajectory(stamp=stamp), points=points)


class Adapter(unittest.TestCase):
    def test_planning_reference_becomes_point_list_with_original_time(self):
        ref, _ = Planner(PlanSettings(vehicle_limits_source="course_simulation")).plan(*plan_fixture(0.05, 0.02))
        self.assertTrue(ref["valid"], ref["reason"])
        traj = adapt(ref, 0.05)
        self.assertTrue(traj["valid"])
        self.assertEqual(traj["timestamp_s"], ref["timestamp_s"])
        self.assertEqual(traj["valid_for_s"], ref["valid_for_s"])
        self.assertTrue(traj["simulation_only"])
        s = [p["s_m"] for p in traj["points"]]
        self.assertTrue(all(b > a for a, b in zip(s, s[1:])))
        self.assertAlmostEqual(traj["points"][0]["y_m"], 0.05 + 0.02*traj["points"][0]["x_m"])
        self.assertAlmostEqual(traj["points"][0]["yaw_rad"], math.atan(0.02))
        self.assertLessEqual(traj["points"][0]["target_speed_mps"], 0.3)
        self.assertAlmostEqual(s[1]-s[0], 0.05, places=6)

    def test_invalid_or_stopping_planner_output_is_invalid_with_stop(self):
        for bad in (None, {}, {"valid": False, "reason": "stale_road"},
                    {"valid": True, "stop_requested": True, "frame_id": "base_link", "timestamp_s": 1.0}):
            traj = adapt(bad, 0.05)
            self.assertFalse(traj["valid"])
            self.assertTrue(traj["stop_required"])
            self.assertEqual(traj["points"], [])

    def test_wrong_frame_is_rejected(self):
        ref, _ = Planner(PlanSettings(vehicle_limits_source="course_simulation")).plan(*plan_fixture())
        ref["frame_id"] = "odom"
        self.assertFalse(adapt(ref, 0.05)["valid"])


class ReferenceErrors(unittest.TestCase):
    def test_sign_conventions(self):
        info, why = errors(straight_trajectory(y=0.1), 10, 0.1, 0.3, 0.6)
        self.assertIsNone(why)
        self.assertAlmostEqual(info["e_y_m"], -0.1)   # path left of car: car is right of path
        self.assertAlmostEqual(info["e_psi_rad"], 0.0)
        info, _ = errors(straight_trajectory(y=-0.1), 10, 0.1, 0.3, 0.6)
        self.assertAlmostEqual(info["e_y_m"], 0.1)
        info, _ = errors(straight_trajectory(slope=0.1, start_x=0.0), 10, 0.1, 0.3, 0.6)
        self.assertAlmostEqual(info["e_psi_rad"], -math.atan(0.1))  # path heads left: car heading is right of it

    def test_backward_extension_is_limited_and_short_path_rejected(self):
        self.assertIsNone(errors(straight_trajectory(start_x=0.5), 10, 0.1, 0.3, 0.6)[1])
        self.assertEqual(errors(straight_trajectory(start_x=0.7), 10, 0.1, 0.3, 0.6)[1],
                         "origin_before_path_start")
        short = straight_trajectory(length=0.2, start_x=0.0)
        self.assertEqual(errors(short, 10, 0.1, 0.3, 0.6)[1], "reference_too_short")
        self.assertEqual(errors(straight_trajectory(start_x=-3.0, length=2.0), 10, 0.1, 0.3, 0.6)[1],
                         "origin_past_path_end")

    def test_bad_geometry(self):
        traj = straight_trajectory()
        traj["points"][3]["x_m"] = float("nan")
        self.assertEqual(errors(traj, 10, 0.1, 0.3, 0.6)[1], "nonfinite_point")
        traj = straight_trajectory()
        traj["points"][4] = dict(traj["points"][3])
        traj["points"][4]["s_m"] += 0.05
        self.assertEqual(errors(traj, 10, 0.1, 0.3, 0.6)[1], "repeated_point")


class VehicleRecord(unittest.TestCase):
    def test_fields_match_the_identification_vehicle_params(self):
        offline = runpy.run_path(str(ROOT / "offline" / "vehicle_identification" / "vehicle_params.py"))
        fields = set(offline["VehicleParams"].__dataclass_fields__)
        self.assertEqual(set(vars(Vehicle())) - {"valid", "source"}, fields)

    def test_unmeasured_and_course_records_are_never_valid(self):
        self.assertFalse(Vehicle().valid)
        self.assertIsNotNone(Vehicle().problem())
        course = course_vehicle()
        self.assertFalse(course.valid)
        self.assertIsNone(course.problem())
        for source in ("unmeasured", "course_simulation", "offline_test_assumption"):
            with self.assertRaises(ValueError, msg=source):
                Vehicle(**dict(vars(course), valid=True, source=source))

    def test_valid_record_must_be_consistent(self):
        for change in ({"rear_axle_from_cg_m": 0.4}, {"steering_offset_rad": 0.9}, {"steering_gain_rad": 0.0},
                       {"drive_max": -0.1}, {"steering_rate_limit_rad_s": 0.0}, {"mass_kg": 0.0},
                       {"body_rear_extent_from_cg_m": 0.1}, {"steering_delay_s": 2.0},
                       {"braking_deceleration_mps2": 0.0}, {"wheelbase_m": float("nan")}):
            with self.assertRaises(ValueError, msg=str(change)):
                measured(**change)
        measured(steering_gain_rad=-0.3, steering_offset_rad=0.02, steering_min_rad=-0.25,
                 steering_max_rad=0.3)

    def test_records_feed_planning_with_the_measured_limits(self):
        params, limits = measured(steering_delay_s=0.05, drive_delay_s=0.12).records()
        self.assertEqual(limits["rear_axle_x_m"], -0.132)
        self.assertEqual(limits["actuation_delay_s"], 0.12)
        self.assertEqual(params["steering_map"]["gain_rad"], math.pi/4)
        self.assertEqual(sorted(p[0] for p in limits["footprint_xy_m"])[::3], [-0.198, 0.297])
        road, state, obstacles, _, now = plan_fixture(0.05, 0.02)
        ref, _ = Planner(PlanSettings()).plan(road, state, obstacles, limits, now)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertFalse(ref["simulation_only"])


class ModelParity(unittest.TestCase):
    """The inlined model must be the validated offline model, line for line."""

    def test_inlined_step_equals_offline_prediction_model(self):
        rng = random.Random(7)
        for vehicle in (course_vehicle(), measured(steering_gain_rad=-0.3, steering_offset_rad=0.02,
                                                  steering_min_rad=-0.25, steering_max_rad=0.3,
                                                  steering_rate_limit_rad_s=1.2, drag_kg_per_m=1.7)):
            params = plant_params(vehicle)
            for _ in range(200):
                state = [rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-3, 3), rng.uniform(-0.5, 1.5),
                         rng.uniform(vehicle.steering_min_rad, vehicle.steering_max_rad)]
                control = [rng.uniform(-1, 1), rng.uniform(-1, 1)]
                dt = rng.choice((0.01, 0.05, 0.1))
                expected = plant_step(state, control, dt, params)
                actual = model_step(state, control, dt, vehicle)
                for a, b in zip(actual, expected):
                    self.assertAlmostEqual(a, b, delta=1e-12)

    def test_unclipped_step_agrees_inside_the_limits(self):
        vehicle = course_vehicle()
        state, control = [0.0, 0.0, 0.1, 0.4, 0.05], [0.2, 0.1]
        self.assertEqual(model_step(state, control, 0.1, vehicle),
                         model_step(state, control, 0.1, vehicle, rate_limited=False))


class SettingsChecks(unittest.TestCase):
    def test_invalid_settings_are_rejected(self):
        for kwargs in ({"vehicle_params_source": "notebook"}, {"horizon_n": 1}, {"sqp_iterations": 0},
                       {"dt_pred_s": float("nan")}, {"dt_max_s": 0.01}, {"shadow": 1},
                       {"v_exec_max_mps": -0.1}, {"q_v": 0.0}):
            with self.assertRaises(ValueError, msg=str(kwargs)):
                Settings(**kwargs)

    def test_vehicle_source_selection(self):
        record = measured()
        self.assertIs(namespace["mpc_vehicle_params"](Settings(), record), record)
        self.assertEqual(namespace["mpc_vehicle_params"](Settings(vehicle_params_source="course_simulation"),
                                                         record).source, "course_simulation")
        self.assertFalse(namespace["mpc_vehicle_params"](Settings(), None).valid)


class SolverChecks(unittest.TestCase):
    def setUp(self):
        self.vehicle = measured()
        self.solver = Solver(Settings(), self.vehicle)

    def solve(self, traj=None, speed=0.3, delta=0.0, cap=0.3, last_drive=0.0, vehicle=None):
        solver = self.solver if vehicle is None else Solver(Settings(), vehicle)
        traj = straight_trajectory() if traj is None else traj
        return solver.solve(traj["points"], [0.0, 0.0, 0.0, speed, delta], cap, last_drive)

    def test_centred_holds_and_offsets_steer_back_symmetrically(self):
        out, why = self.solve()
        self.assertIsNone(why)
        self.assertAlmostEqual(out["steer0"], 0.0, places=4)
        self.assertGreater(out["drive0"], 0.0)
        left, _ = self.solve(straight_trajectory(y=-0.1))    # car left of the path
        right, _ = self.solve(straight_trajectory(y=0.1))
        self.assertLess(left["steer0"], 0.0)                 # -> steer right
        self.assertAlmostEqual(left["steer0"], -right["steer0"], places=4)
        heading, _ = self.solve(straight_trajectory(slope=-0.1, start_x=0.0))  # heading left of path
        self.assertLess(heading["steer0"], 0.0)

    def test_curvature_feedforward_and_negative_gain_mapping(self):
        out, _ = self.solve(arc_trajectory(0.6))
        self.assertGreater(out["delta0"], 0.0)
        flipped = measured(steering_gain_rad=-math.pi/4)
        other, _ = self.solve(arc_trajectory(0.6), vehicle=flipped)
        self.assertAlmostEqual(other["delta0"], out["delta0"], places=4)
        self.assertAlmostEqual(other["steer0"], -out["steer0"], places=4)

    def test_constraints_hold_on_the_whole_sequence(self):
        vehicle = measured(steering_rate_limit_rad_s=0.8, steering_offset_rad=0.03, steering_max_rad=0.5,
                           drive_min=-0.5, drive_max=0.4)
        step = 0.8*0.1
        for delta in (0.03, 0.3, -0.2):
            out, _ = self.solve(straight_trajectory(y=0.3, slope=0.3), delta=delta, vehicle=vehicle)
            deltas = [vehicle.steering_gain_rad*s + vehicle.steering_offset_rad for _, s in out["inputs"]]
            self.assertLessEqual(abs(deltas[0]-delta), step+1e-4)
            self.assertTrue(all(abs(b-a) <= step+1e-4 for a, b in zip(deltas, deltas[1:])))
            self.assertTrue(all(-math.pi/4-1e-4 <= v <= 0.5+1e-4 for v in deltas))
            self.assertTrue(all(-1e-6 <= d <= 0.4+1e-6 for d, _ in out["inputs"]))   # forward only
            self.assertTrue(all(abs(s) <= 1+1e-6 for _, s in out["inputs"]))

    def test_prediction_is_the_clipped_model_of_the_returned_inputs(self):
        out, _ = self.solve(straight_trajectory(y=-0.1))
        z = [0.0, 0.0, 0.0, 0.3, 0.0]
        for (drive, steer), v, delta in zip(out["inputs"], out["pred_v"], out["pred_delta"]):
            z = model_step(z, (drive, steer), 0.1, self.vehicle)
            self.assertAlmostEqual(z[3], v, places=12)
            self.assertAlmostEqual(z[4], delta, places=12)

    def test_starts_from_rest_and_respects_the_speed_cap(self):
        out, _ = self.solve(speed=0.0)
        self.assertGreater(out["drive0"], 0.0)
        capped, _ = self.solve(speed=0.0, cap=0.1)
        self.assertLess(capped["pred_v"][-1], out["pred_v"][-1])

    def test_short_reference_and_bad_inputs(self):
        self.assertEqual(self.solve(straight_trajectory(length=0.2, start_x=0.0), speed=1.0),
                         (None, "reference_too_short"))
        with self.assertRaises(Exception):
            self.solve(speed=float("nan"))


class ControllerBranches(unittest.TestCase):
    NOW = 10.0

    DEFAULT = object()

    def step(self, ctrl, traj=DEFAULT, speed=0.3, dt=0.1, first=False, now=NOW):
        return ctrl.step(straight_trajectory(y=-0.1) if traj is self.DEFAULT else traj, speed, dt, now, first)

    def test_shadow_computes_candidates_but_applies_zero_and_never_raises(self):
        ctrl = Controller(Settings(enabled=True, shadow=True, vehicle_params_source="course_simulation"))
        out = self.step(ctrl, first=True, dt=0.0)
        self.assertEqual((out["drive"], out["steer"]), (0.0, 0.0))
        d = out["debug"]
        self.assertEqual(d["branch"], "shadow")
        self.assertEqual(d["vehicle_source"], "course_simulation")
        self.assertLess(d["candidate_steer_action"], 0.0)   # path at y=-0.1: car is left of it -> steer right
        self.assertGreater(d["candidate_drive"], 0.0)
        for bad in (None, {"schema": "reference_trajectory_v0.1", "valid": False, "reason": "stale_road"},
                    straight_trajectory(stamp=1.0)):
            out = self.step(ctrl, traj=bad)
            self.assertEqual((out["drive"], out["steer"]), (0.0, 0.0))
            self.assertEqual(out["debug"]["branch"], "ref_invalid")
        self.assertEqual(self.step(ctrl, traj=dict(straight_trajectory(), stop_required=True))["debug"]["branch"], "stop")

    def test_unmeasured_vehicle_is_logged_in_shadow_and_refused_in_execution(self):
        out = self.step(Controller(Settings(enabled=True)), first=True, dt=0.0)
        self.assertEqual((out["debug"]["branch"], out["drive"], out["steer"]), ("vehicle_invalid", 0.0, 0.0))
        for ctrl in (Controller(Settings(enabled=True, shadow=False, v_exec_max_mps=0.3)),
                     Controller(Settings(enabled=True, shadow=False, v_exec_max_mps=0.3,
                                         vehicle_params_source="course_simulation")),
                     verified(v_exec_max_mps=0.0)):
            with self.assertRaises(Stop):
                self.step(ctrl, first=True, dt=0.0)
            self.assertEqual(ctrl.last_debug["branch"], "gate_refused")
        self.assertEqual(self.step(verified(), first=True, dt=0.0)["debug"]["branch"], "solved")

    def test_first_step_dt_zero_is_accepted_later_bad_dt_locks(self):
        ctrl = verified()
        self.step(ctrl, first=True, dt=0.0)
        for dt in (0.0, -0.1, 0.5, float("nan")):
            with self.assertRaises(Stop, msg=str(dt)):
                self.step(ctrl, dt=dt)
            self.assertEqual(ctrl.last_debug["reject_reason"], "dt_out_of_range")

    def test_rejections_lock_in_execution_mode(self):
        cases = {"ref_missing": None,
                 "stale_road": {"schema": "reference_trajectory_v0.1", "valid": False, "reason": "stale_road"},
                 "ref_expired_or_future": straight_trajectory(stamp=9.0),
                 "stop_required": dict(straight_trajectory(), stop_required=True),
                 "reference_too_short": straight_trajectory(length=0.2, start_x=0.0),
                 "overspeed": straight_trajectory()}
        for reason, traj in cases.items():
            ctrl = verified()
            with self.assertRaises(Stop, msg=reason):
                ctrl.step(traj, 0.45 if reason == "overspeed" else 0.3, 0.1, self.NOW, False)
            d = ctrl.last_debug
            self.assertEqual(reason, d["reject_reason"])
            self.assertEqual((d["applied_steer_action"], d["applied_drive"]), (0.0, 0.0))
        with self.assertRaises(Stop):
            self.step(verified(), speed=float("nan"))
        with self.assertRaises(Stop):
            self.step(verified(), traj=dict(straight_trajectory(), frame_id="odom"))

    def test_no_automatic_recovery_object_is_rebuilt_only_on_explicit_restart(self):
        ctrl = verified()
        self.step(ctrl, first=True, dt=0.0)
        with self.assertRaises(Stop):
            self.step(ctrl, traj={"schema": "reference_trajectory_v0.1", "valid": False, "reason": "x"})
        # The node rebuilds the controller only on is_first_policy_step (state-3 request).
        self.assertEqual((ctrl.applied_steer, ctrl.applied_drive), (0.0, 0.0))
        self.assertEqual(ctrl.history[-1][1:], (0.0, 0.0))
        self.assertIsNone(ctrl.nominal)

    def test_start_from_rest_drives_forward(self):
        out = self.step(verified(), speed=0.0, first=True, dt=0.0)
        self.assertEqual(out["debug"]["branch"], "solved")
        self.assertGreater(out["drive"], 0.0)

    def test_steering_estimate_replays_the_applied_commands(self):
        class Full:
            def solve(self, points, z0, cap, last_drive, nominal=None):
                return {"inputs": [(0.1, 1.0)]*10, "drive0": 0.1, "steer0": 1.0, "delta0": math.pi/4,
                        "iterations": 1, "solve_s": 0.0, "pred_e_y": [0.0], "pred_v": [0.3]}, None
        vehicle = measured(steering_offset_rad=0.02, steering_max_rad=0.5)
        ctrl = verified(vehicle=vehicle)
        ctrl.solver = Full()
        self.step(ctrl, first=True, dt=0.0)
        self.assertAlmostEqual(ctrl.last_debug["delta_est_rad"], 0.02)   # zero command maps to the offset
        for k in range(1, 6):
            self.step(ctrl)
            expected = min(0.5, 0.02 + math.pi/2*0.1*k)                     # rate limited towards the clip
            self.assertAlmostEqual(ctrl.last_debug["delta_est_rad"], expected, places=6)

    def test_actuator_delay_is_bridged_with_the_commands_already_sent(self):
        seen = []

        class Recorder:
            def solve(self, points, z0, cap, last_drive, nominal=None):
                seen.append(list(z0))
                return {"inputs": [(0.0, 0.5)]*10, "drive0": 0.0, "steer0": 0.5, "delta0": 0.4,
                        "iterations": 1, "solve_s": 0.0, "pred_e_y": [0.0], "pred_v": [0.3]}, None
        ctrl = verified(vehicle=measured(steering_delay_s=0.2, drive_delay_s=0.2, drag_kg_per_m=0.0))
        ctrl.solver = Recorder()
        self.step(ctrl, first=True, dt=0.0)
        self.assertAlmostEqual(seen[0][0], 0.3*0.2, places=6)   # coasting through the delay
        self.assertAlmostEqual(seen[0][4], 0.0)                  # nothing sent yet reaches the wheels
        self.step(ctrl)
        self.step(ctrl)
        self.assertGreater(seen[-1][4], 0.0)                     # the first command has now arrived

    def test_short_reference_dropout_coasts_then_locks(self):
        ctrl = verified(reference_dropout_tolerance_s=0.25)
        self.step(ctrl, first=True, dt=0.0)
        steer = ctrl.applied_steer
        bad = {"schema": "reference_trajectory_v0.1", "valid": False, "reason": "not_straight_enough"}
        for _ in range(2):                                  # 0.0 s and 0.1 s into the dropout
            out = self.step(ctrl, traj=bad)
            self.assertEqual(out["debug"]["branch"], "ref_hold")
            self.assertEqual((out["drive"], out["steer"]), (0.0, steer))
        self.assertEqual(self.step(ctrl)["debug"]["branch"], "solved")   # a valid frame resets the window
        for _ in range(3):
            self.step(ctrl, traj=bad)                        # 0.0, 0.1, 0.2 s
        with self.assertRaises(Stop):
            self.step(ctrl, traj=bad)                        # 0.3 s >= 0.25 s: locked
        self.assertEqual(ctrl.last_debug["branch"], "ref_invalid")

    def test_dropout_tolerance_never_covers_unsafe_rejections(self):
        for reason, call in (("overspeed", lambda c: c.step(straight_trajectory(), 0.45, 0.1, self.NOW, False)),
                             ("dt_out_of_range", lambda c: c.step(straight_trajectory(), 0.3, 0.5, self.NOW, False)),
                             ("stop_required", lambda c: c.step(dict(straight_trajectory(), stop_required=True),
                                                                0.3, 0.1, self.NOW, False))):
            ctrl = verified(reference_dropout_tolerance_s=0.5)
            self.step(ctrl, first=True, dt=0.0)
            with self.assertRaises(Stop, msg=reason):
                call(ctrl)
        with self.assertRaises(ValueError):
            Settings(reference_dropout_tolerance_s=0.8)

    def test_over_budget_result_is_not_executed(self):
        ctrl = verified(max_step_time_s=1e-9)
        with self.assertRaises(Stop):
            self.step(ctrl, first=True, dt=0.0)
        self.assertEqual(ctrl.last_debug["branch"], "over_budget")
        self.assertEqual(ctrl.last_debug["applied_steer_action"], 0.0)

    def test_solver_failure_is_not_success(self):
        class Failing:
            def solve(self, *args):
                raise Stop("solver_status:primal infeasible")
        ctrl = verified()
        ctrl.solver = Failing()
        with self.assertRaises(Stop):
            self.step(ctrl, first=True, dt=0.0)
        self.assertEqual(ctrl.last_debug["branch"], "solver_fail")
        shadow = Controller(Settings(enabled=True, vehicle_params_source="course_simulation"), solver=Failing())
        out = self.step(shadow, first=True, dt=0.0)
        self.assertEqual((out["drive"], out["steer"]), (0.0, 0.0))
        self.assertEqual(out["debug"]["branch"], "solver_fail")

    def test_out_of_bounds_solution_is_rejected(self):
        class Wild:
            def solve(self, *args):
                return {"inputs": [(0.1, 5.0)]*10, "drive0": 0.1, "steer0": 5.0, "delta0": 5.0,
                        "iterations": 1, "solve_s": 0.0, "pred_e_y": [0.0], "pred_v": [0.3]}, None
        ctrl = verified()
        ctrl.solver = Wild()
        with self.assertRaises(Stop):
            self.step(ctrl, first=True, dt=0.0)
        self.assertEqual(ctrl.last_debug["reject_reason"], "action_out_of_bounds")

    def test_zero_target_gives_zero_drive_without_solving(self):
        ctrl = verified()
        out = self.step(ctrl, traj=straight_trajectory(speed=0.0), first=True, dt=0.0)
        self.assertEqual(out["debug"]["branch"], "zero_target")
        self.assertEqual(out["drive"], 0.0)

    def test_debug_json_is_finite_and_complete(self):
        ctrl = Controller(Settings(enabled=True, vehicle_params_source="course_simulation"))
        self.step(ctrl, first=True, dt=0.0, speed=float("nan"))
        data = json.loads(debug_json(ctrl.last_debug))
        for key in ("branch", "candidate_steer_action", "candidate_drive", "applied_steer_action",
                    "reject_reason", "step_s", "delta_est_rad", "vehicle_source"):
            self.assertIn(key, data)


def body_points(path, x, y, psi, x_min=-0.2, x_max=2.5, speed=0.3):
    """World polyline [(x, y, heading, kappa)] seen from the car, as v0.1 points."""
    c, s = math.cos(psi), math.sin(psi)
    pts = []
    for px, py, th, kap in path:
        dx, dy = px-x, py-y
        bx, by = c*dx + s*dy, -s*dx + c*dy
        if x_min <= bx <= x_max:
            arc = 0.0 if not pts else pts[-1]["s_m"] + math.hypot(bx-pts[-1]["x_m"], by-pts[-1]["y_m"])
            pts.append({"s_m": arc, "x_m": bx, "y_m": by, "yaw_rad": th-psi, "curvature_1pm": kap,
                        "target_speed_mps": speed})
    return pts


def run_plant(controller, reference, z0, plant, steps, command_delay_s=0.0, noise=None, period=0.1):
    """Course model at 0.05 s, controller every `period`; reference(k, z) -> trajectory."""
    z = list(z0)
    pending = [(0.0, 0.0)]*round(command_delay_s/PLANT_DT)
    log = []
    for k in range(steps):
        traj = reference(k, z, k*period)
        speed = z[3] if noise is None else max(0.0, z[3] + noise())
        out = controller.step(traj, speed, 0.0 if k == 0 else period, k*period, k == 0)
        command = (out["drive"], out["steer"])
        for _ in range(round(period/PLANT_DT)):
            pending.append(command)
            z = list(plant_step(z, pending.pop(0), PLANT_DT, plant))
        log.append({"z": z, "drive": out["drive"], "steer": out["steer"],
                    "branch": out["debug"]["branch"], "step_s": out["debug"]["step_s"]})
    return log


def straight_world(length=40.0):
    return [(-1.0 + 0.05*i, 0.0, 0.0, 0.0) for i in range(int(length/0.05))]


def straight_reference(path):
    def reference(k, z, now):
        x, y, psi = z[0], z[1], z[2]
        near = [q for q in path if q[0] >= x-0.3][:60]
        return dict(straight_trajectory(stamp=now), path_id=k, points=body_points(near, x, y, psi))
    return reference


class ClosedLoop(unittest.TestCase):
    """Course prediction model as plant; E_LIMIT is the pre-fixed pass criterion."""

    def check(self, log, label, vehicle, cross=None):
        ey = [row["z"][1] for row in log] if cross is None else cross
        tail = ey[-len(ey)//3:]
        rmse = math.sqrt(sum(v*v for v in tail)/len(tail))
        self.assertTrue(all(row["branch"] == "solved" for row in log), label)
        self.assertLessEqual(max(abs(v) for v in ey), E_LIMIT, label)
        self.assertLessEqual(rmse, E_LIMIT/2, label)
        steer = [row["steer"] for row in log]
        self.assertTrue(all(abs(v) <= 1.0 for v in steer), label)
        self.assertTrue(all(row["drive"] >= 0.0 for row in log), label)
        rate = max(abs(b-a) for a, b in zip(steer, steer[1:]))*abs(vehicle.steering_gain_rad)
        self.assertLessEqual(rate, vehicle.steering_rate_limit_rad_s*0.1 + 1e-4, label)
        return rmse

    def run_straight(self, vehicle=None, plant=None, z0=(0.0, 0.15, 0.0, 0.3, 0.0), steps=150, **kw):
        vehicle = measured() if vehicle is None else vehicle
        plant = plant_params(vehicle) if plant is None else plant
        return run_plant(verified(vehicle=vehicle), straight_reference(straight_world()), z0, plant, steps, **kw)

    def test_offsets_and_heading_errors_converge(self):
        vehicle = measured()
        for y0, psi0 in ((0.15, 0.0), (-0.15, 0.0), (0.0, 0.0), (0.0, 0.09), (0.0, -0.09), (0.1, -0.08)):
            log = self.run_straight(z0=(0.0, y0, psi0, 0.3, 0.0))
            self.check(log, f"y0={y0} psi0={psi0}", vehicle)

    def test_start_from_rest_reaches_the_cap(self):
        log = self.run_straight(z0=(0.0, 0.12, 0.0, 0.0, 0.0), steps=200)
        self.check(log, "rest start", measured())
        self.assertAlmostEqual(log[-1]["z"][3], 0.3, delta=0.03)

    def test_plant_mismatch_like_the_pid_baseline(self):
        """tune.py scenarios: drive x0.7-0.8, drag x1.4-1.5, steering x0.85, offset +-2-3 deg, 100 ms delay."""
        vehicle = measured()
        cases = {"motor0.75_drag1.4": plant_params(vehicle, motor_scale=0.75, drag_scale=1.4),
                 "gain0.85_offset-3deg": plant_params(vehicle, gain_scale=0.85, offset=math.radians(-3),
                                                      motor_scale=0.7, drag_scale=1.5),
                 "offset+2deg": plant_params(vehicle, offset=math.radians(2), motor_scale=0.8)}
        for label, plant in cases.items():
            log = self.run_straight(plant=plant, z0=(0.0, 0.12, 0.0, 0.3, 0.0), steps=200)
            self.check(log, label, vehicle)
            self.assertAlmostEqual(log[-1]["z"][3], 0.3, delta=0.05, msg=label)
        for delay in (0.1, 0.2):
            log = self.run_straight(command_delay_s=delay, z0=(0.0, 0.12, 0.0, 0.3, 0.0))
            self.check(log, f"unmodelled delay {delay}", vehicle)

    def test_identified_delay_is_used_by_the_prediction(self):
        vehicle = measured(steering_delay_s=0.2, drive_delay_s=0.2)
        log = self.run_straight(vehicle=vehicle, command_delay_s=0.2, z0=(0.0, 0.15, 0.0, 0.3, 0.0))
        known = self.check(log, "modelled delay", vehicle)
        blind = self.check(self.run_straight(command_delay_s=0.2, z0=(0.0, 0.15, 0.0, 0.3, 0.0)),
                           "same delay, unmodelled", measured())
        self.assertLessEqual(known, blind + 1e-3)

    def test_speed_noise(self):
        for seed in (1, 2, 3):
            rng = random.Random(seed)
            log = self.run_straight(z0=(0.0, 0.1, 0.0, 0.3, 0.0), noise=lambda: rng.gauss(0, 0.02))
            self.check(log, f"noise seed {seed}", measured())

    def test_step_timing_report(self):
        log = self.run_straight(steps=200)
        times = sorted(row["step_s"] for row in log)
        p95 = times[int(0.95*len(times))-1]
        print(f"\nMPC step time (laptop, not the target platform): mean {statistics.mean(times)*1e3:.2f} ms, "
              f"p95 {p95*1e3:.2f} ms, max {times[-1]*1e3:.2f} ms; "
              f"over 50 ms budget: {sum(t > 0.05 for t in times)}/{len(times)}")
        self.assertEqual(sum(t > 0.05 for t in times), 0)


def road_record(points, stamp=10.0, age=0.05, **kw):
    road = {"valid": True, "time_aligned": True, "frame_id": "base_link", "timestamp_s": stamp,
            "measurement_timestamp_s": stamp-age, "source_age_s": age, "centerline_xy": points}
    road.update(kw)
    state = {"timestamp_s": stamp, "speed_valid": True, "yaw_rate_valid": True}
    return road, state


def arc_centerline(kappa, x0=0.3, x1=2.0, n=45):
    """Circle of curvature kappa through the origin, tangent to +x, sampled over x in [x0, x1]."""
    pts = []
    for i in range(n):
        x = x0 + (x1-x0)*i/(n-1)
        pts.append((x, 0.0) if kappa == 0 else (x, (1-math.sqrt(1-(kappa*x)**2))/kappa))
    return pts


class BypassReference(unittest.TestCase):
    def cfg(self, **kw):
        return Settings(**kw)

    def test_straight_and_curved_roads_give_signed_curvature_and_extended_start(self):
        for kappa in (0.0, 0.4, -0.4):
            road, state = road_record(arc_centerline(kappa))
            traj = from_road(road, state, self.cfg())
            self.assertTrue(traj["valid"], traj["reason"])
            self.assertTrue(traj["planning_bypassed"])
            self.assertEqual(traj["reference_source"], "estimation_centerline")
            mid = traj["points"][len(traj["points"])//2]
            self.assertAlmostEqual(mid["curvature_1pm"], kappa, delta=0.06)   # quadratic fit of an arc
            self.assertAlmostEqual(traj["points"][0]["x_m"], -0.1)       # fit extended back to the car
            self.assertAlmostEqual(traj["extrapolated_back_m"], 0.4)
            s = [p["s_m"] for p in traj["points"]]
            self.assertTrue(all(b > a for a, b in zip(s, s[1:])))
            info, why = errors(traj, 10, 0.1, 0.3, 0.6)
            self.assertIsNone(why)
            self.assertAlmostEqual(info["e_y_m"], 0.0, delta=0.03)       # car sits on the arc; fit bias at the car
            self.assertAlmostEqual(info["e_psi_rad"], 0.0, delta=0.06)   # ~2.7 deg fit bias at kappa=0.4

    def test_timestamp_and_lifetime_follow_the_estimate_not_the_clock(self):
        road, state = road_record(arc_centerline(0.0), age=0.05)
        traj = from_road(road, state, self.cfg())
        self.assertEqual(traj["timestamp_s"], state["timestamp_s"])
        self.assertAlmostEqual(traj["valid_for_s"], min(0.1, 0.2-0.05))

    def test_rejections(self):
        cfg = self.cfg()
        good = arc_centerline(0.0)
        cases = {
            "road_or_state_invalid": road_record(good, valid=False),
            "road_not_aligned_to_state": road_record(good, timestamp_s=9.5),
            "road_stale": road_record(good, age=0.25),
            "near_coverage_missing": road_record(arc_centerline(0.0, x0=0.8)),
            "forward_coverage_short": road_record(arc_centerline(0.0, x0=0.3, x1=0.9)),
            "curvature_too_large": road_record([(0.3 + 0.05*i, 0.55*(0.3 + 0.05*i)**2) for i in range(40)]),
            "bad_centerline": road_record(good[:3]),
            "fit_error_too_large": road_record([(x, y + (0.1 if i % 2 else 0.0)) for i, (x, y) in enumerate(good)]),
        }
        for reason, (road, state) in cases.items():
            traj = from_road(road, state, cfg)
            self.assertFalse(traj["valid"], reason)
            self.assertTrue(traj["stop_required"])
            self.assertEqual(traj["reason"], reason)
        road, state = road_record(good)
        state["speed_valid"] = False
        self.assertEqual(from_road(road, state, cfg)["reason"], "road_or_state_invalid")
        self.assertFalse(from_road(None, None, cfg)["valid"])

    def test_bypass_execution_needs_explicit_acknowledgement(self):
        base = dict(reference_source="estimation_centerline")
        traj = from_road(*road_record(arc_centerline(0.0)), Settings(**base))
        with self.assertRaises(Stop):
            verified(**base).step(traj, 0.3, 0.0, 10.0, True)
        ctrl = verified(bypass_acknowledged=True, **base)
        self.assertEqual(ctrl.step(traj, 0.3, 0.0, 10.0, True)["debug"]["branch"], "solved")
        self.assertEqual(ctrl.last_debug["planning_bypassed"], True)
        with self.assertRaises(ValueError):
            Settings(reference_source="odometry")


def world_path(segments, step=0.02, lead=1.0):
    """World polyline starting `lead` metres before the origin; segments = [(length, kappa), ...]."""
    bounds, acc = [], 0.0
    for length, kappa in segments:
        acc += length
        bounds.append((acc, kappa))
    total = lead + acc
    pts, x, y, th, s = [], -lead, 0.0, 0.0, 0.0
    while s <= total + 1e-9:
        kappa = 0.0
        if s >= lead:
            kappa = next((k for end, k in bounds if s-lead < end), bounds[-1][1])
        pts.append((x, y, th, kappa))
        x, y, th, s = x + step*math.cos(th), y + step*math.sin(th), th + step*kappa, s + step
    return pts


def centerline_reference(path, cfg, noise_sd=0.0, seed=0):
    """Estimator-like view: centerline points 0.3-1.8 m ahead, through the bypass adapter."""
    rng = random.Random(seed)

    def reference(k, z, now):
        x, y, psi = z[0], z[1], z[2]
        c, sn = math.cos(psi), math.sin(psi)
        near_i = min(range(len(path)), key=lambda i: (path[i][0]-x)**2 + (path[i][1]-y)**2)
        body = []
        for px, py, _, _ in path[max(0, near_i-25):near_i+125]:
            dx, dy = px-x, py-y
            bx, by = c*dx + sn*dy, -sn*dx + c*dy
            if 0.3 <= bx <= 1.8:
                body.append((bx, by + (rng.gauss(0, noise_sd) if noise_sd else 0.0)))
        road = {"valid": True, "time_aligned": True, "frame_id": "base_link", "timestamp_s": now,
                "measurement_timestamp_s": now-0.05, "source_age_s": 0.05, "centerline_xy": body[::5]}
        return from_road(road, {"timestamp_s": now, "speed_valid": True, "yaw_rate_valid": True}, cfg)
    return reference


def cross_track(path, log):
    errors_m = []
    for row in log:
        x, y = row["z"][0], row["z"][1]
        near = min(path, key=lambda q: (q[0]-x)**2 + (q[1]-y)**2)
        errors_m.append(-(x-near[0])*math.sin(near[2]) + (y-near[1])*math.cos(near[2]))
    return errors_m


class BendClosedLoop(unittest.TestCase):
    """Bypass reference on arcs (R = 2.5 m) with the course model plant; same pre-fixed E_LIMIT."""
    check = ClosedLoop.check

    def run_bend(self, path, y0=0.0, plant=None, command_delay_s=0.0, noise_sd=0.0, seed=0):
        vehicle = measured()
        ctrl = verified(vehicle=vehicle, reference_source="estimation_centerline", bypass_acknowledged=True,
                        bypass_target_speed_mps=0.3)
        log = run_plant(ctrl, centerline_reference(path, ctrl.cfg, noise_sd, seed), (0.0, y0, 0.0, 0.3, 0.0),
                        plant_params(vehicle) if plant is None else plant, 150, command_delay_s)
        return log, cross_track(path, log), vehicle

    def test_left_right_and_s_bends(self):
        scenarios = {"left": [(1.0, 0.0), (6.0, 0.4)], "right": [(1.0, 0.0), (6.0, -0.4)],
                     "s_bend": [(1.0, 0.0), (2.5, 0.4), (2.5, -0.4), (1.0, 0.0)]}
        for name, segments in scenarios.items():
            for y0 in (0.0, 0.1, -0.1):
                log, cross, vehicle = self.run_bend(world_path(segments), y0=y0)
                rmse = self.check(log, f"{name} y0={y0}", vehicle, cross)
                print(f"\n{name} y0={y0:+.1f}: max |e_y| {max(abs(v) for v in cross):.3f} m, "
                      f"tail RMSE {rmse:.3f} m", end="")

    def test_bends_with_mismatch_delay_and_centerline_noise(self):
        path = world_path([(1.0, 0.0), (6.0, 0.4)])
        vehicle = measured()
        for label, plant, delay in (("gain0.85", plant_params(vehicle, gain_scale=0.85), 0.0),
                                    ("offset-3deg", plant_params(vehicle, offset=math.radians(-3)), 0.0),
                                    ("delay0.1", None, 0.1)):
            log, cross, _ = self.run_bend(path, y0=0.05, plant=plant, command_delay_s=delay)
            self.check(log, label, vehicle, cross)
        for seed in (1, 2, 3):
            log, cross, _ = self.run_bend(path, y0=0.05, noise_sd=0.01, seed=seed)
            self.check(log, f"noise seed {seed}", vehicle, cross)


class ConfigChecks(unittest.TestCase):
    def load(self, name):
        import yaml
        text = (ROOT / "config" / name).read_text(encoding="utf-8")
        return yaml.safe_load(text)["/**/ai4r_policy"]["ros__parameters"]

    def test_shipped_yaml_sections_match_declared_defaults_and_are_off(self):
        params = self.load("ai4r_policy.yaml")
        self.assertEqual(params["mpc"], vars(Settings()))
        self.assertFalse(params["mpc"]["enabled"])
        self.assertTrue(params["mpc"]["shadow"])
        self.assertEqual(params["mpc"]["v_exec_max_mps"], 0.0)
        self.assertEqual(params["vehicle"], vars(Vehicle()))
        self.assertFalse(params["vehicle"]["valid"])
        self.assertEqual(params["id_test"]["mode"], "off")

    def test_bypass_overlay_is_shadow_and_unacknowledged(self):
        overlay = self.load("ai4r_policy_mpc_bypass.yaml")
        self.assertEqual(overlay["mpc"], {"enabled": True, "shadow": True,
                                          "reference_source": "estimation_centerline",
                                          "vehicle_params_source": "course_simulation"})
        merged = dict(self.load("ai4r_policy.yaml")["mpc"], **overlay["mpc"])
        self.assertFalse(merged["bypass_acknowledged"])
        Settings(**merged)

    def test_prototype_overlay_enables_shadow_only(self):
        overlay = self.load("ai4r_policy_mpc_prototype.yaml")
        self.assertEqual(overlay["mpc"], {"enabled": True, "shadow": True,
                                          "vehicle_params_source": "course_simulation",
                                          "max_backward_extension_m": 2.0})
        self.assertEqual(overlay["planning"], {"vehicle_limits_source": "course_simulation",
                                               "max_near_x_m": 1.45, "max_curvature_1pm": 0.3,
                                               "max_fit_error_m": 0.06})
        PlanSettings(**dict(self.load("ai4r_policy.yaml")["planning"], **overlay["planning"]))
        self.assertEqual(set(overlay["required_sensors"]),
                         {"cone_detections", "wheel_speed", "imu_angular_velocity"})
        merged = dict(self.load("ai4r_policy.yaml")["mpc"], **overlay["mpc"])
        Settings(**merged)


class NodeEntry(unittest.TestCase):
    """The real calculate_policy_actions, with a stand-in node (no ROS)."""

    def make_node(self, vehicle=None, **mpc):
        api = planning_api["api"]
        obs, ages, stamps, receipts = api["input_record"](speed=0.2, yaw=0.0)
        for key in ("fiducial_detections", "lidar_scan", "imu_orientation", "imu_specific_force"):
            obs[key] = None
        published = []
        node = SimpleNamespace(
            estimation_settings=api["Settings"](), policy_frame_id="base_link",
            planning_settings=PlanSettings(vehicle_limits_source="course_simulation"),
            mpc_settings=Settings(**mpc) if mpc is not None else None, mpc_controller=None,
            mpc_debug_publisher=SimpleNamespace(publish=lambda msg: published.append(msg.data)),
            sensor_timeout_s={key: 0.5 for key in obs}, heading_reference=None,
            motion_history=api["Motion"](api["Settings"]()),
            observations={key: SimpleNamespace(received_at=receipts.get(key), received_ros_ns=10_000_000_000)
                          for key in obs},
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_050_000_000)))
        if vehicle is not None:
            node.vehicle_settings = vehicle
        return node, (obs, ages, stamps, receipts), published

    def test_disabled_mpc_keeps_zero_actions_and_publishes_nothing(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(enabled=False)
        result = namespace["calculate_policy_actions"](node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual(result, (0.0, 0.0, None, None, None))
        self.assertEqual(published, [])

    def test_shadow_entry_returns_zero_and_logs_candidate(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(
            enabled=True, shadow=True, vehicle_params_source="course_simulation")
        method = namespace["calculate_policy_actions"]
        drive, steer, pan, debug1, debug2 = method(node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual((drive, steer, pan), (0.0, 0.0, None))
        record = json.loads(published[-1])
        self.assertTrue(record["simulation_only"])
        self.assertEqual(record["applied_steer_action"], 0.0)
        self.assertIn(record["branch"], ("shadow", "ref_invalid"))
        if record["branch"] == "shadow":
            self.assertIsNotNone(debug1)
            self.assertIsNotNone(debug2)
            self.assertIsNotNone(record["candidate_steer_action"])

    def test_bypass_entry_uses_estimator_centerline_and_marks_the_log(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(
            enabled=True, shadow=True, reference_source="estimation_centerline",
            vehicle_params_source="course_simulation")
        method = namespace["calculate_policy_actions"]
        drive, steer, pan, debug1, debug2 = method(node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual((drive, steer), (0.0, 0.0))
        record = json.loads(published[-1])
        self.assertEqual(record["reference_source"], "estimation_centerline")
        self.assertTrue(record["planning_bypassed"])
        self.assertIn(record["branch"], ("shadow", "ref_invalid"))
        if record["branch"] == "ref_invalid":
            self.fail(record["reject_reason"])

    def test_measured_vehicle_record_replaces_the_estimator_placeholders(self):
        vehicle = measured()
        node, (obs, ages, stamps, receipts), _ = self.make_node(vehicle=vehicle, enabled=False)
        namespace["calculate_policy_actions"](node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual(node.estimation_output["vehicle_limits"], vehicle.records()[1])
        node, (obs, ages, stamps, receipts), _ = self.make_node(vehicle=Vehicle(), enabled=False)
        namespace["calculate_policy_actions"](node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual(node.estimation_output["vehicle_limits"]["source"], "unmeasured")

    def test_execution_refused_and_logged_when_unverified(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(enabled=True, shadow=False)
        with self.assertRaises(Stop):
            namespace["calculate_policy_actions"](node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual(json.loads(published[-1])["branch"], "gate_refused")


if __name__ == "__main__":
    unittest.main()
