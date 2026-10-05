"""Minimal lateral MPC contracts against the actual single-file source; no ROS needed.

The pure MPC definitions are extracted from scripts/policy_node.py like test_planning does.
The closed loops use a nonlinear kinematic plant whose parameters differ from the
controller's: they show that this implementation is self-consistent under modest
mismatch, not that the real car behaves the same way.
"""
import ast
from pathlib import Path
import json
import math
import runpy
import statistics
import time
from types import SimpleNamespace
import unittest

planning_api = runpy.run_path(str(Path(__file__).with_name("test_planning.py")))
namespace = planning_api["api"]["namespace"]
SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "policy_node.py"
NAMES = {"wrap_angle", "MPCStop", "MPCSettings", "SpeedPI", "reference_trajectory_from_planning",
         "reference_trajectory_from_road",
         "mpc_reference_errors", "MinimalLateralMPC", "MPCController", "mpc_debug_json"}
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
definitions = [item for item in tree.body if isinstance(item, (ast.FunctionDef, ast.ClassDef))
               and item.name in NAMES]
assert {item.name for item in definitions} == NAMES
namespace.update(time=time, json=json, String=lambda data: SimpleNamespace(data=data))
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), namespace)
Settings = namespace["MPCSettings"]
Stop = namespace["MPCStop"]
Controller = namespace["MPCController"]
Solver = namespace["MinimalLateralMPC"]
SpeedPI = namespace["SpeedPI"]
adapt = namespace["reference_trajectory_from_planning"]
from_road = namespace["reference_trajectory_from_road"]
errors = namespace["mpc_reference_errors"]
debug_json = namespace["mpc_debug_json"]
Planner, PlanSettings, plan_fixture = (planning_api["Planner"], planning_api["Settings"],
                                       planning_api["fixture"])

# Closed-loop pass criteria, fixed before looking at results: 1.0 m road, 0.25 m car, 0.05 m margin.
E_LIMIT = (1.0-0.25)/2 - 0.05
L, L_R = 0.33, 0.132


def verified(**kwargs):
    base = dict(enabled=True, shadow=False, mapping_verified=True, limits_verified=True,
                v_exec_max_mps=0.3)
    base.update(kwargs)
    return Settings(**base)


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


class SettingsChecks(unittest.TestCase):
    def test_mapping_roundtrip_with_offset(self):
        cfg = Settings(steering_offset_rad=0.02, steering_sign=-1.0, steering_gain_rad_per_action=0.25)
        for action in (-1.0, -0.3, 0.0, 0.7, 1.0):
            self.assertAlmostEqual(cfg.action_of_delta(cfg.delta_of_action(action)), action)
        self.assertAlmostEqual(cfg.delta_of_action(0.0), 0.02)   # zero command is not 0 rad

    def test_invalid_settings_are_rejected(self):
        for kwargs in ({"steering_sign": 0.5}, {"steering_gain_rad_per_action": 0.0},
                       {"horizon_n": 1}, {"dt_pred_s": float("nan")}, {"drive_max": 1.5},
                       {"steering_offset_rad": 0.5}, {"dt_max_s": 0.01}, {"shadow": 1}):
            with self.assertRaises(ValueError, msg=str(kwargs)):
                Settings(**kwargs)

    def test_delta_range_follows_reachable_mapping(self):
        cfg = Settings(steering_gain_rad_per_action=0.2, steering_limit_rad=0.35)
        self.assertEqual(cfg.delta_range(), (-0.2, 0.2))
        cfg = Settings(steering_gain_rad_per_action=0.6, steering_limit_rad=0.35)
        self.assertEqual(cfg.delta_range(), (-0.35, 0.35))


class SolverChecks(unittest.TestCase):
    def setUp(self):
        self.cfg = Settings()
        self.solver = Solver(self.cfg)

    def solve(self, e_y=0.0, e_psi=0.0, v=0.3, kappa=0.0, prev=0.0):
        return self.solver.solve(e_y, e_psi, v, [kappa]*self.cfg.horizon_n, prev)

    def test_centred_is_zero_and_corrections_are_symmetric(self):
        self.assertAlmostEqual(self.solve()["delta0"], 0.0, places=5)
        left, right = self.solve(e_y=0.15)["delta0"], self.solve(e_y=-0.15)["delta0"]
        self.assertLess(left, 0)          # left of path -> steer right (negative)
        self.assertAlmostEqual(left, -right, places=5)
        self.assertLess(self.solve(e_psi=0.1)["delta0"], 0)   # heading left of path -> steer right

    def test_constraints_hold_including_first_step_from_previous_delta(self):
        step = self.cfg.steering_rate_limit_rad_s*self.cfg.dt_pred_s
        low, high = self.cfg.delta_range()
        for prev in (0.0, 0.1, -0.2, 0.02):
            out = self.solve(e_y=0.3, e_psi=0.2, prev=prev)
            seq = out["delta_seq"]
            self.assertLessEqual(abs(seq[0]-prev), step+1e-4)
            self.assertTrue(all(abs(b-a) <= step+1e-4 for a, b in zip(seq, seq[1:])))
            self.assertTrue(all(low-1e-4 <= v <= high+1e-4 for v in seq))   # accepted residual tolerance
            self.assertLess(out["violation"], 1e-4)
        # Large initial error saturates the rate bound relative to prev, not relative to zero.
        self.assertAlmostEqual(self.solve(e_y=0.5, prev=0.2)["delta0"], 0.2-step, places=4)

    def test_curvature_feedforward(self):
        out = self.solve(kappa=0.5)
        self.assertGreater(out["delta0"], 0.0)

    def test_predicted_state_matches_the_model(self):
        out = self.solve(e_y=0.1, e_psi=0.05)
        a = 0.3*0.1
        ey, ep, d = 0.1, 0.05, out["delta_seq"][0]
        ey, ep = ey + a*(ep + L_R/L*d), ep + a/L*d
        self.assertAlmostEqual(out["pred_e_y"][0], ey, places=9)
        self.assertAlmostEqual(out["pred_e_psi"][0], ep, places=9)

    def test_bad_inputs_fail_cleanly(self):
        with self.assertRaises(Exception):
            self.solve(e_y=float("nan"))


class ControllerBranches(unittest.TestCase):
    NOW = 10.0

    DEFAULT = object()

    def step(self, ctrl, traj=DEFAULT, speed=0.3, dt=0.1, first=False, now=NOW):
        return ctrl.step(straight_trajectory(y=-0.1) if traj is self.DEFAULT else traj, speed, dt, now, first)

    def test_shadow_computes_candidates_but_applies_zero_and_never_raises(self):
        ctrl = Controller(Settings(enabled=True, shadow=True, steering_offset_rad=0.03, v_exec_max_mps=0.0))
        out = self.step(ctrl, first=True, speed=0.0, dt=0.0)
        self.assertEqual((out["drive"], out["steer"]), (0.0, 0.0))
        d = out["debug"]
        self.assertEqual(d["branch"], "shadow")
        self.assertTrue(d["model_speed_substituted"])
        self.assertIsNotNone(d["candidate_delta_rad"])
        self.assertLess(d["candidate_steer_action"], 0.0)         # path at y=-0.1: car is left of it -> steer right
        self.assertAlmostEqual(ctrl.prev_delta, 0.03)             # zero command, not 0 rad
        for bad in (None, {"schema": "reference_trajectory_v0.1", "valid": False, "reason": "stale_road"},
                    straight_trajectory(stamp=1.0)):
            out = self.step(ctrl, traj=bad)
            self.assertEqual((out["drive"], out["steer"]), (0.0, 0.0))
            self.assertEqual(out["debug"]["branch"], "ref_invalid")
        self.assertEqual(self.step(ctrl, traj=dict(straight_trajectory(), stop_required=True))["debug"]["branch"], "stop")

    def test_gate_refuses_unverified_execution(self):
        for kwargs in ({"mapping_verified": False}, {"limits_verified": False}, {"v_exec_max_mps": 0.0}):
            ctrl = Controller(verified(**kwargs))
            with self.assertRaises(Stop):
                self.step(ctrl, first=True, dt=0.0)
            self.assertEqual(ctrl.last_debug["branch"], "gate_refused")
        ctrl = Controller(verified())
        self.assertEqual(self.step(ctrl, first=True, dt=0.0)["debug"]["branch"], "solved")

    def test_first_step_dt_zero_is_accepted_later_bad_dt_locks(self):
        ctrl = Controller(verified())
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
                 "reference_too_short": straight_trajectory(length=0.2, start_x=0.0)}
        for reason, traj in cases.items():
            ctrl = Controller(verified())
            with self.assertRaises(Stop, msg=reason):
                ctrl.step(traj, 0.3, 0.1, self.NOW, False)
            d = ctrl.last_debug
            self.assertEqual(reason, d["reject_reason"])
            self.assertEqual((d["applied_steer_action"], d["applied_drive"]), (0.0, 0.0))
        with self.assertRaises(Stop):
            self.step(Controller(verified()), speed=float("nan"))
        with self.assertRaises(Stop):
            self.step(Controller(verified()), traj=dict(straight_trajectory(), frame_id="odom"))

    def test_no_automatic_recovery_object_is_rebuilt_only_on_explicit_restart(self):
        ctrl = Controller(verified())
        with self.assertRaises(Stop):
            self.step(ctrl, traj={"schema": "reference_trajectory_v0.1", "valid": False, "reason": "x"})
        # The node rebuilds the controller only on is_first_policy_step (state-3 request).
        self.assertEqual(ctrl.applied_steer, 0.0)
        self.assertEqual(ctrl.prev_delta, ctrl.cfg.delta_of_action(0.0))

    def test_low_speed_holds_steering_and_runs_pi(self):
        ctrl = Controller(verified())
        out = self.step(ctrl, speed=0.0, first=True, dt=0.0)
        self.assertEqual(out["debug"]["branch"], "low_speed")
        self.assertEqual(out["steer"], 0.0)
        self.assertGreater(out["drive"], 0.0)
        self.assertLessEqual(out["drive"], 0.15)

    def test_previous_delta_follows_applied_command_with_offset(self):
        ctrl = Controller(verified(steering_offset_rad=0.02))
        out = self.step(ctrl, first=True, dt=0.0)
        self.assertAlmostEqual(ctrl.prev_delta, ctrl.cfg.delta_of_action(out["steer"]))
        step = ctrl.cfg.steering_rate_limit_rad_s*ctrl.cfg.dt_pred_s
        self.assertLessEqual(abs(ctrl.cfg.delta_of_action(out["steer"]) - 0.02), step+1e-6)

    def test_over_budget_result_is_not_executed(self):
        ctrl = Controller(verified(max_step_time_s=1e-9))
        with self.assertRaises(Stop):
            self.step(ctrl, first=True, dt=0.0)
        self.assertEqual(ctrl.last_debug["branch"], "over_budget")
        self.assertEqual(ctrl.last_debug["applied_steer_action"], 0.0)

    def test_solver_failure_is_not_success(self):
        class Failing:
            def solve(self, *args):
                raise Stop("solver_status:primal infeasible")
        ctrl = Controller(verified(), solver=Failing())
        with self.assertRaises(Stop):
            self.step(ctrl, first=True, dt=0.0)
        self.assertEqual(ctrl.last_debug["branch"], "solver_fail")
        shadow = Controller(Settings(enabled=True), solver=Failing())
        out = self.step(shadow, first=True, dt=0.0)
        self.assertEqual((out["drive"], out["steer"]), (0.0, 0.0))
        self.assertEqual(out["debug"]["branch"], "solver_fail")

    def test_clip_counter_stops_after_repeated_saturation(self):
        class Wild:
            def solve(self, *args):
                return {"delta0": 5.0, "iterations": 1, "solve_s": 0.0}
        ctrl = Controller(verified(max_clip_count=1), solver=Wild())
        self.step(ctrl, first=True, dt=0.0)          # clip 1 allowed (reported, applied clipped)
        self.assertEqual(ctrl.last_debug["applied_steer_action"], 1.0)
        with self.assertRaises(Stop):
            self.step(ctrl)

    def test_zero_target_gives_zero_drive_without_steering_solve(self):
        ctrl = Controller(verified())
        out = self.step(ctrl, traj=straight_trajectory(speed=0.0), first=True, dt=0.0)
        self.assertEqual(out["debug"]["branch"], "zero_target")
        self.assertEqual(out["drive"], 0.0)

    def test_debug_json_is_finite_and_complete(self):
        ctrl = Controller(Settings(enabled=True))
        self.step(ctrl, first=True, dt=0.0, speed=float("nan"))
        data = json.loads(debug_json(ctrl.last_debug))
        for key in ("branch", "candidate_steer_action", "applied_steer_action", "reject_reason", "step_s"):
            self.assertIn(key, data)


class SpeedControl(unittest.TestCase):
    def test_saturation_reset_and_antiwindup(self):
        pi = SpeedPI(1.0, 0.4, 0.0, 0.15)
        for _ in range(50):
            drive = pi.update(0.3, 0.0, 0.1)
        self.assertEqual(drive, 0.15)
        self.assertLess(pi.integral, 0.5)           # not wound up while saturated
        self.assertEqual(pi.update(0.0, 0.2, 0.1), 0.0)
        self.assertEqual(pi.integral, 0.0)
        self.assertEqual(pi.update(0.3, 0.0, 0.0, integrate=False), 0.15)
        self.assertEqual(pi.integral, 0.0)


def simulate(controller, true_gain=0.3, true_offset=0.0, delay_steps=1, y0=0.15, psi0=0.0,
             steps=300, v0=0.3, dt=0.1, tau=0.5, drive_gain=2.0, noise=None):
    """Nonlinear CG kinematic plant; the road is the world x axis (y = 0)."""
    x, y, psi, v = 0.0, y0, psi0, v0
    pending = [0.0]*delay_steps
    log = []
    for k in range(steps):
        pts = []
        for i in range(60):
            wx = x - 0.2 + 0.05*i
            dx, dy = wx - x, -y
            pts.append({"s_m": 0.05*i, "x_m": math.cos(psi)*dx + math.sin(psi)*dy,
                        "y_m": -math.sin(psi)*dx + math.cos(psi)*dy, "yaw_rad": -psi,
                        "curvature_1pm": 0.0, "target_speed_mps": 0.3})
        traj = {"schema": "reference_trajectory_v0.1", "frame_id": "base_link", "timestamp_s": k*dt,
                "valid_for_s": 0.1, "valid": True, "stop_required": False, "status": "TRACK",
                "reason": None, "path_id": k, "simulation_only": True, "source_ages_s": {}, "points": pts}
        meas_v = v if noise is None else max(0.0, v + noise(k))
        out = controller.step(traj, meas_v, 0.0 if k == 0 else dt, k*dt, k == 0)
        pending.append(out["steer"])
        steer = pending.pop(0)
        delta = true_gain*steer + true_offset
        beta = math.atan(L_R/L*math.tan(delta))
        x += v*math.cos(psi+beta)*dt
        y += v*math.sin(psi+beta)*dt
        psi += v/L*math.tan(delta)*math.cos(beta)*dt
        v += dt/tau*(drive_gain*out["drive"] - v)
        log.append((y, psi, v, out["steer"], out["debug"]["step_s"]))
    return log


class ClosedLoop(unittest.TestCase):
    """Self-consistency under modest mismatch; E_LIMIT is the pre-fixed pass criterion."""

    def check(self, log, label):
        ey = [abs(row[0]) for row in log]
        tail = [row[0] for row in log[-len(log)//3:]]
        rmse = math.sqrt(sum(v*v for v in tail)/len(tail))
        self.assertLessEqual(max(ey), E_LIMIT, label)
        self.assertLessEqual(rmse, E_LIMIT/2, label)
        steer = [row[3] for row in log]
        self.assertTrue(all(abs(v) <= 1.0 for v in steer), label)
        rate = max(abs(b-a) for a, b in zip(steer, steer[1:]))
        self.assertLessEqual(rate*0.3, 1.0*0.1 + 1e-6, label)       # <= rate limit * dt (rad)
        return rmse

    def test_offsets_and_heading_errors_converge(self):
        for y0, psi0 in ((0.15, 0.0), (-0.15, 0.0), (0.0, 0.0), (0.0, 0.09), (0.0, -0.09), (0.1, -0.08)):
            log = simulate(Controller(verified()), y0=y0, psi0=psi0)
            self.check(log, f"y0={y0} psi0={psi0}")

    def test_start_from_rest_with_speed_loop(self):
        log = simulate(Controller(verified()), v0=0.0, y0=0.12, steps=400)
        self.check(log, "rest start")
        self.assertAlmostEqual(log[-1][2], 0.3, delta=0.03)

    def test_model_mismatch_gain_offset_and_delay(self):
        for gain in (0.21, 0.39):
            for delay in (0, 1, 2):
                log = simulate(Controller(verified()), true_gain=gain, delay_steps=delay, y0=0.12)
                self.check(log, f"gain={gain} delay={delay}")
        log = simulate(Controller(verified()), true_offset=0.02, y0=0.0)   # unmodelled 1.1 deg zero offset
        self.assertLessEqual(max(abs(r[0]) for r in log), E_LIMIT)

    def test_speed_noise(self):
        seeds = []
        for seed in (1, 2, 3):
            import random
            rng = random.Random(seed)
            seeds.append(simulate(Controller(verified()), y0=0.1, noise=lambda k: rng.gauss(0, 0.02)))
        for i, log in enumerate(seeds):
            self.check(log, f"noise seed {i+1}")

    def test_step_timing_report(self):
        log = simulate(Controller(verified()), steps=200)
        times = sorted(row[4] for row in log)
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
            Controller(verified(**base)).step(traj, 0.3, 0.0, 10.0, True)
        ctrl = Controller(verified(bypass_acknowledged=True, **base))
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


def simulate_curve(controller, path, y0=0.0, psi0=0.0, steps=150, dt=0.1, v0=0.3, tau=0.5,
                   drive_gain=2.0, true_gain=0.3, delay_steps=1, noise_sd=0.0, seed=0):
    import random
    rng = random.Random(seed)
    x, y, psi, v = 0.0, y0, psi0, v0
    pending = [0.0]*delay_steps
    cfg = controller.cfg
    log = []
    for k in range(steps):
        c, sn = math.cos(psi), math.sin(psi)
        body = []
        # estimator-like view: only path points within ~2.5 m of arc ahead of the car
        near_i = min(range(len(path)), key=lambda i: (path[i][0]-x)**2 + (path[i][1]-y)**2)
        for px, py, th, kap in path[max(0, near_i-25):near_i+125]:
            dx, dy = px-x, py-y
            bx, by = c*dx + sn*dy, -sn*dx + c*dy
            if 0.3 <= bx <= 1.8:
                body.append((bx, by + (rng.gauss(0, noise_sd) if noise_sd else 0.0)))
        body = body[::5]
        road = {"valid": True, "time_aligned": True, "frame_id": "base_link", "timestamp_s": k*dt,
                "measurement_timestamp_s": k*dt-0.05, "source_age_s": 0.05, "centerline_xy": body}
        state = {"timestamp_s": k*dt, "speed_valid": True, "yaw_rate_valid": True}
        traj = from_road(road, state, cfg)
        out = controller.step(traj, v, 0.0 if k == 0 else dt, k*dt, k == 0)
        pending.append(out["steer"])
        delta = true_gain*pending.pop(0)
        beta = math.atan(L_R/L*math.tan(delta))
        # true cross-track error: signed distance to the nearest world path point (left positive)
        near = min(path, key=lambda q: (q[0]-x)**2 + (q[1]-y)**2)
        cross = -(x-near[0])*math.sin(near[2]) + (y-near[1])*math.cos(near[2])
        log.append((cross, out["steer"], out["debug"]["branch"], v))
        x += v*math.cos(psi+beta)*dt
        y += v*math.sin(psi+beta)*dt
        psi += v/L*math.tan(delta)*math.cos(beta)*dt
        v += dt/tau*(drive_gain*out["drive"] - v)
    return log


class BendClosedLoop(unittest.TestCase):
    """Bypass reference on arcs (R = 2.5 m) with the nonlinear plant; same pre-fixed E_LIMIT."""

    def controller(self):
        return Controller(verified(reference_source="estimation_centerline", bypass_acknowledged=True,
                                   bypass_target_speed_mps=0.3))

    def check(self, log, label):
        self.assertTrue(all(row[2] in ("solved", "low_speed") for row in log),
                        f"{label}: {sorted({row[2] for row in log})}")
        cross = [row[0] for row in log]
        tail = cross[-len(cross)//3:]
        rmse = math.sqrt(sum(v*v for v in tail)/len(tail))
        self.assertLessEqual(max(abs(v) for v in cross), E_LIMIT, label)
        self.assertLessEqual(rmse, E_LIMIT/2, label)
        steer = [row[1] for row in log]
        self.assertTrue(all(abs(v) <= 1.0 for v in steer), label)
        self.assertLessEqual(max(abs(b-a) for a, b in zip(steer, steer[1:]))*0.3, 0.1+1e-6, label)
        return max(abs(v) for v in cross), rmse

    def test_left_right_and_s_bends(self):
        scenarios = {"left": [(1.0, 0.0), (6.0, 0.4)], "right": [(1.0, 0.0), (6.0, -0.4)],
                     "s_bend": [(1.0, 0.0), (2.5, 0.4), (2.5, -0.4), (1.0, 0.0)]}
        for name, segments in scenarios.items():
            for y0 in (0.0, 0.1, -0.1):
                log = simulate_curve(self.controller(), world_path(segments), y0=y0)
                peak, rmse = self.check(log, f"{name} y0={y0}")
                print(f"\n{name} y0={y0:+.1f}: max |e_y| {peak:.3f} m, tail RMSE {rmse:.3f} m", end="")

    def test_bends_with_gain_mismatch_delay_and_centerline_noise(self):
        path = world_path([(1.0, 0.0), (6.0, 0.4)])
        for gain, delay in ((0.21, 1), (0.39, 1), (0.3, 2)):
            log = simulate_curve(self.controller(), path, y0=0.05, true_gain=gain, delay_steps=delay)
            self.check(log, f"gain={gain} delay={delay}")
        for seed in (1, 2, 3):
            log = simulate_curve(self.controller(), path, y0=0.05, noise_sd=0.01, seed=seed)
            self.check(log, f"noise seed {seed}")


class ConfigChecks(unittest.TestCase):
    def load(self, name):
        import yaml
        text = (Path(__file__).resolve().parents[1] / "config" / name).read_text(encoding="utf-8")
        return yaml.safe_load(text)["/**/ai4r_policy"]["ros__parameters"]

    def test_shipped_yaml_mpc_section_matches_declared_defaults_and_is_off(self):
        section = self.load("ai4r_policy.yaml")["mpc"]
        self.assertEqual(section, vars(Settings()))
        self.assertFalse(section["enabled"])
        self.assertTrue(section["shadow"])
        self.assertFalse(section["mapping_verified"] or section["limits_verified"])
        self.assertEqual(section["v_exec_max_mps"], 0.0)

    def test_bypass_overlay_is_shadow_and_unacknowledged(self):
        overlay = self.load("ai4r_policy_mpc_bypass.yaml")
        self.assertEqual(overlay["mpc"], {"enabled": True, "shadow": True,
                                          "reference_source": "estimation_centerline"})
        merged = dict(self.load("ai4r_policy.yaml")["mpc"], **overlay["mpc"])
        self.assertFalse(merged["bypass_acknowledged"])
        Settings(**merged)

    def test_prototype_overlay_enables_shadow_only(self):
        overlay = self.load("ai4r_policy_mpc_prototype.yaml")
        self.assertEqual(overlay["mpc"], {"enabled": True, "shadow": True})
        self.assertEqual(overlay["planning"], {"vehicle_limits_source": "course_simulation"})
        self.assertEqual(set(overlay["required_sensors"]),
                         {"cone_detections", "wheel_speed", "imu_angular_velocity"})
        merged = dict(self.load("ai4r_policy.yaml")["mpc"], **overlay["mpc"])
        Settings(**merged)


class NodeEntry(unittest.TestCase):
    """The real calculate_policy_actions, with a stand-in node (no ROS)."""

    def make_node(self, **mpc):
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
        return node, (obs, ages, stamps, receipts), published

    def test_disabled_mpc_keeps_zero_actions_and_publishes_nothing(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(enabled=False)
        result = namespace["calculate_policy_actions"](node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual(result, (0.0, 0.0, None, None, None))
        self.assertEqual(published, [])

    def test_shadow_entry_returns_zero_and_logs_candidate(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(enabled=True, shadow=True)
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
            enabled=True, shadow=True, reference_source="estimation_centerline")
        method = namespace["calculate_policy_actions"]
        drive, steer, pan, debug1, debug2 = method(node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual((drive, steer), (0.0, 0.0))
        record = json.loads(published[-1])
        self.assertEqual(record["reference_source"], "estimation_centerline")
        self.assertTrue(record["planning_bypassed"])
        self.assertIn(record["branch"], ("shadow", "ref_invalid"))
        if record["branch"] == "ref_invalid":
            self.fail(record["reject_reason"])

    def test_execution_refused_and_logged_when_unverified(self):
        node, (obs, ages, stamps, receipts), published = self.make_node(enabled=True, shadow=False)
        with self.assertRaises(Stop):
            namespace["calculate_policy_actions"](node, obs, ages, stamps, receipts["wheel_speed"], 0.0, True)
        self.assertEqual(json.loads(published[-1])["branch"], "gate_refused")


if __name__ == "__main__":
    unittest.main()
