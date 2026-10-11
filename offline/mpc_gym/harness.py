"""Run the policy_node MPC in the PID baseline's exact Dream Gym experiment.

The loop mirrors offline/control_pid/tune.py simulate() line for line: same
road specs, plant (BicycleModelDynamic with the scenario's mismatch), oracle
cubic reference with the same noise/dropout random stream, command delay,
footprint check, completion rule and metrics. Only the controller is swapped
through an adapter, so PID and MPC rows are directly comparable.
test_mpc_gym.py checks that the PID adapter reproduces tune.simulate exactly.
"""
import ast
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
import statistics
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "offline" / "control_pid"))
import tune  # noqa: E402  (PID baseline experiment, unchanged)
from controller import Controller, Gains  # noqa: E402
from dreamgym.envs import Road  # noqa: E402

SOURCE = ROOT / "scripts" / "policy_node.py"
MPC_NAMES = {"finite_number", "wrap_angle", "MPCStop", "VehicleParamsSettings",
             "course_simulation_vehicle_params", "course_model_step", "MPCSettings",
             "mpc_vehicle_params", "mpc_reference_errors", "mpc_path_projector",
             "VehicleModelMPC", "MPCController"}
TRAJECTORY_LIFETIME_S = 0.15     # the PID consumer's maximum reference age
SAMPLE_SPACING_M = 0.05


def load_mpc():
    """MPC definitions from the single-file policy (no ROS import), as tests/test_mpc.py does."""
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    definitions = [item for item in tree.body if isinstance(item, (ast.FunctionDef, ast.ClassDef))
                   and item.name in MPC_NAMES]
    missing = MPC_NAMES - {item.name for item in definitions}
    if missing:
        raise RuntimeError(f"policy_node.py lacks {sorted(missing)}")
    namespace = {"dataclass": dataclass, "math": math, "time": time, "Real": Real}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace


def notebook_vehicle(api, drive_min=-0.3, drive_max=0.35, delay_s=0.0):
    """The notebook plant as a vehicle record: SIMULATOR parameters, not a car measurement.

    Drive limits default to the PID baseline's Gains so both controllers share the
    same actuator range (the MPC itself never commands below zero).
    """
    config = tune.CONFIG["bicycle_model_config"]
    axle, steering = config["axle_geometry"], config["steering"]
    body, force = config["body_geometry"], config["longitudinal_force_model"]
    limit = steering["requested_front_wheel_angle_magnitude_limit_rad"]
    rates = steering["physical_front_wheel_angle_rate_bounds_rad_per_s"]
    rear = axle["rear_axle_distance_from_center_of_mass_m"]
    return api["VehicleParamsSettings"](
        valid=True, source="sim:dreamgym-0.4.0-notebook",
        wheelbase_m=axle["front_axle_distance_from_center_of_mass_m"] + rear, rear_axle_from_cg_m=rear,
        body_front_extent_from_cg_m=body["front_extent_from_center_of_mass_m"],
        body_rear_extent_from_cg_m=body["rear_extent_from_center_of_mass_m"], body_width_m=body["width_m"],
        steering_gain_rad=limit,
        steering_offset_rad=steering["physical_front_wheel_angle_offset_from_request_rad"],
        steering_min_rad=-limit, steering_max_rad=limit,
        steering_rate_limit_rad_s=min(-rates["lower"], rates["upper"]),
        drive_min=drive_min, drive_max=drive_max,
        drive_response_model="Dream Gym 0.4.0 notebook classic force model",
        braking_behavior="Dream Gym direction-change latch; the MPC never commands negative drive",
        steering_delay_s=delay_s, drive_delay_s=delay_s,
        mass_kg=config["mass_properties"]["mass_kg"],
        motor_gain_n=force["normalized_motor_command_force_gain_n"],
        drag_kg_per_m=force["quadratic_aerodynamic_drag_coefficient_kg_per_m"],
        speed_max_mps=1.5, braking_deceleration_mps2=0.5, safety_margin_m=0.05)


def poly_to_trajectory(reference, pad_m):
    """PID mock reference (cubic y(x) in base_link) -> ReferenceTrajectory v0.1 point list.

    Samples the polynomial on its x range every 5 cm (yaw = atan y', curvature
    y''/(1+y'^2)^1.5) and continues the end tangent straight for pad_m, so the MPC
    horizon stays covered where the road (and the mock planner's range) ends.
    """
    base = {"schema": "reference_trajectory_v0.1", "frame_id": "base_link",
            "timestamp_s": reference.get("timestamp_s"), "valid_for_s": TRAJECTORY_LIFETIME_S,
            "stop_required": False, "status": "TRACK", "reason": None, "path_id": None,
            "simulation_only": True, "source_ages_s": {}, "points": []}
    if not reference.get("valid"):
        return dict(base, valid=False, stop_required=True, reason="mock_reference_invalid")
    poly = np.polynomial.Polynomial(reference["path_coeffs"])
    first, second = poly.deriv(), poly.deriv(2)
    lo, hi = reference["x_range_m"]
    xs = np.append(np.arange(lo, hi, SAMPLE_SPACING_M), hi)
    if xs[-1] - xs[-2] < 1e-3:
        xs = xs[:-1]
    points, s = [], 0.0
    speed = float(reference["target_speed_mps"])
    for x in xs:
        y, dy, ddy = float(poly(x)), float(first(x)), float(second(x))
        if points:
            s += math.hypot(x - points[-1]["x_m"], y - points[-1]["y_m"])
        points.append({"s_m": s, "x_m": float(x), "y_m": y, "yaw_rad": math.atan(dy),
                       "curvature_1pm": ddy / (1 + dy*dy)**1.5, "target_speed_mps": speed})
    end = points[-1]
    c, sn = math.cos(end["yaw_rad"]), math.sin(end["yaw_rad"])
    for i in range(1, int(math.ceil(pad_m / SAMPLE_SPACING_M)) + 1):
        d = i*SAMPLE_SPACING_M
        points.append({"s_m": end["s_m"] + d, "x_m": end["x_m"] + c*d, "y_m": end["y_m"] + sn*d,
                       "yaw_rad": end["yaw_rad"], "curvature_1pm": 0.0, "target_speed_mps": speed})
    return dict(base, valid=True, points=points)


class PIDAdapter:
    """The PID baseline exactly as tune.simulate uses it."""

    name = "pid"

    def __init__(self, gains: Gains):
        self.gains = gains
        self.steering_limit_rad = gains.steering_limit_rad
        self.drive_min, self.drive_max = gains.drive_min, gains.drive_max

    def reset(self, scenario, longitudinal):
        self.controller = Controller(self.gains)

    def calculate(self, reference, speed, now, dt):
        return self.controller.calculate(reference, speed, now, dt)

    def summary(self):
        return {}


class MPCAdapter:
    """policy_node MPCController in execution mode (shadow False) on the Gym plant.

    A rejection raises MPCStop on the car (locked stop, explicit state-3 restart).
    Offline, like the PID's invalid-reference reset, this step sends zero and the
    next step rebuilds the controller; every such stop is counted and reported.
    """

    name = "mpc"

    def __init__(self, api, weights=None, horizon_n=10, delay_s=0.0, label=None):
        self.api, self.weights, self.horizon_n = api, dict(weights or {}), horizon_n
        self.vehicle = notebook_vehicle(api, delay_s=delay_s)
        self.steering_limit_rad = self.vehicle.steering_gain_rad
        self.drive_min, self.drive_max = max(0.0, self.vehicle.drive_min), self.vehicle.drive_max
        if label:
            self.name = label

    def reset(self, scenario, longitudinal):
        cap = 1.0 if longitudinal else scenario.speed
        self.settings = self.api["MPCSettings"](enabled=True, shadow=False, v_exec_max_mps=cap,
                                                horizon_n=self.horizon_n, **self.weights)
        self.pad_m = (cap + self.settings.overspeed_margin_mps)*self.horizon_n*self.settings.dt_pred_s + 0.5
        self.controller = None
        self.solve_s, self.step_s, self.stops, self.branches = [], [], [], {}

    def calculate(self, reference, speed, now, dt):
        trajectory = poly_to_trajectory(reference, self.pad_m)
        first = self.controller is None
        if first:
            self.controller = self.api["MPCController"](self.settings, self.vehicle)
        try:
            out = self.controller.step(trajectory, speed, 0.0 if first else dt, now, first)
        except self.api["MPCStop"] as stop:
            self.stops.append((now, str(stop)))
            debug = self.controller.last_debug
            self.branches[debug.get("branch")] = self.branches.get(debug.get("branch"), 0) + 1
            self.controller = None
            return 0.0, 0.0, {"valid": False, "reason": str(stop)}
        debug = out["debug"]
        self.branches[debug["branch"]] = self.branches.get(debug["branch"], 0) + 1
        self.step_s.append(debug["step_s"])
        if debug["solve_s"] is not None:
            self.solve_s.append(debug["solve_s"])
        return out["drive"], out["steer"], {"valid": True}

    def summary(self):
        steps = sorted(self.step_s) or [0.0]
        p95 = steps[max(0, int(math.ceil(0.95*len(steps)))-1)]
        return {"mpc_step_mean_ms": 1e3*statistics.mean(steps), "mpc_step_p95_ms": 1e3*p95,
                "mpc_step_max_ms": 1e3*steps[-1], "mpc_over_budget": sum(t > 0.05 for t in steps),
                "mpc_stops": len(self.stops),
                "mpc_stop_reasons": ";".join(sorted({reason for _, reason in self.stops})),
                "mpc_branches": ";".join(f"{k}:{v}" for k, v in sorted(self.branches.items(), key=str))}


def run(adapter, scenario, seed=0, longitudinal=False):
    """tune.simulate with the controller supplied by `adapter` (same order of every random draw)."""
    spec = tune.road_spec('straight' if longitudinal else scenario.road)
    length = tune.road_length(spec)
    road = Road(road_spec=deepcopy(spec))
    model = tune.vehicle(scenario)
    adapter.reset(scenario, longitudinal)
    rng = np.random.default_rng(seed)
    queue = deque([(0.0, 0.0)] * scenario.delay_steps)
    records = []
    duration = 28.0 if longitudinal else length / scenario.speed + 8.0
    complete, failure = False, ''
    for step in range(math.ceil(duration / scenario.dt)):
        now = step * scenario.dt
        state = model.state
        pose, motion = state['world_pose'], state['body_motion']
        closest = road.find_closest_point_to(pose['x_m'], pose['y_m'])
        px, py, _, _, progress, tangent, _ = closest
        ey = -(pose['x_m'] - px) * math.sin(tangent) + (pose['y_m'] - py) * math.cos(tangent)
        heading_error = math.atan2(math.sin(pose['heading_rad'] - tangent),
                                   math.cos(pose['heading_rad'] - tangent))
        speed = motion['longitudinal_velocity_mps']
        target = tune.speed_schedule(now) if longitudinal else scenario.speed
        ref = tune.reference_for(road, state, progress, length, now, target, rng, scenario)
        measurement = max(0, speed + rng.normal(0, scenario.speed_noise_mps))
        drive, steer, diagnostic = adapter.calculate(ref, measurement, now, 0.0 if step == 0 else scenario.dt)
        queue.append((drive, steer))
        applied_drive, applied_steer = queue.popleft()
        body = tune.CONFIG['bicycle_model_config']['body_geometry']
        half_width = (0.5 * body['width_m'] * abs(math.cos(heading_error))
                      + max(body['front_extent_from_center_of_mass_m'],
                            body['rear_extent_from_center_of_mass_m']) * abs(math.sin(heading_error)))
        clearance = 0.5 - abs(ey) - half_width
        records.append({'time_s': now, 'x_m': pose['x_m'], 'y_m': pose['y_m'],
                        'progress_m': progress, 'lateral_error_m': ey,
                        'heading_error_rad': heading_error, 'speed_mps': speed,
                        'target_speed_mps': target, 'drive': drive, 'steer': steer,
                        'clearance_m': clearance, 'reference_valid': diagnostic['valid']})
        if not all(math.isfinite(v) for v in [ey, speed, drive, steer]):
            failure = 'nonfinite'
            break
        if clearance < 0:
            failure = 'footprint_outside_lane'
            break
        if not longitudinal and progress >= length - 0.3:
            complete = True
            break
        model.set_action_request(applied_drive,
                                 applied_steer * adapter.steering_limit_rad * scenario.steering_scale)
        model.integrate(scenario.dt, 'rk4', update_stored_state=True)
    if longitudinal:
        complete = not failure
    if not complete and not failure:
        failure = 'timeout'
    row = metrics(records, scenario, seed, complete, failure, longitudinal, adapter)
    row.update(adapter.summary())
    return row, records


def metrics(records, scenario, seed, complete, failure, longitudinal, adapter):
    """tune.simulate's row, with the saturation bounds of the controller under test."""
    errors = np.array([r['lateral_error_m'] for r in records])
    velocity_errors = np.array([r['target_speed_mps'] - r['speed_mps'] for r in records])
    drives = np.array([r['drive'] for r in records])
    steers = np.array([r['steer'] for r in records])
    times = np.array([r['time_s'] for r in records])
    speeds = np.array([r['speed_mps'] for r in records])
    late = (times >= 3) if not longitudinal else np.logical_or.reduce([
        (times >= a) & (times < b) for a, b in [(3, 5), (9, 11), (15, 17), (20, 22), (26, 28)]])
    stopping = (times >= 17) & (times < 22)
    stopped = np.flatnonzero(stopping & (speeds <= 0.03))
    return {'controller': adapter.name, 'scenario': scenario.name, 'seed': seed, 'complete': complete,
            'failure': failure, 'duration_s': records[-1]['time_s'],
            'progress_m': records[-1]['progress_m'],
            'lateral_rmse_m': float(np.sqrt(np.mean(errors ** 2))),
            'lateral_max_m': float(np.max(np.abs(errors))),
            'speed_rmse_mps': float(np.sqrt(np.mean(velocity_errors ** 2))),
            'steady_speed_rmse_mps': float(np.sqrt(np.mean(velocity_errors[late] ** 2))) if late.any() else 10.0,
            'min_clearance_m': min(r['clearance_m'] for r in records),
            'steer_rate_rms_per_s': float(np.sqrt(np.mean(np.diff(steers) ** 2)) / scenario.dt),
            'drive_rate_rms_per_s': float(np.sqrt(np.mean(np.diff(drives) ** 2)) / scenario.dt),
            'drive_saturation_fraction': float(np.mean((drives <= adapter.drive_min + 1e-6)
                                                       | (drives >= adapter.drive_max - 1e-6))),
            'steer_saturation_fraction': float(np.mean(np.abs(steers) >= 0.999)),
            'stop_time_s': float(times[stopped[0]] - 17) if stopped.size else None,
            'stop_distance_m': (records[stopped[0]]['progress_m'] - records[int(round(17/scenario.dt))]['progress_m'])
                               if stopped.size else None}
