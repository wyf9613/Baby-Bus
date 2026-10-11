"""Minimal end-to-end flow test: Dream Gym camera cones -> the real policy pipeline.

Runs scripts/policy_node.py's own calculate_policy_actions (estimation -> Planning
-> MPC) and _store (wheel/gyro motion history) with a given policy YAML, closing
the loop through the Dream Gym car. The timeline follows the car: wheel speed and
gyro at 20 Hz, cone batches at 10 Hz with 50 ms acquisition latency (positions
from the previous step), one policy step per cone batch (cone_detection trigger).
Any MPCStop in execution mode is a locked stop on the car and ends the run here.

Car conditions (2026-10-06, car .27), all optional so the defaults keep the run above:
- trigger="timer": one policy step per 50 ms environment step (the car's 20 Hz timer);
- cone_latency/latency_jitter/gap_probability: acquisition-to-publish delay of the
  camera (~0.2 s on the car) and dropped batches (the car showed 331 ms gaps);
- plant=NewcarPlant(...): the identified longitudinal response (deadband, ESC drag
  brake, static friction, delay) and steering sign/offset of the car, imposed on the
  Gym car by inverting its own force law, so cones/geometry stay the Gym's;
- the MVP controller (control.enabled) runs through the same loop for comparison.

  python offline/mpc_gym/pipeline_sim.py .verification/car-trial/mpc_exec.yaml
  python offline/mpc_gym/pipeline_sim.py <yaml> --noise 0.02 --offset 0.15 --speed 0.2
  python offline/mpc_gym/pipeline_sim.py <yaml> --car newcar27 --distance 3.0
"""
import argparse
import ast
from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from numbers import Real
import random
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "offline" / "control_pid"))
import tune  # noqa: E402  (notebook configuration, unchanged)
import gymnasium as gym  # noqa: E402
import dreamgym  # noqa: E402,F401  (registers the environment)

SOURCE = ROOT / "scripts" / "policy_node.py"
NAMES = {"finite_number", "wrap_angle", "Observation", "EstimationSettings", "FirstOrderSampleFilter",
         "EstimationMotionHistory", "_estimation_median", "_estimation_solve", "_estimation_fit_boundary",
         "EstimationPipeline", "PlanningSettings", "CenterlinePlanner", "evaluate_planning_path",
         "planning_vehicle_limits", "MPCStop", "VehicleParamsSettings", "course_simulation_vehicle_params",
         "course_model_step", "MPCSettings", "mpc_vehicle_params", "reference_trajectory_from_planning",
         "reference_trajectory_from_road", "mpc_reference_errors", "mpc_path_projector", "VehicleModelMPC",
         "MPCController", "mpc_debug_json", "ControlSettings", "PolicyController", "ControlPID",
         "PolicyStopRequest", "control_path_geometry", "_control_poly_eval", "_control_poly_roots"}
METHODS = {"calculate_policy_actions", "_store"}
GYM_TO_ROS_COLOUR = {1: 2, 0: 1}        # Gym blue 1 / yellow 0 -> ConeDetection BLUE 2 / YELLOW 1
ENV_DT, CONE_PERIOD_STEPS, CONE_LATENCY_S = 0.05, 2, 0.05
FIT = ROOT / "offline" / "vehicle_identification" / "results" / "newcar27_drive_fit.json"
# Gym (course notebook) longitudinal law, inverted by NewcarPlant.
GYM_MASS_KG, GYM_MOTOR_N, GYM_DRAG = 3.0, 10.0, 1.0
GYM_STEER_RAD = math.pi/4
# Timing measured on the car: camera acquisition-to-publish ~0.19-0.22 s, 20 Hz timer,
# occasional ~0.3 s gaps between cone batches.
# The 2026-10-06 course: boundary cones at y ~ +0.33/-0.38 m, ~0.2 m apart.
NEWCAR27_CONDITIONS = dict(trigger="timer", cone_latency=0.19, latency_jitter=0.03, gap_probability=0.05,
                           lane_width_m=0.72, cone_spacing_m=0.2)


class NewcarPlant:
    """The identified car .27 between the policy request and the Gym car.

    Longitudinal: v' = (a if u >= d0 else e)*(u(t - tau) - d0) - c*v|v|, wheels held at
    rest until u >= u_break has held for t_break (fit_drive_from_logs.py). The Gym drive
    that produces this acceleration is found by inverting the Gym's own force law.
    Steering: the car turned RIGHT for a positive request, so the Gym (left positive)
    receives the negated request plus a wheel-angle offset (rad).
    """

    def __init__(self, a, d0, e, c, tau, u_break, t_break, steering_sign=-1.0, steering_offset_rad=0.0):
        self.a, self.d0, self.e, self.c, self.tau = a, d0, e, c, tau
        self.u_break, self.t_break = u_break, t_break
        self.steering_sign, self.steering_offset_rad = steering_sign, steering_offset_rad
        self.queue = deque()
        self.held_for, self.stuck = 0.0, True

    @classmethod
    def from_fit(cls, path=FIT, **changes):
        fit = json.loads(Path(path).read_text(encoding="utf-8"))
        p = fit["params"]
        values = dict(a=p["a"], d0=p["d0"], e=p["e"], c=p["c"], tau=p["tau"],
                      u_break=fit["breakaway"]["u_break"], t_break=p["t_break"])
        values.update(changes)
        return cls(**values)

    def gym_action(self, t, request, speed):
        drive, steer = request
        self.queue.append((t, drive))
        delayed = 0.0
        while self.queue and self.queue[0][0] <= t - self.tau + 1e-9:
            delayed = self.queue[0][1]
            if len(self.queue) > 1 and self.queue[1][0] <= t - self.tau + 1e-9:
                self.queue.popleft()
            else:
                break
        if self.stuck:
            self.held_for = self.held_for + ENV_DT if delayed >= self.u_break else 0.0
            self.stuck = self.held_for < self.t_break
        if self.stuck:
            gym_drive = 0.0
        else:
            accel = (self.a if delayed >= self.d0 else self.e)*(delayed - self.d0) - self.c*speed*abs(speed)
            target = speed + accel*ENV_DT
            if target <= 0.0:
                target = 0.0
                if delayed < self.u_break:
                    self.stuck, self.held_for = True, 0.0
            gym_drive = (GYM_MASS_KG*(target - speed)/ENV_DT + GYM_DRAG*speed*abs(speed))/GYM_MOTOR_N
            if speed < 0.03:
                gym_drive = max(0.0, gym_drive)       # never enter the Gym's reverse latch
        gym_steer = self.steering_sign*steer + self.steering_offset_rad/GYM_STEER_RAD
        return (min(1.0, max(-1.0, gym_drive)), min(1.0, max(-1.0, gym_steer)))


def load_policy():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in NAMES]
    missing = NAMES - {n.name for n in nodes}
    if missing:
        raise RuntimeError(f"policy_node.py lacks {sorted(missing)}")
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PolicyNode")
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    namespace = {"dataclass": dataclass, "math": math, "time": time, "Real": Real, "json": json,
                 "deepcopy": deepcopy, "String": lambda data: SimpleNamespace(data=data),
                 "ConeDetection": SimpleNamespace(COLOR_BLUE=2, COLOR_YELLOW=1)}
    exec(compile(ast.Module(body=nodes + methods, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace


def make_env(offset, heading, noise, seed, lane_width_m=None, cone_spacing_m=None):
    spec = tune.road_spec("straight")
    if lane_width_m is not None:
        spec["elements"][0]["lanes"]["reference_lane"]["width_m"] = {"start": lane_width_m, "end": lane_width_m}
    if cone_spacing_m is not None:
        for path in spec["cone_paths"].values():
            path["profiles"]["regular"]["inter_cone_spacing"] = {"distribution": "fixed", "value_m": cone_spacing_m}
    observation = {"cone_detections": {
        "maximum_detection_count": 24,
        "acquisition": {"horizontal_field_of_view_deg": 80.0, "forward_position_upper_bound_m": 4.0,
                        "position_error_in_camera_frame": {
                            "forward": {"noise_standard_deviation_m": noise},
                            "left": {"noise_standard_deviation_m": noise}}},
        "positions_in_body_frame_m": {"destination": "observation", "scaling_factor": 1.0},
        "color_ids": {"destination": "observation"}, "count": {"destination": "observation"}},
        "fixed_noise_seed": seed}
    initial = {"pose": {"mode": "road_relative", "anchor": "reference_line",
                        "progress": {"unit": "metres", "lower": 0.0, "upper": 0.0},
                        "lateral_offset_m": {"lower": offset, "upper": offset},
                        "heading_error_rad": {"lower": heading, "upper": heading}},
               "body_motion": {"longitudinal_velocity": {"source": "absolute",
                                                         "value_mps": {"lower": 0.0, "upper": 0.0}}}}
    return gym.make("dreamgym/autonomous_driving_env", road_spec=spec,
                    bicycle_model_config=tune.CONFIG["bicycle_model_config"], observation_config=observation,
                    initial_state_config=initial, integration_config=tune.CONFIG["integration_config"],
                    termination_config={"planar_speed_bounds_mps": {"lower": 0.0, "upper": 10.0},
                                        "absolute_lateral_error_from_target_line_upper_bound_m": 2.0},
                    observation_format="dict", render_mode=None)


def cone_batch(observation):
    n = int(observation["cone_detections_count"][0])
    xs = observation["cone_detections_forward_positions_in_body_frame_m"][:n]
    ys = observation["cone_detections_left_positions_in_body_frame_m"][:n]
    colours = observation["cone_detections_color_ids"][:n]
    return {"detections": [(float(x), float(y), 0.0, GYM_TO_ROS_COLOUR.get(int(c), 0), 0.95)
                           for x, y, c in zip(xs, ys, colours)],
            "acquisition_to_publish_latency_s": CONE_LATENCY_S}


def make_node(api, params):
    sections = {"estimation": api["EstimationSettings"], "planning": api["PlanningSettings"],
                "mpc": api["MPCSettings"], "vehicle": api["VehicleParamsSettings"],
                "control": api["ControlSettings"]}
    settings = {name: cls(**params.get(name, {})) for name, cls in sections.items()}
    sensors = ("cone_detections", "fiducial_detections", "lidar_scan", "lidar_cartesian", "wheel_speed",
               "imu_orientation", "imu_angular_velocity", "imu_specific_force")
    clock = SimpleNamespace(now_s=0.0)
    node = SimpleNamespace(
        estimation_settings=settings["estimation"], planning_settings=settings["planning"],
        mpc_settings=settings["mpc"], vehicle_settings=settings["vehicle"], mpc_controller=None,
        policy_frame_id=params.get("policy_frame_id", "base_link"), heading_reference=None,
        sensor_timeout_s={name: 0.5 for name in sensors}, timestamp_tolerance_s=0.05,
        observations={name: None for name in sensors}, debug=[],
        motion_history=api["EstimationMotionHistory"](settings["estimation"]), clock=clock,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(clock.now_s*1e9))),
        _warn=lambda key, text: None, control_settings=settings["control"],
        controller=api["PolicyController"](settings["control"]), control_has_run=False,
        control_diagnostics=None, last_control_reference_deadline_s=None,
        distance_limiter=SimpleNamespace(distance_m=0.0),
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None))
    node.mpc_debug_publisher = SimpleNamespace(publish=lambda msg: node.debug.append(json.loads(msg.data)))
    for method in METHODS:
        setattr(node, method, api[method].__get__(node))
    return node


def run(params, offset=0.0, heading=0.0, noise=0.0, seed=1, duration=20.0, start_time=100.0,
        trigger="cone", cone_latency=CONE_LATENCY_S, latency_jitter=0.0, gap_probability=0.0,
        plant=None, distance_m=None, lane_width_m=None, cone_spacing_m=None):
    """One closed-loop run. Defaults reproduce the original flow test (cone trigger,
    50 ms latency, course car); the keyword arguments add the car conditions above.
    distance_m ends the run as completed once the wheel-integrated distance reaches it."""
    api = load_policy()
    env = make_env(offset, heading, noise, seed, lane_width_m, cone_spacing_m)
    observation, _ = env.reset(seed=seed)
    node = make_node(api, params)
    mpc_on = node.mpc_settings.enabled
    stops = (api["MPCStop"], api["PolicyStopRequest"])
    rng = random.Random(seed*7919 + 1)
    request, history = (0.0, 0.0), deque(maxlen=int(1.0/ENV_DT)+2)
    result = {"locked_stop": None, "steps": 0, "terminated": None, "completed": False}
    trace = {"t": [], "speed": [], "drive": [], "lateral": []}
    last_policy_t, first, skip, distance = None, True, 0, 0.0
    try:
        for step in range(int(duration/ENV_DT)):
            t = start_time + step*ENV_DT
            node.clock.now_s = t
            state = env.unwrapped.car.state
            motion = state["body_motion"]
            speed = abs(float(motion["longitudinal_velocity_mps"]))
            ros_ns = round(t*1e9)
            history.append(observation)
            node._store("wheel_speed", speed, None, t, ros_ns)
            node._store("imu_angular_velocity", (0.0, 0.0, float(motion["yaw_rate_rad_per_s"])),
                        round((t-0.005)*1e9), t, ros_ns)
            new_cones = False
            if step % CONE_PERIOD_STEPS == 0 and step > 0:
                if skip:
                    skip -= 1
                elif gap_probability and rng.random() < gap_probability:
                    skip = 1                     # this batch and the next: a ~0.3 s gap
                else:
                    latency = cone_latency + rng.uniform(0.0, latency_jitter)
                    back = min(len(history)-1, max(1, round(latency/ENV_DT)))
                    node._store("cone_detections", dict(cone_batch(history[-1-back]),
                                                        acquisition_to_publish_latency_s=latency),
                                round((t-back*ENV_DT)*1e9), t, ros_ns)
                    new_cones = True
            # As on the car, state 3 starts only once the required cone topic has data.
            if (trigger == "timer" and node.observations["cone_detections"] is not None) or new_cones:
                values = {k: (None if o is None else o.value) for k, o in node.observations.items()}
                ages = {k: (None if o is None else max(t-o.received_at, 0.0 if o.stamp_ns is None
                                                       else (ros_ns-o.stamp_ns)/1e9))
                        for k, o in node.observations.items()}
                stamps = {k: (None if o is None else o.stamp_ns) for k, o in node.observations.items()}
                dt = 0.0 if first else t-last_policy_t
                try:
                    out = node.calculate_policy_actions(values, ages, stamps, dt, 0.0 if first else t-start_time,
                                                        first)
                    request = (float(out[0]), float(out[1]))
                except stops as stop:
                    result["locked_stop"] = {"time_s": round(t-start_time, 2), "reason": str(stop),
                                             "planning": (node.planning_output or {}).get("reason"),
                                             "road": (node.estimation_output or {}).get("road", {}).get("status")}
                    break
                last_policy_t, first = t, False
                result["steps"] += 1
            distance += speed*ENV_DT
            node.distance_limiter.distance_m = distance
            pose = state["world_pose"]
            trace["t"].append(round(t-start_time, 3))
            trace["speed"].append(speed)
            trace["drive"].append(request[0])
            trace["lateral"].append(float(pose["y_m"]))
            if distance_m is not None and distance >= distance_m:
                result["completed"] = True
                break
            action = request if plant is None else plant.gym_action(t, request, speed)
            observation, _, terminated, truncated, _ = env.step(np.array(action, dtype=np.float32))
            if terminated or truncated:
                result["terminated"] = "terminated" if terminated else "truncated"
                break
    finally:
        env.close()
    state = env.unwrapped.car.state
    moving = [i for i, v in enumerate(trace["speed"]) if v > 0.05]
    tracking = trace["speed"][moving[0]+int(2.0/ENV_DT):] if moving else []
    target = node.planning_settings.cruise_speed_mps
    drives = trace["drive"]
    result.update(distance_m=round(distance, 3), peak_speed_mps=round(max(trace["speed"], default=0.0), 3),
                  start_delay_s=None if not moving else round(trace["t"][moving[0]], 2),
                  speed_rmse_mps=None if not tracking else round(
                      math.sqrt(sum((v-target)**2 for v in tracking)/len(tracking)), 3),
                  lateral_max_m=round(max((abs(y) for y in trace["lateral"]), default=0.0), 3),
                  lateral_rmse_m=round(math.sqrt(sum(y*y for y in trace["lateral"])/max(1, len(trace["lateral"]))), 3),
                  drive_jumps=sum(1 for a, b in zip(drives, drives[1:]) if abs(b-a) > 0.05),
                  trace=trace)
    branches = Counter(d["branch"] for d in node.debug)
    reasons = Counter(d["reject_reason"] for d in node.debug if d["reject_reason"])
    result.update(mpc_enabled=mpc_on, branches=dict(branches), reject_reasons=dict(reasons),
                  final_x_m=round(float(state["world_pose"]["x_m"]), 2),
                  final_y_m=round(float(state["world_pose"]["y_m"]), 3),
                  final_speed_mps=round(float(state["body_motion"]["longitudinal_velocity_mps"]), 3),
                  max_abs_e_y_m=round(max((abs(d["e_y_m"]) for d in node.debug if d["e_y_m"] is not None),
                                          default=float("nan")), 3),
                  step_p95_ms=round(1e3*float(np.percentile([d["step_s"] for d in node.debug if d["step_s"]]
                                                            or [0.0], 95)), 1))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("yaml", type=Path)
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--heading", type=float, default=0.0)
    parser.add_argument("--noise", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--car", choices=("course", "newcar27"), default="course",
                        help="newcar27: timer trigger, 0.2 s camera latency with jitter and gaps, fitted plant")
    parser.add_argument("--distance", type=float, default=None, help="end as completed at this distance (m)")
    args = parser.parse_args()
    params = yaml.safe_load(args.yaml.read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
    extra = {} if args.car == "course" else dict(NEWCAR27_CONDITIONS, plant=NewcarPlant.from_fit())
    result = run(params, args.offset, args.heading, args.noise, args.seed, args.duration,
                 distance_m=args.distance, **extra)
    result.pop("trace")
    print(json.dumps(result, indent=2))
    sys.exit(1 if result["locked_stop"] else 0)


if __name__ == "__main__":
    main()
