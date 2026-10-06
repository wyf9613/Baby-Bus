"""Minimal end-to-end flow test: Dream Gym camera cones -> the real policy pipeline.

Runs scripts/policy_node.py's own calculate_policy_actions (estimation -> Planning
-> MPC) and _store (wheel/gyro motion history) with a given policy YAML, closing
the loop through the Dream Gym car. The timeline follows the car: wheel speed and
gyro at 20 Hz, cone batches at 10 Hz with 50 ms acquisition latency (positions
from the previous step), one policy step per cone batch (cone_detection trigger).
Any MPCStop in execution mode is a locked stop on the car and ends the run here.

  python offline/mpc_gym/pipeline_sim.py .verification/car-trial/mpc_exec.yaml
  python offline/mpc_gym/pipeline_sim.py <yaml> --noise 0.02 --offset 0.15 --speed 0.2
"""
import argparse
import ast
from collections import Counter
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from numbers import Real
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
         "MPCController", "mpc_debug_json"}
METHODS = {"calculate_policy_actions", "_store"}
GYM_TO_ROS_COLOUR = {1: 2, 0: 1}        # Gym blue 1 / yellow 0 -> ConeDetection BLUE 2 / YELLOW 1
ENV_DT, CONE_PERIOD_STEPS, CONE_LATENCY_S = 0.05, 2, 0.05


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


def make_env(offset, heading, noise, seed):
    spec = tune.road_spec("straight")
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
                "mpc": api["MPCSettings"], "vehicle": api["VehicleParamsSettings"]}
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
        _warn=lambda key, text: None)
    node.mpc_debug_publisher = SimpleNamespace(publish=lambda msg: node.debug.append(json.loads(msg.data)))
    for method in METHODS:
        setattr(node, method, api[method].__get__(node))
    return node


def run(params, offset=0.0, heading=0.0, noise=0.0, seed=1, duration=20.0, start_time=100.0):
    api = load_policy()
    env = make_env(offset, heading, noise, seed)
    observation, _ = env.reset(seed=seed)
    node = make_node(api, params)
    mpc_on = node.mpc_settings.enabled
    action, previous_observation = (0.0, 0.0), observation
    result = {"locked_stop": None, "steps": 0, "terminated": None}
    last_policy_t, first = None, True
    try:
        for step in range(int(duration/ENV_DT)):
            t = start_time + step*ENV_DT
            node.clock.now_s = t
            state = env.unwrapped.car.state
            motion = state["body_motion"]
            ros_ns = round(t*1e9)
            node._store("wheel_speed", abs(float(motion["longitudinal_velocity_mps"])), None, t, ros_ns)
            node._store("imu_angular_velocity", (0.0, 0.0, float(motion["yaw_rate_rad_per_s"])),
                        round((t-0.005)*1e9), t, ros_ns)
            if step % CONE_PERIOD_STEPS == 0 and step > 0:
                node._store("cone_detections", cone_batch(previous_observation),
                            round((t-CONE_LATENCY_S)*1e9), t, ros_ns)
                values = {k: (None if o is None else o.value) for k, o in node.observations.items()}
                ages = {k: (None if o is None else max(t-o.received_at, 0.0 if o.stamp_ns is None
                                                       else (ros_ns-o.stamp_ns)/1e9))
                        for k, o in node.observations.items()}
                stamps = {k: (None if o is None else o.stamp_ns) for k, o in node.observations.items()}
                dt = 0.0 if first else t-last_policy_t
                try:
                    out = node.calculate_policy_actions(values, ages, stamps, dt, 0.0 if first else t-start_time,
                                                        first)
                    action = (float(out[0]), float(out[1]))
                except api["MPCStop"] as stop:
                    result["locked_stop"] = {"time_s": round(t-start_time, 2), "reason": str(stop),
                                             "planning": (node.planning_output or {}).get("reason"),
                                             "road": (node.estimation_output or {}).get("road", {}).get("status")}
                    break
                last_policy_t, first = t, False
                result["steps"] += 1
            previous_observation = observation
            observation, _, terminated, truncated, _ = env.step(np.array(action, dtype=np.float32))
            if terminated or truncated:
                result["terminated"] = "terminated" if terminated else "truncated"
                break
    finally:
        env.close()
    state = env.unwrapped.car.state
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
    args = parser.parse_args()
    params = yaml.safe_load(args.yaml.read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
    result = run(params, args.offset, args.heading, args.noise, args.seed, args.duration)
    print(json.dumps(result, indent=2))
    sys.exit(1 if result["locked_stop"] else 0)


if __name__ == "__main__":
    main()
