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
from collections import Counter, deque
from copy import deepcopy
import json
import math
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

sys.path.insert(0, str(ROOT / "tools"))
from policy_offline import bind_methods, load_policy_namespace, policy_settings  # noqa: E402

METHODS = ("calculate_policy_actions", "_mpc_jump_guard", "_store", "_robust_control_enabled", "_mpc_uses_planning",
           "_select_control_reference", "_report_control_status")
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
    turn_drag (default 0): d0 and u_break rise by turn_drag*|steering request in force|, the
    cornering drag car 05 showed on 2026-10-19 (tools/car05_run_stats.py: ~0.03 more drive at
    full lock), which the plain fit lacks.
    """

    def __init__(self, a, d0, e, c, tau, u_break, t_break, steering_sign=-1.0, steering_offset_rad=0.0,
                 turn_drag=0.0):
        self.a, self.d0, self.e, self.c, self.tau = a, d0, e, c, tau
        self.u_break, self.t_break, self.turn_drag = u_break, t_break, turn_drag
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
        drag = self.turn_drag*abs(steer)
        d0, u_break = self.d0 + drag, self.u_break + drag
        if self.stuck:
            self.held_for = self.held_for + ENV_DT if delayed >= u_break else 0.0
            self.stuck = self.held_for < self.t_break
        if self.stuck:
            gym_drive = 0.0
        else:
            accel = (self.a if delayed >= d0 else self.e)*(delayed - d0) - self.c*speed*abs(speed)
            target = speed + accel*ENV_DT
            if target <= 0.0:
                target = 0.0
                if delayed < u_break:
                    self.stuck, self.held_for = True, 0.0
            gym_drive = (GYM_MASS_KG*(target - speed)/ENV_DT + GYM_DRAG*speed*abs(speed))/GYM_MOTOR_N
            if speed < 0.03:
                gym_drive = max(0.0, gym_drive)       # never enter the Gym's reverse latch
        gym_steer = self.steering_sign*steer + self.steering_offset_rad/GYM_STEER_RAD
        return (min(1.0, max(-1.0, gym_drive)), min(1.0, max(-1.0, gym_steer)))


def load_policy():
    return load_policy_namespace({"String": lambda data: SimpleNamespace(data=data),
                                  "ConeDetection": SimpleNamespace(COLOR_BLUE=2, COLOR_YELLOW=1)})


REAL_TRACKS_DIR = Path(__file__).resolve().parent / "real_tracks"
# Right-angle/U-turn: both boundaries throughout, R 1.0/1.5 m (the two radii the
# right-angle-bend-is-own-goal memory note tracks), straight run-in/run-out so the
# cone history is built before the turn. Not required by the assessment; see that
# memory note and docs/UPSTREAM_FIXES_FOR_CAR_CN.md section 7 (single-sided bends).
SYNTHETIC_BENDS = {
    "right_angle_r1": (1.0, 90.0), "right_angle_r1_5": (1.5, 90.0), "u_turn": (1.0, 180.0)}


def road_spec(road="straight"):
    """straight; gentle_arc (2 m, R 5 m 45 deg, 4 m, cones on both sides); course (the
    notebook CONE_ROAD_SPEC unchanged: its R 5 m bend has LEFT cones only and the
    following straight has sparse, displaced right cones); s_bend (R 3 m, +-60 deg);
    right_angle_r1, right_angle_r1_5, u_turn (both boundaries, synthetic); or a name
    under real_tracks/ (built by real_track.py from a recorded run.jsonl's own
    dead-reckoned path and measured cone spacing -- see that tool's docstring for
    why this is a geometry/statistics fit, not the real cones placed literally)."""
    if road in ("straight", "s_bend"):
        return tune.road_spec(road)
    if road in SYNTHETIC_BENDS:
        radius_m, sweep_deg = SYNTHETIC_BENDS[road]
        spec = deepcopy(tune.CONFIG["CONE_ROAD_SPEC"])
        start, bend = deepcopy(spec["elements"][0]), deepcopy(spec["elements"][1])
        start["geometry"]["length_m"] = 2.0
        bend["geometry"] = {"type": "circular_arc", "curvature_per_m": 1.0/radius_m, "sweep_angle_deg": sweep_deg}
        bend["active_cone_paths"] = {"left_boundary": "regular", "right_boundary": "regular"}
        end = deepcopy(start)
        end["geometry"]["length_m"] = 2.0
        spec["elements"] = [start, bend, end]
        return spec
    track_file = REAL_TRACKS_DIR / f"{road}.json"
    if track_file.exists():
        track = json.loads(track_file.read_text(encoding="utf-8"))
        spec = deepcopy(tune.CONFIG["CONE_ROAD_SPEC"])
        elements = []
        for i, element in enumerate(track["elements"]):
            # The first element must carry "lanes" (nothing precedes it to inherit
            # from); later elements can use either template, "lanes" absent means
            # the cross-section continues unchanged (cross_section_width_interpolation).
            straight_template = i == 0 or element["geometry"]["type"] == "straight"
            base = deepcopy(spec["elements"][0] if straight_template else spec["elements"][1])
            base["geometry"] = element["geometry"]
            base["active_cone_paths"] = {"left_boundary": "regular", "right_boundary": "regular"}
            elements.append(base)
        spec["elements"] = elements
        if track.get("lane_width_m"):
            for element in spec["elements"]:
                if "lanes" in element:   # only the first element carries it; cross-
                    element["lanes"]["reference_lane"]["width_m"] = {    # section width
                        "start": track["lane_width_m"], "end": track["lane_width_m"]}  # is constant after
        if track.get("cone_spacing_m"):
            for path in spec["cone_paths"].values():
                path["profiles"]["regular"]["inter_cone_spacing"] = {
                    "distribution": "fixed", "value_m": track["cone_spacing_m"]}
        return spec
    spec = deepcopy(tune.CONFIG["CONE_ROAD_SPEC"])
    if road == "gentle_arc":
        start, bend = deepcopy(spec["elements"][0]), deepcopy(spec["elements"][1])
        start["geometry"]["length_m"] = 2.0
        bend["active_cone_paths"] = {"left_boundary": "regular", "right_boundary": "regular"}
        end = deepcopy(start)
        end["geometry"]["length_m"] = 4.0
        spec["elements"] = [start, bend, end]
    elif road != "course":
        raise ValueError(f"unknown road {road!r}")
    return spec


def make_env(offset, heading, noise, seed, lane_width_m=None, cone_spacing_m=None, road="straight",
             start_progress_m=0.0):
    spec = road_spec(road)
    if lane_width_m is not None:
        for element in spec["elements"]:
            if "lanes" in element:
                element["lanes"]["reference_lane"]["width_m"] = {"start": lane_width_m, "end": lane_width_m}
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
                        "progress": {"unit": "metres", "lower": start_progress_m, "upper": start_progress_m},
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
    # Planning V2 runs in-line (async worker disabled), see tools/policy_offline.py.
    settings = policy_settings(api, params, synchronous_planning=True)
    sensors = ("cone_detections", "fiducial_detections", "lidar_scan", "lidar_cartesian", "wheel_speed",
               "imu_orientation", "imu_angular_velocity", "imu_specific_force")
    clock = SimpleNamespace(now_s=0.0)
    node = SimpleNamespace(
        estimation_settings=settings["estimation"], planning_settings=settings["planning"],
        lattice_settings=settings["lattice"], mpc_settings=settings["mpc"], vehicle_settings=settings["vehicle"], mpc_controller=None,
        policy_frame_id=params.get("policy_frame_id", "base_link"), heading_reference=None,
        # The policy YAML's own sensor_timeout_s (all 0.5 in ai4r_policy.yaml, so V2 runs are
        # unchanged); the centerline overlay raises cone_detections to main's 1.5 s.
        sensor_timeout_s={name: params.get("sensor_timeout_s", {}).get(name, 0.5) for name in sensors},
        timestamp_tolerance_s=0.05,
        observations={name: None for name in sensors}, debug=[],
        motion_history=api["EstimationMotionHistory"](settings["estimation"]), clock=clock,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(clock.now_s*1e9))),
        _warn=lambda key, text: None, control_settings=settings["control"],
        controller=api["PolicyController"](settings["control"]), control_has_run=False,
        control_diagnostics=None, last_control_reference_deadline_s=None,
        distance_limiter=SimpleNamespace(distance_m=0.0),
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None),
        # MVP reference manager (robust control) runs on the replay clock.
        publish_state=lambda: None, _monotonic=lambda: clock.now_s)
    node.mpc_debug_publisher = SimpleNamespace(publish=lambda msg: node.debug.append(json.loads(msg.data)))
    bind_methods(node, api, METHODS)
    return node


def run(params, offset=0.0, heading=0.0, noise=0.0, seed=1, duration=20.0, start_time=100.0,
        trigger="cone", cone_latency=CONE_LATENCY_S, latency_jitter=0.0, gap_probability=0.0,
        plant=None, distance_m=None, lane_width_m=None, cone_spacing_m=None, road="straight",
        start_progress_m=0.0, lattice_delivery_s=None):
    """One closed-loop run. Defaults reproduce the original flow test (cone trigger,
    50 ms latency, course car); the keyword arguments add the car conditions above.
    distance_m ends the run as completed once the wheel-integrated distance reaches it.

    lattice_delivery_s (2026-10-18): params carry shadow.*/planner_control.* (the MVP2.1
    lattice); the real ShadowLatticePlanner runs in-process and each result becomes
    visible after its own measured planning time + lattice_delivery_s, as in
    tools/replay_run_jsonl.py --planner-control, but here the MPC drives the car."""
    api = load_policy()
    env = make_env(offset, heading, noise, seed, lane_width_m, cone_spacing_m, road, start_progress_m)
    observation, _ = env.reset(seed=seed)
    node = make_node(api, params)
    lattice = lattice_delivery_s is not None
    if lattice:
        from replay_run_jsonl import enable_lattice   # tools/ is on sys.path (see above)
        node.async_planner = node.mpc_held_reference = None
        enable_lattice(node, api, params, lattice_delivery_s)
    mpc_on = node.mpc_settings.enabled
    stops = (api["MPCStop"], api["PolicyStopRequest"])
    rng = random.Random(seed*7919 + 1)
    request, history = (0.0, 0.0), deque(maxlen=int(1.0/ENV_DT)+2)
    result = {"locked_stop": None, "steps": 0, "terminated": None, "completed": False}
    trace = {"t": [], "speed": [], "drive": [], "lateral": [], "progress": []}
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
                if lattice:
                    node._shadow_poll()          # the node's 20 Hz shadow timer
                try:
                    out = node.calculate_policy_actions(values, ages, stamps, dt, 0.0 if first else t-start_time,
                                                        first)
                    request = (float(out[0]), float(out[1]))
                    if lattice:
                        node._shadow_offer()     # after the action, as run_policy_step does
                except stops as stop:
                    result["locked_stop"] = {"time_s": round(t-start_time, 2), "reason": str(stop),
                                             "planning": (node.planning_output or {}).get("reason"),
                                             "road": (node.estimation_output or {}).get("road", {}).get("status")}
                    break
                last_policy_t, first = t, False
                result["steps"] += 1
            distance += speed*ENV_DT
            node.distance_limiter.distance_m = distance
            # Road-relative error from the Gym ground truth (equals world y on the straight).
            truth = env.unwrapped.get_current_ground_truth()
            trace["t"].append(round(t-start_time, 3))
            trace["speed"].append(speed)
            trace["drive"].append(request[0])
            trace["lateral"].append(float(truth["lateral_error_from_target_line_m"]))
            trace["progress"].append(float(truth["reference_line_progress_m"]))
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
    if lattice:
        result["reference_selection"] = dict(Counter(d.get("reference_selection") for d in node.debug))
        result["selection_reasons"] = dict(Counter(
            str(d.get("reference_selection_reason")).split(":")[0] for d in node.debug
            if d.get("reference_selection") == "baseline").most_common(4))
        result["lattice_results"] = dict(Counter(r.get("status") for r in node.lattice_results))
        sources = [d.get("reference_selection") for d in node.debug if d.get("reference_selection")]
        result["reference_switches"] = sum(1 for a, b in zip(sources, sources[1:]) if a != b)
    result.update(mpc_enabled=mpc_on, branches=dict(branches), reject_reasons=dict(reasons),
                  final_x_m=round(float(state["world_pose"]["x_m"]), 2),
                  final_y_m=round(float(state["world_pose"]["y_m"]), 3),
                  final_lateral_m=round(trace["lateral"][-1], 3) if trace["lateral"] else None,
                  final_progress_m=round(trace["progress"][-1], 3) if trace["progress"] else None,
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
