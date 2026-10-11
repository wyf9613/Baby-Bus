#!/usr/bin/env python3
"""Replay recorded car bags through the real estimator, Planning and MPC, frame by frame.

    source /opt/ros/jazzy/setup.bash; source <ws with dream_interfaces>/install/setup.bash
    python3 tools/replay_policy_bag.py --params <deployed ai4r_policy.yaml> <bag dir> [<bag dir> ...]
    python3 tools/replay_policy_bag.py --params ... --set estimation.known_lane_width_m=1.0 <bags>

Why: on 2026-10-08 the shadow runs (G2) had 5-79 % valid references, rejected as
invalid_estimates or insufficient_near_or_far_coverage. mpc_debug does not say WHY the
road was invalid; this replay does, per frame, from the estimator's own road status.

How: the definitions are taken from scripts/policy_node.py (as offline/mpc_gym/pipeline_sim.py
does, without Gym). Recorded /car/cone_detections, /car/wheel_speed_m_per_sec and
/car/imu/data are fed in receive order through the node's own _store, with the car's
validation rules; a policy step runs on the 20 Hz timer once cones exist, like the car.
Per bag it prints the estimator road status and visibility, Planning's reason and the
replayed MPC branch, and the branches recorded on the car for comparison (fidelity check).

Assumptions (printed): the IMU mounting rotation is not in the bags (no /tf_static); the
default is the rotation printed by DREAM's imu_mount_tf on car .27 (180 deg about y,
base_link <- fsm300_sensor). The replay starts with an empty motion history, unlike the
car, so the first ~0.4 s may differ. --set overrides any section.key for what-if runs.
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

import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "scripts" / "policy_node.py"
NAMES = {"finite_number", "wrap_angle", "Observation", "valid_frame", "quaternion_xyzw", "quaternion_product",
         "quaternion_inverse", "rotate_vector", "EstimationSettings", "FirstOrderSampleFilter",
         "EstimationMotionHistory", "_estimation_median", "_estimation_solve", "_estimation_fit_boundary",
         "EstimationPipeline", "PlanningSettings", "CenterlinePlanner", "evaluate_planning_path",
         "planning_vehicle_limits", "MPCStop", "VehicleParamsSettings", "course_simulation_vehicle_params",
         "course_model_step", "MPCSettings", "mpc_vehicle_params", "reference_trajectory_from_planning",
         "reference_trajectory_from_road", "mpc_reference_errors", "mpc_path_projector", "VehicleModelMPC",
         "MPCController", "mpc_debug_json", "ControlSettings", "PolicyController", "ControlPID",
         "PolicyStopRequest", "control_path_geometry", "_control_poly_eval", "_control_poly_roots"}
METHODS = {"calculate_policy_actions", "_store"}
SENSORS = ("cone_detections", "fiducial_detections", "lidar_scan", "lidar_cartesian", "wheel_speed",
           "imu_orientation", "imu_angular_velocity", "imu_specific_force")
TOPICS = {"cones": "/car/cone_detections", "wheel": "/car/wheel_speed_m_per_sec", "imu": "/car/imu/data",
          "debug": "/car/mpc_debug"}
IMU_MOUNT_XYZW = (0.0, 1.0, 0.0, 0.0)     # base_link <- fsm300_sensor, car .27 imu_mount_tf (2026-10-08)
PERIOD_S = 0.05                            # the car's 20 Hz policy timer


def load_policy(cone_detection_class):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in NAMES]
    missing = NAMES - {n.name for n in nodes}
    if missing:
        raise SystemExit(f"policy_node.py lacks {sorted(missing)}; update this tool")
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PolicyNode")
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    namespace = {"dataclass": dataclass, "math": math, "time": time, "Real": Real, "json": json,
                 "deepcopy": deepcopy, "String": lambda data: SimpleNamespace(data=data),
                 "ConeDetection": cone_detection_class}
    exec(compile(ast.Module(body=nodes + methods, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace


def set_value(params, dotted, text):
    section, _, key = dotted.partition(".")
    if not key:
        raise SystemExit(f"--set needs section.key=value, got {dotted!r}")
    params.setdefault(section, {})[key] = yaml.safe_load(text)


def make_node(api, params):
    sections = {"estimation": api["EstimationSettings"], "planning": api["PlanningSettings"],
                "mpc": api["MPCSettings"], "vehicle": api["VehicleParamsSettings"],
                "control": api["ControlSettings"]}
    settings = {name: cls(**params.get(name, {})) for name, cls in sections.items()}
    timeouts = {name: 0.5 for name in SENSORS}
    timeouts.update(params.get("sensor_timeout_s", {}))
    clock = SimpleNamespace(now_s=0.0)
    node = SimpleNamespace(
        estimation_settings=settings["estimation"], planning_settings=settings["planning"],
        mpc_settings=settings["mpc"], vehicle_settings=settings["vehicle"], mpc_controller=None,
        policy_frame_id=params.get("policy_frame_id", "base_link"), heading_reference=None,
        sensor_timeout_s=timeouts, timestamp_tolerance_s=params.get("timestamp_tolerance_s", 0.05),
        observations={name: None for name in SENSORS}, debug=[], warnings=Counter(),
        motion_history=api["EstimationMotionHistory"](settings["estimation"]), clock=clock,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(clock.now_s*1e9))),
        control_settings=settings["control"], controller=api["PolicyController"](settings["control"]),
        control_has_run=False, control_diagnostics=None, last_control_reference_deadline_s=None,
        distance_limiter=SimpleNamespace(distance_m=0.0), estimation_output=None, planning_output=None,
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None))
    node._warn = lambda key, text: node.warnings.update([key])
    node.mpc_debug_publisher = SimpleNamespace(publish=lambda msg: node.debug.append(json.loads(msg.data)))
    for method in METHODS:
        setattr(node, method, api[method].__get__(node))
    return node


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    path = Path(path)
    uri, storage = str(path), ""
    if path.is_dir() and not (path / "metadata.yaml").exists():
        files = [f for f in path.iterdir() if f.suffix in (".mcap", ".db3")]
        if len(files) != 1:
            raise SystemExit(f"{path}: no metadata.yaml and {len(files)} storage files")
        uri, storage = str(files[0]), "mcap" if files[0].suffix == ".mcap" else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in TOPICS.values() if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    messages = []
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        messages.append((recv_ns, topic, deserialize_message(data, classes[topic])))
    messages.sort(key=lambda m: m[0])
    return messages, set(wanted)


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000 + header.stamp.nanosec


def feed(api, node, topic, msg, now_s, ros_ns, imu_mount):
    """The node's own callbacks, reduced to their validation and _store calls."""
    if topic == TOPICS["wheel"]:
        if api["finite_number"](msg.data) and msg.data >= 0.0:
            node._store("wheel_speed", float(msg.data), None, now_s, ros_ns)
        else:
            node._warn("wheel_invalid", "")
    elif topic == TOPICS["imu"]:
        frame = msg.header.frame_id
        if not api["valid_frame"](frame):
            node._warn("imu_frame", "")
            return
        body_from_sensor = (0.0, 0.0, 0.0, 1.0) if frame == node.policy_frame_id else imu_mount
        fields = (("imu_angular_velocity", msg.angular_velocity_covariance, msg.angular_velocity),
                  ("imu_specific_force", msg.linear_acceleration_covariance, msg.linear_acceleration))
        for name, covariance, measurement in fields:
            if covariance[0] == -1.0:
                continue
            try:
                node._store(name, api["rotate_vector"](body_from_sensor, measurement), stamp_ns(msg.header),
                            now_s, ros_ns)
            except ValueError:
                node._warn(name, "")
    elif topic == TOPICS["cones"]:
        if msg.header.frame_id != node.policy_frame_id:
            node._warn("cone_frame", "")
            return
        latency = msg.acquisition_to_publish_latency_s
        if not api["finite_number"](latency) or latency < 0.0:
            node._warn("cone_invalid", "")
            return
        colours = (api["ConeDetection"].COLOR_YELLOW, api["ConeDetection"].COLOR_BLUE)
        cones = []
        for cone in msg.detections:
            point = (cone.position.x, cone.position.y, cone.position.z)
            confidence = cone.classification_confidence
            if (not all(api["finite_number"](v) for v in point) or not api["finite_number"](confidence)
                    or not 0.0 <= confidence <= 1.0 or cone.color not in colours):
                node._warn("cone_invalid", "")
                return
            cones.append((*point, cone.color, confidence))
        node._store("cone_detections", {"detections": cones, "acquisition_to_publish_latency_s": float(latency)},
                    stamp_ns(msg.header), now_s, ros_ns)


def snapshot(node, now_s, ros_ns):
    values, ages, stamps = {}, {}, {}
    for name, o in node.observations.items():
        if o is None:
            values[name] = ages[name] = stamps[name] = None
            continue
        age = (ros_ns - o.stamp_ns)/1e9 if o.stamp_ns is not None else now_s - o.received_at
        ages[name], stamps[name] = age, o.stamp_ns
        values[name] = deepcopy(o.value) if 0.0 <= age < node.sensor_timeout_s[name] else None
    return values, ages, stamps


def replay(api, params, path, imu_mount):
    messages, topics = read_bag(path)
    node = make_node(api, params)
    stops = (api["MPCStop"], api["PolicyStopRequest"])
    recorded = Counter()
    rec_reasons = Counter()
    steps = []
    first, last_step, i = True, None, 0
    if not messages:
        return {"bag": Path(path).name, "steps": 0, "topics": sorted(topics)}
    t0 = messages[0][0]/1e9
    tick = t0 + PERIOD_S
    end = messages[-1][0]/1e9
    while tick <= end:
        while i < len(messages) and messages[i][0]/1e9 <= tick:
            recv_ns, topic, msg = messages[i]
            if topic == TOPICS["debug"]:
                d = json.loads(msg.data)
                recorded[d.get("branch")] += 1
                if d.get("reject_reason"):
                    rec_reasons[d["reject_reason"]] += 1
            else:
                feed(api, node, topic, msg, recv_ns/1e9, recv_ns, imu_mount)
            i += 1
        if node.observations["cone_detections"] is not None:
            ros_ns = round(tick*1e9)
            node.clock.now_s = tick
            values, ages, stamps = snapshot(node, tick, ros_ns)
            dt = 0.0 if first else tick - last_step
            before = len(node.debug)
            row = {"t": round(tick - t0, 2)}
            try:
                node.calculate_policy_actions(values, ages, stamps, dt, 0.0 if first else tick - t0, first)
            except stops as stop:
                row["stop"] = str(stop)
            road = (node.estimation_output or {}).get("road") or {}
            state = (node.estimation_output or {}).get("state") or {}
            row.update(road_status=road.get("status"), visibility=road.get("visibility"),
                       road_valid=road.get("valid"), lane_width=road.get("lane_width_m"),
                       speed_valid=state.get("speed_valid"), yaw_rate_valid=state.get("yaw_rate_valid"),
                       planning=(node.planning_output or {}).get("reason"),
                       planning_valid=(node.planning_output or {}).get("valid"))
            if len(node.debug) > before:
                row.update(branch=node.debug[-1].get("branch"), reject=node.debug[-1].get("reject_reason"))
            steps.append(row)
            first, last_step = False, tick
        tick += PERIOD_S
    n = len(steps)
    widths = [s["lane_width"] for s in steps if isinstance(s.get("lane_width"), (int, float))]
    return {"bag": Path(path).name, "steps": n, "topics": sorted(topics),
            "road_status": dict(Counter(s["road_status"] for s in steps).most_common()),
            "visibility": dict(Counter(s["visibility"] for s in steps).most_common()),
            "state_invalid": sum(1 for s in steps if not (s["speed_valid"] and s["yaw_rate_valid"])),
            "planning_reason": dict(Counter(s["planning"] for s in steps).most_common()),
            "replay_branch": dict(Counter(s.get("branch") for s in steps).most_common()),
            "recorded_branch": dict(recorded.most_common()),
            "recorded_reject": dict(rec_reasons.most_common()),
            "lane_width_m": None if not widths else [round(min(widths), 3), round(sorted(widths)[len(widths)//2], 3),
                                                      round(max(widths), 3)],
            "warnings": dict(node.warnings), "frames": steps}


def share(counter, n):
    return ", ".join(f"{k} {v} ({100*v/max(n, 1):.0f}%)" for k, v in counter.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--params", type=Path, required=True, help="the deployed ai4r_policy.yaml")
    parser.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE")
    parser.add_argument("--imu-mount", type=float, nargs=4, default=IMU_MOUNT_XYZW, metavar=("X", "Y", "Z", "W"))
    parser.add_argument("--json-out", type=Path, help="write per-bag results, including every frame")
    args = parser.parse_args()
    try:
        from dream_interfaces.msg import ConeDetection
    except ImportError:
        raise SystemExit("dream_interfaces is not importable: source the workspace that built it")
    params = yaml.safe_load(args.params.read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
    for item in args.set:
        key, _, value = item.partition("=")
        set_value(params, key, value)
    api = load_policy(ConeDetection)
    print(f"params {args.params} | overrides {args.set or 'none'} | IMU mount xyzw {tuple(args.imu_mount)}")
    results = []
    for path in args.bags:
        r = replay(api, deepcopy(params), path, tuple(args.imu_mount))
        results.append(r)
        n = r["steps"]
        print(f"\n== {r['bag']}: {n} steps")
        if not n:
            print(f"   no policy steps (topics found: {r['topics']})")
            continue
        print(f"   road status:     {share(r['road_status'], n)}")
        print(f"   visibility:      {share(r['visibility'], n)}")
        print(f"   lane width m:    min/median/max {r['lane_width_m']}; speed or yaw rate invalid in {r['state_invalid']} steps")
        print(f"   planning reason: {share(r['planning_reason'], n)}")
        print(f"   MPC branch, replay:   {share(r['replay_branch'], n)}")
        rec = sum(r["recorded_branch"].values())
        print(f"   MPC branch, recorded: {share(r['recorded_branch'], rec)}"
              + (f"; rejects {r['recorded_reject']}" if r["recorded_reject"] else ""))
        if r["warnings"]:
            print(f"   ignored inputs:  {r['warnings']}")
    total = Counter()
    for r in results:
        total.update(r.get("road_status", {}))
    print(f"\nAll bags, road status: {share(total, sum(total.values()))}")
    if args.json_out:
        args.json_out.write_text(json.dumps(results, indent=1), encoding="utf-8")
        print(f"per-frame results: {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
