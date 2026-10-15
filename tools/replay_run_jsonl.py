#!/usr/bin/env python3
"""Replay a recorded run.jsonl (docs/experiments/.../evidence/*/run.jsonl) through
the real Estimation, road_centerline Planning and MPC, shadow-by-shadow against
the car's own recorded PID actions. No ROS needed.

    python tools/replay_run_jsonl.py docs/experiments/2026-10-14/evidence/car15/visualization4/run.jsonl
    python tools/replay_run_jsonl.py <run.jsonl> --qp-solver dense --json-out results.json

Why: the fixture-based tests (tests/test_mpc_real_frames.py) cover a handful of
single/few-frame snapshots; this replays a whole ~20 s real run (car15,
road_centerline/ordered_xy, 2026-10-14) to check the MPC port over a full
closed-loop timeline instead of isolated frames.

How: scripts/policy_node.py is loaded without ROS by tools/policy_offline.py (as
offline/mpc_gym/pipeline_sim.py and tools/replay_policy_bag.py do). The run.jsonl
recorder's "cones"/"wheel"/"gyro" events feed the node's own _store, in file
order, exactly like the car's callbacks; a policy step runs at every recorded
"control_sample" event (the real 20 Hz timer tick), using whatever sensor state
is stored by that point in the file -- so this replays the exact same tick
count and sensor history the car saw.

The replay node runs mpc.enabled/shadow only (control.enabled: false): the
car's own PID actions are already in the log (each control_sample's "actions"/
"applied"), so there is no need to recompute them, and running the PID branch
here too would raise PolicyStopRequest on every transient planning failure
and skip the MPC block below it in the same calculate_policy_actions call --
exactly the moments most worth comparing. This also means the replay never
needs control.enabled and mpc.enabled on at once (which PolicyNode.__init__
refuses for the real actuator path); it only ever runs the shadow MPC.

Assumptions: the run's own "parameters" event (logged once) gives the real
deployed estimation/planning values; anything it does not cover
(estimation.road_geometry, single_side_offset_fraction, ...) defaults to
config/ai4r_policy_mpc_centerline.yaml's overlay, since that is what the
SIMPLE_CENTERLINE_FOLLOW_CN.md docs say this era of car15 runs used. --set
overrides any section[.sub].key for what-if runs.

The "adapted ref vs logged trajectory" deviation is a replay-fidelity check,
not a closed-loop tracking error. Both sides are motion-transported into the
same current-tick base_link frame before comparing (that alone turned out not
to matter much on the car15 runs: speeds are low). What does matter: most
steps on these runs are near_field_unobserved/near_field_requires_both_
boundaries, where planning.allow_forward_only accepts a forward-only
(truncated-near-end) centerline instead of the full one, so the two sides can
legitimately cover different near-end ranges; it also rises across a road
dropout the MPC bridges with mpc.reference_dropout_tolerance_s while the PID's
logged Path does not (the documented reference-handoff jump). See the printed
NOTE and the per-step --json-out's road_status/mpc_branch before treating a
large number as a regression.
"""
import argparse
from collections import Counter
from copy import deepcopy
import json
import math
from pathlib import Path
import statistics
import sys
from types import SimpleNamespace

import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from policy_offline import bind_methods, load_policy_namespace, policy_settings  # noqa: E402
from make_deploy_yaml import KEY, merge  # noqa: E402

METHODS = ("calculate_policy_actions", "_store", "_robust_control_enabled", "_mpc_uses_planning",
           "_select_control_reference", "_report_control_status")
SENSORS = ("cone_detections", "fiducial_detections", "lidar_scan", "lidar_cartesian", "wheel_speed",
           "imu_orientation", "imu_angular_velocity", "imu_specific_force")
def load_policy():
    return load_policy_namespace({"String": lambda data: SimpleNamespace(data=data)})


def base_params(**overrides):
    """config/ai4r_policy.yaml + newcar27 + mpc_newcar27 + mpc_centerline, then
    mpc.enabled/shadow forced on and control.enabled forced off (see the
    module docstring: the replay only ever runs the shadow MPC)."""
    layers = ["ai4r_policy.yaml", "ai4r_policy_newcar27.yaml",
              "ai4r_policy_mpc_newcar27.yaml", "ai4r_policy_mpc_centerline.yaml"]
    base = yaml.safe_load((ROOT.parent / "config" / layers[0]).read_text(encoding="utf-8"))
    for layer in layers[1:]:
        merge(base, yaml.safe_load((ROOT.parent / "config" / layer).read_text(encoding="utf-8")))
    p = base[KEY]["ros__parameters"]
    # "vehicle" (car .27's fitted drive/steering record from ai4r_policy_mpc_newcar27.yaml),
    # not "course_simulation": shadow mode reads vehicle.* regardless of vehicle.valid,
    # and the course model's generic deadband/gain would make drive incomparable to
    # the real car's logged actions (course_simulation needs far less drive for 0.2 m/s).
    p["mpc"].update(enabled=True, shadow=True, vehicle_params_source="vehicle")
    p["control"]["enabled"] = False   # unused (never runs); its own validation still applies
    merge(p, overrides)
    return p


def apply_recorded_parameters(params, recorded):
    """Layer the run's own logged "parameters" event (exact deployed estimation/
    planning values) on top of the defaults; control.enabled/mpc.enabled stay
    forced as base_params set them regardless of what the car itself ran."""
    # control.* is skipped: control.enabled is always forced false below (the
    # replay never runs the PID branch, see the module docstring), and the
    # recorded dict's control keys are a presentation subset that does not
    # all match ControlSettings' field names.
    for dotted, value in (recorded or {}).items():
        section, _, key = dotted.partition(".")
        if section in ("control",) or section not in params or not isinstance(params[section], dict):
            continue
        params[section][key] = value
    # Forced regardless of what the recorded run itself used: see base_params.
    params["control"]["enabled"] = False
    params["mpc"]["enabled"] = True
    params["mpc"]["shadow"] = True


def set_value(params, dotted, text):
    *sections, key = dotted.split(".")
    target = params
    for section in sections:
        target = target.setdefault(section, {})
    target[key] = yaml.safe_load(text)


def make_node(api, params):
    settings = policy_settings(api, params, synchronous_planning=True)
    timeouts = {name: 0.5 for name in SENSORS}
    timeouts.update(params.get("sensor_timeout_s", {}))
    clock = SimpleNamespace(now_s=0.0)
    node = SimpleNamespace(
        estimation_settings=settings["estimation"], planning_settings=settings["planning"],
        lattice_settings=settings["lattice"], mpc_settings=settings["mpc"], vehicle_settings=settings["vehicle"],
        mpc_controller=None, policy_frame_id=params.get("policy_frame_id", "base_link"),
        heading_reference=None, sensor_timeout_s=timeouts,
        timestamp_tolerance_s=params.get("timestamp_tolerance_s", 0.05),
        observations={name: None for name in SENSORS}, debug=[], warnings=Counter(),
        motion_history=api["EstimationMotionHistory"](settings["estimation"]), clock=clock,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(clock.now_s*1e9))),
        control_settings=settings["control"], controller=api["PolicyController"](settings["control"]),
        control_has_run=False, control_diagnostics=None, last_control_reference_deadline_s=None,
        distance_limiter=SimpleNamespace(distance_m=0.0), estimation_output=None, planning_output=None,
        planning_diagnostics=None, async_planner=None, mpc_held_reference=None,
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None),
        publish_state=lambda: None, _monotonic=lambda: clock.now_s)
    node._warn = lambda key, text: node.warnings.update([key])
    node.mpc_debug_publisher = SimpleNamespace(publish=lambda msg: node.debug.append(json.loads(msg.data)))
    bind_methods(node, api, METHODS)
    return node


def load_run(path):
    events, recorded_parameters = [], None
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        event = json.loads(line)
        events.append(event)
        if event["kind"] == "parameters":
            recorded_parameters = event["value"]
    return events, recorded_parameters


def nearest(values, target_t, max_age_s=0.3):
    """The last (t, ...) entry with t <= target_t, within max_age_s."""
    best = None
    for t, *rest in values:
        if t <= target_t and (best is None or t > best[0]):
            best = (t, *rest)
    if best is not None and target_t - best[0] <= max_age_s:
        return best
    return None


def path_point_distance(points, x, y):
    """Min distance from (x, y) to the polyline samples_xy/points list."""
    best = math.inf
    for a, b in zip(points, points[1:]):
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        dx, dy = bx-ax, by-ay
        length2 = dx*dx+dy*dy
        t = 0.0 if length2 <= 1e-12 else max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/length2))
        px, py = ax+t*dx, ay+t*dy
        best = min(best, math.hypot(px-x, py-y))
    return best


def replay(api, params, path):
    events, recorded_parameters = load_run(path)
    apply_recorded_parameters(params, recorded_parameters)
    node = make_node(api, params)
    stops = (api["MPCStop"], api["PolicyStopRequest"])
    t0 = events[0]["time_unix_s"]
    trajectories = []     # (t, [[x, y], ...]) from logged "trajectory" events
    steps = []
    first, last_step = True, None
    for event in events:
        t, kind, value = event["time_unix_s"], event["kind"], event["value"]
        now_s, ros_ns = t, round(t*1e9)
        node.clock.now_s = now_s
        if kind == "cones":
            node._store("cone_detections",
                       {"detections": [tuple(p) for p in value["positions"]],
                        "acquisition_to_publish_latency_s": value["latency_s"]},
                       round(value["stamp_s"]*1e9), now_s, ros_ns)
        elif kind == "wheel":
            if api["finite_number"](value) and value >= 0.0:
                node._store("wheel_speed", float(value), None, now_s, ros_ns)
        elif kind == "gyro":
            node._store("imu_angular_velocity", (0.0, 0.0, float(value["z"])),
                       round(value["stamp_s"]*1e9), now_s, ros_ns)
        elif kind == "trajectory" and value.get("point_count"):
            trajectories.append((t, value["positions_xy"]))
        elif kind == "control_sample":
            if node.observations["cone_detections"] is None:
                continue
            values = {k: (None if o is None else o.value) for k, o in node.observations.items()}
            ages = {k: (None if o is None else now_s-o.received_at) for k, o in node.observations.items()}
            stamps = {k: (None if o is None else o.stamp_ns) for k, o in node.observations.items()}
            dt = 0.0 if first else now_s-last_step
            row = {"t": round(t-t0, 3), "logged_actions": value.get("actions"),
                  "logged_applied": value.get("applied"), "logged_path_error_m": value.get("path_error_m")}
            try:
                node.calculate_policy_actions(values, ages, stamps, dt, 0.0 if first else t-t0, first)
            except stops as stop:
                row["stop"] = str(stop)
            road = (node.estimation_output or {}).get("road") or {}
            debug = node.mpc_controller.last_debug if node.mpc_controller else {}
            row.update(road_status=road.get("status"), visibility=road.get("visibility"),
                      planning_reason=(node.planning_output or {}).get("reason"),
                      planning_valid=(node.planning_output or {}).get("valid"),
                      mpc_branch=debug.get("branch"), mpc_reject_reason=debug.get("reject_reason"),
                      mpc_steer=debug.get("candidate_steer_action"), mpc_drive=debug.get("candidate_drive"))
            traj_ref = nearest(trajectories, t)
            adapted = getattr(node, "_trajectory_reference", None)
            if (traj_ref is not None and len(traj_ref[1]) >= 2 and adapted
                    and adapted.get("valid") and adapted["path"]["type"] == "CARTESIAN_CENTERLINE"):
                mpc_points = adapted["path"]["samples_xy"]
                # The logged trajectory's points are in base_link AT ITS OWN
                # (slightly earlier) timestamp, a moving frame; motion-transport
                # them into the current tick's frame before comparing, the same
                # way transport_control_reference reframes a delayed reference.
                old_t = traj_ref[0]
                pose, problem = (None, "same_tick") if old_t == t else node.motion_history.pose_between(old_t, t)
                if len(mpc_points) >= 2 and (pose is not None or problem == "same_tick"):
                    current_frame = traj_ref[1] if pose is None else [
                        node.motion_history.transform_xy(p, pose) for p in traj_ref[1]]
                    row["reference_deviation_m"] = max(
                        path_point_distance(mpc_points, x, y) for x, y in current_frame)
            steps.append(row)
            first, last_step = False, now_s
    return steps, node.warnings


def summarize(bag_name, steps, warnings):
    n = len(steps)
    valid_branches = ("solved", "shadow")
    mpc_valid = [s for s in steps if s["mpc_branch"] in valid_branches]
    steer_pairs = [(s["mpc_steer"], s["logged_actions"]["steer"]) for s in mpc_valid
                  if s.get("logged_actions") and api_finite(s["mpc_steer"])]
    deviations = [s["reference_deviation_m"] for s in steps if "reference_deviation_m" in s]
    result = {"run": bag_name, "steps": n,
              "mpc_valid_rate": None if not n else round(len(mpc_valid)/n, 3),
              "mpc_branch": dict(Counter(s["mpc_branch"] for s in steps).most_common()),
              "mpc_reject_reason": dict(Counter(s["mpc_reject_reason"] for s in steps
                                                if s.get("mpc_reject_reason")).most_common()),
              "road_status": dict(Counter(s["road_status"] for s in steps).most_common()),
              "steer_rmse": None, "steer_max_abs_diff": None,
              "reference_deviation_max_m": None if not deviations else round(max(deviations), 4),
              "reference_deviation_mean_m": None if not deviations else round(sum(deviations)/len(deviations), 4),
              "reference_deviation_median_m": None if not deviations else round(statistics.median(deviations), 4),
              "warnings": dict(warnings)}
    if steer_pairs:
        diffs = [abs(a-b) for a, b in steer_pairs]
        result["steer_rmse"] = round(math.sqrt(sum(d*d for d in diffs)/len(diffs)), 4)
        result["steer_max_abs_diff"] = round(max(diffs), 4)
    return result


def api_finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_jsonl", type=Path)
    parser.add_argument("--qp-solver", choices=["dense", "osqp"], default="dense",
                        help="dense: no osqp needed, matches the car's Jetson (car-jetson-no-osqp)")
    parser.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    params = base_params()
    params["mpc"]["qp_solver"] = args.qp_solver
    for item in args.set:
        key, _, value = item.partition("=")
        set_value(params, key, value)
    api = load_policy()
    steps, warnings = replay(api, deepcopy(params), args.run_jsonl)
    result = summarize(args.run_jsonl.name, steps, warnings)
    print(f"== {result['run']}: {result['steps']} control steps")
    print(f"   MPC shadow valid rate:  {result['mpc_valid_rate']}")
    print(f"   MPC branch:             {result['mpc_branch']}")
    if result["mpc_reject_reason"]:
        print(f"   MPC reject reasons:     {result['mpc_reject_reason']}")
    print(f"   road status:            {result['road_status']}")
    print(f"   steer vs logged PID:    RMSE {result['steer_rmse']}, max |diff| {result['steer_max_abs_diff']}")
    print(f"   adapted ref vs logged trajectory: median {result['reference_deviation_median_m']} m,"
          f" mean {result['reference_deviation_mean_m']} m, max {result['reference_deviation_max_m']} m")
    print("   NOTE: most of this run's steps are near_field_unobserved/near_field_requires_both_"
          "boundaries, where planning.allow_forward_only accepts a forward-only (truncated-near-end)"
          " centerline instead of the full one; the logged Path may cover a different near-end range,"
          " which alone can dominate this metric. It also rises across road dropouts the MPC bridges"
          " with mpc.reference_dropout_tolerance_s while the PID's logged Path does not (the documented"
          " reference-handoff jump, SIMPLE_CENTERLINE_FOLLOW_CN.md). Read it as a replay-fidelity signal,"
          " not a tracking error, and check road_status/mpc_branch in --json-out for a given t before"
          " treating a run's number as a regression.")
    if result["warnings"]:
        print(f"   ignored inputs:         {result['warnings']}")
    if args.json_out:
        args.json_out.write_text(json.dumps({"summary": result, "steps": steps}, indent=1), encoding="utf-8")
        print(f"per-step results: {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
