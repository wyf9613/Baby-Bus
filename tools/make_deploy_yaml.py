#!/usr/bin/env python3
"""Build the single policy YAML that DREAM reads on car .27 (babysitter guide 5.2).

    python tools/make_deploy_yaml.py --mode mpc-shadow --cones yellow-left [--distance 3.0]
        [--road-age-s 0.6] [--out DIR]

Layers, merged recursively in order (the repository files stay unchanged):
  steer-arc:  ai4r_policy.yaml + ai4r_policy_newcar27.yaml, controllers off, id_test arc sequence
  mvp:        ai4r_policy.yaml + ai4r_policy_newcar27.yaml
  mpc-shadow: ... + ai4r_policy_mpc_newcar27.yaml
  mpc-exec:   ... + the same, with shadow off and v_exec_max_mps <= 0.2 (needs vehicle.valid: true)
  mpc-shadow-centerline, mpc-exec-centerline: ... + ai4r_policy_mpc_centerline.yaml (2026-10-15
    port of main's ordered_xy estimation + road_centerline planning; right-angle/S-bend
    capable, unlike the y_of_x quadratic fit the plain mpc-shadow/mpc-exec modes keep)
  mpc-shadow-lattice21, mpc-exec-lattice21: mpc-*-centerline + ai4r_policy_mpc_lattice21.yaml
    (2026-10-16: the MVP2.1 lattice from origin/mvp2.1 feeds the MPC, road_centerline as
    fallback; exec added 2026-10-18 after the closed-loop Gym comparison in
    offline/mpc_gym/results/car_readiness_lattice21_car05.md). Every other mode asserts
    shadow.* and planner_control.* are off.
--longitudinal feedback (mpc-exec* with --car 05 only, 2026-10-20) adds
ai4r_policy_mpc_car05_feedback.yaml: wheel-speed PI drive, steering-only QP. Variants for
the car: feedback-nomm (measured_motion off, to separate its effect on steering) and
feedback-soft (kp 0.15 / ki 0.1, if the PI drive oscillates on the car).
--speed-profile v05 (mpc-exec-lattice21 with --car 05 only, 2026-10-27) replaces the 0.2 m/s
ceiling: speed cap and cruise speed 0.5, hard overspeed 0.70, horizon N 6, lattice path
1.4 m -- the Gym configuration in docs/MPC_PROGRESS_LOG.md 2026-10-27. --v-exec-max is
ignored with it. Not yet run on a car (first planned 2026-10-28): measure the coast
distance from 0.5 m/s before the first full run.
--cones (required except steer-arc) writes estimation.left_colour/right_colour. Check the
road on site first: the G2 field (2026-10-08) was blue-left, the course road on 10-13/14
yellow-left. A swapped pair makes every road invalid_width_or_order.
--road-age-s raises the camera age budget Planning may use (planning.max_source_age_s,
estimation.max_source_age_s and sensor_timeout_s.cone_detections together; the smallest
of the three applies). Default: unchanged, 0.4 s.
Writes DIR/ai4r_policy.yaml (copy it to the car as config/ai4r_policy.yaml), DIR/mode.txt
and DIR/policy_node.py, and prints both SHA-256 values to compare on the car.
Needs PyYAML only.
"""
import argparse
from copy import deepcopy
import hashlib
from pathlib import Path
import shutil
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
KEY = "/**/ai4r_policy"


def merge(target, changes):
    for name, value in changes.items():
        if isinstance(value, dict) and isinstance(target.get(name), dict):
            merge(target[name], value)
        else:
            target[name] = deepcopy(value)


# Steering map from steady arcs (vehicle_test_tools.py steering-arcs): u = 0 first for
# the straight drift, then both signs at two magnitudes. 2 s per stage at ~0.2 m/s is
# ~0.4 m of arc each, about 3 m in all; the first ~0.6 s of each stage is discarded.
ARC_STEERING = [0.0, 0.25, -0.25, 0.4, -0.4, 0.0]


def arc_test(drive):
    return {"mode": "arc", "confirmed": True, "negative_drive_confirmed": False,
            "durations_s": [2.0]*len(ARC_STEERING), "drive_values": [drive]*len(ARC_STEERING),
            "steering_values": list(ARC_STEERING), "drive_min": 0.0, "drive_max": 0.35,
            "steering_abs_max": 0.4, "speed_limit_mps": 0.35, "start_speed_max_mps": 0.03,
            "settle_s": 2.0, "max_dt_s": 0.15, "max_run_s": 30.0}


# ROS ConeDetection colour IDs (yellow=1, blue=2) as (left_colour, right_colour).
CONE_LAYOUTS = {"blue-left": (2, 1), "yellow-left": (1, 2)}


# --longitudinal variants on top of ai4r_policy_mpc_car05_feedback.yaml.
FEEDBACK_VARIANTS = {"feedback": {}, "feedback-nomm": {"measured_motion": False},
                     "feedback-soft": {"fb_kp": 0.15, "fb_ki": 0.1}}


# --speed-profile: (mpc, planning, shadow) changes. v05 = the 2026-10-27 Gym configuration:
# 0.5 m/s needs a 1.4 m lattice path (1.0 m ran out of preview coverage), N 6 covers one
# full steering swing (0.74 rad / 1.21 rad/s ~ 0.6 s), and the hard line must exceed
# cap + soft + cut (0.5 + 0.10 + 0.05 with the feedback overlay).
SPEED_PROFILES = {"v05": ({"v_exec_max_mps": 0.5, "overspeed_hard_mps": 0.70, "horizon_n": 6},
                          {"cruise_speed_mps": 0.5}, {"length_m": 1.4})}


def build(mode, distance, arc_drive=0.32, v_exec_max=0.2, cones="blue-left", road_age_s=None, car="27",
          run_time_s=None, longitudinal="joint", speed_profile=None):
    is_lattice21 = mode in ("mpc-shadow-lattice21", "mpc-exec-lattice21")
    is_shadow = mode in ("mpc-shadow", "mpc-shadow-centerline", "mpc-shadow-lattice21")
    is_exec = mode in ("mpc-exec", "mpc-exec-centerline", "mpc-exec-lattice21")
    is_centerline = mode in ("mpc-shadow-centerline", "mpc-exec-centerline", "mpc-shadow-lattice21",
                             "mpc-exec-lattice21")
    layers = ["ai4r_policy.yaml", "ai4r_policy_newcar27.yaml"]
    if mode not in ("mvp", "steer-arc"):
        layers.append("ai4r_policy_mpc_newcar27.yaml")
    if is_centerline:
        # 2026-10-15: main's ordered_xy estimation + road_centerline planning,
        # right-angle/S-bend capable (see docs/right-angle-bend memory note).
        layers.append("ai4r_policy_mpc_centerline.yaml")
    if is_lattice21:
        layers.append("ai4r_policy_mpc_lattice21.yaml")
    if car != "27" and mode not in ("mvp", "steer-arc"):
        # Cross-car vehicle.* substitution (never identified on this car); see
        # that overlay file's header and docs/MPC_PROGRESS_LOG.md 2026-10-15.
        layers.append("ai4r_policy_mpc_car05.yaml")
        if car == "01":     # 2026-10-22: car 05's record with car 01's measured steering
            layers.append("ai4r_policy_mpc_car01.yaml")
    if longitudinal != "joint":
        if not (is_exec and car in ("05", "01")):
            raise ValueError("--longitudinal feedback* is for mpc-exec* modes with --car 05 or 01 only")
        layers.append("ai4r_policy_mpc_car05_feedback.yaml")
    base = yaml.safe_load((ROOT / "config" / layers[0]).read_text(encoding="utf-8"))
    for layer in layers[1:]:
        merge(base, yaml.safe_load((ROOT / "config" / layer).read_text(encoding="utf-8")))
    p = base[KEY]["ros__parameters"]
    p["control"]["max_distance_m"] = distance        # 0 = no distance budget (loop tracks)
    if run_time_s is not None:
        p["control"]["max_run_time_s"] = run_time_s  # 0 = no time budget
    p["estimation"]["left_colour"], p["estimation"]["right_colour"] = CONE_LAYOUTS[cones]
    if road_age_s is not None:
        p["planning"]["max_source_age_s"] = road_age_s
        p["estimation"]["max_source_age_s"] = max(p["estimation"]["max_source_age_s"], road_age_s)
        p["sensor_timeout_s"]["cone_detections"] = max(p["sensor_timeout_s"]["cone_detections"], road_age_s)
    if is_exec:
        p["mpc"].update(shadow=False, v_exec_max_mps=min(p["mpc"]["v_exec_max_mps"], v_exec_max))
    if longitudinal != "joint":
        p["mpc"].update(FEEDBACK_VARIANTS[longitudinal])
    if speed_profile is not None:
        if not (mode == "mpc-exec-lattice21" and car == "05"):
            raise ValueError("--speed-profile is for mpc-exec-lattice21 with --car 05 only")
        mpc, planning, shadow = SPEED_PROFILES[speed_profile]
        p["mpc"].update(mpc)
        p["planning"].update(planning)
        p["shadow"].update(shadow)
        m = p["mpc"]
        assert m["overspeed_hard_mps"] > m["v_exec_max_mps"]+m["overspeed_soft_margin_mps"]+m["overspeed_cut_margin_mps"]
    if mode == "steer-arc":
        # Identification only: no controller owns the actions.
        p["control"]["enabled"] = False
        p["mpc"]["enabled"] = False
        # Async planning needs a controller to own it (startup check); none runs here.
        p["planning"]["lattice"]["async_enabled"] = False
        p["id_test"] = arc_test(arc_drive)

    assert p["policy_update_mode"] == "timer" and p["policy_update_rate_hz"] == 20.0
    assert p["required_sensors"] == ["cone_detections", "wheel_speed", "imu_angular_velocity"]
    if mode == "steer-arc":
        assert p["control"]["enabled"] is False and p["mpc"]["enabled"] is False
        return base, layers
    assert p["id_test"]["mode"] == "off"
    if is_lattice21:
        assert p["shadow"]["enabled"] is True and p["planner_control"]["enabled"] is True
    else:
        # The MVP2.1 lattice must never ride along in another profile unnoticed.
        assert not p.get("shadow", {}).get("enabled", False)
        assert not p.get("planner_control", {}).get("enabled", False)
    if mode == "mvp":
        assert p["control"]["enabled"] is True and p["mpc"]["enabled"] is False
        assert p["control"]["mode"] == "mvp" and p["control"]["mvp_steering_direction"] == -1.0
    else:
        assert p["control"]["enabled"] is False and p["mpc"]["enabled"] is True
        assert p["mpc"]["vehicle_params_source"] == "vehicle"
        assert p["mpc"]["reference_source"] == "planning"
        assert p["mpc"]["qp_solver"] == "dense", "the car has no osqp (G1, 2026-10-08)"
        assert p["planning"]["vehicle_limits_source"] == "upstream"
        if is_centerline:
            assert p["estimation"]["road_geometry"] == "ordered_xy"
            assert p["planning"]["algorithm"] == "road_centerline"
        else:
            assert p["planning"]["algorithm"] != "road_centerline"
        assert p["mpc"]["longitudinal_mode"] == ("joint" if longitudinal == "joint" else "feedback")
        if is_shadow:
            assert p["mpc"]["shadow"] is True
        else:
            assert p["vehicle"]["valid"] is True, \
                "mpc-exec: put the measured values in ai4r_policy_mpc_newcar27.yaml and set vehicle.valid: true first"
    return base, layers


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", required=True, choices=[
        "mvp", "mpc-shadow", "mpc-exec", "mpc-shadow-centerline", "mpc-exec-centerline",
        "mpc-shadow-lattice21", "mpc-exec-lattice21", "steer-arc"])
    parser.add_argument("--distance", type=float, default=3.0,
                        help="control.max_distance_m: (0, 3] for straights, up to 5 for a curve run, "
                             "0 = no limit (loop tracks)")
    parser.add_argument("--run-time-s", type=float,
                        help="control.max_run_time_s: (0, 600], or 0 = no limit; default from the layers")
    parser.add_argument("--arc-drive", type=float, default=0.32,
                        help="steer-arc: constant drive request, just above the 0.289 deadband / 0.30 breakaway")
    parser.add_argument("--v-exec-max", type=float, default=0.2,
                        help="mpc-exec: mpc.v_exec_max_mps cap, in (0, 0.2]; 0.15 for a first curve run")
    parser.add_argument("--cones", choices=sorted(CONE_LAYOUTS),
                        help="which colour is on the LEFT of the road; required except for steer-arc")
    parser.add_argument("--road-age-s", type=float,
                        help="camera age budget for Planning, in (0.4, 0.6]; default unchanged (0.4)")
    parser.add_argument("--car", default="27", choices=["27", "05", "01"],
                        help="05: cross-car vehicle.* substitution, never identified on that car "
                             "(see config/ai4r_policy_mpc_car05.yaml); only meaningful for mpc-shadow* "
                             "/mpc-exec* modes")
    parser.add_argument("--longitudinal", default="joint", choices=["joint", *FEEDBACK_VARIANTS],
                        help="feedback*: wheel-speed PI drive with a steering-only QP (mpc-exec* + --car 05)")
    parser.add_argument("--speed-profile", choices=sorted(SPEED_PROFILES),
                        help="v05: 0.5 m/s, hard 0.70, N 6, lattice 1.4 m (mpc-exec-lattice21 + --car 05); "
                             "replaces --v-exec-max")
    parser.add_argument("--out", type=Path, default=ROOT / ".verification" / "car-deploy")
    args = parser.parse_args()
    if args.cones is None:
        if args.mode != "steer-arc":
            parser.error("--cones blue-left|yellow-left is required: look at the road first")
        args.cones = "blue-left"   # steer-arc runs no estimator-dependent controller
    if args.road_age_s is not None and not 0.4 < args.road_age_s <= 0.6:
        parser.error("--road-age-s must be in (0.4, 0.6]")
    if not (args.distance == 0 or 0 < args.distance <= 5.0):
        parser.error("--distance must be 0 (no limit) or in (0, 5]")
    if args.run_time_s is not None and not (args.run_time_s == 0 or 0 < args.run_time_s <= 600):
        parser.error("--run-time-s must be 0 (no limit) or in (0, 600]")
    if args.distance > 3.0:
        print(f"NOTE: max_distance_m {args.distance} > 3 m: curve runs only, keep a person beside the car")
    if args.distance == 0 or args.run_time_s == 0:
        print("NOTE: no distance and/or time budget: only the operator Stop, sensor checks and the "
              "controller's own guards end the run")
    if not 0.29 <= args.arc_drive <= 0.35:
        parser.error("--arc-drive must be in [0.29, 0.35]")
    if not 0 < args.v_exec_max <= 0.2:
        parser.error("--v-exec-max must be in (0, 0.2]")
    try:
        params, layers = build(args.mode, args.distance, args.arc_drive, args.v_exec_max,
                               args.cones, args.road_age_s, args.car, args.run_time_s, args.longitudinal,
                               args.speed_profile)
    except ValueError as exc:
        parser.error(str(exc))
    args.out.mkdir(parents=True, exist_ok=True)
    yaml_path, node_path = args.out / "ai4r_policy.yaml", args.out / "policy_node.py"
    yaml_path.write_bytes(yaml.safe_dump(params, sort_keys=False).encode("utf-8"))
    shutil.copyfile(ROOT / "scripts" / "policy_node.py", node_path)
    p = params[KEY]["ros__parameters"]
    road_age = min(p["planning"]["max_source_age_s"], p["estimation"]["max_source_age_s"],
                   p["sensor_timeout_s"]["cone_detections"])
    (args.out / "mode.txt").write_text(
        f"{args.mode} {args.distance} run_time_s={p['control']['max_run_time_s']} cones={args.cones} "
        f"longitudinal={args.longitudinal} speed_profile={args.speed_profile} "
        f"road_age_s={road_age} layers={'+'.join(layers)}\n",
        encoding="utf-8")
    print(f"mode {args.mode}, layers {' + '.join(layers)}, max_distance_m {args.distance}, "
          f"max_run_time_s {p['control']['max_run_time_s']}")
    print(f"cones {args.cones} (left_colour {p['estimation']['left_colour']}, right_colour "
          f"{p['estimation']['right_colour']}; yellow=1 blue=2) | planning road age budget {road_age} s")
    if args.mode == "steer-arc":
        print("id_test", {k: p["id_test"][k] for k in ("mode", "steering_values", "drive_values", "durations_s")})
    print("control.enabled", p["control"]["enabled"], "| mpc",
          {k: p["mpc"][k] for k in ("enabled", "shadow", "v_exec_max_mps", "overspeed_hard_mps", "horizon_n",
                                    "qp_solver", "longitudinal_mode", "fb_kp", "fb_ki", "measured_motion")},
          "| vehicle", p["vehicle"]["source"], "valid", p["vehicle"]["valid"])
    print("MVP2.1 lattice: shadow.enabled", p.get("shadow", {}).get("enabled", False),
          "length_m", p.get("shadow", {}).get("length_m"), "| planning.cruise_speed_mps",
          p["planning"]["cruise_speed_mps"], "| planner_control", p.get("planner_control", {"enabled": False}))
    if args.speed_profile:
        print(f"NOTE: speed profile {args.speed_profile}: --v-exec-max ignored; measure the coast distance "
              "from this speed before the first full run")
    print(f"{yaml_path}\n  sha256 {sha256(yaml_path)}")
    print(f"{node_path}\n  sha256 {sha256(node_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
