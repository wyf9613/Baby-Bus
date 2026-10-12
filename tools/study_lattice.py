#!/usr/bin/env python3
"""Hardware-free V2/actual controller closed loop using the teacher bicycle model.

Stdlib only. Numerical runs use a relaxed budget and REPORT actual elapsed times;
they do not qualify the Jetson 20 Hz runtime, lidar coverage or physical braking.
"""
import argparse
import json
import math
from pathlib import Path
import runpy
import statistics


ROOT = Path(__file__).resolve().parents[1]
ns = runpy.run_path(str(ROOT/"tests"/"test_estimation.py"))["namespace"]


def road_record(x, y, heading, now, bend, near):
    c, s = math.cos(heading), math.sin(heading)
    road = {"valid": True, "visibility": "both", "frame_id": "base_link",
            "timestamp_s": now, "source_age_s": 0.01}
    for key, side in (("centerline_xy", 0), ("left_boundary_xy", 0.5), ("right_boundary_xy", -0.5)):
        points = []
        for j in range(51):
            wx = x-0.5+j*0.08
            wy, slope = bend*wx*wx, 2*bend*wx
            norm = math.hypot(1, slope)
            wx2, wy2 = wx-side*slope/norm, wy+side/norm
            dx, dy = wx2-x, wy2-y
            px, py = c*dx+s*dy, -s*dx+c*dy
            if -0.3 <= px <= 3:
                points.append((px, py))
        if points[0][0] <= near <= points[-1][0]:
            road[key] = [(near, ns["_lattice_interp"](points, near)), *[p for p in points if p[0] > near]]
        else:
            road[key] = [p for p in points if p[0] >= near]
    return road


def rollout(bend, near, steps, mode="mvp", budget_s=5.0):
    planning = ns["PlanningSettings"](algorithm="lattice_v2", max_source_age_s=0.4, reference_lifetime_s=0.2)
    lattice = ns["LatticeSettings"](budget_s=budget_s, clear_start_assumed=True,
        model_source="course_simulation" if mode == "calibrated" else "course_approximation")
    planner = ns["FrenetLatticePlanner"](planning, lattice)
    controller = ns["PolicyController"](ns["ControlSettings"](enabled=True, mode=mode,
        vehicle_params_source="course_simulation" if mode == "calibrated" else "upstream"))
    x, y, heading, speed, steering = 0.0, -0.1, 0.0, 0.0, 0.0
    trace, runtimes, failure = [], [], None
    for step in range(steps):
        now = 10+step*0.05
        beta = math.atan(0.4*math.tan(steering))
        yaw = speed/0.33*math.cos(beta)*math.tan(steering)
        state = {"speed_valid": True, "yaw_rate_valid": True, "frame_id": "base_link",
                 "timestamp_s": now, "speed_mps": speed, "yaw_rate_rps": yaw,
                 "source_age_s": {"wheel_speed": 0.01, "imu_angular_velocity": 0.01}}
        obstacle = {"available": True, "frame_id": "base_link", "timestamp_s": now,
                    "source_age_s": 0.01, "points_xyz_m": []}
        ref, diag = planner.plan(road_record(x, y, heading, now, bend, near), state, obstacle, {}, now, mvp=mode == "mvp")
        runtimes.append(diag["elapsed_s"])
        if not ref["valid"]:
            failure = {"step": step, "reason": ref["reason"]}
            break
        drive, steer, control = controller.calculate(ref, state, {}, now, 0.05)
        if not control["valid"] or control["stop_requested"]:
            failure = {"step": step, "reason": control["reason"]}
            break
        requested = steer*math.pi/4
        steering += max(-math.pi/2*0.05, min(math.pi/2*0.05, requested-steering))
        beta = math.atan(0.4*math.tan(steering))
        yaw = speed/0.33*math.cos(beta)*math.tan(steering)
        x += speed*math.cos(heading+beta)*0.05
        y += speed*math.sin(heading+beta)*0.05
        heading += yaw*0.05
        speed = max(0.0, speed+(10*drive-speed*speed)/3*0.05)
        trace.append({"t": step*0.05, "x": x, "y": y, "heading": heading, "speed": speed,
                      "steering": steering, "drive": drive, "road_error_m": (y-bend*x*x)/math.hypot(1, 2*bend*x),
                      "score": diag["selected"]["score"], "elapsed_s": diag["elapsed_s"]})
    ordered = sorted(runtimes)
    return {"bend": bend, "near_m": near, "controller": mode, "steps": len(trace), "failure": failure,
            "initial_error_m": -0.1, "final_error_m": trace[-1]["road_error_m"] if trace else None,
            "progress_x_m": x, "final_speed_mps": speed,
            "runtime_median_s": statistics.median(runtimes),
            "runtime_p95_s": ordered[int(0.95*(len(ordered)-1))], "runtime_max_s": max(runtimes),
            "over_nominal_35ms": sum(t > 0.035 for t in runtimes), "trace": trace}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--budget-s", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=ROOT/".verification"/"lattice-study.json")
    args = parser.parse_args()
    if not 1 <= args.steps <= 1000:
        parser.error("steps must be in 1..1000")
    if not math.isfinite(args.budget_s) or args.budget_s <= 0:
        parser.error("budget must be finite and positive")
    results = [rollout(bend, near, args.steps, mode, args.budget_s) for mode, bend, near in
               (("mvp", 0, 0.1), ("mvp", 0.04, 0.1), ("mvp", 0, 0.9), ("calibrated", 0.04, 0.1))]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"model": "teacher_approximation", "budget_s": args.budget_s,
        "runtime_qualification": "Windows offline only; nominal budget exceedances reported",
        "scenarios": results}, indent=2), encoding="utf-8")
    for result in results:
        print(json.dumps({k: v for k, v in result.items() if k != "trace"}))
    if any(r["failure"] for r in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
