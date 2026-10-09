#!/usr/bin/env python3
"""Partial Tuesday replay with an explicit stationary-motion assumption.

confidence-probe.json preserves full cone confidence/positions and source age.
The driving JSONL files do not, so they cannot reproduce exact planning inputs.
This tool compares reference acceptance on recorded cones, NOT car trajectories.
Run with a Python environment containing PyYAML; no ROS or hardware is used.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import runpy

import yaml


def replay(root):
    ns = runpy.run_path(str(root / "tests/test_estimation.py"))["namespace"]
    evidence = root / "docs/experiments/2026-10-06/evidence/newcar-27-20261006/newcar-test-evidence"
    params = yaml.safe_load((evidence / "deployed-config.yaml").read_text())["/**/ai4r_policy"]["ros__parameters"]
    estimator_settings = ns["EstimationSettings"](**params["estimation"])
    planner_settings = ns["PlanningSettings"](**params["planning"])
    control_settings = ns["ControlSettings"](**dict(params["control"], robustness_enabled=True,
        reference_hold_max_s=1.0, reference_hold_max_distance_m=0.25))
    motion = ns["EstimationMotionHistory"](estimator_settings)
    estimator = ns["EstimationPipeline"](estimator_settings, motion_history=motion)
    planner = ns["CenterlinePlanner"](planner_settings)
    manager = ns["ControlReferenceManager"](control_settings)
    controller = ns["PolicyController"](control_settings)
    frames = sorted(json.loads((evidence / "confidence-probe.json").read_text()),
                    key=lambda row: row["stamp_ns"]/1e9+row["age"])
    beginning = frames[0]["stamp_ns"]/1e9+frames[0]["age"]
    ending = frames[-1]["stamp_ns"]/1e9+frames[-1]["age"]+1.1
    index, latest = 0, None
    counts, modes = Counter(), Counter()
    baseline_active = robust_active = False
    baseline_stop = robust_stop = None
    trace = []
    # Synthetic stationary wheel/gyro history, including camera-delay warmup.
    for tick in range(int((ending-beginning+0.5)/0.05)+1):
        now = beginning-0.5+tick*0.05
        for name in ("wheel_speed", "imu_angular_velocity"):
            motion.observe(name, 0.0, tick, now, now)
        while index < len(frames) and frames[index]["stamp_ns"]/1e9+frames[index]["age"] <= now:
            latest = frames[index]
            index += 1
        if latest is None:
            continue
        age = now-latest["stamp_ns"]/1e9
        road = estimator.road(latest["batch"], latest["stamp_ns"], age)
        road = estimator._align_road(road, now, True)
        state = {"valid": True, "speed_valid": True, "yaw_rate_valid": True,
                 "speed_mps": 0.0, "yaw_rate_rps": 0.0, "timestamp_s": now,
                 "frame_id": "base_link",
                 "source_age_s": {"wheel_speed": 0.0, "imu_angular_velocity": 0.0}}
        reference, _ = planner.plan(road, state, {}, {}, now,
            {"road": 0.5, "wheel_speed": 0.15, "imu_angular_velocity": 0.15}, mvp=True)
        reason = reference["reason"]
        counts[reason] += 1
        if reference["valid"]:
            baseline_active = robust_active = True
        elapsed = now-beginning
        if baseline_active and baseline_stop is None and not reference["valid"]:
            baseline_stop = {"elapsed_s": round(elapsed, 3), "reason": reason}
        if robust_active and robust_stop is None:
            try:
                selected = manager.select(reference, road, state, motion, now, elapsed+1, 0.0, 0.2)
                _, _, diagnostics = controller.calculate(selected, state, {}, now, 0.05)
                if not diagnostics["valid"] or diagnostics["stop_requested"]:
                    raise ValueError(diagnostics["reason"])
                modes[manager.mode] += 1
                trace.append({"elapsed_s": round(elapsed, 3), "mode": manager.mode,
                              "upstream_reason": reason})
            except (ValueError, TypeError, ArithmeticError) as error:
                robust_stop = {"elapsed_s": round(elapsed, 3), "reason": str(error)}
    return {"scope": "recorded cone reference replay under SYNTHETIC stationary wheel/gyro feedback",
            "assumptions": ["v=0 and yaw_rate=0; not measured driving feedback",
                            "20 Hz deterministic updates; no measured scheduling delay",
                            "final Tuesday thresholds applied to the earlier confidence probe"],
            "full_driving_replay": "unreconstructable: driving JSONL omits per-cone confidence and height",
            "recorded_frames": len(frames), "planning_reason_counts": dict(counts),
            "legacy_first_stop": baseline_stop, "robust_first_stop": robust_stop,
            "robust_mode_counts": dict(modes), "trace": trace}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = replay(Path(__file__).resolve().parents[1])
    encoded = json.dumps(report, ensure_ascii=False, indent=2)+"\n"
    if args.output:
        args.output.write_text(encoded)
    print(json.dumps({key: value for key, value in report.items() if key != "trace"},
                     ensure_ascii=False, indent=2))
