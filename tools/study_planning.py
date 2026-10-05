"""Bounded delayed/noisy straight-road replay of actual estimator + V1 planner.

No ROS, actuator or controller; timings exclude telemetry, estimator and plots.
Prints a reproducible JSON report with source/configuration hashes.
"""
import hashlib
import json
import math
from pathlib import Path
import random
import runpy
import statistics
import time

root = Path(__file__).resolve().parents[1]
api = runpy.run_path(str(root/"tests"/"test_estimation.py"))
cfg = api["namespace"]["PlanningSettings"](vehicle_limits_source="course_simulation")
errors, runtimes, failures, references = [], [], {}, []
for seed in range(5):
    rng = random.Random(seed)
    estimator = api["Pipeline"](api["Settings"]())
    planner = api["namespace"]["CenterlinePlanner"](cfg)
    for index in range(20):
        source_time = 10.0+index*0.1
        speed, slope, offset = 0.1, 0.04, 0.1*math.sin(index/5)
        args = api["input_record"](speed=speed, yaw=0.0)
        stamp = round(source_time*1e9)
        for key in args[2]:
            if key != "wheel_speed":
                args[2][key] = stamp
        args[3]["wheel_speed"] = 1.0+index*0.1
        xs = [0.5+0.3*i for i in range(8)]
        left = [(x, offset+slope*x+0.5+rng.gauss(0, 0.003)) for x in xs]
        right = [(x, offset+slope*x-0.5+rng.gauss(0, 0.003)) for x in xs]
        args[0]["cone_detections"] = api["cones"](left, right)
        estimates = estimator.update(*args, source_time+0.05)
        t0 = time.perf_counter()
        ref, diagnostics = planner.plan(estimates["road"], estimates["state"],
            estimates["obstacles"], estimates["vehicle_limits"], source_time+0.05)
        runtimes.append((time.perf_counter()-t0)*1000)
        if not ref["valid"]:
            failures[ref["reason"]] = failures.get(ref["reason"], 0)+1
            continue
        lo, hi = ref["path"]["range"]
        a0, a1 = ref["path"]["coeffs_low_to_high"]
        errors.append(math.sqrt(sum((a0+a1*x-(offset+slope*(x+speed*0.05)))**2
                                    for x in (lo, (lo+hi)/2, hi))/3))
        references.append({"seed": seed, "frame": index, "reference": ref,
                           "diagnostics": diagnostics})

report = {"scenario": "5x20 straight-road frames, 3 mm lateral noise, 50 ms source delay",
          "frames_total": 100, "valid_frames": len(references), "failure_reasons": failures,
          "mean_frame_path_rmse_m": statistics.mean(errors) if errors else None,
          "max_frame_path_rmse_m": max(errors) if errors else None,
          "planner_runtime_ms": {"median": statistics.median(runtimes),
                                 "p95": sorted(runtimes)[94], "max": max(runtimes)},
          "configuration": vars(cfg),
          "vehicle_limits": api["namespace"]["planning_vehicle_limits"]({}, cfg),
          "source_sha256": hashlib.sha256(api["SOURCE"].read_bytes()).hexdigest(),
          "limits": "synthetic replay, not controller closed-loop, ROS gate, or physical qualification",
          "references": references}
print(json.dumps(report, indent=2, allow_nan=False))
if len(references) != 100:
    raise SystemExit("Unexpected rejection in the defined straight-road replay")
