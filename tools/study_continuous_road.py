"""Reproducible offline comparison of actual estimator definitions, without ROS.

Run with Python 3 from any directory. The baseline is the user's local version
1b759ed. Results are synthetic geometry/software timings, not car acceptance.
No sensor, vehicle, git mutation, or runtime configuration is started/changed.
"""
import ast
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real
from pathlib import Path
import statistics
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
BASELINE = "1b759ed"
NAMES = {"finite_number", "EstimationSettings", "FirstOrderSampleFilter",
         "EstimationMotionHistory", "_estimation_median", "_estimation_solve",
         "_estimation_fit_boundary", "EstimationPipeline"}


def load(source):
    parsed = ast.parse(source)
    definitions = [item for item in parsed.body
                   if isinstance(item, (ast.ClassDef, ast.FunctionDef)) and item.name in NAMES]
    namespace = dict(dataclass=dataclass, math=math, deepcopy=deepcopy, Real=Real)
    exec(compile(ast.Module(body=definitions, type_ignores=[]), "policy_node.py", "exec"), namespace)
    options = dict(history_propagation="continuous", road_history_mode="distance",
                   history_merge_single_side=True, history_merge_radius_m=0.08)
    if "continuous_road_output" in namespace["EstimationSettings"].__dataclass_fields__:
        options["continuous_road_output"] = True
    return namespace["EstimationPipeline"], namespace["EstimationSettings"](**options)


def scenario(pipeline_class, settings, yaw, dense=False):
    pipeline = pipeline_class(settings)
    xs = [-0.5+i*(3.5/63) for i in range(64)] if dense else [-0.3+i*0.3 for i in range(12)]
    speed, last_camera = 0.2, 10.0
    batch = None
    output, durations, errors = [], [], []
    for tick in range(13):
        now, dt = round(10.0+tick*0.05, 9), tick*0.05
        angle = yaw*dt
        dx = speed*math.sin(angle)/yaw if yaw else speed*dt
        dy = speed*(1-math.cos(angle))/yaw if yaw else 0.0
        if tick in (0, 1, 2, 3, 4, 12):
            # Observe a fixed world road, with a 0.3 s camera pause then recovery.
            last_camera = now
            c, s = math.cos(angle), math.sin(angle)
            batch = {"detections": [(c*(x-dx)+s*(y-dy), -s*(x-dx)+c*(y-dy),
                                      0.0, colour, 0.95)
                                     for colour, y in ((2, 0.5), (1, -0.5)) for x in xs]}
        begin = time.perf_counter_ns()
        result = pipeline.update(
            {"cone_detections": batch, "wheel_speed": speed, "imu_angular_velocity": (0, 0, yaw)},
            {"cone_detections": now-last_camera, "wheel_speed": 0.0, "imu_angular_velocity": 0.0},
            {"cone_detections": round(last_camera*1e9), "imu_angular_velocity": round(now*1e9)},
            {"wheel_speed": now}, now)
        durations.append((time.perf_counter_ns()-begin)/1e6)
        road = result["road"]
        output.append(dict(now_s=now, valid=road["valid"], status=road["status"],
                           degraded=road["degraded"], source_s=road["measurement_timestamp_s"]))
        if road["valid"]:
            c, s = math.cos(angle), math.sin(angle)
            errors.extend(dy+s*x+c*y for x, y in road["centerline_xy"])
            assert road["timestamp_s"] == result["state"]["timestamp_s"]
            assert road["measurement_timestamp_s"] == last_camera
        assert len(pipeline._road_cones) <= 128
    return output, durations, errors


def main():
    baseline = subprocess.run(
        ["git", "-c", f"safe.directory={ROOT.as_posix()}", "show", f"{BASELINE}:scripts/policy_node.py"],
        cwd=ROOT, check=True, capture_output=True, encoding="utf-8").stdout
    current = (ROOT / "scripts/policy_node.py").read_text(encoding="utf-8")
    report = {"baseline_commit": BASELINE,
              "current_policy_sha256": hashlib.sha256(current.encode()).hexdigest(),
              "limits": "Synthetic Windows Python; no ROS, detector, target CPU or physical car."}
    for label, source in (("baseline", baseline), ("continuous", current)):
        pipeline_class, settings = load(source)
        examples = {}
        timings, errors = [], []
        for yaw, name in ((0.0, "straight"), (0.4, "turn")):
            records, _, _ = scenario(pipeline_class, settings, yaw)
            examples[name] = records
        # Warm up, then include full update work at the cache bound, not only fit().
        for _ in range(5):
            scenario(pipeline_class, settings, 0.4, dense=True)
        for _ in range(100):
            _, elapsed, residuals = scenario(pipeline_class, settings, 0.4, dense=True)
            timings.extend(elapsed)
            errors.extend(residuals)
        ordered = sorted(timings)
        report[label] = {"scenarios": examples, "dense_full_update_samples": len(timings),
                         "median_ms": statistics.median(timings),
                         "p95_ms": ordered[math.ceil(0.95*len(ordered))-1],
                         "max_ms": max(timings),
                         "world_center_rmse_m": math.sqrt(sum(e*e for e in errors)/len(errors))}
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
