"""Reproducible real-cone geometry benchmark, with no ROS or vehicle connection.

Loads actual pure policy definitions. No algorithm copies. Masking occurs before
the stable estimator, after one genuine two-sided width-learning frame. Each
masked observation is then reconstructed directly, without hidden-side history.
This is a geometry sequence, not a timed-sensor replay or closed-loop test.
"""
import argparse
import ast
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import platform
import runpy
import statistics
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]


def load(source):
    baseline = runpy.run_path(str(ROOT / "tests/test_estimation.py"))["namespace"]
    ns = dict(baseline, time=time)
    tree = ast.parse(source.read_text(encoding="utf-8"))
    definitions = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
        and (n.name in ("ShadowSettings", "ShadowFrame", "ShadowLatticePlanner")
             or n.name.startswith("_shadow_")) and n.name != "_shadow_worker_entry"]
    # The planner's shared module constant (2026-10-20).
    definitions += [n for n in tree.body if isinstance(n, ast.Assign)
                    and [getattr(t, "id", None) for t in n.targets] == ["EXECUTION_TAIL_M"]]
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), ns)
    return ns


def stats(values):
    ordered = sorted(values)
    return {"count": len(values), "mean": statistics.mean(values), "median": statistics.median(values),
            "p95": ordered[min(len(ordered)-1, int(.95*(len(ordered)-1)))],
            "minimum": ordered[0], "maximum": ordered[-1]}


def prefix(ns, points, length):
    if not points:
        return points
    lengths = ns["_shadow_lengths"](points)
    end = min(length, lengths[-1])
    result = [p for p, s in zip(points, lengths) if s < end]
    result.append(ns["_shadow_at"](points, lengths, end))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=int, default=60)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--before-source", type=Path)
    parser.add_argument("--before-config", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/experiments/2026-10-15/evidence/shadow")
    args = parser.parse_args()
    source = ROOT / "scripts/policy_node.py"
    config_path = ROOT / "config/ai4r_policy.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
    ns = load(source)
    log_path = ROOT / "docs/experiments/2026-10-14/evidence/car15/visualization4/run.jsonl"
    rows = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    frames, speed = [], 0.0
    for row in rows:
        if row["kind"] == "wheel":
            speed = row["value"]
        elif row["kind"] == "cones":
            frames.append({"cone_index": len(frames), "elapsed_s": row["elapsed_s"],
                           "speed_mps": speed, **row["value"]})
    chosen = frames[args.start:args.start+args.frames]
    if not chosen or args.repeats < 1:
        parser.error("nonempty frame selection and positive repeats required")
    settings = ns["EstimationSettings"](**config["estimation"])
    scenarios = {}
    for scenario in ("both", "left_only", "right_only", "left_short", "right_short"):
        pipeline = ns["EstimationPipeline"](settings)
        seed = None
        for frame in frames[:args.start]:
            warm = pipeline._history_road({"detections": frame["positions"]}, round(frame["stamp_s"]*1e9), 0, frame["stamp_s"])
            if getattr(pipeline, "_learned_lane_width_m", None):
                seed = frame["cone_index"]
                break
        if seed is None:
            raise RuntimeError("No earlier genuine two-sided width-learning frame")
        scenario_jobs = []
        for frame in chosen:
            side = "left" if scenario.startswith("left") else "right"
            colour = settings.left_colour if side == "left" else settings.right_colour
            detections = frame["positions"] if scenario == "both" else [p for p in frame["positions"] if p[3] == colour]
            started = time.perf_counter()
            road = pipeline.road({"detections": detections}, round(frame["stamp_s"]*1e9), 0)
            estimation_s = time.perf_counter()-started
            # Both raw-colour removal and absent opposite boundary are verified.
            if scenario != "both":
                hidden = "right" if side == "left" else "left"
                assert not road.get(hidden+"_boundary_xy"), (scenario, frame["cone_index"])
            if scenario.endswith("short"):
                road = deepcopy(road)
                road["centerline_xy"] = prefix(ns, road.get("centerline_xy"), .15)
                road[side+"_boundary_xy"] = prefix(ns, road.get(side+"_boundary_xy"), .15)
            record = {"road": road, "state": {"speed_mps": frame["speed_mps"]},
                "settings": config["shadow"].copy(), "preview_m": config["planning"]["preview_m"],
                "stamp_s": frame["stamp_s"], "control_settings": config["control"],
                "control_period_s": 1/config["policy_update_rate_hz"],
                "target_speed_mps": config["planning"]["cruise_speed_mps"],
                "generation": 1, "job_id": frame["cone_index"], "submitted_monotonic_s": time.monotonic()}
            scenario_jobs.append((record, estimation_s, len(detections), seed))
        scenarios[scenario] = scenario_jobs
    report = {"date": "2026-10-15", "mode": "static real-cone geometry sequence with artificial side masking",
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "log_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
        "python": sys.version, "platform": platform.platform(), "processor": platform.processor(),
        "start_cone_index": chosen[0]["cone_index"], "end_cone_index": chosen[-1]["cone_index"],
        "recorded_elapsed_range_s": [chosen[0]["elapsed_s"], chosen[-1]["elapsed_s"]],
        "recorded_speed_range_mps": [min(f["speed_mps"] for f in chosen), max(f["speed_mps"] for f in chosen)],
        "candidate_count": config["shadow"]["sample_count"], "repeats": args.repeats,
        "limitations": ["no ROS, no physical vehicle, no closed loop", "frame coordinates remain at acquisition",
            "both scenario retains original recorded colours; not every frame actually sees both sides",
            "speed associated by last recorded receipt, not sensor-time alignment",
            "one prior two-sided frame seeds width; no hidden-side geometry/history is passed",
            "short cases artificially truncate reference AND known boundary to 0.15 m support",
            "wire timing excludes ROS/DDS and asynchronous queue wait; first job includes process startup"],
        "cases": []}
    for rate in (1.0, 10.0):
        for scenario, jobs in scenarios.items():
            planner = ns["ShadowLatticePlanner"](ns["ShadowSettings"](**{**config["shadow"], "rate_hz": rate}))
            planner.plan(jobs[0][0])  # Untimed warm-up.
            runs, timings = [], []
            for repeat in range(args.repeats):
                for record, estimation_s, detections, seed in jobs:
                    result = planner.plan(record)
                    if scenario != "both" and result["status"] != "NO_ROAD":
                        assert result["known_boundary_sides"] == ["left" if scenario.startswith("left") else "right"]
                    timings.append(result["timing_s"])
                    selected = result.get("selected")
                    runs.append({"cone_index": record["job_id"], "repeat": repeat, "status": result["status"],
                        "detections": detections, "width_seed_cone_index": seed, "road_status": record["road"].get("status"),
                        "road_visibility": record["road"].get("visibility"), "known_sides": result.get("known_boundary_sides"),
                        "center_support_m": result.get("observed_centerline_length_m"),
                        "feasible_count": result.get("feasible_count"), "selected_family": selected["family"] if selected else None,
                        "turn_angle_rad": result.get("turn_angle_rad"), "check_length_m": result.get("check_length_m"),
                        "selected_collision": selected["collision"] if selected else None,
                        "planning_s": result["elapsed_s"], "estimation_s": estimation_s})
            stage_ms = {key: stats([t[key]*1000 for t in timings]) for key in timings[0]}
            planned_timings = [t for t, r in zip(timings, runs) if r["status"] != "NO_ROAD"]
            entry = {"scenario": scenario, "rate_hz": rate, "frames": len(jobs),
                "status_counts": dict(Counter(r["status"] for r in runs)), "timing_ms": stage_ms,
                "unique_frame_status_counts": dict(Counter(r["status"] for r in runs if r["repeat"] == 0)),
                "planned_timing_ms": {key: stats([t[key]*1000 for t in planned_timings]) for key in timings[0]} if planned_timings else {},
                "estimation_ms": stats([r["estimation_s"]*1000 for r in runs]),
                "geometry_plus_planning_ms": stats([(r["estimation_s"]+r["planning_s"])*1000 for r in runs]),
                "over_requested_period": sum(r["planning_s"] > 1/rate for r in runs), "runs": runs}
            report["cases"].append(entry)
            print(f'{scenario} @{rate:g} Hz: {entry["status_counts"]}; mean {stage_ms["total"]["mean"]:.2f} ms / p95 {stage_ms["total"]["p95"]:.2f} ms', flush=True)
    if args.before_source and args.before_config:
        before_ns = load(args.before_source)
        before_config = yaml.safe_load(args.before_config.read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
        before = before_ns["ShadowLatticePlanner"](before_ns["ShadowSettings"](**before_config["shadow"]))
        after = ns["ShadowLatticePlanner"](ns["ShadowSettings"](**config["shadow"]))
        # Instrument only the current prototype's removed method, not older lattice code.
        execution = before_ns["ShadowLatticePlanner"]._execution
        meter = {"seconds": 0.0}
        def measure_execution(self, *params, **kwargs):
            started = time.perf_counter()
            try:
                return execution(self, *params, **kwargs)
            finally:
                meter["seconds"] += time.perf_counter()-started
        before_ns["ShadowLatticePlanner"]._execution = measure_execution
        old_times, new_times, execution_times = [], [], []
        before.plan(scenarios["both"][0][0]); after.plan(scenarios["both"][0][0])
        for record, _, _, _ in scenarios["both"]:
            meter["seconds"] = 0.0
            for planner, values in ((before, old_times), (after, new_times)):
                started = time.perf_counter(); planner.plan(record); values.append((time.perf_counter()-started)*1000)
            execution_times.append(meter["seconds"]*1000)
        report["paired_before_after_ms"] = {"before_source_sha256": hashlib.sha256(args.before_source.read_bytes()).hexdigest(),
            "before": stats(old_times), "after": stats(new_times), "removed_execution": stats(execution_times),
            "mean_speedup": statistics.mean(old_times)/statistics.mean(new_times)}
    process = subprocess.Popen([sys.executable, str(source), "--shadow-worker"], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8")
    wire_runs = []
    try:
        for record, _, _, _ in scenarios["left_only"][:8]:
            record = {**record, "settings": config["shadow"], "submitted_monotonic_s": time.monotonic()}
            started = time.perf_counter()
            process.stdin.write(json.dumps(record)+"\n");process.stdin.flush()
            line = process.stdout.readline()
            if not line:
                raise RuntimeError(process.stderr.read())
            received = time.perf_counter()
            result = json.loads(line)
            wire_runs.append({"roundtrip_ms": (time.perf_counter()-started)*1000,
                "parent_json_decode_ms": (time.perf_counter()-received)*1000,
                "planning_ms": result["elapsed_s"]*1000,
                "child_json_decode_ms": result["wire_timing_s"]["json_decode_s"]*1000,
                "child_json_encode_ms": result["wire_timing_s"]["json_encode_s"]*1000})
    finally:
        process.communicate(timeout=10)
    report["worker_ms"] = {"startup_first_job": wire_runs[0],
        "steady": {key: stats([r[key] for r in wire_runs[1:]]) for key in wire_runs[0]}}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "timing-single-side.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print('Saved '+str(args.output / "timing-single-side.json"), flush=True)


if __name__ == "__main__":
    main()
