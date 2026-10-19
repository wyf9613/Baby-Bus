"""Paired replay of recorded current inputs and partial historical references.

No ROS. No baseline control or archived lattice implementation is loaded.
Fixture provenance explicitly describes the approximate history alignment.
"""
import argparse
import ast
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real
from pathlib import Path
import statistics
import time

ROOT = Path(__file__).resolve().parents[2]


def load(source):
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = {"ShadowSettings", "ShadowFrame", "ShadowLatticePlanner", "finite_number", "wrap_angle"}
    body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
            and (n.name in names or n.name.startswith("_shadow_")) and n.name != "_shadow_worker_entry"]
    ns = dict(math=math, dataclass=dataclass, Real=Real, deepcopy=deepcopy, time=time)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(source), "exec"), ns)
    return ns


def prefix(path, length):
    result, distance = [path[0][:2]], 0.0
    for a, b in zip(path, path[1:]):
        step = math.dist(a[:2], b[:2])
        if distance+step >= length:
            u = (length-distance)/max(step, 1e-9)
            result.append(tuple(a[k]+u*(b[k]-a[k]) for k in (0, 1)))
            break
        result.append(b[:2])
        distance += step
    return result


def crossing(points, x):
    for a, b in zip(points, points[1:]):
        if min(a[0], b[0]) <= x <= max(a[0], b[0]) and abs(b[0]-a[0]) > 1e-9:
            u = (x-a[0])/(b[0]-a[0])
            return a[1]+u*(b[1]-a[1])
    return None


def statistics_of(values):
    values = sorted(values)
    return {"count": len(values), "mean": statistics.mean(values), "median": statistics.median(values),
            "maximum": values[-1]} if values else None


def evaluate(ns, fixture):
    results, previous = [], None
    for entry in fixture["frames"]:
        job = deepcopy(entry["job"])
        config = ns["ShadowSettings"](**job["settings"])
        raw = ns["ShadowLatticePlanner"](config)._plan_geometry(job)
        # The paired before output must reproduce the actual deployed planner.
        assert raw["status"] == entry["recorded_status"]
        assert raw["selected"]["id"] == entry["recorded_selected"]["id"]
        assert len(raw["selected"]["path"]) == len(entry["recorded_selected"]["path"])
        assert max(abs(a-b) for p, q in zip(raw["selected"]["path"], entry["recorded_selected"]["path"])
                   for a, b in zip(p, q)) < 1e-12
        job["previous_reference"] = previous
        fused = ns["ShadowLatticePlanner"](config).plan(job)
        if (fused.get("fusion") or {}).get("guide"):
            previous = fused["fusion"]["guide"]
        for side in ("left", "right"):
            assert fused["planning_input"]["road"].get(side+"_boundary_xy") == job["road"].get(side+"_boundary_xy")
        assert fused["planning_input"]["road"].get("boundary_source") == job["road"].get("boundary_source")
        if raw["status"] == "VALID":
            assert fused["status"] == "VALID", (job["job_id"], fused.get("fusion"))
        results.append({"job": job, "raw": raw, "fused": fused})
    return results


def metrics(ns, results, mode):
    frames = []
    for i, record in enumerate(results):
        r, job = record[mode], record["job"]
        if r["status"] != "VALID":
            continue
        p = prefix(r["selected"]["path"], r["check_length_m"])
        f = {"job_id": job["job_id"], "visibility": job["road"].get("visibility"),
            "max_abs_lateral_m": max(abs(v[1]) for v in p), "endpoint_xy_m": p[-1],
            "elapsed_s": r["elapsed_s"], "fusion": {k: (r.get("fusion") or {}).get(k)
                for k in ("status", "reason", "raw_fallback_used", "history_frames", "removed_guidance_loops")}}
        if i and job["shadow_odometry_epoch"] == results[i-1]["job"]["shadow_odometry_epoch"]:
            old = ns["_shadow_pose_points"](results[i-1][mode]["selected"]["path"],
                         results[i-1]["job"]["shadow_pose"], job["shadow_pose"])
            anchor = crossing(old, 0.0)
            differences = []
            for j in range(1, math.floor(max(q[0] for q in p)/0.025)+1):
                x = j*0.025
                a, b = crossing(p, x), crossing(old, x)
                if a is not None and b is not None:
                    differences.append(a-b)
            if differences:
                f["continuity_rms_m"] = math.sqrt(statistics.mean(d*d for d in differences))
                if anchor is not None:
                    f["shape_rms_m"] = math.sqrt(statistics.mean((d+anchor)**2 for d in differences))
        frames.append(f)
    summary = {"valid": len(frames), "total": len(results), "by_visibility": {}}
    for key in ("max_abs_lateral_m", "continuity_rms_m", "shape_rms_m", "elapsed_s"):
        summary[key] = statistics_of([f[key] for f in frames if key in f])
    for visibility in ("both", "left_only", "right_only"):
        fs = [f for f in frames if f["visibility"] == visibility]
        summary["by_visibility"][visibility] = {"valid": len(fs), **{
            k: statistics_of([f[k] for f in fs if k in f]) for k in ("max_abs_lateral_m", "continuity_rms_m", "shape_rms_m")}}
    return {"summary": summary, "frames": frames}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=ROOT/"tests/fixtures/shadow_temporal_recorded_run.json")
    parser.add_argument("--output", type=Path, default=ROOT/".verification/shadow-temporal/replay.json")
    args = parser.parse_args()
    source = ROOT/"scripts/policy_node.py"
    ns, fixture = load(source), json.loads(args.fixture.read_text(encoding="utf-8"))
    results = evaluate(ns, fixture)
    report = {"provenance": fixture["provenance"], "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "fixture_sha256": hashlib.sha256(args.fixture.read_bytes()).hexdigest(),
        "hard_boundaries_identical": True, "raw_outputs_match_recorded": True,
        "before": metrics(ns, results, "raw"), "after": metrics(ns, results, "fused")}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    for mode in ("before", "after"):
        print(mode, json.dumps(report[mode]["summary"], ensure_ascii=False))
    for frame in report["after"]["frames"]:
        print(frame["job_id"], frame["fusion"], "prefix_y", round(frame["max_abs_lateral_m"], 3))


if __name__ == "__main__":
    main()
