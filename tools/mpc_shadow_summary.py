#!/usr/bin/env python3
"""Summarise MPC shadow (G2) recordings: one table row per scenario bag.

    python3 tools/mpc_shadow_summary.py ~/ai4r-car-bags/g2-*        # rosbag2 directories
    python3 tools/mpc_shadow_summary.py --jsonl run.jsonl          # one mpc_debug JSON per line

Reads /car/mpc_debug (std_msgs/String, the JSON published once per policy step).
Bags need the ROS environment (rosbag2_py); JSONL needs nothing. Prints, per input:
branch and reject-reason shares, median e_y / e_psi / candidate angle over valid
shadow samples, the sign check (candidate angle opposite to the error), the
candidate drive while pushed near 0.2 m/s, timing and the safety check that
nothing was applied. It reports; the scenario's tape and angle record decide.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
import sys

TOPIC = "/car/mpc_debug"


STORAGE_BY_SUFFIX = {".mcap": "mcap", ".db3": "sqlite3"}


def bag_uri(path):
    """(uri, storage_id). A bag whose recorder was killed has no metadata.yaml; then the
    single storage file is opened directly with the storage named by its suffix."""
    path = Path(path)
    if not path.exists():
        raise SystemExit(f"{path}: does not exist (was the recording started?)")
    if path.is_file():
        return str(path), STORAGE_BY_SUFFIX.get(path.suffix, "")
    if (path / "metadata.yaml").exists():
        return str(path), ""
    files = sorted(f for f in path.iterdir() if f.suffix in STORAGE_BY_SUFFIX)
    if len(files) != 1:
        raise SystemExit(f"{path}: no metadata.yaml and {len(files)} .mcap/.db3 files; cannot open")
    print(f"{path.name}: no metadata.yaml (recording not closed cleanly); reading {files[0].name}")
    return str(files[0]), STORAGE_BY_SUFFIX[files[0].suffix]


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    uri, storage_id = bag_uri(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        rows.append(json.loads(deserialize_message(data, String).data))
    return rows


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def values(rows, key):
    return [r[key] for r in rows if isinstance(r.get(key), (int, float)) and not isinstance(r.get(key), bool)]


def median(xs):
    return statistics.median(xs) if xs else None


def p95(xs):
    return sorted(xs)[min(len(xs)-1, int(0.95*len(xs)))] if xs else None


def fmt(x, scale=1.0, digits=3):
    return "-" if x is None else f"{x*scale:.{digits}f}"


def summarise(name, rows):
    n = len(rows)
    out = {"name": name, "steps": n}
    if not n:
        return out
    branches = Counter(r.get("branch") for r in rows)
    reasons = Counter(r.get("reject_reason") for r in rows if r.get("reject_reason"))
    valid = [r for r in rows if r.get("branch") == "shadow"]
    moving = [r for r in valid if (r.get("speed_mps") or 0.0) > 0.05]
    push = [r for r in moving if 0.15 <= r["speed_mps"] <= 0.25]
    applied = sum(1 for r in rows if (r.get("applied_drive") or 0.0) != 0.0 or (r.get("applied_steer_action") or 0.0) != 0.0)
    signs = [(r["e_y_m"], r["e_psi_rad"], r["candidate_delta_rad"]) for r in valid
             if all(isinstance(r.get(k), (int, float)) for k in ("e_y_m", "e_psi_rad", "candidate_delta_rad"))]
    # Opposite sign to the dominant error (lateral error weighted like a 1 m look-ahead heading error).
    corrective = [d*(ey + epsi) < 0 for ey, epsi, d in signs if abs(ey + epsi) > 0.02 and abs(d) > 1e-3]
    out.update(
        valid_share=len(valid)/n, moving_share=len(moving)/n, branches=dict(branches), reasons=dict(reasons.most_common(5)),
        e_y=median(values(valid, "e_y_m")), e_psi=median(values(valid, "e_psi_rad")),
        delta=median(values(valid, "candidate_delta_rad")), steer=median(values(valid, "candidate_steer_action")),
        corrective_share=(sum(corrective)/len(corrective)) if corrective else None, corrective_n=len(corrective),
        drive_push=median(values(push, "candidate_drive")), push_n=len(push),
        step_p95=p95(values(rows, "step_s")), step_max=max(values(rows, "step_s"), default=None),
        solve_p95=p95(values(rows, "solve_s")), dt_median=median(values(rows, "dt")), dt_max=max(values(rows, "dt"), default=None),
        ref_age_max=max(values(rows, "ref_age_s"), default=None), applied_nonzero=applied,
        phases=dict(Counter(r.get("phase") for r in rows)),
        sources={k: sorted({str(r.get(k)) for r in rows}) for k in
                 ("vehicle_source", "reference_source", "simulation_only", "planning_bypassed", "shadow")})
    return out


def report(results):
    print(f"{'scenario':<22} {'steps':>5} {'valid':>6} {'moving':>6} {'e_y m':>7} {'e_psi':>7} {'delta':>7} "
          f"{'fix%':>5} {'drive@.2':>8} {'step95':>7} {'stepmax':>7} {'dtmax':>6} {'applied':>7}")
    for r in results:
        if not r["steps"]:
            print(f"{r['name']:<22} {0:>5}  no {TOPIC} messages")
            continue
        fix = "-" if r["corrective_share"] is None else f"{100*r['corrective_share']:.0f}"
        print(f"{r['name']:<22} {r['steps']:>5} {100*r['valid_share']:>5.0f}% {100*r['moving_share']:>5.0f}% "
              f"{fmt(r['e_y']):>7} {fmt(r['e_psi']):>7} {fmt(r['delta']):>7} {fix:>5} {fmt(r['drive_push']):>8} "
              f"{fmt(r['step_p95'], 1e3, 1):>7} {fmt(r['step_max'], 1e3, 1):>7} {fmt(r['dt_max'], 1e3, 0):>6} "
              f"{r['applied_nonzero']:>7}")
    print("(valid = branch shadow; moving = valid with speed > 0.05 m/s; e_y, e_psi, delta = medians over valid "
          "samples in m / rad; fix% = share of valid samples whose candidate angle opposes the error; drive@.2 = "
          "median candidate drive at 0.15-0.25 m/s; step in ms; applied must be 0)")
    for r in results:
        if not r["steps"]:
            continue
        print(f"\n{r['name']}: branches {r['branches']}; reject reasons {r['reasons'] or 'none'}; phases {r['phases']}")
        print(f"  sources {r['sources']}; ref_age max {fmt(r['ref_age_max'])} s; dt median "
              f"{fmt(r['dt_median'], 1e3, 0)} ms; solve p95 {fmt(r['solve_p95'], 1e3, 1)} ms; "
              f"push samples {r['push_n']}; sign samples {r['corrective_n']}")
    problems = []
    for r in results:
        if not r["steps"]:
            problems.append(f"{r['name']}: no messages")
            continue
        if r["applied_nonzero"]:
            problems.append(f"{r['name']}: {r['applied_nonzero']} steps applied a nonzero action in shadow")
        bad = {b: c for b, c in r["branches"].items() if b in ("solver_fail", "over_budget")}
        if bad:
            problems.append(f"{r['name']}: {bad}")
        if r["sources"]["shadow"] != ["True"]:
            problems.append(f"{r['name']}: shadow flag {r['sources']['shadow']}")
    print("\nSafety and solver checks: " + ("OK" if not problems else "; ".join(problems)))
    return not problems


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("inputs", nargs="+", type=Path, help="rosbag2 directories, or JSONL files with --jsonl")
    parser.add_argument("--jsonl", action="store_true", help="inputs are JSONL files of mpc_debug messages")
    parser.add_argument("--json-out", type=Path, help="also write the summaries as JSON")
    args = parser.parse_args()
    results = [summarise(p.name, read_jsonl(p) if args.jsonl else read_bag(p)) for p in args.inputs]
    ok = report(results)
    if args.json_out:
        args.json_out.write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
