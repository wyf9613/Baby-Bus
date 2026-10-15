"""Audit and extract recorded cone runs without touching either checkout's files."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def git(*args):
    return subprocess.check_output(["git", "-c", f"safe.directory={ROOT.as_posix()}",
                                    "-C", str(ROOT), *args])


def audit(output, ref="origin/fix/newcar-control"):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    commit = git("rev-parse", ref).decode().strip()
    paths = git("ls-tree", "-r", "--name-only", commit).decode().splitlines()
    runs = []
    for path in paths:
        if not path.endswith(".jsonl") or "2026-10-14" not in path:
            continue
        data = git("show", f"{commit}:{path}")
        rows = [json.loads(line) for line in data.splitlines() if line.strip()]
        counts = Counter(r["kind"] for r in rows)
        cones = [r for r in rows if r["kind"] == "cones"]
        speeds = [r for r in rows if r["kind"] == "wheel"]
        gyro = [r for r in rows if r["kind"] == "gyro"]
        samples = np.array([[r["time_unix_s"], r["value"]] for r in speeds])
        distance = float(np.trapezoid(samples[:, 1], samples[:, 0])) if len(samples) else 0
        result_path = path.rsplit("/", 1)[0] + "/result.json"
        result = json.loads(git("show", f"{commit}:{result_path}")) if result_path in paths else None
        active = [r["time_unix_s"] for r in rows if r["kind"] == "policy_state" and r["value"] == 3]
        report = dict(path=path, commit=commit, sha256=hashlib.sha256(data).hexdigest(),
                      bytes=len(data), events=dict(counts), wheel_integral_m=distance,
                      cone_frames=len(cones), gyro_samples=len(gyro), result=result,
                      active_start_s=min(active) if active else None,
                      time_span_s=rows[-1]["time_unix_s"]-rows[0]["time_unix_s"],
                      cone_gaps_s=np.diff([r["time_unix_s"] for r in cones]).tolist(),
                      cone_latency_s=[r["value"]["latency_s"] for r in cones])
        target = output / path.rsplit("/", 2)[-2]
        target.mkdir(exist_ok=True)
        (target / "run.jsonl").write_bytes(data)
        (target / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        report["extracted_path"] = str(target / "run.jsonl")
        runs.append(report)
    (output / "audit.json").write_text(json.dumps(dict(source_ref=ref, commit=commit, runs=runs),
                                                  indent=2), encoding="utf-8")
    for run in runs:
        print(json.dumps({k:run[k] for k in ("path", "events", "wheel_integral_m", "cone_frames", "gyro_samples", "result")}, ensure_ascii=False))
    return runs


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ref", default="origin/fix/newcar-control")
    args = parser.parse_args()
    audit(args.output,args.ref)
