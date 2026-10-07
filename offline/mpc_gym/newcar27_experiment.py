"""MVP vs joint MPC on the identified car .27 under the car's timing (offline only).

1. Calibration: the MVP with the 2026-10-06 car settings runs through NewcarPlant;
   its start delay, peak and steady speed are compared with run B12.
2. Matrix: both controllers, same seeds, over start pose, cone noise, deadband drift
   (battery), steering offset and camera gaps. A run completes at 3 m wheel distance.

Writes results/newcar27_comparison.json and results/newcar27_comparison.md.
These are simulation results for one fitted car; they are not car evidence.

  python offline/mpc_gym/newcar27_experiment.py [--quick]
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import statistics

import yaml

import pipeline_sim as sim

ROOT = sim.ROOT
RESULTS = Path(__file__).resolve().parent / "results"
KEY = "/**/ai4r_policy"
B12 = {"start_delay_s": 0.53, "peak_speed_mps": 0.333, "steady_speed_mps": 0.20}   # extended-2m.jsonl


def layered(*names):
    """Merge YAML files the way ROS applies several params files (later wins, per key)."""
    params = {}
    for name in names:
        layer = yaml.safe_load((ROOT / "config" / name).read_text(encoding="utf-8"))[KEY]["ros__parameters"]
        for key, value in layer.items():
            if isinstance(value, dict):
                params.setdefault(key, {}).update(value)
            else:
                params[key] = value
    return params


def mvp_params():
    return layered("ai4r_policy.yaml", "ai4r_policy_newcar27.yaml")


def mpc_params(**mpc):
    params = layered("ai4r_policy.yaml", "ai4r_policy_newcar27.yaml", "ai4r_policy_mpc_newcar27.yaml")
    # Simulation only: execute with the fitted record as if it had been measured on site.
    params["vehicle"].update(valid=True)
    params["mpc"].update(shadow=False, **mpc)
    return params


def run(params, plant_changes=None, **kwargs):
    plant = sim.NewcarPlant.from_fit(**(plant_changes or {}))
    conditions = dict(sim.NEWCAR27_CONDITIONS)
    conditions.update(kwargs.pop("conditions", {}))
    return sim.run(params, duration=kwargs.pop("duration", 25.0), distance_m=3.0, plant=plant,
                   **conditions, **kwargs)


def steady(result):
    trace = result["trace"]
    moving = [i for i, v in enumerate(trace["speed"]) if v > 0.05]
    tail = trace["speed"][moving[0]+40:] if moving else []
    return statistics.mean(tail) if tail else None


def calibration():
    result = run(mvp_params(), conditions={"gap_probability": 0.0}, offset=0.0, seed=1)
    return {"sim": {"start_delay_s": result["start_delay_s"], "peak_speed_mps": result["peak_speed_mps"],
                    "steady_speed_mps": round(steady(result), 3) if steady(result) else None,
                    "completed": result["completed"], "stop": result["locked_stop"]},
            "car_B12": B12}


def scenarios(quick):
    poses = [dict(offset=0.0, heading=0.0), dict(offset=0.15, heading=0.0), dict(offset=-0.15, heading=0.08)]
    noises = [0.02] if quick else [0.02, 0.05]
    seeds = [1] if quick else [1, 2, 3]
    deadbands = [0.27, 0.289, 0.305] if not quick else [0.289, 0.305]
    offsets = [0.0, 0.03] if quick else [0.0, 0.03, -0.03]
    for pose in poses:
        for noise in noises:
            for seed in seeds:
                yield dict(pose, noise=noise, seed=seed), {}
    for d0 in deadbands:
        yield dict(noise=0.02, seed=1), {"d0": d0}
    for offset in offsets:
        yield dict(noise=0.02, seed=1), {"steering_offset_rad": offset}
    yield dict(noise=0.02, seed=4, conditions={"gap_probability": 0.2}), {}


def summarise(rows):
    done = [r for r in rows if r["completed"]]
    rmse = [r["speed_rmse_mps"] for r in rows if r["speed_rmse_mps"] is not None]
    return {"runs": len(rows), "completed": len(done),
            "stops": sorted({r["locked_stop"]["reason"] for r in rows if r["locked_stop"]}),
            "lateral_max_m": max(r["lateral_max_m"] for r in rows),
            "lateral_rmse_mean_m": round(statistics.mean(r["lateral_rmse_m"] for r in rows), 3),
            "peak_speed_max_mps": max(r["peak_speed_mps"] for r in rows),
            "speed_rmse_mean_mps": round(statistics.mean(rmse), 3) if rmse else None,
            "drive_jumps_mean": round(statistics.mean(r["drive_jumps"] for r in rows), 1),
            "mpc_step_p95_ms_max": max((r.get("step_p95_ms") or 0.0) for r in rows)}


def matrix(quick, horizons):
    controllers = {"MVP": lambda: mvp_params()}
    for n in horizons:
        controllers[f"MPC N={n}"] = (lambda n=n: mpc_params(horizon_n=n))
    out = {}
    for name, make in controllers.items():
        rows = []
        for kwargs, plant in scenarios(quick):
            kwargs = deepcopy(kwargs)
            result = run(make(), plant, **kwargs)
            result.pop("trace")
            rows.append(dict(result, scenario={**{k: v for k, v in kwargs.items()}, **plant}))
        out[name] = {"summary": summarise(rows), "runs": rows}
        print(name, json.dumps(out[name]["summary"]))
    return out


def markdown(cal, table):
    lines = ["# Car .27 in simulation: MVP vs joint MPC", "",
             "Simulation only: Dream Gym geometry and cones; longitudinal response, delay, static friction and "
             "steering sign of car .27 imposed from `offline/vehicle_identification/results/newcar27_drive_fit.json`; "
             "20 Hz timer, 0.19-0.22 s camera latency, 5 % dropped cone batches. Not car evidence.", "",
             "## Calibration (MVP, car settings) against run B12", "",
             "| | start delay s | peak m/s | steady m/s |", "|---|---:|---:|---:|",
             f"| car B12 | {cal['car_B12']['start_delay_s']} | {cal['car_B12']['peak_speed_mps']} | "
             f"{cal['car_B12']['steady_speed_mps']} |",
             f"| simulation | {cal['sim']['start_delay_s']} | {cal['sim']['peak_speed_mps']} | "
             f"{cal['sim']['steady_speed_mps']} |", "",
             "## Matrix", "",
             "| controller | completed 3 m | stops | max lateral m | mean lateral RMSE m | peak m/s | "
             "mean speed RMSE m/s | drive jumps | MPC step p95 ms |",
             "|---|---:|---|---:|---:|---:|---:|---:|---:|"]
    for name, data in table.items():
        s = data["summary"]
        lines.append(f"| {name} | {s['completed']}/{s['runs']} | {', '.join(s['stops']) or '-'} | "
                     f"{s['lateral_max_m']} | {s['lateral_rmse_mean_m']} | {s['peak_speed_max_mps']} | "
                     f"{s['speed_rmse_mean_mps']} | {s['drive_jumps_mean']} | {s['mpc_step_p95_ms_max'] or '-'} |")
    lines += ["", "**Read the completion column with care:** the MVP locks on ANY rejected planning frame, while the "
              "MPC holds its last plan for up to 0.3 s. Most of the gap is that fault-handling difference "
              "(the same rejections ended every car run on 2026-10-06), not tracking quality. The simulation drops "
              "more cone batches than the car showed, so absolute distances are pessimistic for both.",
              "", "Drive jumps: steps whose requested drive changed by more than 0.05.",
              "Scenarios: start pose (0 / +0.15 m / -0.15 m with 0.08 rad), cone noise, deadband drift "
              "(battery), steering offset, 20 % dropped cone batches. Per-run rows are in the JSON."]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--horizons", type=int, nargs="+", default=[10])
    args = parser.parse_args()
    cal = calibration()
    print("calibration", json.dumps(cal))
    table = matrix(args.quick, args.horizons)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "newcar27_comparison.json").write_text(json.dumps({"calibration": cal, "matrix": table}, indent=1),
                                                       encoding="utf-8")
    (RESULTS / "newcar27_comparison.md").write_text(markdown(cal, table), encoding="utf-8")


if __name__ == "__main__":
    main()
