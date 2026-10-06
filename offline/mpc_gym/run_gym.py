"""MPC vs PID baseline in Dream Gym under the PID experiment's exact conditions.

Usage: python offline/mpc_gym/run_gym.py --tune       (then validates)
       python offline/mpc_gym/run_gym.py --validate   (uses results/selected_weights.json)

Tuning mirrors offline/control_pid/tune.py: the same training scenarios, the same
score formulas and a grid of the same size (27 longitudinal + 27 lateral). The
validation runs PID (its saved selected_gains.json) and MPC side by side on the
PID validation scenarios and seeds, plus labelled MPC-only studies.
"""
import argparse
import csv
from dataclasses import asdict
import hashlib
from importlib.metadata import version
import itertools
import json
import math
from pathlib import Path
import platform
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import harness
from harness import Gains, MPCAdapter, PIDAdapter, run, tune

HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "results"
PID_GAINS = harness.ROOT / "offline/control_pid/results/selected_gains.json"
SEEDS = [101, 202, 303]
Scenario = tune.Scenario
SPEED_SCENARIOS = [Scenario('speed_nominal', 'straight', initial_y=0, initial_heading=0),
                   Scenario('speed_mismatch', 'straight', initial_y=0, initial_heading=0,
                            motor_scale=0.75, drag_scale=1.4)]
VALIDATION = [Scenario('notebook_05', speed=0.5), Scenario('notebook_10', speed=1.0),
              Scenario('s_bend_05', 's_bend', speed=0.5, initial_y=-0.2, initial_heading=-0.15),
              Scenario('s_bend_12', 's_bend', speed=1.2, initial_y=0.2, initial_heading=0.15),
              Scenario('noisy_reference', 's_bend', reference_noise_m=0.015,
                       heading_noise_rad=0.015, speed_noise_mps=0.02),
              Scenario('plant_mismatch', 's_bend', motor_scale=0.7, drag_scale=1.5,
                       steering_scale=0.85, steering_offset_rad=math.radians(-3)),
              Scenario('command_delay_100ms', 's_bend', delay_steps=2),
              Scenario('dt_100ms', 's_bend', dt=0.1),
              Scenario('reference_dropout', 's_bend', dropout=True)]
LOW_SPEED = Scenario('notebook_03', speed=0.3)   # extra: speed range planned for the car


def speed_score(trials):
    """tune.tune() longitudinal score, unchanged."""
    return float(np.mean([100*(not t['complete']) + t['speed_rmse_mps']
                          + 2*t['steady_speed_rmse_mps'] + 0.01*t['drive_rate_rms_per_s']
                          + (0.2 if t['stop_time_s'] is None else 0.02*t['stop_time_s']) for t in trials]))


def lateral_score(trials):
    """tune.tune() lateral score, unchanged."""
    return float(np.mean([100*(not t['complete']) + t['lateral_rmse_m']
                          + 0.25*t['lateral_max_m'] + 0.005*t['steer_rate_rms_per_s'] for t in trials]))


def write_csv(path, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def tune_weights(api):
    rows, best, best_score = [], {}, math.inf
    for q_v, r_drive, r_ddrive in itertools.product([3.0, 10.0, 30.0], [0.3, 1.0, 3.0], [0.5, 2.0, 8.0]):
        weights = dict(q_v=q_v, r_drive=r_drive, r_ddrive=r_ddrive)
        trials = [run(MPCAdapter(api, weights), s, longitudinal=True)[0] for s in SPEED_SCENARIOS]
        score = speed_score(trials)
        rows.append({"stage": "speed", "score": score, **weights,
                     "mpc_stops": sum(t["mpc_stops"] for t in trials)})
        if score < best_score:
            best, best_score = weights, score
    print("Speed selection:", best, "score", best_score, flush=True)
    speed_best, best_score = dict(best), math.inf
    for q_ey, q_epsi, r_ddelta in itertools.product([10.0, 20.0, 40.0], [2.0, 5.0, 10.0], [0.5, 2.0, 8.0]):
        weights = dict(speed_best, q_ey=q_ey, q_epsi=q_epsi, r_ddelta=r_ddelta)
        trials = [run(MPCAdapter(api, weights), s)[0] for s in tune.TRAINING]
        score = lateral_score(trials)
        rows.append({"stage": "lateral", "score": score, **weights,
                     "mpc_stops": sum(t["mpc_stops"] for t in trials)})
        if score < best_score:
            best, best_score = weights, score
    print("Lateral selection:", best, "score", best_score, flush=True)
    write_csv(OUTPUT / "tuning_candidates.csv", rows)
    (OUTPUT / "selected_weights.json").write_text(json.dumps(best, indent=2) + "\n", encoding="utf-8")
    return best


def mean(rows, key):
    values = [r[key] for r in rows if r.get(key) is not None]
    return float(np.mean(values)) if values else None


def validate(api, weights):
    gains = Gains(**json.loads(PID_GAINS.read_text(encoding="utf-8")))
    controllers = [PIDAdapter(gains), MPCAdapter(api, weights)]
    rows, examples = [], {}
    for scenario in VALIDATION + [LOW_SPEED]:
        for adapter in controllers:
            for seed in SEEDS:
                row, trace = run(adapter, scenario, seed)
                rows.append(row)
                if seed == SEEDS[0]:
                    examples[(adapter.name, scenario.name)] = trace
                    write_csv(OUTPUT / f"{adapter.name}_{scenario.name}_trace.csv", trace)
        print("Validation:", scenario.name, flush=True)
    for adapter in controllers:
        row, trace = run(adapter, Scenario('speed_steps', 'straight', initial_y=0, initial_heading=0),
                         longitudinal=True)
        rows.append(row)
        examples[(adapter.name, "speed_steps")] = trace
        write_csv(OUTPUT / f"{adapter.name}_speed_steps_trace.csv", trace)
    # MPC-only studies, labelled separately from the matched comparison.
    studies = []
    delay = Scenario('command_delay_100ms', 's_bend', delay_steps=2)
    known = MPCAdapter(api, weights, delay_s=0.1, label="mpc_known_delay")
    studies += [run(known, delay, seed)[0] for seed in SEEDS]
    for n in (5, 20):
        variant = MPCAdapter(api, weights, horizon_n=n, label=f"mpc_N{n}")
        studies += [run(variant, s, seed)[0] for s in (VALIDATION[3], VALIDATION[5]) for seed in SEEDS]
    print("MPC studies done", flush=True)
    write_csv(OUTPUT / "validation_metrics.csv", rows)
    write_csv(OUTPUT / "mpc_studies.csv", studies)
    plot(examples)
    write_experiment(weights, gains)
    write_comparison(rows, studies, weights, gains)
    print("Complete:", sum(r["complete"] for r in rows), "/", len(rows), flush=True)


def plot(examples):
    fig, axes = plt.subplots(3, 2, figsize=(13, 11))
    for ax, name in zip(axes[0], ["s_bend_12", "plant_mismatch"]):
        for controller, style in (("pid", "-"), ("mpc", "--")):
            trace = examples[(controller, name)]
            ax.plot([r["time_s"] for r in trace], [r["lateral_error_m"] for r in trace], style,
                    label=controller)
        ax.set(title=f"{name}: cross-track error", xlabel="Time [s]", ylabel="m")
    for ax, name in zip(axes[1], ["s_bend_12", "plant_mismatch"]):
        for controller, style in (("pid", "-"), ("mpc", "--")):
            trace = examples[(controller, name)]
            ax.plot([r["time_s"] for r in trace], [r["steer"] for r in trace], style, label=controller)
        ax.set(title=f"{name}: normalized steering", xlabel="Time [s]")
    for controller, style in (("pid", "-"), ("mpc", "--")):
        trace = examples[(controller, "speed_steps")]
        axes[2, 0].plot([r["time_s"] for r in trace], [r["speed_mps"] for r in trace], style, label=controller)
        axes[2, 1].plot([r["time_s"] for r in trace], [r["drive"] for r in trace], style, label=controller)
    trace = examples[("pid", "speed_steps")]
    axes[2, 0].plot([r["time_s"] for r in trace], [r["target_speed_mps"] for r in trace], "k:", label="target")
    axes[2, 0].set(title="Speed steps and stop", xlabel="Time [s]", ylabel="m/s")
    axes[2, 1].set(title="Normalized drive", xlabel="Time [s]")
    for ax in axes.flat:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUTPUT / "comparison.png", dpi=140)
    plt.close(fig)


def write_experiment(weights, gains):
    files = [harness.SOURCE, Path(harness.__file__), Path(__file__),
             harness.ROOT / "offline/control_pid/tune.py", harness.ROOT / "offline/control_pid/controller.py"]
    metadata = {"dreamgym_version": version("dreamgym"), "osqp_version": version("osqp"),
                "python_version": platform.python_version(), "numpy_version": np.__version__,
                "source_sha256": {str(p.relative_to(harness.ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in files},
                "notebook_sha256": hashlib.sha256(tune.NOTEBOOK.read_bytes()).hexdigest(),
                "mpc_weights": weights, "pid_gains": asdict(gains),
                "validation_scenarios": [asdict(s) for s in VALIDATION + [LOW_SPEED]], "seeds": SEEDS,
                "training_scenarios": [asdict(s) for s in tune.TRAINING],
                "reference": "oracle road -> local cubic (tune.reference_for); MPC samples it to v0.1 points "
                             "and continues the end tangent straight to cover its horizon",
                "mpc_vehicle": "notebook simulator parameters relabelled valid for simulation only"}
    (OUTPUT / "experiment.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def write_comparison(rows, studies, weights, gains):
    def fmt(value, digits=4):
        return "—" if value is None else f"{value:.{digits}f}"
    lines = ["# MPC vs PID baseline in Dream Gym", "",
             "Generated by `offline/mpc_gym/run_gym.py`; simulation only. Both controllers run in the same",
             "process on the PID experiment's scenarios, oracle reference, noise seeds and metrics",
             "(offline/control_pid/tune.py); `test_mpc_gym.py` checks the shared loop reproduces it exactly.", "",
             f"- MPC weights (tuned on the PID training scenarios, same score and grid size): `{weights}`",
             f"- PID gains (PID subteam's selection, unchanged): speed {gains.speed_kp}/{gains.speed_ki}/"
             f"{gains.speed_kd}, lateral {gains.lateral_kp}/{gains.lateral_ki}/{gains.lateral_kd}, "
             f"heading {gains.heading_kp}", "",
             "## Matched comparison (mean over seeds 101/202/303; max for maximum offset)", "",
             "| Scenario | Ctrl | Done | Cross-track RMSE [m] | Max offset [m] | Steady speed RMSE [m/s] "
             "| Steer rate RMS [1/s] | Min clearance [m] | MPC step p95 [ms] | MPC stops |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for name in dict.fromkeys(r["scenario"] for r in rows):
        for controller in ("pid", "mpc"):
            group = [r for r in rows if r["scenario"] == name and r["controller"] == controller]
            if not group:
                continue
            p95 = max((r.get("mpc_step_p95_ms") or 0) for r in group) if controller == "mpc" else None
            stops = sum(r.get("mpc_stops", 0) for r in group) if controller == "mpc" else None
            lines.append(
                f"| {name} | {controller} | {sum(r['complete'] for r in group)}/{len(group)} | "
                f"{fmt(mean(group, 'lateral_rmse_m'))} | {fmt(max(r['lateral_max_m'] for r in group))} | "
                f"{fmt(mean(group, 'steady_speed_rmse_mps'))} | {fmt(mean(group, 'steer_rate_rms_per_s'), 3)} | "
                f"{fmt(min(r['min_clearance_m'] for r in group))} | {fmt(p95, 1)} | "
                f"{'—' if stops is None else stops} |")
    steps = {r["controller"]: r for r in rows if r["scenario"] == "speed_steps"}
    lines += ["", "## Speed steps and stop (straight road, 0.5 → 1.0 → 0.4 → 0 → 0.6 m/s)", "",
              "| Ctrl | Speed RMSE [m/s] | Steady speed RMSE [m/s] | Stop time [s] | Stop distance [m] |",
              "| --- | --- | --- | --- | --- |"]
    for controller in ("pid", "mpc"):
        r = steps[controller]
        lines.append(f"| {controller} | {fmt(r['speed_rmse_mps'])} | {fmt(r['steady_speed_rmse_mps'])} | "
                     f"{fmt(r['stop_time_s'], 2)} | {fmt(r['stop_distance_m'], 3)} |")
    lines += ["", "The MPC sends forward drive only (its model has no reverse latch), so it stops by coasting;",
              "the PID brakes with the simulator's negative-drive latch. A missing stop time means the speed",
              "did not fall to 0.03 m/s within the 5 s stop window.", "",
              "## MPC-only studies (not part of the matched comparison)", "",
              "| Study | Scenario | Done | Cross-track RMSE [m] | Max offset [m] | Steer rate RMS [1/s] "
              "| Step p95 [ms] |", "| --- | --- | --- | --- | --- | --- | --- |"]
    for key in dict.fromkeys((r["controller"], r["scenario"]) for r in studies):
        group = [r for r in studies if (r["controller"], r["scenario"]) == key]
        lines.append(f"| {key[0]} | {key[1]} | {sum(r['complete'] for r in group)}/{len(group)} | "
                     f"{fmt(mean(group, 'lateral_rmse_m'))} | {fmt(max(r['lateral_max_m'] for r in group))} | "
                     f"{fmt(mean(group, 'steer_rate_rms_per_s'), 3)} | "
                     f"{fmt(max(r['mpc_step_p95_ms'] for r in group), 1)} |")
    lines += ["", "`mpc_known_delay`: the 100 ms command delay is given to the MPC (steering/drive delay 0.1 s).",
              "`mpc_N5` / `mpc_N20`: prediction horizon 0.5 s / 2.0 s instead of 1.0 s.", "",
              "## Limits", "",
              "- The reference is the oracle road through the PID mock planner, not cone-based estimation/planning.",
              "- The MPC predicts with the simulator's own parameters; only the mismatch, delay, noise and dropout",
              "  scenarios test robustness. Nothing here is real-car evidence.",
              "- The MPC reference continues the end tangent straight to cover its horizon near the road end.",
              "- An MPC rejection is a locked stop on the car; here the next step rebuilds the controller (counted",
              "  as `MPC stops`), mirroring the PID's offline reset on an invalid reference.",
              "- Step times are laptop times, not Jetson times."]
    (OUTPUT / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tune", action="store_true")
    parser.add_argument("--validate", action="store_true")
    args = parser.parse_args()
    if not args.tune and not args.validate:
        parser.error("Choose --tune or --validate")
    if version("dreamgym") != "0.4.0":
        raise RuntimeError("This experiment requires the notebook pin dreamgym==0.4.0")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    api = harness.load_mpc()
    if args.tune:
        weights = tune_weights(api)
    else:
        weights = json.loads((OUTPUT / "selected_weights.json").read_text(encoding="utf-8"))
    validate(api, weights)
    print(f"Elapsed: {time.perf_counter()-started:.1f}s; output: {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
