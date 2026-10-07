"""Fit the longitudinal drive model of one car from archived run telemetry.

Model (requested normalized drive u, wheel speed v >= 0, forward only):

    v' = a*(u - d0) - c*v*|v|        u = u(t - tau) >= d0   (drive)
    v' = e*(u - d0) - c*v*|v|        u < d0                 (ESC drag brake)
    stuck at v = 0 until u >= u_break has held for t_break

a = motor_gain_n/mass_kg (m/s^2 per unit above the deadband), c = drag_kg_per_m/mass
(1/m), e = braking slope below the deadband. Requests below d0 do not coast
freely on this car: B06 fell from 0.18 m/s to rest within ~0.3 s at u ~ 0.1. Units are the REQUESTED drive, not the vehicle
interface's applied value; the applied/requested ratio is reported separately.

Output-error fit: each run is simulated from its own request sequence and
compared with the logged wheel speed. No numerical derivative of the wheel
speed is taken: the topic is sample-and-hold and repeats values 0.1-0.2 s.
Only policy state 3 is used; after a stop the wheel topic holds its last
value for ~0.5 s and would bias a coast-down fit.

    python offline/vehicle_identification/fit_drive_from_logs.py
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "docs/experiments/2026-10-06/evidence/newcar-27-20261006/newcar-test-evidence"
# Runs with real motion on car .27 (see the experiment index). B08 used other
# speed gains, which makes it a useful check of the plant fit, not the controller.
# B06 (stop-and-go, before the sustaining compensation) is the only fit run whose
# request drops well below the deadband, so it carries the braking slope e.
FIT_RUNS = {"B06": "attempt6-confirmed-motion.jsonl", "B10": "compensated-curvature-stop.jsonl",
            "B11": "compensated-repeat.jsonl", "B12": "extended-2m.jsonl"}
CHECK_RUNS = {"B07": "main-3m-before-steering-fix.jsonl", "B08": "main-3m-steering-fix-attempt1.jsonl",
              "B14": "confidence85-coverage.jsonl"}
# Runs whose request never started the car: they bound the breakaway effort.
NO_MOTION_RUNS = {"B02": "attempt2.jsonl", "B03": "attempt3.jsonl"}
SIM_DT = 0.01
NAMES = ("a", "d0", "e", "c", "tau", "t_break")


def load_run(path):
    """Requested drive, applied drive and wheel speed during the active (state 3) window."""
    rows = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]
    state, start, end = None, None, None
    requests, applied, wheel = [], [], []
    for row in rows:
        kind, value, t = row["kind"], row["value"], row["elapsed_s"]
        if kind == "policy_state":
            if value == 3 and state != 3 and start is None:
                start = t
            if value != 3 and state == 3 and end is None:
                end = t
            state = value
        if start is None or (end is not None and t > end):
            continue
        if kind == "actions":
            requests.append((t, float(value["drive"])))
        elif kind == "applied":
            applied.append((t, float(value["drive"])))
        elif kind == "wheel":
            wheel.append((t, float(value)))
    if start is None:
        raise ValueError(f"{path}: no active window")
    end = end if end is not None else max(t for t, _ in wheel)
    return {"start": start, "end": end, "requests": requests, "applied": applied,
            "wheel": [(t, v) for t, v in wheel if t <= end]}


def held(series, t, default=0.0):
    """Zero-order hold of a (time, value) series."""
    value = default
    for ts, v in series:
        if ts > t:
            break
        value = v
    return value


def simulate(run, p, breakaway, sample_times):
    a, d0, e, c, tau, t_break = p
    u_break = breakaway[0]
    v, held_for, moving = 0.0, 0.0, False
    out, t = [], run["start"]
    times = iter(sample_times)
    next_sample = next(times, None)
    while next_sample is not None:
        while next_sample is not None and next_sample <= t + 1e-9:
            out.append(v)
            next_sample = next(times, None)
        u = held(run["requests"], t - tau)
        if not moving:
            held_for = held_for + SIM_DT if u >= u_break else 0.0
            moving = held_for >= t_break
        if moving:
            v = max(0.0, v + SIM_DT*((a if u >= d0 else e)*(u - d0) - c*v*abs(v)))
            # A stopped wheel with sub-breakaway effort sticks again.
            moving = v > 0.0 or u >= u_break
        t += SIM_DT
    return np.array(out)


def first_motion(run):
    first_request = next((t for t, u in run["requests"] if u > 0), None)
    first_wheel = next((t for t, v in run["wheel"] if v > 0.02), None)
    return first_request, first_wheel


def fit(runs, breakaway):
    data = [(run, np.array([t for t, _ in run["wheel"]]), np.array([v for _, v in run["wheel"]]))
            for run in runs.values()]

    def residual(p):
        return np.concatenate([simulate(run, p, breakaway, times) - speeds for run, times, speeds in data])
    # The logged first-start wait (~0.5 s) includes PI ramp-up to u_break; the fit
    # estimates the stiction wait once u_break is reached, shared by restarts.
    start = np.array([20.0, 0.28, 3.0, 2.0, 0.1, 0.2])
    result = least_squares(residual, start, bounds=([0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                                                    [100.0, 0.34, 50.0, 50.0, 0.4, 1.0]),
                           diff_step=1e-3)
    return result.x


def evaluate(runs, p, breakaway):
    report = {}
    for name, run in runs.items():
        times = np.array([t for t, _ in run["wheel"]])
        speeds = np.array([v for _, v in run["wheel"]])
        predicted = simulate(run, p, breakaway, times)
        request_t, wheel_t = first_motion(run)
        model_t = next((t for t, v in zip(times, predicted) if v > 0.02), None)
        report[name] = {
            "rmse_mps": float(np.sqrt(np.mean((predicted - speeds)**2))),
            "peak_measured_mps": float(speeds.max()), "peak_model_mps": float(predicted.max()),
            "start_delay_measured_s": None if wheel_t is None else round(wheel_t - request_t, 3),
            "start_delay_model_s": None if model_t is None or request_t is None else round(model_t - request_t, 3),
            "trace": [[round(float(t - run["start"]), 3), round(float(m), 4), round(float(s), 4)]
                      for t, m, s in zip(times, speeds, predicted)]}
    return report


def applied_ratio(runs):
    ratios = []
    for run in runs.values():
        for t, u in run["requests"]:
            if u > 0.1:
                applied = held(run["applied"], t + 0.15, None)
                if applied:
                    ratios.append(applied/u)
    return float(np.median(ratios)) if ratios else None


def breakaway_from_logs(moving, stuck):
    """Lowest request seen to start the car, highest seen not to, and the start wait."""
    started = [max(u for t, u in run["requests"] if t <= first_motion(run)[1]) for run in moving.values()]
    failed = [max(u for _, u in run["requests"]) for run in stuck.values()]
    waits = [first_motion(run)[1] - first_motion(run)[0] for run in moving.values()]
    return {"started_with": min(started), "did_not_start_with": max(failed),
            "start_wait_s": float(np.median(waits))}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path,
                        default=Path(__file__).resolve().parent / "results" / "newcar27_drive_fit.json")
    args = parser.parse_args()
    fit_runs = {k: load_run(EVIDENCE / v) for k, v in FIT_RUNS.items()}
    check_runs = {k: load_run(EVIDENCE / v) for k, v in CHECK_RUNS.items()}
    stuck_runs = {k: load_run(EVIDENCE / v) for k, v in NO_MOTION_RUNS.items()}
    observed = breakaway_from_logs({**fit_runs, **check_runs}, stuck_runs)
    # Breakaway threshold: halfway between the highest failed and lowest successful request.
    breakaway = ((observed["did_not_start_with"] + observed["started_with"])/2, observed["start_wait_s"])
    p = fit(fit_runs, breakaway)
    a, d0, e, c, tau, t_break = p
    steady = {f"{v:.2f}": round(float(d0 + c*v*v/a), 4) for v in (0.1, 0.15, 0.2, 0.25, 0.3)}
    result = {
        "car": "10.43.254.27", "date": "2026-10-06", "units": "requested normalized drive",
        "model": "v' = (a if u>=d0 else e)*(u(t-tau)-d0) - c*v|v|; stuck until u >= u_break held t_break",
        "params": dict(zip(NAMES, map(float, p))),
        "breakaway": {"u_break": breakaway[0], "t_break_s_fitted": float(t_break), **observed},
        "steady_state_drive_by_speed": steady,
        "slope_mps_per_unit_drive_at_0.2": round(float(a/(2*c*0.2)), 2),
        "applied_over_requested_median": applied_ratio({**fit_runs, **check_runs}),
        "fit": evaluate(fit_runs, p, breakaway), "check": evaluate(check_runs, p, breakaway),
        "caveats": ["wheel speed is sample-and-hold, not chassis displacement",
                    "one car, one day, one battery state: d0 drifts with battery",
                    "B03/B02 bound the breakaway from below only at 0.25/0.15"]}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1), encoding="utf-8")
    summary = {k: v for k, v in result.items() if k not in ("fit", "check")}
    summary["rmse"] = {name: round(r["rmse_mps"], 4) for part in ("fit", "check") for name, r in result[part].items()}
    summary["start_delay"] = {name: (r["start_delay_measured_s"], r["start_delay_model_s"])
                              for part in ("fit", "check") for name, r in result[part].items()}
    summary["peak"] = {name: (round(r["peak_measured_mps"], 3), round(r["peak_model_mps"], 3))
                       for part in ("fit", "check") for name, r in result[part].items()}
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
