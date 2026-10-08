#!/usr/bin/env python3
"""MPC step-time measurement on the machine it runs on (G1 in docs/MPC_PROTOTYPE.md).

No ROS. Loads the actual MPC definitions from scripts/policy_node.py through
tests/test_mpc.py, closes the loop on the course prediction model and records
MPCController.step wall time for each horizon N. The first step of a run builds the
OSQP object and is reported separately; the controller's own budget check
(mpc.max_step_time_s) applies to every step, so the over-budget count includes it.

    python3 tools/mpc_platform_timing.py --out DIR [--horizons 10 8 5] [--repeats 3]

Exit status 0 only if every run finished, no step exceeded the budget at the default
horizon, and OSQP honoured time_limit. Plant and controller are simulated together on
one core: this measures computation, not sensor timing, executor scheduling or the
ROS node's other callbacks.
"""
import argparse
import json
import math
import os
from pathlib import Path
import platform
import runpy
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
T = runpy.run_path(str(ROOT / "tests" / "test_mpc.py"), run_name="mpc_platform_timing")
Settings, Solver, Stop = T["Settings"], T["Solver"], T["Stop"]
DEFAULT_N = Settings().horizon_n
BUDGET_S = Settings().max_step_time_s


def scenario_runs(horizon):
    """(name, controller, reference, z0, plant, steps): straight, new-car start from rest, S-bend."""
    measured, newcar, verified, plant_params = T["measured"], T["newcar"], T["verified"], T["plant_params"]
    runs = []
    vehicle = measured()
    runs.append(("straight_offset", verified(vehicle=vehicle, horizon_n=horizon),
                 T["straight_reference"](T["straight_world"]()), (0.0, 0.15, 0.0, 0.3, 0.0),
                 plant_params(vehicle), 200))
    car = newcar()
    runs.append(("newcar27_from_rest", verified(vehicle=car, horizon_n=horizon, v_exec_max_mps=0.3),
                 T["straight_reference"](T["straight_world"]()), (0.0, 0.1, 0.0, 0.0, 0.0),
                 plant_params(car), 200))
    ctrl = verified(vehicle=vehicle, horizon_n=horizon, reference_source="estimation_centerline",
                    bypass_acknowledged=True, bypass_target_speed_mps=0.3)
    path = T["world_path"]([(1.0, 0.0), (2.5, 0.4), (2.5, -0.4), (1.0, 0.0)])
    runs.append(("s_bend_bypass", ctrl, T["centerline_reference"](path, ctrl.cfg),
                 (0.0, 0.0, 0.0, 0.3, 0.0), plant_params(vehicle), 150))
    return runs


def quantile(sorted_values, q):
    return sorted_values[min(len(sorted_values)-1, int(q*len(sorted_values)))]


def stats_ms(values):
    s = sorted(values)
    return {"n": len(s), "mean_ms": statistics.mean(s)*1e3, "p50_ms": quantile(s, 0.50)*1e3,
            "p95_ms": quantile(s, 0.95)*1e3, "p99_ms": quantile(s, 0.99)*1e3, "max_ms": s[-1]*1e3}


def measure(horizon, repeats):
    result = {"horizon_n": horizon, "scenarios": {}}
    all_warm, all_steps = [], []
    for rep in range(repeats):
        for name, ctrl, reference, z0, plant, steps in scenario_runs(horizon):
            entry = result["scenarios"].setdefault(name, {"completed": 0, "stopped": [], "first_ms": [],
                                                          "warm": [], "branches": {}})
            try:
                log = T["run_plant"](ctrl, reference, z0, plant, steps)
            except Stop as exc:
                entry["stopped"].append(f"repeat {rep}: {exc}")
                continue
            times = [row["step_s"] for row in log if row["step_s"] is not None]
            entry["completed"] += 1
            entry["first_ms"].append(times[0]*1e3)
            entry["warm"].extend(times[1:])
            all_warm.extend(times[1:])
            all_steps.extend(times)
            for row in log:
                entry["branches"][row["branch"]] = entry["branches"].get(row["branch"], 0) + 1
    for entry in result["scenarios"].values():
        warm = entry.pop("warm")
        entry["warm"] = stats_ms(warm) if warm else None
    result["warm"] = stats_ms(all_warm) if all_warm else None
    result["first_step_max_ms"] = max((v for e in result["scenarios"].values() for v in e["first_ms"]),
                                      default=None)
    result["over_budget"] = sum(t > BUDGET_S for t in all_steps)
    result["steps"] = len(all_steps)
    result["runs_stopped"] = sum(len(e["stopped"]) for e in result["scenarios"].values())
    return result


def time_limit_check():
    """OSQP must stop at time_limit instead of running to max_iter (the guard if a solve stalls)."""
    traj = T["straight_trajectory"](y=0.1)
    out = {}
    for label, limit in (("tiny_limit", 1e-6), ("default_limit", Settings().solver_time_limit_s)):
        solver = Solver(Settings(solver_time_limit_s=limit, solver_max_iter=1_000_000), T["measured"]())
        t0 = time.perf_counter()
        try:
            solver.solve(traj["points"], [0.0, 0.0, 0.0, 0.3, 0.0], 0.3, 0.3)
            status = "solved"
        except Stop as exc:
            status = str(exc)
        out[label] = {"limit_s": limit, "status": status, "wall_ms": (time.perf_counter()-t0)*1e3}
    tiny = out["tiny_limit"]["status"].lower()
    out["time_limit_effective"] = "time" in tiny and "limit" in tiny
    out["default_solves"] = out["default_limit"]["status"] == "solved"
    return out


def platform_info():
    import numpy
    import osqp
    import scipy
    info = {"python": sys.version.split()[0], "executable": sys.executable, "machine": platform.machine(),
            "platform": platform.platform(), "cpu_count": os.cpu_count(), "numpy": numpy.__version__,
            "scipy": scipy.__version__, "osqp": getattr(osqp, "__version__", "unknown")}
    model = Path("/proc/device-tree/model")
    if model.exists():
        info["device_model"] = model.read_bytes().decode(errors="replace").strip("\x00\n ")
    if hasattr(os, "getloadavg"):
        info["loadavg_1_5_15"] = os.getloadavg()
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--horizons", type=int, nargs="+", default=[10, 8, 5])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--out", type=Path, help="directory for mpc-timing.json")
    args = parser.parse_args()

    report = {"platform": platform_info(), "budget_ms": BUDGET_S*1e3, "default_horizon_n": DEFAULT_N,
              "horizons": [], "time_limit": time_limit_check()}
    print("Platform: " + ", ".join(f"{k} {v}" for k, v in report["platform"].items() if k != "executable"))
    for horizon in args.horizons:
        report["horizons"].append(measure(horizon, args.repeats))

    print(f"\nMPC step time on this machine (budget {BUDGET_S*1e3:.0f} ms, {args.repeats} repeats per scenario)")
    print(f"{'N':>3} {'steps':>6} {'mean':>8} {'p95':>8} {'p99':>8} {'max':>8} {'first':>8} {'over':>5} {'stops':>6}")
    for h in report["horizons"]:
        w = h["warm"] or {k: math.nan for k in ("mean_ms", "p95_ms", "p99_ms", "max_ms")}
        first = h["first_step_max_ms"] if h["first_step_max_ms"] is not None else math.nan
        print(f"{h['horizon_n']:>3} {h['steps']:>6} {w['mean_ms']:>7.2f}  {w['p95_ms']:>7.2f}  {w['p99_ms']:>7.2f}  "
              f"{w['max_ms']:>7.2f}  {first:>7.2f}  {h['over_budget']:>4}  {h['runs_stopped']:>5}")
    print("(ms; mean/p95/p99/max exclude each run's first step, which builds OSQP; 'first' is the worst of those;"
          " 'over' counts all steps above budget)")
    for h in report["horizons"]:
        for name, e in h["scenarios"].items():
            if e["stopped"]:
                print(f"N={h['horizon_n']} {name}: stopped {e['stopped']}")
    tl = report["time_limit"]
    print(f"\nOSQP time_limit: tiny limit -> {tl['tiny_limit']['status']} ({tl['tiny_limit']['wall_ms']:.2f} ms); "
          f"default {tl['default_limit']['limit_s']*1e3:.0f} ms -> {tl['default_limit']['status']} "
          f"({tl['default_limit']['wall_ms']:.2f} ms)")

    default = next((h for h in report["horizons"] if h["horizon_n"] == DEFAULT_N), None)
    checks = {"all_runs_finished": all(h["runs_stopped"] == 0 for h in report["horizons"]),
              "default_horizon_within_budget": default is not None and default["over_budget"] == 0,
              "time_limit_effective": tl["time_limit_effective"], "default_limit_solves": tl["default_solves"]}
    report["checks"] = checks
    report["pass"] = all(checks.values())
    print("\nChecks: " + ", ".join(f"{k} {'OK' if v else 'FAIL'}" for k, v in checks.items()))
    print(f"mpc-platform-timing: {'PASS' if report['pass'] else 'FAIL'}")
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "mpc-timing.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
