"""Car-readiness matrix: the real estimator -> Planning V2 -> controller chain in Dream Gym.

    python offline/mpc_gym/car_readiness.py                    # full matrix, writes results/
    python offline/mpc_gym/car_readiness.py --quick            # 3 seeds, MPC only, straight + gentle_arc
    python offline/mpc_gym/car_readiness.py --roads straight --variants fixes --controllers mpc

Conditions are the car .27 ones from newcar27_experiment (fitted drive plant, 20 Hz timer,
0.19-0.22 s camera latency, 5 % dropped cone batches, 2 cm cone noise). Each run must cover
3 m (the car's control.max_distance_m); curve runs start 2-2.5 m before the bend and cover 5 m.

Variants: "baseline" = the shipped base YAML + car overlays with every upstream fix switched
OFF (current Estimation/Planning behaviour); "fixes" = the same with FIXES applied (the
mpc-car-run changes, see docs/UPSTREAM_FIXES_FOR_CAR_CN.md). Controllers: "mpc" (joint MPC,
execution mode, fitted vehicle record) and "mvp" (the Planning team's robust MVP control,
as an upstream reference). Simulation only; not car evidence.
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import newcar27_experiment as x  # noqa: E402

RESULTS = HERE / "results"
# road -> (start progress m, distance m, lane widths m). Straight runs cover the car's 3 m
# limit. Curve runs start ~2-2.5 m before the bend, so the cone history is built on the
# straight before the camera loses one side in the bend, and cover 5 m (the car overlay
# raises control.max_distance_m to 5 m for curve runs). The notebook course keeps 1.0 m.
ROADS = {"straight": (0.0, 3.0, (0.72, 1.0)), "gentle_arc": (0.0, 5.0, (0.72, 1.0)),
         "course": (7.5, 5.0, (1.0,)), "s_bend": (1.5, 5.0, (0.72, 1.0))}
SEEDS = tuple(range(1, 11))
# Start poses (lateral offset m, heading rad), as planned for the car: centred, 5 cm left,
# 5 degrees left. Curves are run from the centred start only.
STARTS = {"centre": (0.0, 0.0), "left5cm": (0.05, 0.0), "head5deg": (0.0, 0.087)}
# The upstream fixes, switched on together. Each entry is (section path, key, value);
# BASELINE holds the code defaults so "baseline" is explicit even if overlays change.
FIXES = [("estimation", "history_propagation", "continuous"),     # A1
         ("estimation", "road_history_mode", "distance"),           # A2
         ("estimation", "history_merge_radius_m", 0.08),            # A2b
         ("estimation", "history_merge_single_side", True),          # A3
         ("planning.lattice", "blind_extension_mode", "parallel"),   # B1
         ("planning.lattice", "max_near_gap_m", 1.0),                # B2
         ("planning.lattice", "max_extension_m", 1.4)]               # B2
# The car overlay enables FIXES, so the baseline resets every one to the code default.
BASELINE = [("estimation", "history_propagation", "camera"),
            ("estimation", "road_history_mode", "time"),
            ("estimation", "history_merge_radius_m", 0.0),
            ("estimation", "history_merge_single_side", False),
            ("planning.lattice", "blind_extension_mode", "fade"),
            ("planning.lattice", "max_near_gap_m", 0.9),
            ("planning.lattice", "max_extension_m", 1.3)]


def apply(params, changes):
    for path, key, value in changes:
        node = params
        for part in path.split("."):
            node = node.setdefault(part, {})
        node[key] = value
    return params


def params_for(controller, variant, extra=()):
    params = x.mpc_params() if controller == "mpc" else x.mvp_params()
    return apply(params, list(FIXES if variant == "fixes" else BASELINE)+list(extra))


def parse_extra(items):
    """--extra planning.lattice.recovery_enabled=false -> [("planning.lattice", "recovery_enabled", False)]."""
    import yaml
    out = []
    for item in items:
        key, _, value = item.partition("=")
        path, _, name = key.rpartition(".")
        if not path or not name:
            raise SystemExit(f"--extra needs section[.sub].key=value, got {item!r}")
        out.append((path, name, yaml.safe_load(value)))
    return out


def one(job):
    controller, variant, road, lane, start_pose, seed, extra = job
    start, distance, _ = ROADS[road]
    offset, heading = STARTS[start_pose]
    result = x.run(params_for(controller, variant, extra), noise=0.02, seed=seed, road=road, start_progress_m=start,
                   distance_m=distance, duration=12.0+distance/0.2, offset=offset, heading=heading,
                   conditions={"lane_width_m": lane})
    stop = result.get("locked_stop") or {}
    lateral = result["trace"]["lateral"]
    return {"controller": controller, "variant": variant, "road": road, "lane_width_m": lane,
            "start": start_pose, "seed": seed,
            "completed": bool(result["completed"]), "distance_m": result["distance_m"],
            "stop_reason": stop.get("reason"), "stop_planning": stop.get("planning"), "stop_road": stop.get("road"),
            "hard_stop": bool(stop.get("reason", "").startswith("stop:")),
            "lateral_max_m": round(max((abs(v) for v in lateral), default=0.0), 3),
            "step_p95_ms": result.get("step_p95_ms"), "branches": result.get("branches")}


def summarise(rows):
    groups = {}
    for row in rows:
        key = (row["controller"], row["variant"], row["road"], row["lane_width_m"], row["start"])
        groups.setdefault(key, []).append(row)
    table = []
    for (controller, variant, road, lane, start), group in sorted(groups.items()):
        done = [r for r in group if r["completed"]]
        reasons = Counter(r["stop_planning"] or r["stop_reason"] for r in group if not r["completed"])
        table.append({"controller": controller, "variant": variant, "road": road, "lane_width_m": lane,
                      "start": start,
                      "completed": f"{len(done)}/{len(group)}",
                      "hard_stops": sum(r["hard_stop"] for r in group),
                      "lateral_max_m": max((r["lateral_max_m"] for r in done), default=None),
                      "median_distance_m": sorted(r["distance_m"] for r in group)[len(group)//2],
                      "stop_reasons": dict(reasons.most_common(3))})
    return table


def markdown(table, args):
    lines = ["# Car readiness in Dream Gym", "",
             "Generated by `offline/mpc_gym/car_readiness.py`. Car .27 conditions (fitted drive plant, 20 Hz "
             "timer, 0.19-0.22 s camera latency, 5 % dropped cone batches, 2 cm cone noise); every run must "
             "cover 3 m (straight) or 5 m (curves, starting 2-2.5 m before the bend). Simulation only, "
             "not car evidence.", "",
             f"Seeds {args.seeds[0]}-{args.seeds[-1]}. `baseline` = current Estimation/Planning behaviour; "
             "`fixes` = the mpc-car-run switches (see docs/UPSTREAM_FIXES_FOR_CAR_CN.md).", "",
             "| controller | variant | road | lane m | start | completed | hard stops | max lateral m (completed) "
             "| median distance m | main stop reasons |", "|---|---|---|---:|---|---:|---:|---:|---:|---|"]
    for t in table:
        lines.append(f"| {t['controller']} | {t['variant']} | {t['road']} | {t['lane_width_m']} | {t['start']} "
                     f"| {t['completed']} "
                     f"| {t['hard_stops']} | {t['lateral_max_m']} | {t['median_distance_m']} | "
                     + ", ".join(f"{k} {v}" for k, v in t["stop_reasons"].items()) + " |")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--roads", nargs="+", default=list(ROADS), choices=list(ROADS))
    parser.add_argument("--variants", nargs="+", default=["baseline", "fixes"], choices=["baseline", "fixes"])
    parser.add_argument("--controllers", nargs="+", default=["mpc", "mvp"], choices=["mpc", "mvp"])
    parser.add_argument("--seeds", nargs="+", type=int, default=list(SEEDS))
    parser.add_argument("--quick", action="store_true", help="3 seeds, MPC, straight + gentle_arc")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--extra", action="append", default=[], metavar="SECTION.KEY=VALUE",
                        help="extra override on top of the variant, e.g. planning.lattice.recovery_enabled=false")
    parser.add_argument("--out", default="car_readiness", help="results/<out>.json and .md")
    args = parser.parse_args()
    if args.quick:
        args.seeds, args.controllers, args.roads = [1, 2, 3], ["mpc"], ["straight", "gentle_arc"]
    extra = parse_extra(args.extra)
    jobs = [(c, v, r, lane, pose, s, extra) for c in args.controllers for v in args.variants for r in args.roads
            for lane in ROADS[r][2] for pose in (STARTS if r == "straight" else ("centre",)) for s in args.seeds]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(one, jobs))
    table = summarise(rows)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / f"{args.out}.json").write_text(json.dumps({"fixes": FIXES, "extra": extra, "rows": rows,
                                                          "table": table}, indent=1), encoding="utf-8")
    text = markdown(table, args)
    (RESULTS / f"{args.out}.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
