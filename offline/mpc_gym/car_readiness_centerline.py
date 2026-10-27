"""Car-readiness matrix for the road_centerline/ordered_xy MPC port, in Dream Gym.

    python offline/mpc_gym/car_readiness_centerline.py                 # full matrix
    python offline/mpc_gym/car_readiness_centerline.py --quick         # 3 seeds, straight + right_angle_r1

Same car .27 conditions as offline/mpc_gym/car_readiness.py, but on
config/ai4r_policy_mpc_centerline.yaml's stack (estimation.road_geometry:
ordered_xy, planning.algorithm: road_centerline) instead of Planning V2
(lattice_v2) -- the two are mutually exclusive (PolicyNode.__init__ now
rejects lattice_v2 with ordered_xy), so this is a separate script rather than
another car_readiness.py variant. Adds the right-angle/U-turn roads
(pipeline_sim.SYNTHETIC_BENDS) and the car15_v4 reconstruction
(offline/mpc_gym/real_track.py) to the straight/gentle_arc set
car_readiness.py already covers for V2. Simulation only; not car evidence.

MPC only, deliberately: this mpc-car-run branch's own PolicyController (the
MVP/PID path) does not understand the ORDERED_CENTERLINE path encoding
road_centerline produces (it raises unsupported_path_encoding on the first
control step) -- porting that support was out of scope for the MPC port (see
docs/MPC_PROGRESS_LOG.md, 2026-10-15). A PID baseline on the same roads needs
origin/main's own policy_node.py/control stack, run separately; this script
does not attempt that comparison.
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pipeline_sim as sim  # noqa: E402
from newcar27_experiment import layered, run as newcar27_run  # noqa: E402

RESULTS = HERE / "results"
# road -> (start progress m, distance m, lane widths m), same convention as
# car_readiness.ROADS. car15_v4's own measured width (real_tracks/car15_v4.json,
# ~0.93 m) is used instead of a fixed pair; start progress 0 since the
# reconstructed path begins wherever the recorded run itself started.
ROADS = {"straight": (0.0, 3.0, (0.72, 1.0)), "gentle_arc": (0.0, 5.0, (0.72, 1.0)),
         "right_angle_r1": (1.0, 4.0, (0.72, 1.0)), "right_angle_r1_5": (1.0, 4.5, (0.72, 1.0)),
         "u_turn": (1.0, 5.0, (0.72, 1.0)), "car15_v4": (0.0, 4.0, (None,))}
SEEDS = tuple(range(1, 11))
STARTS = {"centre": (0.0, 0.0), "left5cm": (0.05, 0.0), "head5deg": (0.0, 0.087)}


def centerline_params(car05_mpc=False, lattice21=False, speed=None):
    layers = ["ai4r_policy.yaml", "ai4r_policy_newcar27.yaml", "ai4r_policy_mpc_newcar27.yaml",
              "ai4r_policy_mpc_centerline.yaml"]
    if lattice21:
        # 2026-10-18: the MVP2.1 lattice (shadow.* + planner_control) feeding the MPC,
        # layered as make_deploy_yaml does for mpc-shadow-lattice21, but executing.
        layers.append("ai4r_policy_mpc_lattice21.yaml")
    params = layered(*layers)
    if car05_mpc:
        # 2026-10-18: car 05's mpc/control settings (cap 0.15, soft overspeed, start
        # timeout, restarts, run time) on the same .27 plant and vehicle record; the
        # car 05 vehicle.* substitution itself is not simulated here.
        car05 = layered("ai4r_policy_mpc_car05.yaml")
        params["mpc"].update(car05["mpc"])
        params["control"].update(car05["control"])
    if speed is not None:
        # 2026-10-19 speed sweep: the MPC cap and the reference speed together (the
        # lattice only supplies a path; its speed is planning.cruise_speed_mps).
        params["mpc"]["v_exec_max_mps"] = speed
        params["planning"]["cruise_speed_mps"] = speed
    params["vehicle"]["valid"] = True   # simulation only: see newcar27_experiment.mpc_params
    params["mpc"]["shadow"] = False
    return params


def one(job):
    road, lane, start_pose, seed, car05_mpc, lattice21, speed = job
    start, distance, _ = ROADS[road]
    offset, heading = STARTS[start_pose]
    conditions = {} if lane is None else {"lane_width_m": lane}
    result = newcar27_run(centerline_params(car05_mpc, lattice21, speed), noise=0.02, seed=seed, road=road,
                          start_progress_m=start, distance_m=distance, duration=12.0+distance/0.2, offset=offset,
                          heading=heading, conditions=conditions, lattice_delivery_s=0.05 if lattice21 else None)
    stop = result.get("locked_stop") or {}
    lateral = result["trace"]["lateral"]
    return {"road": road, "lane_width_m": lane, "start": start_pose, "seed": seed,
            "completed": bool(result["completed"]), "distance_m": result["distance_m"],
            "stop_reason": stop.get("reason"), "stop_planning": stop.get("planning"), "stop_road": stop.get("road"),
            "hard_stop": bool(stop.get("reason", "").startswith("stop:")),
            "lateral_max_m": round(max((abs(v) for v in lateral), default=0.0), 3),
            "peak_speed_mps": round(max(result["trace"]["speed"], default=0.0), 3),
            "elapsed_s": round(result["trace"]["t"][-1] - result["trace"]["t"][0], 2) if result["trace"]["t"] else None,
            "step_p95_ms": result.get("step_p95_ms"), "branches": result.get("branches"),
            "reject_reasons": result.get("reject_reasons"),
            "reference_selection": result.get("reference_selection"),
            "selection_reasons": result.get("selection_reasons"), "lattice_results": result.get("lattice_results"),
            "reference_switches": result.get("reference_switches"),
            **{k: result.get(k) for k in ("new_path_steer_jumps", "steer_jumps", "full_lock_ticks", "steer_ticks",
                                          "steer_swings", "s_above_0_40")}}


def summarise(rows):
    groups = {}
    for row in rows:
        key = (row["road"], row["lane_width_m"], row["start"])
        groups.setdefault(key, []).append(row)
    table = []
    for (road, lane, start), group in sorted(groups.items(), key=lambda kv: tuple(map(str, kv[0]))):
        done = [r for r in group if r["completed"]]
        reasons = Counter(r["stop_reason"] for r in group if not r["completed"])
        table.append({"road": road, "lane_width_m": lane, "start": start,
                      "completed": f"{len(done)}/{len(group)}",
                      "hard_stops": sum(r["hard_stop"] for r in group),
                      "lateral_max_m": max((r["lateral_max_m"] for r in done), default=None),
                      "median_distance_m": sorted(r["distance_m"] for r in group)[len(group)//2],
                      "peak_speed_max_mps": max(r.get("peak_speed_mps", 0.0) for r in group),
                      "stop_reasons": dict(reasons.most_common(3))})
    return table


def markdown(table, args):
    lines = ["# road_centerline/ordered_xy MPC readiness in Dream Gym", "",
             "Generated by `offline/mpc_gym/car_readiness_centerline.py`. MPC only (see the module "
             "docstring: this branch's own PID does not understand ORDERED_CENTERLINE). Car .27 "
             "conditions (fitted drive plant, 20 Hz timer, 0.19-0.22 s camera latency, 5 % dropped cone "
             "batches, 2 cm cone noise); config/ai4r_policy_mpc_centerline.yaml (estimation.road_geometry: "
             "ordered_xy, planning.algorithm: road_centerline), not Planning V2. Straight/gentle_arc runs "
             "cover 3/5 m; right_angle/u_turn cover 4-5 m (2 m run-in, 2 m run-out); car15_v4 is the "
             "reconstructed car15 path (offline/mpc_gym/real_track.py), its own full length. Simulation "
             "only, not car evidence.",
             "", f"Seeds {args.seeds[0]}-{args.seeds[-1]}."
             + (" **--lattice21**: the MVP2.1 lattice (ai4r_policy_mpc_lattice21.yaml) feeds the executing "
                "MPC, centerline as fallback; results visible after planning time + 0.05 s."
                if getattr(args, "lattice21", False) else "")
             + (" **--car05-mpc**: car 05's mpc/control sections on the .27 plant."
                if getattr(args, "car05_mpc", False) else "")
             + (f" **--speed {args.speed}**: mpc.v_exec_max_mps and planning.cruise_speed_mps."
                if getattr(args, "speed", None) is not None else ""), "",
             "| road | lane m | start | completed | hard stops | max lateral m (completed) "
             "| median distance m | peak m/s | main stop reasons |", "|---|---:|---|---:|---:|---:|---:|---:|---|"]
    for t in table:
        lines.append(f"| {t['road']} | {t['lane_width_m']} | {t['start']} "
                     f"| {t['completed']} "
                     f"| {t['hard_stops']} | {t['lateral_max_m']} | {t['median_distance_m']} | {t['peak_speed_max_mps']} | "
                     + ", ".join(f"{k} {v}" for k, v in t["stop_reasons"].items()) + " |")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--roads", nargs="+", default=list(ROADS), choices=list(ROADS))
    parser.add_argument("--seeds", nargs="+", type=int, default=list(SEEDS))
    parser.add_argument("--quick", action="store_true", help="3 seeds, straight + right_angle_r1")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--lattice21", action="store_true",
                        help="the MVP2.1 lattice feeds the MPC (config/ai4r_policy_mpc_lattice21.yaml)")
    parser.add_argument("--car05-mpc", action="store_true",
                        help="also apply config/ai4r_policy_mpc_car05.yaml's mpc/control sections")
    parser.add_argument("--speed", type=float, default=None,
                        help="override mpc.v_exec_max_mps and planning.cruise_speed_mps (speed sweep)")
    parser.add_argument("--out", default="car_readiness_centerline", help="results/<out>.json and .md")
    args = parser.parse_args()
    if args.quick:
        args.seeds, args.roads = [1, 2, 3], ["straight", "right_angle_r1"]
    jobs = [(r, lane, pose, s, args.car05_mpc, args.lattice21, args.speed) for r in args.roads
            for lane in ROADS[r][2] for pose in (STARTS if r == "straight" else ("centre",)) for s in args.seeds]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(one, jobs))
    table = summarise(rows)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / f"{args.out}.json").write_text(json.dumps({"rows": rows, "table": table}, indent=1),
                                               encoding="utf-8", newline="\n")
    text = markdown(table, args)
    (RESULTS / f"{args.out}.md").write_text(text, encoding="utf-8", newline="\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
