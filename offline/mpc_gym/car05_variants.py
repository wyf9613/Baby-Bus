"""Car 05 MPC + lattice 2.1 in Dream Gym on car-05-like plants, with parameter overrides.

    python offline/mpc_gym/car05_variants.py                                   # committed car 05 config
    python offline/mpc_gym/car05_variants.py --plant-d0 0.31 --plant-ubreak 0.37   # used battery
    python offline/mpc_gym/car05_variants.py --set mpc.high_speed_mps=0.15 --set vehicle.drive_max=0.40
    python offline/mpc_gym/car05_variants.py --plant-turn-drag 0.06 --feedback   # PI drive (10-20)
    python offline/mpc_gym/car05_variants.py --set planning.lattice.steering_limit_rad=0.42

Builds on car_readiness_centerline.py (--lattice21 --car05-mpc at --speed 0.20), and also
takes car 05's own vehicle drive and steering record from config/ai4r_policy_mpc_car05.yaml
(car_readiness only applies its mpc/control sections). The plant is the .27 drive fit with
its deadband (d0) and start threshold (u_break) overridable -- 2026-10-19 used 0.23/0.24 for
a fresh battery, 0.289/0.298 (the fit) and 0.31/0.37 for a used one -- and steers like car
05 (--plant-steer-rad, default 0.42 rad = 24 deg at full lock; the Gym default is 45 deg).

Lattice planning time is measured on this computer, so a repeat differs by 1-3 completions
per 60 runs (seen 2026-10-19): compare variants on the same plant, and repeat when close.
--lattice-planning-s fixes it instead (2026-10-21; tools/replay_run_jsonl.py has the same
option), so variants differ only by their settings.
The shadow.* section is ai4r_policy_mpc_lattice21.yaml's (check_length_mode "travel"), not
car 05's: add --set shadow.check_length_mode=travel_plus_window for car 05's lattice.
--json-out writes every run's row (selection reasons, lattice statuses, lateral, speed).
--plant-turn-drag adds cornering drag (deadband + k*|steering|; 0 = none, as before 10-20).
--feedback layers config/ai4r_policy_mpc_car05_feedback.yaml (mpc.longitudinal_mode feedback).
Simulation only, not car evidence: the plant has no held wheel readings, which the 10-19
car runs showed (tools/car05_run_stats.py).
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import car_readiness_centerline as c  # noqa: E402
import newcar27_experiment as ne  # noqa: E402
import pipeline_sim as sim  # noqa: E402

ROADS = ("straight", "gentle_arc", "u_turn", "right_angle_r1_5", "right_angle_r1")


def parse_value(text):
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(text.lower(), text)


_CONFIGURED = False


def configure(args):
    """Patch the readiness script's parameter builder and plant, once per process."""
    global _CONFIGURED
    if _CONFIGURED:          # pool workers are reused: patching again would nest the wrappers
        return
    _CONFIGURED = True
    if args.bend_runout_m is not None:
        sim.SYNTHETIC_RUNOUT_M = args.bend_runout_m
    car05 = ne.layered("ai4r_policy_mpc_car05.yaml")
    feedback = ne.layered("ai4r_policy_mpc_car05_feedback.yaml")["mpc"] if args.feedback else {}
    overrides = [s.split("=", 1) for s in args.set]
    base = c.centerline_params

    def params(car05_mpc=False, lattice21=False, speed=None):
        p = base(car05_mpc, lattice21, speed)
        for key, value in car05["vehicle"].items():
            if key.startswith("drive") or key.startswith("steering"):
                p["vehicle"][key] = value
        p["mpc"].update(feedback)
        for path, value in overrides:          # section.name or section.sub.name
            *parents, name = path.split(".")
            target = p
            for key in parents:
                target = target[key]
            assert name in target, f"--set {path}: no such parameter"
            target[name] = parse_value(value)
        return p
    c.centerline_params = params

    action = sim.NewcarPlant.gym_action

    def gym_action(self, t, request, speed):
        drive, steer = action(self, t, request, speed)
        return drive, steer*args.plant_steer_rad/sim.GYM_STEER_RAD
    sim.NewcarPlant.gym_action = gym_action

    run = c.newcar27_run
    plant = {"d0": args.plant_d0, "u_break": args.plant_ubreak, "turn_drag": args.plant_turn_drag}
    c.newcar27_run = lambda p, **kw: run(p, plant_changes=plant, **kw)

    if args.lattice_planning_s is not None:
        import replay_run_jsonl as replay     # pipeline_sim imports enable_lattice from it per run
        enable = replay.enable_lattice
        replay.enable_lattice = lambda node, api, params, delivery_s, planning_s=None: enable(
            node, api, params, delivery_s, args.lattice_planning_s)


def one(job):
    args, road, lane, seed = job
    configure(args)
    return c.one((road, lane, "centre", seed, True, True, args.speed))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--plant-d0", type=float, default=0.23)
    parser.add_argument("--plant-ubreak", type=float, default=0.24)
    parser.add_argument("--plant-steer-rad", type=float, default=0.42)
    parser.add_argument("--plant-turn-drag", type=float, default=0.0)
    parser.add_argument("--feedback", action="store_true", help="mpc.longitudinal_mode feedback (car 05 PI overlay)")
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--set", action="append", default=[], metavar="section.name=value",
                        help="override a parameter after the car 05 layers (repeatable)")
    parser.add_argument("--roads", nargs="+", default=list(ROADS), choices=list(c.ROADS))
    parser.add_argument("--seeds", type=int, default=6, help="seeds 1..N per road and lane")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--lattice-planning-s", type=float, default=None,
                        help="fixed lattice planning time instead of this computer's (deterministic)")
    parser.add_argument("--bend-runout-m", type=float, default=None,
                        help="straight after the synthetic bends (default pipeline_sim's 2.0 m); "
                             "the run distance is unchanged, so this only moves the road's end")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    jobs = [(args, r, lane, s) for r in args.roads for lane in (0.72, 1.0) for s in range(1, args.seeds+1)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(one, jobs))
    done = sum(r["completed"] for r in rows)
    reasons = Counter(r["stop_reason"] for r in rows if not r["completed"])
    print(f"plant d0={args.plant_d0} u_break={args.plant_ubreak} steer={args.plant_steer_rad} "
          f"turn_drag={args.plant_turn_drag} {'feedback' if args.feedback else 'joint'} set={args.set or '-'}: {done}/{len(rows)} completed, peak {max(r['peak_speed_mps'] for r in rows):.2f} m/s, "
          f"overspeed {reasons.get('stop:overspeed', 0)}, no_motion {reasons.get('stop:no_motion_after_breakaway', 0)}, "
          f"too_many_restarts {reasons.get('stop:too_many_restarts', 0)}")
    for road in args.roads:
        group = [r for r in rows if r["road"] == road]
        lat = max((r["lateral_max_m"] for r in group if r["completed"]), default=None)
        sel = Counter()
        for r in group:
            sel.update(r.get("reference_selection") or {})
        share = sel.get("planner", 0)/max(1, sum(sel.values()))
        switches = sum(r.get("reference_switches") or 0 for r in group)
        print(f"  {road:17s} {sum(r['completed'] for r in group):2d}/{len(group)}  max lateral {lat}  "
              f"lattice {share:.0%} of ticks, {switches} switches")
    print("  stop reasons:", dict(reasons.most_common(6)))
    if args.json_out:
        args.json_out.write_text(json.dumps({"args": {k: str(v) for k, v in vars(args).items()},
                                             "rows": rows}, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
