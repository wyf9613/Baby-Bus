"""Statistics of car 05 MPC exec runs from the compact per-tick fixtures (no ROS, no bags).

    python tools/car05_run_stats.py                       # all tests/fixtures/car05_20261019_runs/*.json.gz
    python tools/car05_run_stats.py path/to/run.json.gz   # one run

Fixtures come from the 2026-10-19 car bags (one row per /car/mpc_debug message, plus the
latest raw wheel speed, IMU yaw rate and set point at that tick). They are open-loop
records: use them for baselines and for fitting, not to judge a changed controller.
Sections: stops and restarts, cornering drive (steady drive by |steer|), the stall ->
restart -> overspeed cycle, reference switches (steering jump in each direction), and
full-lock reversals.
"""
import gzip
import json
from pathlib import Path
import statistics as st
import sys
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures" / "car05_20261019_runs"


def load(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def runs(ticks):
    """Split the ticks into policy runs: a gap over 0.3 s or a stop starts a new run."""
    out, cur = [], []
    for row in ticks:
        if cur and (row["t"]-cur[-1]["t"] > 0.3 or cur[-1]["branch"] == "stop"):
            out.append(cur)
            cur = []
        cur.append(row)
    if cur:
        out.append(cur)
    return out


def steer(row):
    return row.get("applied_steer_action") or 0.0


def report(doc):
    ticks = doc["ticks"]
    print(f"===== {doc['bag']}  ({len(ticks)} ticks)")
    for k, run in enumerate(runs(ticks), 1):
        moving = [r for r in run if r["branch"] in ("solved", "plan_hold")]
        if not moving:
            continue
        end = run[-1]
        dist = sum((a.get("speed_mps") or 0)*(b.get("dt") or 0) for a, b in zip(run, run[1:]))
        counted = [r["restarts"] for r in run if r.get("restarts") is not None]
        print(f"  run {k}: t={run[0]['t']:.1f}+{run[-1]['t']-run[0]['t']:.1f}s ~{dist:.2f} m, "
              f"end {end['branch']}:{end.get('reject_reason')}, "
              f"restarts {max(counted) if counted else 'n/a (field added in a5dad7f)'}")
    tr = [r for r in ticks if r["branch"] in ("solved", "plan_hold")]
    track = [r for r in tr if r["phase"] == "tracking"]

    # Restarts: tracking -> starting, and what preceded them.
    restarts = [(a, b) for a, b in zip(tr, tr[1:]) if a["phase"] == "tracking" and b["phase"] == "starting"]
    why = Counter("stall" if b.get("speed_override") == "stall" else "below moving speed" for _, b in restarts)
    switched = sum(1 for i, (a, b) in enumerate(zip(tr, tr[1:])) if a["phase"] == "tracking" and b["phase"] == "starting"
                   and len({r.get("reference_selection") for r in tr[max(0, i-20):i+1]}) > 1)
    print(f"  restarts {len(restarts)}: {dict(why)}; with a reference switch in the last 1 s: {switched}")

    # Cornering drive: ~0.4 s windows at 0.12-0.28 m/s, fresh readings, no overspeed action.
    print("  steady drive by |steer| (|accel| < 0.15 m/s^2) | decelerating windows:")
    fresh = [r for r in track if r["branch"] == "solved" and r.get("wheel_fresh") is not False]
    pts = []
    for a, b, c in zip(fresh, fresh[4:], fresh[8:]):
        if c["t"]-a["t"] > 0.6 or not 0.12 <= (b.get("speed_mps") or 0) <= 0.28 or b.get("overspeed_state") != "ok":
            continue
        acc = ((c.get("speed_mps") or 0)-(a.get("speed_mps") or 0))/(c["t"]-a["t"])
        win = [r for r in fresh if a["t"] <= r["t"] <= c["t"]]
        pts.append((st.mean(abs(steer(r)) for r in win), st.mean(r.get("applied_drive") or 0 for r in win), acc))
    for lo, hi in ((0, 0.15), (0.15, 0.35), (0.35, 0.6), (0.6, 0.85), (0.85, 1.01)):
        steady = [d for s, d, acc in pts if lo <= s < hi and abs(acc) < 0.15]
        slowing = [d for s, d, acc in pts if lo <= s < hi and acc < -0.15]
        if steady or slowing:
            print(f"    |steer| [{lo:.2f},{hi:.2f}): steady n={len(steady):3d} drive "
                  f"{st.median(steady) if steady else float('nan'):.3f} | slowing n={len(slowing):3d} drive "
                  f"{st.median(slowing) if slowing else float('nan'):.3f}")

    # Stop-go cycle: start -> tracking handovers followed by soft/cut overspeed within 2.5 s.
    hand = [(i, b) for i, (a, b) in enumerate(zip(tr, tr[1:])) if a["phase"] == "starting" and b["phase"] == "tracking"]
    over = sum(1 for i, b in hand if any(r.get("overspeed_state") in ("soft", "cut")
                                          for r in tr[i+1:i+60] if r["t"]-b["t"] <= 2.5))
    print(f"  start -> tracking handovers {len(hand)}, followed by soft/cut overspeed within 2.5 s: {over}")

    # Reference switches: steering change on the switching tick, each direction.
    for src, dst in (("planner", "baseline"), ("baseline", "planner")):
        jumps = [(abs(steer(b)-steer(a)), abs((b.get("e_y_m") or 0)-(a.get("e_y_m") or 0)))
                 for a, b in zip(track, track[1:]) if a.get("reference_selection") == src
                 and b.get("reference_selection") == dst and b["t"]-a["t"] < 0.12]
        if jumps:
            print(f"  {src} -> {dst}: {len(jumps)} switches, |dsteer| median {st.median(j[0] for j in jumps):.2f} "
                  f"max {max(j[0] for j in jumps):.2f}, |de_y| median {st.median(j[1] for j in jumps):.2f} "
                  f"max {max(j[1] for j in jumps):.2f}")

    # Steering: full-lock share and reversals within 1 s.
    last, rev = None, 0
    for r in track:
        s = steer(r)
        if abs(s) >= 0.99:
            sign = 1 if s > 0 else -1
            if last and last[1] != sign and r["t"]-last[0] < 1.0:
                rev += 1
            last = (r["t"], sign)
    if track:
        print(f"  full lock {sum(abs(steer(r)) >= 0.99 for r in track)/len(track):.0%} of tracking, "
              f"reversals within 1 s: {rev}")
    for src in ("planner", "baseline"):
        ey = sorted(abs(r["e_y_m"]) for r in track if r.get("reference_selection") == src and r.get("e_y_m") is not None)
        ep = sorted(abs(r["e_psi_rad"]) for r in track if r.get("reference_selection") == src and r.get("e_psi_rad") is not None)
        if ey:
            print(f"  {src:8s} |e_y| median {st.median(ey):.3f} p95 {ey[int(.95*len(ey))]:.3f}; "
                  f"|e_psi| median {st.median(ep):.3f} p95 {ep[int(.95*len(ep))]:.3f}")


def main(argv):
    paths = [Path(a) for a in argv] or sorted(FIXTURES.glob("*.json.gz"))
    for path in paths:
        report(load(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
