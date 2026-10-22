#!/usr/bin/env python3
"""Per-run metrics from car bags, the same for every controller (MPC, PID, ...). No ROS.

    python tools/car_run_metrics.py BAG_DIR [BAG_DIR ...] [--cones auto|blue-left|yellow-left]
        [--label NAME] [--json-out FILE] [--csv-out FILE]

Why (2026-10-22): the comparisons PID vs MPC, joint vs PI-MPC and parameter A/Bs need one
definition of "how did this run go", computed from topics every policy publishes. Until now
each car day had its own one-off script.

Inputs (all recorded by tools/record_car_bag.sh): /car/policy_fsm_state_string (run split),
/car/drive_and_steer_applied_normalized, /car/wheel_speed_m_per_sec, /car/imu/data (+ /tf_static
for the IMU mounting), /car/cone_detections, /car/traxxas_state; /car/mpc_debug only adds the
MPC stop reason and the lattice share when present.

A run is "Publishing policy actions" up to the next "Publishing zero actions". Per run:
  - end: MPC stop reason if any, "manual_disable" when the vehicle went Disabled within 0.3 s
    before the policy stopped, otherwise the state text;
  - distance (wheel speed integral), heading (body yaw rate integral; IMU mounting from
    /tf_static, default 180 deg about y as on cars .27/05/01) and bends of >= 30 deg;
  - speed mean while moving, p95, max, seconds above 0.30 and 0.40 m/s;
  - steering: share at full lock (|s| >= 0.95), mean |ds/dt|, full-swing reversals (-0.3 <-> +0.3);
  - cones (camera frame relative to the car, no ground truth): offset from the centre of the
    nearest left/right cone pair at 0.15-1.0 m ahead, the smallest gap to each boundary within
    0.5 m ahead, "touch" (gap < 0.10 m) and "cross" (a boundary cone on the wrong side of the car
    within 0.5 m ahead and 0.6 m sideways) events, each in >= 2 consecutive frames, merged
    within 0.5 s. A cross is evidence of leaving the road, not a measurement of how far.
--cones auto takes, per run, the colour whose nearby cones sit further left at the run start.
Needs mcap and mcap-ros2-support (pip), as tools/bag_birdseye.py.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics

FULL_LOCK = 0.95
SWING = 0.3
TOUCH_GAP_M = 0.10
NEAR_X_M = (-0.1, 0.5)
NEAR_Y_M = 0.6
PAIR_X_M = (0.15, 1.0)
EVENT_MERGE_S = 0.5
BEND_DEG = 30.0
YELLOW, BLUE = 1, 2
DEFAULT_IMU_MOUNT = (0.0, 1.0, 0.0, 0.0)     # base_link <- fsm300_sensor (xyzw)
TOPICS = ("/car/policy_fsm_state_string", "/car/drive_and_steer_applied_normalized",
          "/car/wheel_speed_m_per_sec", "/car/imu/data", "/car/cone_detections",
          "/car/traxxas_state", "/car/mpc_debug", "/tf_static")


def rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def load(bag):
    """Time series (seconds from the bag's first message) of the topics above."""
    from mcap_ros2.reader import read_ros2_messages
    bag = Path(bag)
    files = [bag] if bag.suffix == ".mcap" else sorted(bag.glob("*.mcap"))
    if not files:
        raise SystemExit(f"{bag}: no .mcap file")
    s = {k: [] for k in ("state", "steer", "speed", "yaw_raw", "cones", "vehicle", "mpc")}
    imu_frame, mounts, t0 = None, {}, None
    for f in files:
        for m in read_ros2_messages(str(f), topics=list(TOPICS)):
            t = m.log_time_ns/1e9
            t0 = t if t0 is None else t0
            t -= t0
            topic, msg = m.channel.topic, m.ros_msg
            if topic == "/car/policy_fsm_state_string":
                s["state"].append((t, msg.data))
            elif topic == "/car/drive_and_steer_applied_normalized":
                s["steer"].append((t, msg.steer))
            elif topic == "/car/wheel_speed_m_per_sec":
                s["speed"].append((t, msg.data))
            elif topic == "/car/imu/data":
                imu_frame = msg.header.frame_id
                v = msg.angular_velocity
                s["yaw_raw"].append((t, (v.x, v.y, v.z)))
            elif topic == "/car/cone_detections":
                s["cones"].append((t, [(c.position.x, c.position.y, c.color) for c in msg.detections]))
            elif topic == "/car/traxxas_state":
                s["vehicle"].append((t, msg.data))
            elif topic == "/car/mpc_debug":
                s["mpc"].append((t, json.loads(msg.data)))
            elif topic == "/tf_static":
                for tr in msg.transforms:
                    r = tr.transform.rotation
                    mounts[tr.child_frame_id] = (r.x, r.y, r.z, r.w)
    mount = mounts.get(imu_frame)
    s["imu_mount"] = mount or DEFAULT_IMU_MOUNT
    s["imu_mount_source"] = "tf_static" if mount else "default (no /tf_static for the IMU frame)"
    s["yaw"] = [(t, rotate(s["imu_mount"], v)[2]) for t, v in s.pop("yaw_raw")]
    return s


def split_runs(states):
    """[(start, end, end_text)] from the policy state strings."""
    runs, start = [], None
    for t, text in states:
        if text.startswith("Publishing policy actions") and start is None:
            start = t
        elif text.startswith("Publishing zero actions") and start is not None:
            runs.append((start, t, text))
            start = None
    if start is not None:
        runs.append((start, states[-1][0], "bag ended"))
    return runs


def window(series, t0, t1):
    times = [t for t, _ in series]
    return series[bisect.bisect_left(times, t0):bisect.bisect_right(times, t1)]


def integrate(series, t0, t1):
    """[(t, cumulative integral)] of a sampled rate (rectangle rule, gaps over 0.5 s skipped)."""
    out, total, last = [], 0.0, None
    for t, v in window(series, t0, t1):
        if last is not None and t-last <= 0.5:
            total += v*(t-last)
        out.append((t, total))
        last = t
    return out


def bends(heading, min_deg=BEND_DEG, max_s=6.0, back_deg=5.0, flat_s=2.0):
    """Turns of at least min_deg (reached within max_s): [(start, end, degrees)].

    A turn ends at its extreme heading, once the heading has come back by back_deg or has not
    gone further for flat_s, so heading noise inside one bend does not split it.
    """
    out, i = [], 0
    while i < len(heading):
        t0, h0 = heading[i]
        first = next(((k, h) for k, (t, h) in enumerate(heading[i:], i)
                      if t-t0 <= max_s and abs(math.degrees(h-h0)) >= min_deg), None)
        if first is None:
            i += 1
            continue
        sign = 1 if first[1] > h0 else -1
        k_ext = first[0]
        for k in range(first[0]+1, len(heading)):
            t, h = heading[k]
            if sign*(h-heading[k_ext][1]) > 0:
                k_ext = k
            elif (math.degrees(sign*(heading[k_ext][1]-h)) >= back_deg
                    or t-heading[k_ext][0] >= flat_s):
                break
        out.append((t0, heading[k_ext][0], math.degrees(heading[k_ext][1]-h0)))
        i = k_ext+1
    return out


def left_colour_at_start(cones, t0, seconds=1.0):
    ys = {YELLOW: [], BLUE: []}
    for _, frame in window(cones, t0, t0+seconds):
        for x, y, c in frame:
            if 0.0 < x < 1.5 and c in ys:
                ys[c].append(y)
    if not ys[YELLOW] or not ys[BLUE]:
        return None
    return YELLOW if statistics.median(ys[YELLOW]) > statistics.median(ys[BLUE]) else BLUE


def cone_events(cones, t0, t1, left):
    """Lateral offset samples, boundary gaps and debounced touch/cross events in [t0, t1]."""
    right = BLUE if left == YELLOW else YELLOW
    offsets, gaps = [], {"left": [], "right": []}
    flags = []          # (t, set of flags in this frame)
    for t, frame in window(cones, t0, t1):
        near = [(x, y, c) for x, y, c in frame if NEAR_X_M[0] <= x <= NEAR_X_M[1] and abs(y) <= NEAR_Y_M]
        lgap = min((y for x, y, c in near if c == left), default=None)
        rgap = min((-y for x, y, c in near if c == right), default=None)
        f = set()
        for side, gap in (("left", lgap), ("right", rgap)):
            if gap is not None:
                gaps[side].append(gap)
                if gap < 0:
                    f.add(f"cross_{side}")
                elif gap < TOUCH_GAP_M:
                    f.add(f"touch_{side}")
        flags.append((t, f))
        pair = [(x, y, c) for x, y, c in frame if PAIR_X_M[0] <= x <= PAIR_X_M[1]]
        yl = min(((x, y) for x, y, c in pair if c == left and y > 0), default=None)
        yr = min(((x, y) for x, y, c in pair if c == right and y < 0), default=None)
        if yl and yr:
            offsets.append(-(yl[1]+yr[1])/2)        # car left of the pair centre is positive
    events = []
    for kind in ("cross_left", "cross_right", "touch_left", "touch_right"):
        last_end, streak = None, []
        for t, f in flags:
            if kind in f:
                streak.append(t)
                continue
            if len(streak) >= 2:
                if last_end is not None and streak[0]-last_end <= EVENT_MERGE_S:
                    events[-1]["end"] = streak[-1]
                else:
                    events.append({"kind": kind, "start": streak[0], "end": streak[-1]})
                last_end = streak[-1]
            streak = []
        if len(streak) >= 2:
            events.append({"kind": kind, "start": streak[0], "end": streak[-1]})
    # a cross supersedes a touch of the same side at the same time
    crosses = [e for e in events if e["kind"].startswith("cross")]
    events = [e for e in events if e["kind"].startswith("cross") or not any(
        c["kind"][6:] == e["kind"][6:] and c["start"]-EVENT_MERGE_S <= e["start"] <= c["end"]+EVENT_MERGE_S
        for c in crosses)]
    return offsets, gaps, sorted(events, key=lambda e: e["start"])


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    return v[min(len(v)-1, int(round(q*(len(v)-1))))]


def run_metrics(s, t0, t1, end_text, cones_mode="auto"):
    speed = window(s["speed"], t0, t1)
    steer = window(s["steer"], t0, t1)
    distance = integrate(s["speed"], t0, t1)
    heading = integrate(s["yaw"], t0, t1)
    vehicle = window(s["vehicle"], t0-30, t1)
    enabled = any(v == "Enabled" for _, v in window(s["vehicle"], t0, t1)) or (
        bool(vehicle) and vehicle[-1][1] == "Enabled" and vehicle[-1][0] <= t0)
    mpc = window(s["mpc"], t0, t1+0.2)
    stop = next((d.get("reject_reason") for _, d in reversed(mpc) if d.get("branch") == "stop"), None)
    disabled_before = any(v == "Disabled" for t, v in window(s["vehicle"], t1-0.3, t1))
    end = stop or ("manual_disable" if disabled_before else end_text)
    if disabled_before and stop:
        end = f"{stop} (vehicle Disabled just before)"
    moving = [v for _, v in speed if v > 0.05]
    dt = lambda series: [(b[0]-a[0]) for a, b in zip(series, series[1:])]
    above = lambda limit: sum(d for (t, v), d in zip(speed, dt(speed)) if v > limit and d <= 0.5)
    rates = [abs(b[1]-a[1])/(b[0]-a[0]) for a, b in zip(steer, steer[1:]) if 0 < b[0]-a[0] <= 0.5]
    swings, side = 0, 0
    for _, v in steer:
        now = 1 if v >= SWING else (-1 if v <= -SWING else 0)
        if now and side and now != side:
            swings += 1
        side = now or side
    if cones_mode == "auto":
        left = left_colour_at_start(s["cones"], t0)
        cones_source = "auto" if left else "auto failed (one colour at start), assumed blue-left"
        left = left or BLUE
    else:
        left = BLUE if cones_mode == "blue-left" else YELLOW
        cones_source = cones_mode
    offsets, gaps, events = cone_events(s["cones"], t0, t1, left)
    hd = [math.degrees(h) for _, h in heading]
    row = {
        "start_s": round(t0, 2), "end_s": round(t1, 2), "duration_s": round(t1-t0, 2),
        "vehicle_enabled": enabled, "end": end,
        "distance_m": round(distance[-1][1], 2) if distance else 0.0,
        "heading_net_deg": round(hd[-1], 1) if hd else 0.0,
        "heading_min_deg": round(min(hd), 1) if hd else 0.0, "heading_max_deg": round(max(hd), 1) if hd else 0.0,
        "bends": [{"start_s": round(a, 2), "end_s": round(b, 2), "deg": round(d, 1)} for a, b, d in bends(heading)],
        "speed_mean_moving": round(statistics.mean(moving), 3) if moving else None,
        "speed_p95": round(percentile([v for _, v in speed], 0.95), 3) if speed else None,
        "speed_max": round(max(v for _, v in speed), 3) if speed else None,
        "s_above_0_30": round(above(0.30), 2), "s_above_0_40": round(above(0.40), 2),
        "full_lock_share": round(sum(1 for _, v in steer if abs(v) >= FULL_LOCK)/len(steer), 3) if steer else None,
        "steer_rate_mean_per_s": round(statistics.mean(rates), 3) if rates else None,
        "steer_swings": swings,
        "left_colour": left, "cones_source": cones_source,
        "offset_p50_m": round(percentile([abs(o) for o in offsets], 0.5), 3) if offsets else None,
        "offset_p95_m": round(percentile([abs(o) for o in offsets], 0.95), 3) if offsets else None,
        "offset_samples": len(offsets),
        "min_gap_left_m": round(min(gaps["left"]), 3) if gaps["left"] else None,
        "min_gap_right_m": round(min(gaps["right"]), 3) if gaps["right"] else None,
        "crosses": sum(1 for e in events if e["kind"].startswith("cross")),
        "touches": sum(1 for e in events if e["kind"].startswith("touch")),
        "events": [{**e, "start": round(e["start"], 2), "end": round(e["end"], 2)} for e in events],
    }
    if mpc:
        sel = [d.get("reference_selection") for _, d in mpc if d.get("reference_selection")]
        modes = {d.get("longitudinal_mode") for _, d in mpc if d.get("longitudinal_mode")}
        row.update(mpc_stop=stop, lattice_share=round(sel.count("planner")/len(sel), 3) if sel else None,
                   longitudinal_mode=",".join(sorted(modes)) or None)
    return row


def summarise(bag, label, rows):
    real = [r for r in rows if r["vehicle_enabled"] and r["distance_m"] >= 0.05]
    print(f"\n== {label or Path(bag).name}: {len(rows)} runs, {len(real)} driven")
    for i, r in enumerate(rows, 1):
        if r not in real:
            why = "vehicle not enabled" if not r["vehicle_enabled"] else "did not move"
            print(f"  run {i:2d} t={r['start_s']:7.2f}  ({why}; end: {r['end']})")
            continue
        bends_txt = ", ".join(f"{b['deg']:+.0f}" for b in r["bends"]) or "-"
        print(f"  run {i:2d} t={r['start_s']:7.2f} {r['duration_s']:5.1f}s {r['distance_m']:5.2f} m  "
              f"heading {r['heading_min_deg']:+.0f}..{r['heading_max_deg']:+.0f} bends [{bends_txt}]  "
              f"v mean {r['speed_mean_moving']} max {r['speed_max']} >0.40 {r['s_above_0_40']}s  "
              f"full-lock {r['full_lock_share']:.0%} swings {r['steer_swings']}  "
              f"offset p50/p95 {r['offset_p50_m']}/{r['offset_p95_m']} m  "
              f"cross {r['crosses']} touch {r['touches']}  end: {r['end']}"
              + (f"  lattice {r['lattice_share']:.0%}" if r.get("lattice_share") is not None else ""))
        for e in r["events"]:
            if e["kind"].startswith("cross"):
                print(f"         {e['kind']} {e['start']:.2f}-{e['end']:.2f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--cones", default="auto", choices=["auto", "blue-left", "yellow-left"])
    parser.add_argument("--label", action="append", default=[], help="one per bag, in order")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--csv-out", type=Path)
    args = parser.parse_args()
    results = []
    for k, bag in enumerate(args.bags):
        s = load(bag)
        label = args.label[k] if k < len(args.label) else None
        rows = [run_metrics(s, a, b, text, args.cones) for a, b, text in split_runs(s["state"])]
        summarise(bag, label, rows)
        print(f"  IMU mounting: {s['imu_mount_source']}")
        results.append({"bag": str(bag), "label": label, "runs": rows})
    if args.json_out:
        args.json_out.write_text(json.dumps(results, indent=1), encoding="utf-8")
    if args.csv_out:
        flat = [{"bag": Path(r["bag"]).name, "label": r["label"], "run": i, **{k: v for k, v in row.items()
                 if k not in ("bends", "events")}, "bends_deg": " ".join(f"{b['deg']:+.0f}" for b in row["bends"])}
                for r in results for i, row in enumerate(r["runs"], 1)]
        if flat:
            keys = list(dict.fromkeys(k for row in flat for k in row))
            with args.csv_out.open("w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(flat)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
