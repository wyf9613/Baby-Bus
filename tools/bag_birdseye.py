#!/usr/bin/env python3
"""Bird's-eye view of a car bag: dead-reckoned path, cone map, run events, MPC lines.

    python tools/bag_birdseye.py BAG.mcap [BAG2.mcap ...] [--out DIR]
        [--at T [T ...]]          # snapshots: reference, MPC prediction, lattice path at T
        [--gif T0 T1] [--fps 10]  # animation of [T0, T1], following the car

Times are seconds from the bag's first message, as in tools/export_car_run.py and the
mpc_debug timelines. Needs mcap, mcap-ros2-support, numpy and matplotlib (not ROS).

Outputs, per bag, in --out (default .verification/birdseye):
  <bag>_runs.png       one panel per policy run, each in its own frame (the car starts at the
                       origin heading +x; the car is carried back between runs, which dead
                       reckoning cannot see): path coloured by wheel speed, every cone
                       detection of that run within --cone-range of the car placed on the map
                       (clusters are cones; yellow/blue/other; camera depth noise shows as
                       radial streaks), and events: stops (with the reason), restarts
                       (tracking -> starting), soft/cut overspeed onsets, stall overrides,
                       lattice/centerline switches.
  <bag>_at_<T>.png     a 3 m window around the car at T, in the frame of the run around T
                       (or of the bag start): that frame's detections, the
                       accepted reference (/car/policy_trajectory), the MPC prediction
                       (/car/mpc_prediction) and the lattice path (/car/shadow_trajectory).
  <bag>_<T0>-<T1>.gif  the same window, frame by frame.

The pose is dead reckoning, not ground truth: x/y/yaw integrate the IMU yaw rate and the
wheel speed, so long runs drift by a few percent. The yaw rate is the gyro rotated into
base_link with /tf_static (base_link -> the IMU frame), as the policy node does; car 05's IMU
is mounted upside down (180 deg about y), so the raw z has the opposite sign. Without that
transform the sign is taken from gravity. The gyro bias comes from the stationary samples.
The wheel-speed driver holds its last reading for ~3 s after the wheels stop (2026-10-19);
a reading unchanged for 1.2 s (normal gaps are below 1.18 s), or for 0.5 s with a still
gyro and |steering| >= 0.3, counts as 0 m/s here so a stopped car does not keep moving.
Shorter holds still count, so distance is overstated around stalls (10-19 bags: cone
motion in the camera did not give a usable wheel-speed scale check because of them).
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONE_COLOURS = {1: "#d4a000", 2: "#1f5fd6"}          # ROS ConeDetection: yellow=1, blue=2
PATH_TOPICS = {"/car/policy_trajectory": "reference", "/car/mpc_prediction": "MPC prediction",
               "/car/shadow_trajectory": "lattice path"}
HELD_ANY_S, HELD_STILL_S, STILL_RPS, STALL_STEER = 1.2, 0.5, 0.02, 0.3


def read_bag(path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    d = {"imu": [], "wheel": [], "steer": [], "cones": [], "debug": [], "events": [],
         "paths": {t: [] for t in PATH_TOPICS}}
    gyro, mounts, imu_frame, gravity = [], {}, None, []
    t0 = None
    with open(path, "rb") as f:
        for _, channel, message, m in make_reader(f, decoder_factories=[DecoderFactory()]).iter_decoded_messages():
            t0 = message.log_time if t0 is None else t0
            t, topic = (message.log_time - t0)/1e9, channel.topic
            if topic == "/car/imu/data":
                w = m.angular_velocity
                gyro.append((t, (float(w.x), float(w.y), float(w.z))))
                imu_frame = m.header.frame_id
                if len(gravity) < 200:
                    gravity.append(float(m.linear_acceleration.z))
            elif topic in ("/tf_static", "/tf"):
                for tr in m.transforms:
                    q = tr.transform.rotation
                    mounts[tr.child_frame_id] = (tr.header.frame_id, (q.w, q.x, q.y, q.z))
            elif topic == "/car/wheel_speed_m_per_sec":
                d["wheel"].append((t, float(m.data)))
            elif topic == "/car/drive_and_steer_set_point_normalized":
                d["steer"].append((t, float(m.steer)))
            elif topic == "/car/cone_detections":
                seen = t - float(getattr(m, "acquisition_to_publish_latency_s", 0.0) or 0.0)
                d["cones"].append((seen, [(p.position.x, p.position.y, int(p.color)) for p in m.detections]))
            elif topic == "/car/mpc_debug":
                row = json.loads(m.data)
                row["t"] = t
                d["debug"].append(row)
            elif topic in ("/car/policy_fsm_state_string", "/car/traxxas_state"):
                d["events"].append((t, topic.rsplit("/", 1)[1], m.data))
            elif topic in PATH_TOPICS:
                stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
                # base_link at the reference time: shift by the stamp's lag behind the log time.
                lag = message.log_time/1e9 - stamp
                pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
                d["paths"][topic].append((t, t - lag if abs(lag) < 2.0 else t, pts))
    mount = mounts.get(imu_frame)
    if mount and mount[0] == "base_link":
        d["imu_mount"] = f"/tf_static base_link <- {imu_frame}"
        d["imu"] = [(t, rotate(mount[1], w)[2]) for t, w in gyro]
    else:
        sign = -1.0 if gravity and np.median(gravity) < 0 else 1.0     # z down: mounted upside down
        d["imu_mount"] = f"no transform for {imu_frame!r}; gravity sign {sign:+.0f}"
        d["imu"] = [(t, sign*w[2]) for t, w in gyro]
    return d


def rotate(q, v):
    """Rotate vector v by unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz - z*vy), 2*(z*vx - x*vz), 2*(x*vy - y*vx)
    return (vx + w*tx + y*tz - z*ty, vy + w*ty + z*tx - x*tz, vz + w*tz + x*ty - y*tx)


def dead_reckon(d):
    """Pose arrays (T, X, Y, YAW, V) on the IMU clock; V is the speed actually integrated."""
    imu = np.array(d["imu"]) if d["imu"] else np.zeros((0, 2))
    wheel, steer = d["wheel"], d["steer"]
    wt = [w[0] for w in wheel]
    st = [s[0] for s in steer]
    # Gyro bias from samples with the wheels at rest.
    rest = [z for t, z in d["imu"] if wheel and _at(wheel, wt, t) == 0.0]
    bias = float(np.median(rest)) if len(rest) > 50 else 0.0
    changed, last_value = [], None                  # time each wheel reading last changed
    for t, v in wheel:
        changed.append(t if v != last_value else changed[-1])
        last_value = v
    T, X, Y, YAW, V = [], [], [], [], []
    x = y = yaw = 0.0
    window = []                                      # (t, |yaw rate|) of the last HELD_STILL_S
    prev = None
    for t, z in (imu if len(imu) else []):
        rate = z - bias
        window.append((t, abs(rate)))
        while window and window[0][0] < t - HELD_STILL_S:
            window.pop(0)
        v = 0.0
        i = bisect.bisect_right(wt, t) - 1
        if i >= 0:
            v = wheel[i][1]
            held = t - changed[i]
            still = max(r for _, r in window) < STILL_RPS
            turning = abs(_at(steer, st, t) or 0.0) >= STALL_STEER
            if v > 0 and (held >= HELD_ANY_S or (held >= HELD_STILL_S and still and turning)):
                v = 0.0
        if prev is not None:
            h = t - prev
            if 0 < h < 0.2:
                yaw += rate*h
                x += v*math.cos(yaw)*h
                y += v*math.sin(yaw)*h
        prev = t
        T.append(t); X.append(x); Y.append(y); YAW.append(yaw); V.append(v)
    return {k: np.array(a) for k, a in zip("T X Y YAW V".split(), (T, X, Y, YAW, V))} | {"bias": bias}


def _at(series, times, t):
    i = bisect.bisect_right(times, t) - 1
    return series[i][1] if i >= 0 else None


def pose_at(pose, t):
    T = pose["T"]
    return tuple(float(np.interp(t, T, pose[k])) for k in ("X", "Y", "YAW"))


def to_world(pose, t, pts):
    x0, y0, yaw = pose_at(pose, t)
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x0 + c*px - s*py, y0 + s*px + c*py) for px, py in pts]


def rebased(pose, t0):
    """The pose arrays in the frame of the pose at t0 (origin, heading +x)."""
    x0, y0, yaw0 = pose_at(pose, t0)
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    dx, dy = pose["X"] - x0, pose["Y"] - y0
    return {**pose, "X": c*dx - s*dy, "Y": s*dx + c*dy, "YAW": pose["YAW"] - yaw0}


def runs(debug, min_s=1.0):
    """Policy runs from mpc_debug (a gap over 0.3 s or a stop starts a new one), >= min_s long."""
    out, cur = [], []
    for row in debug:
        if cur and (row["t"] - cur[-1]["t"] > 0.3 or cur[-1].get("branch") == "stop"):
            out.append(cur)
            cur = []
        cur.append(row)
    out += [cur] if cur else []
    return [r for r in out if r[-1]["t"] - r[0]["t"] >= min_s]


def events(debug):
    ev = []
    for a, b in zip(debug, debug[1:]):
        if b["t"] - a["t"] > 0.3:
            continue
        if a.get("phase") == "tracking" and b.get("phase") == "starting":
            ev.append((b["t"], "restart", ""))
        if (a.get("overspeed_state") or "ok") == "ok" and b.get("overspeed_state") in ("soft", "cut"):
            ev.append((b["t"], "overspeed", b["overspeed_state"]))
        if a.get("speed_override") != "stall" and b.get("speed_override") == "stall":
            ev.append((b["t"], "stall", ""))
        src_a, src_b = a.get("reference_selection"), b.get("reference_selection")
        if src_a and src_b and src_a != src_b:
            ev.append((b["t"], "switch", f"{src_a}->{src_b}"))
    for row in debug:
        if row.get("branch") == "stop":
            ev.append((row["t"], "stop", row.get("reject_reason") or ""))
    return ev


STYLE = {"stop": dict(marker="X", s=120, c="#d62728", zorder=6, label="stop"),
         "restart": dict(marker="^", s=60, c="#ff7f0e", zorder=5, label="restart (tracking -> starting)"),
         "overspeed": dict(marker="D", s=36, c="#c71585", zorder=5, label="soft/cut overspeed onset"),
         "stall": dict(marker="s", s=40, c="#8c564b", zorder=5, label="stall override"),
         "switch": dict(marker=".", s=30, c="#17becf", zorder=4, label="lattice/centerline switch")}


def cone_points(d, pose, cone_range, t_from=-math.inf, t_to=math.inf):
    pts = {}
    for t, dets in d["cones"]:
        if not t_from <= t <= t_to or not len(pose["T"]):
            continue
        near = [(x, y, c) for x, y, c in dets if math.hypot(x, y) <= cone_range]
        for (wx, wy), (_, _, c) in zip(to_world(pose, t, [(x, y) for x, y, _ in near]), near):
            pts.setdefault(c, []).append((wx, wy))
    return pts


def draw_cones(ax, pts, alpha, size):
    for c, xy in pts.items():
        a = np.array(xy)
        ax.scatter(a[:, 0], a[:, 1], s=size, c=CONE_COLOURS.get(c, "#888888"), alpha=alpha, linewidths=0,
                   zorder=2)


def run_panel(ax, d, pose, run, k, cone_range, marks, plt):
    from matplotlib.collections import LineCollection
    lo, hi = run[0]["t"], run[-1]["t"]
    local = rebased(pose, lo)
    draw_cones(ax, cone_points(d, local, cone_range, lo, hi), alpha=0.15, size=7)
    sel = (local["T"] >= lo) & (local["T"] <= hi)
    xy = np.column_stack([local["X"][sel], local["Y"][sel]])
    line = None
    if len(xy) > 1:
        line = LineCollection(np.stack([xy[:-1], xy[1:]], axis=1), cmap="viridis", norm=plt.Normalize(0, 0.4),
                              linewidths=2.5, zorder=4)
        line.set_array(local["V"][sel][1:])
        ax.add_collection(line)
    seen = set()
    for t, kind, text in events(run):
        x, y, _ = pose_at(local, t)
        style = dict(STYLE[kind])
        if kind in seen:
            style.pop("label")
        seen.add(kind)
        ax.scatter([x], [y], **style)
        if kind == "stop":
            ax.annotate(text, (x, y), xytext=(6, -12), textcoords="offset points", fontsize=7,
                        color="#d62728", zorder=7)
    for t in marks:
        if lo <= t <= hi:
            x, y, _ = pose_at(local, t)
            ax.annotate(f"{t:g}s", (x, y), xytext=(8, 8), textcoords="offset points", fontsize=8,
                        arrowprops=dict(arrowstyle="->", lw=0.8), zorder=7)
    dist = float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))) if len(xy) > 1 else 0.0
    end = run[-1]
    reason = end.get("reject_reason") if end.get("branch") == "stop" else "(no stop: run ended)"
    ax.set_title(f"R{k}: t {lo:.1f}-{hi:.1f} s, {dist:.1f} m, end: {reason}", fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    if seen:
        ax.legend(loc="best", fontsize=6)
    return line


def overview(d, pose, name, out, cone_range, marks=()):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rs = runs(d["debug"])
    if not rs:
        return False
    cols = min(3, len(rs))
    rows = math.ceil(len(rs)/cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5.5*cols, 5*rows + 0.8), squeeze=False)
    line = None
    for k, (ax, run) in enumerate(zip(axes.flat, rs), 1):
        line = run_panel(ax, d, pose, run, k, cone_range, marks, plt) or line
    for ax in list(axes.flat)[len(rs):]:
        ax.axis("off")
    if line is not None:
        fig.colorbar(line, ax=list(axes.flat), shrink=0.6, label="wheel speed (m/s)")
    fig.suptitle(f"{name}\none frame per run (car starts at the origin heading +x); dead reckoning "
                 f"(gyro bias {pose['bias']:+.4f} rad/s; {d.get('imu_mount')}), not ground truth", fontsize=9)
    fig.savefig(out, dpi=140)
    plt.close(fig)
    return True


def latest(series, t):
    """Last (log time, pose time, points) at or before t with points, or None."""
    best = None
    for item in series:
        if item[0] > t:
            break
        if item[2]:
            best = item
    return best


def frame(ax, d, pose, t, cone_cloud, half=3.0, trail_s=8.0):
    x, y, yaw = pose_at(pose, t)
    draw_cones(ax, cone_cloud, alpha=0.05, size=6)
    current = None
    for ct, dets in d["cones"]:
        if ct > t:
            break
        current = (ct, dets)
    if current:
        pts = to_world(pose, current[0], [(a, b) for a, b, _ in current[1]])
        for (wx, wy), (_, _, c) in zip(pts, current[1]):
            ax.scatter([wx], [wy], s=50, c=CONE_COLOURS.get(c, "#888888"), edgecolors="k", linewidths=0.5, zorder=5)
    sel = (pose["T"] >= t - trail_s) & (pose["T"] <= t)
    ax.plot(pose["X"][sel], pose["Y"][sel], color="#555555", lw=1.2, zorder=3)
    styles = {"/car/policy_trajectory": dict(color="#2ca02c", lw=2.0),
              "/car/mpc_prediction": dict(color="#d62728", lw=2.0, marker=".", ms=4),
              "/car/shadow_trajectory": dict(color="#9467bd", lw=1.5, ls="--")}
    for topic, label in PATH_TOPICS.items():
        item = latest(d["paths"][topic], t)
        if item and t - item[0] < 1.5:
            a = np.array(to_world(pose, item[1], item[2]))
            ax.plot(a[:, 0], a[:, 1], label=label, zorder=6, **styles[topic])
    ax.arrow(x, y, 0.25*math.cos(yaw), 0.25*math.sin(yaw), width=0.03, color="k", zorder=7)
    row = None
    for r in d["debug"]:
        if r["t"] > t:
            break
        row = r
    info = f"t {t:.1f} s"
    if row and t - row["t"] < 0.3:
        def num(k):
            v = row.get(k)
            return "-" if v is None else f"{v:.2f}"
        info += (f" | {row.get('branch')} {row.get('phase')} | v {num('speed_mps')} | drive {num('applied_drive')}"
                 f" steer {num('applied_steer_action')}\nref {row.get('reference_selection') or '-'}"
                 f" | e_y {num('e_y_m')} e_psi {num('e_psi_rad')} | overspeed {row.get('overspeed_state') or '-'}")
        if row.get("reject_reason"):
            info += f" | {row['reject_reason']}"
    ax.set_title(info, fontsize=8, loc="left")
    ax.set_xlim(x - half, x + half)
    ax.set_ylim(y - half, y + half)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=7, loc="lower left")


def run_frame(d, pose, t):
    """Pose rebased to the start of the run around t, and that run's time window."""
    for run in runs(d["debug"]):
        if run[0]["t"] - 0.5 <= t <= run[-1]["t"] + 2.0:
            return rebased(pose, run[0]["t"]), run[0]["t"], run[-1]["t"] + 2.0
    return pose, -math.inf, math.inf


def snapshot(d, pose, t, out, cone_range):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pose, lo, hi = run_frame(d, pose, t)
    fig, ax = plt.subplots(figsize=(7, 7))
    frame(ax, d, pose, t, cone_points(d, pose, cone_range, lo, hi))
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def animation(d, pose, t0, t1, fps, out, cone_range):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    pose, lo, hi = run_frame(d, pose, t0)
    cloud = cone_points(d, pose, cone_range, lo, hi)
    fig, ax = plt.subplots(figsize=(6, 6))
    times = np.arange(t0, t1, 1.0/fps)

    def update(t):
        ax.clear()
        frame(ax, d, pose, float(t), cloud)
    FuncAnimation(fig, update, frames=times).save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / ".verification" / "birdseye")
    parser.add_argument("--at", type=float, nargs="+", default=[], metavar="T")
    parser.add_argument("--gif", type=float, nargs=2, metavar=("T0", "T1"))
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--cone-range", type=float, default=2.0,
                        help="map only detections this close to the car (m); far ones are noisier")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    for bag in args.bags:
        files = sorted(bag.glob("*.mcap")) if bag.is_dir() else [bag]
        for path in files:
            name = path.stem.removesuffix("_0")
            d = read_bag(path)
            pose = dead_reckon(d)
            if not len(pose["T"]):
                print(f"{name}: no IMU data, skipped")
                continue
            target = args.out / f"{name}_runs.png"
            if overview(d, pose, name, target, args.cone_range, args.at):
                print(target)
            else:
                print(f"{name}: no policy run in /car/mpc_debug")
            for t in args.at:
                target = args.out / f"{name}_at_{t:g}.png"
                snapshot(d, pose, t, target, args.cone_range)
                print(target)
            if args.gif:
                target = args.out / f"{name}_{args.gif[0]:g}-{args.gif[1]:g}.gif"
                animation(d, pose, args.gif[0], args.gif[1], args.fps, target, args.cone_range)
                print(target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
