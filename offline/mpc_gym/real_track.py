"""Reconstruct a Dream Gym road_spec's geometry/cone statistics from a recorded
run.jsonl (dead-reckoned path + observed cone positions), for a closed-loop
Gym comparison on the actual bend the car drove.

    python offline/mpc_gym/real_track.py docs/experiments/2026-10-14/evidence/car15/visualization4/run.jsonl \
        --name car15_v4 --out offline/mpc_gym/real_tracks/car15_v4.json

Why: dream-gym's road_core schema (dreamgym/envs/road_core/{schema,cone_schema}.py) only
places cones PARAMETRICALLY -- geometry elements (straight/circular_arc) plus a sampled
inter-cone spacing/lateral-displacement distribution per cone path. There is no "literal
list of absolute cone coordinates" placement mode, so a real run's cone positions cannot
be dropped into Gym as-is; the closest faithful reconstruction is to fit the car's OWN path
into the same straight/circular_arc elements the schema uses, and measure the real lane
width and cone spacing to calibrate the (still synthetic, still regularly sampled) cone
placement. This is deliberately not the "road generated from scratch" the user called "too
regular": the geometry comes from where the car actually turned, not a hand-picked spec.

How:
  1. Dead-reckon (x, y, yaw) from the run's own "wheel"/"gyro" events (simple Euler
     integration at each gyro sample's own dt; no estimator, no cone input at all).
  2. Signed curvature along that path (headed change / arc length, smoothed over a short
     window), segmented into straight (|curvature| below --straight-curvature-1pm) and
     circular_arc runs (near-constant curvature), each converted to a geometry element.
  3. Per "cones" event, body-frame detections -> world frame via the path pose nearest that
     event's own stamp_s; left/right cones (by ROS colour id) matched by nearest-y to the
     path, giving a lane-width sample per event and inter-cone spacing per side from
     consecutive detections sorted by forward distance along the path.
Output: a JSON {"elements": [...], "lane_width_m": ..., "cone_spacing_m": ...,
"path_length_m": ..., "source": run.jsonl path}, loaded by pipeline_sim.road_spec(road=name)
once placed under offline/mpc_gym/real_tracks/.
"""
import argparse
from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
TRACKS_DIR = Path(__file__).resolve().parent / "real_tracks"


def load_events(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def dead_reckon(events):
    """(t, x, y, yaw) at every gyro sample, Euler-integrated from wheel/gyro alone."""
    path = [(events[0]["time_unix_s"], 0.0, 0.0, 0.0)]
    speed, last_t = 0.0, events[0]["time_unix_s"]
    for event in events:
        if event["kind"] == "wheel" and isinstance(event["value"], (int, float)):
            speed = max(0.0, float(event["value"]))
        elif event["kind"] == "gyro":
            t = event["value"]["stamp_s"]
            dt = t - last_t
            if 0.0 < dt < 0.5:
                _, x, y, yaw = path[-1]
                yaw_mid = yaw + 0.5*dt*event["value"]["z"]
                path.append((t, x+speed*dt*math.cos(yaw_mid), y+speed*dt*math.sin(yaw_mid),
                            yaw+dt*event["value"]["z"]))
            last_t = t
    return path


def pose_at(path, t):
    times = [p[0] for p in path]
    i = min(len(path)-1, max(0, bisect_right(times, t)-1))
    return path[i]


def path_length(path):
    return sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(path, path[1:]))


def curvature_series(path, window_m=0.3):
    """Signed curvature at each path sample: heading change over arc length, averaged
    over a short centred window to damp per-sample gyro/wheel noise."""
    s = [0.0]
    for a, b in zip(path, path[1:]):
        s.append(s[-1] + math.hypot(b[1]-a[1], b[2]-a[2]))
    out = []
    for i, (t, x, y, yaw) in enumerate(path):
        lo = next((j for j in range(i, -1, -1) if s[i]-s[j] >= window_m/2), 0)
        hi = next((j for j in range(i, len(path)) if s[j]-s[i] >= window_m/2), len(path)-1)
        ds = s[hi]-s[lo]
        dyaw = math.atan2(math.sin(path[hi][3]-path[lo][3]), math.cos(path[hi][3]-path[lo][3]))
        out.append(0.0 if ds <= 1e-6 else dyaw/ds)
    return s, out


def fit_segments(path, straight_curvature_1pm=0.08, min_segment_m=0.3):
    """Straight/circular_arc elements covering the whole path, in dream-gym's schema
    shape (dreamgym/envs/road_core/schema.py _STRAIGHT_GEOMETRY_KEYS/_CIRCULAR_ARC_
    GEOMETRY_KEYS). Runs shorter than min_segment_m are folded into a neighbour so cone
    placement (which needs a minimum element length) never sees a near-zero-length one."""
    s, kappa = curvature_series(path)
    total = s[-1]
    if total <= 0:
        raise ValueError("path has zero length; not enough wheel/gyro events")
    runs, start_i, is_straight = [], 0, abs(kappa[0]) < straight_curvature_1pm
    for i in range(1, len(kappa)+1):
        straight_now = i < len(kappa) and abs(kappa[i]) < straight_curvature_1pm
        if i == len(kappa) or straight_now != is_straight:
            runs.append((start_i, i, is_straight))
            start_i, is_straight = i, straight_now if i < len(kappa) else is_straight
    merged = []
    for start_i, end_i, straight in runs:
        if merged and s[end_i-1]-s[start_i] < min_segment_m:
            merged[-1] = (merged[-1][0], end_i, merged[-1][2])
        else:
            merged.append((start_i, end_i, straight))
    elements = []
    for start_i, end_i, straight in merged:
        length = s[end_i-1]-s[start_i]
        if length <= 1e-6:
            continue
        if straight:
            elements.append({"geometry": {"type": "straight", "length_m": round(length, 3)}})
        else:
            mean_kappa = statistics.mean(kappa[start_i:end_i]) or 1e-6
            # curvature_per_m's sign carries the turn direction (as tune.road_spec's
            # s_bend does); sweep_angle_deg is always the (positive) magnitude.
            elements.append({"geometry": {
                "type": "circular_arc", "curvature_per_m": round(mean_kappa, 4),
                "sweep_angle_deg": round(math.degrees(abs(mean_kappa)*length), 2)}})
    return elements, total


def cone_measurements(events, path):
    """(lane_width_m samples, {colour: [consecutive-x spacing_m]}) from real cones events,
    body-frame detections matched to the path pose at that event's own stamp_s."""
    widths, spacings = [], {1: [], 2: []}
    for event in events:
        if event["kind"] != "cones":
            continue
        _, _, _, yaw = pose_at(path, event["value"]["stamp_s"])
        by_colour = {1: [], 2: []}
        for x, y, _z, colour, confidence in event["value"]["positions"]:
            if confidence >= 0.5 and colour in by_colour:
                by_colour[colour].append((x, y))
        if by_colour[1] and by_colour[2]:
            left_y = statistics.median(y for _, y in by_colour[1])
            right_y = statistics.median(y for _, y in by_colour[2])
            if left_y > right_y:    # width is a distance, not signed left-right order
                widths.append(left_y-right_y)
        for colour, points in by_colour.items():
            points = sorted(points)
            spacings[colour].extend(b[0]-a[0] for a, b in zip(points, points[1:]) if b[0] > a[0])
    return widths, spacings


def build_road_spec(run_jsonl, name, straight_curvature_1pm=0.08, min_segment_m=0.3):
    events = load_events(run_jsonl)
    path = dead_reckon(events)
    elements, total_length = fit_segments(path, straight_curvature_1pm, min_segment_m)
    widths, spacings = cone_measurements(events, path)
    all_spacings = [v for side in spacings.values() for v in side if 0.05 < v < 1.0]
    return {"name": name, "source": str(run_jsonl), "path_length_m": round(total_length, 3),
            "elements": elements,
            "lane_width_m": round(statistics.median(widths), 3) if widths else None,
            "lane_width_samples": len(widths),
            "cone_spacing_m": round(statistics.median(all_spacings), 3) if all_spacings else None,
            "cone_spacing_samples": len(all_spacings),
            "element_types": dict(Counter(e["geometry"]["type"] for e in elements))}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run_jsonl", type=Path)
    parser.add_argument("--name", required=True, help="road name pipeline_sim.road_spec() will load")
    parser.add_argument("--straight-curvature-1pm", type=float, default=0.08)
    parser.add_argument("--min-segment-m", type=float, default=0.3)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    spec = build_road_spec(args.run_jsonl, args.name, args.straight_curvature_1pm, args.min_segment_m)
    out = args.out or (TRACKS_DIR / f"{args.name}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(spec, indent=1), encoding="utf-8")
    print(f"{out}: path {spec['path_length_m']} m, {spec['element_types']}, "
          f"lane width {spec['lane_width_m']} m ({spec['lane_width_samples']} samples), "
          f"cone spacing {spec['cone_spacing_m']} m ({spec['cone_spacing_samples']} samples)")
    for element in spec["elements"]:
        print(" ", element["geometry"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
