"""MVP2.1 lattice; relocated from pinned f11428b, without algorithm tuning."""

from dataclasses import dataclass
import math
import time
from .frenet import ShadowFrame, _shadow_lengths, _shadow_at
from .geometry import (
    finite_number,
    wrap_angle,
)


@dataclass
class ShadowSettings:
    """New shadow-only design. No settings or code from the archived lattice.

    Dimensions below are explicitly provisional; replace with measurements for
    the tested car. No actuator/vehicle model or clearance padding is used.
    This module never produces actuator messages or changes baseline admission.
    """
    enabled: bool = False
    rate_hz: float = 1.0
    length_m: float = 1.0
    sample_count: int = 48
    path_spacing_m: float = 0.025
    handoff_allowance_s: float = 0.10
    score_min_length_m: float = 0.2
    search_half_width_m: float = 0.45
    preparation_width_fraction: float = 0.15
    body_front_m: float = 0.25
    body_rear_m: float = 0.25
    body_width_m: float = 0.30
    sweep_translation_step_m: float = 0.01
    sweep_rotation_step_rad: float = 0.025
    weight_lateral: float = 6.0
    weight_heading: float = 1.5
    weight_progress: float = 2.0
    weight_smoothness: float = 0.04
    # A control consumer can hold/read beyond the diagnostic prefix. Opt-in
    # execution profile evaluates the entire generated path, including preview.
    coverage_mode: str = 'prefix'

    def __post_init__(self):
        if self.coverage_mode not in ('prefix', 'generated'):
            raise ValueError('shadow.coverage_mode must be prefix or generated')
        if not isinstance(self.enabled, bool):
            raise ValueError("shadow.enabled must be boolean")
        if not isinstance(self.sample_count, int) or isinstance(self.sample_count, bool) or self.sample_count < 8:
            raise ValueError("shadow.sample_count must be an integer >=8")
        for name, value in vars(self).items():
            if name in ("enabled", "sample_count", "coverage_mode"):
                continue
            if not finite_number(value) or value < 0:
                raise ValueError("shadow."+name+" must be finite and nonnegative")
        for name in ("rate_hz", "length_m", "path_spacing_m", "score_min_length_m",
                     "search_half_width_m", "body_front_m", "body_rear_m", "body_width_m",
                     "sweep_translation_step_m", "sweep_rotation_step_rad"):
            if getattr(self, name) == 0:
                raise ValueError("shadow."+name+" must be positive")


def _shadow_points(values):
    """Retain finite unique neighbours; even ONE point is usable road data."""
    points = []
    for p in values or []:
        if isinstance(p, (tuple, list)) and len(p) >= 2 and all(finite_number(v) for v in p[:2]):
            point = (float(p[0]), float(p[1]))
            if not points or math.dist(point, points[-1]) > 1e-9:
                points.append(point)
    return points








def _shadow_halton(index, base):
    result, factor = 0.0, 1.0
    while index:
        index, digit = divmod(index, base)
        factor /= base
        result += digit*factor
    return result


def _shadow_curve(knots, length, spacing):
    """C1 Cartesian Hermite interpolation of Frenet-sampled intermediate knots.

    Starting tangent is the real vehicle heading. Free intermediate knots admit
    immediate recentering AND an outward setup; there is no forced end curvature.
    """
    tangents = [(1.0, 0.0)]
    for i in range(1, len(knots)):
        a, b = knots[i-1], knots[min(i+1, len(knots)-1)]
        norm = math.dist(a, b)
        tangents.append(((b[0]-a[0])/norm, (b[1]-a[1])/norm) if norm > 1e-9 else tangents[-1])
    dense = [knots[0]]
    for i, (a, b) in enumerate(zip(knots, knots[1:])):
        chord = math.dist(a, b)
        if chord < 1e-9:
            continue
        count = max(8, math.ceil(chord/(spacing/3)))
        for j in range(1, count+1):
            u = j/count
            h = (2*u**3-3*u*u+1, u**3-2*u*u+u, -2*u**3+3*u*u, u**3-u*u)
            dense.append(tuple(h[0]*a[k]+h[1]*chord*tangents[i][k]+h[2]*b[k]
                               +h[3]*chord*tangents[i+1][k] for k in (0, 1)))
    dense = _shadow_points(dense)
    if len(dense) == 1:
        dense.append((dense[0][0]+length, dense[0][1]))
    arc = _shadow_lengths(dense)
    count = max(2, math.ceil(length/spacing))
    points = [_shadow_at(dense, arc, length*i/count) for i in range(count+1)]
    headings = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(points, points[1:])]
    return [(x, y, 0.0 if i == 0 else headings[min(i, len(headings)-1)]) for i, (x, y) in enumerate(points)]


def _shadow_segment_hit(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    if (max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0])
            or max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1])):
        return False
    return cross(a, b, c)*cross(a, b, d) <= 0 and cross(c, d, a)*cross(c, d, b) <= 0


def _shadow_collision(path, check_length, boundaries, cfg):
    """Full unpadded rectangle, supported boundary segments only.

    Sweep is adaptively sampled in translation AND heading. It is a numerical
    check, not a certified continuous-sweep or physical-contact guarantee.
    Endpoint inclusion and interpolation prevent simply skipping grid intervals.
    """
    arc = _shadow_lengths([p[:2] for p in path])
    end = min(check_length, arc[-1])
    poses = [path[0]]
    for i, (a, b) in enumerate(zip(path, path[1:])):
        if arc[i] >= end:
            break
        fraction = min(1.0, (end-arc[i])/(arc[i+1]-arc[i]))
        angle = wrap_angle(b[2]-a[2])*fraction
        distance = (arc[i+1]-arc[i])*fraction
        count = max(1, math.ceil(distance/cfg.sweep_translation_step_m),
                    math.ceil(abs(angle)/cfg.sweep_rotation_step_rad))
        for j in range(1, count+1):
            u = fraction*j/count
            poses.append((a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), a[2]+wrap_angle(b[2]-a[2])*u))
    for pose_index, (x, y, heading) in enumerate(poses):
        c, sn = math.cos(heading), math.sin(heading)
        rectangle = [(x+c*bx-sn*by, y+sn*bx+c*by) for bx, by in
                     ((-cfg.body_rear_m, -cfg.body_width_m/2), (cfg.body_front_m, -cfg.body_width_m/2),
                      (cfg.body_front_m, cfg.body_width_m/2), (-cfg.body_rear_m, cfg.body_width_m/2))]
        xmin, xmax = min(p[0] for p in rectangle), max(p[0] for p in rectangle)
        ymin, ymax = min(p[1] for p in rectangle), max(p[1] for p in rectangle)
        for side, points in boundaries.items():
            for p in points:
                if not xmin <= p[0] <= xmax or not ymin <= p[1] <= ymax:
                    continue
                bx, by = c*(p[0]-x)+sn*(p[1]-y), -sn*(p[0]-x)+c*(p[1]-y)
                if -cfg.body_rear_m <= bx <= cfg.body_front_m and abs(by) <= cfg.body_width_m/2:
                    return {"side": side, "point_xy_m": p, "pose": [x, y, heading], "pose_index": pose_index}
            for a, b in zip(points, points[1:]):
                if (max(a[0], b[0]) < xmin or min(a[0], b[0]) > xmax
                        or max(a[1], b[1]) < ymin or min(a[1], b[1]) > ymax):
                    continue
                if any(_shadow_segment_hit(a, b, rectangle[j], rectangle[(j+1)%4]) for j in range(4)):
                    return {"side": side, "segment_xy_m": [a, b], "pose": [x, y, heading], "pose_index": pose_index}
    return None


class ShadowLatticePlanner:
    """Fresh bounded sampler / prefix ranker, deliberately permissive about road.

    Input validity flags, short coverage, absent sides, slope, curvature and
    normalized steering requests are NOT admission gates. Only absent usable
    geometry suppresses generation. Known-prefix collisions remove candidates;
    ALL_COLLIDE keeps the ranked diagnostic candidate without calling it valid.
    """
    def __init__(self, settings):
        self.cfg = settings

    def plan(self, job):
        started = time.perf_counter()
        timing = {key: 0.0 for key in ("input", "frenet", "sampling", "generation",
                                       "collision", "scoring", "selection", "assembly")}
        cfg, road = self.cfg, job.get("road") or {}
        state = job.get("state") or {}
        v = state.get("speed_mps")
        speed = abs(v) if finite_number(v) else 0.0
        observed_handoff = job.get("handoff_estimate_s", 0.0)
        observed_handoff = observed_handoff if finite_number(observed_handoff) else 0.0
        handoff = max(cfg.handoff_allowance_s, observed_handoff)
        hold = 1/cfg.rate_hz+handoff
        check = speed*hold
        score_length = max(cfg.score_min_length_m, check)
        preview = job.get("preview_m", 0.0)
        preview = preview if finite_number(preview) and preview > 0 else cfg.score_min_length_m
        horizon = max(cfg.length_m, score_length, check+preview)
        control_read_length = check+preview
        if cfg.coverage_mode == 'generated':
            check = score_length = horizon
        points = _shadow_points(road.get("centerline_xy"))
        left, right = (_shadow_points(road.get(side+"_boundary_xy")) for side in ("left", "right"))
        width = road.get("lane_width_m")
        width = width if finite_number(width) and width > 0 else None
        reference_origin = "centerline"
        phase_started = time.perf_counter()
        timing["input"] = phase_started-started
        if not points:
            boundary = left or right
            if not boundary:
                elapsed = time.perf_counter()-started
                timing.update(total=elapsed, overhead=elapsed-sum(timing.values()))
                return {"status": "NO_ROAD", "selected": None, "candidates": [],
                        "candidate_count": 0, "feasible_count": 0, "road_status": road.get("status"),
                        "input_stamp_s": job.get("stamp_s"), "elapsed_s": elapsed, "timing_s": timing}
            side = "left" if left else "right"
            base = ShadowFrame(boundary)
            offset = width/2 if width else cfg.search_half_width_m
            points = [tuple(p[k]+(-1 if side == "left" else 1)*offset*n[k] for k in (0, 1))
                      for s in base.lengths for p, _, n in [base.at(s)]]
            reference_origin = "boundary_offset_inferred"
        frame = ShadowFrame(points)
        s0, d0 = frame.project((0.0, 0.0))
        # Detect direction from actually supplied centreline, never from tail extension.
        a = frame.at(max(0.0, s0))[1]
        b = frame.at(min(frame.end, max(0.0, s0)+horizon))[1]
        entry_gap = max(0.0, -s0)
        entry_weight = entry_gap/(entry_gap+preview)
        turn = wrap_angle(math.atan2(b[1], b[0])-(1-entry_weight)*math.atan2(a[1], a[0]))
        amplitude = (width or 2*cfg.search_half_width_m)*cfg.preparation_width_fraction
        preparation = -amplitude*max(-1.0, min(1.0, turn/(math.pi/2)))
        search = max(cfg.search_half_width_m, abs(d0))
        next_phase = time.perf_counter()
        timing["frenet"] = next_phase-phase_started
        phase_started = next_phase
        # Explicit behaviours plus paired Halton coverage: repeatable and mirror symmetric.
        samples = [("follow", (0.0, 0.0, 0.0)), ("hold", (d0, d0, d0)),
                   ("fast_center", (0.0, 0.0, 0.0)),
                   ("prepare", (preparation, preparation/2, 0.0)),
                   ("left_setup", (amplitude, amplitude/2, 0.0)),
                   ("right_setup", (-amplitude, -amplitude/2, 0.0))]
        for index in range(1, math.ceil((cfg.sample_count-6)/2)+1):
            offsets = tuple((2*_shadow_halton(index, base)-1)*search for base in (2, 3, 5))
            samples.extend(("halton", tuple(sign*v for v in offsets)) for sign in (1, -1))
        sources = road.get("boundary_source") or {}
        boundaries = {side: value for side, value in (("left", left), ("right", right))
                      if value and sources.get(side) in ("observed", "observed_with_history")}
        timing["sampling"] = time.perf_counter()-phase_started
        candidates = []
        for candidate_id, (family, offsets) in enumerate(samples[:cfg.sample_count]):
            phase_started = time.perf_counter()
            early = 0.12 if family == "fast_center" else 0.25
            distances = (early*horizon, 0.6*horizon, horizon)
            knots = [(0.0, 0.0)]
            for distance, offset in zip(distances, offsets):
                p, _, n = frame.at(s0+distance)
                knots.append((p[0]+offset*n[0], p[1]+offset*n[1]))
            path = _shadow_curve(knots, horizon, cfg.path_spacing_m)
            next_phase = time.perf_counter()
            timing["generation"] += next_phase-phase_started
            phase_started = next_phase
            collision = _shadow_collision(path, check, boundaries, cfg)
            next_phase = time.perf_counter()
            timing["collision"] += next_phase-phase_started
            phase_started = next_phase
            arc = _shadow_lengths([p[:2] for p in path])
            lateral, heading_cost, progress, smooth, total_weight = 0.0, 0.0, 0.0, 0.0, 0.0
            previous_heading = path[0][2]
            scoring_samples = []
            for i, (x, y, heading) in enumerate(path):
                if arc[i] > score_length+1e-9:
                    break
                s, d = frame.project((x, y), heading)
                target = preparation*min(1.0, max(0.0, arc[i]/max(score_length, 1e-9)))
                tangent = frame.at(s)[1]
                weight = 2-arc[i]/max(score_length, 1e-9)
                error = d-target
                lateral += weight*error**2
                # During setup the target normal motion is deliberate, not heading failure.
                target_heading = math.atan2(tangent[1], tangent[0])+math.atan2(preparation, max(score_length, 1e-9))
                heading_cost += weight*wrap_angle(heading-target_heading)**2
                progress += weight*max(0.0, abs(error)-abs(d0))**2
                smooth += wrap_angle(heading-previous_heading)**2
                previous_heading = heading
                total_weight += weight
                scoring_samples.append([arc[i], d, target, error, 0 <= s <= frame.end])
            costs = {"lateral": lateral/total_weight, "heading": heading_cost/total_weight,
                     "progress": progress/total_weight,
                     "smoothness": smooth/max(1, len(scoring_samples))}
            cost = sum(getattr(cfg, "weight_"+key)*value for key, value in costs.items())
            candidates.append({"id": candidate_id, "family": family, "offsets_m": offsets,
                "cost": cost, "costs": costs, "collision": collision, "path": path,
                "scoring_samples": scoring_samples})
            timing["scoring"] += time.perf_counter()-phase_started
        phase_started = time.perf_counter()
        feasible = [c for c in candidates if c["collision"] is None]
        selected = min(feasible or candidates, key=lambda c: (c["cost"], c["id"]))
        next_phase = time.perf_counter()
        timing["selection"] = next_phase-phase_started
        phase_started = next_phase
        result = {"status": "VALID" if feasible else "ALL_COLLIDE", "selected": selected,
                "candidates": candidates, "candidate_count": len(candidates), "feasible_count": len(feasible),
                "input_stamp_s": job.get("stamp_s"), "measurement_stamp_s": road.get("measurement_timestamp_s"),
                "road_status": road.get("status"), "road_sources": sources,
                "planning_input": {"road": road, "state": state, "preview_m": preview},
                "shadow_settings": vars(cfg).copy(), "controller_preview_m": preview,
                "reference_chart_xy": list(frame.points), "reference_origin": reference_origin, "reference": [frame.at(s0+horizon*i/40)[0] for i in range(41)],
                "reference_observed_range_s_m": [0.0, frame.end], "start_s_m": s0, "start_d_m": d0,
                "turn_angle_rad": turn, "entry_heading_weight": entry_weight, "preparation_offset_m": preparation,
                "speed_mps": speed, "speed_available": finite_number(v), "hold_time_s": hold,
                "handoff_estimate_s": handoff, "requested_rate_hz": cfg.rate_hz,
                "check_length_m": check, "score_length_m": score_length, "generated_length_m": horizon,
                "control_read_length_m": control_read_length, "known_boundary_sides": list(boundaries),
                "body_dimensions_m": [cfg.body_front_m, cfg.body_rear_m, cfg.body_width_m],
                "body_dimensions_source": "shadow_configuration_provisional",
                "collision_method": "adaptive_pose_sampling_no_padding",
                "checked_only_known_supported_boundaries": True,
                "observed_centerline_length_m": frame.end}
        finished = time.perf_counter()
        timing["assembly"] = finished-phase_started
        elapsed = finished-started
        timing["overhead"] = max(0.0, elapsed-sum(timing.values()))
        timing["total"] = elapsed
        result.update(elapsed_s=elapsed, timing_s=timing, late_for_requested_period=elapsed > 1/cfg.rate_hz)
        return result
