"""MVP2.1 lattice; synced from 3652e9b with the local generated-path execution profile."""

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
    # Only a single boundary with no supplied centreline/width uses this scale.
    # It moves the CG guide inward; it is neither lane width nor body clearance.
    unknown_width_reference_scale: float = 4/3
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
    fusion_enabled: bool = True
    fusion_window_s: float = 1.0
    fusion_max_samples: int = 21
    fusion_spacing_m: float = 0.10
    fusion_memory_weight: float = 1.0
    fusion_rear_support_m: float = 0.30

    collision_coverage: str = 'prefix'
    score_coverage: str = 'prefix'

    def __post_init__(self):
        for name in ('collision_coverage', 'score_coverage'):
            if getattr(self, name) not in ('prefix', 'generated'):
                raise ValueError('shadow.'+name+' must be prefix or generated')
        if not isinstance(self.enabled, bool):
            raise ValueError("shadow.enabled must be boolean")
        if not isinstance(self.fusion_enabled, bool):
            raise ValueError("shadow.fusion_enabled must be boolean")
        if type(self.fusion_max_samples) is not int or not 1 <= self.fusion_max_samples <= 128:
            raise ValueError("shadow.fusion_max_samples must be an integer in [1,128]")
        if not isinstance(self.sample_count, int) or isinstance(self.sample_count, bool) or self.sample_count < 8:
            raise ValueError("shadow.sample_count must be an integer >=8")
        for name, value in vars(self).items():
            if name in ("enabled", "sample_count", "fusion_enabled", "fusion_max_samples", "collision_coverage", "score_coverage"):
                continue
            if not finite_number(value) or value < 0:
                raise ValueError("shadow."+name+" must be finite and nonnegative")
        for name in ("rate_hz", "length_m", "path_spacing_m", "score_min_length_m",
                     "search_half_width_m", "unknown_width_reference_scale", "body_front_m", "body_rear_m", "body_width_m",
                     "sweep_translation_step_m", "sweep_rotation_step_rad", "fusion_window_s", "fusion_spacing_m"):
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


def _shadow_pose_points(points, source_pose, target_pose):
    """Road samples already aligned to their state times -> current body frame."""
    if (len(source_pose) != 3 or len(target_pose) != 3
            or not all(finite_number(v) for v in (*source_pose, *target_pose))):
        raise ValueError("invalid shadow odometry pose")
    x, y, angle = source_pose
    tx, ty, target_angle = target_pose
    c, s = math.cos(angle), math.sin(angle)
    rc, rs = math.cos(target_angle), math.sin(target_angle)
    result = []
    for px, py in _shadow_points(points):
        wx, wy = x+c*px-s*py-tx, y+s*px+c*py-ty
        result.append((rc*wx+rs*wy, -rs*wx+rc*wy))
    return result


def _shadow_normal_match(points, p, tangent, normal, expected=0.0, first_segment=0):
    """Match ordered, agreeing road branches; no body-x/curvature admission gate."""
    matches = []
    for index in range(max(0, first_segment-1), len(points)-1):
        a, b = points[index:index+2]
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        denominator = normal[0]*dy-normal[1]*dx
        if length < 1e-9 or abs(denominator) < 1e-9:
            continue
        agreement = (dx*tangent[0]+dy*tangent[1])/length
        if agreement <= 0:
            continue  # Opposite U-turn leg is not the same ordered road branch.
        ax, ay = a[0]-p[0], a[1]-p[1]
        d = (ax*dy-ay*dx)/denominator
        u = (ax*normal[1]-ay*normal[0])/denominator
        if -1e-8 <= u <= 1+1e-8:
            matches.append((abs(d-expected), d, agreement, index))
    if not matches:
        return None
    _, d, agreement, index = min(matches)
    return d, agreement, index


def _shadow_reference_spline(points, spacing):
    """Arc-parameter Hermite fit; preserve vertical and returning road branches."""
    points = _shadow_points(points)
    if len(points) < 3:
        return points
    arc = _shadow_lengths(points)
    derivatives = []
    for i in range(len(points)):
        lo, hi = max(0, i-1), min(len(points)-1, i+1)
        ds = max(1e-9, arc[hi]-arc[lo])
        derivatives.append(tuple((points[hi][k]-points[lo][k])/ds for k in (0, 1)))
    result = [points[0]]
    for i, (a, b) in enumerate(zip(points, points[1:])):
        ds = arc[i+1]-arc[i]
        count = max(1, math.ceil(ds/spacing))
        for j in range(1, count+1):
            u = j/count
            h = (2*u**3-3*u*u+1, u**3-2*u*u+u, -2*u**3+3*u*u, u**3-u*u)
            result.append(tuple(h[0]*a[k]+h[1]*ds*derivatives[i][k]+h[2]*b[k]
                                +h[3]*ds*derivatives[i+1][k] for k in (0, 1)))
    return _shadow_points(result)


def _shadow_erase_reference_loops(points):
    """Remove self-intersecting excursions in guidance, never observed boundaries.

    No negative-x or large-turn filter: an ordinary U bend has no self-crossing.
    Degenerate fits are returned to the caller's raw-planning fallback.
    """
    points, removed = list(points), 0
    for _ in range(8):
        crossing = None
        for i in range(len(points)-3):
            for j in range(i+2, len(points)-1):
                if math.dist(points[i], points[j+1]) < 1e-8:
                    continue  # Preserve genuinely closed route support.
                if _shadow_segment_hit(points[i], points[i+1], points[j], points[j+1]):
                    crossing = (i, j)
                    break
            if crossing:
                break
        if crossing is None:
            break
        i, j = crossing
        points = points[:i+1]+points[j+1:]
        removed += 1
    return points, removed


def _shadow_simplify_reference(points, tolerance):
    """Bound geometric approximation error, avoiding dense Frenet projection cost."""
    if len(points) < 3:
        return points
    keep, pending = {0, len(points)-1}, [(0, len(points)-1)]
    while pending:
        lo, hi = pending.pop()
        a, b = points[lo], points[hi]
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm2 = dx*dx+dy*dy
        farthest, maximum = None, tolerance*tolerance
        for i in range(lo+1, hi):
            p = points[i]
            u = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm2)) if norm2 > 1e-12 else 0.0
            distance2 = (p[0]-a[0]-u*dx)**2+(p[1]-a[1]-u*dy)**2
            if distance2 > maximum:
                farthest, maximum = i, distance2
        if farthest is not None:
            keep.add(farthest)
            pending.extend(((lo, farthest), (farthest, hi)))
    return [points[i] for i in sorted(keep)]


def _shadow_fuse_road(job, cfg):
    """Robust local road guidance from deduplicated, motion-aligned snapshots.

    Real boundary samples supply normal intersections against a stable guide,
    avoiding the folding of a large offset around noisy boundary corners.
    Fused/inferred guidance NEVER replaces the current hard collision boundaries.
    """
    now, target_pose = job.get("stamp_s"), job.get("shadow_pose")
    if not finite_number(now) or not target_pose or len(target_pose) != 3:
        return None, {"status": "RAW_ONLY", "reason": "no_shadow_pose"}
    epoch = job.get("shadow_odometry_epoch")
    frames, seen = [], set()
    for snapshot in reversed(job.get("road_history") or []):
        stamp, pose = snapshot.get("stamp_s"), snapshot.get("pose")
        if (not finite_number(stamp) or not pose or len(pose) != 3
                or snapshot.get("odometry_epoch") != epoch or not 0 <= now-stamp <= cfg.fusion_window_s+1e-6):
            continue
        road = snapshot.get("road") or {}
        measurement = road.get("measurement_timestamp_s")
        key = measurement if finite_number(measurement) else stamp
        if key in seen:
            continue
        seen.add(key)
        transformed = dict(road)
        for field in ("centerline_xy", "left_boundary_xy", "right_boundary_xy"):
            transformed[field] = _shadow_pose_points(road.get(field), pose, target_pose)
        transformed["age_s"] = max(0.0, now-measurement) if finite_number(measurement) else now-stamp
        frames.append(transformed)
        if len(frames) >= cfg.fusion_max_samples:
            break
    raw = job.get("road") or {}
    if not frames:
        return None, {"status": "RAW_ONLY", "reason": "no_aligned_history"}
    prior = job.get("previous_reference") or {}
    prior_age = now-prior.get("stamp_s", now)
    prior_points = []
    if (prior.get("odometry_epoch") == epoch and prior.get("pose")
            and 0 <= prior_age <= cfg.fusion_window_s+2/cfg.rate_hz):
        prior_points = _shadow_pose_points(prior.get("points"), prior["pose"], target_pose)
    guide = prior_points
    if not guide:
        # Prefer the freshest actually two-sided reference for branch association.
        reliable = [r for r in frames if r.get("visibility") == "both" and r.get("centerline_xy")]
        guide = (reliable[0] if reliable else frames[0]).get("centerline_xy") or []
    if len(guide) < 2:
        return None, {"status": "RAW_ONLY", "reason": "insufficient_fit_support"}
    guide, removed = _shadow_erase_reference_loops(guide)
    if len(guide) < 2:
        return None, {"status": "RAW_ONLY", "reason": "degenerate_reference"}
    base = ShadowFrame(guide)
    # Append new measured support only on the same ordered, agreeing branch.
    for point in _shadow_points(raw.get("centerline_xy")):
        s, d = base.project(point)
        if s > base.end+cfg.fusion_spacing_m and abs(d) < cfg.search_half_width_m:
            guide.append(point)
            base = ShadowFrame(guide)
    s0, _ = base.project((0.0, 0.0))
    # Roll the fit window forward along ROAD ARC, retaining only local rear
    # support. A negative s0 means support starts ahead: keep the inferred
    # connection to the car rather than introducing a short-road gate.
    lo = max(0.0, s0-cfg.fusion_rear_support_m) if s0 >= 0 else s0
    hi = min(base.end, s0+1.5*max(cfg.length_m, cfg.score_min_length_m))
    hi = max(hi, lo+cfg.fusion_spacing_m)
    count = min(64, max(2, math.ceil((hi-lo)/cfg.fusion_spacing_m)))
    width = raw.get("lane_width_m")
    width = width if finite_number(width) and width > 0 else None
    scale = max(2*cfg.path_spacing_m, (width or 2*cfg.search_half_width_m)*0.10)
    last_segments = {}
    offsets, strengths, supports, locations = [], [], [], []
    for i in range(count+1):
        s = lo+(hi-lo)*i/count
        p, tangent, normal = base.at(s)
        observations = []
        if prior_points and 0 <= s <= base.end:
            observations.append((0.0, cfg.fusion_memory_weight*math.exp(-prior_age/cfg.fusion_window_s)))
        supported = False
        for index, road in enumerate(frames):
            weight = math.exp(-road["age_s"]/(cfg.fusion_window_s/2))
            w = road.get("lane_width_m")
            w = w if finite_number(w) and w > 0 else width
            sides = {}
            sources = road.get("boundary_source") or {}
            for side, sign in (("left", 1), ("right", -1)):
                if sources.get(side) not in ("observed", "observed_with_history"):
                    continue
                key = (index, side)
                match = _shadow_normal_match(road[side+"_boundary_xy"], p, tangent, normal,
                    sign*(w or 2*cfg.search_half_width_m)/2, last_segments.get(key, 0))
                if match:
                    sides[side] = match
                    last_segments[key] = match[2]
            if len(sides) == 2:
                a, b = sides["left"], sides["right"]
                d, quality = (a[0]+b[0])/2, min(a[1], b[1])**2
                if a[0] <= b[0]:
                    quality *= 0.1  # Conflicting evidence is weak, not a planning veto.
            elif sides and w:
                side, match = next(iter(sides.items()))
                d = match[0]+(-1 if side == "left" else 1)*w/2
                quality = 0.65*match[1]**2
            else:
                key = (index, "center")
                match = _shadow_normal_match(road.get("centerline_xy") or [], p, tangent, normal,
                                             0.0, last_segments.get(key, 0))
                if match is None:
                    continue
                last_segments[key] = match[2]
                d, quality = match[0], 0.4*match[1]**2
            # Only finite observations participate; no resulting road is rejected.
            if finite_number(d) and quality > 0:
                observations.append((d, weight*quality))
                supported = True
        # Cap repeated spatial evidence: history-backed copies do not become
        # independent votes merely because the estimator runs at 20 Hz.
        clusters = {}
        for value, weight in observations:
            key = round(value/scale)
            accumulated, total_weight, maximum_weight = clusters.get(key, (0.0, 0.0, 0.0))
            clusters[key] = (accumulated+value*weight, total_weight+weight, max(maximum_weight, weight))
        observations = sorted((value/total, maximum) for value, total, maximum in clusters.values() if total > 0)
        if observations:
            total = sum(w for _, w in observations)
            cumulative = 0.0
            middle = observations[-1][0]
            for value, weight in observations:
                cumulative += weight
                if cumulative >= total/2:
                    middle = value
                    break
            robust = [(v, w*min(1.0, scale/max(abs(v-middle), 1e-9))) for v, w in observations]
            strength = sum(w for _, w in robust)
            offset = sum(v*w for v, w in robust)/strength
        else:
            offset, strength = 0.0, 0.0
        offsets.append(offset)
        strengths.append(strength)
        supports.append(supported)
        locations.append((p, normal))
    # Small spatial regularisation of noisy offsets, not a curvature/steering cap.
    fitted = offsets[:]
    for _ in range(3):
        fitted = [fitted[0]]+[(strengths[i]*offsets[i]+0.25*(fitted[i-1]+fitted[i+1]))
            /(strengths[i]+0.5) for i in range(1, len(fitted)-1)]+[fitted[-1]]
    points = [(p[0]+d*n[0], p[1]+d*n[1]) for (p, n), d in zip(locations, fitted)]
    points, extra = _shadow_erase_reference_loops(points)
    points = _shadow_reference_spline(points, cfg.fusion_spacing_m/2)
    points = _shadow_simplify_reference(points, cfg.path_spacing_m/4)
    points, after_fit_loops = _shadow_erase_reference_loops(points)
    if len(points) < 2:
        return None, {"status": "RAW_ONLY", "reason": "empty_fit"}
    road = dict(raw)
    road.update(centerline_xy=points, centerline_source="shadow_temporal_fusion",
                shadow_reference_is_inferred=True)
    # Current boundaries and their provenance are copied verbatim from raw.
    reference = {"points": points, "pose": list(target_pose), "stamp_s": now, "odometry_epoch": epoch}
    return road, {"status": "FUSED", "history_frames": len(frames), "measurement_epochs": len(seen),
        "prior_used": bool(prior_points), "supported_fit_samples": sum(supports), "fit_samples": len(supports),
        "removed_guidance_loops": removed+extra+after_fit_loops, "guide": reference,
        "oldest_measurement_age_s": max(r["age_s"] for r in frames),
        "source_age_is_not_refreshed": True,
        "reference_crop": {"source_length_m": base.end, "origin_s_m": s0,
            "window_s_m": [lo, hi], "trimmed_rear_m": max(0.0, min(lo, base.end)),
            "retained_rear_m": max(0.0, s0-lo), "output_length_m": _shadow_lengths(points)[-1]},
        "hard_boundaries": "unchanged_current_input", "reference_is_observed_boundary": False}


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
        """Fusion cannot reduce raw planner availability: retry raw on any failure."""
        collection = job.get("temporal_collection")
        collection = collection if isinstance(collection, dict) else {}
        if not self.cfg.fusion_enabled or not job.get("road_history") or collection.get("reset"):
            result = self._plan_geometry(job)
            reason = "motion_alignment_reset" if collection.get("reset") else "history_unavailable"
            result["fusion"] = {"status": "RAW_FALLBACK" if collection.get("reset") else "RAW_ONLY" if self.cfg.fusion_enabled else "DISABLED",
                "reason": reason if self.cfg.fusion_enabled else "fusion_disabled",
                "degraded": bool(self.cfg.fusion_enabled), "raw_fallback_used": bool(self.cfg.fusion_enabled),
                "selected_source": "raw", "raw_status": result["status"], "fused_status": None,
                "collection": collection}
            if self.cfg.fusion_enabled:
                result["temporal_input"] = {key: job.get(key) for key in
                    ("road_history", "shadow_pose", "shadow_odometry_epoch", "previous_reference", "temporal_collection")}
            return result
        started = time.perf_counter()
        road, fusion, attempt_s = None, {}, 0.0
        try:
            road, fusion = _shadow_fuse_road(job, self.cfg)
        except Exception as error:
            fusion = {"status": "RAW_FALLBACK", "reason": "fusion_error", "error": str(error)}
        fusion_s = time.perf_counter()-started
        result = None
        if road is not None:
            attempt_started = time.perf_counter()
            try:
                result = self._plan_geometry({**job, "road": road})
            except Exception as error:
                fusion.update(reason="fused_planning_error", error=str(error))
            fusion["fused_status"] = result.get("status") if result else "ERROR"
            if result is None or result.get("status") != "VALID":
                attempt_s = time.perf_counter()-attempt_started
                fusion.update(status="RAW_FALLBACK", fused_status=result.get("status") if result else "ERROR")
                fusion.setdefault("reason", "no_feasible_fused_candidate")
                if result:
                    fusion["failed_fused_attempt"] = {"reference": road.get("centerline_xy"),
                        "candidate_count": result.get("candidate_count"), "feasible_count": result.get("feasible_count"),
                        "selected": result.get("selected"), "elapsed_s": result.get("elapsed_s")}
                result = None
        if result is None:
            result = self._plan_geometry(job)
            fusion.pop("guide", None)
            fusion.update(raw_fallback_used=True, selected_source="raw", raw_status=result["status"])
        else:
            fusion.update(raw_fallback_used=False, selected_source="fused", raw_status=None)
        fusion.setdefault("reason", "temporal_reference")
        fusion["collection"] = collection
        fusion["degraded"] = fusion.get("status") != "FUSED" or bool(collection.get("events"))
        result["fusion"] = fusion
        result["temporal_input"] = {key: job.get(key) for key in
            ("road_history", "shadow_pose", "shadow_odometry_epoch", "previous_reference", "temporal_collection")}
        result["raw_planning_input"] = {"road": job.get("road"), "state": job.get("state")}
        if fusion.get("status") == "FUSED":
            result["reference_support_kind"] = "fused_guidance_not_observed_boundary"
            result["reference_observed_range_s_m"] = None
            result["observed_centerline_length_m"] = _shadow_lengths(_shadow_points(
                (job.get("road") or {}).get("centerline_xy")))[-1] if _shadow_points(
                (job.get("road") or {}).get("centerline_xy")) else 0.0
        elapsed = time.perf_counter()-started
        timing = result["timing_s"]
        old_total = timing.pop("total")
        timing.update(temporal_fusion=fusion_s, discarded_fused_plan=attempt_s)
        timing["overhead"] += max(0.0, elapsed-old_total-fusion_s-attempt_s)
        timing["total"] = elapsed
        result.update(elapsed_s=elapsed, late_for_requested_period=elapsed > 1/self.cfg.rate_hz)
        return result

    def _plan_geometry(self, job):
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
        if cfg.collision_coverage == 'generated':
            check = horizon
        if cfg.score_coverage == 'generated':
            score_length = horizon
        points = _shadow_points(road.get("centerline_xy"))
        left, right = (_shadow_points(road.get(side+"_boundary_xy")) for side in ("left", "right"))
        width = road.get("lane_width_m")
        width = width if finite_number(width) and width > 0 else None
        reference_origin = "centerline"
        reference_offset, unknown_width_scaled = None, False
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
            if width is None and bool(left) != bool(right):
                offset *= cfg.unknown_width_reference_scale
                unknown_width_scaled = True
            reference_offset = offset
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
        knot_frames = {}  # Identical station lookups across candidates, per job.
        for candidate_id, (family, offsets) in enumerate(samples[:cfg.sample_count]):
            phase_started = time.perf_counter()
            early = 0.12 if family == "fast_center" else 0.25
            distances = (early*horizon, 0.6*horizon, horizon)
            knots = [(0.0, 0.0)]
            for distance, offset in zip(distances, offsets):
                if distance not in knot_frames:
                    knot_frames[distance] = frame.at(s0+distance)
                p, _, n = knot_frames[distance]
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
                "reference_boundary_offset_m": reference_offset,
                "unknown_width_reference_scaled": unknown_width_scaled,
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
