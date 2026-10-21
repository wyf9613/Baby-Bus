"""MVP2.1 planning; relocated from pinned f11428b, without algorithm tuning."""

from dataclasses import dataclass
from copy import deepcopy
import math
from .geometry import (
    _path_lengths,
    _path_points,
    _path_project,
    _path_sample,
    _path_tangent,
    finite_number,
)


@dataclass
class PlanningSettings:
    """Direct sampled-road follower or optional legacy V1 straight planner.

    No dimensions or braking capability are guessed. Required vehicle_limits
    must be supplied by calibration or an explicitly labelled offline fixture.
    Constant-twist compensation is opt-in and is a short-time approximation.
    """
    # road_centerline follows estimator samples; centerline retains legacy V1.
    algorithm: str = "centerline"
    preview_m: float = 0.35
    allow_forward_only: bool = False
    cruise_speed_mps: float = 0.2
    max_source_age_s: float = 0.2
    reference_lifetime_s: float = 0.1
    max_near_x_m: float = 0.5
    min_forward_x_m: float = 1.5
    max_fit_error_m: float = 0.03
    max_curvature_1pm: float = 0.05
    max_lateral_offset_m: float = 0.2
    max_heading_error_rad: float = 0.15
    max_sample_gap_m: float = 0.25
    max_alignment_dt_s: float = 0.1
    compensate_constant_twist: bool = False
    vehicle_limits_source: str = "upstream"
    # Explicit OFFLINE assumptions, not limits identified on the physical car.
    simulation_speed_max_mps: float = 0.5
    simulation_braking_deceleration_mps2: float = 0.5
    simulation_actuation_delay_s: float = 0.1
    simulation_safety_margin_m: float = 0.05

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == "algorithm":
                if value not in ("centerline", "road_centerline"):
                    raise ValueError("planning.algorithm must be centerline or road_centerline")
            elif name == "allow_forward_only":
                if not isinstance(value, bool):
                    raise ValueError("planning.allow_forward_only must be bool")
            elif name == "vehicle_limits_source":
                if value not in ("upstream", "course_simulation"):
                    raise ValueError("planning vehicle limits source must be upstream or course_simulation")
            elif name == "compensate_constant_twist":
                if not isinstance(value, bool):
                    raise ValueError("planning compensation flag must be bool")
            elif not finite_number(value) or value <= 0:
                raise ValueError(f"planning.{name} must be finite and positive")
        if self.max_near_x_m >= self.min_forward_x_m:
            raise ValueError("planning near range must precede far range")


def planning_vehicle_limits(upstream, settings):
    """Resolve an explicit profile without modifying estimation/calibration.

    Geometry comes from the course notebook's bicycle_model_config. Speed,
    braking, delay and margin are separately labelled configurable assumptions.
    'upstream' is the default and never silently falls back to simulation.
    """
    if settings.vehicle_limits_source == "upstream":
        return deepcopy(upstream) if isinstance(upstream, dict) else {}
    front, rear, width = 0.60*0.33*1.5, 0.40*0.33*1.5, 0.25
    return {"valid": True, "source": "course_simulation", "simulation_only": True,
            "vehicle_reference_point": "cg_ground_projection", "wheelbase_m": 0.33,
            "rear_axle_x_m": -0.40*0.33,
            "footprint_xy_m": [(-rear, -width/2), (front, -width/2),
                               (front, width/2), (-rear, width/2)],
            "speed_max_mps": settings.simulation_speed_max_mps,
            "braking_deceleration_mps2": settings.simulation_braking_deceleration_mps2,
            "actuation_delay_s": settings.simulation_actuation_delay_s,
            "safety_margin_m": settings.simulation_safety_margin_m,
            "parameter_sources": {
                "geometry": "docs/ad_gym_system_project_v2026_09_18.ipynb:bicycle_model_config",
                "motion_limits": "planning.simulation_*:explicit_offline_assumptions"}}


def evaluate_planning_path(reference, x_m, now_s):
    """Shared V1 consumer evaluator. Invalid/expired/out-of-range means None.

    For ORDERED_CENTERLINE the coordinate argument is arc length s (metres);
    legacy encodings keep body x. Reusing geometry in a later frame needs motion
    alignment; checking its deadline alone does not perform that alignment.
    """
    if not isinstance(reference, dict) or not reference.get("valid"):
        return None
    stamp, lifetime = reference.get("timestamp_s"), reference.get("valid_for_s")
    if (not all(finite_number(v) for v in (stamp, lifetime, now_s, x_m))
            or lifetime <= 0 or not stamp <= now_s < stamp+lifetime):
        return None
    path = reference.get("path") or {}
    if path.get("type") == "ORDERED_CENTERLINE":
        points = _path_points(path.get("samples_xy"))
        if points is None or path.get("independent_variable") != "s_m":
            return None
        lengths = _path_lengths(points)
        if not 0 <= x_m <= lengths[-1]:
            return None
        position = _path_sample(points, lengths, x_m)
        tangent = _path_tangent(points, lengths, x_m)
        return {"position_xy_m": position, "heading_rad": math.atan2(tangent[1], tangent[0]),
                "curvature_1pm": None}
    if path.get("type") == "CARTESIAN_CENTERLINE":
        points = CenterlinePlanner._points(path.get("samples_xy"))
        if points is None or not points[0][0] <= x_m <= points[-1][0]:
            return None
        for a, b in zip(points, points[1:]):
            if x_m <= b[0]:
                slope = (b[1]-a[1])/(b[0]-a[0])
                return {"position_xy_m": (x_m, a[1]+slope*(x_m-a[0])),
                        "heading_rad": math.atan(slope), "curvature_1pm": 0.0}
    coeffs, bounds = path.get("coeffs_low_to_high"), path.get("range")
    origin, scale = path.get("origin"), path.get("scale")
    if (path.get("type") != "CARTESIAN_Y_OF_X" or path.get("independent_variable") != "x_m"
            or not isinstance(coeffs, (list, tuple)) or len(coeffs) != 2
            or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(finite_number(v) for v in (*coeffs, *bounds, origin, scale))
            or scale <= 0 or bounds[1] <= bounds[0] or not bounds[0] <= x_m <= bounds[1]):
        return None
    y = coeffs[0]+coeffs[1]*(x_m-origin)/scale
    return {"position_xy_m": (x_m, y), "heading_rad": math.atan(coeffs[1]/scale),
            "curvature_1pm": 0.0}


class CenterlinePlanner:
    """Bounded V1 planner: one shared straight Cartesian path, no search.

    Accepts the estimation_output dictionaries in this file. Rejects single-side
    roads in this first operating domain. No obstacle response or old-path reuse.
    Failure requests a stop; the controller/framework must implement that stop.
    All output geometry is in the fixed current base_link frame at now_s.
    """
    def __init__(self, settings, frame_id="base_link"):
        self.settings, self.frame_id = settings, frame_id
        self.sequence = 0

    @staticmethod
    def _points(value):
        if not isinstance(value, (list, tuple)) or not 2 <= len(value) <= 201:
            return None
        points = []
        for point in value:
            if (not isinstance(point, (list, tuple)) or len(point) != 2
                    or not all(finite_number(v) for v in point)):
                return None
            points.append(tuple(float(v) for v in point))
        return points if all(b[0] > a[0] for a, b in zip(points, points[1:])) else None

    @staticmethod
    def _interpolate(points, x):
        if x < points[0][0]-1e-9 or x > points[-1][0]+1e-9:
            return None  # Never extrapolate boundary support.
        x = min(points[-1][0], max(points[0][0], x))
        lower, upper = 0, len(points)-1
        while upper-lower > 1:
            middle = (lower+upper)//2
            if points[middle][0] < x:
                lower = middle
            else:
                upper = middle
        a, b = points[lower], points[upper]
        return a[1] + (b[1]-a[1])*(x-a[0])/(b[0]-a[0])

    def plan(self, road, state, obstacles, vehicle_limits, now_s,
             source_timeout_s=None, mvp=False):
        self.sequence += 1
        cfg = self.settings
        diagnostics = {"obstacle_response_enabled": False,
                       "alignment": "none", "vehicle_limits_source": None}
        reference = {
            "schema_version": "planning_reference_v1", "planner_version": "centerline_v1",
            "trajectory_id": self.sequence, "timestamp_s": now_s,
            "generated_at_s": now_s, "frame_id": self.frame_id,
            "vehicle_reference_point": "cg_ground_projection",
            "valid": False, "status": "INVALID_INPUT", "reason": None,
            "valid_for_s": 0.0, "path": None, "target_speed_mps": 0.0,
            "speed_profile": None, "stop_requested": True, "stop_reason": None,
            "stop_at_path_m": None, "source_ages_s": {},
        }

        def fail(reason):
            reference["reason"] = reference["stop_reason"] = reason
            return reference, diagnostics

        if not finite_number(now_s):
            return fail("invalid_clock")
        if not isinstance(road, dict) or not isinstance(state, dict):
            return fail("missing_estimates")
        if not road.get("valid") or not state.get("speed_valid") or not state.get("yaw_rate_valid"):
            return fail("invalid_estimates")
        if road.get("frame_id") != self.frame_id or state.get("frame_id") != self.frame_id:
            return fail("frame_mismatch")
        speed, yaw = state.get("speed_mps"), state.get("yaw_rate_rps")
        if not finite_number(speed) or speed < 0 or not finite_number(yaw):
            return fail("invalid_motion_state")
        if not finite_number(state.get("timestamp_s")) or abs(state["timestamp_s"]-now_s) > 1e-6:
            return fail("state_frame_not_current")
        stamp = road.get("timestamp_s")
        if not finite_number(stamp) or now_s < stamp:
            return fail("invalid_road_timestamp")
        delta = now_s-stamp
        state_ages = state.get("source_age_s") or {}
        ages = {"road": road.get("source_age_s"),
                "wheel_speed": state_ages.get("wheel_speed"),
                "imu_angular_velocity": state_ages.get("imu_angular_velocity")}
        if not all(finite_number(v) and v >= 0 for v in ages.values()):
            return fail("missing_source_age")
        # Timestamp age and reported age are independent conservative checks;
        # never add acquisition latency a second time.
        ages["road"] = max(ages["road"], delta)
        reference["source_ages_s"] = dict(ages)
        if isinstance(obstacles, dict):
            reference["source_ages_s"]["obstacles"] = obstacles.get("source_age_s")
        deadlines = []
        for name, age in ages.items():
            limit = cfg.max_source_age_s
            if source_timeout_s is not None:
                timeout = source_timeout_s.get(name)
                if not finite_number(timeout) or timeout <= 0:
                    return fail("invalid_source_timeout")
                limit = min(limit, timeout)
            if age >= limit:
                return fail("stale_" + name)
            deadlines.append(limit-age)
        if road.get("visibility") != "both":
            return fail("v1_requires_both_boundaries")
        curvature = road.get("local_curvature_1pm")
        if not finite_number(curvature) or abs(curvature) > cfg.max_curvature_1pm:
            return fail("unsupported_road_curvature")
        center = self._points(road.get("centerline_xy"))
        left = self._points(road.get("left_boundary_xy"))
        right = self._points(road.get("right_boundary_xy"))
        if any(points is None for points in (center, left, right)):
            return fail("invalid_geometry")
        if delta > 1e-6:
            if not cfg.compensate_constant_twist:
                return fail("motion_alignment_required")
            if delta > cfg.max_alignment_dt_s:
                return fail("alignment_interval_too_long")
            # Constant filtered body speed/yaw rate over the short source age.
            # This is an explicit approximation, not measured odometry/EKF.
            angle = yaw*delta
            if abs(yaw) < 1e-8:
                tx, ty = speed*delta, 0.0
            else:
                tx, ty = speed*math.sin(angle)/yaw, speed*(1-math.cos(angle))/yaw
            cosine, sine = math.cos(angle), math.sin(angle)
            def transform(points):
                return [(cosine*(x-tx)+sine*(y-ty),
                         -sine*(x-tx)+cosine*(y-ty)) for x, y in points]
            center, left, right = map(transform, (center, left, right))
            diagnostics["alignment"] = "constant_twist_approximation"
        else:
            diagnostics["alignment"] = "same_timestamp"
        for points in (center, left, right):
            if any(b[0] <= a[0] or b[0]-a[0] > cfg.max_sample_gap_m
                   for a, b in zip(points, points[1:])):
                return fail("unsupported_geometry_order_or_gap")
        lo = max(0.0, center[0][0], left[0][0], right[0][0])
        hi = min(center[-1][0], left[-1][0], right[-1][0])
        if lo > cfg.max_near_x_m or hi < cfg.min_forward_x_m or hi <= lo:
            return fail("insufficient_near_or_far_coverage")
        # Fit only current, jointly supported points. Mean-centred OLS avoids
        # an unnecessarily high polynomial degree; a0 is never forced to zero.
        samples = [(lo, self._interpolate(center, lo))]
        samples.extend((x, y) for x, y in center if lo < x < hi)
        samples.append((hi, self._interpolate(center, hi)))
        mean_x = sum(x for x, _ in samples)/len(samples)
        mean_y = sum(y for _, y in samples)/len(samples)
        denominator = sum((x-mean_x)**2 for x, _ in samples)
        if denominator <= 1e-12:
            return fail("degenerate_centerline")
        a1 = sum((x-mean_x)*(y-mean_y) for x, y in samples)/denominator
        a0 = mean_y-a1*mean_x
        error = max(abs(a0+a1*x-y) for x, y in samples)
        diagnostics.update(fit_max_error_m=error, lateral_offset_m=a0,
                           heading_error_rad=math.atan(a1), x_range_m=[lo, hi])
        if error > cfg.max_fit_error_m:
            return fail("not_straight_enough")
        if abs(a0) > cfg.max_lateral_offset_m or abs(math.atan(a1)) > cfg.max_heading_error_rad:
            return fail("outside_v1_offset_or_heading_domain")
        if mvp:
            # Student first-run profile: geometry/freshness/domain checks above
            # remain, but a short low-speed run does not require a measured car
            # footprint, steering-angle map or braking model to generate a path.
            # This is NOT a footprint/stopping-distance qualification.
            diagnostics.update(operating_profile="mvp_low_speed", simulation_only=False, calibration_required=False,
                               footprint_check_enabled=False, stopping_model_enabled=False)
            reference.update(valid=True, status="TRACK", reason="mvp_centerline_available",
                             operating_profile="mvp_low_speed", simulation_only=False,
                             valid_for_s=min(cfg.reference_lifetime_s, *deadlines),
                             path={"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m",
                                   "origin": 0.0, "scale": 1.0,
                                   "coeffs_low_to_high": [a0, a1], "range": [lo, hi]},
                             target_speed_mps=cfg.cruise_speed_mps,
                             stop_requested=False, stop_reason=None)
            return reference, diagnostics
        limits = planning_vehicle_limits(vehicle_limits, cfg)
        diagnostics["vehicle_limits_source"] = limits.get("source")
        diagnostics["simulation_only"] = limits.get("simulation_only", False)
        reference["vehicle_limits_source"] = limits.get("source")
        reference["simulation_only"] = limits.get("simulation_only", False)
        reference["vehicle_limits_parameter_sources"] = deepcopy(limits.get("parameter_sources", {}))
        if (not limits.get("valid") or limits.get("source") in (None, "unmeasured")
                or limits.get("vehicle_reference_point") != "cg_ground_projection"):
            return fail("vehicle_limits_unavailable_or_reference_mismatch")
        required = ("speed_max_mps", "braking_deceleration_mps2",
                    "actuation_delay_s", "safety_margin_m")
        if any(not finite_number(limits.get(k)) or limits[k] < 0 for k in required):
            return fail("invalid_vehicle_limits")
        if limits["speed_max_mps"] <= 0 or limits["braking_deceleration_mps2"] <= 0:
            return fail("invalid_vehicle_limits")
        # Footprint is a polygon, not an x-ordered road; validate separately.
        footprint = limits.get("footprint_xy_m")
        if (not isinstance(footprint, (list, tuple)) or not 3 <= len(footprint) <= 32
                or any(not isinstance(p, (list, tuple)) or len(p) != 2
                       or not all(finite_number(v) for v in p) for p in footprint)):
            return fail("missing_vehicle_footprint")
        front, rear = max(p[0] for p in footprint), min(p[0] for p in footprint)
        body_left, body_right = max(p[1] for p in footprint), min(p[1] for p in footprint)
        if not rear < 0 < front or not body_right < 0 < body_left:
            return fail("invalid_vehicle_footprint")
        margin = limits["safety_margin_m"]
        # Enclose the footprint in a rectangle at the reference tangent. Trim
        # the path so all four corners have boundary support, then check every
        # piecewise-linear boundary breakpoint shifted by each corner offset.
        cosine, sine = math.cos(math.atan(a1)), math.sin(math.atan(a1))
        offsets = [(cosine*x-sine*y, sine*x+cosine*y)
                   for x in (rear, front) for y in (body_right, body_left)]
        boundary_lo = max(left[0][0], right[0][0])
        boundary_hi = min(left[-1][0], right[-1][0])
        lo = max(lo, boundary_lo-min(dx for dx, _ in offsets))
        hi = min(hi, boundary_hi-max(dx for dx, _ in offsets))
        # Near coverage was checked on the observed road above. Reserving room
        # for the rear of the body moves the reference start; that trim must not
        # falsely imply that the underlying near observations disappeared.
        if hi < cfg.min_forward_x_m or hi <= lo:
            return fail("insufficient_footprint_supported_range")
        diagnostics["x_range_m"] = [lo, hi]
        knots = sorted({lo, hi} | {x-dx for points in (left, right)
                                  for x, _ in points for dx, _ in offsets if lo < x-dx < hi})
        clearance = min(min(self._interpolate(left, x+dx)-(a0+a1*x+dy),
                            a0+a1*x+dy-self._interpolate(right, x+dx))*cosine
                        for x in knots for dx, dy in offsets)
        diagnostics["centerline_clearance_m"] = clearance-margin
        if clearance < margin:
            return fail("insufficient_centerline_body_clearance")
        # Check current body against the nearest supported road cross-section.
        # This is a bounded straight-road near-field assumption, not a swept
        # return-to-centre trajectory or an extrapolated output path.
        near_left, near_right = self._interpolate(left, lo), self._interpolate(right, lo)
        diagnostics["near_body_check"] = "nearest_supported_cross_section"
        if near_left < body_left+margin or near_right > body_right-margin:
            return fail("vehicle_outside_supported_corridor")
        available = (hi-lo)*math.hypot(1, a1)-front-margin
        delay, braking = limits["actuation_delay_s"], limits["braking_deceleration_mps2"]
        stopping = speed*delay+speed*speed/(2*braking)
        diagnostics.update(available_stopping_distance_m=available,
                           required_stopping_distance_m=stopping)
        if available <= 0 or stopping > available or speed > limits["speed_max_mps"]:
            return fail("current_speed_outside_stopping_domain")
        visibility_speed = max(0.0, math.sqrt((braking*delay)**2+2*braking*available)-braking*delay)
        target = min(cfg.cruise_speed_mps, limits["speed_max_mps"], visibility_speed)
        reference.update(valid=True, status="TRACK", reason="straight_centerline_available",
                         valid_for_s=min(cfg.reference_lifetime_s, *deadlines),
                         path={"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m",
                               "origin": 0.0, "scale": 1.0,
                               "coeffs_low_to_high": [a0, a1], "range": [lo, hi]},
                         target_speed_mps=target, stop_requested=False, stop_reason=None)
        return reference, diagnostics


class RoadCenterlinePlanner:
    """Pass through the estimator's curved centerline with a fixed speed target.

    No candidate search, offset recovery trajectory, body model or straight-line
    refit. Freshness and geometry encoding are checked synchronously. Forward-
    only observations are an explicit MVP option, not a claim of known free
    road around the body. This profile has no obstacle avoidance.
    """
    def __init__(self, settings, frame_id="base_link"):
        self.settings, self.frame_id = settings, frame_id
        self.sequence = 0

    def plan(self, road, state, obstacles, vehicle_limits, now_s,
             source_timeout_s=None, mvp=False, camera_only=False):
        self.sequence += 1
        cfg = self.settings
        ref = {"schema_version": "planning_reference_v1", "planner_version": "road_centerline",
               "trajectory_id": self.sequence, "timestamp_s": now_s,
               "frame_id": self.frame_id, "vehicle_reference_point": "cg_ground_projection",
               "valid": False, "status": "INVALID_INPUT", "reason": None,
               "valid_for_s": 0.0, "path": None, "target_speed_mps": 0.0,
               "stop_requested": True, "stop_reason": None, "simulation_only": False}
        diag = {"candidate_search_enabled": False, "footprint_check_enabled": False,
                "obstacle_response_enabled": False, "forward_only": False}

        def fail(reason):
            ref.update(reason=reason, stop_reason=reason)
            diag["reason"] = reason
            return ref, diag

        if not mvp:
            return fail("road_centerline_requires_mvp")
        if not finite_number(now_s) or not isinstance(road, dict) or not isinstance(state, dict):
            return fail("invalid_centerline_input")
        if not road.get("valid"):
            forward = road.get("forward_geometry")
            if (cfg.allow_forward_only
                    and (road.get("status") == "near_field_unobserved"
                         or (road.get("status") == "near_field_requires_both_boundaries"
                             and isinstance(forward, dict)
                             and forward.get("single_side_inferred_from_learned_width") is True))
                    and isinstance(forward, dict) and forward.get("geometry_valid") is True
                    and forward.get("time_aligned") is True):
                road = forward  # Keep the estimator output intact; no extension.
                diag["forward_only"] = True
            else:
                return fail("invalid_road_"+str(road.get("status")))
        inferred_single_side = (cfg.allow_forward_only
                                and road.get("single_side_inferred_from_learned_width") is True
                                and road.get("visibility") in ("left_only", "right_only")
                                and road.get("lane_width_source") == "last_two_sided_observation"
                                and finite_number(road.get("lane_width_m"))
                                and road["lane_width_m"] > 0
                                and finite_number(road.get("lane_width_observed_at_s"))
                                and road["lane_width_observed_at_s"] <= now_s)
        if camera_only and (road.get("visibility") != "both"
                or road.get("time_aligned") is not False
                or road.get("motion_compensated") is not False
                or road.get("tracking_basis") != "uncompensated_camera_frame"):
            return fail("camera_fallback_requires_current_two_sided_road")
        if ((road.get("visibility") != "both" and not inferred_single_side)
                or (not camera_only and road.get("time_aligned") is not True)
                or state.get("speed_valid") is not True
                or (not camera_only and state.get("yaw_rate_valid") is not True)):
            return fail("centerline_or_motion_unavailable")
        if any(obj.get("frame_id") != self.frame_id or not finite_number(obj.get("timestamp_s"))
               or abs(obj["timestamp_s"]-now_s) > 1e-6
               for obj in ((state,) if camera_only else (road, state))):
            return fail("centerline_frame_or_time_mismatch")
        if camera_only and (road.get("frame_id") != self.frame_id
                or not finite_number(road.get("timestamp_s"))
                or not 0 <= now_s-road["timestamp_s"] < cfg.max_source_age_s):
            return fail("camera_fallback_invalid_measurement_time")
        if (not finite_number(state.get("speed_mps")) or state["speed_mps"] < 0
                or (not camera_only and not finite_number(state.get("yaw_rate_rps")))):
            return fail("invalid_motion_state")
        ages = {"road": road.get("source_age_s"), **(state.get("source_age_s") or {})}
        diag["source_ages_s"], diag["source_age_limits_s"] = dict(ages), {}
        deadlines = [cfg.reference_lifetime_s]
        for name in (("road", "wheel_speed") if camera_only else
                     ("road", "wheel_speed", "imu_angular_velocity")):
            limit = cfg.max_source_age_s
            if source_timeout_s is not None:
                timeout = source_timeout_s.get(name)
                if not finite_number(timeout) or timeout <= 0:
                    return fail("invalid_source_timeout")
                limit = min(limit, timeout)
            age = ages.get(name)
            diag["source_age_limits_s"][name] = limit
            if not finite_number(age) or not 0 <= age < limit:
                return fail("stale_"+name)
            deadlines.append(limit-age)
        if road.get("degraded"):
            remaining = road.get("prediction_remaining_s")
            if not finite_number(remaining) or remaining <= 0:
                return fail("expired_road_prediction")
            deadlines.append(remaining)
        ordered = road.get("geometry_encoding") == "ordered_xy"
        points = (_path_points if ordered else CenterlinePlanner._points)(road.get("centerline_xy"))
        if points is None or (not ordered and points[-1][0] <= 0):
            return fail("invalid_centerline_samples")
        if any((math.hypot(b[0]-a[0], b[1]-a[1]) if ordered else b[0]-a[0])
               > cfg.max_sample_gap_m+1e-8 for a, b in zip(points, points[1:])):
            return fail("centerline_sample_gap")
        # No offset/heading/curvature rejection: the feedback controller pulls
        # the car toward the observed centerline instead of planning a return.
        path = {"type": "CARTESIAN_CENTERLINE", "independent_variable": "x_m",
                "samples_xy": points, "range": [points[0][0], points[-1][0]],
                "preview_m": cfg.preview_m}
        if ordered:
            lengths = _path_lengths(points)
            if _path_project(points, lengths) is None:
                return fail("ordered_branch_unavailable")
            path.update(type="ORDERED_CENTERLINE", independent_variable="s_m",
                        range=[0.0, lengths[-1]])
        # Explicit constant-speed policy, including valid predicted roads.
        # Do not consume their speed advisory; retain original prediction expiry.
        ref.update(valid=True, status="TRACK", reason="road_centerline_available",
                   valid_for_s=min(deadlines), path=path, target_speed_mps=cfg.cruise_speed_mps,
                   stop_requested=False, stop_reason=None, operating_profile="mvp_centerline_follow")
        if camera_only:
            ref.update(planner_version="gyro_fallback_camera_follow",
                       reason="gyro_unavailable_camera_tracking",
                       measurement_timestamp_s=road["timestamp_s"], time_aligned=False,
                       operating_profile="gyro_fallback_camera_follow")
            diag.update(gyro_fallback=True, motion_compensated=False,
                        tracking_basis="uncompensated_camera_frame")
        diag.update(reason=ref["reason"], point_count=len(points),
                    x_range_m=[min(p[0] for p in points), max(p[0] for p in points)],
                    path_encoding=path["type"], path_length_m=path["range"][1] if ordered else None,
                    inferred_single_side=inferred_single_side,
                    boundary_visibility=road.get("visibility"),
                    centerline_source=road.get("centerline_source", "two_boundaries"),
                    lane_width_m=road.get("lane_width_m"),
                    inward_offset_m=road.get("inward_offset_m"),
                    lane_width_observed_at_s=road.get("lane_width_observed_at_s"),
                    road_degraded=bool(road.get("degraded")), speed_policy="constant",
                    target_speed_mps=cfg.cruise_speed_mps,
                    reference_timestamp_s=ref["timestamp_s"], reference_valid_for_s=ref["valid_for_s"],
                    source_measurement_timestamp_s=road.get("measurement_timestamp_s"),
                    recommended_speed_scale=road.get("recommended_speed_scale"))
        return ref, diag
