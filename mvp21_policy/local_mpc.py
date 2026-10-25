"""Original user MPC definitions extracted unchanged; controller is not upstream PID."""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from numbers import Real
import math, time, json, os, sys
def finite_number(value):
    """Reject booleans/strings/NaN/infinity instead of turning them into actions."""
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def _path_points(value):
    """Validate an ordered polyline, without assuming increasing body x."""
    if not isinstance(value, (list, tuple)) or not 2 <= len(value) <= 201:
        return None
    if any(not isinstance(p, (list, tuple)) or len(p) != 2
           or not all(finite_number(v) for v in p) for p in value):
        return None
    points = [tuple(float(v) for v in p) for p in value]
    if any(math.hypot(b[0]-a[0], b[1]-a[1]) <= 1e-6 for a, b in zip(points, points[1:])):
        return None
    return points

def _path_lengths(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    return lengths

def _path_sample(points, lengths, distance):
    distance = min(lengths[-1], max(0.0, distance))
    for i, (a, b) in enumerate(zip(points, points[1:])):
        if distance <= lengths[i+1]+1e-9:
            u = (distance-lengths[i])/(lengths[i+1]-lengths[i])
            return (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
    return points[-1]

def _path_tangent(points, lengths, distance, window=0.15):
    a = _path_sample(points, lengths, distance-window)
    b = _path_sample(points, lengths, distance+window)
    norm = math.hypot(b[0]-a[0], b[1]-a[1])
    return ((b[0]-a[0])/norm, (b[1]-a[1])/norm)

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
    """Shared polynomial/sample/ordered-centreline evaluator. For ORDERED_CENTERLINE
    the coordinate argument is arc length s (metres); legacy encodings keep body x.
    Invalid/expired/out-of-range means None.

    Reusing this geometry in a later body frame additionally needs motion
    alignment; checking its deadline alone does not perform that alignment.
    """
    if not isinstance(reference, dict) or not reference.get("valid"):
        return None
    stamp, lifetime = reference.get("timestamp_s"), reference.get("valid_for_s")
    if (not all(finite_number(v) for v in (stamp, lifetime, now_s, x_m))
            or lifetime <= 0 or not stamp <= now_s < stamp+lifetime):
        return None
    path = reference.get("path") or {}
    # Ported from main 2026-10-15: road_centerline's two sample encodings.
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
    if "control_cached_path_samples" in reference or path.get("type") == "CARTESIAN_SAMPLES":
        try:
            point = _control_sample_at(_control_sample_points(reference), x_m)
        except (ValueError, TypeError, ArithmeticError):
            return None
        return {"position_xy_m": point[:2], "heading_rad": point[2], "curvature_1pm": point[3]}
    coeffs, bounds = path.get("coeffs_low_to_high"), path.get("range")
    origin, scale = path.get("origin"), path.get("scale")
    if (path.get("type") != "CARTESIAN_Y_OF_X" or path.get("independent_variable") != "x_m"
            or not isinstance(coeffs, (list, tuple)) or not 1 <= len(coeffs) <= 6
            or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(finite_number(v) for v in (*coeffs, *bounds, origin, scale))
            or scale <= 0 or bounds[1] <= bounds[0] or not bounds[0] <= x_m <= bounds[1]):
        return None
    u = (x_m-origin)/scale
    y = _control_poly_eval(coeffs, u)
    slope = _control_poly_eval([i*c for i, c in enumerate(coeffs) if i] or [0.0], u)/scale
    second = _control_poly_eval([i*(i-1)*c for i, c in enumerate(coeffs) if i > 1] or [0.0], u)/scale**2
    return {"position_xy_m": (x_m, y), "heading_rad": math.atan(slope),
            "curvature_1pm": second/math.hypot(1, slope)**3}

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
        speed_scale = road.get("recommended_speed_scale", 1.0)
        if not finite_number(speed_scale) or not 0 < speed_scale <= 1:
            return fail("invalid_road_speed_advisory")
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
        if road.get("degraded"):
            remaining = road.get("prediction_remaining_s")
            if not finite_number(remaining) or remaining <= 0:
                return fail("expired_road_prediction")
            deadlines.append(remaining)
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
                             target_speed_mps=cfg.cruise_speed_mps*speed_scale,
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
        target = min(cfg.cruise_speed_mps*speed_scale, limits["speed_max_mps"], visibility_speed)
        reference.update(valid=True, status="TRACK", reason="straight_centerline_available",
                         valid_for_s=min(cfg.reference_lifetime_s, *deadlines),
                         path={"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m",
                               "origin": 0.0, "scale": 1.0,
                               "coeffs_low_to_high": [a0, a1], "range": [lo, hi]},
                         target_speed_mps=target, stop_requested=False, stop_reason=None)
        return reference, diagnostics

class MPCStop(Exception):
    """The MPC refuses to continue. The framework turns any policy exception
    into zero actions (state 2), which needs an explicit operator request to leave."""

@dataclass
class VehicleParamsSettings:
    """Runtime carrier of the identification VehicleParams (vehicle.* parameters).

    Field names, units and conventions match offline/vehicle_identification/
    vehicle_params.py and docs/vehicle-params-test-plan.md: metres, radians
    (left positive), seconds, normalized drive/steering requests. ROS parameters
    cannot be None, so valid=False marks the record as NOT approved for the car;
    its zero defaults are never read as facts. valid=True needs a measured source
    label and passes every consistency check below. The longitudinal coefficients
    (mass, motor gain above the drive deadband, brake gain below it, quadratic
    drag) are the numeric drive_response_model of offline/mpc_prediction_model;
    drive_breakaway/breakaway_wait_s describe static friction at rest, which the
    prediction model does not contain. speed/braking/margin are Planning's limits.
    """
    valid: bool = False
    source: str = "unmeasured"
    wheelbase_m: float = 0.0
    rear_axle_from_cg_m: float = 0.0
    body_front_extent_from_cg_m: float = 0.0
    body_rear_extent_from_cg_m: float = 0.0
    body_width_m: float = 0.0
    steering_gain_rad: float = 0.0          # delta = gain*steering_action + offset
    steering_offset_rad: float = 0.0
    steering_min_rad: float = 0.0
    steering_max_rad: float = 0.0
    steering_rate_limit_rad_s: float = 0.0
    drive_min: float = 0.0
    drive_max: float = 0.0
    drive_response_model: str = ""         # free-text record of the fitted equation and its range
    braking_behavior: str = ""             # measured zero/negative-drive and ESC rules, free text
    steering_delay_s: float = 0.0
    drive_delay_s: float = 0.0
    mass_kg: float = 0.0
    motor_gain_n: float = 0.0
    drag_kg_per_m: float = 0.0
    drive_deadband: float = 0.0            # no drive force below this request
    brake_gain_n: float = 0.0              # force per unit request below the deadband (ESC drag brake)
    drive_breakaway: float = 0.0           # request needed to start from rest, 0 = no static friction
    breakaway_wait_s: float = 0.0          # how long drive_breakaway must hold before the wheels turn
    speed_max_mps: float = 0.0
    braking_deceleration_mps2: float = 0.0
    safety_margin_m: float = 0.0

    NOT_MEASURED = ("unmeasured", "course_simulation", "offline_test_assumption")
    TEXT_FIELDS = ("source", "drive_response_model", "braking_behavior")

    def __post_init__(self):
        if not isinstance(self.valid, bool) or not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("vehicle.valid must be bool and vehicle.source a nonempty label")
        for name, value in vars(self).items():
            if name in self.TEXT_FIELDS:
                if not isinstance(value, str):
                    raise ValueError(f"vehicle.{name} must be a string")
            elif name != "valid" and not finite_number(value):
                raise ValueError(f"vehicle.{name} must be finite")
        if not self.valid:
            return
        if self.source in self.NOT_MEASURED:
            raise ValueError("vehicle.valid requires a measured source label")
        problem = self.problem()
        if problem:
            raise ValueError(f"vehicle parameters inconsistent: {problem}")

    def problem(self):
        """First inconsistency as a short reason, or None. Does not look at valid/source."""
        wheelbase, rear = self.wheelbase_m, self.rear_axle_from_cg_m
        if not 0 < rear < wheelbase:
            return "need 0 < rear_axle_from_cg_m < wheelbase_m"
        if (self.body_front_extent_from_cg_m < wheelbase-rear or self.body_rear_extent_from_cg_m < rear
                or self.body_width_m <= 0):
            return "body extents must enclose the axles and width must be positive"
        if abs(self.steering_gain_rad) < 1e-3:
            return "steering_gain_rad must be nonzero"
        if not (-math.pi/2 < self.steering_min_rad <= self.steering_offset_rad
                <= self.steering_max_rad < math.pi/2) or self.steering_min_rad >= self.steering_max_rad:
            return "need -pi/2 < steering_min <= offset <= steering_max < pi/2"
        if self.steering_rate_limit_rad_s <= 0:
            return "steering_rate_limit_rad_s must be positive"
        if not -1 <= self.drive_min < self.drive_max <= 1 or self.drive_max <= 0:
            return "need -1 <= drive_min < drive_max <= 1 with drive_max > 0"
        if not (0 <= self.steering_delay_s <= 1 and 0 <= self.drive_delay_s <= 1):
            return "delays must lie in [0, 1] s"
        if self.mass_kg <= 0 or self.motor_gain_n <= 0 or self.drag_kg_per_m < 0:
            return "need mass_kg > 0, motor_gain_n > 0, drag_kg_per_m >= 0"
        if not 0 <= self.drive_deadband < self.drive_max or self.brake_gain_n < 0:
            return "need 0 <= drive_deadband < drive_max and brake_gain_n >= 0"
        if (self.drive_breakaway and not self.drive_deadband <= self.drive_breakaway <= self.drive_max
                or not 0 <= self.breakaway_wait_s <= 2):
            return "need drive_deadband <= drive_breakaway <= drive_max and breakaway_wait_s in [0, 2]"
        if self.speed_max_mps <= 0 or self.braking_deceleration_mps2 <= 0 or self.safety_margin_m < 0:
            return "need speed_max_mps > 0, braking_deceleration_mps2 > 0, safety_margin_m >= 0"
        return None

    def records(self):
        """estimation_output vehicle_params / vehicle_limits records built from this one."""
        rear, front, half = self.body_rear_extent_from_cg_m, self.body_front_extent_from_cg_m, self.body_width_m/2
        delay = max(self.steering_delay_s, self.drive_delay_s)
        params = {"valid": self.valid, "source": self.source,
                  "vehicle_reference_point": "cg_ground_projection",
                  "effective_wheelbase_m": self.wheelbase_m,
                  "rear_axle_from_cg_m": self.rear_axle_from_cg_m,
                  "steering_offset": self.steering_offset_rad,
                  "steering_map": {"type": "linear", "gain_rad": self.steering_gain_rad,
                                   "offset_rad": self.steering_offset_rad},
                  "steering_limit": [self.steering_min_rad, self.steering_max_rad],
                  "steering_rate_limit_rad_s": self.steering_rate_limit_rad_s,
                  "drive_range": [self.drive_min, self.drive_max],
                  "drive_response": {"type": "deadband_linear_force_quadratic_drag", "mass_kg": self.mass_kg,
                                     "motor_gain_n": self.motor_gain_n, "drag_kg_per_m": self.drag_kg_per_m,
                                     "drive_deadband": self.drive_deadband, "brake_gain_n": self.brake_gain_n,
                                     "drive_breakaway": self.drive_breakaway,
                                     "breakaway_wait_s": self.breakaway_wait_s,
                                     "description": self.drive_response_model},
                  "braking_behavior": self.braking_behavior,
                  "steering_delay_s": self.steering_delay_s, "drive_delay_s": self.drive_delay_s,
                  "actuation_delay_s": delay,
                  # Calibrated PolicyController view of the same record: angle reached
                  # at |steer| = 1 on the tighter side, and the sign of the request.
                  "steering_limit_rad": min(abs(self.steering_min_rad), abs(self.steering_max_rad)),
                  "steering_direction": 1.0 if self.steering_gain_rad >= 0 else -1.0}
        tightest = min(abs(self.steering_min_rad), abs(self.steering_max_rad))
        limits = {"valid": self.valid, "source": self.source,
                  "vehicle_reference_point": "cg_ground_projection",
                  "footprint_xy_m": [(-rear, -half), (front, -half), (front, half), (-rear, half)],
                  "wheelbase_m": self.wheelbase_m, "rear_axle_x_m": -self.rear_axle_from_cg_m,
                  "curvature_limit_1pm": math.tan(tightest)/self.wheelbase_m,
                  "speed_max_mps": self.speed_max_mps,
                  "acceleration_max_mps2": self.motor_gain_n*(self.drive_max-self.drive_deadband)/self.mass_kg,
                  "braking_deceleration_mps2": self.braking_deceleration_mps2,
                  "actuation_delay_s": delay, "safety_margin_m": self.safety_margin_m}
        return params, limits


def course_simulation_vehicle_params():
    """Course notebook bicycle_model_config as a vehicle record: simulation, NEVER valid.

    Geometry, steering and drive response are the notebook values validated in
    offline/mpc_prediction_model. Speed/braking/margin repeat Planning's labelled
    offline assumptions (planning.simulation_*), not notebook facts.
    """
    return VehicleParamsSettings(
        valid=False, source="course_simulation", wheelbase_m=0.33, rear_axle_from_cg_m=0.132,
        body_front_extent_from_cg_m=0.297, body_rear_extent_from_cg_m=0.198, body_width_m=0.25,
        steering_gain_rad=math.pi/4, steering_offset_rad=0.0, steering_min_rad=-math.pi/4,
        steering_max_rad=math.pi/4, steering_rate_limit_rad_s=math.pi/2, drive_min=-1.0, drive_max=1.0,
        drive_response_model="course notebook: m*dv/dt = motor_gain_n*drive - drag_kg_per_m*v*|v|",
        braking_behavior="course notebook: direction-change latch, NOT modelled; forward drive only",
        steering_delay_s=0.0, drive_delay_s=0.0, mass_kg=3.0, motor_gain_n=10.0, drag_kg_per_m=1.0,
        drive_deadband=0.0, brake_gain_n=10.0,
        speed_max_mps=0.5, braking_deceleration_mps2=0.5, safety_margin_m=0.05)

def course_model_step(state, control, dt, p, rate_limited=True):
    """One RK4 step of offline/mpc_prediction_model/vehicle_model.step, in plain floats.

    state = (x, y, heading, vx, delta) at the CG; control = (drive, steering_action);
    p is a VehicleParamsSettings. rate_limited=False drops the steering range and
    rate clips: used only to linearise inside the MPC, whose constraints keep both
    clips inactive. No reverse latch: callers must not command negative drive.
    """
    wheelbase, rear = p.wheelbase_m, p.rear_axle_from_cg_m
    target = p.steering_gain_rad*control[1] + p.steering_offset_rad
    if rate_limited:
        target = min(p.steering_max_rad, max(p.steering_min_rad, target))
        rate = min(p.steering_rate_limit_rad_s, max(-p.steering_rate_limit_rad_s, (target-state[4])/dt))
    else:
        rate = (target-state[4])/dt
    effort = control[0] - p.drive_deadband
    force = (p.motor_gain_n if effort >= 0 else p.brake_gain_n)*effort

    def derivative(z):
        psi, vx, delta = z[2], z[3], z[4]
        yaw_rate = vx*math.tan(delta)/wheelbase
        vy = rear*yaw_rate
        return (vx*math.cos(psi) - vy*math.sin(psi), vx*math.sin(psi) + vy*math.cos(psi), yaw_rate,
                (force - p.drag_kg_per_m*vx*abs(vx))/p.mass_kg, rate)

    k1 = derivative(state)
    k2 = derivative([s + dt*k/2 for s, k in zip(state, k1)])
    k3 = derivative([s + dt*k/2 for s, k in zip(state, k2)])
    k4 = derivative([s + dt*k for s, k in zip(state, k3)])
    return [s + dt*(a + 2*b + 2*c + d)/6 for s, a, b, c, d in zip(state, k1, k2, k3, k4)]

@dataclass
class MPCSettings:
    """Vehicle-model MPC: algorithm settings only; the car comes from vehicle.*.

    Prediction uses course_model_step (the validated course prediction model) and
    optimises normalized drive and steering together. vehicle_params_source picks
    the measured vehicle.* record or the course-simulation record, which is shadow
    only. shadow=False is refused unless vehicle.valid (measured) and a positive
    v_exec_max_mps were set deliberately.
    """
    enabled: bool = False
    shadow: bool = True
    # Explicit low-speed hardware experiment with UNMEASURED Gym assumptions.
    # This does not turn vehicle.valid true or claim calibration.
    gym_trial_acknowledged: bool = False
    speed_feedback_enabled: bool = False
    reference_join_repair_enabled: bool = False
    learned_planner_speed_match_enabled: bool = False
    # Kinematic steering preview at the fresh measured wheel speed. This is
    # zero-order hold, NOT an identified acceleration/braking model.
    speed_feedback_measured_motion: bool = False
    speed_feedback_continuous_enabled: bool = False
    speed_feedback_drive_slew_per_s: float = 2.0
    speed_feedback_kp: float = 2.0
    speed_feedback_ki: float = 0.5
    speed_feedback_start_drive: float = 0.3
    speed_feedback_start_pulse_s: float = 0.15
    speed_feedback_drive_max: float = 0.35
    # Experimental sustaining effort, not a calibrated force-model deadband.
    speed_feedback_sustain_drive: float = 0.0
    speed_feedback_accel_tau_s: float = 0.05
    speed_feedback_coast_margin_mps: float = 0.01
    trial_fixed_drive: float = 0.0
    trial_follow_planning: bool = False  # no fixed duration; abnormal-state gates remain
    vehicle_params_source: str = "vehicle"
    v_exec_max_mps: float = 0.0       # execution speed cap; must be > 0 for shadow: false
    adaptive_horizon_enabled: bool = False
    adaptive_horizon_min_n: int = 2
    reference_end_margin_m: float = 0.02
    overspeed_margin_mps: float = 0.1
    overspeed_stop_enabled: bool = True  # explicit trial override; target control still active
    # Execution only: a reference rejection shorter than this coasts (drive 0, steering
    # held) instead of locking; a longer or any other rejection still locks. 0 = lock at once.
    reference_dropout_tolerance_s: float = 0.0
    # Start from rest (vehicle.drive_breakaway > 0): hold the breakaway drive until the
    # wheel speed reaches moving_speed_mps; no motion within breakaway_timeout_s locks.
    # Coming back to rest while tracking restarts at most max_restarts times per run.
    moving_speed_mps: float = 0.05
    breakaway_timeout_s: float = 1.5
    # The start drive rises from vehicle.drive_breakaway at this rate (per s, capped at
    # vehicle.drive_max): the needed effort drifts with the battery. 0 = constant.
    breakaway_ramp_per_s: float = 0.0
    max_restarts: int = 3
    # Drive-bias estimate (integral action on the model's one-step speed residual):
    # bias += drive_bias_gain * (predicted - measured speed) * dt, clipped to +/-drive_bias_max.
    # It shifts the effective deadband. 0 gain = off.
    drive_bias_gain: float = 0.0
    drive_bias_max: float = 0.05
    # Late updates: dt_max_s < dt <= dt_hard_max_s is a tolerable (plan-hold) rejection,
    # dt above dt_hard_max_s locks.
    dt_hard_max_s: float = 0.4
    horizon_n: int = 10
    dt_pred_s: float = 0.1            # fixed prediction step; measured dt is only range-checked
    dt_max_s: float = 0.2
    sqp_iterations: int = 1            # 1 = real-time iteration on the shifted previous solution
    q_ey: float = 20.0
    q_epsi: float = 5.0
    q_v: float = 10.0
    r_delta: float = 1.0
    r_ddelta: float = 2.0
    r_drive: float = 1.0
    r_ddrive: float = 2.0
    terminal_scale: float = 5.0
    start_bridge_enabled: bool = False  # bounded ordered-centerline start admission
    max_backward_extension_m: float = 0.6
    reference_spacing_m: float = 0.05
    max_step_time_s: float = 0.05
    solver_time_limit_s: float = 0.03
    solver_max_iter: int = 400
    constraint_tolerance: float = 1e-4
    # QP solver. "osqp" needs the osqp package; "dense" is an exact dual active-set
    # (Goldfarb-Idnani) solver using numpy only, for computers without osqp.
    qp_solver: str = "osqp"
    # Reference source. "planning" consumes the Planning V1 reference (straight roads only).
    # "estimation_centerline" BYPASSES Planning: a quadratic fit of the estimator's centerline,
    # so bends are possible but Planning's body-clearance and stopping-distance checks are lost.
    reference_source: str = "planning"
    bypass_acknowledged: bool = False
    bypass_target_speed_mps: float = 0.2
    bypass_max_curvature_1pm: float = 0.8
    bypass_max_fit_error_m: float = 0.04
    bypass_min_forward_m: float = 1.0
    bypass_fit_max_x_m: float = 1.5   # fit window: a quadratic is poor far ahead on a bend
    bypass_max_source_age_s: float = 0.2
    bypass_valid_for_s: float = 0.1

    def __post_init__(self):
        if self.start_bridge_enabled and not 0 < self.max_backward_extension_m <= 1.2:
            raise ValueError("start bridge requires extension in (0, 1.2] m")
        if not isinstance(self.adaptive_horizon_enabled, bool):
            raise ValueError("adaptive_horizon_enabled must be bool")
        if (isinstance(self.adaptive_horizon_min_n, bool) or not isinstance(self.adaptive_horizon_min_n, int)
                or not 2 <= self.adaptive_horizon_min_n <= self.horizon_n):
            raise ValueError("adaptive_horizon_min_n outside horizon")
        if not finite_number(self.reference_end_margin_m) or not 0 <= self.reference_end_margin_m <= 0.2:
            raise ValueError("invalid reference_end_margin_m")
        for name in ("enabled", "shadow", "bypass_acknowledged", "gym_trial_acknowledged", "trial_follow_planning", "speed_feedback_enabled", "reference_join_repair_enabled", "learned_planner_speed_match_enabled", "speed_feedback_measured_motion", "overspeed_stop_enabled", "start_bridge_enabled", "speed_feedback_continuous_enabled"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"mpc.{name} must be bool")
        if self.reference_source not in ("planning", "estimation_centerline"):
            raise ValueError("mpc.reference_source must be planning or estimation_centerline")
        if self.vehicle_params_source not in ("vehicle", "course_simulation", "partial_vehicle"):
            raise ValueError("mpc.vehicle_params_source must be vehicle or course_simulation")
        if self.qp_solver not in ("osqp", "dense"):
            raise ValueError("mpc.qp_solver must be osqp or dense")
        for name in ("horizon_n", "sqp_iterations", "solver_max_iter", "max_restarts"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"mpc.{name} must be an integer")
        for name, value in vars(self).items():
            if not isinstance(value, (bool, str)) and not finite_number(value):
                raise ValueError(f"mpc.{name} must be finite")
        if not 2 <= self.horizon_n <= 50 or not 1 <= self.sqp_iterations <= 5:
            raise ValueError("mpc.horizon_n must be in [2, 50] and mpc.sqp_iterations in [1, 5]")
        positive = ("dt_pred_s", "dt_max_s", "q_ey", "q_epsi", "q_v", "r_delta", "r_ddelta", "r_drive",
                    "r_ddrive", "terminal_scale", "overspeed_margin_mps", "max_backward_extension_m",
                    "reference_spacing_m", "max_step_time_s", "solver_time_limit_s", "solver_max_iter",
                    "constraint_tolerance", "bypass_target_speed_mps", "bypass_max_curvature_1pm",
                    "bypass_max_fit_error_m", "bypass_min_forward_m", "bypass_fit_max_x_m",
                    "bypass_max_source_age_s", "bypass_valid_for_s")
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f"mpc.{name} must be positive")
        if self.dt_max_s < self.dt_pred_s:
            raise ValueError("mpc.dt_max_s must not be below dt_pred_s")
        if not self.dt_max_s <= self.dt_hard_max_s <= 1.0:
            raise ValueError("mpc.dt_hard_max_s must be in [dt_max_s, 1] s")
        if (not 0 < self.moving_speed_mps <= 0.5 or not 0 < self.breakaway_timeout_s <= 5
                or not 0 <= self.max_restarts <= 10 or not 0 <= self.breakaway_ramp_per_s <= 1):
            raise ValueError("mpc start settings out of range")
        if not 0 <= self.drive_bias_gain <= 10 or not 0 <= self.drive_bias_max <= 0.2:
            raise ValueError("mpc.drive_bias_gain must be in [0, 10] and drive_bias_max in [0, 0.2]")
        if not 0.0 <= self.reference_dropout_tolerance_s <= 0.5:
            raise ValueError("mpc.reference_dropout_tolerance_s must be in [0, 0.5] s")
        if not 0 < self.speed_feedback_drive_slew_per_s <= 10.0:
            raise ValueError("speed_feedback_drive_slew_per_s must be in (0, 10]")
        if (self.speed_feedback_kp < 0 or self.speed_feedback_ki < 0
                or not 0 < self.speed_feedback_start_drive <= self.speed_feedback_drive_max <= 0.36):
            raise ValueError("speed feedback gains/drive limits invalid")
        if (not 0 <= self.speed_feedback_sustain_drive <= self.speed_feedback_start_drive
                or not 0 < self.speed_feedback_accel_tau_s <= 1.0
                or not 0 < self.speed_feedback_coast_margin_mps <= 0.1):
            raise ValueError("speed feedback sustaining/coast settings invalid")
        if not 0 < self.speed_feedback_start_pulse_s <= 0.5:
            raise ValueError("start pulse must be in (0, 0.5] s")
        if self.speed_feedback_measured_motion and not self.speed_feedback_enabled:
            raise ValueError("measured motion prediction requires speed feedback")
        if self.speed_feedback_enabled and (not self.gym_trial_acknowledged or self.trial_fixed_drive != 0):
            raise ValueError("speed feedback requires explicit trial and no fixed drive")
        if self.trial_fixed_drive not in (0.0, 0.35):
            raise ValueError("trial_fixed_drive must be zero or 0.35")
        if self.trial_follow_planning and (not self.gym_trial_acknowledged or not (self.trial_fixed_drive == 0.35 or self.speed_feedback_enabled)):
            raise ValueError("trial_follow_planning requires explicit fixed-drive trial")
        if self.v_exec_max_mps < 0:
            raise ValueError("mpc.v_exec_max_mps must be nonnegative")


def mpc_vehicle_params(cfg, vehicle):
    """The vehicle record the MPC predicts with: course simulation, or the vehicle.* record."""
    if cfg.vehicle_params_source == "course_simulation":
        return course_simulation_vehicle_params()
    if cfg.vehicle_params_source == "partial_vehicle":
        from dataclasses import replace
        if vehicle is None:
            raise ValueError("partial_vehicle requires a supplied parameter record")
        # Measured lateral geometry/map/delay; longitudinal force law is still assumed.
        fields = ('wheelbase_m', 'rear_axle_from_cg_m', 'body_front_extent_from_cg_m', 'body_rear_extent_from_cg_m', 'body_width_m', 'steering_gain_rad', 'steering_offset_rad', 'steering_min_rad', 'steering_max_rad', 'steering_rate_limit_rad_s', 'steering_delay_s', 'drive_delay_s')
        return replace(course_simulation_vehicle_params(), valid=False, source=vehicle.source,
                       **{name: getattr(vehicle, name) for name in fields})
    return vehicle if vehicle is not None else VehicleParamsSettings()

def reference_trajectory_from_planning(ref, spacing_m=0.05):
    """Planning V1/V2 reference -> ReferenceTrajectory v0.1 point list (base_link).

    The original timestamp, lifetime and source ages are passed through unchanged;
    expiry is checked by the consumer. Invalid input yields valid=False/stop_required.
    V2 CARTESIAN_SAMPLES are copied without refitting or resampling. V2
    CARTESIAN_Y_OF_X polynomials are evaluated at their own origin and scale.
    A V2 speed_profile is retained for time-based MPC shadow evaluation.
    """
    out = {"schema": "reference_trajectory_v0.1", "frame_id": None, "timestamp_s": None,
           "valid_for_s": 0.0, "valid": False, "stop_required": True, "status": None,
           "reason": "missing_reference", "path_id": None, "simulation_only": None,
           "source_ages_s": {}, "points": [], "speed_profile": None,
           "speed_profile_timestamp_s": None, "planned_stop": False, "stop_at_path_m": None}
    if not isinstance(ref, dict):
        return out
    out.update(frame_id=ref.get("frame_id"), timestamp_s=ref.get("timestamp_s"),
               valid_for_s=ref.get("valid_for_s", 0.0), status=ref.get("status"),
               reason=ref.get("reason") or ref.get("stop_reason"), path_id=ref.get("trajectory_id"),
               simulation_only=ref.get("simulation_only", False),
               source_ages_s=dict(ref.get("source_ages_s") or {}),
               reference_source="planning", planning_bypassed=False)
    if ref.get("stop_requested") is True and (
            ref.get("valid") is True or ref.get("reason") not in ControlReferenceManager.SOFT_REASONS):
        # Planning failure can have no path. An explicit stop must reach MPC's
        # stop branch even when the reference itself is invalid. V1 and V2 set
        # stop_requested on EVERY failure, so the transient reasons the MVP
        # bridges (SOFT_REASONS: stale/invalid road, coverage, pending async
        # worker, ...) stay bounded dropouts here too; a valid stop or e.g.
        # no_feasible_lattice_trajectory still stops immediately.
        out.update(valid=True, reason=out["reason"] or "planner_stop")
        return out
    if ref.get("valid") is not True:
        out["reason"] = out["reason"] or "planner_stop"
        return out

    planned_stop = ref.get("planned_stop") is True

    def reject(reason):
        # A malformed planned stop is an explicit stop, not a tolerable dropout.
        out.update(valid=planned_stop, reason=reason)
        return out

    if ref.get("frame_id") != "base_link" or not finite_number(ref.get("timestamp_s")):
        return reject("bad_frame_or_timestamp")
    path, speed = ref.get("path"), ref.get("target_speed_mps")
    if (not isinstance(path, dict) or not finite_number(speed) or speed < 0
            or not finite_number(spacing_m) or spacing_m <= 0):
        return reject("bad_path")

    profile = ref.get("speed_profile")
    if profile is not None:
        if (not isinstance(profile, (list, tuple)) or not 2 <= len(profile) <= 601
                or any(not isinstance(row, (list, tuple)) or len(row) != 2
                       or not all(finite_number(v) for v in row)
                       or row[0] < 0 or row[1] < 0 for row in profile)
                or abs(profile[0][0]) > 1e-6
                or any(b[0] <= a[0] for a, b in zip(profile, profile[1:]))):
            return reject("bad_speed_profile")
        profile = [tuple(row) for row in profile]
        # Transport advances timestamp_s; the profile keeps its planning-time origin.
        profile_stamp = ref.get("speed_profile_timestamp_s")
        profile_stamp = ref["timestamp_s"] if profile_stamp is None else profile_stamp
        if not finite_number(profile_stamp) or profile_stamp > ref["timestamp_s"] + 1e-9:
            return reject("bad_speed_profile")
    else:
        profile_stamp = None
    stop_at = ref.get("stop_at_path_m")
    if planned_stop and (profile is None or profile[-1][1] > 1e-6
                         or not finite_number(stop_at) or stop_at < 0):
        return reject("bad_planned_stop")
    out.update(speed_profile=profile, speed_profile_timestamp_s=profile_stamp, planned_stop=planned_stop,
               stop_at_path_m=stop_at if planned_stop else None)

    # New 2026-10-15: main's road_centerline output (ORDERED_CENTERLINE,
    # arc-length s, no x-monotone assumption). Resampled at spacing_m along s
    # from the path's own near end (s=0 is the end closest to the car body,
    # per _estimation_order_boundary); mpc_path_projector then re-derives the
    # per-step nearest point, so only even spacing/finite values matter here.
    if path.get("type") == "ORDERED_CENTERLINE":
        xy = _path_points(path.get("samples_xy"))
        if xy is None or path.get("independent_variable") != "s_m":
            return reject("bad_ordered_path")
        lengths = _path_lengths(xy)
        total_s = lengths[-1]
        if not finite_number(total_s) or total_s <= 0:
            return reject("bad_ordered_path")
        sample_intervals = total_s/spacing_m
        if not finite_number(sample_intervals) or sample_intervals > 199:
            return reject("path_too_long")
        raw = []
        for i in range(int(sample_intervals)+2):
            s = min(total_s, i*spacing_m)
            px, py = _path_sample(xy, lengths, s)
            tx, ty = _path_tangent(xy, lengths, s)
            near_duplicate = raw and s-raw[-1][0] < 0.25*spacing_m
            if near_duplicate and s < total_s:
                continue
            if near_duplicate:
                # The forced final point (s=total_s) must still be appended even
                # when close to the previous one: replace it rather than adding a
                # near-duplicate, which mpc_reference_errors rejects as a zero-
                # length segment (repeated_point).
                raw[-1] = (s, px, py, math.atan2(ty, tx))
            else:
                raw.append((s, px, py, math.atan2(ty, tx)))
            if s >= total_s:
                break
        if len(raw) < 2:
            return reject("path_too_short")
        # Curvature from the heading change over the resampled spacing (central
        # difference; one-sided at the ends), clipped as a safety net against
        # cone noise. It is only a steering feedforward term, not a constraint.
        points = []
        for i, (s, px, py, yaw) in enumerate(raw):
            lo = raw[i-1] if i > 0 else raw[i]
            hi = raw[i+1] if i < len(raw)-1 else raw[i]
            ds = hi[0]-lo[0]
            kappa = 0.0 if ds <= 1e-6 else max(-5.0, min(5.0, wrap_angle(hi[3]-lo[3])/ds))
            points.append({"s_m": s, "x_m": px, "y_m": py, "yaw_rad": yaw,
                           "curvature_1pm": kappa, "target_speed_mps": speed})
        out.update(valid=True, stop_required=False, reason=None, points=points,
                   reference_path_type="ORDERED_CENTERLINE")
        return out

    transported = "control_cached_path_samples" in ref
    if transported or path.get("type") == "CARTESIAN_SAMPLES":
        samples = ref.get("control_cached_path_samples") if transported else path.get(
            "samples_xy_heading_curvature")
        if (path.get("independent_variable") != "x_m"
                or path.get("type") not in ("CARTESIAN_SAMPLES", "CARTESIAN_Y_OF_X")
                or not isinstance(samples, (list, tuple)) or not 2 <= len(samples) <= 201
                or any(not isinstance(p, (list, tuple)) or len(p) != 4
                       or not all(finite_number(v) for v in p) for p in samples)
                or any(b[0] <= a[0] for a, b in zip(samples, samples[1:]))
                or samples[-1][0] <= max(0.0, samples[0][0])
                or any(abs(p[2]) >= math.pi/2 for p in samples)):
            return reject("bad_sample_path")
        if not transported:
            bounds = path.get("range")
            if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                    or not all(finite_number(v) for v in bounds)
                    or bounds[0] != samples[0][0] or bounds[1] != samples[-1][0]):
                return reject("bad_sample_path")
        points, distance = [], 0.0
        for i, (x, y, heading, curvature) in enumerate(samples):
            if i:
                distance += math.hypot(x-samples[i-1][0], y-samples[i-1][1])
            points.append({"s_m": distance, "x_m": x, "y_m": y, "yaw_rad": heading,
                           "curvature_1pm": curvature, "target_speed_mps": speed})
        out.update(valid=True, stop_required=False, reason=None, points=points)
        return out

    coeffs, bounds = path.get("coeffs_low_to_high"), path.get("range")
    origin, scale = path.get("origin"), path.get("scale")
    if (path.get("type") != "CARTESIAN_Y_OF_X" or path.get("independent_variable") != "x_m"
            or not isinstance(coeffs, (list, tuple)) or not 1 <= len(coeffs) <= 6
            or not all(finite_number(v) for v in coeffs)
            or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(finite_number(v) for v in bounds)
            or not finite_number(origin) or not finite_number(scale) or scale <= 0
            or bounds[1] <= bounds[0] or not finite_number(bounds[1]-bounds[0])):
        return reject("bad_path")
    lo, hi = bounds

    def evaluate_at(x):
        if len(coeffs) == 2:
            return evaluate_planning_path(ref, x, ref["timestamp_s"])
        # Planning V2 encodes y(x)=sum(c_i*u^i), u=(x-origin)/scale.
        # Horner's rule evaluates y and its first two derivatives together.
        try:
            u = (x-origin)/scale
            value, first, second = coeffs[-1], 0.0, 0.0
            for coefficient in reversed(coeffs[:-1]):
                second = second*u + 2*first
                first = first*u + value
                value = value*u + coefficient
            slope, bend = first/scale, second/(scale*scale)
            curvature = bend/math.hypot(1.0, slope)**3
        except (OverflowError, ZeroDivisionError):
            return None
        if not all(finite_number(v) for v in (value, slope, bend, curvature)):
            return None
        return {"position_xy_m": (x, value), "heading_rad": math.atan(slope),
                "curvature_1pm": curvature}

    start = evaluate_at(lo)
    if start is None:
        return reject("path_not_evaluable")
    heading = start["heading_rad"]
    step_x = spacing_m*math.cos(heading)
    if not finite_number(step_x) or step_x <= 0:
        return reject("bad_path")
    sample_intervals = (hi-lo)/step_x
    if not finite_number(sample_intervals) or sample_intervals > 199:
        return reject("path_too_long")
    count = int(sample_intervals) + 1
    points = []
    for i in range(count+1):
        x = min(hi, lo + i*step_x)
        sample = evaluate_at(x)
        if sample is None:
            return reject("path_not_evaluable")
        px, py = sample["position_xy_m"]
        s = 0.0 if not points else points[-1]["s_m"] + math.hypot(px-points[-1]["x_m"], py-points[-1]["y_m"])
        if points and s - points[-1]["s_m"] < 0.25*spacing_m:
            break   # the clipped end can land a hair past the previous sample: no near-duplicate
        points.append({"s_m": s, "x_m": px, "y_m": py, "yaw_rad": sample["heading_rad"],
                       "curvature_1pm": sample["curvature_1pm"], "target_speed_mps": speed})
    if len(points) < 2:
        return reject("path_too_short")
    out.update(valid=True, stop_required=False, reason=None, points=points)
    return out

def mpc_reference_errors(traj, n, dt_pred, speed, max_extension, start_heading_rad=None):
    """Vehicle (origin, heading 0 in base_link) against a v0.1 point list.

    e_y > 0: vehicle left of the path. e_psi = vehicle heading - path heading.
    Behind the first point the first segment is extended by at most max_extension
    (only meaningful for a straight reference; do not inherit this for curves).
    Returns (info, None) or (None, reason). No end-point repetition: if the
    N-step travel is not covered by the path the reference is rejected.
    """
    points = traj.get("points") if isinstance(traj, dict) else None
    if traj.get("frame_id") != "base_link" or not isinstance(points, list) or len(points) < 2:
        return None, "bad_points"
    keys = ("s_m", "x_m", "y_m", "curvature_1pm", "target_speed_mps")
    for p in points:
        if not isinstance(p, dict) or not all(finite_number(p.get(k)) for k in keys):
            return None, "nonfinite_point"
    cumulative, best = [0.0], None
    for a, b in zip(points, points[1:]):
        length = math.hypot(b["x_m"]-a["x_m"], b["y_m"]-a["y_m"])
        if length < 1e-6:
            return None, "repeated_point"
        cumulative.append(cumulative[-1]+length)
    # Use the same active-branch selection as the solver, including the
    # forward-heading filter at hairpins. Do not let a nearby return leg
    # reject a reference which the optimisation projects onto the outbound leg.
    projection = mpc_path_projector(points, max_extension, start_heading_rad).details(0.0, 0.0)
    px, py, theta, _, _, _ = projection["projection"]
    if projection["before_start"]:
        return None, "origin_before_path_start"
    if projection["past_end"]:
        return None, "origin_past_path_end"
    i, t = projection["segment_index"], projection["raw_fraction"]
    s_proj = projection["s_proj_m"]
    horizon = speed*dt_pred*n
    if s_proj + horizon > cumulative[-1] + 1e-9:
        return None, "reference_too_short"
    kappas = []
    for k in range(n):
        s_k = s_proj + speed*dt_pred*k
        j = max(0, min(len(points)-1, sum(1 for c in cumulative if c <= s_k)-1))
        kappas.append(points[j]["curvature_1pm"])
    return {"e_y_m": px*math.sin(theta) - py*math.cos(theta), "e_psi_rad": wrap_angle(-theta),
            "kappas": kappas, "s_proj_m": s_proj, "path_remaining_m": cumulative[-1]-s_proj,
            "extended_start": s_proj < 0.0, "v_ref_mps": points[0]["target_speed_mps"]}, None

def mpc_path_projector(points, max_extension, start_heading_rad=None):
    """Shared projection for admission, diagnostics and MPC prediction.

    Prefer segments facing within about 105 degrees of body +x. If none do,
    use all segments. The filter is fixed over a solve; it does not certify
    global route progress at intersections. Never extend the path past its end.
    """
    segments, cumulative = [], [0.0]
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]
        length2 = dx*dx+dy*dy
        length = math.sqrt(length2)
        segments.append((a, b, dx, dy, length2, -max_extension/length if i == 0 else 0.0))
        cumulative.append(cumulative[-1]+length)
    forward_indices = [i for i, (_, _, dx, _, length2, _) in enumerate(segments)
                       if dx/math.sqrt(length2) >= -0.25] or list(range(len(segments)))

    def details(x, y):
        best = None
        for i in forward_indices:
            a, b, dx, dy, length2, low = segments[i]
            t = ((x-a["x_m"])*dx+(y-a["y_m"])*dy)/length2
            clamped = min(1.0, max(0.0 if i == 0 and start_heading_rad is not None else low, t))
            px, py = a["x_m"]+clamped*dx, a["y_m"]+clamped*dy
            distance = (px-x)**2+(py-y)**2
            if best is None or distance < best[0]:
                best = (distance, i, t, clamped, px, py, math.atan2(dy, dx),
                        a if clamped < 0.5 else b, 0.0 if i == 0 and start_heading_rad is not None else low)
            if i == 0 and start_heading_rad is not None:
                tx, ty = math.cos(start_heading_rad), math.sin(start_heading_rad)
                along = (x-a["x_m"])*tx+(y-a["y_m"])*ty
                if along < 0:
                    bounded = max(-max_extension, along)
                    bx, by = a["x_m"]+bounded*tx, a["y_m"]+bounded*ty
                    distance = (bx-x)**2+(by-y)**2
                    if best is None or distance < best[0]:
                        length = math.sqrt(length2)
                        best = (distance, 0, along/length, bounded/length, bx, by,
                                start_heading_rad, a, -max_extension/length)
        _, i, t, clamped, px, py, heading, near, low = best
        s = cumulative[i]+clamped*(cumulative[i+1]-cumulative[i])
        past = i == len(segments)-1 and t > 1.0+1e-9
        before = i == 0 and t < low-1e-9
        if i == 0 and start_heading_rad is not None:
            # The admitted start ray uses the fitted heading, not the first
            # short segment's tangent. A cropped lateral foot can be slightly
            # behind that segment while already ahead of the fitted start.
            first = points[0]
            fitted_along = ((x-first["x_m"])*math.cos(start_heading_rad)
                            +(y-first["y_m"])*math.sin(start_heading_rad))
            before = fitted_along < -max_extension-1e-9
        return {"projection": (px, py, heading, 0.0 if i == 0 and clamped < 0 and start_heading_rad is not None else near["curvature_1pm"],
                                near["target_speed_mps"], past),
                "segment_index": i, "raw_fraction": t, "s_proj_m": s,
                "path_remaining_m": cumulative[-1]-s, "past_end": past,
                "before_start": before}

    def project(x, y):
        return details(x, y)["projection"]
    project.details = details
    return project

class VehicleModelMPC:
    """LTV-MPC on the course prediction model, solved as a condensed QP with OSQP.

    State z = [x, y, heading, v, delta] in base_link at the current reference time,
    input u = [drive, steering_action] (normalized). Each SQP iteration linearises
    course_model_step (without its clips) along the nominal input sequence; the QP
    bounds keep both clips inactive: steering range and rate, and forward-only drive.
    Cost: lateral and heading error to the projected path point, speed error,
    steering against curvature feedforward, drive against its steady-state value,
    and changes of both inputs.

    Drive deadband: while the car moves, drive is bounded below by the effective
    deadband (vehicle.drive_deadband + the controller's bias estimate), so the QP
    works only where the model is linear (force = motor_gain*(drive - deadband)).
    A drive at the bound means no drive force; requests below it would engage
    the ESC drag brake, which this optimisation does not plan with.

    With mpc.qp_solver "osqp", one OSQP object is set up on the first solve and
    updated afterwards: the constraint matrix is constant and P keeps a fixed dense
    upper-triangular pattern, so only P values, q and the bounds change (warm-started).
    With "dense", each QP is solved exactly from scratch by _solve_dense (numpy only).
    """

    def __init__(self, cfg, vehicle):
        self.cfg, self.vehicle = cfg, vehicle
        try:
            import numpy
            if cfg.qp_solver == "osqp":
                import osqp
                from scipy import sparse
            else:
                osqp = sparse = None
        except ImportError as exc:
            raise RuntimeError(f"MPC qp_solver {cfg.qp_solver} needs numpy"
                               f"{', scipy and osqp' if cfg.qp_solver == 'osqp' else ''}: {exc}") from exc
        self._np, self._osqp, self._sparse = numpy, osqp, sparse
        self._model = None
        rows, cols = numpy.triu_indices(2*cfg.horizon_n)
        order = numpy.lexsort((rows, cols))          # CSC order: by column, then row
        self._triu = (rows[order], cols[order])

    def effective(self, drive_bias=0.0):
        """The vehicle record with the deadband shifted by the bias estimate."""
        from dataclasses import replace    # local: offline harnesses load this class alone
        p = self.vehicle
        if not drive_bias:
            return p
        deadband = min(p.drive_max-1e-3, max(0.0, p.drive_deadband+drive_bias))
        return replace(p, drive_deadband=deadband,
                       drive_breakaway=max(p.drive_breakaway, deadband) if p.drive_breakaway else 0.0)

    def input_bounds(self, p=None, moving=False, allow_braking=False):
        p = self.vehicle if p is None else p
        drive = (max(0.0, p.drive_min, p.drive_deadband if moving and not allow_braking else 0.0), p.drive_max)
        ends = sorted(((p.steering_min_rad-p.steering_offset_rad)/p.steering_gain_rad,
                       (p.steering_max_rad-p.steering_offset_rad)/p.steering_gain_rad))
        if self.cfg.speed_feedback_enabled:
            return (drive[0], min(drive[1], self.cfg.speed_feedback_drive_max)), (max(-0.5, ends[0]), min(0.5, ends[1]))
        if self.cfg.gym_trial_acknowledged and self.cfg.trial_fixed_drive:
            return (0.35, 0.35), (max(-0.2, ends[0]), min(0.2, ends[1]))
        if self.cfg.gym_trial_acknowledged:
            # Trial caps constrain the optimisation itself, not just final output.
            return (drive[0], min(drive[1], 0.2)), (max(-0.2, ends[0]), min(0.2, ends[1]))
        return drive, (max(-1.0, ends[0]), min(1.0, ends[1]))

    def steady_drive(self, speed, p=None):
        p = self.vehicle if p is None else p
        return p.drive_deadband + p.drag_kg_per_m*speed*speed/p.motor_gain_n

    def _motion_step(self, z, u, h, p, rate_limited=True):
        """Same lateral model, optionally hold measured longitudinal speed.

        The balancing effort is internal prediction math; it is NEVER sent to
        the car. The actual normalized drive remains the wheel-speed PI output.
        """
        if self.cfg.speed_feedback_measured_motion:
            u = (self.steady_drive(z[3], p), u[1])
        return course_model_step(z, u, h, p, rate_limited=rate_limited)

    def _linearise(self, z, u, h, p=None):
        """Forward-difference Jacobians of the unclipped step at (z, u)."""
        p, np = self.vehicle if p is None else p, self._np
        base = self._motion_step(z, u, h, p, rate_limited=False)
        A, B = np.zeros((5, 5)), np.zeros((5, 2))
        # The model is translation invariant: x/y enter only as additive
        # positions. These Jacobian columns are exact, without two RK4 calls.
        A[0, 0], A[1, 1] = 1.0, 1.0
        for j in range(2, 5):
            eps = 1e-6*max(1.0, abs(z[j]))
            shifted = list(z)
            shifted[j] += eps
            A[:, j] = (np.array(self._motion_step(shifted, u, h, p, rate_limited=False))-base)/eps
        # In measured-speed mode _motion_step replaces drive internally.
        # Its drive derivative is exactly zero; avoid a redundant RK4 call.
        for j in ((1,) if self.cfg.speed_feedback_measured_motion else range(2)):
            shifted = list(u)
            shifted[j] += 1e-6
            B[:, j] = (np.array(self._motion_step(z, shifted, h, p, rate_limited=False))-base)/1e-6
        return base, A, B

    def _solve_qp(self, P, q, constraints, lower, upper, deadline_s=None):
        """min 0.5 U'PU + q'U, lower <= constraints U <= upper -> (status, U, iterations)."""
        if deadline_s is not None and time.perf_counter() >= deadline_s:
            return "run time limit reached", None, 0
        if self.cfg.qp_solver == "dense":
            return self._solve_dense(P, q, constraints, lower, upper, deadline_s)
        sp, cfg = self._sparse, self.cfg
        qp_limit_s = cfg.solver_time_limit_s
        if deadline_s is not None:
            qp_limit_s = min(qp_limit_s, max(1e-6, deadline_s-time.perf_counter()))
        size = P.shape[0]
        if len(self._triu[0]) != size*(size+1)//2:
            # OSQP's update API cannot change problem dimensions.
            rows, cols = self._np.triu_indices(size)
            order = self._np.lexsort((rows, cols))
            self._triu = (rows[order], cols[order])
            self._model = None
        Px = P[self._triu]
        if self._model is None:
            size = P.shape[0]
            self._model = self._osqp.OSQP()
            self._model.setup(sp.csc_matrix((Px, self._triu), shape=(size, size)), q, sp.csc_matrix(constraints),
                              lower, upper, verbose=False, eps_abs=1e-6, eps_rel=1e-6,
                              max_iter=cfg.solver_max_iter, time_limit=qp_limit_s, polishing=False)
        else:
            self._model.update(Px=Px, q=q, l=lower, u=upper)
            self._model.update_settings(time_limit=qp_limit_s)
        result = self._model.solve()
        return str(result.info.status), result.x, int(result.info.iter)

    def _solve_dense(self, P, q, constraints, lower, upper, deadline_s=None):
        """Goldfarb-Idnani dual active set (Math. Programming 27, 1983) for a positive-definite P.

        Starts from the unconstrained minimum and adds the most violated one-sided
        constraint C_p U >= b_p until none is violated; a constraint whose multiplier
        would turn negative is dropped on the way. Finite and exact up to rounding,
        unlike ADMM. J and R satisfy J'N = [R; 0] for the active normals N and are
        refactorised by one small QR whenever the active set changes. Status words
        follow OSQP; solver_max_iter and solver_time_limit_s bound the work.
        """
        np, cfg = self._np, self.cfg
        t0 = time.perf_counter()
        deadline_s = min(t0+cfg.solver_time_limit_s, deadline_s) if deadline_s is not None else t0+cfg.solver_time_limit_s
        if t0 >= deadline_s:
            return "run time limit reached", None, 0
        C = np.vstack([constraints, -constraints])
        b = np.concatenate([lower, -upper])
        keep = np.isfinite(b)
        C, b = C[keep], b[keep]
        try:
            J0 = np.linalg.inv(np.linalg.cholesky(P)).T      # P^-1 = J0 J0'
        except np.linalg.LinAlgError:
            return "non_convex", None, 0
        x = -J0 @ (J0.T @ q)
        active, u, J, R = [], np.zeros(0), J0, np.zeros((0, 0))
        tol = 1e-9*(1.0 + float(np.max(np.abs(b), initial=0.0)))
        iterations = 0
        while True:
            if time.perf_counter() >= deadline_s:
                return "run time limit reached", x, iterations
            slack = C @ x - b
            p = int(np.argmin(slack))
            if slack[p] >= -tol:
                return "solved", x, iterations
            if p in active:
                return "numerical_error", x, iterations
            n_p, u_plus = C[p], np.append(u, 0.0)
            while True:
                iterations += 1
                if iterations > cfg.solver_max_iter:
                    return "maximum iterations reached", x, iterations
                if time.perf_counter() >= deadline_s:
                    return "run time limit reached", x, iterations
                k = len(active)
                d = J.T @ n_p
                z = J[:, k:] @ d[k:]                         # primal step direction
                r = np.linalg.solve(R, d[:k]) if k else np.zeros(0)
                t1, drop = math.inf, None                    # partial step: a multiplier reaches 0
                for j in range(k):
                    if r[j] > 1e-12 and u_plus[j]/r[j] < t1:
                        t1, drop = u_plus[j]/r[j], j
                zn = float(z @ n_p)
                t2 = (b[p] - float(n_p @ x))/zn if zn > 1e-12 else math.inf   # full step: p satisfied
                t = min(t1, t2)
                if t == math.inf:
                    return "primal infeasible", x, iterations
                if t2 < math.inf:
                    x = x + t*z
                u_plus[:k] -= t*r
                u_plus[k] += t
                if t2 <= t1:
                    active.append(p)
                    u = u_plus
                else:
                    active.pop(drop)
                    u_plus = np.delete(u_plus, drop)
                Q, R_full = np.linalg.qr(J0.T @ C[active].T, mode="complete")
                J, R = J0 @ Q, R_full[:len(active), :len(active)]
                if t2 <= t1:
                    break

    def reset(self):
        """Forget the solver and its warm start, e.g. after a stop or a failure."""
        self._model = None

    def solve(self, points, z0, speed_cap, last_drive, nominal=None, drive_bias=0.0, moving=False,
              speed_targets=None, allow_braking=False, horizon_steps=None, fixed_drive_request=None, start_heading_rad=None, deadline_s=None):
        """Returns a prediction over known support, with unchanged step duration."""
        np, cfg, n, h = self._np, self.cfg, self.cfg.horizon_n, self.cfg.dt_pred_s
        if horizon_steps is not None:
            if (isinstance(horizon_steps, bool) or not isinstance(horizon_steps, int)
                    or not cfg.adaptive_horizon_min_n <= horizon_steps <= n):
                raise MPCStop("invalid_horizon_steps")
            n = horizon_steps
        p = self.effective(drive_bias)
        t0 = time.perf_counter()
        phase_s = {"support_probe": 0.0, "linearisation": 0.0, "assembly": 0.0, "qp": 0.0, "prediction": 0.0}
        self.last_timing = phase_s
        self.last_qp_variables = 0
        self.last_support_steps = None
        cpu_started=time.thread_time()
        def deadline_reached():
            elapsed=time.perf_counter()-t0
            cpu=time.thread_time()-cpu_started
            phase_s["wall_elapsed"]=elapsed
            phase_s["thread_cpu"]=cpu
            phase_s["wall_minus_thread_cpu"]=max(0.,elapsed-cpu)
            return deadline_s is not None and time.perf_counter() >= deadline_s
        if deadline_reached():
            return None, "deadline_exceeded"
        project = mpc_path_projector(points, cfg.max_backward_extension_m, start_heading_rad)
        if (speed_targets is not None and (not isinstance(speed_targets, (list, tuple))
                or len(speed_targets) != n
                or any(not finite_number(v) or v < 0 for v in speed_targets))):
            raise MPCStop("invalid_speed_targets")
        if allow_braking and self.cfg.speed_feedback_measured_motion:
            raise MPCStop("measured_motion_braking_unavailable")
        if allow_braking and (speed_targets is None or p.brake_gain_n <= 0):
            raise MPCStop("braking_model_unavailable")
        (d_lo, d_hi), (s_lo, s_hi) = self.input_bounds(p, moving, allow_braking)
        if fixed_drive_request is not None:
            if not finite_number(fixed_drive_request) or not d_lo <= fixed_drive_request <= d_hi:
                raise MPCStop("invalid_feedback_drive_request")
            d_lo = d_hi = fixed_drive_request
        g, o = p.steering_gain_rad, p.steering_offset_rad
        z0 = [float(v) for v in z0]
        if not all(math.isfinite(v) for v in z0 + [speed_cap if speed_cap != math.inf else 0.0, last_drive]):
            raise MPCStop("solver_nonfinite_input")
        if nominal is None:
            hold = min(s_hi, max(s_lo, (z0[4]-o)/g))
            first_speed = speed_targets[0] if speed_targets is not None else points[0]["target_speed_mps"]
            nominal_drive = 0.0 if allow_braking and first_speed <= 1e-3 else self.steady_drive(min(speed_cap, first_speed), p)
            nominal = [(nominal_drive, hold)]*n
        nominal = [nominal[min(k, len(nominal)-1)] for k in range(n)]
        U_bar = np.array([[min(d_hi, max(d_lo, d)), min(s_hi, max(s_lo, s))] for d, s in nominal]).ravel()
        # Cheap clipped rollout before ANY Jacobian/QP assembly. Choose the
        # first horizon from the same model, nominal controls and finite path,
        # rather than paying for an overlong linearisation and halving later.
        if horizon_steps is not None and speed_targets is None:
            probe_start = time.perf_counter()
            probe, supported = list(z0), 0
            for k in range(n):
                if deadline_reached():
                    phase_s["support_probe"] = time.perf_counter()-probe_start
                    return None, "deadline_exceeded"
                probe = self._motion_step(probe, U_bar[2*k:2*k+2], h, p)
                support = project.details(probe[0], probe[1])
                if support["past_end"] or support["path_remaining_m"] < cfg.reference_end_margin_m-1e-6:
                    break
                supported += 1
            phase_s["support_probe"] = time.perf_counter()-probe_start
            self.last_support_steps = supported
            if supported < cfg.adaptive_horizon_min_n:
                return None, "reference_too_short"
            if supported < n:
                n = supported
                U_bar = U_bar[:2*n]
        iterations = 0
        for _ in range(cfg.sqp_iterations):
            if deadline_reached():
                return None, "deadline_exceeded"
            phase_start = time.perf_counter()
            states, A_list, B_list = [z0], [], []
            for k in range(n):
                if deadline_reached():
                    return None, "deadline_exceeded"
                nxt, A, B = self._linearise(states[-1], U_bar[2*k:2*k+2], h, p)
                states.append(list(nxt))
                A_list.append(A)
                B_list.append(B)
            phase_s["linearisation"] += time.perf_counter()-phase_start
            phase_start = time.perf_counter()
            # x_k = base_k + G_k U  (condensed, k = 1..n)
            G = np.zeros((n, 5, 2*n))
            for k in range(n):
                if k:
                    G[k] = A_list[k] @ G[k-1]
                G[k][:, 2*k:2*k+2] = B_list[k]
            base = np.array(states[1:]) - np.einsum("kij,j->ki", G, U_bar)
            rows, targets, weights = [], [], []

            def add(row, target, weight):
                rows.append(row)
                targets.append(target)
                weights.append(weight)
            reference = []
            for k in range(n):
                zk = states[k+1]
                px, py, theta, kappa, v_point, past_end = project(zk[0], zk[1])
                if past_end:
                    return None, "reference_too_short"
                theta = zk[2] + math.atan2(math.sin(theta-zk[2]), math.cos(theta-zk[2]))
                v_ref = min(speed_targets[k] if speed_targets is not None else v_point, speed_cap)
                reference.append((px, py, theta, kappa, v_ref))
                normal = np.array([-math.sin(theta), math.cos(theta)])
                scale = cfg.terminal_scale if k == n-1 else 1.0
                add(normal @ G[k][0:2], normal @ (np.array([px, py]) - base[k][0:2]), cfg.q_ey*scale)
                add(G[k][2], theta - base[k][2], cfg.q_epsi*scale)
                add(G[k][3], v_ref - base[k][3], cfg.q_v)
                steer_row, drive_row = np.zeros(2*n), np.zeros(2*n)
                steer_row[2*k+1], drive_row[2*k] = g, 1.0
                add(steer_row, math.atan(p.wheelbase_m*kappa) - o, cfg.r_delta)
                drive_ref = 0.0 if allow_braking and v_ref <= 1e-3 else self.steady_drive(v_ref, p)
                add(drive_row, drive_ref, cfg.r_drive)
                d_steer, d_drive = steer_row.copy(), drive_row.copy()
                if k:
                    d_steer[2*k-1], d_drive[2*k-2] = -g, -1.0
                add(d_steer, 0.0 if k else z0[4]-o, cfg.r_ddelta)
                add(d_drive, 0.0 if k else last_drive, cfg.r_ddrive)
            M, b, w = np.array(rows), np.array(targets), np.array(weights)
            P = 2*(M.T*w) @ M
            q = -2*(M.T*w) @ b
            step = p.steering_rate_limit_rad_s*h
            rate = np.zeros((n, 2*n))
            for k in range(n):
                rate[k, 2*k+1] = g
                if k:
                    rate[k, 2*k-1] = -g
            constraints = np.vstack([np.eye(2*n), rate])
            lower = np.concatenate([np.tile([d_lo, s_lo], n), -step*np.ones(n)])
            upper = np.concatenate([np.tile([d_hi, s_hi], n), step*np.ones(n)])
            lower[2*n] += z0[4]-o
            upper[2*n] += z0[4]-o
            phase_s["assembly"] += time.perf_counter()-phase_start
            if deadline_reached():
                return None, "deadline_exceeded"
            phase_start = time.perf_counter()
            # Exact elimination U = fixed + selector*steer for feedback mode.
            # Keep the FULL objective, bounds and rate constraints; only remove
            # the n drive variables that are already equality-constrained.
            if fixed_drive_request is not None:
                fixed = np.zeros(2*n)
                fixed[0::2] = fixed_drive_request
                free = np.arange(1, 2*n, 2)
                reduced = constraints[:, free]
                shifted = constraints @ fixed
                lo, hi = lower-shifted, upper-shifted
                keep = np.any(reduced != 0.0, axis=1)
                if np.any(lo[~keep] > cfg.constraint_tolerance) or np.any(hi[~keep] < -cfg.constraint_tolerance):
                    raise MPCStop("fixed_drive_infeasible")
                self.last_qp_variables = n
                status, x, solver_iterations = self._solve_qp(
                    P[np.ix_(free, free)], (q+P @ fixed)[free],
                    reduced[keep], lo[keep], hi[keep], deadline_s)
                if status == "solved":
                    full = fixed.copy()
                    full[free] = x
                    x = full
            else:
                self.last_qp_variables = 2*n
                status, x, solver_iterations = self._solve_qp(P, q, constraints, lower, upper, deadline_s)
            phase_s["qp"] += time.perf_counter()-phase_start
            if deadline_reached():
                return None, "deadline_exceeded"
            if status != "solved":
                self.reset()
                raise MPCStop(f"solver_status:{status}")
            U = np.asarray(x, dtype=float)
            if not np.all(np.isfinite(U)):
                raise MPCStop("solver_nonfinite")
            row = constraints @ U
            violation = float(max(np.max(row-upper), np.max(lower-row), 0.0))
            if violation > cfg.constraint_tolerance:
                self.reset()
                raise MPCStop("constraint_residual")
            iterations += solver_iterations
            U_bar = np.clip(U, np.tile([d_lo, s_lo], n), np.tile([d_hi, s_hi], n))
        # Report the CLIPPED model's prediction of the accepted inputs.
        phase_start = time.perf_counter()
        predicted, pred_xy, z = [], [], z0
        for k in range(n):
            if deadline_reached():
                return None, "deadline_exceeded"
            z = self._motion_step(z, U_bar[2*k:2*k+2], h, p)
            px, py, theta, _, _, past_end = project(z[0], z[1])
            if horizon_steps is not None and (past_end or
                    project.details(z[0], z[1])["path_remaining_m"] < cfg.reference_end_margin_m-1e-6):
                return None, "reference_too_short"
            predicted.append((-(z[0]-px)*math.sin(theta) + (z[1]-py)*math.cos(theta), z[3], z[4]))
            pred_xy.append((float(z[0]), float(z[1])))
        phase_s["prediction"] += time.perf_counter()-phase_start
        if deadline_reached():
            return None, "deadline_exceeded"
        inputs = [(float(U_bar[2*k]), float(U_bar[2*k+1])) for k in range(n)]
        return {"inputs": inputs, "drive0": inputs[0][0], "steer0": inputs[0][1],
                "delta0": g*inputs[0][1] + o, "violation": violation, "iterations": iterations,
                "pred_e_y": [r[0] for r in predicted], "pred_v": [r[1] for r in predicted],
                "pred_delta": [r[2] for r in predicted], "pred_xy": pred_xy,
                "horizon_steps": n, "solve_s": float(time.perf_counter()-t0)}, None

class MPCController:
    """One step per policy update; every branch fills last_debug before returning/raising.

    Shadow: nothing is ever applied (applied = 0, 0) and rejections do not raise, so the
    valid-reference statistics keep accumulating. Non-shadow: any rejection raises MPCStop,
    i.e. a locked stop that needs an explicit state-3 request; there is no automatic recovery.
    The exception is a TOLERABLE rejection (reference unavailable, late update, over budget)
    shorter than reference_dropout_tolerance_s: the last solved input sequence keeps running
    (drive never above the last applied value); without a plan the car coasts, steering held.
    There is no wheel-angle sensor: the initial steering angle and the delayed part of the
    motion are replayed through the model from the commands actually applied.

    Start from rest (vehicle.drive_breakaway > 0): phase "starting" applies the breakaway
    drive with the MPC's steering until the wheels move, then phase "tracking" lets the MPC
    choose the drive above the effective deadband (vehicle.drive_deadband + bias).
    """

    def __init__(self, cfg, vehicle=None, solver=None):
        self.cfg = cfg
        self.vehicle = mpc_vehicle_params(cfg, vehicle)
        self.vehicle_problem = self.vehicle.problem()
        if solver is None and self.vehicle_problem is None:
            solver = VehicleModelMPC(cfg, self.vehicle)
        self.solver = solver
        self.last_debug = {}
        self.reset()

    def reset(self):
        self.applied_steer = 0.0
        self.applied_drive = 0.0
        self.clock_s = 0.0
        self.history = []          # (issue time on clock_s, drive, steer) actually applied
        self.nominal = None
        self.invalid_since = None  # clock_s of the first rejected reference in a row
        self.plan = None           # (clock_s of the solve, input sequence) for plan-hold
        self.phase = "starting"
        self.start_since = None    # clock_s when the breakaway drive was first applied
        self.restarts = 0
        self.bias = 0.0
        self.last_speed = None     # (clock_s, measured speed) of the previous step
        self.feedback_started_s = None
        self.feedback_moving = False
        self.feedback_integral = 0.0
        self.feedback_previous_speed = None
        self.feedback_command = 0.0
        self.feedback_acceleration = 0.0
        self.feedback_coasting = False
        self.feedback_stopped_since = None
        self.step_times = []
        if self.solver is not None and hasattr(self.solver, "reset"):
            self.solver.reset()

    def speed_feedback_drive(self, speed, target, dt):
        """Sustaining feedforward plus PI, with bounded delay anticipation.

        Starting duration is a maximum, not a promise to keep accelerating once
        target speed is reached. Coast hysteresis avoids toggling on noisy gyro/
        wheel timing. Unknown braking behaviour is never treated as a brake.
        """
        c = self.cfg
        if target <= 0:
            self.feedback_integral = 0.0
            self.feedback_command = 0.0
            self.feedback_coasting = True
            return 0.0
        if self.feedback_started_s is None:
            self.feedback_started_s = self.clock_s
        elapsed = self.clock_s-self.feedback_started_s
        acceleration = 0.0
        if self.feedback_previous_speed is not None and dt > 0:
            acceleration = min(1.0, max(0.0, (speed-self.feedback_previous_speed)/dt))
        if dt > 0:
            alpha = 1.0-math.exp(-dt/c.speed_feedback_accel_tau_s)
            self.feedback_acceleration += alpha*(acceleration-self.feedback_acceleration)
        self.feedback_previous_speed = speed
        projected = speed+max(acceleration, self.feedback_acceleration)*(self.vehicle.drive_delay_s+0.1)
        if speed >= 0.02:
            self.feedback_moving = True
            self.feedback_stopped_since = None
        elif self.feedback_moving:
            if self.feedback_stopped_since is None:
                self.feedback_stopped_since = self.clock_s
            elif self.clock_s-self.feedback_stopped_since >= 2.0:
                raise MPCStop("speed_feedback_motion_lost")

        if not c.speed_feedback_continuous_enabled and speed >= target+c.speed_feedback_coast_margin_mps:
            self.feedback_coasting = True
        elif (not c.speed_feedback_continuous_enabled and self.feedback_coasting and speed <= max(0.0, target-0.02)
                and speed <= target):
            self.feedback_coasting = False
        if c.speed_feedback_continuous_enabled:
            self.feedback_coasting = False
        if self.feedback_coasting:
            # Learn excessive sustaining effort from actual overspeed only.
            # Projected acceleration alone must not wind the integral down.
            if speed > target:
                self.feedback_integral = max(-c.speed_feedback_sustain_drive,
                    self.feedback_integral+c.speed_feedback_ki*(target-speed)*max(0.0, dt))
            self.feedback_command = 0.0
            return 0.0
        # Prediction reduces effort smoothly; measured speed controls the
        # integral. A predicted overshoot alone cannot command a full coast.
        error = target-speed
        ff = c.speed_feedback_sustain_drive
        anticipated_error = target-(projected if c.speed_feedback_continuous_enabled
                                    else max(speed, min(projected, speed+0.5*target)))
        unsaturated = ff+c.speed_feedback_kp*anticipated_error+self.feedback_integral
        if (0 <= unsaturated <= c.speed_feedback_drive_max
                or unsaturated < 0 and error > 0
                or unsaturated > c.speed_feedback_drive_max and error < 0):
            self.feedback_integral += c.speed_feedback_ki*error*max(0.0, dt)
        self.feedback_integral = min(c.speed_feedback_drive_max-ff,
                                     max(-ff, self.feedback_integral))
        command = min(c.speed_feedback_drive_max,
                      max(0.0, ff+c.speed_feedback_kp*anticipated_error+self.feedback_integral))
        if (elapsed < c.speed_feedback_start_pulse_s-1e-9 and speed < target
                and (not c.speed_feedback_continuous_enabled or not self.feedback_moving)):
            command = c.speed_feedback_start_drive
            # Stage the open-loop part during the known actuation delay.
            # Escalate to the configured start effort only if still needed.
            if elapsed < self.vehicle.drive_delay_s:
                command = min(command, ff+0.04)
        # Continuous mode removes the binary coast latch; PI can still reach
        # zero for persistent excess speed. There is no positive throttle floor.
        # Keep the initial measured breakaway request; slew later adjustments.
        if c.speed_feedback_continuous_enabled and elapsed > 0 and dt > 0:
            change = c.speed_feedback_drive_slew_per_s*dt
            command = min(self.feedback_command+change, max(self.feedback_command-change, command))
        self.feedback_command = min(c.speed_feedback_drive_max, max(0.0, command))
        if c.speed_feedback_continuous_enabled:
            self.feedback_coasting = self.feedback_command <= 1e-9 and speed > target
        return self.feedback_command

    def predicted_speed(self, speed, start_s, end_s):
        """Longitudinal model from start_s to end_s with the drive commands actually in force."""
        p = self.vehicle
        d0 = min(p.drive_max, max(0.0, p.drive_deadband + self.bias))
        count = max(1, math.ceil((end_s-start_s)/0.01))
        h = (end_s-start_s)/count
        for i in range(count):
            effort = self.command_at(start_s + i*h - p.drive_delay_s)[0] - d0
            force = (p.motor_gain_n if effort >= 0 else p.brake_gain_n)*effort
            speed = max(0.0, speed + h*(force - p.drag_kg_per_m*speed*abs(speed))/p.mass_kg)
        return speed

    def update_bias(self, speed):
        """Integral action: shift the effective deadband by the one-step speed residual."""
        cfg, last = self.cfg, self.last_speed
        self.last_speed = (self.clock_s, speed)
        if (cfg.drive_bias_gain <= 0 or last is None or self.phase != "tracking"
                or speed < cfg.moving_speed_mps or last[1] < cfg.moving_speed_mps or self.clock_s <= last[0]):
            return
        residual = self.predicted_speed(last[1], last[0], self.clock_s) - speed
        self.bias += cfg.drive_bias_gain*residual*(self.clock_s-last[0])
        self.bias = min(cfg.drive_bias_max, max(-cfg.drive_bias_max, self.bias))

    def command_at(self, time_s):
        """Applied command in force at time_s; zero before the run, held after the last."""
        command = (0.0, 0.0)
        for issued, drive, steer in self.history:
            if issued > time_s + 1e-9:
                break
            command = (drive, steer)
        return command

    def initial_state(self, speed):
        """Model state when the command computed now takes effect.

        Steering is replayed from rest for long enough to settle, then the whole
        state is propagated over the larger actuator delay with the commands already
        sent; the channel with the shorter delay holds its last command meanwhile.
        """
        p, now = self.vehicle, self.clock_s
        steer_delay, drive_delay = p.steering_delay_s, p.drive_delay_s
        delay = max(steer_delay, drive_delay)
        # 0.02 s substeps on the same clock as the command times, so switches land on substeps.
        sub = 0.02
        count = math.ceil(((p.steering_max_rad-p.steering_min_rad)/p.steering_rate_limit_rad_s + 0.2)/sub)
        window = count*sub
        z = [0.0, 0.0, 0.0, 0.0, p.steering_offset_rad]
        for i in range(count):
            t = now - window + i*sub
            z = course_model_step(z, (0.0, self.command_at(t-steer_delay)[1]), sub, p)
        delta_now = z[4]
        z = [0.0, 0.0, 0.0, speed, delta_now]
        if delay > 0:
            count = math.ceil(delay/0.02)
            sub = delay/count
            for i in range(count):
                t = now + i*sub
                drive = self.command_at(t-drive_delay)[0]
                if self.cfg.speed_feedback_measured_motion:
                    # Hold the fresh measured speed through the delay; the
                    # uncalibrated Gym throttle force must not inflate it.
                    drive = p.drive_deadband+p.drag_kg_per_m*z[3]*abs(z[3])/p.motor_gain_n
                z = course_model_step(z, (drive, self.command_at(t-steer_delay)[1]), sub, p)
        return z, delta_now

    def gate_problem(self):
        cfg = self.cfg
        if cfg.shadow:
            return None
        if cfg.trial_fixed_drive and not cfg.gym_trial_acknowledged:
            return "fixed_drive_requires_explicit_trial"
        if cfg.gym_trial_acknowledged:
            if cfg.reference_dropout_tolerance_s != 0:
                return "trial_requires_no_actuator_plan_hold"
            if cfg.vehicle_params_source not in ("course_simulation", "partial_vehicle") or cfg.reference_source != "planning":
                return "gym_trial_requires_course_model_and_planning"
            # User-selected speed-feedback trial supports up to 0.25 m/s.
            # Other unmeasured/fixed-drive trial modes retain the 0.1 m/s gate.
            feedback_cap = .50 if cfg.learned_planner_speed_match_enabled else (.30 if cfg.reference_join_repair_enabled else .25)
            if not 0 < cfg.v_exec_max_mps <= (feedback_cap if cfg.speed_feedback_enabled else 0.1):
                return (f"feedback_trial_speed_cap_must_be_at_most_{feedback_cap}" if cfg.speed_feedback_enabled
                        else "gym_trial_speed_cap_must_be_at_most_0_1")
            return None
        if cfg.vehicle_params_source != "vehicle":
            return "course_simulation_is_shadow_only"
        if not self.vehicle.valid:
            return "vehicle_params_not_measured"
        if cfg.v_exec_max_mps <= 0:
            return "v_exec_max_not_set"
        if cfg.reference_source == "estimation_centerline" and not cfg.bypass_acknowledged:
            return "planning_bypass_not_acknowledged"
        return None

    @staticmethod
    def _start_bridge_geometry(points):
        """Fit a local straight trend; ignore derivative spikes, not bent geometry."""
        if (not isinstance(points, list) or len(points) < 2
                or any(not isinstance(p, dict) or not all(finite_number(p.get(k))
                    for k in ("x_m", "y_m")) for p in points)):
            return None, {}, "start_bridge_geometry"
        xy = [(points[0]["x_m"], points[0]["y_m"])];span = 0.0
        for p in points[1:]:
            x, y = p["x_m"], p["y_m"]
            length = math.hypot(x-xy[-1][0], y-xy[-1][1])
            if length < 1e-8:
                continue
            fraction = min(1.0, (0.4-span)/length)
            xy.append((xy[-1][0]+fraction*(x-xy[-1][0]), xy[-1][1]+fraction*(y-xy[-1][1])))
            span += fraction*length
            if span >= 0.4-1e-8:
                break
        if span < 0.15:
            return None, {"bridge_fit_window_m": span}, "start_bridge_geometry"
        mx, my = sum(x for x,y in xy)/len(xy), sum(y for x,y in xy)/len(xy)
        xx = sum((x-mx)**2 for x,y in xy); yy = sum((y-my)**2 for x,y in xy)
        cross = sum((x-mx)*(y-my) for x,y in xy)
        heading = 0.5*math.atan2(2*cross, xx-yy)
        tx, ty = math.cos(heading), math.sin(heading)
        if (xy[-1][0]-xy[0][0])*tx+(xy[-1][1]-xy[0][1])*ty < 0:
            heading = wrap_angle(heading+math.pi);tx,ty = -tx,-ty
        deviation = max(abs(-(x-mx)*ty+(y-my)*tx) for x,y in xy)
        diag = {"bridge_fit_window_m": span, "bridge_fit_max_deviation_m": deviation,
                "bridge_fit_heading_rad": heading}
        return heading, diag, "start_bridge_geometry" if deviation > 0.025 else None

    def step(self, traj, speed_mps, dt, now_s, is_first):
        cfg, t0 = self.cfg, time.perf_counter()
        d = {"branch": None, "reject_reason": None, "shadow": cfg.shadow, "dt": dt,
             "speed_mps": speed_mps, "v_ref_mps": None, "v_ref_end_mps": None,
             "reference_speed_mode": "scalar", "planned_stop": False,
             "e_y_m": None, "e_psi_rad": None,
             "candidate_delta_rad": None, "candidate_steer_action": None, "candidate_drive": None,
             "applied_steer_action": 0.0, "applied_drive": 0.0, "path_id": None, "ref_age_s": None,
             "simulation_only": None, "status": None, "iterations": None, "solve_s": None,
             "pred_xy": None, "step_s": None, "delta_est_rad": None, "pred_e_y_end_m": None, "pred_v_end_mps": None,
             "vehicle_source": self.vehicle.source, "vehicle_valid": self.vehicle.valid, "dropout_s": None,
               "gym_trial_acknowledged": cfg.gym_trial_acknowledged,
             "reference_source": None, "planning_bypassed": None, "phase": self.phase,
             "drive_bias": self.bias, "plan_index": None, "step_p95_s": None}
        self.last_debug = d
        if isinstance(traj, dict):
            d["path_id"], d["simulation_only"] = traj.get("path_id"), traj.get("simulation_only")
            d["reference_source"] = traj.get("reference_source")
            d["planning_bypassed"] = traj.get("planning_bypassed")
            if finite_number(traj.get("timestamp_s")) and finite_number(now_s):
                d["ref_age_s"] = now_s - traj["timestamp_s"]

        def record(drive, steer):
            d["phase"], d["drive_bias"] = self.phase, self.bias
            self.applied_drive, self.applied_steer = drive, steer
            self.history.append((self.clock_s, drive, steer))
            keep = self.clock_s - 3.0
            while len(self.history) > 1 and self.history[1][0] <= keep:
                self.history.pop(0)
            d.update(applied_steer_action=steer, applied_drive=drive)

        def reject(branch, reason, tolerable=False):
            d["branch"], d["reject_reason"] = branch, reason
            d["step_s"] = time.perf_counter()-t0
            reference_meta = traj if isinstance(traj, dict) else {}
            reference_points = reference_meta.get("points")
            d.update(reference_path_id=reference_meta.get("path_id"),
                     reference_point_count=len(reference_points) if isinstance(reference_points, (list, tuple)) else 0)
            if reason == "reference_too_short":
                # Preserve the exact rejected frame, not the preceding source
                # selection event (which may describe a different trajectory).
                d.update(rejected_reference_points=traj.get("points"),
                         rejected_reference_path_id=traj.get("path_id"),
                         rejected_reference_timestamp_s=traj.get("timestamp_s"))
            self.nominal = None
            if tolerable and not cfg.shadow and cfg.reference_dropout_tolerance_s > 0:
                if self.invalid_since is None:
                    self.invalid_since = self.clock_s
                d["dropout_s"] = self.clock_s - self.invalid_since
                if d["dropout_s"] < cfg.reference_dropout_tolerance_s:
                    if self.plan is None:
                        # No solved plan yet: coast with the steering held.
                        d["branch"] = "ref_hold"
                        held = self.applied_steer
                        record(0.0, held)
                        return {"drive": 0.0, "steer": held, "debug": d}
                    solved_at, inputs = self.plan
                    index = int((self.clock_s-solved_at)/cfg.dt_pred_s + 1e-9)
                    if index < len(inputs):
                        # Short dropout: keep executing the last solved sequence, never
                        # increasing the drive. Its age is bounded by the tolerance.
                        d["branch"], d["plan_index"] = "plan_hold", index
                        drive, steer = min(inputs[index][0], self.applied_drive), inputs[index][1]
                        record(drive, steer)
                        return {"drive": drive, "steer": steer, "debug": d}
            self.plan = None
            record(0.0, 0.0)
            if not cfg.shadow:
                raise MPCStop(f"{branch}:{reason}")
            return {"drive": 0.0, "steer": 0.0, "debug": d}

        def accept(branch, steer_action, drive):
            d["branch"] = branch
            d["step_s"] = time.perf_counter()-t0
            self.step_times = (self.step_times + [d["step_s"]])[-200:]
            d["step_p95_s"] = sorted(self.step_times)[int(0.95*(len(self.step_times)-1))]
            if d["step_s"] > cfg.max_step_time_s:
                return reject("over_budget", f"step {d['step_s']:.4f}s", tolerable=True)
            steer, out_drive = (0.0, 0.0) if cfg.shadow else (steer_action, drive)
            self.invalid_since = None
            record(out_drive, steer)
            return {"drive": out_drive, "steer": steer, "debug": d}

        problem = self.gate_problem()
        if problem:
            return reject("gate_refused", problem)
        if self.vehicle_problem is not None:
            return reject("vehicle_invalid", self.vehicle_problem)
        if not is_first and not (finite_number(dt) and 0.0 < dt <= cfg.dt_hard_max_s):
            return reject("ref_invalid", "dt_out_of_range")
        self.clock_s += 0.0 if is_first else dt
        if not finite_number(speed_mps) or speed_mps < 0:
            return reject("ref_invalid", "speed_invalid")
        d["dt"] = dt if not is_first else 0.0
        self.update_bias(speed_mps)
        breakaway = self.vehicle.drive_breakaway > 0
        if speed_mps >= cfg.moving_speed_mps:
            self.phase, self.start_since = "tracking", None
        elif self.phase == "tracking" and breakaway:
            self.phase, self.start_since = "starting", None
            self.restarts += 1
            if not cfg.shadow and self.restarts > cfg.max_restarts:
                return reject("stop", "too_many_restarts")
        d["phase"] = self.phase
        if not is_first and dt > cfg.dt_max_s:
            return reject("ref_invalid", "dt_late", tolerable=True)
        if not isinstance(traj, dict) or traj.get("schema") != "reference_trajectory_v0.1":
            return reject("ref_invalid", "ref_missing", tolerable=True)
        stamp, lifetime = traj.get("timestamp_s"), traj.get("valid_for_s")
        if not traj.get("valid"):
            return reject("ref_invalid", traj.get("reason") or "ref_invalid", tolerable=True)
        if traj.get("stop_required"):
            return reject("stop", traj.get("reason") or "stop_required")
        if (not finite_number(stamp) or not finite_number(lifetime) or not finite_number(now_s)
                or not stamp-1e-6 <= now_s < stamp+lifetime):
            return reject("ref_invalid", "ref_expired_or_future", tolerable=True)
        planned_stop = traj.get("planned_stop") is True
        d["planned_stop"] = planned_stop
        if planned_stop and not cfg.shadow:
            # The execution QP still excludes calibrated braking and a hard
            # stop-position constraint. Keep the existing locked-stop behavior.
            return reject("stop", "planned_stop_execution_not_qualified")
        speed_targets = None
        profile = traj.get("speed_profile") if cfg.shadow else None
        if planned_stop and profile is None:
            return reject("stop", "bad_speed_profile")
        if profile is not None:
            if (not isinstance(profile, (list, tuple)) or len(profile) < 2
                    or any(not isinstance(row, (list, tuple)) or len(row) != 2
                           or not all(finite_number(v) for v in row)
                           or row[0] < 0 or row[1] < 0 for row in profile)
                    or abs(profile[0][0]) > 1e-6
                    or any(b[0] <= a[0] for a, b in zip(profile, profile[1:]))):
                return reject("stop" if planned_stop else "ref_invalid", "bad_speed_profile")
            if planned_stop and (profile[-1][1] > 1e-6 or self.vehicle.brake_gain_n <= 0):
                return reject("stop", "braking_model_unavailable")
            profile_stamp = traj.get("speed_profile_timestamp_s", stamp)
            if not finite_number(profile_stamp) or profile_stamp > now_s:
                return reject("stop" if planned_stop else "ref_invalid", "bad_speed_profile_timestamp")
            first_time = max(0.0, now_s-profile_stamp) + max(self.vehicle.drive_delay_s, self.vehicle.steering_delay_s)
            last_time = first_time + cfg.horizon_n*cfg.dt_pred_s
            if last_time > profile[-1][0] + 1e-9:
                return reject("stop" if planned_stop else "ref_invalid", "speed_profile_horizon_short")

            def speed_at(t):
                for a, b in zip(profile, profile[1:]):
                    if t <= b[0]:
                        weight = max(0.0, (t-a[0])/(b[0]-a[0]))
                        return a[1] + weight*(b[1]-a[1])
                return profile[-1][1]

            speed_targets = [speed_at(first_time+(k+1)*cfg.dt_pred_s)
                             for k in range(cfg.horizon_n)]
            d.update(reference_speed_mode="time_profile", v_ref_end_mps=speed_targets[-1])
        if cfg.start_bridge_enabled and traj.get("reference_path_type") != "ORDERED_CENTERLINE":
            return reject("ref_invalid", "start_bridge_requires_ordered_centerline")
        if cfg.start_bridge_enabled:
            points = traj.get("points")
            first = points[0] if isinstance(points, list) and points and isinstance(points[0], dict) else {}
            d.update(reference_first_xy_m=[first.get("x_m"), first.get("y_m")],
                     start_extension_limit_m=cfg.max_backward_extension_m)
        start_heading, bridge_geometry_reason = None, None
        if cfg.start_bridge_enabled:
            start_heading, bridge_fit, bridge_geometry_reason = self._start_bridge_geometry(traj.get("points"))
            d.update(bridge_fit)
        info, reason = mpc_reference_errors(traj, cfg.horizon_n, cfg.dt_pred_s, 0.0,
                                            cfg.max_backward_extension_m, start_heading)
        if info is None:
            return reject("ref_invalid", reason, tolerable=True)
        d.update(e_y_m=info["e_y_m"], e_psi_rad=info["e_psi_rad"],
                 v_ref_mps=speed_targets[0] if speed_targets is not None else info["v_ref_mps"])
        d.update(start_extension_m=max(0.0, -info["s_proj_m"]),
                 start_bridge_active=cfg.start_bridge_enabled and info["extended_start"])
        if cfg.start_bridge_enabled and info["extended_start"]:
            # This is only tangent-based projection through a short near-car
            # blind zone. Never invent a bend or a lateral lane transition.
            if bridge_geometry_reason:
                return reject("ref_invalid", bridge_geometry_reason)
            if abs(info["e_psi_rad"]) > 0.35 or abs(info["e_y_m"]) > 0.20:
                return reject("ref_invalid", "start_bridge_alignment")
        if cfg.overspeed_stop_enabled and not cfg.shadow and speed_mps > cfg.v_exec_max_mps + cfg.overspeed_margin_mps:
            return reject("stop", "overspeed")
        cap = cfg.v_exec_max_mps if cfg.v_exec_max_mps > 0 else math.inf
        if min(d["v_ref_mps"], cap) <= 0.0:
            self.nominal, self.plan = None, None
            return accept("zero_target", self.applied_steer, 0.0)
        z0, delta_now = self.initial_state(speed_mps)
        d.update(delta_est_rad=delta_now, longitudinal_prediction_mode=
                 "measured_speed_hold" if cfg.speed_feedback_measured_motion else "force_model")
        extra = {} if start_heading is None else {"start_heading_rad": start_heading}
        if self.bias or self.vehicle.drive_deadband > 0:
            extra.update(drive_bias=self.bias, moving=self.phase == "tracking")
        if speed_targets is not None:
            extra.update(speed_targets=speed_targets, allow_braking=planned_stop)
        if cfg.speed_feedback_enabled:
            try:
                feedback_drive = self.speed_feedback_drive(speed_mps, min(d["v_ref_mps"], cap),
                                                           0.0 if is_first else dt)
            except MPCStop as error:
                return reject("stop", str(error))
            extra["fixed_drive_request"] = feedback_drive
            d.update(longitudinal_mode="speed_feedback", speed_feedback_drive=feedback_drive,
                     speed_feedback_target=min(d["v_ref_mps"], cap))
        horizon_steps = None
        if cfg.adaptive_horizon_enabled and speed_targets is None:
            support = mpc_path_projector(traj["points"], cfg.max_backward_extension_m, start_heading).details(z0[0], z0[1])
            remaining = support["path_remaining_m"]
            # z0 already includes the known actuator-delay travel. Do not
            # subtract delay twice or fill unknown geometry with repeated points.
            usable = max(0.0, remaining-cfg.reference_end_margin_m)
            travel_speed = max(speed_mps, z0[3], min(d["v_ref_mps"], cap), 0.05)
            horizon_steps = min(cfg.horizon_n, int(math.floor(0.8*usable/(travel_speed*cfg.dt_pred_s)+1e-9)))
            d.update(path_remaining_m=remaining, nominal_horizon_n=cfg.horizon_n,
                     effective_horizon_n=horizon_steps, prediction_dt_s=cfg.dt_pred_s)
            d.update(delay_state_xy_m=[z0[0], z0[1]], delay_state_speed_mps=z0[3],
                     usable_path_m=usable, estimated_travel_speed_mps=travel_speed)
            if support["past_end"] or usable <= 0.0:
                return reject("ref_invalid", "reference_too_short", tolerable=True)
            if horizon_steps < cfg.adaptive_horizon_min_n:
                # Constant-speed + 20% margin is a scheduling estimate, not a
                # proof that the model's minimum horizon cannot fit. Let the
                # SAME nonlinear solver check two steps and the end margin.
                horizon_steps = cfg.adaptive_horizon_min_n
                d.update(effective_horizon_n=horizon_steps, minimum_horizon_support_check=True)
            extra["horizon_steps"] = horizon_steps
        if cfg.adaptive_horizon_enabled or cfg.speed_feedback_enabled:
            # Reserve publication/acceptance overhead inside the same 50 ms cap.
            extra["deadline_s"] = t0+cfg.max_step_time_s-min(0.005, cfg.max_step_time_s*0.1)
        solve_started = time.perf_counter()
        attempt_timings = []
        try:
            attempt_start = time.perf_counter()
            solution, reason = self.solver.solve(traj["points"], z0, cap, self.applied_drive, self.nominal, **extra)
            attempt_timings.append(time.perf_counter()-attempt_start)
            # Check progressively shorter horizons over the same observed path.
            # Bound retries by the existing total step budget; never advance PI,
            # controller time, or history twice and never extend the path end.
            d["horizon_attempts"] = [horizon_steps] if horizon_steps is not None else []
            while (solution is None and reason == "reference_too_short"
                   and horizon_steps is not None and horizon_steps > cfg.adaptive_horizon_min_n):
                spent = time.perf_counter()-t0
                remaining_budget = extra["deadline_s"]-time.perf_counter()
                predicted_retry_s = attempt_timings[-1]*0.6
                if remaining_budget <= max(0.002, predicted_retry_s):
                    d["horizon_retry_budget_exhausted"] = True
                    break
                horizon_steps = max(cfg.adaptive_horizon_min_n, horizon_steps//2)
                extra["horizon_steps"] = horizon_steps
                d["effective_horizon_n"] = horizon_steps
                d["horizon_retry"] = True
                d["horizon_attempts"].append(horizon_steps)
                attempt_start = time.perf_counter()
                solution, reason = self.solver.solve(traj["points"], z0, cap, self.applied_drive, self.nominal, **extra)
                attempt_timings.append(time.perf_counter()-attempt_start)
        except MPCStop as exc:
            return reject("solver_fail", str(exc))
        except Exception as exc:
            return reject("solver_fail", f"solver_exception:{type(exc).__name__}")
        finally:
            d.update(solver_total_s=time.perf_counter()-solve_started,
                     solver_attempt_s=attempt_timings,
                     solver_phases_s=dict(getattr(self.solver, "last_timing", {})),
                     qp_variables=getattr(self.solver, "last_qp_variables", None),
                     model_supported_horizon_n=getattr(self.solver, "last_support_steps", None))
        if solution is None:
            return reject("over_budget" if reason == "deadline_exceeded" else "ref_invalid", reason, tolerable=True)
        if solution.get("horizon_steps") is not None:
            d["effective_horizon_n"] = solution["horizon_steps"]
        drive, steer = solution["drive0"], solution["steer0"]
        if not (finite_number(drive) and finite_number(steer) and abs(steer) <= 1.0 + 1e-6
                and -1e-6 <= drive <= 1.0 + 1e-6):
            return reject("solver_fail", "action_out_of_bounds")
        drive, steer = min(1.0, max(0.0, drive)), min(1.0, max(-1.0, steer))
        inputs = solution["inputs"]
        self.nominal = inputs[1:] + inputs[-1:]
        if breakaway and self.phase == "starting" and not cfg.shadow:
            # Static friction: the MPC's small model-based drive would not start the car.
            if self.start_since is None:
                self.start_since = self.clock_s
            if self.clock_s - self.start_since > cfg.breakaway_timeout_s:
                return reject("stop", "no_motion_after_breakaway")
            drive = min(self.vehicle.drive_max, self.vehicle.drive_breakaway
                        + cfg.breakaway_ramp_per_s*(self.clock_s - self.start_since))
        self.plan = (self.clock_s, inputs)
        d.update(candidate_delta_rad=solution["delta0"], candidate_steer_action=steer, candidate_drive=drive,
                 status="solved", iterations=solution["iterations"], solve_s=solution["solve_s"],
                 pred_e_y_end_m=solution["pred_e_y"][-1], pred_v_end_mps=solution["pred_v"][-1],
                 pred_xy=solution.get("pred_xy"))
        return accept("shadow" if cfg.shadow else "solved", steer, drive)

def mpc_debug_json(debug):
    """Compact JSON for the mpc_debug topic; non-finite numbers become null."""
    clean = {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in debug.items()}
    return json.dumps(clean, separators=(",", ":"))

def _control_poly_eval(coeffs, x):
    result = 0.0
    for coefficient in reversed(coeffs):
        result = result*x+coefficient
    return result

def _control_poly_roots(coeffs):
    """All real roots on [-1,1], using derivative isolation + bounded bisection.

    The caller supplies degree <= 9. Each derivative has smaller degree; there
    are at most degree monotone intervals and 60 bisections per interval. Handles
    repeated roots at derivative roots. No NumPy or external offline imports.
    """
    coeffs = list(coeffs)
    while len(coeffs) > 1 and coeffs[-1] == 0:
        coeffs.pop()
    size = max(abs(v) for v in coeffs)
    if not finite_number(size):
        raise ValueError("nonfinite_path_geometry")
    if size == 0 or len(coeffs) == 1:
        return []
    coeffs = [v/size for v in coeffs]
    if len(coeffs) == 2:
        root = -coeffs[0]/coeffs[1]
        return [root] if -1 <= root <= 1 else []
    critical = _control_poly_roots([i*v for i, v in enumerate(coeffs) if i])
    knots = sorted(set([-1.0, *critical, 1.0]))
    roots = [x for x in knots if abs(_control_poly_eval(coeffs, x)) <= 1e-12]
    for low, high in zip(knots, knots[1:]):
        f_low, f_high = _control_poly_eval(coeffs, low), _control_poly_eval(coeffs, high)
        if (f_low > 0) == (f_high > 0) or f_low == 0 or f_high == 0:
            continue
        for _ in range(60):
            mid = low/2+high/2
            f_mid = _control_poly_eval(coeffs, mid)
            if f_mid == 0:
                low = high = mid
                break
            if (f_low > 0) == (f_mid > 0):
                low, f_low = mid, f_mid
            else:
                high = mid
        roots.append(low/2+high/2)
    return sorted(set(roots))

def control_path_geometry(path):
    """Bounded closest observed point for y(x), up to fifth degree.

    u=(x-origin)/scale, low-to-high coefficients, physical range in metres.
    Front-only support is retained; endpoint tangent error is an approximation,
    not evidence of observed road at the car origin. No path extrapolation.
    """
    if (not isinstance(path, dict) or path.get("type") != "CARTESIAN_Y_OF_X"
            or path.get("independent_variable") != "x_m"):
        raise ValueError("unsupported_path_encoding")
    coeffs, bounds = path.get("coeffs_low_to_high"), path.get("range")
    origin, scale = path.get("origin"), path.get("scale")
    if (not isinstance(coeffs, (list, tuple)) or not 1 <= len(coeffs) <= 6
            or not all(finite_number(v) for v in coeffs)):
        raise ValueError("invalid_polynomial")
    if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(finite_number(v) for v in bounds)
            or not bounds[0] < bounds[1] or bounds[1] <= 0
            or not finite_number(origin) or not finite_number(scale) or scale <= 0):
        raise ValueError("invalid_path_range_or_normalization")
    mid, half = bounds[0]/2+bounds[1]/2, bounds[1]/2-bounds[0]/2
    offset, factor = (mid-origin)/scale, half/scale
    # Compose y(mid+half*t), t in [-1,1], with <= 6 coefficients.
    y = [0.0]*len(coeffs)
    for i, coefficient in enumerate(coeffs):
        for j in range(i+1):
            y[j] += coefficient*math.comb(i, j)*offset**(i-j)*factor**j
    dy = [i*v for i, v in enumerate(y) if i] or [0.0]
    stationary = [mid*half, half*half]+[0.0]*max(0, 2*len(y)-4)
    for i, a in enumerate(y):
        for j, b in enumerate(dy):
            stationary[i+j] += a*b
    candidates = [-1.0, 1.0, *_control_poly_roots(stationary)]
    costs = [(mid+half*t)**2+_control_poly_eval(y, t)**2 for t in candidates]
    if not all(finite_number(v) for v in costs):
        raise ValueError("nonfinite_path_geometry")
    t = candidates[min(range(len(costs)), key=costs.__getitem__)]
    x, value = mid+half*t, _control_poly_eval(y, t)
    slope = _control_poly_eval(dy, t)/half
    ddy = [i*v for i, v in enumerate(dy) if i] or [0.0]
    second = _control_poly_eval(ddy, t)/half**2
    heading = math.atan(slope)
    geometry = {"path_error_m": -x*math.sin(heading)+value*math.cos(heading),
                "path_heading_rad": heading, "curvature_1pm": second/math.hypot(1, slope)**3,
                "closest_x_m": x, "closest_y_m": value, "path_degree": len(coeffs)-1,
                "closest_at_range_end": t in (-1.0, 1.0)}
    if not all(finite_number(v) for k, v in geometry.items() if k != "closest_at_range_end"):
        raise ValueError("nonfinite_path_geometry")
    return geometry

def _control_sample_points(reference):
    """Validate bounded body-x samples: x[m], y[m], heading[rad], curvature[1/m].

    Private transported samples take precedence over the original-frame path.
    Strictly increasing x retains the current forward-only preview contract.
    """
    path = reference.get("path") or {}
    transported = "control_cached_path_samples" in reference
    if not transported and (path.get("type") != "CARTESIAN_SAMPLES"
                            or path.get("independent_variable") != "x_m"):
        raise ValueError("unsupported_path_encoding")
    points = (reference["control_cached_path_samples"] if transported else
              path.get("samples_xy_heading_curvature"))
    if (not isinstance(points, (list, tuple)) or not 2 <= len(points) <= 201
            or any(not isinstance(p, (list, tuple)) or len(p) != 4
                   or not all(finite_number(v) for v in p) for p in points)
            or any(b[0] <= a[0] for a, b in zip(points, points[1:]))
            or points[-1][0] <= max(0.0, points[0][0])
            or any(abs(p[2]) >= math.pi/2 for p in points)):
        raise ValueError("invalid_control_cached_curve")
    if not transported:
        bounds = path.get("range")
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                or not all(finite_number(v) for v in bounds)
                or bounds[0] != points[0][0] or bounds[1] != points[-1][0]):
            raise ValueError("invalid_sample_path_range")
    return points

def _control_sample_at(points, x):
    """Binary lookup with position/tangent/curvature interpolation; no extrapolation."""
    if not points[0][0] <= x <= points[-1][0]:
        raise ValueError("outside_supported_range")
    lo, hi = 0, len(points)-1
    while hi-lo > 1:
        mid = (lo+hi)//2
        if points[mid][0] < x:
            lo = mid
        else:
            hi = mid
    a, b = points[lo], points[hi]
    t = (x-a[0])/(b[0]-a[0])
    return (x, a[1]+t*(b[1]-a[1]), wrap_angle(a[2]+t*wrap_angle(b[2]-a[2])),
            a[3]+t*(b[3]-a[3]))

def control_reference_snapshot(reference):
    """Small independent control envelope; upstream rich records stay intact."""
    selected = {k: v for k, v in reference.items() if k not in
                ("frenet_path", "speed_profile", "longitudinal_profile", "support", "control_cached_path_samples")}
    if "path" in selected:
        path = selected["path"]
        if path.get("type") == "CARTESIAN_SAMPLES":
            selected["path"] = {k: deepcopy(v) for k, v in path.items() if k != "samples_xy_heading_curvature"}
            # Immutable geometry is safe to share between control envelopes.
            selected["path"]["samples_xy_heading_curvature"] = tuple(
                tuple(p) for p in path["samples_xy_heading_curvature"])
        else:
            selected["path"] = deepcopy(path)
    if "control_cached_path_samples" in reference:
        selected["control_cached_path_samples"] = [tuple(p) for p in reference["control_cached_path_samples"]]
    return selected

class ControlReferenceManager:
    """MVP Cartesian-reference cache; never edits the upstream planning output.

    Cache budgets start at acceptance of a DISTINCT observed cone frame. Motion
    is integrated incrementally, preserving the original source timestamp. The
    manager generates geometry, never cached actuator commands. Prediction may
    end earlier than its budgets when observed support or motion history ends.
    """
    SOFT_REASONS = frozenset({
        "invalid_estimates", "stale_road", "v1_requires_both_boundaries",
        "unsupported_road_curvature", "insufficient_near_or_far_coverage",
        "not_straight_enough", "outside_v1_offset_or_heading_domain",
        "invalid_estimates_or_boundaries", "insufficient_lattice_coverage",
        "expired_road_prediction", "lattice_time_budget_exceeded",
        "source_deadline_during_planning", "planning_pending", "planning_worker_timeout",
        "planning_worker_unavailable", "insufficient_body_supported_horizon",
        "lattice_candidate_cap", "cartesian_conversion_failed"})

    def __init__(self, settings):
        self.settings = settings
        self.clear()

    def clear(self):
        self.reference = None
        self.source_stamp_s = self.accepted_at_s = self.accepted_distance_m = None
        self.frame_time_s = None
        self.endpoints = None
        self.curve_samples = None
        self.last_target_speed_mps = None
        self.last_selection_s = None
        self.recovery_frames = 0
        self.recovering = False
        self.mode = "TRACKING"
        self.diagnostics = {}

    def problem(self, monotonic_s, distance_m):
        if self.reference is None:
            return "No trusted control reference"
        elapsed, distance = monotonic_s-self.accepted_at_s, distance_m-self.accepted_distance_m
        if not all(finite_number(v) and v >= 0 for v in (elapsed, distance)):
            return "Control cache clock/distance invalid"
        if elapsed+1e-9 >= self.settings.reference_hold_max_s:
            return "Control reference hold time exhausted"
        if distance+1e-9 >= self.settings.reference_hold_max_distance_m:
            return "Control reference hold distance exhausted"
        return None

    def select(self, reference, road, state, motion, now_s, monotonic_s, distance_m, execution_lifetime_s):
        if (state.get("valid") is not True or not finite_number(execution_lifetime_s)
                or execution_lifetime_s <= 0):
            raise ValueError("Control recovery motion feedback unavailable")
        source_stamp = road.get("measurement_timestamp_s")
        new_frame = finite_number(source_stamp) and source_stamp != self.source_stamp_s
        usable = reference.get("valid") is True
        if usable:
            stamp, lifetime = reference.get("timestamp_s"), reference.get("valid_for_s")
            if not all(finite_number(v) for v in (stamp, lifetime)) or lifetime <= 0 or stamp > now_s:
                raise ValueError("Invalid upstream control reference timestamp")
            if (reference.get("frame_id") != state.get("frame_id")
                    or reference.get("vehicle_reference_point") != "cg_ground_projection"
                    or abs(stamp-state.get("timestamp_s", math.inf)) > 1e-6):
                raise ValueError("control_frame_or_time_mismatch")
            if (reference.get("stop_requested") or reference.get("target_speed_mps") == 0
                    or reference.get("planned_stop")):
                # A V2 deceleration trajectory must not fall back to older cruise.
                self.clear()
                return reference
            usable = now_s < stamp+lifetime
        reason = reference.get("reason") if not reference.get("valid") else "reference_expired"
        if not usable and reason not in self.SOFT_REASONS | {"reference_expired"}:
            raise ValueError("Planning: " + str(reason))

        path = reference.get("path") if usable else None
        sampled = isinstance(path, dict) and path.get("type") == "CARTESIAN_SAMPLES"
        supported = sampled or (isinstance(path, dict) and path.get("type") == "CARTESIAN_Y_OF_X"
                    and path.get("independent_variable") == "x_m"
                    and isinstance(path.get("coeffs_low_to_high"), (list, tuple))
                    and 1 <= len(path["coeffs_low_to_high"]) <= 6)
        if usable and not supported:
            # Other path encodings keep their original control/deadline behavior.
            self.clear()
            return reference
        if usable and road.get("degraded") and self.reference is None:
            # Upstream predictions may be consumed, but cannot seed trust or
            # extend their original deadline without an observed control cache.
            return reference
        if usable:
            if sampled:
                _control_sample_points(reference)
            else:
                control_path_geometry(path)  # Validate the polynomial contract.
            if not finite_number(source_stamp) or source_stamp > now_s:
                raise ValueError("Invalid trusted road timestamp")
            if self.source_stamp_s is not None and source_stamp < self.source_stamp_s:
                raise ValueError("Control road sample moved backwards")
            if (not road.get("degraded")
                    and (self.reference is None or source_stamp != self.source_stamp_s)):
                self.reference = control_reference_snapshot(reference)
                self.source_stamp_s = source_stamp
                self.accepted_at_s, self.accepted_distance_m = monotonic_s, distance_m
                self.frame_time_s = stamp
                self.curve_samples = None
                if sampled or "control_cached_path_samples" in reference:
                    self.curve_samples = [tuple(p) for p in _control_sample_points(reference)]
                    self.endpoints = [self.curve_samples[0][:2], self.curve_samples[-1][:2]]
                else:
                    coeffs = path["coeffs_low_to_high"]
                    self.endpoints = [(x, _control_poly_eval(coeffs, (x-path["origin"])/path["scale"]))
                                      for x in path["range"]]
                if self.curve_samples is None and len(coeffs) > 2:
                    lo, hi = path["range"]
                    count = max(1, math.ceil((hi-lo)/0.025))
                    if count > 200:
                        raise ValueError("Control cache curve sampling limit")
                    samples = []
                    for i in range(count+1):
                        x = min(hi, lo+(hi-lo)*i/count)
                        point = evaluate_planning_path(reference, x, now_s)
                        if point is None:
                            raise ValueError("Control cache curve sample unavailable")
                        samples.append((*point["position_xy_m"], point["heading_rad"], point["curvature_1pm"]))
                    self.curve_samples = samples

        if not usable or road.get("degraded"):
            problem = self.problem(monotonic_s, distance_m)
            if problem:
                raise ValueError(problem)
        if self.reference is not None:
            pose, problem = motion.pose_between(self.frame_time_s, now_s)
            if problem:
                raise ValueError("Control cache motion: " + problem)
            endpoints = [motion.transform_xy(p, pose) for p in self.endpoints]
            (x0, y0), (x1, y1) = endpoints
            if (not all(finite_number(v) for point in endpoints for v in point)
                    or x1 <= max(0.0, x0)):
                raise ValueError("Control cached path support exhausted")
            slope = (y1-y0)/(x1-x0)
            intercept = y0-slope*x0
            self.endpoints, self.frame_time_s = endpoints, now_s
            if self.curve_samples is not None:
                self.curve_samples = [(*motion.transform_xy(p[:2], pose),
                    wrap_angle(p[2]-pose["yaw_rad"]), p[3]) for p in self.curve_samples]
                if any(b[0] <= a[0] for a, b in zip(self.curve_samples, self.curve_samples[1:])):
                    raise ValueError("Control cached curve no longer y(x)")
            predicted_path = {"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m",
                              "origin": 0.0, "scale": 1.0,
                              "coeffs_low_to_high": [intercept, slope],
                              "range": [max(0.0, x0), x1]}

        selected = control_reference_snapshot(reference if usable else self.reference)
        if not usable:
            selected["path"] = predicted_path
            if self.curve_samples is not None:
                selected["path"] = deepcopy(self.reference["path"])
                selected["control_cached_path_samples"] = list(self.curve_samples)
            selected["target_speed_mps"] = min(selected["target_speed_mps"], self.last_target_speed_mps)
        selected.update(timestamp_s=now_s, valid_for_s=execution_lifetime_s)
        healthy = usable and not road.get("degraded")
        waiting = not usable and reason == "planning_pending"
        if healthy and (self.recovering or self.mode == "DEGRADED"):
            self.recovering = True
            if new_frame:
                self.recovery_frames += 1
            if self.recovery_frames >= self.settings.recovery_confirm_frames:
                self.recovering = False
        elif not healthy and not waiting:
            self.recovering, self.recovery_frames = True, 0
        self.mode = ("TRACKING" if (healthy or waiting and self.mode == "TRACKING")
                     and not self.recovering else "DEGRADED")
        target = selected["target_speed_mps"]
        if self.mode == "DEGRADED":
            target = min(target, self.settings.degraded_speed_mps)
        if self.last_target_speed_mps is not None and self.last_selection_s is not None:
            target = min(target, self.last_target_speed_mps+self.settings.speed_recovery_accel_mps2*
                         max(0.0, monotonic_s-self.last_selection_s))
        selected["target_speed_mps"] = self.last_target_speed_mps = target
        self.last_selection_s = monotonic_s
        self.diagnostics = {
            "tracking_mode": self.mode, "upstream_rejection_reason": (
                "upstream_road_prediction" if usable and road.get("degraded") else None if usable else reason),
            "original_measurement_timestamp_s": self.source_stamp_s,
            "original_source_age_s": now_s-self.source_stamp_s,
            "cache_age_s": monotonic_s-self.accepted_at_s,
            "prediction_distance_m": distance_m-self.accepted_distance_m,
            "remaining_path_range_m": [max(0.0, self.endpoints[0][0]), self.endpoints[-1][0]]}
        return selected
