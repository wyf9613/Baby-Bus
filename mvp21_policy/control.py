"""MVP2.1 control; relocated from pinned f11428b, without algorithm tuning."""

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
    wrap_angle,
)
from .planning import (
    CenterlinePlanner,
)


@dataclass
class ControlSettings:
    """Forward-only candidate with direct MVP and optional calibrated profiles.

    enabled defaults false for bare-node teaching exercises; the shipped YAML
    selects the integrated policy. Gains are the offline selected candidates,
    not physical tuning evidence. Negative effort is never issued by this node.
    """
    enabled: bool = False
    mode: str = "calibrated"
    vehicle_params_source: str = "upstream"
    speed_kp: float = 1.4
    speed_ki: float = 0.6
    speed_kd: float = 0.0
    lateral_kp: float = 1.2
    lateral_ki: float = 0.1
    lateral_kd: float = 0.0
    heading_kp: float = 0.8
    derivative_tau_s: float = 0.15
    drive_max: float = 0.15
    steering_max_normalized: float = 0.5
    max_dt_s: float = 0.2
    startup_grace_s: float = 0.5
    # Local MPC integration: neutral-only refresh and first planner acquisition.
    # Zero retains the upstream PID lifecycle.
    reference_refresh_grace_s: float = 0.0
    planner_startup_wait_s: float = 0.0
    # Per explicit run. Zero disables that test budget; odometry still updates.
    max_distance_m: float = 3.0
    max_run_time_s: float = 30.0
    # Optional terminal manoeuvre after tracking loses usable boundary geometry.
    # Fixed effort and zero steering; raw wheel odometry bounds the extra travel.
    # Zero distance disables it. This does not identify a physical finish line.
    end_straight_distance_m: float = 0.0
    end_straight_drive_normalized: float = 0.1
    end_straight_max_time_s: float = 3.0
    # Direct normalized steering for the first-run MVP, no wheel-angle model.
    mvp_lateral_kp: float = 1.0
    mvp_lateral_ki: float = 0.0
    mvp_heading_kp: float = 0.5
    mvp_steering_direction: float = 1.0
    # Normalized policy units per second; zero preserves unrestricted legacy
    # response. Operator/fault/planner stops always bypass this motion limiter.
    mvp_steering_slew_rate_per_s: float = 0.0
    # Measured normalized effort needed to sustain the selected crawl speed.
    # Optional and zero by default; stop/invalid paths bypass this compensation.
    mvp_drive_feedforward: float = 0.0
    # Low-pass only current-frame PID errors at boundary-source transitions.
    # Zero keeps legacy feedback; invalid input/operator stop bypasses it.
    mvp_error_filter_tau_s: float = 0.0
    # Ordered-XY corner entry: continuous local/forward-target feedback plus
    # preview-tangent anticipation. False retains the archived MVP calculation.
    mvp_ordered_cornering_enabled: bool = False
    # Control-only recovery from transient road failures; legacy code defaults
    # stay off. Shipped MVP YAML enables it. These are experiment budgets.
    robustness_enabled: bool = False
    # Explicit camera-frame tracking when yaw rate is unavailable. Latched for
    # this run; it never invents gyro samples or dead-reckons an old path.
    gyro_fallback_enabled: bool = False
    gyro_fallback_max_camera_age_s: float = 0.35
    gyro_fallback_speed_mps: float = 0.2
    reference_hold_max_s: float = 1.0
    reference_hold_max_distance_m: float = 0.25

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("control.enabled must be bool")
        if not isinstance(self.robustness_enabled, bool):
            raise ValueError("control.robustness_enabled must be bool")
        if not isinstance(self.gyro_fallback_enabled, bool):
            raise ValueError("control.gyro_fallback_enabled must be bool")
        if self.gyro_fallback_enabled and self.mode != "mvp":
            raise ValueError("Gyro fallback requires control.mode=mvp")
        if not isinstance(self.mvp_ordered_cornering_enabled, bool):
            raise ValueError("control.mvp_ordered_cornering_enabled must be bool")
        if self.mode not in ("mvp", "calibrated"):
            raise ValueError("control.mode must be mvp or calibrated")
        if self.vehicle_params_source not in ("upstream", "course_simulation"):
            raise ValueError("control.vehicle_params_source must be upstream or course_simulation")
        for name in ("speed_kp", "speed_ki", "speed_kd", "lateral_kp", "lateral_ki",
                     "lateral_kd", "heading_kp", "mvp_lateral_kp", "mvp_lateral_ki", "mvp_heading_kp"):
            if not finite_number(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"control.{name} must be finite and nonnegative")
        for name in ("derivative_tau_s", "drive_max", "steering_max_normalized",
                     "max_dt_s", "startup_grace_s",
                     "reference_hold_max_s", "reference_hold_max_distance_m",
                     "end_straight_max_time_s", "gyro_fallback_max_camera_age_s",
                     "gyro_fallback_speed_mps"):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"control.{name} must be finite and positive")
        for name in ("max_distance_m", "max_run_time_s", "end_straight_distance_m", "mvp_error_filter_tau_s",
                     "reference_refresh_grace_s", "planner_startup_wait_s"):
            if not finite_number(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"control.{name} must be finite and nonnegative (0 disables)")
        if (not finite_number(self.mvp_steering_slew_rate_per_s)
                or self.mvp_steering_slew_rate_per_s < 0):
            raise ValueError("control.mvp_steering_slew_rate_per_s must be finite and nonnegative")
        if self.reference_refresh_grace_s > .2:
            raise ValueError("control.reference_refresh_grace_s must not exceed 0.2")
        if self.planner_startup_wait_s >= self.startup_grace_s:
            raise ValueError("planner startup wait must fit inside control startup grace")
        if self.drive_max > 1 or self.steering_max_normalized > 1:
            raise ValueError("control action limits must not exceed 1")
        if not finite_number(self.mvp_steering_direction) or self.mvp_steering_direction not in (-1.0, 1.0):
            raise ValueError("control.mvp_steering_direction must be -1.0 or 1.0")
        if (not finite_number(self.mvp_drive_feedforward)
                or not 0 <= self.mvp_drive_feedforward <= self.drive_max):
            raise ValueError("control.mvp_drive_feedforward must be within [0, drive_max]")
        if (not finite_number(self.end_straight_drive_normalized)
                or not 0 < self.end_straight_drive_normalized <= 1.0):
            raise ValueError("control.end_straight_drive_normalized must be within (0, 1]")
        if self.end_straight_distance_m > 0:
            if self.mode != "mvp":
                raise ValueError("End straight requires control.mode=mvp")
            if self.end_straight_drive_normalized > self.drive_max:
                raise ValueError("End straight effort exceeds control.drive_max")


class PolicyStopRequest(Exception):
    """Student code requests a normal stop; the framework owns publication/FSM."""


class RunDistanceLimiter:
    """Unsigned raw-wheel-speed odometry, trapezoidal integration on monotonic time.

    Active only in policy state 3. Start resets the per-run budget; stopping
    keeps the final estimate visible. Frequent supervisor/policy checks share
    one integration clock, so a cached sample is never double-counted. Encoder
    scale, slip, slew/coasting and sampling errors affect physical stop distance.
    """
    def __init__(self, maximum_m, max_gap_s):
        self.maximum_m, self.max_gap_s = maximum_m, max_gap_s
        self.reset()

    def reset(self, now_s=None, speed_mps=None):
        self.distance_m = 0.0
        self.previous_time_s, self.previous_speed_mps = now_s, speed_mps

    def advance(self, speed_mps, now_s):
        if not all(finite_number(v) for v in (speed_mps, now_s)) or speed_mps < 0:
            return "Invalid wheel-speed distance input"
        if self.previous_time_s is not None:
            dt = now_s-self.previous_time_s
            if not 0 <= dt <= self.max_gap_s+1e-9:
                return "Wheel-speed distance clock/gap invalid"
            self.distance_m += 0.5*(self.previous_speed_mps+speed_mps)*dt
        self.previous_time_s, self.previous_speed_mps = now_s, speed_mps
        if not finite_number(self.distance_m):
            return "Wheel-speed distance estimate invalid"
        if self.maximum_m > 0 and self.distance_m+1e-9 >= self.maximum_m:
            return f"Distance limit reached: {self.distance_m:.3f} m / {self.maximum_m:.3f} m; explicit resume required"
        return None


@dataclass
class VehicleCalibrationSettings:
    """Measured candidate inputs in the existing YAML; no inferred car facts.

    Steering angle is the wheel angle reached at policy steer=+/-1 AFTER the
    vehicle interface's configured scaling. direction=+1 means a positive
    policy request turns left, -1 means right. Geometry uses the CG ground point.
    valid=True requires a source label and all physical fields; untouched zero
    defaults stay unavailable. No vehicle-interface settings are modified here.
    """
    valid: bool = False
    source: str = "unmeasured"
    wheelbase_m: float = 0.0
    rear_axle_from_cg_m: float = 0.0
    steering_limit_rad: float = 0.0
    steering_direction: float = 1.0
    front_extent_m: float = 0.0
    rear_extent_m: float = 0.0
    width_m: float = 0.0
    speed_max_mps: float = 0.0
    braking_deceleration_mps2: float = 0.0
    actuation_delay_s: float = 0.0
    safety_margin_m: float = 0.0

    def __post_init__(self):
        if not isinstance(self.valid, bool) or not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("vehicle calibration requires a boolean valid and a source label")
        numeric = [v for k, v in vars(self).items() if k not in ("valid", "source")]
        if not all(finite_number(v) for v in numeric):
            raise ValueError("vehicle calibration values must be finite")
        if self.steering_direction not in (-1.0, 1.0):
            raise ValueError("vehicle.steering_direction must be -1.0 or 1.0")
        if any(v < 0 for k, v in vars(self).items() if k not in ("valid", "source", "steering_direction")):
            raise ValueError("vehicle dimensions and limits must be nonnegative")
        if not self.valid:
            return
        if self.source in ("unmeasured", "course_simulation", "offline_test_assumption"):
            raise ValueError("valid vehicle calibration requires a measured source label")
        for name in ("wheelbase_m", "rear_axle_from_cg_m", "steering_limit_rad", "front_extent_m",
                     "rear_extent_m", "width_m", "speed_max_mps", "braking_deceleration_mps2"):
            if getattr(self, name) <= 0:
                raise ValueError(f"vehicle.{name} must be positive when calibration is valid")
        if (not self.rear_axle_from_cg_m < self.wheelbase_m or self.steering_limit_rad >= math.pi/2
                or self.rear_extent_m < self.rear_axle_from_cg_m
                or self.front_extent_m < self.wheelbase_m-self.rear_axle_from_cg_m):
            raise ValueError("vehicle geometry or steering range is inconsistent")

    def records(self):
        params = {"valid": self.valid, "source": self.source,
                  "vehicle_reference_point": "cg_ground_projection",
                  "effective_wheelbase_m": self.wheelbase_m,
                  "rear_axle_from_cg_m": self.rear_axle_from_cg_m,
                  "steering_limit_rad": self.steering_limit_rad,
                  "steering_direction": self.steering_direction}
        limits = {"valid": self.valid, "source": self.source,
                  "vehicle_reference_point": "cg_ground_projection",
                  "footprint_xy_m": [(-self.rear_extent_m, -self.width_m/2),
                                     (self.front_extent_m, -self.width_m/2),
                                     (self.front_extent_m, self.width_m/2),
                                     (-self.rear_extent_m, self.width_m/2)],
                  "speed_max_mps": self.speed_max_mps,
                  "braking_deceleration_mps2": self.braking_deceleration_mps2,
                  "actuation_delay_s": self.actuation_delay_s,
                  "safety_margin_m": self.safety_margin_m}
        return params, limits


class ControlPID:
    """Derivative on measurement, filtered; conditional anti-windup."""
    def __init__(self, kp, ki, kd, tau):
        self.kp, self.ki, self.kd, self.tau = kp, ki, kd, tau
        self.integral = self.derivative = 0.0
        self.previous = None

    def update(self, error, measurement, dt, low, high, feedforward=0.0):
        increment = error*dt
        if dt > 0 and self.previous is not None:
            self.derivative += dt/(self.tau+dt)*((measurement-self.previous)/dt-self.derivative)
        self.previous = measurement
        base = feedforward+self.kp*error-self.kd*self.derivative
        proposed = base+self.ki*(self.integral+increment)
        if low <= proposed <= high or proposed > high and error < 0 or proposed < low and error > 0:
            self.integral += increment
        result = base+self.ki*self.integral
        if not all(finite_number(v) for v in (result, self.integral, self.derivative)):
            raise ValueError("nonfinite_control_state")
        return max(low, min(high, result))


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


def _control_centerline_geometry(path):
    """Nearest segment cross-track error plus a forward tangent for steering.

    O(N), N <= 201. Use observed points only, with no polynomial fit, extension
    or recovery path. Endpoint tangents approximate error for front-only input.
    """
    points = CenterlinePlanner._points(path.get("samples_xy"))
    preview = path.get("preview_m")
    bounds = path.get("range")
    if (points is None or path.get("independent_variable") != "x_m"
            or not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(finite_number(v) for v in bounds)
            or list(bounds) != [points[0][0], points[-1][0]] or points[-1][0] <= 0
            or not finite_number(preview) or preview <= 0):
        raise ValueError("invalid_centerline_path")
    nearest = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        u = max(0.0, min(1.0, -(a[0]*dx+a[1]*dy)/(dx*dx+dy*dy)))
        x, y = a[0]+u*dx, a[1]+u*dy
        cost = x*x+y*y
        if nearest is None or cost < nearest[0]:
            nearest = (cost, x, y, math.atan2(dy, dx), i, u)
    _, x, y, tangent, index, u = nearest
    target_x = max(points[0][0], min(points[-1][0], max(0.0, x)+preview))
    segment = len(points)-2
    for i, (_, b) in enumerate(zip(points, points[1:])):
        if target_x <= b[0]:
            segment = i
            break
    a, b = points[segment], points[segment+1]
    slope = (b[1]-a[1])/(b[0]-a[0])
    target_y = a[1]+slope*(target_x-a[0])
    # Curvature is diagnostic only in MVP. Use three neighboring observed points.
    j = max(1, min(len(points)-2, segment+1))
    curvature = 0.0
    if len(points) >= 3:
        a0, b0, c0 = points[j-1:j+2]
        ab, bc, ac = (b0[0]-a0[0], b0[1]-a0[1]), (c0[0]-b0[0], c0[1]-b0[1]), (c0[0]-a0[0], c0[1]-a0[1])
        curvature = 2*(ab[0]*bc[1]-ab[1]*bc[0])/(math.hypot(*ab)*math.hypot(*bc)*math.hypot(*ac))
    result = {"path_error_m": -x*math.sin(tangent)+y*math.cos(tangent),
              "path_heading_rad": math.atan(slope), "curvature_1pm": curvature,
              "closest_x_m": x, "closest_y_m": y, "preview_x_m": target_x, "preview_y_m": target_y,
              "closest_at_range_end": index == 0 and u == 0 or index == len(points)-2 and u == 1,
              "path_encoding": "CARTESIAN_CENTERLINE"}
    if not all(finite_number(v) for k, v in result.items() if k not in ("path_encoding", "closest_at_range_end")):
        raise ValueError("nonfinite_centerline_geometry")
    return result


def _control_ordered_geometry(path):
    points = _path_points(path.get("samples_xy"))
    preview = path.get("preview_m")
    if points is None or path.get("independent_variable") != "s_m" or not finite_number(preview) or preview <= 0:
        raise ValueError("invalid_ordered_path")
    lengths = _path_lengths(points)
    bounds = path.get("range")
    if (not isinstance(bounds, (tuple, list)) or len(bounds) != 2
            or not all(finite_number(v) for v in bounds)
            or bounds[0] != 0 or abs(bounds[1]-lengths[-1]) > 1e-6):
        raise ValueError("invalid_ordered_path_range")
    nearest = _path_project(points, lengths)
    if nearest is None:
        raise ValueError("ordered_branch_unavailable")
    _, distance, (x, y), index, u = nearest
    target_s = min(lengths[-1], distance+preview)
    target = _path_sample(points, lengths, target_s)
    tx, ty = _path_tangent(points, lengths, distance)
    hx, hy = _path_tangent(points, lengths, target_s)
    # Signed local-normal error and an arc-length preview tangent still drive
    # the existing PI + heading P. No pure-pursuit curvature law is used.
    before = max(0.0, target_s-0.15)
    after = min(lengths[-1], target_s+0.15)
    a = _path_tangent(points, lengths, before)
    b = _path_tangent(points, lengths, after)
    curvature = wrap_angle(math.atan2(b[1], b[0])-math.atan2(a[1], a[0]))/(after-before)
    return {"path_error_m": -x*ty+y*tx, "path_heading_rad": math.atan2(hy, hx),
            "curvature_1pm": curvature, "closest_x_m": x, "closest_y_m": y,
            "closest_s_m": distance, "preview_s_m": target_s,
            "preview_x_m": target[0], "preview_y_m": target[1],
            # Zero on an interior projection; continuous as the observed
            # first segment reaches/passes the vehicle's normal projection.
            "unobserved_entry_distance_m": max(0.0, x*tx+y*ty) if index == 0 and u == 0 else 0.0,
            "closest_at_range_end": index == 0 and u == 0 or index == len(points)-2 and u == 1,
            "forward_target_only": index == 0 and u == 0 and x*tx+y*ty > 1e-6,
            "path_encoding": "ORDERED_CENTERLINE"}


def control_path_geometry(path):
    """Bounded closest observed point for y(x), up to fifth degree.

    u=(x-origin)/scale, low-to-high coefficients, physical range in metres.
    Front-only support is retained; endpoint tangent error is an approximation,
    not evidence of observed road at the car origin. No path extrapolation.
    """
    if isinstance(path, dict) and path.get("type") == "CARTESIAN_CENTERLINE":
        return _control_centerline_geometry(path)
    if isinstance(path, dict) and path.get("type") == "ORDERED_CENTERLINE":
        return _control_ordered_geometry(path)
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


class ControlReferenceManager:
    """MVP straight-reference cache; never edits the upstream planning output.

    Cache budgets start at acceptance of a DISTINCT observed cone frame. Motion
    is integrated incrementally, preserving the original source timestamp. The
    manager generates geometry, never cached actuator commands. Prediction may
    end earlier than its budgets when observed support or motion history ends.
    """
    SOFT_REASONS = frozenset({
        "invalid_estimates", "stale_road", "v1_requires_both_boundaries",
        "unsupported_road_curvature", "insufficient_near_or_far_coverage",
        "not_straight_enough", "outside_v1_offset_or_heading_domain"})

    def __init__(self, settings):
        self.settings = settings
        self.clear()

    def clear(self):
        self.reference = None
        self.source_stamp_s = self.accepted_at_s = self.accepted_distance_m = None
        self.frame_time_s = None
        self.endpoints = None
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
        usable = reference.get("valid") is True
        if usable:
            stamp, lifetime = reference.get("timestamp_s"), reference.get("valid_for_s")
            if not all(finite_number(v) for v in (stamp, lifetime)) or lifetime <= 0 or stamp > now_s:
                raise ValueError("Invalid upstream control reference timestamp")
            if (reference.get("frame_id") != state.get("frame_id")
                    or reference.get("vehicle_reference_point") != "cg_ground_projection"
                    or abs(stamp-state.get("timestamp_s", math.inf)) > 1e-6):
                raise ValueError("control_frame_or_time_mismatch")
            if reference.get("stop_requested") or reference.get("target_speed_mps") == 0:
                return reference
            usable = now_s < stamp+lifetime
        reason = reference.get("reason") if not reference.get("valid") else "reference_expired"
        if not usable and reason not in self.SOFT_REASONS | {"reference_expired"}:
            raise ValueError("Planning: " + str(reason))

        path = reference.get("path") if usable else None
        straight = (isinstance(path, dict) and path.get("type") == "CARTESIAN_Y_OF_X"
                    and path.get("independent_variable") == "x_m"
                    and isinstance(path.get("coeffs_low_to_high"), (list, tuple))
                    and 1 <= len(path["coeffs_low_to_high"]) <= 2)
        if usable and not straight:
            # Other path encodings keep their original control/deadline behavior.
            self.clear()
            return reference
        if usable:
            control_path_geometry(path)  # Validate the full polynomial contract.
            if not finite_number(source_stamp) or source_stamp > now_s:
                raise ValueError("Invalid trusted road timestamp")
            if self.source_stamp_s is not None and source_stamp < self.source_stamp_s:
                raise ValueError("Control road sample moved backwards")
            if self.reference is None or source_stamp != self.source_stamp_s:
                self.reference = deepcopy(reference)
                self.source_stamp_s = source_stamp
                self.accepted_at_s, self.accepted_distance_m = monotonic_s, distance_m
                self.frame_time_s = stamp
                coeffs = path["coeffs_low_to_high"]
                self.endpoints = [(x, _control_poly_eval(coeffs, (x-path["origin"])/path["scale"]))
                                  for x in path["range"]]

        if not usable:
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
            predicted_path = {"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m",
                              "origin": 0.0, "scale": 1.0,
                              "coeffs_low_to_high": [intercept, slope],
                              "range": [max(0.0, x0), x1]}

        selected = deepcopy(reference if usable else self.reference)
        if not usable:
            selected["path"] = predicted_path
        selected.update(timestamp_s=now_s, valid_for_s=execution_lifetime_s)
        self.mode = "TRACKING" if usable else "DEGRADED"
        self.diagnostics = {
            "tracking_mode": self.mode, "upstream_rejection_reason": None if usable else reason,
            "original_measurement_timestamp_s": self.source_stamp_s,
            "original_source_age_s": now_s-self.source_stamp_s,
            "cache_age_s": monotonic_s-self.accepted_at_s,
            "prediction_distance_m": distance_m-self.accepted_distance_m,
            "remaining_path_range_m": list(selected["path"]["range"])}
        return selected


class PolicyController:
    """Single-file forward-only control; failures request the framework stop.

    Does not implement ESC negative-drive brake/hold, physical enabling, frame
    propagation or automatic resume. All those concerns stay explicit.
    """
    def __init__(self, settings, frame_id="base_link"):
        self.settings, self.frame_id = settings, frame_id
        self.clear()

    def clear(self):
        cfg = self.settings
        self.speed = ControlPID(cfg.speed_kp, cfg.speed_ki, cfg.speed_kd, cfg.derivative_tau_s)
        self.lateral = ControlPID(cfg.lateral_kp, cfg.lateral_ki, cfg.lateral_kd, cfg.derivative_tau_s)
        self.mvp_lateral = ControlPID(cfg.mvp_lateral_kp, cfg.mvp_lateral_ki, 0.0, cfg.derivative_tau_s)
        self._mvp_previous_steer = 0.0
        self._mvp_filtered_errors = None

    def calculate(self, reference, state, vehicle_params, now_s, dt):
        try:
            if (not isinstance(reference, dict) or reference.get("valid") is not True
                    or not isinstance(state, dict) or state.get("speed_valid") is not True):
                raise ValueError("invalid_planning_or_speed")
            stamp, lifetime = reference.get("timestamp_s"), reference.get("valid_for_s")
            if (not all(finite_number(v) for v in (stamp, lifetime, now_s, dt))
                    or lifetime <= 0 or not 0 <= dt <= self.settings.max_dt_s
                    or not stamp <= now_s < stamp+lifetime):
                raise ValueError("reference_expired_or_control_gap")
            if (reference.get("frame_id") != self.frame_id or state.get("frame_id") != self.frame_id
                    or reference.get("vehicle_reference_point") != "cg_ground_projection"
                    or not finite_number(state.get("timestamp_s"))
                    or abs(state["timestamp_s"]-stamp) > 1e-6):
                raise ValueError("control_frame_or_time_mismatch")
            # now_s may be slightly later due to computation, but state and path
            # must come from the SAME cycle. This caller never replays a reference.
            target, speed = reference.get("target_speed_mps"), state.get("speed_mps")
            if not all(finite_number(v) and v >= 0 for v in (target, speed)):
                raise ValueError("invalid_speed")
            stop = reference.get("stop_requested")
            if not isinstance(stop, bool):
                raise ValueError("invalid_stop_request")
            if stop or target == 0:
                self.clear()
                return 0.0, 0.0, {"valid": True, "stop_requested": True, "reason": "planning_stop_request"}
            if self.settings.mode == "mvp":
                if reference.get("simulation_only"):
                    raise ValueError("simulation_reference_rejected_for_mvp")
                geometry = control_path_geometry(reference.get("path"))
                cfg = self.settings
                heading = geometry["path_heading_rad"]
                lateral_error = geometry["path_error_m"]
                error_basis = "nearest_segment_normal"
                local_weight = None
                target_bearing = None
                if cfg.mvp_ordered_cornering_enabled and geometry.get("path_encoding") == "ORDERED_CENTERLINE":
                    # Far-only cones cannot establish a local cross-track
                    # error, but their measured preview tangent still tells
                    # us about an approaching corner. Blend rather than
                    # swapping error conventions at the first-point boundary.
                    # Weight approaches zero for distant support and one as
                    # the observed entry reaches the car. No extrapolated
                    # road or assumed wheel-angle/curvature feedforward.
                    preview = reference["path"]["preview_m"]
                    target_bearing = math.atan2(geometry["preview_y_m"], geometry["preview_x_m"])
                    local_weight = preview/(preview+geometry["unobserved_entry_distance_m"])
                    lateral_error = ((1-local_weight)*geometry["preview_y_m"]
                                     + local_weight*lateral_error)
                    heading = wrap_angle(target_bearing+local_weight*wrap_angle(heading-target_bearing))
                    error_basis = "blended_ordered_corner"
                elif ((geometry.get("path_encoding") == "CARTESIAN_CENTERLINE"
                        and reference["path"]["range"][0] > 1e-9)
                        or geometry.get("forward_target_only")):
                    # The curve does not observe the vehicle origin. Projecting
                    # a distant bend's tangent back to the car manufactures a
                    # local cross-track error (seen as a 0.41 m jump on car 15).
                    # Aim at the existing observed preview point instead; no
                    # road extension, stale-reference hold or guessed near lane.
                    lateral_error = geometry["preview_y_m"]
                    target_bearing = math.atan2(geometry["preview_y_m"], geometry["preview_x_m"])
                    heading = target_bearing
                    error_basis = "observed_forward_target"
                raw_errors = (lateral_error, heading)
                if cfg.mvp_error_filter_tau_s > 0 and self._mvp_filtered_errors is not None:
                    weight = dt/(cfg.mvp_error_filter_tau_s+dt)
                    old_lateral, old_heading = self._mvp_filtered_errors
                    lateral_error = old_lateral+weight*(lateral_error-old_lateral)
                    heading = wrap_angle(old_heading+weight*wrap_angle(heading-old_heading))
                self._mvp_filtered_errors = (lateral_error, heading)
                # Direct normalized PI + heading P, no degree-to-servo map or
                # curvature feedforward borrowed from the bicycle simulation.
                steer = self.mvp_lateral.update(lateral_error, -lateral_error, dt,
                    -cfg.steering_max_normalized, cfg.steering_max_normalized,
                    cfg.mvp_heading_kp*heading)
                drive = self.speed.update(target-speed, speed, dt, 0.0, cfg.drive_max,
                                          feedforward=cfg.mvp_drive_feedforward)
                requested_steer = cfg.mvp_steering_direction*steer
                output_steer = requested_steer
                if cfg.mvp_steering_slew_rate_per_s > 0:
                    step = cfg.mvp_steering_slew_rate_per_s*dt
                    output_steer = max(self._mvp_previous_steer-step,
                                       min(self._mvp_previous_steer+step, requested_steer))
                self._mvp_previous_steer = output_steer
                return drive, output_steer, {
                    **geometry, "valid": True, "stop_requested": False, "reason": "mvp_tracking",
                    "control_lateral_error_m": lateral_error, "error_basis": error_basis,
                    "raw_lateral_error_m": raw_errors[0], "raw_heading_error_rad": raw_errors[1],
                    "corner_local_weight": local_weight,
                    "preview_tangent_heading_rad": geometry["path_heading_rad"],
                    "target_bearing_rad": target_bearing,
                    "steer_before_slew": requested_steer,
                    "steer_slew_limited": output_steer != requested_steer,
                    "heading_error_rad": heading, "speed_error_mps": target-speed,
                    "operating_profile": "mvp_low_speed", "simulation_only": False}
            params = vehicle_params
            simulated = self.settings.vehicle_params_source == "course_simulation"
            if simulated:
                if reference.get("simulation_only") is not True:
                    raise ValueError("simulation_control_requires_simulation_reference")
                params = {"valid": True, "source": "course_simulation",
                          "vehicle_reference_point": "cg_ground_projection",
                          "effective_wheelbase_m": 0.33, "rear_axle_from_cg_m": 0.132,
                          "steering_limit_rad": math.pi/4, "steering_direction": 1.0}
            elif reference.get("simulation_only"):
                raise ValueError("simulation_reference_rejected_for_live_control")
            if (not simulated and isinstance(params, dict) and
                    (params.get("simulation_only") or params.get("source") in
                     ("course_simulation", "offline_test_assumption"))):
                raise ValueError("simulation_geometry_rejected_for_live_control")
            if (not isinstance(params, dict) or params.get("valid") is not True
                    or params.get("source") in (None, "", "unmeasured")
                    or params.get("vehicle_reference_point") != "cg_ground_projection"):
                raise ValueError("vehicle_control_geometry_unavailable")
            wheelbase, rear, limit, direction = (params.get(k) for k in
                ("effective_wheelbase_m", "rear_axle_from_cg_m", "steering_limit_rad", "steering_direction"))
            if (not all(finite_number(v) for v in (wheelbase, rear, limit, direction))
                    or not 0 < rear < wheelbase or not 0 < limit < math.pi/2 or direction not in (-1, 1)):
                raise ValueError("invalid_vehicle_control_geometry")
            geometry = control_path_geometry(reference.get("path"))
            ey, curvature = geometry["path_error_m"], geometry["curvature_1pm"]
            if abs(rear*curvature) >= 1:
                raise ValueError("curvature_outside_vehicle_geometry")
            beta = math.asin(rear*curvature)
            feedforward = math.atan(wheelbase*curvature/math.cos(beta))
            heading = wrap_angle(geometry["path_heading_rad"]-beta)
            cfg = self.settings
            delta_limit = limit*cfg.steering_max_normalized
            delta = self.lateral.update(ey, -ey, dt, -delta_limit, delta_limit,
                                        feedforward+cfg.heading_kp*heading)
            drive = self.speed.update(target-speed, speed, dt, 0.0, cfg.drive_max)
            return drive, direction*delta/limit, {
                **geometry, "valid": True, "stop_requested": False, "reason": "tracking",
                "heading_error_rad": heading, "speed_error_mps": target-speed,
                "steering_ff_rad": feedforward, "vehicle_params_source": params["source"],
                "simulation_only": simulated}
        except (ValueError, TypeError, ArithmeticError) as error:
            self.clear()
            return 0.0, 0.0, {"valid": False, "stop_requested": True, "reason": str(error)}
