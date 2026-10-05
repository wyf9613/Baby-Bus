"""Hardware-free candidate. Nothing here publishes ROS or enables a vehicle."""
from dataclasses import dataclass
import math
from numbers import Real

import numpy as np


@dataclass(frozen=True)
class Gains:
    speed_kp: float = 0.7
    speed_ki: float = 0.3
    speed_kd: float = 0.0
    lateral_kp: float = 1.0
    lateral_ki: float = 0.0
    lateral_kd: float = 0.0
    heading_kp: float = 1.5
    derivative_tau_s: float = 0.15
    drive_min: float = -0.3
    drive_max: float = 0.35
    wheelbase_m: float = 0.33
    rear_axle_from_cg_m: float = 0.132
    steering_limit_rad: float = math.pi / 4


class PID:
    """Derivative on measurement; conditional integration prevents windup."""

    def __init__(self, kp: float, ki: float, kd: float, tau: float):
        self.kp, self.ki, self.kd, self.tau = kp, ki, kd, tau
        self.integral = self.derivative = 0.0
        self.previous = None

    def update(self, error: float, measurement: float, dt: float,
               low: float, high: float, feedforward: float = 0.0) -> float:
        if dt > 0:
            if self.previous is not None:
                raw = (measurement - self.previous) / dt
                alpha = dt / (self.tau + dt)
                self.derivative += alpha * (raw - self.derivative)
            increment = error * dt
        else:
            increment = 0.0
        self.previous = measurement
        base = feedforward + self.kp * error - self.kd * self.derivative
        proposed = base + self.ki * (self.integral + increment)
        if (low <= proposed <= high or proposed > high and error < 0
                or proposed < low and error > 0):
            self.integral += increment
        return float(np.clip(base + self.ki * self.integral, low, high))


def _finite(value: object) -> bool:
    return isinstance(value, Real) and not isinstance(value, (bool, np.bool_)) and math.isfinite(value)


class ReferenceError(ValueError):
    """A rejected upstream contract; the message is a diagnostic reason."""


@dataclass(frozen=True)
class ControlReference:
    coeffs: tuple[float, ...]
    bounds: tuple[float, float]
    origin: float
    scale: float
    target_speed_mps: float
    stop_requested: bool
    structured: bool


def read_reference(reference: dict, now_s: float, max_age_s: float) -> ControlReference:
    """Consume upstream Cartesian geometry unchanged, plus the legacy mock.

    y(x) = sum(a_i * ((x-origin)/scale)**i), with physical x bounds in metres.
    Structured references describe a fixed body frame: this offline consumer
    requires same-cycle geometry instead of silently reusing it after motion.
    Source-age/stop/lifetime checks remain owned by the producer and consumer.
    """
    if not isinstance(reference, dict) or reference.get('valid') is not True:
        raise ReferenceError('invalid_reference')
    if not _finite(now_s) or not _finite(max_age_s) or max_age_s <= 0:
        raise ReferenceError('invalid_clock_or_age_limit')
    stamp = reference.get('timestamp_s')
    if not _finite(stamp) or not 0 <= now_s - stamp <= max_age_s:
        raise ReferenceError('stale_or_future_reference')
    structured = 'path' in reference
    lifetime = reference.get('valid_for_s')
    if structured or 'valid_for_s' in reference:
        if not _finite(lifetime) or lifetime <= 0 or now_s - stamp >= lifetime:
            raise ReferenceError('expired_reference')
    if (reference.get('frame_id', 'base_link') != 'base_link'
            or reference.get('vehicle_reference_point', 'cg_ground_projection') != 'cg_ground_projection'):
        raise ReferenceError('unsupported_frame_or_reference_point')
    if structured:
        if ('frame_id' not in reference or 'vehicle_reference_point' not in reference):
            raise ReferenceError('missing_frame_or_reference_point')
        if abs(now_s - stamp) > 1e-6:
            raise ReferenceError('reference_frame_not_current')
        path = reference['path']
        if (not isinstance(path, dict) or path.get('type') != 'CARTESIAN_Y_OF_X'
                or path.get('independent_variable') != 'x_m'):
            raise ReferenceError('unsupported_path_encoding')
        coeffs, bounds = path.get('coeffs_low_to_high'), path.get('range')
        origin, scale = path.get('origin'), path.get('scale')
    else:
        coeffs, bounds = reference.get('path_coeffs'), reference.get('x_range_m')
        origin, scale = 0.0, 1.0
    if (not isinstance(coeffs, (list, tuple)) or not 1 <= len(coeffs) <= 6
            or not all(_finite(v) for v in coeffs)):
        raise ReferenceError('invalid_polynomial')
    if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
            or not all(_finite(v) for v in bounds)
            or not bounds[0] < bounds[1] or bounds[1] <= 0):
        raise ReferenceError('invalid_path_range')
    if not _finite(origin) or not _finite(scale) or scale <= 0:
        raise ReferenceError('invalid_path_normalization')
    target = reference.get('target_speed_mps')
    stop = reference.get('stop_requested', False)
    if not _finite(target) or target < 0 or not isinstance(stop, bool):
        raise ReferenceError('invalid_speed_or_stop_request')
    return ControlReference(tuple(float(v) for v in coeffs), tuple(float(v) for v in bounds),
                            float(origin), float(scale), 0.0 if stop else float(target),
                            stop, structured)


def closest_path_geometry(coeffs: tuple[float, ...], bounds: tuple[float, float],
                          origin: float = 0.0, scale: float = 1.0) -> dict:
    """Minimize x*x+y(x)*y(x) on the supplied range, including both ends.

    Quintics can have multiple local minima. Evaluate all real stationary
    points of the distance polynomial (degree <= 9), rather than one Newton
    seed. Normalize the search interval to [-1,1] for the root solve. No path
    extension to x=0 is made for front-only observations. Endpoint errors use
    the tangent at that observed endpoint, an explicit near-field approximation.
    """
    polynomial = np.polynomial.Polynomial
    midpoint = bounds[0]/2 + bounds[1]/2
    half_range = bounds[1]/2 - bounds[0]/2
    x_poly = polynomial([midpoint, half_range])
    y_poly = polynomial(coeffs)(polynomial([(midpoint-origin)/scale, half_range/scale]))
    stationary = (x_poly*x_poly + y_poly*y_poly).deriv()
    candidates = [-1.0, 1.0]
    for root in stationary.roots():
        if abs(root.imag) <= 1e-8 and -1.0 <= root.real <= 1.0:
            candidates.append(float(root.real))
    costs = [float(x_poly(t)**2 + y_poly(t)**2) for t in candidates]
    if not all(math.isfinite(cost) for cost in costs):
        raise ReferenceError('nonfinite_path_geometry')
    t = candidates[int(np.argmin(costs))]
    x, y = float(x_poly(t)), float(y_poly(t))
    slope = float(y_poly.deriv()(t)/half_range)
    second = float(y_poly.deriv(2)(t)/half_range**2)
    heading = math.atan(slope)
    curvature = second / math.hypot(1.0, slope)**3
    error = -x*math.sin(heading) + y*math.cos(heading)
    if not all(math.isfinite(v) for v in (x, y, slope, second, heading, curvature, error)):
        raise ReferenceError('nonfinite_path_geometry')
    return {'path_error_m': error, 'path_heading_rad': heading,
            'curvature_1pm': curvature, 'closest_x_m': x, 'closest_y_m': y,
            'closest_at_range_end': t in (-1.0, 1.0),
            'path_degree': len(coeffs)-1}


def path_errors(coeffs: list[float], bounds: list[float], origin: float = 0.0,
                scale: float = 1.0) -> tuple[float, float, float]:
    """Legacy three-value helper, with support through fifth order."""
    geometry = closest_path_geometry(tuple(coeffs), tuple(bounds), origin, scale)
    return geometry['path_error_m'], geometry['path_heading_rad'], geometry['curvature_1pm']


class Controller:
    """Lateral PID + heading P + curvature feedforward, shared speed PI/PID."""

    def __init__(self, gains: Gains):
        self.gains = gains
        self.speed = PID(gains.speed_kp, gains.speed_ki, gains.speed_kd,
                         gains.derivative_tau_s)
        self.lateral = PID(gains.lateral_kp, gains.lateral_ki, gains.lateral_kd,
                           gains.derivative_tau_s)
        self.stopping = False

    def calculate(self, reference: dict, speed_mps: float, now_s: float,
                  dt: float, max_age_s: float = 0.15) -> tuple[float, float, dict]:
        try:
            if not _finite(speed_mps) or speed_mps < 0 or not _finite(dt) or dt < 0:
                raise ReferenceError('invalid_feedback_or_dt')
            parsed = read_reference(reference, now_s, max_age_s)
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                geometry = closest_path_geometry(parsed.coeffs, parsed.bounds, parsed.origin, parsed.scale)
        except (ValueError, TypeError, ArithmeticError, np.linalg.LinAlgError) as error:
            self.__init__(self.gains)
            return 0.0, 0.0, {'valid': False, 'reason': str(error), 'stop_requested': True}

        ey, heading, curvature = (geometry['path_error_m'], geometry['path_heading_rad'],
                                  geometry['curvature_1pm'])
        target = parsed.target_speed_mps
        g = self.gains
        # CG-path curvature: beta = asin(l_r * kappa), not rear-axle curvature.
        beta = math.asin(float(np.clip(g.rear_axle_from_cg_m * curvature, -0.95, 0.95)))
        delta_ff = math.atan(g.wheelbase_m * curvature / math.cos(beta))
        heading_error = math.atan2(math.sin(heading - beta), math.cos(heading - beta))
        delta = self.lateral.update(
            ey, -ey, dt, -g.steering_limit_rad, g.steering_limit_rad,
            delta_ff + g.heading_kp * heading_error)

        # Explicit simulation-only brake/hold mode. A zero target clears windup.
        if target == 0:
            self.speed = PID(g.speed_kp, g.speed_ki, g.speed_kd, g.derivative_tau_s)
            self.stopping = True
            drive = max(g.drive_min, -g.speed_kp * speed_mps) if speed_mps > 0.03 else g.drive_min
        else:
            if self.stopping:
                self.speed = PID(g.speed_kp, g.speed_ki, g.speed_kd, g.derivative_tau_s)
                self.stopping = False
            drive = self.speed.update(target - speed_mps, speed_mps, dt,
                                      g.drive_min, g.drive_max)
        return drive, delta / g.steering_limit_rad, {
            **geometry, 'valid': True, 'heading_error_rad': heading_error,
            'steering_ff_rad': delta_ff, 'stop_requested': parsed.stop_requested or target == 0,
            'reference_format': 'planning_path' if parsed.structured else 'legacy_mock',
        }
