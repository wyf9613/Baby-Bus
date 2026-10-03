"""Hardware-free candidate. Nothing here publishes ROS or enables a vehicle."""
from dataclasses import dataclass
import math

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


def path_errors(coeffs: list[float], bounds: list[float]) -> tuple[float, float, float]:
    """Closest point on y(x); positive error requests motion to the left.

    A small bounded grid initializes Newton refinement. This is local geometry,
    not a world-frame y error. Coefficients are in ascending power order.
    """
    a = np.asarray(coeffs, dtype=float)
    d1 = np.polynomial.polynomial.polyder(a)
    d2 = np.polynomial.polynomial.polyder(d1)
    xs = np.linspace(*bounds, 31)
    ys = np.polynomial.polynomial.polyval(xs, a)
    x = float(xs[np.argmin(xs * xs + ys * ys)])
    for _ in range(5):
        y = float(np.polynomial.polynomial.polyval(x, a))
        slope = float(np.polynomial.polynomial.polyval(x, d1))
        second = float(np.polynomial.polynomial.polyval(x, d2))
        denominator = 1 + slope * slope + y * second
        if abs(denominator) < 1e-8:
            break
        x = float(np.clip(x - (x + y * slope) / denominator, *bounds))
    y = float(np.polynomial.polynomial.polyval(x, a))
    slope = float(np.polynomial.polynomial.polyval(x, d1))
    second = float(np.polynomial.polynomial.polyval(x, d2))
    heading = math.atan(slope)
    error = -x * math.sin(heading) + y * math.cos(heading)
    curvature = second / (1 + slope * slope) ** 1.5
    return error, heading, curvature


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
            age = now_s - reference['timestamp_s']
            coeffs, bounds = reference['path_coeffs'], reference['x_range_m']
            target = reference['target_speed_mps']
            valid = (reference['valid'] is True and 0 <= age <= max_age_s
                     and len(coeffs) == 4 and len(bounds) == 2
                     and bounds[0] <= 0 < bounds[1]
                     and target >= 0 and speed_mps >= 0 and dt >= 0
                     and all(math.isfinite(v) for v in
                             [*coeffs, *bounds, target, speed_mps, dt, now_s, age]))
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.__init__(self.gains)
            return 0.0, 0.0, {'valid': False}

        ey, heading, curvature = path_errors(coeffs, bounds)
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
            'valid': True, 'path_error_m': ey, 'heading_error_rad': heading_error,
            'curvature_1pm': curvature, 'steering_ff_rad': delta_ff,
        }
