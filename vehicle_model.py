"""Low-speed MPC prediction model for the course cone-following notebook.

state = [COM_x_m, COM_y_m, heading_rad, body_vx_mps, steering_rad]
control = [normalized_drive, normalized_steering_target], each in [-1, 1].
Coordinates: x forward at heading=0, y left, positive heading/steering left.
Pure prediction only: no Gym, ROS, observations, controller, or hidden state.

Assumes no lateral slip, classic linear motor gain and quadratic drag.
Does NOT implement the simulator's direction-change latch or dynamic tyre model.
Negative drive can predict reverse motion: do not use this alone to validate
braking/parking. Add the latch state before modelling stop/reverse transitions.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class VehicleParams:
    front_axle_m: float = 0.198
    rear_axle_m: float = 0.132
    mass_kg: float = 3.0
    motor_gain_n: float = 10.0
    drag_kg_per_m: float = 1.0
    steering_limit_rad: float = np.pi / 4
    steering_rate_lower_rad_s: float = -np.pi / 2
    steering_rate_upper_rad_s: float = np.pi / 2

    def __post_init__(self):
        values = np.array(list(vars(self).values()), dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError('Parameters must be finite')
        if min(self.front_axle_m, self.rear_axle_m, self.mass_kg, self.motor_gain_n) <= 0:
            raise ValueError('Axle distances, mass and motor gain must be positive')
        if self.drag_kg_per_m < 0 or not 0 < self.steering_limit_rad < np.pi / 2:
            raise ValueError('Invalid drag or steering limit')
        if not self.steering_rate_lower_rad_s <= 0 <= self.steering_rate_upper_rad_s:
            raise ValueError('Steering rate bounds must contain zero')

    @classmethod
    def from_notebook_config(cls, config):
        """Read the explicit classic-model fields in bicycle_model_config.

        This is a course-notebook adapter, not a general Dream Gym config parser.
        """
        axle = config['axle_geometry']
        force = config['longitudinal_force_model']
        steering = config['steering']
        if steering['physical_front_wheel_angle_offset_from_request_rad'] != 0:
            raise ValueError('This model assumes zero steering offset')
        rates = steering['physical_front_wheel_angle_rate_bounds_rad_per_s']
        return cls(
            front_axle_m=axle['front_axle_distance_from_center_of_mass_m'],
            rear_axle_m=axle['rear_axle_distance_from_center_of_mass_m'],
            mass_kg=config['mass_properties']['mass_kg'],
            motor_gain_n=force['normalized_motor_command_force_gain_n'],
            drag_kg_per_m=force['quadratic_aerodynamic_drag_coefficient_kg_per_m'],
            steering_limit_rad=steering['requested_front_wheel_angle_magnitude_limit_rad'],
            steering_rate_lower_rad_s=rates['lower'],
            steering_rate_upper_rad_s=rates['upper'],
        )


def step(state, control, dt, params=None):
    """Return next state using one RK4 step; never modify inputs.

    dt is seconds and must be positive. Steering target is reached at the end
    of the interval when permitted by rate limits, matching the course plant.
    Heading remains unwrapped for smooth multi-step prediction. Invalid/out of
    bounds controls raise ValueError instead of silently hiding optimizer errors.
    """
    p = VehicleParams() if params is None else params
    s = np.asarray(state, dtype=float)
    u = np.asarray(control, dtype=float)
    if s.shape != (5,) or u.shape != (2,):
        raise ValueError('Expected state shape (5,) and control shape (2,)')
    if not np.all(np.isfinite(s)) or not np.all(np.isfinite(u)):
        raise ValueError('State and control must be finite')
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be positive and finite')
    if np.any(np.abs(u) > 1):
        raise ValueError('Normalized controls must lie in [-1, 1]')
    if abs(s[4]) > p.steering_limit_rad + 1e-12:
        raise ValueError('Initial steering exceeds model limit')
    wheelbase = p.front_axle_m + p.rear_axle_m
    steering_rate = np.clip(
        (u[1] * p.steering_limit_rad - s[4]) / dt,
        p.steering_rate_lower_rad_s, p.steering_rate_upper_rad_s,
    )

    def derivative(z):
        _, _, psi, vx, delta = z
        yaw_rate = vx * np.tan(delta) / wheelbase
        vy = p.rear_axle_m * yaw_rate
        return np.array([
            vx * np.cos(psi) - vy * np.sin(psi),
            vx * np.sin(psi) + vy * np.cos(psi),
            yaw_rate,
            (p.motor_gain_n * u[0] - p.drag_kg_per_m * vx * abs(vx)) / p.mass_kg,
            steering_rate,
        ])

    k1 = derivative(s)
    k2 = derivative(s + dt * k1 / 2)
    k3 = derivative(s + dt * k2 / 2)
    k4 = derivative(s + dt * k3)
    return s + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
