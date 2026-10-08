#!/usr/bin/env python3
"""
This AI4R policy node:
> Reads observations from the various sensors,
> Calculates actions to take,
> Publishes those actions so that that get implemented on the actual car.
> Does all that with one executor thread.

This node is provided a single monotlithic python file, which means that it
contains lots of lines of code for the real-time ROS details of:
> Getting observations into a format that is easy for the policy to use.
> Getting the policies action into a format that can be passed to the actaul car.
> A state machine to mangage the behaviour of the node.
> Error checking and handling.
You do NOT need to worry about those detail to edit and use this node.

You SHOULD:
> Start reading the code in this file at the calculate_policy_actions() function.
> You should especially focus on reading the block of comments marked as:
  > EXPLANATION OF THE OBSERVATIONS
  > EXPLANATION OF THE ACTIONS
> Implements your policy code between the INSERT POLICY CODE markers.
> NEVER do any of the following in your policy code because they will block the
  real-time loop and crash the car:
  > NEVER sleep!
  > NEVER wait for input!
  > NEVER run an unbounded computation loop!
  > NEVER do a ROS spin!
> Also read the YAML files in the "config" folder. Those files allow you to change
  key values within the policy and across the greater system without editing the
  code of the respective nodes. For YAML parameter file changes to take effect,
  you need to restart the respective nodes.

The nominal operator sequence for running this policy on the actual car is:
  1. Start this policy_node (on startup it continually publishes zero drive and
     zero steering).
  2. Request vehicle Enable for the Traxxas node and wait for its Enabled status
     (the zero commands from this policy_node are required to satisfy the Traxxas
     node's pre-enable neutral-command guard).
  3. Run your policy code by requesting policy state 3 (for example by clicking the
     respective button in Foxglove).
  4. Stop your policy code by requesting policy state 2, which publishes zeros.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
from numbers import Real
import time
import traceback

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, Int8, String, UInt16
from dream_interfaces.msg import (ConeDetection, ConeDetections, DriveAndSteer,
                                  FiducialDetections)
from tf2_ros import Buffer, TransformException, TransformListener


# These values and the request/status topics match the previous AI4R policy.
FSM_STATE_NOT_PUBLISHING_ACTIONS = 1
FSM_STATE_PUBLISHING_ZERO_ACTIONS = 2
FSM_STATE_PUBLISHING_POLICY_ACTION = 3
STATE_NAMES = {1: "Not publishing any actions", 2: "Publishing zero actions",
               3: "Publishing policy actions"}
SENSORS = ("cone_detections", "fiducial_detections", "lidar_scan", "lidar_cartesian", "wheel_speed", "imu_orientation",
           "imu_angular_velocity", "imu_specific_force")
STAMPED_SENSORS = tuple(name for name in SENSORS if name != "wheel_speed")
FIDUCIAL_DICTIONARY_SIZES = {
    **{f"DICT_{bits}X{bits}_{count}": count
       for bits in (4, 5, 6, 7) for count in (50, 100, 250, 1000)},
    "DICT_ARUCO_ORIGINAL": 1024,
}


@dataclass
class Observation:
    """One accepted measurement, with independent receipt and sample times."""
    value: object
    received_at: float             # monotonic seconds, not a ROS timestamp
    stamp_ns: int | None = None     # wheel-speed messages have no header
    received_ros_ns: int | None = None  # receipt proxy, NOT wheel acquisition time


def finite_number(value):
    """Reject booleans/strings/NaN/infinity instead of turning them into actions."""
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def valid_frame(frame):
    return bool(frame) and not frame.startswith("/") and not any(c.isspace() for c in frame)


def quaternion_xyzw(quaternion):
    """ROS quaternion order is (x, y, z, w), with identity (0, 0, 0, 1)."""
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(finite_number(v) for v in values):
        raise ValueError("quaternion contains a nonfinite value")
    norm = math.hypot(*values)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion has no usable orientation")
    return tuple(v / norm for v in values)


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def quaternion_inverse(q):
    # Inputs have already been normalized, so conjugation is the inverse.
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(q, vector):
    values = (vector.x, vector.y, vector.z)
    if not all(finite_number(v) for v in values):
        raise ValueError("IMU vector contains a nonfinite value")
    rotated = quaternion_product(quaternion_product(q, (*values, 0.0)), quaternion_inverse(q))
    if not all(finite_number(v) for v in rotated[:3]):
        raise ValueError("rotated IMU vector is not finite")
    return rotated[:3]


def transform_position(translation, rotation, position):
    values = (position.x, position.y, position.z)
    offsets = (translation.x, translation.y, translation.z)
    if not all(finite_number(v) for v in values + offsets):
        raise ValueError("pose position or TF translation contains a nonfinite value")
    rotated = quaternion_product(
        quaternion_product(rotation, (*values, 0.0)), quaternion_inverse(rotation))
    result = tuple(rotated[i] + offsets[i] for i in range(3))
    if not all(finite_number(v) for v in result):
        raise ValueError("transformed pose position is not finite")
    return result


def roll_pitch_yaw(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))),
            math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class EstimationSettings:
    """Prototype tuning, not a calibration or permission to drive.

    Course framework contracts take priority; the Word meeting draft only
    supplies additional internal EstimatedState/RoadState records.
    Screenshot names v_mps/yaw_rate_radps are additive aliases. Distances are
    metres, angles radians, and all output stamps use the node's ROS clock.
    Parameters live in the EXISTING ai4r_policy.yaml under estimation.*.
    """
    speed_tau_s: float = 0.15
    yaw_rate_tau_s: float = 0.08
    max_source_age_s: float = 0.5
    motion_max_gap_s: float = 0.15
    motion_max_interval_s: float = 0.3
    min_cone_confidence: float = 0.5
    road_forward_max_m: float = 3.0
    road_min_coverage_m: float = 0.5
    road_sample_spacing_m: float = 0.1
    road_max_gap_m: float = 1.5
    huber_delta_m: float = 0.08
    road_width_min_m: float = 0.4
    road_width_max_m: float = 2.0
    road_max_abs_slope: float = 1.0
    # 0 means unknown: single-sided geometry stays invalid until measured or
    # explicitly supplied for a known simulated road. Never guess a lane width.
    known_lane_width_m: float = 0.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not finite_number(value):
                raise ValueError(f"estimation.{name} must be finite")
            if name == "min_cone_confidence":
                if not 0.0 <= value <= 1.0:
                    raise ValueError("min_cone_confidence must be in [0,1]")
            elif name == "known_lane_width_m":
                if value != 0.0 and not self.road_width_min_m <= value <= self.road_width_max_m:
                    raise ValueError("known width must be 0 (unknown) or inside width bounds")
            elif value <= 0.0:
                raise ValueError(f"estimation.{name} must be positive")
        if self.road_width_min_m >= self.road_width_max_m:
            raise ValueError("road width bounds must increase")
        if self.road_min_coverage_m > self.road_forward_max_m:
            raise ValueError("road coverage exceeds the forward range")
        if self.road_sample_spacing_m > self.road_min_coverage_m:
            raise ValueError("road spacing exceeds minimum coverage")
        if self.road_forward_max_m / self.road_sample_spacing_m > 200:
            raise ValueError("road sampling is limited to 201 points")
        if max(self.motion_max_gap_s, self.motion_max_interval_s) > self.max_source_age_s:
            raise ValueError("motion timing limits cannot exceed the source-age limit")


class FirstOrderSampleFilter:
    """Causal low-pass; a cached sample is assimilated exactly once.

    sample_key identifies an accepted source message, not its numerical value.
    sample_time_s is ROS sample time for gyro, monotonic receipt for wheel speed;
    each filter stays in its own time domain. Source freshness is checked by the
    caller. New runs and discontinuities reset rather than bridge stale history.
    """
    def __init__(self, tau_s, reset_gap_s):
        self.tau_s = tau_s
        self.reset_gap_s = reset_gap_s
        self.clear()

    def clear(self):
        self.value = self.sample_key = self.sample_time_s = None

    def update(self, value, sample_key, sample_time_s):
        if not finite_number(value) or sample_key is None or not finite_number(sample_time_s):
            self.clear()
            return None
        if sample_key == self.sample_key:
            return self.value
        elapsed = None if self.sample_time_s is None else sample_time_s - self.sample_time_s
        if self.value is None or elapsed is None or elapsed <= 0 or elapsed >= self.reset_gap_s:
            self.value = float(value)
        else:
            alpha = -math.expm1(-elapsed / self.tau_s)
            self.value += alpha * (value - self.value)
        self.sample_key, self.sample_time_s = sample_key, sample_time_s
        return self.value


class EstimationMotionHistory:
    """Bounded, causal filtered wheel/gyro history on the ROS clock.

    Wheel time is ROS receipt (its message has no acquisition stamp). Motion
    assumes FORWARD, no-slip planar driving; unsigned wheel speed cannot support
    reverse compensation. Never substitute missing history with current speed.
    """
    def __init__(self, settings):
        self.settings = settings
        self.filters = {
            "wheel_speed": FirstOrderSampleFilter(settings.speed_tau_s, settings.max_source_age_s),
            "imu_angular_velocity": FirstOrderSampleFilter(settings.yaw_rate_tau_s, settings.max_source_age_s)}
        self.clear()

    def clear(self):
        self.samples = {name: [] for name in self.filters}
        for filter_ in self.filters.values():
            filter_.clear()
        self.last_ros_now = None

    def check_clock(self, now_s):
        if not finite_number(now_s):
            self.clear()
            return False
        if self.last_ros_now is not None and now_s < self.last_ros_now:
            self.clear()
        self.last_ros_now = now_s
        return True

    def clear_source(self, name):
        self.samples[name] = []
        self.filters[name].clear()

    def observe(self, name, value, key, sample_time_s, ros_time_s):
        filter_ = self.filters[name]
        if (not finite_number(value) or not finite_number(ros_time_s)
                or (name == "wheel_speed" and value < 0)):
            self.clear_source(name)
            return
        if key == filter_.sample_key and key is not None:
            return
        if self.samples[name] and ros_time_s <= self.samples[name][-1][0]:
            self.clear_source(name)
        filtered = filter_.update(value, key, sample_time_s)
        if not finite_number(filtered):
            self.clear_source(name)
            return
        samples = self.samples[name]
        samples.append((ros_time_s, filtered))
        # Bounded memory, retaining enough history for the admitted interval.
        cutoff = ros_time_s - 2*self.settings.max_source_age_s
        while len(samples) > 2 and samples[1][0] < cutoff:
            samples.pop(0)
        del samples[:-512]

    def sample_at(self, name, time_s):
        return next((item for item in reversed(self.samples[name]) if item[0] <= time_s), None)

    def current(self, name, now_s):
        sample = self.sample_at(name, now_s)
        return (None if sample is None or now_s-sample[0] > self.settings.motion_max_gap_s
                else sample[1])

    def pose_between(self, start_s, end_s):
        if not all(finite_number(v) for v in (start_s, end_s)) or end_s < start_s:
            return None, "invalid_motion_time"
        if end_s-start_s > self.settings.motion_max_interval_s + 1e-9:
            return None, "motion_interval_too_long"
        x = y = yaw = 0.0
        knots = sorted({start_s, end_s} | {
            t for samples in self.samples.values() for t, _ in samples if start_s < t < end_s})
        indices = {name: -1 for name in self.samples}
        for begin, end in zip(knots, knots[1:]):
            held = {}
            for name, samples in self.samples.items():
                index = indices[name]
                while index+1 < len(samples) and samples[index+1][0] <= begin:
                    index += 1
                indices[name] = index
                held[name] = samples[index] if index >= 0 else None
            speed, rate = held["wheel_speed"], held["imu_angular_velocity"]
            if speed is None or rate is None:
                return None, "missing_motion_history"
            if max(end-speed[0], end-rate[0]) > self.settings.motion_max_gap_s + 1e-9:
                return None, "motion_history_gap"
            dt = end-begin
            angle, distance = rate[1]*dt, speed[1]*dt
            if not all(math.isfinite(v) for v in (angle, distance)):
                return None, "nonfinite_motion"
            if abs(angle) < 1e-6:
                dx = distance*(1-angle*angle/6)
                dy = distance*(angle/2-angle**3/24)
            else:
                dx, dy = distance*math.sin(angle)/angle, distance*(1-math.cos(angle))/angle
            c, s = math.cos(yaw), math.sin(yaw)
            x, y, yaw = x+c*dx-s*dy, y+s*dx+c*dy, yaw+angle
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                return None, "nonfinite_motion"
        return {"dx_m": x, "dy_m": y, "yaw_rad": yaw, "interval_s": end_s-start_s}, None

    @staticmethod
    def transform_xy(point, pose):
        x, y = point[0]-pose["dx_m"], point[1]-pose["dy_m"]
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        return (c*x+s*y, -s*x+c*y)


def _estimation_median(values):
    ordered = sorted(values)
    n = len(ordered)
    return ordered[n // 2] if n % 2 else (ordered[n // 2 - 1] + ordered[n // 2]) / 2.0


def _estimation_solve(matrix, rhs):
    """Pivoted solve of a maximum 3x3 scaled normal system; no extra runtime dependency."""
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    n = len(rows)
    for i in range(n):
        pivot = max(range(i, n), key=lambda j: abs(rows[j][i]))
        rows[i], rows[pivot] = rows[pivot], rows[i]
        divisor = rows[i][i]
        if abs(divisor) < 1e-10 or not math.isfinite(divisor):
            return None
        rows[i] = [v / divisor for v in rows[i]]
        for j in range(n):
            if j != i:
                factor = rows[j][i]
                rows[j] = [a - factor*b for a, b in zip(rows[j], rows[i])]
    result = [row[-1] for row in rows]
    return result if all(math.isfinite(v) for v in result) else None


def _estimation_fit_boundary(points, settings):
    """Finite, bounded Huber IRLS followed by an inlier refit.

    Fits y(x) only for modest local slopes. Ordered points, measured support and
    gaps determine validity. This reconstructs a road, not a driving reference.
    """
    points = sorted(points)
    # Keep bounded work, retaining points across the visible range.
    if len(points) > 64:
        points = [points[round(i*(len(points)-1)/63)] for i in range(64)]
    if len(points) < 2:
        return None
    x0, x1 = points[0][0], points[-1][0]
    if x1 - x0 < settings.road_min_coverage_m:
        return None
    origin, scale = (x0+x1)/2.0, (x1-x0)/2.0
    degree = 2 if len(points) >= 5 and len({p[0] for p in points}) >= 3 else 1
    basis = [[((p[0]-origin)/scale)**j for j in range(degree+1)] for p in points]
    slopes = [(b[1]-a[1])/(b[0]-a[0]) for i, a in enumerate(points)
              for b in points[i+1:] if b[0]-a[0] > 1e-6]
    if not slopes:
        return None
    slope = _estimation_median(slopes)
    intercept = _estimation_median([p[1]-slope*p[0] for p in points])
    coeffs = [intercept+slope*origin, slope*scale] + ([0.0] if degree == 2 else [])
    residuals = []
    for _ in range(8):
        residuals = [sum(c*b for c, b in zip(coeffs, row))-p[1]
                     for row, p in zip(basis, points)]
        weights = [p[2]*min(1.0, settings.huber_delta_m/max(abs(r), 1e-12))
                   for p, r in zip(points, residuals)]
        matrix = [[sum(w*row[i]*row[j] for w, row in zip(weights, basis))
                   for j in range(degree+1)] for i in range(degree+1)]
        rhs = [sum(w*row[i]*p[1] for w, row, p in zip(weights, basis, points))
               for i in range(degree+1)]
        coeffs = _estimation_solve(matrix, rhs)
        if coeffs is None:
            return None
    residuals = [sum(c*b for c, b in zip(coeffs, row))-p[1] for row, p in zip(basis, points)]
    inliers = [i for i, r in enumerate(residuals) if abs(r) <= 2*settings.huber_delta_m]
    if len(inliers) < degree+1 or len(inliers) < 0.6*len(points):
        return None
    # Restrict support to a near contiguous inlier segment; never bridge a large gap.
    segment = []
    for i in inliers:
        if segment and points[i][0]-points[segment[-1]][0] > settings.road_max_gap_m:
            break
        segment.append(i)
    if len(segment) < degree+1:
        return None
    inliers = segment
    matrix = [[sum(points[k][2]*basis[k][i]*basis[k][j] for k in inliers)
               for j in range(degree+1)] for i in range(degree+1)]
    rhs = [sum(points[k][2]*basis[k][i]*points[k][1] for k in inliers)
           for i in range(degree+1)]
    coeffs = _estimation_solve(matrix, rhs)
    if coeffs is None:
        return None
    lo, hi = points[inliers[0]][0], points[inliers[-1]][0]
    if hi-lo < settings.road_min_coverage_m:
        return None
    c2 = coeffs[2] if degree == 2 else 0.0
    a2 = c2/(scale*scale)
    a1 = coeffs[1]/scale - 2*a2*origin
    a0 = coeffs[0] - coeffs[1]*origin/scale + a2*origin*origin
    if max(abs(a1+2*a2*x) for x in (lo, hi)) > settings.road_max_abs_slope:
        return None
    rms = math.sqrt(sum((a0+a1*points[i][0]+a2*points[i][0]**2-points[i][1])**2
                        for i in inliers)/len(inliers))
    if rms > settings.huber_delta_m:
        return None
    return {"coeffs": (a0, a1, a2), "range": (lo, hi), "rms_m": rms,
            "inlier_fraction": len(inliers)/len(points)}


class EstimationPipeline:
    """Local geometry aligned to the state epoch with bounded planar odometry.

    Draft names remain canonical; aliases support the supplied screenshots.
    Raw road() reconstructs acquisition-frame geometry. update() transports it
    to the current body frame, or rejects it if historical motion is missing.
    This is approximate forward odometry, not SLAM or calibrated localization.
    """
    def __init__(self, settings, frame_id="base_link", left_colour=2, right_colour=1,
                 motion_history=None):
        self.settings, self.frame_id = settings, frame_id
        self.left_colour, self.right_colour = left_colour, right_colour
        self.motion = motion_history or EstimationMotionHistory(settings)

    def _fresh(self, value, age):
        return value is not None and finite_number(age) and 0 <= age < self.settings.max_source_age_s

    def road(self, batch, stamp_ns, age):
        timestamp = stamp_ns/1e9 if stamp_ns is not None and stamp_ns > 0 else None
        output = {"timestamp_s": timestamp, "frame_id": self.frame_id, "valid": False,
                  "centerline_xy": [], "left_boundary_xy": None, "right_boundary_xy": None,
                  "lane_width_m": None, "local_curvature_1pm": None, "x_range_m": None,
                  "confidence": 0.0, "visibility": "none", "status": "unavailable",
                  "source_age_s": age, "motion_compensated": False,
                  "boundary_source": {"left": "absent", "right": "absent"}}
        if not self._fresh(batch, age) or timestamp is None:
            return output
        sides = {self.left_colour: [], self.right_colour: []}
        for x, y, z, colour, confidence in batch["detections"]:
            if (all(finite_number(v) for v in (x, y, z, confidence))
                    and 0 <= x <= self.settings.road_forward_max_m
                    and abs(y) <= self.settings.road_width_max_m + self.settings.road_forward_max_m*self.settings.road_max_abs_slope
                    and self.settings.min_cone_confidence <= confidence <= 1.0
                    and confidence > 0 and colour in sides):
                sides[colour].append((x, y, confidence))
        left = _estimation_fit_boundary(sides[self.left_colour], self.settings)
        right = _estimation_fit_boundary(sides[self.right_colour], self.settings)
        output["status"] = "insufficient_geometry"
        if left is None and right is None:
            return output
        if left is None or right is None:
            output["visibility"] = "left_only" if left is not None else "right_only"
            if self.settings.known_lane_width_m == 0:
                output["status"] = "single_side_width_unknown"
                return output
        fits = [fit for fit in (left, right) if fit is not None]
        lo, hi = max(f["range"][0] for f in fits), min(f["range"][1] for f in fits)
        if hi-lo < self.settings.road_min_coverage_m:
            output["status"] = "insufficient_common_range"
            return output
        count = min(200, math.ceil((hi-lo)/self.settings.road_sample_spacing_m))
        center, widths, samples = [], [], {"left": [], "right": []}
        for i in range(count+1):
            x = lo+(hi-lo)*i/count
            evaluated = []
            for side, fit in (("left", left), ("right", right)):
                if fit is None:
                    evaluated.append(None)
                    continue
                a0, a1, a2 = fit["coeffs"]
                y, slope = a0+a1*x+a2*x*x, a1+2*a2*x
                samples[side].append((x, y))
                evaluated.append((y, slope))
            if all(v is not None for v in evaluated):
                (yl, sl), (yr, sr) = evaluated
                slope = (sl+sr)/2
                width = (yl-yr)/math.hypot(1, slope)
                center.append((x, (yl+yr)/2))
            else:
                y, slope = next(v for v in evaluated if v is not None)
                width = self.settings.known_lane_width_m
                # Offset along the local normal, not simply along body y.
                direction = -1 if left is not None else 1
                half = direction*width/2/math.hypot(1, slope)
                center.append((x-half*slope, y+half))
            widths.append(width)
        if (any(not self.settings.road_width_min_m <= w <= self.settings.road_width_max_m for w in widths)
                or any(b[0] <= a[0] for a, b in zip(center, center[1:]))
                or center[0][0] < 0
                or center[-1][0]-center[0][0] < self.settings.road_min_coverage_m):
            output["status"] = "invalid_width_or_order"
            return output
        output.update(valid=True, centerline_xy=center, lane_width_m=_estimation_median(widths),
                      x_range_m=[center[0][0], center[-1][0]],
                      confidence=min(f["inlier_fraction"] for f in fits) * (1.0 if len(fits) == 2 else 0.5),
                      visibility="both" if len(fits) == 2 else output["visibility"],
                      status="observed" if len(fits) == 2 else "single_side_inferred",
                      fit_rms_m=max(f["rms_m"] for f in fits))
        for side, fit in (("left", left), ("right", right)):
            output[f"{side}_boundary_xy"] = samples[side] if fit is not None else None
            output["boundary_source"][side] = "observed" if fit is not None else "absent"
        if len(fits) == 2:
            a1 = (left["coeffs"][1]+right["coeffs"][1])/2
            a2 = (left["coeffs"][2]+right["coeffs"][2])/2
            slope = a1+2*a2*lo
            output["local_curvature_1pm"] = 2*a2/(1+slope*slope)**1.5
        # For an offset curve, do not publish its boundary curvature as the
        # centreline's curvature. None is honest until that derivation is added.
        return output

    def _align_road(self, road, now_s, state_valid):
        source_time = road["timestamp_s"]
        road.update(measurement_timestamp_s=source_time, timestamp_s=None,
                    time_aligned=False, motion_compensation=None)
        if not road["valid"]:
            return road
        pose, problem = self.motion.pose_between(source_time, now_s)
        if not state_valid:
            problem = "motion_state_unavailable"
        if problem is None:
            points = [self.motion.transform_xy(p, pose) for p in road["centerline_xy"]]
            selected = [i for i, p in enumerate(points)
                        if all(math.isfinite(v) for v in p)
                        and 0 <= p[0] <= self.settings.road_forward_max_m]
            center = [points[i] for i in selected]
            if (any(not all(math.isfinite(v) for v in p) for p in points)
                    or len(center) < 2
                    or center[-1][0]-center[0][0] < self.settings.road_min_coverage_m
                    or any(b[0] <= a[0] or abs(b[1]-a[1]) > self.settings.road_max_abs_slope*(b[0]-a[0])
                           for a, b in zip(center, center[1:]))):
                problem = "motion_transformed_geometry_invalid"
            else:
                road["centerline_xy"] = center
                road["x_range_m"] = [center[0][0], center[-1][0]]
                for side in ("left", "right"):
                    key = f"{side}_boundary_xy"
                    if road[key] is not None:
                        boundary = [self.motion.transform_xy(p, pose) for p in road[key]]
                        road[key] = [p for p in boundary if 0 <= p[0] <= self.settings.road_forward_max_m]
                        if any(not all(math.isfinite(v) for v in p) for p in boundary):
                            problem = "motion_transformed_geometry_invalid"
                # Rigid motion preserves curvature at the SAME physical point.
                # If clipping removes that first point, do not relabel its value.
                if selected[0] != 0:
                    road["local_curvature_1pm"] = None
                road.update(timestamp_s=now_s, time_aligned=True,
                            motion_compensated=pose["interval_s"] > 0,
                            motion_compensation=pose)
        if problem is not None:
            road.update(valid=False, status=problem, timestamp_s=None,
                        time_aligned=False, motion_compensated=False,
                        centerline_xy=[], left_boundary_xy=None, right_boundary_xy=None,
                        lane_width_m=None, x_range_m=None, local_curvature_1pm=None,
                        confidence=0.0, motion_compensation=None)
        return road

    def update(self, observations, ages, stamps, sample_receipts, now_s,
               sample_receipt_ros_ns=None):
        clock_valid = self.motion.check_clock(now_s)
        receipt_stamps = sample_receipt_ros_ns or {}
        for name, component in (("wheel_speed", None), ("imu_angular_velocity", 2)):
            value = observations.get(name)
            if not clock_valid or not self._fresh(value, ages.get(name)):
                self.motion.clear_source(name)
                continue
            if component is not None:
                value = value[component]
            stamp = stamps.get(name)
            if name == "imu_angular_velocity" and (stamp is None or stamp <= 0):
                self.motion.clear_source(name)
                continue
            key = stamp if stamp is not None else sample_receipts.get(name)
            sample_time = stamp/1e9 if stamp is not None else sample_receipts.get(name)
            # Production uses the ROS receipt saved by _store; standalone
            # numerical snapshots can map receipt age onto their supplied clock.
            receipt = receipt_stamps.get(name)
            ros_time = (stamp/1e9 if stamp is not None else
                        receipt/1e9 if receipt is not None else now_s-ages[name])
            self.motion.observe(name, value, key, sample_time, ros_time)

        speed = self.motion.current("wheel_speed", now_s) if clock_valid else None
        yaw_rate = self.motion.current("imu_angular_velocity", now_s) if clock_valid else None
        state = {"timestamp_s": now_s, "frame_id": self.frame_id,
                 "speed_mps": speed, "yaw_rate_rps": yaw_rate,
                 "v_mps": speed, "yaw_rate_radps": yaw_rate,
                 "speed_valid": speed is not None, "yaw_rate_valid": yaw_rate is not None,
                 "valid": speed is not None and yaw_rate is not None,
                 "confidence": 1.0 if speed is not None and yaw_rate is not None else 0.0,
                 "lateral_error_m": None, "heading_error_rad": None,
                 "road_relative_valid": False, "mode": "causal_filtered_hold",
                 "sample_time_basis": {"wheel_speed": "ros_receipt_proxy",
                                       "imu_angular_velocity": "ros_acquisition"},
                 "source_age_s": {k: ages.get(k) for k in ("wheel_speed", "imu_angular_velocity")},
                 "source_stamp_ns": {k: stamps.get(k) for k in ("wheel_speed", "imu_angular_velocity")}}
        road = self.road(observations.get("cone_detections"), stamps.get("cone_detections"),
                         ages.get("cone_detections"))
        road = self._align_road(road, now_s, state["valid"])
        lidar = observations.get("lidar_cartesian")
        lidar_stamp = stamps.get("lidar_cartesian")
        available = self._fresh(lidar, ages.get("lidar_cartesian")) and lidar_stamp is not None
        source_time = lidar_stamp/1e9 if available else None
        pose, problem = (self.motion.pose_between(source_time, now_s) if available
                         else (None, "unavailable"))
        available = available and state["valid"] and problem is None
        points = []
        if available:
            for point in lidar["points_xyz"]:
                x, y = self.motion.transform_xy(point, pose)
                if not all(finite_number(v) for v in (x, y, point[2])):
                    available, points = False, []
                    break
                points.append((x, y, point[2]))
        obstacle = {"timestamp_s": now_s if available else None,
                    "measurement_timestamp_s": source_time,
                    "frame_id": self.frame_id, "available": available,
                    "source_age_s": ages.get("lidar_cartesian"),
                    "points_xyz_m": points,
                    "scan_indices": list(lidar["scan_indices"]) if available else [],
                    "time_aligned": available,
                    "motion_compensated": available and pose["interval_s"] > 0,
                    "motion_compensation": pose if available else None,
                    "deskewed": False}
        # Physical calibration is not present in the project. Expose missing
        # values rather than copy simulation parameters into real-car limits.
        params = {"valid": False, "source": "unmeasured", "effective_wheelbase_m": None,
                  "steering_offset": None, "steering_map": None, "steering_limit": None,
                  "drive_response": None, "actuation_delay_s": None}
        limits = {"valid": False, "source": "unmeasured", "footprint_xy_m": None,
                  "wheelbase_m": None, "rear_axle_x_m": None, "curvature_limit_1pm": None,
                  "speed_max_mps": None, "acceleration_max_mps2": None,
                  "braking_deceleration_mps2": None, "safety_margin_m": None}
        alignment = {"valid": state["valid"] and road["valid"] and road["time_aligned"],
                     "timestamp_s": now_s,
                     "method": "forward_planar_filtered_odometry",
                     "wheel_time_basis": "ros_receipt_proxy",
                     "wheel_acquisition_time_known": False,
                     "assumptions": ["forward_motion", "planar_no_slip", "causal_filtered_hold"]}
        return {"state": state, "road": road, "obstacles": obstacle, "alignment": alignment,
                "vehicle_params": params, "vehicle_limits": limits}


@dataclass
class PlanningSettings:
    """V1 straight-road operating domain; thresholds are engineering settings.

    No dimensions or braking capability are guessed. Required vehicle_limits
    must be supplied by calibration or an explicitly labelled offline fixture.
    Constant-twist compensation is opt-in and is a short-time approximation.
    """
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
            if name == "vehicle_limits_source":
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
    # Per explicit policy run, wheel-speed odometry (metres), not road lookahead.
    max_distance_m: float = 3.0
    max_run_time_s: float = 30.0
    # Direct normalized steering for the first-run MVP, no wheel-angle model.
    mvp_lateral_kp: float = 1.0
    mvp_lateral_ki: float = 0.0
    mvp_heading_kp: float = 0.5
    mvp_steering_direction: float = 1.0
    # Measured normalized effort needed to sustain the selected crawl speed.
    # Optional and zero by default; stop/invalid paths bypass this compensation.
    mvp_drive_feedforward: float = 0.0

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("control.enabled must be bool")
        if self.mode not in ("mvp", "calibrated"):
            raise ValueError("control.mode must be mvp or calibrated")
        if self.vehicle_params_source not in ("upstream", "course_simulation"):
            raise ValueError("control.vehicle_params_source must be upstream or course_simulation")
        for name in ("speed_kp", "speed_ki", "speed_kd", "lateral_kp", "lateral_ki",
                     "lateral_kd", "heading_kp", "mvp_lateral_kp", "mvp_lateral_ki", "mvp_heading_kp"):
            if not finite_number(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"control.{name} must be finite and nonnegative")
        for name in ("derivative_tau_s", "drive_max", "steering_max_normalized",
                     "max_dt_s", "startup_grace_s", "max_distance_m", "max_run_time_s"):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"control.{name} must be finite and positive")
        if self.drive_max > 1 or self.steering_max_normalized > 1:
            raise ValueError("control action limits must not exceed 1")
        if not finite_number(self.mvp_steering_direction) or self.mvp_steering_direction not in (-1.0, 1.0):
            raise ValueError("control.mvp_steering_direction must be -1.0 or 1.0")
        if (not finite_number(self.mvp_drive_feedforward)
                or not 0 <= self.mvp_drive_feedforward <= self.drive_max):
            raise ValueError("control.mvp_drive_feedforward must be within [0, drive_max]")


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
        if self.distance_m+1e-9 >= self.maximum_m:
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
                # Direct normalized PI + heading P, no degree-to-servo map or
                # curvature feedforward borrowed from the bicycle simulation.
                steer = self.mvp_lateral.update(geometry["path_error_m"], -geometry["path_error_m"], dt,
                    -cfg.steering_max_normalized, cfg.steering_max_normalized,
                    cfg.mvp_heading_kp*heading)
                drive = self.speed.update(target-speed, speed, dt, 0.0, cfg.drive_max,
                                          feedforward=cfg.mvp_drive_feedforward)
                return drive, cfg.mvp_steering_direction*steer, {
                    **geometry, "valid": True, "stop_requested": False, "reason": "mvp_tracking",
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


class PolicyNode(Node):
    def __init__(self, **kwargs):
        super().__init__("ai4r_policy", **kwargs)

        # All supplied policy settings are startup-only. Restart this node after
        # editing its YAML; it always restarts in the zero-action state.
        defaults = {
            "policy_update_mode": "cone_detection",
            "policy_update_rate_hz": 50.0,
            "required_sensors": ["cone_detections"],
            "cone_empty_timeout_s": 1.0,
            "supervision_rate_hz": 20.0,
            "status_period_s": 0.5,
            "timestamp_tolerance_s": 0.05,
            "policy_frame_id": "base_link",
        }
        defaults.update({f"sensor_timeout_s.{name}": 0.5 for name in SENSORS})
        for name, default in defaults.items():
            # YAML cannot infer a string-array type from []. Allow that startup
            # representation, then check its contents explicitly below.
            self.declare_parameter(name, default, ParameterDescriptor(
                read_only=True, dynamic_typing=(name == "required_sensors")))

        self.policy_update_mode = self.get_parameter("policy_update_mode").value
        self.policy_update_rate_hz = self.get_parameter("policy_update_rate_hz").value
        required_parameter = self.get_parameter("required_sensors")
        # Jazzy's C YAML parser represents an explicit [] as NOT_SET, while a
        # Python parameter override can carry a typed empty string array. An
        # omitted key still uses our declared default ["cone_detections"].
        self.required_sensors = ([] if required_parameter.type_ == Parameter.Type.NOT_SET
                                 else required_parameter.value)
        self.cone_empty_timeout_s = self.get_parameter("cone_empty_timeout_s").value
        self.supervision_rate_hz = self.get_parameter("supervision_rate_hz").value
        self.status_period_s = self.get_parameter("status_period_s").value
        self.timestamp_tolerance_s = self.get_parameter("timestamp_tolerance_s").value
        self.policy_frame_id = self.get_parameter("policy_frame_id").value
        self.sensor_timeout_s = {
            name: self.get_parameter(f"sensor_timeout_s.{name}").value for name in SENSORS}
        self._validate_parameters()
        self.add_on_set_parameters_callback(self._reject_clock_change)

        # TO ADD A STUDENT PARAMETER, follow these THREE steps:
        # 1. Declare it here, for example:
        # self.declare_parameter('speed_kp', 0.2, ParameterDescriptor(read_only=True))
        # 2. Add speed_kp: 0.2 under ros__parameters in ai4r_policy.yaml.
        # 3. Read it here, then use self.speed_kp in your policy below:
        # self.speed_kp = self.get_parameter('speed_kp').value
        # YAML alone does not declare a parameter. A value such as 0.2 is a
        # floating-point number; 0 is an integer, which is a different ROS type.

        estimation_defaults = EstimationSettings()
        estimation_values = {}
        for name, default in vars(estimation_defaults).items():
            parameter_name = f"estimation.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            estimation_values[name] = self.get_parameter(parameter_name).value
        self.estimation_settings = EstimationSettings(**estimation_values)
        self.estimation_output = None
        self.motion_history = EstimationMotionHistory(self.estimation_settings)

        planning_values = {}
        for name, default in vars(PlanningSettings()).items():
            parameter_name = f"planning.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            planning_values[name] = self.get_parameter(parameter_name).value
        self.planning_settings = PlanningSettings(**planning_values)
        self.planning_output = None
        self.planning_diagnostics = None

        for prefix, settings_type in (("control", ControlSettings), ("vehicle", VehicleCalibrationSettings)):
            values = {}
            for name, default in vars(settings_type()).items():
                parameter_name = f"{prefix}.{name}"
                self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
                values[name] = self.get_parameter(parameter_name).value
            setattr(self, f"{prefix}_settings", settings_type(**values))
        if self.control_settings.enabled:
            if not {"cone_detections", "wheel_speed", "imu_angular_velocity"}.issubset(self.required_sensors):
                raise ValueError("Integrated control requires cones, wheel_speed and imu_angular_velocity")
            simulation_planning = self.planning_settings.vehicle_limits_source == "course_simulation"
            simulation_control = self.control_settings.vehicle_params_source == "course_simulation"
            if self.control_settings.mode == "calibrated" and simulation_planning != simulation_control:
                raise ValueError("Planning and control simulation profiles must be selected together")
            if (self.policy_update_mode == "timer" and
                    1.0/self.policy_update_rate_hz >= self.planning_settings.reference_lifetime_s):
                raise ValueError("Integrated control timer period must be shorter than the planning reference lifetime")
        self.controller = PolicyController(self.control_settings, self.policy_frame_id)
        self.distance_limiter = RunDistanceLimiter(self.control_settings.max_distance_m, self.control_settings.max_dt_s)
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False

        self.fsm_state = FSM_STATE_PUBLISHING_ZERO_ACTIONS
        self.state_reason = "Startup: waiting for an explicit policy request"
        self.observations = {name: None for name in SENSORS}
        self.last_nonempty_cones = None
        self.heading_reference = None
        self.policy_started_at = None
        self.previous_policy_step_at = None
        self._monotonic = time.monotonic
        self._last_ros_time_ns = None
        self._last_status_at = -math.inf
        self._last_warning_at = {}

        reliable_one = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        sensor_one = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.action_publisher = self.create_publisher(
            DriveAndSteer, "drive_and_steer_set_point_normalized", reliable_one)
        self.pan_publisher = self.create_publisher(Float32, "pan_set_point_normalized", reliable_one)
        self.state_publisher = self.create_publisher(Int8, "policy_fsm_state_value", 10)
        self.state_text_publisher = self.create_publisher(String, "policy_fsm_state_string", 10)
        self.heading_publisher = self.create_publisher(Float32, "imu_heading_angle", 10)
        self.debug1_publisher = self.create_publisher(Float32, "debug1", 10)
        self.debug2_publisher = self.create_publisher(Float32, "debug2", 10)

        self.create_subscription(ConeDetections, "cone_detections",
                                 self.cone_detection_callback, reliable_one)
        self.create_subscription(FiducialDetections, "fiducial_detections",
                                 self.fiducial_detection_callback, reliable_one)
        self.create_subscription(LaserScan, "scan", self.lidar_callback, sensor_one)
        self.create_subscription(Float32, "wheel_speed_m_per_sec",
                                 self.wheel_speed_callback, reliable_one)
        self.create_subscription(Imu, "imu/data", self.imu_callback, imu_qos)
        self.create_subscription(UInt16, "policy_fsm_transition_request",
                                 self.fsm_transition_request_callback, 10)

        # DREAM owns mounting TF, including the camera's fixed zero-pan pose
        # relative to ground-level base_link. Physical panning needs measured
        # dynamic TF; this policy only looks up transforms and does not
        # broadcast a competing tree.
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # These timers use a steady clock so a ROS clock adjustment cannot stop
        # supervision. Only timer mode creates a timer that runs student code.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.supervision_timer = self.create_timer(
            1.0 / self.supervision_rate_hz, self.supervision_callback, clock=self.steady_clock)
        self.policy_timer = None
        if self.policy_update_mode == "timer":
            self.policy_timer = self.create_timer(
                1.0 / self.policy_update_rate_hz, self.run_policy_step, clock=self.steady_clock)
        self.publish_zero_actions()
        self.publish_state()
        self.get_logger().info(
            f"Policy update source: {self.policy_update_mode}; required sensors: "
            f"{list(self.required_sensors)}; namespace: {self.get_namespace()}")

    def _validate_parameters(self):
        if self.policy_update_mode not in ("cone_detection", "fiducial_detection", "lidar", "timer"):
            raise ValueError(
                "policy_update_mode must be cone_detection, fiducial_detection, lidar, or timer")
        required = self.required_sensors
        if not isinstance(required, (list, tuple)) or not all(isinstance(s, str) for s in required):
            raise ValueError("required_sensors must be a list of sensor names")
        if len(set(required)) != len(required) or any(s not in SENSORS for s in required):
            raise ValueError(f"required_sensors must contain unique names from {SENSORS}")
        trigger = {"cone_detection": "cone_detections",
                   "fiducial_detection": "fiducial_detections"}.get(self.policy_update_mode)
        if trigger is not None and trigger not in required:
            raise ValueError(f"{self.policy_update_mode} mode requires {trigger} in required_sensors")
        if self.policy_update_mode == "lidar" and not {"lidar_scan", "lidar_cartesian"}.intersection(required):
            raise ValueError("lidar mode requires lidar_scan or lidar_cartesian in required_sensors")
        positive = {"policy_update_rate_hz": self.policy_update_rate_hz,
                    "cone_empty_timeout_s": self.cone_empty_timeout_s,
                    "supervision_rate_hz": self.supervision_rate_hz,
                    "status_period_s": self.status_period_s, **self.sensor_timeout_s}
        for name, value in positive.items():
            if not finite_number(value) or value <= 0.0 or not math.isfinite(1.0 / value):
                raise ValueError(f"{name} must be finite and positive")
        # ROS timers have nanosecond resolution: reject a period that truncates
        # to zero, rather than accidentally creating a continuously-ready timer.
        if max(self.policy_update_rate_hz, self.supervision_rate_hz) > 1e9:
            raise ValueError("timer frequencies cannot exceed nanosecond resolution")
        if not finite_number(self.timestamp_tolerance_s) or self.timestamp_tolerance_s < 0.0:
            raise ValueError("timestamp_tolerance_s must be finite and nonnegative")
        if not valid_frame(self.policy_frame_id):
            raise ValueError("policy_frame_id must be a nonempty frame without whitespace or leading /")
        if self.get_parameter("use_sim_time").value:
            raise ValueError("use_sim_time=true is unsupported for this live-robot policy")

    @staticmethod
    def _reject_clock_change(parameters):
        if any(p.name == "use_sim_time" for p in parameters):
            return SetParametersResult(successful=False, reason="Live policy clock is startup-only")
        return SetParametersResult(successful=True)

    # ---- Measurement acceptance and freshness --------------------------------
    def _times(self):
        monotonic_now = self._monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        self.motion_history.check_clock(ros_now_ns/1e9)
        if self._last_ros_time_ns is not None and ros_now_ns < self._last_ros_time_ns:
            for name in STAMPED_SENSORS:
                self.observations[name] = None
            self.last_nonempty_cones = None
            if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION
                    and any(s in STAMPED_SENSORS for s in self.required_sensors)):
                self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, "ROS clock moved backwards")
        self._last_ros_time_ns = ros_now_ns
        return monotonic_now, ros_now_ns

    @staticmethod
    def _stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    @staticmethod
    def _age(observation, monotonic_now, ros_now_ns):
        age = max(0.0, monotonic_now - observation.received_at)
        if observation.stamp_ns is not None:
            age = max(age, (ros_now_ns - observation.stamp_ns) / 1e9)
        return age

    def _store(self, name, value, stamp_ns, monotonic_now, ros_now_ns):
        if stamp_ns is not None:
            stamp_age = (ros_now_ns - stamp_ns) / 1e9
            previous = self.observations[name]
            if (stamp_ns <= 0 or stamp_age >= self.sensor_timeout_s[name]
                    or stamp_age < -self.timestamp_tolerance_s
                    or (previous is not None and stamp_ns <= previous.stamp_ns)):
                self._warn(name, f"Ignoring stale, future, repeated, or out-of-order {name} sample")
                return False
        self.observations[name] = Observation(value, monotonic_now, stamp_ns, ros_now_ns)
        # Record EVERY accepted wheel/gyro sample, including those between
        # camera triggers. Current snapshots cannot reconstruct past motion.
        if name in ("wheel_speed", "imu_angular_velocity"):
            scalar = value if name == "wheel_speed" else value[2]
            self.motion_history.observe(
                name, scalar, stamp_ns if stamp_ns is not None else monotonic_now,
                stamp_ns/1e9 if stamp_ns is not None else monotonic_now,
                stamp_ns/1e9 if stamp_ns is not None else ros_now_ns/1e9)
        return True

    def _fresh(self, name, monotonic_now, ros_now_ns):
        observation = self.observations[name]
        if name == "lidar_cartesian" and not self._fresh("lidar_scan", monotonic_now, ros_now_ns):
            return False  # Derived points cannot outlive their matching raw scan.
        return (observation is not None
                and self._age(observation, monotonic_now, ros_now_ns) < self.sensor_timeout_s[name])

    def health_problem(self, monotonic_now, ros_now_ns):
        for name in self.required_sensors:
            if not self._fresh(name, monotonic_now, ros_now_ns):
                return f"Required sensor {name} is missing or stale"
        if "cone_detections" in self.required_sensors:
            if (self.last_nonempty_cones is None or
                    self._age(self.last_nonempty_cones, monotonic_now, ros_now_ns)
                    >= self.cone_empty_timeout_s):
                return "No recent nonempty cone detections"
        return None

    def _warn(self, key, message):
        # One message per cause per three seconds, so a 50 Hz error does not
        # flood the terminal. State changes are reported immediately separately.
        now = self._monotonic()
        if now - self._last_warning_at.get(key, -math.inf) >= 3.0:
            self.get_logger().warning(message)
            self._last_warning_at[key] = now

    def cone_detection_callback(self, msg):
        now, ros_now = self._times()
        if msg.header.frame_id != self.policy_frame_id:
            self._warn("cone_frame", f"Cone frame must be {self.policy_frame_id}; got {msg.header.frame_id!r}")
            return
        latency = msg.acquisition_to_publish_latency_s
        if not finite_number(latency) or latency < 0.0:
            self._warn("cone_invalid", "Ignoring cone batch with invalid acquisition-to-publication latency")
            return
        # The detector already rejects bad depth. Validate the public message
        # at this boundary too, so malformed data does not refresh its watchdog.
        cones = []
        for cone in msg.detections:
            point = (cone.position.x, cone.position.y, cone.position.z)
            confidence = cone.classification_confidence
            if (not all(finite_number(v) for v in point)
                    or not finite_number(confidence) or not 0.0 <= confidence <= 1.0
                    or cone.color not in (ConeDetection.COLOR_YELLOW, ConeDetection.COLOR_BLUE)):
                self._warn("cone_invalid", "Ignoring cone batch with invalid position, colour, or confidence")
                return
            cones.append((*point, cone.color, confidence))
        value = {"detections": cones, "acquisition_to_publish_latency_s": float(latency)}
        if self._store("cone_detections", value, self._stamp_ns(msg.header), now, ros_now):
            if cones:
                self.last_nonempty_cones = self.observations["cone_detections"]
            if self.policy_update_mode == "cone_detection":
                self.run_policy_step()

    def fiducial_detection_callback(self, msg):
        now, ros_now = self._times()
        source_frame = msg.header.frame_id
        dictionary_name = msg.dictionary_name
        if not valid_frame(source_frame):
            self._warn("fiducial_frame", "Fiducial batch has no valid source frame")
            return
        if dictionary_name not in FIDUCIAL_DICTIONARY_SIZES:
            self._warn("fiducial_dictionary", "Fiducial batch has an invalid dictionary name")
            return
        dictionary_size = FIDUCIAL_DICTIONARY_SIZES[dictionary_name]

        # Validate the whole camera-frame batch before looking up one transform
        # at its acquisition time. A malformed member rejects the whole batch;
        # repeated IDs are retained because association belongs to the student.
        validated = []
        for detection in msg.detections:
            marker_id = detection.id
            marker_size = detection.marker_size_m
            pose_valid = detection.pose_valid
            corners = tuple((corner.u, corner.v) for corner in detection.corners)
            error = detection.reprojection_error_px
            if (not isinstance(marker_id, int) or isinstance(marker_id, bool)
                    or not 0 <= marker_id < dictionary_size
                    or not finite_number(marker_size) or marker_size <= 0.0
                    or len(corners) != 4
                    or not all(finite_number(value) for corner in corners for value in corner)
                    or not isinstance(pose_valid, bool)
                    or not isinstance(error, Real) or isinstance(error, bool)
                    or math.isinf(error) or (math.isfinite(error) and error < 0.0)):
                self._warn("fiducial_invalid", "Ignoring fiducial batch with invalid marker data")
                return
            position = detection.pose.position
            orientation = detection.pose.orientation
            if pose_valid:
                try:
                    camera_orientation = quaternion_xyzw(orientation)
                except ValueError as exc:
                    self._warn("fiducial_invalid", f"Ignoring fiducial batch with invalid pose: {exc}")
                    return
                supplied_norm = math.hypot(orientation.x, orientation.y, orientation.z, orientation.w)
                if (not math.isclose(supplied_norm, 1.0, rel_tol=0.0, abs_tol=1e-3)
                        or not all(finite_number(v) for v in
                                   (position.x, position.y, position.z))
                        or not math.isfinite(error)):
                    self._warn("fiducial_invalid", "Ignoring fiducial batch with invalid valid-pose data")
                    return
            else:
                camera_orientation = None
                placeholder = (position.x, position.y, position.z,
                               orientation.x, orientation.y, orientation.z, orientation.w)
                if (not all(finite_number(v) for v in placeholder)
                        or placeholder != (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)):
                    self._warn("fiducial_invalid", "Ignoring invalid-pose marker without its placeholder pose")
                    return
            validated.append((marker_id, marker_size, corners, pose_valid,
                              position, camera_orientation, float(error)))

        stamp_ns = self._stamp_ns(msg.header)
        try:
            if source_frame == self.policy_frame_id:
                translation = None
                policy_from_camera = (0.0, 0.0, 0.0, 1.0)
            else:
                # Nonblocking and exactly one lookup per batch at acquisition.
                transform = self.tf_buffer.lookup_transform(
                    self.policy_frame_id, source_frame, Time.from_msg(msg.header.stamp))
                translation = transform.transform.translation
                policy_from_camera = quaternion_xyzw(transform.transform.rotation)
                if not all(finite_number(v) for v in
                           (translation.x, translation.y, translation.z)):
                    raise ValueError("TF translation contains a nonfinite value")
        except (TransformException, ValueError) as exc:
            self._warn("fiducial_tf", f"Fiducial camera transform unavailable: {exc}")
            return

        records = []
        for marker_id, marker_size, corners, pose_valid, position, camera_orientation, error in validated:
            if pose_valid:
                if translation is None:
                    policy_position = (position.x, position.y, position.z)
                else:
                    try:
                        policy_position = transform_position(
                            translation, policy_from_camera, position)
                    except ValueError as exc:
                        # Finite inputs can still overflow during the transform.
                        # Reject the whole batch without refreshing its watchdog.
                        self._warn("fiducial_invalid",
                                   f"Ignoring fiducial batch with invalid transformed pose: {exc}")
                        return
                policy_orientation = quaternion_product(policy_from_camera, camera_orientation)
                policy_orientation = tuple(
                    value / math.hypot(*policy_orientation) for value in policy_orientation)
            else:
                policy_position = None
                policy_orientation = None
            records.append({
                "id": marker_id,
                "marker_size_m": marker_size,
                "corners": corners,
                "pose_valid": pose_valid,
                "position_xyz": policy_position,
                "orientation_xyzw": policy_orientation,
                "reprojection_error_px": error,
            })
        value = {"dictionary_name": dictionary_name, "source_frame_id": source_frame,
                 "detections": records}
        if self._store("fiducial_detections", value, stamp_ns, now, ros_now):
            if self.policy_update_mode == "fiducial_detection":
                self.run_policy_step()

    def lidar_callback(self, msg):
        now, ros_now = self._times()
        metadata = (msg.angle_min, msg.angle_max, msg.angle_increment,
                    msg.time_increment, msg.scan_time, msg.range_min, msg.range_max)
        if (not valid_frame(msg.header.frame_id) or not msg.ranges
                or not all(finite_number(v) for v in metadata)
                or msg.angle_increment <= 0.0 or msg.angle_max < msg.angle_min
                or msg.time_increment < 0.0 or msg.scan_time < 0.0
                or msg.range_min < 0.0 or msg.range_max <= msg.range_min
                or (msg.intensities and len(msg.intensities) != len(msg.ranges))
                or abs(msg.angle_min + (len(msg.ranges)-1)*msg.angle_increment - msg.angle_max)
                > max(0.001, 1.1*msg.angle_increment)):
            self._warn("lidar_invalid", "Ignoring lidar scan with invalid shape, frame, or metadata")
            return
        # Infinity/NaN and out-of-range individual rays are normal possibilities
        # in LaserScan. Preserve them for the student to filter, not as zeros.
        scan = {"ranges": list(msg.ranges), "intensities": list(msg.intensities),
                "frame_id": msg.header.frame_id, "angle_min": msg.angle_min,
                "angle_max": msg.angle_max, "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment, "scan_time": msg.scan_time,
                "range_min": msg.range_min, "range_max": msg.range_max}
        stamp_ns = self._stamp_ns(msg.header)
        if not self._store("lidar_scan", scan, stamp_ns, now, ros_now):
            return
        # One executor thread: finish both representations before any policy
        # step. A new accepted scan always invalidates the old Cartesian result;
        # failed conversion must never pair old indices with new raw ranges.
        self.observations["lidar_cartesian"] = None
        try:
            cartesian = self._convert_lidar_scan(scan, msg.header.stamp)
            self._store("lidar_cartesian", cartesian, stamp_ns, now, ros_now)
        except (TransformException, ValueError, OverflowError) as exc:
            self._warn("lidar_cartesian", f"Lidar Cartesian conversion unavailable: {exc}")
        if self.policy_update_mode == "lidar":
            self.run_policy_step()

    def _convert_lidar_scan(self, scan, stamp):
        """Convert one scan with a single nonblocking acquisition-time TF lookup."""
        offsets = (0.0, 0.0, 0.0)
        x_axis, y_axis = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
        if scan["frame_id"] != self.policy_frame_id:
            transform = self.tf_buffer.lookup_transform(
                self.policy_frame_id, scan["frame_id"], Time.from_msg(stamp))
            translation = transform.transform.translation
            x, y, z, w = quaternion_xyzw(transform.transform.rotation)
            offsets = (translation.x, translation.y, translation.z)
            if not all(finite_number(v) for v in offsets):
                raise ValueError("TF translation contains a nonfinite value")
            # The scan plane has z=0. Rotate its two basis vectors once, then
            # reuse them for every ray instead of repeating quaternion work.
            x_axis = (1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y))
            y_axis = (2*(x*y - w*z), 1 - 2*(x*x + z*z), 2*(y*z + w*x))
        points, indices = [], []
        for index, distance in enumerate(scan["ranges"]):
            if not finite_number(distance) or not scan["range_min"] <= distance <= scan["range_max"]:
                continue
            angle = scan["angle_min"] + index * scan["angle_increment"]
            local_x, local_y = distance * math.cos(angle), distance * math.sin(angle)
            point = tuple(offsets[axis] + local_x*x_axis[axis] + local_y*y_axis[axis]
                          for axis in range(3))
            if not all(math.isfinite(v) for v in point):
                raise ValueError("Cartesian lidar point is not finite")
            points.append(point)
            indices.append(index)
        return {"points_xyz": points, "scan_indices": indices, "frame_id": self.policy_frame_id}

    def wheel_speed_callback(self, msg):
        now, ros_now = self._times()
        if finite_number(msg.data) and msg.data >= 0.0:
            self._store("wheel_speed", msg.data, None, now, ros_now)
        else:
            self._warn("wheel_invalid", "Ignoring invalid unsigned wheel-speed measurement")

    def imu_callback(self, msg):
        now, ros_now = self._times()
        if not valid_frame(msg.header.frame_id):
            self._warn("imu_frame", "IMU measurement has no valid sensor frame")
            return
        try:
            if msg.header.frame_id == self.policy_frame_id:
                body_from_sensor = (0.0, 0.0, 0.0, 1.0)
            else:
                # Nonblocking: never wait for TF inside a sensor callback.
                transform = self.tf_buffer.lookup_transform(
                    self.policy_frame_id, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                body_from_sensor = quaternion_xyzw(transform.transform.rotation)
        except (TransformException, ValueError) as exc:
            self._warn("imu_tf", f"IMU mounting transform unavailable: {exc}")
            return

        stamp = self._stamp_ns(msg.header)
        # A partial IMU message may contain gyro but no orientation. Covariance
        # [0] == -1 marks an ABSENT field; its numeric zeros are not measurements.
        fields = (("imu_orientation", msg.orientation_covariance, msg.orientation),
                  ("imu_angular_velocity", msg.angular_velocity_covariance, msg.angular_velocity),
                  ("imu_specific_force", msg.linear_acceleration_covariance, msg.linear_acceleration))
        for name, covariance, measurement in fields:
            if covariance[0] == -1.0:
                continue
            try:
                if name == "imu_orientation":
                    # BNO reports q_ENU_from_sensor. We need q_ENU_from_body:
                    # q_ENU_from_sensor * inverse(q_body_from_sensor).
                    value = quaternion_product(quaternion_xyzw(measurement), quaternion_inverse(body_from_sensor))
                else:
                    value = rotate_vector(body_from_sensor, measurement)
                accepted = self._store(name, value, stamp, now, ros_now)
                if accepted and name == "imu_orientation" and self.heading_reference is not None:
                    heading = wrap_angle(roll_pitch_yaw(value)[2] - self.heading_reference)
                    self.heading_publisher.publish(Float32(data=math.degrees(heading)))
            except ValueError as exc:
                self._warn(name, f"Ignoring invalid {name}: {exc}")

    # ---- Main policy step: only the selected trigger calls this ---------------
    def run_policy_step(self):
        now, ros_now = self._times()
        if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
            return
        problem = self.health_problem(now, ros_now) or self._control_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return

        # Each field is a snapshot: optional stale observations become None.
        # ages/stamps remain available for understanding why a field is absent.
        values, ages, stamps = {}, {}, {}
        for name in SENSORS:
            observation = self.observations[name]
            ages[name] = None if observation is None else self._age(observation, now, ros_now)
            stamps[name] = None if observation is None else observation.stamp_ns
            values[name] = deepcopy(observation.value) if self._fresh(name, now, ros_now) else None
        first = self.previous_policy_step_at is None
        dt = 0.0 if first else now - self.previous_policy_step_at
        elapsed = now - self.policy_started_at
        self.previous_policy_step_at = now

        try:
            result = self.calculate_policy_actions(values, ages, stamps, dt, elapsed, first)
            drive, steer, pan, debug1, debug2 = result
            # Validate ALL outputs before publishing ANY of them.
            outputs = [drive, steer] + [v for v in (pan, debug1, debug2) if v is not None]
            if not all(finite_number(v) for v in outputs):
                raise ValueError("Policy outputs must be finite numbers; only pan/debug may be None")
            for debug in (debug1, debug2):
                if debug is not None and abs(debug) > 3.4028234663852886e38:
                    raise ValueError("Debug output does not fit a Float32")
            if any(abs(v) > 1.0 for v in (drive, steer) + (() if pan is None else (pan,))):
                self._warn("saturation", "Clipping policy action to normalized [-1, 1]")
            drive, steer = max(-1.0, min(1.0, drive)), max(-1.0, min(1.0, steer))
            pan = None if pan is None else max(-1.0, min(1.0, pan))
        except PolicyStopRequest as stop:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, str(stop))
            return
        except Exception:
            self.get_logger().error("Student policy failed:\n" + traceback.format_exc())
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, "Policy calculation or output was invalid")
            return

        # A calculation may have taken longer than a sensor's deadline. Check
        # again after it returns; never refresh stale driving actions on a timer.
        now, ros_now = self._times()
        if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
            return
        problem = self.health_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return
        problem = self._control_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return
        self._publish_actions(drive, steer, pan)
        if debug1 is not None:
            self.debug1_publisher.publish(Float32(data=float(debug1)))
        if debug2 is not None:
            self.debug2_publisher.publish(Float32(data=float(debug2)))

    def calculate_policy_actions(self, observations, sensor_age_s, sensor_stamp_ns,
                                 dt, policy_elapsed_s, is_first_policy_step):
        """
        STUDENT STARTING POINT

        This function:
        > Calculates one action from the most recent observations.
        > Is called repeatedly at the specified frequency (hence why it is important
          that you do NOT block this function with sleep or while loops or similar).
        > Is called only when this policy_node is in the publishing-policy state and
          only while the required observations are healthy.
        > Does NOT need to publish ROS messages or manage the state machine itself
          (that is all taken care of in other functions).
        """

        # CONE DETECTOR OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        cone_batch = observations["cone_detections"]
        cone_data_available = cone_batch is not None
        cones = [] if cone_batch is None else cone_batch["detections"]
        cone_acquisition_to_publish_latency_s = (None if cone_batch is None else
                                                 cone_batch["acquisition_to_publish_latency_s"])
        cone_measurement_age_s = sensor_age_s["cone_detections"]
        num_cones = len(cones)
        x_coords = [cone[0] for cone in cones]
        y_coords = [cone[1] for cone in cones]
        z_coords = [cone[2] for cone in cones]
        cone_colour = [cone[3] for cone in cones]
        cone_confidence = [cone[4] for cone in cones]

        # FIDUCIAL MARKER OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        fiducial_data = observations["fiducial_detections"]
        fiducials_available = fiducial_data is not None
        fiducial_dictionary_name = (None if fiducial_data is None
                                    else fiducial_data["dictionary_name"])
        fiducial_source_frame_id = (None if fiducial_data is None
                                    else fiducial_data["source_frame_id"])
        fiducials = [] if fiducial_data is None else fiducial_data["detections"]

        # WHEEL SPEED OBSERVATION: EXTRACT INTO LOCAL VARIABLE
        wheel_speed_in_meters_per_second = observations["wheel_speed"]

        # 2D LIDAR SCAN OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        lidar_scan = observations["lidar_scan"]
        lidar_scan_available = lidar_scan is not None
        lidar_ranges = [] if lidar_scan is None else lidar_scan["ranges"]
        lidar_intensities = [] if lidar_scan is None else lidar_scan["intensities"]
        lidar_cartesian = observations["lidar_cartesian"]
        lidar_cartesian_available = lidar_cartesian is not None
        lidar_points_xyz = [] if lidar_cartesian is None else lidar_cartesian["points_xyz"]
        lidar_scan_indices = [] if lidar_cartesian is None else lidar_cartesian["scan_indices"]

        # IMU OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        orientation_xyzw = observations["imu_orientation"]
        roll_angle_in_radians = pitch_angle_in_radians = heading_angle_in_radians = None
        if orientation_xyzw is not None:
            roll_angle_in_radians, pitch_angle_in_radians, yaw = roll_pitch_yaw(orientation_xyzw)
            if self.heading_reference is not None:
                heading_angle_in_radians = wrap_angle(yaw - self.heading_reference)
        angular_velocity_rad_per_sec = observations["imu_angular_velocity"]
        specific_force_m_per_sec_squared = observations["imu_specific_force"]

        # ===============================
        # EXPLANATION OF THE OBSERVATIONS
        # ===============================
        # NOTE: All lengths use meters and all angles use radians
        #
        # CONE DETECTOR OBSERVATIONS:
        # - LOCAL VARIABLE NAMES: x_coords/y_coords/z_coords and cone_colour/confidence
        #   - These are all parallel lists.
        #   - Index i describes one cone.
        #   - In the normal base_link frame, +x is forwards, +y left, +z upwards
        #     from nominal ground directly below the vehicle CG.
        #   - Hence, z_coords is the detected point's nominal height above that
        #     ground plane (it is NOT the cone's total height).
        #   - The fixed camera pose does NOT compensate for terrain or vehicle pitch.
        #   - The (x,y,z) coords are CAR coordinates (they are NOT world positions).
        #   - Use ConeDetection.COLOR_YELLOW / COLOR_BLUE (1/2).
        #   - A valid empty frame gives num_cones == 0 and cone_data_available True.
        #   - If cone detections are configured to be optional for this policy_node
        #     and no fresh cone detection message is available, then num_cones is 0
        #     and cone_data_available is False.
        # - LOCAL VARIABLE NAME: cone_acquisition_to_publish_latency_s
        #   - This is the fixed time from camera acquisition (end-of-exposure for OAK-D)
        #     to immediately before the detector called publish().
        #   - Exposure duration is excluded: this is not exposure-start-to-publication
        #     latency.
        #   - It includes device/host processing and queues, excludes downstream transport,
        #     and is None when no fresh cone batch is available.
        # - LOCAL VARIABLE NAME: cone_measurement_age_s
        #   - This is how old the acquisition is NOW, including transport and time spent
        #     waiting for this policy step.
        #   - It uses the same end-of-exposure reference for OAK-D.
        #   - Use this for measurement age; DO NOT add the publication latency again.
        #   - Age remains available for a stale batch, and is None before any batch.
        #   - Empty batches have timing too; zero cones does not mean zero latency.
        #
        # WHEEL SPEED OBSERVATION
        # - LOCAL VARIABLE NAME: wheel_speed_in_meters_per_second
        #   - This is the wheel speed estimate based on the measurement of the
        #     angular speed of the gearbox.
        #   - The vehicle's Traxxas node performs the calculations to estimate wheel
        #     speed from sparse encoder periods.
        #   - Sparse encoder periods means that low speeds take longer to measure.
        #   - Wheel speed is UNSIGNED, i.e., it does not tell you forward versus reverse.
        #   - The encoder_timeout_seconds for the vehicle's Traxxas node controls
        #     when it concludes that no encoder ticks means "wheels are stopped".
        #   - The sensor timeout here in this policy_node detects missing wheel speed
        #     telemetry.
        #   - None is not a measured zero.
        #
        # FIDUCIAL MARKER OBSERVATIONS
        # - LOCAL VARIABLE NAME: fiducials
        #   - This is a list of marker dictionaries. Each dictionary has:
        #     - id (int)
        #     - marker_size_m (float)
        #     - corners (four canonical-order (u,v) pixel pairs)
        #     - pose_valid (bool)
        #     - position_xyz (tuples in policy_frame_id; or None when pose_valid is false)
        #     - orientation_xyzw (tuples in policy_frame_id; or None when pose_valid is false)
        #     - reprojection_error_px (float; NaN means unavailable).
        # - LOCAL VARIABLE NAME: fiducial_dictionary_name
        #   - This scopes every marker ID in this batch.
        # - LOCAL VARIABLE NAME: fiducial_source_frame_id
        #   - This is the camera optical frame for corners; corners remain source-image pixels
        #     after the poses are transformed.
        # - ADDITIONAL NOTES:
        #   - Repeated IDs remain separate records in their received order.
        #   - An accepted empty batch gives fiducials == [] and fiducials_available True.
        #     It is fresh data and can trigger a step; marker absence has no automatic
        #     timeout or remembered pose.
        #   - Optional missing/stale data gives fiducials_available False.
        #
        # 2D LIDAR SCAN OBSERVATIONS
        # - LOCAL VARIABLE NAME: lidar_scan_available
        #   - This is False when the raw scan is missing/stale.
        # - LOCAL VARIABLE NAME: lidar_ranges[i]
        #   - This is the distance measured at angle_min + i * angle_increment.
        #   - It is measured in the frame: lidar_scan['frame_id'] (which is the
        #     lidar's original frame).
        #   - The raw scan and its origin are unchanged by the base_link
        #     ground-origin convention.
        #   - The lidar data is NOT rotated, it is in the frame of the lidar device.
        # - LOCAL VARIABLE NAME: lidar_intensities[i]
        #   - When supplied, this is matched to the i-th entry in lidar_ranges.
        #   - The list is empty if the lidar supplies no intensities; check before indexing.
        #   - This is the intensity of the ray when received back at the detector.
        #   - Intensity is device-specific; it is not a calibrated accuracy estimate.
        # - LOCAL VARIABLE NAME: lidar_scan
        #   - This is the dictionary with all details of the lidar scan.
        #   - Most relevant is that it contains:
        #     - angle_min / angle_max / angle_increment
        #     - time_increment
        #     - scan_time
        #     - range_min / range_max
        #   - You should retain only finite rays between range_min and range_max
        #     when your algorithm needs to detect real hits.
        #   - Infinity can mean no return; NaN is not a zero-distance obstacle.
        # - LOCAL VARIABLE NAME: lidar_cartesian_available
        #   - This is False when the conversion of the lidar scan ranges into
        #     cartesian coordinates is missing/stale.
        #   - Hence, this distinguishes unavailable conversion from a fresh scan
        #     with no usable returns (True with an empty list).
        # - LOCAL VARIABLE NAME: lidar_points_xyz
        #   - This is a list of (x, y, z) tuples in metres in policy_frame_id,
        #     which is normally body x forward, y left, z up.
        #   - Only finite returns within the scan's inclusive range limits are
        #     included. Every tuple is a usable point; there are no placeholders.
        # - LOCAL VARIABLE NAME: lidar_scan_indices[j]
        #   - This is matched to the length of lidar_points_xyz
        #   - This is the original ray index for point j, hence the original range
        #     can be extracted as: lidar_ranges[lidar_scan_indices[j]]
        # - ADDITIONAL NOTES:
        #   - Both forms are prepared once per accepted scan, before a policy
        #     update. Conversion is always enabled, including for radial-only
        #     policies. Use either representation or both in your code below.
        #   - Edit config/lidar_mount.yaml to change the mounting pose, then run
        #     dream runtime restart rplidar_c1. This affects Cartesian points;
        #     the raw scan stays in its original lidar frame.
        #   - Cartesian points always match the latest accepted raw scan. If
        #     its transform/conversion fails, raw data remains available and the
        #     old Cartesian result is discarded immediately.
        #   - Require lidar_scan and/or lidar_cartesian in required_sensors to
        #     stop driving when the data your policy uses is unavailable.
        #     Cartesian data expires at either representation's timeout.
        #   - The scan stamp is the first ray's acquisition time.
        #   - One mounting transform is used for the whole scan; points have no
        #     per-ray motion correction or IMU levelling.
        #   - The scan's timing fields are retained.
        #
        # IMU OBSERVATIONS:
        # - LOCAL VARIABLE NAME: orientation_xyzw
        #   - This is the orientation quaternion as an (x, y, z, w) tuple, or None.
        # - LOCAL VARIABLE NAMES: roll_angle_in_radians, pitch_angle_in_radians
        #   - A +x roll is forwards-axis rotation (i.e., car body tilts to the right).
        #   - A +y pitch is left-axis rotation (i.e., car nose dips down).
        # - LOCAL VARIABLE NAME: heading_angle_in_radians
        #   - This is the yaw of the car.
        #   - This is a relative heading that is re-zeroed ONLY when entering policy state.
        #   - It is wrapped to [-pi, pi].
        # - LOCAL VARIABLE NAME: angular_velocity_rad_per_sec
        #   - This is the 3-axis gyroscope measurement, i.e., angular velocity about
        #     each axis.
        # - LOCAL VARIABLE NAME: specific_force_m_per_sec_squared
        #   - This is the 3-axis accelerometer measurement, i.e., linear acceleration
        #     along each axis.
        #   - Specific force means that it INCLUDES GRAVITY; hence it is NOT pure
        #     driving acceleration.
        # - ADDITIONAL NOTES:
        #   - IMU quaternion/roll/pitch/vectors are in the policy body frame after
        #     the external mounting TF is applied.
        #   - Angles can drift.
        #   - Absolute orientation uses magnetic ENU (east/north/up).
        #   - Partial messages are normal and each IMU field may independently be None.
        #
        # OTHER RELEVANT INFORMATION:
        # - LOCAL VARIABLE NAME: sensor_age_s[name]
        #   - This is the age in seconds of the last accepted sample of that "name"
        #     sensor; or None if none exists.
        # - LOCAL VARIABLE NAME: sensor_stamp_ns[name]
        #   - This holds the respective ROS stamp in ns (None for wheel speed).
        #   - If an optional observations is expired, then its value is None.
        #
        # - LOCAL VARIABLE NAME: dt
        #   - This is the ACTUAL monotonic seconds between policy steps.
        #   - It is 0.0 on the first step.
        #   - Hence, if you use dt in your policy code, then you need to have a
        #     guard in your code to avoid division by dt.
        #   - If you operate this policy_node in timer mode, then it can reuse a
        #     sensor sample across several steps. In other words, a policy step is
        #     not necessarily a new measurement.
        #
        # - LOCAL VARIABLE NAME: policy_elapsed_s
        #   - This is the time (in seconds) since entering policy state.
        #
        # - LOCAL VARIABLE NAME: is_first_policy_step
        #   - This flag allows you reset an integrator (or similar) once per run.

        # ===============================
        # EXPLANATION OF THE ACTIONS
        # ===============================
        # ACTIONS are NORMALIZED values in [-1, 1]:
        #
        # - LOCAL VARIABLE NAME: drive_action
        #   - This requests motor effort (ESC), NOT speed in m/s.
        #
        # - LOCAL VARIABLE NAME: steering_action
        #   - This requests steering position; zero is centre.
        #   - This is NOT an angle in radians; it is a request normalized
        #     over the configured steering interval, with steering trim applied.
        #
        # - LOCAL VARIABLE NAME: camera_pan_action
        #   - This is the pan-servo target position.
        #   - This is NOT an angle in radians; it is a request normalized
        #     over the full pan-servo range available.
        #   - None means send no target and retain its current position.
        #   - It can move even while vehicle drive is disabled.
        #   - Zero means centre.
        #
        # Values outside [-1,1] are clipped. NaN/infinity/programming errors stop
        # the policy. Invalid data never becomes a motor command.

        # Initialize drive and steering to zero; None holds the camera pan position.
        drive_action = 0.0
        steering_action = 0.0
        camera_pan_action = None
        debug1 = None
        debug2 = None

        # =======================================
        # START OF: INSERT POLICY CODE BELOW HERE
        # =======================================

        # Preserve course framework inputs/actions/timing. The Word draft fills
        # internal group-record gaps only; screenshot spellings are aliases.
        # Downstream code may consume self.estimation_output["state"/"road"/
        # "obstacles"/"vehicle_params"/"vehicle_limits"]. Valid geometry is aligned
        # to state.timestamp_s; measurement_timestamp_s keeps source time. No
        # ReferenceTrajectory or target speed is generated by the estimator.
        if is_first_policy_step or not hasattr(self, "estimator"):
            self.estimator = EstimationPipeline(
                self.estimation_settings, self.policy_frame_id,
                ConeDetection.COLOR_BLUE, ConeDetection.COLOR_YELLOW, self.motion_history)
        sample_receipts = {name: None if obs is None else obs.received_at
                           for name, obs in self.observations.items()}
        receipt_ros_ns = {name: None if obs is None else obs.received_ros_ns
                          for name, obs in self.observations.items()}
        self.estimation_output = self.estimator.update(
            observations, sensor_age_s, sensor_stamp_ns, sample_receipts,
            self.get_clock().now().nanoseconds / 1e9, receipt_ros_ns)

        # Supply explicit car calibration at the integration boundary without
        # changing the estimator or substituting simulation for missing facts.
        calibration = getattr(self, "vehicle_settings", VehicleCalibrationSettings())
        if calibration.valid:
            params, limits = calibration.records()
            self.estimation_output["vehicle_params"] = params
            self.estimation_output["vehicle_limits"] = limits

        if is_first_policy_step or not hasattr(self, "planner"):
            self.planner = CenterlinePlanner(self.planning_settings, self.policy_frame_id)
        estimates = self.estimation_output
        source_timeouts = {"road": min(self.estimation_settings.max_source_age_s,
                                      self.sensor_timeout_s["cone_detections"]),
                           "wheel_speed": min(self.estimation_settings.max_source_age_s,
                                              self.estimation_settings.motion_max_gap_s,
                                              self.sensor_timeout_s["wheel_speed"]),
                           "imu_angular_velocity": min(self.estimation_settings.max_source_age_s,
                                                       self.estimation_settings.motion_max_gap_s,
                                                       self.sensor_timeout_s["imu_angular_velocity"])}
        self.planning_output, self.planning_diagnostics = self.planner.plan(
            estimates["road"], estimates["state"], estimates["obstacles"],
            estimates["vehicle_limits"], estimates["state"]["timestamp_s"], source_timeouts,
            mvp=getattr(self, "control_settings", ControlSettings()).mode == "mvp")

        control = getattr(self, "control_settings", ControlSettings())
        if control.enabled:
            if is_first_policy_step or not hasattr(self, "controller"):
                self.controller = PolicyController(control, self.policy_frame_id)
                self.control_has_run = False
            # Delayed camera data may predate the reset motion history at start.
            # Bounded neutral priming is allowed only BEFORE the first valid
            # control step; any fault after tracking requires explicit resume.
            if (not self.planning_output["valid"] and not self.control_has_run
                    and estimates["state"]["valid"]
                    and estimates["road"]["status"] == "missing_motion_history"
                    and policy_elapsed_s < control.startup_grace_s):
                self.control_diagnostics = {"valid": False, "stop_requested": False,
                                            "reason": "priming_motion_history"}
                return 0.0, 0.0, None, None, None
            if not self.planning_output["valid"]:
                reason = "Planning: " + str(self.planning_output["reason"])
                self.get_logger().warning("Planning rejection details: " + str({
                    "reason": self.planning_output["reason"],
                    "road_status": estimates["road"].get("status"),
                    "road_valid": estimates["road"].get("valid"),
                    "road_visibility": estimates["road"].get("visibility"),
                    "road_source_age_s": estimates["road"].get("source_age_s"),
                    "state_valid": estimates["state"].get("valid"),
                    "speed_valid": estimates["state"].get("speed_valid"),
                    "yaw_rate_valid": estimates["state"].get("yaw_rate_valid"),
                    "planning_diagnostics": self.planning_diagnostics,
                }))
                raise PolicyStopRequest(reason)
            drive_action, steering_action, self.control_diagnostics = self.controller.calculate(
                self.planning_output, estimates["state"], estimates["vehicle_params"],
                self.get_clock().now().nanoseconds/1e9, dt)
            if not self.control_diagnostics["valid"] or self.control_diagnostics["stop_requested"]:
                reason = "Control: " + self.control_diagnostics["reason"]
                raise PolicyStopRequest(reason)
            self.control_has_run = True
            self.last_control_reference_deadline_s = (
                self.planning_output["timestamp_s"]+self.planning_output["valid_for_s"])
            # Existing debug topics: lateral error (m), per-run distance (m).
            # The existing state string reports planning/control stop reasons.
            debug1 = self.control_diagnostics["path_error_m"]
            debug2 = self.distance_limiter.distance_m

        # =====================================
        # END OF: INSERT POLICY CODE ABOVE HERE
        # =====================================
        return drive_action, steering_action, camera_pan_action, debug1, debug2

    # ---- State requests, watchdog, and publication ---------------------------
    def fsm_transition_request_callback(self, msg):
        requested = msg.data
        now, ros_now = self._times()
        if requested not in STATE_NAMES:
            self._warn("state_request", f"Ignoring invalid policy state request: {requested}")
            return
        if requested == self.fsm_state:
            return  # No accidental heading/integrator/time reset on a repeat.
        if requested == FSM_STATE_PUBLISHING_POLICY_ACTION:
            problem = self.health_problem(now, ros_now)
            if problem:
                self.state_reason = f"Policy start refused: {problem}"
                self.get_logger().warning(self.state_reason)
                self.publish_state()
                return
            self.heading_reference = None
            if self._fresh("imu_orientation", now, ros_now):
                self.heading_reference = roll_pitch_yaw(self.observations["imu_orientation"].value)[2]
            # Optional missing heading stays unavailable until the next real
            # policy-state entry; a later IMU sample does not silently re-tare.
            self.policy_started_at = now
            self.previous_policy_step_at = None
        self._change_state(requested, "Operator request")

    def _change_state(self, state, reason):
        self.fsm_state = state
        self.state_reason = reason
        # An internal consumer must not mistake the previous run's estimate
        # for an active result after a stop, source expiry, or explicit restart.
        self.estimation_output = None
        self.motion_history.clear()
        self.planning_output = None
        self.planning_diagnostics = None
        self.controller.clear()
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False
        if state == FSM_STATE_PUBLISHING_POLICY_ACTION:
            wheel = self.observations.get("wheel_speed")
            speed = wheel.value if wheel is not None else None
            self.distance_limiter.reset(self._monotonic(), speed)
        if state == FSM_STATE_PUBLISHING_ZERO_ACTIONS:
            if self.control_settings.enabled:
                self.debug2_publisher.publish(Float32(data=float(self.distance_limiter.distance_m)))
            self.publish_zero_actions()
        self.get_logger().info(f"{STATE_NAMES[state]}: {reason}")
        self.publish_state()

    def _control_problem(self, monotonic_now, ros_now_ns):
        """Distance/time budgets and reference expiry need no sensor callback."""
        if not self.control_settings.enabled:
            return None
        wheel = self.observations.get("wheel_speed")
        if wheel is None or not self._fresh("wheel_speed", monotonic_now, ros_now_ns):
            return "Wheel speed unavailable for distance limit"
        problem = self.distance_limiter.advance(wheel.value, monotonic_now)
        if problem:
            return problem
        if (self.policy_started_at is not None and
                monotonic_now-self.policy_started_at >= self.control_settings.max_run_time_s):
            return "Run time limit reached; explicit resume required"
        deadline = self.last_control_reference_deadline_s
        if deadline is not None and ros_now_ns/1e9 >= deadline:
            return "Control reference expired; explicit resume required"
        if (not self.control_has_run and self.policy_started_at is not None
                and monotonic_now-self.policy_started_at >= self.control_settings.startup_grace_s):
            return "Control startup history deadline exceeded; explicit resume required"
        return None

    def supervision_callback(self):
        now, ros_now = self._times()
        if self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION:
            problem = self.health_problem(now, ros_now) or self._control_problem(now, ros_now)
            if problem:
                self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
        elif self.fsm_state == FSM_STATE_PUBLISHING_ZERO_ACTIONS:
            self.publish_zero_actions()
        if now - self._last_status_at >= self.status_period_s:
            self.publish_state()

    def publish_state(self):
        self.state_publisher.publish(Int8(data=self.fsm_state))
        self.state_text_publisher.publish(String(data=f"{STATE_NAMES[self.fsm_state]}: {self.state_reason}"))
        self._last_status_at = self._monotonic()

    def publish_zero_actions(self):
        self._publish_actions(0.0, 0.0, None)

    def _publish_actions(self, drive, steer, pan):
        command = DriveAndSteer()
        command.units = DriveAndSteer.UNITS_NORMALIZED
        command.drive, command.steer = float(drive), float(steer)
        self.action_publisher.publish(command)
        if pan is not None:
            self.pan_publisher.publish(Float32(data=float(pan)))

    def stop_before_shutdown(self):
        # Best effort while ROS is still usable. State 1 continues to mean no
        # publication; process loss is handled by the independent vehicle node.
        if self.fsm_state != FSM_STATE_NOT_PUBLISHING_ACTIONS and self.context.ok():
            self.publish_zero_actions()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PolicyNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_before_shutdown()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
