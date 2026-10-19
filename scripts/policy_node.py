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
import json
import queue
import signal
import subprocess
import sys
import threading
import time
import traceback


def _shadow_worker_entry():
    """Run only pure definitions from this SAME source; no ROS or hardware.

    The protocol is deliberately one job / one JSON result. Transport and
    process startup happen off the ROS executor, with latest-only mailboxes.
    """
    import ast
    import hashlib
    import os
    from pathlib import Path as FilePath
    if hasattr(os, "nice"):
        try:
            os.nice(10)  # Prefer the baseline ROS executor under CPU contention.
        except OSError:
            pass
    source_bytes = FilePath(__file__).read_bytes()
    tree = ast.parse(source_bytes.decode("utf-8"))
    names = {"ShadowSettings", "ShadowFrame", "ShadowLatticePlanner",
             "finite_number", "wrap_angle"}
    definitions = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
                   and (n.name in names or n.name.startswith("_shadow_"))
                   and n.name != "_shadow_worker_entry"]
    namespace = {"math": math, "dataclass": dataclass, "Real": Real,
                 "deepcopy": deepcopy, "time": time}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), __file__, "exec"), namespace)
    for line in sys.stdin:
        job = None
        decode_started = time.perf_counter()
        try:
            job = json.loads(line)
            decode_s = time.perf_counter()-decode_started
            result = namespace["ShadowLatticePlanner"](
                namespace["ShadowSettings"](**job["settings"])).plan(job)
        except Exception as error:
            decode_s = time.perf_counter()-decode_started
            result = {"status": "WORKER_ERROR", "error": str(error), "selected": None}
        result["generation"] = None if job is None else job.get("generation")
        result["job_id"] = None if job is None else job.get("job_id")
        result["source_sha256"] = hashlib.sha256(source_bytes).hexdigest()
        result["execution_anchor"] = None if job is None else {
            "pose": job.get("shadow_pose"), "odometry_epoch": job.get("shadow_odometry_epoch")}
        result["shadow_only"] = not bool(job and job.get("planner_control_enabled"))
        encode_started = time.perf_counter()
        encoded = json.dumps(result, allow_nan=False, separators=(",", ":"))
        wire_timing = {"json_decode_s": decode_s, "json_encode_s": time.perf_counter()-encode_started}
        print(encoded[:-1]+',"wire_timing_s":'+json.dumps(wire_timing)+'}', flush=True)


if __name__ == "__main__" and "--shadow-worker" in sys.argv:
    _shadow_worker_entry()
    sys.exit(0)

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
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
    # Local observed cones only, propagated incrementally between camera frames.
    road_history_s: float = 5.0
    road_rear_support_m: float = 0.5
    # Brief quality-drop bridge, not the lifetime of near-field observations.
    # Zero disables prediction; source and motion deadlines still apply.
    road_hold_s: float = 0.2
    # ROS cone colour IDs: yellow=1, blue=2. YAML selects the course layout.
    left_colour: int = 2
    right_colour: int = 1
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
    # Remember the last valid TWO-SIDED observed width for this policy run.
    # A fresh single boundary then defines an inward normal-offset centreline.
    # This is a constant-track-width assumption, not a hold of an old path.
    single_side_from_learned_width: bool = False
    # Fraction of the FULL learned/known lane width to offset inward from a
    # single observed boundary. 0.5 is geometric centre; smaller hugs that side.
    # This does not change width learning or the two-boundary centreline.
    single_side_offset_fraction: float = 0.5
    # Ordered XY curves support vertical and returning road segments. The
    # legacy y(x) estimator remains selectable for the archived workshop run.
    road_geometry: str = "y_of_x"
    # Imported mpc-car-run switches (docs/ESTIMATION_V1_CN.md); defaults keep
    # the original behaviour. "continuous" moves the cone history to every
    # policy step with short wheel/gyro intervals, so a camera gap no longer
    # needs one long motion interval and does not clear the history.
    history_propagation: str = "camera"
    # "distance" keeps a cone until it falls behind road_rear_support_m (the
    # local window bounds the dead-reckoned travel to ~3.5 m), with
    # road_history_max_s as a stationary backstop instead of road_history_s.
    road_history_mode: str = "time"
    road_history_max_s: float = 20.0
    # Merge the visible side of a one-sided frame with the history of both
    # sides instead of returning the one-sided frame unfused.
    history_merge_single_side: bool = False
    # A fresh cone replaces history cones within this radius (0 = the original
    # 2*huber_delta_m). It must stay below half the cone spacing, or a fresh
    # detection of cone k also deletes cone k-1 and leaves holes near the car.
    history_merge_radius_m: float = 0.0
    # Opt-in alternative: reconstruct an interrupted camera output from the
    # incrementally moved, observed cone cache. No unseen-road extrapolation.
    continuous_road_output: bool = False
    road_prediction_max_distance_m: float = 0.2

    def __post_init__(self):
        if (type(self.left_colour) is not int or type(self.right_colour) is not int
                or {self.left_colour, self.right_colour} != {1, 2}):
            raise ValueError("estimation left/right colours must be distinct yellow=1 and blue=2")
        if self.history_propagation not in ("camera", "continuous"):
            raise ValueError("estimation.history_propagation must be camera or continuous")
        if self.road_history_mode not in ("time", "distance"):
            raise ValueError("estimation.road_history_mode must be time or distance")
        if self.road_geometry not in ("y_of_x", "ordered_xy"):
            raise ValueError("estimation.road_geometry must be y_of_x or ordered_xy")
        if not isinstance(self.history_merge_single_side, bool):
            raise ValueError("estimation.history_merge_single_side must be bool")
        if not isinstance(self.continuous_road_output, bool):
            raise ValueError("estimation.continuous_road_output must be bool")
        if not isinstance(self.single_side_from_learned_width, bool):
            raise ValueError("estimation.single_side_from_learned_width must be bool")
        if self.continuous_road_output and self.history_propagation != "continuous":
            raise ValueError("continuous road output requires continuous history propagation")
        for name, value in vars(self).items():
            if name in ("history_propagation", "road_history_mode", "road_geometry", "history_merge_single_side",
                        "continuous_road_output", "single_side_from_learned_width"):
                continue
            if not finite_number(value):
                raise ValueError(f"estimation.{name} must be finite")
            if name == "min_cone_confidence":
                if not 0.0 <= value <= 1.0:
                    raise ValueError("min_cone_confidence must be in [0,1]")
            elif name == "known_lane_width_m":
                if value != 0.0 and not self.road_width_min_m <= value <= self.road_width_max_m:
                    raise ValueError("known width must be 0 (unknown) or inside width bounds")
            elif name == "single_side_offset_fraction":
                if not 0.0 < value < 1.0:
                    raise ValueError("single-side offset fraction must be in (0, 1)")
            elif name == "history_merge_radius_m":
                if value < 0.0:
                    raise ValueError("estimation.history_merge_radius_m must be >= 0")
            elif name == "road_hold_s":
                # The continuous version integrates each short motion segment;
                # admit the requested wider hold there, preserving legacy limits.
                hold_limit = 1.0 if self.continuous_road_output else 0.25
                if not 0.0 <= value <= hold_limit:
                    raise ValueError(f"road prediction hold must be in [0, {hold_limit}] seconds")
            elif value <= 0.0:
                raise ValueError(f"estimation.{name} must be positive")
        if self.road_width_min_m >= self.road_width_max_m:
            raise ValueError("road width bounds must increase")
        if self.road_min_coverage_m > self.road_forward_max_m:
            raise ValueError("road coverage exceeds the forward range")
        if self.road_sample_spacing_m > self.road_min_coverage_m:
            raise ValueError("road spacing exceeds minimum coverage")
        if (self.road_forward_max_m + self.road_rear_support_m) / self.road_sample_spacing_m > 200:
            raise ValueError("road sampling is limited to 201 points")
        if self.road_history_s > 5.0 or self.road_rear_support_m > 1.0:
            raise ValueError("local road history is limited to 5 s and 1 m behind the car")
        if not self.road_history_s <= self.road_history_max_s <= 30.0:
            raise ValueError("road_history_max_s must be in [road_history_s, 30] s")
        if self.road_prediction_max_distance_m > 0.5:
            raise ValueError("road prediction travel is limited to 0.5 m")

        if max(self.motion_max_gap_s, self.motion_max_interval_s) > self.max_source_age_s:
            raise ValueError("motion timing limits cannot exceed the source-age limit")

    @property
    def history_lifetime_s(self):
        """Cone lifetime: road_history_s, or the stationary backstop in distance mode."""
        return self.road_history_max_s if self.road_history_mode == "distance" else self.road_history_s


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
        self.reset_id = getattr(self, "reset_id", 0) + 1
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
        x = y = yaw = travelled = 0.0
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
            travelled += abs(distance)
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
        return {"dx_m": x, "dy_m": y, "yaw_rad": yaw, "interval_s": end_s-start_s,
                "travelled_distance_m": travelled}, None

    @staticmethod
    def transform_xy(point, pose):
        x, y = point[0]-pose["dx_m"], point[1]-pose["dy_m"]
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        return (c*x+s*y, -s*x+c*y)

    @staticmethod
    def inverse_transform_xy(point, pose):
        """A point in the frame at the END of pose -> the frame at its START."""
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        return (c*point[0]-s*point[1]+pose["dx_m"], s*point[0]+c*point[1]+pose["dy_m"])


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
    # Three fixed baselines keep robust initialization linear in point count;
    # do not build all N*(N-1)/2 pairs for each side of every camera frame.
    strides = sorted({1, max(1, len(points)//4), max(1, len(points)//2)})
    slopes = [(points[i+d][1]-points[i][1])/(points[i+d][0]-points[i][0])
              for d in strides for i in range(len(points)-d) if points[i+d][0]-points[i][0] > 1e-6]
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
        updated = _estimation_solve(matrix, rhs)
        if updated is None:
            return None
        converged = max(abs(a-b) for a, b in zip(coeffs, updated)) < 1e-7
        coeffs = updated
        if converged:
            break
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
            "support_points": [points[i] for i in inliers],
            "inlier_fraction": len(inliers)/len(points)}


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


def _path_project(points, lengths, query=(0.0, 0.0), forward=True):
    """Project onto the local travel branch; a returning leg is not a shortcut.

    Forward motion cannot acquire a rear-facing segment on a cold start. The
    branch is selected in the current body frame on every tick, so a completed
    U turn naturally presents its exit as forward-facing. At ties prefer the
    earlier path segment. No global Euclidean reordering is performed.
    """
    best = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = lengths[i+1]-lengths[i]
        if forward and dx/length < -0.25:
            continue
        u = min(1.0, max(0.0, ((query[0]-a[0])*dx+(query[1]-a[1])*dy)/length**2))
        point = (a[0]+u*dx, a[1]+u*dy)
        cost = (point[0]-query[0])**2+(point[1]-query[1])**2
        candidate = (cost, lengths[i]+u*length, point, i, u)
        if best is None or cost < best[0]-1e-9:
            best = candidate
    return best


def _estimation_order_boundary(points, settings):
    """Bounded local nearest-neighbour chain with direction continuity.

    Confidence filters run before this function. Near duplicates are removed;
    growth never bridges a gap larger than road_max_gap_m, or makes an abrupt
    reverse. A sparse/ambiguous cone layout is not a complete track map.
    """
    unique = []
    for p in sorted(points, key=lambda p: (-p[2], p[0], p[1])):
        if not any(math.hypot(p[0]-q[0], p[1]-q[1]) < 0.03 for q in unique):
            unique.append(p)
    unique = sorted(unique, key=lambda p: (p[0]**2+p[1]**2, p[0], p[1]))[:64]
    if len(unique) < 2:
        return None
    nearest_spacings = [min(math.hypot(p[0]-q[0], p[1]-q[1])
                            for j, q in enumerate(unique) if j != i)
                        for i, p in enumerate(unique)]
    # Do not close a hairpin by jumping across its open end after ordinary
    # adjacent cones run out. The absolute configured gap remains an upper
    # bound, with a local spacing bound for densely populated tracks.
    chain_gap = min(settings.road_max_gap_m,
                    max(3*settings.road_sample_spacing_m, 3*_estimation_median(nearest_spacings)))
    def grow(direction, seed, used):
        chain, current = [], unique[seed]
        while len(used) < len(unique):
            choices = []
            for i, p in enumerate(unique):
                if i in used:
                    continue
                dx, dy = p[0]-current[0], p[1]-current[1]
                distance = math.hypot(dx, dy)
                cosine = (dx*direction[0]+dy*direction[1])/distance
                if distance <= chain_gap+1e-9 and cosine >= -0.25:
                    choices.append((distance*(1+0.5*(1-cosine)), distance, i, dx, dy))
            if not choices:
                break
            _, distance, i, dx, dy = min(choices)
            used.add(i)
            current = unique[i]
            chain.append(current)
            direction = (dx/distance, dy/distance)
        return chain

    # The nearest same-colour point can be a misclassification on the OTHER
    # boundary. Compare at most six near seeds and retain the best supported
    # contiguous chain, rather than letting an isolated short cluster win.
    # Near-seed bounds keep a long but distant branch from replacing entry.
    choices = []
    nearest_range = math.hypot(*unique[0][:2])
    for seed in range(min(6, len(unique))):
        if math.hypot(*unique[seed][:2]) > nearest_range+chain_gap:
            continue
        used = {seed}
        ahead = grow((1.0, 0.0), seed, used)
        behind = grow((-1.0, 0.0), seed, used)
        chain = list(reversed(behind))+[unique[seed]]+ahead
        lengths = _path_lengths(chain)
        if len(chain) >= 2 and lengths[-1] >= settings.road_min_coverage_m:
            near = min(math.hypot(*p[:2]) for p in chain)
            support = sum(p[2] for p in chain)/(1+max(0.0, near-nearest_range)/chain_gap)
            choices.append((support, lengths[-1], -near, -seed, chain, lengths))
    if not choices:
        return None
    _, _, _, _, chain, lengths = max(choices, key=lambda c: c[:4])
    return {"support_points": chain, "range": (0.0, lengths[-1]),
            "lengths": lengths, "rms_m": None,
            "inlier_fraction": len(chain)/len(unique), "encoding": "ordered_xy"}


class EstimationPipeline:
    """Local geometry aligned to the state epoch with bounded planar odometry.

    Draft names remain canonical; aliases support the supplied screenshots.
    Raw road() reconstructs acquisition-frame geometry. update() transports it
    to the current body frame, or rejects it if historical motion is missing.
    This is approximate forward odometry, not SLAM or calibrated localization.
    """
    QUALITY_FAILURES = ("insufficient_geometry", "single_side_width_unknown",
                        "single_side_inferred", "insufficient_common_range")
    # A stale or missing camera frame while wheel/gyro stay continuous. With
    # history_propagation=continuous these end the output, not the history.
    CAMERA_GAP_FAILURES = ("unavailable", "motion_interval_too_long")

    def __init__(self, settings, frame_id="base_link", left_colour=None, right_colour=None,
                 motion_history=None):
        self.settings, self.frame_id = settings, frame_id
        self.left_colour = settings.left_colour if left_colour is None else left_colour
        self.right_colour = settings.right_colour if right_colour is None else right_colour
        self.motion = motion_history or EstimationMotionHistory(settings)
        self._clear_road_history()

    def _clear_road_history(self, *, keep_learned_width=False):
        if not keep_learned_width:
            self._learned_lane_width_m = None
            self._learned_lane_width_stamp_s = None
        self._road_cones = []  # (x, y, z, colour, confidence, original_seen_s)
        self._road_stamp = None
        self._fused_source_s = None  # camera time of the last fused frame
        self._road_frame = None
        self._observation_stamp = None
        self._observation_road = None
        self._trusted_road = None  # Unaligned acquisition-frame geometry only.
        self._failure_since_s = None
        self._prediction_motion_stamp = None
        self._prediction_distance_m = 0.0
        self._prediction_pose = None
        self._prediction_expired_source_s = None
        self._road_reset_id = self.motion.reset_id

    def _advance_prediction_distance(self, now_s):
        """Count travel once per policy epoch, independent of camera rebasing.

        Integrated speed counts turning as travel, rather than using a net
        displacement that could hide a manoeuvre. The unsigned wheel contract
        still supports forward motion only. A missing motion segment discards
        both the prediction anchor and the cone cache.
        """
        if self._prediction_motion_stamp is None:
            return
        pose, problem = self.motion.pose_between(self._prediction_motion_stamp, now_s)
        if problem is not None:
            self._clear_road_history()
            return
        self._prediction_distance_m += pose["travelled_distance_m"]
        total = self._prediction_pose
        c, s = math.cos(total["yaw_rad"]), math.sin(total["yaw_rad"])
        total["dx_m"] += c*pose["dx_m"]-s*pose["dy_m"]
        total["dy_m"] += s*pose["dx_m"]+c*pose["dy_m"]
        total["yaw_rad"] += pose["yaw_rad"]
        total["interval_s"] += pose["interval_s"]
        total["travelled_distance_m"] = self._prediction_distance_m
        self._prediction_motion_stamp = now_s

    def _advance_history(self, now_s):
        """Continuous mode: move the cone history to now_s over one short interval.

        Called every policy step, so no single motion interval spans a camera
        gap. Cones behind road_rear_support_m or past their lifetime drop out.
        A motion problem still clears the history, as in camera mode.
        """
        if self._road_reset_id != self.motion.reset_id:
            self._clear_road_history()
            return
        if not self._road_cones or self._road_stamp is None or now_s <= self._road_stamp:
            return
        pose, problem = self.motion.pose_between(self._road_stamp, now_s)
        if problem is not None:
            self._clear_road_history()
            return
        cones = []
        for x, y, z, colour, confidence, seen in self._road_cones:
            x, y = self.motion.transform_xy((x, y), pose)
            if x >= -self.settings.road_rear_support_m and now_s-seen <= self.settings.history_lifetime_s:
                cones.append((x, y, z, colour, confidence, seen))
        self._road_cones, self._road_stamp = cones, now_s

    def _history_at(self, source_s):
        """History cones in the camera frame at source_s: (cones, None) or (None, problem).

        Camera mode stores history at the last fused camera time (earlier than
        source_s); continuous mode has already moved it past source_s.
        """
        if self._road_stamp is None:
            return [], None
        if source_s >= self._road_stamp:
            pose, problem = self.motion.pose_between(self._road_stamp, source_s)
            move = self.motion.transform_xy
        else:
            pose, problem = self.motion.pose_between(source_s, self._road_stamp)
            move = self.motion.inverse_transform_xy
        if problem is not None:
            return None, problem
        return [(*move(p[:2], pose), *p[2:]) for p in self._road_cones], None

    @staticmethod
    def _observed_support_conflicts(x, y, fit, tolerance):
        """Check real inlier support, not a global quadratic's model residual.

        A changing view of a bend changes its best quadratic even when the
        observed cones agree. That must not discard all unseen near history.
        Actual contradictions within overlapping measured support still clear
        history; no comparison or extension is made outside the observed span.
        """
        if fit.get("encoding") == "ordered_xy":
            points = [p[:2] for p in fit["support_points"]]
            projection = _path_project(points, _path_lengths(points), (x, y), forward=False)
            _, _, _, i, u = projection
            if (i == 0 and u == 0) or (i == len(points)-2 and u == 1):
                return False  # Beyond observed endpoints is not a contradiction.
            return math.sqrt(projection[0]) > tolerance
        if not fit["range"][0] <= x <= fit["range"][1]:
            return False
        points = fit["support_points"]
        at_x = [p for p in points if abs(p[0]-x) <= 1e-9]
        if at_x:
            return min(abs(y-p[1]) for p in at_x) > tolerance
        for a, b in zip(points, points[1:]):
            if a[0] < x < b[0]:
                observed_y = a[1]+(b[1]-a[1])*(x-a[0])/(b[0]-a[0])
                return abs(y-observed_y) > tolerance
        return False

    def _history_road(self, batch, stamp_ns, age, now_s):
        """Merge bounded, validated observations; never extrapolate a blind start.

        At most 64 cones per side, with latest observations taking precedence
        within the Huber residual radius. Stored ages refer to real detections,
        not predictions. A fresh two-sided fit seeds observed history; when
        enabled, valid partial fits can extend existing two-sided support.
        A bounded quality-drop bridge may reuse the last fully valid corridor.
        """
        if self._road_reset_id != self.motion.reset_id:
            self._clear_road_history()
        source_s = stamp_ns / 1e9 if stamp_ns is not None else None
        if (not self._fresh(batch, age) or source_s is None or source_s > now_s
                or (self._observation_stamp is not None and source_s < self._observation_stamp)):
            out_of_order = (self._observation_stamp is not None and source_s is not None
                            and source_s < self._observation_stamp)
            # A camera gap (stale or missing batch) keeps a continuously moved
            # history; out-of-order or future stamps still clear it.
            if not (self.settings.history_propagation == "continuous" and not out_of_order
                    and (source_s is None or source_s <= now_s)):
                self._clear_road_history()
            result = self.road(batch, stamp_ns, age)
            if out_of_order:
                result.update(valid=False, status="out_of_order_camera", centerline_xy=[],
                              left_boundary_xy=None, right_boundary_xy=None)
            return result
        expired = any(now_s - p[5] > self.settings.history_lifetime_s for p in self._road_cones)
        if source_s == self._fused_source_s and self._road_frame is not None and not expired:
            result = deepcopy(self._road_frame)
            result["source_age_s"] = age
            return result

        if source_s != self._observation_stamp or self._observation_road is None:
            self._observation_stamp = source_s
            self._observation_road = self.road(batch, stamp_ns, age)
            # Learn ONLY from a fresh raw frame that independently fits BOTH
            # sides. Merged history and inferred/predicted roads never teach
            # a width or refresh its acquisition timestamp.
            if (self.settings.single_side_from_learned_width
                    and self._observation_road["valid"]
                    and self._observation_road["visibility"] == "both"):
                self._learned_lane_width_m = self._observation_road["lane_width_m"]
                self._learned_lane_width_stamp_s = source_s
        current = deepcopy(self._observation_road)
        current["source_age_s"] = age
        if current.get("single_side_inferred_from_learned_width"):
            # Use today's visible boundary with the remembered HALF width.
            # Old opposite-side cones must not veto or distort that trajectory.
            # Width remains fixed until both boundaries are observed again;
            # current camera and wheel/gyro deadlines still govern every step.
            if self.settings.road_geometry == "ordered_xy":
                # Retain only the actually observed visible chain for a
                # bounded camera dropout. Do not fabricate opposite cones.
                self._road_cones = [(x, y, 0.0, colour, confidence, source_s)
                    for colour, fit in current["_boundary_fits"].items()
                    if (colour == self.left_colour) == (current["visibility"] == "left_only")
                    for x, y, confidence in fit["support_points"]]
                self._road_stamp = self._fused_source_s = source_s
                self._road_frame = deepcopy(current)
            else:
                self._clear_road_history(keep_learned_width=True)
            return current
        if not current["valid"] or current["visibility"] != "both":
            if current["status"] not in self.QUALITY_FAILURES:
                self._clear_road_history()
                return current
            # Even a one-sided fit can contradict an old corridor. Never hide
            # an observed change behind a prediction from the other boundary.
            placed, _ = self._history_at(source_s)
            for x, y, _, colour, _, _ in placed or []:
                fit = current.get("_boundary_fits", {}).get(colour)
                if fit is None:
                    continue
                if self._observed_support_conflicts(x, y, fit, 2*self.settings.huber_delta_m):
                    self._clear_road_history()
                    current.update(valid=False, status="observed_history_conflict")
                    return current
            # history_merge_single_side: the visible side(s) of a one-sided or
            # short-overlap frame join the history of both sides below. A failed
            # merge returns this frame unchanged and leaves the history alone.
            if not (self.settings.history_merge_single_side and self._road_cones
                    and current.get("_boundary_fits")):
                return current
            partial = True
        else:
            partial = False
        fits = current.pop("_boundary_fits")
        placed, _ = self._history_at(source_s)
        previous = [p for p in placed or []
                    if 0 <= now_s-p[5] <= self.settings.history_lifetime_s
                    and -self.settings.road_rear_support_m <= p[0] <= self.settings.road_forward_max_m]
        # An observed change in the overlapping corridor invalidates old support.
        radius = 2*self.settings.huber_delta_m
        for colour, fit in fits.items():
            if any(p[3] == colour and self._observed_support_conflicts(p[0], p[1], fit, radius)
                   for p in previous):
                previous = []
                break
        fresh = [(x, y, 0.0, colour, confidence, source_s)
                 for colour, fit in fits.items() for x, y, confidence in fit["support_points"]]
        merge_radius = self.settings.history_merge_radius_m or radius
        for point in fresh:
            previous = [p for p in previous if p[3] != point[3]
                        or math.hypot(p[0]-point[0], p[1]-point[1]) > merge_radius]
        merged = previous + fresh
        bounded = []
        for colour in (self.left_colour, self.right_colour):
            side = sorted((p for p in merged if p[3] == colour), key=lambda p: p[0])
            if len(side) > 64:
                side = [side[round(i*(len(side)-1)/63)] for i in range(64)]
            bounded.extend(side)
        result = self.road({"detections": [p[:5] for p in bounded]}, stamp_ns, age)
        if not result["valid"] or (partial and result["visibility"] != "both"):
            if partial:
                current.pop("_support_cones", None)
                return current
            # Never let a history-induced bad fit poison subsequent frames.
            bounded, result = fresh, current
        else:
            result.pop("_boundary_fits", None)
            # Keep only inliers of the combined fit, including their original age.
            supported = result.pop("_support_cones")
            bounded = [p for p in bounded if p[:5] in supported]
        current.pop("_support_cones", None)
        result.pop("_support_cones", None)
        result["history_used"] = any(p[5] < source_s for p in bounded)
        if result["history_used"]:
            result["status"] = "observed_with_history"
            result["boundary_source"] = {"left": "observed_with_history", "right": "observed_with_history"}
            if partial:
                # The side without fresh support in this frame is history only.
                result["boundary_source"] = {
                    side: "observed_with_history" if colour in fits else "history"
                    for side, colour in (("left", self.left_colour), ("right", self.right_colour))}
                result["merged_partial_frame"] = True
        result["oldest_observation_timestamp_s"] = min(p[5] for p in bounded)
        self._road_cones, self._road_stamp = bounded, source_s
        self._fused_source_s = source_s
        self._road_frame = deepcopy(result)
        return result

    def _predict_road(self, current_frame, now_s):
        """Transport one last fully valid observation; never promote prediction.

        Geometry remains anchored at its real acquisition timestamp. Repeated
        failed batches do not renew either the observation or the hold budget.
        """
        if self._trusted_road is None or self.settings.road_hold_s == 0:
            return None, "no_trusted_road"
        if self.settings.continuous_road_output:
            return self._predict_continuous_road(current_frame, now_s)
        if self._failure_since_s is None:
            self._failure_since_s = now_s
        elapsed = now_s-self._failure_since_s
        source_s = self._trusted_road["timestamp_s"]
        age = now_s-source_s
        oldest = self._trusted_road.get("oldest_observation_timestamp_s", source_s)
        remaining = min(self.settings.road_hold_s-elapsed,
                        self.settings.max_source_age_s-age,
                        self.settings.motion_max_interval_s-age,
                        self.settings.history_lifetime_s-(now_s-oldest))
        if elapsed < 0 or age < 0 or remaining <= 1e-9:
            return None, "prediction_deadline_expired"
        predicted = deepcopy(self._trusted_road)
        predicted.update(source_age_s=age, history_used=True)
        predicted = self._align_road(predicted, now_s, True)
        if not predicted["valid"]:
            return None, predicted["status"]
        predicted.update(status="predicted_history", degraded=True,
                         current_frame=current_frame, prediction_status="active",
                         prediction_elapsed_s=elapsed, prediction_age_s=age,
                         prediction_remaining_s=remaining, recommended_speed_scale=0.5,
                         confidence=0.5*predicted["confidence"],
                         boundary_source={"left": "predicted_history", "right": "predicted_history"})
        return predicted, "active"

    def _predict_continuous_road(self, current_frame, now_s):
        """Fit moved REAL observations, keeping the last accepted camera epoch.

        Incremental wheel/gyro segments each obey the usual motion limits. The
        total prediction still has independent source-age, hold and travel
        deadlines. Neither a predicted fit nor a repeated frame renews them.
        """
        if self._failure_since_s is None:
            self._failure_since_s = now_s
        source_s = self._trusted_road["timestamp_s"]
        age, elapsed = now_s-source_s, now_s-self._failure_since_s
        remaining = min(self.settings.road_hold_s-elapsed,
                        self.settings.max_source_age_s-age)
        distance_left = self.settings.road_prediction_max_distance_m-self._prediction_distance_m
        if age < 0 or elapsed < 0 or remaining <= 1e-9 or distance_left <= 1e-9:
            return None, "prediction_deadline_expired"
        speed = self.motion.current("wheel_speed", now_s)
        if speed is not None and speed > 1e-9:
            # Advisory time at current filtered speed; acceleration can consume
            # distance faster, so consumers must still recheck every update.
            remaining = min(remaining, distance_left/speed)
        placed, problem = self._history_at(now_s)
        if problem is not None:
            return None, problem
        if not placed:
            return None, "no_observed_history"
        oldest = min(p[5] for p in placed)
        remaining = min(remaining, self.settings.history_lifetime_s-(now_s-oldest))
        if remaining <= 1e-9:
            return None, "near_field_history_expired"
        predicted = self.road({"detections": [p[:5] for p in placed]}, round(now_s*1e9), 0.0)
        predicted.update(timestamp_s=now_s, history_used=True, oldest_observation_timestamp_s=oldest)
        predicted = self._align_road(predicted, now_s, True)
        if (self.settings.road_geometry == "ordered_xy" and not predicted["valid"]
                and (predicted.get("forward_geometry") or {}).get("single_side_inferred_from_learned_width")):
            predicted = predicted["forward_geometry"]
            predicted["valid"] = True
        if not predicted["valid"]:
            return None, predicted["status"]
        # The fit above is in today's body frame; it is not a new camera sample.
        # Preserve acquisition provenance separately from the aligned epoch.
        predicted.update(measurement_timestamp_s=source_s, source_age_s=age,
                         motion_compensated=age > 0, motion_compensation=deepcopy(self._prediction_pose),
                         fusion_mode="observed_cones_wheel_gyro_incremental",
                         status="predicted_history", degraded=True,
                         current_frame=current_frame, prediction_status="active",
                         prediction_elapsed_s=elapsed, prediction_age_s=age,
                         prediction_remaining_s=remaining,
                         prediction_distance_m=self._prediction_distance_m,
                         prediction_remaining_distance_m=distance_left,
                         recommended_speed_scale=0.5,
                         confidence=0.5*predicted["confidence"],
                         boundary_source={side: "predicted_history" if predicted.get(side+"_boundary_xy")
                                          else "inferred_width" for side in ("left", "right")})
        return predicted, "active"

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
        output["boundary_diagnostics"] = {
            side: {"raw_count": 0, "filtered_count": 0, "fit_valid": False,
                   "fit_status": "unavailable", "inlier_count": 0,
                   "fit_rms_m": None, "x_range_m": None}
            for side in ("left", "right")}
        if not self._fresh(batch, age) or timestamp is None:
            return output
        sides = {self.left_colour: [], self.right_colour: []}
        for x, y, z, colour, confidence in batch["detections"]:
            if colour in sides:
                side = "left" if colour == self.left_colour else "right"
                output["boundary_diagnostics"][side]["raw_count"] += 1
            if (all(finite_number(v) for v in (x, y, z, confidence))
                    and -self.settings.road_rear_support_m <= x <= self.settings.road_forward_max_m
                    and abs(y) <= self.settings.road_width_max_m + self.settings.road_forward_max_m*self.settings.road_max_abs_slope
                    and self.settings.min_cone_confidence <= confidence <= 1.0
                    and confidence > 0 and colour in sides):
                sides[colour].append((x, y, confidence))
        if self.settings.road_geometry == "ordered_xy":
            return self._ordered_road(output, sides)
        left = _estimation_fit_boundary(sides[self.left_colour], self.settings)
        right = _estimation_fit_boundary(sides[self.right_colour], self.settings)
        output["_boundary_fits"] = {colour: fit for colour, fit in
                                    ((self.left_colour, left), (self.right_colour, right)) if fit is not None}
        for side, colour, fit in (("left", self.left_colour, left), ("right", self.right_colour, right)):
            diag = output["boundary_diagnostics"][side]
            diag["filtered_count"] = len(sides[colour])
            points = sides[colour]
            diag["fit_status"] = ("insufficient_points" if len(points) < 2 else
                                  "insufficient_x_coverage" if max(p[0] for p in points)-min(p[0] for p in points)
                                  < self.settings.road_min_coverage_m else "fit_rejected")
            if fit is not None:
                diag.update(fit_valid=True, fit_status="accepted", inlier_count=len(fit["support_points"]),
                            fit_rms_m=fit["rms_m"], x_range_m=list(fit["range"]))
        output["status"] = "insufficient_geometry"
        if left is None and right is None:
            return output
        lane_width = self.settings.known_lane_width_m
        learned_single_side = False
        if left is None or right is None:
            output["visibility"] = "left_only" if left is not None else "right_only"
            if (self.settings.single_side_from_learned_width
                    and self._learned_lane_width_m is not None):
                lane_width = self._learned_lane_width_m
                learned_single_side = True
            if lane_width == 0:
                output["status"] = "single_side_width_unknown"
                return output
        fits = [fit for fit in (left, right) if fit is not None]
        lo, hi = max(f["range"][0] for f in fits), min(f["range"][1] for f in fits)
        if hi-lo < self.settings.road_min_coverage_m:
            if (len(fits) == 2 and self.settings.single_side_from_learned_width
                    and self._learned_lane_width_m is not None):
                # A few returning FAR cones do not yet define a two-sided
                # corridor beside the visible near boundary. Keep the near
                # side with the last learned width until common support returns.
                chosen = min(fits, key=lambda f: (f["range"][0],
                                                  -(f["range"][1]-f["range"][0])))
                left, right = (chosen, None) if chosen is left else (None, chosen)
                output["visibility"] = "left_only" if left is not None else "right_only"
                output["single_side_fallback_reason"] = "insufficient_common_range"
                lane_width = self._learned_lane_width_m
                learned_single_side = True
                fits = [chosen]
                lo, hi = chosen["range"]
            else:
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
                width = lane_width
                # Offset along the local normal, not simply along body y.
                direction = -1 if left is not None else 1
                offset = direction*width*self.settings.single_side_offset_fraction/math.hypot(1, slope)
                center.append((x-offset*slope, y+offset))
            widths.append(width)
        if (any(not self.settings.road_width_min_m <= w <= self.settings.road_width_max_m for w in widths)
                or any(b[0] <= a[0] for a, b in zip(center, center[1:]))
                or center[0][0] < -self.settings.road_rear_support_m
                or center[-1][0]-center[0][0] < self.settings.road_min_coverage_m):
            output["status"] = "invalid_width_or_order"
            return output
        output.update(valid=True, centerline_xy=center, lane_width_m=_estimation_median(widths),
                      x_range_m=[center[0][0], center[-1][0]],
                      confidence=min(f["inlier_fraction"] for f in fits) * (1.0 if len(fits) == 2 else 0.5),
                      visibility="both" if len(fits) == 2 else output["visibility"],
                      status="observed" if len(fits) == 2 else "single_side_inferred",
                      fit_rms_m=max(f["rms_m"] for f in fits))
        if learned_single_side:
            output.update(single_side_inferred_from_learned_width=True,
                          lane_width_source="last_two_sided_observation",
                          lane_width_observed_at_s=self._learned_lane_width_stamp_s,
                          inward_offset_m=lane_width*self.settings.single_side_offset_fraction,
                          centerline_source="left_boundary_inward_offset" if left is not None
                          else "right_boundary_inward_offset")
        for side, fit in (("left", left), ("right", right)):
            output[f"{side}_boundary_xy"] = samples[side] if fit is not None else None
            output["boundary_source"][side] = "observed" if fit is not None else "absent"
        output["_boundary_fits"] = {colour: fit for colour, fit in
                                    ((self.left_colour, left), (self.right_colour, right)) if fit is not None}
        output["_support_cones"] = [(x, y, 0.0, colour, confidence)
                                    for colour, fit in output["_boundary_fits"].items()
                                    for x, y, confidence in fit["support_points"]]
        if len(fits) == 2:
            a0 = (left["coeffs"][0]+right["coeffs"][0])/2
            a1 = (left["coeffs"][1]+right["coeffs"][1])/2
            a2 = (left["coeffs"][2]+right["coeffs"][2])/2
            output["_center_coeffs"] = (a0, a1, a2)
            slope = a1+2*a2*lo
            output["local_curvature_1pm"] = 2*a2/(1+slope*slope)**1.5
        # For an offset curve, do not publish its boundary curvature as the
        # centreline's curvature. None is honest until that derivation is added.
        return output

    def _ordered_road(self, output, sides, reference_side=None):
        """Normal-paired, ordered local boundary chains and single-side offsets.

        Pairing follows the opposite chain's travel order and requires agreeing
        tangents. It never pairs a U-turn entry to its rear-facing exit. Samples
        are restricted to measured support; no endpoint tangent extension.
        """
        cfg = self.settings
        fits = {name: _estimation_order_boundary(sides[colour], cfg)
                for name, colour in (("left", self.left_colour), ("right", self.right_colour))}
        output["geometry_encoding"] = "ordered_xy"
        output["_boundary_fits"] = {}
        for name, colour in (("left", self.left_colour), ("right", self.right_colour)):
            fit = fits[name]
            diag = output["boundary_diagnostics"][name]
            diag.update(filtered_count=len(sides[colour]), fit_valid=fit is not None,
                        fit_status="accepted" if fit else "insufficient_chain_support")
            if fit:
                diag.update(inlier_count=len(fit["support_points"]),
                            arc_length_m=fit["lengths"][-1])
                output["_boundary_fits"][colour] = fit
        output["status"] = "insufficient_geometry"
        if not any(fits.values()):
            return output
        near = lambda name: min(p[0]**2+p[1]**2 for p in fits[name]["support_points"])
        side = reference_side or min((name for name in fits if fits[name]), key=near)
        fit, other = fits[side], fits["right" if side == "left" else "left"]
        boundary = [p[:2] for p in fit["support_points"]]
        total = min(fit["lengths"][-1], cfg.road_forward_max_m+cfg.road_rear_support_m)
        count = min(200, math.ceil(total/cfg.road_sample_spacing_m))
        candidates, runs, current, previous_other_s = [], [], [], -1.0
        pairing_conflict = False
        for i in range(count+1):
            distance = total*i/count
            p = _path_sample(boundary, fit["lengths"], distance)
            tx, ty = _path_tangent(boundary, fit["lengths"], distance)
            nx, ny = (ty, -tx) if side == "left" else (-ty, tx)
            matches = []
            if other:
                for j, (a, b) in enumerate(zip(other["support_points"], other["support_points"][1:])):
                    dx, dy = b[0]-a[0], b[1]-a[1]
                    length = other["lengths"][j+1]-other["lengths"][j]
                    if (tx*dx+ty*dy)/length < 0.3:
                        continue
                    denominator = nx*dy-ny*dx
                    if abs(denominator) < 1e-9:
                        continue
                    ax, ay = a[0]-p[0], a[1]-p[1]
                    width = (ax*dy-ay*dx)/denominator
                    u = (ax*ny-ay*nx)/denominator
                    if -1e-6 <= u <= 1+1e-6 and not cfg.road_width_min_m <= width <= cfg.road_width_max_m:
                        pairing_conflict = True
                    other_s = other["lengths"][j]+min(1.0, max(0.0, u))*length
                    if (cfg.road_width_min_m <= width <= cfg.road_width_max_m
                            and -1e-6 <= u <= 1+1e-6 and other_s >= previous_other_s-0.03):
                        # Prefer the expected width once measured, otherwise
                        # the first inward boundary crossing on this branch.
                        expected = self._learned_lane_width_m
                        score = abs(width-expected) if expected is not None else width
                        matches.append((score, width, other_s))
            if matches:
                _, width, previous_other_s = min(matches)
                current.append(((p[0]+nx*width/2, p[1]+ny*width/2), width))
            elif current:
                runs.append(current)
                current = []
            candidates.append((p, nx, ny))
        if current:
            runs.append(current)
        runs = [run for run in runs if len(run) >= 2
                and _path_lengths([p for p, _ in run])[-1] >= cfg.road_min_coverage_m]
        inferred = not runs
        if (inferred and reference_side is None and other is not None
                and other["lengths"][-1] > fit["lengths"][-1]):
            # Common support has disappeared: the nearest boundary may now
            # be only a short noisy fragment. Prefer the longer actually
            # observed chain for remembered-width inference, once per frame.
            return self._ordered_road(output, sides,
                                      reference_side="right" if side == "left" else "left")
        if not inferred:
            # Prefer the nearest usable contiguous corridor over a longer far
            # branch; discarded gaps never become implicit connecting lines.
            run = min(runs, key=lambda r: min(p[0]**2+p[1]**2 for p, _ in r))
            center = [p for p, _ in run]
            width = _estimation_median([w for _, w in run])
        else:
            width = (self._learned_lane_width_m if cfg.single_side_from_learned_width
                     else None) or cfg.known_lane_width_m
            output["visibility"] = side+"_only"
            if width == 0:
                output["status"] = "single_side_width_unknown" if other is None else "insufficient_common_range"
                return output
            # A visible but contradictory opposite chain must not be hidden by
            # a width fallback. Missing/too-short chains are handled normally.
            if other is not None and pairing_conflict:
                output["status"] = "invalid_boundary_pairing"
                return output
            if other is not None:
                output["single_side_fallback_reason"] = "insufficient_common_range"
            offset = width*cfg.single_side_offset_fraction
            center = [(p[0]+nx*offset, p[1]+ny*offset) for p, nx, ny in candidates]
        # One corner-cutting pass removes polyline heading steps. Bound each
        # displacement by the local half-width, retaining measured endpoints.
        # This is geometric smoothing, not a calibrated turning-radius planner.
        smooth = [center[0]]
        for a, p, b in zip(center, center[1:], center[2:]):
            q = ((a[0]+2*p[0]+b[0])/4, (a[1]+2*p[1]+b[1])/4)
            smooth.append(q if math.hypot(q[0]-p[0], q[1]-p[1]) <= width/8 else p)
        smooth.append(center[-1])
        if _path_points(smooth) is None or _path_lengths(smooth)[-1] < cfg.road_min_coverage_m:
            output["status"] = "invalid_ordered_geometry"
            return output
        # Normal offsets change arc length, especially on an inner boundary.
        # Resample the resulting centreline by its OWN length for consumers.
        center_lengths = _path_lengths(smooth)
        count = min(200, math.ceil(center_lengths[-1]/cfg.road_sample_spacing_m))
        smooth = [_path_sample(smooth, center_lengths, center_lengths[-1]*i/count) for i in range(count+1)]
        output.update(valid=True, centerline_xy=smooth, lane_width_m=width,
                      x_range_m=[min(p[0] for p in smooth), max(p[0] for p in smooth)],
                      arc_length_m=_path_lengths(smooth)[-1], local_curvature_1pm=None,
                      visibility=side+"_only" if inferred else "both",
                      status="single_side_inferred" if inferred else "observed",
                      confidence=min(f["inlier_fraction"] for f in fits.values() if f)*(0.5 if inferred else 1.0),
                      centerline_source=side+"_boundary_inward_offset" if inferred else "two_boundaries")
        supported = []
        for name, colour in (("left", self.left_colour), ("right", self.right_colour)):
            f = fits[name]
            output[name+"_boundary_xy"] = [p[:2] for p in f["support_points"]] if f else None
            output["boundary_source"][name] = "observed" if f else "absent"
            if f:
                supported.extend((x, y, 0.0, colour, confidence) for x, y, confidence in f["support_points"])
        output["_support_cones"] = supported
        if inferred and self._learned_lane_width_m is not None:
            output.update(single_side_inferred_from_learned_width=True,
                          lane_width_source="last_two_sided_observation",
                          inward_offset_m=width*cfg.single_side_offset_fraction,
                          lane_width_observed_at_s=self._learned_lane_width_stamp_s)
        return output

    def _align_road(self, road, now_s, state_valid):
        center_coeffs = road.pop("_center_coeffs", None)
        road.pop("_boundary_fits", None)
        road.pop("_support_cones", None)
        road["geometry_valid"] = False
        road["near_field"] = {"valid": False, "status": "unavailable",
                              "coverage_x_m": None, "uses_history": road.get("history_used", False),
                              "oldest_observation_age_s": None, "footprint_validated": False}
        source_time = road["timestamp_s"]
        road.update(measurement_timestamp_s=source_time, timestamp_s=None,
                    time_aligned=False, motion_compensation=None)
        if not road["valid"]:
            return road
        if road.get("geometry_encoding") == "ordered_xy":
            return self._align_ordered_road(road, source_time, now_s, state_valid)
        pose, problem = self.motion.pose_between(source_time, now_s)
        if not state_valid:
            problem = "motion_state_unavailable"
        if problem is None:
            points = [self.motion.transform_xy(p, pose) for p in road["centerline_xy"]]
            selected = [i for i, p in enumerate(points)
                        if all(math.isfinite(v) for v in p)
                        and -self.settings.road_rear_support_m <= p[0] <= self.settings.road_forward_max_m]
            center = [points[i] for i in selected]
            if (any(not all(math.isfinite(v) for v in p) for p in points)
                    or len(center) < 2
                    or center[-1][0]-center[0][0] < self.settings.road_min_coverage_m
                    or any(b[0] <= a[0] or abs(b[1]-a[1]) > self.settings.road_max_abs_slope*(b[0]-a[0])
                           for a, b in zip(center, center[1:]))):
                problem = "motion_transformed_geometry_invalid"
            else:
                if center_coeffs is not None:
                    slope = center_coeffs[1]+2*center_coeffs[2]*road["centerline_xy"][selected[0]][0]
                    road["local_curvature_1pm"] = 2*center_coeffs[2]/(1+slope*slope)**1.5
                road["centerline_xy"] = center
                road["x_range_m"] = [center[0][0], center[-1][0]]
                for side in ("left", "right"):
                    key = f"{side}_boundary_xy"
                    if road[key] is not None:
                        boundary = [self.motion.transform_xy(p, pose) for p in road[key]]
                        road[key] = [p for p in boundary
                                     if -self.settings.road_rear_support_m <= p[0] <= self.settings.road_forward_max_m]
                        if any(not all(math.isfinite(v) for v in p) for p in boundary):
                            problem = "motion_transformed_geometry_invalid"
                # Rigid motion preserves curvature at the SAME physical point.
                # If clipping removes that first point, do not relabel its value.
                if selected[0] != 0 and center_coeffs is None:
                    road["local_curvature_1pm"] = None
                road.update(timestamp_s=now_s, time_aligned=True,
                            motion_compensated=pose["interval_s"] > 0,
                            motion_compensation=pose)
                road["geometry_valid"] = problem is None
                boundaries = [road.get(f"{side}_boundary_xy") for side in ("left", "right")]
                if (any(not points or any(b[0] <= a[0] for a, b in zip(points, points[1:]))
                        for points in boundaries)):
                    problem = "near_field_requires_both_boundaries"
                else:
                    lo = max(center[0][0], *(points[0][0] for points in boundaries))
                    hi = min(center[-1][0], *(points[-1][0] for points in boundaries))
                    oldest = road.get("oldest_observation_timestamp_s", source_time)
                    road["near_field"].update(coverage_x_m=[lo, hi],
                                              oldest_observation_age_s=max(0.0, now_s-oldest))
                    if lo > 1e-9 or hi < 0:
                        problem = "near_field_unobserved"
                    elif now_s-oldest > self.settings.history_lifetime_s:
                        problem = "near_field_history_expired"
                    else:
                        road["near_field"].update(valid=True, status="observed_support")
        if problem is not None:
            if (road["geometry_valid"] and (problem == "near_field_unobserved"
                    or (problem == "near_field_requires_both_boundaries"
                        and road.get("single_side_inferred_from_learned_width")))):
                # Retain validated, aligned FORWARD observations separately.
                # Estimator validity stays false; only an explicitly configured
                # planning extension may consume this incomplete support.
                forward = deepcopy(road)
                forward.update(valid=False, status=problem)
                road["forward_geometry"] = forward
            road["near_field"]["status"] = problem
            road.update(valid=False, status=problem, timestamp_s=None,
                        time_aligned=False, motion_compensated=False,
                        centerline_xy=[], left_boundary_xy=None, right_boundary_xy=None,
                        lane_width_m=None, x_range_m=None, local_curvature_1pm=None,
                        confidence=0.0, motion_compensation=None)
        return road

    def _align_ordered_road(self, road, source_time, now_s, state_valid):
        pose, problem = self.motion.pose_between(source_time, now_s)
        if not state_valid:
            problem = "motion_state_unavailable"
        if problem is None:
            center = [self.motion.transform_xy(p, pose) for p in road["centerline_xy"]]
            points = _path_points(center)
            if points is None:
                problem = "motion_transformed_geometry_invalid"
            else:
                lengths = _path_lengths(points)
                projection = _path_project(points, lengths)
                if projection is None:
                    problem = "ordered_branch_unavailable"
                else:
                    # Keep one contiguous local arc around this branch. Body
                    # x may turn back; clipping/sorting by x would break U turns.
                    near_s = projection[1]
                    lo = max(0.0, near_s-self.settings.road_rear_support_m)
                    hi = min(lengths[-1], near_s+self.settings.road_forward_max_m)
                    center = [_path_sample(points, lengths, lo)]
                    center.extend(p for p, s in zip(points, lengths) if lo+1e-6 < s < hi-1e-6)
                    center.append(_path_sample(points, lengths, hi))
                    if hi-lo < self.settings.road_min_coverage_m or _path_points(center) is None:
                        problem = "motion_transformed_geometry_invalid"
                    else:
                        road.update(centerline_xy=center, arc_length_m=_path_lengths(center)[-1],
                                    x_range_m=[min(p[0] for p in center), max(p[0] for p in center)],
                                    timestamp_s=now_s, time_aligned=True, geometry_valid=True,
                                    motion_compensated=pose["interval_s"] > 0, motion_compensation=pose)
                        for side in ("left", "right"):
                            key = side+"_boundary_xy"
                            if road[key] is not None:
                                road[key] = [self.motion.transform_xy(p, pose) for p in road[key]]
                        road["near_field"].update(coverage_s_m=[lo-near_s, hi-near_s],
                            oldest_observation_age_s=now_s-road.get("oldest_observation_timestamp_s", source_time))
                        if road.get("single_side_inferred_from_learned_width"):
                            problem = "near_field_requires_both_boundaries"
                        elif near_s <= 1e-6 or near_s >= lengths[-1]-1e-6:
                            problem = "near_field_unobserved"
                        elif road["near_field"]["oldest_observation_age_s"] > self.settings.history_lifetime_s:
                            problem = "near_field_history_expired"
                        else:
                            road["near_field"].update(valid=True, status="observed_support")
        if problem is not None:
            if road["geometry_valid"] and problem in ("near_field_unobserved", "near_field_requires_both_boundaries"):
                forward = deepcopy(road)
                forward.update(valid=False, status=problem)
                road["forward_geometry"] = forward
            road["near_field"]["status"] = problem
            road.update(valid=False, status=problem, timestamp_s=None, time_aligned=False,
                        centerline_xy=[], left_boundary_xy=None, right_boundary_xy=None,
                        lane_width_m=None, x_range_m=None, arc_length_m=None, confidence=0.0)
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
        continuous = self.settings.history_propagation == "continuous"
        if not state["valid"]:
            self._clear_road_history()
            road = self.road(observations.get("cone_detections"), stamps.get("cone_detections"),
                             ages.get("cone_detections"))
        else:
            if continuous:
                if self.settings.continuous_road_output:
                    self._advance_prediction_distance(now_s)
                self._advance_history(now_s)
            road = self._history_road(observations.get("cone_detections"), stamps.get("cone_detections"),
                                      ages.get("cone_detections"), now_s)
        observed = (self._observation_road if road["status"] != "unavailable"
                    and self._observation_road is not None else road)
        current_frame = {"timestamp_s": road["timestamp_s"],
                         "source_age_s": ages.get("cone_detections"),
                         "fit_valid": observed["valid"] and observed["visibility"] == "both",
                         "status": observed["status"], "visibility": observed["visibility"],
                         "boundary_diagnostics": deepcopy(observed["boundary_diagnostics"])}
        failure = road["status"]
        if (self.settings.continuous_road_output and self._failure_since_s is not None
                and self._trusted_road is not None and road["valid"]
                and road["timestamp_s"] == self._trusted_road["timestamp_s"]):
            # Re-presenting the old successful camera batch after an outage is
            # not recovery. Only a new accepted camera epoch resets the hold.
            road.update(valid=False, status="camera_frame_not_advanced", centerline_xy=[],
                        left_boundary_xy=None, right_boundary_xy=None, lane_width_m=None,
                        x_range_m=None, local_curvature_1pm=None, confidence=0.0)
            failure = road["status"]
        if (self.settings.continuous_road_output and self._prediction_expired_source_s is not None
                and road["timestamp_s"] is not None
                and road["timestamp_s"] <= self._prediction_expired_source_s):
            road.update(valid=False, status="prediction_deadline_expired", centerline_xy=[],
                        left_boundary_xy=None, right_boundary_xy=None, lane_width_m=None,
                        x_range_m=None, local_curvature_1pm=None, confidence=0.0)
            failure = road["status"]
        trusted_candidate = (deepcopy(road) if road["valid"] and
            (road["visibility"] == "both" or self.settings.road_geometry == "ordered_xy"
             and road.get("single_side_inferred_from_learned_width")) else None)
        road = self._align_road(road, now_s, state["valid"])
        road.update(current_frame=current_frame, degraded=False, prediction_status="not_used",
                    prediction_elapsed_s=None, prediction_age_s=None,
                    prediction_remaining_s=None, recommended_speed_scale=1.0 if road["valid"] else 0.0)
        if road["valid"]:
            if (self.settings.continuous_road_output and
                    (self._trusted_road is None or
                     trusted_candidate["timestamp_s"] != self._trusted_road["timestamp_s"])):
                self._prediction_motion_stamp = now_s
                self._prediction_distance_m = road["motion_compensation"]["travelled_distance_m"]
                self._prediction_pose = deepcopy(road["motion_compensation"])
                self._prediction_expired_source_s = None
            self._trusted_road = trusted_candidate
            self._failure_since_s = None
        elif (road.get("forward_geometry") or {}).get("single_side_inferred_from_learned_width"):
            # Fresh one-sided geometry is a new observation each frame, not
            # a dead-reckoned hold. Legacy y(x) only preserves width here;
            # ordered XY also retains the visible chain for bounded prediction.
            road["prediction_status"] = "fresh_single_side_geometry"
            if self.settings.road_geometry == "ordered_xy" and trusted_candidate is not None:
                forward = road["forward_geometry"]
                if self._trusted_road is None or trusted_candidate["timestamp_s"] != self._trusted_road["timestamp_s"]:
                    self._prediction_motion_stamp = now_s
                    self._prediction_distance_m = forward["motion_compensation"]["travelled_distance_m"]
                    self._prediction_pose = deepcopy(forward["motion_compensation"])
                    self._prediction_expired_source_s = None
                self._trusted_road, self._failure_since_s = trusted_candidate, None
        elif state["valid"] and (failure in self.QUALITY_FAILURES or
                (self.settings.continuous_road_output and road["status"] in
                 self.CAMERA_GAP_FAILURES + ("camera_frame_not_advanced",))):
            predicted, reason = self._predict_road(current_frame, now_s)
            if predicted is not None:
                road = predicted
            else:
                road["prediction_status"] = reason
                # Continuous mode keeps the (still conflict-checked) history;
                # only the trusted-corridor prediction ends here.
                if continuous:
                    if self.settings.continuous_road_output and self._trusted_road is not None:
                        self._prediction_expired_source_s = self._trusted_road["timestamp_s"]
                        self._prediction_motion_stamp = None
                    self._trusted_road, self._failure_since_s = None, None
                else:
                    self._clear_road_history()
        elif road["status"] == "near_field_unobserved":
            # Keep validated forward observations for near-field reconstruction,
            # but never use an incomplete startup corridor as a driving fallback.
            self._trusted_road = None
            self._failure_since_s = None
            self._prediction_motion_stamp = None
        elif continuous and road["status"] == "prediction_deadline_expired":
            # Keep measured near support for a NEW camera frame, but this old
            # camera epoch cannot re-arm an expired prediction on its own.
            self._trusted_road, self._failure_since_s = None, None
            self._prediction_motion_stamp = None
        elif continuous and road["status"] in self.CAMERA_GAP_FAILURES:
            # Camera gap: no output this step, but the history keeps moving with
            # wheel/gyro so the near field is still covered when cones return.
            self._trusted_road = None
            self._failure_since_s = None
        else:
            self._clear_road_history()
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
             source_timeout_s=None, mvp=False):
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
        if ((road.get("visibility") != "both" and not inferred_single_side)
                or road.get("time_aligned") is not True
                or state.get("speed_valid") is not True or state.get("yaw_rate_valid") is not True):
            return fail("centerline_or_motion_unavailable")
        if any(obj.get("frame_id") != self.frame_id or not finite_number(obj.get("timestamp_s"))
               or abs(obj["timestamp_s"]-now_s) > 1e-6 for obj in (road, state)):
            return fail("centerline_frame_or_time_mismatch")
        if (not finite_number(state.get("speed_mps")) or state["speed_mps"] < 0
                or not finite_number(state.get("yaw_rate_rps"))):
            return fail("invalid_motion_state")
        ages = {"road": road.get("source_age_s"), **(state.get("source_age_s") or {})}
        diag["source_ages_s"], diag["source_age_limits_s"] = dict(ages), {}
        deadlines = [cfg.reference_lifetime_s]
        for name in ("road", "wheel_speed", "imu_angular_velocity"):
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
    reference_hold_max_s: float = 1.0
    reference_hold_max_distance_m: float = 0.25

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("control.enabled must be bool")
        if not isinstance(self.robustness_enabled, bool):
            raise ValueError("control.robustness_enabled must be bool")
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
                     "end_straight_max_time_s"):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"control.{name} must be finite and positive")
        for name in ("max_distance_m", "max_run_time_s", "end_straight_distance_m", "mvp_error_filter_tau_s"):
            if not finite_number(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"control.{name} must be finite and nonnegative (0 disables)")
        if (not finite_number(self.mvp_steering_slew_rate_per_s)
                or self.mvp_steering_slew_rate_per_s < 0):
            raise ValueError("control.mvp_steering_slew_rate_per_s must be finite and nonnegative")
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
    fusion_enabled: bool = True
    fusion_window_s: float = 1.0
    fusion_max_samples: int = 21
    fusion_spacing_m: float = 0.10
    fusion_memory_weight: float = 1.0
    fusion_rear_support_m: float = 0.30

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("shadow.enabled must be boolean")
        if not isinstance(self.fusion_enabled, bool):
            raise ValueError("shadow.fusion_enabled must be boolean")
        if type(self.fusion_max_samples) is not int or not 1 <= self.fusion_max_samples <= 128:
            raise ValueError("shadow.fusion_max_samples must be an integer in [1,128]")
        if not isinstance(self.sample_count, int) or isinstance(self.sample_count, bool) or self.sample_count < 8:
            raise ValueError("shadow.sample_count must be an integer >=8")
        for name, value in vars(self).items():
            if name in ("enabled", "sample_count", "fusion_enabled", "fusion_max_samples"):
                continue
            if not finite_number(value) or value < 0:
                raise ValueError("shadow."+name+" must be finite and nonnegative")
        for name in ("rate_hz", "length_m", "path_spacing_m", "score_min_length_m",
                     "search_half_width_m", "body_front_m", "body_rear_m", "body_width_m",
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


def _shadow_lengths(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1]+math.dist(a, b))
    return lengths


def _shadow_at(points, lengths, s):
    """Piecewise linear arc chart with explicitly inferred tangent tails."""
    if len(points) == 1:
        return points[0][0]+s, points[0][1]
    if s <= 0:
        i = 0
    elif s >= lengths[-1]:
        i = len(points)-2
    else:
        i = next(i for i in range(len(points)-1) if s <= lengths[i+1])
    u = (s-lengths[i])/(lengths[i+1]-lengths[i])
    return tuple(points[i][k]+u*(points[i+1][k]-points[i][k]) for k in (0, 1))


class ShadowFrame:
    """Local two-dimensional Frenet chart; no monotonic-body-x restriction."""
    def __init__(self, points):
        self.points = points
        self.lengths = _shadow_lengths(points)
        self.end = self.lengths[-1]

    def at(self, s):
        p = _shadow_at(self.points, self.lengths, s)
        a = _shadow_at(self.points, self.lengths, s-0.04)
        b = _shadow_at(self.points, self.lengths, s+0.04)
        length = math.dist(a, b)
        tangent = ((b[0]-a[0])/length, (b[1]-a[1])/length) if length > 1e-9 else (1.0, 0.0)
        return p, tangent, (-tangent[1], tangent[0])

    def project(self, point, heading=0.0):
        options = []
        if len(self.points) == 1:
            s = point[0]-self.points[0][0]
            return s, point[1]-self.points[0][1]
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            length = self.lengths[i+1]-self.lengths[i]
            tx, ty = (b[0]-a[0])/length, (b[1]-a[1])/length
            u = ((point[0]-a[0])*tx+(point[1]-a[1])*ty)/length
            # The first/last rays are inferred chart support, not hard boundaries.
            u = max(0.0, u) if i else u
            u = min(1.0, u) if i < len(self.points)-2 else u
            q = (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
            angle = wrap_angle(math.atan2(ty, tx)-heading)
            cost = math.dist(q, point)**2+0.02*angle**2
            options.append((cost, self.lengths[i]+u*length))
        s = min(options)[1]
        p, _, n = self.at(s)
        return s, (point[0]-p[0])*n[0]+(point[1]-p[1])*n[1]


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
                "reference_origin": reference_origin, "reference": [frame.at(s0+horizon*i/40)[0] for i in range(41)],
                "reference_observed_range_s_m": [0.0, frame.end], "start_s_m": s0, "start_d_m": d0,
                "turn_angle_rad": turn, "entry_heading_weight": entry_weight, "preparation_offset_m": preparation,
                "speed_mps": speed, "speed_available": finite_number(v), "hold_time_s": hold,
                "handoff_estimate_s": handoff, "requested_rate_hz": cfg.rate_hz,
                "check_length_m": check, "score_length_m": score_length, "generated_length_m": horizon,
                "control_read_length_m": check+preview, "known_boundary_sides": list(boundaries),
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



class ShadowWorker:
    """Latest-only transport thread and separate CPU process; never block control."""
    def __init__(self, source_file):
        self.source_file = source_file
        self.jobs, self.results = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.process = None
        self.thread = threading.Thread(target=self._serve, name="shadow-transport", daemon=True)
        self.thread.start()

    @staticmethod
    def _replace(mailbox, value):
        try:
            mailbox.get_nowait()
        except queue.Empty:
            pass
        try:
            mailbox.put_nowait(value)
        except queue.Full:
            pass

    def submit(self, job):
        if not self.closed.is_set():
            self._replace(self.jobs, job)

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def _serve(self):
        try:
            process = subprocess.Popen([sys.executable, self.source_file, "--shadow-worker"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, encoding="utf-8", bufsize=1,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            self.process = process
            if self.closed.is_set():
                process.terminate()
                return
            while not self.closed.is_set():
                try:
                    job = self.jobs.get(timeout=0.1)
                except queue.Empty:
                    continue
                process.stdin.write(json.dumps(job, allow_nan=False)+"\n")
                process.stdin.flush()
                line = process.stdout.readline()
                if not line:
                    raise RuntimeError("shadow process exited")
                result = json.loads(line)
                result["wire_json"] = line.rstrip()
                result["received_monotonic_s"] = time.monotonic()
                result["submitted_monotonic_s"] = job["submitted_monotonic_s"]
                self._replace(self.results, result)
        except Exception as error:
            self._replace(self.results, {"status": "TRANSPORT_ERROR", "error": str(error)})
        finally:
            if self.process is not None:
                if self.process.poll() is None:
                    self.process.terminate()
                try:
                    self.process.communicate(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.communicate()

    def close(self):
        self.closed.set()
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()


@dataclass
class PlannerControlSettings:
    """Opt-in consumer of the fresh lattice; baseline remains the default.

    There is no fixed source-trajectory TTL or steering/curvature admission
    threshold. Fresh feedback, rolling known-boundary checks and remaining
    preview coverage decide whether to use the path or this cycle's baseline.
    """
    enabled: bool = False

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("planner_control.enabled must be boolean")


class PlannerExecutionManager:
    """Geometry-only high-rate adapter; never publishes actions or requests Stop.

    A result's input pose/epoch survives arbitrarily long computation. The
    short execution reference is rebuilt at each CURRENT state timestamp;
    source age is retained separately and cannot be renewed by reception.
    Only the selected path is checked, without a controller rollout or cost.
    """
    def __init__(self, frame_id="base_link"):
        self.frame_id = frame_id
        self.clear()

    def clear(self, reason="waiting_for_planner"):
        self.active = None
        self.reason = reason
        self.last_job_id = None
        self.last_result_stamp_s = None
        self.last_result_status = None
        self.diagnostics = {}

    def accept(self, result):
        # A completed planning failure immediately selects baseline, rather
        # than silently continuing the previous successful trajectory.
        self.active = None
        self.last_job_id = result.get("job_id")
        self.last_result_stamp_s = result.get("input_stamp_s")
        self.last_result_status = result.get("status")
        self.reason = "planner_"+str(result.get("status", "missing_status")).lower()
        if result.get("status") != "VALID":
            return
        anchor = result.get("execution_anchor") or {}
        pose = anchor.get("pose")
        selected = result.get("selected") or {}
        path = selected.get("path")
        if (not isinstance(pose, (list, tuple)) or len(pose) != 3
                or not all(finite_number(v) for v in pose)
                or not isinstance(anchor.get("odometry_epoch"), int)
                or not finite_number(self.last_result_stamp_s)
                or not isinstance(path, (list, tuple)) or len(path) < 2
                or any(not isinstance(p, (list, tuple)) or len(p) != 3
                       or not all(finite_number(v) for v in p) for p in path)
                or any(math.hypot(b[0]-a[0], b[1]-a[1]) <= 1e-9
                       for a, b in zip(path, path[1:]))):
            self.reason = "planner_invalid_execution_payload"
            return
        self.active = {"path": [tuple(p) for p in path], "pose": list(pose),
            "epoch": anchor["odometry_epoch"], "stamp_s": self.last_result_stamp_s,
            "measurement_stamp_s": result.get("measurement_stamp_s"), "progress_m": 0.0}
        self.reason = "planner_ready"

    def select(self, road, state, pose, epoch, cfg, preview_m, target_speed_mps,
               control_period_s, execution_lifetime_s):
        started = time.perf_counter()
        now = state.get("timestamp_s")
        self.diagnostics = {"requested_source": "planner", "mode": "baseline",
            "fallback": True, "reason": self.reason, "timestamp_s": now,
            "job_id": self.last_job_id, "planner_status": self.last_result_status,
            "input_stamp_s": self.last_result_stamp_s,
            "source_age_s": now-self.last_result_stamp_s if finite_number(now)
                and finite_number(self.last_result_stamp_s) else None}

        def fallback(reason):
            self.diagnostics.update(reason=reason, elapsed_s=time.perf_counter()-started)
            return None

        active = self.active
        if active is None:
            return fallback(self.reason)
        if epoch != active["epoch"]:
            self.active = None
            self.reason = "motion_epoch_changed"
            return fallback(self.reason)
        if (state.get("frame_id") != self.frame_id or state.get("speed_valid") is not True
                or state.get("yaw_rate_valid") is not True or not finite_number(now)
                or now < active["stamp_s"] or not all(finite_number(v) for v in pose)):
            return fallback("execution_motion_unavailable")
        points = _shadow_pose_points(active["path"], active["pose"], pose)
        arc = _path_lengths(points)
        # Remove previously passed arc BEFORE projecting: a returning leg may
        # remain ahead; deleting by x sign would destroy genuine U bends.
        start = active["progress_m"]
        local = [_path_sample(points, arc, start)]
        local.extend(p for p, s in zip(points, arc) if s > start+1e-6)
        if len(local) < 2:
            return fallback("preview_coverage_exhausted")
        local_arc = _path_lengths(local)
        nearest = _path_project(local, local_arc)
        if nearest is None:
            return fallback("execution_branch_unavailable")
        progress = min(arc[-1], start+nearest[1])
        active["progress_m"] = progress
        remaining = arc[-1]-progress
        speed = abs(state["speed_mps"])
        needed = preview_m+speed*control_period_s
        self.diagnostics.update(progress_m=progress, remaining_length_m=remaining,
            preview_m=preview_m, required_coverage_m=needed,
            remaining_preview_time_s=max(0.0, remaining-preview_m)/speed if speed > 1e-9 else None,
            measurement_stamp_s=active["measurement_stamp_s"], odometry_epoch=epoch,
            current_pose=list(pose), execution_lifetime_s=execution_lifetime_s)
        if remaining+1e-9 < needed:
            return fallback("preview_coverage_exhausted")
        cropped = [_path_sample(points, arc, progress)]
        cropped.extend(p for p, s in zip(points, arc) if s > progress+1e-6)
        lengths = _path_lengths(cropped)
        # Use exactly the same geometric reader as the actual PID, including
        # its tangent smoothing beyond the preview. This is NOT a rollout.
        path = {"type": "ORDERED_CENTERLINE", "independent_variable": "s_m",
                "samples_xy": cropped, "range": [0.0, lengths[-1]], "preview_m": preview_m}
        try:
            geometry = control_path_geometry(path)
        except (ValueError, TypeError, ArithmeticError) as error:
            return fallback("execution_geometry: "+str(error))
        # Projecting again inside the controller may select a later segment.
        # Check coverage at ITS actual preview position as well.
        if lengths[-1]-geometry["closest_s_m"]+1e-9 < needed:
            return fallback("preview_coverage_exhausted")
        check = min(lengths[-1], max(needed,
            geometry["preview_s_m"]+0.15))  # Existing PID tangent window.
        sources = road.get("boundary_source") or {}
        boundaries = {side: _shadow_points(road.get(side+"_boundary_xy"))
            for side in ("left", "right") if sources.get(side) in ("observed", "observed_with_history")
            and road.get(side+"_boundary_xy")}
        if boundaries and (road.get("time_aligned") is not True
                or road.get("frame_id") != self.frame_id
                or not finite_number(road.get("timestamp_s"))
                or abs(road["timestamp_s"]-now) > 1e-6):
            return fallback("known_boundary_alignment_unavailable")
        sweep = []
        for i, p in enumerate(cropped):
            heading = _path_tangent(cropped, lengths, lengths[i], window=cfg.path_spacing_m/2)
            sweep.append((*p, math.atan2(heading[1], heading[0])))
        # Include the actual vehicle footprint and the geometric connection to
        # the closest reference point when tracking has a lateral error.
        collision = _shadow_collision([(0.0, 0.0, 0.0)], 0.0, boundaries, cfg)
        connector = math.hypot(*cropped[0])
        if connector > 1e-9:
            sweep.insert(0, (0.0, 0.0, 0.0))
        collision = collision or _shadow_collision(sweep, check+connector, boundaries, cfg)
        self.diagnostics.update(check_length_m=check, connector_length_m=connector,
            known_boundary_sides=list(boundaries), known_boundaries_xy=boundaries,
            collision=collision, remaining_path_xy=cropped,
            checked_through_preview=True)
        if collision:
            return fallback("known_boundary_collision")
        self.diagnostics.update(mode="planner", fallback=False, reason="planner_tracking",
            elapsed_s=time.perf_counter()-started)
        return {"schema_version": "planning_reference_v1", "planner_version": "lattice_execution",
            "trajectory_id": self.last_job_id, "timestamp_s": now, "valid_for_s": execution_lifetime_s,
            "frame_id": self.frame_id, "vehicle_reference_point": "cg_ground_projection",
            "valid": True, "status": "VALID", "reason": "planner_tracking", "path": path,
            "target_speed_mps": target_speed_mps, "stop_requested": False, "stop_reason": None,
            "simulation_only": False, "original_input_stamp_s": active["stamp_s"]}


@dataclass
class LocalGridSettings:
    """Forward-only 3 m x 3 m map. No clustering/confidence admission gates."""
    enabled: bool = False
    resolution_m: float = 0.05
    history_s: float = 0.3
    inflation_radius_m: float = 0.10

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("local_grid.enabled must be boolean")
        if (not finite_number(self.resolution_m) or not 0.02 <= self.resolution_m <= 0.5
                or abs(3/self.resolution_m-round(3/self.resolution_m)) > 1e-6):
            raise ValueError("local_grid.resolution_m must divide 3 m, in [0.02, 0.5]")
        if not finite_number(self.history_s) or not 0 <= self.history_s <= 0.5:
            raise ValueError("local_grid.history_s must be in [0, 0.5] seconds")
        if not finite_number(self.inflation_radius_m) or not 0 <= self.inflation_radius_m <= 1.5:
            raise ValueError("local_grid.inflation_radius_m must be in [0, 1.5] metres")


def _local_grid_ray(origin, endpoint, resolution, cells):
    """Clip to the forward rectangle, then visit cells using Bresenham.

    Endpoints outside the rectangle still provide free-space evidence INSIDE it.
    The ray starts at the acquisition-time lidar origin, moved with its endpoint.
    """
    xmin, xmax, ymin, ymax = 0.0, 3.0-1e-9, -1.5, 1.5-1e-9
    x, y = origin
    dx, dy = endpoint[0]-x, endpoint[1]-y
    begin, end = 0.0, 1.0
    for p, q in ((-dx, x-xmin), (dx, xmax-x), (-dy, y-ymin), (dy, ymax-y)):
        if p == 0:
            if q < 0:
                return
        elif p < 0:
            begin = max(begin, q/p)
        else:
            end = min(end, q/p)
        if begin > end:
            return
    ix = min(cells-1, max(0, math.floor((x+begin*dx)/resolution)))
    iy = min(cells-1, max(0, math.floor((y+begin*dy+1.5)/resolution)))
    ex = min(cells-1, max(0, math.floor((x+end*dx)/resolution)))
    ey = min(cells-1, max(0, math.floor((y+end*dy+1.5)/resolution)))
    ax, ay = abs(ex-ix), abs(ey-iy)
    sx, sy = (1 if ix < ex else -1), (1 if iy < ey else -1)
    error = ax-ay
    while True:
        yield iy*cells+ix
        if ix == ex and iy == ey:
            return
        twice = 2*error
        if twice > -ay:
            error -= ay
            ix += sx
        if twice < ax:
            error += ax
            iy += sy


class LocalOccupancyGrid:
    """Short scan history reprojected into the CURRENT body frame on each render.

    Unknown=-1, free=0, hit=100. Newer observed free space clears older hits;
    within one scan hits take precedence over clearing rays. Unobserved cells
    lose history back to UNKNOWN, never automatically to free. No object tracker,
    SLAM, per-beam deskew, car-size expansion or actuator output is implemented.
    """
    def __init__(self, settings):
        self.settings = settings
        self.cells = round(3/settings.resolution_m)
        radius = settings.inflation_radius_m/settings.resolution_m
        reach = math.ceil(radius)
        self.inflation_offsets = [(dx, dy) for dy in range(-reach, reach+1)
                                  for dx in range(-reach, reach+1)
                                  if dx*dx+dy*dy <= radius*radius+1e-9]
        self.clear()

    def clear(self):
        self.scans = []
        self.last_stamp_s = None

    def add_scan(self, stamp_s, origin, points):
        if not finite_number(stamp_s) or not all(finite_number(v) for v in origin):
            return False
        if self.last_stamp_s is not None and stamp_s <= self.last_stamp_s:
            return False  # Cached control/timer reads are not new measurements.
        rays = [(float(p[0]), float(p[1])) for p in points
                if len(p) >= 2 and finite_number(p[0]) and finite_number(p[1])]
        self.last_stamp_s = stamp_s
        self.scans.append((stamp_s, tuple(origin), rays))
        self.scans = [s for s in self.scans if stamp_s-s[0] <= self.settings.history_s+1e-9][-16:]
        return True

    def empty(self, now_s, reason):
        unknown = [-1]*(self.cells*self.cells)
        return {"data": unknown, "inflated_data": list(unknown), "timestamp_s": now_s,
                "valid": False, "time_aligned": False, "status": reason,
                "measurement_timestamp_s": None, "source_age_s": None,
                "scan_count": 0, "occupied_cells": 0}

    def render(self, now_s, motion):
        if not self.scans:
            return self.empty(now_s, "NO_SCAN")
        latest = self.scans[-1]
        latest_pose, problem = motion.pose_between(latest[0], now_s)
        aligned = problem is None
        # Without motion history we can still inspect a correctly timestamped
        # SINGLE scan while parked. It is explicitly not a current-time map.
        scans = ([s for s in self.scans if now_s-s[0] <= self.settings.history_s+1e-9
                  or s is latest] if aligned else [latest])
        data, used = [-1]*(self.cells*self.cells), 0
        for stamp, origin, points in scans:
            if aligned:
                pose, error = (latest_pose, None) if stamp == latest[0] else motion.pose_between(stamp, now_s)
                if error is not None:
                    continue
                origin = motion.transform_xy(origin, pose)
                points = [motion.transform_xy(point, pose) for point in points]
            free, occupied = set(), set()
            for endpoint in points:
                free.update(_local_grid_ray(origin, endpoint, self.settings.resolution_m, self.cells))
                if 0 <= endpoint[0] < 3 and -1.5 <= endpoint[1] < 1.5:
                    col = min(self.cells-1, math.floor(endpoint[0]/self.settings.resolution_m))
                    row = min(self.cells-1, math.floor((endpoint[1]+1.5)/self.settings.resolution_m))
                    occupied.add(row*self.cells+col)
            for index in free:
                data[index] = 0
            for index in occupied:
                data[index] = 100
            used += 1
        inflated = list(data)
        hits = [index for index, value in enumerate(data) if value == 100]
        for index in hits:
            row, col = divmod(index, self.cells)
            for dx, dy in self.inflation_offsets:
                x, y = col+dx, row+dy
                if 0 <= x < self.cells and 0 <= y < self.cells:
                    target = y*self.cells+x
                    if inflated[target] != 100:
                        inflated[target] = 90  # Margin cost, distinct from measured hits.
        return {"data": data, "inflated_data": inflated,
                "timestamp_s": now_s if aligned else latest[0],
                "measurement_timestamp_s": latest[0], "source_age_s": max(0.0, now_s-latest[0]),
                "valid": True, "time_aligned": aligned,
                "status": "ALIGNED" if aligned else "SCAN_ONLY_NO_MOTION",
                "scan_count": used, "occupied_cells": len(hits)}


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

        grid_values = {}
        for name, default in vars(LocalGridSettings()).items():
            parameter_name = f"local_grid.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            grid_values[name] = self.get_parameter(parameter_name).value
        self.local_grid_settings = LocalGridSettings(**grid_values)
        self.local_grid = LocalOccupancyGrid(self.local_grid_settings)
        self.local_grid_output = None
        self._local_grid_motion_reset_id = self.motion_history.reset_id
        self._lidar_grid_origin = None

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
        shadow_values = {}
        for name, default in vars(ShadowSettings()).items():
            parameter_name = f"shadow.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            shadow_values[name] = self.get_parameter(parameter_name).value
        self.shadow_settings = ShadowSettings(**shadow_values)
        self.declare_parameter("planner_control.enabled", False, ParameterDescriptor(read_only=True))
        self.planner_control_settings = PlannerControlSettings(
            enabled=self.get_parameter("planner_control.enabled").value)
        if self.planner_control_settings.enabled and (
                not self.shadow_settings.enabled or not self.control_settings.enabled
                or self.control_settings.mode != "mvp"
                or self.planning_settings.algorithm != "road_centerline"
                or self.policy_update_mode != "timer"):
            raise ValueError("Planner control requires shadow, timer updates and MVP road_centerline control")
        self.planner_execution = PlannerExecutionManager(self.policy_frame_id)
        self.planner_execution_publisher = self.create_publisher(String, "planner_execution", 1)
        self._last_execution_mode = None
        if self.control_settings.enabled:
            if (self.estimation_settings.road_geometry == "ordered_xy"
                    and self.planning_settings.algorithm != "road_centerline"):
                raise ValueError("Ordered XY geometry requires planning.algorithm=road_centerline")
            if (self.control_settings.end_straight_distance_m > 0
                    and self.planning_settings.algorithm != "road_centerline"):
                raise ValueError("End straight requires planning.algorithm=road_centerline")
            if (self.planning_settings.algorithm == "road_centerline"
                    and self.control_settings.mode != "mvp"):
                raise ValueError("Road centerline following requires control.mode=mvp")
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
        self.control_reference_manager = ControlReferenceManager(self.control_settings)
        self.last_control_tracking_mode = None
        self.distance_limiter = RunDistanceLimiter(self.control_settings.max_distance_m, self.control_settings.max_dt_s)
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False
        self._end_straight_start_distance_m = None
        self._end_straight_started_at_s = None

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
        self.declare_parameter("visualization.enabled", True, ParameterDescriptor(read_only=True))
        self.trajectory_enabled = self.get_parameter("visualization.enabled").value
        self.trajectory_publisher = self.create_publisher(Path, "policy_trajectory", 1)
        self._trajectory_reference = None
        self._trajectory_last_publish_s = -math.inf
        self._publish_trajectory(self.get_clock().now().nanoseconds, force=True)
        self.shadow_generation = self.shadow_job_id = 0
        self.shadow_last_submit_s = -math.inf
        self.shadow_handoff_estimate_s = self.shadow_settings.handoff_allowance_s
        self.shadow_last_result_s = None
        self.shadow_road_history = []
        self.shadow_pose = [0.0, 0.0, 0.0]
        self.shadow_pose_stamp_s = None
        self.shadow_odometry_epoch = 0
        self.shadow_previous_reference = None
        self.shadow_collection_events = []
        self.shadow_collection_reset_id = 0
        self.shadow_worker = None
        self.shadow_timer = None
        if self.shadow_settings.enabled:
            self.shadow_path_publisher = self.create_publisher(Path, "shadow_trajectory", 1)
            self.shadow_prefix_publisher = self.create_publisher(Path, "shadow_checked_prefix", 1)
            self.shadow_reference_publisher = self.create_publisher(Path, "shadow_reference", 1)
            self.shadow_diagnostics_publisher = self.create_publisher(String, "shadow_planning", 1)
            self.shadow_worker = ShadowWorker(__file__)
            self.shadow_timer = self.create_timer(1/self.supervision_rate_hz, self._shadow_poll)
            self._shadow_clear()

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
        self.local_grid_timer = None
        if self.local_grid_settings.enabled:
            self.local_grid_publisher = self.create_publisher(OccupancyGrid, "local_occupancy_grid", reliable_one)
            self.local_inflated_grid_publisher = self.create_publisher(OccupancyGrid, "local_inflated_grid", reliable_one)
            self.local_grid_status_publisher = self.create_publisher(String, "local_grid_status", reliable_one)
            # Independent perception/display update, including in zero-action
            # state. This timer never calls run_policy_step or enables motion.
            self.local_grid_timer = self.create_timer(0.1, self._local_grid_tick, clock=self.steady_clock)
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
        self._lidar_grid_origin = (float(offsets[0]), float(offsets[1]))
        return {"points_xyz": points, "scan_indices": indices, "frame_id": self.policy_frame_id}

    def _local_grid_tick(self):
        """Publish perception only; map failures do not enter the control FSM."""
        try:
            self._local_grid_update()
        except Exception as error:
            self._warn("local_grid_publish", f"Local grid publication unavailable: {error}")

    def _local_grid_update(self):
        started = time.perf_counter()
        now, ros_now = self._times()
        now_s = ros_now/1e9
        try:
            if self.motion_history.reset_id != self._local_grid_motion_reset_id:
                self.local_grid.clear()
                self._local_grid_motion_reset_id = self.motion_history.reset_id
            if not self._fresh("lidar_cartesian", now, ros_now) or self._lidar_grid_origin is None:
                self.local_grid.clear()
                result = self.local_grid.empty(now_s, "SCAN_OR_TF_UNAVAILABLE")
            else:
                observation = self.observations["lidar_cartesian"]
                self.local_grid.add_scan(observation.stamp_ns/1e9, self._lidar_grid_origin,
                                         observation.value["points_xyz"])
                result = self.local_grid.render(now_s, self.motion_history)
                result["source_age_s"] = self._age(observation, now, ros_now)
        except Exception as error:
            self.local_grid.clear()
            result = self.local_grid.empty(now_s, "MAP_ERROR")
            self._warn("local_grid", f"Local grid unavailable: {error}")
        self.local_grid_output = result
        for publisher, field in ((self.local_grid_publisher, "data"),
                                 (self.local_inflated_grid_publisher, "inflated_data")):
            message = OccupancyGrid()
            message.header.frame_id = self.policy_frame_id
            message.header.stamp = Time(nanoseconds=round(result["timestamp_s"]*1e9)).to_msg()
            message.info.map_load_time = message.header.stamp
            message.info.resolution = float(self.local_grid_settings.resolution_m)
            message.info.width = message.info.height = self.local_grid.cells
            message.info.origin.position.y = -1.5
            message.info.origin.orientation.w = 1.0
            message.data = result[field]
            publisher.publish(message)
        result["update_time_s"] = time.perf_counter()-started
        status = {key: value for key, value in result.items() if key not in ("data", "inflated_data")}
        status.update(frame_id=self.policy_frame_id, resolution_m=self.local_grid_settings.resolution_m,
                      origin_xy_m=[0.0, -1.5], size_m=[3.0, 3.0],
                      inflation_radius_m=self.local_grid_settings.inflation_radius_m,
                      inflation_includes_vehicle_size=False, deskewed=False)
        self.local_grid_status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))

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
        problem = self._control_health_problem(now, ros_now) or self._control_problem(now, ros_now)
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
            if self._end_straight_start_distance_m is not None:
                # Terminal mode never resumes cone following, even if cones
                # return. The independent supervisor stops it at its budget.
                result = (self.control_settings.end_straight_drive_normalized,
                          0.0, None, None, self.distance_limiter.distance_m)
            else:
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
        problem = self._control_health_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return
        problem = self._control_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return
        self._publish_actions(drive, steer, pan)
        if hasattr(self, "planner_execution_publisher"):
            self._publish_planner_execution()
        # Offer AFTER the baseline action, never on its calculation/exception path.
        # This is a diagnostic side channel; failures cannot change the action/FSM.
        if getattr(self, "shadow_worker", None) is not None:
            try:
                self._shadow_offer()
            except Exception as error:
                self._warn("shadow_submit", f"Shadow submission unavailable: {error}")
        if hasattr(self, "trajectory_publisher"):
            self._publish_trajectory(ros_now)
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
                motion_history=self.motion_history)
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
            planner_type = (RoadCenterlinePlanner if self.planning_settings.algorithm == "road_centerline"
                            else CenterlinePlanner)
            self.planner = planner_type(self.planning_settings, self.policy_frame_id)
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
            planner_requested = bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False))
            planner_reference = self._select_planner_reference(estimates) if planner_requested else None
            # Delayed camera data may predate the reset motion history at start.
            # Bounded neutral priming is allowed only BEFORE the first valid
            # control step; any fault after tracking requires explicit resume.
            if (planner_reference is None and not self.planning_output["valid"] and not self.control_has_run
                    and estimates["state"]["valid"]
                    and estimates["road"]["status"] == "missing_motion_history"
                    and policy_elapsed_s < control.startup_grace_s):
                self.control_diagnostics = {"valid": False, "stop_requested": False,
                                            "reason": "priming_motion_history"}
                return 0.0, 0.0, None, None, None
            selected_reference = planner_reference if planner_reference is not None else self.planning_output
            if planner_reference is not None and hasattr(self, "control_reference_manager"):
                self.control_reference_manager.clear()
            if planner_reference is None and self._robust_control_enabled():
                try:
                    selected_reference = self._select_control_reference(estimates)
                except (ValueError, TypeError, ArithmeticError) as error:
                    raise PolicyStopRequest("Control reference: " + str(error)) from error
            if not selected_reference["valid"]:
                if (control.end_straight_distance_m > 0 and self.control_has_run
                        and control.mode == "mvp"
                        and self.planning_settings.algorithm == "road_centerline"
                        and self.planning_output["reason"] == "invalid_road_insufficient_geometry"
                        and estimates["state"]["valid"]
                        and estimates["state"]["speed_valid"]
                        and estimates["state"]["yaw_rate_valid"]):
                    # Authorized geometry-loss fallback: once per explicit
                    # run, finish straight instead of restarting on new cones.
                    self._end_straight_start_distance_m = self.distance_limiter.distance_m
                    self._end_straight_started_at_s = self._monotonic()
                    self.last_control_reference_deadline_s = None
                    self._trajectory_reference = None
                    self.state_reason = (f"End straight: {control.end_straight_distance_m:.3f} m "
                                         "after insufficient boundary geometry")
                    self.get_logger().info(self.state_reason)
                    self.publish_state()
                    return control.end_straight_drive_normalized, 0.0, None, None, self.distance_limiter.distance_m
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
            # Only the selected path reaches the PID. Save its previous state
            # so a malformed planner reference cannot reset the baseline PID.
            previous_controller = deepcopy(self.controller) if planner_reference is not None else None
            try:
                drive_action, steering_action, self.control_diagnostics = self.controller.calculate(
                    selected_reference, estimates["state"], estimates["vehicle_params"],
                    self.get_clock().now().nanoseconds/1e9, dt)
            except Exception as error:
                if planner_reference is None:
                    raise
                self.control_diagnostics = {"valid": False, "stop_requested": True,
                    "reason": "planner_controller_exception: "+str(error)}
            if planner_reference is not None and (
                    not self.control_diagnostics["valid"] or self.control_diagnostics["stop_requested"]):
                self.controller = previous_controller
                self.planner_execution.diagnostics.update(mode="baseline", fallback=True,
                    reason="planner_control_rejected: "+str(self.control_diagnostics.get("reason")))
                selected_reference = self.planning_output
                if self._robust_control_enabled():
                    selected_reference = self._select_control_reference(estimates)
                drive_action, steering_action, self.control_diagnostics = self.controller.calculate(
                    selected_reference, estimates["state"], estimates["vehicle_params"],
                    self.get_clock().now().nanoseconds/1e9, dt)
            if not self.control_diagnostics["valid"] or self.control_diagnostics["stop_requested"]:
                reason = "Control: " + self.control_diagnostics["reason"]
                raise PolicyStopRequest(reason)
            self.control_has_run = True
            self._trajectory_reference = selected_reference
            self.last_control_reference_deadline_s = (
                selected_reference["timestamp_s"]+selected_reference["valid_for_s"])
            if self._robust_control_enabled() and selected_reference.get("planner_version") != "lattice_execution":
                self._report_control_status()
            if planner_requested:
                self.control_diagnostics["reference_source"] = self.planner_execution.diagnostics.get("mode")
                self.control_diagnostics["reference_selection_reason"] = self.planner_execution.diagnostics.get("reason")
                self.planner_execution.diagnostics.update(drive_action=drive_action,
                    steering_action=steering_action, control_dt_s=dt)
            # Existing debug topics: lateral error (m), per-run distance (m).
            # The existing state string reports planning/control stop reasons.
            debug1 = self.control_diagnostics["path_error_m"]
            debug2 = self.distance_limiter.distance_m
            # Bounded diagnostic logging: expose the actual controller inputs,
            # including forward-only targeting, instead of inferring them from
            # separately delivered debug/action topics. No extra ROS interface.
            log_now = self._monotonic()
            if log_now-getattr(self, "_last_tracking_log_at", -math.inf) >= 0.5:
                self._last_tracking_log_at = log_now
                info = self.control_diagnostics
                self.get_logger().info("Tracking: " + str({
                    "timestamp_s": selected_reference["timestamp_s"],
                    "path_error_m": info["path_error_m"],
                    "control_lateral_error_m": info.get("control_lateral_error_m", info["path_error_m"]),
                    "heading_error_rad": info["heading_error_rad"],
                    "error_basis": info.get("error_basis", "calibrated"),
                    "path_encoding": info.get("path_encoding"),
                    "closest_s_m": info.get("closest_s_m"),
                    "preview_s_m": info.get("preview_s_m"),
                    "raw_heading_error_rad": info.get("raw_heading_error_rad"),
                    "corner_local_weight": info.get("corner_local_weight"),
                    "preview_tangent_heading_rad": info.get("preview_tangent_heading_rad"),
                    "target_bearing_rad": info.get("target_bearing_rad"),
                    "preview_xy_m": [info.get("preview_x_m"), info.get("preview_y_m")],
                    "steer_requested": steering_action,
                    "steer_before_slew": info.get("steer_before_slew", steering_action),
                    "steer_slew_limited": info.get("steer_slew_limited", False),
                    "curvature_1pm": info["curvature_1pm"],
                    "speed_mps": estimates["state"]["speed_mps"],
                    "yaw_rate_rps": estimates["state"]["yaw_rate_rps"],
                    "road_status": estimates["road"].get("status"),
                    "centerline_source": self.planning_diagnostics.get("centerline_source"),
                    "inferred_single_side": self.planning_diagnostics.get("inferred_single_side"),
                    "inward_offset_m": self.planning_diagnostics.get("inward_offset_m"),
                    "lane_width_observed_at_s": self.planning_diagnostics.get("lane_width_observed_at_s"),
                    "reference_valid_for_s": selected_reference["valid_for_s"],
                }))

        # =====================================
        # END OF: INSERT POLICY CODE ABOVE HERE
        # =====================================
        return drive_action, steering_action, camera_pan_action, debug1, debug2

    # ---- Control-only reference recovery; upstream algorithms stay unchanged --
    def _select_planner_reference(self, estimates):
        """Opt-in adapter faults return THIS tick's baseline, never Stop."""
        try:
            road = estimates.get("road") or {}
            if not any(road.get(k) for k in ("centerline_xy", "left_boundary_xy", "right_boundary_xy")):
                road = road.get("forward_geometry") or {}
            state = estimates["state"]
            # In planner mode advance the same short-interval odometry before
            # execution. The post-action offer is idempotent at this timestamp.
            self._shadow_collect_road(road, state["timestamp_s"])
            if self.planning_output.get("valid") and self.planning_output.get("stop_requested"):
                self.planner_execution.diagnostics = {"mode": "baseline", "fallback": True,
                    "reason": "baseline_stop_request", "requested_source": "planner"}
                return None
            period = 1/self.policy_update_rate_hz
            return self.planner_execution.select(road, state, self.shadow_pose, self.shadow_odometry_epoch,
                self.shadow_settings, self.planning_settings.preview_m, self.planning_settings.cruise_speed_mps,
                period, self.control_settings.max_dt_s+period)
        except Exception as error:
            self.planner_execution.diagnostics = {"mode": "baseline", "fallback": True,
                "requested_source": "planner", "reason": "execution_exception: "+str(error)}
            return None

    def _publish_planner_execution(self):
        """Record every executed control tick; transport faults cannot stop it."""
        try:
            enabled = self.planner_control_settings.enabled
            info = dict(self.planner_execution.diagnostics) if enabled else {
                "requested_source": "baseline", "mode": "baseline", "fallback": False,
                "reason": "baseline_selected"}
            ref = getattr(self, "_trajectory_reference", None) or {}
            control = self.control_diagnostics or {}
            info.update(timestamp_s=ref.get("timestamp_s"), generation=self.shadow_generation,
                reference_version=ref.get("planner_version"),
                baseline_valid=bool(self.planning_output and self.planning_output.get("valid")),
                baseline_reason=(self.planning_output or {}).get("reason"),
                speed_mps=(self.estimation_output or {}).get("state", {}).get("speed_mps"),
                preview_xy_m=[control.get("preview_x_m"), control.get("preview_y_m")],
                heading_error_rad=control.get("heading_error_rad"),
                steer_before_slew=control.get("steer_before_slew"),
                steer_after_slew=info.get("steering_action"),
                reference_valid_for_s=ref.get("valid_for_s"))
            if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
                info.update(mode="idle", fallback=False, reason=self.state_reason)
            self.planner_execution_publisher.publish(String(data=json.dumps(info, allow_nan=False)))
            mode = (info["mode"], info["reason"])
            if mode != self._last_execution_mode:
                self.get_logger().info("Reference selection: "+json.dumps(info, separators=(",", ":")))
                self._last_execution_mode = mode
        except Exception as error:
            self._warn("planner_execution_diagnostics", "Execution diagnostics unavailable: "+str(error))

    def _robust_control_enabled(self):
        cfg = self.control_settings
        return cfg.enabled and cfg.mode == "mvp" and cfg.robustness_enabled

    def _control_health_problem(self, monotonic_now, ros_now_ns):
        problem = self.health_problem(monotonic_now, ros_now_ns)
        if (getattr(self, "_end_straight_start_distance_m", None) is not None
                and problem == "No recent nonempty cone detections"):
            # Fresh empty camera frames are expected outside the cone FOV.
            # health_problem already checked EVERY required source's freshness.
            return None
        if not problem or not self._robust_control_enabled():
            return problem
        # Only cone availability may be bridged. Never exempt wheel/gyro or any
        # other required field, even if the cone fault is the first in the list.
        for name in self.required_sensors:
            if name != "cone_detections" and not self._fresh(name, monotonic_now, ros_now_ns):
                return f"Required sensor {name} is missing or stale"
        manager = getattr(self, "control_reference_manager", None)
        if manager is None or manager.reference is None:
            return problem
        cache_problem = manager.problem(monotonic_now, self.distance_limiter.distance_m)
        if cache_problem:
            return cache_problem + "; explicit resume required"
        return None

    def _select_control_reference(self, estimates):
        if not hasattr(self, "control_reference_manager"):
            self.control_reference_manager = ControlReferenceManager(self.control_settings)
        state = estimates["state"]
        # Execution cadence is independent of the planner's remaining ROAD
        # lifetime. Fresh motion is checked each cycle; the original source
        # timestamps and hold budgets are kept independently in the cache.
        ages = state.get("source_age_s") or {}
        remaining = [self.sensor_timeout_s[name]-ages.get(name, math.inf)
                     for name in ("wheel_speed", "imu_angular_velocity")]
        return self.control_reference_manager.select(
            self.planning_output, estimates["road"], state, self.motion_history,
            state["timestamp_s"], self._monotonic(), self.distance_limiter.distance_m,
            min(0.2, self.control_settings.max_dt_s, *remaining))

    def _report_control_status(self):
        manager = self.control_reference_manager
        self.control_diagnostics.update(manager.diagnostics)
        if manager.reference is None:
            return
        if manager.mode != getattr(self, "last_control_tracking_mode", None):
            self.last_control_tracking_mode = manager.mode
            self.state_reason = "Control: " + str(manager.diagnostics)
            self.get_logger().info(self.state_reason)
            self.publish_state()

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
        if hasattr(self, "control_reference_manager"):
            self.control_reference_manager.clear()
        self.last_control_tracking_mode = None
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False
        if hasattr(self, "planner_execution"):
            self.planner_execution.clear()
            self._last_execution_mode = None
        self._end_straight_start_distance_m = None
        self._end_straight_started_at_s = None
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_generation += 1
            self.shadow_last_submit_s = -math.inf
            self.shadow_handoff_estimate_s = self.shadow_settings.handoff_allowance_s
            self.shadow_last_result_s = None
            self.shadow_road_history = []
            self.shadow_pose_stamp_s = None
            self.shadow_pose = [0.0, 0.0, 0.0]
            self.shadow_odometry_epoch = getattr(self, "shadow_odometry_epoch", 0)+1
            self.shadow_previous_reference = None
            self.shadow_collection_events = []
            self.shadow_collection_reset_id = 0
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
        self._trajectory_reference = None
        if hasattr(self, "trajectory_publisher"):
            self._publish_trajectory(self.get_clock().now().nanoseconds, force=True)
            self._trajectory_last_publish_s = -math.inf
        # Baseline state/zero publications have priority over shadow diagnostics.
        if getattr(self, "shadow_worker", None) is not None:
            self._shadow_clear()
        if hasattr(self, "planner_execution_publisher"):
            self._publish_planner_execution()

    def _shadow_clear(self):
        """Clear shadow displays on every run boundary; old results are fenced."""
        if hasattr(self, "planner_execution"):
            self.planner_execution.clear()
        try:
            stamp = self.get_clock().now().nanoseconds/1e9
            for publisher in (self.shadow_path_publisher, self.shadow_prefix_publisher,
                              self.shadow_reference_publisher):
                self._shadow_publish_path(publisher, [], stamp)
            self.shadow_diagnostics_publisher.publish(String(data=json.dumps({
                "status": "IDLE", "generation": self.shadow_generation,
                "shadow_only": not bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False))})))
        except Exception as error:
            self._warn("shadow_clear", f"Shadow display clearing unavailable: {error}")

    def _shadow_collect_road(self, road, stamp):
        """Bounded 20 Hz collection; planner execution also calls before control.

        Accumulate short read-only motion intervals; no per-frame fitting and no
        changes to the estimator's motion history. Alignment faults clear only
        shadow memory. Original source epochs are not renewed by reconstruction.
        """
        previous = getattr(self, "shadow_pose_stamp_s", None)
        pose = getattr(self, "shadow_pose", [0.0, 0.0, 0.0])
        epoch = getattr(self, "shadow_odometry_epoch", 0)
        history = getattr(self, "shadow_road_history", [])
        reason, reset = ("bootstrap" if previous is None else "aligned"), False
        if previous is not None and stamp != previous:
            try:
                motion, motion_reason = self.motion_history.pose_between(previous, stamp)
            except Exception as error:
                motion, motion_reason = None, "motion_exception: "+str(error)
            if motion is None:
                history, pose, epoch = [], [0.0, 0.0, 0.0], epoch+1
                self.shadow_previous_reference = None
                reason, reset = motion_reason, True
                self.shadow_collection_reset_id = getattr(self, "shadow_collection_reset_id", 0)+1
                events = getattr(self, "shadow_collection_events", [])
                events.append({"id": self.shadow_collection_reset_id, "stamp_s": stamp,
                               "reason": motion_reason, "odometry_epoch": epoch})
                self.shadow_collection_events = events[-64:]
                self._warn("shadow_alignment", "Shadow temporal history reset: "+str(motion_reason))
            else:
                c, s = math.cos(pose[2]), math.sin(pose[2])
                pose = [pose[0]+c*motion["dx_m"]-s*motion["dy_m"],
                        pose[1]+s*motion["dx_m"]+c*motion["dy_m"], pose[2]+motion["yaw_rad"]]
        measurement = road.get("measurement_timestamp_s")
        key = measurement if finite_number(measurement) else stamp
        history = [h for h in history if stamp-h["stamp_s"] <= self.shadow_settings.fusion_window_s
                   and h.get("measurement_key") != key]
        keys = ("centerline_xy", "left_boundary_xy", "right_boundary_xy", "boundary_source",
                "lane_width_m", "lane_width_source", "inward_offset_m", "status", "visibility",
                "timestamp_s", "measurement_timestamp_s", "time_aligned", "degraded", "frame_id")
        snapshot = {"road": {key: deepcopy(road.get(key)) for key in keys}, "stamp_s": stamp,
                    "pose": list(pose), "odometry_epoch": epoch, "measurement_key": key}
        state = (getattr(self, "estimation_output", None) or {}).get("state") or {}
        snapshot["state"] = {key: state.get(key) for key in
            ("timestamp_s", "frame_id", "speed_mps", "yaw_rate_rps", "speed_valid", "yaw_rate_valid")}
        history.append(snapshot)
        self.shadow_road_history = history[-self.shadow_settings.fusion_max_samples:]
        self.shadow_pose, self.shadow_pose_stamp_s, self.shadow_odometry_epoch = pose, stamp, epoch
        self.shadow_collection_diagnostics = {"status": "RESET" if reset else "BOOTSTRAP" if previous is None else "ALIGNED",
            "reason": reason, "reset": reset, "buffer_samples": len(self.shadow_road_history), "odometry_epoch": epoch,
            "events": deepcopy(getattr(self, "shadow_collection_events", [])),
            "reset_events_total": getattr(self, "shadow_collection_reset_id", 0)}
        return snapshot["road"]

    def _shadow_offer(self):
        now = self._monotonic()
        estimates = self.estimation_output
        if not estimates:
            return
        road = estimates.get("road") or {}
        forward = road.get("forward_geometry") or {}
        if not any(road.get(k) for k in ("centerline_xy", "left_boundary_xy", "right_boundary_xy")):
            road = forward
        stamp = estimates.get("state", {}).get("timestamp_s")
        stamp = stamp if finite_number(stamp) else self.get_clock().now().nanoseconds/1e9
        if self.shadow_settings.fusion_enabled or getattr(getattr(self, "planner_control_settings", None), "enabled", False):
            road = self._shadow_collect_road(road, stamp)
        if now-self.shadow_last_submit_s < 1/self.shadow_settings.rate_hz:
            return
        self.shadow_job_id += 1
        job = {"settings": vars(self.shadow_settings).copy(),
            "road": deepcopy(road),
            "state": {key: estimates.get("state", {}).get(key) for key in
                      ("speed_mps", "yaw_rate_rps", "timestamp_s", "speed_valid", "yaw_rate_valid")},
            "stamp_s": stamp,
            "generation": self.shadow_generation, "job_id": self.shadow_job_id,
            "submitted_monotonic_s": now, "preview_m": self.planning_settings.preview_m,
            "handoff_estimate_s": self.shadow_handoff_estimate_s,
            "shadow_pose": list(getattr(self, "shadow_pose", [0.0, 0.0, 0.0])),
            "shadow_odometry_epoch": getattr(self, "shadow_odometry_epoch", 0),
            "planner_control_enabled": bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False)),
            "shadow_only": not bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False))}
        if self.shadow_settings.fusion_enabled:
            job.update(road_history=deepcopy(getattr(self, "shadow_road_history", [])),
                shadow_pose=list(getattr(self, "shadow_pose", [0.0, 0.0, 0.0])),
                shadow_odometry_epoch=getattr(self, "shadow_odometry_epoch", 0),
                temporal_collection=getattr(self, "shadow_collection_diagnostics", None),
                previous_reference=deepcopy(getattr(self, "shadow_previous_reference", None)))
        self.shadow_worker.submit(job)
        self.shadow_last_submit_s = now

    def _shadow_publish_path(self, publisher, points, stamp_s):
        message = Path()
        message.header.frame_id = self.policy_frame_id
        message.header.stamp = Time(nanoseconds=round(stamp_s*1e9)).to_msg()
        for point in points:
            pose = PoseStamped(header=message.header)
            pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
            heading = point[2] if len(point) > 2 else 0.0
            pose.pose.orientation.z = math.sin(heading/2)
            pose.pose.orientation.w = math.cos(heading/2)
            message.poses.append(pose)
        publisher.publish(message)

    def _shadow_poll(self):
        try:
            result = self.shadow_worker.poll()
            if result is None:
                return
            if result.get("status") == "TRANSPORT_ERROR":
                self._warn("shadow_worker", "Shadow worker failed: "+result.get("error", "unknown"))
                self._shadow_clear()
                if getattr(getattr(self, "planner_control_settings", None), "enabled", False):
                    self.planner_execution.accept(result)
                self.shadow_diagnostics_publisher.publish(String(data=json.dumps(result)))
                self.shadow_worker.close()
                self.shadow_worker = ShadowWorker(__file__)
                return
            if (self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION
                    or result.get("generation") != self.shadow_generation):
                return
            if getattr(getattr(self, "planner_control_settings", None), "enabled", False):
                self.planner_execution.accept(result)
            guide = (result.get("fusion") or {}).get("guide")
            if (result.get("status") == "VALID" and guide
                    and guide.get("odometry_epoch") == getattr(self, "shadow_odometry_epoch", 0)):
                self.shadow_previous_reference = guide
            round_trip = self._monotonic()-result.get("submitted_monotonic_s", self._monotonic())
            self.shadow_handoff_estimate_s = max(self.shadow_handoff_estimate_s, round_trip)
            received = self._monotonic()
            previous = getattr(self, "shadow_last_result_s", None)
            interval = received-previous if previous is not None else None
            self.shadow_last_result_s = received
            transport = {"round_trip_s": round_trip, "completed_interval_s": interval,
                "completed_rate_hz": 1/interval if interval is not None and interval > 0 else None,
                "handoff_exceeded_estimate": round_trip > result.get("handoff_estimate_s", round_trip)}
            # Header is the INPUT frame time, never the completion/publication time.
            stamp = result.get("input_stamp_s")
            stamp = stamp if finite_number(stamp) else self.get_clock().now().nanoseconds/1e9
            selected = result.get("selected")
            safe = result.get("status") == "VALID"
            path = selected["path"] if selected and safe else []
            prefix = []
            if path:
                arc = _shadow_lengths([p[:2] for p in path])
                end = min(result["check_length_m"], arc[-1])
                prefix = [p for p, s in zip(path, arc) if s <= end]
                if end > 0:
                    p = _shadow_at([p[:2] for p in path], arc, end)
                    prefix.append((*p, prefix[-1][2]))
            self._shadow_publish_path(self.shadow_path_publisher, path, stamp)
            self._shadow_publish_path(self.shadow_prefix_publisher, prefix, stamp)
            self._shadow_publish_path(self.shadow_reference_publisher, result.get("reference", []), stamp)
            wire = result.get("wire_json") or json.dumps(result)
            # The large candidate JSON was encoded off the executor. Append only
            # small transport metadata here, instead of re-encoding all points.
            self.shadow_diagnostics_publisher.publish(String(data=wire[:-1]+',"transport":'+json.dumps(transport)+'}'))
            # Retain pending faults until a diagnostic carrying them was published.
            acknowledged = ((result.get("fusion") or {}).get("collection") or {}).get("events") or []
            if acknowledged:
                reset_id = max(event.get("id", 0) for event in acknowledged)
                self.shadow_collection_events = [event for event in getattr(self, "shadow_collection_events", [])
                                                 if event.get("id", 0) > reset_id]
            summary = {key: result.get(key) for key in ("status", "job_id", "candidate_count", "feasible_count",
                "speed_mps", "hold_time_s", "check_length_m", "score_length_m", "elapsed_s", "late_for_requested_period", "timing_s")}
            summary.update(selected_id=selected.get("id") if selected else None,
                selected_family=selected.get("family") if selected else None,
                result_age_s=self.get_clock().now().nanoseconds/1e9-stamp,
                **transport)
            if result.get("fusion"):
                summary["fusion"] = {k: result["fusion"].get(k) for k in
                    ("status", "reason", "degraded", "raw_fallback_used", "selected_source",
                     "fused_status", "raw_status", "history_frames", "prior_used", "removed_guidance_loops", "reference_crop", "collection")}
            if result.get("error"):
                summary["error"] = result["error"]
            self.get_logger().info("Shadow planning: "+json.dumps(summary, separators=(",", ":")))
        except Exception as error:
            self._warn("shadow_publish", f"Shadow diagnostics unavailable: {error}")

    def _publish_trajectory(self, ros_now_ns, force=False):
        """Display the accepted reference at 10 Hz; existing stops clear it."""
        if not self.trajectory_enabled:
            return
        now = self._monotonic()
        if not force and now-self._trajectory_last_publish_s < 0.1:
            return
        message = Path()
        message.header.frame_id = self.policy_frame_id
        message.header.stamp = Time(nanoseconds=ros_now_ns).to_msg()
        try:
            ref = self._trajectory_reference
            if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION and ref
                    and ref.get("valid") is True and ref.get("frame_id") == self.policy_frame_id
                    and ref["timestamp_s"] <= ros_now_ns/1e9 < ref["timestamp_s"]+ref["valid_for_s"]):
                path = ref["path"]
                if path.get("type") in ("CARTESIAN_CENTERLINE", "ORDERED_CENTERLINE"):
                    points = path["samples_xy"]
                else:
                    cached = ref.get("control_cached_path_samples")
                    lo, hi = (cached[0][0], cached[-1][0]) if cached else path["range"]
                    points = []
                    for i in range(81):
                        x = lo if i == 0 else hi if i == 80 else lo+(hi-lo)*i/80
                        sample = evaluate_planning_path(ref, x, ros_now_ns/1e9)
                        if sample is None:
                            points = []
                            break
                        points.append(sample["position_xy_m"])
                if (2 <= len(points) <= 201
                        and all(len(p) == 2 and all(finite_number(v) for v in p) for p in points)):
                    message.header.stamp = Time(nanoseconds=round(ref["timestamp_s"]*1e9)).to_msg()
                    for x, y in points:
                        pose = PoseStamped(header=message.header)
                        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                        pose.pose.orientation.w = 1.0
                        message.poses.append(pose)
        except Exception as error:
            message.poses = []
            self._warn("trajectory_display", f"Trajectory display unavailable: {error}")
        try:
            self.trajectory_publisher.publish(message)
            self._trajectory_last_publish_s = now
        except Exception as error:
            self._warn("trajectory_display", f"Trajectory display publication unavailable: {error}")

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
        if (self.control_settings.max_run_time_s > 0 and self.policy_started_at is not None and
                monotonic_now-self.policy_started_at >= self.control_settings.max_run_time_s):
            return "Run time limit reached; explicit resume required"
        start_distance = getattr(self, "_end_straight_start_distance_m", None)
        if start_distance is not None:
            for name in ("wheel_speed", "imu_angular_velocity"):
                observation = self.observations.get(name)
                timeout = min(self.sensor_timeout_s[name], self.estimation_settings.max_source_age_s,
                              self.estimation_settings.motion_max_gap_s)
                if (observation is None
                        or self._age(observation, monotonic_now, ros_now_ns) >= timeout):
                    return f"End straight required sensor {name} is missing or stale"
            if self.distance_limiter.distance_m-start_distance+1e-9 >= self.control_settings.end_straight_distance_m:
                return "End straight distance reached; explicit resume required"
            if monotonic_now-self._end_straight_started_at_s >= self.control_settings.end_straight_max_time_s:
                return "End straight time limit reached; explicit resume required"
            return None  # No road reference is consumed in fixed-command mode.
        deadline = self.last_control_reference_deadline_s
        manager = getattr(self, "control_reference_manager", None)
        if (self._robust_control_enabled() and manager is not None
                and manager.mode == "DEGRADED"):
            problem = manager.problem(monotonic_now, self.distance_limiter.distance_m)
            if problem:
                return problem + "; explicit resume required"
        if deadline is not None and ros_now_ns/1e9 >= deadline:
            return "Control reference expired; explicit resume required"
        if (not self.control_has_run and self.policy_started_at is not None
                and monotonic_now-self.policy_started_at >= self.control_settings.startup_grace_s):
            return "Control startup history deadline exceeded; explicit resume required"
        return None

    def supervision_callback(self):
        now, ros_now = self._times()
        if self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION:
            problem = self._control_health_problem(now, ros_now) or self._control_problem(now, ros_now)
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
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_worker.close()

    def destroy_node(self):
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_worker.close()
        return super().destroy_node()


def main(args=None):
    # Keep the context valid until the final zero command has been published.
    # ROS's asynchronous signal handler can otherwise invalidate it mid-publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    previous_handlers = {sig: signal.signal(sig, request_stop)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
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
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
