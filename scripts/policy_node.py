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
import gc
import json
import math
import json
import os
import queue
import subprocess
import sys
import threading
from pathlib import Path
from numbers import Real
import time
import traceback
import uuid

# One BLAS/OpenMP thread, set before anything imports NumPy; the lattice and planner
# workers inherit it. The MPC's QPs are small, and a BLAS pool per process competes
# with the ROS executor and the workers for the Jetson's cores (from 45d492f on
# checkpoint/rnn-mpc-car13-20261026). An operator's own setting is kept.
for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "BLIS_NUM_THREADS"):
    os.environ.setdefault(_name, "1")

# ---- MVP2.1 lattice planner (ported from origin/mvp2.1 3652e9b) ----------
# Worker entry, verbatim. It runs only the pure _shadow_* / Shadow* definitions
# of this same file, before any ROS import below.

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
    # mpc-car-run adaptation (2026-10-20): the planner also reads the shared
    # module constant EXECUTION_TAIL_M, defined below this entry point.
    definitions += [n for n in tree.body if isinstance(n, ast.Assign)
                    and [getattr(t, "id", None) for t in n.targets] == ["EXECUTION_TAIL_M"]]
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


if __name__ == "__main__" and sys.argv[1:] == ["--shadow-worker"]:
    _shadow_worker_entry()
    sys.exit(0)


if sys.argv[1:] == ["--planner-worker"]:
    # The worker uses only the pure definitions below. Do not load ROS/DDS/TF
    # libraries on every reconstruction; PolicyNode is never instantiated here.
    Node = object
else:
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from sensor_msgs.msg import Imu, LaserScan, PointCloud
    from geometry_msgs.msg import PoseStamped, Point32
    from nav_msgs.msg import Path as TrajectoryPath   # pathlib.Path stays the planner worker's
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


@dataclass
class LidarObstacleSettings:
    """前方矩形检测区 / Forward detection with an independent stop switch."""
    enabled: bool = False  # 裸节点保持兼容；本分支 YAML 开启 / Enabled by branch YAML.
    stop_enabled: bool = True  # 检测与停车分开 / Detect-only mode keeps telemetry.
    body_width_m: float = 0.250
    front_from_cg_m: float = 0.297
    side_margin_m: float = 0.02
    roi_half_width_m: float = 0.0  # 0 沿用车宽 / Explicit detection half-width override.
    range_min_m: float = 0.2  # 可观测区下限，不伪造驱动标定 / Observation lower bound.
    allow_no_return_clear: bool = False  # 激进假设由 YAML 显式开启 / Explicit opt-in.
    stop_distance_m: float = 1.0  # 从车头量起，待上车确认 / From bumper; provisional.
    cluster_gap_m: float = 0.06
    min_cluster_points: int = 2
    min_known_fraction: float = 0.7

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("lidar_obstacle.enabled must be boolean")
        if not isinstance(self.stop_enabled, bool):
            raise ValueError("lidar_obstacle.stop_enabled must be boolean")
        if not isinstance(self.allow_no_return_clear, bool):
            raise ValueError("lidar_obstacle.allow_no_return_clear must be boolean")
        for name in ("body_width_m", "front_from_cg_m", "stop_distance_m", "cluster_gap_m"):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"lidar_obstacle.{name} must be positive and finite")
        if not finite_number(self.side_margin_m) or self.side_margin_m < 0:
            raise ValueError("lidar_obstacle.side_margin_m must be finite and nonnegative")
        if not finite_number(self.roi_half_width_m) or self.roi_half_width_m < 0:
            raise ValueError("lidar_obstacle.roi_half_width_m must be finite and nonnegative")
        if not finite_number(self.range_min_m) or self.range_min_m < 0:
            raise ValueError("lidar_obstacle.range_min_m must be finite and nonnegative")
        if (not isinstance(self.min_cluster_points, int) or isinstance(self.min_cluster_points, bool)
                or not 1 <= self.min_cluster_points <= 16):
            raise ValueError("lidar_obstacle.min_cluster_points must be an integer in [1,16]")
        if not finite_number(self.min_known_fraction) or not 0 < self.min_known_fraction <= 1:
            raise ValueError("lidar_obstacle.min_known_fraction must be in (0,1]")


def lidar_roi_bounds(settings):
    half_width = settings.roi_half_width_m or settings.body_width_m/2 + settings.side_margin_m
    return ((settings.front_from_cg_m, settings.front_from_cg_m+settings.stop_distance_m),
            (-half_width, half_width))


def _lidar_ray_rectangle(origin, direction, bounds):
    """Ray/XY slab interval, measured along the original 3D unit scan ray."""
    enter, leave = 0.0, math.inf
    for axis, (lo, hi) in enumerate(bounds):
        if abs(direction[axis]) < 1e-12:
            if not lo <= origin[axis] <= hi:
                return None
        else:
            a, b = (lo-origin[axis])/direction[axis], (hi-origin[axis])/direction[axis]
            enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
    return (enter, leave) if leave > enter and leave > 0 else None


def detect_lidar_obstacles(scan, cartesian, settings, stamp_ns=None):
    """O(N), no ROS dependency. 保留近点与原始索引 / Keep close hits and ray indices.

    Finite/range filtering and mounting TF precede this function. Sparse groups
    remain candidates (UNKNOWN), never silently become free space. No smoothing
    across edges, inferred object class, temporal tracking or scan deskew.
    """
    cfg = settings
    bounds = lidar_roi_bounds(cfg)
    half_width = bounds[1][1]
    result = {"status": "UNKNOWN", "reason": "invalid_scan", "available": False,
              "measurement_timestamp_s": None if stamp_ns is None else stamp_ns/1e9,
              "frame_id": "base_link", "roi_xy_m": bounds,
              "points_xyz_m": [], "scan_indices": [], "clusters": [],
              "candidate_clusters": [], "nearest_bumper_distance_m": None,
              "known_fraction": 0.0, "usable_fraction": 0.0, "roi_ray_count": 0,
              "blind_roi_ray_count": 0, "assumed_clear_ray_count": 0,
              "allow_no_return_clear": cfg.allow_no_return_clear}
    if not isinstance(scan, dict) or not isinstance(cartesian, dict):
        return result
    ranges, points, indices = scan.get("ranges"), cartesian.get("points_xyz"), cartesian.get("scan_indices")
    origin = cartesian.get("ray_origin_xyz_m")
    axes = cartesian.get("ray_axes_xyz")
    if (cartesian.get("frame_id") != "base_link" or not isinstance(ranges, (list, tuple))
            or not 3 <= len(ranges) <= 4096 or not isinstance(points, (list, tuple))
            or not isinstance(indices, (list, tuple)) or len(points) != len(indices)
            or not isinstance(origin, (list, tuple)) or len(origin) != 3
            or not isinstance(axes, (list, tuple)) or len(axes) != 2
            or not all(isinstance(a, (list, tuple)) and len(a) == 3 for a in axes)
            or not all(finite_number(v) for a in (origin, *axes) for v in a)):
        return result
    metadata = [scan.get(k) for k in ("angle_min", "angle_increment", "range_min", "range_max")]
    if not all(finite_number(v) for v in metadata):
        return result
    angle_min, increment, range_min, range_max = metadata
    effective_min = max(range_min, cfg.range_min_m)
    result["effective_range_min_m"] = effective_min
    if effective_min >= range_max:
        result["reason"] = "range_min_exceeds_sensor_max"
        return result
    # 首版仅支持水平全周扫描 / Require a level full revolution for ROI coverage.
    if (increment <= 0 or range_min < 0 or range_max <= range_min
            or abs(len(ranges)*increment-2*math.pi) > 1.5*increment
            or any(abs(a[2]) > 0.1 for a in axes)
            or origin[0] >= bounds[0][0]):
        result["reason"] = "unsupported_scan_or_mount"
        return result
    previous_index = -1
    for point, index in zip(points, indices):
        if (not isinstance(index, int) or isinstance(index, bool) or not previous_index < index < len(ranges)
                or not isinstance(point, (tuple, list)) or len(point) != 3
                or not all(finite_number(v) for v in point)):
            return result
        previous_index = index
    result.update(available=True, reason="no_hits_in_roi")
    groups, group = [], []
    previous = None
    for point, index in zip(points, indices):
        x, y, _ = point
        if not (bounds[0][0] <= x <= bounds[0][1] and abs(y) <= half_width):
            if group:
                groups.append(group)
                group = []
            previous = None
            continue
        # No bridging across missing/invalid/out-of-ROI rays. Adaptive allowance
        # accommodates the increasing beam spacing at larger measured ranges.
        if previous is not None:
            last_point, last_index = previous
            gap = cfg.cluster_gap_m + 2*math.dist(point, origin)*increment
            if index != last_index+1 or math.dist(point[:2], last_point[:2]) > gap:
                groups.append(group)
                group = []
        group.append((point, index))
        previous = (point, index)
        result["points_xyz_m"].append(tuple(point))
        result["scan_indices"].append(index)
    if group:
        groups.append(group)
    for group in groups:
        xs, ys = [p[0][0] for p in group], [p[0][1] for p in group]
        cluster = {"point_count": len(group), "bbox_xy_m": (min(xs), max(xs), min(ys), max(ys)),
                   "centroid_xy_m": (sum(xs)/len(xs), sum(ys)/len(ys)),
                   "nearest_bumper_distance_m": min(xs)-cfg.front_from_cg_m,
                   "points_xyz_m": [tuple(p) for p, _ in group],
                   "scan_indices": [i for _, i in group]}
        key = "clusters" if len(group) >= cfg.min_cluster_points else "candidate_clusters"
        result[key].append(cluster)
    if result["points_xyz_m"]:
        result["nearest_bumper_distance_m"] = min(p[0] for p in result["points_xyz_m"])-cfg.front_from_cg_m

    # 只评价量程可见部分 / Evaluate the observable ROI, not the blind prefix.
    # Positive infinity is an explicit aggressive no-return assumption, not
    # measured free space. NaN/-inf/out-of-range and nearer occlusion stay unknown.
    # Already supplied valid close hits above remain obstacles even <effective_min.
    known, assumed, eligible, blind = 0, 0, 0, 0
    for i, distance in enumerate(ranges):
        angle = angle_min + i*increment
        cosine, sine = math.cos(angle), math.sin(angle)
        direction = tuple(cosine*axes[0][k]+sine*axes[1][k] for k in range(3))
        interval = _lidar_ray_rectangle(origin, direction, bounds)
        if interval is None:
            continue
        enter, leave = interval
        observable_enter = max(enter, effective_min)
        if leave <= observable_enter:
            blind += 1
            continue
        eligible += 1
        if (finite_number(distance) and effective_min <= distance <= range_max
                and distance >= observable_enter):
            known += 1
        elif (cfg.allow_no_return_clear and isinstance(distance, Real) and distance == math.inf
              and range_max >= leave):
            assumed += 1
    result["roi_ray_count"] = eligible
    result["blind_roi_ray_count"] = blind
    result["assumed_clear_ray_count"] = assumed
    result["known_fraction"] = known/eligible if eligible else 0.0
    result["usable_fraction"] = (known+assumed)/eligible if eligible else 0.0
    if result["clusters"]:
        result.update(status="OBSTACLE", reason="cluster_in_stop_roi")
    elif result["candidate_clusters"]:
        result["reason"] = "sparse_hit_in_stop_roi"
    elif eligible < 3 or result["usable_fraction"] < cfg.min_known_fraction:
        result["reason"] = "insufficient_roi_returns"
    else:
        result["status"] = "CLEAR"
        if assumed:
            result["reason"] = "no_hits_in_observable_roi_assumed_clear"
    return result


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
    # mpc-car-run adaptation (from sim-branch 2c41240, 2026-10-21): "fixed" uses
    # left_colour/right_colour. "auto" tries both mappings on fresh two-sided
    # observations at the start of each run and locks the one that passes the
    # width gate (EstimationPipeline.road); the road stays invalid until then.
    colour_assignment: str = "fixed"
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
    # 2026-10-28 (car 01, Gym right angles): a 1-2 frame invalid_ordered_geometry /
    # invalid_boundary_pairing still clears the cone history, but true keeps the learned
    # width, so the next single-sided frames are not "width unknown" stops.
    keep_width_on_geometry_glitch: bool = False
    # Fraction of the FULL learned/known lane width to offset inward from a
    # single observed boundary. 0.5 is geometric centre; smaller hugs that side.
    # This does not change width learning or the two-boundary centreline.
    single_side_offset_fraction: float = 0.5
    # Ordered XY curves support vertical and returning road segments. The
    # legacy y(x) estimator remains selectable for the archived workshop run.
    road_geometry: str = "y_of_x"
    # ordered_xy pairing: how far (m of opposite-boundary arc length) a pair may
    # step back along the other boundary before the centerline run is split.
    # 0.03 is main's behaviour; 2 cm cone noise occasionally splits the run there
    # and _ordered_road keeps the nearest short piece (reference_too_short). Larger
    # values also pair misclassified cones (car05 false-seed fixture), see
    # config/ai4r_policy_mpc_centerline.yaml.
    ordered_pairing_backtrack_m: float = 0.03
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
        if self.colour_assignment not in ("fixed", "auto"):
            raise ValueError("estimation.colour_assignment must be fixed or auto")
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
        if not isinstance(self.keep_width_on_geometry_glitch, bool):
            raise ValueError("estimation.keep_width_on_geometry_glitch must be bool")
        if self.continuous_road_output and self.history_propagation != "continuous":
            raise ValueError("continuous road output requires continuous history propagation")
        for name, value in vars(self).items():
            if name in ("colour_assignment", "history_propagation", "road_history_mode", "road_geometry",
                        "history_merge_single_side", "continuous_road_output", "single_side_from_learned_width",
                        "keep_width_on_geometry_glitch"):
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
            elif name == "ordered_pairing_backtrack_m":
                if not 0.0 <= value <= 0.3:
                    raise ValueError("estimation.ordered_pairing_backtrack_m must be in [0, 0.3]")
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
    # Ordering / pairing flickers: estimation.keep_width_on_geometry_glitch keeps the width.
    GEOMETRY_GLITCH_FAILURES = ("invalid_ordered_geometry", "invalid_boundary_pairing")

    def __init__(self, settings, frame_id="base_link", left_colour=None, right_colour=None,
                 motion_history=None):
        self.settings, self.frame_id = settings, frame_id
        self.left_colour = settings.left_colour if left_colour is None else left_colour
        self.right_colour = settings.right_colour if right_colour is None else right_colour
        # mpc-car-run adaptation (from sim-branch 2c41240): run-scoped colour lock.
        self._colour_assignment_locked = settings.colour_assignment == "fixed"
        self._colour_assignment_evidence = []
        self.motion = motion_history or EstimationMotionHistory(settings)
        self._clear_road_history()

    def _clear_history_after_failure(self, status):
        """An unrecoverable road failure clears the history; a geometry flicker may keep the width."""
        self._clear_road_history(keep_learned_width=self.settings.keep_width_on_geometry_glitch
                                 and status in self.GEOMETRY_GLITCH_FAILURES)

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
                self._clear_history_after_failure(current["status"])
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
        """Resolve colour independently of configuration before using history.

        Probe fresh, observed two-sided roads without learned/known-width
        inference. Never acquire an assignment from an inferred or predicted
        boundary. A pipeline is recreated for each explicit policy run; ordinary
        road-history clears deliberately keep its established colour mapping.

        mpc-car-run adaptation (from sim-branch 2c41240, mvp21_policy/estimation.py):
        verbatim; the raw reconstruction this wraps is _road() below (this
        branch's former road(), identical to the sim branch's _road()).
        """
        if not self._colour_assignment_locked:
            from dataclasses import replace     # the offline/test namespaces have no module imports
            hypotheses, usable = [], []
            probe_settings = replace(self.settings, colour_assignment="fixed",
                                     known_lane_width_m=0.0,
                                     single_side_from_learned_width=False)
            for left, right in ((1, 2), (2, 1)):
                probe = EstimationPipeline(probe_settings, self.frame_id,
                                           left_colour=left, right_colour=right)
                road = probe._road(batch, stamp_ns, age)
                observed = (road["valid"] and road["visibility"] == "both"
                            and len(road["centerline_xy"]) >= 2)
                distance = (math.sqrt(_path_project(road["centerline_xy"],
                    _path_lengths(road["centerline_xy"]), (0.0, 0.0), forward=False)[0])
                    if observed else None)
                evidence = dict(left_colour=left, right_colour=right,
                    valid=observed, road_status=road["status"],
                    lane_width_m=road.get("lane_width_m"), distance_to_ego_m=distance,
                    boundary_diagnostics=deepcopy(road["boundary_diagnostics"]))
                hypotheses.append((road, evidence))
                if observed:
                    usable.append(evidence)
            usable.sort(key=lambda item: item["distance_to_ego_m"])
            if (len(usable) == 1 or len(usable) == 2 and
                    usable[1]["distance_to_ego_m"]-usable[0]["distance_to_ego_m"]
                    > self.settings.road_sample_spacing_m):
                chosen = usable[0]
                self.left_colour, self.right_colour = chosen["left_colour"], chosen["right_colour"]
                self._colour_assignment_locked = True
                self._colour_assignment_evidence = [item for _, item in hypotheses]
            else:
                pending = "ambiguous" if usable else "pending"
                # Keep both probe diagnostics, with no assigned physical sides
                # or reusable history. The configured fixed mapping is ignored.
                result = hypotheses[0][0]
                status = ("boundary_colour_assignment_"+pending if self._fresh(batch, age)
                          and stamp_ns is not None and stamp_ns > 0 else "unavailable")
                result.update(valid=False, status=status,
                    visibility="unassigned", centerline_xy=[], left_boundary_xy=None,
                    right_boundary_xy=None, lane_width_m=None,
                    boundary_source={"left": "unassigned", "right": "unassigned"})
                result["boundary_diagnostics"] = {
                    side: dict(raw_count=0, filtered_count=0, fit_valid=False,
                        fit_status="colour_assignment_unresolved", inlier_count=0,
                        fit_rms_m=None, x_range_m=None)
                    for side in ("left", "right")}
                for key in ("_boundary_fits", "_support_cones", "_center_coeffs"):
                    result.pop(key, None)
                result["colour_assignment"] = dict(mode="auto", status=pending,
                    left_colour=None, right_colour=None,
                    hypotheses=[item for _, item in hypotheses])
                return result
        result = self._road(batch, stamp_ns, age)
        result["colour_assignment"] = dict(mode=self.settings.colour_assignment,
            status="locked" if self.settings.colour_assignment == "auto" else "fixed",
            left_colour=self.left_colour, right_colour=self.right_colour,
            hypotheses=deepcopy(self._colour_assignment_evidence))
        return result

    def _road(self, batch, stamp_ns, age):
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
                            and -1e-6 <= u <= 1+1e-6
                            and other_s >= previous_other_s-cfg.ordered_pairing_backtrack_m):
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
                         "boundary_diagnostics": deepcopy(observed["boundary_diagnostics"]),
                         # mpc-car-run adaptation (from sim-branch 2c41240)
                         "colour_assignment": deepcopy(observed.get("colour_assignment"))}
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
            self._clear_history_after_failure(road["status"])
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
    """V1 straight-road planner, V2 lattice planner, or main's direct sampled-road
    follower (road_centerline, ported 2026-10-15 for the ordered_xy MPC port).

    No dimensions or braking capability are guessed. Required vehicle_limits
    must be supplied by calibration or an explicitly labelled offline fixture.
    Constant-twist compensation is opt-in and is a short-time approximation.
    """
    # road_centerline follows estimator samples directly (main); centerline is
    # the legacy V1 straight planner; lattice_v2 is the Frenet lattice planner.
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
                if value not in ("centerline", "lattice_v2", "road_centerline"):
                    raise ValueError("planning.algorithm must be centerline, lattice_v2 or road_centerline")
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


@dataclass
class LatticeSettings:
    """V2 development defaults; teacher geometry is an approximation, not calibration.

    CSV sampling keeps ROS startup parameter types unambiguous. Hard limits and
    score scales are independent. Near extension requires an explicit clear-start
    assumption; no-return lidar alone does not establish free road boundaries.
    """
    model_source: str = "course_approximation"
    transition_lengths_m: str = "0.6,1.0,1.5"
    terminal_offsets_m: str = "-0.25,0.0,0.25"
    durations_s: str = "3.0"
    terminal_speeds_mps: str = "0.1,0.2"
    obstacle_check_enabled: bool = True
    clear_start_assumed: bool = False
    max_near_gap_m: float = 0.9
    max_extension_m: float = 1.3
    extension_margin_growth: float = 0.08
    blind_lateral_change_m: float = 0.08
    blind_steering_limit_rad: float = 0.3490658504
    geometry_step_m: float = 0.025
    time_step_s: float = 0.05
    horizon_m: float = 2.0
    score_time_s: float = 3.0
    budget_s: float = 0.035
    max_candidates: int = 81
    max_obstacle_points: int = 2048
    obstacle_margin_m: float = 0.05
    acceleration_max_mps2: float = 0.5
    braking_mps2: float = 0.5
    lateral_acceleration_max_mps2: float = 0.5
    actuation_delay_s: float = 0.2
    speed_max_mps: float = 0.5
    wheelbase_m: float = 0.33
    rear_axle_from_cg_m: float = 0.132
    front_extent_m: float = 0.297
    rear_extent_m: float = 0.198
    width_m: float = 0.25
    steering_limit_rad: float = math.pi/4
    steering_rate_limit_radps: float = math.pi/2
    hard_margin_m: float = 0.05
    d_scale_m: float = 0.125
    clearance_soft_m: float = 0.125
    blind_center_weight: float = 0.25
    clearance_peak_ratio: float = 0.5
    steering_scale_rad: float = math.pi/9
    steering_rate_scale_radps: float = math.pi/4
    steering_rate_weight: float = 0.5
    consistency_range_m: float = 1.0
    speed_scale_mps: float = 0.2
    acceleration_scale_mps2: float = 0.3
    acceleration_weight: float = 0.25
    weight_center: float = 0.30
    weight_clearance: float = 0.25
    weight_steering: float = 0.20
    weight_consistency: float = 0.15
    weight_motion: float = 0.10
    tie_tolerance: float = 0.01
    fit_position_error_m: float = 0.01
    fit_heading_error_rad: float = 0.03
    fit_curvature_error_1pm: float = 0.15
    preview_m: float = 0.35
    speed_preview_s: float = 0.3
    minimum_output_m: float = 0.4
    # Optional MVP2 execution and simple, checked centerline recovery.
    async_enabled: bool = False
    recovery_enabled: bool = False
    direct_sample_output: bool = False
    worker_timeout_s: float = 0.3
    recovery_speed_mps: float = 0.12
    initial_heading_max_rad: float = 1.22
    low_speed_yaw_tolerance_rps: float = 0.12
    update_rate_hz: float = 10.0
    # mpc-car-run switch (docs/UPSTREAM_FIXES_FOR_CAR_CN.md). "fade" continues
    # each boundary's own near-end slope and curvature into the blind zone;
    # "parallel" extends both boundaries as straight lines along the CENTERLINE
    # tangent, keeping the observed near-end width, so one boundary's fit noise
    # is not extrapolated ~1 m into a narrowing corridor.
    blind_extension_mode: str = "fade"

    def __post_init__(self):
        csv = {"transition_lengths_m", "terminal_offsets_m", "durations_s", "terminal_speeds_mps"}
        for name, value in vars(self).items():
            if name == "model_source":
                if value not in ("course_approximation", "course_simulation", "upstream"):
                    raise ValueError("invalid lattice model source")
            elif name == "blind_extension_mode":
                if value not in ("fade", "parallel"):
                    raise ValueError("planning.lattice.blind_extension_mode must be fade or parallel")
            elif name in csv:
                samples = self.samples(name)
                if not 1 <= len(samples) <= 9 or len(set(samples)) != len(samples):
                    raise ValueError("lattice sampling must have 1..9 distinct values")
                if name != "terminal_offsets_m" and any(v <= 0 for v in samples):
                    raise ValueError("lattice lengths, durations and speeds must be positive")
            elif isinstance(value, bool):
                if name not in ("obstacle_check_enabled", "clear_start_assumed", "async_enabled", "recovery_enabled", "direct_sample_output"):
                    raise ValueError("invalid lattice boolean")
            elif name in ("obstacle_check_enabled", "clear_start_assumed", "async_enabled", "recovery_enabled", "direct_sample_output"):
                raise ValueError("lattice flags must be booleans")
            elif not finite_number(value) or value <= 0:
                raise ValueError("lattice numeric settings must be finite and positive: " + name)
        if not 0 <= self.blind_center_weight <= 1 or not 0 <= self.clearance_peak_ratio <= 1:
            raise ValueError("lattice proportions must be in [0,1]")
        if not self.rear_axle_from_cg_m < self.wheelbase_m or self.steering_limit_rad >= math.pi/2:
            raise ValueError("invalid lattice vehicle geometry")
        if self.rear_extent_m < self.rear_axle_from_cg_m or self.front_extent_m < self.wheelbase_m-self.rear_axle_from_cg_m:
            raise ValueError("lattice body must cover both axles")
        if self.geometry_step_m > 0.05 or self.time_step_s > 0.1 or self.horizon_m > 3:
            raise ValueError("lattice checking resolution/range exceeds bounded domain")
        if self.initial_heading_max_rad >= math.pi/2 or self.worker_timeout_s > 1.0:
            raise ValueError("recovery must stay forward and worker timeout <= 1 s")
        if self.update_rate_hz > 20.0:
            raise ValueError("planning rate must not exceed 20 Hz")
        if any(t < self.score_time_s for t in self.samples("durations_s")):
            raise ValueError("all longitudinal candidates must cover the common scoring window")
        count = len(self.samples("transition_lengths_m"))*(len(self.samples("terminal_offsets_m"))+1)*(
            len(self.samples("durations_s"))*len(self.samples("terminal_speeds_mps"))+1)
        if not isinstance(self.max_candidates, int) or count > self.max_candidates or self.max_candidates > 256:
            raise ValueError("candidate product exceeds lattice cap")
        if not isinstance(self.max_obstacle_points, int) or not 1 <= self.max_obstacle_points <= 10000:
            raise ValueError("invalid obstacle cap")

    def samples(self, name):
        try:
            values = [float(v.strip()) for v in getattr(self, name).split(",")]
        except (ValueError, AttributeError):
            raise ValueError("invalid lattice CSV: " + name) from None
        if not all(math.isfinite(v) for v in values):
            raise ValueError("nonfinite lattice sample")
        return values


def _lattice_corridor(clipped, extension, step, cfg):
    """Centerline, left and right LatticeCurves sharing one blind-zone extension rule."""
    curves = [LatticeCurve(p, extension, step) for p in clipped]
    if cfg.blind_extension_mode != "parallel" or extension <= 0:
        return curves
    slope = curves[0].at_x(curves[0].points[0][0])[1]
    return [LatticeCurve(p, extension, step, extension_slope=slope) for p in clipped]


def _lattice_derivative(coeffs, order=1):
    for _ in range(order):
        coeffs = [i*c for i, c in enumerate(coeffs) if i] or [0.0]
    return coeffs


def _lattice_interp(points, x):
    if not points or not points[0][0]-1e-8 <= x <= points[-1][0]+1e-8:
        raise ValueError("outside_supported_range")
    lo, hi = 0, len(points)-1
    while hi-lo > 1:
        mid = (lo+hi)//2
        if points[mid][0] < x:
            lo = mid
        else:
            hi = mid
    a, b = points[lo], points[hi]
    return a[1]+(b[1]-a[1])*(x-a[0])/(b[0]-a[0])


class LatticeCurve:
    """Clamped cubic y(x), bounded C2 backward extension and Simpson arc table.

    Local roads must remain functions of body x. Arc lookup is approximate to
    the configured fine grid; actual sampled path is checked after conversion.
    No extension forward beyond observed support is allowed.
    """
    def __init__(self, points, extension, step, extension_slope=None):
        self.points = CenterlinePlanner._points(points)
        if self.points is None:
            raise ValueError("invalid_reference_points")
        n = len(points)
        h = [points[i+1][0]-points[i][0] for i in range(n-1)]
        second = [0.0]*n
        # O(N) clamped spline. Endpoint tangents from three-point quadratics
        # avoid the artificial zero curvature of natural end conditions.
        diag, rhs, upper = [1.0]*n, [0.0]*n, [0.0]*n
        def tangent(sample, x):
            if len(sample) < 3:
                return (sample[-1][1]-sample[0][1])/(sample[-1][0]-sample[0][0])
            base = sample[0][0]
            coeffs = _estimation_solve([[1, q-base, (q-base)**2] for q, _ in sample], [v for _, v in sample])
            if coeffs is None:
                raise ValueError("endpoint_tangent_singular")
            return coeffs[1]+2*coeffs[2]*(x-base)
        diag[0], upper[0] = 2*h[0], h[0]
        rhs[0] = 6*((points[1][1]-points[0][1])/h[0]-tangent(points[:3], points[0][0]))
        for i in range(1, n-1):
            diag[i] = 2*(h[i-1]+h[i])
            upper[i] = h[i]
            rhs[i] = 6*((points[i+1][1]-points[i][1])/h[i]-(points[i][1]-points[i-1][1])/h[i-1])
        diag[-1] = 2*h[-1]
        rhs[-1] = 6*(tangent(points[-3:], points[-1][0])-(points[-1][1]-points[-2][1])/h[-1])
        for i in range(1, n):
            factor = h[i-1]/diag[i-1]
            diag[i] -= factor*upper[i-1]
            rhs[i] -= factor*rhs[i-1]
        for i in range(n-1, -1, -1):
            second[i] = (rhs[i]-upper[i]*(second[i+1] if i+1 < n else 0))/diag[i]
        self.segments = []
        for i, length in enumerate(h):
            self.segments.append((points[i][0], points[i+1][0], [points[i][1],
                (points[i+1][1]-points[i][1])/length-length*(2*second[i]+second[i+1])/6,
                second[i]/2, (second[i+1]-second[i])/(6*length)]))
        # Bounded curvature-fading cubic extension: match y/y'/y'' at observed
        # start, fade y'' to zero at the rear endpoint instead of unlimited fit.
        first = self.segments[0]
        self.xmin, self.xmax = first[0]-extension, points[-1][0]
        self.extension = (self.xmin, first[0], [first[2][0], first[2][1], second[0]/2,
                                              second[0]/(6*extension) if extension > 0 else 0.0])
        if extension_slope is not None:
            # Straight extension along a supplied (centerline) slope; C0 at the join.
            self.extension = (self.xmin, first[0], [first[2][0], extension_slope, 0.0, 0.0])
        count = math.ceil((self.xmax-self.xmin)/step)
        if count > 400:
            raise ValueError("reference_grid_cap")
        self.arc = [(self.xmin, 0.0)]
        for i in range(count):
            a = self.xmin+(self.xmax-self.xmin)*i/count
            b = self.xmin+(self.xmax-self.xmin)*(i+1)/count
            length = (b-a)*(math.hypot(1, self.at_x(a)[1])+4*math.hypot(1, self.at_x((a+b)/2)[1])+
                            math.hypot(1, self.at_x(b)[1]))/6
            self.arc.append((b, self.arc[-1][1]+length))
        self.inverse_arc = [(s, x) for x, s in self.arc]
        self._s_cache = {}
        probes = [x for x, _ in self.arc]
        for a, b, coeffs in self.segments:
            probes.extend((a, b))
            if abs(coeffs[3]) > 1e-12:
                stationary = a-coeffs[2]/(3*coeffs[3])
                if a < stationary < b:
                    probes.append(stationary)
        self.max_slope = max(abs(self.at_x(x)[1]) for x in probes)
        self.max_second = max(abs(self.at_x(x)[2]) for x in probes)

    def at_x(self, x):
        if not self.xmin-1e-8 <= x <= self.xmax+1e-8:
            raise ValueError("curve_range")
        if x < self.points[0][0]:
            c = self.extension[2]
            u = x-self.points[0][0]
            return c[0]+u*(c[1]+u*(c[2]+u*c[3])), c[1]+u*(2*c[2]+3*c[3]*u), 2*c[2]+6*c[3]*u, 6*c[3]
        lo, hi = 0, len(self.segments)-1
        while lo < hi:
            mid = (lo+hi)//2
            if self.segments[mid][1] < x:
                lo = mid+1
            else:
                hi = mid
        origin, _, coeffs = self.segments[lo]
        u = x-origin
        a, b, c, d = coeffs
        return a+u*(b+u*(c+u*d)), b+u*(2*c+3*d*u), 2*c+6*d*u, 6*d

    def at_s(self, s):
        if s in self._s_cache:
            return self._s_cache[s]
        x = _lattice_interp(self.inverse_arc, s)
        y, slope, second, third = self.at_x(x)
        norm = math.hypot(1, slope)
        curvature = second/norm**3
        rate = third/norm**4-3*slope*second**2/norm**6
        result = (x, y, 1/norm, slope/norm, curvature, rate)
        if len(self._s_cache) < 600:
            self._s_cache[s] = result
        return result

    def project_origin(self):
        # Distance derivative roots on each cubic segment, plus endpoints.
        options = []
        intervals = ([(self.xmin, self.points[0][0])] if self.xmin < self.points[0][0] else [])
        intervals += [(a, b) for a, b, _ in self.segments]
        for a, b in intervals:
            mid, half = (a+b)/2, (b-a)/2
            y, dy, ddy, dddy = self.at_x(mid)
            c = [y, dy*half, ddy*half**2/2, dddy*half**3/6]
            dc = _lattice_derivative(c)
            derivative = [mid*half, half*half]+[0.0]*4
            for i, v in enumerate(c):
                for j, w in enumerate(dc):
                    derivative[i+j] += v*w
            for u in [-1.0, 1.0, *_control_poly_roots(derivative)]:
                x = mid+half*u
                options.append((x*x+_control_poly_eval(c, u)**2, x))
        _, x = min(options)
        if not self.xmin+1e-5 < x < self.xmax-1e-5:
            raise ValueError("projection_at_support_end")
        y, slope, _, _ = self.at_x(x)
        if abs(x+y*slope) > 1e-4:
            raise ValueError("nonorthogonal_projection")
        s = _lattice_interp(self.arc, x)
        return s, (-y+x*slope)/math.hypot(1, slope)

    def record(self):
        return {"type": "CUBIC_Y_OF_X_ARCLENGTH", "segments": deepcopy(self.segments),
                "extension": deepcopy(self.extension), "arc_table": list(self.arc),
                "observed_x_range_m": [self.points[0][0], self.xmax],
                "supported_x_range_m": [self.xmin, self.xmax]}

    @classmethod
    def from_record(cls, record):
        curve = cls.__new__(cls)
        curve.segments = deepcopy(record["segments"])
        curve.extension = deepcopy(record["extension"])
        curve.xmin, curve.xmax = record["supported_x_range_m"]
        curve.points = [(record["observed_x_range_m"][0], curve.segments[0][2][0])]
        curve.arc = [tuple(p) for p in record["arc_table"]]
        curve.inverse_arc = [(s, x) for x, s in curve.arc]
        curve._s_cache = {}
        return curve


def evaluate_frenet_path(reference, s_ref_m, now_s):
    """Reconstruct the original serialized V2 curve, never refit it for display."""
    try:
        if not reference.get("valid") or not reference["timestamp_s"] <= now_s < reference["timestamp_s"]+reference["valid_for_s"]:
            return None
        path = reference["frenet_path"]
        lateral = path["lateral"]
        if not finite_number(s_ref_m) or not lateral["range"][0] <= s_ref_m <= lateral["range"][1]:
            return None
        return _lattice_geometry(LatticeCurve.from_record(path["reference_curve"]), lateral["origin"],
            lateral["coeffs_low_to_high"], lateral["scale"], lateral["terminal_offset_m"], s_ref_m)
    except (KeyError, ValueError, TypeError, ArithmeticError):
        return None


def _lattice_lateral(d, first, second, target, length):
    b0, b1, b2 = d, first*length, second*length**2/2
    tail = _estimation_solve([[1, 1, 1], [3, 4, 5], [6, 12, 20]],
                            [target-b0-b1-b2, -b1-2*b2, -2*b2])
    if tail is None:
        raise ValueError("lateral_solve")
    return [b0, b1, b2, *tail]


def _lattice_geometry(curve, s0, coeffs, length, target, s):
    u = (s-s0)/length
    if u >= 1:
        d, first, second = target, 0.0, 0.0
    else:
        d = _control_poly_eval(coeffs, u)
        first = _control_poly_eval(_lattice_derivative(coeffs), u)/length
        second = _control_poly_eval(_lattice_derivative(coeffs, 2), u)/length**2
    x, y, tx, ty, k, dk = curve.at_s(s)
    a = 1-k*d
    q = math.hypot(a, first)
    if a <= 0.2 or q < 0.2:
        raise ValueError("frenet_degenerate")
    curvature = (k*(a*a+2*first*first)+a*second+dk*d*first)/q**3
    return {"s": s, "d": d, "x": x-d*ty, "y": y+d*tx,
            "heading": math.atan2(a*ty+first*tx, a*tx-first*ty),
            "curvature": curvature, "q": q, "support": "observed" if x >= curve.points[0][0] else "extrapolated"}


def _lattice_steering(curvature, model):
    rear, length = model["rear_axle_from_cg_m"], model["wheelbase_m"]
    den = 1-(rear*curvature)**2
    if den <= 0:
        raise ValueError("unreachable_cg_curvature")
    return math.atan(length*curvature/math.sqrt(den))


class FrenetLatticePlanner:
    """Bounded forward local planner with full body/static-point checks.

    It never enables a car or resumes a stopped framework. Unknown near road
    requires clear_start_assumed. Sampling is a bounded engineering collision
    approximation, not a continuous safety proof or a calibrated braking model.
    """
    def __init__(self, settings, lattice=None, frame_id="base_link", clock=None):
        self.settings, self.cfg, self.frame_id = settings, lattice or LatticeSettings(), frame_id
        self.clock = clock or time.perf_counter
        self.sequence, self.previous, self.stop_latched = 0, None, False
        self.observed_origin_seen = False

    def _model(self, limits):
        cfg = self.cfg
        names = ("wheelbase_m", "rear_axle_from_cg_m", "front_extent_m", "rear_extent_m", "width_m",
                 "steering_limit_rad", "steering_rate_limit_radps", "acceleration_max_mps2",
                 "braking_mps2", "lateral_acceleration_max_mps2", "actuation_delay_s", "speed_max_mps")
        model = {name: getattr(cfg, name) for name in names}
        model["source"] = cfg.model_source
        if cfg.model_source == "upstream":
            if not isinstance(limits, dict) or not limits.get("valid") or limits.get("source") in (None, "unmeasured"):
                raise ValueError("lattice_vehicle_limits_unavailable")
            model = {name: limits.get(name) for name in names}
            model["source"] = limits["source"]
            model["braking_mps2"] = limits.get("braking_deceleration_mps2")
            # No default fallback for missing measured motion limits.
            if any(not finite_number(model.get(name)) or model[name] <= 0 for name in names):
                raise ValueError("incomplete_lattice_vehicle_limits")
        return model

    def _clearance(self, geom, left, right, obstacles, model):
        cfg = self.cfg
        c, s = math.cos(geom["heading"]), math.sin(geom["heading"])
        # Boundary signed vertical distances converted to local normal metres.
        clearance = math.inf
        body_length = model["rear_extent_m"]+model["front_extent_m"]
        radius = math.hypot(max(model["front_extent_m"], model["rear_extent_m"]), model["width_m"]/2)
        sweep_guard = cfg.geometry_step_m*geom.get("q", 1.0)*(1+radius*abs(geom["curvature"]))/2
        # Include longitudinal edge subdivisions, and reserve an interpolation
        # sag bound for the curved boundaries between these footprint samples.
        edge_guard = max(left.max_second, right.max_second)*(body_length/4)**2/8
        for bx in [-model["rear_extent_m"]+body_length*i/4 for i in range(5)]:
            for by in (-model["width_m"]/2, model["width_m"]/2):
                x, y = geom["x"]+c*bx-s*by, geom["y"]+s*bx+c*by
                yl, sl, _, _ = left.at_x(x)
                yr, sr, _, _ = right.at_x(x)
                extra = cfg.extension_margin_growth*max(0.0, max(left.points[0][0], right.points[0][0])-x)
                clearance = min(clearance, (yl-y)/math.hypot(1, left.max_slope)-cfg.hard_margin_m-extra-edge_guard-sweep_guard,
                                (y-yr)/math.hypot(1, right.max_slope)-cfg.hard_margin_m-extra-edge_guard-sweep_guard)
        nearby = obstacles
        if isinstance(obstacles, dict):
            nearby = []
            reach = radius+cfg.obstacle_margin_m+cfg.hard_margin_m+sweep_guard
            for ix in range(math.floor((geom["x"]-reach)/0.25), math.floor((geom["x"]+reach)/0.25)+1):
                for iy in range(math.floor((geom["y"]-reach)/0.25), math.floor((geom["y"]+reach)/0.25)+1):
                    nearby.extend(obstacles.get((ix, iy), ()))
        for ox, oy in nearby:
            dx, dy = ox-geom["x"], oy-geom["y"]
            bx, by = c*dx+s*dy, -s*dx+c*dy
            outside_x = max(-model["rear_extent_m"]-bx, bx-model["front_extent_m"], 0.0)
            outside_y = max(abs(by)-model["width_m"]/2, 0.0)
            clearance = min(clearance, math.hypot(outside_x, outside_y)-cfg.obstacle_margin_m-cfg.hard_margin_m-sweep_guard)
        return clearance

    def plan(self, road, state, obstacles, vehicle_limits, now_s, source_timeout_s=None, mvp=False,
             _stop_before_x=None, _started=None):
        self.sequence += 1
        started, cfg = self.clock() if _started is None else _started, self.cfg
        self._consistency_cache = {}
        diag = {"candidates": [], "obstacle_response_enabled": cfg.obstacle_check_enabled,
                "model_source": cfg.model_source, "near_support": None, "initialization": {}}
        ref = {"schema_version": "planning_reference_v2", "planner_version": "frenet_lattice_v2",
               "trajectory_id": self.sequence, "timestamp_s": now_s, "generated_at_s": now_s,
               "frame_id": self.frame_id, "vehicle_reference_point": "cg_ground_projection",
               "valid": False, "status": "INVALID_INPUT", "reason": None, "valid_for_s": 0.0,
               "path": None, "target_speed_mps": 0.0, "speed_profile": None,
               "stop_requested": True, "stop_reason": None, "source_ages_s": {},
               "simulation_only": cfg.model_source == "course_simulation",
               "model_assumed": cfg.model_source != "upstream", "vehicle_limits_source": cfg.model_source}

        def fail(reason, status="INVALID_INPUT"):
            ref.update(reason=reason, stop_reason=reason, status=status)
            diag["elapsed_s"] = self.clock()-started
            self.previous = None
            return ref, diag

        def budget():
            if self.clock()-started > cfg.budget_s:
                raise TimeoutError("lattice_time_budget_exceeded")

        try:
            if not finite_number(now_s) or not isinstance(road, dict) or not isinstance(state, dict):
                raise ValueError("invalid_lattice_input")
            if (not road.get("valid") and road.get("status") == "near_field_unobserved"
                    and cfg.clear_start_assumed and not self.observed_origin_seen):
                forward = road.get("forward_geometry")
                if (isinstance(forward, dict) and forward.get("geometry_valid") is True
                        and forward.get("time_aligned") is True
                        and forward.get("status") == "near_field_unobserved"):
                    road = deepcopy(forward)
                    # Explicit planner assumption, never estimator acceptance.
                    road["valid"] = True
                    diag["controlled_near_extension"] = True
            if not road.get("valid") or road.get("visibility") != "both" or not state.get("speed_valid") or not state.get("yaw_rate_valid"):
                raise ValueError("invalid_estimates_or_boundaries")
            if any(obj.get("frame_id") != self.frame_id or not finite_number(obj.get("timestamp_s")) or
                   abs(obj["timestamp_s"]-now_s) > 1e-6 for obj in (road, state)):
                raise ValueError("lattice_frame_or_time_mismatch")
            speed, yaw = state.get("speed_mps"), state.get("yaw_rate_rps")
            if not finite_number(speed) or speed < 0 or not finite_number(yaw):
                raise ValueError("invalid_motion_state")
            ages = {"road": road.get("source_age_s"), **(state.get("source_age_s") or {})}
            deadlines = []
            for name in ("road", "wheel_speed", "imu_angular_velocity"):
                limit = self.settings.max_source_age_s
                if source_timeout_s is not None:
                    timeout = source_timeout_s.get(name)
                    if not finite_number(timeout) or timeout <= 0:
                        raise ValueError("invalid_source_timeout")
                    limit = min(limit, timeout)
                age = ages.get(name)
                if not finite_number(age) or not 0 <= age < limit:
                    raise ValueError("stale_"+name)
                deadlines.append(limit-age)
            speed_scale = road.get("recommended_speed_scale", 1.0)
            if diag.get("controlled_near_extension"):
                speed_scale = 1.0  # Explicit bounded startup; not prediction.
            if not finite_number(speed_scale) or not 0 < speed_scale <= 1:
                raise ValueError("invalid_road_speed_advisory")
            if road.get("degraded"):
                remaining = road.get("prediction_remaining_s")
                if not finite_number(remaining) or remaining <= 0:
                    raise ValueError("expired_road_prediction")
                deadlines.append(remaining)
            desired_speed = self.settings.cruise_speed_mps*speed_scale
            diag.update(road_degraded=bool(road.get("degraded")), recommended_speed_scale=speed_scale,
                        prediction_remaining_s=road.get("prediction_remaining_s"))
            obstacle_points = []
            if cfg.obstacle_check_enabled:
                if (not isinstance(obstacles, dict) or not obstacles.get("available") or
                    obstacles.get("frame_id") != self.frame_id or not finite_number(obstacles.get("timestamp_s")) or
                    abs(obstacles["timestamp_s"]-now_s) > 1e-6):
                    raise ValueError("obstacles_unavailable_or_unaligned")
                age = obstacles.get("source_age_s")
                limit = min(self.settings.max_source_age_s, (source_timeout_s or {}).get("obstacles", self.settings.max_source_age_s))
                if not finite_number(age) or not 0 <= age < limit:
                    raise ValueError("stale_obstacles")
                raw = obstacles.get("points_xyz_m")
                if not isinstance(raw, (list, tuple)) or len(raw) > cfg.max_obstacle_points:
                    raise ValueError("obstacle_point_cap_or_missing")
                for p in raw:
                    if not isinstance(p, (tuple, list)) or len(p) != 3 or not all(finite_number(v) for v in p):
                        raise ValueError("invalid_obstacle_point")
                    # Do not discard close objects on the basis of height.
                    if -cfg.rear_extent_m-0.5 <= p[0] <= cfg.horizon_m+cfg.front_extent_m+0.5 and abs(p[1]) <= 3:
                        obstacle_points.append((p[0], p[1]))
                ages["obstacles"] = age
                deadlines.append(limit-age)
            ref["source_ages_s"] = ages
            obstacle_index = {}
            for ox, oy in obstacle_points:
                obstacle_index.setdefault((math.floor(ox/0.25), math.floor(oy/0.25)), []).append((ox, oy))
            model = self._model(vehicle_limits)
            if speed > model["speed_max_mps"]:
                raise ValueError("initial_speed_exceeds_model")
            keys = ("centerline_xy", "left_boundary_xy", "right_boundary_xy")
            points = [CenterlinePlanner._points(road.get(k)) for k in keys]
            if any(p is None for p in points):
                raise ValueError("invalid_road_geometry")
            near, far = max(p[0][0] for p in points), min(p[-1][0] for p in points)
            if near <= 0 <= far:
                self.observed_origin_seen = True
            if near > cfg.max_near_gap_m+1e-9 or far <= near or far-near < 0.3:
                raise ValueError("insufficient_lattice_coverage")
            if near > 1e-6 and not cfg.clear_start_assumed:
                raise ValueError("near_gap_requires_clear_start_assumption")
            for p in points:
                if any(b[0]-a[0] > self.settings.max_sample_gap_m+1e-8 for a, b in zip(p, p[1:])):
                    raise ValueError("road_sample_gap")
            # All three curves share exactly the same observed x support.
            clipped = []
            for p in points:
                clipped.append([(near, CenterlinePlanner._interpolate(p, near)),
                                *[(x, y) for x, y in p if near < x < far],
                                (far, CenterlinePlanner._interpolate(p, far))])
            extension = max(0.0, near+model["rear_extent_m"]+0.15)
            if extension < 1e-9:
                extension = 0.0
            if extension > cfg.max_extension_m:
                raise ValueError("extension_limit")
            curve, left, right = _lattice_corridor(clipped, extension, cfg.geometry_step_m, cfg)
            s0, d0 = curve.project_origin()
            rx, ry, tx, ty, k, dk = curve.at_s(s0)
            heading_error = -math.atan2(ty, tx)
            if (abs(heading_error) > (cfg.initial_heading_max_rad if cfg.recovery_enabled else 0.5)
                    or (not cfg.recovery_enabled and abs(d0) > 0.4)):
                raise ValueError("initial_frenet_domain")
            a = 1-k*d0
            dprime = a*math.tan(heading_error)
            # Explicit low-speed forward approximation; no divide by near zero.
            initial_k = yaw/speed if speed >= 0.05 else 0.0
            if speed < 0.05 and abs(yaw) > (cfg.low_speed_yaw_tolerance_rps if cfg.recovery_enabled else 0.05):
                raise ValueError("low_speed_yaw_inconsistent")
            dsecond = (initial_k*(a*a+dprime*dprime)**1.5-k*(a*a+2*dprime*dprime)-dk*d0*dprime)/a
            sdot = speed/math.hypot(a, dprime)
            diag["initialization"] = {"s0": s0, "d0": d0, "dprime": dprime, "dsecond": dsecond,
                                      "curvature_method": "yaw_over_speed" if speed >= 0.05 else "low_speed_zero_assumption",
                                      "acceleration_method": "zero_start_assumption"}
            diag["near_support"] = {"observed_x_start_m": near, "extension_m": extension,
                                     "method": "c2_curvature_fade" if extension > 0 else "observed_support",
                                     "clear_start_assumed": cfg.clear_start_assumed}
            # Trim for full body, with extra allowance for orientation projection.
            end_x = far-math.hypot(model["front_extent_m"], model["width_m"]/2)-0.02
            if _stop_before_x is not None:
                end_x = min(end_x, _stop_before_x)
            end_s = min(s0+cfg.horizon_m, _lattice_interp(curve.arc, end_x))
            horizon = end_s-s0
            if horizon < cfg.minimum_output_m:
                raise ValueError("insufficient_body_supported_horizon")
            reference_record = curve.record()
            # Transform previous path using measured current twist over the actual
            # short frame interval. It is a scoring hint only, never a fallback.
            previous = []
            if self.previous is not None:
                dt = now_s-self.previous["timestamp_s"]
                if 0 < dt < self.previous["valid_for_s"]:
                    turn = yaw*dt
                    dx = speed*dt if abs(yaw) < 1e-8 else speed*math.sin(turn)/yaw
                    dy = 0 if abs(yaw) < 1e-8 else speed*(1-math.cos(turn))/yaw
                    c, sn = math.cos(turn), math.sin(turn)
                    previous = [(c*(p[0]-dx)+sn*(p[1]-dy), -sn*(p[0]-dx)+c*(p[1]-dy)) for p in self.previous["points"]]
                    if any(b[0] <= a[0] for a, b in zip(previous, previous[1:])):
                        previous = []
            if self.stop_latched:
                if not previous:
                    raise ValueError("planned_stop_requires_explicit_restart")
                if _stop_before_x is None:
                    # Commit to the previous stop endpoint in the new frame;
                    # disappearance of a lidar return never resumes a stop.
                    return self.plan(road, state, obstacles, vehicle_limits, now_s, source_timeout_s, mvp,
                                     _stop_before_x=previous[-1][0], _started=started)
            accepted = []
            count = 0
            targets = cfg.samples("terminal_offsets_m")
            if all(abs(d0-target) > 1e-6 for target in targets):
                targets.append(d0)  # Preserve current offset as a real option.
            targets.sort(key=lambda target: (abs(target), target))
            search_truncated = False
            for length in sorted(cfg.samples("transition_lengths_m"), reverse=True):
                if search_truncated:
                    break
                for target in targets:
                    # This is a deliberate search cutoff, NOT a total deadline
                    # overrun. Reserve 40% for ranking and output revalidation.
                    if accepted and self.clock()-started >= cfg.budget_s*0.6:
                        search_truncated = True
                        break
                    budget()
                    row = {"length_m": length, "offset_m": target, "reason": None}
                    diag["candidates"].append(row)
                    if length > horizon+1e-8:
                        row["reason"] = "transition_exceeds_common_horizon"
                        continue
                    coeffs = _lattice_lateral(d0, dprime, dsecond, target, length)
                    grid_n = math.ceil(horizon/cfg.geometry_step_m)
                    geometries = []
                    try:
                        for i in range(grid_n+1):
                            if i % 8 == 0:
                                budget()
                            g = _lattice_geometry(curve, s0, coeffs, length, target, s0+horizon*i/grid_n)
                            g["steering"] = _lattice_steering(g["curvature"], model)
                            if abs(g["steering"]) > model["steering_limit_rad"]:
                                raise ValueError("steering_limit")
                            if g["support"] == "extrapolated" and (abs(g["d"]-d0) > cfg.blind_lateral_change_m or
                                   abs(g["steering"]) > cfg.blind_steering_limit_rad):
                                raise ValueError("blind_correction_limit")
                            g["clearance"] = self._clearance(g, left, right, obstacle_index, model)
                            if g["clearance"] < 0:
                                raise ValueError("body_boundary_or_obstacle_collision")
                            if geometries and g["x"] <= geometries[-1]["x"]:
                                raise ValueError("nonmonotonic_cartesian_path")
                            g["distance"] = (geometries[-1]["distance"]+math.hypot(g["x"]-geometries[-1]["x"],
                                g["y"]-geometries[-1]["y"])) if geometries else 0.0
                            geometries.append(g)
                    except ValueError as error:
                        row["reason"] = str(error)
                        continue
                    lateral = {"coeffs_low_to_high": coeffs, "origin": s0, "scale": length,
                               "range": [s0, end_s], "transition_end_s": s0+length, "terminal_offset_m": target}
                    # Current speed must be stoppable within checked full-body path.
                    distance = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a, b in zip(geometries, geometries[1:]))
                    if speed*model["actuation_delay_s"]+speed**2/(2*model["braking_mps2"])+cfg.hard_margin_m > distance:
                        row["reason"] = "current_speed_cannot_stop_in_horizon"
                        continue
                    times = [(t, v*speed_scale, False) for t in cfg.samples("durations_s") for v in cfg.samples("terminal_speeds_mps")]
                    # Always include a terminal-stop quintic at the common end.
                    times.append((cfg.score_time_s, 0.0, True))
                    for duration, terminal_speed, stopping in times:
                        if _stop_before_x is not None and not stopping:
                            continue
                        if stopping and _stop_before_x is None and any(not v[1]["stop"] for v in accepted):
                            continue
                        budget()
                        count += 1
                        if count > cfg.max_candidates:
                            raise TimeoutError("lattice_candidate_cap")
                        candidate = {**row, "duration_s": duration, "terminal_speed_mps": terminal_speed, "stop": stopping}
                        diag["candidates"].append(candidate)
                        try:
                            # Convert physical terminal speed using terminal Q.
                            terminal_srate = terminal_speed/geometries[-1]["q"]
                            if not stopping:
                                q_table = [(g["s"], g["q"]) for g in geometries]
                                # Quartic end s depends on the end rate. Resolve
                                # Q at that actual end, not at the spatial horizon.
                                for _ in range(6):
                                    predicted_end = s0+duration*(sdot+terminal_srate)/2
                                    if not s0 <= predicted_end <= end_s:
                                        raise ValueError("longitudinal_exceeds_support")
                                    terminal_srate = terminal_speed/_lattice_interp(q_table, predicted_end)
                            stop_end, polynomial_duration = end_s, duration
                            if stopping and sdot > 1e-6:
                                # Stop promptly rather than crawl all the way to
                                # the horizon. Short profiles hold their end to
                                # cover the common 3-second scoring window.
                                stop_end = min(end_s, s0+sdot*duration/2)
                                polynomial_duration = 2*(stop_end-s0)/sdot
                            longitudinal = self._longitudinal(s0, sdot, stop_end, polynomial_duration, terminal_srate, stopping)
                            motion = self._motion(curve, lateral, longitudinal, model, geometries, speed, budget)
                            if "spatial_costs" not in row:
                                row["spatial_costs"] = self._spatial_score(geometries, previous, curve)
                            costs = self._score(motion, speed, stopping, row["spatial_costs"], desired_speed)
                            total = sum(getattr(cfg, "weight_"+key)*value for key, value in costs.items())
                            candidate.update(reason="feasible", costs=costs, cost=total, score=-total)
                            accepted.append((total, candidate, lateral, longitudinal, geometries, motion))
                        except ValueError as error:
                            candidate["reason"] = str(error)
                    row["reason"] = "geometry_feasible"
            if not accepted:
                if _stop_before_x is None and obstacle_points:
                    # If no full-horizon bypass exists, establish a new common
                    # STOP horizon before the nearest point inside the road.
                    # Do not compare short stopped paths against cruise paths.
                    blockers = [ox for ox, oy in obstacle_points if near <= ox <= far and
                                right.at_x(ox)[0] <= oy <= left.at_x(ox)[0]]
                    if blockers:
                        stop_x = min(blockers)-model["front_extent_m"]-cfg.hard_margin_m-cfg.obstacle_margin_m-0.08
                        result, stop_diag = self.plan(road, state, obstacles, vehicle_limits, now_s,
                            source_timeout_s, mvp, _stop_before_x=stop_x, _started=started)
                        stop_diag["cruise_rejections"] = diag["candidates"]
                        stop_diag["behavior"] = "obstacle_stop"
                        return result, stop_diag
                return fail("no_feasible_lattice_trajectory", "NO_FEASIBLE_PATH")
            # Behaviour is selected before soft scoring: voluntary long-horizon
            # stop must not win against safe cruise merely by moving very slowly.
            cruising = [item for item in accepted if not item[1]["stop"]]
            if cruising:
                accepted = cruising
            minimum = min(item[0] for item in accepted)
            near_best = [v for v in accepted if v[0] <= minimum+cfg.tie_tolerance]
            near_best.sort(key=lambda v: (v[1]["costs"]["consistency"] if previous else 0,
                              -min(g["clearance"] for g in v[4]), v[1]["costs"]["steering"],
                              v[1]["length_m"], v[1]["offset_m"], v[1]["duration_s"]))
            ordered = near_best+sorted([v for v in accepted if v not in near_best], key=lambda v: v[0])
            for _, choice, lateral, longitudinal, geometries, motion in ordered:
                budget()
                try:
                    if cfg.direct_sample_output:
                        path, errors = self._sampled_output(geometries, budget)
                    else:
                        path, errors = self._compatibility(geometries, left, right, obstacle_index, model, budget,
                                                          lateral["scale"], max(p["speed"] for p in motion))
                except ValueError as error:
                    choice["reason"] = ("sample_output_" if cfg.direct_sample_output else "compatibility_")+str(error)
                    continue
                lifetime = min(self.settings.reference_lifetime_s, *deadlines)
                if self.clock()-started >= lifetime:
                    raise TimeoutError("source_deadline_during_planning")
                # A positive preview target permits starting from zero without
                # replacing the longitudinal initial speed by the desired speed.
                target_speed = _lattice_interp([(p["t"], p["speed"]) for p in motion], min(cfg.speed_preview_s, longitudinal["duration_s"]))
                if road.get("degraded") and not choice["stop"]:
                    target_speed = min(target_speed, desired_speed)
                ref.update(valid=True, status="STOP" if choice["stop"] else "TRACK", reason="lattice_selected",
                           valid_for_s=lifetime, path=path, target_speed_mps=target_speed,
                           stop_requested=False, stop_reason=None,
                           planned_stop=choice["stop"], stop_at_path_m=_lattice_interp(
                               [(g["s"], g["distance"]) for g in geometries], motion[-1]["s"]) if choice["stop"] else None,
                           speed_profile=[(p["t"], p["speed"]) for p in motion],
                           longitudinal_profile=longitudinal,
                           frenet_path={"type": "FRENET_SPATIAL", "reference_curve": reference_record, "lateral": lateral},
                           control_preview_m=cfg.preview_m, support=diag["near_support"],
                           operating_profile="lattice_approximate" if ref["model_assumed"] else "lattice_measured")
                diag.update(selected=choice, output_errors=errors, output_encoding=path["type"], elapsed_s=self.clock()-started,
                            longitudinal_candidates=count, consistency_method="short_constant_twist_hint" if previous else "unavailable")
                self.previous = {"timestamp_s": now_s, "valid_for_s": lifetime,
                                 "points": [(g["x"], g["y"]) for g in geometries]}
                self.stop_latched = self.stop_latched or choice["stop"]
                diag["search_truncated"] = search_truncated
                diag["selection_scope"] = "best_of_evaluated_feasible_candidates"
                return ref, diag
            return fail("no_valid_cartesian_output", "NO_FEASIBLE_PATH")
        except TimeoutError as error:
            return fail(str(error), "TIME_BUDGET_EXCEEDED")
        except (ValueError, TypeError, ArithmeticError) as error:
            return fail(str(error))

    def _longitudinal(self, s0, initial_rate, end, duration, terminal_speed, stopping):
        # Normalized coefficients; longitudinal acceleration initially assumed 0.
        c = [s0, initial_rate*duration, 0.0]
        if stopping:
            if initial_rate <= 1e-6:
                return {"type": "S_REF_OF_TIME", "coeffs_low_to_high": [s0], "duration_s": duration,
                        "time_range_s": [0.0, duration], "reference_s_range_m": [s0, end], "stop": True}
            tail = _estimation_solve([[1, 1, 1], [3, 4, 5], [6, 12, 20]],
                                    [end-s0-c[1], -c[1], 0.0])
            c += tail
        else:
            # Caller supplies the reference-line rate converted from physical speed.
            change = terminal_speed*duration-c[1]
            c += [change, -change/2]
        return {"type": "S_REF_OF_TIME", "coeffs_low_to_high": c, "duration_s": duration,
                "time_range_s": [0.0, duration], "reference_s_range_m": [s0, end], "stop": stopping}

    def _motion(self, curve, lateral, longitudinal, model, geometries, initial_speed, budget):
        cfg = self.cfg
        duration, c = longitudinal["duration_s"], longitudinal["coeffs_low_to_high"]
        evaluation_time = max(duration, cfg.score_time_s)
        n = math.ceil(evaluation_time/cfg.time_step_s)
        if n > 600:
            raise ValueError("longitudinal_sampling_cap")
        rates, accelerations = _lattice_derivative(c), _lattice_derivative(c, 2)
        motion = []
        position_grid = [g["s"] for g in geometries]
        def sample_geometry(s):
            lo, hi = 0, len(geometries)-1
            while hi-lo > 1:
                mid = (lo+hi)//2
                if position_grid[mid] < s:
                    lo = mid
                else:
                    hi = mid
            a, b = geometries[lo], geometries[hi]
            u = (s-a["s"])/(b["s"]-a["s"])
            return {key: a[key]+u*(b[key]-a[key]) for key in ("q", "curvature", "steering", "distance")}
        for i in range(n+1):
            if i % 8 == 0:
                budget()
            t = evaluation_time*i/n
            u = min(1.0, t/duration)
            s = _control_poly_eval(c, u)
            ds = _control_poly_eval(rates, u)/duration
            if ds < -1e-7 or not lateral["origin"]-1e-7 <= s <= lateral["range"][1]+1e-7:
                raise ValueError("longitudinal_reverses_or_exceeds_support")
            s = min(lateral["range"][1], max(lateral["origin"], s))
            g = sample_geometry(s)
            speed = max(0.0, ds)*g["q"]
            steering = g["steering"]
            acceleration = 0.0 if not motion else (speed-motion[-1]["speed"])/(t-motion[-1]["t"])
            steering_rate = 0.0 if not motion else (steering-motion[-1]["steering"])/(t-motion[-1]["t"])
            remaining = geometries[-1]["distance"]-g["distance"]
            # Cruise is a soft target, not a reason to reject measured overshoot.
            # Every profile must still respect the physical/development speed cap.
            if speed > model["speed_max_mps"]+1e-5:
                raise ValueError("speed_limit")
            if acceleration > model["acceleration_max_mps2"]+1e-5 or acceleration < -model["braking_mps2"]-1e-5:
                raise ValueError("acceleration_limit")
            if longitudinal["stop"] and acceleration > 1e-4:
                raise ValueError("stop_trajectory_accelerates")
            if abs(speed*speed*g["curvature"]) > model["lateral_acceleration_max_mps2"]:
                raise ValueError("lateral_acceleration_limit")
            if abs(steering_rate) > model["steering_rate_limit_radps"]:
                raise ValueError("steering_rate_limit")
            if not longitudinal["stop"] and speed*model["actuation_delay_s"]+speed**2/(2*model["braking_mps2"])+cfg.hard_margin_m > remaining:
                raise ValueError("visibility_stopping_limit")
            motion.append({"t": t, "s": s, "speed": speed, "acceleration": acceleration,
                           "steering": steering, "steering_rate": steering_rate})
        if abs(motion[0]["speed"]-initial_speed) > 1e-4:
            raise ValueError("initial_speed_mismatch")
        return motion

    def _spatial_score(self, geometries, previous, curve):
        cfg = self.cfg
        weighted, denominator, clearance_values = 0.0, 0.0, []
        consistency = []
        for a, b in zip(geometries, geometries[1:]):
            ds = b["s"]-a["s"]
            for g in (a, b):
                weight = cfg.blind_center_weight if g["support"] == "extrapolated" else 1.0
                weighted += ds/2*weight*(g["d"]/cfg.d_scale_m)**2
                denominator += ds/2*weight
            clearance_values.append((ds, (max(0, 1-a["clearance"]/cfg.clearance_soft_m)**2+
                                               max(0, 1-b["clearance"]/cfg.clearance_soft_m)**2)/2))
        for g in geometries:
            if g["s"]-geometries[0]["s"] > cfg.consistency_range_m:
                break
            if previous:
                if g["s"] not in self._consistency_cache:
                    rx, ry, tx, ty, _, _ = curve.at_s(g["s"])
                    # Intersect previous polyline with the SAME reference normal.
                    hits = []
                    for p, q in zip(previous, previous[1:]):
                        ap = (p[0]-rx)*tx+(p[1]-ry)*ty
                        aq = (q[0]-rx)*tx+(q[1]-ry)*ty
                        if ap*aq <= 0 and abs(aq-ap) > 1e-10:
                            u = -ap/(aq-ap)
                            px, py = p[0]+u*(q[0]-p[0]), p[1]+u*(q[1]-p[1])
                            hits.append(-(px-rx)*ty+(py-ry)*tx)
                    unique = sorted(set(round(v, 10) for v in hits))
                    self._consistency_cache[g["s"]] = unique[0] if len(unique) == 1 else None
                old_d = self._consistency_cache[g["s"]]
                if old_d is not None:
                    consistency.append(((g["d"]-old_d)/cfg.d_scale_m)**2)
        peak = max(max(0, 1-g["clearance"]/cfg.clearance_soft_m)**2 for g in geometries)
        clearance = (1-cfg.clearance_peak_ratio)*sum(ds*v for ds, v in clearance_values)/sum(ds for ds, _ in clearance_values)+cfg.clearance_peak_ratio*peak
        return {"center": weighted/denominator, "clearance": clearance,
                "consistency": sum(consistency)/len(consistency) if consistency else 0.0}

    def _score(self, motion, initial_speed, stopping, spatial_costs, cruise_speed=None):
        cfg = self.cfg
        steering_cost, motion_cost, total_t = 0.0, 0.0, 0.0
        for a, b in zip(motion, motion[1:]):
            dt = max(0.0, min(b["t"], cfg.score_time_s)-a["t"])
            if dt == 0:
                break
            for p in (a, b):
                steering_cost += dt/2*((p["steering"]/cfg.steering_scale_rad)**2+
                    cfg.steering_rate_weight*(p["steering_rate"]/cfg.steering_rate_scale_radps)**2)/(1+cfg.steering_rate_weight)
                desired = max(0.0, initial_speed*(1-p["t"]/cfg.score_time_s)) if stopping else (
                    self.settings.cruise_speed_mps if cruise_speed is None else cruise_speed)
                motion_cost += dt/2*(((p["speed"]-desired)/cfg.speed_scale_mps)**2+
                    cfg.acceleration_weight*(p["acceleration"]/cfg.acceleration_scale_mps2)**2)/(1+cfg.acceleration_weight)
            total_t += dt
        return {**spatial_costs, "steering": steering_cost/total_t, "motion": motion_cost/total_t}

    def _sampled_output(self, geometries, budget):
        """Reuse the full checked Frenet geometry, with no Cartesian refit.

        Body/obstacle/steering checks ran at these exact points and longitudinal
        limits ran before selection. Export at most 201 points, O(N); do not
        decimate the checked grid or add another trajectory search.
        """
        if not 2 <= len(geometries) <= 201:
            raise ValueError("sampling_cap")
        points = []
        for i, g in enumerate(geometries):
            if i % 8 == 0:
                budget()
            points.append((g["x"], g["y"], g["heading"], g["curvature"]))
        path = {"type": "CARTESIAN_SAMPLES", "independent_variable": "x_m",
                "range": [points[0][0], points[-1][0]],
                "samples_xy_heading_curvature": points}
        _control_sample_points({"path": path})
        if points[-1][0]-points[0][0] < self.cfg.minimum_output_m:
            raise ValueError("short_output")
        budget()
        return path, {"conversion": "none", "sample_count": len(points)}

    def _compatibility(self, geometries, left, right, obstacles, model, budget, transition_length, maximum_speed):
        cfg = self.cfg
        # Only expose the continuous initial polynomial portion to the current
        # single-polynomial consumer. The full Frenet trajectory stays in ref.
        start_s = geometries[0]["s"]
        geometries = [g for g in geometries if g["s"] <= start_s+transition_length+1e-8]
        # Initial position, heading and curvature are interpolation constraints;
        # solve the remaining cubic..quintic coefficients on normalized x.
        origin, end = geometries[0]["x"], geometries[-1]["x"]
        scale = end-origin
        if scale < cfg.minimum_output_m:
            raise ValueError("short_output")
        first = geometries[0]
        slope = math.tan(first["heading"])
        c = [first["y"], slope*scale, first["curvature"]*math.hypot(1, slope)**3*scale**2/2]
        matrix, rhs = [[0.0]*3 for _ in range(3)], [0.0]*3
        for g in geometries:
            u = (g["x"]-origin)/scale
            row = [u**3, u**4, u**5]
            residual = g["y"]-_control_poly_eval(c, u)
            for i in range(3):
                rhs[i] += row[i]*residual
                for j in range(3):
                    matrix[i][j] += row[i]*row[j]
        tail = _estimation_solve(matrix, rhs)
        if tail is None:
            raise ValueError("fit_singular")
        c += tail
        path = {"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m", "origin": origin,
                "scale": scale, "coeffs_low_to_high": c, "range": [origin, end]}
        errors = {"position_m": 0.0, "heading_rad": 0.0, "curvature_1pm": 0.0}
        for i, g in enumerate(geometries):
            if i % 8 == 0:
                budget()
            u = (g["x"]-origin)/scale
            y = _control_poly_eval(c, u)
            dy = _control_poly_eval(_lattice_derivative(c), u)/scale
            ddy = _control_poly_eval(_lattice_derivative(c, 2), u)/scale**2
            third = _control_poly_eval(_lattice_derivative(c, 3), u)/scale**3
            heading, curvature = math.atan(dy), ddy/math.hypot(1, dy)**3
            errors["position_m"] = max(errors["position_m"], abs(y-g["y"]))
            errors["heading_rad"] = max(errors["heading_rad"], abs(wrap_angle(heading-g["heading"])))
            errors["curvature_1pm"] = max(errors["curvature_1pm"], abs(curvature-g["curvature"]))
            if (errors["position_m"] > cfg.fit_position_error_m or errors["heading_rad"] > cfg.fit_heading_error_rad or
                    errors["curvature_1pm"] > cfg.fit_curvature_error_1pm):
                raise ValueError("fit_error")
            if abs(_lattice_steering(curvature, model)) > model["steering_limit_rad"]:
                raise ValueError("fit_steering_limit")
            curvature_rate_x = third/math.hypot(1, dy)**3-3*dy*ddy**2/math.hypot(1, dy)**5
            den = 1-(model["rear_axle_from_cg_m"]*curvature)**2
            steering_rate = abs(curvature_rate_x)*maximum_speed/math.hypot(1, dy)*model["wheelbase_m"]/(math.sqrt(den)*(
                1+(model["wheelbase_m"]**2-model["rear_axle_from_cg_m"]**2)*curvature**2))
            if steering_rate > model["steering_rate_limit_radps"] or maximum_speed**2*abs(curvature) > model["lateral_acceleration_max_mps2"]:
                raise ValueError("fit_motion_limit")
            if self._clearance({**g, "y": y, "heading": heading}, left, right, obstacles, model) < 0:
                raise ValueError("fit_collision")
        return path, errors


class MvpRecoveryPlanner:
    """Bounded local centerline following, with a controller rollout.

    Uses observed two-sided corridors and the existing approximate vehicle.
    It makes no global route decision. K <= 120 footprint checks; no candidate
    product or quintic output conversion. Only the worker runs these checks.
    """
    def __init__(self, planner, control):
        self.planner, self.control = planner, control

    def plan(self, road, state, obstacles, limits, now_s, timeouts):
        started, cfg = time.perf_counter(), self.planner.cfg
        ref = {"valid": False, "reason": "recovery_unavailable", "timestamp_s": now_s,
               "frame_id": self.planner.frame_id, "vehicle_reference_point": "cg_ground_projection",
               "target_speed_mps": 0.0, "stop_requested": True, "valid_for_s": 0.0}
        diag = {"planner": "bounded_centerline", "rollout_steps": 0}
        try:
            if cfg.obstacle_check_enabled:
                raise ValueError("recovery_requires_cone_only_profile")
            if not road.get("valid"):
                if (cfg.clear_start_assumed and not self.planner.observed_origin_seen
                        and road.get("status") == "near_field_unobserved"):
                    road = road.get("forward_geometry") or {}
                    if not road.get("geometry_valid") or not road.get("time_aligned"):
                        raise ValueError("invalid_recovery_road")
                else:
                    raise ValueError("invalid_recovery_road")
            if (not state.get("valid", state.get("speed_valid") and state.get("yaw_rate_valid"))
                    or state.get("frame_id") != self.planner.frame_id
                    or road.get("frame_id") != self.planner.frame_id
                    or abs(road.get("timestamp_s", math.inf)-now_s) > 1e-6):
                raise ValueError("recovery_frame_or_motion_invalid")
            remaining = [self.planner.settings.reference_lifetime_s]
            ages = state.get("source_age_s") or {}
            for name in ("wheel_speed", "imu_angular_velocity"):
                remaining.append(timeouts.get(name, 0.15)-ages.get(name, math.inf))
            age = road.get("source_age_s", math.inf)
            remaining.append(min(self.planner.settings.max_source_age_s, timeouts.get("road", 0.4))-age)
            if road.get("degraded"):
                remaining.append(road.get("prediction_remaining_s", 0.0))
            lifetime = min(remaining)
            if not finite_number(lifetime) or lifetime <= 0:
                raise ValueError("source_deadline_during_planning")
            points = [CenterlinePlanner._points(road.get(k)) for k in
                      ("centerline_xy", "left_boundary_xy", "right_boundary_xy")]
            if any(p is None or len(p) > 201 for p in points):
                raise ValueError("invalid_recovery_geometry")
            near, far = max(p[0][0] for p in points), min(p[-1][0] for p in points)
            if near > cfg.max_near_gap_m or far-near < cfg.minimum_output_m:
                raise ValueError("insufficient_lattice_coverage")
            model = self.planner._model(limits)
            if not finite_number(state.get("speed_mps")) or not 0 <= state["speed_mps"] <= model["speed_max_mps"]:
                raise ValueError("invalid_recovery_speed")
            if state["speed_mps"] < 0.05 and abs(state["yaw_rate_rps"]) > cfg.low_speed_yaw_tolerance_rps:
                raise ValueError("low_speed_yaw_inconsistent")
            extension = max(0.0, near+model["rear_extent_m"]+0.15)
            if extension > cfg.max_extension_m or (near > 0 and not cfg.clear_start_assumed):
                raise ValueError("extension_limit")
            clipped = [[(near, CenterlinePlanner._interpolate(p, near)),
                        *[(x, y) for x, y in p if near < x < far],
                        (far, CenterlinePlanner._interpolate(p, far))] for p in points]
            # Quadratic fit on normalized coordinates: fixed 3x3 solve, O(N).
            center = clipped[0]
            origin, scale = near, far-near
            rows = [[1.0, (x-origin)/scale, ((x-origin)/scale)**2] for x, _ in center]
            coeffs = _estimation_solve([[sum(r[i]*r[j] for r in rows) for j in range(3)] for i in range(3)],
                                      [sum(r[i]*p[1] for r, p in zip(rows, center)) for i in range(3)])
            if coeffs is None or max(abs(_control_poly_eval(coeffs, r[1])-p[1]) for r, p in zip(rows, center)) > 0.04:
                raise ValueError("recovery_fit_error")
            left, right = _lattice_corridor(clipped, extension, 0.05, cfg)[1:]
            lo, hi = max(0.0, near-extension), min(cfg.horizon_m, far-math.hypot(model["front_extent_m"], model["width_m"]/2)-0.04)
            if hi-lo < cfg.minimum_output_m:
                raise ValueError("insufficient_body_supported_horizon")
            def geometry(x):
                u = (x-origin)/scale
                y = _control_poly_eval(coeffs, u)
                dy = (coeffs[1]+2*coeffs[2]*u)/scale
                curvature = 2*coeffs[2]/scale**2/math.hypot(1, dy)**3
                return y, math.atan(dy), curvature
            if abs(geometry(0.0)[1]) >= cfg.initial_heading_max_rad:
                raise ValueError("initial_frenet_domain")
            # Check the predicted controller response from the actual body pose,
            # rather than rejecting a fixed lateral offset or testing only the
            # centerline footprint. This is an approximate rollout, not calibration.
            if (state["speed_mps"]*model["actuation_delay_s"]+state["speed_mps"]**2/(2*model["braking_mps2"])
                    +cfg.hard_margin_m > hi-lo):
                raise ValueError("current_speed_cannot_stop_in_horizon")
            x = y = heading = delta = 0.0
            if state["speed_mps"] >= 0.05:
                initial_k = state["yaw_rate_rps"]/state["speed_mps"]
                if abs(model["rear_axle_from_cg_m"]*initial_k) >= 1:
                    raise ValueError("initial_curvature_outside_model")
                delta = math.atan(model["wheelbase_m"]*initial_k/
                                  math.sqrt(1-(model["rear_axle_from_cg_m"]*initial_k)**2))
            step = 0.025
            minimum_clearance = math.inf
            for i in range(120):
                curvature = math.cos(math.atan(model["rear_axle_from_cg_m"]/model["wheelbase_m"]*math.tan(delta)))*math.tan(delta)/model["wheelbase_m"]
                g = {"x": x, "y": y, "heading": heading, "curvature": curvature, "q": 1.0}
                clearance = self.planner._clearance(g, left, right, [], model)
                minimum_clearance = min(minimum_clearance, clearance)
                if clearance < 0:
                    raise ValueError("recovery_body_boundary_conflict")
                diag["rollout_steps"] = i+1
                if x >= hi-step:
                    break
                px = min(hi, max(lo, x+cfg.preview_m*math.cos(heading)))
                py, tangent, k = geometry(px)
                ey = -math.sin(heading)*(px-x)+math.cos(heading)*(py-y)
                steer = self.control.mvp_lateral_kp*ey+self.control.mvp_heading_kp*wrap_angle(tangent-heading)
                steer = max(-self.control.steering_max_normalized, min(self.control.steering_max_normalized, steer))
                requested = steer*model["steering_limit_rad"]
                # Rate limit using the faster of observed/commanded crawl speeds.
                ds_time = step/max(state["speed_mps"], cfg.recovery_speed_mps)
                change = model["steering_rate_limit_radps"]*ds_time
                delta += max(-change, min(change, requested-delta))
                beta = math.atan(model["rear_axle_from_cg_m"]/model["wheelbase_m"]*math.tan(delta))
                nx = x+step*math.cos(heading+beta)
                if nx <= x:
                    raise ValueError("recovery_not_forward")
                x, y = nx, y+step*math.sin(heading+beta)
                heading = wrap_angle(heading+step*math.cos(beta)*math.tan(delta)/model["wheelbase_m"])
                if time.perf_counter()-started >= min(cfg.budget_s, lifetime):
                    raise ValueError("lattice_time_budget_exceeded")
            else:
                raise ValueError("recovery_rollout_cap")
            elapsed = time.perf_counter()-started
            if elapsed >= min(cfg.budget_s, lifetime):
                raise ValueError("lattice_time_budget_exceeded" if cfg.budget_s <= lifetime
                                 else "source_deadline_during_planning")
            if near <= 0 <= far:
                self.planner.observed_origin_seen = True
            correction = abs(geometry(0.0)[0]) > 0.08 or abs(geometry(0.0)[1]) > 0.15
            speed = min(self.planner.settings.cruise_speed_mps,
                        cfg.recovery_speed_mps if correction else self.planner.settings.cruise_speed_mps)
            if road.get("degraded"):
                speed = min(speed, self.planner.settings.cruise_speed_mps*road["recommended_speed_scale"])
            ref.update(valid=True, reason="centerline_recovery" if correction else "centerline_tracking",
                       planner_version="mvp2_centerline", schema_version="planning_reference_v2",
                       valid_for_s=lifetime, stop_requested=False, planned_stop=False, target_speed_mps=speed,
                       control_preview_m=cfg.preview_m,
                       path={"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m", "origin": origin,
                             "scale": scale, "coeffs_low_to_high": coeffs, "range": [lo, hi]},
                       simulation_only=cfg.model_source == "course_simulation", operating_profile="mvp2_local_corridor")
            diag.update(minimum_clearance_m=minimum_clearance, correction=correction)
        except (ValueError, TypeError, ArithmeticError) as error:
            ref["reason"] = str(error)
            diag.update(reason=str(error), body_conflict=str(error) == "recovery_body_boundary_conflict")
        diag["elapsed_s"] = time.perf_counter()-started
        return ref, diag


def _mvp_plan(planner, control, args):
    """Simple feasible corridor first; lattice only when it adds a solution."""
    if planner.cfg.recovery_enabled and control.mode == "mvp" and not planner.stop_latched:
        ref, simple = MvpRecoveryPlanner(planner, control).plan(*args)
        if ref["valid"]:
            return ref, simple
        if simple.get("elapsed_s", 0) >= planner.cfg.budget_s:
            return ref, simple
    else:
        simple = None
    ref, diag = planner.plan(*args, mvp=control.mode == "mvp",
                             _started=planner.clock()-simple["elapsed_s"] if simple else None)
    if simple:
        diag["centerline_attempt"] = simple
        diag["centerline_rejection"] = simple.get("reason")
        # An observed corridor conflict is never hidden by the cache.
        if not ref["valid"] and simple.get("body_conflict"):
            ref["reason"] = "recovery_body_boundary_conflict"
    if ref.get("reason") == "no_valid_cartesian_output":
        conversion_reasons = {row.get("reason") for row in diag.get("candidates", [])
                              if str(row.get("reason", "")).startswith("compatibility_")}
        if conversion_reasons and conversion_reasons <= {
                "compatibility_fit_singular", "compatibility_fit_error", "compatibility_short_output"}:
            ref["reason"] = "cartesian_conversion_failed"
    return ref, diag


def _planner_worker_main():
    planner, generation = None, None
    # Startup objects are permanent: keep them out of later collections so a
    # full collection cannot stall a planning job (see main()).
    gc.collect()
    gc.freeze()
    sys.stdout.write('{"ready":true}\n')
    sys.stdout.flush()
    for line in sys.stdin:
        job = {}
        try:
            job = json.loads(line)
            if planner is None or generation != job["generation"]:
                planner = FrenetLatticePlanner(PlanningSettings(**job["planning"]),
                                              LatticeSettings(**job["lattice"]), job["frame_id"])
                generation = job["generation"]
            ref, diag = _mvp_plan(planner, ControlSettings(**job["control"]), job["args"])
            result = {"generation": generation, "reference": ref, "diagnostics": diag, "road": job["args"][0]}
        except Exception as error:
            result = {"generation": job.get("generation"), "error": str(error)}
        sys.stdout.write(json.dumps(result, allow_nan=False)+"\n")
        sys.stdout.flush()


class AsyncLatticePlanner:
    """One worker process, one in-flight job, no ROS-thread pipe writes/waits.

    I/O threads carry bounded plain dictionaries. The child imports this same
    script, never initializes ROS and never publishes actuator commands.
    """
    def __init__(self):
        self.requests, self.results = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
        self.pending_at = None
        self.ready = False
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--planner-worker"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._write, daemon=True).start()
        threading.Thread(target=self._read, daemon=True).start()

    def _write(self):
        try:
            while True:
                job = self.requests.get()
                if job is None:
                    return
                self.process.stdin.write(json.dumps(job, allow_nan=False)+"\n")
                self.process.stdin.flush()
        except (OSError, ValueError):
            return

    def _read(self):
        try:
            for line in self.process.stdout:
                result = json.loads(line)
                if result.get("ready"):
                    self.ready = True
                else:
                    self.results.put_nowait(result)
        except (OSError, ValueError, queue.Full):
            return

    def submit(self, job, now_s):
        if self.pending_at is not None or not self.ready:
            return False
        self.requests.put_nowait(job)
        self.pending_at = now_s
        return True

    def poll(self):
        try:
            result = self.results.get_nowait()
        except queue.Empty:
            return None
        self.pending_at = None
        return result

    def close(self):
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=0.05)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=0.1)
        for pipe in (self.process.stdin, self.process.stdout):
            pipe.close()


def transport_control_reference(reference, motion, now_s):
    """Reframe a delayed curve once; preserve acquisition/absolute expiry."""
    old = reference["timestamp_s"]
    remaining = old+reference["valid_for_s"]-now_s
    if remaining <= 0:
        raise ValueError("reference_expired")
    pose, problem = motion.pose_between(old, now_s)
    if problem:
        raise ValueError("Control delayed path motion: " + problem)
    if ("control_cached_path_samples" in reference
            or reference["path"].get("type") == "CARTESIAN_SAMPLES"):
        original = _control_sample_points(reference)
    else:
        lo, hi = reference["path"]["range"]
        count = min(200, max(1, math.ceil((hi-lo)/0.025)))
        original = []
        for i in range(count+1):
            point = evaluate_planning_path(reference, min(hi, lo+(hi-lo)*i/count), old)
            if point is None:
                raise ValueError("invalid_delayed_path")
            original.append((*point["position_xy_m"], point["heading_rad"], point["curvature_1pm"]))
    samples = [(*motion.transform_xy(p[:2], pose), wrap_angle(p[2]-pose["yaw_rad"]), p[3])
               for p in original]
    if samples[-1][0] <= max(0, samples[0][0]) or any(b[0] <= a[0] for a, b in zip(samples, samples[1:])):
        raise ValueError("Control delayed path support exhausted")
    # Nested planner records are read-only here; only this new envelope and
    # newly created geometry change. Avoid copying a whole Frenet trace.
    result = dict(reference)
    result.update(timestamp_s=now_s, valid_for_s=remaining, control_cached_path_samples=samples)
    # speed_profile times are relative to the ORIGINAL planning time; keep it
    # across repeated transports so consumers do not restart the profile.
    result.setdefault("speed_profile_timestamp_s", old)
    return result


def mpc_planning_reference(planning_output, held, motion, now_s):
    """Planning reference for the MPC: (reference, new_held, held_age_s or None).

    Async planning reports planning_pending on cycles without a new result.
    Only then is the last accepted reference (held = (reference, accepted_s))
    motion-transported to now_s; transport keeps its original absolute expiry,
    so holding never renews trust. Any other invalid output, a failed transport
    or expiry drops the hold and passes the original output to the MPC's own
    dropout handling. The MVP's ControlReferenceManager is not involved.
    """
    if planning_output.get("valid") is True:
        return planning_output, (planning_output, now_s), None
    if held is None or planning_output.get("reason") != "planning_pending":
        return planning_output, None, None
    try:
        transported = transport_control_reference(held[0], motion, now_s)
    except (ValueError, TypeError, ArithmeticError, KeyError):
        return planning_output, None, None
    return transported, (transported, held[1]), now_s - held[1]


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
    # 0 disables that budget (loop tracks); wheel-speed freshness is still required.
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
    # Control-only recovery from transient road failures; legacy code defaults
    # stay off. Shipped MVP YAML enables it. These are experiment budgets.
    robustness_enabled: bool = False
    reference_hold_max_s: float = 1.0
    reference_hold_max_distance_m: float = 0.25
    degraded_speed_mps: float = 0.2
    speed_recovery_accel_mps2: float = 0.15
    recovery_confirm_frames: int = 1
    gyro_bias_learning_enabled: bool = False
    gyro_bias_window_s: float = 1.0
    gyro_bias_max_rps: float = 0.12

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("control.enabled must be bool")
        if not isinstance(self.robustness_enabled, bool):
            raise ValueError("control.robustness_enabled must be bool")
        if not isinstance(self.gyro_bias_learning_enabled, bool):
            raise ValueError("control.gyro_bias_learning_enabled must be bool")
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
                     "reference_hold_max_s", "reference_hold_max_distance_m", "gyro_bias_window_s", "gyro_bias_max_rps"):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"control.{name} must be finite and positive")
        # 0 = no per-run budget (loop tracks, as in mvp2.1); wheel-speed checks still apply.
        for name in ("max_distance_m", "max_run_time_s"):
            if not finite_number(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"control.{name} must be finite and nonnegative (0 = no limit)")
        if (not finite_number(self.degraded_speed_mps) or self.degraded_speed_mps <= 0
                or not finite_number(self.speed_recovery_accel_mps2) or self.speed_recovery_accel_mps2 <= 0
                or not isinstance(self.recovery_confirm_frames, int) or self.recovery_confirm_frames < 1):
            raise ValueError("invalid control speed recovery settings")
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
        if self.maximum_m > 0 and self.distance_m+1e-9 >= self.maximum_m:   # 0 = no limit
            return f"Distance limit reached: {self.distance_m:.3f} m / {self.maximum_m:.3f} m; explicit resume required"
        return None


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
    vehicle_params_source: str = "vehicle"
    v_exec_max_mps: float = 0.0       # execution speed cap; must be > 0 for shadow: false
    overspeed_margin_mps: float = 0.1  # locks above cap + this; only while the soft margin is 0
    # Soft overspeed (2026-10-18; 0 = off, the lock above applies). Above cap + soft the QP
    # drive floor drops from the deadband to 0 (the MVP's PID can always cut drive); above
    # that + cut the drive is forced to 0. It locks only when the speed stays above the soft
    # line for cut_timeout_s, or above overspeed_hard_mps (0 = off) for hard_persist_s.
    overspeed_soft_margin_mps: float = 0.0
    overspeed_cut_margin_mps: float = 0.03
    overspeed_cut_timeout_s: float = 1.0
    overspeed_hard_mps: float = 0.0
    overspeed_hard_persist_s: float = 0.2
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
    # A restart within restart_merge_s of the last counted one is the same episode (a stall
    # restart that dips below moving_speed_mps again while pulling away); 0 = count each.
    restart_merge_s: float = 0.0
    # Speed-dependent drive cap (2026-10-19, 0 = off): at or above high_speed_mps the QP's
    # drive bound is high_speed_drive_max instead of vehicle.drive_max, so a low battery gets
    # the larger cap when crawling or starting, and a fresh one cannot overshoot with it.
    high_speed_drive_max: float = 0.0
    high_speed_mps: float = 0.15
    # Tracking without a stop for this long clears the restart count (0 = never), so a
    # long or loop run is not ended by occasional stalls spread over many minutes.
    restart_count_reset_s: float = 0.0
    # Execution only: a numerical QP failure (solver status, constraint residual, nonfinite
    # solution, action out of bounds) is a tolerable dropout instead of a lock. Bad inputs
    # and unexpected exceptions still lock.
    solver_fail_tolerable: bool = False
    # Execution only (2026-10-28, car 01): a road_centerline stop for a 1-2 frame ordering /
    # pairing failure (GEOMETRY_GLITCH_REASONS) is a tolerable dropout instead of a lock, so
    # the last solved sequence runs for up to reference_dropout_tolerance_s. Car start-ups and
    # Gym right-angle corners ended whole runs on such single frames.
    geometry_glitch_tolerable: bool = False
    # Drive-bias estimate (integral action on the model's one-step speed residual):
    # bias += drive_bias_gain * (predicted - measured speed) * dt, clipped to +/-drive_bias_max.
    # It shifts the effective deadband. 0 gain = off.
    drive_bias_gain: float = 0.0
    drive_bias_max: float = 0.05
    # Late updates: dt_max_s < dt <= dt_hard_max_s is a tolerable (plan-hold) rejection,
    # dt above dt_hard_max_s locks.
    dt_hard_max_s: float = 0.4
    # Held wheel reading (2026-10-19, 0 = off). The car's wheel-speed driver keeps its last
    # value ~3 s after the wheel slows or stops. A raw value unchanged for wheel_hold_stale_s
    # is not fresh: the soft-overspeed timer and the drive bias pause. It means a stalled car
    # (speed passed as 0, so the start drive is reapplied) when the gyro also stayed below
    # wheel_stall_yaw_rps while the applied steering was at least wheel_stall_min_steer, or
    # for wheel_hold_straight_s at any steering (moving readings changed within 1.18 s).
    wheel_hold_stale_s: float = 0.0
    wheel_hold_straight_s: float = 1.5
    wheel_stall_yaw_rps: float = 0.02
    wheel_stall_min_steer: float = 0.3
    # Centerline jump guard (2026-10-19, 0 = off). A road_centerline reference whose heading
    # error exceeds switch_max_epsi_rad, or whose errors jumped from the previous solved step
    # by more than switch_max_epsi_rad / switch_max_dey_m, is a tolerable dropout (the last
    # solved inputs keep running) for at most switch_hold_s in a row; after that it is used.
    switch_hold_s: float = 0.0
    switch_max_epsi_rad: float = 0.5
    switch_max_dey_m: float = 0.15
    # Turn drag (2026-10-19 plan, 0 = off): at normalized steering |s| holding a speed takes
    # turn_drive_per_steer*|s| more drive than straight. It shifts the effective deadband
    # with the bias, in the QP and in the bias estimate's one-step prediction.
    turn_drive_per_steer: float = 0.0
    # Start hysteresis (0 = moving_speed_mps, i.e. none): tracking returns to the start
    # phase only below rest_speed_mps (<= moving_speed_mps).
    rest_speed_mps: float = 0.0
    # Start handover (0 = off): for handover_s after the start phase hands over to tracking,
    # drive is capped at the effective deadband (deadband + bias + turn) + handover_drive_margin.
    handover_s: float = 0.0
    handover_drive_margin: float = 0.04
    # Reference-source blend (0 = off): for switch_blend_s after the reference switches
    # between the lattice and the centerline, steering moves at most
    # switch_steer_rate_per_s*dt per tick (both directions; switch_hold_s is unchanged).
    switch_blend_s: float = 0.0
    switch_steer_rate_per_s: float = 1.5
    # Longitudinal mode (2026-10-20). "joint": the QP optimises drive and steering together
    # (the settings above). "feedback": while tracking, drive is a wheel-speed PI --
    # fb_sustain_drive (0 = vehicle.drive_deadband) + turn drag of the steering in force
    # + fb_kp*error + integral (+/-fb_integral_max, frozen on held wheel readings), moving at
    # most fb_slew_per_s -- and the QP solves steering only, with that drive fixed. The start
    # phase, overspeed guards and the high-speed drive cap are unchanged; bias and handover
    # are joint-only. Gains: the .27 drive fit gives ~10 m/s^2 per unit drive and the loop
    # sees ~0.4 s of delay (0.26 s drive + held readings), so kp 0.5 (15724bc, on the Gym's
    # 3x softer drive) oscillates; 0.2/0.2 held 0.2 m/s on 54 mismatched plants (10-20 sweep).
    # measured_motion (feedback only): the prediction holds the measured speed instead of
    # integrating the drive model (ported from origin/feature/mpc-model 15724bc).
    longitudinal_mode: str = "joint"
    fb_kp: float = 0.2
    fb_ki: float = 0.2
    fb_sustain_drive: float = 0.0
    fb_integral_max: float = 0.12
    fb_slew_per_s: float = 1.5
    measured_motion: bool = False
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
    max_backward_extension_m: float = 0.6
    # Past the reference end the last segment is extended straight (curvature 0) by at most
    # this, so a path that reaches the MVP's preview point but not the whole horizon still
    # solves (2026-10-18). 0 = off: such a path is rejected as reference_too_short.
    max_forward_extension_m: float = 0.0
    # ORDERED_CENTERLINE only: length of path start fitted for the straight lead-in
    # to the car (reference_trajectory_from_planning). Not used by V1/V2 paths.
    ordered_lead_in_fit_m: float = 1.0
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
    # Reference curvature limit (2026-10-22, docs/MPC_REFERENCE_CURVATURE_LIMIT_PLAN_CN.md).
    # "off": the reference is not touched and mpc_debug is unchanged. "hooks": when the
    # largest curvature over a reference_hook_window_m arc within the first
    # reference_hook_trigger_length_m exceeds reference_hook_curvature_1pm, the reference is
    # re-tracked from its own start at most reference_curvature_max_1pm (0 = the vehicle's
    # tan(min |steering limit|)/wheelbase_m); a result closer to a known boundary than half
    # the body width + reference_curvature_clearance_m is dropped for the original.
    reference_curvature_limit_mode: str = "off"
    reference_hook_curvature_1pm: float = 4.0
    reference_hook_window_m: float = 0.15
    reference_hook_trigger_length_m: float = 0.8
    reference_curvature_max_1pm: float = 0.0
    reference_curvature_clearance_m: float = 0.03

    def __post_init__(self):
        for name in ("enabled", "shadow", "bypass_acknowledged", "solver_fail_tolerable", "measured_motion",
                     "geometry_glitch_tolerable"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"mpc.{name} must be bool")
        if self.reference_source not in ("planning", "estimation_centerline"):
            raise ValueError("mpc.reference_source must be planning or estimation_centerline")
        if self.vehicle_params_source not in ("vehicle", "course_simulation"):
            raise ValueError("mpc.vehicle_params_source must be vehicle or course_simulation")
        if self.qp_solver not in ("osqp", "dense"):
            raise ValueError("mpc.qp_solver must be osqp or dense")
        if self.longitudinal_mode not in ("joint", "feedback"):
            raise ValueError("mpc.longitudinal_mode must be joint or feedback")
        if self.reference_curvature_limit_mode not in ("off", "hooks"):
            raise ValueError("mpc.reference_curvature_limit_mode must be off or hooks (keep off quoted)")
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
                    "r_ddrive", "terminal_scale", "overspeed_margin_mps", "max_backward_extension_m", "ordered_lead_in_fit_m",
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
        if self.v_exec_max_mps < 0:
            raise ValueError("mpc.v_exec_max_mps must be nonnegative")
        if (self.overspeed_soft_margin_mps < 0 or self.overspeed_cut_margin_mps < 0
                or self.overspeed_cut_timeout_s <= 0 or self.overspeed_hard_mps < 0
                or self.overspeed_hard_persist_s < 0):
            raise ValueError("mpc overspeed settings out of range")
        if self.overspeed_hard_mps > 0 and (
                self.overspeed_soft_margin_mps <= 0
                or self.overspeed_hard_mps <= self.v_exec_max_mps + self.overspeed_soft_margin_mps
                + self.overspeed_cut_margin_mps):
            raise ValueError("mpc.overspeed_hard_mps needs the soft margin and must exceed cap + soft + cut")
        if not 0.0 <= self.max_forward_extension_m <= 1.0 or self.restart_count_reset_s < 0:
            raise ValueError("mpc.max_forward_extension_m must be in [0, 1] m, restart_count_reset_s >= 0")
        if (not 0.0 <= self.wheel_hold_stale_s <= 2.0
                or not self.wheel_hold_stale_s <= self.wheel_hold_straight_s <= 3.0
                or not 0.0 < self.wheel_stall_yaw_rps <= 0.2 or not 0.0 < self.wheel_stall_min_steer <= 1.0):
            raise ValueError("mpc wheel hold settings out of range")
        if (not 0.0 <= self.switch_hold_s <= 0.5 or not 0.0 < self.switch_max_epsi_rad <= 1.5
                or not 0.0 < self.switch_max_dey_m <= 0.5):
            raise ValueError("mpc switch guard settings out of range")
        if (not 0.0 <= self.restart_merge_s <= 5.0 or not 0.0 <= self.high_speed_drive_max <= 1.0
                or not 0.0 < self.high_speed_mps <= 0.5):
            raise ValueError("mpc restart merge / high-speed drive settings out of range")
        if (not 0.0 <= self.turn_drive_per_steer <= 0.2
                or not 0.0 <= self.rest_speed_mps <= self.moving_speed_mps
                or not 0.0 <= self.handover_s <= 2.0 or not 0.0 <= self.handover_drive_margin <= 0.2
                or not 0.0 <= self.switch_blend_s <= 2.0 or not 0.0 < self.switch_steer_rate_per_s <= 20.0):
            raise ValueError("mpc turn drag / start hysteresis / handover / switch blend settings out of range")
        if (not 0.0 <= self.fb_kp <= 3.0 or not 0.0 <= self.fb_ki <= 3.0
                or not 0.0 <= self.fb_sustain_drive <= 1.0 or not 0.0 <= self.fb_integral_max <= 0.2
                or not 0.0 < self.fb_slew_per_s <= 10.0):
            raise ValueError("mpc feedback (fb_*) settings out of range")
        if self.measured_motion and self.longitudinal_mode != "feedback":
            # The joint QP chooses drive through the drive model; holding speed would blind it.
            raise ValueError("mpc.measured_motion needs longitudinal_mode feedback")
        if self.switch_hold_s > 0 and self.reference_dropout_tolerance_s <= self.switch_hold_s:
            # The guard rides on the tolerable dropout; without it a guarded tick would lock.
            raise ValueError("mpc.switch_hold_s needs reference_dropout_tolerance_s above it")
        if (self.reference_hook_curvature_1pm <= 0 or not 0.05 < self.reference_hook_window_m <= 0.5
                or not 0.0 < self.reference_hook_trigger_length_m <= 2.0
                or self.reference_curvature_max_1pm < 0 or self.reference_curvature_clearance_m < 0):
            raise ValueError("mpc reference curvature limit settings out of range")


def mpc_vehicle_params(cfg, vehicle):
    """The vehicle record the MPC predicts with: course simulation, or the vehicle.* record."""
    if cfg.vehicle_params_source == "course_simulation":
        return course_simulation_vehicle_params()
    return vehicle if vehicle is not None else VehicleParamsSettings()


def _ordered_lead_in(raw, fit_m):
    """Smoothed start (x, y, heading) of a resampled ORDERED_CENTERLINE, or None.

    raw: [(s, x, y, yaw)] from s=0 (the end nearest the car). A quadratic in the
    principal-axis frame of the first fit_m of path, evaluated at s=0: unlike
    the first 5 cm tangent it does not multiply one sample's noise by the gap to
    the car, and unlike a line it keeps straights and bends of any radius exact
    (a line gives a gentle bend its chord heading: 0.1 rad on R 5 m).
    Replay of car 05's 2026-10-15 bags: |e_y - cone-pair centre| p95 0.105 ->
    0.075, 0.175 -> 0.141, 0.039 -> 0.016 m. A smooth tilt the road estimate
    itself carries (one misplaced near cone, 18:18) is only partly removed
    (0.12 -> 0.08 m there): geometry alone cannot tell it from a real bend.
    """
    window = [r for r in raw if r[0] <= min(fit_m, raw[-1][0])+1e-9]
    if len(window) < 3 or window[-1][0]-window[0][0] < 0.3:
        return None
    mx, my = sum(r[1] for r in window)/len(window), sum(r[2] for r in window)/len(window)
    sxx = sum((r[1]-mx)**2 for r in window)
    syy = sum((r[2]-my)**2 for r in window)
    sxy = sum((r[1]-mx)*(r[2]-my) for r in window)
    axis = 0.5*math.atan2(2*sxy, sxx-syy)
    ux, uy = math.cos(axis), math.sin(axis)
    if ux*(window[-1][1]-window[0][1])+uy*(window[-1][2]-window[0][2]) < 0:
        axis, ux, uy = wrap_angle(axis+math.pi), -ux, -uy
    along = [(r[1]-mx)*ux+(r[2]-my)*uy for r in window]
    across = [-(r[1]-mx)*uy+(r[2]-my)*ux for r in window]
    moments = [sum(t**k for t in along) for k in range(5)]
    c = _estimation_solve([[moments[i+j] for j in range(3)] for i in range(3)],
                          [sum(e*t**i for t, e in zip(along, across)) for i in range(3)])
    if c is None:
        return None
    t0 = along[0]
    e0, slope = c[0]+c[1]*t0+c[2]*t0*t0, c[1]+2*c[2]*t0
    return mx+t0*ux-e0*uy, my+t0*uy+e0*ux, wrap_angle(axis+math.atan(slope))


def reference_trajectory_from_planning(ref, spacing_m=0.05, lead_in_fit_m=1.0, max_lead_in_m=0.6):
    """Planning V1/V2 reference -> ReferenceTrajectory v0.1 point list (base_link).

    The original timestamp, lifetime and source ages are passed through unchanged;
    expiry is checked by the consumer. Invalid input yields valid=False/stop_required.
    V2 CARTESIAN_SAMPLES are copied without refitting or resampling. V2
    CARTESIAN_Y_OF_X polynomials are evaluated at their own origin and scale.
    A V2 speed_profile is retained for time-based MPC shadow evaluation.
    ORDERED_CENTERLINE paths that start ahead of the car get a straight lead-in
    (lead_in_fit_m, max_lead_in_m; see that branch).
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
        # Lead-in (2026-10-15, car 05 18:18 replay): the path often starts
        # 0.4-0.75 m ahead (near field unobserved) and mpc_path_projector would
        # extend its FIRST 5 cm segment back to the body; see _ordered_lead_in.
        # Only when the gap fits in max_lead_in_m (mpc.max_backward_extension_m);
        # otherwise the projector's extension applies as before. Paths starting
        # at or behind the body are unchanged.
        start = _ordered_lead_in(raw, lead_in_fit_m)
        if start is not None:
            x0, y0, theta = start
            hx, hy = math.cos(theta), math.sin(theta)
            gap = x0*hx+y0*hy                       # path start ahead of the car's projection
            if spacing_m < gap <= max_lead_in_m:
                lead = [(0.0, x0-k*spacing_m*hx, y0-k*spacing_m*hy, theta)
                        for k in range(math.ceil(gap/spacing_m)+1, 0, -1)]
                s_all = _path_lengths([(r[1], r[2]) for r in lead+raw])
                raw = [(s_all[i], r[1], r[2], r[3]) for i, r in enumerate(lead+raw)]
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
        out.update(valid=True, stop_required=False, reason=None, points=points)
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


def reference_trajectory_from_road(road, state, cfg):
    """BYPASSES Planning: estimator centerline -> quadratic fit -> v0.1 point list (base_link).

    y = c0 + c1 u + c2 u^2 with u = x - mean(x), so bends that Planning V1 rejects are allowed.
    Lost relative to Planning: body-clearance, stopping-distance and speed-capping checks;
    the target speed is the fixed cfg.bypass_target_speed_mps. The road must already be aligned
    to the state timestamp. The fit is extrapolated back to the car by at most
    cfg.max_backward_extension_m (the fitted curve, not a straight tangent).
    Curvature is positive for a path bending left.
    """
    out = {"schema": "reference_trajectory_v0.1", "frame_id": None, "timestamp_s": None,
           "valid_for_s": 0.0, "valid": False, "stop_required": True, "status": "BYPASS",
           "reason": "missing_road", "path_id": None, "simulation_only": False,
           "source_ages_s": {}, "points": [], "reference_source": "estimation_centerline",
           "planning_bypassed": True}
    if not isinstance(road, dict) or not isinstance(state, dict):
        return out

    def fail(reason):
        out["reason"] = reason
        return out

    out.update(frame_id=road.get("frame_id"), timestamp_s=state.get("timestamp_s"),
               path_id=road.get("measurement_timestamp_s"))
    age = road.get("source_age_s")
    out["source_ages_s"] = {"road": age}
    if (not road.get("valid") or not road.get("time_aligned") or road.get("frame_id") != "base_link"
            or not state.get("speed_valid") or not state.get("yaw_rate_valid")):
        return fail("road_or_state_invalid")
    if (not finite_number(state.get("timestamp_s")) or not finite_number(road.get("timestamp_s"))
            or abs(road["timestamp_s"]-state["timestamp_s"]) > 1e-6):
        return fail("road_not_aligned_to_state")
    if not finite_number(age) or age < 0 or age >= cfg.bypass_max_source_age_s:
        return fail("road_stale")
    points = road.get("centerline_xy")
    if (not isinstance(points, (list, tuple)) or not 5 <= len(points) <= 201
            or any(not isinstance(q, (list, tuple)) or len(q) != 2 or not all(finite_number(v) for v in q)
                   for q in points)
            or any(b[0] <= a[0] for a, b in zip(points, points[1:]))):
        return fail("bad_centerline")
    lo = points[0][0]
    points = [q for q in points if q[0] <= cfg.bypass_fit_max_x_m]
    if lo > cfg.max_backward_extension_m:
        return fail("near_coverage_missing")
    if len(points) < 5 or points[-1][0] < cfg.bypass_min_forward_m:
        return fail("forward_coverage_short")
    hi = points[-1][0]
    mean_x = sum(q[0] for q in points)/len(points)
    sums = [sum((q[0]-mean_x)**k for q in points) for k in range(5)]
    moments = [sum(q[1]*(q[0]-mean_x)**k for q in points) for k in range(3)]
    coeffs = _estimation_solve([[sums[0], sums[1], sums[2]], [sums[1], sums[2], sums[3]],
                                [sums[2], sums[3], sums[4]]], moments)
    if coeffs is None:
        return fail("degenerate_fit")
    c0, c1, c2 = coeffs
    error = max(abs(c0 + c1*(q[0]-mean_x) + c2*(q[0]-mean_x)**2 - q[1]) for q in points)
    if error > cfg.bypass_max_fit_error_m:
        return fail("fit_error_too_large")

    def sample(x):
        u = x-mean_x
        slope = c1 + 2*c2*u
        return c0 + c1*u + c2*u*u, slope, 2*c2/(1+slope*slope)**1.5

    x, last = min(lo, -0.1), None
    samples = []
    while True:
        y, slope, kappa = sample(x)
        s = 0.0 if last is None else samples[-1]["s_m"] + math.hypot(x-last[0], y-last[1])
        samples.append({"s_m": s, "x_m": x, "y_m": y, "yaw_rad": math.atan(slope),
                        "curvature_1pm": kappa, "target_speed_mps": cfg.bypass_target_speed_mps})
        last = (x, y)
        if x >= hi:
            break
        x += cfg.reference_spacing_m/math.sqrt(1+slope*slope)
        if hi - x < 0.25*cfg.reference_spacing_m:
            x = hi            # no near-duplicate last point
    if max(abs(q["curvature_1pm"]) for q in samples) > cfg.bypass_max_curvature_1pm:
        return fail("curvature_too_large")
    out.update(valid=True, stop_required=False, reason=None, points=samples,
               valid_for_s=min(cfg.bypass_valid_for_s, cfg.bypass_max_source_age_s-age),
               fit_max_error_m=error, extrapolated_back_m=max(0.0, lo-samples[0]["x_m"]))
    return out


# 2026-10-22: reference curvature limit (option B, hooks only). On car 01's sharp right
# bends the lattice (and at times the estimator's centerline) handed the MPC references
# turning 110-300 deg within 0.3-0.6 m; the MPC held full lock after them and overshot.
# Only such hooks are smoothed; see docs/MPC_REFERENCE_CURVATURE_LIMIT_PLAN_CN.md.
REFERENCE_SMOOTH_STEP_M = 0.01
REFERENCE_SMOOTH_LOOKAHEAD_M = 0.25
REFERENCE_SHIFT_LENGTH_M = 0.4


def reference_curvature_limit_1pm(cfg, vehicle):
    """mpc.reference_curvature_max_1pm, or the vehicle's tightest steady curvature (0 = unknown)."""
    if cfg.reference_curvature_max_1pm > 0:
        return cfg.reference_curvature_max_1pm
    steer = min(abs(vehicle.steering_min_rad), abs(vehicle.steering_max_rad))
    if vehicle.wheelbase_m <= 0 or not 0 < steer < math.pi/2:
        return 0.0
    return math.tan(steer)/vehicle.wheelbase_m


def _reference_segments(xy):
    """(arc length at each kept point, heading of each segment); repeated points are skipped."""
    kept, headings = [xy[0]], []
    for b in xy[1:]:
        a = kept[-1]
        if math.hypot(b[0]-a[0], b[1]-a[1]) > 1e-6:
            headings.append(math.atan2(b[1]-a[1], b[0]-a[0]))
            kept.append(b)
    return _path_lengths(kept), headings


def reference_peak_curvature_1pm(xy, window_m, length_m):
    """Largest |heading change| / arc over a window_m arc starting within the first length_m.

    Geometric (segment headings over the window), not neighbouring yaw_rad: at 5 cm
    spacing that is too noisy and fires on ordinary bends.
    """
    if len(xy) < 2:
        return 0.0
    s, headings = _reference_segments(xy)
    peak, j = 0.0, 0
    for i in range(len(headings)):
        if s[i] > length_m:
            break
        j = max(j, i)
        while j < len(headings)-1 and s[j+1]-s[i] < window_m:
            j += 1
        arc = s[j+1]-s[i]
        if arc < 0.9*window_m:
            break        # the rest of the path is shorter than one window
        turn = wrap_angle(headings[j]-headings[i])
        peak = max(peak, abs(turn)/arc)
    return peak


def _reference_curvature_limited(points, limit, lookahead=REFERENCE_SMOOTH_LOOKAHEAD_M,
                                 ds=REFERENCE_SMOOTH_STEP_M):
    """Pure-pursuit re-tracking of a point list at |curvature| <= limit.

    Starts at the reference's own first point along its first segment (not the car's
    heading, so an existing lateral error is not counted as a shape change). Returns
    (resampled v0.1 points, the 1 cm path as (x, y, s)) or (None, None) below 3 points.
    """
    xy = [(p["x_m"], p["y_m"]) for p in points]
    dense, arc = [xy[0]], [0.0]
    for a, b in zip(xy, xy[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length <= 1e-6:
            continue
        n = max(1, math.ceil(length/ds))
        for i in range(1, n+1):
            dense.append((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n))
            arc.append(arc[-1]+length/n)
    if len(dense) < 3:
        return None, None
    x, y = dense[0]
    heading = math.atan2(dense[1][1]-y, dense[1][0]-x)
    track, j, goal = [(x, y, heading, 0.0)], 0, 0
    search = int(math.ceil(0.6/ds))
    for _ in range(int(arc[-1]/ds)):
        # Nearest dense point, searched forward only and within a bounded window.
        best, nearest = j, math.inf
        for i in range(j, min(len(dense), j+search)):
            d2 = (dense[i][0]-x)**2+(dense[i][1]-y)**2
            if d2 < nearest:
                best, nearest = i, d2
        j = best
        goal = max(goal, j)
        while goal < len(dense)-1 and arc[goal]-arc[j] < lookahead:
            goal += 1
        dx, dy = dense[goal][0]-x, dense[goal][1]-y
        lx = math.cos(heading)*dx+math.sin(heading)*dy
        ly = -math.sin(heading)*dx+math.cos(heading)*dy
        d2 = lx*lx+ly*ly
        kappa = 0.0 if d2 < 1e-9 else max(-limit, min(limit, 2*ly/d2))
        heading += kappa*ds
        x, y = x+math.cos(heading)*ds, y+math.sin(heading)*ds
        track.append((x, y, heading, kappa))
        if j >= len(dense)-2:
            break
    # Resample at the reference's own mean spacing; speeds from the nearest original s_m.
    s0 = points[0]["s_m"]
    spacing = (points[-1]["s_m"]-s0)/max(1, len(points)-1) or 0.05
    step = max(1, int(round(spacing/ds)))
    out, k = [], 0
    for i in range(0, len(track), step):
        x, y, heading, kappa = track[i]
        s = s0+i*ds
        while k < len(points)-1 and abs(points[k+1]["s_m"]-s) <= abs(points[k]["s_m"]-s):
            k += 1
        out.append({**points[k], "s_m": s, "x_m": x, "y_m": y, "yaw_rad": wrap_angle(heading),
                    "curvature_1pm": kappa})
    if len(out) < 3:
        return None, None
    return out, [(x, y, i*ds) for i, (x, y, _, _) in enumerate(track)]


def _reference_polyline_distance(point, polyline):
    best = math.inf
    for a, b in zip(polyline, polyline[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        u = 0.0 if length2 <= 1e-12 else min(1.0, max(0.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2))
        best = min(best, math.hypot(a[0]+u*dx-point[0], a[1]+u*dy-point[1]))
    return best


def limit_reference_curvature(traj, road, cfg, vehicle, half_width_m, check_length_m):
    """mpc.reference_curvature_limit_mode on a v0.1 reference: (traj, debug or None).

    "off" returns the same traj object and no debug. "hooks" returns a copy with new
    points only when the hook trigger fires, the limited path has >= 3 points and its
    first check_length_m keeps half_width_m + clearance from road's known boundaries
    (one side is checked alone; none, nothing to check); otherwise the same traj.
    Pure: no ROS, no node state.
    """
    if cfg.reference_curvature_limit_mode == "off":
        return traj, None
    limit = reference_curvature_limit_1pm(cfg, vehicle)
    debug = {"triggered": False, "peak_1pm": 0.0, "limit_1pm": limit, "applied": False,
             "reason": None, "max_shift_m": 0.0}
    points = traj.get("points") if isinstance(traj, dict) else None
    if (not isinstance(points, list) or len(points) < 3 or traj.get("valid") is not True
            or traj.get("stop_required") is True):
        return traj, debug
    debug["peak_1pm"] = reference_peak_curvature_1pm(
        [(p["x_m"], p["y_m"]) for p in points], cfg.reference_hook_window_m, cfg.reference_hook_trigger_length_m)
    if debug["peak_1pm"] <= cfg.reference_hook_curvature_1pm:
        return traj, debug
    debug["triggered"] = True
    if limit <= 0:
        debug["reason"] = "no_curvature_limit"     # vehicle record without wheelbase/steering limits
        return traj, debug
    limited, path = _reference_curvature_limited(points, limit)
    if limited is None:
        debug["reason"] = "too_short"
        return traj, debug
    original = [(p["x_m"], p["y_m"]) for p in points]
    shifted = [(p["x_m"], p["y_m"]) for p in limited]
    debug["max_shift_m"] = max(
        [_reference_polyline_distance(q, original) for q, p in zip(shifted, limited)
         if p["s_m"]-limited[0]["s_m"] <= REFERENCE_SHIFT_LENGTH_M]
        + [_reference_polyline_distance(q, shifted) for q, p in zip(original, points)
           if p["s_m"]-points[0]["s_m"] <= REFERENCE_SHIFT_LENGTH_M])
    road = road if isinstance(road, dict) else {}
    if not any(road.get(k) for k in ("centerline_xy", "left_boundary_xy", "right_boundary_xy")):
        road = road.get("forward_geometry") or {}      # as _select_planner_reference reads it
    required = half_width_m+cfg.reference_curvature_clearance_m
    checked = [(x, y) for x, y, s in path if s <= check_length_m]
    lo_x, hi_x = min(x for x, _ in checked)-required, max(x for x, _ in checked)+required
    lo_y, hi_y = min(y for _, y in checked)-required, max(y for _, y in checked)+required
    for side in ("left_boundary_xy", "right_boundary_xy"):
        boundary = road.get(side)
        if not isinstance(boundary, (list, tuple)) or not boundary:
            continue
        boundary = [(float(q[0]), float(q[1])) for q in boundary]
        # Only boundary segments within `required` of the checked path's box can fail it.
        near = [(a, b) for a, b in zip(boundary, boundary[1:] or boundary)
                if max(a[0], b[0]) >= lo_x and min(a[0], b[0]) <= hi_x
                and max(a[1], b[1]) >= lo_y and min(a[1], b[1]) <= hi_y]
        if any(_reference_polyline_distance(q, segment) < required for segment in near for q in checked):
            debug["reason"] = "rejected_clearance"
            return traj, debug
    debug["applied"] = True
    return {**traj, "points": limited}, debug


def mpc_reference_errors(traj, n, dt_pred, speed, max_extension, max_forward=0.0):
    """Vehicle (origin, heading 0 in base_link) against a v0.1 point list.

    e_y > 0: vehicle left of the path. e_psi = vehicle heading - path heading.
    Behind the first point the first segment is extended by at most max_extension
    (only meaningful for a straight reference; do not inherit this for curves).
    Past the last point the last segment is extended straight by at most max_forward
    (curvature 0 there). Returns (info, None) or (None, reason). No end-point
    repetition: if the N-step travel is not covered the reference is rejected.
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
    last = len(points)-2
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]
        length = cumulative[i+1]-cumulative[i]
        t = (-a["x_m"]*dx - a["y_m"]*dy)/(length*length)
        low = -max_extension/length if i == 0 else 0.0
        high = 1.0 + max_forward/length if i == last else 1.0
        clamped = min(high, max(low, t))
        px, py = a["x_m"]+clamped*dx, a["y_m"]+clamped*dy
        distance = math.hypot(px, py)
        if best is None or distance < best[0]:
            best = (distance, i, t, clamped, px, py, math.atan2(dy, dx))
    _, i, t, clamped, px, py, theta = best
    if i == 0 and t < -max_extension/(cumulative[1]-cumulative[0]) - 1e-9:
        return None, "origin_before_path_start"
    if i == last and t > 1.0 + max_forward/(cumulative[-1]-cumulative[-2]) + 1e-9:
        return None, "origin_past_path_end"
    s_proj = cumulative[i] + clamped*(cumulative[i+1]-cumulative[i])
    horizon = speed*dt_pred*n
    if s_proj + horizon > cumulative[-1] + max_forward + 1e-9:
        return None, "reference_too_short"
    kappas = []
    for k in range(n):
        s_k = s_proj + speed*dt_pred*k
        if s_k > cumulative[-1] + 1e-9:
            kappas.append(0.0)       # straight forward extension
            continue
        j = max(0, min(len(points)-1, sum(1 for c in cumulative if c <= s_k)-1))
        kappas.append(points[j]["curvature_1pm"])
    return {"e_y_m": px*math.sin(theta) - py*math.cos(theta), "e_psi_rad": wrap_angle(-theta),
            "kappas": kappas, "s_proj_m": s_proj, "path_remaining_m": cumulative[-1]-s_proj,
            "extended_start": i == 0 and t < 0.0, "v_ref_mps": points[0]["target_speed_mps"]}, None


def mpc_path_projector(points, max_extension, max_forward=0.0):
    """Nearest-point projection onto a v0.1 point list (base_link).

    Returns project(x, y) -> (px, py, heading, curvature, target_speed, past_end).
    Behind the first point the first segment is extended by at most max_extension;
    past the last point it is extended straight by at most max_forward (curvature 0
    there), beyond which the projection clamps and past_end is True.

    Ported 2026-10-15 (ORDERED_CENTERLINE support): segments pointing more than
    ~105 deg away from the body's own +x axis (base_link, fixed for the whole
    prediction horizon) are tried first, so a disconnected-but-Euclidean-close
    branch near a sharp bend or U-turn never wins over the segment actually
    ahead of the car (mirrors _path_project's forward-branch rule). This is
    computed once per solve(), not per query. Falls back to every segment if
    that would exclude all of them, so a valid projection is never refused.
    """
    segments, last = [], len(points)-2
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]
        length2 = dx*dx + dy*dy
        segments.append((a, b, dx, dy, length2, -max_extension/math.sqrt(length2) if i == 0 else 0.0,
                         1.0 + max_forward/math.sqrt(length2) if i == last else 1.0))
    forward_indices = [i for i, (_, _, dx, _, length2, _, _) in enumerate(segments)
                       if dx/math.sqrt(length2) >= -0.25] or list(range(len(segments)))

    def project(x, y):
        best = None
        for i in forward_indices:
            a, b, dx, dy, length2, low, high = segments[i]
            t = ((x-a["x_m"])*dx + (y-a["y_m"])*dy)/length2
            clamped = min(high, max(low, t))
            px, py = a["x_m"]+clamped*dx, a["y_m"]+clamped*dy
            distance = (px-x)**2 + (py-y)**2
            if best is None or distance < best[0]:
                best = (distance, i, t, clamped, px, py, math.atan2(dy, dx), a if clamped < 0.5 else b)
        _, i, t, clamped, px, py, heading, near = best
        extended = i == last and clamped > 1.0      # on the straight forward extension
        return (px, py, heading, 0.0 if extended else near["curvature_1pm"], near["target_speed_mps"],
                i == last and t > segments[i][6]+1e-9)
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

    def input_bounds(self, p=None, moving=False, allow_braking=False, drive_floor_zero=False, drive_cap=None):
        p = self.vehicle if p is None else p
        floor = moving and not allow_braking and not drive_floor_zero
        low = max(0.0, p.drive_min, p.drive_deadband if floor else 0.0)
        high = p.drive_max if drive_cap is None else min(p.drive_max, drive_cap)
        drive = (low, max(low, high))
        ends = sorted(((p.steering_min_rad-p.steering_offset_rad)/p.steering_gain_rad,
                       (p.steering_max_rad-p.steering_offset_rad)/p.steering_gain_rad))
        return drive, (max(-1.0, ends[0]), min(1.0, ends[1]))

    def steady_drive(self, speed, p=None):
        p = self.vehicle if p is None else p
        return p.drive_deadband + p.drag_kg_per_m*speed*speed/p.motor_gain_n

    def motion_step(self, z, u, h, p, rate_limited=True, hold_speed=False):
        """course_model_step; hold_speed (mpc.measured_motion) replaces the drive by the drive
        that holds the current speed, so the prediction keeps the measured speed and only
        steering shapes it. That drive is prediction math only, never sent to the car."""
        if hold_speed:
            u = (self.steady_drive(z[3], p), u[1])
        return course_model_step(z, u, h, p, rate_limited=rate_limited)

    def _linearise(self, z, u, h, p=None, hold_speed=False):
        """Forward-difference Jacobians of the unclipped step at (z, u)."""
        p, np = self.vehicle if p is None else p, self._np
        base = self.motion_step(z, u, h, p, False, hold_speed)
        A, B = np.zeros((5, 5)), np.zeros((5, 2))
        for j in range(5):
            eps = 1e-6*max(1.0, abs(z[j]))
            shifted = list(z)
            shifted[j] += eps
            A[:, j] = (np.array(self.motion_step(shifted, u, h, p, False, hold_speed))-base)/eps
        for j in range(2):
            shifted = list(u)
            shifted[j] += 1e-6
            B[:, j] = (np.array(self.motion_step(z, shifted, h, p, False, hold_speed))-base)/1e-6
        return base, A, B

    def _solve_qp(self, P, q, constraints, lower, upper):
        """min 0.5 U'PU + q'U, lower <= constraints U <= upper -> (status, U, iterations)."""
        if self.cfg.qp_solver == "dense":
            return self._solve_dense(P, q, constraints, lower, upper)
        sp, cfg = self._sparse, self.cfg
        Px = P[self._triu]
        if self._model is None:
            size = P.shape[0]
            self._model = self._osqp.OSQP()
            self._model.setup(sp.csc_matrix((Px, self._triu), shape=(size, size)), q, sp.csc_matrix(constraints),
                              lower, upper, verbose=False, eps_abs=1e-6, eps_rel=1e-6,
                              max_iter=cfg.solver_max_iter, time_limit=cfg.solver_time_limit_s, polishing=False)
        else:
            self._model.update(Px=Px, q=q, l=lower, u=upper)
        result = self._model.solve()
        return str(result.info.status), result.x, int(result.info.iter)

    def _solve_dense(self, P, q, constraints, lower, upper):
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
                if time.perf_counter()-t0 > cfg.solver_time_limit_s:
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
              speed_targets=None, allow_braking=False, drive_floor_zero=False, drive_cap=None,
              fixed_drive=None, hold_speed=False):
        """Returns (solution, None) or (None, reference_reason); raises MPCStop on solver failure.

        drive_floor_zero (soft overspeed): while moving, drive may go below the deadband,
        into the model's drag-brake region, without planning a stop. drive_cap lowers the
        drive bound below vehicle.drive_max (never below the deadband floor).
        fixed_drive (longitudinal_mode feedback): every drive input is pinned to this value
        (clipped into the drive bounds), so the QP chooses steering only. hold_speed: see
        motion_step."""
        np, cfg, n, h = self._np, self.cfg, self.cfg.horizon_n, self.cfg.dt_pred_s
        p = self.effective(drive_bias)
        t0 = time.perf_counter()
        project = mpc_path_projector(points, cfg.max_backward_extension_m, cfg.max_forward_extension_m)
        if (speed_targets is not None and (not isinstance(speed_targets, (list, tuple))
                or len(speed_targets) != n
                or any(not finite_number(v) or v < 0 for v in speed_targets))):
            raise MPCStop("invalid_speed_targets")
        if allow_braking and (speed_targets is None or p.brake_gain_n <= 0):
            raise MPCStop("braking_model_unavailable")
        (d_lo, d_hi), (s_lo, s_hi) = self.input_bounds(p, moving, allow_braking, drive_floor_zero, drive_cap)
        if fixed_drive is not None:
            if not finite_number(fixed_drive):
                raise MPCStop("solver_nonfinite_input")
            d_lo = d_hi = min(d_hi, max(d_lo, float(fixed_drive)))
        g, o = p.steering_gain_rad, p.steering_offset_rad
        z0 = [float(v) for v in z0]
        if not all(math.isfinite(v) for v in z0 + [speed_cap if speed_cap != math.inf else 0.0, last_drive]):
            raise MPCStop("solver_nonfinite_input")
        if nominal is None:
            hold = min(s_hi, max(s_lo, (z0[4]-o)/g))
            first_speed = speed_targets[0] if speed_targets is not None else points[0]["target_speed_mps"]
            nominal_drive = 0.0 if allow_braking and first_speed <= 1e-3 else self.steady_drive(min(speed_cap, first_speed), p)
            nominal = [(nominal_drive, hold)]*n
        U_bar = np.array([[min(d_hi, max(d_lo, d)), min(s_hi, max(s_lo, s))] for d, s in nominal]).ravel()
        iterations = 0
        for _ in range(cfg.sqp_iterations):
            states, A_list, B_list = [z0], [], []
            for k in range(n):
                nxt, A, B = self._linearise(states[-1], U_bar[2*k:2*k+2], h, p, hold_speed)
                states.append(list(nxt))
                A_list.append(A)
                B_list.append(B)
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
            status, x, solver_iterations = self._solve_qp(P, q, constraints, lower, upper)
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
        predicted, pred_xy, z = [], [], z0
        end, before = points[-1], points[-2]
        end_dir = math.atan2(end["y_m"]-before["y_m"], end["x_m"]-before["x_m"])
        forward_extension = 0.0
        for k in range(n):
            z = self.motion_step(z, U_bar[2*k:2*k+2], h, p, hold_speed=hold_speed)
            px, py, theta, _, _, _ = project(z[0], z[1])
            predicted.append((-(z[0]-px)*math.sin(theta) + (z[1]-py)*math.cos(theta), z[3], z[4]))
            pred_xy.append((float(z[0]), float(z[1])))
            if cfg.max_forward_extension_m > 0:
                forward_extension = max(forward_extension, (px-end["x_m"])*math.cos(end_dir)
                                        + (py-end["y_m"])*math.sin(end_dir))
        inputs = [(float(U_bar[2*k]), float(U_bar[2*k+1])) for k in range(n)]
        return {"inputs": inputs, "drive0": inputs[0][0], "steer0": inputs[0][1],
                "delta0": g*inputs[0][1] + o, "violation": violation, "iterations": iterations,
                "pred_e_y": [r[0] for r in predicted], "pred_v": [r[1] for r in predicted],
                "pred_delta": [r[2] for r in predicted], "pred_xy": pred_xy,
                "forward_extension_m": float(forward_extension),
                "solve_s": float(time.perf_counter()-t0)}, None


class WheelHoldMonitor:
    """Is the raw wheel-speed reading fresh, and is a held reading a stalled car?

    The car's wheel-speed driver holds its last value for ~3 s after the wheel slows or
    stops (10-19 bags: 22 holds of 3.0 s), while readings of a moving car changed within
    1.18 s (p95 0.48 s). The gyro tells a stall apart: a car steering at least
    wheel_stall_min_steer must turn, so |yaw rate| below wheel_stall_yaw_rps over the last
    wheel_hold_stale_s (the car may roll on for a while after its last reading) is a stall.
    A stall stays declared until the reading changes (10-19 evening: dropping it when the
    restarted car's steering fell below the threshold flickered and counted extra restarts).
    Off (always fresh, never stalled) while cfg.wheel_hold_stale_s is 0. Pure logic on the
    ROS clock; the caller feeds every tick.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.value = None          # raw reading being timed
        self.since_s = None        # ROS time it first appeared
        self.yaw = []              # (ROS time, |yaw rate|) within the last wheel_hold_stale_s
        self.latched = False       # a stall was declared on the current held value

    def update(self, raw_speed, yaw_rate, applied_steer, now_s):
        cfg = self.cfg
        out = {"held_s": None, "fresh": True, "stalled": False}
        if cfg.wheel_hold_stale_s <= 0:
            return out
        if not finite_number(now_s) or (self.yaw and now_s < self.yaw[-1][0]):
            self.reset()
            return out
        # Unknown yaw counts as turning, so it can never declare a stall.
        self.yaw.append((now_s, abs(yaw_rate) if finite_number(yaw_rate) else math.inf))
        while self.yaw and self.yaw[0][0] < now_s - cfg.wheel_hold_stale_s:
            self.yaw.pop(0)
        if (not finite_number(raw_speed) or raw_speed <= cfg.moving_speed_mps
                or raw_speed != self.value or self.since_s is None):
            # A new value, rest, or no usable reading: timing restarts with this sample.
            self.value = raw_speed if finite_number(raw_speed) and raw_speed > cfg.moving_speed_mps else None
            self.since_s = now_s if self.value is not None else None
            self.latched = False
            out["held_s"] = 0.0 if self.value is not None else None
            return out
        held = now_s - self.since_s
        still = max(v for _, v in self.yaw) < cfg.wheel_stall_yaw_rps
        steering = finite_number(applied_steer) and abs(applied_steer) >= cfg.wheel_stall_min_steer
        self.latched = self.latched or still and (held >= cfg.wheel_hold_stale_s and steering
                                                  or held >= cfg.wheel_hold_straight_s)
        out.update(held_s=held, fresh=held < cfg.wheel_hold_stale_s, stalled=self.latched)
        return out


class MPCController:
    """One step per policy update; every branch fills last_debug before returning/raising.

    Shadow: nothing is ever applied (applied = 0, 0) and rejections do not raise, so the
    valid-reference statistics keep accumulating. Non-shadow: any rejection raises MPCStop,
    i.e. a locked stop that needs an explicit state-3 request; there is no automatic recovery.
    The exception is a TOLERABLE rejection (reference unavailable, late update, over budget)
    shorter than reference_dropout_tolerance_s: the last solved input sequence keeps running
    (drive never above the last applied value); without a plan the car coasts, steering held.
    During an active breakaway, a short dropout holds the last applied start drive instead
    of taking a lower drive from the QP's tracking sequence. Stop guards still take priority.
    There is no wheel-angle sensor: the initial steering angle and the delayed part of the
    motion are replayed through the model from the commands actually applied.

    Start from rest (vehicle.drive_breakaway > 0): phase "starting" applies the breakaway
    drive with the MPC's steering until the wheels move, then phase "tracking" lets the MPC
    choose the drive above the effective deadband (vehicle.drive_deadband + bias).
    """
    # mpc.geometry_glitch_tolerable: road_centerline stops that are 1-2 frame estimator
    # geometry failures (no hold exists for them; see MPCSettings).
    GEOMETRY_GLITCH_REASONS = frozenset({"invalid_road_invalid_ordered_geometry",
                                         "invalid_road_invalid_boundary_pairing"})

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
        self.last_restart_s = None  # clock_s of the last counted restart
        self.tracking_since = None  # clock_s when the current stretch of tracking began
        self.handover_since = None  # clock_s when the start phase last handed over to tracking
        self.overspeed_state = "ok"
        self.overspeed_since = None  # clock_s of the first step above the soft line in a row
        self.hard_since = None       # clock_s of the first step above overspeed_hard_mps in a row
        self.bias = 0.0
        self.last_speed = None     # (clock_s, measured speed) of the previous step
        self.fb_integral = 0.0     # longitudinal_mode feedback: PI integral (kept across restarts)
        self.fb_command = None     # last PI drive; None = PI not engaged (start phase, or just reset)
        self.step_times = []
        if self.solver is not None and hasattr(self.solver, "reset"):
            self.solver.reset()

    def turn_drive(self, steer):
        """Extra drive the turn drag takes at normalized steering steer (0 while off)."""
        return self.cfg.turn_drive_per_steer*abs(steer)

    def predicted_speed(self, speed, start_s, end_s):
        """Longitudinal model from start_s to end_s with the commands actually in force.

        The deadband carries the turn drag of the steering command in force, so the bias
        does not learn it a second time."""
        p = self.vehicle
        count = max(1, math.ceil((end_s-start_s)/0.01))
        h = (end_s-start_s)/count
        for i in range(count):
            drive, steer = self.command_at(start_s + i*h - p.drive_delay_s)
            d0 = min(p.drive_max, max(0.0, p.drive_deadband + self.bias + self.turn_drive(steer)))
            effort = drive - d0
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

    def feedback_drive(self, speed, target, dt, fresh, drive_cap, turn=0.0):
        """longitudinal_mode feedback: the wheel-speed PI drive and its debug fields.

        The first call after the start phase (fb_command None) slews from the start drive in
        force; the integral keeps what it learned (matching it to the start drive would carry
        the start's overshoot on: 10-19, 74% of handovers reached the soft line). A held wheel
        reading neither integrates nor moves the drive. Soft overspeed caps it at the sustain drive and cut sets 0; the
        integral then only falls (the speed is above the target). drive_cap applies at once,
        other changes at most fb_slew_per_s*dt. turn (turn_drive of the steering in force) is
        feedforward: a bend's drag left to the integral overshoots when the bend ends.
        """
        c, p = self.cfg, self.vehicle
        ff = (c.fb_sustain_drive or p.drive_deadband) + turn
        lim, err = c.fb_integral_max, target - speed
        top = max(0.0, min(p.drive_max, drive_cap))
        info = {"fb_target": target, "fb_error": err, "fb_frozen": False}
        if self.fb_command is None:
            self.fb_command = min(top, max(0.0, self.applied_drive))
        if not fresh:
            self.fb_command = min(top, self.fb_command)
            info["fb_frozen"] = True
        else:
            raw = ff + c.fb_kp*err + self.fb_integral
            if 0.0 <= raw <= top or (raw < 0.0 and err > 0) or (raw > top and err < 0):
                self.fb_integral = min(lim, max(-lim, self.fb_integral + c.fb_ki*err*dt))
            command = min(top, max(0.0, ff + c.fb_kp*err + self.fb_integral))
            if self.overspeed_state == "soft":
                command = min(command, ff)
            change = c.fb_slew_per_s*dt
            command = min(self.fb_command + change, max(self.fb_command - change, command))
            if self.overspeed_state == "cut":
                command = 0.0
            self.fb_command = min(top, max(0.0, command))
        info.update(fb_integral=self.fb_integral, fb_command=self.fb_command)
        return self.fb_command, info

    def command_at(self, time_s):
        """Applied command in force at time_s; zero before the run, held after the last."""
        command = (0.0, 0.0)
        for issued, drive, steer in self.history:
            if issued > time_s + 1e-9:
                break
            command = (drive, steer)
        return command

    def initial_state(self, speed, hold_speed=False):
        """Model state when the command computed now takes effect.

        Steering is replayed from rest for long enough to settle, then the whole
        state is propagated over the larger actuator delay with the commands already
        sent; the channel with the shorter delay holds its last command meanwhile.
        hold_speed (mpc.measured_motion): the speed is held through the delay instead.
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
                if hold_speed:
                    drive = p.drive_deadband + p.drag_kg_per_m*z[3]*z[3]/p.motor_gain_n
                z = course_model_step(z, (drive, self.command_at(t-steer_delay)[1]), sub, p)
        return z, delta_now

    def overspeed_check(self, speed, fresh=True, dt=0.0):
        """Soft overspeed state ("ok"/"soft"/"cut") and whether it must lock now.

        A held (not fresh) wheel reading does not advance the cut timeout (its start moves
        forward by dt) and neither cuts nor releases the drive: it is the last value of a
        slowing or stopped wheel, never a fast one (10-19 18:20 bags: drive held at 0 or the
        soft floor on such readings stalled the car in 5 of 10 cases). The hard line keeps
        timing on any reading.
        """
        cfg, now = self.cfg, self.clock_s
        line = cfg.v_exec_max_mps + cfg.overspeed_soft_margin_mps
        if speed > line:
            if self.overspeed_since is None:
                self.overspeed_since = now
            elif not fresh:
                self.overspeed_since += dt
        else:
            self.overspeed_since = None
        if cfg.overspeed_hard_mps > 0 and speed > cfg.overspeed_hard_mps:
            self.hard_since = now if self.hard_since is None else self.hard_since
        else:
            self.hard_since = None
        self.overspeed_state = ("ok" if not fresh else "cut" if speed > line + cfg.overspeed_cut_margin_mps
                                else "soft" if speed > line else "ok")
        return ((self.overspeed_since is not None
                 and now - self.overspeed_since >= cfg.overspeed_cut_timeout_s - 1e-9)
                or (self.hard_since is not None
                    and now - self.hard_since >= cfg.overspeed_hard_persist_s - 1e-9))

    def gate_problem(self):
        cfg = self.cfg
        if cfg.shadow:
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

    def step(self, traj, speed_mps, dt, now_s, is_first, speed_fresh=True, speed_raw=None, steer_slew=None):
        """steer_slew: the largest steering change allowed this tick (None = no limit)."""
        cfg, t0 = self.cfg, time.perf_counter()
        d = {"branch": None, "reject_reason": None, "shadow": cfg.shadow, "dt": dt,
             "speed_mps": speed_mps, "v_ref_mps": None, "v_ref_end_mps": None,
             "reference_speed_mode": "scalar", "planned_stop": False,
             "e_y_m": None, "e_psi_rad": None,
             "candidate_delta_rad": None, "candidate_steer_action": None, "candidate_drive": None,
             "applied_steer_action": 0.0, "applied_drive": 0.0, "path_id": None, "ref_age_s": None,
             "simulation_only": None, "status": None, "iterations": None, "solve_s": None,
             "step_s": None, "delta_est_rad": None, "pred_e_y_end_m": None, "pred_v_end_mps": None,
             "vehicle_source": self.vehicle.source, "vehicle_valid": self.vehicle.valid, "dropout_s": None,
             "reference_source": None, "planning_bypassed": None, "phase": self.phase,
             "drive_bias": self.bias, "plan_index": None, "step_p95_s": None,
             # New 2026-10-15: the full predicted horizon (base_link x,y at the
             # reference time), for the /car/mpc_prediction Foxglove line.
             "pred_xy": None,
             # New 2026-10-18: soft overspeed state and time above the soft line;
             # how far the prediction ran onto the straight forward extension.
             "overspeed_state": None, "overspeed_s": None, "forward_extension_m": None,
             # New 2026-10-19 (night): turn-drag deadband shift, start handover cap active, steering
             # change limit after a reference-source switch (None = none).
             "turn_drive": 0.0, "handover": False, "steer_slew": steer_slew,
             # New 2026-10-20: longitudinal mode and its wheel-speed PI (None while not engaged).
             "longitudinal_mode": cfg.longitudinal_mode, "prediction_mode": "force_model",
             "fb_target": None, "fb_error": None, "fb_integral": None, "fb_command": None,
             "fb_frozen": None, "fb_bypass": None}
        soft = cfg.overspeed_soft_margin_mps > 0 and not cfg.shadow
        self.last_debug = d
        if isinstance(traj, dict):
            d["path_id"], d["simulation_only"] = traj.get("path_id"), traj.get("simulation_only")
            d["reference_source"] = traj.get("reference_source")
            d["planning_bypassed"] = traj.get("planning_bypassed")
            if finite_number(traj.get("timestamp_s")) and finite_number(now_s):
                d["ref_age_s"] = now_s - traj["timestamp_s"]

        def slew(steer):
            if steer_slew is None:
                return steer
            return min(self.applied_steer + steer_slew, max(self.applied_steer - steer_slew, steer))

        def record(drive, steer):
            d["phase"], d["drive_bias"] = self.phase, self.bias
            self.applied_drive, self.applied_steer = drive, steer
            self.history.append((self.clock_s, drive, steer))
            keep = self.clock_s - 3.0
            while len(self.history) > 1 and self.history[1][0] <= keep:
                self.history.pop(0)
            d.update(applied_steer_action=steer, applied_drive=drive)

        def reject(branch, reason, tolerable=False):
            # A tolerable reference gap must not mask a stop while an old plan is held.
            # These guards also run below for a fresh, successfully solved reference.
            # With the soft overspeed on, its lock was already decided for this step.
            if tolerable and not cfg.shadow:
                if (not soft and finite_number(speed_mps)
                        and speed_mps > cfg.v_exec_max_mps + cfg.overspeed_margin_mps):
                    branch, reason, tolerable = "stop", "overspeed", False
                elif (self.phase == "starting" and self.start_since is not None
                      and self.clock_s-self.start_since > cfg.breakaway_timeout_s):
                    branch, reason, tolerable = "stop", "no_motion_after_breakaway", False
            d["branch"], d["reject_reason"] = branch, reason
            d["step_s"] = time.perf_counter()-t0
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
                        drive, steer = min(inputs[index][0], self.applied_drive), slew(inputs[index][1])
                        if self.phase == "starting" and self.start_since is not None:
                            # Breakaway overrides drive0 after the QP is solved, so its
                            # future inputs may be too small to start the real car. Keep
                            # only the already-applied start effort, never ramp up here.
                            drive = min(self.applied_drive, self.vehicle.drive_max)
                        if soft and self.overspeed_state != "ok":
                            drive = 0.0      # above the soft line: coast while the plan holds
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
            steer, out_drive = (0.0, 0.0) if cfg.shadow else (slew(steer_action), drive)
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
        feedback = cfg.longitudinal_mode == "feedback"
        if speed_fresh:
            if not feedback:         # the PI integral takes the bias's place
                self.update_bias(speed_mps)
        else:
            self.last_speed = None   # a held reading is no residual; restart after it
        breakaway = self.vehicle.drive_breakaway > 0
        if speed_mps >= cfg.moving_speed_mps:
            if self.phase != "tracking" or self.tracking_since is None:
                self.tracking_since = self.clock_s
            if self.phase == "starting" and breakaway:
                self.handover_since = self.clock_s
            self.phase, self.start_since = "tracking", None
            if (cfg.restart_count_reset_s > 0
                    and self.clock_s - self.tracking_since >= cfg.restart_count_reset_s - 1e-9):
                self.restarts = 0
        elif self.phase == "tracking" and breakaway and speed_mps < (cfg.rest_speed_mps or cfg.moving_speed_mps):
            # Between rest_speed_mps and moving_speed_mps the phase stays as it is (hysteresis).
            self.phase, self.start_since, self.tracking_since = "starting", None, None
            self.handover_since = None
            self.fb_command = None   # the start drive takes over; the PI re-engages bumplessly
            if (cfg.restart_merge_s <= 0 or self.last_restart_s is None
                    or self.clock_s - self.last_restart_s >= cfg.restart_merge_s - 1e-9):
                self.restarts += 1
                self.last_restart_s = self.clock_s
            if not cfg.shadow and self.restarts > cfg.max_restarts:
                return reject("stop", "too_many_restarts")
        d["restarts"] = self.restarts
        d["phase"] = self.phase
        if soft:
            must_stop = self.overspeed_check(speed_mps, speed_fresh, d["dt"])
            d["overspeed_state"] = self.overspeed_state
            d["overspeed_s"] = (0.0 if self.overspeed_since is None
                                else self.clock_s - self.overspeed_since)
            if must_stop:
                return reject("stop", "overspeed")
        if not is_first and dt > cfg.dt_max_s:
            return reject("ref_invalid", "dt_late", tolerable=True)
        if not isinstance(traj, dict) or traj.get("schema") != "reference_trajectory_v0.1":
            return reject("ref_invalid", "ref_missing", tolerable=True)
        stamp, lifetime = traj.get("timestamp_s"), traj.get("valid_for_s")
        if not traj.get("valid"):
            return reject("ref_invalid", traj.get("reason") or "ref_invalid", tolerable=True)
        if traj.get("stop_required"):
            if (cfg.geometry_glitch_tolerable and traj.get("reason") in self.GEOMETRY_GLITCH_REASONS
                    and traj.get("planned_stop") is not True):
                return reject("ref_invalid", traj["reason"], tolerable=True)
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
            # Profile time is measured from the original planning time, which
            # motion transport keeps in speed_profile_timestamp_s.
            profile_stamp = traj.get("speed_profile_timestamp_s")
            profile_stamp = stamp if profile_stamp is None else profile_stamp
            if not finite_number(profile_stamp) or profile_stamp > stamp + 1e-9:
                return reject("stop" if planned_stop else "ref_invalid", "bad_speed_profile")
            first_time = (max(0.0, now_s-profile_stamp)
                          + max(self.vehicle.drive_delay_s, self.vehicle.steering_delay_s))
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
        info, reason = mpc_reference_errors(traj, cfg.horizon_n, cfg.dt_pred_s, 0.0,
                                            cfg.max_backward_extension_m, cfg.max_forward_extension_m)
        if info is None:
            return reject("ref_invalid", reason, tolerable=True)
        d.update(e_y_m=info["e_y_m"], e_psi_rad=info["e_psi_rad"],
                 v_ref_mps=speed_targets[0] if speed_targets is not None else info["v_ref_mps"])
        if not cfg.shadow and not soft and speed_mps > cfg.v_exec_max_mps + cfg.overspeed_margin_mps:
            return reject("stop", "overspeed")
        cap = cfg.v_exec_max_mps if cfg.v_exec_max_mps > 0 else math.inf
        if min(d["v_ref_mps"], cap) <= 0.0:
            self.nominal, self.plan = None, None
            return accept("zero_target", self.applied_steer, 0.0)
        # Feedback PI only while tracking (or without a start phase), and not on a shadow
        # time profile, which needs the joint QP's drive.
        pi = feedback and (self.phase == "tracking" or not breakaway) and speed_targets is None
        hold = pi and cfg.measured_motion
        if feedback:
            d.update(fb_bypass="speed_profile" if speed_targets is not None else None,
                     prediction_mode="measured_speed_hold" if hold else "force_model")
        z0, delta_now = self.initial_state(speed_mps, hold) if hold else self.initial_state(speed_mps)
        d["delta_est_rad"] = delta_now
        turn = self.turn_drive(self.applied_steer)   # feedback: PI feedforward, not a QP bias
        d["turn_drive"] = turn
        extra = {}
        if pi:
            pass                     # drive is fixed below; no bias or deadband floor in the QP
        elif self.bias or turn or self.vehicle.drive_deadband > 0:
            extra = {"drive_bias": self.bias + turn, "moving": self.phase == "tracking"}
        if speed_targets is not None:
            extra.update(speed_targets=speed_targets, allow_braking=planned_stop)
        if soft and self.overspeed_state != "ok" and not pi:
            extra["drive_floor_zero"] = True
        # The raw wheel reading leads the filtered speed by ~0.1-0.2 s; taking the larger
        # drops the low-speed cap as soon as the car pulls away (10-19 18:45: 0.09 -> 0.47
        # m/s out of a hairpin while the filtered speed still allowed 0.40).
        pace = max(speed_mps, speed_raw) if finite_number(speed_raw) else speed_mps
        if cfg.high_speed_drive_max > 0 and pace >= cfg.high_speed_mps:
            extra["drive_cap"] = cfg.high_speed_drive_max
        d["drive_cap"] = extra.get("drive_cap", self.vehicle.drive_max)
        if pi:
            fb, info = self.feedback_drive(speed_mps, min(d["v_ref_mps"], cap), d["dt"], speed_fresh,
                                           d["drive_cap"], turn)
            d.update(info)
            extra.pop("drive_cap", None)   # already applied to fb
            extra["fixed_drive"] = fb
            if hold:
                extra["hold_speed"] = True
        try:
            solution, reason = self.solver.solve(traj["points"], z0, cap, self.applied_drive, self.nominal, **extra)
        except MPCStop as exc:
            numerical = str(exc).startswith("solver_status:") or str(exc) in ("constraint_residual",
                                                                               "solver_nonfinite")
            return reject("solver_fail", str(exc), tolerable=cfg.solver_fail_tolerable and numerical)
        except Exception as exc:
            return reject("solver_fail", f"solver_exception:{type(exc).__name__}")
        if solution is None:
            return reject("ref_invalid", reason, tolerable=True)
        drive, steer = solution["drive0"], solution["steer0"]
        if not (finite_number(drive) and finite_number(steer) and abs(steer) <= 1.0 + 1e-6
                and -1e-6 <= drive <= 1.0 + 1e-6):
            return reject("solver_fail", "action_out_of_bounds", tolerable=cfg.solver_fail_tolerable)
        drive, steer = min(1.0, max(0.0, drive)), min(1.0, max(-1.0, steer))
        inputs = solution["inputs"]
        self.nominal = inputs[1:] + inputs[-1:]
        if soft and self.overspeed_state == "cut":
            # Well above the soft line: no drive, and no warm start that carries it.
            drive, self.nominal = 0.0, None
        if breakaway and self.phase == "starting" and not cfg.shadow:
            # Static friction: the MPC's small model-based drive would not start the car.
            if self.start_since is None:
                self.start_since = self.clock_s
            if self.clock_s - self.start_since > cfg.breakaway_timeout_s:
                return reject("stop", "no_motion_after_breakaway")
            drive = min(self.vehicle.drive_max, self.vehicle.drive_breakaway
                        + cfg.breakaway_ramp_per_s*(self.clock_s - self.start_since))
        if (cfg.handover_s > 0 and not feedback and self.phase == "tracking" and self.handover_since is not None
                and self.clock_s - self.handover_since < cfg.handover_s - 1e-9):
            # Just pulled away: the start drive's momentum overshot (10-19: 74% of handovers
            # reached the soft overspeed line within 2.5 s). Stay near the effective deadband.
            p = self.vehicle
            d_eff = min(p.drive_max-1e-3, max(0.0, p.drive_deadband + self.bias + turn))
            drive, d["handover"] = min(drive, d_eff + cfg.handover_drive_margin), True
        self.plan = (self.clock_s, inputs)
        d.update(candidate_delta_rad=solution["delta0"], candidate_steer_action=steer, candidate_drive=drive,
                 status="solved", iterations=solution["iterations"], solve_s=solution["solve_s"],
                 pred_e_y_end_m=solution["pred_e_y"][-1], pred_v_end_mps=solution["pred_v"][-1],
                 # .get: test doubles for self.solver predate this 2026-10-15 field.
                 pred_xy=solution.get("pred_xy"), forward_extension_m=solution.get("forward_extension_m"))
        return accept("shadow" if cfg.shadow else "solved", steer, drive)


def mpc_debug_json(debug):
    """Compact JSON for the mpc_debug topic; non-finite numbers become null."""
    clean = {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in debug.items()}
    return json.dumps(clean, separators=(",", ":"))
# Vehicle-identification policy: pure scheduling logic, no ROS or hardware I/O.
IDENTIFICATION_DEFAULTS = {
    "mode": "off",                 # off / steering / drive / brake / arc
    "confirmed": False,            # operator has reviewed this specific sequence
    "negative_drive_confirmed": False,
    "durations_s": [1.0],
    "drive_values": [0.0],
    "steering_values": [0.0],
    "drive_min": 0.0,
    "drive_max": 0.0,
    "steering_abs_max": 0.0,
    "speed_limit_mps": 0.0,         # must be selected before enabling a test
    "start_speed_max_mps": 0.03,
    "settle_s": 2.0,               # zero requests before and after the sequence
    "max_dt_s": 0.15,
    "max_run_s": 30.0,
}


class IdentificationExperiment:
    """One finite sequence per explicit policy start; limits are requests, not brakes."""

    def __init__(self, config):
        self.config = dict(config)
        c = self.config
        self.mode = c["mode"]
        if self.mode not in ("off", "steering", "drive", "brake", "arc"):
            raise ValueError("id_test.mode must be off, steering, drive, brake or arc")
        self.finished = False
        self.reason = ""
        self.last_elapsed = None
        self.ends = []
        if self.mode == "off":
            return
        if c["confirmed"] is not True:
            raise ValueError("Set id_test.confirmed only after reviewing the test")
        for key in ("drive_min", "drive_max", "steering_abs_max", "speed_limit_mps",
                    "start_speed_max_mps", "settle_s", "max_dt_s", "max_run_s"):
            if not finite_number(c[key]):
                raise ValueError(f"id_test.{key} must be finite")
        if not (-1 <= c["drive_min"] <= 0 <= c["drive_max"] <= 1):
            raise ValueError("Invalid identification drive limits")
        if not 0 <= c["steering_abs_max"] <= 1:
            raise ValueError("Invalid identification steering limit")
        if not (0 <= c["start_speed_max_mps"] < c["speed_limit_mps"]):
            raise ValueError("Set a positive speed limit above the start-speed threshold")
        if not (0 < c["max_dt_s"] < c["settle_s"] and
                2 * c["settle_s"] < c["max_run_s"] <= 120):
            raise ValueError("Require max_dt < settle and 2*settle < max_run <= 120 s")
        durations, drives, steers = (c[k] for k in
                                     ("durations_s", "drive_values", "steering_values"))
        if not (1 <= len(durations) <= 100 and len(durations) == len(drives) == len(steers)):
            raise ValueError("Test arrays must have the same length (1..100)")
        total = c["settle_s"]
        for duration, drive, steer in zip(durations, drives, steers):
            if not all(finite_number(v) for v in (duration, drive, steer)):
                raise ValueError("Test arrays must contain finite numbers")
            if duration <= c["max_dt_s"]:
                raise ValueError("Each stage must last longer than max_dt_s")
            if not c["drive_min"] <= drive <= c["drive_max"]:
                raise ValueError("Drive request exceeds the reviewed limits")
            if abs(steer) > c["steering_abs_max"]:
                raise ValueError("Steering request exceeds the reviewed limit")
            if self.mode == "steering" and drive != 0:
                raise ValueError("Steering tests require zero drive throughout")
            if self.mode in ("drive", "brake") and steer != 0:
                raise ValueError("Drive/brake tests require zero steering requests")
            # arc: constant forward drive with constant steering per stage, for
            # the steering map from yaw rate / speed (no wheel-angle protractor).
            if self.mode == "arc" and drive < 0:
                raise ValueError("Arc tests drive forward only")
            if drive < 0 and (self.mode != "brake" or
                              c["negative_drive_confirmed"] is not True):
                raise ValueError("Negative drive requires brake mode and ESC confirmation")
            total += duration
            self.ends.append(total)
        self.total_s = total + c["settle_s"]
        if self.total_s > c["max_run_s"]:
            raise ValueError("Sequence including settle periods exceeds max_run_s")

    def step(self, elapsed, dt, speed):
        c = self.config
        result = {"drive": 0.0, "steer": 0.0, "stage": -1,
                  "phase": "off", "terminal": False, "reason": ""}
        if self.mode == "off":
            return result
        if self.finished:
            return {**result, "phase": "finished", "terminal": True,
                    "reason": self.reason}
        reason = ""
        if not all(finite_number(v) for v in (elapsed, dt, speed)):
            reason = "Missing or nonfinite time/speed"
        elif elapsed < 0 or dt < 0 or speed < 0:
            reason = "Negative time or unsigned speed"
        elif self.last_elapsed is None and (elapsed > c["max_dt_s"] or
                                           speed > c["start_speed_max_mps"]):
            reason = "Start must be timely and stationary within the selected threshold"
        elif self.last_elapsed is not None and (elapsed < self.last_elapsed or
                elapsed - self.last_elapsed > c["max_dt_s"] or dt > c["max_dt_s"]):
            reason = "Policy timing gap: sequence aborted, no catch-up"
        elif speed >= c["speed_limit_mps"]:
            reason = "Speed guard reached"
        elif elapsed < c["settle_s"] and speed > c["start_speed_max_mps"]:
            reason = "Vehicle moved during pre-test zero interval"
        elif elapsed >= self.total_s:
            reason = "Sequence complete; zero request is not proof of physical stopping"
        if reason:
            self.finished, self.reason = True, reason
            return {**result, "phase": "finished", "terminal": True, "reason": reason}
        self.last_elapsed = elapsed
        if elapsed < c["settle_s"]:
            return {**result, "phase": "settle"}
        for index, end in enumerate(self.ends):
            if elapsed < end:
                return {**result, "phase": "stage", "stage": index,
                        "drive": c["drive_values"][index],
                        "steer": c["steering_values"][index]}
        return {**result, "phase": "tail"}


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


def _planning_control_geometry(reference, now_s):
    if ("control_cached_path_samples" in reference
            or (reference.get("path") or {}).get("type") == "CARTESIAN_SAMPLES"):
        return _control_cached_geometry(reference)
    geometry = control_path_geometry(reference.get("path"))
    if reference.get("planner_version") in ("frenet_lattice_v2", "mvp2_centerline"):
        preview = reference.get("control_preview_m")
        bounds = reference["path"]["range"]
        if not finite_number(preview) or preview <= 0:
            raise ValueError("invalid_lattice_control_preview")
        x = min(bounds[1], max(bounds[0], preview))
        point = evaluate_planning_path(reference, x, now_s)
        if point is None:
            raise ValueError("lattice_preview_unavailable")
        # A planned path starts at the car, so closest-point error alone is zero.
        geometry.update(path_error_m=point["position_xy_m"][1], path_heading_rad=point["heading_rad"],
                        curvature_1pm=point["curvature_1pm"], control_preview_x_m=x)
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


def _control_cached_geometry(reference):
    """Query transported Cartesian curve samples without refitting or flattening.

    Samples hold x/y, tangent heading and curvature. Rigid motion preserves
    curvature. Interpolation is confined to observed path support; V2 retains
    its body-x preview convention, while V1 uses the nearest segment.
    """
    points = _control_sample_points(reference)

    def interpolate(a, b, t):
        return (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                wrap_angle(a[2]+t*wrap_angle(b[2]-a[2])), a[3]+t*(b[3]-a[3]))

    candidates = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, -(a[0]*dx+a[1]*dy)/(dx*dx+dy*dy)))
        p = interpolate(a, b, t)
        candidates.append((p[0]**2+p[1]**2, i, t, p))
    _, index, t, closest = min(candidates)
    x, y, heading, curvature = closest
    geometry = {"path_error_m": -x*math.sin(heading)+y*math.cos(heading),
                "path_heading_rad": heading, "curvature_1pm": curvature,
                "closest_x_m": x, "closest_y_m": y,
                "path_degree": (len(reference["path"]["coeffs_low_to_high"])-1
                                if reference["path"]["type"] == "CARTESIAN_Y_OF_X" else None),
                "path_encoding": reference["path"]["type"],
                "closest_at_range_end": index == 0 and t == 0 or index == len(points)-2 and t == 1}
    if reference.get("planner_version") in ("frenet_lattice_v2", "mvp2_centerline"):
        preview = reference.get("control_preview_m")
        if not finite_number(preview) or preview <= 0:
            raise ValueError("invalid_lattice_control_preview")
        preview = max(points[0][0], min(points[-1][0], preview))
        _, y, heading, curvature = _control_sample_at(points, preview)
        geometry.update(path_error_m=y, path_heading_rad=heading, curvature_1pm=curvature,
                        control_preview_x_m=preview)
    return geometry


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
                geometry = _planning_control_geometry(reference, now_s)
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
            geometry = _planning_control_geometry(reference, now_s)
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


# ---- MVP2.1 lattice planner and execution adapter --------------------------
# Ported from origin/mvp2.1 3652e9b (scripts/policy_node.py 2900-3858) on
# 2026-10-16. Verbatim except where marked "mpc-car-run adaptation". Keep it
# that way so later mvp2.1 changes can be diffed in directly.

# mpc-car-run adaptation (2026-10-20): the margin PlannerExecutionManager.select
# checks past preview_m (mvp2.1 wrote 0.15 inline there). The planner's
# check_length_mode "max"/"travel_plus_window" uses the same value so that the
# planning-time collision check covers the execution check's window
# (docs/LATTICE_CHECK_LENGTH_PLAN_CN.md).
EXECUTION_TAIL_M = 0.15


@dataclass
class ShadowSettings:
    """New shadow-only design. No settings or code from the archived lattice.

    Dimensions below are explicitly provisional; replace with measurements for
    the tested car. No actuator/vehicle model is used. Optional LiDAR ranking
    and swept-body feasibility share the configured rectangle and margin.
    This planner returns geometry; the policy adapter owns waiting/stopping.
    """
    enabled: bool = False
    rate_hz: float = 1.0
    length_m: float = 1.0
    sample_count: int = 48
    path_spacing_m: float = 0.025
    handoff_allowance_s: float = 0.10
    # mpc-car-run adaptation (2026-10-20): planning-time collision check length.
    # "travel" (mvp2.1): speed*hold. "max": max(speed*hold, preview+EXECUTION_TAIL_M).
    # "travel_plus_window": speed*hold+preview+EXECUTION_TAIL_M.
    check_length_mode: str = "travel"
    # mpc-car-run adaptation (from sim-branch 8f1f01d, 2026-10-21): "prefix" keeps the
    # lengths above; "generated" checks (collision) or scores (score) the whole
    # generated path, max(length_m, score length, check+preview).
    collision_coverage: str = "prefix"
    score_coverage: str = "prefix"
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
    # Loss alone never stops; optional body overlap rejection is independent of cost.
    obstacle_avoidance_enabled: bool = False
    obstacle_check_enabled: bool = False
    weight_obstacle: float = 30.0
    obstacle_margin_m: float = 0.02
    fusion_enabled: bool = True
    fusion_window_s: float = 1.0
    fusion_max_samples: int = 21
    fusion_spacing_m: float = 0.10
    fusion_memory_weight: float = 1.0
    fusion_rear_support_m: float = 0.30
    # fusion-prior-reset experiment (2026-10-28): false never uses the previous reference
    # as the fusion guide (car 01 U-turn: the prior guide shrank and pointed right).
    fusion_use_prior: bool = True

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("shadow.enabled must be boolean")
        if not isinstance(self.fusion_enabled, bool):
            raise ValueError("shadow.fusion_enabled must be boolean")
        if not isinstance(self.fusion_use_prior, bool):
            raise ValueError("shadow.fusion_use_prior must be boolean")
        if not isinstance(self.obstacle_avoidance_enabled, bool):
            raise ValueError("shadow.obstacle_avoidance_enabled must be boolean")
        if not isinstance(self.obstacle_check_enabled, bool):
            raise ValueError("shadow.obstacle_check_enabled must be boolean")
        if type(self.fusion_max_samples) is not int or not 1 <= self.fusion_max_samples <= 128:
            raise ValueError("shadow.fusion_max_samples must be an integer in [1,128]")
        if not isinstance(self.sample_count, int) or isinstance(self.sample_count, bool) or self.sample_count < 8:
            raise ValueError("shadow.sample_count must be an integer >=8")
        if not isinstance(self.check_length_mode, str) or \
                self.check_length_mode not in ("travel", "max", "travel_plus_window"):
            raise ValueError("shadow.check_length_mode must be travel, max or travel_plus_window")
        for name in ("collision_coverage", "score_coverage"):
            if not isinstance(getattr(self, name), str) or getattr(self, name) not in ("prefix", "generated"):
                raise ValueError("shadow."+name+" must be prefix or generated")
        for name, value in vars(self).items():
            if name in ("enabled", "sample_count", "fusion_enabled", "fusion_use_prior", "fusion_max_samples",
                        "obstacle_avoidance_enabled", "obstacle_check_enabled",
                        "check_length_mode", "collision_coverage", "score_coverage"):
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
        # mpc-car-run adaptation (from sim-branch 92ef303): binary search, the same
        # first i with s <= lengths[i+1]. Imported here: _shadow_worker_entry runs
        # these definitions in a fixed namespace without module-level imports.
        from bisect import bisect_left
        i = bisect_left(lengths, s)-1
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
    if (cfg.fusion_use_prior and prior.get("odometry_epoch") == epoch and prior.get("pose")
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


def _shadow_obstacle_input(estimates, lidar_cfg, frame_id):
    """One ROI extraction for planning and for avoiding an unchecked baseline."""
    if lidar_cfg is None or not lidar_cfg.enabled:
        return {}
    source = estimates.get("obstacles") or {}
    bounds = lidar_roi_bounds(lidar_cfg)
    result = {key: source.get(key) for key in
              ("available", "time_aligned", "timestamp_s", "measurement_timestamp_s", "source_age_s")}
    result.update(roi_xy_m=bounds, points_xyz_m=[list(p) for p in source.get("points_xyz_m", [])
        if bounds[0][0] <= p[0] <= bounds[0][1] and bounds[1][0] <= p[1] <= bounds[1][1]])
    if source.get("frame_id") != frame_id:
        result["available"] = False
    return result


def _shadow_soft_road_failure(reason):
    """Geometry gaps only; motion faults and explicit conflicts are not bridged."""
    return reason in {"invalid_road_invalid_ordered_geometry", "invalid_road_invalid_boundary_pairing",
        "invalid_road_motion_transformed_geometry_invalid", "invalid_road_insufficient_geometry",
        "invalid_road_unavailable"}


def _shadow_obstacle_context(job, cfg):
    """Index the current aligned ROI once per job. Missing input adds no cost."""
    source = job.get("obstacles") or {}
    diag = {"status": "DISABLED", "point_count": 0, "weight": cfg.weight_obstacle,
            "margin_m": cfg.obstacle_margin_m, "coverage": "generated",
            "measurement_timestamp_s": source.get("measurement_timestamp_s"),
            "source_age_s": source.get("source_age_s"), "roi_xy_m": source.get("roi_xy_m")}
    cell = max(cfg.body_front_m, cfg.body_rear_m, cfg.body_width_m)
    grid = {}
    if cfg.obstacle_avoidance_enabled and (cfg.weight_obstacle > 0 or cfg.obstacle_check_enabled):
        diag["status"] = "UNAVAILABLE"
        stamp = source.get("timestamp_s")
        if (source.get("available") is True and source.get("time_aligned") is True
                and finite_number(stamp) and finite_number(job.get("stamp_s"))
                and abs(stamp-job["stamp_s"]) <= 1e-6):
            bounds = source.get("roi_xy_m")
            for point in source.get("points_xyz_m") or []:
                if len(point) < 2 or not all(finite_number(v) for v in point[:2]):
                    continue
                x, y = point[:2]
                if bounds and not (bounds[0][0] <= x <= bounds[0][1] and bounds[1][0] <= y <= bounds[1][1]):
                    continue
                grid.setdefault((math.floor(x/cell), math.floor(y/cell)), []).append((x, y))
                diag["point_count"] += 1
            diag["status"] = "ACTIVE" if grid else "CLEAR"
    return grid, cell, diag


def _shadow_obstacle_distances(pose, context, cfg, padding, overlap_only=False):
    """Nearby measured points and signed distance to this oriented footprint."""
    grid, cell, _ = context
    x, y, heading = pose
    c, s = math.cos(heading), math.sin(heading)
    half = cfg.body_width_m/2
    along, across = max(cfg.body_front_m, cfg.body_rear_m)+padding, half+padding
    rx, ry = abs(c)*along+abs(s)*across, abs(s)*along+abs(c)*across
    for ix in range(math.floor((x-rx)/cell), math.floor((x+rx)/cell)+1):
        for iy in range(math.floor((y-ry)/cell), math.floor((y+ry)/cell)+1):
            for ox, oy in grid.get((ix, iy), ()):
                dx, dy = ox-x, oy-y
                local_x, local_y = c*dx+s*dy, -s*dx+c*dy
                qx = max(local_x-cfg.body_front_m, -cfg.body_rear_m-local_x)
                qy = abs(local_y)-half
                if overlap_only and (qx >= padding or qy >= padding):
                    continue
                distance = max(qx, qy) if qx <= 0.0 or qy <= 0.0 else math.hypot(qx, qy)
                yield distance, (ox, oy)


def _shadow_obstacle_collision(path, context, cfg):
    """Whole-path body sweep against measured points, using existing resolutions."""
    if not cfg.obstacle_check_enabled or not context[0]:
        return None
    previous = path[0]
    for point in path:
        angle = wrap_angle(point[2]-previous[2])
        count = max(1, math.ceil(math.dist(point[:2], previous[:2])/cfg.sweep_translation_step_m),
                    math.ceil(abs(angle)/cfg.sweep_rotation_step_rad))
        for i in range(1, count+1):
            u = i/count
            pose = (previous[0]+u*(point[0]-previous[0]), previous[1]+u*(point[1]-previous[1]),
                    previous[2]+u*angle)
            for distance, hit in _shadow_obstacle_distances(pose, context, cfg, cfg.obstacle_margin_m, True):
                if distance < cfg.obstacle_margin_m-1e-9:
                    return {"side": "lidar", "point_xy_m": hit, "pose": pose,
                            "clearance_m": distance, "reason": "body_obstacle_overlap"}
        previous = point
    return None


def _shadow_obstacle_score(path, context, cfg):
    """Finite whole-path proximity cost, using sampled oriented body rectangles.

    Half a body width is the soft fall-off scale, not another collision gate.
    Peak plus mean exposure avoids dependence on the number of laser returns.
    A reported clearance is the minimum among points in the local query boxes;
    None means no nearby point was scored, not verified free space.
    """
    if not context[0]:
        return 0.0, None
    half = cfg.body_width_m/2
    padding = cfg.obstacle_margin_m+half
    peak, exposure, clearance = 0.0, 0.0, None
    for pose in path:
        penalty = 0.0
        for distance, _ in _shadow_obstacle_distances(pose, context, cfg, padding):
            clearance = distance if clearance is None else min(clearance, distance)
            penalty = max(penalty, max(0.0, (padding-distance)/half)**2)
        peak = max(peak, penalty)
        exposure += penalty
    return peak+exposure/max(1, len(path)), clearance


class ShadowLatticePlanner:
    """Fresh bounded sampler / prefix ranker, deliberately permissive about road.

    Input validity flags, short coverage, absent sides, slope, curvature and
    normalized steering requests are NOT admission gates. Only absent usable
    geometry suppresses generation. Known-road and optional LiDAR overlaps remove candidates;
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
        preview = job.get("preview_m", 0.0)
        preview = preview if finite_number(preview) and preview > 0 else cfg.score_min_length_m
        # mpc-car-run adaptation (2026-10-20): mvp2.1 checks only speed*hold, but
        # PlannerExecutionManager.select checks to preview+EXECUTION_TAIL_M past
        # the car on every tick while the path is in use.
        travel_check = speed*hold
        execution_window = preview+EXECUTION_TAIL_M
        if cfg.check_length_mode == "max":
            check = max(travel_check, execution_window)
        elif cfg.check_length_mode == "travel_plus_window":
            check = travel_check+execution_window
        else:
            check = travel_check
        score_length = max(cfg.score_min_length_m, check)
        horizon = max(cfg.length_m, score_length, check+preview)
        # mpc-car-run adaptation (from sim-branch 8f1f01d): the path is generated to
        # horizon either way; the controller still reads check+preview of it.
        control_read_length = check+preview
        if cfg.collision_coverage == "generated":
            check = horizon
        if cfg.score_coverage == "generated":
            score_length = horizon
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
        obstacle_context = _shadow_obstacle_context(job, cfg)
        candidates = []
        # mpc-car-run adaptation (from sim-branch 92ef303): candidates share knot
        # stations, so look each one up once per job.
        knot_frames = {}
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
            if collision is None:
                collision = _shadow_obstacle_collision(path, obstacle_context, cfg)
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
            obstacle_cost, obstacle_clearance = _shadow_obstacle_score(path, obstacle_context, cfg)
            if cfg.obstacle_avoidance_enabled:
                costs["obstacle"] = obstacle_cost
            cost = sum(getattr(cfg, "weight_"+key)*value for key, value in costs.items())
            candidates.append({"id": candidate_id, "family": family, "offsets_m": offsets,
                "cost": cost, "costs": costs, "collision": collision, "path": path,
                "scoring_samples": scoring_samples})
            if cfg.obstacle_avoidance_enabled:
                candidates[-1]["obstacle_clearance_m"] = obstacle_clearance
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
                "check_length_mode": cfg.check_length_mode, "travel_check_m": travel_check,
                "execution_window_m": execution_window,
                "collision_coverage": cfg.collision_coverage, "score_coverage": cfg.score_coverage,
                "control_read_length_m": control_read_length, "known_boundary_sides": list(boundaries),
                "body_dimensions_m": [cfg.body_front_m, cfg.body_rear_m, cfg.body_width_m],
                "body_dimensions_source": "shadow_configuration_provisional",
                "collision_method": "adaptive_pose_sampling_no_padding",
                "checked_only_known_supported_boundaries": True,
                "observed_centerline_length_m": frame.end}
        if cfg.obstacle_avoidance_enabled:
            result["planning_input"]["obstacles"] = job.get("obstacles")
            result["obstacle_avoidance"] = {**obstacle_context[2],
                "selected_cost": selected["costs"].get("obstacle", 0.0),
                "selected_weighted_cost": cfg.weight_obstacle*selected["costs"].get("obstacle", 0.0),
                "selected_nearby_clearance_m": selected.get("obstacle_clearance_m"),
                "hard_rejection": cfg.obstacle_check_enabled,
                "minimum_straight_gap_m": cfg.body_width_m+2*cfg.obstacle_margin_m}
            result["stop_required"] = bool(cfg.obstacle_check_enabled and obstacle_context[0] and not feasible)
            if result["stop_required"]:
                result["reason"] = "no_passable_obstacle_path"
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
    # mpc-car-run adaptation: the MPC consumes the lattice only while this tick's
    # road_centerline baseline is itself valid. mvp2.1 lets the (deliberately
    # permissive) lattice drive through baseline rejections; false restores that.
    require_valid_baseline: bool = True

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("planner_control.enabled must be boolean")
        if not isinstance(self.require_valid_baseline, bool):
            raise ValueError("planner_control.require_valid_baseline must be boolean")


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
        self.last_good_road = self.last_good_road_pose = self.last_good_road_epoch = None
        self.road_dropout_started_s = None

    def accept(self, result, preserve_no_road=False):
        # A road-data gap is not an obstacle collision. Retain only an already
        # checked path; its identity, anchor and original timestamps stay intact.
        if (preserve_no_road and result.get("status") == "NO_ROAD" and not result.get("stop_required")
                and self.active is not None and self.active.get("obstacle_checked")):
            self.active["retained_no_road_job_id"] = result.get("job_id")
            return
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
            "measurement_stamp_s": result.get("measurement_stamp_s"), "progress_m": 0.0,
            "obstacle_checked": bool((result.get("obstacle_avoidance") or {}).get("hard_rejection")
                and (result.get("obstacle_avoidance") or {}).get("status") == "ACTIVE")}
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
        if "retained_no_road_job_id" in active:
            self.diagnostics["retained_no_road_job_id"] = active["retained_no_road_job_id"]
        if remaining+1e-9 < needed:
            return fallback("preview_coverage_exhausted")
        cropped = [_path_sample(points, arc, progress)]
        cropped.extend(p for p, s in zip(points, arc) if s > progress+1e-6)
        lengths = _path_lengths(cropped)
        # Use exactly the same geometric reader as the actual PID, including
        # its tangent smoothing beyond the preview. This is NOT a rollout.
        path = {"type": "ORDERED_CENTERLINE", "independent_variable": "s_m",
                "samples_xy": cropped, "range": [0.0, lengths[-1]], "preview_m": preview_m}
        # mpc-car-run adaptation: mvp2.1 reads the cropped path with the MVP
        # PID's control_path_geometry, whose ORDERED_CENTERLINE reader this
        # branch does not have (its PID only reads y(x)). The consumer here is
        # the MPC, so project the car with the same forward-branch rule and
        # treat preview_m as the distance the MPC must track (the caller passes
        # max(planning.preview_m, MPC horizon travel)). The 0.15 m margin past
        # it (EXECUTION_TAIL_M) is kept from mvp2.1 (there: the PID tangent window).
        closest = _path_project(cropped, lengths)
        if closest is None:
            return fallback("execution_branch_unavailable")
        if lengths[-1]-closest[1]+1e-9 < needed:
            return fallback("preview_coverage_exhausted")
        check = min(lengths[-1], max(needed, closest[1]+preview_m+EXECUTION_TAIL_M))
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
        defaults.update({f"id_test.{name}": value
                         for name, value in IDENTIFICATION_DEFAULTS.items()})
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
        self.get_logger().info("Policy revision: fusion-mpc-lidar-road-hold-20261028")
        lidar_values = {}
        for name, default in vars(LidarObstacleSettings()).items():
            key = f"lidar_obstacle.{name}"
            self.declare_parameter(key, default, ParameterDescriptor(read_only=True))
            lidar_values[name] = self.get_parameter(key).value
        self.lidar_obstacle_settings = LidarObstacleSettings(**lidar_values)
        self.lidar_obstacle_output = None
        self._lidar_obstacle_stamp_ns = None
        self._lidar_obstacle_publication_key = None
        self.identification_config = {
            name: self.get_parameter(f"id_test.{name}").value
            for name in IDENTIFICATION_DEFAULTS}
        for name in ("durations_s", "drive_values", "steering_values"):
            self.identification_config[name] = list(self.identification_config[name])
        self.identification = IdentificationExperiment(self.identification_config)
        if self.identification.mode != "off":
            if self.policy_update_mode != "timer" or "wheel_speed" not in self.required_sensors:
                raise ValueError("Identification requires timer mode and required wheel_speed")
            if 1.0 / self.policy_update_rate_hz >= self.identification_config["max_dt_s"]:
                raise ValueError("Timer interval must be shorter than id_test.max_dt_s")
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
        # Ported 2026-10-15 (main's road_centerline port): the ordered_xy
        # estimator output only has a consumer in road_centerline; lattice_v2's
        # corridor geometry still needs the legacy y(x) boundary fit.
        if (self.estimation_settings.road_geometry == "ordered_xy"
                and self.planning_settings.algorithm != "road_centerline"):
            raise ValueError("Ordered XY geometry requires planning.algorithm=road_centerline")
        if (self.planning_settings.algorithm == "lattice_v2"
                and self.estimation_settings.road_geometry != "y_of_x"):
            raise ValueError("lattice_v2 requires estimation.road_geometry=y_of_x")
        lattice_values = {}
        for name, default in vars(LatticeSettings()).items():
            parameter_name = f"planning.lattice.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            lattice_values[name] = self.get_parameter(parameter_name).value
        self.lattice_settings = LatticeSettings(**lattice_values)
        self.planning_output = None
        self.planning_diagnostics = None

        mpc_values = {}
        for name, default in vars(MPCSettings()).items():
            parameter_name = f"mpc.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            mpc_values[name] = self.get_parameter(parameter_name).value
        self.mpc_settings = MPCSettings(**mpc_values)
        self.mpc_controller = None
        vehicle_values = {}
        for name, default in vars(VehicleParamsSettings()).items():
            parameter_name = f"vehicle.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            vehicle_values[name] = self.get_parameter(parameter_name).value
        self.vehicle_settings = VehicleParamsSettings(**vehicle_values)
        if self.mpc_settings.enabled and self.mpc_settings.reference_source == "planning":
            simulated_mpc = self.mpc_settings.vehicle_params_source == "course_simulation"
            simulated_planning = self.planning_settings.vehicle_limits_source == "course_simulation"
            if simulated_mpc != simulated_planning:
                raise ValueError("mpc.vehicle_params_source and planning.vehicle_limits_source "
                                 "must select course_simulation together")
        # Both produce drive/steer requests; exactly one may own the actions.
        if self.identification.mode != "off" and self.mpc_settings.enabled:
            raise ValueError("id_test.mode and mpc.enabled cannot both be active")
        for prefix, settings_type in (("control", ControlSettings),):
            values = {}
            for name, default in vars(settings_type()).items():
                parameter_name = f"{prefix}.{name}"
                self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
                values[name] = self.get_parameter(parameter_name).value
            setattr(self, f"{prefix}_settings", settings_type(**values))
        if self.mpc_settings.enabled and self.control_settings.enabled:
            raise ValueError("control.enabled and mpc.enabled cannot both be active")
        # The MVP controller and the MPC on a Planning reference share these
        # input, obstacle and reference-lifetime requirements.
        if self.control_settings.enabled or self._mpc_uses_planning():
            if not {"cone_detections", "wheel_speed", "imu_angular_velocity"}.issubset(self.required_sensors):
                raise ValueError("Integrated control requires cones, wheel_speed and imu_angular_velocity")
            if (self.planning_settings.algorithm == "lattice_v2" and self.lattice_settings.obstacle_check_enabled
                    and "lidar_cartesian" not in self.required_sensors):
                raise ValueError("Lattice obstacle checking requires lidar_cartesian")
            simulation_planning = (self.lattice_settings.model_source == "course_simulation"
                if self.planning_settings.algorithm == "lattice_v2" else
                self.planning_settings.vehicle_limits_source == "course_simulation")
            simulation_control = self.control_settings.vehicle_params_source == "course_simulation"
            if (self.control_settings.enabled and self.control_settings.mode == "calibrated"
                    and simulation_planning != simulation_control):
                raise ValueError("Planning and control simulation profiles must be selected together")
            if (self.control_settings.enabled and self.planning_settings.algorithm == "road_centerline"
                    and self.control_settings.mode != "mvp"):
                raise ValueError("Road centerline following requires control.mode=mvp")
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
        self.planning_generation = 0
        self.last_planning_submit_s = -math.inf
        self.last_async_fault = None
        self.last_worker_restart_s = -math.inf
        self.last_planner_elapsed_s = None
        self.async_planner = None
        self.mpc_held_reference = None
        self.gyro_bias_rps = 0.0
        self.gyro_bias_samples = []
        if self.lattice_settings.async_enabled:
            if (self.planning_settings.algorithm != "lattice_v2" or self.policy_update_mode != "timer"
                    or not (self._robust_control_enabled() or self._mpc_uses_planning())):
                raise ValueError("Async planning requires lattice_v2 in timer mode with robust MVP "
                                 "control or the MPC on the Planning reference")
            self.async_planner = AsyncLatticePlanner()

        # MVP2.1 lattice (ported from origin/mvp2.1). shadow.* runs the planner in a
        # separate low-priority process, display/diagnostics only. planner_control
        # additionally lets the MPC track its selected path, falling back to this
        # tick's road_centerline reference. Both are off by default.
        shadow_values = {}
        for name, default in vars(ShadowSettings()).items():
            parameter_name = f"shadow.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            shadow_values[name] = self.get_parameter(parameter_name).value
        self.shadow_settings = ShadowSettings(**shadow_values)
        planner_control_values = {}
        for name, default in vars(PlannerControlSettings()).items():
            parameter_name = f"planner_control.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            planner_control_values[name] = self.get_parameter(parameter_name).value
        self.planner_control_settings = PlannerControlSettings(**planner_control_values)
        # mpc-car-run adaptation: mvp2.1 requires the MVP PID as the consumer; this
        # branch's PID cannot read ORDERED_CENTERLINE, so the MPC is the consumer.
        if self.planner_control_settings.enabled and (
                not self.shadow_settings.enabled or not self._mpc_uses_planning()
                or self.planning_settings.algorithm != "road_centerline"
                or self.policy_update_mode != "timer"):
            raise ValueError("Planner control requires shadow, timer updates and the MPC on "
                             "road_centerline Planning")
        self.planner_preview_m = self._planner_preview_m()
        self.planner_execution = PlannerExecutionManager(self.policy_frame_id)
        self._last_execution_mode = None
        self._mpc_trajectory = None

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
        self.lidar_obstacle_publisher = self.create_publisher(String, "lidar_obstacles", 1)
        self.lidar_points_publisher = self.create_publisher(PointCloud, "lidar_roi_points", 1)
        self.lidar_roi_publisher = self.create_publisher(TrajectoryPath, "lidar_roi", 1)
        self.heading_publisher = self.create_publisher(Float32, "imu_heading_angle", 10)
        self.debug1_publisher = self.create_publisher(Float32, "debug1", 10)
        self.debug2_publisher = self.create_publisher(Float32, "debug2", 10)
        self.mpc_debug_publisher = self.create_publisher(String, "mpc_debug", 10)
        self.identification_publisher = self.create_publisher(String, "identification_sample", 10)
        # Ported from main 2026-10-15 (f972f9e): the accepted-reference green
        # line, shared by the MVP/PID path and the MPC path below.
        self.declare_parameter("visualization.enabled", True, ParameterDescriptor(read_only=True))
        self.trajectory_enabled = self.get_parameter("visualization.enabled").value
        self.trajectory_publisher = self.create_publisher(TrajectoryPath, "policy_trajectory", 1)
        self._trajectory_reference = None
        self._trajectory_last_publish_s = -math.inf
        # New 2026-10-15: the MPC's own predicted horizon, separate from the
        # accepted reference above so Foxglove can show both at once.
        self.mpc_prediction_publisher = self.create_publisher(TrajectoryPath, "mpc_prediction", 1)
        self._publish_trajectory(self.get_clock().now().nanoseconds, force=True)
        self._publish_mpc_prediction(self.get_clock().now().nanoseconds, None)
        # MVP2.1 shadow lattice state, publishers and worker (as on mvp2.1).
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
            self.shadow_path_publisher = self.create_publisher(TrajectoryPath, "shadow_trajectory", 1)
            self.shadow_prefix_publisher = self.create_publisher(TrajectoryPath, "shadow_checked_prefix", 1)
            self.shadow_reference_publisher = self.create_publisher(TrajectoryPath, "shadow_reference", 1)
            self.shadow_diagnostics_publisher = self.create_publisher(String, "shadow_planning", 1)
            self.shadow_worker = ShadowWorker(__file__)
            self.shadow_timer = self.create_timer(1/self.supervision_rate_hz, self._shadow_poll)
            self._shadow_clear()
        if self.planner_control_settings.enabled:
            self.planner_execution_publisher = self.create_publisher(String, "planner_execution", 1)

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
            if getattr(getattr(self, "lidar_obstacle_settings", None), "enabled", False):
                self.observations["lidar_scan"] = None
                self.observations["lidar_cartesian"] = None
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
        result = {"points_xyz": points, "scan_indices": indices, "frame_id": self.policy_frame_id}
        if getattr(getattr(self, "lidar_obstacle_settings", None), "enabled", False):
            result.update(ray_origin_xyz_m=offsets, ray_axes_xyz=(x_axis, y_axis))
        return result

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
                if accepted and name == "imu_angular_velocity":
                    self._observe_stationary_gyro(value[2], now, ros_now)
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
        # Submit after publishing; transport stays off the control executor.
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

        # Finite identification sequence; off keeps the zero-action starter.
        # No disk writes or waits here. The separate recorder logs observations
        # and actual published requests; this sample describes the planned step.
        identification = getattr(self, "identification", None)
        if identification is not None and identification.mode != "off":
            if is_first_policy_step:
                self.identification = IdentificationExperiment(self.identification_config)
                self.identification_run_id = uuid.uuid4().hex
            step = self.identification.step(
                policy_elapsed_s, dt, wheel_speed_in_meters_per_second)
            sample = {
                "schema": 1, "run_id": self.identification_run_id,
                "policy_monotonic_s": self._monotonic(),
                "policy_ros_ns": self.get_clock().now().nanoseconds,
                "elapsed_s": policy_elapsed_s, "dt_s": dt,
                "mode": self.identification.mode, **step,
                "speed_mps": wheel_speed_in_meters_per_second,
                "body_yaw_rate_rad_s": (None if angular_velocity_rad_per_sec is None
                                        else angular_velocity_rad_per_sec[2]),
                "sensor_age_s": sensor_age_s, "sensor_stamp_ns": sensor_stamp_ns,
                "config": self.identification_config if is_first_policy_step else None,
            }
            self.identification_publisher.publish(String(data=json.dumps(sample, allow_nan=False)))
            if step["terminal"]:
                self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, step["reason"])
            return step["drive"], step["steer"], None, float(step["stage"]), policy_elapsed_s

        # Preserve course framework inputs/actions/timing. The Word draft fills
        # internal group-record gaps only; screenshot spellings are aliases.
        # Downstream code may consume self.estimation_output["state"/"road"/
        # "obstacles"/"vehicle_params"/"vehicle_limits"]. Valid geometry is aligned
        # to state.timestamp_s; measurement_timestamp_s keeps source time. No
        # ReferenceTrajectory or target speed is generated by the estimator.
        if is_first_policy_step or not hasattr(self, "estimator"):
            # Sides come from estimation.left_colour/right_colour (startup-only).
            # estimation.colour_assignment auto overrides them once both sides are seen.
            self.estimator = EstimationPipeline(
                self.estimation_settings, self.policy_frame_id, motion_history=self.motion_history)
            self._reported_colour_assignment = None
        sample_receipts = {name: None if obs is None else obs.received_at
                           for name, obs in self.observations.items()}
        receipt_ros_ns = {name: None if obs is None else obs.received_ros_ns
                          for name, obs in self.observations.items()}
        motion_observations = observations
        if (getattr(self, "control_settings", ControlSettings()).gyro_bias_learning_enabled
                and observations.get("imu_angular_velocity") is not None):
            rates = observations["imu_angular_velocity"]
            motion_observations = {**observations, "imu_angular_velocity":
                (rates[0], rates[1], rates[2]-getattr(self, "gyro_bias_rps", 0.0))}
        self.estimation_output = self.estimator.update(
            motion_observations, sensor_age_s, sensor_stamp_ns, sample_receipts,
            self.get_clock().now().nanoseconds / 1e9, receipt_ros_ns)
        # mpc-car-run adaptation (from sim-branch 2c41240): log each auto lock once per run.
        assignment = self.estimation_output["road"].get("colour_assignment") or {}
        mapping = (assignment.get("left_colour"), assignment.get("right_colour"))
        if (assignment.get("mode") == "auto" and assignment.get("status") == "locked"
                and mapping != getattr(self, "_reported_colour_assignment", None)):
            self._reported_colour_assignment = mapping
            self.get_logger().info("Boundary colour assignment: " + str(assignment))
        # A measured vehicle.* record replaces the estimator's "unmeasured"
        # placeholders; nothing is substituted while it is not valid.
        vehicle = getattr(self, "vehicle_settings", None)
        if vehicle is not None and vehicle.valid:
            self.estimation_output["vehicle_params"], self.estimation_output["vehicle_limits"] = vehicle.records()

        if is_first_policy_step or not hasattr(self, "planner"):
            if self.planning_settings.algorithm == "lattice_v2":
                self.planner = FrenetLatticePlanner(self.planning_settings,
                    getattr(self, "lattice_settings", LatticeSettings()), self.policy_frame_id)
            else:
                # road_centerline ported from main 2026-10-15: same plan() signature
                # as CenterlinePlanner, so the call site below is unchanged.
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
        source_timeouts["obstacles"] = min(self.estimation_settings.max_source_age_s,
                                          self.sensor_timeout_s["lidar_cartesian"])
        if getattr(self, "async_planner", None) is not None:
            estimates = self._async_planning_cycle(estimates, source_timeouts)
        else:
            args = (estimates["road"], estimates["state"], estimates["obstacles"],
                    estimates["vehicle_limits"], estimates["state"]["timestamp_s"], source_timeouts)
            if self.planning_settings.algorithm == "lattice_v2":
                self.planning_output, self.planning_diagnostics = _mvp_plan(
                    self.planner, getattr(self, "control_settings", ControlSettings()), args)
            else:
                self.planning_output, self.planning_diagnostics = self.planner.plan(
                    *args, mvp=getattr(self, "control_settings", ControlSettings()).mode == "mvp")

        control = getattr(self, "control_settings", ControlSettings())
        if control.enabled:
            if is_first_policy_step or not hasattr(self, "controller"):
                self.controller = PolicyController(control, self.policy_frame_id)
                self.control_has_run = False
            # Delayed camera data may predate the reset motion history at start.
            # Bounded neutral priming is allowed only BEFORE the first valid
            # control step; any fault after tracking requires explicit resume.
            # mpc-car-run adaptation (from sim-branch 2c41240): estimation.colour_assignment
            # auto also waits here for its first two-sided observation.
            if (not self.planning_output["valid"] and not self.control_has_run
                    and estimates["state"]["valid"]
                    and (estimates["road"]["status"] in ("missing_motion_history",
                                                         "boundary_colour_assignment_pending",
                                                         "boundary_colour_assignment_ambiguous")
                         or (getattr(self, "async_planner", None) is not None
                             and self.planning_output.get("reason") in ControlReferenceManager.SOFT_REASONS))
                    and policy_elapsed_s < control.startup_grace_s):
                self.control_diagnostics = {"valid": False, "stop_requested": False,
                    "reason": ("priming_colour_assignment" if estimates["road"]["status"].startswith(
                        "boundary_colour_assignment_") else "priming_motion_history")}
                return 0.0, 0.0, None, None, None
            if (getattr(self, "async_planner", None) is not None and not self.control_has_run
                    and estimates["road"].get("degraded")
                    and policy_elapsed_s < control.startup_grace_s):
                # A predicted first result cannot establish control trust. Stay
                # neutral until an observed result arrives, rather than drive
                # once and then lock-stop on the next pending worker cycle.
                self.control_diagnostics = {"valid": False, "stop_requested": False,
                                            "reason": "awaiting_observed_reference"}
                return 0.0, 0.0, None, None, None
            selected_reference = self.planning_output
            if self._robust_control_enabled():
                try:
                    selected_reference = self._select_control_reference(estimates)
                except (ValueError, TypeError, ArithmeticError) as error:
                    raise PolicyStopRequest("Control reference: " + str(error)) from error
            if not selected_reference["valid"]:
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
                selected_reference, estimates["state"], estimates["vehicle_params"],
                self.get_clock().now().nanoseconds/1e9, dt)
            if not self.control_diagnostics["valid"] or self.control_diagnostics["stop_requested"]:
                reason = "Control: " + self.control_diagnostics["reason"]
                raise PolicyStopRequest(reason)
            self.control_has_run = True
            self._trajectory_reference = selected_reference
            self.last_control_reference_deadline_s = (
                selected_reference["timestamp_s"]+selected_reference["valid_for_s"])
            if self._robust_control_enabled():
                self._report_control_status()
            # Existing debug topics: lateral error (m), per-run distance (m).
            # The existing state string reports planning/control stop reasons.
            debug1 = self.control_diagnostics["path_error_m"]
            debug2 = self.distance_limiter.distance_m

        # Vehicle-model MPC (drive and steering) on the Planning reference, or on the
        # estimator centerline in bypass mode (see MPCController). The prediction
        # model and the vehicle record come from mpc_vehicle_params().
        # Shadow mode computes candidates but applies zeros; a rejected step raises
        # MPCStop in non-shadow mode, which the framework turns into zero actions.
        mpc_settings = getattr(self, "mpc_settings", None)
        if mpc_settings is not None and mpc_settings.enabled:
            if is_first_policy_step or getattr(self, "mpc_controller", None) is None:
                self.mpc_controller = MPCController(mpc_settings, getattr(self, "vehicle_settings", None))
                self.mpc_has_run = False
                self.mpc_held_reference = None
                self.wheel_hold = WheelHoldMonitor(mpc_settings)
                self._jump_guard_s, self._jump_ref = 0.0, None
                self._ref_source, self._blend_left_s = None, 0.0
            state = estimates["state"]
            planning_ref, held_age = self.planning_output, None
            if mpc_settings.reference_source == "planning":
                planning_ref, self.mpc_held_reference, held_age = mpc_planning_reference(
                    self.planning_output, getattr(self, "mpc_held_reference", None),
                    self.motion_history, state["timestamp_s"])
            # MVP2.1 lattice (planner_control.enabled): its aligned, cropped path
            # replaces this tick's road_centerline reference when it passes the
            # execution checks; otherwise the baseline above is used unchanged.
            planner_control = getattr(self, "planner_control_settings", None)
            planner_requested = planner_control is not None and planner_control.enabled
            planner_traj = self._select_mpc_planner_trajectory(estimates) if planner_requested else None
            shadow_cfg = getattr(self, "shadow_settings", None)
            if (planner_requested and shadow_cfg is not None and shadow_cfg.obstacle_avoidance_enabled
                    and shadow_cfg.obstacle_check_enabled):
                obstacle_input = _shadow_obstacle_input(estimates,
                    getattr(self, "lidar_obstacle_settings", None), self.policy_frame_id)
                has_obstacles = (obstacle_input.get("available") is True
                    and obstacle_input.get("time_aligned") is True
                    and obstacle_input.get("timestamp_s") == state.get("timestamp_s")
                    and bool(obstacle_input.get("points_xyz_m")))
                active = self.planner_execution.active or {}
                expired_hold = self.planner_execution.diagnostics.get("reason") == "road_dropout_timeout"
                if expired_hold or has_obstacles and (planner_traj is None or not active.get("obstacle_checked")):
                    # Do not drive the unchecked centerline while waiting/replanning.
                    self.planner_execution.diagnostics.update(mode="waiting", fallback=False,
                        reason="road_dropout_timeout" if expired_hold else "waiting_for_obstacle_checked_path")
                    self.mpc_controller = None  # discard held inputs while output is zero
                    self.mpc_held_reference = self._trajectory_reference = self._mpc_trajectory = None
                    return 0.0, 0.0, None, None, None
            # Same bounded priming as the MVP: delayed camera data may predate the
            # motion history reset at start. Only BEFORE the first accepted MPC step.
            # With async planning, also wait out the worker's first result and a
            # predicted (degraded) first road, as the MVP does.
            grace = getattr(self, "control_settings", ControlSettings()).startup_grace_s
            priming = not mpc_settings.shadow and not getattr(self, "mpc_has_run", False) and policy_elapsed_s < grace
            async_planning = getattr(self, "async_planner", None) is not None
            # mpc-car-run adaptation (from sim-branch 2c41240): and for the colour lock
            # of estimation.colour_assignment auto, within the same grace.
            if priming and planner_traj is None and (
                    (not planning_ref["valid"] and state["valid"]
                     and (estimates["road"]["status"] in ("missing_motion_history",
                                                          "boundary_colour_assignment_pending",
                                                          "boundary_colour_assignment_ambiguous")
                          or async_planning
                          and planning_ref.get("reason") in ControlReferenceManager.SOFT_REASONS))
                    or async_planning and estimates["road"].get("degraded")):
                return 0.0, 0.0, None, None, None
            if planner_traj is not None:
                traj = planner_traj
            elif mpc_settings.reference_source == "estimation_centerline":
                traj = reference_trajectory_from_road(estimates["road"], state, mpc_settings)
            else:
                traj = reference_trajectory_from_planning(planning_ref, mpc_settings.reference_spacing_m,
                                                          mpc_settings.ordered_lead_in_fit_m,
                                                          mpc_settings.max_backward_extension_m)
            # 2026-10-22: hook-only reference curvature limit (lattice and centerline alike).
            # "off" (every deployed profile) never calls it, so mpc_debug stays as it was.
            curvature_debug = None
            if mpc_settings.reference_curvature_limit_mode != "off":
                vehicle = mpc_vehicle_params(mpc_settings, getattr(self, "vehicle_settings", None))
                shadow_cfg = getattr(self, "shadow_settings", None) or ShadowSettings()
                traj, curvature_debug = limit_reference_curvature(
                    traj, estimates["road"], mpc_settings, vehicle,
                    (vehicle.body_width_m or shadow_cfg.body_width_m)/2,
                    self.planning_settings.preview_m+EXECUTION_TAIL_M)
            speed = state["speed_mps"] if state.get("speed_valid") else None
            # 2026-10-19: a held wheel reading of a stalled car is passed as 0 (the start
            # drive is reapplied); any held reading pauses the soft-overspeed timer and bias.
            rates = motion_observations.get("imu_angular_velocity")
            if getattr(self, "wheel_hold", None) is None:
                self.wheel_hold, self._jump_guard_s, self._jump_ref = WheelHoldMonitor(mpc_settings), 0.0, None
            hold = self.wheel_hold.update(
                observations.get("wheel_speed"), rates[2] if rates is not None else None,
                self.mpc_controller.applied_steer, state["timestamp_s"])
            raw_speed = speed
            if hold["stalled"] and speed is not None:
                speed = 0.0
            guard = getattr(self, "_mpc_jump_guard", None)   # absent on partial offline stand-ins
            traj, gate = guard(traj, speed, dt, guard=planner_traj is None) if guard else (traj, None)
            # Reference-source blend (mpc.switch_blend_s, 0 = off): after the reference switches
            # between the lattice and the centerline, limit the steering change per tick
            # (10-19: centerline -> lattice moved it by the QP's full step, 0.27-0.39).
            source = "planner" if planner_traj is not None else "baseline"
            previous = getattr(self, "_ref_source", None)
            if previous is not None and previous != source:
                self._blend_left_s = mpc_settings.switch_blend_s
            self._ref_source = source
            slew = {}
            if getattr(self, "_blend_left_s", 0.0) > 1e-9 and finite_number(dt) and dt > 0:
                slew = {"steer_slew": mpc_settings.switch_steer_rate_per_s*dt}
                self._blend_left_s -= dt
            try:
                mpc_result = self.mpc_controller.step(traj, speed, dt, state["timestamp_s"], is_first_policy_step,
                                                      speed_fresh=hold["fresh"],
                                                      speed_raw=observations.get("wheel_speed"), **slew)
            finally:
                self.mpc_controller.last_debug.update(reference_held=held_age is not None,
                                                      reference_held_age_s=held_age,
                                                      wheel_held_s=hold["held_s"], wheel_fresh=hold["fresh"],
                                                      speed_override="stall" if hold["stalled"] else None,
                                                      speed_raw_mps=raw_speed, reference_gate=gate,
                                                      reference_gate_s=self._jump_guard_s)
                if curvature_debug is not None:
                    self.mpc_controller.last_debug.update(reference_curvature=curvature_debug)
                if planner_requested:
                    self.mpc_controller.last_debug.update(
                        reference_selection=self.planner_execution.diagnostics.get("mode"),
                        reference_selection_reason=self.planner_execution.diagnostics.get("reason"))
                assignment = estimates["road"].get("colour_assignment") or {}
                if assignment.get("mode") == "auto":     # 2026-10-21; fixed mode leaves mpc_debug as it was
                    self.mpc_controller.last_debug.update(colour_assignment={
                        key: assignment.get(key) for key in ("status", "left_colour", "right_colour")})
                publisher = getattr(self, "mpc_debug_publisher", None)
                if publisher is not None:
                    publisher.publish(String(data=mpc_debug_json(self.mpc_controller.last_debug)))
            drive_action, steering_action = mpc_result["drive"], mpc_result["steer"]
            debug = mpc_result["debug"]
            if debug["branch"] in ("solved", "shadow", "zero_target"):
                self.mpc_has_run = True
            debug1, debug2 = debug["e_y_m"], debug["candidate_delta_rad"]
            if planner_requested:
                self._mpc_trajectory = traj
            # Ported 2026-10-15: reuse main's green-line publisher for the MPC's
            # own adapted reference (traj), and publish the predicted horizon
            # on a separate topic. Both are visualization only.
            if traj.get("valid"):
                self._trajectory_reference = {
                    "valid": True, "frame_id": self.policy_frame_id,
                    "timestamp_s": traj["timestamp_s"], "valid_for_s": traj["valid_for_s"],
                    "path": {"type": "CARTESIAN_CENTERLINE",
                             "samples_xy": [[p["x_m"], p["y_m"]] for p in traj["points"]]}}
            else:
                self._trajectory_reference = None
            if hasattr(self, "mpc_prediction_publisher"):
                self._publish_mpc_prediction(self.get_clock().now().nanoseconds,
                                             debug["pred_xy"] if debug["branch"] in
                                             ("solved", "shadow") else None)

        # Below are examples for other policies, not part of the active code above.
        #
        # Code for a "working" policy is NOT provided because it tend to causing
        # anchoring and minimal changes from the provided code.
        #
        # The following comments are example for how the observation variables
        # available in this function can be used to implement various aspects
        # of a policy.
        #
        # Example: a state variable for a speed controller (uncomment to use):
        # if is_first_policy_step:
        #     self.speed_error_integral = 0.0
        # if wheel_speed_in_meters_per_second is not None and dt > 0.0:
        #     speed_error = 0.5 - wheel_speed_in_meters_per_second  # target: m/s
        #     self.speed_error_integral += speed_error * dt
        #     drive_action = self.speed_kp * speed_error  # declare speed_kp above
        #     debug1 = wheel_speed_in_meters_per_second
        #     debug2 = drive_action
        #
        # Example: examine yellow cones without assuming a cone always exists:
        # yellow_indices = [i for i in range(num_cones)
        #                   if cone_colour[i] == ConeDetection.COLOR_YELLOW]
        #
        # Example: retain only usable lidar returns:
        # if lidar_scan_available:
        #     hits = [(lidar_scan['angle_min'] + i * lidar_scan['angle_increment'], distance)
        #             for i, distance in enumerate(lidar_ranges)
        #             if math.isfinite(distance)
        #             and lidar_scan['range_min'] <= distance <= lidar_scan['range_max']]
        #
        # Example: inspect body-frame points and their original ranges:
        # if lidar_cartesian_available:
        #     for (x, y, z), ray_index in zip(lidar_points_xyz, lidar_scan_indices):
        #         distance_from_lidar = lidar_ranges[ray_index]
        #         # Your algorithm can use x/y/z, distance_from_lidar, or both.

        # =====================================
        # END OF: INSERT POLICY CODE ABOVE HERE
        # =====================================
        return drive_action, steering_action, camera_pan_action, debug1, debug2

    def _observe_stationary_gyro(self, rate, now, ros_now):
        """Learn only during publishing-zero with fresh near-zero wheel speed.

        Operator must leave the body stationary during the neutral interval.
        A fixed per-run bias is subtracted only downstream, preserving raw
        observations, source identity and independent IMU-field freshness.
        """
        cfg = self.control_settings
        if not cfg.gyro_bias_learning_enabled:
            return
        samples = self.gyro_bias_samples
        wheel = self.observations.get("wheel_speed")
        if (self.fsm_state != FSM_STATE_PUBLISHING_ZERO_ACTIONS
                or not self._fresh("wheel_speed", now, ros_now) or wheel.value > 0.005
                or abs(rate) > cfg.gyro_bias_max_rps):
            samples.clear()
            return
        if samples and now-samples[-1][0] < 0.05:
            return
        samples.append((now, rate))
        while len(samples) > 32 or len(samples) > 2 and now-samples[1][0] >= cfg.gyro_bias_window_s:
            samples.pop(0)
        if len(samples) >= 10 and now-samples[0][0]+1e-9 >= cfg.gyro_bias_window_s:
            mean = sum(v for _, v in samples)/len(samples)
            if sum((v-mean)**2 for _, v in samples)/len(samples) <= 0.003**2:
                self.gyro_bias_rps = mean

    def _async_planning_cycle(self, estimates, source_timeouts):
        now = self._monotonic()
        stamp = estimates["state"]["timestamp_s"]
        worker = self.async_planner
        result = worker.poll()
        pending_age = None if worker.pending_at is None else now-worker.pending_at
        failed = worker.process.poll() is not None
        if failed or pending_age is not None and pending_age >= self.lattice_settings.worker_timeout_s:
            self.last_async_fault = "planning_worker_unavailable" if failed else "planning_worker_timeout"
            if now-getattr(self, "last_worker_restart_s", -math.inf) >= 0.5:
                worker.close()
                self.async_planner = worker = AsyncLatticePlanner()
                self.last_worker_restart_s = now
        accepted = False
        selected_estimates = estimates
        if result is not None and result.get("generation") == self.planning_generation:
            if result.get("error"):
                self.last_async_fault = "planning_worker_unavailable"
            else:
                ref = result["reference"]
                self.planning_diagnostics = result["diagnostics"]
                self.last_planner_elapsed_s = result["diagnostics"].get("elapsed_s")
                if ref.get("valid"):
                    try:
                        transported = transport_control_reference(ref, self.motion_history, stamp)
                        road = dict(result["road"])
                        road["timestamp_s"] = stamp
                        self.planning_output = transported
                        selected_estimates = {**estimates, "road": road}
                        self.last_async_fault = None
                        accepted = True
                    except (ValueError, TypeError, ArithmeticError) as error:
                        self.last_async_fault = "source_deadline_during_planning"
                        self.planning_diagnostics["discarded_result"] = str(error)
                else:
                    self.last_async_fault = ref.get("reason")
        if not accepted:
            reason = self.last_async_fault or "planning_pending"
            self.planning_output = {"valid": False, "reason": reason, "timestamp_s": stamp,
                                    "valid_for_s": 0.0, "stop_requested": True}
            self.planning_diagnostics = {"async": True, "reason": reason, "pending_age_s": pending_age}
        if (worker.process.poll() is None and worker.ready and worker.pending_at is None
                and now-self.last_planning_submit_s >= 1.0/self.lattice_settings.update_rate_hz):
            job = {"generation": self.planning_generation, "frame_id": self.policy_frame_id,
                   "planning": vars(self.planning_settings), "lattice": vars(self.lattice_settings),
                   "control": vars(self.control_settings),
                   "args": [estimates["road"], estimates["state"], estimates["obstacles"],
                            estimates["vehicle_limits"], stamp, source_timeouts]}
            worker.submit(deepcopy(job), now)
            self.last_planning_submit_s = now
        return selected_estimates

    # ---- MVP2.1 lattice: MPC consumer and shadow side channel ------------------
    # Shadow methods below are verbatim from origin/mvp2.1 3652e9b except where
    # marked "mpc-car-run adaptation". The consumer methods are new: mvp2.1 feeds
    # the MVP PID, this branch feeds the MPC.
    def _planner_preview_m(self):
        """Path length the selected lattice must still cover ahead of the car.

        mpc-car-run adaptation: mvp2.1 sizes this by the MVP PID's preview point.
        The MPC tracks its whole horizon, so take the larger of planning.preview_m
        and the horizon travel at the execution cap (cruise speed in shadow),
        including the actuator delay replayed before the first prediction step.
        """
        mpc = self.mpc_settings
        vehicle = mpc_vehicle_params(mpc, getattr(self, "vehicle_settings", None))
        speed = mpc.v_exec_max_mps if mpc.v_exec_max_mps > 0 else self.planning_settings.cruise_speed_mps
        delay = max(vehicle.drive_delay_s, vehicle.steering_delay_s)
        return max(self.planning_settings.preview_m, speed*(mpc.horizon_n*mpc.dt_pred_s+delay))

    def _select_planner_reference(self, estimates):
        """Return a checked reference or None for the caller's fallback/wait."""
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
            # mpc-car-run adaptation: see PlannerControlSettings.require_valid_baseline.
            manager = self.planner_execution
            baseline_valid = self.planning_output.get("valid") is True
            reused_road, dropout_age = False, 0.0
            if (self.planner_control_settings.require_valid_baseline
                    and not baseline_valid):
                active = manager.active or {}
                limit = self.mpc_settings.reference_dropout_tolerance_s
                can_bridge = (self.mpc_settings.geometry_glitch_tolerable and limit > 0
                    and self.shadow_settings.obstacle_check_enabled and active.get("obstacle_checked")
                    and _shadow_soft_road_failure(self.planning_output.get("reason"))
                    and manager.last_good_road is not None
                    and manager.last_good_road_epoch == self.shadow_odometry_epoch == active.get("epoch"))
                if can_bridge:
                    if manager.road_dropout_started_s is None:
                        manager.road_dropout_started_s = self._monotonic()
                    dropout_age = self._monotonic()-manager.road_dropout_started_s
                    if dropout_age >= limit-1e-9:
                        manager.diagnostics = {"mode": "waiting", "fallback": False,
                            "reason": "road_dropout_timeout", "road_dropout_elapsed_s": dropout_age,
                            "road_dropout_limit_s": limit, "requested_source": "planner"}
                        return None
                    # Transport the last trustworthy road support, never mark it
                    # as a new measurement. select still checks motion/coverage/body.
                    road = deepcopy(manager.last_good_road)
                    for side in ("left", "right"):
                        key = side+"_boundary_xy"
                        road[key] = _shadow_pose_points(road.get(key) or [],
                            manager.last_good_road_pose, self.shadow_pose)
                    road.update(timestamp_s=state["timestamp_s"], time_aligned=True)
                    reused_road = True
                else:
                    manager.diagnostics = {"mode": "baseline", "fallback": True,
                        "reason": "baseline_invalid: "+str(self.planning_output.get("reason")),
                        "requested_source": "planner"}
                    return None
            elif baseline_valid:
                manager.road_dropout_started_s = None
            period = 1/self.policy_update_rate_hz
            # mpc-car-run adaptation: MPC preview distance and dt limit, not the PID's.
            reference = manager.select(road, state, self.shadow_pose, self.shadow_odometry_epoch,
                self.shadow_settings, self.planner_preview_m, self.planning_settings.cruise_speed_mps,
                period, self.mpc_settings.dt_max_s+period)
            if reference is not None and reused_road:
                manager.diagnostics.update(reason="planner_tracking_road_dropout",
                    road_dropout_elapsed_s=dropout_age, road_dropout_limit_s=limit,
                    road_support="motion_transported_cached",
                    road_measurement_stamp_s=road.get("measurement_timestamp_s"))
            elif reference is not None and baseline_valid:
                keys = ("left_boundary_xy", "right_boundary_xy", "boundary_source", "frame_id",
                        "timestamp_s", "measurement_timestamp_s", "time_aligned")
                manager.last_good_road = {key: deepcopy(road.get(key)) for key in keys}
                manager.last_good_road_pose = list(self.shadow_pose)
                manager.last_good_road_epoch = self.shadow_odometry_epoch
            elif manager.diagnostics.get("reason") == "known_boundary_collision":
                manager.last_good_road = None  # a confirmed conflict cannot become a soft gap
                manager.road_dropout_started_s = None
            return reference
        except Exception as error:
            self.planner_execution.diagnostics = {"mode": "baseline", "fallback": True,
                "requested_source": "planner", "reason": "execution_exception: "+str(error)}
            return None

    def _select_mpc_planner_trajectory(self, estimates):
        """The lattice as an MPC trajectory, or None for the caller's fallback/wait.

        Checked with the pure adapter and the horizon projection BEFORE
        MPCController.step: step advances its clock, command history, bias and
        held plan, and runs once per tick. So unlike mvp2.1's PID (which is
        restored and re-run on the baseline), a path the solver itself then
        rejects is an ordinary tolerable MPC dropout, not a same-tick retry.
        """
        cfg = self.mpc_settings
        reference = self._select_planner_reference(estimates)
        if reference is None:
            return None
        try:
            traj = reference_trajectory_from_planning(reference, cfg.reference_spacing_m,
                                                      cfg.ordered_lead_in_fit_m, cfg.max_backward_extension_m)
            reason = traj.get("reason")
            if traj.get("valid") is True and not traj.get("stop_required"):
                speed = estimates["state"].get("speed_mps")
                speed = speed if finite_number(speed) and speed > 0 else 0.0
                cap = cfg.v_exec_max_mps if cfg.v_exec_max_mps > 0 else self.planning_settings.cruise_speed_mps
                # No forward extension here (2026-10-18): a lattice path short of the horizon
                # falls back to the baseline, which the solver may still extend.
                info, reason = mpc_reference_errors(traj, cfg.horizon_n, cfg.dt_pred_s, max(speed, cap),
                                                    cfg.max_backward_extension_m)
                if info is not None:
                    self.planner_execution.diagnostics.update(mpc_e_y_m=info["e_y_m"],
                        mpc_e_psi_rad=info["e_psi_rad"], mpc_path_remaining_m=info["path_remaining_m"])
                    return traj
        except (ValueError, TypeError, ArithmeticError, KeyError) as error:
            reason = "exception: "+str(error)
        self.planner_execution.diagnostics.update(mode="baseline", fallback=True,
                                                  reason="mpc_reference_rejected: "+str(reason))
        return None

    def _mpc_jump_guard(self, traj, speed, dt, guard=True):
        """Hold the last solved inputs over a jumping road_centerline reference.

        road_centerline planning only. 10-19 car runs: on falling back from the lattice
        (and inside centerline stretches) the centerline estimate jumped for a tick or two
        (e_psi up to 1.2 rad) and the steering swung from -1 to +1. Compared with the
        previous solved step, a jump makes
        this tick a tolerable dropout ("baseline_jump": the MPC's plan_hold runs) for at
        most mpc.switch_hold_s in a row; after that the reference is used as it is.
        Lattice ticks (guard=False) only refresh the previous solved errors. Returns
        (trajectory for the MPC, gate label or None).
        """
        cfg, mpc = self.mpc_settings, self.mpc_controller
        last = mpc.last_debug or {}
        if last.get("branch") == "solved" and finite_number(last.get("e_y_m")) and finite_number(last.get("e_psi_rad")):
            self._jump_ref = (last["e_y_m"], last["e_psi_rad"])
        ref = getattr(self, "_jump_ref", None)
        if (not guard or cfg.switch_hold_s <= 0 or cfg.shadow or mpc.plan is None
                or self.planning_settings.algorithm != "road_centerline"
                or not isinstance(traj, dict) or traj.get("valid") is not True or traj.get("stop_required")):
            self._jump_guard_s = 0.0
            return traj, None
        cap = cfg.v_exec_max_mps if cfg.v_exec_max_mps > 0 else self.planning_settings.cruise_speed_mps
        try:
            info, _ = mpc_reference_errors(traj, cfg.horizon_n, cfg.dt_pred_s, max(speed or 0.0, cap),
                                           cfg.max_backward_extension_m, cfg.max_forward_extension_m)
        except (ValueError, TypeError, ArithmeticError, KeyError):
            info = None
        if info is None:          # the MPC rejects such a reference itself
            self._jump_guard_s = 0.0
            return traj, None
        e_y, e_psi = info["e_y_m"], info["e_psi_rad"]
        jump = abs(e_psi) > cfg.switch_max_epsi_rad or ref is not None and (
            abs(e_psi-ref[1]) > cfg.switch_max_epsi_rad or abs(e_y-ref[0]) > cfg.switch_max_dey_m)
        if not jump:
            self._jump_guard_s = 0.0
            return traj, None
        if self._jump_guard_s >= cfg.switch_hold_s - 1e-9:
            return traj, "baseline_jump_expired"
        self._jump_guard_s += dt if finite_number(dt) and dt > 0 else 0.0
        return {**traj, "valid": False, "reason": "baseline_jump"}, "baseline_jump"

    def _publish_planner_execution(self):
        """Record every executed control tick; transport faults cannot stop it."""
        try:
            info = dict(self.planner_execution.diagnostics)
            # mpc-car-run adaptation: report the MPC's step instead of the PID's.
            traj = getattr(self, "_mpc_trajectory", None) or {}
            mpc = getattr(getattr(self, "mpc_controller", None), "last_debug", None) or {}
            info.update(timestamp_s=traj.get("timestamp_s"), generation=self.shadow_generation,
                path_id=traj.get("path_id"),
                baseline_valid=bool(self.planning_output and self.planning_output.get("valid")),
                baseline_reason=(self.planning_output or {}).get("reason"),
                speed_mps=(self.estimation_output or {}).get("state", {}).get("speed_mps"),
                mpc_branch=mpc.get("branch"), mpc_reject_reason=mpc.get("reject_reason"),
                e_y_m=mpc.get("e_y_m"), e_psi_rad=mpc.get("e_psi_rad"),
                candidate_steer_action=mpc.get("candidate_steer_action"),
                applied_steer_action=mpc.get("applied_steer_action"),
                applied_drive=mpc.get("applied_drive"),
                reference_valid_for_s=traj.get("valid_for_s"))
            if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
                info.update(mode="idle", fallback=False, reason=self.state_reason)
            self.planner_execution_publisher.publish(String(data=json.dumps(info, allow_nan=False)))
            mode = (info.get("mode"), info.get("reason"))
            if mode != self._last_execution_mode:
                self.get_logger().info("Reference selection: "+json.dumps(info, separators=(",", ":")))
                self._last_execution_mode = mode
        except Exception as error:
            self._warn("planner_execution_diagnostics", "Execution diagnostics unavailable: "+str(error))

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
            "submitted_monotonic_s": now, "preview_m": self.planner_preview_m,  # mpc-car-run adaptation
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
        if self.shadow_settings.obstacle_avoidance_enabled:
            job["obstacles"] = _shadow_obstacle_input(estimates,
                getattr(self, "lidar_obstacle_settings", None), self.policy_frame_id)
        self.shadow_worker.submit(job)
        self.shadow_last_submit_s = now

    def _shadow_publish_path(self, publisher, points, stamp_s):
        message = TrajectoryPath()  # mpc-car-run adaptation: nav_msgs Path alias
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
            if (self.shadow_settings.obstacle_check_enabled and result.get("stop_required")
                    and getattr(getattr(self, "planner_control_settings", None), "enabled", False)):
                try:
                    self.shadow_diagnostics_publisher.publish(String(data=json.dumps(result)))
                finally:
                    self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS,
                        "LiDAR planning: no passable path; explicit resume required")
                return
            if getattr(getattr(self, "planner_control_settings", None), "enabled", False):
                self.planner_execution.accept(result, preserve_no_road=(
                    self.shadow_settings.obstacle_check_enabled
                    and self.mpc_settings.geometry_glitch_tolerable
                    and self.mpc_settings.reference_dropout_tolerance_s > 0))
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
                "speed_mps", "hold_time_s", "check_length_m", "check_length_mode", "score_length_m", "elapsed_s",
                "collision_coverage", "score_coverage",
                "late_for_requested_period", "timing_s")}
            summary.update(selected_id=selected.get("id") if selected else None,
                selected_family=selected.get("family") if selected else None,
                result_age_s=self.get_clock().now().nanoseconds/1e9-stamp,
                **transport)
            if result.get("obstacle_avoidance"):
                summary["obstacle_avoidance"] = result["obstacle_avoidance"]
            if result.get("fusion"):
                summary["fusion"] = {k: result["fusion"].get(k) for k in
                    ("status", "reason", "degraded", "raw_fallback_used", "selected_source",
                     "fused_status", "raw_status", "history_frames", "prior_used", "removed_guidance_loops", "reference_crop", "collection")}
            if result.get("error"):
                summary["error"] = result["error"]
            self.get_logger().info("Shadow planning: "+json.dumps(summary, separators=(",", ":")))
        except Exception as error:
            self._warn("shadow_publish", f"Shadow diagnostics unavailable: {error}")

    # ---- Bounded reference recovery and control execution ---------------------
    def _robust_control_enabled(self):
        cfg = self.control_settings
        return cfg.enabled and cfg.mode == "mvp" and cfg.robustness_enabled

    def _mpc_uses_planning(self):
        mpc = getattr(self, "mpc_settings", None)
        return mpc is not None and mpc.enabled and mpc.reference_source == "planning"

    def _control_health_problem(self, monotonic_now, ros_now_ns):
        lidar_problem = self._lidar_stop_problem(monotonic_now, ros_now_ns)
        if lidar_problem:
            return lidar_problem
        problem = self.health_problem(monotonic_now, ros_now_ns)
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
        self.control_diagnostics["gyro_bias_rps"] = getattr(self, "gyro_bias_rps", 0.0)
        self.control_diagnostics["last_planner_elapsed_s"] = getattr(self, "last_planner_elapsed_s", None)
        if manager.reference is None:
            return
        self.state_reason = "Control: " + str({**manager.diagnostics,
            "gyro_bias_rps": self.control_diagnostics["gyro_bias_rps"],
            "last_planner_elapsed_s": self.control_diagnostics["last_planner_elapsed_s"]})
        if manager.mode != getattr(self, "last_control_tracking_mode", None):
            self.last_control_tracking_mode = manager.mode
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
            problem = self._lidar_stop_problem(now, ros_now) or self.health_problem(now, ros_now)
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
        if hasattr(self, "gyro_bias_samples"):
            self.gyro_bias_samples.clear()
        self.planning_generation = getattr(self, "planning_generation", 0)+1
        self.last_planning_submit_s = -math.inf
        self.last_async_fault = None
        self.fsm_state = state
        self.state_reason = reason
        # An internal consumer must not mistake the previous run's estimate
        # for an active result after a stop, source expiry, or explicit restart.
        self.estimation_output = None
        self.motion_history.clear()
        self.planning_output = None
        self.planning_diagnostics = None
        self.mpc_held_reference = None
        self.controller.clear()
        if hasattr(self, "control_reference_manager"):
            self.control_reference_manager.clear()
        self.last_control_tracking_mode = None
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False
        # MVP2.1 lattice: fence every result and odometry epoch of the previous run.
        if hasattr(self, "planner_execution"):
            self.planner_execution.clear()
            self._last_execution_mode = None
        self._mpc_trajectory = None
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
            if self._budgets_active():
                self.debug2_publisher.publish(Float32(data=float(self.distance_limiter.distance_m)))
            self.publish_zero_actions()
        self.get_logger().info(f"{STATE_NAMES[state]}: {reason}")
        self.publish_state()
        # Ported from main 2026-10-15: clear both Foxglove lines on every stop,
        # restart or explicit resume, so a stale line never outlives its run.
        self._trajectory_reference = None
        if hasattr(self, "trajectory_publisher"):
            self._publish_trajectory(self.get_clock().now().nanoseconds, force=True)
            self._trajectory_last_publish_s = -math.inf
        if hasattr(self, "mpc_prediction_publisher"):
            self._publish_mpc_prediction(self.get_clock().now().nanoseconds, None)
        # Baseline state/zero publications have priority over shadow diagnostics.
        if getattr(self, "shadow_worker", None) is not None:
            self._shadow_clear()
        if hasattr(self, "planner_execution_publisher"):
            self._publish_planner_execution()

    def _budgets_active(self):
        """Distance/time budgets guard every controller that can issue drive."""
        mpc = getattr(self, "mpc_settings", None)
        return self.control_settings.enabled or (mpc is not None and mpc.enabled and not mpc.shadow)

    def _lidar_stop_problem(self, monotonic_now, ros_now_ns):
        """检测/显示与停车分开 / Detect-only mode never gates the planner or MPC."""
        cfg = getattr(self, "lidar_obstacle_settings", None)
        if cfg is None or not cfg.enabled:
            return None
        raw, cart = (self.observations.get(name) for name in ("lidar_scan", "lidar_cartesian"))
        fresh = (self._fresh("lidar_scan", monotonic_now, ros_now_ns)
                 and self._fresh("lidar_cartesian", monotonic_now, ros_now_ns)
                 and raw.stamp_ns == cart.stamp_ns)
        if not fresh:
            self._lidar_obstacle_stamp_ns = None
            self.lidar_obstacle_output = dict(status="UNKNOWN", available=False,
                reason="missing_stale_or_unmatched_lidar", frame_id=self.policy_frame_id,
                measurement_timestamp_s=None if raw is None else raw.stamp_ns/1e9,
                points_xyz_m=[], scan_indices=[], clusters=[], candidate_clusters=[])
        elif raw.stamp_ns != self._lidar_obstacle_stamp_ns:
            try:
                self.lidar_obstacle_output = detect_lidar_obstacles(raw.value, cart.value, cfg, raw.stamp_ns)
            except (ValueError, TypeError, KeyError, ArithmeticError) as error:
                self.lidar_obstacle_output = dict(status="UNKNOWN", available=False,
                    reason="detector_error: "+str(error), frame_id=self.policy_frame_id,
                    measurement_timestamp_s=raw.stamp_ns/1e9, points_xyz_m=[], scan_indices=[],
                    clusters=[], candidate_clusters=[])
            self._lidar_obstacle_stamp_ns = raw.stamp_ns
        result = self.lidar_obstacle_output
        result.update(roi_xy_m=lidar_roi_bounds(cfg), stop_enabled=cfg.stop_enabled,
            mode="stop" if cfg.stop_enabled else "detect_only",
            stop_requested=cfg.stop_enabled and result["status"] != "CLEAR",
            source_age_s=None if raw is None else self._age(raw, monotonic_now, ros_now_ns))
        key = (result["measurement_timestamp_s"], result["status"], result["reason"])
        if key != self._lidar_obstacle_publication_key:
            # Telemetry failure must not change control admission or fabricate clear data.
            try:
                self.lidar_obstacle_publisher.publish(String(data=json.dumps(result, allow_nan=False)))
                stamp = Time(nanoseconds=raw.stamp_ns if fresh else ros_now_ns).to_msg()
                cloud = PointCloud()
                cloud.header.frame_id, cloud.header.stamp = self.policy_frame_id, stamp
                cloud.points = [Point32(x=float(x), y=float(y), z=float(z))
                                for x,y,z in result["points_xyz_m"]]
                self.lidar_points_publisher.publish(cloud)
                rectangle = TrajectoryPath(header=cloud.header)
                (lo, hi), (right, left) = result["roi_xy_m"]
                for x,y in ((lo,right),(hi,right),(hi,left),(lo,left),(lo,right)):
                    pose = PoseStamped(header=rectangle.header)
                    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                    pose.pose.orientation.w = 1.0
                    rectangle.poses.append(pose)
                self.lidar_roi_publisher.publish(rectangle)
                self._lidar_obstacle_publication_key = key
            except Exception as error:
                self._warn("lidar_status", "LiDAR visualization unavailable: "+str(error))
        if result["stop_requested"]:
            return "Lidar stop: "+result["reason"]+"; explicit resume required"
        return None

    def _publish_trajectory(self, ros_now_ns, force=False):
        """Display the accepted reference (PID or MPC) at 10 Hz; stops clear it.

        Ported from main 2026-10-15 (f972f9e/a38a4d7), unmodified: it already
        handles CARTESIAN_CENTERLINE/ORDERED_CENTERLINE sample lists directly,
        which is also the shape the MPC block above wraps its own traj into.
        """
        if not self.trajectory_enabled:
            return
        now = self._monotonic()
        if not force and now-self._trajectory_last_publish_s < 0.1:
            return
        message = TrajectoryPath()
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

    def _publish_mpc_prediction(self, ros_now_ns, pred_xy):
        """New 2026-10-15: the MPC's own predicted horizon (base_link x,y at
        the reference time), on its own topic so Foxglove can show it next to
        the accepted-reference line. Visualization only: never fed back into
        control, and published every MPC step rather than throttled to 10 Hz
        because it changes every step. None/empty clears the line.
        """
        message = TrajectoryPath()
        message.header.frame_id = self.policy_frame_id
        message.header.stamp = Time(nanoseconds=ros_now_ns).to_msg()
        try:
            if (pred_xy and self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION
                    and 1 <= len(pred_xy) <= 201
                    and all(len(p) == 2 and all(finite_number(v) for v in p) for p in pred_xy)):
                for x, y in pred_xy:
                    pose = PoseStamped(header=message.header)
                    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                    pose.pose.orientation.w = 1.0
                    message.poses.append(pose)
        except Exception as error:
            message.poses = []
            self._warn("mpc_prediction_display", f"MPC prediction display unavailable: {error}")
        try:
            self.mpc_prediction_publisher.publish(message)
        except Exception as error:
            self._warn("mpc_prediction_display", f"MPC prediction publication unavailable: {error}")

    def _control_problem(self, monotonic_now, ros_now_ns):
        """Distance/time budgets and reference expiry need no sensor callback."""
        if not self._budgets_active():
            return None
        wheel = self.observations.get("wheel_speed")
        if wheel is None or not self._fresh("wheel_speed", monotonic_now, ros_now_ns):
            return "Wheel speed unavailable for distance limit"
        problem = self.distance_limiter.advance(wheel.value, monotonic_now)
        if problem:
            return problem
        if (self.policy_started_at is not None and self.control_settings.max_run_time_s > 0 and
                monotonic_now-self.policy_started_at >= self.control_settings.max_run_time_s):
            return "Run time limit reached; explicit resume required"
        if not self.control_settings.enabled:
            return None     # the MPC checks its own reference age and startup
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
        lidar_problem = self._lidar_stop_problem(now, ros_now)
        if self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION:
            problem = lidar_problem or self._control_health_problem(now, ros_now) or self._control_problem(now, ros_now)
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
        if getattr(self, "async_planner", None) is not None:
            self.async_planner.close()
            self.async_planner = None
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PolicyNode()
        # Move everything allocated at startup (modules, parameters, settings)
        # to the permanent generation. A full collection of that heap took
        # 80-150 ms in the 2026-10-12 test runs, longer than the 50 ms control
        # step; later collections only scan objects created while running.
        gc.collect()
        gc.freeze()
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
    if sys.argv[1:] == ["--planner-worker"]:
        _planner_worker_main()
    else:
        main()
