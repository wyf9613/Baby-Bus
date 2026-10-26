"""Forward LiDAR stop detection and wheel-speed stop confirmation; no ROS."""

from dataclasses import dataclass
from numbers import Real
import math

from .geometry import finite_number


@dataclass
class LidarObstacleSettings:
    """前方矩形检测区 / Straight-ahead stop-only detector, not a bypass planner."""
    enabled: bool = False  # 裸节点保持兼容；本分支 YAML 开启 / Enabled by branch YAML.
    body_width_m: float = 0.250
    front_from_cg_m: float = 0.297
    side_margin_m: float = 0.02
    range_min_m: float = 0.2  # 可观测区下限，不伪造驱动标定 / Observation lower bound.
    allow_no_return_clear: bool = False  # 激进假设由 YAML 显式开启 / Explicit opt-in.
    stop_distance_m: float = 1.0  # 从车头量起，待上车确认 / From bumper; provisional.
    cluster_gap_m: float = 0.06
    min_cluster_points: int = 2
    min_known_fraction: float = 0.7

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("lidar_obstacle.enabled must be boolean")
        if not isinstance(self.allow_no_return_clear, bool):
            raise ValueError("lidar_obstacle.allow_no_return_clear must be boolean")
        for name in ("body_width_m", "front_from_cg_m", "stop_distance_m", "cluster_gap_m"):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"lidar_obstacle.{name} must be positive and finite")
        if not finite_number(self.side_margin_m) or self.side_margin_m < 0:
            raise ValueError("lidar_obstacle.side_margin_m must be finite and nonnegative")
        if not finite_number(self.range_min_m) or self.range_min_m < 0:
            raise ValueError("lidar_obstacle.range_min_m must be finite and nonnegative")
        if (not isinstance(self.min_cluster_points, int) or isinstance(self.min_cluster_points, bool)
                or not 1 <= self.min_cluster_points <= 16):
            raise ValueError("lidar_obstacle.min_cluster_points must be an integer in [1,16]")
        if not finite_number(self.min_known_fraction) or not 0 < self.min_known_fraction <= 1:
            raise ValueError("lidar_obstacle.min_known_fraction must be in (0,1]")


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
    half_width = cfg.body_width_m/2 + cfg.side_margin_m
    bounds = ((cfg.front_from_cg_m, cfg.front_from_cg_m+cfg.stop_distance_m),
              (-half_width, half_width))
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


class StopFeedback:
    """停车看轮速，不看油门 / Confirm zero wheel-speed feedback, not zero effort.

    This is observation, not an active brake or external ground-speed sensor.
    Fresh post-request samples must report EXACT zero across the hold interval.
    """
    def __init__(self, zero_hold_s=0.2):
        if not finite_number(zero_hold_s) or zero_hold_s <= 0:
            raise ValueError("Positive stop-feedback hold required")
        self.zero_hold_s = zero_hold_s
        self.reset(None, 0.0)

    def reset(self, policy_state, now):
        self.policy_state = policy_state
        self.started_at = now if policy_state == 2 else None
        self.zero_since = self.last_zero_at = None
        self.zero_samples = 0

    def _clear_zero(self):
        self.zero_since = self.last_zero_at = None
        self.zero_samples = 0

    def update(self, policy_state, speed_mps, received_at, now, fresh, max_gap_s=0.5):
        if (policy_state != self.policy_state
                or (self.started_at is not None and now < self.started_at)):
            self.reset(policy_state, now)
        result = dict(state=0, phase="NOT_REQUESTED", speed_mps=None, zero_observed_s=0.0,
                      zero_samples=0, confirmed=False, source="wheel_speed_feedback",
                      active_braking=False)
        if policy_state == 3:
            return result
        if policy_state == 1:
            return dict(result, state=4, phase="NOT_PUBLISHING")
        if (not fresh or not finite_number(speed_mps) or speed_mps < 0
                or not finite_number(received_at) or received_at > now):
            self._clear_zero()
            return dict(result, state=3, phase="UNKNOWN")
        result.update(state=1, phase="STOPPING", speed_mps=float(speed_mps))
        if speed_mps > 0 or received_at <= self.started_at:
            self._clear_zero()
            return result
        if self.last_zero_at != received_at:
            if (self.last_zero_at is not None
                    and (received_at < self.last_zero_at or received_at-self.last_zero_at >= max_gap_s)):
                self._clear_zero()
            if self.zero_since is None:
                self.zero_since = received_at
            self.last_zero_at = received_at
            self.zero_samples += 1
        observed = max(0.0, self.last_zero_at-self.zero_since)
        result.update(zero_observed_s=observed, zero_samples=self.zero_samples)
        if self.zero_samples >= 2 and observed >= self.zero_hold_s-1e-9:
            result.update(state=2, phase="STOPPED", confirmed=True)
        return result
