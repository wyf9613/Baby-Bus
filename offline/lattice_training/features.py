"""Versioned observation/candidate features; evaluator truth is never read."""
import math

import numpy as np

from .source import candidate_api

FEATURE_VERSION = "lattice_scene_candidate_v3"
ROAD_X = np.linspace(0, 3, 7)
PATH_FRACTIONS = np.linspace(0, 1, 12)
_SCENE_NAMES = [
    "speed_mps", "yaw_rate_rps", "speed_valid", "yaw_valid", "road_valid", "forward_geometry_valid",
    "road_degraded", "road_width_m", "road_confidence", "road_near_x_m", "road_far_x_m",
    "road_source_age_s", "road_prediction_remaining_s", "recommended_speed_scale"]
for _name in ("cone_detections", "wheel_speed", "imu_angular_velocity", "lidar_cartesian"):
    _SCENE_NAMES += [_name+"_available", _name+"_age_s"]
for _side in ("centerline", "left_boundary", "right_boundary"):
    for _index in range(len(ROAD_X)):
        _SCENE_NAMES += [f"{_side}_{_index}_y_m", f"{_side}_{_index}_supported"]
_SCENE_NAMES += [f"lidar_sector_{i}_min_m" for i in range(8)]
_SCENE_NAMES += ["obstacle_points_available"]
_CANDIDATE_NAMES = ["length_m", "offset_m", "duration_s", "terminal_speed_mps", "planned_stop",
                    "minimum_clearance_m", "reference_lifetime_s", "rule_center", "rule_clearance", "rule_steering",
                    "rule_consistency", "rule_motion"]
for _index in range(len(PATH_FRACTIONS)):
    _CANDIDATE_NAMES += [f"path_{_index}_{field}" for field in ("x_m", "y_m", "heading_rad", "curvature_1pm")]
for _index in range(7):
    _CANDIDATE_NAMES += [f"speed_{_index}_mps"]
FEATURE_NAMES = tuple(_SCENE_NAMES + _CANDIDATE_NAMES)


def _number(value, default=0.0):
    return float(value) if value is not None and math.isfinite(value) else default


def scene_features(observation):
    estimates = observation["estimation"]
    state, raw = estimates["state"], estimates["road"]
    forward = raw.get("forward_geometry") or {}
    road = raw if raw.get("valid") else forward
    bounds = road.get("x_range_m") or [0.0, 0.0]
    values = [_number(state.get("speed_mps")), _number(state.get("yaw_rate_rps")),
              float(state.get("speed_valid", False)), float(state.get("yaw_rate_valid", False)),
              float(raw.get("valid", False)), float(forward.get("geometry_valid", False)),
              float(road.get("degraded", False)), _number(road.get("lane_width_m")),
              _number(road.get("confidence")), float(bounds[0]), float(bounds[1]),
              _number(road.get("source_age_s"), 0.5), _number(road.get("prediction_remaining_s")),
              _number(road.get("recommended_speed_scale"), 1.0)]
    for name in ("cone_detections", "wheel_speed", "imu_angular_velocity", "lidar_cartesian"):
        values += [float(observation["sensor_available"].get(name, False)),
                   _number(observation["sensor_age_s"].get(name), 0.5)]
    for key in ("centerline_xy", "left_boundary_xy", "right_boundary_xy"):
        points = road.get(key) or []
        for x in ROAD_X:
            supported = bool(points and points[0][0] <= x <= points[-1][0])
            value = np.interp(x, [p[0] for p in points], [p[1] for p in points]) if supported else 0.0
            values += [float(value), float(supported)]
    obstacles = estimates["obstacles"]
    sectors = [4.0]*8
    if obstacles.get("available"):
        for x, y, _ in obstacles.get("points_xyz_m", []):
            if x < 0:
                continue
            angle = math.atan2(y, x)
            index = min(7, max(0, int((angle+math.pi/2)/math.pi*8)))
            sectors[index] = min(sectors[index], math.hypot(x, y))
    values += sectors+[float(obstacles.get("available", False))]
    result = np.asarray(values, dtype=np.float64)
    if len(result) != len(_SCENE_NAMES) or not np.isfinite(result).all():
        raise ValueError("Invalid scene feature schema")
    return result


def candidate_features(candidate):
    descriptor, ref = candidate["descriptor"], candidate["reference"]
    values = [descriptor[key] for key in ("length_m", "offset_m", "duration_s", "terminal_speed_mps")]
    values += [float(ref["planned_stop"]), candidate["minimum_clearance_m"], ref["valid_for_s"]]
    values += [descriptor["costs"][key] for key in ("center", "clearance", "steering", "consistency", "motion")]
    api = candidate_api()
    path = ref["frenet_path"]
    curve = api.LatticeCurve.from_record(path["reference_curve"])
    lateral = path["lateral"]
    lo, hi = lateral["range"]
    for fraction in PATH_FRACTIONS:
        geom = api._lattice_geometry(curve, lateral["origin"], lateral["coeffs_low_to_high"],
                                     lateral["scale"], lateral["terminal_offset_m"], lo+(hi-lo)*fraction)
        values += [geom["x"], geom["y"], geom["heading"], geom["curvature"]]
    profile = ref["speed_profile"]
    for t in np.linspace(0, ref["longitudinal_profile"]["duration_s"], 7):
        values.append(float(np.interp(t, [p[0] for p in profile], [p[1] for p in profile])))
    result = np.asarray(values, dtype=np.float64)
    if len(result) != len(_CANDIDATE_NAMES) or not np.isfinite(result).all():
        raise ValueError("Invalid candidate feature schema")
    return result


def encode_candidate(observation, candidate, *, context=None):
    return np.concatenate((scene_features(observation) if context is None else context,
                           candidate_features(candidate)))
