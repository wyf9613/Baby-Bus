"""Virtual sensor conversion and reuse of the deployed pure estimator."""
import ast
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import math
from numbers import Real
import sys
from types import ModuleType

import numpy as np
from dreamgym.envs import Road

from .notebook import ROOT

ESTIMATOR_SOURCE = ROOT / "scripts/policy_node.py"
_DEFINITIONS = {
    "finite_number", "EstimationSettings", "FirstOrderSampleFilter",
    "EstimationMotionHistory", "_estimation_median", "_estimation_solve",
    "_estimation_fit_boundary", "EstimationPipeline",
}


def make_estimator(settings=None):
    """Load actual estimator definitions; no rclpy or test-module imports."""
    data = ESTIMATOR_SOURCE.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    module_name = "_baby_bus_sim_estimator_" + digest[:16]
    if module_name not in sys.modules:
        nodes = [node for node in ast.parse(data).body
                 if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in _DEFINITIONS]
        if {node.name for node in nodes} != _DEFINITIONS:
            raise RuntimeError("Deployed estimator definitions changed; update the simulation adapter")
        module = ModuleType(module_name)
        module.__dict__.update(dataclass=dataclass, math=math, Real=Real, deepcopy=deepcopy)
        sys.modules[module_name] = module
        try:
            exec(compile(ast.Module(body=nodes, type_ignores=[]), str(ESTIMATOR_SOURCE), "exec"), module.__dict__)
        except Exception:
            del sys.modules[module_name]
            raise
    api = sys.modules[module_name]
    config = api.EstimationSettings(**(settings or {}))
    return api.EstimationPipeline(config, "base_link", 2, 1), digest


def native_sensor_packets(observation, observation_config, confidence):
    """Map Gym colours/padding and LiDAR no-returns to the policy contract.

    The planar simulator COM maps to the ground projection of the CG. z=0 is
    a simulated detected point. Confidence is an explicit synthetic constant.
    """
    count = int(observation["cone_detections_count"][0])
    forward = observation["cone_detections_forward_positions_in_body_frame_m"]
    left = observation["cone_detections_left_positions_in_body_frame_m"]
    colours = observation["cone_detections_color_ids"]
    if not 0 <= count <= min(len(forward), len(left), len(colours)):
        raise ValueError("Invalid native cone count")
    # Dream Gym yellow=0/blue=1; DREAM message yellow=1/blue=2.
    colour_map = {Road.CONE_COLOR_TO_ID["yellow"]: 1, Road.CONE_COLOR_TO_ID["blue"]: 2}
    detections = []
    for index in range(count):
        colour = colour_map.get(int(colours[index]))
        x, y = float(forward[index]), float(left[index])
        if colour is not None and math.isfinite(x) and math.isfinite(y):
            detections.append((x, y, 0.0, colour, confidence))
    packets = {
        "cone_detections": {"cone_detections": {"detections": detections,
                                                 "acquisition_to_publish_latency_s": 0.0}},
        # Unsigned CG-forward speed is an idealized wheel telemetry proxy.
        "wheel_speed": {"wheel_speed": abs(float(observation["body_longitudinal_velocity_mps"][0]))},
        "imu_angular_velocity": {"imu_angular_velocity":
                                 (0.0, 0.0, float(observation["yaw_rate_rad_per_s"][0]))},
        "lidar": {"lidar_scan": None, "lidar_cartesian": None},
    }
    if "obstacle_lidar_ranges_m" not in observation:
        return packets
    lidar = observation_config["obstacle_lidar"]
    acquisition = lidar["acquisition"]
    minimum, maximum = acquisition["minimum_range_m"], acquisition["maximum_range_m"]
    ranges = np.asarray(observation["obstacle_lidar_ranges_m"], dtype=float)
    if lidar.get("range_encoding", "metres") == "fraction_of_maximum_range":
        ranges = ranges * maximum
    fov = math.radians(acquisition["horizontal_field_of_view_deg"])
    if len(ranges) == 1:
        angles = np.zeros(1)
    else:
        angles = np.linspace(-fov/2, fov/2, len(ranges), endpoint=fov < 2*math.pi-1e-12)
    mount = lidar["mounting_pose_in_body_frame"]
    points, indices, scan_ranges = [], [], []
    for index, (distance, angle) in enumerate(zip(ranges, angles)):
        # max is the simulator's no-return sentinel. A saturated hit at max is
        # observationally indistinguishable; never manufacture a range-limit wall.
        hit = math.isfinite(distance) and minimum <= distance < maximum
        scan_ranges.append(float(distance) if hit else math.inf)
        if hit:
            heading = angle + mount["heading_offset_from_body_rad"]
            points.append((mount["forward_from_center_of_mass_m"] + distance*math.cos(heading),
                           mount["left_from_center_of_mass_m"] + distance*math.sin(heading), 0.0))
            indices.append(index)
    increment = float(angles[1]-angles[0]) if len(angles) > 1 else fov
    packets["lidar"] = {
        "lidar_scan": {"ranges": scan_ranges, "intensities": [], "frame_id": "sim_lidar",
                       "angle_min": float(angles[0]), "angle_max": float(angles[-1]),
                       "angle_increment": increment, "time_increment": 0.0, "scan_time": 0.0,
                       "range_min": minimum, "range_max": maximum},
        "lidar_cartesian": {"points_xyz": points, "scan_indices": indices, "frame_id": "base_link"},
    }
    return packets
