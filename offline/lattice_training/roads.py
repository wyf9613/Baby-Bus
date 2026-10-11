"""Self-contained adaptation of the upskill random road generation fragment.

Source: AI4R/ad_gym_unified_rl_mpc_training_and_evaluation.ipynb,
clean-random-road-distribution, and its Dream Gym 0.2.2 road_core/random_generation.py.
The original range sampler/geometry/element loop is retained below; ranges are
scaled for the course car, with nested v0.4 lanes and explicit double cone borders.
No imports or runtime reads from the upskill project are required.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np


@dataclass(frozen=True)
class RoadRandomizationSettings:
    element_count_min: int = 8
    element_count_max: int = 10
    straight_probability: float = 0.55
    straight_length_min_m: float = 2.0
    straight_length_max_m: float = 4.0
    radius_min_m: float = 4.5
    radius_max_m: float = 8.0
    sweep_min_deg: float = 20.0
    sweep_max_deg: float = 35.0
    first_length_min_m: float = 6.0
    first_length_max_m: float = 8.0
    lane_width_min_m: float = 1.0
    lane_width_max_m: float = 1.2
    cone_spacing_m: float = 0.5
    speed_limit_min_kph: float = 0.72
    speed_limit_max_kph: float = 1.08
    max_heading_deg: float = 120.0
    obstacle_count_min: int = 3
    obstacle_count_max: int = 6
    obstacle_length_min_m: float = 0.20
    obstacle_length_max_m: float = 0.35
    obstacle_width_min_m: float = 0.16
    obstacle_width_max_m: float = 0.28
    obstacle_spacing_m: float = 3.5
    minimum_passage_m: float = 0.55
    obstacle_edge_clearance_m: float = 0.04

    def __post_init__(self):
        for name in ("element_count_min", "element_count_max"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not 2 <= value <= 40:
                raise ValueError(f"Invalid random road setting: {name}")
        if self.element_count_max < self.element_count_min:
            raise ValueError("Random road element range is reversed")
        for prefix, suffix in (("straight_length", "m"), ("radius", "m"), ("sweep", "deg"),
                               ("first_length", "m"), ("lane_width", "m"), ("speed_limit", "kph")):
            low, high = (getattr(self, f"{prefix}_{endpoint}_{suffix}") for endpoint in ("min", "max"))
            if not math.isfinite(low) or not math.isfinite(high) or low <= 0 or high < low:
                raise ValueError(f"Invalid random road range: {prefix}")
        if not 0 <= self.straight_probability <= 1 or not self.sweep_max_deg <= self.max_heading_deg <= 180:
            raise ValueError("Invalid random road probability or heading bound")
        if isinstance(self.cone_spacing_m, bool) or not math.isfinite(self.cone_spacing_m) or self.cone_spacing_m <= 0:
            raise ValueError("Cone spacing must be finite and positive")
        if any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in
               (self.obstacle_count_min, self.obstacle_count_max)) or not self.obstacle_count_min <= self.obstacle_count_max <= 40:
            raise ValueError("Invalid obstacle count range")
        for prefix in ("obstacle_length", "obstacle_width"):
            low, high = getattr(self, prefix+"_min_m"), getattr(self, prefix+"_max_m")
            if not 0 < low <= high or not math.isfinite(high):
                raise ValueError("Invalid obstacle dimensions")
        if (not all(math.isfinite(v) for v in (self.obstacle_spacing_m, self.minimum_passage_m,
                                               self.obstacle_edge_clearance_m))
                or self.obstacle_spacing_m < 1.5 or self.minimum_passage_m < 0.45
                or self.obstacle_edge_clearance_m < 0):
            raise ValueError("Invalid passage/obstacle spacing")
        if self.obstacle_count_max and self.obstacle_width_max_m+self.minimum_passage_m+self.obstacle_edge_clearance_m > self.lane_width_min_m:
            raise ValueError("Obstacle range leaves insufficient space for the reserved passage")

    def configuration(self):
        # Same nested distribution schema as the upskill notebook, smaller dimensions.
        def limits(low, high):
            return {"lower": low, "upper": high}
        return {"element_count_range": limits(self.element_count_min, self.element_count_max),
                "straight_element_probability": self.straight_probability,
                "straight": {"length_range_m": limits(self.straight_length_min_m, self.straight_length_max_m)},
                "circular_arc": {"radius_range_m": limits(self.radius_min_m, self.radius_max_m),
                                 "sweep_angle_range_deg": limits(self.sweep_min_deg, self.sweep_max_deg)},
                "speed_limit_range_kph": limits(self.speed_limit_min_kph, self.speed_limit_max_kph),
                "first_element": {"require_straight": True,
                                  "straight_length_range_m": limits(self.first_length_min_m, self.first_length_max_m)},
                "initial_cross_section": {"reference_line_role": "lane_center",
                                          "reference_lane_width_range_m": limits(self.lane_width_min_m, self.lane_width_max_m)},
                "active_lane_minimum_width_m": 0.05}


# Adapted fragment: _uniform, straight/arc samplers and generate_random_road_spec
# from upskill's installed dreamgym/envs/road_core/random_generation.py.
def _uniform(rng, interval):
    if interval["upper"] == interval["lower"]:
        return float(interval["lower"])
    return float(rng.uniform(interval["lower"], interval["upper"]))


def _random_straight_geometry(rng, config, *, first=False):
    interval = (config["first_element"]["straight_length_range_m"] if first
                else config["straight"]["length_range_m"])
    return {"type": "straight", "length_m": _uniform(rng, interval)}


def _random_circular_arc_geometry(rng, config):
    arc = config["circular_arc"]
    radius = _uniform(rng, arc["radius_range_m"])
    sign = -1.0 if rng.random() < 0.5 else 1.0
    return {"type": "circular_arc", "curvature_per_m": sign/radius,
            "sweep_angle_deg": _uniform(rng, arc["sweep_angle_range_deg"])}


def generate_random_road_spec(seed, settings=None):
    settings = settings or RoadRandomizationSettings()
    config = settings.configuration()
    rng = np.random.default_rng(seed)
    count = config["element_count_range"]
    count = int(rng.integers(count["lower"], count["upper"]+1))
    elements, heading = [], 0.0
    for index in range(count):
        first = index == 0
        if first or rng.random() < config["straight_element_probability"]:
            geometry = _random_straight_geometry(rng, config, first=first)
        else:
            geometry = _random_circular_arc_geometry(rng, config)
            turning = math.copysign(geometry["sweep_angle_deg"], geometry["curvature_per_m"])
            if abs(heading+turning) > settings.max_heading_deg:
                geometry["curvature_per_m"] *= -1
                turning *= -1
            heading += turning
        element = {"geometry": geometry, "speed_limit_kph": _uniform(rng, config["speed_limit_range_kph"]),
                   "active_cone_paths": {"left_boundary": "regular", "right_boundary": "regular"}}
        if first:
            width = _uniform(rng, config["initial_cross_section"]["reference_lane_width_range_m"])
            element["lanes"] = {"reference_line_role": "lane_center",
                                "reference_lane": {"width_m": {"start": width, "end": width}, "surface": "tarmac"}}
        elements.append(element)
    # v0.4 requires named definitions for the active paths. Cone geometry stays
    # unchanged; spacing is an explicit experiment setting (notebook: 0.5m).
    cone_paths = {side: {"placement": {"mode": "lane_line", "lane_line_index": lane},
                        "color": color, "radius_m": 0.08,
                        "profiles": {"regular": {"inter_cone_spacing": {
                            "distribution": "fixed", "value_m": settings.cone_spacing_m}}}}
                  for side, lane, color in (("left_boundary", 1, "blue"), ("right_boundary", -1, "yellow"))}
    road = {"active_lane_minimum_width_m": config["active_lane_minimum_width_m"], "elements": elements,
            "cone_paths": cone_paths, "obstacles": []}
    obstacle_rng = np.random.default_rng(np.random.SeedSequence([seed, 0xBABA]))
    count = int(obstacle_rng.integers(settings.obstacle_count_min, settings.obstacle_count_max+1))
    available = road_length(road)-6.0
    capacity = max(0, int(available/settings.obstacle_spacing_m))
    if count > capacity:
        count = capacity
    if count < settings.obstacle_count_min:
        raise ValueError("Road too short to place the requested obstacles with safe spacing")
    # One random side per road reserves a continuous opposite-side corridor,
    # rather than merely leaving a disconnected gap beside each obstacle.
    side = -1 if obstacle_rng.random() < .5 else 1
    for index in range(count):
        spacing = available/max(count, 1)
        jitter = min(.2,max(0.0,(spacing-settings.obstacle_spacing_m)/2))
        progress = 3.0+(index+.5)*spacing+float(obstacle_rng.uniform(-jitter,jitter))
        length = float(obstacle_rng.uniform(settings.obstacle_length_min_m, settings.obstacle_length_max_m))
        obstacle_width = float(obstacle_rng.uniform(settings.obstacle_width_min_m, settings.obstacle_width_max_m))
        inner = -width/2+settings.minimum_passage_m
        lower = inner+obstacle_width/2
        upper = width/2-settings.obstacle_edge_clearance_m-obstacle_width/2
        offset = side*float(obstacle_rng.uniform(lower, upper))
        road["obstacles"].append({"geometry": {"type": "rectangle", "length_m": length, "width_m": obstacle_width},
            "placement": {"reference_line_progress_m": progress, "lateral_offset_m": offset, "heading_offset_rad": 0.0}})
    if road["obstacles"] and not obstacle_passage_report(road)["passable"]:
        raise ValueError("Generated obstacle layout failed its reserved full-body passage check")
    return road


def obstacle_passage_report(spec, *, vehicle_width_m=.25, front_m=.297, rear_m=.198, margin_m=.05):
    """Check a continuous reserved route with inflated full-car rectangles at 0.1m."""
    from dreamgym.envs import Road
    from dreamgym.envs.obstacle_geometry import rectangle_polygon, closed_rectangle_intersection_mask
    road = Road(road_spec=spec)
    obstacles = spec.get("obstacles", [])
    side = math.copysign(1.0, obstacles[0]["placement"]["lateral_offset_m"]) if obstacles else 1.0
    width = road.get_reference_lane_bounds_at_progress(1.0)
    lane_width = width["left_bound_m"]-width["right_bound_m"]
    gaps = [lane_width/2+abs(o["placement"]["lateral_offset_m"])-o["geometry"]["width_m"]/2 for o in obstacles]
    offset = -side*(lane_width/2-min(gaps, default=lane_width)/2)
    progression = np.arange(front_m+margin_m, road_length(spec)-front_m-margin_m, .1)
    centers, headings = road.convert_progression_to_coordinates(progression)
    centers += np.column_stack((-np.sin(headings), np.cos(headings)))*offset
    polygons, clearance, contact = road.get_obstacle_polygons(), float("inf"), False
    for center, heading in zip(centers, headings):
        body = rectangle_polygon(center, heading_rad=float(heading), forward_extent_m=front_m+margin_m,
                                 rear_extent_m=rear_m+margin_m, width_m=vehicle_width_m+2*margin_m)
        contact = contact or bool(closed_rectangle_intersection_mask(body, polygons).any())
        for x,y in body:
            cx,cy,_,_,s,tangent,_ = road.find_closest_point_to(float(x),float(y))
            bounds = road.get_reference_lane_bounds_at_progress(s)
            lateral = -(x-cx)*math.sin(tangent)+(y-cy)*math.cos(tangent)
            clearance = min(clearance,lateral-bounds['right_bound_m'],bounds['left_bound_m']-lateral)
    return {"minimum_gap_m": min(gaps, default=lane_width), "reserved_offset_m": offset,
            "inflated_body_width_m": vehicle_width_m+2*margin_m, "road_clearance_m": clearance,
            "obstacle_contact": contact, "passable": not contact and clearance >= 0,
            "sampling_spacing_m": .1, "continuous_swept_proof": False}


def road_length(road):
    return sum(element["geometry"]["length_m"] if element["geometry"]["type"] == "straight" else
               math.radians(element["geometry"]["sweep_angle_deg"])/abs(element["geometry"]["curvature_per_m"])
               for element in road["elements"])


def road_identity(road):
    # Different obstacle layouts/speed annotations on the same road geometry
    # must remain in the same dataset partition.
    geometry = {"elements": [{k:v for k,v in e.items() if k!="speed_limit_kph"} for e in road["elements"]],
                "cone_paths": road.get("cone_paths",{}),
                "active_lane_minimum_width_m":road.get("active_lane_minimum_width_m")}
    return "road:"+hashlib.sha256(json.dumps(geometry, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:24]


def randomization_provenance(settings=None):
    return {"source_notebook": "AI4R/ad_gym_unified_rl_mpc_training_and_evaluation.ipynb",
            "source_notebook_sha256": "0a6725ce363b94d83bb4dd1727caf88c7a193363113d62eac19b0cc6cca26ac2",
            "source_cell": "clean-random-road-distribution",
            "source_fragment": "dreamgym 0.2.2 road_core/random_generation.py generate_random_road_spec",
            "source_fragment_sha256": "6ca75671d8ea85818f978b6b1465fedb0f255083cca0ba60351aff3f4661e4d0",
            "adaptation": "baby_bus_nested_v04_connected_passage_obstacles_v2",
            "settings": asdict(settings or RoadRandomizationSettings())}
