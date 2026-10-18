"""Structured, untimed, single-path inputs; no resampling or planner costs."""
from copy import deepcopy
import json
import math

import numpy as np

from mvp21_policy.candidates import validate_candidate

FEATURE_VERSION = 'spatial_path_policy_v1'
INPUT_GROUPS = ('current_state', 'current_road_and_obstacles', 'candidate_path')
ROAD_ROLES = ('centerline', 'left_boundary', 'right_boundary')
BOUNDARY_SOURCES = ('observed', 'observed_with_history', 'history',
                    'predicted_history', 'inferred_width', 'inferred', 'unknown')
OBSTACLE_SECTORS = 8
OBSTACLE_RANGE_M = 4.0
FEATURE_SCHEMA = dict(
    state_columns=['speed_mps', 'yaw_rate_rps', 'speed_valid', 'yaw_rate_valid'],
    road_roles=list(ROAD_ROLES), boundary_sources=list(BOUNDARY_SOURCES),
    road_representation='native_ordered_xy_polylines_optional_independently',
    obstacle_representation='nearest_observed_xy_and_present_per_front_sector',
    obstacle_sectors=OBSTACLE_SECTORS, obstacle_range_m=OBSTACLE_RANGE_M,
    obstacle_fov_rad=[-math.pi/2, math.pi/2],
    candidate_columns=['p_m', 's_relative_m', 'd_m'],
    candidate_geometry_columns=['x_m', 'y_m', 'heading_rad'],
    candidate_count_per_input=1, speed_curve_input=False,
    coordinate_frame='decision_base_link', vehicle_reference_point='cg_ground_projection')


def _points(values, columns, name):
    result = np.asarray([] if values is None else values, dtype=float)
    if result.size == 0:
        return np.empty((0, columns))
    if result.ndim != 2 or result.shape[1] != columns or not np.isfinite(result).all():
        raise ValueError(f'{name} requires finite {columns}-column points')
    return result


def scene_features(observation):
    """Preserve supplied curves; summarize obstacles using the agreed sectors."""
    if observation.get('frame_id', 'base_link') != 'base_link':
        raise ValueError('Policy observation must use decision base_link')
    estimates = observation['estimation']
    state = estimates['state']
    speed_valid, yaw_valid = bool(state.get('speed_valid')), bool(state.get('yaw_rate_valid'))
    current = [state['speed_mps'] if speed_valid else 0.,
               state['yaw_rate_rps'] if yaw_valid else 0., float(speed_valid), float(yaw_valid)]
    if not np.isfinite(current).all():
        raise ValueError('Invalid current state')
    raw = estimates.get('road') or {}
    forward = raw.get('forward_geometry') or {}
    curves = {}
    for role in ROAD_ROLES:
        # A missing centerline or road.valid=False must not discard a usable side.
        source = raw if raw.get(role+'_xy') else forward
        points = _points(source.get(role+'_xy'), 2, 'Road '+role)
        side = role.split('_')[0]
        provenance = (source.get('boundary_source') or {}).get(side, 'unknown')
        if role == 'centerline':
            provenance = source.get('centerline_source', 'unknown')
        if provenance not in BOUNDARY_SOURCES:
            provenance = 'unknown'
        curves[role] = dict(points_xy=points.tolist(), source=provenance)
    obstacles = estimates.get('obstacles') or {}
    available = bool(obstacles.get('available'))
    sectors = [[0., 0., 0.] for _ in range(OBSTACLE_SECTORS)]
    nearest = [None]*OBSTACLE_SECTORS
    if available:
        for x, y, z in _points(obstacles.get('points_xyz_m'), 3, 'Obstacles'):
            distance = math.hypot(x, y)
            if x < 0 or distance > OBSTACLE_RANGE_M:
                continue
            index = min(OBSTACLE_SECTORS-1, max(0,
                int((math.atan2(y, x)+math.pi/2)/math.pi*OBSTACLE_SECTORS)))
            identity = (distance, float(x), float(y), float(z))
            if nearest[index] is None or identity < nearest[index]:
                nearest[index] = identity
                sectors[index] = [float(x), float(y), 1.]
    return dict(current_state=current,
        current_road_and_obstacles=dict(road=curves,
            obstacles=dict(available=available, sectors_xy_present=sectors)))


def candidate_features(candidate):
    """Only the original spatial contract is read; timing can be entirely absent."""
    if not isinstance(candidate, dict):
        raise ValueError('Exactly one candidate path is required per input')
    path = candidate['reference']['frenet_path']
    if path.get('type') != 'SAMPLED_FRENET':
        raise ValueError('Only current sampled Frenet candidates are supported')
    record = validate_candidate(path['candidate'])
    origin = record['s_origin_m']
    return dict(samples=[[p, s-origin, d] for p, s, d in record['samples']],
                geometry_xy_heading=deepcopy(record['geometry_xy_heading']))


def encode_candidate(observation, candidate, *, context=None):
    result = dict(schema_version=FEATURE_VERSION,
        **deepcopy(scene_features(observation) if context is None else context),
        candidate_path=candidate_features(candidate))
    validate_input(result)
    return result


def validate_input(record):
    if not isinstance(record, dict) or record.get('schema_version') != FEATURE_VERSION:
        raise ValueError('Expected the structured spatial-path input schema')
    if set(record) != {'schema_version', *INPUT_GROUPS}:
        raise ValueError('Spatial-path inputs must contain exactly three groups')
    state = np.asarray(record['current_state'], dtype=float)
    if state.shape != (4,) or not np.isfinite(state).all() or not np.isin(state[2:], [0., 1.]).all():
        raise ValueError('Invalid structured current state')
    scene = record['current_road_and_obstacles']
    if set(scene['road']) != set(ROAD_ROLES):
        raise ValueError('Road roles must be independently represented')
    for role in ROAD_ROLES:
        curve = scene['road'][role]
        _points(curve['points_xy'], 2, 'Road '+role)
        if curve['source'] not in BOUNDARY_SOURCES:
            raise ValueError('Unknown boundary source encoding')
    obstacles = scene['obstacles']
    sectors = _points(obstacles['sectors_xy_present'], 3, 'Obstacle sectors')
    if (not isinstance(obstacles['available'], bool) or sectors.shape != (OBSTACLE_SECTORS, 3)
            or not np.isin(sectors[:, 2], [0., 1.]).all()
            or (not obstacles['available'] and np.any(sectors))
            or np.any(sectors[sectors[:, 2] == 0, :2])):
        raise ValueError('Invalid obstacle availability/sector masks')
    path = record['candidate_path']
    if set(path) != {'samples', 'geometry_xy_heading'}:
        raise ValueError('Only spatial candidate fields are accepted')
    samples = _points(path['samples'], 3, 'Candidate Frenet samples')
    geometry = _points(path['geometry_xy_heading'], 3, 'Candidate geometry')
    if (len(samples) < 2 or len(samples) != len(geometry) or abs(samples[0, 0]) > 1e-8
            or np.any(np.diff(samples[:, 0]) <= 0)
            or not np.allclose(np.diff(samples[:, 0]), np.linalg.norm(np.diff(geometry[:, :2], axis=0), axis=1),
                               rtol=1e-6, atol=1e-8)):
        raise ValueError('Invalid spatial path ordering/arc geometry')
    return record


def pack_inputs(records):
    """Portable ragged storage: one validated JSON object per row, never pickle."""
    return np.asarray([json.dumps(validate_input(record), separators=(',', ':'), allow_nan=False)
                       for record in records])


def unpack_input(record):
    return validate_input(json.loads(str(record)) if isinstance(record, (str, np.str_)) else record)
