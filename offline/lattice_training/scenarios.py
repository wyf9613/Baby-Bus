"""Explicit expanded collection roads; no changes to the native plant/planner."""
from copy import deepcopy
import numpy as np

SUITE_KINDS = ('suite_straight', 'suite_left', 'suite_right', 'uturn_left', 'uturn_right')


def expanded_road_spec(kind, seed):
    if kind not in SUITE_KINDS:
        raise ValueError('Unknown expanded scene family')
    rng = np.random.default_rng(seed)
    turn = -1 if kind.endswith('right') else 1
    uturn = kind.startswith('uturn')
    radius = float(rng.uniform(1.0, 1.35) if uturn else rng.uniform(2.8, 3.6))
    width = 2*radius if uturn else float(rng.uniform(1.1, 1.3))
    entry = float(rng.uniform(3.2, 3.8))
    exit_length = entry if uturn else float(rng.uniform(3.3, 4.0))
    active = {'left_boundary': 'regular', 'right_boundary': 'regular'}
    outer = 'right_boundary' if turn > 0 else 'left_boundary'
    first = dict(geometry=dict(type='straight', length_m=entry), speed_limit_kph=.72,
        lanes=dict(reference_line_role='lane_center', reference_lane=dict(
            width_m=dict(start=width, end=width), surface='tarmac')), active_cone_paths=active)
    elements = [first]
    if kind != 'suite_straight':
        elements.append(dict(geometry=dict(type='circular_arc', curvature_per_m=turn/radius,
            sweep_angle_deg=180. if uturn else float(rng.uniform(50., 75.))), speed_limit_kph=.72,
            active_cone_paths={outer: 'regular'} if uturn else deepcopy(active)))
    elements.append(dict(geometry=dict(type='straight', length_m=exit_length), speed_limit_kph=.72,
        active_cone_paths={outer: 'regular'} if uturn else deepcopy(active)))
    cones = {side: dict(placement=dict(mode='lane_line', lane_line_index=lane), color=color,
        radius_m=.08, profiles=dict(regular=dict(inter_cone_spacing=dict(
            distribution='fixed', value_m=.35))))
        for side, lane, color in [('left_boundary', 1, 'blue'), ('right_boundary', -1, 'yellow')]}
    road = dict(active_lane_minimum_width_m=.05, elements=elements, cone_paths=cones, obstacles=[])
    # The U-turn width is twice its reference radius: the two inner straight
    # boundaries coincide. Only the entry emits inner cones; the arc and return
    # emit outer cones. Thus there is one inner straight, with no inner arc.
    from .roads import road_length
    length = road_length(road)
    layout = ('none', 'left', 'right', 'alternating', 'offset_pair')[seed % 5]
    if layout != 'none':
        for index, station in enumerate((1.8, length-1.8)):
            side = (1 if layout == 'left' else -1 if layout == 'right' else
                    (1 if index == 0 else -1))
            lateral = side*(width*.34 if layout != 'offset_pair' else width*.24)
            road['obstacles'].append(dict(geometry=dict(type='rectangle',
                length_m=float(rng.uniform(.20, .30)), width_m=float(rng.uniform(.16, .22))),
                placement=dict(reference_line_progress_m=station, lateral_offset_m=lateral,
                               heading_offset_rad=0.)))
    return road


def road_input(observation, mode):
    """A controlled missing output, shared by native generation and policy data.

    This removes supplied centerlines after perception, rather than claiming
    that the camera naturally failed to fit one. Boundaries and sensors remain
    the measured values; no map/oracle geometry is inserted.
    """
    if mode == 'perception':
        return observation
    if mode != 'no_centerline':
        raise ValueError('Unknown collection road-input mode')
    result = deepcopy(observation)
    road = result['estimation']['road']
    for source in (road, road.get('forward_geometry') or {}):
        source['centerline_xy'] = []
        source.pop('centerline_source', None)
    return result
