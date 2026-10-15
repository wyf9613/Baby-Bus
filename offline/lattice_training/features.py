"""Encode current state, current road/obstacles and candidate trajectory only."""
import numpy as np
from .reference import trajectory_samples

FEATURE_VERSION = 'state_road_trajectory_v1'
ROAD_POINTS = 16
OBSTACLE_POINTS = 256
TRAJECTORY_POINTS = 16
TRAJECTORY_COLUMNS = ('time_s','x_m','y_m','heading_rad','speed_mps','s_ref_relative_m','d_m')
_STATE_NAMES = ('speed_mps','yaw_rate_rps','speed_valid','yaw_rate_valid')
_ROAD_NAMES = tuple(f'{side}_{i}_{field}' for side in ('centerline','left_boundary','right_boundary')
                    for i in range(ROAD_POINTS) for field in ('x_m','y_m','present'))
_OBSTACLE_NAMES = ('obstacles_available',)+tuple(f'obstacle_{i}_{field}'
    for i in range(OBSTACLE_POINTS) for field in ('x_m','y_m','z_m','present'))
_TRAJECTORY_NAMES = tuple(f'trajectory_{i}_{field}' for i in range(TRAJECTORY_POINTS)
                          for field in TRAJECTORY_COLUMNS)
FEATURE_NAMES = _STATE_NAMES+_ROAD_NAMES+_OBSTACLE_NAMES+_TRAJECTORY_NAMES
INPUT_GROUPS = {
    'current_state': (0,len(_STATE_NAMES)),
    'current_road_and_obstacles': (len(_STATE_NAMES),len(_STATE_NAMES+_ROAD_NAMES+_OBSTACLE_NAMES)),
    'candidate_trajectory': (len(_STATE_NAMES+_ROAD_NAMES+_OBSTACLE_NAMES),len(FEATURE_NAMES)),
}


def _points(values,columns,name):
    result=np.asarray(values if values is not None else [],dtype=float)
    if result.size==0:return np.empty((0,columns))
    if result.ndim!=2 or result.shape[1]!=columns or not np.isfinite(result).all():
        raise ValueError(f'{name} requires finite {columns}-column points')
    return result


def _polyline(values):
    points=_points(values,2,'Road');result=np.zeros((ROAD_POINTS,3))
    if not len(points):return result.ravel()
    points=points[np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>1e-9]]
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    samples=np.linspace(0,arc[-1],ROAD_POINTS)
    for axis in (0,1):result[:,axis]=np.interp(samples,arc,points[:,axis])
    result[:,2]=1.
    return result.ravel()


def scene_features(observation):
    estimates=observation['estimation'];state=estimates['state']
    speed_valid=bool(state.get('speed_valid'));yaw_valid=bool(state.get('yaw_rate_valid'))
    values=[state['speed_mps'] if speed_valid else 0.,state['yaw_rate_rps'] if yaw_valid else 0.,
            float(speed_valid),float(yaw_valid)]
    road=estimates['road']
    if not road.get('valid'):road=road.get('forward_geometry') or road
    for side in ('centerline','left_boundary','right_boundary'):
        values.extend(_polyline(road.get(side+'_xy')))
    obstacles=estimates['obstacles'];available=bool(obstacles.get('available'))
    points=_points(obstacles.get('points_xyz_m') if available else [],3,'Obstacles')
    if len(points)>OBSTACLE_POINTS:
        raise ValueError(f'Obstacle input exceeds {OBSTACLE_POINTS} points; refusing silent truncation')
    # Canonical packet ordering; no sector aggregation or distance cost.
    if len(points):points=points[np.lexsort((points[:,2],points[:,1],points[:,0]))]
    padded=np.zeros((OBSTACLE_POINTS,4));padded[:len(points),:3]=points;padded[:len(points),3]=1.
    values.extend([float(available),*padded.ravel()])
    result=np.asarray(values,dtype=float)
    if result.shape!=(INPUT_GROUPS['candidate_trajectory'][0],) or not np.isfinite(result).all():
        raise ValueError('Invalid current-state/road/obstacle input')
    return result


def candidate_features(candidate):
    return trajectory_samples(candidate,TRAJECTORY_POINTS).ravel()


def encode_candidate(observation,candidate,*,context=None):
    result=np.concatenate((scene_features(observation) if context is None else context,candidate_features(candidate)))
    if result.shape!=(len(FEATURE_NAMES),) or not np.isfinite(result).all():
        raise ValueError('Invalid state/road/trajectory input')
    return result
