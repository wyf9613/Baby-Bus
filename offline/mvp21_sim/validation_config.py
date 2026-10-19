"""Offline module choices and integer simulation clocks; no ROS overrides."""
from copy import deepcopy
import math

from .policy_selection import DEFAULT_MODEL

DEFAULT = dict(schema='module_validation_v1',
    modules=dict(perception='observed',planner='native',policy='native',control='mvp'),
    frequencies_hz=dict(physics=20.,perception=20.,planner=1.,policy=1.,control=20.),
    camera_latency_s=.05,planner_latency_s=.1,terminal_support_m=2.,
    max_steps=2400,stop_tail_s=1.,model_path=str(DEFAULT_MODEL),
    cstar_solver={},cstar_objective={},cruise_speed_mps=.2)


def configuration(value=None):
    result=deepcopy(DEFAULT)
    for key,item in (value or {}).items():
        if key not in result:raise ValueError('Unknown validation setting: '+key)
        if key in ('modules','frequencies_hz'):
            if not isinstance(item,dict) or set(item)-set(result[key]):raise ValueError('Unknown '+key)
            result[key].update(item)
        else:result[key]=deepcopy(item)
    if result['schema']!=DEFAULT['schema']:raise ValueError('Unsupported validation schema')
    for key,allowed in dict(perception=('observed','perfect','no_centerline'),planner=('native','mvp1_straight'),
                            policy=('native','model'),control=('mvp','cstar')).items():
        if result['modules'][key] not in allowed:raise ValueError('Unknown '+key+' module')
    rates=result['frequencies_hz'];base=rates['physics']
    for key,value in rates.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Positive finite module frequency required: '+key)
        if value>base or not math.isclose(base/value,round(base/value),abs_tol=1e-8):
            raise ValueError('Module periods must be integer physics ticks: '+key)
    for key in ('camera_latency_s','planner_latency_s','terminal_support_m','stop_tail_s'):
        value=result[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
            raise ValueError('Nonnegative finite '+key+' required')
    if not math.isclose(result['camera_latency_s']*base,round(result['camera_latency_s']*base),abs_tol=1e-8):
        raise ValueError('Camera latency must be integer physics ticks')
    if type(result['max_steps']) is not int or result['max_steps']<1:raise ValueError('Positive max_steps required')
    if not isinstance(result['cruise_speed_mps'],(int,float)) or not math.isfinite(result['cruise_speed_mps']) or result['cruise_speed_mps']<=0:
        raise ValueError('Positive cruise_speed_mps required')
    if result['modules']['control']=='cstar':
        from offline.lattice_training.cstar import SolverSettings
        from offline.lattice_training.objective import CStarTrackingSettings
        SolverSettings(**result['cstar_solver']);CStarTrackingSettings(**{**result['cstar_objective'],'dt_s':1/rates['control']})
    return result
