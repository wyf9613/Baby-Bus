"""Planner-independent Frenet candidate input for policy scoring.

s_ref is distance on a named reference; p is distance on the candidate itself.
The authored Cartesian samples remain the execution geometry. Exporting Frenet
descriptors never refits a path or changes the native planner decision.
"""
from copy import deepcopy
import hashlib
import json
import math
from .geometry import finite_number
from .frenet import ShadowFrame,_shadow_lengths

FRENET_CANDIDATE_SCHEMA='policy_frenet_candidate_v2'
SAMPLE_COLUMNS=('p_m','s_ref_m','d_m')


def validate_candidate(record):
    if record.get('schema_version')!=FRENET_CANDIDATE_SCHEMA:
        raise ValueError('Unsupported policy Frenet candidate schema')
    if record.get('frame_id')!='base_link' or record.get('vehicle_reference_point')!='cg_ground_projection':
        raise ValueError('Policy Frenet frame/reference point mismatch')
    if not isinstance(record.get('reference_id'),str) or not record['reference_id']:
        raise ValueError('Policy Frenet reference identity required')
    if record.get('sample_columns')!=list(SAMPLE_COLUMNS):
        raise ValueError('Policy Frenet units/columns mismatch')
    samples,geometry=record.get('samples'),record.get('geometry_xy_heading')
    if (not isinstance(samples,(list,tuple)) or not 2<=len(samples)<=1001
            or not isinstance(geometry,(list,tuple)) or len(geometry)!=len(samples)):
        raise ValueError('Frenet samples require matching authored geometry')
    if (any(len(row)!=3 or not all(finite_number(v) for v in row) for row in samples)
            or any(len(row)!=3 or not all(finite_number(v) for v in row) for row in geometry)
            or abs(samples[0][0])>1e-8 or any(b[0]<=a[0] for a,b in zip(samples,samples[1:]))):
        raise ValueError('Invalid Frenet samples or candidate distance ordering')
    if not finite_number(record.get('timestamp_s')) or not finite_number(record.get('s_origin_m')):
        raise ValueError('Frenet input timestamp/origin required')
    for a,b,pa,pb in zip(samples,samples[1:],geometry,geometry[1:]):
        if not math.isclose(b[0]-a[0],math.hypot(pb[0]-pa[0],pb[1]-pa[1]),rel_tol=1e-6,abs_tol=1e-8):
            raise ValueError('Candidate arc distance must match authored geometry')
    return record


def from_lattice(candidate,result,*,frame_id='base_link'):
    reference=result.get('reference_chart_xy') or result['planning_input']['road'].get('centerline_xy')
    if not reference:
        raise ValueError('Lattice result has no reference chart')
    frame=ShadowFrame(reference);path=[tuple(row) for row in candidate['path']]
    progress=_shadow_lengths([row[:2] for row in path]);samples=[]
    for i,(x,y,heading) in enumerate(path):
        station,lateral=frame.project((x,y),heading)
        samples.append([progress[i],station,lateral])
    identity=hashlib.sha256(json.dumps(dict(points=reference,frame=frame_id,
        timestamp_s=result['input_stamp_s']),sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return validate_candidate(dict(schema_version=FRENET_CANDIDATE_SCHEMA,frame_id=frame_id,
        vehicle_reference_point='cg_ground_projection',timestamp_s=result['input_stamp_s'],reference_id=identity,
        reference_geometry=dict(type='ORDERED_XY',samples_xy=deepcopy(reference)),
        reference_origin=result.get('reference_origin'),s_origin_m=result['start_s_m'],
        sample_columns=list(SAMPLE_COLUMNS),samples=samples,geometry_xy_heading=path))
