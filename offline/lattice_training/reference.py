"""Decode the current sampled candidate and its authored time law."""
import math
import numpy as np
from mvp21_policy.candidates import validate_candidate


def polynomial(coefficients,value):
    result=0.
    for coefficient in reversed(coefficients):result=result*value+coefficient
    return result


def sampled_candidate(reference):
    path=reference['frenet_path']
    if path.get('type')!='SAMPLED_FRENET':
        raise ValueError('Only current sampled Frenet candidates are supported')
    return validate_candidate(path['candidate'])


def spatial_reference(reference):
    record=sampled_candidate(reference);samples=np.asarray(record['samples'],dtype=float)
    geometry=np.asarray(record['geometry_xy_heading'],dtype=float);angles=np.unwrap(geometry[:,2])
    def at(progress):
        if not samples[0,0]-1e-8<=progress<=samples[-1,0]+1e-8:
            raise ValueError('Candidate distance outside authored path')
        return dict(x=float(np.interp(progress,samples[:,0],geometry[:,0])),
            y=float(np.interp(progress,samples[:,0],geometry[:,1])),
            heading=float(np.interp(progress,samples[:,0],angles)),
            s=float(np.interp(progress,samples[:,0],samples[:,1])),
            d=float(np.interp(progress,samples[:,0],samples[:,2])))
    return (0.,float(samples[-1,0])),at


def trajectory_samples(candidate,count):
    reference=candidate['reference'];record=sampled_candidate(reference)
    (low,high),at=spatial_reference(reference)
    timing=reference['longitudinal_profile'];duration=float(timing['duration_s'])
    coefficients=np.asarray(timing['coeffs_low_to_high'],dtype=float)
    profile=np.asarray(reference['speed_profile'],dtype=float)
    if (timing.get('type')!='P_PATH_OF_TIME' or not math.isfinite(duration) or duration<=0
            or coefficients.ndim!=1 or not 1<=len(coefficients)<=6 or not np.isfinite(coefficients).all()
            or profile.ndim!=2 or profile.shape[1]!=2 or len(profile)<2 or not np.isfinite(profile).all()
            or np.any(np.diff(profile[:,0])<=0) or profile[0,0]>0 or profile[-1,0]<duration
            or np.any(profile[:,1]<0)):
        raise ValueError('Candidate requires explicit path-arc timing and speed samples')
    end=float(polynomial(coefficients,1.))
    if not reference.get('planned_stop') and not timing.get('stop') and not math.isclose(end,high,abs_tol=1e-7):
        raise ValueError('Candidate timing must cover the full path; implicit extensions are unsupported')
    rows=[]
    for time in np.linspace(0,duration,count):
        progress=float(polynomial(coefficients,time/duration))
        if not low-1e-8<=progress<=high+1e-8:raise ValueError('Timed trajectory exceeds authored geometry')
        point=at(min(high,max(low,progress)))
        rows.append([time,point['x'],point['y'],point['heading'],
            float(np.interp(time,profile[:,0],profile[:,1])),point['s']-record['s_origin_m'],point['d']])
    return np.asarray(rows,dtype=float)
