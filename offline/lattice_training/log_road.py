"""Reconstruct a measured cone corridor for the existing native plant.

Wheel/gyro odometry is approximate. This is an offline measured-map experiment,
not surveyed geometry, an exact camera replay, or a claim about the unseen tail.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from offline.planner_sim import PlannerSimulationEnv, SensorTiming, load_notebook
from .dataset import json_safe, write_json


def read_log(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def odometry(rows, gyro_sign=-1.):
    wheel = np.array([(r['time_unix_s'], r['value']) for r in rows if r['kind'] == 'wheel'])
    gyro = np.array([(r['value']['stamp_s'], r['value']['z']) for r in rows if r['kind'] == 'gyro'])
    if len(wheel) < 2 or len(gyro) < 2:
        raise ValueError('Reconstruction requires wheel and stamped gyro samples')
    start = next(r['time_unix_s'] for r in rows if r['kind'] == 'phase' and r['value'] == 'Start policy once')
    times = np.arange(start-.4, rows[-1]['time_unix_s']+.01, .005)
    speed = np.interp(times, wheel[:,0], wheel[:,1])
    yaw = np.interp(times, gyro[:,0], gyro[:,1])
    # At rest yaw bias is measurable; retain both its value and applied correction.
    stationary = gyro[(gyro[:,0] < start) & (gyro[:,0] > start-3),1]
    bias = float(np.median(stationary)) if len(stationary) else 0.
    yaw = (yaw-bias)*gyro_sign
    heading = np.r_[0., np.cumsum((yaw[:-1]+yaw[1:])*.0025)]
    midpoint = (heading[:-1]+heading[1:])/2
    distances = (speed[:-1]+speed[1:])*.0025
    xy = np.vstack((np.zeros(2), np.cumsum(distances[:,None]*np.column_stack((np.cos(midpoint),np.sin(midpoint))),axis=0)))
    poses = np.column_stack((times,xy,heading,np.r_[0.,np.cumsum(distances)]))
    return poses, dict(start_time_s=start, gyro_bias_rps=bias, raw_gyro_to_map_yaw_sign=gyro_sign,
        gyro_sign_source='offline inference: raw logged sensor z has opposite sign to observed clockwise bend; mounting TF absent from log',
        wheel_gap_max_s=float(np.diff(wheel[:,0]).max()), gyro_gap_max_s=float(np.diff(gyro[:,0]).max()),
        method='trapezoidal_wheel_gyro_planar_no_slip', interpolation='linear_offline', independent_pose_truth=False)


def reconstruct(path, output):
    rows=read_log(path); poses, diagnostics=odometry(rows)
    parameters=next(r['value'] for r in rows if r['kind']=='parameters')
    left=parameters['estimation.left_colour']; right=parameters['estimation.right_colour']
    measurements=[]
    for row in rows:
        if row['kind'] != 'cones': continue
        # Header is already the acquisition reference. Latency is diagnostic,
        # not a second offset to subtract from this timestamp.
        stamp=row['value']['stamp_s']
        px,py,angle=[np.interp(stamp,poses[:,0],poses[:,i]) for i in (1,2,3)]
        c,s=math.cos(angle),math.sin(angle)
        for x,y,z,colour,confidence in row['value']['positions']:
            if colour in (left,right) and confidence >= .75 and .25 <= x <= 3.0:
                measurements.append([px+c*x-s*y,py+s*x+c*y,colour,confidence,stamp])
    points=np.array(measurements)
    if len(points)<20: raise ValueError('Insufficient measured cones')
    # Dense repeated observations are summarized as robust boundary bins rather
    # than claiming individual cone identities from this unlabelled log.
    boundaries={}
    for name,colour in (('left',left),('right',right)):
        side=points[points[:,2]==colour]
        bins=[]
        for x in np.arange(side[:,0].min(),side[:,0].max(),.15):
            group=side[(side[:,0]>=x)&(side[:,0]<x+.15)]
            if len(group)>=6:
                bins.append([float(np.median(group[:,0])),float(np.median(group[:,1])),len(group),
                             float(np.median(np.abs(group[:,1]-np.median(group[:,1]))))])
        boundaries[name]=np.array(bins)
    low=max(b[0,0] for b in boundaries.values()); high=min(b[-1,0] for b in boundaries.values())
    if high-low<2: raise ValueError('Not enough common bilateral map coverage')
    x=np.linspace(low,high,max(12,int((high-low)/.1)+1))
    # Smooth in a stated physical window; keep raw measurements/bins alongside.
    def fit(side):
        result=[]
        for query in x:
            near=side[abs(side[:,0]-query)<=.6]
            if len(near)<4: near=side[np.argsort(abs(side[:,0]-query))[:4]]
            coeff=np.polynomial.polynomial.polyfit(near[:,0]-query,near[:,1],2)
            result.append(coeff[0])
        return np.array(result)
    yl,yr=fit(boundaries['left']),fit(boundaries['right'])
    y=(yl+yr)/2; slope=np.gradient(y,x); heading=np.unwrap(np.arctan(slope))
    width=(yl-yr)/np.sqrt(1+slope*slope)
    if (width<.4).any() or (width>2).any():
        diagnostic=Path(output);diagnostic.mkdir(parents=True,exist_ok=True)
        write_json(diagnostic/'reconstruction-failure.json',dict(x=x.tolist(),left_y=yl.tolist(),right_y=yr.tolist(),
            width_m=width.tolist(),boundary_bins={k:v.tolist() for k,v in boundaries.items()},
            odometry_samples=poses.tolist(),measured_points=points.tolist()))
        raise ValueError(f'Reconstructed bilateral corridor has invalid width: {width.min():.3f}..{width.max():.3f}m')
    center=np.column_stack((x,y)); lengths=np.linalg.norm(np.diff(center,axis=0),axis=1)
    theta=float(heading[0]); origin=center[0]; c,s=math.cos(theta),math.sin(theta)
    rotation=np.array([[c,-s],[s,c]])
    local=(center-origin) @ rotation
    entry_m=1.0
    local[:,0]+=entry_m
    template=load_notebook(with_obstacles=False)
    spec=deepcopy(template.road_spec); spec['elements']=[]; spec['cone_paths']={}; spec['obstacles']=[]
    for index,length in enumerate(lengths):
        turning=float(heading[index+1]-heading[index]); curvature=turning/float(length)
        geometry=({'type':'straight','length_m':float(length)} if abs(curvature)<1e-7 else
                  {'type':'circular_arc','curvature_per_m':curvature,'sweep_angle_deg':math.degrees(abs(turning))})
        spec['elements'].append({'geometry':geometry,'speed_limit_kph':.72,
            'lanes':{'reference_line_role':'lane_center','reference_lane':{
                'width_m':{'start':float(width[index]),'end':float(width[index+1])},'surface':'tarmac'}},
            'active_cone_paths':{}})
    entry=deepcopy(spec['elements'][0]);entry['geometry']={'type':'straight','length_m':entry_m}
    entry['lanes']['reference_lane']['width_m']={'start':float(width[0]),'end':float(width[0])}
    spec['elements'].insert(0,entry)
    # Repeated spatial observations are fused at ~0.15m resolution. Positions
    # remain the measured boundary estimates, not regular synthetic cone paths.
    cones=[]
    for name,colour in (('left',left),('right',right)):
        for bx,by,count,mad in boundaries[name]:
            if low<=bx<=high:
                point=(np.array([bx,by])-origin) @ rotation
                point[0]+=entry_m
                cones.append(dict(x_m=float(point[0]),y_m=float(point[1]),colour=int(colour),
                                  observations=int(count),vertical_mad_m=float(mad)))
    from dreamgym.envs import Road
    road=Road(road_spec=spec)
    generated,_=road.convert_progression_to_coordinates(entry_m+np.r_[0.,np.cumsum(lengths)])
    error=np.linalg.norm(generated-local,axis=1)
    initial_odo=np.array([np.interp(diagnostics['start_time_s'],poses[:,0],poses[:,i]) for i in (1,2)])
    initial_local=(initial_odo-origin) @ rotation
    initial_local[0]+=entry_m
    nearest=road.find_closest_point_to(*map(float,initial_local))
    # Original initial CG precedes the first observed cone. The explicit 1m
    # buffer supplies a bounded assumed entry corridor and rear body support;
    # it is kept separate from the length actually covered by both boundaries.
    report=dict(format='recorded_cone_road_v1',source_path=str(Path(path).resolve()),
        source_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),parameters=parameters,
        source_time_span_s=[rows[0]['time_unix_s'],rows[-1]['time_unix_s']],
        recorded_policy_rate_hz=parameters['policy_update_rate_hz'],
        camera_period_median_s=float(np.median(np.diff([r['value']['stamp_s'] for r in rows if r['kind']=='cones']))),
        camera_latency_median_s=float(np.median([r['value']['latency_s'] for r in rows if r['kind']=='cones'])),
        odometry=diagnostics,odometry_samples=poses.tolist(),measured_points=points.tolist(),
        boundary_bins={k:v.tolist() for k,v in boundaries.items()},
        common_observed_x_range_m=[float(low),float(high)], map_origin_world_xy=origin.tolist(),
        map_heading_world_rad=theta,centerline_local_xy=local.tolist(),width_m=width.tolist(),
        road_spec=spec,cones=cones,road_length_m=float(lengths.sum())+entry_m,
        observed_corridor_length_m=float(lengths.sum()),first_bilateral_observed_station_m=entry_m,
        entry_buffer_m=entry_m,entry_buffer_observed=False,
        road_fit_max_error_m=float(error.max()),road_fit_rms_error_m=float(np.sqrt(np.mean(error**2))),
        original_initial_pose_local_xy=initial_local.tolist(),original_initial_progress_m=float(nearest[4]),
        log_complete=True, surveyed_road_complete=False, unseen_end_extended=False,
        reconstruction_settings=dict(confidence_min=.75,body_x_m=[.25,3.0],bin_m=.15,min_bin_observations=6,
                                     smoothing_half_window_m=.6),
        limitations=['wheel/gyro map has unmeasured scale/slip/depth error',
                     'fused boundary bins do not prove individual cone identity',
                     'road end is last shared observed boundary, not a surveyed physical finish',
                     'entry buffer extrapolates first boundary tangent and width; native controlled clear-start checks still apply'])
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    write_json(output/'road.json',report)
    write_json(output/'road_spec.json',spec)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for name,colour,color in (('left',left,'goldenrod'),('right',right,'royalblue')):
        p=points[points[:,2]==colour]; axes[0].scatter(p[:,0],p[:,1],s=2,alpha=.1,c=color)
        b=boundaries[name]; axes[0].plot(b[:,0],b[:,1],'.-',c=color,label=f'{name} measured bins')
        p=np.array([[r['x_m'],r['y_m']] for r in cones if r['colour']==colour])
        axes[1].scatter(p[:,0],p[:,1],s=12,c=color,label=name)
    axes[0].plot(poses[:,1],poses[:,2],c='gray',label='wheel/gyro odometry')
    axes[0].plot(x,y,c='black',label='fused centerline')
    axes[1].plot(generated[:,0],generated[:,1],c='black',label='native plant centerline')
    for ax in axes: ax.axis('equal');ax.legend();ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid(alpha=.2)
    axes[0].set_title('Recorded road reconstruction');axes[1].set_title('Measured common corridor in Dream Gym')
    fig.tight_layout();fig.savefig(output/'road.png',dpi=160);plt.close(fig)
    print(json.dumps({k:report[k] for k in ('road_length_m','road_fit_max_error_m','common_observed_x_range_m','camera_period_median_s','camera_latency_median_s')},allow_nan=False))
    return report


def environment(report, *, seed=0, progress=.3, offset=0., speed=.1, latency_steps=4, max_steps=30000, estimation_settings=None):
    from dreamgym.envs.road_core.cones import ResolvedCone, ResolvedConeSet
    config=load_notebook(with_obstacles=False);config.road_spec=deepcopy(report['road_spec'])
    for key,value in (('progress',progress),('lateral_offset_m',offset),('heading_error_rad',0.)):
        config.initial_state_config['pose'][key].update(lower=value,upper=value)
    config.initial_state_config['body_motion']['longitudinal_velocity']['value_mps'].update(lower=speed,upper=speed)
    env=PlannerSimulationEnv(config,with_obstacles=False,max_steps=max_steps,estimation_settings=estimation_settings,
        sensor_timing=SensorTiming(period_steps={'cone_detections':2},delay_steps={'cone_detections':latency_steps}))
    env.reset(seed=seed)
    records=[]
    for i,cone in enumerate(report['cones']):
        position=(cone['x_m'],cone['y_m'])
        station=env._backend.road.find_closest_point_to(*position)[4]
        records.append(ResolvedCone(position,position,'yellow' if cone['colour']==1 else 'blue',cone['colour']-1,.04,
                                    'measured','recorded',0,i,float(station),float(station),0.))
    env._backend._resolved_cones=ResolvedConeSet(np.array([r.position_world_m for r in records]),
        np.array([r.color_id for r in records]),np.array([r.radius_m for r in records]),tuple(records))
    # Match policy's geometric left/right mapping to the recorded parameter readback.
    env.estimator.left_colour=report['parameters']['estimation.left_colour']
    env.estimator.right_colour=report['parameters']['estimation.right_colour']
    # Re-prime sensors after installing the measured map; no generated cones leak
    # into estimator history or pending acquisition packets.
    from collections import deque
    env._pending={k:deque() for k in env._pending};env._latest={}
    for _ in range(latency_steps+2): env.step((0.,0.))
    return env


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();reconstruct(args.log,args.output)
