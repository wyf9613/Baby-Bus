"""20 Hz native planner validation on a recorded road, using existing C* control.

The measured-map plant generates current visible sensor observations. Native
planning runs every tick with direct Frenet output and real wall budgets.
C* solves the full reference, then only its receding-horizon prefix is applied.
"""
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np

from .cstar import CStarSolver, NativePrediction, SolverSettings
from .dataset import write_json, json_safe
from .log_road import environment
from .objective import CStarTrackingSettings, reference_plan
from .source import candidate_api, planner_source_sha256


def local_curve_bounds(curve,low,high):
    """Exact derivative bounds of native cubics over the footprint interval."""
    if low<curve.xmin-1e-8 or high>curve.xmax+1e-8:raise ValueError('curve_range')
    probes=[low,high]
    intervals=[(curve.extension[0],curve.extension[1],curve.extension[1],curve.extension[2])]
    intervals += [(a,b,a,coeff) for a,b,coeff in curve.segments]
    for a,b,origin,coeff in intervals:
        lo,hi=max(a,low),min(b,high)
        if hi<lo:continue
        probes.extend((lo,hi))
        if abs(coeff[3])>1e-12:
            stationary=origin-coeff[2]/(3*coeff[3])
            if lo<stationary<hi:probes.append(stationary)
    values=[curve.at_x(x) for x in probes]
    return max(abs(v[1]) for v in values),max(abs(v[2]) for v in values)


def map_road_input(env,observation,*,lookahead_m=2.0,state_input='measured'):
    """Explicit map oracle to isolate planner failures from camera reconstruction.

    This is ground-truth geometry from the recorded-map plant, NOT a perception
    pass. Measured wheel/gyro state and all native planner checks remain.
    """
    from copy import deepcopy
    result=deepcopy(observation);raw=env._backend.car._state_values();road=env._backend.road
    current=env._info['evaluation']['ground_truth']['reference_line_progress_m']
    length=sum(e['geometry'].get('length_m',0.) if e['geometry']['type']=='straight' else
        math.radians(e['geometry']['sweep_angle_deg'])/abs(e['geometry']['curvature_per_m']) for e in env.config.road_spec['elements'])
    stations=np.linspace(max(0.,current-.5),min(length,current+lookahead_m),max(3,math.ceil(lookahead_m/.05)+11))
    center,angles=road.convert_progression_to_coordinates(stations)
    normals=np.column_stack((-np.sin(angles),np.cos(angles)))
    bounds=[road.get_reference_lane_bounds_at_progress(float(s)) for s in stations]
    left=center+normals*np.array([b['left_bound_m'] for b in bounds])[:,None]
    right=center+normals*np.array([b['right_bound_m'] for b in bounds])[:,None]
    c,s=math.cos(raw[2]),math.sin(raw[2]);rotation=np.array([[c,-s],[s,c]])
    paths=[(p-np.array(raw[:2]))@rotation for p in (center,left,right)]
    record=dict(timestamp_s=observation['timestamp_s'],frame_id='base_link',valid=True,geometry_valid=True,
        time_aligned=True,visibility='both',status='recorded_map_oracle',source_age_s=.2,
        recommended_speed_scale=1.,degraded=False,centerline_xy=paths[0].tolist(),
        left_boundary_xy=paths[1].tolist(),right_boundary_xy=paths[2].tolist(),
        lane_width_m=float(np.mean([b['left_bound_m']-b['right_bound_m'] for b in bounds])),
        local_curvature_1pm=float(road.find_closest_point_to(float(raw[0]),float(raw[1]))[-1]),
        road_input_source='recorded_map_ground_truth')
    result['estimation']['road']=record
    if state_input=='plant_truth':
        result['estimation']['state'].update(timestamp_s=observation['timestamp_s'],frame_id='base_link',
            speed_mps=float(abs(raw[3])),v_mps=float(abs(raw[3])),yaw_rate_rps=float(raw[5]),yaw_rate_radps=float(raw[5]),
            speed_valid=True,yaw_rate_valid=True,
            source_age_s={'wheel_speed':0.,'imu_angular_velocity':0.},state_input_source='plant_truth')
    return result


def make_native(configuration):
    api=candidate_api()
    from .pipeline import make_planner
    planning=dict(algorithm='lattice_v2',max_source_age_s=.4,reference_lifetime_s=.2,cruise_speed_mps=.2)
    lattice=dict(model_source='course_simulation',clear_start_assumed=True,obstacle_check_enabled=False)
    planning.update(configuration.get('planning',{}));lattice.update(configuration.get('lattice',{}))
    planner=make_planner(require_cartesian_compatibility=configuration.get('require_cartesian_compatibility',False))
    if configuration.get('clearance_bound_scope')=='local_footprint_experiment':
        # Counterfactual only: identical native checks and thresholds, with exact
        # cubic derivative bounds restricted to this footprint and sweep guard.
        # Production policy source and trained model fingerprints are untouched.
        parent=type(planner)
        class LocalFootprintPlanner(parent):
            def _clearance(self,geom,left,right,obstacles,model):
                c,s=math.cos(geom['heading']),math.sin(geom['heading'])
                xs=[geom['x']+c*bx-s*by for bx in (-model['rear_extent_m'],model['front_extent_m'])
                    for by in (-model['width_m']/2,model['width_m']/2)]
                saved=[(curve,curve.max_slope,curve.max_second) for curve in (left,right)]
                try:
                    for curve in (left,right):
                        curve.max_slope,curve.max_second=local_curve_bounds(curve,min(xs),max(xs))
                    return super()._clearance(geom,left,right,obstacles,model)
                finally:
                    for curve,slope,second in saved:curve.max_slope,curve.max_second=slope,second
        planner.__class__=LocalFootprintPlanner
    planner.settings=api.PlanningSettings(**planning);planner.cfg=api.LatticeSettings(**lattice)
    planner.clock=time.perf_counter if configuration.get('enforce_planner_wall_clock',True) else (lambda:0.)
    return planner


def query(planner, observation):
    e=observation['estimation']
    started=time.perf_counter()
    reference,diagnostics=planner.plan(e['road'],e['state'],e['obstacles'],e['vehicle_limits'],observation['timestamp_s'],
        source_timeout_s={'road':.4,'wheel_speed':.15,'imu_angular_velocity':.15,'obstacles':.4})
    return reference,diagnostics,time.perf_counter()-started


def contract(reference, now):
    errors=[]
    for key,value in (('schema_version','planning_reference_v2'),('frame_id','base_link'),('valid',True)):
        if reference.get(key)!=value:errors.append(key)
    if not reference.get('valid'):return errors
    cartesian=reference.get('path')
    if cartesian is not None and cartesian.get('type')!='CARTESIAN_Y_OF_X':errors.append('cartesian_path')
    if reference.get('frenet_path',{}).get('type')!='FRENET_SPATIAL':errors.append('frenet_path')
    if not 0<=now-reference['timestamp_s']<reference['valid_for_s']:errors.append('freshness')
    if reference.get('stop_requested') is not False:errors.append('stop_requested')
    if not math.isfinite(reference['target_speed_mps']) or reference['target_speed_mps']<0:errors.append('target_speed')
    path=reference['path']
    if path is not None and (not 1<=len(path['coeffs_low_to_high'])<=6 or not all(math.isfinite(v) for v in path['coeffs_low_to_high'])):errors.append('path_coefficients')
    api=candidate_api()
    try:
        if path is not None:
            for x in np.linspace(*path['range'],11):
                point=api.evaluate_planning_path(reference,float(x),now)
                if point is None:errors.append('cartesian_evaluation');break
        lateral=reference['frenet_path']['lateral']
        for s in np.linspace(*lateral['range'],11):
            point=api.evaluate_frenet_path(reference,float(s),now)
            if point is None:errors.append('frenet_evaluation');break
    except (ValueError,KeyError,TypeError) as error:
        errors.append('evaluation:'+str(error))
    return errors


def settings(configuration):
    # Sensor delivery uses measured .20s latency on the existing .05s plant step.
    return {'motion_max_interval_s':.4,**configuration.get('estimation',{})}


def probe(report, configuration, stations):
    rows=[]
    for station in stations:
        with environment(report,progress=float(station),offset=configuration.get('initial_offset_m',0.),
                         speed=configuration.get('initial_speed_mps',.1),estimation_settings=settings(configuration)) as env:
            planner=make_native(configuration)
            observation=(map_road_input(env,env._frame,lookahead_m=configuration.get('map_lookahead_m',2.),state_input=configuration.get('state_input','measured'))
                         if configuration.get('road_input')=='map_oracle' else env._frame)
            ref,diag,wall=query(planner,observation)
            rows.append(dict(progress_m=float(station),valid=ref['valid'],reason=ref['reason'],wall_s=wall,
                road_status=env._frame['estimation']['road']['status'],diagnostics=diag,reference=ref,
                contract_errors=contract(ref,env._frame['timestamp_s']) if ref['valid'] else []))
    return rows


def run(report,configuration,output,*,seed=0,control_period_s=.5,max_steps=1200):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    objective=CStarTrackingSettings()
    solver_cfg=SolverSettings(**configuration.get('solver',{}))
    solver=CStarSolver(objective,solver_cfg)
    planner=make_native(configuration)
    trace=[];solves=[];planner_timings=[];fault=None;completion=None;failed_frame=None
    start=time.perf_counter()
    with environment(report,seed=seed,progress=configuration.get('initial_progress_m',.3),
                     offset=configuration.get('initial_offset_m',0.),speed=configuration.get('initial_speed_mps',.1),
                     estimation_settings=settings(configuration),max_steps=max_steps+20) as env:
        initial=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        # Exact, predeclared finish inside measured support, with enough forward
        # corridor for the native minimum output and complete body clearance.
        from .pipeline import road_goal_progress
        goal=configuration.get('goal_progress_m',road_goal_progress(env))
        checker=NativePrediction(env,solver_cfg)
        period_steps=max(1,round(control_period_s/env.dt_s));controls=None;control_index=0;previous=(0.,0.)
        stream=(output/'trace.jsonl').open('w',encoding='utf-8',buffering=1)
        for step in range(max_steps):
            progress=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
            if progress>=goal:
                completion='declared_measured_corridor_goal_reached';break
            observation=(map_road_input(env,env._frame,lookahead_m=configuration.get('map_lookahead_m',2.),state_input=configuration.get('state_input','measured'))
                         if configuration.get('road_input')=='map_oracle' else env._frame)
            ref,diag,wall=query(planner,observation)
            planner_timings.append(wall)
            errors=contract(ref,env._frame['timestamp_s']) if ref['valid'] else []
            if not ref['valid'] or errors:
                fault='planning:'+str(ref['reason']) if not ref['valid'] else 'contract:'+','.join(errors)
                failed_frame=dict(observation=observation,reference=ref,diagnostics=diag,pose=list(env._backend.car._state_values()))
                break
            if configuration.get('enforce_planner_wall_clock',True) and wall>=env.dt_s:
                fault='planner_missed_20hz_deadline';failed_frame=dict(reference=ref,diagnostics=diag,wall_s=wall);break
            if controls is None or control_index>=period_steps or ref['planned_stop']:
                candidate=dict(candidate_id=str(diag.get('selected_candidate_id',ref['trajectory_id'])),reference=ref)
                solved=solver.solve(env,candidate,previous,seed=seed+step)
                solves.append(dict(step=step,timestamp_s=env._frame['timestamp_s'],reference=ref,
                    diagnostics=solved['diagnostics'],status=solved['status'],reason=solved['reason'],
                    full_plan=reference_plan(candidate,objective),loss=solved.get('loss'),margins=solved.get('margins')))
                print(json.dumps(dict(event='cstar',step=step,progress_m=progress,status=solved['status'],reason=solved['reason'],
                                      wall_s=solved['diagnostics']['wall_s']),allow_nan=False),flush=True)
                if solved['status']!='approximate':
                    fault='cstar:'+solved['reason'];failed_frame=dict(observation=env._frame,reference=ref,diagnostics=diag);break
                controls=solved['controls'];control_index=0
            action=np.asarray(controls[control_index],dtype=np.float32);control_index+=1
            # Planner reference is valid at this simulation tick. C* wall time is
            # excluded from simulated time, as in the existing offline pipeline.
            raw_before=env._backend.car._state_values()
            _,_,terminated,truncated,info=env.step(action)
            raw=env._backend.car._state_values();margins=checker.margins(raw,raw_before)
            row=dict(step=step,timestamp_s=env._frame['timestamp_s'],x_m=float(raw[0]),y_m=float(raw[1]),
                heading_rad=float(raw[2]),speed_mps=float(math.hypot(raw[3],raw[4])),
                progress_m=float(info['evaluation']['ground_truth']['reference_line_progress_m']),
                action=action.tolist(),planner_wall_s=wall,reference=ref,diagnostics=diag,
                road_status=env._frame['estimation']['road']['status'],constraint_margins=margins,
                contract_errors=errors,control_solution_step=solves[-1]['step'])
            trace.append(row);stream.write(json.dumps(json_safe(row),allow_nan=False)+'\n');previous=tuple(action)
            if min(margins.values()) < -1e-7:fault='execution_constraint_violation';break
            if terminated or truncated:fault='environment_finished';break
            if ref['planned_stop'] and row['speed_mps']<=objective.stop_speed_tolerance_mps:
                completion='planned_stop_complete';break
        else:fault='maximum_steps_reached'
        stream.close()
        final=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        import hashlib
        summary=dict(format='native_20hz_recorded_road_experiment_v1',configuration=configuration,seed=seed,
            experiment_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            planner_source_sha256=planner_source_sha256(),road_source_sha256=report['source_sha256'],
            environment_configuration=env.configuration(),native_planner=True,learned_scorer=False,
            resolved_planning_settings=asdict(planner.settings),resolved_lattice_settings=asdict(planner.cfg),
            road_input=configuration.get('road_input','camera_estimation'),perception_qualified=False,
            state_input=configuration.get('state_input','measured'),
            clearance_bound_scope=('continuous_geometry' if hasattr(type(planner),'_check_body_interval')
                                   else configuration.get('clearance_bound_scope','native_global')),
            configured_clearance_bound_scope=configuration.get('clearance_bound_scope','native_global'),
            body_clearance_method=('continuous_cubic_to_oriented_rectangle' if hasattr(type(planner),'_check_body_interval')
                                   else 'sampled_body_global_derivative_guards'),
            swept_body_check=('bounded_adaptive_pose_intervals' if hasattr(type(planner),'_check_body_interval')
                              else 'subtracted_spatial_sweep_guard'),
            cartesian_checks=planner.require_cartesian_compatibility,candidate_representation='frenet_native',
            allow_unobserved_origin=False,planner_wall_clock_enforced=configuration.get('enforce_planner_wall_clock',True),
            policy_rate_hz=1/env.dt_s,camera_rate_hz=1/(2*env.dt_s),camera_latency_s=4*env.dt_s,
            controller='existing_CStarSolver_full_path_solution_receding_horizon_prefix',control_period_s=control_period_s,
            controller_wall_time_counts_as_simulation_time=False,runtime_qualified=False,
            objective=objective.specification(),solver_settings=asdict(solver_cfg),
            initial_progress_m=initial,goal_progress_m=goal,final_progress_m=final,
            simulated_seconds=len(trace)*env.dt_s,steps=len(trace),fault=fault,completion=completion,
            completed=fault is None and completion=='declared_measured_corridor_goal_reached',
            valid_path_count=len(trace),invalid_path_count=int(fault is not None and fault.startswith('planning:')),
            path_contract_error_count=sum(len(r['contract_errors']) for r in trace),
            planner_wall_max_s=max((r['planner_wall_s'] for r in trace),default=0),
            all_planner_queries=len(planner_timings),all_planner_wall_max_s=max(planner_timings,default=0),
            planner_wall_p95_s=float(np.percentile([r['planner_wall_s'] for r in trace],95)) if trace else None,
            wall_s=time.perf_counter()-start,cstar_solves=len(solves),
            minimum_constraint_margins={k:min(r['constraint_margins'][k] for r in trace) for k in trace[0]['constraint_margins']} if trace else {},
            road_complete=False,goal_definition=('explicit_configuration_goal' if 'goal_progress_m' in configuration
                else 'existing_pipeline_road_goal_progress'))
        write_json(output/'summary.json',summary);write_json(output/'solves.json',solves)
        if failed_frame:write_json(output/'failure.json',failed_frame)
        print(json.dumps({k:summary[k] for k in ('completed','steps','initial_progress_m','final_progress_m','goal_progress_m','fault','wall_s')},allow_nan=False),flush=True)
        return summary


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--road',required=True);parser.add_argument('--config');parser.add_argument('--output',required=True)
    parser.add_argument('--probe',action='store_true');parser.add_argument('--stations',default='.3,.8,1.3,1.8,2.3')
    parser.add_argument('--seed',type=int,default=0);parser.add_argument('--control-period-s',type=float,default=.5)
    args=parser.parse_args();report=json.loads(Path(args.road).read_text(encoding='utf-8'))
    config=json.loads(Path(args.config).read_text(encoding='utf-8')) if args.config else {}
    if args.probe:
        rows=probe(report,config,[float(s) for s in args.stations.split(',')]);write_json(Path(args.output)/'probes.json',rows)
        print(json.dumps([{k:r[k] for k in ('progress_m','valid','reason','wall_s','road_status','contract_errors')} for r in rows]))
    else:
        summary=run(report,config,args.output,seed=args.seed,control_period_s=args.control_period_s)
        if not summary['completed']:raise SystemExit(2)


if __name__=='__main__':main()
