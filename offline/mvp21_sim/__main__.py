"""20 Hz offline MVP2.1 perception/planning/control and independent shadow probe."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

from offline.lattice_training.cstar import NativePrediction, SolverSettings
from offline.lattice_training.dataset import json_safe, write_json
from offline.lattice_training.pipeline import road_goal_progress
from .source import api, parameters, verify_preserved_files, UPSTREAM_COMMIT, SOURCE_SHA256, CONFIG_SHA256, ROOT
from .simulation import recorded_environment, ideal_observation, plant_action

DEFAULT_ROAD = ROOT/'offline/lattice_training/artifacts/2026-10-15-native-diagnosis/recorded-road.json'


def run(road, output, *, road_input='perception', max_steps=1200, seed=0, progress=None,
        initial_speed=0., latency_steps=4, steering_sign=-1., shadow_enabled=True):
    preserved = verify_preserved_files()
    active, config = api(), parameters()
    if config['planning']['algorithm'] != 'road_centerline' or config['control']['mode'] != 'mvp':
        raise ValueError('This adapter verifies the pinned road_centerline/MVP chain only')
    if config['control']['robustness_enabled'] or config['control']['end_straight_distance_m']:
        raise ValueError('A changed upstream recovery mode requires its lifecycle adapter')
    output = Path(output)
    if output.exists():
        raise FileExistsError('Use a new output directory: '+str(output))
    output.mkdir(parents=True)
    planner = active.RoadCenterlinePlanner(active.PlanningSettings(**config['planning']))
    controller = active.PolicyController(active.ControlSettings(**config['control']))
    shadow_cfg = active.ShadowSettings(**config['shadow'])
    shadow = active.ShadowLatticePlanner(shadow_cfg)
    jobs = []
    offer = active.ShadowJobSource()
    offer.shadow_settings, offer.planning_settings = shadow_cfg, planner.settings
    offer.shadow_last_submit_s, offer.shadow_handoff_estimate_s = -math.inf, 0.
    offer.shadow_generation, offer.shadow_job_id = 1, 0
    offer.shadow_worker = SimpleNamespace(submit=jobs.append)
    offer._monotonic = lambda: offer.clock_s
    traces, statuses, control_times = [], Counter(), []
    fault, failure, shadow_count = None, None, 0
    started = time.perf_counter()
    with recorded_environment(road, seed=seed, progress=progress, speed=initial_speed,
            latency_steps=latency_steps, max_steps=max_steps+latency_steps+20) as env:
        if not math.isclose(env.dt_s, 1/config['policy_update_rate_hz'], abs_tol=1e-12):
            raise ValueError('Plant step and upstream policy frequency differ')
        checker = NativePrediction(env, SolverSettings(safety_margin_m=0.))
        initial = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        goal = road_goal_progress(env)
        source_timeouts = dict(road=min(config['estimation']['max_source_age_s'],config['sensor_timeout_s']['cone_detections']),
            **{name:min(config['estimation']['max_source_age_s'],config['estimation']['motion_max_gap_s'],
                config['sensor_timeout_s'][name]) for name in ('wheel_speed','imu_angular_velocity')})
        with (output/'trace.jsonl').open('w',encoding='utf-8') as stream, \
                (output/'shadow.jsonl').open('w',encoding='utf-8') as shadow_stream:
            for step in range(max_steps):
                current = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
                if current >= goal:
                    break
                observation = env._frame if road_input=='perception' else ideal_observation(env,env._frame)
                estimates, now = observation['estimation'], observation['timestamp_s']
                cycle_started = time.perf_counter()
                ref, diagnostics = planner.plan(estimates['road'],estimates['state'],estimates['obstacles'],
                    estimates['vehicle_limits'],now,source_timeouts,mvp=True)
                if not ref['valid']:
                    fault = 'planning:'+str(ref['reason'])
                    failure = dict(observation=observation,reference=ref,diagnostics=diagnostics)
                    break
                drive, steer, control = controller.calculate(ref,estimates['state'],estimates['vehicle_params'],
                    now,0. if step==0 else env.dt_s)
                control_wall = time.perf_counter()-cycle_started
                control_times.append(control_wall)
                if not control['valid'] or control['stop_requested']:
                    fault = 'control:'+control['reason']
                    failure = dict(observation=observation,reference=ref,diagnostics=control)
                    break
                if shadow_enabled and shadow_cfg.enabled:
                    # Offline synchronous geometry probe, separate from actions.
                    # ROS worker/process/queue transport is not emulated here.
                    offer.estimation_output, offer.clock_s = estimates, now
                    offer._shadow_offer()
                if jobs:
                    job = jobs.pop()
                    try:
                        result = shadow.plan(job)
                    except Exception as error:
                        result = dict(status='ERROR',error=type(error).__name__+':'+str(error))
                    statuses[result['status']] += 1
                    shadow_count += 1
                    shadow_stream.write(json.dumps(json_safe(dict(step=step,result=result,
                        decision_pose=list(env._backend.car._state_values()[:3]))),allow_nan=False)+'\n')
                action = plant_action(drive,steer,actuator_steering_sign=steering_sign)
                raw_before = env._backend.car._state_values()
                _, _, terminated, truncated, info = env.step(action)
                raw = env._backend.car._state_values()
                margins = checker.margins(raw,raw_before)
                row = dict(step=step,timestamp_s=now,progress_m=float(info['evaluation']['ground_truth']['reference_line_progress_m']),
                    decision_pose=list(raw_before[:3]),
                    x_m=float(raw[0]),y_m=float(raw[1]),heading_rad=float(raw[2]),speed_mps=math.hypot(raw[3],raw[4]),
                    raw_action=[drive,steer],plant_action=action,reference=ref,planning=diagnostics,control=control,
                    road_status=estimates['road']['status'],road_visibility=estimates['road'].get('visibility'),
                    road_boundary_source=estimates['road'].get('boundary_source'),state=estimates['state'],
                    planner_controller_wall_s=control_wall,plant_corner_road_margin_m=margins['road_clearance_m'])
                traces.append(row)
                stream.write(json.dumps(json_safe(row),allow_nan=False)+'\n')
                if margins['road_clearance_m'] < -1e-7:
                    fault = 'execution:plant_body_corner_outside_road'
                    failure = dict(observation=observation,reference=ref,control=control,pose=list(raw),margins=margins)
                    break
                if terminated or truncated:
                    fault = 'environment_finished'
                    break
            else:
                fault = 'maximum_steps_reached'
        final = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        summary = dict(format='mvp21_preserved_simulation_v1',checkpoint=preserved,upstream_commit=UPSTREAM_COMMIT,
            source_sha256=SOURCE_SHA256,config_sha256=CONFIG_SHA256,road_source_sha256=road['source_sha256'],
            upstream_parameters=config,environment=env.configuration(),road_input=road_input,
            initial_progress_m=initial,final_progress_m=final,goal_progress_m=goal,steps=len(traces),fault=fault,
            completed=fault is None and final>=goal,simulated_seconds=len(traces)*env.dt_s,
            wall_s=time.perf_counter()-started,valid_reference_count=len(traces),
            planner_controller_wall_max_s=max(control_times,default=0.),
            planner_controller_over_50ms=sum(t>env.dt_s for t in control_times),
            maximum_executed_speed_mps=max((r['speed_mps'] for r in traces),default=initial_speed),
            target_speed_mps=config['planning']['cruise_speed_mps'],
            speed_mean_absolute_error_mps=sum(abs(r['speed_mps']-config['planning']['cruise_speed_mps'])
                for r in traces)/len(traces) if traces else None,
            minimum_plant_corner_road_margin_m=min((r['plant_corner_road_margin_m'] for r in traces),default=None),
            actuator_mapping=dict(steering_sign=steering_sign,drive_gain=1.,drive_offset=0.,
                explanation='Upstream car positive steering is right; pinned plant positive steering is left.'),
            shadow=dict(enabled=shadow_enabled and shadow_cfg.enabled,calls=shadow_count,status_counts=dict(statuses),
                influences_actions=False,transport='synchronous offline geometry probe; no ROS worker transport'),
            policy_module_overwritten=False,learned_scorer_used=False,
            limitations=['No ROS, hardware, enabling or real-time qualification.',
                'Physical car drive feedforward/gains remain unchanged on the course simulation plant.',
                'Plant corner margin is a diagnostic, not continuous full-body sweep certification.',
                'Shadow result is never connected to control; shadow dimensions are upstream assumptions.'])
        write_json(output/'summary.json',summary)
        if failure is not None:
            write_json(output/'failure.json',failure)
    verify_preserved_files()
    print(json.dumps({k:summary[k] for k in ('completed','steps','initial_progress_m','final_progress_m','fault')}) ,flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--road',type=Path,default=DEFAULT_ROAD)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--road-input',choices=('perception','map'),default='perception')
    parser.add_argument('--max-steps',type=int,default=1200)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--progress',type=float)
    parser.add_argument('--initial-speed',type=float,default=0.)
    parser.add_argument('--latency-steps',type=int,default=4)
    parser.add_argument('--steering-sign',type=float,choices=(-1.,1.),default=-1.)
    parser.add_argument('--no-shadow',action='store_true')
    args = parser.parse_args()
    if args.max_steps<1 or args.latency_steps<0 or args.initial_speed<0:
        parser.error('Positive max_steps and nonnegative latency/speed required')
    result = run(json.loads(args.road.read_text(encoding='utf-8')),args.output,road_input=args.road_input,
        max_steps=args.max_steps,seed=args.seed,progress=args.progress,initial_speed=args.initial_speed,
        latency_steps=args.latency_steps,steering_sign=args.steering_sign,shadow_enabled=not args.no_shadow)
    return 0 if result['completed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
