"""One-Hz lattice delivery and 20-Hz control with measured-motion path reuse."""
from collections import Counter
import json
import math
from pathlib import Path
import time

from mvp21_policy.control import PolicyController, ControlSettings
from mvp21_policy.lattice import ShadowSettings
from mvp21_policy.planning import RoadCenterlinePlanner, PlanningSettings
from mvp21_policy.reference import LatticeReferenceManager
from mvp21_policy.transport import ShadowJobSource
from offline.lattice_training.cstar import NativePrediction, SolverSettings
from offline.lattice_training.dataset import json_safe, write_json
from offline.lattice_training.pipeline import road_goal_progress
from .delivery import SimulatedPlannerWorker
from .simulation import recorded_environment, ideal_observation, plant_action
from .source import parameters, runtime_provenance, verify_preserved_files, UPSTREAM_COMMIT, SOURCE_SHA256, CONFIG_SHA256


def run_cached(road, output, *, road_input='perception', max_steps=1200, seed=0, progress=None,
        initial_speed=0., latency_steps=4, steering_sign=-1., delivery_delay_s=.1,
        measured_delay=False, handoff_budget_s=None, lattice_coverage='generated', lattice_length_m=1.3):
    preserved, config = verify_preserved_files(), parameters()
    settings = ShadowSettings(**config['shadow'])
    settings.coverage_mode = lattice_coverage
    if lattice_length_m is not None:
        settings.length_m = lattice_length_m
    settings.__post_init__()
    if not settings.enabled or settings.rate_hz != 1.:
        raise ValueError('This experiment requires the original enabled 1 Hz planner')
    if measured_delay and handoff_budget_s is None:
        raise ValueError('Measured service needs an explicit profiled handoff budget')
    if handoff_budget_s is not None and (not math.isfinite(handoff_budget_s) or handoff_budget_s < 0):
        raise ValueError('Nonnegative handoff budget required')
    output = Path(output)
    if output.exists():
        raise FileExistsError('Use a new output directory: '+str(output))
    output.mkdir(parents=True)
    planner = RoadCenterlinePlanner(PlanningSettings(**config['planning']))
    controller = PolicyController(ControlSettings(**config['control']))
    manager = LatticeReferenceManager(max_source_age_s=config['planning']['max_source_age_s'])
    worker = SimulatedPlannerWorker(delay_s=delivery_delay_s,measured=measured_delay)
    offer = ShadowJobSource()
    offer.shadow_settings, offer.planning_settings = settings,planner.settings
    offer.shadow_last_submit_s = -math.inf
    offer.shadow_generation, offer.shadow_job_id = 1,0
    offer.shadow_worker = worker
    offer._monotonic = lambda: worker.now_s
    rows, submission_times, statuses = [],[],Counter()
    fault = failure = None
    first_submit_s = None
    accepted, priming = 0,0
    has_control = False
    started = time.perf_counter()
    with recorded_environment(road,seed=seed,progress=progress,speed=initial_speed,
            latency_steps=latency_steps,max_steps=max_steps+latency_steps+20) as env:
        # A fixed delay is a declared experiment assumption, rounded to delivery
        # polling ticks. Measured CPU service instead needs an explicit budget.
        budget = handoff_budget_s if handoff_budget_s is not None else math.ceil(delivery_delay_s/env.dt_s-1e-9)*env.dt_s
        offer.shadow_handoff_estimate_s = max(settings.handoff_allowance_s,budget)
        checker = NativePrediction(env,SolverSettings(safety_margin_m=0.))
        initial = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        goal = road_goal_progress(env)
        source_timeouts = dict(road=min(config['estimation']['max_source_age_s'],config['sensor_timeout_s']['cone_detections']),
            **{name:min(config['estimation']['max_source_age_s'],config['estimation']['motion_max_gap_s'],
                config['sensor_timeout_s'][name]) for name in ('wheel_speed','imu_angular_velocity')})
        with (output/'trace.jsonl').open('w',encoding='utf-8') as trace_stream, \
                (output/'shadow.jsonl').open('w',encoding='utf-8') as planner_stream:
            for step in range(max_steps):
                current = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
                if current >= goal:
                    break
                observation = env._frame if road_input=='perception' else ideal_observation(env,env._frame)
                estimates,now = observation['estimation'],observation['timestamp_s']
                cycle_started = time.perf_counter()
                envelope,diagnostics = planner.plan(estimates['road'],estimates['state'],estimates['obstacles'],
                    estimates['vehicle_limits'],now,source_timeouts,mvp=True)
                ref = None
                try:
                    if not envelope['valid']:
                        raise ValueError('admission:'+str(envelope['reason']))
                    packet = worker.poll(now)
                    if packet is not None:
                        result = packet['result']
                        statuses[result['status']] += 1
                        planner_stream.write(json.dumps(json_safe(dict(step=step,result=result,
                            delivery={k:v for k,v in packet.items() if k not in ('job','result')},
                            decision_pose=list(env._backend.car._state_values()[:3]))),allow_nan=False)+'\n')
                        accepted += manager.accept(result,now,env.estimator.motion)
                        offer.shadow_handoff_estimate_s = max(offer.shadow_handoff_estimate_s,packet['round_trip_s'])
                    worker.now_s = now
                    offer.estimation_output = estimates
                    previous_id = offer.shadow_job_id
                    offer._shadow_offer()
                    if offer.shadow_job_id != previous_id:
                        submission_times.append(now)
                        if first_submit_s is None:
                            first_submit_s = now
                    if manager.points is None:
                        if now-first_submit_s >= 1/settings.rate_hz+offer.shadow_handoff_estimate_s:
                            raise ValueError('lattice_startup_delivery_timeout')
                        drive,steer = 0.,0.
                        control = dict(valid=False,stop_requested=False,reason='awaiting_first_lattice')
                        priming += 1
                    else:
                        ref = manager.select(envelope,estimates['state'],env.estimator.motion,now)
                        drive,steer,control = controller.calculate(ref,estimates['state'],estimates['vehicle_params'],
                            now,env.dt_s if has_control else 0.)
                        if not control['valid'] or control['stop_requested']:
                            raise ValueError('control:'+control['reason'])
                        has_control = True
                except (ValueError,TypeError,ArithmeticError) as error:
                    fault = str(error)
                    failure = dict(step=step,observation=observation,reference=ref,
                        admission=envelope,cache=manager.diagnostics,
                        last_result=manager.result,fault=fault)
                    break
                control_wall = time.perf_counter()-cycle_started
                action = plant_action(drive,steer,actuator_steering_sign=steering_sign)
                raw_before = env._backend.car._state_values()
                _,_,terminated,truncated,info = env.step(action)
                raw = env._backend.car._state_values()
                margins = checker.margins(raw,raw_before)
                row = dict(step=step,timestamp_s=now,progress_m=float(info['evaluation']['ground_truth']['reference_line_progress_m']),
                    decision_pose=list(raw_before[:3]),x_m=float(raw[0]),y_m=float(raw[1]),heading_rad=float(raw[2]),
                    speed_mps=math.hypot(raw[3],raw[4]),raw_action=[drive,steer],plant_action=action,
                    reference=ref,planning=diagnostics,control=control,state=estimates['state'],
                    road_status=estimates['road']['status'],road_visibility=estimates['road'].get('visibility'),
                    road_boundary_source=estimates['road'].get('boundary_source'),
                    cache=dict(manager.diagnostics),planner_controller_wall_s=control_wall,
                    plant_corner_road_margin_m=margins['road_clearance_m'])
                rows.append(row)
                trace_stream.write(json.dumps(json_safe(row),allow_nan=False)+'\n')
                if margins['road_clearance_m'] < -1e-7:
                    fault = 'execution:plant_body_corner_outside_road'
                    failure = dict(step=step,observation=observation,reference=ref,control=control,pose=list(raw),margins=margins)
                    break
                if terminated or truncated:
                    fault = 'environment_finished'
                    break
            else:
                fault = 'maximum_steps_reached'
        if fault is not None:
            # Model the stop REQUEST explicitly. Neutral effort is not a proof
            # of braking or physical standstill on the vehicle.
            controller.clear()
            failure = failure or dict(fault=fault)
            failure.update(stop_requested=True,stop_action=[0.,0.])
            if fault != 'environment_finished':
                env.step((0.,0.))
                failure['after_zero_action_pose'] = list(env._backend.car._state_values())
        final = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        summary = dict(format='mvp21_one_hz_cached_v1',checkpoint=preserved,upstream_commit=UPSTREAM_COMMIT,
            source_sha256=SOURCE_SHA256,config_sha256=CONFIG_SHA256,runtime_modules=runtime_provenance(),
            road_source_sha256=road['source_sha256'],upstream_parameters=config,environment=env.configuration(),
            road_input=road_input,path_source='lattice',lattice_cadence='cached',lattice_generation_rate_hz=1.,
            delivery=dict(mode='measured_cpu_plus_delay' if measured_delay else 'fixed_simulated_delay',
                delay_s=delivery_delay_s,initial_handoff_budget_s=budget,
                final_handoff_estimate_s=offer.shadow_handoff_estimate_s,
                submission_times_s=submission_times,submitted=len(submission_times),accepted=accepted,
                maximum_round_trip_s=max((p['round_trip_s'] for p in worker.completed),default=None)),
            initial_progress_m=initial,final_progress_m=final,goal_progress_m=goal,steps=len(rows),fault=fault,
            completed=fault is None and final>=goal,simulated_seconds=len(rows)*env.dt_s,
            wall_s=time.perf_counter()-started,valid_reference_count=sum(r['reference'] is not None for r in rows),
            zero_action_startup_cycles=priming,maximum_executed_speed_mps=max((r['speed_mps'] for r in rows),default=initial_speed),
            target_speed_mps=config['planning']['cruise_speed_mps'],
            speed_mean_absolute_error_mps=sum(abs(r['speed_mps']-config['planning']['cruise_speed_mps'])
                for r in rows)/len(rows) if rows else None,
            minimum_plant_corner_road_margin_m=min((r['plant_corner_road_margin_m'] for r in rows),default=None),
            shadow=dict(enabled=True,calls=len(worker.completed),status_counts=dict(statuses),influences_actions=True,
                effective_settings=vars(settings),transport='discrete-event service; vehicle advances until delivery'),
            policy_module_overwritten=False,learned_scorer_used=False,
            real_time_qualified=False,hardware_tested=False,deployed=False,speed_tracking_qualified=False,
            limitations=['Delivery is a causal simulation model, not ROS executor real-time qualification.',
                'Unchanged physical drive feedforward and PID on the course plant; speed tracking not qualified.',
                'Four plant corner diagnostic, not continuous full-body proof; terminal stopping not tested.',
                'VALID retains upstream known-boundary prefix semantics; cache validity is not collision certification.'])
        write_json(output/'summary.json',summary)
        if failure is not None:
            write_json(output/'failure.json',failure)
    verify_preserved_files()
    print(json.dumps({k:summary[k] for k in ('completed','steps','final_progress_m','fault')}),flush=True)
    return summary
