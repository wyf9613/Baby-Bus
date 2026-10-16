"""Simulation sensor/clock/delivery ports for the shared modular MVP2.1 execution."""
from collections import Counter
import json
import math
from pathlib import Path
import time

from mvp21_policy.contracts import required_sensor_problem
from mvp21_policy.control import PolicyController, ControlSettings
from mvp21_policy.execution import PlannerControlSettings, PlannerExecutionManager, calculate_selected_control, priming_motion_history
from mvp21_policy.geometry import finite_number
from mvp21_policy.lattice import ShadowSettings
from mvp21_policy.planning import RoadCenterlinePlanner, PlanningSettings
from mvp21_policy.transport import ShadowJobSource
from offline.lattice_training.cstar import NativePrediction, SolverSettings
from offline.lattice_training.dataset import json_safe, write_json
from offline.lattice_training.pipeline import road_goal_progress
from .delivery import SimulatedPlannerWorker
from .simulation import recorded_environment, ideal_observation, plant_action
from .source import (parameters, runtime_provenance, verify_preserved_files, UPSTREAM_COMMIT,
                     SOURCE_SHA256, CONFIG_SHA256, UPSTREAM_CONFIG_SHA256)


def run_runtime(road, output, *, road_input='perception', max_steps=1200, seed=0, progress=None,
                initial_speed=0., latency_steps=4, steering_sign=-1., path_source=None,
                shadow_enabled=True, delivery_delay_s=.1, measured_delay=False, handoff_budget_s=None,
                collision_coverage=None, score_coverage=None, lattice_length_m=None, planner_rate_hz=None):
    preserved, config = verify_preserved_files(), parameters()
    if path_source is None:
        path_source = 'lattice' if config['planner_control']['enabled'] else 'perception'
    if path_source not in ('lattice', 'perception') or road_input not in ('perception', 'map'):
        raise ValueError('Unknown path source or road input')
    requested = path_source == 'lattice'
    if requested and not shadow_enabled:
        raise ValueError('Native execution requires the planner to be enabled')
    settings = ShadowSettings(**config['shadow'])
    for name, value in (('collision_coverage', collision_coverage), ('score_coverage', score_coverage),
                        ('length_m', lattice_length_m), ('rate_hz', planner_rate_hz)):
        if value is not None:
            setattr(settings, name, value)
    settings.__post_init__()
    if requested and not settings.enabled:
        raise ValueError('Native execution requires shadow.enabled')
    if config['control']['robustness_enabled'] or config['control']['end_straight_distance_m']:
        raise ValueError('This port does not emulate optional recovery lifecycle modes')
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
    manager = PlannerExecutionManager(generation=1)
    worker = SimulatedPlannerWorker(delay_s=delivery_delay_s, measured=measured_delay)
    offer = ShadowJobSource()
    offer.planner_control_settings = PlannerControlSettings(enabled=requested)
    offer.shadow_settings, offer.planning_settings = settings, planner.settings
    offer.shadow_last_submit_s, offer.shadow_generation, offer.shadow_job_id = -math.inf, 1, 0
    offer.shadow_worker, offer._monotonic = worker, lambda: worker.now_s
    rows, submissions, statuses, modes, reasons = [], [], Counter(), Counter(), Counter()
    fault = failure = None
    accepted = startup_baseline = after_native_fallback = 0
    ever_native = False
    has_control = False
    last_nonempty_stamp = None
    started = time.perf_counter()
    with recorded_environment(road, seed=seed, progress=progress, speed=initial_speed,
            latency_steps=latency_steps, max_steps=max_steps+latency_steps+20) as env:
        if not math.isclose(env.dt_s, 1/config['policy_update_rate_hz'], abs_tol=1e-12):
            raise ValueError('Plant and control periods differ')
        budget = handoff_budget_s if handoff_budget_s is not None else math.ceil(delivery_delay_s/env.dt_s-1e-9)*env.dt_s
        offer.shadow_handoff_estimate_s = max(settings.handoff_allowance_s, budget)
        offer.motion_history = env.estimator.motion
        checker = NativePrediction(env, SolverSettings(safety_margin_m=0.))
        initial = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        goal = road_goal_progress(env)
        source_timeouts = dict(road=min(config['estimation']['max_source_age_s'], config['sensor_timeout_s']['cone_detections']),
            **{name: min(config['estimation']['max_source_age_s'], config['estimation']['motion_max_gap_s'],
                config['sensor_timeout_s'][name]) for name in ('wheel_speed', 'imu_angular_velocity')})
        with (output/'trace.jsonl').open('w', encoding='utf-8') as trace_stream, \
                (output/'shadow.jsonl').open('w', encoding='utf-8') as planner_stream:
            for step in range(max_steps):
                if float(env._info['evaluation']['ground_truth']['reference_line_progress_m']) >= goal:
                    break
                observation = env._frame if road_input == 'perception' else ideal_observation(env, env._frame)
                estimates, now = observation['estimation'], observation['timestamp_s']
                ages = observation['sensor_age_s']
                fresh = {name: observation['sensor_available'].get(name, False)
                    and finite_number(ages.get(name)) and ages[name] < config['sensor_timeout_s'][name]
                    for name in config['required_sensors']}
                cones = observation['sensors'].get('cone_detections') or {}
                if cones.get('detections') and finite_number(ages.get('cone_detections')):
                    last_nonempty_stamp = now-ages['cone_detections']
                problem = required_sensor_problem(config['required_sensors'], fresh,
                    None if last_nonempty_stamp is None else now-last_nonempty_stamp, config['cone_empty_timeout_s'])
                if problem:
                    fault, failure = 'sensors:'+problem, dict(step=step, observation=observation)
                    break
                cycle_started = time.perf_counter()
                baseline, diagnostics = planner.plan(estimates['road'], estimates['state'], estimates['obstacles'],
                    estimates['vehicle_limits'], now, source_timeouts, mvp=True)
                packet = worker.poll(now)
                if packet is not None:
                    result = packet['result']
                    statuses[result['status']] += 1
                    planner_stream.write(json.dumps(json_safe(dict(step=step, result=result,
                        delivery={k: v for k, v in packet.items() if k not in ('job', 'result')},
                        decision_pose=list(env._backend.car._state_values()[:3]))), allow_nan=False)+'\n')
                    if manager.accept(result):
                        accepted += 1
                        guide = (result.get('fusion') or {}).get('guide')
                        if (result.get('status') == 'VALID' and guide
                                and guide.get('odometry_epoch') == getattr(offer, 'shadow_odometry_epoch', 0)):
                            offer.shadow_previous_reference = guide
                        offer.shadow_handoff_estimate_s = max(offer.shadow_handoff_estimate_s, packet['round_trip_s'])
                worker.now_s, offer.estimation_output = now, estimates
                native = manager.prepare(estimates, baseline, offer, settings, planner.settings,
                    controller.settings, config['policy_update_rate_hz']) if requested else None
                if not requested:
                    manager.diagnostics = dict(mode='baseline', fallback=False, reason='baseline_selected')
                priming = native is None and priming_motion_history(baseline, estimates, has_control, step*env.dt_s, controller.settings)
                if native is None and not baseline['valid'] and not priming:
                    fault, failure = 'planning:'+baseline['reason'], dict(step=step, observation=observation,
                        baseline=baseline, execution=dict(manager.diagnostics))
                    break
                if priming:
                    drive = steer = 0.
                    reference = None
                    control = dict(valid=False, stop_requested=False, reason='priming_motion_history')
                    manager.diagnostics = dict(mode='priming', fallback=False, reason='priming_motion_history')
                else:
                    controller, drive, steer, control, reference = calculate_selected_control(
                        controller, baseline, native, estimates['state'], estimates['vehicle_params'],
                        now, 0. if step == 0 else env.dt_s, manager if requested else None)
                    has_control = control['valid'] and not control['stop_requested']
                if not priming and (not control['valid'] or control['stop_requested']):
                    fault, failure = 'control:'+control['reason'], dict(step=step, reference=reference, control=control)
                    break
                mode = manager.diagnostics['mode']
                modes[mode] += 1
                if requested and mode == 'baseline':
                    reasons[manager.diagnostics['reason']] += 1
                    if ever_native:
                        after_native_fallback += 1
                    else:
                        startup_baseline += 1
                ever_native |= mode == 'planner'
                # Like ROS, offer only after selecting the action. Snapshot
                # collection is idempotent at the current state timestamp.
                if shadow_enabled and settings.enabled:
                    identifier = offer.shadow_job_id
                    offer._shadow_offer()
                    if offer.shadow_job_id != identifier:
                        submissions.append(now)
                action = plant_action(drive, steer, actuator_steering_sign=steering_sign)
                control_wall = time.perf_counter()-cycle_started
                raw_before = env._backend.car._state_values()
                _, _, terminated, truncated, info = env.step(action)
                raw = env._backend.car._state_values()
                margins = checker.margins(raw, raw_before)
                row = dict(step=step, timestamp_s=now, progress_m=float(info['evaluation']['ground_truth']['reference_line_progress_m']),
                    decision_pose=list(raw_before[:3]), x_m=float(raw[0]), y_m=float(raw[1]), heading_rad=float(raw[2]),
                    speed_mps=math.hypot(raw[3], raw[4]), raw_action=[drive, steer], plant_action=action,
                    reference=reference, planning=diagnostics, control=control, state=estimates['state'],
                    execution=dict(manager.diagnostics), road_status=estimates['road']['status'],
                    road_visibility=estimates['road'].get('visibility'), road_boundary_source=estimates['road'].get('boundary_source'),
                    planner_controller_wall_s=control_wall,
                    plant_corner_road_margin_m=margins['road_clearance_m'])
                rows.append(row)
                trace_stream.write(json.dumps(json_safe(row), allow_nan=False)+'\n')
                if margins['road_clearance_m'] < -1e-7:
                    fault, failure = 'execution:plant_body_corner_outside_road', dict(step=step, pose=list(raw), margins=margins)
                    break
                if terminated or truncated:
                    fault = 'environment_finished'
                    break
            else:
                fault = 'maximum_steps_reached'
        if fault is not None:
            controller.clear()
            failure = failure or dict(fault=fault)
            failure.update(stop_requested=True, stop_action=[0., 0.])
            write_json(output/'failure.json', failure)
        final = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        completed = fault is None and final >= goal
        summary = dict(format='mvp21_shared_execution_v1', checkpoint=preserved, upstream_commit=UPSTREAM_COMMIT,
            source_sha256=SOURCE_SHA256, config_sha256=CONFIG_SHA256, upstream_config_sha256=UPSTREAM_CONFIG_SHA256,
            runtime_modules=runtime_provenance(), runtime_parameters=config, environment=env.configuration(),
            road_source_sha256=road['source_sha256'], road_input=road_input, path_source=path_source,
            lattice_generation_rate_hz=settings.rate_hz, control_rate_hz=config['policy_update_rate_hz'],
            initial_progress_m=initial, final_progress_m=final, goal_progress_m=goal, steps=len(rows),
            completed=completed, fault=fault, valid_reference_count=len(rows), simulated_seconds=len(rows)*env.dt_s,
            planner_reference_count=modes['planner'], baseline_reference_count=modes['baseline'],
            startup_baseline_cycles=startup_baseline, fallback_after_first_native_cycles=after_native_fallback,
            zero_action_priming_cycles=modes['priming'], fallback_reasons=dict(reasons),
            native_only_passed=completed and modes['planner'] > 0 and modes['baseline'] == 0 and modes['priming'] == 0,
            delivery=dict(mode='measured_cpu_plus_delay' if measured_delay else 'fixed_simulated_delay',
                delay_s=delivery_delay_s, submitted=len(submissions), accepted=accepted, submission_times_s=submissions),
            shadow=dict(enabled=shadow_enabled and settings.enabled, calls=len(worker.completed),
                status_counts=dict(statuses), influences_actions=requested, effective_settings=vars(settings)),
            maximum_executed_speed_mps=max((r['speed_mps'] for r in rows), default=initial_speed),
            target_speed_mps=config['planning']['cruise_speed_mps'],
            minimum_plant_corner_road_margin_m=min((r['plant_corner_road_margin_m'] for r in rows), default=None),
            wall_s=time.perf_counter()-started, learned_scorer_used=False, real_time_qualified=False,
            hardware_tested=False, deployed=False, speed_tracking_qualified=False)
        write_json(output/'summary.json', summary)
    verify_preserved_files()
    print(json.dumps({key: summary[key] for key in ('completed', 'steps', 'final_progress_m', 'fault',
        'planner_reference_count', 'baseline_reference_count', 'native_only_passed')}), flush=True)
    return summary
