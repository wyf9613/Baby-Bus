"""Simulation sensor/clock/delivery ports for the shared modular MVP2.1 execution."""
from collections import Counter
from copy import deepcopy
import hashlib
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
from .simulation import recorded_environment, fixed_road_environment, ideal_observation, plant_action
from .source import (parameters, runtime_provenance, verify_preserved_files, UPSTREAM_COMMIT,
                     SOURCE_SHA256, CONFIG_SHA256, UPSTREAM_CONFIG_SHA256)


def run_runtime(road, output, *, road_input='perception', max_steps=1200, seed=0, progress=None,
                initial_speed=0., latency_steps=4, steering_sign=-1., path_source=None,
                shadow_enabled=True, delivery_delay_s=.1, measured_delay=False, handoff_budget_s=None,
                collision_coverage=None, score_coverage=None, lattice_length_m=None, planner_rate_hz=None,
                detailed_trace=False, stop_tail_s=0., terminal_support_m=2., policy_model_path=None,validation_config=None):
    preserved, config = verify_preserved_files(), parameters()
    modular=validation_config is not None
    if modular:
        from .validation_config import configuration
        validation_config=configuration(validation_config);rates=validation_config['frequencies_hz']
        road_input={'observed':'perception','perfect':'map','no_centerline':'no_centerline'}[validation_config['modules']['perception']]
        max_steps=validation_config['max_steps'];stop_tail_s=validation_config['stop_tail_s']
        latency_steps=round(validation_config['camera_latency_s']*rates['physics'])
        delivery_delay_s=validation_config['planner_latency_s'];terminal_support_m=validation_config['terminal_support_m']
        planner_rate_hz=rates['planner'];config['policy_update_rate_hz']=rates['control']
        config['planning']['cruise_speed_mps']=validation_config['cruise_speed_mps']
        policy_model_path=validation_config['model_path'] if validation_config['modules']['policy']=='model' else None
    if path_source is None:
        path_source = 'lattice' if config['planner_control']['enabled'] else 'perception'
    if path_source not in ('lattice', 'perception') or road_input not in ('perception', 'map', 'no_centerline'):
        raise ValueError('Unknown path source or road input')
    requested = path_source == 'lattice'
    if policy_model_path is not None and not requested:
        raise ValueError('Policy ranking requires the native candidate execution path')
    selector=None
    if policy_model_path is not None:
        from .policy_selection import FrozenPolicySelector
        selector=FrozenPolicySelector(policy_model_path,cruise_speed_mps=config['planning']['cruise_speed_mps'])
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
    if not math.isfinite(stop_tail_s) or stop_tail_s < 0:
        raise ValueError('Nonnegative zero-publication tail required')
    output = Path(output)
    if output.exists():
        raise FileExistsError('Use a new output directory: '+str(output))
    output.mkdir(parents=True)
    planner = RoadCenterlinePlanner(PlanningSettings(**config['planning']))
    controller = PolicyController(ControlSettings(**config['control']))
    manager = PlannerExecutionManager(generation=1)
    worker = SimulatedPlannerWorker(delay_s=delivery_delay_s, measured=measured_delay,candidate_selector=selector)
    if modular:
        from .validation_ports import ModularPlannerWorker
        if measured_delay:raise ValueError('Modular clocks currently use explicit fixed delivery; CPU time is recorded separately')
        worker=ModularPlannerWorker(validation_config,selector)
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
    fixed = road.get('format') in ('fixed_uturn_validation_road_v1','runtime_road_fixture_v1')
    factory = fixed_road_environment if fixed else recorded_environment
    options = dict(terminal_support_m=terminal_support_m) if fixed else {}
    if modular:
        if not fixed:raise ValueError('Modular validation requires a normalized road fixture')
        options.update(physics_hz=rates['physics'],perception_hz=rates['perception'])
    with factory(road, seed=seed, progress=progress, speed=initial_speed,
            latency_steps=latency_steps, max_steps=max_steps+latency_steps+math.ceil(stop_tail_s*(rates['physics'] if modular else 20))+20,
            **options) as env:
        if not modular and not math.isclose(env.dt_s, 1/config['policy_update_rate_hz'], abs_tol=1e-12):
            raise ValueError('Plant and control periods differ')
        control_stride=round(1/(env.dt_s*config['policy_update_rate_hz']))
        control_updates=0;reference_pose=None
        perfect=None
        if modular and road_input=='map':
            from .validation_ports import PerfectPerception
            perfect=PerfectPerception(round(rates['physics']/rates['perception']))
        if modular and validation_config['modules']['control']=='cstar':
            from .validation_ports import CStarController
            controller=CStarController(env,controller.settings,validation_config)
        budget = handoff_budget_s if handoff_budget_s is not None else math.ceil(delivery_delay_s/env.dt_s-1e-9)*env.dt_s
        offer.shadow_handoff_estimate_s = max(settings.handoff_allowance_s, budget)
        offer.motion_history = env.estimator.motion
        # Passive truth monitor only: no C* solve/predict/control is called.
        checker = NativePrediction(env, SolverSettings(safety_margin_m=0.))
        initial_raw_state = list(env._backend.car._state_values())
        initial_timestamp = env._frame['timestamp_s']
        initial = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        goal = road_goal_progress(env)
        source_timeouts = dict(road=min(config['estimation']['max_source_age_s'], config['sensor_timeout_s']['cone_detections']),
            **{name: min(config['estimation']['max_source_age_s'], config['estimation']['motion_max_gap_s'],
                config['sensor_timeout_s'][name]) for name in ('wheel_speed', 'imu_angular_velocity')})
        with (output/'trace.jsonl').open('w', encoding='utf-8') as trace_stream, \
                (output/'shadow.jsonl').open('w', encoding='utf-8') as planner_stream, \
                (output/'submissions.jsonl').open('w', encoding='utf-8') as submission_stream:
            for step in range(max_steps):
                if float(env._info['evaluation']['ground_truth']['reference_line_progress_m']) >= goal:
                    break
                observation = env._frame
                raw_estimator_road=deepcopy(observation['estimation']['road']) if modular else None
                perception_road = deepcopy(observation['estimation']['road']) if road_input == 'no_centerline' else None
                if road_input == 'map':
                    observation = perfect.apply(env,env._frame,step) if perfect else ideal_observation(env, env._frame)
                elif road_input == 'no_centerline':
                    from offline.lattice_training.scenarios import road_input as mask_road_input
                    observation = mask_road_input(env._frame, 'no_centerline')
                estimates, now = observation['estimation'], observation['timestamp_s']
                ages = observation['sensor_age_s']
                fresh = {name: observation['sensor_available'].get(name, False)
                    and finite_number(ages.get(name)) and ages[name] < config['sensor_timeout_s'][name]
                    for name in config['required_sensors']}
                cones = observation['sensors'].get('cone_detections') or {}
                if cones.get('detections') and finite_number(ages.get('cone_detections')):
                    last_nonempty_stamp = now-ages['cone_detections']
                if perfect is not None:
                    age=estimates['road']['source_age_s'];fresh['cone_detections']=age<config['sensor_timeout_s']['cone_detections']
                    last_nonempty_stamp=now-age
                problem = required_sensor_problem(config['required_sensors'], fresh,
                    None if last_nonempty_stamp is None else now-last_nonempty_stamp, config['cone_empty_timeout_s'])
                if problem:
                    fault, failure = 'sensors:'+problem, dict(step=step, observation=observation)
                    break
                cycle_started = time.perf_counter()
                control_due=step%control_stride==0
                if control_due:
                    baseline, diagnostics = planner.plan(estimates['road'], estimates['state'], estimates['obstacles'],
                        estimates['vehicle_limits'], now, source_timeouts, mvp=True)
                packet = worker.poll(now)
                if packet is not None:
                    result = packet['result']
                    statuses[result['status']] += 1
                    planner_stream.write(json.dumps(json_safe(dict(step=step, result=result,
                        delivery={k: v for k, v in packet.items() if k not in ('job', 'result')},
                        job=packet['job'] if detailed_trace else None,
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
                    controller.settings, config['policy_update_rate_hz']) if requested and control_due else None
                if not requested and control_due:
                    manager.diagnostics = dict(mode='baseline', fallback=False, reason='baseline_selected')
                if control_due:priming = native is None and priming_motion_history(baseline, estimates, has_control, step*env.dt_s, controller.settings)
                if control_due and native is None and not baseline['valid'] and not priming:
                    fault, failure = 'planning:'+baseline['reason'], dict(step=step, observation=observation,
                        baseline=baseline, execution=dict(manager.diagnostics))
                    break
                if not control_due:
                    pass  # Actuator holds the last published control, with its original reference frame.
                elif priming:
                    drive = steer = 0.
                    reference = None
                    control = dict(valid=False, stop_requested=False, reason='priming_motion_history')
                    manager.diagnostics = dict(mode='priming', fallback=False, reason='priming_motion_history')
                else:
                    controller, drive, steer, control, reference = calculate_selected_control(
                        controller, baseline, native, estimates['state'], estimates['vehicle_params'],
                        now, 0. if step == 0 else control_stride*env.dt_s, manager if requested else None)
                    has_control = control['valid'] and not control['stop_requested']
                if control_due:
                    reference_pose=list(env._backend.car._state_values()[:3]);control_updates+=1
                if control_due and not priming and (not control['valid'] or control['stop_requested']):
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
                if selector is not None and mode=='planner':selector.commit(manager.diagnostics.get('job_id'))
                # Like ROS, offer only after selecting the action. Snapshot
                # collection is idempotent at the current state timestamp.
                if shadow_enabled and settings.enabled:
                    identifier = offer.shadow_job_id
                    if selector is not None or modular:
                        from .policy_selection import scoring_observation
                        worker.policy_observation=scoring_observation(observation)
                    offer._shadow_offer()
                    if offer.shadow_job_id != identifier:
                        submissions.append(now)
                        if detailed_trace:
                            job = worker.pending['job'] if worker.pending['job']['job_id'] == offer.shadow_job_id else worker.waiting
                            submission_stream.write(json.dumps(json_safe(dict(step=step, timestamp_s=now,
                                decision_pose=list(env._backend.car._state_values()[:3]), job=job)), allow_nan=False)+'\n')
                action = plant_action(drive, steer, actuator_steering_sign=steering_sign)
                control_wall = time.perf_counter()-cycle_started
                raw_before = env._backend.car._state_values()
                _, _, terminated, truncated, info = env.step(action)
                raw = env._backend.car._state_values()
                margins = checker.margins(raw, raw_before)
                row = dict(step=step, timestamp_s=now, progress_m=float(info['evaluation']['ground_truth']['reference_line_progress_m']),
                    phase='tracking',
                    control_updated=control_due,reference_pose=reference_pose,
                    selection_backend='policy' if selector is not None else 'native',
                    decision_pose=list(raw_before[:3]), x_m=float(raw[0]), y_m=float(raw[1]), heading_rad=float(raw[2]),
                    speed_mps=math.hypot(raw[3], raw[4]), raw_action=[drive, steer], plant_action=action,
                    reference=reference, planning=diagnostics, control=control, state=estimates['state'],
                    execution=dict(manager.diagnostics), road_status=estimates['road']['status'],
                    road_visibility=estimates['road'].get('visibility'), road_boundary_source=estimates['road'].get('boundary_source'),
                    planner_controller_wall_s=control_wall,
                    plant_corner_road_margin_m=margins['road_clearance_m'])
                if detailed_trace:
                    row.update(raw_state=list(raw), raw_state_before=list(raw_before),
                        observation=observation, baseline=baseline,
                        active_job_id=manager.diagnostics.get('job_id'),
                        ground_truth_contacts=info['evaluation']['contacts'])
                    if road_input == 'no_centerline':
                        row['perception_road'] = perception_road
                    if modular:row['raw_estimator_road']=raw_estimator_road
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
        final = float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        zero_rows = []
        if fault is not None:
            controller.clear()
            failure = failure or dict(fault=fault)
            failure.update(fault=fault, timestamp_s=env._frame['timestamp_s'],
                raw_state=list(env._backend.car._state_values()),
                observation=failure.get('observation',env._frame) if detailed_trace else failure.get('observation'),
                stop_requested=True, stop_action=[0., 0.])
            if detailed_trace and road_input=='no_centerline':failure['perception_road']=perception_road
            # The live FSM keeps publishing zero until explicit resume. Model
            # its actuator consequence, including coasting, without auto-retry.
            with (output/'trace.jsonl').open('a', encoding='utf-8') as stream:
                for index in range(math.ceil(stop_tail_s/env.dt_s)):
                    if env._done:
                        break
                    before = list(env._backend.car._state_values())
                    obs = env._frame
                    if perfect is not None:obs=perfect.apply(env,obs,step+index)
                    _, _, terminated, truncated, info = env.step((0., 0.))
                    raw = list(env._backend.car._state_values())
                    row = dict(step=len(rows)+index, timestamp_s=obs['timestamp_s'], phase='zero_hold',
                        progress_m=float(info['evaluation']['ground_truth']['reference_line_progress_m']),
                        decision_pose=before[:3], raw_state_before=before, raw_state=raw,
                        x_m=raw[0], y_m=raw[1], heading_rad=raw[2], speed_mps=math.hypot(raw[3],raw[4]),
                        raw_action=[0.,0.], plant_action=[0.,0.], reference=None,
                        execution=dict(mode='zero', reason=fault), control=dict(valid=False,stop_requested=True,reason=fault),
                        observation=obs if detailed_trace else None,
                        plant_corner_road_margin_m=checker.margins(raw)['road_clearance_m'])
                    zero_rows.append(row)
                    stream.write(json.dumps(json_safe(row), allow_nan=False)+'\n')
            failure['zero_publication_simulated_cycles'] = len(zero_rows)
            failure['plant_terminated_before_zero_tail_finished'] = bool(env._done)
            write_json(output/'failure.json', failure)
            if modular and detailed_trace and not rows and not zero_rows:
                # A recorded failure pose is not a simulated control interval.
                raw=list(env._backend.car._state_values());obs=failure.get('observation') or env._frame
                snapshot=dict(step=0,timestamp_s=obs['timestamp_s'],phase='fault_snapshot',action_applied=False,
                    control_updated=False,reference_pose=raw[:3],progress_m=final,raw_state_before=raw,raw_state=raw,
                    raw_action=[0.,0.],plant_action=[0.,0.],reference=None,observation=obs,
                    execution=dict(mode='zero',reason=fault),plant_corner_road_margin_m=checker.margins(raw)['road_clearance_m'])
                if road_input=='no_centerline':snapshot['perception_road']=perception_road
                snapshot['raw_estimator_road']=raw_estimator_road
                with (output/'trace.jsonl').open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(json_safe(snapshot),allow_nan=False)+'\n')
        completed = fault is None and final >= goal
        summary = dict(format='mvp21_shared_execution_v1', checkpoint=preserved, upstream_commit=UPSTREAM_COMMIT,
            source_sha256=SOURCE_SHA256, config_sha256=CONFIG_SHA256, upstream_config_sha256=UPSTREAM_CONFIG_SHA256,
            runtime_modules=runtime_provenance(), runtime_parameters=config, environment=env.configuration(),
            road_source_sha256=road.get('source_sha256') or hashlib.sha256(
                json.dumps(road, sort_keys=True).encode()).hexdigest(), road_input=road_input, path_source=path_source,
            lattice_generation_rate_hz=settings.rate_hz, control_rate_hz=config['policy_update_rate_hz'],
            perception_rate_hz=1/(env.dt_s*env.timing.period_steps.get('cone_detections',1)),
            initial_raw_state=initial_raw_state, initial_timestamp_s=initial_timestamp,
            camera_latency_s=env.dt_s*env.timing.delay_steps.get('cone_detections',0),
            initial_progress_m=initial, final_progress_m=final, goal_progress_m=goal, steps=len(rows),
            runtime_max_steps=max_steps,
            validation_config=validation_config,physics_rate_hz=1/env.dt_s,control_updates=control_updates,
            completed=completed, fault=fault, valid_reference_count=len(rows), simulated_seconds=len(rows)*env.dt_s,
            planner_reference_count=modes['planner'], baseline_reference_count=modes['baseline'],
            startup_baseline_cycles=startup_baseline, fallback_after_first_native_cycles=after_native_fallback,
            zero_action_priming_cycles=modes['priming'], fallback_reasons=dict(reasons),
            zero_publication_tail_cycles=len(zero_rows),
            after_zero_tail_progress_m=float(env._info['evaluation']['ground_truth']['reference_line_progress_m']),
            native_only_passed=selector is None and completed and modes['planner'] > 0 and modes['baseline'] == 0 and modes['priming'] == 0,
            selected_backend_only_passed=completed and modes['planner']>0 and modes['baseline']==0 and modes['priming']==0,
            delivery=dict(mode='measured_cpu_plus_delay' if measured_delay else 'fixed_simulated_delay',
                delay_s=delivery_delay_s, submitted=len(submissions), accepted=accepted, submission_times_s=submissions),
            shadow=dict(enabled=shadow_enabled and settings.enabled, calls=len(worker.completed),
                status_counts=dict(statuses), influences_actions=requested, effective_settings=vars(settings)),
            maximum_executed_speed_mps=max((r['speed_mps'] for r in rows), default=initial_speed),
            target_speed_mps=config['planning']['cruise_speed_mps'],
            minimum_plant_corner_road_margin_m=min((r['plant_corner_road_margin_m'] for r in rows), default=None),
            wall_s=time.perf_counter()-started, learned_scorer_enabled=selector is not None,
            learned_scorer_used=selector is not None and selector.candidate_calls>0,
            cstar_controls_used=modular and validation_config['modules']['control']=='cstar',
            selection_backend='policy' if selector is not None else 'native',
            policy=selector.provenance() if selector is not None else None,
            policy_reference_count=modes['planner'] if selector is not None else 0,
            policy_ranking_calls=selector.ranking_calls if selector is not None else 0,
            policy_candidate_calls=selector.candidate_calls if selector is not None else 0,
            policy_errors=selector.errors if selector is not None else 0,
            policy_state_commits=selector.state_commits if selector is not None else 0,
            real_time_qualified=False,
            hardware_tested=False, deployed=False, speed_tracking_qualified=False)
        write_json(output/'summary.json', summary)
        if modular:
            write_json(output/'module-events.json',dict(policy_ticks=worker.selection_events))
            write_json(output/'generation-records.json',worker.generated)
    verify_preserved_files()
    print(json.dumps({key: summary[key] for key in ('completed', 'steps', 'final_progress_m', 'fault',
        'planner_reference_count', 'baseline_reference_count', 'native_only_passed')}), flush=True)
    return summary
