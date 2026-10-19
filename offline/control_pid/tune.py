"""Reproducible staged tuning against the notebook's exact Dream Gym model.

Usage: python offline/control_pid/tune.py --tune
       python offline/control_pid/tune.py --validate
"""
import argparse
from collections import deque
from dataclasses import asdict, dataclass, replace
from copy import deepcopy
import csv
import hashlib
from importlib.metadata import version
import itertools
import json
import math
from pathlib import Path
import platform
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from dreamgym.envs import BicycleModelDynamic, Road

from controller import Controller, Gains
from notebook_model import NOTEBOOK, ROOT, notebook_config

OUTPUT = ROOT / 'offline/control_pid/results'


@dataclass(frozen=True)
class Scenario:
    name: str
    road: str = 'notebook'
    speed: float = 0.8
    initial_y: float = 0.15
    initial_heading: float = 0.08
    initial_speed: float = 0.0
    reference_noise_m: float = 0.0
    heading_noise_rad: float = 0.0
    speed_noise_mps: float = 0.0
    motor_scale: float = 1.0
    drag_scale: float = 1.0
    steering_scale: float = 1.0
    steering_offset_rad: float = 0.0
    delay_steps: int = 0
    dt: float = 0.05
    dropout: bool = False


CONFIG = notebook_config()


def road_spec(kind: str) -> dict:
    spec = deepcopy(CONFIG['CONE_ROAD_SPEC'])
    if kind == 'straight':
        spec['elements'] = [deepcopy(spec['elements'][0])]
        spec['elements'][0]['geometry']['length_m'] = 30.0
    elif kind == 's_bend':
        start = deepcopy(spec['elements'][0])
        start['geometry']['length_m'] = 4.0
        bend = deepcopy(spec['elements'][1])
        bend['geometry'] = {'type': 'circular_arc', 'curvature_per_m': 1 / 3,
                            'sweep_angle_deg': 60.0}
        opposite = deepcopy(bend)
        opposite['geometry']['curvature_per_m'] *= -1
        end = deepcopy(start)
        end['geometry']['length_m'] = 5.0
        spec['elements'] = [deepcopy(e) for e in [start, bend, opposite, bend, opposite, end]]
    return spec


def road_length(spec: dict) -> float:
    return sum(e['geometry']['length_m'] if e['geometry']['type'] == 'straight'
               else math.radians(abs(e['geometry']['sweep_angle_deg']))
               / abs(e['geometry']['curvature_per_m']) for e in spec['elements'])


def vehicle(scenario: Scenario) -> BicycleModelDynamic:
    config = deepcopy(CONFIG['bicycle_model_config'])
    force = config['longitudinal_force_model']
    force['normalized_motor_command_force_gain_n'] *= scenario.motor_scale
    force['quadratic_aerodynamic_drag_coefficient_kg_per_m'] *= scenario.drag_scale
    config['steering']['physical_front_wheel_angle_offset_from_request_rad'] = scenario.steering_offset_rad
    model = BicycleModelDynamic(config)
    model.reset({'world_pose': {'y_m': scenario.initial_y,
                               'heading_rad': scenario.initial_heading},
                 'body_motion': {'longitudinal_velocity_mps': scenario.initial_speed}})
    return model


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def speed_schedule(t: float) -> float:
    # Initial acceleration, up-step, down-step, stop/hold and restart.
    if t < 5:
        return 0.5
    if t < 11:
        return 1.0
    if t < 17:
        return 0.4
    if t < 22:
        return 0.0
    return 0.6


def reference_for(road: Road, state: dict, progress: float, length: float,
                  now: float, speed: float, rng: np.random.Generator,
                  scenario: Scenario) -> dict:
    # Oracle planner: Road geometry is used ONLY by this offline mock upstream.
    distances = np.linspace(max(0, progress - 0.6), min(length, progress + 2.0), 27)
    points, _ = road.convert_progression_to_coordinates(distances)
    pose = state['world_pose']
    heading = pose['heading_rad']
    diff = points.astype(float) - [pose['x_m'], pose['y_m']]
    c, s = math.cos(heading), math.sin(heading)
    xs, ys = c * diff[:, 0] + s * diff[:, 1], -s * diff[:, 0] + c * diff[:, 1]
    keep = (xs >= -0.6) & (xs <= 2.0)
    xs, ys = xs[keep], ys[keep]
    if len(xs) < 4 or np.ptp(xs) < 0.1:
        return {'valid': False}
    # Correlated rigid shift/rotation errors, not independent noisy path points.
    ys += rng.normal(0, scenario.reference_noise_m)
    ys += xs * math.tan(rng.normal(0, scenario.heading_noise_rad))
    coeffs = np.polynomial.polynomial.polyfit(xs, ys, 3)
    return {'timestamp_s': now, 'valid': not (scenario.dropout and 5 <= now < 5.3),
            'path_coeffs': coeffs.tolist(), 'x_range_m': [float(min(0, xs.min())), float(xs.max())],
            'target_speed_mps': speed}


def simulate(gains: Gains, scenario: Scenario, seed: int = 0,
             longitudinal: bool = False) -> tuple[dict, list[dict]]:
    spec = road_spec('straight' if longitudinal else scenario.road)
    length = road_length(spec)
    road = Road(road_spec=deepcopy(spec))
    model, controller = vehicle(scenario), Controller(gains)
    rng = np.random.default_rng(seed)
    queue = deque([(0.0, 0.0)] * scenario.delay_steps)
    records = []
    duration = 28.0 if longitudinal else length / scenario.speed + 8.0
    complete, failure = False, ''
    for step in range(math.ceil(duration / scenario.dt)):
        now = step * scenario.dt
        state = model.state
        pose, motion = state['world_pose'], state['body_motion']
        closest = road.find_closest_point_to(pose['x_m'], pose['y_m'])
        px, py, _, _, progress, tangent, _ = closest
        ey = -(pose['x_m'] - px) * math.sin(tangent) + (pose['y_m'] - py) * math.cos(tangent)
        heading_error = math.atan2(math.sin(pose['heading_rad'] - tangent),
                                  math.cos(pose['heading_rad'] - tangent))
        speed = motion['longitudinal_velocity_mps']
        target = speed_schedule(now) if longitudinal else scenario.speed
        ref = reference_for(road, state, progress, length, now, target, rng, scenario)
        measurement = max(0, speed + rng.normal(0, scenario.speed_noise_mps))
        drive, steer, diagnostic = controller.calculate(
            ref, measurement, now, 0.0 if step == 0 else scenario.dt)
        queue.append((drive, steer))
        applied_drive, applied_steer = queue.popleft()
        # Conservative oriented footprint margin to road boundaries, NOT a
        # simulator collision detector. Uses notebook body dimensions.
        body = CONFIG['bicycle_model_config']['body_geometry']
        half_width = (0.5 * body['width_m'] * abs(math.cos(heading_error))
                      + max(body['front_extent_from_center_of_mass_m'],
                            body['rear_extent_from_center_of_mass_m']) * abs(math.sin(heading_error)))
        clearance = 0.5 - abs(ey) - half_width
        records.append({'time_s': now, 'x_m': pose['x_m'], 'y_m': pose['y_m'],
                        'progress_m': progress, 'lateral_error_m': ey,
                        'heading_error_rad': heading_error, 'speed_mps': speed,
                        'target_speed_mps': target, 'drive': drive, 'steer': steer,
                        'clearance_m': clearance, 'reference_valid': diagnostic['valid']})
        if not all(math.isfinite(v) for v in [ey, speed, drive, steer]):
            failure = 'nonfinite'
            break
        if clearance < 0:
            failure = 'footprint_outside_lane'
            break
        if not longitudinal and progress >= length - 0.3:
            complete = True
            break
        model.set_action_request(applied_drive,
                                 applied_steer * gains.steering_limit_rad * scenario.steering_scale)
        model.integrate(scenario.dt, 'rk4', update_stored_state=True)
    if longitudinal:
        complete = not failure
    if not complete and not failure:
        failure = 'timeout'
    errors = np.array([r['lateral_error_m'] for r in records])
    velocity_errors = np.array([r['target_speed_mps'] - r['speed_mps'] for r in records])
    drives = np.array([r['drive'] for r in records])
    steers = np.array([r['steer'] for r in records])
    times = np.array([r['time_s'] for r in records])
    speeds = np.array([r['speed_mps'] for r in records])
    late = (times >= 3) if not longitudinal else np.logical_or.reduce([
        (times >= a) & (times < b) for a, b in [(3,5),(9,11),(15,17),(20,22),(26,28)]])
    stopping = (times >= 17) & (times < 22)
    stopped = np.flatnonzero(stopping & (speeds <= 0.03))
    row = {'scenario': scenario.name, 'seed': seed, 'complete': complete,
           'failure': failure, 'duration_s': records[-1]['time_s'],
           'progress_m': records[-1]['progress_m'],
           'lateral_rmse_m': float(np.sqrt(np.mean(errors ** 2))),
           'lateral_max_m': float(np.max(np.abs(errors))),
           'speed_rmse_mps': float(np.sqrt(np.mean(velocity_errors ** 2))),
           'steady_speed_rmse_mps': float(np.sqrt(np.mean(velocity_errors[late] ** 2))) if late.any() else 10.0,
           'min_clearance_m': min(r['clearance_m'] for r in records),
           'steer_rate_rms_per_s': float(np.sqrt(np.mean(np.diff(steers) ** 2)) / scenario.dt),
           'drive_rate_rms_per_s': float(np.sqrt(np.mean(np.diff(drives) ** 2)) / scenario.dt),
           'drive_saturation_fraction': float(np.mean((drives <= gains.drive_min + 1e-6)
                                                      | (drives >= gains.drive_max - 1e-6))),
           'steer_saturation_fraction': float(np.mean(np.abs(steers) >= 0.999)),
           'stop_time_s': float(times[stopped[0]] - 17) if stopped.size else None,
           'stop_distance_m': (records[stopped[0]]['progress_m'] - records[int(round(17/scenario.dt))]['progress_m'])
                              if stopped.size else None}
    return row, records


TRAINING = [Scenario('train_notebook'),
            Scenario('train_s_bend', 's_bend', initial_y=-0.2, initial_heading=-0.12),
            Scenario('train_offset', 's_bend', initial_y=0.2, initial_heading=0.12,
                     steering_offset_rad=math.radians(2), motor_scale=0.8)]


def tune() -> Gains:
    base = Gains()
    rows = []
    best, best_score = base, math.inf
    # PI grid plus filtered derivative candidates; select on more than one plant.
    for kp, ki, kd in itertools.product([0.4,0.7,1.0,1.4], [0.1,0.3,0.6], [0.0,0.04]):
        candidate = replace(base, speed_kp=kp, speed_ki=ki, speed_kd=kd)
        trials = [simulate(candidate, s, longitudinal=True)[0] for s in [
            Scenario('speed_nominal','straight',initial_y=0,initial_heading=0),
            Scenario('speed_mismatch','straight',initial_y=0,initial_heading=0,
                     motor_scale=0.75,drag_scale=1.4)]]
        score = float(np.mean([100*(not t['complete']) + t['speed_rmse_mps']
                               + 2*t['steady_speed_rmse_mps'] + 0.01*t['drive_rate_rms_per_s']
                               + (0.2 if t['stop_time_s'] is None else 0.02*t['stop_time_s']) for t in trials]))
        rows.append({'stage':'speed', 'score':score, **asdict(candidate)})
        if score < best_score:
            best, best_score = candidate, score
    pi_rows = [r for r in rows if r['speed_kd'] == 0]
    simplest = min(pi_rows, key=lambda r:r['score'])
    if simplest['score'] <= best_score * 1.02:
        best = Gains(**{k:simplest[k] for k in asdict(base)})
        best_score = simplest['score']
    print('Speed selection (prefer PI within 2% of best score):',
          best.speed_kp,best.speed_ki,best.speed_kd,'score',best_score,flush=True)
    best_speed = best
    best_score = math.inf
    for kp, heading, kd in itertools.product([0.4,0.8,1.2], [0.8,1.4,2.0], [0.0,0.08,0.2]):
        candidate = replace(best_speed,lateral_kp=kp,heading_kp=heading,lateral_kd=kd)
        trials = [simulate(candidate,s)[0] for s in TRAINING]
        score = float(np.mean([100*(not t['complete']) + t['lateral_rmse_m']
                               + 0.25*t['lateral_max_m'] + 0.005*t['steer_rate_rms_per_s'] for t in trials]))
        rows.append({'stage':'lateral', 'score':score, **asdict(candidate)})
        if score < best_score:
            best, best_score = candidate, score
    # Integral ablation: test whether persistent trim benefits outweigh drift.
    base = best
    for ki in [0.0,0.03,0.1]:
        candidate = replace(base,lateral_ki=ki)
        trials = [simulate(candidate,s)[0] for s in TRAINING]
        score = float(np.mean([100*(not t['complete']) + t['lateral_rmse_m']
                               + 0.25*t['lateral_max_m'] + 0.005*t['steer_rate_rms_per_s'] for t in trials]))
        rows.append({'stage':'lateral_integral', 'score':score, **asdict(candidate)})
        if score < best_score:
            best,best_score = candidate,score
    write_csv(OUTPUT/'tuning_candidates.csv',rows)
    (OUTPUT/'selected_gains.json').write_text(json.dumps(asdict(best),indent=2)+'\n',encoding='utf-8')
    print('Lateral selection:',asdict(best),'score',best_score,flush=True)
    return best


def validate(gains: Gains) -> None:
    scenarios = [Scenario('notebook_05',speed=0.5),Scenario('notebook_10',speed=1.0),
                 Scenario('s_bend_05','s_bend',speed=0.5,initial_y=-0.2,initial_heading=-0.15),
                 Scenario('s_bend_12','s_bend',speed=1.2,initial_y=0.2,initial_heading=0.15),
                 Scenario('noisy_reference','s_bend',reference_noise_m=0.015,
                          heading_noise_rad=0.015,speed_noise_mps=0.02),
                 Scenario('plant_mismatch','s_bend',motor_scale=0.7,drag_scale=1.5,
                          steering_scale=0.85,steering_offset_rad=math.radians(-3)),
                 Scenario('command_delay_100ms','s_bend',delay_steps=2),
                 Scenario('dt_100ms','s_bend',dt=0.1),
                 Scenario('reference_dropout','s_bend',dropout=True)]
    all_rows, examples = [], {}
    for scenario in scenarios:
        for seed in [101,202,303]:
            row, trace = simulate(gains,scenario,seed)
            all_rows.append(row)
            if seed == 101:
                examples[scenario.name] = trace
                write_csv(OUTPUT/f'{scenario.name}_trace.csv',trace)
        print('Validation:',scenario.name, 'complete', all(r['complete'] for r in all_rows[-3:]),flush=True)
    for name, candidate in [('selected',gains),('speed_p_only',replace(gains,speed_ki=0,speed_kd=0))]:
        row,trace = simulate(candidate,Scenario(name,'straight',initial_y=0,initial_heading=0),
                             longitudinal=True)
        all_rows.append(row)
        write_csv(OUTPUT/f'{name}_speed_trace.csv',trace)
        examples[name] = trace
    write_csv(OUTPUT/'validation_metrics.csv',all_rows)
    fig, axes = plt.subplots(2,2,figsize=(12,8))
    for name in ['notebook_05','s_bend_12','noisy_reference','plant_mismatch']:
        trace=examples[name]
        ts=np.array([r['time_s'] for r in trace])
        axes[0,0].plot(ts,[r['lateral_error_m'] for r in trace],label=name)
        axes[0,1].plot(ts,[r['steer'] for r in trace],label=name)
    for name in ['selected','speed_p_only']:
        trace=examples[name]
        axes[1,0].plot([r['time_s'] for r in trace],[r['speed_mps'] for r in trace],label=name)
        axes[1,1].plot([r['time_s'] for r in trace],[r['drive'] for r in trace],label=name)
    trace=examples['selected']
    axes[1,0].plot([r['time_s'] for r in trace],[r['target_speed_mps'] for r in trace],'k--',label='reference')
    for ax,label in zip(axes.flat,['Cross-track error [m]','Normalized steering',
                                  'Longitudinal speed [m/s]','Normalized drive']):
        ax.set(xlabel='Time [s]',ylabel=label)
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUTPUT/'validation.png',dpi=150)
    plt.close(fig)
    metadata={'dreamgym_version':version('dreamgym'),
              'python_version':platform.python_version(),
              'numpy_version':np.__version__,
              'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in [Path(__file__),Path(__file__).with_name('controller.py'),
                                         Path(__file__).with_name('notebook_model.py')]},
              'notebook_sha256':hashlib.sha256(NOTEBOOK.read_bytes()).hexdigest(),
              'model':CONFIG['bicycle_model_config'],'integration':CONFIG['integration_config'],
              'validation_scenarios':[asdict(s) for s in scenarios],
              'validation_seeds':[101,202,303], 'training_scenarios':[asdict(s) for s in TRAINING],
              'reference':'oracle road -> local cubic; not cone-based perception',
              'gains':asdict(gains)}
    (OUTPUT/'experiment.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    summary = [
        '# PID baseline offline results', '',
        'Generated from validation_metrics.csv; simulation only.', '',
        'The selected gains are the best of the tested coarse grids, not a global optimum.',
        'Speed PI is preferred when its training score is within 2% of the best PID.', '',
        '## Selected gains', '',
        f'- Speed Kp / Ki / Kd: {gains.speed_kp} / {gains.speed_ki} / {gains.speed_kd}',
        f'- Lateral Kp / Ki / Kd: {gains.lateral_kp} / {gains.lateral_ki} / {gains.lateral_kd}',
        f'- Heading Kp: {gains.heading_kp}', '',
        '## Validation', '',
        '| Scenario | Completed / runs | Cross-track RMSE [m] | Maximum offset [m] | Steady speed RMSE [m/s] |',
        '| --- | --- | --- | --- | --- |',
    ]
    for name in dict.fromkeys(r['scenario'] for r in all_rows):
        group = [r for r in all_rows if r['scenario'] == name]
        summary.append(f"| {name} | {sum(r['complete'] for r in group)} / {len(group)} | "
                       f"{np.mean([r['lateral_rmse_m'] for r in group]):.4f} | "
                       f"{max(r['lateral_max_m'] for r in group):.4f} | "
                       f"{np.mean([r['steady_speed_rmse_mps'] for r in group]):.4f} |")
    selected = next(r for r in all_rows if r['scenario'] == 'selected')
    summary += ['', f"Selected speed step/stop run: speed <= 0.03 m/s after "
                f"{selected['stop_time_s']:.2f} s and {selected['stop_distance_m']:.3f} m "
                'following the zero-speed request from 0.4 m/s.', '',
                '## Limits', '',
                '- Ground-truth road geometry supplies the cubic mock planner; no cone-based planning is validated.',
                '- Speed feedback is simulated body longitudinal velocity, with optional added noise.',
                '- Seeds 101/202/303 alter noise; noiseless repetitions are deterministic.',
                '- Completion means reaching within 0.3 m of the road end without a local footprint-margin failure.',
                '- The footprint margin uses local tangent geometry; it is not an obstacle collision test.',
                '- Stop/hold uses the Dream Gym direction latch. Physical braking and steering calibration are untested.',
                '- The ROS policy, required-sensor contract, explicit recovery and vehicle watchdog have not been ported or tested.']
    (OUTPUT/'summary.md').write_text('\n'.join(summary)+'\n',encoding='utf-8')
    print('Validation complete:',sum(r['complete'] for r in all_rows),'/',len(all_rows),flush=True)


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--tune',action='store_true')
    parser.add_argument('--validate',action='store_true')
    args=parser.parse_args()
    if not args.tune and not args.validate:
        parser.error('Choose --tune or --validate')
    if version('dreamgym') != '0.4.0':
        raise RuntimeError('This experiment requires the notebook pin dreamgym==0.4.0')
    OUTPUT.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter()
    gains=tune() if args.tune else Gains(**json.loads((OUTPUT/'selected_gains.json').read_text()))
    validate(gains)
    print(f'Elapsed: {time.perf_counter()-started:.1f}s; output: {OUTPUT}',flush=True)


if __name__ == '__main__':
    main()
