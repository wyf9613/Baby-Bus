"""Open-loop model vs Dream Gym validation; no policy or state correction.

Run with the course Python. Outputs PNG, CSV, and JSON alongside the notebook.
Ground truth is used only for validation, not supplied to an MPC policy.
The brake case intentionally exposes the predictor's missing reverse latch.
"""
import argparse
import ast
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import gymnasium as gym
import dreamgym  # registers the environment
from vehicle_model import CourseModelParams, step

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
NAMES = ['x_m', 'y_m', 'heading_rad', 'vx_mps', 'steering_rad']


def notebook_configs(path):
    """Read only literal/NumPy configuration assignments, never run the notebook."""
    wanted = {'bicycle_model_config', 'integration_config'}
    found = {}
    for cell in json.loads(path.read_text(encoding='utf-8'))['cells']:
        if cell['cell_type'] != 'code':
            continue
        tree = ast.parse(''.join(cell['source']))
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1:
                target = node.targets[0]
                if isinstance(target, ast.Name) and target.id in wanted:
                    # Accept data literals and np.deg2rad, not arbitrary notebook code.
                    for item in ast.walk(node.value):
                        if isinstance(item, ast.Name) and item.id != 'np':
                            raise ValueError('Config references unsupported name: ' + item.id)
                        if isinstance(item, ast.Attribute) and not (
                            isinstance(item.value, ast.Name) and item.value.id == 'np'
                            and item.attr == 'deg2rad'
                        ):
                            raise ValueError('Only np.deg2rad is supported in configs')
                        if isinstance(item, ast.Call) and not isinstance(item.func, ast.Attribute):
                            raise ValueError('Unsupported call in config')
                    found[target.id] = eval(compile(ast.Expression(node.value), str(path), 'eval'),
                                            {'__builtins__': {}, 'np': np})
    if found.keys() != wanted:
        raise ValueError('Missing notebook model/integration configuration')
    return found['bicycle_model_config'], found['integration_config']


def read_state(env):
    s = env.unwrapped.car.state
    pose = s['world_pose']
    return np.array([pose['x_m'], pose['y_m'], pose['heading_rad'],
                     s['body_motion']['longitudinal_velocity_mps'],
                     s['front_wheel_steering_angle_rad']], dtype=float)


def action_sequence(name, count, dt):
    t = np.arange(count) * dt
    u = np.zeros((count, 2), dtype=np.float32)
    u[:, 0] = .1
    if name == 'turn':
        u[t >= 1, 1] = .3
    elif name == 'slalom':
        u[:, 0] = .15 + .05 * np.sin(t)
        u[:, 1] = .35 * np.sin(1.5 * t)
    elif name == 'brake':
        u[t >= 1, 0] = -1
    return u


def run_case(name, config, integration, initial, actions):
    p = CourseModelParams.from_notebook_config(config)
    dt = integration['environment_step_duration_s']
    fixed = lambda value: {'lower': float(value), 'upper': float(value)}
    yaw_rate = initial[3] * np.tan(initial[4]) / (p.front_axle_m + p.rear_axle_m)
    initial_config = {
        'pose': {'mode': 'world_frame', 'x_m': fixed(initial[0]),
                 'y_m': fixed(initial[1]), 'heading_rad': fixed(initial[2])},
        'body_motion': {
            'longitudinal_velocity': {'source': 'absolute', 'value_mps': fixed(initial[3])},
            'lateral_velocity_mps': fixed(p.rear_axle_m * yaw_rate),
            'yaw_rate_rad_per_s': fixed(yaw_rate),
        },
        'front_wheel_steering_angle_rad': fixed(initial[4]),
    }
    # A long straight reference road avoids route-end termination. No controller
    # reads it; lateral bounds are relaxed for these open-loop manoeuvres.
    road = {'elements': [{'geometry': {'type': 'straight', 'length_m': 1000.},
            'lanes': {'reference_line_role': 'lane_center',
                      'reference_lane': {'width_m': {'start': 100.}}}}]}
    env = gym.make('dreamgym/autonomous_driving_env', road_spec=road,
                   bicycle_model_config=config, integration_config=integration,
                   initial_state_config=initial_config, observation_config={},
                   termination_config={'planar_speed_bounds_mps': {'lower': 0., 'upper': 100.},
                     'absolute_lateral_error_from_target_line_upper_bound_m': 1000.},
                   observation_format='dict', render_mode=None)
    predicted, measured = [initial.copy()], []
    try:
        env.reset(seed=0)
        measured.append(read_state(env))
        np.testing.assert_allclose(measured[0], initial, atol=1e-12, rtol=0)
        for index, action in enumerate(actions):
            predicted.append(step(predicted[-1], action, dt, p))
            _, _, terminated, truncated, _ = env.step(action)
            measured.append(read_state(env))
            if (terminated or truncated) and index != len(actions)-1:
                raise RuntimeError(f'{name}: Gym ended at step {index+1}; cannot compare full sequence')
    finally:
        env.close()
    return np.asarray(predicted), np.asarray(measured)


def save_results(out, name, predicted, measured, actions, dt):
    t = np.arange(len(predicted)) * dt
    error = predicted - measured
    error[:, 2] = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
    position_error = np.linalg.norm(error[:, :2], axis=1)
    fig, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(measured[:, 0], measured[:, 1], label='Dream Gym', lw=2)
    ax.plot(predicted[:, 0], predicted[:, 1], '--', label='Prediction')
    ax.set(xlabel='COM x [m]', ylabel='COM y [m]', title='Open-loop trajectory', aspect='equal')
    ax.legend()
    axes[0, 1].plot(t, position_error)
    axes[0, 1].set(title='Position error norm', xlabel='Time [s]', ylabel='Error [m]')
    for ax, column, title, unit in [(axes[1, 0], 2, 'Heading error (wrapped)', 'rad'),
                                   (axes[1, 1], 3, 'Longitudinal speed error', 'm/s'),
                                   (axes[2, 0], 4, 'Front-wheel angle error', 'rad')]:
        ax.plot(t, error[:, column])
        ax.set(title=title, xlabel='Time [s]', ylabel=f'Prediction - Gym [{unit}]')
    ax = axes[2, 1]
    ax.step(t[:-1], actions[:, 0], where='post', label='Drive')
    ax.step(t[:-1], actions[:, 1], where='post', label='Steering')
    ax.set(title='Identical commands to both models', xlabel='Time [s]', ylabel='Normalized command')
    ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=.25)
    note = ' — expected mismatch: prediction omits reverse latch' if name == 'brake' else ''
    fig.suptitle(f'{name}: dt={dt:g} s, {len(actions)} steps{note}')
    fig.savefig(out / f'{name}.png', dpi=160)
    plt.close(fig)
    with (out / f'{name}.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'drive_for_next_step', 'steer_for_next_step'] +
                        [f'{prefix}_{n}' for prefix in ['prediction', 'gym', 'error'] for n in NAMES])
        for i in range(len(t)):
            u = actions[i].tolist() if i < len(actions) else ['', '']
            writer.writerow([t[i], *u, *predicted[i], *measured[i], *error[i]])
    return {'steps': len(actions), 'dt_s': dt,
            'position_rmse_m': float(np.sqrt(np.mean(position_error**2))),
            'position_max_m': float(position_error.max()),
            'state_rmse': dict(zip(NAMES, np.sqrt(np.mean(error**2, axis=0)).tolist())),
            'state_max_abs_error': dict(zip(NAMES, np.max(np.abs(error), axis=0).tolist())),
            'expected_latch_mismatch': name == 'brake'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--notebook', type=Path, default=REPO/'docs/ad_gym_system_project_v2026_09_18.ipynb')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'vehicle_model_validation')
    parser.add_argument('--scenario', choices=['all', 'straight', 'turn', 'slalom', 'brake'], default='all')
    parser.add_argument('--steps', type=int, default=200)
    parser.add_argument('--initial-state', type=float, nargs=5, default=[0, 0, 0, 1, 0],
                        metavar=('X', 'Y', 'PSI', 'VX', 'DELTA'))
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error('--steps must be positive')
    config, integration = notebook_configs(args.notebook)
    dt = integration['environment_step_duration_s']
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenarios = ['straight', 'turn', 'slalom', 'brake'] if args.scenario == 'all' else [args.scenario]
    notebook = args.notebook.resolve()
    summary = {'notebook': (notebook.relative_to(REPO).as_posix() if notebook.is_relative_to(REPO)
                            else str(notebook)), 'initial_state': args.initial_state,
               'bicycle_model_config': config, 'integration_config': integration, 'cases': {}}
    for name in scenarios:
        actions = action_sequence(name, args.steps, dt)
        predicted, measured = run_case(name, config, integration, np.array(args.initial_state), actions)
        result = save_results(args.output_dir, name, predicted, measured, actions, dt)
        summary['cases'][name] = result
        print(f'{name}: position RMSE={result["position_rmse_m"]:.6g} m; '
              f'max={result["position_max_m"]:.6g} m', flush=True)
    (args.output_dir/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    print('Saved:', args.output_dir)


if __name__ == '__main__':
    main()
