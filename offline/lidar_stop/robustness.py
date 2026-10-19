"""Bounded straight-road robustness experiments, each with immutable records."""
from copy import deepcopy
from datetime import datetime, timezone
import json

from offline.lattice_training.dataset import write_json
from offline.mvp21_sim.validation_workspace import normalize_road, source_snapshot
from offline.mvp21_sim.visualize_runtime import write_viewer
from .__main__ import run_case, render_report, scenario, SIMULATION_CONTROL


def trials():
    result = [dict(name=name, base=name) for name in ('empty', 'central', 'outside', 'coast')]
    for name, width, lateral in (
            ('wide', .6, 0.), ('thin', .02, 0.),
            ('left_overlap', .08, .10), ('right_overlap', .08, -.10),
            ('left_edge_1mm', .04, .144), ('right_edge_1mm', .04, -.144),
            ('left_clear_5mm', .04, .150), ('right_clear_5mm', .04, -.150)):
        result.append(dict(name=name, base='central', width_m=width, lateral_m=lateral,
                           expectation='clear' if 'clear' in name else 'stop'))
    for name, sensor, expectation in (
            ('lidar_10hz', dict(period_steps=2), 'stop'),
            ('lidar_10hz_delay50ms', dict(period_steps=2, delay_steps=1), 'stop'),
            ('lidar_5hz', dict(period_steps=4), 'sensor_fault'),
            ('delay150ms', dict(delay_steps=3), 'sensor_fault'),
            ('one_scan_lost', dict(dropout_windows_s=((3., 3.05),)), 'stop'),
            ('outage_300ms', dict(dropout_windows_s=((3., 3.3),)), 'sensor_fault'),
            ('startup_missing', dict(dropout_windows_s=((0., 1.),)), 'sensor_fault')):
        result.append(dict(name=name, base='central', sensor_config=sensor, expectation=expectation))
    for seed in range(3):
        result.append(dict(name='noise_5mm_seed'+str(seed), base='central', expectation='stop',
                           sensor_config=dict(noise_std_m=.005, noise_seed=seed)))
    for side, lateral in (('left', .144), ('right', -.144)):
        result.append(dict(name=side+'_edge_10hz_delay50ms', base='central', width_m=.04,
            lateral_m=lateral, expectation='stop', sensor_config=dict(period_steps=2, delay_steps=1)))
    result.append(dict(name='obstacle_at_start', base='central', obstacle_progress_m=.735,
                       expectation='early_stop'))
    return result


def fixture(trial):
    case = deepcopy(scenario(trial['base']))
    if 'width_m' in trial:
        road = deepcopy(case['road_spec'])
        # normalize_road stores obstacle specs in road_spec, not a truth feed.
        obstacle = road['obstacles'][0]
        obstacle['geometry']['width_m'] = trial['width_m']
        obstacle['placement']['lateral_offset_m'] = trial['lateral_m']
        case = normalize_road(road, name=trial['name'])
    if 'obstacle_progress_m' in trial:
        case['road_spec']['obstacles'][0]['placement']['reference_line_progress_m'] = trial['obstacle_progress_m']
    case.update(title=trial['name'], fixture_id='lidar_matrix_'+trial['name'], collection='lidar_stop')
    return case


def render_matrix(summaries, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    lines = ['# LiDAR straight-road robustness matrix', '',
        'MVP2.1 observed sensor pipeline; 0.2 m/s target; unchanged 0.15 m front-bumper trigger.',
        'Fault stops are reported separately from obstacle stops. Negative braking is simulation only.', '',
        '| Case | Expected behavior | Met | Trigger reason | Trigger gap cm | Final gap cm | Contact |',
        '|---|---|---|---|---:|---:|---|']
    names = list(summaries)
    fig, ax = plt.subplots(figsize=(11, max(5, .32*len(names))))
    for index, (name, summary) in enumerate(summaries.items()):
        stop = summary['obstacle_stop']; event = stop['trigger'] or {}; check = summary['scenario_acceptance']
        met = check['expectation_met'] and summary['speed_tracking_simulation_check']['passed']
        fmt = lambda value: '—' if value is None else f'{value*100:.2f}'
        lines.append(f"| {name} | {check['expected_behavior']} | {met} | {event.get('reason','clear')} | "
                     f"{fmt(event.get('front_gap_m'))} | {fmt(stop['final_front_gap_m'])} | {stop['obstacle_contact']} |")
        rows = [json.loads(line) for line in (output/name/'trace.jsonl').read_text().splitlines()]
        gaps = [r['evaluation_front_gap_m'] for r in rows if r['evaluation_front_gap_m'] is not None]
        if gaps:
            ax.barh(index, min(gaps)*100, color='#c54c42' if stop['obstacle_contact'] or not met else '#31826b')
        else:
            ax.text(.5, index, 'outside corridor / no obstacle', fontsize=7, va='center')
    ax.set(yticks=range(len(names)), yticklabels=names, xlabel='Minimum actual front clearance (cm)',
           title='Green: expected behavior met; red: contact or unmet expectation')
    ax.axvline(0., color='black', linewidth=1); ax.invert_yaxis(); ax.grid(axis='x', alpha=.2)
    fig.tight_layout(); fig.savefig(output/'robustness-results.png', dpi=160); plt.close(fig)
    unmet = [name for name, s in summaries.items() if not (s['scenario_acceptance']['expectation_met']
             and s['speed_tracking_simulation_check']['passed'])]
    lines += ['', 'Unmet cases: '+(', '.join(unmet) or 'none'), '',
        'The 61-ray, 180-degree scan can delay detection at a body edge. A fresh empty scan is not proof of free space.',
        'The freshness limit is 150 ms. A lower rate/long outage may intentionally latch a sensor fault.',
        'Each directory retains road, sensor configuration, source timestamps, commands and evaluation-only truth.',
        'No change to ROS/common YAML, no car deployment, no physical braking qualification.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def run_matrix(output):
    output.mkdir(parents=True)
    source = source_snapshot(); summaries = {}; payloads = []
    specs = trials()
    for index, trial in enumerate(specs):
        print(f"Matrix {index+1}/{len(specs)}: {trial['name']}", flush=True)
        summary, payload = run_case(trial['base'], output/trial['name'], case=fixture(trial),
            sensor_config=trial.get('sensor_config'), expectation=trial.get('expectation'))
        summary['scenario_acceptance']['name'] = trial['name']
        write_json(output/trial['name']/'summary.json', summary)
        summaries[trial['name']] = summary; payloads.append(payload)
    render_report({name:summaries[name] for name in ('empty','central','outside','coast')}, output)
    render_matrix(summaries, output)
    write_viewer(payloads, output)
    met = {name: bool(s['scenario_acceptance']['expectation_met'] and s['speed_tracking_simulation_check']['passed'])
           for name, s in summaries.items()}
    unchanged = source['sha256'] == source_snapshot()['sha256']
    result = dict(created_at=datetime.now(timezone.utc).isoformat(), source=source,
        source_unchanged_during_run=unchanged, simulation_control=SIMULATION_CONTROL,
        trials=specs, scenarios=summaries, expectations_met=met,
        all_expectations_met=all(met.values()) and unchanged)
    write_json(output/'suite.json', result)
    print(json.dumps(dict(met=sum(met.values()), total=len(met), unmet=[n for n,v in met.items() if not v],
                          source_unchanged=unchanged), ensure_ascii=False), flush=True)
    return 0 if result['all_expectations_met'] else 1
