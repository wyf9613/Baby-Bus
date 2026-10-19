"""Run the shared MVP2.1 runtime with a 15 cm front-bumper stop monitor."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from offline.lattice_training.dataset import write_json
from offline.lattice_training.visualize import scene_data
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.source import ROOT
from offline.mvp21_sim.validation_config import configuration
from offline.mvp21_sim.validation_workspace import normalize_road, source_snapshot, clock_audit
from offline.mvp21_sim.visualize_runtime import compact_case, write_viewer


# Match the existing course plant: drag*v^2 / motor_gain = .2^2 / 10.
# These are explicit OFFLINE settings, never written into the ROS/common YAML.
SIMULATION_CONTROL = dict(mvp_drive_feedforward=.004, speed_kp=.35, speed_ki=0., drive_max=.08)


def scenario(name):
    road = json.loads((ROOT/'offline/mvp21_sim/workspace/roads/quick-straight.json').read_text())
    if name != 'empty':
        road['obstacles'] = [dict(geometry=dict(type='rectangle', length_m=.2, width_m=.2),
            placement=dict(reference_line_progress_m=1.7,
                           lateral_offset_m=.4 if name == 'outside' else 0., heading_offset_rad=0.))]
    case = normalize_road(road, name=name)
    case.update(title={'empty':'无障碍直道', 'central':'前方障碍，15 cm 触发',
                      'outside':'走廊外障碍', 'coast':'零油门对照，保留失败'}[name],
                collection='lidar_stop', fixture_id='lidar_stop_'+name)
    return case


def true_front_gap(pose, polygons, body):
    """Evaluation ONLY: axis-aligned straight-road footprint/obstacle gap."""
    x, y, yaw = pose[:3]
    c, s = math.cos(yaw), math.sin(yaw)
    gaps = []
    for polygon in polygons:
        points = [(c*(px-x)+s*(py-y), -s*(px-x)+c*(py-y)) for px, py in polygon]
        if (min(p[1] for p in points) <= body['width_m']/2
                and max(p[1] for p in points) >= -body['width_m']/2
                and max(p[0] for p in points) >= -body['rear_extent_from_center_of_mass_m']):
            gaps.append(min(p[0] for p in points)-body['front_extent_from_center_of_mass_m'])
    return min(gaps, default=None)


def run_case(name, output, *, case=None, sensor_config=None, expectation=None,
             brake_drive=None):
    case = scenario(name) if case is None else case
    expectation = expectation or ('coast_failure' if name=='coast' else
                                  'stop' if name=='central' else 'clear')
    profile = configuration(dict(max_steps=400, stop_tail_s=1.))
    summary = run_runtime(case, output, seed=0, detailed_trace=True, validation_config=profile,
        simulation_control_overrides=SIMULATION_CONTROL,
        obstacle_stop_config=dict(enabled=True, trigger_distance_m=.15),
        simulation_brake_drive=(0. if name == 'coast' else -.1) if brake_drive is None else brake_drive,
        stop_timeout_s=5., stopped_hold_s=1., lidar_sensor_config=sensor_config)
    write_json(output/'road.json', case)
    rows = [json.loads(line) for line in (output/'trace.jsonl').read_text().splitlines()]
    body = summary['environment']['bicycle_model_config']['body_geometry']
    polygons = scene_data(summary['environment']['road_spec'])['obstacles']
    for row in rows:
        row['evaluation_front_gap_m'] = true_front_gap(row['raw_state'], polygons, body)
    # Augment evaluation records only; the controller never receives these gaps.
    with (output/'trace.jsonl').open('w') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    stop = summary['obstacle_stop']
    stop['final_front_gap_m'] = rows[-1]['evaluation_front_gap_m'] if rows else None
    # Early protective stops need not accelerate to cruise first.
    peak = max((r['speed_mps'] for r in rows), default=0.)
    minimum_peak = .18 if expectation in ('stop', 'clear', 'coast_failure') else 0.
    summary['speed_tracking_simulation_check'] = dict(target_mps=.2,
        peak_mps=peak, peak_acceptance_mps=[minimum_peak,.22],
        passed=bool(rows) and minimum_peak<=peak<=.22)
    reason = (stop['trigger'] or {}).get('reason')
    physically_stopped = (stop['stopped_hold_s']+1e-9 >= 1. and stop['final_speed_mps']<=.01
                          and not stop['obstacle_contact'])
    obstacle_success = (stop['passed'] and stop['final_front_gap_m'] is not None
                        and stop['final_front_gap_m'] > 0)
    if expectation in ('stop', 'coast_failure'):
        success = obstacle_success
    elif expectation == 'clear':
        success = summary['completed'] and stop['trigger'] is None and not stop['obstacle_contact']
    elif expectation == 'sensor_fault':
        success = bool(reason and reason.startswith('lidar_') and reason!='lidar_obstacle'
                       and physically_stopped)
    elif expectation == 'early_stop':
        success = bool(reason=='lidar_obstacle' and physically_stopped and
                       stop['final_front_gap_m'] is not None and stop['final_front_gap_m']>0)
    else:
        raise ValueError('Unknown LiDAR scenario expectation: '+str(expectation))
    expected = (bool(reason=='lidar_obstacle') and stop['obstacle_contact'] and not success
                if expectation=='coast_failure' else success)
    summary['scenario_acceptance'] = dict(name=name, passed=bool(success),
        expected_behavior=expectation, expected_failure=expectation=='coast_failure',
        expectation_met=bool(expected),
        statement='Stop at 15 cm trigger; no claim of 15 cm final clearance')
    write_json(output/'summary.json', summary)
    audit = clock_audit(output, profile)
    if not audit['checks_passed']:
        raise AssertionError('Runtime clock audit failed: '+str(audit))
    return summary, compact_case(output.parent, dict(name=output.name, start='road_start',
                                summary=summary, timing_audit=audit))


def render_report(summaries, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(summaries), 2, figsize=(11, 2.8*len(summaries)), squeeze=False)
    lines = ['# MVP2.1 LiDAR stop simulation', '',
        'Observed LiDAR; native MVP2.1 planner and actual MVP controller. Front-bumper trigger = 0.15 m.',
        'The simulated negative brake is Dream Gym only. ROS/common config and vehicle model are unchanged.', '',
        '| Scenario | Expected outcome met | Peak speed m/s | Trigger gap m | Final gap m | Final speed m/s | Contact |',
        '|---|---|---:|---:|---:|---:|---|']
    for index, (name, summary) in enumerate(summaries.items()):
        rows = [json.loads(line) for line in (output/name/'trace.jsonl').read_text().splitlines()]
        t = [r['timestamp_s']-rows[0]['timestamp_s'] for r in rows]
        stop = summary['obstacle_stop'];trigger = stop['trigger'] or {}
        lines.append(f"| {name} | {summary['scenario_acceptance']['expectation_met']} | "
            f"{summary['speed_tracking_simulation_check']['peak_mps']:.4f} | {trigger.get('front_gap_m')} | "
            f"{stop['final_front_gap_m']} | {stop['final_speed_mps']:.4f} | {stop['obstacle_contact']} |")
        axes[index,0].plot(t,[r['speed_mps'] for r in rows],label='actual')
        axes[index,0].axhline(.2,color='gray',linestyle='--',label='target')
        axes[index,1].plot(t,[r['evaluation_front_gap_m'] for r in rows],label='evaluation gap')
        # Plot the decision's observed corridor gap separately from truth.
        observed=[(r['timestamp_s']-rows[0]['timestamp_s'],
                   (r.get('obstacle_stop') or r.get('control',{}).get('obstacle_stop') or {}).get('nearest_front_gap_m')) for r in rows]
        axes[index,1].plot([p[0] for p in observed],[p[1] for p in observed],linestyle=':',label='LiDAR gap')
        axes[index,1].set_xlim(0.,max(t[-1],.05))
        if not any(r['evaluation_front_gap_m'] is not None for r in rows):
            axes[index,1].text(.5,.5,'No obstacle in forward corridor',ha='center',
                transform=axes[index,1].transAxes)
        axes[index,1].axhline(.15,color='orange',linestyle='--',label='trigger')
        axes[index,1].axhline(0,color='red',linestyle='--',label='contact plane')
        for ax in axes[index]:
            ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=7)
        axes[index,0].set_ylabel('Speed (m/s)');axes[index,0].set_title(name)
        axes[index,1].set_ylabel('Front gap (m)')
    fig.tight_layout();fig.savefig(output/'lidar-stop-results.png',dpi=160);plt.close(fig)
    if 'central' in summaries:
        rows=[json.loads(line) for line in (output/'central/trace.jsonl').read_text().splitlines()]
        t=[r['timestamp_s']-rows[0]['timestamp_s'] for r in rows]
        fig,axes=plt.subplots(1,2,figsize=(10,3.5))
        axes[0].plot(t,[r['speed_mps'] for r in rows]);axes[0].axhline(.2,color='gray',linestyle='--')
        axes[0].set(xlabel='Simulation time (s)',ylabel='Speed (m/s)',title='MVP2.1 + LiDAR stop')
        axes[1].plot(t,[r['evaluation_front_gap_m'] for r in rows],label='actual front gap')
        axes[1].axhline(.15,color='orange',linestyle='--',label='15 cm trigger')
        axes[1].axhline(0,color='red',linestyle='--',label='contact')
        axes[1].set(xlabel='Simulation time (s)',ylabel='Front gap (m)',title='Stops before contact')
        axes[1].legend(fontsize=8)
        for ax in axes:ax.grid(alpha=.2)
        fig.tight_layout();fig.savefig(output/'central-stop.png',dpi=160);plt.close(fig)
    lines += ['', 'coast is a deliberately retained zero-drive failure, not a successful stop.',
              'Mission completion and obstacle-stop success are evaluated separately.',
              'Scope: straight static obstacle, ideal 61-beam 180-degree sensor; no physical-car qualification.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=('empty','central','outside','coast','all'),default='all')
    parser.add_argument('--matrix', action='store_true', help='Run the bounded robustness matrix')
    args=parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory to preserve earlier evidence')
    if args.matrix:
        if args.scenario!='all':
            parser.error('--matrix cannot be combined with a single --scenario')
        from .robustness import run_matrix
        return run_matrix(args.output)
    args.output.mkdir(parents=True)
    source=source_snapshot();summaries={};payloads=[]
    for name in (('empty','central','outside','coast') if args.scenario=='all' else (args.scenario,)):
        summary,payload=run_case(name,args.output/name)
        summaries[name]=summary;payloads.append(payload)
    render_report(summaries,args.output)
    write_viewer(payloads,args.output)
    unchanged = source['sha256'] == source_snapshot()['sha256']
    result=dict(created_at=datetime.now(timezone.utc).isoformat(),source=source,
        source_unchanged_during_run=unchanged,
        simulation_control=SIMULATION_CONTROL,scenarios=summaries,
        all_expectations_met=unchanged and all(s['scenario_acceptance']['expectation_met'] and
            s['speed_tracking_simulation_check']['passed'] for s in summaries.values()))
    write_json(args.output/'suite.json',result)
    print(json.dumps({name:dict(acceptance=s['scenario_acceptance'],stop=s['obstacle_stop'])
                      for name,s in summaries.items()},ensure_ascii=False,indent=2))
    return 0 if result['all_expectations_met'] else 1


if __name__=='__main__':
    raise SystemExit(main())
