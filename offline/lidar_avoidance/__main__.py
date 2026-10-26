"""Reproducible observed-LiDAR static bypass suite; no ROS or hardware access."""
import argparse
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
import math
from pathlib import Path

from offline.lattice_training.dataset import write_json
from offline.lattice_training.visualize import scene_data
from offline.lidar_stop.__main__ import SIMULATION_CONTROL
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.source import ROOT
from offline.mvp21_sim.validation_workspace import normalize_road, source_snapshot, digest
from offline.mvp21_sim.visualize_runtime import compact_case, write_viewer


SCENARIOS = ('clear', 'central', 'left', 'right', 'blocked', 'dropout', 'stop_only')


def scenario(name):
    if name not in SCENARIOS:
        raise ValueError('Unknown avoidance scenario')
    spec = json.loads((ROOT/'offline/mvp21_sim/workspace/roads/quick-straight.json').read_text())
    spec['elements'][0]['geometry']['length_m'] = 5.
    spec['elements'][0]['lanes']['reference_lane']['width_m'] = dict(start=1.4, end=1.4)
    if name not in ('clear', 'dropout'):
        spec['obstacles'] = [dict(geometry=dict(type='rectangle', length_m=.2,
            width_m=1.3 if name == 'blocked' else .2), placement=dict(
                reference_line_progress_m=2.8, lateral_offset_m=.18 if name=='left' else -.18 if name=='right' else 0.,
                heading_offset_rad=0.))]
    case = normalize_road(spec, name='lidar_avoidance_'+name)
    case.update(title='LiDAR avoidance: '+name, collection='lidar_avoidance')
    return case


def run_case(name, output, *, seed=0, measured_delay=False, planner_rate_hz=5.):
    case = scenario(name)
    sensor = dict(period_steps=2, delay_steps=1)
    if name == 'dropout':
        sensor['dropout_windows_s'] = ((3., 6.),)
    summary = run_runtime(case, output, max_steps=700, seed=seed, detailed_trace=True,
        latency_steps=1, lattice_length_m=2.5, planner_rate_hz=planner_rate_hz,
        collision_coverage='generated', score_coverage='generated', delivery_delay_s=.1,
        measured_delay=measured_delay, handoff_budget_s=.5 if measured_delay else None,
        simulation_control_overrides=SIMULATION_CONTROL,
        obstacle_stop_config=dict(enabled=True, trigger_distance_m=.15, max_source_age_s=.30),
        simulation_brake_drive=-.1, stop_timeout_s=5., stopped_hold_s=1.,
        lidar_sensor_config=sensor,
        lidar_avoidance_config=None if name=='stop_only' else dict(enabled=True))
    write_json(output/'road.json', case)
    rows = [json.loads(line) for line in (output/'trace.jsonl').read_text().splitlines()]
    packets = [json.loads(line) for line in (output/'shadow.jsonl').read_text().splitlines()]
    tracking = [r for r in rows if r['phase']=='tracking']
    info = summary.get('lidar_avoidance') or {}
    stop = summary['obstacle_stop']
    contact = stop['obstacle_contact']
    returned = bool(rows and abs(rows[-1]['y_m']) <= .12 and abs(rows[-1]['heading_rad']) <= .20)
    peak = summary['maximum_executed_speed_mps']
    margins = [r['plant_corner_road_margin_m'] for r in rows]
    safe = bool(rows and not contact and min(margins) >= -1e-7 and peak <= .22)
    held = max(info.get('stopped_hold_s', 0.), stop.get('stopped_hold_s', 0.))
    protective = bool(not summary['completed'] and held >= 1.-1e-9
                      and stop['final_speed_mps'] <= .01 and safe)
    if name in ('blocked', 'dropout', 'stop_only'):
        expected = 'protective_stop' if name!='stop_only' else 'original_stop_control'
        passed = protective
        if name=='blocked':
            passed &= info.get('stop_reason') == 'no_feasible_avoidance_path'
        elif name=='dropout':
            passed &= (stop.get('trigger') or {}).get('reason') == 'lidar_stale_or_bad_timestamp'
        else:
            passed &= (stop.get('trigger') or {}).get('reason') == 'lidar_obstacle'
    else:
        expected = 'complete_and_return' if name!='clear' else 'clear_road_completion'
        passed = bool(summary['completed'] and safe and returned)
    rejected = Counter(c['avoidance']['rejected'] for p in packets
        for c in p['result'].get('candidates', []) if c.get('avoidance', {}).get('rejected'))
    acceptance = dict(name=name, passed=bool(passed), expected=expected,
        obstacle_contact=contact, peak_speed_mps=peak, final_lateral_offset_m=rows[-1]['y_m'] if rows else None,
        returned_to_centerline=returned, minimum_body_road_margin_m=min(margins, default=None),
        maximum_lateral_excursion_m=max((abs(r['y_m']) for r in rows), default=0.),
        stopped_hold_s=held, candidate_rejection_counts=dict(rejected),
        actual_native_cycles=summary['planner_reference_count'],
        actual_baseline_cycles=summary['baseline_reference_count'],
        startup_baseline_cycles=summary['startup_baseline_cycles'],
        causal_delivery=all(p['delivery']['delivered_s']+1e-9>=p['delivery']['ready_s'] for p in packets),
        control_clock_20hz=all(abs(b['timestamp_s']-a['timestamp_s']-.05)<1e-6
            for a,b in zip(tracking,tracking[1:])))
    acceptance['passed'] &= acceptance['causal_delivery'] and acceptance['control_clock_20hz']
    summary['avoidance_acceptance'] = acceptance
    write_json(output/'summary.json', summary)
    payload = compact_case(output.parent, dict(name=name, start='road_start', summary=summary,
        timing_audit=dict(checks_passed=acceptance['causal_delivery'] and acceptance['control_clock_20hz'])))
    # Keep full decisions in raw JSONL. Thin candidate geometry only for replay
    # size and overlay the actual hard rejection, including curvature/side gates.
    by_id = {p['result']['job_id']:p['result'] for p in packets}
    for delivery in payload['deliveries']:
        source = {c['id']:c for c in by_id[delivery['id']].get('candidates', [])}
        for candidate in delivery['candidates']:
            diagnostics = source[candidate['id']].get('avoidance') or {}
            candidate['avoidance'] = diagnostics
            if diagnostics.get('rejected'):
                candidate['collision'] = dict(reason=diagnostics['rejected'])
            path = candidate['path']
            candidate['path'] = path[::4]+([path[-1]] if path and path[-1]!=path[::4][-1] else [])
    print(json.dumps(dict(scenario=name, **acceptance), ensure_ascii=False), flush=True)
    return summary, payload


def report(summaries, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    figure, axes = plt.subplots(len(summaries), 2, figsize=(12, 3*len(summaries)), squeeze=False)
    lines = ['# MVP2.1 static LiDAR avoidance: offline experiment', '',
        'Observed cones/LiDAR and measured-motion odometry drive decisions. Ground truth is evaluation only.',
        'Road: 5 m straight, 1.4 m wide; static obstacle at 2.8 m. Native candidates plus smooth lateral holds.',
        'Explicit experiment: 2.5 m horizon, '+str(next(iter(summaries.values()))['lattice_generation_rate_hz'])+' Hz generation, 20 Hz control, 10 Hz LiDAR, 50 ms scan delay.',
        'Planner delivery: '+next(iter(summaries.values()))['delivery']['mode']+'; configured transport delay 100 ms.',
        'Common ROS YAML, learned model and vehicle dynamics are unchanged. Negative braking is simulator-only.', '',
        '| Case | Expected outcome met | Completed | Native / baseline cycles | Peak m/s | Final y m | Contact | Stop reason |',
        '|---|---|---|---|---:|---:|---|---|']
    for index, (name, s) in enumerate(summaries.items()):
        a=s['avoidance_acceptance'];rows=[json.loads(x) for x in (output/name/'trace.jsonl').read_text().splitlines()]
        scene=scene_data(s['environment']['road_spec'])
        xyaxis, speedaxis = axes[index]
        for side in ('left', 'right'):
            xyaxis.plot(*zip(*scene[side]), color='#64748b', linewidth=1.)
        for polygon in scene['obstacles']:
            xyaxis.add_patch(Polygon(polygon, color='#8b939d'))
        xyaxis.plot([r['x_m'] for r in rows],[r['y_m'] for r in rows],color='#157f75',linewidth=2.,label='Actual MVP trace')
        xyaxis.set(xlim=(0.,5.2),ylim=(-.9,.9),xlabel='World x (m)',ylabel='World y (m)',title=name)
        xyaxis.set_aspect('equal',adjustable='box');xyaxis.grid(alpha=.2)
        if rows:
            times=[r['timestamp_s']-rows[0]['timestamp_s'] for r in rows]
            speedaxis.plot(times,[r['speed_mps'] for r in rows],color='#157f75')
        speedaxis.axhline(.2,color='#64748b',linestyle='--')
        speedaxis.set(xlabel='Simulation time (s)',ylabel='Speed (m/s)',title='Observed plant speed');speedaxis.grid(alpha=.2)
        reason=(s.get('lidar_avoidance')or{}).get('stop_reason') or (s['obstacle_stop'].get('trigger')or{}).get('reason')
        lines.append(f"| {name} | {a['passed']} | {s['completed']} | {s['planner_reference_count']} / {s['baseline_reference_count']} | "
            f"{a['peak_speed_mps']:.4f} | {a['final_lateral_offset_m']:.4f} | {a['obstacle_contact']} | {reason} |")
    figure.tight_layout();figure.savefig(output/'avoidance-results.png',dpi=150);plt.close(figure)
    lines += ['', 'Startup baseline cycles remain counted; these results are not native-only acceptance.',
        'The 1.118 m road / obstacle at 1.7 m development trials stopped protectively; that tighter scenario is not qualified.',
        'Scope excludes dynamic obstacles, turns, reverse motion, general scan coverage and real-car operation.',
        'Sensing margin remains 0.08 m everywhere; future tracking allowance grows from 0 to 0.08 m over 0.65 m. Execution uses 0.08 m.',
        'These numerical margins and the 2.5 1/m curvature limit have not been calibrated on a real vehicle.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=(*SCENARIOS,'all'),default='all')
    parser.add_argument('--measured-delay',action='store_true',help='Include actual planner CPU time in causal delivery')
    parser.add_argument('--planner-rate-hz',type=float,choices=(1.,2.,5.),default=5.)
    args=parser.parse_args()
    if args.output.exists():parser.error('Use a new directory to preserve historical evidence')
    args.output.mkdir(parents=True)
    source=source_snapshot()
    for p in sorted(Path(__file__).parent.glob('*.py')):
        source['files'][p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    source['sha256']=digest(source['files']);write_json(args.output/'source.json',source)
    summaries={};payloads=[]
    for name in SCENARIOS if args.scenario=='all' else (args.scenario,):
        summary,payload=run_case(name,args.output/name,measured_delay=args.measured_delay,
            planner_rate_hz=args.planner_rate_hz);summaries[name]=summary;payloads.append(payload)
    write_viewer(payloads,args.output,split_data=True)
    report(summaries,args.output)
    current={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source['files']}
    if current!=source['files']:raise RuntimeError('Source changed during suite; retain records and rerun from a fresh process')
    passed=all(s['avoidance_acceptance']['passed'] for s in summaries.values())
    write_json(args.output/'suite.json',dict(created_at=datetime.now(ZoneInfo('Australia/Melbourne')).isoformat(),
        passed=passed,source=source,scenarios=summaries,hardware_used=False,ros_avoidance_enabled=False))
    raise SystemExit(0 if passed else 1)


if __name__=='__main__':
    main()
