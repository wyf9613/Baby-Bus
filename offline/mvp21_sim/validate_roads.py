"""Fixed-road runtime validation: 20 Hz perception/control, 1 Hz native planner."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil

from offline.lattice_training.dataset import write_json
from offline.lattice_training.validation_roads import VALIDATION_KINDS, DIRECTORY, validation_case
from .runtime import run_runtime
from .source import ROOT


def read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line]


def timing_audit(directory):
    """Verify recorded acquisition/control/submission clocks and causal use."""
    rows = read_rows(directory/'trace.jsonl')
    submitted = read_rows(directory/'submissions.jsonl')
    delivered = read_rows(directory/'shadow.jsonl')
    tracking = [r for r in rows if r['phase']=='tracking']
    stamps = [r['observation']['sensor_stamp_ns']['cone_detections']/1e9 for r in tracking]
    all_stamps = [r['observation']['sensor_stamp_ns']['cone_detections']/1e9 for r in rows]
    times = [r['timestamp_s'] for r in rows]
    offered = [r['timestamp_s'] for r in submitted]
    ready = {r['result']['job_id']:r['delivery']['delivered_s'] for r in delivered}
    controls = [r for r in tracking if r['execution']['mode']=='planner']
    switches = sum(a.get('active_job_id')!=b.get('active_job_id') for a,b in zip(controls,controls[1:]))
    result = dict(control_period_50ms=all(abs(b-a-.05)<1e-8 for a,b in zip(times,times[1:])),
        camera_acquisition_period_50ms=all(abs(b-a-.05)<1e-8 for a,b in zip(all_stamps,all_stamps[1:])),
        planner_submission_period_1s=all(abs(b-a-1.)<1e-8 for a,b in zip(offered,offered[1:])),
        result_used_after_delivery=all(r['timestamp_s']+1e-9>=ready.get(r['active_job_id'],float('inf')) for r in controls),
        delivery_causal=all(r['delivery']['delivered_s']+1e-9>=r['delivery']['ready_s'] for r in delivered),
        zero_actions_after_fault=all(r['raw_action']==[0.,0.] for r in rows if r['phase']=='zero_hold'),
        camera_acquisitions=len(set(stamps)), camera_acquisitions_including_zero_hold=len(set(all_stamps)),
        camera_intervals_checked=max(0,len(all_stamps)-1), planner_intervals_checked=max(0,len(offered)-1),
        control_ticks=len(tracking), submissions=len(offered),
        deliveries=len(delivered), native_job_switches=switches)
    result['checks_passed'] = all(v for v in result.values() if isinstance(v,bool))
    return result


def protected_hashes():
    paths = [*sorted((ROOT/'mvp21_policy').glob('*.py')), ROOT/'config/ai4r_policy.yaml',
        ROOT/'offline/lattice_training/features.py', ROOT/'offline/lattice_training/cstar.py',
        ROOT/'offline/lattice_training/artifacts/2026-10-17-expanded-collection/model.json']
    return {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def run_suite(output, *, kinds=VALIDATION_KINDS, starts=('road_start','bend_start'), max_steps=2400,policy_model_path=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    protected = protected_hashes()
    write_json(output/'protected-before.json', protected)
    shutil.copytree(DIRECTORY, output/'road-definitions-v2')
    reports = []
    for start in starts:
        for kind in kinds:
            case = validation_case(kind)
            name = kind.removeprefix('validation_uturn_')+'__'+start
            directory = output/name
            summary = run_runtime(case, directory, seed=713, max_steps=max_steps,
                progress=None if start=='road_start' else case['cold_start_progress_m'],
                initial_speed=0., latency_steps=1, detailed_trace=True, stop_tail_s=1.,
                delivery_delay_s=.1, terminal_support_m=2.,policy_model_path=policy_model_path)
            write_json(directory/'road.json', case)
            audit = timing_audit(directory)
            write_json(directory/'timing-audit.json', audit)
            reports.append(dict(name=name, kind=kind, start=start, summary=summary, timing_audit=audit))
    unchanged = protected==protected_hashes()
    result = dict(format='mvp21_fixed_road_runtime_suite_v1', cases=reports,
        controller='mvp21_policy.control.PolicyController', execution='mvp21_policy.execution',
        perception_hz=20, control_hz=20, native_hz=1, camera_latency_s=.05,
        planner_delivery_delay_s=.1, cstar_controls_used=False, learned_policy_used=policy_model_path is not None,
        scenario_width_prior_used=False, common_controller_and_planner_parameters_unchanged=unchanged,
        timing_checks_passed=all(r['timing_audit']['checks_passed'] for r in reports),
        completed=sum(r['summary']['completed'] for r in reports),
        fault_counts=dict(Counter(r['summary']['fault'] for r in reports)))
    write_json(output/'suite.json', result)
    if not unchanged or not result['timing_checks_passed']:
        raise AssertionError('Source protection or clock/causality check failed')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--kinds', nargs='+', choices=VALIDATION_KINDS, default=VALIDATION_KINDS)
    parser.add_argument('--starts', nargs='+', choices=('road_start','bend_start'), default=('road_start','bend_start'))
    parser.add_argument('--max-steps', type=int, default=2400)
    parser.add_argument('--policy-model',type=Path,help='Explicit offline-only frozen policy ranking; default remains native')
    args = parser.parse_args()
    if args.max_steps < 1 or len(set(args.kinds))!=len(args.kinds) or len(set(args.starts))!=len(args.starts):
        parser.error('Positive max-steps and unique cases/start modes required')
    run_suite(args.output, kinds=args.kinds, starts=args.starts, max_steps=args.max_steps,policy_model_path=args.policy_model)


if __name__=='__main__':
    main()
