"""Current-clock native/policy regression on exact archived random-road fixtures."""
import argparse
from collections import Counter
import json
from pathlib import Path

from offline.lattice_training.dataset import write_json
from .diagnose_perception import diagnose_case, annotate_branches
from .policy_selection import DEFAULT_MODEL
from .random_roads import saved_random_roads
from .runtime import run_runtime
from .validate_roads import protected_hashes, timing_audit


def run(output, *, max_steps=2400, model=DEFAULT_MODEL):
    output=Path(output);catalog=saved_random_roads();protected=protected_hashes()
    output.mkdir(parents=True,exist_ok=True)
    if (output/'catalog.json').exists():
        if json.loads((output/'catalog.json').read_text(encoding='utf-8'))!=catalog:
            raise ValueError('Resume catalog differs from saved physical roads')
        if json.loads((output/'protected-before.json').read_text())!=protected:
            raise ValueError('Resume owning modules/model/parameters changed')
    else:
        if any(output.iterdir()):raise FileExistsError('New output directory or valid catalog required')
        write_json(output/'catalog.json',catalog);write_json(output/'protected-before.json',protected)
    reports=[]
    for case in catalog['cases']:
        for mode in ('perception','no_centerline'):
            for backend in ('native','policy'):
                name=case['fixture_id']+'__'+mode
                directory=output/backend/name
                if (directory/'summary.json').exists():
                    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
                    if summary['road_input']!=mode or summary['selection_backend']!=backend or summary['runtime_max_steps']!=max_steps:
                        raise ValueError('Resume run condition differs')
                    if summary['environment']['road_spec']['elements'][:-1]!=case['road_spec']['elements']:
                        raise ValueError('Resume physical road differs')
                else:
                    summary=run_runtime(case,directory,road_input=mode,seed=case['runtime_seed'],max_steps=max_steps,
                        initial_speed=0.,latency_steps=1,detailed_trace=True,stop_tail_s=1.,
                        delivery_delay_s=.1,terminal_support_m=2.,policy_model_path=model if backend=='policy' else None)
                    write_json(directory/'road.json',case)
                audit=timing_audit(directory)
                if not audit['checks_passed']:raise AssertionError('Current-clock audit failed')
                write_json(directory/'timing-audit.json',audit)
                if not (directory/'perception-cones.json').exists():diagnose_case(directory)
                entry=dict(name=name,kind=case['kind'],start='road_start',road_input_mode=mode,
                    split=case['split'],road_seed=case['road_seed'],summary=summary,timing_audit=audit)
                reports.append(dict(backend=backend,**entry))
                for group in ('native','policy'):
                    entries=[{k:v for k,v in r.items() if k!='backend'} for r in reports if r['backend']==group]
                    (output/group).mkdir(exist_ok=True)
                    write_json(output/group/'suite.json',dict(cases=entries,learned_policy_used=group=='policy',
                        complete=len(entries)==len(catalog['cases'])*2))
                print(json.dumps(dict(stage='random_runtime_case',backend=backend,name=name,
                    finished=len(reports),total=len(catalog['cases'])*4,completed=summary['completed'],
                    fault=summary['fault'],steps=summary['steps']),ensure_ascii=False),flush=True)
    for backend in ('native','policy'):annotate_branches(output/backend)
    if protected!=protected_hashes():raise AssertionError('Protected algorithms/model changed')
    result=dict(format='random_road_current_clock_suite_v1',complete=True,cases=reports,
        physical_variants=len(catalog['cases']),control_hz=20,perception_hz=20,native_and_policy_hz=1,
        camera_latency_s=.05,delivery_delay_s=.1,real_time_qualified=False,
        cstar_controls_used=False,scenario_width_prior_used=False,retrained=False,main_policy_enabled=False,
        timing_checks_passed=all(r['timing_audit']['checks_passed'] for r in reports),
        completed=sum(r['summary']['completed'] for r in reports),
        fault_counts=dict(Counter(r['summary']['fault'] for r in reports)))
    write_json(output/'suite.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-steps',type=int,default=2400)
    args=parser.parse_args()
    if args.max_steps<1:parser.error('Positive maximum steps required')
    run(args.output,max_steps=args.max_steps)
