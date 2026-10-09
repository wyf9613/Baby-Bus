#!/usr/bin/env python3
"""Compare legacy Cartesian refitting with direct checked lattice samples.

Uses shipped MVP2 search settings, identical fresh synthetic inputs and actual
planner code. Full-search runs freeze the planner clock (never deadline evidence);
35 ms runs use the real clock. Local wall times are not target-car CPU evidence.
"""
import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path
import runpy
import statistics
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
api = runpy.run_path(str(ROOT/'tests/test_lattice.py'))
ns = api['ns']
params = yaml.safe_load((ROOT/'config/ai4r_policy.yaml').read_text())['/**/ai4r_policy']['ros__parameters']


def timing(values):
    values = sorted(v*1000 for v in values)
    return {'median_ms': statistics.median(values), 'p95_ms': values[int(.95*(len(values)-1))],
            'max_ms': max(values)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repeats', type=int, default=30)
    parser.add_argument('--output', type=Path, default=ROOT/'.verification/lattice-direct-output-study.json')
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('repeats must be positive')
    cfg = ns['LatticeSettings'](**params['planning']['lattice'])
    settings = ns['PlanningSettings'](**{k: v for k, v in params['planning'].items() if k != 'lattice'})
    scenes = [('straight_offset', lambda x: .1), ('gentle_004', lambda x: .04*x*x),
              ('bend_012', lambda x: .12*x*x), ('bend_025', lambda x: .25*x*x),
              ('s_curve', lambda x: .15*math.sin(2*x)),
              ('varying_curvature', lambda x: .04*x*x+.012*x*x*x)]
    results = []
    for name, shape in scenes:
        inputs = api['fixture'](speed=.1)
        for key, side in (('centerline_xy', 0), ('left_boundary_xy', .5), ('right_boundary_xy', -.5)):
            inputs[0][key] = [(x, shape(x)+side) for x, _ in inputs[0][key]]
        row = {'scene': name}
        for bounded in (False, True):
            mode = 'budget_35ms' if bounded else 'full_search_frozen_clock'
            runs = {False: [], True: []}
            reasons = {False: Counter(), True: Counter()}
            selected = {False: [], True: []}
            for repeat in range(args.repeats):
                # Alternate measurement order to reduce warm-cache/order bias.
                for direct in ((False, True) if repeat%2 == 0 else (True, False)):
                    p = ns['FrenetLatticePlanner'](settings, replace(cfg, direct_sample_output=direct),
                        clock=time.monotonic if bounded else lambda: 0.0)
                    start = time.perf_counter()
                    ref, diag = p.plan(*inputs)
                    runs[direct].append(time.perf_counter()-start)
                    reasons[direct][ref['reason']] += 1
                    selected[direct].append(bool(ref['valid']))
            row[mode] = {('direct_samples' if direct else 'legacy_fit'):
                {**timing(runs[direct]), 'accepted': sum(selected[direct]), 'runs': args.repeats,
                 'reasons': dict(reasons[direct])} for direct in (False, True)}
        results.append(row)
        print(json.dumps(row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'profile': 'shipped MVP2 secondary search, synthetic 1 m corridors',
        'clock_note': 'frozen clock only for full-search comparison; real clock for budget_35ms',
        'results': results}, indent=2)+'\n')


if __name__ == '__main__':
    main()
