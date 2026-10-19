"""Report C* failure selections separately from all-failed decisions.

No feasibility labels or C* results are used to alter policy predictions.
"""
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

from .dataset import load_dataset, read_decisions, write_json, file_sha256
from .features import encode_candidate
from .model import SpatialPathPolicy
from .scenarios import SUITE_KINDS


def failure_selection_metrics(records):
    """Pure metric aggregation, also suitable for validation-only model choice."""
    totals = defaultdict(lambda: dict(decisions=0, decisions_with_feasible=0,
        all_nominal_failed=0, selected_nominal_failed=0, avoidable_nominal_failed=0,
        selected_any_trial_failed=0, decisions_with_all_trials_feasible=0,
        avoidable_any_trial_failed=0, feasible_failed_pairs=0, correct_feasible_failed_pairs=0))
    cases = []
    for record in records:
        prediction = np.asarray(record['scores'])
        failed = np.asarray(record['nominal_failed'], dtype=bool)
        any_failed = np.asarray(record['any_failed'], dtype=bool)
        selected = max(range(len(prediction)), key=lambda i: (prediction[i], -int(record['candidate_ids'][i])))
        available = bool((~failed).any())
        axes = (record['split'], record['split']+'/'+record['kind'],
                record['split']+'/'+record['mode'], record['split']+'/'+
                ('width_prior' if record.get('width_prior') else 'default_width'),
                record['split']+('/fresh_roads' if record['kind'] in SUITE_KINDS else '/previous_roads'),
                record['split']+'/layout:'+(record.get('layout') or 'original'))
        for axis in axes:
            t = totals[axis]
            t['decisions'] += 1
            t['decisions_with_feasible'] += int(available)
            t['all_nominal_failed'] += int(not available)
            t['selected_nominal_failed'] += int(failed[selected])
            t['avoidable_nominal_failed'] += int(failed[selected] and available)
            t['selected_any_trial_failed'] += int(any_failed[selected])
            t['decisions_with_all_trials_feasible'] += int((~any_failed).any())
            t['avoidable_any_trial_failed'] += int(any_failed[selected] and (~any_failed).any())
            pairs = prediction[~failed, None] > prediction[None, failed]
            t['feasible_failed_pairs'] += pairs.size
            t['correct_feasible_failed_pairs'] += int(pairs.sum())
        if failed[selected] or any_failed[selected]:
            ids = record['candidate_ids']
            feasible = np.flatnonzero(~failed)
            best = max(feasible, key=lambda i: record['returns'][i]) if len(feasible) else None
            cases.append(dict(group_id=record['group_id'], split=record['split'],
                kind=record['kind'], mode=record['mode'], selected_candidate_id=ids[selected],
                layout=record.get('layout'),
                selected_score=float(prediction[selected]), selected_nominal_failed=bool(failed[selected]),
                selected_any_trial_failed=bool(any_failed[selected]), avoidable=available,
                feasible_candidate_ids=[ids[i] for i in feasible],
                best_feasible_candidate_id=ids[best] if best is not None else None,
                selected_return=float(record['returns'][selected]),
                best_feasible_return=float(record['returns'][best]) if best is not None else None,
                selected_failure_reasons=record['failure_reasons'][selected]))
    for t in totals.values():
        n = t['decisions_with_feasible']
        t['avoidable_failure_rate'] = t['avoidable_nominal_failed']/n if n else None
        n = t['decisions_with_all_trials_feasible']
        t['avoidable_any_trial_failure_rate'] = t['avoidable_any_trial_failed']/n if n else None
        n = t['feasible_failed_pairs']
        t['feasible_failed_pair_accuracy'] = t['correct_feasible_failed_pairs']/n if n else None
    return dict(metrics=dict(totals), cases=cases)


def audit_failures(model_path, dataset, output):
    dataset = Path(dataset)
    data, metadata = load_dataset(dataset)
    model = SpatialPathPolicy.load(model_path)
    if model.provenance['execution_condition'] != metadata['execution_condition']:
        raise ValueError('Model execution condition differs')
    rows = []
    predicted=model.predict_dataset(data)
    lookup={(str(group),str(candidate)):float(score) for group,candidate,score in
            zip(data['groups'],data['candidate_ids'],predicted)}
    nominal_status, reason_counts = defaultdict(int), defaultdict(int)
    feasible_targets, failure_targets = [], []
    for record in read_decisions(dataset/'decisions.jsonl'):
        entries = record['candidates']
        episode = record.get('episode') or {}
        failed = [e['cstar_trials'][0]['solver_status'] != 'approximate' for e in entries]
        values = [float(np.mean(e['label']['returns'])) for e in entries]
        targets = model.training_targets(values)
        for entry, status, target in zip(entries, failed, targets):
            nominal_status['failed' if status else 'approximate'] += 1
            (failure_targets if status else feasible_targets).append(float(target))
            for trial in entry['cstar_trials']:
                if trial['solver_status'] != 'approximate':
                    reason_counts[trial.get('reason', 'unspecified')] += 1
        rows.append(dict(group_id=record['group_id'], split=record['split'],
            kind=episode.get('kind', 'unknown'), mode=episode.get('road_input_mode', 'perception'),
            width_prior=bool(episode.get('use_scenario_width_prior')),
            layout=episode.get('obstacle_layout'),
            candidate_ids=[e['candidate']['candidate_id'] for e in entries],
            scores=[lookup[(record['group_id'],e['candidate']['candidate_id'])] for e in entries],
            nominal_failed=failed,
            any_failed=[any(t['solver_status'] != 'approximate' for t in e['cstar_trials']) for e in entries],
            failure_reasons=[[t.get('reason') for t in e['cstar_trials'] if t['solver_status'] != 'approximate'] for e in entries],
            returns=values))
    report = failure_selection_metrics(rows)
    report.update(format='cstar_failure_selection_audit_v1', model=str(model_path),
        model_sha256=file_sha256(model_path), dataset_sha256=metadata['data_sha256'],
        nominal_candidate_status=dict(nominal_status), trial_failure_reasons=dict(reason_counts),
        prediction_uses_cstar_feasibility=False, learned_closed_loop_run=False,
        target_distribution=dict(
            nominal_feasible=dict(count=len(feasible_targets), below_1e_6=sum(v < 1e-6 for v in feasible_targets)),
            nominal_failed=dict(count=len(failure_targets), below_1e_6=sum(v < 1e-6 for v in failure_targets))),
        score_records=rows)
    write_json(output, report)
    return report


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit_failures(args.model, args.dataset, args.output)
    print(json.dumps({k:v for k,v in report['metrics'].items() if '/' not in k}))
