"""Re-solve selected failed cases after exact replay, without changing labels."""
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np

from .cstar import CStarSolver, SolverSettings, execute_trial, NativePrediction
from .dataset import read_decisions, write_json, file_sha256
from .objective import CStarTrackingSettings, reference_plan, path_completion
from .pipeline import PipelineSettings, initialize_episode
from .roads import RoadRandomizationSettings


def recheck_case(dataset, case, *, solver_settings=None):
    dataset = Path(dataset)
    selected = next(r for r in read_decisions(dataset/'decisions.jsonl') if r['group_id'] == case['group_id'])
    source = Path(selected.get('partition_origin', {}).get('input_file', dataset/'decisions.jsonl'))
    if not source.is_file():
        # A portable merged archive still contains the complete episode prefix.
        source = dataset/'decisions.jsonl'
    episode = selected['episode']
    history = []
    for record in read_decisions(source):
        if record.get('episode', {}).get('id') == episode['id']:
            history.append(record)
        if record['group_id'] == selected['group_id']:
            break
    if not history or history[-1]['group_id'] != selected['group_id']:
        raise ValueError('Original complete episode prefix unavailable')
    labels = selected['label_source']
    settings = PipelineSettings(**labels['collection_settings'])
    objective = CStarTrackingSettings.from_specification(selected['objective'])
    original_safety = SolverSettings(**labels['solver_settings'])
    expanded = dict(asdict(original_safety), population=12, iterations=4, max_evaluations=64, max_wall_s=60.)
    strong = solver_settings or SolverSettings(**expanded)
    road_settings = RoadRandomizationSettings(**labels['road_randomization']['settings'])
    environment, _, _ = initialize_episode(episode['kind'], episode['road_seed'], episode['initial_index'],
                                           settings, objective, road_settings, original_safety)
    with environment:
        replayed_steps = 0
        for prior in history[:-1]:
            transition = prior['successor']['attempts'][-1]
            if not prior['successor']['accepted']:
                raise ValueError('Cannot replay through a failed successor')
            for point in transition['trace']:
                environment.step(point['action'])
                replayed_steps += 1
            np.testing.assert_allclose(environment._backend.car._state_values(), transition['end_state'], rtol=0, atol=1e-9)
        error = float(np.max(np.abs(np.asarray(environment._backend.car._state_values())-episode['start_state'])))
        if error > 1e-9 or abs(environment._frame['timestamp_s']-selected['observation']['timestamp_s']) > 1e-9:
            raise ValueError('Reconstructed decision state/time differs')
        entry = next(e for e in selected['candidates'] if e['candidate']['candidate_id'] == case['selected_candidate_id'])
        seed = entry['cstar_trials'][0]['performance']['seed']
        solver = CStarSolver(objective, strong)
        prediction = NativePrediction(environment, strong)
        physical_body_check = NativePrediction(environment, replace(strong, safety_margin_m=0.))
        plan = reference_plan(entry['candidate'], objective)
        initialization_checks = {}
        for name in ('warm_start', 'transient_warm_start'):
            controls = getattr(solver, name)(prediction, plan['states'])
            states, margins = prediction.predict(controls)
            physical_states, physical_margins = physical_body_check.predict(controls)
            np.testing.assert_array_equal(states, physical_states)
            initialization_checks[name] = dict(negative_margins={k:v for k,v in margins.items() if v < -1e-7},
                physical_body_obstacle_contact=physical_margins['obstacle_contact_margin'] < 0,
                cstar_safety_margin_m=strong.safety_margin_m,
                completion=path_completion(entry['candidate'], states, objective, plan=plan))
        solved = solver.solve(environment, entry['candidate'], episode['initial_control'], seed=seed)
        trials = [execute_trial(environment, entry['candidate'], solved, episode['initial_control'], objective,
                                strong, trial_id=t['trial_id'], profile=t['performance']) for t in entry['cstar_trials']]
    def compact(trial):
        return {k: trial[k] for k in ('solver_status', 'constraints_satisfied', 'reason', 'constraint_margins',
            'completion', 'actual_loss', 'replay_max_state_error') if k in trial}
    return dict(group_id=case['group_id'], candidate_id=case['selected_candidate_id'],
        split=selected['split'], kind=episode['kind'], mode=episode.get('road_input_mode', 'perception'),
        original_source=str(source), original_source_sha256=file_sha256(source),
        replayed_steps=replayed_steps, reconstructed_state_max_error=error,
        original_solver_settings=asdict(original_safety), diagnostic_solver_settings=asdict(strong),
        original_trials=[compact(t) for t in entry['cstar_trials']],
        initializer_diagnostics=initialization_checks,
        diagnostic_solve_status=solved['status'], diagnostic_solve_reason=solved['reason'],
        diagnostic_solver=solved['diagnostics'], diagnostic_trials=[compact(t) for t in trials],
        original_labels_modified=False, model_predictions_modified=False,
        larger_budget_failure_does_not_prove_infeasibility=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int, default=6)
    args = parser.parse_args()
    cases = json.loads(args.audit.read_text())['cases']
    cases.sort(key=lambda c: (not c['avoidable'], c['split'] == 'train', c['group_id']))
    results, seen = [], set()
    for case in cases:
        key = case['kind'], case['mode'], case.get('layout'), case['selected_nominal_failed']
        if key in seen:
            continue
        seen.add(key)
        result = recheck_case(args.dataset, case)
        results.append(result)
        write_json(args.output, dict(cases=results, labels_changed=False, complete=False))
        print(json.dumps({k: result[k] for k in ('group_id', 'diagnostic_solve_status', 'diagnostic_solve_reason') }), flush=True)
        if len(results) >= args.limit:
            break
    write_json(args.output, dict(cases=results, labels_changed=False, complete=True))
