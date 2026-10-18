"""Expanded-road collection using the existing random legal C* continuation."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .cstar import SolverSettings
from .dataset import write_json
from .objective import CStarTrackingSettings
from .pipeline import PipelineSettings, collect_records
from .scenarios import SUITE_KINDS


def collect_suite(output, seeds, *, kinds=SUITE_KINDS, modes=('perception', 'no_centerline'),
                  initial_start_mode='default', use_scenario_width_prior=False, obstacle_layout=None, sample_count=8):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    jobs = []
    def progress(value):
        print(json.dumps(value, ensure_ascii=False), flush=True)
    objective = CStarTrackingSettings()
    solver = SolverSettings(population=6, iterations=1, max_evaluations=12, max_wall_s=30)
    for seed in seeds:
        for kind in kinds:
            for mode in modes:
                records = output/f'{kind}-{seed}-{mode}.jsonl'
                settings = PipelineSettings(kinds=(kind,), train_seeds=(seed,), validation_seeds=(),
                    test_seeds=(), train_only=True, starts_per_road=1, planner_profiles=('standard',),
                    until_road_end=True, max_segments=20, segment_resampling_attempts=8,
                    planner_sample_count=sample_count, planner_rate_hz=20, terminal_support_m=2, road_input_mode=mode,
                    initial_start_mode=initial_start_mode, use_scenario_width_prior=use_scenario_width_prior,
                    obstacle_layout=obstacle_layout)
                report = collect_records(records, settings, objective, solver, progress)
                scene = report['scenes'][0]
                job = dict(records=str(records), seed=seed, kind=kind, road_input_mode=mode,
                    scene_id=scene['scene_id'], decisions=report['decisions'], candidates=report['candidates'],
                    initial_start_mode=initial_start_mode,
                    use_scenario_width_prior=use_scenario_width_prior,
                    obstacle_layout=obstacle_layout,
                    trials=report['trials'], successful_trials=report['successful_trials'],
                    road_completed=scene['road_completed'], faults=scene['faults'],
                    initial_progress_m=scene['initial_progress_m'], final_progress_m=scene['final_progress_m'],
                    goal_progress_m=scene['goal_progress_m'], wall_s=report['wall_s'])
                jobs.append(job)
                write_json(output/'suite.json', dict(jobs=jobs, seeds=list(seeds), kinds=kinds, modes=modes,
                    solver_settings=asdict(solver), complete=False))
                progress(dict(stage='suite_job_complete', **job))
    result = dict(jobs=jobs, seeds=list(seeds), kinds=kinds, modes=modes,
                  solver_settings=asdict(solver), complete=True)
    write_json(output/'suite.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=lambda s: tuple(map(int, s.split(','))), required=True)
    parser.add_argument('--kinds', type=lambda s: tuple(s.split(',')), default=SUITE_KINDS)
    parser.add_argument('--modes', type=lambda s: tuple(s.split(',')), default=('perception', 'no_centerline'))
    parser.add_argument('--start-mode', choices=('default', 'road_start'), default='default')
    parser.add_argument('--scenario-width-prior', action='store_true')
    parser.add_argument('--obstacle-layout', choices=('near_center_alternating',))
    parser.add_argument('--candidate-count', type=int, default=8)
    args = parser.parse_args()
    collect_suite(args.output, args.seeds, kinds=args.kinds, modes=args.modes,
                  initial_start_mode=args.start_mode, use_scenario_width_prior=args.scenario_width_prior,
                  obstacle_layout=args.obstacle_layout, sample_count=args.candidate_count)
