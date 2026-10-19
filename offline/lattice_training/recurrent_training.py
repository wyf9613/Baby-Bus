"""Collect new speed/progress labels and fine tune executed-branch recurrent scoring."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from .cstar import SolverSettings
from .dataset import load_dataset,read_decisions,write_json
from .model import train_model,evaluate_policy
from .objective import CStarTrackingSettings
from .pipeline import PipelineSettings,collect_dataset
from .plots import plot_training
from .scenarios import SUITE_KINDS


def progress(value):print(json.dumps(value,ensure_ascii=False),flush=True)


def selection_failures(model,directory):
    data,_=load_dataset(directory);scores=model.predict_dataset(data)
    results={}
    for split in ('train','validation','test'):
        rows=[r for r in read_decisions(Path(directory)/'decisions.jsonl') if r['split']==split]
        counts=Counter()
        for record in rows:
            positions=np.flatnonzero(data['groups']==record['group_id'])
            selected=str(data['candidate_ids'][positions[np.argmax(scores[positions])]])
            failures={e['candidate']['candidate_id']:e['cstar_trials'][0]['solver_status'] not in ('optimal','approximate')
                      for e in record['candidates']}
            counts['decisions']+=1;counts['selected_nominal_failed']+=failures[selected]
            counts['all_nominal_failed']+=all(failures.values())
            counts['avoidable_nominal_failed']+=failures[selected] and not all(failures.values())
        results[split]=dict(counts)
    return results


def run(output,*,settings=None,solver=None,epochs=120,warm_start=None,collect_only=False):
    output=Path(output)
    settings=settings or PipelineSettings(kinds=(*SUITE_KINDS,'randomized'),
        train_seeds=tuple(range(1500,1510)),validation_seeds=(1600,1601,1602),test_seeds=(1700,1701,1702),
        until_road_end=True,max_segments=20,starts_per_road=1,planner_profiles=('standard',),
        planner_sample_count=8,trials=1,planner_rate_hz=1.,terminal_support_m=2.,
        cruise_speed_mps=.5,random_selection_probability=.5,width_range_m=(.75,1.6),
        single_side_dropout_probability=.65)
    solver=solver or SolverSettings(population=6,iterations=1,max_evaluations=12,max_wall_s=30)
    objective=CStarTrackingSettings(road_progress_weight=1.,road_progress_scale_m=1.)
    if warm_start is None:
        warm_start=Path(__file__).with_name('artifacts')/'2026-10-17-expanded-collection/model.json'
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True)
    write_json(output/'settings.json',dict(pipeline=asdict(settings),solver=asdict(solver),objective=objective.specification(),
        recurrence='fixed_executed_sequence_replayed_with_current_weights_BPTT',
        progress_reward='one_score_unit_per_meter_of_signed_road_reference_progress',
        warm_start=str(warm_start),epochs=epochs,hardware_enabled=False,training_requested=not collect_only))
    started=time.perf_counter()
    collection=collect_dataset(output,settings=settings,objective=objective,solver_settings=solver,progress=progress)
    if collect_only:
        progress(dict(stage='recurrent_collection_complete',**collection))
        return collection
    progress(dict(stage='recurrent_training_start',rows=collection['candidate_rows']))
    model,training=train_model(output/'dataset',output/'model.json',epochs=epochs,hidden=16,seed=42,
        learning_rate=.001,recurrent=True,warm_start=warm_start,progress=progress)
    evaluation=evaluate_policy(model,output/'dataset',output/'evaluation.json')
    plot_training(training,output)
    report=json.loads((output/'decisions.collection.json').read_text(encoding='utf-8'))
    counts=Counter();widths=[];gaps=[]
    for scene in report['scenes']:
        variation=scene.get('road_variation') or {}
        if variation:widths.append(variation['width_m']);gaps.extend(variation['physical_cone_gaps'])
        for item in scene['segments']:counts[item['behavior']]+=1
    result=dict(collection=collection,metrics=evaluation['splits'],selection_failures=selection_failures(model,output/'dataset'),
        behavior_decisions=dict(counts),road_kinds=dict(Counter(s['kind'] for s in report['scenes'])),
        width_range_observed_m=[min(widths),max(widths)] if widths else None,physical_cone_gap_count=len(gaps),
        trainable_parameters=sum(v.size for v in model.parameters.values()),hidden=16,
        recurrent_weights_norm=float(np.linalg.norm(model.parameters['recurrent_w'])),
        epochs=epochs,best_epoch=model.provenance['best_epoch'],wall_s=time.perf_counter()-started,
        policy_deployed=False,model_path=str(output/'model.json'))
    write_json(output/'recurrent-summary.json',result)
    progress(dict(stage='recurrent_training_complete',**result))
    return result


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--epochs',type=int,default=120)
    parser.add_argument('--train-seeds',type=lambda s:tuple(map(int,s.split(','))),default=tuple(range(1500,1510)))
    parser.add_argument('--validation-seeds',type=lambda s:tuple(map(int,s.split(','))),default=(1600,1601,1602))
    parser.add_argument('--test-seeds',type=lambda s:tuple(map(int,s.split(','))),default=(1700,1701,1702))
    parser.add_argument('--kinds',type=lambda s:tuple(s.split(',')),default=(*SUITE_KINDS,'randomized'))
    parser.add_argument('--max-segments',type=int,default=20)
    parser.add_argument('--max-evaluations',type=int,default=12)
    parser.add_argument('--trials',type=int,default=1)
    parser.add_argument('--planner-rate-hz',type=float,default=1.)
    parser.add_argument('--planner-sample-count',type=int,default=8)
    parser.add_argument('--starts-per-road',type=int,default=1)
    parser.add_argument('--initial-start-mode',choices=('default','road_start','stratified_random'),default='default')
    parser.add_argument('--iterations',type=int,default=1)
    parser.add_argument('--population',type=int,default=6)
    parser.add_argument('--collect-only',action='store_true')
    args=parser.parse_args()
    settings=PipelineSettings(kinds=args.kinds,train_seeds=args.train_seeds,validation_seeds=args.validation_seeds,
        test_seeds=args.test_seeds,until_road_end=True,max_segments=args.max_segments,starts_per_road=args.starts_per_road,
        initial_start_mode=args.initial_start_mode,
        planner_profiles=('standard',),planner_sample_count=args.planner_sample_count,trials=args.trials,planner_rate_hz=args.planner_rate_hz,
        terminal_support_m=2.,cruise_speed_mps=.5,random_selection_probability=.5,width_range_m=(.75,1.6),
        single_side_dropout_probability=.65)
    run(args.output,settings=settings,epochs=args.epochs,collect_only=args.collect_only,
        solver=SolverSettings(population=args.population,iterations=args.iterations,max_evaluations=args.max_evaluations,max_wall_s=30))


if __name__=='__main__':main()
