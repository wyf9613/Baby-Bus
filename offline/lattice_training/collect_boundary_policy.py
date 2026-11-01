"""Staged collection launcher for the 39 qualified boundary-first roads.

Prepare once, launch independent workers, then stop at the collection boundary.
Dataset consolidation and RNN training require the next user-triggered stage.
"""
from collections import Counter,defaultdict
from dataclasses import asdict
from pathlib import Path
import argparse,hashlib,json,traceback
from .dataset import write_json
from .pipeline import PipelineSettings,collect_records
from .cstar import SolverSettings
from .objective import CStarTrackingSettings
from .boundary_collection import make_boundary_initializer
from offline.mvp21_sim.source import verify_preserved_files


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    root=Path(__file__).resolve().parents[2]
    files=list((root/'mvp21_policy').glob('*.py'))+list((root/'offline/planner_sim').glob('*.py'))
    files+=[root/'offline/mvp21_sim'/n for n in ('source.py','simulation.py')]
    files+=[Path(__file__).parent/n for n in ('collect_boundary_policy.py','boundary_collection.py','boundary_roads.py',
        'robust_roads.py','screen_roads.py','pipeline.py','cstar.py','objective.py','planner.py','reference.py','features.py','dataset.py')]
    files+=[root/'config/ai4r_policy.yaml']
    return {str(p.relative_to(root)).replace('\\','/'):sha(p) for p in sorted(files)}


def prepare(output,pool,*,starts=4,repeats=2,workers=3):
    output=Path(output).resolve();pool=Path(pool).resolve()
    if output.exists():raise FileExistsError('Choose a new experiment output directory')
    if min(starts,repeats,workers)<1:raise ValueError('Starts, repeats and workers must be positive')
    verify_preserved_files();selection=read(pool/'selection.json');index=read(pool/'index.json')
    entries={e['case']['fixture_id']:e for e in index['cases']};names=selection['qualified']
    if len(names)!=39:raise ValueError('This requested batch must contain exactly the 39 qualified roads')
    output.mkdir(parents=True);(output/'inputs').mkdir();(output/'logs').mkdir();(output/'shards').mkdir()
    assignments={};jobs={};family_counts=defaultdict(Counter);road_counts=Counter();worker_splits=[defaultdict(list) for _ in range(workers)]
    # Freeze before C* labels are observed. Preserve whole construction-seed
    # groups, including mirror pairs and every start/repetition of a road.
    validation_groups={'boundary_seed_4108','boundary_seed_4109'}
    test_groups={'boundary_seed_4102','boundary_seed_4107'}
    maps={}
    for i,name in enumerate(names):
        entry=entries[name];assert entry['system_pass']
        case=entry['case'];group=case['split_group']
        split='validation' if group in validation_groups else 'test' if group in test_groups else 'train'
        if assignments.setdefault(group,split)!=split:raise ValueError('Construction group split conflict')
        road_counts[split]+=1;family_counts[split][case['kind']]+=1
        conefile=pool/name/'irregular/cones.json';cones=read(conefile)['cones']
        sigma=next(v['radial_sigma_base_m'] for v in entry['views'] if v['view']=='noisy')
        frozen=dict(case=case,cones=cones,sigma=sigma,source_cones_sha256=sha(conefile),split=split)
        write_json(output/'inputs'/(name+'.json'),frozen);maps[name]=sha(output/'inputs'/(name+'.json'))
        for repeat in range(repeats):
            seed=731000+i*repeats+repeat;worker=i%workers
            jobs[str(seed)]=dict(fixture=name,split=split,repetition=repeat,worker=worker)
            worker_splits[worker][split].append(seed)
    if set(road_counts)!={'train','validation','test'}:raise ValueError('Missing data partition')
    if set(family_counts['train'])!={'s_left','s_right','u_left','u_right','compound_left','compound_right'}:
        raise ValueError('Training partition must retain all six families')
    base=PipelineSettings(kinds=('boundary_fixture',),until_road_end=True,max_segments=20,starts_per_road=starts,
        planner_profiles=('standard',),planner_sample_count=8,trials=1,planner_rate_hz=1.,terminal_support_m=2.,
        initial_start_mode='stratified_random',cruise_speed_mps=.5,random_selection_probability=.5)
    worker_settings=[]
    for i,splits in enumerate(worker_splits):
        if not all(splits[s] for s in ('train','validation','test')):raise ValueError('Worker must cover nonempty split lists')
        settings=asdict(base)
        settings.update(train_seeds=splits['train'],validation_seeds=splits['validation'],test_seeds=splits['test'])
        worker_settings.append(dict(worker=i,pipeline=settings,expected_episodes=sum(map(len,splits.values()))*starts))
    objective=CStarTrackingSettings(road_progress_weight=1.,road_progress_scale_m=1.)
    solver=SolverSettings(population=12,iterations=5,max_evaluations=64,max_wall_s=30.)
    manifest=dict(source_pool=str(pool),selection_sha256=sha(pool/'selection.json'),qualified=names,frozen_inputs_sha256=maps)
    write_json(output/'input-manifest.json',manifest)
    partition=dict(method='new_grouped_partition_frozen_before_labels',assignments=assignments,
        road_counts=dict(road_counts),family_counts={s:dict(v) for s,v in family_counts.items()},
        grouping='same construction seed, mirrored roads, all starts and repetitions',
        previous_screening_holdouts_retired=True,right_compound_only_available_in_training=True)
    write_json(output/'partition-plan.json',partition)
    settings=dict(stage='collection_prepared',physical_roads=len(names),starts_per_road=starts,repetitions=repeats,
        expected_episodes=len(names)*starts*repeats,workers=worker_settings,jobs=jobs,
        solver=asdict(solver),objective=objective.specification(),input_manifest_sha256=sha(output/'input-manifest.json'),
        runtime_source_sha256=source_hashes(),input_schema_unchanged=True,
        stages=['collect_candidates_and_cstar_labels','consolidate_validate_partition','train_rnn_and_test'],
        advance_only_on_user_feedback=True,automatic_monitoring=False,automatic_training=False,
        sampling='candidate -> C* -> 50/50 random/native legal full-path continuation',
        recurrent_state='replay fixed executed sequence with current training weights; reset per episode')
    write_json(output/'settings.json',settings)
    write_json(output/'stage-status.json',dict(stage='collection',status='PREPARED',next_stage_requires_user_feedback=True))
    print(json.dumps(dict(output=str(output),roads=len(names),episodes=settings['expected_episodes'],partition=dict(road_counts),workers=workers)))


def worker(output,number):
    output=Path(output).resolve();settings=read(output/'settings.json');worker=settings['workers'][number]
    directory=output/'shards'/f'worker-{number}';directory.mkdir()
    status=directory/'status.json'
    try:
        if source_hashes()!=settings['runtime_source_sha256']:raise RuntimeError('Collection runtime changed after preparation')
        jobs={}
        for key,job in settings['jobs'].items():
            if job['worker']!=number:continue
            path=output/'inputs'/(job['fixture']+'.json')
            manifest=read(output/'input-manifest.json')
            if sha(path)!=manifest['frozen_inputs_sha256'][job['fixture']]:raise RuntimeError('Frozen physical fixture changed')
            jobs[int(key)]=dict(read(path),repetition=job['repetition'])
        initializer=make_boundary_initializer(jobs,settings['input_manifest_sha256'])
        def guarded_initializer(*args,**kwargs):
            if source_hashes()!=settings['runtime_source_sha256']:raise RuntimeError('Runtime source changed during collection; refusing mixed labels')
            return initializer(*args,**kwargs)
        def progress(event):
            print(json.dumps(event,ensure_ascii=False),flush=True)
            write_json(status,dict(stage='collection',status='RUNNING',worker=number,event=event))
        write_json(status,dict(stage='collection',status='RUNNING',worker=number,expected_episodes=worker['expected_episodes']))
        report=collect_records(directory/'decisions.jsonl',PipelineSettings(**worker['pipeline']),
            CStarTrackingSettings.from_specification(settings['objective']),SolverSettings(**settings['solver']),
            progress=progress,environment_factory=guarded_initializer)
        write_json(status,dict(stage='collection',status='COMPLETE',worker=number,
            episodes=len(report['scenes']),decisions=report['decisions'],candidates=report['candidates'],
            next_stage_requires_user_feedback=True))
    except BaseException as error:
        write_json(status,dict(stage='collection',status='FAILED',worker=number,error=repr(error),traceback=traceback.format_exc()))
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('prepare');p.add_argument('--output',type=Path,required=True);p.add_argument('--pool',type=Path,required=True)
    p.add_argument('--starts',type=int,default=4);p.add_argument('--repeats',type=int,default=2);p.add_argument('--workers',type=int,default=3)
    p=sub.add_parser('worker');p.add_argument('--output',type=Path,required=True);p.add_argument('--worker',type=int,required=True)
    args=parser.parse_args()
    if args.mode=='prepare':prepare(args.output,args.pool,starts=args.starts,repeats=args.repeats,workers=args.workers)
    else:worker(args.output,args.worker)


if __name__=='__main__':main()
