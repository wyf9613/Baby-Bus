"""Explicit, append-only continuation of the frozen boundary-road batch."""
import argparse,hashlib,json,os,traceback
from pathlib import Path
from .collect_boundary_policy import read,sha,source_hashes
from .collection_recovery import CollectionRecovery
from .boundary_collection import make_boundary_initializer
from .pipeline import PipelineSettings,collect_records,collection_episode_id
from .objective import CStarTrackingSettings
from .cstar import SolverSettings


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    os.replace(temporary,path)


def runtime_hashes():
    result=source_hashes();root=Path(__file__).resolve().parents[2]
    for name in ('offline/lattice_training/collection_recovery.py',
                 'offline/lattice_training/resume_boundary_policy.py',
                 '.verification/run_boundary_resume.py'):
        if name.startswith('.verification/') and not (root/name).is_file():continue
        result[name]=sha(root/name)
    return result


def expected_episodes(settings):
    return {collection_episode_id(kind,seed,index,profile,settings)
        for seeds in (settings.train_seeds,settings.validation_seeds,settings.test_seeds)
        for seed in seeds for kind in settings.kinds
        for index in range(settings.starts_per_road) for profile in settings.planner_profiles}


def prepare(batch,output,before_pipeline):
    batch=Path(batch).resolve();output=Path(output).resolve();before_pipeline=Path(before_pipeline)
    if output.exists():raise FileExistsError('Continuation directory already exists')
    settings=read(batch/'settings.json');current=source_hashes()
    changed={name for name,value in settings['runtime_source_sha256'].items() if current.get(name)!=value}
    if changed!={'offline/lattice_training/pipeline.py'}:
        raise ValueError('Only the recorded collection-resume pipeline extension may differ: '+repr(changed))
    if sha(before_pipeline)!=settings['runtime_source_sha256']['offline/lattice_training/pipeline.py']:
        raise ValueError('Pre-resume pipeline does not match original source fingerprint')
    if sha(batch/'input-manifest.json')!=settings['input_manifest_sha256']:
        raise ValueError('Original input manifest changed')
    manifest=read(batch/'input-manifest.json')
    for name,digest in manifest['frozen_inputs_sha256'].items():
        if sha(batch/'inputs'/(name+'.json'))!=digest:raise ValueError('Physical input changed: '+name)
    workers=[];originals={}
    for item in settings['workers']:
        number=item['worker'];directory=batch/'shards'/f'worker-{number}'
        recovery=CollectionRecovery([directory]);pipeline=PipelineSettings(**item['pipeline'])
        expected=expected_episodes(pipeline)
        if not recovery.completed<=expected or not set(recovery.prefixes)<=expected:
            raise ValueError('Saved episodes fall outside original schedule')
        workers.append(dict(worker=number,completed=len(recovery.completed),remaining=len(expected-recovery.completed),
            unfinished_prefixes={eid:len(rows) for eid,rows in recovery.prefixes.items()}))
        for name in ('decisions.jsonl','decisions.collection.json'):
            path=directory/name;originals[str(path.relative_to(batch))]=sha(path)
    if sum(w['completed'] for w in workers)!=71 or sum(w['remaining'] for w in workers)!=241:
        raise ValueError('This explicit continuation must preserve 71 completed and schedule exactly 241 remaining episodes')
    for name in ('settings.json','input-manifest.json','partition-plan.json'):
        originals[name]=sha(batch/name)
    output.mkdir(parents=True);(output/'logs').mkdir();(output/'shards').mkdir()
    (output/'pipeline-before.py').write_bytes(before_pipeline.read_bytes())
    plan=dict(batch=str(batch),status='PREPARED',remaining_episodes=241,already_completed=71,workers=workers,
        original_file_sha256=originals,runtime_source_sha256=runtime_hashes(),
        source_migration=dict(file='offline/lattice_training/pipeline.py',
            before_sha256=sha(before_pipeline),after_sha256=current['offline/lattice_training/pipeline.py'],
            scope='optional completed-episode skip and deterministic executed-prefix replay; default algorithm unchanged'),
        original_data_immutable=True,emit_only_new_decisions=True,automatic_training=False,
        next_stage_requires_user_feedback=True)
    atomic_json(output/'resume-plan.json',plan)
    atomic_json(output/'stage-status.json',dict(stage='collection_resume',status='PREPARED',remaining_episodes=241))
    print(json.dumps(dict(output=str(output),workers=workers,remaining_episodes=241)))


def verify(plan):
    if runtime_hashes()!=plan['runtime_source_sha256']:
        raise RuntimeError('Continuation runtime changed; refusing mixed-source labels')
    batch=Path(plan['batch'])
    for name,value in plan['original_file_sha256'].items():
        if sha(batch/name)!=value:raise RuntimeError('Immutable original file changed: '+name)


def worker(output,number):
    output=Path(output).resolve();plan=read(output/'resume-plan.json');verify(plan)
    batch=Path(plan['batch']);settings=read(batch/'settings.json');item=settings['workers'][number]
    directory=output/'shards'/f'worker-{number}';directory.mkdir()
    status=directory/'status.json'
    try:
        manifest=read(batch/'input-manifest.json');jobs={}
        for key,job in settings['jobs'].items():
            if job['worker']!=number:continue
            path=batch/'inputs'/(job['fixture']+'.json')
            if sha(path)!=manifest['frozen_inputs_sha256'][job['fixture']]:raise RuntimeError('Frozen map changed')
            jobs[int(key)]=dict(read(path),repetition=job['repetition'])
        recovery=CollectionRecovery([batch/'shards'/f'worker-{number}'])
        initializer=make_boundary_initializer(jobs,settings['input_manifest_sha256'])
        def guarded(*args,**kwargs):
            verify(plan)
            return initializer(*args,**kwargs)
        def progress(event):
            print(json.dumps(event,ensure_ascii=False),flush=True)
            atomic_json(status,dict(stage='collection_resume',status='RUNNING',worker=number,event=event))
            if event['stage']=='collect_episode_complete':
                # Per-episode files survive an interruption during a later rewrite
                # of the large aggregate report; this is producer checkpointing.
                scene=read(directory/'decisions.collection.json')['scenes'][-1]
                identity=hashlib.sha256(scene['episode_id'].encode()).hexdigest()
                atomic_json(directory/'episodes'/(identity+'.json'),scene)
        atomic_json(status,dict(stage='collection_resume',status='RUNNING',worker=number,
            remaining_episodes=plan['workers'][number]['remaining']))
        report=collect_records(directory/'decisions.jsonl',PipelineSettings(**item['pipeline']),
            CStarTrackingSettings.from_specification(settings['objective']),SolverSettings(**settings['solver']),
            progress=progress,environment_factory=guarded,recovery=recovery)
        verify(plan)
        if len(report['scenes'])!=plan['workers'][number]['remaining']:
            raise RuntimeError('Continuation episode coverage differs from frozen schedule')
        atomic_json(status,dict(stage='collection_resume',status='COMPLETE',worker=number,
            episodes=len(report['scenes']),new_decisions=report['decisions'],new_candidates=report['candidates'],
            replayed_prefix_decisions=report['replayed_prefix_decisions'],next_stage_requires_user_feedback=True))
    except BaseException as error:
        atomic_json(status,dict(stage='collection_resume',status='FAILED',worker=number,
            error=repr(error),traceback=traceback.format_exc()))
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('prepare');p.add_argument('--batch',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--before-pipeline',type=Path,required=True)
    p=sub.add_parser('worker');p.add_argument('--output',type=Path,required=True);p.add_argument('--worker',type=int,required=True)
    args=parser.parse_args()
    if args.mode=='prepare':prepare(args.batch,args.output,args.before_pipeline)
    else:worker(args.output,args.worker)


if __name__=='__main__':main()
