"""Validate/merge immutable boundary collection shards, then freeze a training plan."""
import argparse,json,os
from collections import Counter,defaultdict
from contextlib import ExitStack
from pathlib import Path
import numpy as np
from .dataset import read_decisions,import_dataset,load_dataset,write_json,file_sha256,execution_label_contract,objective_hash
from .pipeline import collection_coverage,PipelineSettings,collection_episode_id
from .sequence import sequence_layout
from .model import SpatialPathPolicy
from .collection_recovery import assert_equal
from .resume_boundary_policy import expected_episodes
from offline.mvp21_sim.source import verify_preserved_files


def training_code_hashes():
    root=Path(__file__).resolve().parents[2]
    names=['offline/lattice_training/'+name+'.py' for name in
        ('model','spatial','sequence','features','source','dataset','objective','reference','plots','finalize_boundary_policy')]
    # Detached launchers are local orchestration, not a repository dependency.
    if (root/'.verification/run_boundary_training.py').is_file():
        names+=['.verification/run_boundary_training.py']
    return {name:file_sha256(root/name) for name in names}


def finalize(batch,output):
    batch=Path(batch).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError('Choose a new consolidation directory')
    settings=json.loads((batch/'settings.json').read_text(encoding='utf-8'))
    partition=json.loads((batch/'partition-plan.json').read_text(encoding='utf-8'))
    resume=batch/'resumes/resume-001';plan=json.loads((resume/'resume-plan.json').read_text(encoding='utf-8'))
    for name,value in plan['original_file_sha256'].items():
        if file_sha256(batch/name)!=value:raise ValueError('Original file changed: '+name)
    frozen_manifest=json.loads((batch/'input-manifest.json').read_text(encoding='utf-8'))
    frozen={}
    for fixture,digest in frozen_manifest['frozen_inputs_sha256'].items():
        path=batch/'inputs'/(fixture+'.json')
        if file_sha256(path)!=digest:raise ValueError('Physical input changed: '+fixture)
        frozen[fixture]=json.loads(path.read_text(encoding='utf-8'))
    expected=set().union(*(expected_episodes(PipelineSettings(**w['pipeline'])) for w in settings['workers']))
    directories=[*sorted((batch/'shards').iterdir()),*sorted((resume/'shards').iterdir())]
    scenes={};index=[];groups=set();fingerprints={};source_rows={};contract=None;by_episode=defaultdict(list)
    for directory in directories:
        for name in ('decisions.jsonl','decisions.collection.json'):
            fingerprints[str(directory/name)]=file_sha256(directory/name)
        report=json.loads((directory/'decisions.collection.json').read_text(encoding='utf-8'))
        if directory.parent.parent==resume:
            status=json.loads((directory/'status.json').read_text(encoding='utf-8'))
            life=json.loads((resume/'logs'/(directory.name+'.lifecycle.json')).read_text(encoding='utf-8'))
            if status['status']!='COMPLETE' or life['exit_code']!=0 or not report['collection_complete']:
                raise ValueError('Continuation is not complete: '+str(directory))
        for scene in report['scenes']:
            eid=scene['episode_id']
            if eid in scenes:raise ValueError('Duplicate completed episode: '+eid)
            job=settings['jobs'][str(scene['road_seed'])];item=frozen[job['fixture']]
            if scene['split']!=job['split'] or scene['split']!=partition['assignments'][item['case']['split_group']]:
                raise ValueError('Episode split differs from frozen construction-group partition')
            config=scene['environment_configuration']
            assert_equal(config['boundary_fixture'],item['case'],'frozen_geometry')
            assert_equal(config['physical_cones'],item['cones'],'frozen_cones')
            assert_equal(config['measurement_noise']['sigma_m'],item['sigma'],'radial_sigma')
            # Large immutable maps/failed planner diagnostics stay in their source;
            # the aggregate keeps full executed traces and exact source references.
            small={k:v for k,v in scene.items() if k not in ('environment_configuration','planner_fault')}
            small.update(fixture_id=job['fixture'],split_group_id=item['case']['split_group'],
                source_scene=dict(report=str(directory/'decisions.collection.json'),episode_id=eid),
                planner_fault_reason=(scene.get('planner_fault') or {}).get('reason'))
            scenes[eid]=small
        count=finished_count=0
        finished_ids={s['episode_id'] for s in report['scenes']}
        with (directory/'decisions.jsonl').open('rb') as stream:
            while True:
                offset=stream.tell();line=stream.readline()
                if not line:break
                row=json.loads(line);episode=row['episode'];group=row['group_id'];eid=episode['id']
                if group in groups:raise ValueError('Duplicate decision: '+group)
                groups.add(group)
                current=execution_label_contract(row['label_source'])
                if contract is None:contract=current
                if current!=contract:raise ValueError('C* execution semantics differ across shards')
                if row['objective']!=settings['objective'] or row['label_source']['solver_settings']!=settings['solver']:
                    raise ValueError('C* settings changed')
                job=settings['jobs'][str(episode['road_seed'])]
                if row['split']!=job['split'] or episode['split_group_id']!=frozen[job['fixture']]['case']['split_group']:
                    raise ValueError('Decision split or physical group changed')
                by_episode[eid].append(dict(step=episode['segment'],start=episode['start_state'],
                    control=episode['initial_control'],successor=row['successor'],count=len(row['candidates'])))
                key=(('train','validation','test').index(row['split']),episode['road_seed'],episode['initial_index'],episode['segment'])
                index.append((key,str(directory/'decisions.jsonl'),offset,group));count+=1
                finished_count+=eid in finished_ids
        source_rows[str(directory)]=count
        if finished_count!=report['decisions']:
            raise ValueError('Finalized shard row count disagrees with report')
        if report['collection_complete'] and count!=finished_count:
            raise ValueError('A complete shard has unfinished decision rows')
    if set(scenes)!=expected:raise ValueError('Missing or extra completed episodes')
    if not set(by_episode)<=expected:raise ValueError('Decision has unknown episode')
    for eid,scene in scenes.items():
        rows=sorted(by_episode[eid],key=lambda r:r['step'])
        if [r['step'] for r in rows]!=list(range(len(rows))) or len(rows)!=scene['decisions']:
            raise ValueError('Incomplete recurrent sequence: '+eid)
        for previous,current in zip(rows,rows[1:]):
            if not previous['successor']['accepted']:raise ValueError('Continued after rejected execution')
            action=previous['successor']['attempts'][-1]
            assert_equal(current['start'],action['end_state'],'sequence_state')
            assert_equal(current['control'],action['final_control'],'sequence_control')
    output.mkdir(parents=True)
    merged=output/'decisions.jsonl'
    with ExitStack() as stack,merged.open('wb') as target:
        opened={name:stack.enter_context(Path(name).open('rb')) for name in {row[1] for row in index}}
        for _,name,offset,group in sorted(index):
            opened[name].seek(offset);line=opened[name].readline()
            if json.loads(line)['group_id']!=group:raise ValueError('Input changed during indexed merge')
            target.write(line if line.endswith(b'\n') else line+b'\n')
    aggregate=dict(scenes=[scenes[eid] for eid in sorted(scenes)],decisions=len(index),collection_complete=True,
        base_road_splits={s['scene_id']:s['split'] for s in scenes.values()},
        physical_inputs=str(batch/'input-manifest.json'),source_reports=source_rows)
    coverage=collection_coverage(aggregate,require_road_end=True)
    write_json(output/'decisions.collection.json',aggregate)
    # Recompute every label from its original execution and encode the unchanged
    # structured inputs. This shared importer validates all three split contracts.
    metadata=import_dataset(merged,output/'dataset',allow_collection_variation=True)
    metadata.update(collection_coverage=coverage,road_coverage_complete=coverage['coverage_complete'],
        dataset_scope='39_qualified_boundary_roads_new_partition',partition=partition,
        immutable_source_sha256=fingerprints,raw_merged_records_sha256=file_sha256(merged))
    write_json(output/'dataset/metadata.json',metadata)
    data,_=load_dataset(output/'dataset');layout=sequence_layout(data['groups'],data)
    splits={}
    for split in ('train','validation','test'):
        subset=[s for s in scenes.values() if s['split']==split]
        mask=data['splits']==split
        split_data={k:v[mask] for k,v in data.items()}
        current_layout=sequence_layout(split_data['groups'],split_data)
        splits[split]=dict(roads=len({s['scene_id'] for s in subset}),episodes=len(subset),
            labeled_sequences=len(current_layout),decisions=len(set(data['groups'][mask])),candidates=int(mask.sum()),
            recurrent_links=sum(len(seq)-1 for seq in current_layout),goals_from_sampled_start=sum(s['road_completed'] for s in subset))
    for name,digest in fingerprints.items():
        if file_sha256(name)!=digest:raise ValueError('Source changed during consolidation')
    summary=dict(status='COMPLETE',episodes=len(scenes),physical_roads=len(frozen),decisions=len(index),candidates=len(data['y']),
        labeled_sequences=len(layout),empty_sequences=len(scenes)-len(layout),splits=splits,
        executed_branch_continuity_validated=True,labels_recomputed_from_saved_executions=True,
        fixed_group_partition_preserved=True,old_checkpoint_data_not_merged=True,
        outcomes=dict(Counter(s['terminal_behavior'] for s in scenes.values())),coverage=coverage,
        source_rows=source_rows,source_sha256=fingerprints,data_sha256=metadata['data_sha256'],
        readiness=metadata['training_readiness'],preserved_files=verify_preserved_files(),training_started=False)
    write_json(output/'merge-summary.json',summary)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('source_sha256','coverage','preserved_files')},ensure_ascii=False))
    return summary


def prepare_training(dataset,output,warm_start):
    dataset=Path(dataset).resolve();output=Path(output).resolve();warm_start=Path(warm_start).resolve()
    if output.exists():raise FileExistsError('Choose a new training directory')
    data,metadata=load_dataset(dataset);model=SpatialPathPolicy.load(warm_start)
    if not model.network.recurrent or model.network.hidden!=16 or model.target_temperature!=1.:
        raise ValueError('Expected the current 16-hidden recurrent checkpoint')
    if model.provenance['execution_condition']!=metadata['execution_condition']:
        raise ValueError('Fine-tuning execution condition differs')
    for split in ('train','validation','test'):
        mask=data['splits']==split;sequence_layout(data['groups'][mask],{k:v[mask] for k,v in data.items()})
    if not metadata['training_readiness']['eligible']:raise ValueError('Dataset is not training-ready')
    output.mkdir(parents=True);(output/'logs').mkdir()
    copied=output/'warm-start.json';copied.write_bytes(warm_start.read_bytes())
    root=Path(__file__).resolve().parents[2]
    protected=[warm_start,root/'mvp21_policy/models/path_policy.json',root/'config/ai4r_policy.yaml']
    plan=dict(dataset=str(dataset),output=str(output),warm_start=str(copied),warm_start_origin=str(warm_start),
        dataset_files_sha256={name:file_sha256(dataset/name) for name in ('data.npz','metadata.json','decisions.jsonl')},
        warm_start_sha256=file_sha256(copied),protected_file_sha256={str(p):file_sha256(p) for p in protected},
        training_source_sha256=training_code_hashes(),
        hyperparameters=dict(epochs=200,hidden=16,seed=42,learning_rate=.0002,recurrent=True,
            checkpoint='validation_feasible_then_return',early_stopping_patience=60),
        parameters=sum(p.size for p in model.parameters.values()),features_unchanged=True,
        normalization='preserved_training_only_statistics_from_warm_start',
        recurrent_state='fixed_executed_sequence_replay_current_weights_BPTT',test_used_for_selection=False,
        automatic_closed_loop_evaluation=False,automatic_deployment=False,next_stage_requires_user_feedback=True)
    write_json(output/'training-plan.json',plan)
    write_json(output/'status.json',dict(stage='rnn_training',status='PREPARED'))
    print(json.dumps(dict(output=str(output),candidates=len(data['y']),hyperparameters=plan['hyperparameters'])))


def main():
    parser=argparse.ArgumentParser(description=__doc__);commands=parser.add_subparsers(dest='mode',required=True)
    p=commands.add_parser('merge');p.add_argument('--batch',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p=commands.add_parser('prepare-training');p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--warm-start',type=Path,required=True)
    args=parser.parse_args()
    if args.mode=='merge':finalize(args.batch,args.output)
    else:prepare_training(args.dataset,args.output,args.warm_start)


if __name__=='__main__':main()
