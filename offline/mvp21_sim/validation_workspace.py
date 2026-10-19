"""Road ingestion, immutable result records and automatic replay indexing.

Readers use JSON records only. Simulation imports are deliberately local to the
runner: indexing/serving never executes planners, perception, policy or control.
"""
from copy import deepcopy
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from uuid import uuid4

ROOT=Path(__file__).resolve().parents[2]


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.'+uuid4().hex+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


def source_snapshot():
    paths=[*sorted((ROOT/'mvp21_policy').glob('*.py')),*sorted((ROOT/'offline/mvp21_sim').glob('*.py')),
        *sorted((ROOT/'offline/lattice_training').glob('*.py')),*sorted((ROOT/'offline/planner_sim').glob('*.py')),
        *sorted((ROOT/'offline/lidar_stop').glob('*.py')),
        ROOT/'config/ai4r_policy.yaml', ROOT/'offline/mvp21_sim/preserved-files.json']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return dict(sha256=digest(hashes),files=hashes)


def normalize_road(value,*,name='custom'):
    from offline.lattice_training.roads import road_length
    if value.get('format') in ('fixed_uturn_validation_road_v1','runtime_road_fixture_v1'):
        case=deepcopy(value)
    else:
        spec=deepcopy(value.get('road_spec',value))
        if not isinstance(spec.get('elements'),list) or not spec['elements']:raise ValueError('Road must contain nonempty elements')
        if not isinstance(spec.get('cone_paths'),dict):raise ValueError('Road cone_paths required')
        spec.setdefault('obstacles',[])
        case=dict(format='runtime_road_fixture_v1',kind='custom',variant='custom',direction='left',
            title=name,collection='custom',road_spec=spec,mission_road_length_m=road_length(spec),runtime_seed=0)
    if abs(road_length(case['road_spec'])-case['mission_road_length_m'])>1e-8:raise ValueError('Road mission length differs from geometry; terminal support must be separate')
    case.setdefault('fixture_id',case.get('kind','custom')+'_'+digest(case['road_spec'])[:12])
    case.setdefault('title',case['fixture_id']);case.setdefault('runtime_seed',713)
    return case


def ingest(workspace,path):
    path=Path(path);case=normalize_road(read(path),name=path.stem)
    destination=Path(workspace)/'roads'/(case['fixture_id']+'.json');write(destination,case)
    return destination


def initialize(workspace,*,seed_roads=True,import_history=True):
    from .validation_config import DEFAULT
    workspace=Path(workspace);workspace.mkdir(parents=True,exist_ok=True)
    if not (workspace/'validation.json').exists():
        profile=deepcopy(DEFAULT);profile['model_path']=Path(profile['model_path']).relative_to(ROOT).as_posix()
        write(workspace/'validation.json',profile)
    (workspace/'roads').mkdir(exist_ok=True);(workspace/'runs').mkdir(exist_ok=True)
    if seed_roads:
        from offline.lattice_training.validation_roads import validation_case,VALIDATION_KINDS
        from .random_roads import saved_random_roads
        for case in [*(validation_case(k) for k in VALIDATION_KINDS),*saved_random_roads()['cases']]:
            case=normalize_road(case);destination=workspace/'roads'/(case['fixture_id']+'.json')
            if not destination.exists():write(destination,case)
    if import_history:
        path=ROOT/'offline/lattice_training/artifacts/2026-10-18-current-frequency-policy-evaluation/viewer-manifest.json'
        if path.exists():write(workspace/'imports.json',[path.relative_to(ROOT).as_posix()])
    return index(workspace)


def roads(workspace):
    items=[];errors=[]
    for path in sorted((Path(workspace)/'roads').rglob('*.json')):
        try:items.append((path,normalize_road(read(path),name=path.stem)))
        except Exception as error:errors.append(dict(path=str(path),error=str(error)))
    return items,errors


def index(workspace):
    workspace=Path(workspace);latest={};errors=[];history=[]
    current_source=source_snapshot()['sha256']
    for path in sorted((workspace/'runs').glob('*/record.json')):
        try:
            record=read(path)
            if record['status']!='complete':
                errors.append(dict(path=str(path),status=record['status'],error=record.get('error')));continue
            if record.get('source_changed_during_run'):
                errors.append(dict(path=str(path),status='stale',error='Source changed during run; waiting for revalidation'));continue
            meta=record['case'];meta['data_url']='/runs/'+path.parent.name+'/case.json'
            meta['record_url']='/runs/'+path.parent.name+'/record.json'
            meta['current_source']=record['source']['sha256']==current_source
            history.append(meta)
            if meta['name'] not in latest or record['created_utc']>latest[meta['name']]['created_utc']:
                latest[meta['name']]=dict(created_utc=record['created_utc'],case=meta)
        except Exception as error:errors.append(dict(path=str(path),error=str(error)))
    imports=read(workspace/'imports.json') if (workspace/'imports.json').exists() else []
    for source in imports:
        try:
            path=(ROOT/source).resolve();path.relative_to(ROOT/'offline/lattice_training/artifacts')
            for case in read(path)['cases']:
                payload=(path.parent/case['data_url']).resolve();payload.relative_to(ROOT/'offline/lattice_training/artifacts')
                meta=deepcopy(case);meta['data_url']='/archive/'+payload.relative_to(ROOT/'offline/lattice_training/artifacts').as_posix()
                latest.setdefault(meta['name'],dict(created_utc='',case=meta))
        except Exception as error:errors.append(dict(path=source,error=str(error)))
    cases=[v['case'] for _,v in sorted(latest.items())]
    _,road_errors=roads(workspace)
    template=ROOT/'offline/mvp21_sim/runtime_viewer.html'
    return dict(format='validation_replay_index_v1',cases=cases,revision=digest(cases),history_count=len(history),
        latest_case_name=max(latest.values(),key=lambda v:v['created_utc'])['case']['name'] if latest else None,
        index_url='/api/index',viewer_sha256=hashlib.sha256(template.read_bytes()).hexdigest(),errors=errors+road_errors)


def clock_audit(directory,profile):
    from .validate_roads import read_rows
    rows=read_rows(directory/'trace.jsonl');submissions=read_rows(directory/'submissions.jsonl');packets=read_rows(directory/'shadow.jsonl')
    dt=1/profile['frequencies_hz']['physics'];controls=[r for r in rows if r.get('control_updated')]
    stamps=sorted({r['observation']['sensor_stamp_ns']['cone_detections'] for r in rows})
    due=[r['timestamp_s'] for r in controls];sent=[r['timestamp_s'] for r in submissions]
    checks=dict(physics_ticks=all(abs(b['timestamp_s']-a['timestamp_s']-dt)<1e-8 for a,b in zip(rows,rows[1:])),
        control_ticks=all(abs(b-a-1/profile['frequencies_hz']['control'])<1e-8 for a,b in zip(due,due[1:])),
        perception_ticks=all(abs((b-a)/1e9-1/profile['frequencies_hz']['perception'])<1e-8 for a,b in zip(stamps,stamps[1:])),
        planner_ticks=all(abs(b-a-1/profile['frequencies_hz']['planner'])<1e-8 for a,b in zip(sent,sent[1:])),
        deliveries_causal=all(p['delivery']['delivered_s']+1e-8>=p['delivery']['ready_s'] for p in packets),
        held_actions=all(r['raw_action']==rows[i-1]['raw_action'] for i,r in enumerate(rows) if r['phase']=='tracking' and not r['control_updated']))
    return dict(**checks,checks_passed=all(checks.values()),physics_steps=len(rows),control_updates=len(controls),
        planner_submissions=len(submissions),selected_deliveries=len(packets),rates_hz=profile['frequencies_hz'])


def run(workspace,*,selected=(),all_roads=False,profile_path=None,max_steps=None):
    from .runtime import run_runtime
    from .validation_config import configuration
    from .diagnose_perception import diagnose_case
    from .visualize_runtime import compact_case
    workspace=Path(workspace).resolve();profile=configuration(read(profile_path or workspace/'validation.json'))
    if max_steps is not None:profile['max_steps']=max_steps;profile=configuration(profile)
    model=Path(profile['model_path'])
    if not model.is_absolute():model=(workspace/model) if (workspace/model).exists() else ROOT/model
    profile['model_path']=str(model.resolve())
    items,errors=roads(workspace)
    if selected:
        wanted=set(selected);items=[(p,c) for p,c in items if p.stem in wanted or c['fixture_id'] in wanted or c['kind'] in wanted]
    elif not all_roads:items=items[:1]
    if not items:raise ValueError('No matching valid roads; use --road ID, --all or add roads/*.json')
    write(workspace/'last-request.json',dict(selected=list(selected),all_roads=all_roads,profile_path=str(profile_path) if profile_path else None,max_steps=max_steps))
    results=[]
    for path,case in items:
        source=source_snapshot();identity=deepcopy(profile)
        identity['model_path']=hashlib.sha256(model.read_bytes()).hexdigest() if model.is_file() else str(model)
        profile_id=digest(identity)[:10];case_id=profile_id+'_'+case['fixture_id']
        run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_'+case_id
        container=workspace/'runs'/run_id;container.mkdir(parents=True)
        created=datetime.now(timezone.utc).isoformat();write(container/'record.json',dict(status='running',created_utc=created))
        write(container/'inputs.json',dict(road=case,configuration=profile,source=source))
        try:
            summary=run_runtime(case,container/'raw',seed=case['runtime_seed'],detailed_trace=True,validation_config=profile)
            write(container/'raw/road.json',case)
            audit=clock_audit(container/'raw',profile)
            if not audit['checks_passed']:raise AssertionError('Clock audit failed: '+str(audit))
            diagnose_case(container/'raw')
            entry=dict(name='raw',start='road_start',summary=summary,timing_audit=audit)
            payload=compact_case(container,entry);payload['name']=case_id
            labels=dict(observed='感知模块',perfect='完美感知',no_centerline='删除中线',native='native',model='模型',mvp='MVP',cstar='C*',mvp1_straight='单直线')
            profile_label='/'.join(labels[v] for v in profile['modules'].values())+' · '+profile_id[:6]
            payload['title']+=' · '+profile_label;payload['validation_profile']=profile_label
            write(container/'case.json',payload)
            meta={k:payload[k] for k in ('name','title','start','summary','audit','stop_detail','collection','road_input_mode','split','road_seed','validation_profile')}
            # Index carries summaries only; large inputs/trajectories are fetched lazily.
            meta['summary']={k:summary[k] for k in ('completed','fault','final_progress_m','goal_progress_m',
                'planner_reference_count','baseline_reference_count','selection_backend','learned_scorer_used')}
            record=dict(format='validation_run_record_v1',status='complete',created_utc=created,case=meta,
                source=source,source_changed_during_run=source!=source_snapshot(),
                configuration=profile,case_sha256=hashlib.sha256((container/'case.json').read_bytes()).hexdigest(),
                policy_quality_required=False,real_time_qualified=False)
            write(container/'record.json',record);results.append(record)
        except Exception as error:
            write(container/'record.json',dict(status='error',created_utc=created,error=str(error),source=source,configuration=profile))
            raise
        write(workspace/'index.json',index(workspace))
    return results
