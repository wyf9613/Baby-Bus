"""Generate a boundary-first pool, reuse current sensing/planning, then curate.

No training or dynamic control. Every rejected/generated case remains indexed.
"""
from collections import Counter
from pathlib import Path
import argparse,hashlib,json,math,traceback
import numpy as np
from .boundary_roads import VERSION,WIDTH_PROFILES,generate_boundary_case,boundary_geometry_audit,geometry
from .robust_roads import FAMILIES,vehicle_model,centerline_samples,cone_layout,physical_screen
from .screen_roads import sensor_probe,filter_extra_cones
from .dataset import write_json
from offline.mvp21_sim.source import verify_preserved_files
from offline.mvp21_sim.validation_workspace import source_snapshot


def usable_candidate(result):
    if result.get('status')!='VALID':return False
    for candidate in result.get('candidates') or []:
        path=np.asarray(candidate.get('path') or [],dtype=float)
        if not candidate.get('collision',False) and path.ndim==2 and len(path)>=2 and path.shape[1]>=2 and np.isfinite(path).all():return True
    return False


def system_pass(view):
    return (view.get('completed',True) and not view.get('runtime_exception') and
        view.get('warmup_planner_jobs',0)==0 and
        view['physical']['centerline_passable'] and view['planner_jobs']>0 and
        view.get('valid_candidate_jobs',view['valid_planner_jobs'])==view['planner_jobs'] and
        view['valid_planner_jobs']==view['planner_jobs'])


def record_view_failure(output,view,physical,error):
    frames=[];path=output/'trace.jsonl'
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            try:frames.append(json.loads(line))
            except json.JSONDecodeError:break
    write_json(output/'frames.json',frames)
    moving=[f for f in frames if not f['warmup']];jobs=[f['planner'] for f in moving if f['planner'] is not None]
    summary=dict(view=view,frames=len(moving),exported_boundary_frames=sum(f['exported_boundary'] for f in moving),
        centerline_frames=sum(f['centerline'] for f in moving),planner_jobs=len(jobs),
        valid_planner_jobs=sum(r['status']=='VALID' for r in jobs),valid_candidate_jobs=sum(usable_candidate(r) for r in jobs),
        complete_boundary_coverage=False,complete_planner_coverage=False,physical=physical,
        completed=False,runtime_exception=type(error).__name__+': '+str(error),system_pass=False)
    write_json(output/'error.json',dict(error=summary['runtime_exception'],traceback=traceback.format_exc()))
    write_json(output/'summary.json',summary)
    return summary


def draw_pool_case(root,case,row,samples=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    road=geometry(case);fig,ax=plt.subplots(figsize=(5.5,3.6),constrained_layout=True)
    for side,color in [('left','#3f75b2'),('right','#c6a42b')]:
        points=road.boundaries[side];ax.plot(points[:,0],points[:,1],color=color,lw=1.4)
        controls=np.array(case['boundary_geometry'][side+'_controls_xy']);ax.scatter(controls[:,0],controls[:,1],color=color,s=8,alpha=.6)
    p=road.reference;ax.plot(p[:,0],p[:,1],c='#a4b3c1',ls='--',lw=.8)
    if samples is not None:ax.plot(samples['xy'][:,0],samples['xy'][:,1],c='#388060',lw=1.1)
    ax.set_title(case['kind']+f" / {case['road_seed']} / "+case['generation']['width_profile'],fontsize=9)
    ax.set_aspect('equal');ax.grid(alpha=.12)
    fig.savefig(root/'pool-images'/(case['fixture_id']+'.png'),dpi=130);plt.close(fig)


def write_pool(root,rows):
    write_json(root/'pool.json',dict(generation_version=VERSION,cases=rows,
        generated=len(rows),counts=dict(Counter(r['status'] for r in rows)),
        acceptance=dict(boundary_output_required=False,every_planner_job_has_valid_candidate=True,
            physical_reference_required=True,runtime_exception_allowed=False)))


def curate(entries,limit=24):
    qualified=[e for e in entries if e.get('system_pass')];chosen=[]
    # Cover topology and width timing first; prioritize stronger legal curvature
    # for compound roads, rather than selecting the smoothest/easiest example.
    for family in FAMILIES:
        for profile in WIDTH_PROFILES:
            group=[e for e in qualified if e['case']['kind']==family and e['case']['generation']['width_profile']==profile]
            if group:
                chosen.append(max(group,key=lambda e:e['reference_descriptor']['max_abs_curvature_per_m']))
    if len(chosen)>limit:chosen=chosen[:limit]
    selected={e['case']['fixture_id'] for e in chosen}
    # Preserve the remaining passing cases as an expanded usable pool.
    return dict(qualified=[e['case']['fixture_id'] for e in qualified],recommended=[e['case']['fixture_id'] for e in chosen],
        recommendation_limit=limit,criteria='physical_pass + no exception + every actual planner job has a finite collision-free candidate; boundary availability is observational',
        missing_combinations=[dict(family=f,profile=p) for f in FAMILIES for p in WIDTH_PROFILES
            if not any(e['case']['kind']==f and e['case']['generation']['width_profile']==p for e in qualified)])


GALLERY=r'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>独立边界道路候选池</title>
<style>body{font:15px system-ui;background:#edf2f6;color:#23384b;margin:22px}header,article{background:white;border-radius:12px;padding:16px}header{margin-bottom:16px}h1{margin:0 0 8px}select{margin:10px;padding:5px}#cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:14px}img{width:100%}a{color:#356796}small{line-height:1.7}h3{margin:0 0 6px}.ok{color:#247448}.bad{color:#a1434c}</style>
<header><h1>先生成左右边界，再拟合巡检路线</h1><p>蓝/黄：独立物理边界及控制点；灰虚线：由两侧拟合的中间线；绿实线：车身几何调整后的巡检路线。真实桶仍有固定外移，测距噪声独立逐帧生成。</p>
<p>筛选要求：物理路线可通行、运行无异常、每次规划都有有效候选。边界缺失仅记录；本页不训练 policy，也不表示自主驾驶通过。</p>
<label>拓扑<select id="family"><option value="all">全部</option></select></label><label>宽度组合<select id="profile"><option value="all">全部</option></select></label>
<label>显示<select id="mode"><option value="recommended">推荐道路</option><option value="qualified">全部运行通过</option><option value="all">全部生成与拒绝案例</option></select></label><p id="status">读取索引…</p><a href="viewer.html">进入逐帧查看器</a></header><div id="cards"></div>
<script>let pool;const $=id=>document.getElementById(id);const names={wide_entry:'前宽后窄',wide_exit:'前窄后宽',similar_ends:'两端近似等宽',apex_narrow:'弯中收窄'};async function refresh(){const p=await fetch('pool.json',{cache:'no-store'}).then(r=>r.json());pool=p;for(const [id,field] of [['family','family'],['profile','width_profile']]){const old=$(id).value;const values=[...new Set(p.cases.map(r=>r[field]))];$(id).innerHTML='<option value="all">全部</option>';for(const v of values){let o=document.createElement('option');o.value=v;o.textContent=names[v]||v;$(id).append(o)}$(id).value=old||'all'}draw()}function draw(){const rows=pool.cases.filter(r=>($('family').value==='all'||r.family===$('family').value)&&($('profile').value==='all'||r.width_profile===$('profile').value)&&($('mode').value==='all'||$('mode').value==='qualified'&&r.system_pass||$('mode').value==='recommended'&&r.recommended));$('status').textContent=`已生成 ${pool.generated} 条；状态 ${JSON.stringify(pool.counts)}；当前显示 ${rows.length} 条。`;const cards=$('cards');cards.replaceChildren();for(const r of rows){let a=document.createElement('article');let title=document.createElement('h3');title.textContent=`${r.family} / seed ${r.seed} / ${names[r.width_profile]}`;a.append(title);let image=document.createElement('img');image.src='pool-images/'+r.case_id+'.png';a.append(image);let p=document.createElement('p');p.className=r.system_pass?'ok':'bad';p.textContent=r.recommended?'推荐 · 全部规划有效':r.system_pass?'运行通过':r.status;a.append(p);let detail=document.createElement('small');detail.textContent=`${r.split} · 净宽 ${r.geometry.minimum_clear_width_m?.toFixed(2)}–${r.geometry.maximum_clear_width_m?.toFixed(2)} m · ${r.reasons?.join(', ')||'几何符合'}`;a.append(detail);if(r.runtime_completed){for(const view of ['regular','irregular','noisy']){let link=document.createElement('a');link.href='viewer.html?case='+encodeURIComponent(r.case_id)+'&view='+view;link.textContent=' '+view+'回放 ';a.append(link)}}cards.append(a)}}for(const id of ['family','profile','mode'])$(id).onchange=draw;refresh();setInterval(refresh,10000);</script></html>'''


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seeds',default=','.join(str(s) for s in range(4100,4112)))
    parser.add_argument('--recommend',type=int,default=24);args=parser.parse_args()
    root=args.output
    if root.exists():raise FileExistsError('Choose a new output directory')
    root.mkdir(parents=True);(root/'pool-images').mkdir();verify_preserved_files()
    write_json(root/'vehicle.json',vehicle_model());write_json(root/'source.json',source_snapshot())
    sources={}
    for name in ('boundary_roads.py','robust_roads.py','screen_roads.py','screen_boundaries.py'):
        data=(Path(__file__).parent/name).read_bytes();(root/'runtime-source').mkdir(exist_ok=True)
        (root/'runtime-source'/name).write_bytes(data);sources[name]=hashlib.sha256(data).hexdigest()
    write_json(root/'runtime-source.json',sources)
    write_json(root/'experiment.json',dict(generation_version=VERSION,exclude_adjacent_road_cones=True,paired_noise=True,
        clear_width_range_m=[.5,1.5],planner_start_after_sensor_warmup=True,
        acceptance=dict(boundary_output_required=False,require_every_job_valid=True)))
    (root/'pool.html').write_text(GALLERY,encoding='utf-8')
    seeds=[int(s) for s in args.seeds.split(',')];rows=[];queued=[];entries=[]
    for i,seed in enumerate(seeds):
        split='test' if i>=len(seeds)-2 else 'validation' if i>=len(seeds)-4 else 'train'
        for family in FAMILIES:
            case=generate_boundary_case(family,seed,split);folder=root/case['fixture_id'];folder.mkdir()
            write_json(folder/'road.json',case);audit=boundary_geometry_audit(case)
            row=dict(case_id=case['fixture_id'],family=family,seed=seed,split=split,width_profile=case['generation']['width_profile'],
                geometry=audit,reasons=list(audit['reasons']),status='geometry_rejected',system_pass=False,recommended=False,runtime_completed=False)
            samples=None;layouts={};physicals={}
            if audit['passed']:
                try:
                    from dreamgym.envs import Road
                    Road(road_spec=case['road_spec'])
                    samples=centerline_samples(case)
                    for view in ('regular','irregular'):
                        cones,layout=cone_layout(case,irregular=view!='regular');cones=filter_extra_cones(cones,True)
                        layouts[view]=(cones,layout);physicals[view]=physical_screen(case,cones,samples)
                        row['reasons']+=physicals[view]['reasons']
                    if not row['reasons']:row['status']='queued';queued.append((case,row,samples,layouts,physicals))
                except Exception as error:
                    row['reasons'].append(type(error).__name__+': '+str(error));row['status']='construction_rejected'
                    write_json(folder/'construction-error.json',dict(error=str(error),traceback=traceback.format_exc()))
            row['reasons']=sorted(set(row['reasons']));rows.append(row)
            write_json(folder/'geometry.json',row);draw_pool_case(root,case,row,samples)
            write_pool(root,rows);print(json.dumps(dict(stage='geometry',**row)),flush=True)
    for case,row,samples,layouts,physicals in queued:
        views=[];row['status']='running';write_pool(root,rows)
        for view in ('regular','irregular','noisy'):
            output=root/case['fixture_id']/view;output.mkdir();key='regular' if view=='regular' else 'irregular'
            cones,layout=layouts[key];physical=physicals[key]
            write_json(output/'cones.json',dict(cones=cones,layout=dict(layout,excluded_adjacent_road_cones=True)))
            write_json(output/'physical.json',physical)
            try:
                summary=sensor_probe(case,cones,view,output,paired_noise=True)
                frames=json.loads((output/'frames.json').read_text(encoding='utf-8'))
                jobs=[f['planner'] for f in frames if not f['warmup'] and f['planner'] is not None]
                summary.update(physical=physical,completed=True,runtime_exception=None,valid_candidate_jobs=sum(usable_candidate(r) for r in jobs))
                summary['system_pass']=system_pass(summary);write_json(output/'summary.json',summary)
            except Exception as error:summary=record_view_failure(output,view,physical,error)
            views.append(summary)
            print(json.dumps(dict(stage='runtime',case=case['fixture_id'],view=view,system_pass=summary['system_pass'],
                valid_jobs=summary['valid_planner_jobs'],jobs=summary['planner_jobs'],error=summary.get('runtime_exception'))),flush=True)
        passed=all(system_pass(v) for v in views);row.update(system_pass=passed,status='runtime_pass' if passed else 'runtime_rejected',runtime_completed=True)
        if not passed:row['reasons']+=['planner_or_runtime_failure']
        descriptor=dict(max_abs_curvature_per_m=float(np.max(abs(samples['curvature']))),
            max_steering_deg=float(np.degrees(np.max(abs(samples['steering'])))),
            curvature_variation=float(np.abs(np.diff(samples['curvature'])).sum()))
        entries.append(dict(case=case,views=views,system_pass=passed,boundary_audit=row['geometry'],reference_descriptor=descriptor))
        write_json(root/'index.json',dict(cases=entries));write_pool(root,rows)
    selection=curate(entries,args.recommend);selected=set(selection['recommended'])
    for row in rows:row['recommended']=row['case_id'] in selected
    write_pool(root,rows);write_json(root/'selection.json',selection)
    write_json(root/'run-summary.json',dict(generated=len(rows),geometry_eligible=len(queued),runtime_pass=len(selection['qualified']),
        recommended=len(selected),counts=dict(Counter(r['status'] for r in rows)),boundary_output_required=False))
    verify_preserved_files()


if __name__=='__main__':main()
