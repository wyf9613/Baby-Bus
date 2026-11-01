"""Current-clock old/new/native comparison on the frozen seven noisy Test roads."""
import argparse,csv,hashlib,json,math,traceback
from copy import deepcopy
from pathlib import Path
from collections import Counter
import numpy as np
from offline.lattice_training.dataset import write_json,json_safe,file_sha256
from offline.lattice_training.collection_recovery import assert_equal
from offline.lattice_training.features import encode_candidate,scene_features
from offline.lattice_training.model import SpatialPathPolicy
from mvp21_policy.candidates import from_lattice
from mvp21_policy.estimation import EstimationPipeline,EstimationSettings
from .runtime import run_runtime
from .validation_config import configuration
from .validation_workspace import source_snapshot,clock_audit
from .visualize_runtime import compact_case,write_viewer
from .validate_roads import read_rows
from .diagnose_perception import world_points,body_points
from .source import ROOT,verify_preserved_files

LABELS=dict(native='Native planner',old='旧 policy',new='新 policy')
COLORS=dict(native='#b98329',old='#6c80b5',new='#0e8e80')


def path_quality(directory):
    """Recorded geometry only: angular variation, switches and source overlap."""
    from .ideal_control import world_points as world
    rows=read_rows(directory/'trace.jsonl')
    sources={r['job']['job_id']:r for r in read_rows(directory/'submissions.jsonl')}
    packets={r['result']['job_id']:r for r in read_rows(directory/'shadow.jsonl')}
    turns=[];distances=[];switches=[];overlap=[];previous=None
    for row in rows:
        before,after=np.asarray(row['raw_state_before']),np.asarray(row['raw_state'])
        angle=abs(math.atan2(math.sin(after[2]-before[2]),math.cos(after[2]-before[2])))
        distance=float(np.linalg.norm(after[:2]-before[:2]));turns.append(angle*180/math.pi);distances.append(distance)
        key=(row['execution']['mode'],row['execution'].get('job_id'))
        if previous is not None and key!=previous:switches.append(angle*180/math.pi)
        previous=key
        if row['execution']['mode']!='planner' or row.get('phase')!='tracking':continue
        j=row['execution']['job_id'];packet=packets[j]
        source=world(packet['result']['selected']['path'],sources[j]['decision_pose'])
        reference=world(row['reference']['path']['samples_xy'],row.get('reference_pose') or before)
        a=source[:-1];d=source[1:]-a;den=np.sum(d*d,axis=1)
        for point in reference:
            u=np.clip(np.sum((point-a)*d,axis=1)/np.maximum(den,1e-20),0,1)
            overlap.append(float(np.min(np.linalg.norm(point-a-u[:,None]*d,axis=1))))
    return dict(distance_m=sum(distances),heading_step_p95_deg=float(np.percentile(turns,95)) if turns else None,
        heading_step_max_deg=max(turns,default=0.),turn_per_m_deg=sum(turns)/max(sum(distances),1e-9),
        source_switch_heading_max_deg=max(switches,default=0.),source_switches=len(switches),
        planner_reference_to_original_max_m=max(overlap,default=0.),
        connector_max_m=max(((r.get('control') or {}).get('connector_length_m',0.) for r in rows),default=0.),
        interpretation='Polyline tangent and explicit connector changes retained; these metrics are not a steering-feasibility certificate.')


def sensor_view(directory):
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    config=summary['environment'];physical=config['physical_cones']
    cones=[dict(id=i,xy=[p['x_m'],p['y_m']],colour=2 if p['colour']=='blue' else 1,
        radius_m=p['radius_m'],path=p['side'],progress_m=p['station_m'],route_segment_label='frozen_boundary') for i,p in enumerate(physical)]
    ledger={row['stamp_ns']:row for row in read_rows(directory/'sensor-producer.jsonl')}
    estimator=EstimationPipeline(EstimationSettings(**config['estimation_settings']),'base_link')
    frames=[]
    for index,row in enumerate(read_rows(directory/'trace.jsonl')):
        obs=row['observation'];stamp=obs['sensor_stamp_ns']['cone_detections'];source=ledger[stamp]
        detected=obs['sensors']['cone_detections']['detections']
        assert_equal(detected,source['detected_raw'],'delivered_sensor_packet')
        pose=source['acquisition_pose'];world=world_points(detected,pose)
        raw=estimator.road(obs['sensors']['cone_detections'],stamp,obs['sensor_age_s']['cone_detections'])
        chains={side:[p[:2] for p in (raw.get('_boundary_fits',{}).get(colour) or {}).get('support_points',[])]
            for side,colour in (('left',2),('right',1))}
        road=row.get('raw_estimator_road') or obs['estimation']['road'];current=road.get('current_frame') or {}
        diag=current.get('boundary_diagnostics') or road.get('boundary_diagnostics') or raw.get('boundary_diagnostics') or {}
        both=all((diag.get(side) or {}).get('fit_valid') for side in ('left','right'))
        counts={side:sum(p[3]==colour for p in detected) for side,colour in (('left',2),('right',1))}
        residual=[math.dist(p,cones[i]['xy']) for p,i in zip(world,source['matched_cone_ids'])]
        frames.append(dict(index=index,timestamp_s=row['timestamp_s'],acquisition_timestamp_s=stamp/1e9,
            acquisition_pose=pose,acquisition_pose_basis='exact_sensor_producer_pose',
            detected_raw=detected,detected_world=world,detected_current=body_points(world,row['raw_state_before']),
            matched_cone_ids=source['matched_cone_ids'],query_error_m=0.,truth_match_error_m=max(residual,default=0.),
            detected_count=len(detected),visible_without_cap=source['visible_without_cap'],detected_counts=counts,
            visible_without_cap_counts=source['visible_without_cap_counts'],
            capacity_omitted_sides=[s for s in ('left','right') if source['visible_without_cap_counts'][s]>=2 and counts[s]<2],
            raw_chains_world={s:world_points(p,pose) for s,p in chains.items()},
            raw_chains_current={s:body_points(world_points(p,pose),row['raw_state_before']) for s,p in chains.items()},
            boundary_diagnostics=diag,current_status=current.get('status'),raw_stateless_status=raw['status'],export_status=road['status'],
            both_fit_no_corridor=bool(both and not current.get('fit_valid')),both_fit_no_export=bool(both and not (road.get('left_boundary_xy') or road.get('right_boundary_xy')))))
    write_json(directory/'perception-cones.json',dict(format='noisy_boundary_sensor_audit_v1',cones=cones,
        truth_source='exact frozen physical cone map; packet/source identities captured by producer before delivery',
        measurement_noise=config['measurement_noise'],camera=config['observation_config']['cone_detections'],
        raw_chain_note='Stateless display-only diagnosis; actual estimator output is preserved in trace. Gaussian residuals are intentional.'))
    with (directory/'perception-frames.jsonl').open('w',encoding='utf-8') as stream:
        for frame in frames:stream.write(json.dumps(json_safe(frame),allow_nan=False)+'\n')
    return dict(frames=len(frames),exact_delivered_packets=len(frames),noise_residual_max_m=max((f['truth_match_error_m'] for f in frames),default=0.))


def policy_audit(directory,model_path):
    if model_path is None:return dict(ranking_calls=0,verified_candidates=0)
    model=SpatialPathPolicy.load(model_path);packets=read_rows(directory/'shadow.jsonl');count=0;calls=0;changes=0
    for packet in packets:
        result=packet['result'];info=result.get('policy_selection') or {}
        if info.get('status')=='ERROR':raise ValueError('Policy inference failed: '+str(info))
        if info.get('status')!='SELECTED':continue
        observation=packet['job']['policy_observation'];context=scene_features(observation)
        hidden=info.get('hidden_before');recorded={r['candidate_id']:r for r in info['scores']};scores={}
        for candidate in result['candidates']:
            if candidate['collision'] is not None:continue
            proposal=dict(reference=dict(frenet_path=dict(type='SAMPLED_FRENET',candidate=from_lattice(candidate,result))))
            encoded=encode_candidate(observation,proposal,context=context);score,state=model.step(encoded,hidden)
            digest=hashlib.sha256(json.dumps(encoded,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
            assert_equal(score,recorded[candidate['id']]['score'],'frozen_policy_score')
            if digest!=recorded[candidate['id']]['input_sha256']:raise ValueError('Policy input fingerprint differs')
            scores[candidate['id']]=score;count+=1
            if candidate['id']==info['selected_id']:assert_equal(state.tolist(),info['hidden_selected'],'selected_hidden')
        if max(scores,key=lambda i:(scores[i],-int(i)))!=info['selected_id']:raise ValueError('Policy selected a non-argmax candidate')
        calls+=1;changes+=info['changed_from_native']
    return dict(ranking_calls=calls,verified_candidates=count,changed_from_native_same_pool=changes)


def plots(root,payloads,fixtures):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    figure,axes=plt.subplots(2,4,figsize=(17,9),constrained_layout=True)
    for ax,fixture in zip(axes.flat,fixtures):
        group=[c for c in payloads if c['fixture_id']==fixture]
        scene=group[0]['road']
        for side in ('left','right'):
            p=np.asarray(scene[side]);ax.plot(p[:,0],p[:,1],c='#a5b3bf',lw=1)
        for cone in group[0]['perception']['cones']:
            ax.scatter(*cone['xy'],s=6,c='#708fc4' if cone['colour']==2 else '#ccad57',alpha=.7)
        for case in group:
            frames=case['frames'];p=np.array([f['pose'][:2] for f in frames]+[frames[-1]['after'][:2]])
            color=COLORS[case['selector']];ax.plot(p[:,0],p[:,1],color=color,lw=1.7,label=case['selector'])
            ax.scatter(*p[-1],marker='x',c=color,s=30)
            body=case['body']
            for i in np.linspace(0,len(frames)-1,min(5,len(frames))).astype(int):
                f=frames[i];x,y,yaw=f['pose'][:3];co,si=np.cos(yaw),np.sin(yaw)
                box=np.array([[body['front_extent_from_center_of_mass_m'],body['width_m']/2],
                    [body['front_extent_from_center_of_mass_m'],-body['width_m']/2],
                    [-body['rear_extent_from_center_of_mass_m'],-body['width_m']/2],[-body['rear_extent_from_center_of_mass_m'],body['width_m']/2]])
                ax.add_patch(Polygon(box@np.array([[co,si],[-si,co]])+[x,y],fill=False,edgecolor=color,alpha=.6,lw=.65))
        ax.set_title(fixture.replace('boundary_','')+'\n'+' | '.join(f"{c['selector']} {c['summary']['final_progress_m']:.2f}m" for c in group),fontsize=8)
        ax.set_aspect('equal');ax.grid(alpha=.15);ax.set(xlabel='World x (m)',ylabel='World y (m)')
    axes.flat[-1].axis('off');axes.flat[0].legend(fontsize=8)
    figure.suptitle('Frozen Test roads | paired Gaussian noise | 20 Hz perception/MVP control | 1 Hz planner/policy')
    figure.savefig(root/'test-roads-comparison.png',dpi=150);plt.close(figure)


def run(batch,output,*,selected=(),selectors=('native','old','new'),max_steps=2400,control='mvp'):
    batch=Path(batch).resolve();root=Path(output).resolve()
    if root.exists():raise FileExistsError('Use a new evaluation directory')
    verify_preserved_files();source=source_snapshot();root.mkdir(parents=True)
    files=[]
    for path in sorted((batch/'inputs').glob('*.json')):
        frozen=json.loads(path.read_text(encoding='utf-8'))
        if frozen['split']=='test' and (not selected or frozen['case']['fixture_id'] in selected):files.append((path,frozen))
    if not selected and len(files)!=7:raise ValueError('Expected all seven frozen Test roads')
    if not files:raise ValueError('No selected Test roads')
    models=dict(native=None,old=ROOT/'offline/lattice_training/artifacts/2026-10-19-multistart-policy/model.json',
        new=ROOT/'offline/lattice_training/artifacts/2026-11-01-boundary-policy/model.json')
    protected={str(p):file_sha256(p) for p in [ROOT/'mvp21_policy/models/path_policy.json',ROOT/'config/ai4r_policy.yaml',models['old'],models['new']]}
    manifest=dict(format='boundary_test_runtime_comparison_v1',status='RUNNING',cases=[],roads=[f['case']['fixture_id'] for _,f in files],
        source=source,input_files_sha256={str(p):file_sha256(p) for p,_ in files},protected_file_sha256=protected,
        model_sha256={k:file_sha256(v) if v else None for k,v in models.items()},
        noise='frozen sigma/layout; paired IID radial Gaussian by cone ID/tick; duplicate probability .12; FOV 80deg',
        frequency_hz=dict(perception=20,control=20,planner=1,policy=1),control='explicit ideal tracking' if control=='ideal' else 'actual MVP',cruise_speed_mps=.5,
        no_width_prior=True,radar='dummy',training_performed=False,deployed=False)
    write_json(root/'suite.json',manifest);payloads=[]
    for path,frozen in files:
        original=frozen['case'];fixture=original['fixture_id'];case=deepcopy(original)
        case['split']='test';case['frozen_physical_cones']=frozen['cones'];case['measurement_noise']=dict(sigma_m=frozen['sigma'])
        case['runtime_seed']=int(np.random.SeedSequence([original['road_seed'],1101]).generate_state(1)[0])
        for selector in selectors:
            name=selector+'__'+fixture;directory=root/name
            profile=configuration(dict(modules=dict(perception='observed',planner='native',policy='native' if selector=='native' else 'model',control=control),
                model_path=str(models[selector]) if models[selector] else '',cruise_speed_mps=.5,max_steps=max_steps))
            summary=run_runtime(case,directory,seed=case['runtime_seed'],detailed_trace=True,validation_config=profile)
            write_json(directory/'road.json',case)
            timing=clock_audit(directory,profile)
            if not timing['checks_passed']:raise AssertionError('Module clock audit failed: '+str(timing))
            sensing=sensor_view(directory);ranking=policy_audit(directory,models[selector])
            quality=path_quality(directory);write_json(directory/'path-quality.json',quality)
            if control=='ideal' and quality['planner_reference_to_original_max_m']>1e-8:
                raise ValueError('Ideal execution reference left its delivered source path')
            payload=compact_case(root,dict(name=name,start='road_start',summary=summary,timing_audit=timing))
            payload.update(selector=selector,fixture_id=fixture,title=LABELS[selector]+' · '+original['title'],path_quality=quality)
            data=original['boundary_geometry']
            payload['road']=dict(left=data['left_xy'],right=data['right_xy'],center=data['reference_xy'],progression=data['reference_station_m'],obstacles=[])
            payloads.append(payload)
            row=dict(name=name,selector=selector,fixture_id=fixture,title=original['title'],family=original['kind'],
                completed=summary['completed'],fault=summary['fault'],final_progress_m=summary['final_progress_m'],goal_progress_m=summary['goal_progress_m'],
                planner_frames=summary['planner_reference_count'],baseline_frames=summary['baseline_reference_count'],
                policy_commits=summary['policy_state_commits'],selection_audit=ranking,sensor_audit=sensing,timing_audit=timing,
                path_quality=quality,maximum_executed_speed_mps=summary['maximum_executed_speed_mps'],
                cpu_wall_s=summary['wall_s'])
            manifest['cases'].append(row);write_json(root/'suite.json',manifest)
            if source_snapshot()['sha256']!=source['sha256']:raise ValueError('Runtime source changed during evaluation')
    for name,digest in protected.items():
        if file_sha256(name)!=digest:raise ValueError('Protected model/config changed')
    # Verify shared radial draws for every commonly visible physical cone/tick.
    paired=0
    for _,frozen in files:
        fixture=frozen['case']['fixture_id'];streams={}
        for selector in selectors:
            streams[selector]={(r['tick'],d['source_id']):d['radial_error_m'] for r in read_rows(root/(selector+'__'+fixture)/'sensor-producer.jsonl') for d in r['noise']}
        names=list(streams)
        for i,a in enumerate(names):
            for b in names[i+1:]:
                common=streams[a].keys()&streams[b].keys()
                if any(streams[a][k]!=streams[b][k] for k in common):raise ValueError('Paired measurement noise diverged')
                paired+=len(common)
    write_viewer(payloads,root,split_data=True)
    plots(root,payloads,[f['case']['fixture_id'] for _,f in files])
    template=Path(__file__).with_name('boundary_comparison.html').read_text(encoding='utf-8')
    (root/'comparison.html').write_text(template,encoding='utf-8')
    manifest.update(status='COMPLETE',paired_noise_draws_verified=paired,
        completed_by_selector={s:sum(r['completed'] for r in manifest['cases'] if r['selector']==s) for s in selectors})
    manifest['interpretation_note']=('理想跟踪诊断：精确执行位姿、0.5 m/s 沿折线前进；不使用 PID/执行器，不限制转向或修平折角。感知和 planner 融合保持原逻辑，baseline 回退仍可能发生。' if control=='ideal' else
        '当前 MVP 控制闭环；目标速度与实际速度、回退和运动估计误差应结合原始记录审查。')
    write_json(root/'suite.json',manifest)
    write_json(root/'comparison-index.json',dict(roads=[dict(fixture_id=f['case']['fixture_id'],title=f['case']['title'],sigma_m=f['sigma'],
        cases={s:dict(data_url='viewer-data/'+s+'__'+f['case']['fixture_id']+'.json',name=s+'__'+f['case']['fixture_id']) for s in selectors}) for _,f in files]))
    lines=['# 新 Test 道路：旧 policy / 新 policy / native 真实频率闭环对照','',
        '冻结 7 条 Test 道路，使用原始左右边界和固定桶位作物理真值。感知只收到视场内带噪声的桶位置，保持 80° 视场、无近场裁剪、24 容量及 50 ms 延迟。',
        '径向 IID 高斯标准差逐路沿用冻结配置，12% 重复检测，雷达 dummy，无宽度先验。三个选择器使用相同桶编号/时刻的噪声样本；姿态不同后可见集合及射线方向自然不同。',
        '20 Hz 感知/控制，1 Hz native 生成及模型选路。起点为道路入口静止，八帧静态传感器暖机；之后按每秒重规划持续执行。',
        manifest['interpretation_note'],
        '使用当前主线 48 候选、1.0 m 规划长度和 prefix 检查/评分覆盖，保留融合、shared execution 与 baseline 回退。训练的 8 候选/约1.5 m 分布与此不同。',
        '原始独立边界和真实桶圆只进入仿真及被动碰撞监测，未传给感知、planner 或 policy。所有终止均保留，没有调参、改感知或重训。','',
        '|道路|选择器|进度/终点 m|完成|结束原因|planner / baseline 帧|','|---|---|---|---|---|---|']
    for r in manifest['cases']:lines.append(f"|{r['fixture_id']}|{r['selector']}|{r['final_progress_m']:.2f}/{r['goal_progress_m']:.2f}|{r['completed']}|{r['fault']}|{r['planner_frames']} / {r['baseline_frames']}|")
    lines+=['',f"完成次数：{manifest['completed_by_selector']}。同源噪声复核 {paired} 个共享抽样。",'',
        'comparison.html 支持相同仿真时间下三栏同步观察；viewer.html 保留单案例的完整候选评分、真实桶、检测、原始成链和实际感知曲线。',
        'test-roads-comparison.png 为道路实际轨迹与姿态总览。suite.json 包含原始记录、频率及逐候选/隐状态复核。',
        '100 ms 是显式因果交付模型，CPU 耗时单独保存；这不是 ROS/硬件实时性能资格。baseline 回退参与的结果不能记为 native 或 policy 独立通过。']
    lines+=['','## 路径平滑性（保留折线和切换，不做后处理）','',
        '|道路|选择器|每步航向变化 P95 / 最大 °|总转角/距离 °/m|原候选与执行参考最大距离 m|','|---|---|---|---|---|']
    for r in manifest['cases']:
        q=r['path_quality'];lines.append(f"|{r['fixture_id']}|{r['selector']}|{q['heading_step_p95_deg']:.2f} / {q['heading_step_max_deg']:.2f}|{q['turn_per_m_deg']:.2f}|{q['planner_reference_to_original_max_m']:.3g}|")
    lines+=['','理想端口只将仿真真值用于执行坐标转换，不提供真实道路/锥桶给选路。感知内部时间补偿、planner 历史融合仍保留各自运动估计。',
        '延迟交付/换路存在偏离时，车辆有限速地经过到参考起点的直线连接段，不横移瞬移；连接时车头保持参考切向，不转向横向纠偏方向。该理想端口允许横向平移，遥测按实际位移分解 vx/vy。',
        '姿态直接采用参考折线切向，允许不满足真实车转向能力的折角；通过不代表实车可执行。']
    (root/'REPORT_CN.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(root),cases=len(payloads),completed=manifest['completed_by_selector'],paired_noise_draws=paired),ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--batch',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--road',action='append',default=[]);parser.add_argument('--selectors',default='native,old,new');parser.add_argument('--max-steps',type=int,default=2400)
    parser.add_argument('--control',choices=('mvp','ideal'),default='mvp')
    args=parser.parse_args();run(args.batch,args.output,selected=args.road,selectors=tuple(args.selectors.split(',')),max_steps=args.max_steps,control=args.control)


if __name__=='__main__':main()
