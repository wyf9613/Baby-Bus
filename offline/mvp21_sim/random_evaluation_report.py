"""Audit random-road runtime evidence and add it to the shared lazy viewer."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil

import numpy as np

from offline.lattice_training.dataset import write_json
from offline.lattice_training.model import SpatialPathPolicy
from .diagnose_perception import input_hashes
from .policy_evaluation import audit_selection
from .policy_selection import DEFAULT_MODEL
from .random_roads import saved_random_roads
from .validate_roads import protected_hashes
from .visualize_runtime import compact_case, write_viewer


def case_payload(viewer, backend, name):
    return json.loads((viewer/'viewer-data'/(backend+'__'+name+'.json')).read_text(encoding='utf-8'))


def plot_roads(root, viewer, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    paired=[r for r in rows if r['backend']=='policy' and r['road_input_mode']=='perception']
    figure,axes=plt.subplots(6,6,figsize=(23,20),squeeze=False)
    for ax,row in zip(axes.flat,paired):
        p=case_payload(viewer,'policy',row['name']);n=case_payload(viewer,'native',row['name'])
        for side in ('left','right'):
            xy=np.asarray(p['road'][side]);ax.plot(xy[:,0],xy[:,1],color='#abb8c3',lw=.8)
        for polygon in p['road']['obstacles']:ax.add_patch(Polygon(polygon,color='#8697a5',alpha=.7))
        for c,color,label in ((n,'#c49127','Native'),(p,'#168870','Frozen policy')):
            xy=np.asarray([f['pose'][:2] for f in c['frames']]+[c['frames'][-1]['after'][:2]])
            ax.plot(xy[:,0],xy[:,1],color=color,lw=1.4,label=label)
            ax.scatter(*xy[-1],color=color,marker='o' if c['summary']['completed'] else 'x',s=18)
            for i in np.linspace(0,len(c['frames'])-1,min(4,len(c['frames']))).astype(int):
                f=c['frames'][i];b=c['body'];x,y,yaw=f['pose'][:3]
                corners=np.asarray([[b['front_extent_from_center_of_mass_m'],b['width_m']/2],
                    [b['front_extent_from_center_of_mass_m'],-b['width_m']/2],
                    [-b['rear_extent_from_center_of_mass_m'],-b['width_m']/2],
                    [-b['rear_extent_from_center_of_mass_m'],b['width_m']/2]])
                rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                ax.add_patch(Polygon(corners@rotation.T+[x,y],fill=False,edgecolor=color,lw=.4,alpha=.6))
        ax.set_title(row['kind']+' / '+str(row['road_seed'])+' / '+row['name'].split('__')[0].rsplit('_',1)[-1]+
            f"\nN {n['summary']['final_progress_m']:.2f} / P {p['summary']['final_progress_m']:.2f} m",fontsize=8)
        ax.set_aspect('equal');ax.grid(alpha=.12);ax.tick_params(labelsize=7)
    for ax in list(axes.flat)[len(paired):]:ax.axis('off')
    axes.flat[0].legend(fontsize=7)
    figure.suptitle('33 saved random-road variants | original perception | 20 Hz MVP control / 1 Hz generation + selection\nCircles: mission goal reached with actual fallback allowed; crosses: stopped',fontsize=14)
    figure.tight_layout();figure.savefig(root/'random-road-overview.png',dpi=160)
    figure.savefig(root/'random-road-overview.pdf');plt.close(figure)


def report(root, rows, metrics, catalog):
    lines=['# 历史随机道路：当前系统频率 evaluation','',
        '本轮目标为保存道路接入、当前频率的系统回放与链路核对。按用户最新约束，policy 运行效果不作为本轮目标或验收条件；其已采结果仅作观察对照，不进行调参、模型选择或重训。','',
        '复用原存档的道路几何和障碍，不从种子重新生成；首批三条道路与扩采道路共 28 个旧 scene ID、33 个实际道路版本。五个障碍改版曾共用 scene ID，本轮按实际 road_spec 指纹分开。63 个扩采序列和首批三个序列均有来源记录，包括未产生训练候选的空序列。','',
        '每个道路版本运行原感知、受控删除中线两种输入，再分别运行 native 和冻结 policy，共 132 次当前时钟闭环。均从入口静止启动；相机/实际 MVP PID 为 20 Hz，候选生成及 policy 排序为 1 Hz，50 ms 相机延迟、100 ms 固定仿真交付。没有 C* 控制、宽度先验、重训或主线接入。','',
        '删除中线条件仅在感知之后删除传给规划/选路的中线，保留边界、状态与障碍。它是受控输入缺失，不能冒充相机自然缺失。可视化仍显示原感知曲线，并明确标注输入条件。实际系统首次控制/提交门控仍保留，因此评分前停止不算 policy 选路失败。','',
        '## 到达终点','',
        '|道路输入|选择器|到达 / 道路版本|首次评分/交付前停止|所选路径 / baseline 控制帧|',
        '|---|---|---|---|---|']
    for mode in ('perception','no_centerline'):
        for backend in ('native','policy'):
            subset=[r for r in rows if r['backend']==backend and r['road_input_mode']==mode]
            lines.append(f"|{mode}|{backend}|{sum(r['summary']['completed'] for r in subset)}/{len(subset)}|"
                f"{sum(r['timing_audit']['deliveries']==0 for r in subset)}|"
                f"{sum(r['summary']['planner_reference_count'] for r in subset)} / {sum(r['summary']['baseline_reference_count'] for r in subset)}|")
    lines+=['','到达终点允许共享 MVP baseline 回退，不表示所选后端全程独立通过。具体碰撞、越界、感知/控制停止和超时均保留在逐案例 CSV 与时间轴。','',
        '## 分道路族','',
        '|道路族|版本数|原感知 native 到达|原感知 policy 到达|',
        '|---|---|---|---|']
    for kind in sorted({r['kind'] for r in rows}):
        subset=[r for r in rows if r['kind']==kind and r['road_input_mode']=='perception']
        n=[r for r in subset if r['backend']=='native'];p=[r for r in subset if r['backend']=='policy']
        lines.append(f"|{kind}|{len(p)}|{sum(r['summary']['completed'] for r in n)}|{sum(r['summary']['completed'] for r in p)}|")
    lines+=['','## 推理、动作与时钟复核','',
        f"交付后评分 {metrics['ranking_deliveries']} 组，其中 {metrics['changed_from_native_same_pool']} 组改变同池 native 原选。"
        f"独立核对 {metrics['verified_candidate_scores']} 个候选输入哈希、冻结分数和 argmax；"
        f"复现 native/policy 两组共 {metrics['verified_shared_pid_frames']} 个实际 PID 活动帧。推理错误 {metrics['policy_errors']} 次。",'',
        '仿真时间频率与 CPU 时限分别报告；固定 100 ms 交付不表示计算在 100 ms 内完成。',
        '|选择器|交付作业数|CPU 中位数 ms|P95 ms|最大 ms|超过 1 s|',
        '|---|---|---|---|---|---|']
    for backend,stats in metrics['cpu'].items():
        lines.append(f"|{backend}|{stats['jobs']}|{stats['median_s']*1000:.1f}|{stats['p95_s']*1000:.1f}|{stats['max_s']*1000:.1f}|{stats['over_1s']}|")
    lines+=['','## 判读边界','',
        '保存原 train/validation/test 道路组，仅用于说明来源；本轮不改变训练划分、不选模型。道路已经参与先前分析，不能把全部结果当成新的独立泛化成绩。',
        '模型输入、权重保持不变，仍逐条评分当前状态、感知道路与障碍、单条空间候选；不输入 planner cost、历史或速度曲线。C* 的固定 0.2 m/s 标签与本轮 MVP PID 执行条件差异仍存在。',
        '感知串支路、边界配对/宽度记忆及执行门控等[已知限制](../KNOWN_LIMITATIONS_CN.md)未在此修复。部分失败发生于首次推理前；已推理案例的失败也不能全部归因于感知。','',
        '## 可视化与重现','',
        '在[同一个回放页面](../viewer.html)选择“历史随机道路”，再选择道路输入、选择器及道路。原来的固定掉头案例仍在页面中。',
        '[33 条道路轨迹与姿态总览](random-road-overview.png) · [逐例结果 CSV](results.csv) · [道路来源目录](catalog.json) · [独立审计](verification.json)','',
        '```powershell',
        '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.evaluate_random_roads --output <new-directory>',
        '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.random_evaluation_report --input <new-directory> --viewer <existing-viewer-directory>',
        '```']
    (root/'REPORT_CN.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def generate(root, viewer):
    root,viewer=Path(root),Path(viewer)
    protected=protected_hashes();old_fixed=input_hashes(viewer)
    suite=json.loads((root/'suite.json').read_text(encoding='utf-8'))
    catalog=json.loads((root/'catalog.json').read_text(encoding='utf-8'))
    if not suite['complete'] or len(suite['cases'])!=catalog['physical_variants']*4:raise ValueError('Incomplete random suite')
    if saved_random_roads()!=catalog or protected!=json.loads((root/'protected-before.json').read_text()):
        raise ValueError('Saved roads or protected model/algorithms changed')
    model=SpatialPathPolicy.load(DEFAULT_MODEL);rows=[]
    by_name={(r['backend'],r['name']):r for r in suite['cases']}
    for entry in suite['cases']:
        directory=root/entry['backend']/entry['name']
        audit=audit_selection(directory,model)
        if not audit['timing_audit']['checks_passed']:raise AssertionError('Clock audit failed')
        if entry['backend']=='policy':
            native=by_name['native',entry['name']]['summary'];policy=entry['summary']
            for key in ('environment','runtime_parameters','runtime_modules','initial_raw_state','camera_latency_s',
                        'control_rate_hz','perception_rate_hz','lattice_generation_rate_hz','road_input','runtime_max_steps'):
                if native[key]!=policy[key]:raise AssertionError('Comparison operating conditions differ: '+key)
        rows.append(dict(**entry,audit=audit))
        print(json.dumps(dict(stage='independent_random_audit',finished=len(rows),total=len(suite['cases']),
            name=entry['name'],backend=entry['backend'],scores=audit['verified_candidate_scores'])),flush=True)
    metrics={k:sum(r['audit'][k] for r in rows) for k in ('ranking_deliveries','changed_from_native_same_pool',
        'verified_candidate_scores','verified_shared_pid_frames')}
    metrics['policy_errors']=sum(len(r['audit']['policy_errors']) for r in rows)
    metrics['cpu']={}
    for backend in ('native','policy'):
        times=[t for r in rows if r['backend']==backend for t in r['audit']['total_cpu_seconds']]
        metrics['cpu'][backend]=dict(jobs=len(times),median_s=float(np.median(times)) if times else 0.,
            p95_s=float(np.quantile(times,.95)) if times else 0.,max_s=max(times,default=0.),over_1s=sum(t>1. for t in times))
    write_json(root/'comparison.json',dict(cases=rows,metrics=metrics))
    fields=('case','kind','seed','old_split','input','backend','completed','progress_m','goal_m','stop_detail','steps',
        'selected_frames','baseline_frames','ranking_calls','deliveries','actual_policy_used')
    with (root/'results.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for r in rows:
            s=r['summary'];writer.writerow(dict(case=r['name'],kind=r['kind'],seed=r['road_seed'],old_split=r['split'],
                input=r['road_input_mode'],backend=r['backend'],completed=s['completed'],progress_m=s['final_progress_m'],
                goal_m=s['goal_progress_m'],stop_detail=r['audit']['stop_detail'],steps=s['steps'],
                selected_frames=s['planner_reference_count'],baseline_frames=s['baseline_reference_count'],
                ranking_calls=s['policy_ranking_calls'],deliveries=r['timing_audit']['deliveries'],actual_policy_used=s['learned_scorer_used']))
    manifest_path=viewer/'viewer-manifest.json'
    if manifest_path.exists():old=json.loads(manifest_path.read_text(encoding='utf-8'))['cases']
    else:
        html=(viewer/'viewer.html').read_text(encoding='utf-8')
        old=json.loads(re.search(r'const DATA=(.*);\nconst ',html).group(1))['cases']
    old=[c for c in old if c.get('collection','fixed')!='random']
    backup=viewer/'viewer-fixed-before-random.html'
    if not backup.exists():shutil.copyfile(viewer/'viewer.html',backup)
    def payloads():
        yield from old
        for entry in rows:
            case=compact_case(root/entry['backend'],entry)
            case['original_name']=case['name'];case['name']=entry['backend']+'__'+case['name']
            yield case
    write_viewer(payloads(),viewer,split_data=True)
    shutil.copyfile(Path(__file__).parents[2]/'docs/SIMULATION_LIMITATIONS_CN.md',viewer/'KNOWN_LIMITATIONS_CN.md')
    plot_roads(root,viewer,rows);report(root,rows,metrics,catalog)
    if old_fixed!=input_hashes(viewer) or protected!=protected_hashes():raise AssertionError('Preserved fixed evidence or algorithms changed')
    write_json(root/'verification.json',dict(all_clock_causality_checks_passed=True,
        archived_road_and_obstacle_specs_preserved=True,protected_algorithms_encoder_model_unchanged=True,
        old_fixed_raw_evidence_unchanged=True,policy_scores_inputs_argmax_verified=True,
        all_recorded_pid_actions_reproduced=True,cases=len(rows),combined_viewer_cases=len(old)+len(rows),
        metrics=metrics,real_time_qualified=False,hardware_tested=False,main_policy_enabled=False))
    print(json.dumps(metrics),flush=True)
    return metrics


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--viewer',type=Path,required=True)
    args=parser.parse_args();generate(args.input,args.viewer)
