"""Compare frozen-policy runtime evaluation with the preserved same-clock native run."""
import argparse
from collections import Counter
from copy import deepcopy
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np

from mvp21_policy.candidates import from_lattice
from mvp21_policy.control import PolicyController,ControlSettings
from offline.lattice_training.features import encode_candidate,scene_features
from offline.lattice_training.model import SpatialPathPolicy
from offline.lattice_training.dataset import write_json
from .diagnose_perception import annotate_branches,input_hashes
from .policy_selection import scoring_observation
from .source import ROOT
from .validate_roads import read_rows,protected_hashes,timing_audit
from .visualize_runtime import compact_case,write_viewer


def audit_selection(directory, model):
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    rows=read_rows(directory/'trace.jsonl');by_step={r['step']:r for r in rows}
    packets=read_rows(directory/'shadow.jsonl')
    selected=changed=verified=0;policy_times=[];total_times=[];gaps=[];errors=[]
    for packet in packets:
        result=packet['result'];info=result.get('policy_selection')or{}
        total_times.append(packet['delivery']['planner_wall_s'])
        policy_times.append(packet['delivery'].get('policy_wall_s',0.))
        if info.get('status')=='ERROR':errors.append(info['reason'])
        if info.get('status')!='SELECTED':continue
        observation=packet['job']['policy_observation']
        submit_step=next(r['step'] for r in read_rows(directory/'submissions.jsonl')
            if r['job']['job_id']==result['job_id'])
        if observation!=scoring_observation(by_step[submit_step]['observation']):
            raise AssertionError('Policy did not use its submission-frame estimator snapshot')
        if not math.isclose(observation['timestamp_s'],result['input_stamp_s'],abs_tol=1e-8):
            raise AssertionError('Candidate/input decision frames differ')
        context=scene_features(observation);recorded={r['candidate_id']:r for r in info['scores']}
        feasible=[c for c in result['candidates'] if c['collision'] is None]
        if set(recorded)!={c['id'] for c in feasible}:raise AssertionError('Scored pool differs from native admission')
        scores={}
        for candidate in feasible:
            proposal=dict(reference=dict(frenet_path=dict(type='SAMPLED_FRENET',candidate=from_lattice(candidate,result))))
            encoded=encode_candidate(observation,proposal,context=context)
            digest=hashlib.sha256(json.dumps(encoded,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
            score=model.score(encoded);expected=recorded[candidate['id']]
            if digest!=expected['input_sha256'] or not math.isclose(score,expected['score'],abs_tol=1e-12,rel_tol=0.):
                raise AssertionError('Frozen input/score replay failed')
            scores[candidate['id']]=score;verified+=1
        expected=max(feasible,key=lambda c:(scores[c['id']],-int(c['id'])))
        if result['selected']!=expected or info['selected_id']!=expected['id']:
            raise AssertionError('Executed selection is not the frozen policy argmax')
        selected+=1;changed+=info['changed_from_native']
        if info['score_gap'] is not None:gaps.append(info['score_gap'])
    controller=PolicyController(ControlSettings(**summary['runtime_parameters']['control']))
    pid_frames=0
    for row in rows:
        if row['phase']!='tracking' or row['execution']['mode']=='priming':continue
        drive,steer,control=controller.calculate(row['reference'],row['state'],
            row['observation']['estimation']['vehicle_params'],row['timestamp_s'],0. if row['step']==0 else .05)
        if not control['valid'] or max(abs(drive-row['raw_action'][0]),abs(steer-row['raw_action'][1]))>1e-12:
            raise AssertionError('Recorded action differs from actual stateful MVP PID')
        pid_frames+=1
    contacts=any((r.get('ground_truth_contacts')or{}).get('obstacle_indices_contacted_during_step') or
        (r.get('ground_truth_contacts')or{}).get('obstacle_indices_overlapping_current_pose') for r in rows)
    return dict(ranking_deliveries=selected,changed_from_native_same_pool=changed,verified_candidate_scores=verified,
        verified_shared_pid_frames=pid_frames,policy_errors=errors,policy_cpu_seconds=policy_times,
        total_cpu_seconds=total_times,score_gaps=gaps,obstacle_contact=bool(contacts),
        stop_detail='execution:obstacle_contact' if contacts else summary['fault'],
        timing_audit=timing_audit(directory))


def plots(root, pairs, policy_cases):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    figure,axes=plt.subplots(2,4,figsize=(17,8),squeeze=False)
    for ax,(native,policy) in zip(axes.flat,pairs):
        for side in ('left','right'):
            p=np.asarray(policy['road'][side]);ax.plot(p[:,0],p[:,1],color='#a1b0be',lw=1)
        for obstacle in policy['road']['obstacles']:ax.add_patch(Polygon(obstacle,color='#8796a3',alpha=.7))
        for case,color,label in ((native,'#cc952e','Native'),(policy,'#209276','Frozen policy')):
            points=np.asarray([r['pose'][:2] for r in case['frames']]+[case['frames'][-1]['after'][:2]])
            ax.plot(points[:,0],points[:,1],color=color,lw=1.8,label=label)
            ax.scatter(*points[-1],marker='x',color=color,s=28)
            for i in np.linspace(0,len(case['frames'])-1,min(6,len(case['frames']))).astype(int):
                f=case['frames'][i];yaw=f['pose'][2];p=f['pose'][:2]
                ax.arrow(p[0],p[1],.2*np.cos(yaw),.2*np.sin(yaw),color=color,alpha=.7,head_width=.055,length_includes_head=True)
        ax.set_title(policy['original_name'].split('__')[0]+f"\nNative {native['summary']['final_progress_m']:.2f} m / policy {policy['summary']['final_progress_m']:.2f} m",fontsize=9)
        ax.set_aspect('equal');ax.grid(alpha=.12);ax.set(xlabel='World x (m)',ylabel='World y (m)')
    axes.flat[0].legend(fontsize=8)
    figure.suptitle('Same fixed roads and clocks | 20 Hz perception/MVP control | 1 Hz native generation + selection')
    figure.tight_layout();figure.savefig(root/'policy-native-roads.png',dpi=165);figure.savefig(root/'policy-native-roads.pdf');plt.close(figure)
    figure,axes=plt.subplots(2,2,figsize=(12,10),squeeze=False)
    names=('original_left__road_start','original_right__road_start','curved_left__road_start','half_width_double_left__road_start')
    for ax,name in zip(axes.flat,names):
        case=next(c for c in policy_cases if c['original_name']==name)
        d=next((d for d in case['deliveries'] if (d.get('policy_selection')or{}).get('changed_from_native')),case['deliveries'][0])
        info=d['policy_selection'];scores={r['candidate_id']:r['score'] for r in info['scores']}
        for candidate in d['candidates']:
            p=np.asarray(candidate['path']);colour='#c98c20' if candidate['id']==info['native_selected_id'] else '#168870' if candidate['id']==info['selected_id'] else '#9caec04d'
            ax.plot(p[:,0],p[:,1],color=colour,lw=2 if candidate['id'] in (info['native_selected_id'],info['selected_id']) else .6)
        selected=scores[info['selected_id']];native_score=scores.get(info['native_selected_id'])
        ax.set_title(name.split('__')[0]+f" | t={d['submitted']:.2f}s\npolicy #{info['selected_id']} score={selected:.5f}; native #{info['native_selected_id']} score={native_score:.5f}",fontsize=9)
        ax.set_aspect('equal');ax.grid(alpha=.15);ax.set(xlabel='Decision body x (m)',ylabel='Decision body y (m)')
    figure.suptitle('One candidate per model call | green: policy pick | amber: native pick in the SAME pool')
    figure.tight_layout();figure.savefig(root/'same-pool-selections.png',dpi=165);figure.savefig(root/'same-pool-selections.pdf');plt.close(figure)
    case=max(policy_cases,key=lambda c:len(c['frames']))
    figure,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    t=[f['t'] for f in case['frames']]
    axes[0].plot(t,[np.hypot(f['pose'][3],f['pose'][4]) for f in case['frames']],color='#168870');axes[0].axhline(.2,color='gray',ls='--');axes[0].set_ylabel('Speed (m/s)')
    axes[1].plot(t,[f['action'][0] for f in case['frames']],label='Drive');axes[1].plot(t,[f['action'][1] for f in case['frames']],label='MVP steering');axes[1].legend();axes[1].set_ylabel('Action')
    axes[2].plot(t,[f['progress'] for f in case['frames']]);axes[2].set(xlabel='Simulation time (s)',ylabel='Route progress (m)')
    for ax in axes:ax.grid(alpha=.15)
    figure.suptitle(case['original_name']+' | real MVP PID, frozen-policy spatial path selection')
    figure.tight_layout();figure.savefig(root/'runtime-timeline.png',dpi=165);plt.close(figure)


def report(root, comparison, policy_suite, native_suite):
    rows=comparison['cases'];policy_done=sum(r['policy']['completed'] for r in rows if r['start']=='road_start')
    native_done=sum(r['native']['completed'] for r in rows if r['start']=='road_start')
    lines=['# 冻结 policy 在当前系统频率下的 evaluation','',
        f"入口整路：policy **{policy_done}/8**，native **{native_done}/8**。八个弯前静止启动均在首次规划/评分前停止，不能当作模型选择了失败候选。",'',
        '使用当前冻结空间 policy，仅替换候选排序。native 生成和碰撞接纳、共享 execution 滚动检查、MVP PID、baseline 回退、故障零指令均保持。20 Hz 感知/控制，1 Hz 候选生成与 policy 排序，不等待整条候选跑完。',
        '输入保持当前状态、当前道路与障碍、一条原始 Frenet/Cartesian 候选。没有 planner cost、历史特征、空间展平、速度曲线输入；没有使用真值或场景宽度。每个候选单独调用模型。模型文件、输入编码、权重和训练数据未改变。',
        'C* 不产生控制量、不重新拟合或生成测试标签。当前模型学习的是固定 0.2 m/s 条件的 C* return，输出 sigmoid 分数不是成功概率；实际执行改用 MVP PID，属于明确的执行器/频率/候选分布迁移。此次 evaluation 并不消除这一迁移。','',
        '## 公平对照与已知限制','',
        'native 对照复用之前保存的相同 16 次 20/1 Hz 运行；已逐项检查道路、初始状态、车辆、传感器、时钟、规划/控制参数及实际算法模块一致。没有将训练时 C* 整段执行结果当作本次 baseline。',
        '这些道路已用于可视化和问题分析，本轮是固定道路回归 evaluation，不作为新的独立 holdout 泛化成绩。',
        '沿用 100 ms 因果交付模型；车辆每 50 ms 继续更新。native 与 policy 的真实 CPU 时间分别记录，不能据固定交付时间认定真实 ROS/硬件性能。',
        '[已知感知/执行限制](KNOWN_LIMITATIONS_CN.md)保持未修复：同色串支路、横向成链方向翻转、配对冲突丢边界并清除宽度、单边启动门控。完整锥桶、收到的检测、实际感知曲线继续显示。模型只能重选已有候选，不能补出这些信息。','',
        '|启动|道路|native 进度/终点 m|policy 进度/终点 m|policy 停止原因|policy / baseline 控制帧|',
        '|---|---|---|---|---|---|']
    for row in rows:
        p,n=row['policy'],row['native']
        lines.append(f"|{row['start']}|{row['name']}|{n['final_progress_m']:.2f}/{n['goal_progress_m']:.2f}|{p['final_progress_m']:.2f}/{p['goal_progress_m']:.2f}|{row['audit']['stop_detail'] or '无'}|{p['policy_reference_count']} / {p['baseline_reference_count']}|")
    metrics=comparison['metrics']
    lines+=['','## 选路与执行证据','',
        f"交付后完成评分的候选组 {metrics['ranking_deliveries']} 组；其中 {metrics['changed_from_native_same_pool']} 组与同一候选池的 native 原选不同。独立重放核对了 {metrics['verified_candidate_scores']} 个候选的输入 SHA-256、冻结模型分数与 argmax；{metrics['verified_shared_pid_frames']} 个活动控制帧可由同一有状态 MVP PID 原样复现。policy 推理错误 {metrics['policy_error_count']} 次。",
        '完整原始快照保存在 submissions.jsonl / shadow.jsonl；policy 读取提交时刻的感知道路与障碍，结果交付前不可见。被 native 碰撞检查拒绝的候选不参与评分。模型分数表与 native cost 仅作并列展示，native cost 不进入模型。',
        '当前冻结 policy 在这些未修感知道路和现有 MVP 控制条件下没有达到终点，表现弱于 native。已有结果不能将全部失败归因于感知；同一候选池的不同选择说明模型确实改变了路径，而 C* 标签与实际 PID 的差异、训练覆盖和感知失效的各自贡献尚未被隔离。此次没有通过改阈值、加宽度或事后改选掩盖结果。',
        f"native+policy CPU：中位数 {metrics['total_cpu_median_s']*1000:.1f} ms，P95 {metrics['total_cpu_p95_s']*1000:.1f} ms，最大 {metrics['total_cpu_max_s']*1000:.1f} ms；超过 1 s 的交付作业 {metrics['total_cpu_over_1s']} 个。",'',
        '## 查看与重现','',
        'viewer.html 可在 policy 与 native、入口与弯前启动之间切换。仅按需加载一个案例；分数和候选仅在结果交付后出现。紫色为 policy 当前执行参考，橙色候选为同一池的 native 原选。',
        '```powershell',
        '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.validate_roads --policy-model offline/lattice_training/artifacts/2026-10-17-expanded-collection/model.json --output <new-directory>',
        '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.diagnose_perception --input <new-directory>',
        '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.policy_evaluation --input <new-directory> --native-baseline offline/lattice_training/artifacts/2026-10-18-mvp-runtime-validation',
        '```','',
        '此接入仅用于离线仿真可视化；ROS 主线入口和默认 native 选路保持不变。']
    (root/'REPORT_CN.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def generate(root, baseline):
    root,baseline=Path(root),Path(baseline)
    protected=protected_hashes();baseline_before=input_hashes(baseline)
    policy_suite=json.loads((root/'suite.json').read_text(encoding='utf-8'))
    native_suite=json.loads((baseline/'suite.json').read_text(encoding='utf-8'))
    if not policy_suite['learned_policy_used'] or native_suite['learned_policy_used']:
        raise ValueError('Expected policy run and native baseline')
    if json.loads((root/'protected-before.json').read_text())!=json.loads((baseline/'protected-before.json').read_text()):
        raise ValueError('Actual modules/parameters/encoder/model differ from preserved baseline')
    annotate_branches(root)
    model=SpatialPathPolicy.load(next(c['summary']['policy']['model_path'] for c in policy_suite['cases']))
    native_entries={c['name']:c for c in native_suite['cases']};results=[];all_cases=[];policy_cases=[];native_cases=[]
    for entry in policy_suite['cases']:
        native=native_entries[entry['name']]
        p,n=entry['summary'],native['summary']
        for key in ('runtime_parameters','runtime_modules','initial_raw_state','camera_latency_s','control_rate_hz','perception_rate_hz','lattice_generation_rate_hz'):
            if p[key]!=n[key]:raise ValueError('Baseline operating condition differs: '+key)
        for key in ('road_spec','initial_state_config','observation_config','bicycle_model_config','integration_config','estimation_settings'):
            if p['environment'][key]!=n['environment'][key]:raise ValueError('Baseline environment differs: '+key)
        audit=audit_selection(root/entry['name'],model)
        if not audit['timing_audit']['checks_passed']:raise AssertionError('Clock audit failed')
        results.append(dict(name=entry['name'],kind=entry['kind'],start=entry['start'],policy=p,native=n,audit=audit))
        for source,item,backend,target in ((root,entry,'policy',policy_cases),(baseline,native,'native',native_cases)):
            case=compact_case(source,item);case['original_name']=case['name'];case['name']=backend+'__'+case['name']
            case['summary']['selection_backend']=backend
            target.append(case);all_cases.append(case)
    policy_times=[t for r in results for t in r['audit']['policy_cpu_seconds']]
    total_times=[t for r in results for t in r['audit']['total_cpu_seconds']]
    metrics={key:sum(r['audit'][key] for r in results) for key in ('ranking_deliveries','changed_from_native_same_pool','verified_candidate_scores','verified_shared_pid_frames')}
    metrics.update(policy_error_count=sum(len(r['audit']['policy_errors']) for r in results),
        total_cpu_median_s=float(np.median(total_times)),total_cpu_p95_s=float(np.quantile(total_times,.95)),
        total_cpu_max_s=max(total_times),total_cpu_over_1s=sum(t>1. for t in total_times),
        policy_cpu_p95_s=float(np.quantile(policy_times,.95)))
    comparison=dict(format='frozen_policy_current_frequency_evaluation_v1',cases=results,metrics=metrics,
        native_baseline_reused=True,native_baseline_directory=str(baseline.resolve()),
        baseline_raw_sha256=baseline_before,policy_provenance=policy_suite['cases'][0]['summary']['policy'],
        known_limits='KNOWN_LIMITATIONS_CN.md')
    shutil.copyfile(ROOT/'docs/SIMULATION_LIMITATIONS_CN.md',root/'KNOWN_LIMITATIONS_CN.md')
    write_json(root/'comparison.json',comparison)
    report(root,comparison,policy_suite,native_suite)
    with (root/'results.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        columns=['case','backend','completed','final_progress_m','goal_progress_m','fault','chosen_reference_frames','baseline_frames']
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        for row in results:
            for backend in ('policy','native'):
                s=row[backend];writer.writerow(dict(case=row['name'],backend=backend,completed=s['completed'],
                    final_progress_m=s['final_progress_m'],goal_progress_m=s['goal_progress_m'],
                    fault=row['audit']['stop_detail'] if backend=='policy' else s['fault'],
                    chosen_reference_frames=s['planner_reference_count'],baseline_frames=s['baseline_reference_count']))
    pairs=[(next(c for c in native_cases if c['original_name']==p['original_name']),p) for p in policy_cases if p['start']=='road_start']
    plots(root,pairs,policy_cases)
    # Keep the three-layer keyframe visualization in the comparison package.
    from .perception_report import plot as perception_plot
    selected=[(c,min(len(c['frames'])-1,next((i for i,f in enumerate(c['frames']) if f['perception']['both_fit_no_export']),len(c['frames'])//2)))
        for c in policy_cases if c['original_name'] in ('original_left__road_start','curved_left__road_start','half_width_double_left__road_start')]
    perception_plot(root,selected)
    (root/'PERCEPTION_DIAGNOSIS_CN.md').write_text('# 本轮感知状态\n\n已知限制未修复，见 [限制记录](KNOWN_LIMITATIONS_CN.md)。\n\n'
        '本轮每个案例均附有完整锥桶真值、按采集时刻对齐的收到检测与实际感知输出。'
        'perception-diagnosis.json 保存逐帧成链/输出及检测包复核计数；本轮未更改感知模块。\n',encoding='utf-8')
    shutil.copyfile(root/'policy-native-roads.png',root/'road_start-roads.png')
    write_viewer(all_cases,root,split_data=True)
    if baseline_before!=input_hashes(baseline) or protected!=protected_hashes():raise AssertionError('Preserved evidence/source changed')
    write_json(root/'evaluation-verification.json',dict(
        policy_cases=len(policy_cases),native_baseline_cases=len(native_cases),all_score_hashes_and_argmax_verified=True,
        all_recorded_pid_actions_reproduced=True,clock_causality_audits_passed=True,
        protected_algorithms_parameters_model_unchanged=True,native_baseline_raw_files_unchanged=True,
        native_baseline_raw_sha256=baseline_before,policy_model_sha256=comparison['policy_provenance']['model_sha256'],
        metrics=metrics,policy_inputs='current_state/current_road_and_obstacles/one_untimed_candidate',
        ros_policy_enabled=False,hardware_tested=False))
    print(json.dumps(metrics),flush=True)
    return comparison


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--native-baseline',type=Path,required=True)
    args=parser.parse_args();generate(args.input,args.native_baseline)
