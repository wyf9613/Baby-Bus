"""Render recorded runtime ticks and asynchronous planner deliveries, without replaying control."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from offline.lattice_training.visualize import scene_data
from .validate_roads import read_rows


def write_viewer(cases, root, *, split_data=False):
    """Use one viewer; large comparisons can load a single case on demand."""
    root=Path(root)
    manifest=cases
    if split_data:
        data_directory=root/'viewer-data';data_directory.mkdir(exist_ok=True)
        manifest=[]
        for case in cases:
            if 'frames' not in case:
                if not case.get('data_url'):raise ValueError('Unloaded case needs an existing payload URL')
                manifest.append(case)
                continue
            filename=case['name']+'.json'
            (data_directory/filename).write_text(json.dumps(case,ensure_ascii=False,separators=(',',':'),allow_nan=False),encoding='utf-8')
            summary={key:case['summary'].get(key) for key in ('completed','fault','final_progress_m','goal_progress_m',
                'planner_reference_count','baseline_reference_count','selection_backend','learned_scorer_used')}
            manifest.append(dict(name=case['name'],title=case['title'],start=case['start'],summary=summary,
                stop_detail=case['stop_detail'],audit=case['audit'],data_url='viewer-data/'+filename,
                collection=case.get('collection','fixed'),road_input_mode=case.get('road_input_mode','perception'),
                split=case.get('split'),road_seed=case.get('road_seed'),validation_profile=case.get('validation_profile')))
    template=Path(__file__).with_name('runtime_viewer.html').read_text(encoding='utf-8')
    data=json.dumps(dict(cases=manifest,completed=sum(c['summary']['completed'] for c in manifest)),ensure_ascii=False,separators=(',',':'),allow_nan=False)
    (root/'viewer.html').write_text(template.replace('/*__DATA__*/',data.replace('</','<\\/')),encoding='utf-8')
    (root/'viewer-manifest.json').write_text(json.dumps(dict(cases=manifest),ensure_ascii=False,separators=(',',':'),allow_nan=False),encoding='utf-8')


NAMES = {'original':'原始掉头', 'half_width':'半宽＋半径减半', 'curved':'弯曲进出掉头',
         'half_width_double':'半宽＋半径减半连续掉头'}


def compact_case(root, entry):
    directory = root/entry['name']
    summary = entry['summary']
    case = json.loads((directory/'road.json').read_text(encoding='utf-8'))
    trace = read_rows(directory/'trace.jsonl')
    perception_path=directory/'perception-cones.json'
    perception=json.loads(perception_path.read_text(encoding='utf-8')) if perception_path.exists() else None
    perception_frames=read_rows(directory/'perception-frames.jsonl') if perception is not None else []
    if perception_frames and len(perception_frames)!=len(trace):
        raise ValueError('Perception audit and recorded trace lengths differ')
    offered = read_rows(directory/'submissions.jsonl')
    submissions = {r['job']['job_id']:r for r in offered}
    frames = []
    origin = summary['initial_timestamp_s']
    for index,row in enumerate(trace):
        obs = row['observation']
        estimates = obs['estimation']
        road = row.get('perception_road',estimates['road'])
        road = road if any(road.get(k) for k in ('centerline_xy','left_boundary_xy','right_boundary_xy')) else road.get('forward_geometry') or road
        execution = row['execution']
        frames.append(dict(t=row['timestamp_s']-origin, pose=row['raw_state_before'], after=row['raw_state'],
            mode=execution['mode'], reason=execution['reason'], phase=row['phase'],
            path=((row.get('reference') or {}).get('path') or {}).get('samples_xy',[]),
            road={k:road.get(k) for k in ('centerline_xy','left_boundary_xy','right_boundary_xy','boundary_source','lane_width_m','lane_width_source','status','visibility')},
            obstacles=estimates['obstacles'].get('points_xyz_m',[]),
            camera_age=obs['sensor_age_s']['cone_detections'], camera_stamp=obs['sensor_stamp_ns']['cone_detections']/1e9-origin,
            job=execution.get('job_id'), age=execution.get('source_age_s'),
            remaining=execution.get('remaining_length_m'), action=row['raw_action'], plant_action=row['plant_action'],
            progress=row['progress_m'], margin=row['plant_corner_road_margin_m'],
            lateral_error=(row.get('control') or {}).get('control_lateral_error_m'),
            heading_error=(row.get('control') or {}).get('heading_error_rad')))
        frames[-1]['supplied_centerline_present']=bool(estimates['road'].get('centerline_xy') or (estimates['road'].get('forward_geometry')or{}).get('centerline_xy'))
        if perception_frames:
            frames[-1]['perception']=perception_frames[index]
        frames[-1]['control_updated']=row.get('control_updated',True)
        frames[-1]['action_applied']=row.get('action_applied',True)
        if row.get('reference_pose') and row['reference_pose']!=row['raw_state_before'][:3]:
            from .diagnose_perception import world_points,body_points
            frames[-1]['path']=body_points(world_points(frames[-1]['path'],row['reference_pose']),row['raw_state_before'])
    deliveries = []
    for row in read_rows(directory/'shadow.jsonl'):
        result = row['result']
        source = submissions[result['job_id']]
        deliveries.append(dict(id=result['job_id'], submitted=source['timestamp_s']-origin,
            ready=row['delivery']['ready_s']-origin, delivered=row['delivery']['delivered_s']-origin,
            wall=row['delivery']['planner_wall_s'], pose=source['decision_pose'],
            status=result['status'], feasible=result.get('feasible_count',0), selected=(result.get('selected') or {}).get('id'),
            origin=result.get('reference_origin'), sides=result.get('known_boundary_sides'),
            boundary_offset=result.get('reference_boundary_offset_m'),
            policy_selection=result.get('policy_selection'),
            native_wall=row['delivery'].get('native_wall_s',row['delivery']['planner_wall_s']),
            policy_wall=row['delivery'].get('policy_wall_s',0.),
            candidates=[dict(id=c['id'], path=c.get('path',[]), collision=c.get('collision'), cost=c.get('cost'))
                        for c in result.get('candidates',[])],
            fault=result.get('reason')))
    failure = json.loads((directory/'failure.json').read_text(encoding='utf-8')) if (directory/'failure.json').exists() else None
    contacts = [r.get('ground_truth_contacts') or {} for r in trace]
    obstacle_contact = any(r.get('obstacle_indices_overlapping_current_pose') or
                           r.get('obstacle_indices_contacted_during_step') for r in contacts)
    stop_detail = 'execution:obstacle_contact' if obstacle_contact else summary['fault']
    title=case.get('title') or NAMES[case['variant']]+' · '+('左' if case['direction']=='left' else '右')
    return dict(name=entry['name'], title=title,validation_profile=entry.get('validation_profile'),
        start=entry['start'], summary=summary, audit=entry['timing_audit'], road=scene_data(summary['environment']['road_spec']),
        body=summary['environment']['bicycle_model_config']['body_geometry'], frames=frames,
        deliveries=deliveries, submissions=[dict(id=r['job']['job_id'], t=r['timestamp_s']-origin) for r in offered],
        failure=failure, stop_detail=stop_detail, obstacle_contact=obstacle_contact, definition=case,
        module_events=json.loads((directory/'module-events.json').read_text()) if (directory/'module-events.json').exists() else None,
        perception=perception,collection=case.get('collection','fixed'),road_input_mode=summary.get('road_input','perception'),
        split=case.get('split'),road_seed=case.get('road_seed'))


def render_plots(cases, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    colors = dict(planner='#159074',baseline='#d58918',zero='#d44956',priming='#8b91a1')
    for start in ('road_start','bend_start'):
        group = [c for c in cases if c['start']==start]
        if not group:
            continue
        figure, axes = plt.subplots(2,4,figsize=(16,8),squeeze=False)
        for ax,c in zip(axes.flat,group):
            scene=c['road']; frames=c['frames']
            for side in ('left','right'):
                xy=np.asarray(scene[side]); ax.plot(xy[:,0],xy[:,1],color='#596879',lw=1)
            for obstacle in scene['obstacles']:
                ax.add_patch(Polygon(obstacle,color='#9ca5af',alpha=.7))
            for mode,color in colors.items():
                points=np.asarray([f['pose'][:2] if f['mode']==mode else [np.nan,np.nan] for f in frames])
                if len(points):ax.plot(points[:,0],points[:,1],color=color,lw=2,label=mode)
            for index in np.linspace(0,max(0,len(frames)-1),min(10,len(frames))).astype(int):
                f=frames[index]; x,y,yaw=f['pose'][:3]; body=c['body']
                xy=np.asarray([[body['front_extent_from_center_of_mass_m'],body['width_m']/2],
                    [body['front_extent_from_center_of_mass_m'],-body['width_m']/2],
                    [-body['rear_extent_from_center_of_mass_m'],-body['width_m']/2],
                    [-body['rear_extent_from_center_of_mass_m'],body['width_m']/2]])
                rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                ax.add_patch(Polygon(xy@rotation.T+[x,y],fill=False,edgecolor=colors[f['mode']],lw=.7,alpha=.7))
            s=c['summary']; verdict='GOAL' if s['completed'] else 'Obstacle contact' if c['obstacle_contact'] else 'Single-side reference stop'
            ax.set_title(c['name'].split('__')[0]+f"\n{s['final_progress_m']:.2f}/{s['goal_progress_m']:.2f} m | {verdict}",fontsize=9)
            ax.set_aspect('equal');ax.grid(alpha=.15);ax.set(xlabel='World x (m)',ylabel='World y (m)')
        axes.flat[0].legend(fontsize=7,loc='best')
        figure.suptitle('MVP runtime: 20 Hz perception/control, 1 Hz native planner | '+start)
        figure.tight_layout();figure.savefig(output/(start+'-roads.png'),dpi=170)
        figure.savefig(output/(start+'-roads.pdf'));plt.close(figure)
    # One trace with every source switch and planner event, plus all-case speeds.
    c=max(cases,key=lambda c:len(c['frames']))
    figure,axes=plt.subplots(4,1,figsize=(13,10),sharex=True)
    t=np.asarray([f['t'] for f in c['frames']]);speed=np.asarray([np.hypot(f['pose'][3],f['pose'][4]) for f in c['frames']])
    axes[0].plot(t,speed,label='Actual speed');axes[0].axhline(.2,color='black',ls='--',label='MVP target 0.2 m/s');axes[0].set_ylabel('m/s');axes[0].legend()
    axes[1].plot(t,[f['action'][0] for f in c['frames']],label='Drive');axes[1].plot(t,[f['action'][1] for f in c['frames']],label='MVP steering');axes[1].legend();axes[1].set_ylabel('Normalized')
    axes[2].plot(t,[f['pose'][2] for f in c['frames']],label='Yaw (rad)');axes[2].plot(t,[f['pose'][6] for f in c['frames']],label='Physical steering (rad)');axes[2].legend()
    for f in c['frames']:axes[3].axvspan(f['t'],f['t']+.05,color=colors[f['mode']],alpha=.8,lw=0)
    for d in c['deliveries']:
        axes[3].axvline(d['submitted'],color='black',lw=.5);axes[3].axvline(d['delivered'],color='#3d66d6',lw=.5,ls='--')
    axes[3].set(yticks=[],xlabel='Simulation time (s)',ylabel='Source')
    figure.suptitle(c['name']+' | black: submit / blue: deliver | green: native / amber: baseline / red: zero')
    figure.tight_layout();figure.savefig(output/'runtime-timeline.png',dpi=170);figure.savefig(output/'runtime-timeline.pdf');plt.close(figure)


def write_report(cases, suite, output):
    columns=('name','completed','final_progress_m','goal_progress_m','fault','steps','planner_reference_count',
        'baseline_reference_count','fallback_after_first_native_cycles','maximum_executed_speed_mps')
    with (output/'results.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        for c in cases:writer.writerow(dict(name=c['name'],**{k:c['summary'][k] for k in columns if k!='name'}))
    lines=['# MVP 系统按时钟闭环验证','',
        '20 Hz 锥桶感知、状态估计与 control；1 Hz native planner。使用同一份 common YAML 和 mvp21_policy 的 estimation、lattice、execution、control 模块。',
        'C* 不产生任何控制量；没有接入 learned policy，没有设置场景宽度先验。原有 C* 训练式验证报告保留。','',
        '相机采集到交付延迟 50 ms；规划结果固定延迟 100 ms 可见。等待结果时继续每 50 ms 控制，结果交付后按共享 execution 模块换接。CPU 规划耗时另记；这是仿真时间调度，不能用来认定 ROS 或实车实时性能。','',
        '每条固定道路从入口静止启动；另做八个弯前静止启动的局部诊断。保持道路和障碍不变。2 m 末端感知延伸单独记录，任务终点仍按原道路长度。道路/障碍真值只用于绘图和物理监测，不传给 planner。',
        '初始静止暖机 150 ms，随后开始计时。故障后不自动恢复，额外施加最多 1 s 零 drive/steer；物理碰撞终止的环境无法继续积分，因此对应滑行片段可能少于 1 s。','',
        '|启动|道路|到达终点|进度 / 目标 m|native / baseline 控制帧|停止原因|',
        '|---|---|---|---|---|---|']
    for c in cases:
        s=c['summary'];lines.append(f"|{c['start']}|{c['title']}|{'是' if s['completed'] else '否'}|{s['final_progress_m']:.2f} / {s['goal_progress_m']:.2f}|{s['planner_reference_count']} / {s['baseline_reference_count']}|{c['stop_detail'] or '无'}|")
    lines+=['','## 判读','',f"总计 {suite['completed']}/{len(cases)} 次到达任务终点。入口整路测试 {sum(c['summary']['completed'] for c in cases if c['start']=='road_start')}/8；弯前诊断 {sum(c['summary']['completed'] for c in cases if c['start']=='bend_start')}/8。",
        '到达终点允许实际 MVP baseline 回退；不得把该结果当成 native 全程独立通过。回放时间轴逐帧显示当前来源，规划候选只在结果交付后显示。',
        '当前目标速度仍为 0.2 m/s，真实执行速度见图表和每次 summary；没有把 C* 的速度跟踪或制动行为带入这次运行。',
        f"全部时钟/因果审计通过：{suite['timing_checks_passed']}；common 参数、真实 MVP 模块、policy 输入入口、模型和 C* 文件保护检查通过：{suite['common_controller_and_planner_parameters_unchanged']}。",'',
        '## 暴露的问题及范围','',
        '1. **启动/执行交接**：八个弯前静止案例均在第一个控制周期因 `invalid_road_single_side_width_unknown` 停止，提交数为 0。感知保留单侧，baseline 无有效参考，而共享 node 的 `_shadow_offer()` 位于成功动作发布后；因此这组结果没有检验到 native 单边候选的可用性。与真实 node 的顺序一致，没有为展示绕开入口。',
        '2. **中途参考接续**：两条连续掉头运行到约 5.79/5.80 m 后，滚动 execution 检查报告 `known_boundary_collision`，baseline 又处于单侧宽度未知，因而停止。末次 planner 结果本身仍为 VALID；应区分候选生成成功和随后仍可执行。每帧原因和末次结果保存在 JSONL 中。',
        '3. **障碍响应**：半宽左转与弯曲左右转共三次在末段接触障碍 1。原始 summary 用 `environment_finished` 记录环境终止；本报告及页面根据 ground_truth_contacts 明确标为 `execution:obstacle_contact`。当前 transport 的 native job 只送道路/状态，lattice 检查已知道路边界；baseline 的 diagnostics 也声明 obstacle_response_enabled=False。当前这条直出链路没有形成避障控制闭环。没有将真值障碍塞回规划器。',
        '4. **控制与车辆响应匹配**：入口八次均出现约 0.776 m/s 的启动峰值，目标为 0.2 m/s，随后逐渐下降。common YAML 的 MVP PID 和 drive 前馈原样使用，说明原实车控制参数在当前虚拟 plant 上的启动响应需要单独评估；不能把旧 C* 的跟踪效果当作该 control 的效果。',
        '5. **规划服务时延**：此报告为确定性的 100 ms 结果交付模型，CPU 实测耗时另外保留在 shadow.jsonl/回放中；它不代表逐次 CPU 工作均能在 100 ms 内完成。该机制保证模拟的 20 Hz 控制不因同步计算而推进或跳过仿真时钟，但不代替 ROS 独立 worker 的实时调度验证。','',
        '## 重现','', '```powershell',
        '& ../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.validate_roads --output <new-directory>',
        '& ../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.visualize_runtime --input <new-directory>', '```','']
    (output/'REPORT_CN.md').write_text('\n'.join(lines),encoding='utf-8')


def render(root, *, viewer_only=False):
    root=Path(root)
    suite=json.loads((root/'suite.json').read_text(encoding='utf-8'))
    cases=[compact_case(root,entry) for entry in suite['cases']]
    if not viewer_only:
        write_report(cases,suite,root);render_plots(cases,root)
    write_viewer(cases,root)
    return cases


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--viewer-only',action='store_true',help='Preserve existing report/plots; refresh the viewer only')
    args=parser.parse_args();render(args.input,viewer_only=args.viewer_only)


if __name__=='__main__':main()
