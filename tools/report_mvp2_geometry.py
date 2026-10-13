"""Summarize stored outputs only; never reruns planning."""
from collections import Counter
from pathlib import Path
import json, csv, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_mvp2_geometry import SCENES,stats

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/experiments/2026-10-09/mvp2'
OUT.mkdir(parents=True,exist_ok=True)
data=json.loads((ROOT/'.verification/mvp2-evaluation/results/results.json').read_text())
pipeline=json.loads((ROOT/'.verification/mvp2-evaluation/pipeline.json').read_text())

def transition_stats(rows):
    return {k:{'median':float(np.median([r[k] for r in rows])), 'p95':float(np.quantile([r[k] for r in rows],.95)),
               'max':max(r[k] for r in rows)} for k in ('rms_gap_m','max_gap_m','max_heading_gap_deg','target_speed_jump_mps') if all(k in r for r in rows) } if rows else {}

summary={'static':{},'sequences':[],'loops':[],'pipeline':[],'worker':data['worker'],'metadata':data['metadata']}
lines=['# MVP2 道路几何性能评价（2026-10-09）','',
'被测分支 `mvp2.0`，提交 `ae74be454bccec472c677092afb35de899cdc130`。生产源码与 YAML 在实验前后 SHA256 不变；未提交、未推送。旧 feature 工作区保存于带说明的 Git stash。', '',
'## 方法与评价口径','',
'使用实际单文件源码的纯函数与生命周期方法。采用分支 YAML：中心线优先，lattice 回退，最多 16 候选，35 ms 计算预算，直接采样输出；雷达和障碍检查关闭。5 s 对照只放宽计算预算，输入有效期仍生效。','',
'道路包括直线、左右缓弯 y=±0.04x²、左右中弯 y=±0.12x²、急弯 y=0.25x²、S 弯 y=0.12sin(2x)、从 x=0.8m 起增长的曲率 y=0.3max(0,x−0.8)²。主实验左右边界为 y=f(x)±0.5m，属于竖直宽度，弯处法向宽度会减小；附加 0.6/1.4m 宽度、1.0/0.5m 前向覆盖。','',
'单次实验：每路型 5 起始姿态（居中、偏移±15cm、偏角±约30°），20 次重复，主入口和单独 lattice 各做 35ms/5s 配对；调用顺序随机、真实时钟。多帧开环：10Hz、120帧、沿真值道路前进，每帧0.02m，分别施加0/5mm平滑横向几何扰动。直接闭环：20Hz同步规划压力测试、12秒、偏移−10/0/+10cm、35ms/5s；不启用缓存，第一无效或超时输出终止执行。另测实际异步子进程和完整估计→10Hz规划→20Hz控制链路。','',
'独立指标不使用规划打分：①按时有效输出率和延时分位数；②5mm路径网格、≤2cm车身周边采样对真值道路的最小净空；③35ms与5s轨迹共同覆盖区的几何差/长度比；④相邻帧转到固定世界坐标后的重叠区RMS/最大位置差及航向差；⑤固定前1m的空间均匀采样横向RMSE、完成距离、实际车身净空、转向请求跳变和超过0.5°变化的反向次数。未到1m者不计算前1m误差。','',
'连续性差值本身包含随道路观察范围变化而合理改变的规划；不能仅凭差值认定振荡，需结合路型、入口切换和闭环控制。无输出帧中断比较，不跨故障帧计算连续性，故必须同时查看有效率。','',
'## 主要结论','',
'完成4800次静态配对规划、3840次多帧开环规划、11520次直接闭环规划，以及160次真实子进程请求。主实验35ms调用均无超时；完整估计/异步控制链路另有1次35.09ms计算，缓存容错后未停止。局部样本不能作为整链路最坏耗时保证。','',
'成功的35ms静态轨迹与5s轨迹逐点相同、长度比1；全部48组直接闭环跑满12秒，无实际车身越界。居中前1m空间RMSE：缓弯0.51cm、中弯1.63cm、急弯3.52cm、S弯3.05cm、曲率变化1.59cm。曲率变化后程末端误差6.21cm，前段好成绩不代表后段表现。','',
'连续性弱点在入口切换：曲率变化无扰动也有相邻轨迹最大6.52cm位置差，5mm扰动急弯最大5.10cm，均发生在lattice转中心线。这是重叠区最坏位置差，不等同实际车体跳动；同期原始目标速度也可跳约0.10m/s。参考管理器对恢复加速另有0.15m/s²限制。居中曲率变化闭环转向请求最大跳变2.29°，全部偏移闭环最大6.20°。','',
'中弯、急弯、S弯和曲率变化各有一组约30°反向偏角始终无解，5s对照同样无解；0.6m窄路仅20/100主入口成功，0.5m前向覆盖0/100。完整链路22/24组跑满12秒；急弯和曲率变化的−30°/−10cm起步未获得可用道路估计，3秒启动窗口耗尽停车，补充回放道路状态为insufficient_geometry，属于几何估计失败。','',
'静态有效主入口输出无车身越界，最小净空12.97cm（窄路）；直接闭环实际车身最小净空18.26cm。均限于本模型/道路条件。','',
'## 单次规划','',
'表中为主入口（中心线优先），每行100次35ms调用，5种姿态等量。无效输出与超时分别列出。','',
'|道路|中位/P95/最大 ms|按时有效|失败原因|','|---|---:|---:|---|']
for name in dict.fromkeys(r['case'] for r in data['static']):
    summary['static'][name]={}
    for mode in ('primary','secondary'):
        rows=[r for r in data['static'] if r['case']==name and r['mode']==mode]
        hard=[r['pair']['0.035'] for r in rows]; relaxed=[r['pair']['5.0'] for r in rows]
        s=stats(hard);s['relaxed']=stats(relaxed)
        quality=[r['quality'] for r in hard if r['valid'] and r['on_time']]
        s['min_clearance_m']=min((r['min_clearance_m'] for r in quality),default=None)
        s['collision_outputs']=sum(r['min_clearance_m']<0 for r in quality)
        diff=[r['difference'] for r in rows if r['difference'] and r['pair']['0.035']['on_time']]
        s['budget_path_difference']=transition_stats(diff)
        ratios=[r['pair']['0.035']['quality']['length_m']/r['pair']['5.0']['quality']['length_m'] for r in rows if r['difference']]
        s['min_length_ratio']=min(ratios,default=None)
        summary['static'][name][mode]=s
        if mode=='primary':
            failures=Counter(r['reason'] for r in hard if not r['valid'])
            lines.append(f"|{name}|{s['median_ms']:.2f}/{s['p95_ms']:.2f}/{s['max_ms']:.2f}|{s['valid_on_time']}/{s['n']}|{dict(failures)}|")
lines+=['','单独 lattice 入口的统计及主入口/回退路线计数见 summary.json。附加宽度采用独立真值边界，不沿用1m道路的净空。','',
'## 多帧连续性','', '|道路/扰动|35ms按时有效|重叠区最大位置差 P95/最大 cm|最大航向差 P95/最大 °|','|---|---:|---:|---:|']
for run in data['sequences']:
    item={k:v for k,v in run.items() if k not in ('calls','transitions')};item['continuity']=transition_stats(run['transitions']);summary['sequences'].append(item)
    if run['budget']==.035:
        s=run['summary'];c=item['continuity']
        lines.append(f"|{run['scene']}/{run['noise_m']*1000:.0f}mm|{s['valid_on_time']}/{s['n']}|{c.get('max_gap_m',{}).get('p95',0)*100:.2f}/{c.get('max_gap_m',{}).get('max',0)*100:.2f}|{c.get('max_heading_gap_deg',{}).get('p95',0):.2f}/{c.get('max_heading_gap_deg',{}).get('max',0):.2f}|")
lines+=['','## 直接闭环轨迹质量','',
'下表居中起步。偏移±10cm及5s对照全部记录在loops.csv/summary.json。每条件仅一次闭环，不能作为重复成功概率。','',
'|道路|35ms完成步数/240|行进m|前1m空间RMSE cm|末端误差cm|实际最小净空cm|最大转向请求跳变°|终止原因|',
'|---|---:|---:|---:|---:|---:|---:|---|']
for run in data['loops']:
    item={k:v for k,v in run.items() if k not in ('calls','trace','transitions')};item['timing']=stats(run['calls']);item['continuity']=transition_stats(run['transitions']);summary['loops'].append(item)
    if run['budget']==.035 and run['offset']==0:
        m=run['metrics'];rmse=m['spatial_rmse_first_1m_m'];err=m['final_error_m'];clear=m['min_actual_clearance_m'];jump=m['max_command_jump_deg']
        fmt=lambda x: '—' if x is None else f'{x:.2f}'
        lines.append(f"|{run['scene']}|{m['steps']}|{m['progress_m']:.2f}|{fmt(rmse*100 if rmse is not None else None)}|{fmt(err*100 if err is not None else None)}|{fmt(clear*100 if clear is not None else None)}|{fmt(jump)}|{m['failure']}|")
lines+=['','## 默认异步链路与真实子进程','',
'完整链路使用实际估计、异步生命周期、可信参考管理器和控制方法，但将子进程结果交付延迟设为确定的50ms，隔离操作系统调度噪声；1.2m法向宽度道路，与主实验1m竖直宽度不同，不做逐数值比较。每道路测居中/±30°偏角且±10cm偏移起步，各12秒。完整链路的横向误差是法向近似，主实验使用连续真值投影。','',
'|道路|起步角°/偏移cm|执行步数|行驶里程m|末端误差cm|停止原因|','|---|---:|---:|---:|---|']
for run in pipeline['results']:
    item={k:v for k,v in run.items() if k not in ('trace','outputs','transitions')};item['continuity']=transition_stats(run['transitions']);item['steps']=len(run['trace']);summary['pipeline'].append(item)
    err=run['final_error_m'];lines.append(f"|{run['scene']}|{run['angle_deg']}/{run['offset_m']*100:.0f}|{len(run['trace'])}|{run['distance_m']:.2f}|{err*100 if err is not None else 0:.2f}|{run['stop_reason']}|")
worker=data['worker'];w=worker.get('rows',[])
lines+=['',f"真实子进程启动时间 {worker.get('startup_s',0)*1000:.1f}ms（与单次计算分开）。{len(w)}次请求的管道往返中位/P95/最大为 "+('/'.join(f'{x:.2f}' for x in (np.median([r['roundtrip_ms'] for r in w]),np.quantile([r['roundtrip_ms'] for r in w],.95),max(r['roundtrip_ms'] for r in w))) if w else '未获得结果')+'ms。往返包含序列化、调度、计算和1ms轮询，35ms算法预算不代表35ms端到端保证。', '',
'## 验证与适用范围','',
'源码回归156项通过：估计53、规划18、lattice43、控制42。git diff --check通过。按CONTRIBUTING尝试ROS fast gate，因本机bash不存在而无法启动；本次ROS/实车未测。','',
'Windows本地CPU、合成道路、理想轮速/角速度反馈；近似自行车模型和理想目标速度一阶滞后，不评价0.30驱动前馈是否适合真实ESC。计算耗时没有传播到闭环车体状态。主实验直接送道路几何，完整链路另测合成锥桶；没有真实相机噪声、打滑、标定误差。净空离散误差约数毫米，在斜率较大处更大；不将毫米级差值解释为实车安全裕量。完整ROS Jazzy gate及实车测试未运行，本机无bash/ROS环境。','',
'复现：`python -B tools/evaluate_mvp2_geometry.py`，再 `python -B tools/evaluate_mvp2_pipeline.py`，最后 `python -B tools/report_mvp2_geometry.py`。原始详细轨迹在 `.verification/mvp2-evaluation/`，紧凑统计和图在本目录。']
(OUT/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
flat=[{'scene':r['scene'],'offset':r['offset'],'budget':r['budget'],**r['metrics']} for r in data['loops']]
with (OUT/'loops.csv').open('w',newline='',encoding='utf-8-sig') as f:
    writer=csv.DictWriter(f,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
fig,axes=plt.subplots(2,4,figsize=(16,7))
for ax,name in zip(axes.flat,SCENES):
    from evaluate_mvp2_geometry import truth
    xs=np.linspace(0,2.6,400);ys,_=truth(name,xs);ax.fill_between(xs,ys-.5,ys+.5,color='#eeeeee');ax.plot(xs,ys,'k--',label='Road center')
    for run in data['loops']:
        if run['scene']==name and run['budget']==.035:
            t=run['trace'];ax.plot([r['x'] for r in t],[r['y'] for r in t],label=f"offset {run['offset']:+.1f}m")
            if t and run['metrics']['failure']: ax.scatter(t[-1]['x'],t[-1]['y'],marker='x',color='red')
    ax.set_title(name);ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)');ax.set_aspect('equal',adjustable='datalim')
axes.flat[0].legend(fontsize=7);fig.suptitle('MVP2 geometry / 35 ms / approximate plant; x = first failure');fig.tight_layout();fig.savefig(OUT/'trajectories.png',dpi=150);plt.close(fig)
print('Saved',OUT)
