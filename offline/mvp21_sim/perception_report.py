"""Render the recorded cone/road audit; all poses and outputs stay unchanged."""
import json
from pathlib import Path

import numpy as np

from .diagnose_perception import body_points, annotate_branches
from .validate_roads import read_rows
from .visualize_runtime import compact_case, render


def selected_frames(root, suite):
    cases={entry['name']:entry for entry in suite['cases']}
    return [(compact_case(root,cases[name]),index) for name,index in (
        ('half_width_double_left__road_start',145),
        ('half_width_double_left__road_start',146),
        ('half_width_double_left__road_start',176),
        ('half_width_double_left__road_start',250),
        ('half_width_double_right__road_start',146))]


def plot(root, cases, stem='perception-contact-sheet'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    figure,axes=plt.subplots(len(cases),3,figsize=(14,3.5*len(cases)),squeeze=False)
    for row,(case,index) in enumerate(cases):
        f=case['frames'][index];p=f['perception'];body=case['body']
        truth=body_points([c['xy'] for c in case['perception']['cones']],f['pose'])
        for col,ax in enumerate(axes[row]):
            for side in ('left','right'):
                xy=np.asarray(body_points(case['road'][side],f['pose']));ax.plot(xy[:,0],xy[:,1],color='#d2dbe1',lw=.8)
            ax.add_patch(Polygon([[body['front_extent_from_center_of_mass_m'],body['width_m']/2],
                [body['front_extent_from_center_of_mass_m'],-body['width_m']/2],
                [-body['rear_extent_from_center_of_mass_m'],-body['width_m']/2],
                [-body['rear_extent_from_center_of_mass_m'],body['width_m']/2]],facecolor='#deece9',edgecolor='#238c72'))
            ax.set(xlim=(-1.2,4.6),ylim=(-3.5,3.5),xlabel='Current body x (m)',ylabel='Current body y (m)')
            ax.set_aspect('equal');ax.grid(alpha=.12)
        ax=axes[row,0]
        for colour,color in ((2,'#597dc6'),(1,'#cc9b29')):
            xy=np.asarray([q for q,c in zip(truth,case['perception']['cones']) if c['colour']==colour])
            ax.scatter(xy[:,0],xy[:,1],s=20,facecolors='none',edgecolors=color,linewidths=1)
        ax.set_title(f"{case.get('original_name',case['name']).split('__')[0]}\nt={f['t']:.2f}s | all resolved cones",fontsize=9)
        ax=axes[row,1];xy=np.asarray(p['detected_current'])
        if len(xy):ax.scatter(xy[:,0],xy[:,1],color='#b83c9e',s=19,zorder=3)
        for side in ('left','right'):
            xy=np.asarray(p['raw_chains_current'][side])
            if len(xy):
                ax.plot(xy[:,0],xy[:,1],color='#e68142',ls='--',lw=1)
                ax.annotate('',xy=xy[-1],xytext=xy[-2],arrowprops=dict(arrowstyle='->',color='#e68142'))
        ax.set_title(f"Received L/R={p['detected_counts']['left']}/{p['detected_counts']['right']} | 50ms delay aligned\nOrange: fresh-frame chain order (diagnostic)",fontsize=9)
        ax=axes[row,2]
        for key,color in (('left_boundary_xy','#597dc6'),('right_boundary_xy','#cc9b29'),('centerline_xy','#0b9994')):
            xy=np.asarray(f['road'].get(key)or[])
            if len(xy):ax.plot(xy[:,0],xy[:,1],color=color,lw=2)
        xy=np.asarray(f['path'])
        if len(xy):ax.plot(xy[:,0],xy[:,1],color='#7954b6',ls='--',lw=1.2)
        ax.set_title(f"Export: {p['export_status']}\nPurple dashed: MVP execution reference",fontsize=9)
    figure.suptitle('Same recorded ticks: actual cones / received detections / estimator output',fontsize=13)
    figure.tight_layout(rect=(0,0,1,.98));figure.savefig(root/(stem+'.png'),dpi=165)
    figure.savefig(root/(stem+'.pdf'));plt.close(figure)


def write_report(root, diagnosis):
    cases=diagnosis['cases']
    lines=['# 道路感知诊断：锥桶、成链、道路输出','',
        '本次只诊断感知并补充原 20 Hz/1 Hz 回放。没有修改 estimation、planner、execution、control、common YAML 或模型；没有重新采集轨迹。当前 learned policy 未接入，候选选择来自 native planner。','',
        '## 核心证据','',
        '连续掉头左右案例在 **t=7.30 s** 同时发生：左掉头收到蓝 9 / 黄 4 个锥桶，右掉头反之；两组都成链成功，但最终左右边界均被置空。注意：两种颜色同时可见，不代表当前道路两侧同时可见。',
        '锥桶 ID/世界坐标反查显示：连续左掉头 t=7.30 s 的蓝桶位于后面末段直线及其末端延伸（道路里程 12.94–15.74 m），黄桶来自当前第一个掉头（4.72–5.25 m）。当前内侧蓝桶没有在该检测包中。t=7.25 s 系统已使用这组远支路蓝桶构造出带记忆宽度的中线；这是一项支路关联风险，而不能将两组不同支路的点强行配成当前道路。2 m 末端感知延伸中的锥桶也在真实模拟场景内，本次全部显示并标注。',
        '前一帧 t=7.25 s 的记忆宽度约 1.126735 m；t=7.30 s 返回 invalid_boundary_pairing；下一帧起宽度为空、状态为 insufficient_common_range。左掉头累计 67 帧、右掉头 68 帧，存在“两侧都成链成功但不导出任何边界”的情况。',
        '原始左掉头也有 89 帧同类现象，弯曲左右各 32 帧；不只连续掉头道路存在。','',
        '## 问题在何处','',
        '1. **支路关联与横向链方向**。当前不是 y=f(x) 拟合，而是 ordered_xy 按颜色成链；没有独立的道路支路身份约束。链从车辆 +X/-X 方向生长，接近横向、回头支路时，这个方向选择不等同于道路行进方向。实际连续左掉头的采集姿态从 t=7.25 s 的 87.62° 变成 t=7.30 s 的 90.44°；几乎同一组远处蓝桶，其世界排序从 x=2.77→5.22 m 翻为 x=5.22→2.42 m，黄桶链方向未同步翻转。随后配对要求两链切向夹角余弦至少 0.3，并沿参考侧指定内法线寻找另一侧，宽度在 0.15–2.5 m。朝向翻转和不同支路同时存在，会改变法线、配对及单边偏置方向。不能仅靠放松配对阈值解决。',
        '独立最小复现：左侧 x=1 m、右侧 x=2 m，各放五个 y=-0.7…0.7 m 的点，全部可见、无噪声。两侧均成链成功，但两链均沿 -Y 排列；按蓝左/黄右代表的 +Y 行进方向看，法线方向相反，最终返回 insufficient_common_range。只在独立实验中反转两条链的方向，其他点、配置和配对公式均不变，就得到 observed、1 m 宽度和有效中线。复现 JSON 保留输入、链顺序和反事实输出；没有改写实际模块。该实验说明存在朝向歧义，不证明所有实际失败帧都由完全相同的朝向错误触发。',
        '2. **配对失败连带丢弃观测几何**。_ordered_road 在没有双侧有效配对、宽度未知时，只有 other is None 才保留单侧。两组都存在却配不上时，直接返回，未填充边界；记忆宽度存在但 pairing_conflict 时，也直接返回空边界。拒绝把不同支路组合成道路可能是正确的，但应区分“中线无法确定”和“观测几何不存在”。保留下来的观测段仍需标明支路/方向不确定，不能自动作为已确认的当前左/右边界使用。',
        '3. **一次配对冲突又清除了记忆宽度**。invalid_boundary_pairing 不在 QUALITY_FAILURES 中，update 进入末尾 _clear_road_history()，默认 keep_learned_width=False。于是一次冲突之后，后续单侧观测也失去此前测得的宽度。这与记录中 7.25→7.30→7.35 s 的状态变化一致。',
        '4. **晚期确实也会出现检测缺侧**。例如连续左掉头 t=12.50 s，原始包为左 10 / 右 0；这时没有对侧锥桶可供当前帧配对。页面显示完整真值锥桶、相机视域和实际收到的点，避免与早期“看见但未输出”混在一起。相机仅 80°、前向 4 m、最多 24 点；连续掉头各有 41 帧受到容量截断，但本次没有发现因容量截断而将任意一侧从至少两个点降到不足两个点的帧，不能把主要问题归因于 24 点上限。','',
        '## 三层可视化','',
        '- 空心蓝/黄圆：环境中真实存在的全部锥桶。',
        '- 紫色实点：这一控制时刻实际收到的检测包；原始车体坐标和采集时间可展开查看。',
        '- 蓝/黄实线与青色中线：原始记录中实际导出的感知道路。紫色虚线单独表示 native 执行参考，不当作感知输出。',
        '- 可选橙色虚线：用实际成链函数重查当前原始包的链及方向；只用于诊断，不含历史/宽度记忆，不代替原感知输出。',
        '- 图上检测点用记录的采集时刻姿态对齐到世界或当前车体坐标，不能用晚 50 ms 的交付姿态直接变换。','',
        '真值锥桶用相同固定配置、种子和 Dream Gym 版本重建；逐包重新查询传感器，所有记录的检测坐标和颜色与查询完全一致。检测点与同色真值锥桶的最大残差小于 0.000001 m，仅为浮点精度。无噪声意味着检测点通常与真值点重合，紫色实心点覆盖空心圆属于预期。','',
        '|案例|帧数|两侧成链但未导出边界|相机容量截断帧|',
        '|---|---:|---:|---:|']
    for name,r in cases.items():
        lines.append(f"|{name}|{r['frames']}|{r['both_chains_fit_but_no_exported_boundaries_frames']}|{r['sensor_capacity_frames']}|")
    lines+=['','## 最小修复范围建议（本次未实施）','',
        '先修感知支路关联、链方向和部分道路输出契约：同色但不同路线段的锥桶不能直接互换；横向边界朝向需要结合颜色、邻接支路和连续运动保持稳定。配对失败应报告“无法构造双侧道路”，同时保留对齐的观测段及其归属/方向不确定性，中线有效性和观测几何存在性分开。记忆宽度的失效应与具体可信度原因对应，也不能用记忆宽度去掩盖串支路。',
        '这不能单独保证整车成功：native 仍须面对缺侧时只有部分硬约束的事实，并避免把远支路当作当前道路；先用三层图确认感知修复，再分析陡峭候选及控制。','',
        '## 数据完整性','',
        f"原始输入/轨迹/规划结果/配置摘要未改变：{diagnosis['recorded_inputs_unchanged']}；原算法模块/参数/模型未改变：{diagnosis['owning_modules_parameters_model_unchanged']}。全部 16 个案例、4979 个记录帧完成检测查询与真值对照。",'']
    (root/'PERCEPTION_DIAGNOSIS_CN.md').write_text('\n'.join(lines),encoding='utf-8')


def generate(root):
    root=Path(root);suite=json.loads((root/'suite.json').read_text(encoding='utf-8'))
    annotate_branches(root)
    diagnosis=json.loads((root/'perception-diagnosis.json').read_text(encoding='utf-8'))
    cases=selected_frames(root,suite);plot(root,cases);plot(root,cases[:2],stem='perception-transition');write_report(root,diagnosis)
    render(root,viewer_only=True)


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True)
    generate(parser.parse_args().input)
