"""Build immutable centerline-screen review plots and a standalone local viewer."""
from collections import Counter
from pathlib import Path
import argparse,json,math
import numpy as np
from dreamgym.envs import Road
from .dataset import write_json
from .pipeline import with_terminal_support


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def boundary_residual(root,case_id,view,left,right):
    """Passive nearest true-boundary residual, never an input or admission gate."""
    distances=[];max_bearing=0.;bad_clock=0;frame_count=0;previous_time=None
    path=root/case_id/view/'trace.jsonl'
    for line in path.read_text(encoding='utf-8').splitlines():
        frame=json.loads(line)
        if previous_time is not None and not math.isclose(frame['time_s']-previous_time,.05,abs_tol=1e-8):bad_clock+=1
        previous_time=frame['time_s']
        for x,y,*_ in frame['detected_cones']:max_bearing=max(max_bearing,abs(math.atan2(y,x)))
        if frame['warmup']:continue
        road=frame['road'];geom=road if any(road.get(k) for k in ('left_boundary_xy','right_boundary_xy','centerline_xy')) else road.get('forward_geometry') or {}
        x,y,angle=frame['pose'];c,s=math.cos(angle),math.sin(angle)
        found=False
        for role,target in [('left_boundary_xy',left),('right_boundary_xy',right)]:
            values=np.array(geom.get(role) or [])
            if not len(values):continue
            found=True;points=values@np.array([[c,s],[-s,c]])+[x,y]
            a=target[:-1];v=target[1:]-a;norm=np.sum(v*v,axis=1)
            displacement=points[:,None,:]-a[None,:,:]
            fraction=np.clip(np.sum(displacement*v[None,:,:],axis=2)/norm,0,1)
            error=np.linalg.norm(displacement-fraction[:,:,None]*v[None,:,:],axis=2).min(axis=1)
            distances.extend(error.tolist())
        frame_count+=found
    assert bad_clock==0,'Non-20Hz prescribed sensing trace'
    assert max_bearing<=math.radians(40)+1e-6,'Synthetic detection escaped camera FOV'
    return dict(boundary_points_compared=len(distances),boundary_median_error_m=float(np.median(distances)) if distances else None,
        boundary_error_p95_m=float(np.quantile(distances,.95)) if distances else None,
        observed_boundary_frames=frame_count,camera_max_abs_bearing_deg=math.degrees(max_bearing),
        sensor_clock_20hz_verified=True,association_correctness_guaranteed=False)


VIEWER=r'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>道路生成与中心线筛查</title>
<style>body{margin:0;background:#eef2f6;color:#243042;font:15px system-ui}header{padding:18px 24px;background:#172838;color:white}h2{margin:0 0 8px}main{padding:16px;display:grid;grid-template-columns:310px 1fr;gap:16px}aside,section{background:white;border-radius:12px;padding:16px}select,input{width:100%;margin:7px 0 14px}canvas{width:100%;height:650px;background:#fafcfe}small{color:#627386}pre{white-space:pre-wrap;font:13px system-ui;line-height:1.65}.legend{display:flex;gap:16px;flex-wrap:wrap;margin:8px 0}button{padding:6px 12px}</style>
<header><h2>短片段道路 · 中心线巡检</h2>规定车辆沿中心线移动；当前感知 20 Hz，native planner 1 Hz。没有 policy、PID 或 C* 执行。</header>
<main><aside><label>数据划分</label><select id="split"><option value="all">全部</option><option>train</option><option>validation</option><option>test</option></select>
<label>固定道路</label><select id="case"></select><label>观测条件</label><select id="view"><option value="regular">规则锥桶 · 无额外测距误差</option><option value="irregular">不均匀/缺桶/其他道路干扰</option><option value="noisy">同一不规则道路 + 射线漂移/重复检测</option></select>
<label><input id="onlyfail" type="checkbox" style="width:auto;margin:4px">只列失败道路</label>
<label>失败位置（可直接跳转）</label><select id="failure"><option value="">无失败标注</option></select>
<button id="firstfail">首个失败</button><button id="prevfail">上一处</button><button id="nextfail">下一处</button>
<label>帧</label><input id="frame" type="range" min="0" value="8"><button id="play">播放 / 暂停</button><pre id="fail-status"></pre><pre id="details"></pre>
<small>宽度指锥桶内沿之间的净宽 0.3–1.5 m；桶中心连线间距另加两个桶半径，共 0.08 m。末端另有 2 m 感知支撑，任务终点不变。同种子镜像道路同划分。</small></aside>
<section><div class="legend"><span>● 蓝/黄：真实锥桶</span><span style="color:#d76a22">○ 橙：相机收到的检测</span><span style="color:#c4324b">— 红：感知边界</span><span style="color:#2a8b5b">— 绿：感知中线</span><span style="color:#733ea5">— 紫：native 选择</span></div>
<div><label><input id="showfail" type="checkbox" checked style="width:auto;margin:4px">显示失败区间</label> <b style="color:#c72f43">红粗线：车身几何</b> · <b style="color:#df8d21">橙粗线：感知中断</b> · <b style="color:#7647a7">紫叉：失败规划作业</b></div>
<canvas id="map"></canvas><div id="stamp"></div><small>橙色视场为检测采集时的相机视场（80°），保留 50 ms 延迟。检测按采集姿态显示，拟合曲线按当前姿态显示。没有近场截断。紫叉仅标真实 1 Hz 失败作业，不推断两次作业之间又运行了规划。几何标注是规定姿态下的被动检查。原始记录未重跑或改写。</small></section></main>
<script>
const $=id=>document.getElementById(id);let index,maps,entry,frames,cones,failureData,failureIndex,playing=false,loading=false,loadGeneration=0,scale,ox,oy;
const query=new URLSearchParams(location.search);let pendingFrame=query.has('frame')?parseInt(query.get('frame')):null;
const kindNames={body_road:'车身越界',body_cone:'车身碰桶',steering_angle:'转向角超限',steering_rate:'转向速率超限',perception:'感知无边界',planner:'planner 无有效候选'};
const ctx=$('map').getContext('2d');function pos(x,y){return[ox+x*scale,oy-y*scale]}
function line(points,color,width=1,dash=[]){if(!points?.length)return;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();points.forEach((p,i)=>{const[x,y]=pos(...p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.setLineDash([])}
function world(points,pose){const[x,y,a]=pose,c=Math.cos(a),s=Math.sin(a);return points.map(p=>[x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]])}
function circle(x,y,color,r=3,fill=true){const p=pos(x,y);ctx.beginPath();ctx.arc(...p,r,0,2*Math.PI);ctx.strokeStyle=color;ctx.fillStyle=color;fill?ctx.fill():ctx.stroke()}
function draw(){if(!frames||loading||!frames.length)return;const i=+$('frame').value,f=frames[i],map=maps[entry.case.fixture_id],vw=entry.views.find(v=>v.view===$('view').value);const canvas=$('map');canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=650*devicePixelRatio;ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
const xs=map.left.concat(map.right).map(p=>p[0]),ys=map.left.concat(map.right).map(p=>p[1]);let xmin=Math.min(...xs)-.8,xmax=Math.max(...xs)+.8,ymin=Math.min(...ys)-.8,ymax=Math.max(...ys)+.8;scale=Math.min((canvas.clientWidth-40)/(xmax-xmin),610/(ymax-ymin));ox=20-xmin*scale;oy=630+ymin*scale;
ctx.clearRect(0,0,canvas.clientWidth,650);line(map.left,'#bdc8d4');line(map.right,'#bdc8d4');line(map.center,'#c6d4dd',1,[4,4]);if(map.reference)line(map.reference,'#57718e',1.4);
if($('showfail').checked&&failureData){for(const e of failureData.events){const color=e.kind==='planner'?'#7647a7':e.kind==='perception'?'#df8d21':'#c72f43';const points=e.sample_frames.map(n=>frames[n].pose.slice(0,2));if(e.kind!=='planner')line(points,color,4);for(const p of e.kind==='planner'?points:points.length===1?points:[]){const[x,y]=pos(...p);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x-4,y-4);ctx.lineTo(x+4,y+4);ctx.moveTo(x+4,y-4);ctx.lineTo(x-4,y+4);ctx.stroke()}}}
cones.forEach(c=>circle(c.x_m,c.y_m,c.colour==='blue'?'#3f71b5':'#cba52a',Math.max(2,c.radius_m*scale),true));
const capture=frames[Math.max(0,i-1)].pose;const rays=[[0,0],[4,-4*Math.tan(40*Math.PI/180)],[4,4*Math.tan(40*Math.PI/180)],[0,0]];line(world(rays,capture),'#e0ac6b',1,[4,4]);
world(f.detected_cones,capture).forEach(p=>circle(...p,'#dc7629',4,false));const road=f.road,geom=(road.left_boundary_xy?.length||road.right_boundary_xy?.length||road.centerline_xy?.length)?road:(road.forward_geometry||{});
for(const name of ['left_boundary_xy','right_boundary_xy'])line(world(geom[name]||[],f.pose),'#c4324b',2.3);line(world(geom.centerline_xy||[],f.pose),'#248b5c',2);
if(f.planner){for(const c of f.planner.candidates||[])line(world(c.path,f.pose),c.collision?'#ddd':'#79b99d',.6);if(f.planner.selected)line(world(f.planner.selected.path,f.pose),'#783ba3',2.3)}
const v=index.vehicle;line(world([[-v.rear_m,-v.width_m/2],[v.front_m,-v.width_m/2],[v.front_m,v.width_m/2],[-v.rear_m,v.width_m/2],[-v.rear_m,-v.width_m/2]],f.pose),'#233f66',2);
line(world([[0,0],[.38,0]],f.pose),'#233f66',2);circle(...map.goal,'#b02d59',5,false);
const check=failureData?.per_frame[i];const format=v=>v===undefined?'—':v.toFixed(3)+' m';$('fail-status').textContent=check?`${check.kinds.length?check.kinds.map(k=>kindNames[k]).join(' / '):'本采样帧没有标注失败'}\n车身道路余量 ${format(check.road_clearance_m)}\n车身锥桶余量 ${format(check.cone_clearance_m)}`:failureData?'静止预热，不分类':'该记录尚未生成失败位置标注';
$('stamp').textContent=`帧 ${i}/${frames.length-1} · 道路站位 ${f.station_m.toFixed(2)} m · t=${f.time_s.toFixed(2)}s · ${f.warmup?'静止预热':'规定中心线运动'} · 检测 ${f.detected_cones.length} 个 · road=${f.road.status} · planner=${f.planner?.status||'本帧不规划'}`;
$('details').textContent=`${entry.case.title}\n${entry.case.split} / ${entry.case.split_group}\n长度 ${entry.case.mission_road_length_m.toFixed(2)} m\n宽度控制点 ${entry.case.generation.width_controls_m.map(v=>v.toFixed(2)).join(' → ')} m\n\n中心线几何通过：${vw.physical.centerline_passable?'是':'否'}\n${vw.physical.reasons.join(', ')}\n最大转角 ${vw.physical.max_required_steering_deg.toFixed(1)}°\n车身道路余量 ${vw.physical.minimum_body_road_clearance_m.toFixed(3)} m\n车身锥桶余量 ${vw.physical.minimum_body_cone_clearance_m.toFixed(3)} m\n\n导出边界：${vw.exported_boundary_frames}/${vw.frames} 帧\n导出中线：${vw.centerline_frames}/${vw.frames} 帧\nplanner VALID：${vw.valid_planner_jobs}/${vw.planner_jobs}\n\n此页没有动态驾驶或模型选路验收。`}
function jumpFailure(e){if(!e)return;playing=false;$('frame').value=e.start_frame;$('failure').value=e.id;draw()}
function fillFailures(){const events=failureData?.events||[];$('failure').innerHTML='';if(!events.length){const o=document.createElement('option');o.value='';o.textContent='该观测条件没有失败标注';$('failure').append(o)}for(const e of events){const o=document.createElement('option');o.value=e.id;o.textContent=`${e.label} · s=${e.start_station_m.toFixed(2)}m · 帧${e.start_frame}–${e.end_frame} · ${e.reason}`;$('failure').append(o)}for(const id of ['failure','firstfail','prevfail','nextfail'])$(id).disabled=!events.length}
async function load(){const token=++loadGeneration;loading=true;playing=false;$('details').textContent='正在加载该道路和观测条件…';const chosen=index.cases.find(r=>r.case.fixture_id===$('case').value);if(!chosen){loading=false;return}const dir=chosen.case.fixture_id+'/'+$('view').value;const data=await Promise.all([fetch(dir+'/frames.json').then(r=>r.json()),fetch(dir+'/cones.json').then(r=>r.json()).then(r=>r.cones),fetch(dir+'/failures.json',{cache:'no-store'}).then(r=>r.ok?r.json():null).catch(()=>null)]);if(token!==loadGeneration)return;entry=chosen;[frames,cones,failureData]=data;loading=false;if(!frames.length){$('details').textContent='运行前异常：'+(entry.views.find(v=>v.view===$('view').value).runtime_exception||'没有实际轨迹');return}$('frame').max=frames.length-1;$('frame').value=pendingFrame===null?8:Math.min(frames.length-1,Math.max(0,pendingFrame));pendingFrame=null;fillFailures();draw()}
function choices(preferred){const selected=typeof preferred==='string'?preferred:$('case').value;const failures=new Set((failureIndex?.cases||[]).map(r=>r.case_id));$('case').innerHTML='';index.cases.filter(r=>($('split').value==='all'||r.case.split===$('split').value)&&(!$('onlyfail').checked||failures.has(r.case.fixture_id))).forEach(r=>{let o=document.createElement('option');o.value=r.case.fixture_id;o.textContent=r.case.split+' · '+r.case.title;$('case').append(o)});if([...$('case').options].some(o=>o.value===selected))$('case').value=selected;load()}
$('split').onchange=choices;$('case').onchange=load;$('view').onchange=load;$('frame').oninput=draw;$('play').onclick=()=>playing=!playing;window.onresize=draw;
$('onlyfail').onchange=choices;$('showfail').onchange=draw;$('failure').onchange=()=>jumpFailure(failureData.events.find(e=>e.id===+$('failure').value));$('firstfail').onclick=()=>jumpFailure(failureData?.events[0]);$('nextfail').onclick=()=>jumpFailure(failureData?.events.find(e=>e.start_frame>+$('frame').value));$('prevfail').onclick=()=>jumpFailure([...(failureData?.events||[])].reverse().find(e=>e.start_frame<+$('frame').value));
setInterval(()=>{if(playing&&frames){$('frame').value=(+$('frame').value+1)%frames.length;draw()}},100);
Promise.all([fetch('index.json').then(r=>r.json()),fetch('review-maps.json').then(r=>r.json()),fetch('failure-index.json',{cache:'no-store'}).then(r=>r.ok?r.json():null).catch(()=>null)]).then(([a,b,c])=>{index=a;maps=b;failureIndex=c;$('onlyfail').checked=!!c&&query.get('failures')==='1';$('onlyfail').disabled=!c;if(['regular','irregular','noisy'].includes(query.get('view')))$('view').value=query.get('view');choices(query.get('case')||'')});
</script></html>'''


def build_review(root):
    from .robust_roads import vehicle_model,road_geometry
    index=read(root/'index.json');index['vehicle']=vehicle_model();write_json(root/'index.json',index)
    experiment=read(root/'experiment.json') if (root/'experiment.json').exists() else {}
    boundary_required=experiment.get('acceptance',{}).get('boundary_output_required',True)
    maps={};counts={};rows=[]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for entry in index['cases']:
        case=entry['case'];road=road_geometry(case,case['terminal_sensor_support_m'])
        stations=np.arange(0,case['mission_road_length_m']+2,.04);xy,head=road.convert_progression_to_coordinates(stations)
        if hasattr(road,'boundary_coordinates'):
            left,_=road.boundary_coordinates('left',stations);right,_=road.boundary_coordinates('right',stations)
        else:
            bounds=[road.get_reference_lane_bounds_at_progress(float(s)) for s in stations]
            normal=np.column_stack((-np.sin(head),np.cos(head)))
            left=xy+normal*np.array([b['left_bound_m'] for b in bounds])[:,None]
            right=xy+normal*np.array([b['right_bound_m'] for b in bounds])[:,None]
        goal,_=road.convert_progression_to_coordinates(np.array([case['mission_road_length_m']]))
        reference=[f['pose'][:2] for f in read(root/case['fixture_id']/'regular/frames.json') if not f['warmup']]
        maps[case['fixture_id']]=dict(left=left.tolist(),right=right.tolist(),center=xy.tolist(),goal=goal[0].tolist(),reference=reference)
        for view in entry['views']:
            residual=boundary_residual(root,case['fixture_id'],view['view'],left,right)
            view.update(residual)
            write_json(root/case['fixture_id']/view['view']/'review-diagnostics.json',residual)
            key=(case['split'],view['view']);count=counts.setdefault(key,Counter())
            count['roads']+=1;count['physical_pass']+=view['physical']['centerline_passable']
            count['full_boundary']+=view['complete_boundary_coverage'];count['full_planner']+=view['complete_planner_coverage']
            count['all_three']+=view['physical']['centerline_passable'] and view['complete_boundary_coverage'] and view['complete_planner_coverage']
            count['system_pass']+=bool(view.get('system_pass',view['physical']['centerline_passable'] and view['complete_planner_coverage']))
            count['frames']+=view['frames'];count['boundary_frames']+=view['exported_boundary_frames']
            count['planner_jobs']+=view['planner_jobs'];count['valid_jobs']+=view['valid_planner_jobs']
            rows.append(dict(case=case['fixture_id'],split=case['split'],view=view['view'],**view['physical'],
                boundary_fraction=view['exported_boundary_frames']/max(1,view['frames']),planner_fraction=view['valid_planner_jobs']/max(1,view['planner_jobs'])))
    write_json(root/'index.json',index)
    write_json(root/'review-maps.json',maps);render_viewer(root)
    if (root/'pool.json').exists():render_pool(root)
    for split in ('train','validation','test'):
        cases=[entry for entry in index['cases'] if entry['case']['split']==split]
        if not cases:continue
        fig,axes=plt.subplots(math.ceil(len(cases)/3),3,figsize=(14,4*math.ceil(len(cases)/3)),squeeze=False,constrained_layout=True)
        for ax,entry in zip(axes.flat,cases):
            case=entry['case'];m=maps[case['fixture_id']]
            for side in ('left','right'):
                p=np.array(m[side]);ax.plot(p[:,0],p[:,1],color='#8a9ba8',lw=.8)
            p=np.array(m['center']);ax.plot(p[:,0],p[:,1],color='#566d84',ls='--',lw=.7)
            p=np.array(m['reference'])
            if len(p):ax.plot(p[:,0],p[:,1],color='#57718e',lw=1.)
            cones=read(root/case['fixture_id']/'irregular/cones.json')['cones']
            for color,ink in [('blue','#3d6bb0'),('yellow','#ccab32')]:
                p=np.array([[c['x_m'],c['y_m']] for c in cones if c['colour']==color]);ax.scatter(p[:,0],p[:,1],c=ink,s=8)
            regular,irregular,noisy=entry['views'];flags='geometry '+('OK' if irregular['physical']['centerline_passable'] else 'REJECT')
            ratios=' / '.join(str(v['valid_planner_jobs'])+'/'+str(v['planner_jobs']) for v in entry['views'])
            ax.set_title(case['title']+'\n'+flags+' | native jobs '+ratios,fontsize=9);ax.set_aspect('equal');ax.grid(alpha=.15)
        for ax in axes.flat[len(cases):]:ax.axis('off')
        fig.suptitle(split.upper()+' | actual irregular cones | job counts: regular / irregular / noisy | 2m tail support')
        fig.savefig(root/(split+'-roads.png'),dpi=150);plt.close(fig)
    fields=('roads','physical_pass','full_boundary','full_planner','all_three','system_pass','frames','boundary_frames','planner_jobs','valid_jobs')
    summary={split:{view:{key:counts.get((split,view),Counter())[key] for key in fields} for view in ('regular','irregular','noisy')} for split in ('train','validation','test')}
    write_json(root/'screening-summary.json',dict(splits=summary,cases=rows))
    lines=['# 短道路生成与中心线筛查','',
        '24 条候选道路，6 类 S/U/复合弯，Train/Validation/Test = 12/6/6。每条道路做规则桶、不规则桶、叠加径向误差三次巡检，共 72 次。',
        '同种子的镜像和所有观测版本保持同一划分；失败道路和观测均保留，没有重新抽样至成功。','',
        '宽度 0.3–1.5 m 指锥桶内沿之间的可通行净宽；桶半径 0.04 m，左右桶中心连线间距为 0.38–1.58 m。',
        '车辆参数来自当前模型：轴距 0.33 m、车宽 0.25 m、前轮 ±45°、转向速率 1.571 rad/s。稳态质心最小半径约 0.355 m。',
        '生成的弯道包含曲率渐变片段，并以反解质心运动学检查需要的车身航向、转角和转向速率；车身/锥桶接触单独检查。','',
        '规定车辆以 0.5 m/s 沿中心线移动，不调用 PID/C*/policy。完整位置状态仅用于虚拟传感器和被动几何评估；感知与 planner 只收到原传感器输入。',
        '20 Hz 感知、50 ms 相机延迟、1 Hz 当前 native planner；保留 80° 视场、24 个检测上限和 4 m 前向上界，无近场截断。雷达为 dummy。',
        '不规则桶包含不均匀桶距、随机缺桶、连续缺桶段及另一道路的局部干扰桶。noisy 复用同一物理桶，叠加径向相关漂移和重复检测；幅度未按实车标定。',
        '末端另有显式 2 m 传感器支撑，任务终点和几何筛查区间不变。','',
        '| 划分 | 观测 | 道路数 | 中心线几何通过 | 全帧有边界输出 | 全部 planner 作业 VALID | 三者同时完整 |',
        '|---|---|---:|---:|---:|---:|---:|']
    for split,views in summary.items():
        for view,c in views.items():lines.append(f"| {split} | {view} | {c['roads']} | {c['physical_pass']} | {c['full_boundary']} | {c['full_planner']} | {c['all_three'] if boundary_required else c['system_pass']} |")
    lines+=['','全程道路通过数与逐帧/逐作业可用率分开记录：','',
        '| 观测 | 有边界帧 / 移动帧 | VALID 作业 / 规划作业 |','|---|---:|---:|']
    for view in ('regular','irregular','noisy'):
        totals={key:sum(views[view][key] for views in summary.values()) for key in ('frames','boundary_frames','planner_jobs','valid_jobs')}
        lines.append(f"| {view} | {totals['boundary_frames']}/{totals['frames']} ({totals['boundary_frames']/max(1,totals['frames']):.2%}) | {totals['valid_jobs']}/{totals['planner_jobs']} ({totals['valid_jobs']/max(1,totals['planner_jobs']):.2%}) |")
    lines+=['','“中心线几何未通过”只说明这条指定中心线走法碰桶/越界或超转向限位，不证明不存在其他合法走法。',
        '“导出边界”和“planner VALID”仅表示输出可用，不能保证道路支路关联正确，也不能代替候选实际驾驶。',
        '完整率采用严格全帧/全作业计数，不人为放宽感知门槛。所有原始状态原因和中线输出率另存于每个观测目录。',
        '道路是否像现实、干扰桶是否合理，需结合下面总览和 viewer.html 逐帧审查；本批没有进入训练。','',
        '![Train](train-roads.png)','','![Validation](validation-roads.png)','','![Test](test-roads.png)']
    experiment=read(root/'experiment.json') if (root/'experiment.json').exists() else {}
    if experiment.get('exclude_adjacent_road_cones'):
        lines.insert(2,'**本次已移除所有 adjacent_road 干扰桶。边界桶、缺失、桶距及 noisy 中的测距漂移和重复检测保留。**\n')
        lines=[line.replace('及另一道路的局部干扰桶','；其他道路的局部干扰桶已移除') for line in lines]
    if experiment.get('paired_noise'):
        lines.insert(3,'噪声按原始桶身份与采集 tick 固定；与同模式的有桶对照保持共同桶噪声一致，不能与旧顺序随机噪声逐值混比。\n')
    if experiment.get('generation_version') in ('clear05_dense_boundary_arc_reference_v3','boundary_first_v4'):
        lines=[line.replace('0.3–1.5','0.5–1.5').replace('0.38–1.58','0.58–1.58').replace('中心线','参考路线') for line in lines]
        lines=[line for line in lines if not line.startswith('不规则桶包含')]
        lines.insert(4,'本版参考路线按车身左右余量调整，重新以等行驶弧长采样并反解车身姿态；同时保留原道路几何中线。调整过程只使用物理几何，不按感知/planner 成功与否挑选路线。\n')
        lines.insert(5,'规则桶距 0.18 m；不规则初始桶距 0.12–0.25 m，随机缺桶后补齐超过 0.5 m 的间隙。此上限同时检查实际相邻桶中心距离和名义边界弧长。物理桶固定向外偏移 0–0.08 m，noisy 复用完全相同桶位。\n')
        lines.insert(6,'noisy 每帧独立沿相机射线叠加零均值高斯测距误差，每条道路的标准差固定采自 0.08–0.12 m；保留重复检测。视场、容量、延迟不变。\n')
    if not boundary_required:
        pool=read(root/'pool.json')
        lines=[line.replace('三者同时完整','系统筛选通过').replace('全帧有边界输出','全帧边界（仅观察）') for line in lines]
        lines=[line for line in lines if not line.startswith('24 条候选道路')]
        lines.insert(2,f'本次生成 {pool["generated"]} 条独立边界道路；{len(index["cases"])} 条通过物理预检后进入原频率巡检。全部生成、拒绝原因及推荐集合见 pool.html。\n')
        lines.insert(3,'**边界输出不作为筛选条件。每一次真实 1 Hz 规划必须有有限、无碰撞的有效候选，且运行无异常；几何预检与所有原始失败仍保留。**\n')
        lines.insert(4,'左右物理边界先由各自控制点生成，再拟合中间路线。原生对称 Road 只承载虚拟观测基础设施，不用于真实边界、净宽或车身接触评估。\n')
    (root/'REPORT_CN.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(summary))


def render_viewer(root):
    experiment=read(root/'experiment.json') if (root/'experiment.json').exists() else {}
    html=VIEWER
    if experiment.get('exclude_adjacent_road_cones'):
        html=html.replace('不均匀/缺桶/其他道路干扰','不均匀/缺桶（已移除其他道路桶）')
        html=html.replace('短片段道路 · 中心线巡检','短片段道路 · 已移除额外道路桶')
        html=html.replace('没有 policy、PID 或 C* 执行。','没有 policy、PID 或 C* 执行。其他道路桶已移除；边界缺桶、测距漂移和重复检测保留。')
    if experiment.get('generation_version') in ('clear05_dense_boundary_arc_reference_v3','boundary_first_v4'):
        html=html.replace('0.3–1.5','0.5–1.5').replace('中心线','参考路线')
        html=html.replace('短片段道路 · 已移除额外道路桶','短片段道路 · 0.5 m 净宽下限 / 密集向外桶位')
        html=html.replace('同一不规则道路 + 射线漂移/重复检测','固定外移桶位 + 逐帧高斯测距噪声')
        html=html.replace('末端另有 2 m 感知支撑','桶距上限 0.5 m，固定外移 0–8 cm；每帧独立高斯测距 SD 8–12 cm。末端另有 2 m 感知支撑')
        html=html.replace('● 蓝/黄：真实锥桶','灰虚线：道路几何中线；蓝灰实线：巡检参考路线 · ● 蓝/黄：真实锥桶')
        html=html.replace('最大转角 ${vw.physical.max_required_steering_deg.toFixed(1)}°',
            '参考路线最大横移 ${(vw.physical.maximum_reference_shift_m||0).toFixed(3)} m\\n最大转角 ${vw.physical.max_required_steering_deg.toFixed(1)}°')
    if experiment.get('generation_version')=='boundary_first_v4':
        html=html.replace('短片段道路 · 0.5 m 净宽下限 / 密集向外桶位','独立边界道路 · 原频率巡检')
        html=html.replace('只列失败道路','只列规划或运行未通过道路').replace('失败位置（可直接跳转）','事件位置（边界缺失仅观察）')
        html=html.replace('首个失败','首个事件').replace('上一处','上一事件').replace('下一处','下一事件')
        html=html.replace('感知无边界','边界缺失（观察）').replace('本采样帧没有标注失败','本采样帧没有记录事件')
        html=html.replace('红粗线：车身几何','红粗线：车身几何').replace('橙粗线：感知中断','橙粗线：边界缺失观察')
        html=html.replace('宽度控制点','宽度抽样目标')
        html=html.replace('${entry.case.title}\\n','${entry.case.title}\\n系统筛选：${entry.system_pass?"通过":"未通过"}（边界缺失不扣分）\\n')
        html=html.replace('没有 policy、PID 或 C* 执行。','没有 policy、PID 或 C* 执行。左右边界先生成，再拟合巡检路线。')
        html=html.replace('</header>',' <a href="pool.html" style="color:#bfe0ff">查看候选池与推荐道路</a></header>')
    (root/'viewer.html').write_text(html,encoding='utf-8')


def render_pool(root):
    """Data-only candidate review; status wording is independent of admission."""
    from .screen_boundaries import GALLERY
    labels="runtime_pass:'运行通过',runtime_rejected:'规划或运行未通过',geometry_rejected:'几何未通过',construction_rejected:'构造未通过',queued:'待巡检',running:'巡检中',s_left:'左 S 弯',s_right:'右 S 弯',u_left:'左掉头',u_right:'右掉头',compound_left:'左向复合弯',compound_right:'右向复合弯',clear_width_below_0_5:'局部净宽小于 0.5 m',clear_width_above_1_5:'局部净宽超过 1.5 m',physical_boundaries_self_intersect:'边界自交',planner_or_runtime_failure:'存在无效规划或运行异常',steering_limit:'转向角超限',steering_rate:'转向速率超限',body_outside_road_on_centerline:'参考路线车身越界',body_cone_contact_on_centerline:'参考路线车身碰桶',"
    html=GALLERY.replace('const names={','const names={'+labels)
    html=html.replace('JSON.stringify(pool.counts)',"Object.entries(pool.counts).map(([key,value])=>(names[key]||key)+' '+value).join('，')")
    html=html.replace('${r.family} / seed','${names[r.family]||r.family} / seed')
    html=html.replace(":r.status;a.append(p)",":(names[r.status]||r.status);a.append(p)")
    html=html.replace("r.reasons?.join(', ')","r.reasons?.map(reason=>names[reason]||reason).join('，')")
    html=html.replace('边界缺失仅记录；','边界缺失仅记录；感知静止预热 0.4 s 后才开始规划，所有实际作业均纳入筛选；')
    (root/'pool.html').write_text(html,encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--viewer-only',action='store_true');args=parser.parse_args()
    if args.viewer_only:render_viewer(args.input)
    else:build_review(args.input)
