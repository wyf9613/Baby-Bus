"""Annotate failure locations from immutable prescribed-centerline screen records."""
from collections import Counter
from html import escape
from pathlib import Path
import argparse,json,math
from urllib.parse import urlencode
import numpy as np
from dreamgym.envs import Road
from .dataset import write_json
from .robust_roads import centerline_samples,pose_clearances,road_geometry

KINDS={'body_road':'车身越界','body_cone':'车身碰桶','steering_angle':'转向角超限',
       'steering_rate':'转向速率超限','perception':'感知无边界输出','planner':'planner 无有效候选'}
COLORS={'body_road':'#c72f43','body_cone':'#c72f43','steering_angle':'#c72f43',
        'steering_rate':'#c72f43','perception':'#df8d21','planner':'#7647a7'}


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def failure_intervals(frames,indices,kind,reasons):
    """Group adjacent samples; planner indices name actual 1 Hz jobs only."""
    events=[];current=[];previous_reason=None
    def finish():
        if not current:return
        first,last=current[0],current[-1];a,b=frames[first],frames[last]
        events.append(dict(kind=kind,label=KINDS[kind],start_frame=first,end_frame=last,
            start_time_s=a['time_s'],end_time_s=b['time_s'],start_station_m=a['station_m'],
            end_station_m=b['station_m'],first_pose=a['pose'],sample_frames=list(current),
            failed_samples=len(current),reason=reasons[first],
            sampling='actual_1hz_jobs' if kind=='planner' else '20hz_poses'))
    for index in indices:
        reason=reasons.get(index)
        if reason is None:
            finish();current=[];previous_reason=None
        else:
            if current and reason!=previous_reason:finish();current=[]
            current.append(index);previous_reason=reason
    finish()
    return events


def build_failures(root):
    index=read(root/'index.json');maps=read(root/'review-maps.json');model=index['vehicle']
    experiment=read(root/'experiment.json') if (root/'experiment.json').exists() else {}
    boundary_required=experiment.get('acceptance',{}).get('boundary_output_required',True)
    failures_dir=root/'failures';failures_dir.mkdir(exist_ok=True)
    aggregate=[];geometry_cache={};all_counts=Counter()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    for entry in index['cases']:
        case=entry['case'];name=case['fixture_id'];road=road_geometry(case)
        samples=centerline_samples(case);view_results=[]
        for view in entry['views']:
            folder=root/name/view['view'];frames=read(folder/'frames.json');cones=read(folder/'cones.json')['cones']
            moving=[i for i,f in enumerate(frames) if not f['warmup']]
            points=np.array([[c['x_m'],c['y_m']] for c in cones]);radii=np.array([c['radius_m'] for c in cones])
            key=(name,'regular' if view['view']=='regular' else 'irregular')
            if key not in geometry_cache:
                geometry_cache[key]=[pose_clearances(road,points,radii,model,np.array(f['pose'][:2]),f['pose'][2],f['station_m'])
                    for f in frames]
            margins=geometry_cache[key]
            if moving and view.get('completed',True):
                assert math.isclose(min(margins[i][0] for i in moving),view['physical']['minimum_body_road_clearance_m'],abs_tol=1e-8)
                assert math.isclose(min(margins[i][1] for i in moving),view['physical']['minimum_body_cone_clearance_m'],abs_tol=1e-8)
            reasons={kind:{} for kind in KINDS};per_frame=[]
            for i,f in enumerate(frames):
                if f['warmup']:per_frame.append(None);continue
                station_index=i-8;road_margin,cone_margin=margins[i]
                if road_margin<0:reasons['body_road'][i]='body_outside_road_on_centerline'
                if cone_margin<0:reasons['body_cone'][i]='body_cone_contact_on_centerline'
                if abs(samples['steering'][station_index])>model['steering_limit_rad']+1e-8:
                    reasons['steering_angle'][i]='steering_limit'
                if station_index and abs(samples['steering'][station_index]-samples['steering'][station_index-1])/samples['dt_s']>model['steering_rate_limit_rad_per_s']+1e-8:
                    reasons['steering_rate'][i]='steering_rate'
                if not f['exported_boundary']:reasons['perception'][i]=f['road']['status']
                if f['planner'] is not None:
                    if f['planner']['status']!='VALID':reasons['planner'][i]=f['planner']['status']
                    elif not boundary_required:
                        from .screen_boundaries import usable_candidate
                        if not usable_candidate(f['planner']):reasons['planner'][i]='NO_FINITE_COLLISION_FREE_CANDIDATE'
                per_frame.append(dict(road_clearance_m=road_margin,cone_clearance_m=cone_margin,
                    kinds=[kind for kind,items in reasons.items() if i in items]))
            events=[]
            for kind in KINDS:
                indices=[i for i in moving if frames[i]['planner'] is not None] if kind=='planner' else moving
                events.extend(failure_intervals(frames,indices,kind,reasons[kind]))
            events.sort(key=lambda e:(e['start_frame'],list(KINDS).index(e['kind'])))
            for number,event in enumerate(events):
                event['id']=number
                frame=frames[event['start_frame']]
                event['startup_association_wait']=(event['kind']=='perception' and event['reason'] in
                    ('boundary_colour_assignment_pending','boundary_colour_assignment_ambiguous') and
                    not any(f['exported_boundary'] for f in frames[:event['start_frame']]))
                if event['startup_association_wait']:event['label']='启动颜色确认等待'
                if not boundary_required and event['kind']=='perception':event['label']='边界缺失（观察项）'
                event.update(detected_cones=len(frame['detected_cones']),road_status=frame['road']['status'],
                    link='viewer.html?'+urlencode(dict(case=name,view=view['view'],frame=event['start_frame'],failures=1)))
            counts={kind:len(items) for kind,items in reasons.items()}
            payload=dict(events=events,per_frame=per_frame,counts=counts,runtime_exception=view.get('runtime_exception'),
                first_failure_frame=events[0]['start_frame'] if events else None,
                definitions=dict(perception='No exported left OR right boundary, matching existing screening criterion; a missing centerline alone is not failure.',
                    planner='Only actual 1 Hz job results are classified; no new jobs or outputs are inferred between samples.',
                    geometry='Passive body/road/cone checks on prescribed poses, not actual dynamic collisions.'))
            write_json(folder/'failures.json',payload);view_results.append(payload)
            view['failure_events']=len(events)
            all_counts.update({kind:bool(value) for kind,value in counts.items()})
        if not any(r['events'] or r.get('runtime_exception') for r in view_results):continue
        first=min((dict(e,view=view['view']) for view,r in zip(entry['views'],view_results) for e in r['events']),key=lambda e:e['start_frame'],default=None)
        aggregate.append(dict(case_id=name,title=case['title'],split=case['split'],first=first,
            gate_failure=(not boundary_required and not entry.get('system_pass',True)) or any(r.get('runtime_exception') or any(e['kind']!='perception' or boundary_required for e in r['events']) for r in view_results),
            views=[dict(view=v['view'],events=len(r['events']),counts=r['counts'],first=r['events'][0] if r['events'] else None,
                runtime_exception=r.get('runtime_exception')) for v,r in zip(entry['views'],view_results)]))
        m=maps[name]
        extent=np.concatenate([np.array(m[side]) for side in ('left','right')])
        span=np.ptp(extent,axis=0)
        height=min(6,max(3.2,4.3*span[1]/max(span[0],.1)+1.2))
        fig,axes=plt.subplots(1,3,figsize=(14,height),constrained_layout=True)
        for ax,view,r in zip(axes,entry['views'],view_results):
            for side in ('left','right'):
                p=np.array(m[side]);ax.plot(p[:,0],p[:,1],c='#a4afba',lw=.8)
            p=np.array(m['center']);ax.plot(p[:,0],p[:,1],c='#cad1d9',lw=.8,ls='--')
            if m.get('reference'):
                p=np.array(m['reference']);ax.plot(p[:,0],p[:,1],c='#57718e',lw=.8)
            folder=root/name/view['view'];frames=read(folder/'frames.json');cones=read(folder/'cones.json')['cones']
            for color,ink in [('blue','#3f6fb0'),('yellow','#c8a52a')]:
                xy=np.array([[c['x_m'],c['y_m']] for c in cones if c['colour']==color]);ax.scatter(xy[:,0],xy[:,1],c=ink,s=8)
            labelled=set()
            for event in r['events']:
                kind=event['kind'];color=COLORS[kind];ids=event['sample_frames'];xy=np.array([frames[i]['pose'][:2] for i in ids])
                category='geometry' if kind.startswith(('body_','steering_')) else kind
                label=category if category not in labelled else None;labelled.add(category)
                ax.plot(xy[:,0],xy[:,1],c=color,lw=0 if kind=='planner' else 3,
                    marker='x' if kind=='planner' else '.',ms=5,label=label,alpha=.85)
            if r['events']:
                event=r['events'][0];x,y,angle=event['first_pose'];c,s=math.cos(angle),math.sin(angle)
                body=np.array([[-model['rear_m'],-model['width_m']/2],[model['front_m'],-model['width_m']/2],
                    [model['front_m'],model['width_m']/2],[-model['rear_m'],model['width_m']/2]])@np.array([[c,s],[-s,c]])+[x,y]
                ax.add_patch(Polygon(body,fill=False,edgecolor='#172e49',lw=1.3))
                reason=('startup colour confirmation' if event['startup_association_wait'] else
                    event['reason'].replace('_',' '))
                from textwrap import fill
                ax.annotate(f"first: s={event['start_station_m']:.2f}m | frame {event['start_frame']}\n"+fill(reason,38),
                    xy=(x,y),xytext=(.03,.03),textcoords='axes fraction',fontsize=7,va='bottom',
                    bbox=dict(boxstyle='round,pad=.3',fc='white',ec='#b9c3cf',alpha=.9),
                    arrowprops=dict(arrowstyle='->',lw=.6))
            ax.set_title(view['view']+f" | {len(r['events'])} intervals",fontsize=10);ax.set_aspect('equal');ax.grid(alpha=.15)
            if labelled:ax.legend(fontsize=7,loc='upper left')
        fig.suptitle(case['title']+' | red: body/geometry, orange: no road output, purple x: failed native jobs',fontsize=10)
        fig.savefig(failures_dir/(name+'.png'),dpi=150);plt.close(fig)
    write_json(root/'index.json',index)
    gated=aggregate if boundary_required else [r for r in aggregate if r['gate_failure']]
    write_json(root/'event-index.json',dict(cases=aggregate,roads_with_events=len(aggregate)))
    write_json(root/'failure-index.json',dict(cases=gated,failed_roads=len(gated),total_roads=len(index['cases']),
        view_counts_with_failure=dict(all_counts)))
    lines=['# 失败位置索引（已移除额外道路桶）','',
        f'{len(gated)}/{len(index["cases"])} 条道路在至少一种观测条件下有筛查未通过。原始轨迹没有重跑或改写。',
        '图中红色为车身几何问题，橙色为无边界输出的 20 Hz 区间，紫色叉为失败的真实 1 Hz 规划作业。车框与箭头标出该观测条件下的首个失败。',
        'planner 区间只汇总连续失败的规划采样，不推断两次作业之间又运行了规划。几何接触为规定姿态的被动检查，不是实车或动态驾驶碰撞。',
        '部分首个标记位于起点，是颜色关联尚未确认的启动等待，不等于行进中道路失效。此处单独标明，原始全程覆盖统计没有修改。','',
        '| 道路 | 划分 | 观测 | 首个失败站位 | 类型与原始原因 | 跳转 |','|---|---|---|---:|---|---|']
    for row in aggregate:
        for view in row['views']:
            e=view['first']
            if e:lines.append(f"| {row['title']} | {row['split']} | {view['view']} | {e['start_station_m']:.2f} m | {e['label']}：{e['reason']} | [帧 {e['start_frame']}]({e['link']}) |")
    for row in aggregate:lines+=['',f"## {row['title']}",'',f"![失败区间](failures/{row['case_id']}.png)"]
    if not boundary_required:
        lines.insert(2,'边界缺失仅作为观察事件，不纳入道路筛选失败；failure-index.json 只索引几何、规划或运行异常，event-index.json 保留全部事件。')
    (root/'FAILURE_LOCATIONS_CN.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    render_gallery(root,gated)
    print(json.dumps(dict(failed_roads=len(gated),total_roads=len(index['cases']),roads_with_observations=len(aggregate),view_counts_with_failure=dict(all_counts))))


def render_gallery(root,aggregate):
    cards=[]
    for row in aggregate:
        links=[]
        for view in row['views']:
            e=view['first']
            if e:links.append(f'<a href="{escape(e["link"],quote=True)}">{escape(view["view"])}：{escape(e["label"])} · s={e["start_station_m"]:.2f}m · 帧{e["start_frame"]}</a>')
            elif view.get('runtime_exception'):links.append(escape(view['view'])+'：'+escape(view['runtime_exception']))
            else:links.append(escape(view['view'])+'：本条件无标注失败')
        cards.append(f'<section><h2>{escape(row["title"])} <small>{escape(row["split"])}</small></h2><p>'+"<br>".join(links)+
            f'</p><a href="failures/{row["case_id"]}.png"><img src="failures/{row["case_id"]}.png"></a></section>')
    html='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>失败位置总览</title>
<style>body{font:15px system-ui;background:#edf2f7;color:#26374b;margin:24px}header,section{background:white;padding:20px;border-radius:12px;margin-bottom:18px}h1,h2{margin:0 0 10px}small{font-size:14px;color:#657586}img{width:100%;height:auto}a{color:#365e98;line-height:1.9}</style>
<header><h1>14 条道路的失败位置</h1><p>已移除额外道路桶。红色：车身几何问题；橙色：感知无边界输出；紫叉：失败的真实规划作业。</p>
<p>点击条件链接跳到原始帧；点击图片放大。启动颜色确认等待与弯中失效单独标注。原始记录和通过率未改变。</p><a href="viewer.html?failures=1">进入逐帧查看器（只列失败道路）</a></header>'''+''.join(cards)+'</html>'
    html=html.replace('14 条道路',f'{len(aggregate)} 条道路')
    experiment=read(root/'experiment.json') if (root/'experiment.json').exists() else {}
    if experiment.get('generation_version') in ('clear05_dense_boundary_arc_reference_v3','boundary_first_v4'):
        html=html.replace('已移除额外道路桶。','已移除额外道路桶。净宽下限 0.5 m，桶距上限 0.5 m，固定向外偏移 0–8 cm，逐帧独立高斯测距 SD 8–12 cm；巡检使用车身余量调整后的参考路线。')
    if not experiment.get('acceptance',{}).get('boundary_output_required',True):
        html=html.replace('失败位置','规划或运行未通过的位置').replace('橙色：感知无边界输出','橙色：边界缺失观察（不参与筛选）')
    (root/'failures.html').write_text(html,encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--gallery-only',action='store_true');args=parser.parse_args()
    if args.gallery_only:render_gallery(args.input,read(args.input/'failure-index.json')['cases'])
    else:build_failures(args.input)
