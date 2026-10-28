"""Control-owned static bypass task; workers receive immutable task snapshots.

The first version supports a near-straight observed road, one target, and a
configured passing side. No classifier, unseen map, or RNN state is used.
"""
from copy import deepcopy
import math
import time
import numpy as np
from .geometry import finite_number, wrap_angle


def world(points, pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return [[pose[0]+c*p[0]-s*p[1],pose[1]+s*p[0]+c*p[1]] for p in points]


def body(points, pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return [[c*(p[0]-pose[0])+s*(p[1]-pose[1]),-s*(p[0]-pose[0])+c*(p[1]-pose[1])] for p in points]


def quintic(u):
    u=max(0.,min(1.,u));return u*u*u*(10.+u*(-15.+6.*u))


def task_offset(task, s):
    if s < task['shift_end_s']:
        return task['entry_d']+(task['goal_d']-task['entry_d'])*quintic((s-task['entry_s'])/(task['shift_end_s']-task['entry_s']))
    if task['return_start_s'] is not None:
        return task['goal_d']*(1.-quintic((s-task['return_start_s'])/task['return_length_m']))
    return task['goal_d']


def clusters(points, gap):
    cells={}
    for p in points:cells.setdefault((math.floor(p[0]/gap),math.floor(p[1]/gap)),[]).append(p)
    groups=[]
    while cells:
        key=next(iter(cells));pending=[key];values=cells.pop(key)
        while pending:
            key=pending.pop()
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    other=(key[0]+dx,key[1]+dy)
                    if other in cells:values.extend(cells.pop(other));pending.append(other)
        groups.append(values)  # A single valid beam is not free space.
    return groups


class AvoidanceManeuverManager:
    def __init__(self, cfg):
        self.cfg=cfg;self.phase='NORMAL';self.task=None;self.revision=0;self.next_id=0
        self.epoch=None;self.anchor=None;self.support={};self.pose=None;self.stamp=None
        self.last_measurement=None;self.admitted=False;self.error=None;self.last_completed_id=None
        self.diagnostics={}

    @property
    def active(self):return self.task is not None

    def fail(self, reason):
        self.error=self.error or reason;self.describe();return self.error

    def road_snapshot(self, road, pose, now):
        if (not isinstance(road,dict) or road.get('frame_id')!='base_link' or road.get('time_aligned') is not True
                or not finite_number(road.get('timestamp_s')) or abs(road['timestamp_s']-now)>1e-6):return
        measured=road.get('measurement_timestamp_s')
        if not finite_number(measured):
            age=road.get('source_age_s');measured=now-age if finite_number(age) else None
        if not finite_number(measured) or not 0<=now-measured<self.cfg.road_support_max_age_s:return
        xy=road.get('centerline_xy') or []
        if self.anchor is None and len(xy)>=2:
            a=np.asarray(xy,dtype=float)[:,:2]
            if not np.isfinite(a).all():return
            center=a.mean(axis=0);delta=a-center
            _,axes=np.linalg.eigh(delta.T@delta);axis=axes[:,-1]
            if float(axis@(a[-1]-a[0]))<0:axis=-axis
            normal=np.array([-axis[1],axis[0]])
            if (float(np.max(np.abs(delta@normal)))>self.cfg.maneuver_straight_residual_m
                    or float(np.ptp(delta@axis))<.5):return
            self.anchor=[*world([center.tolist()],pose)[0],pose[2]+math.atan2(axis[1],axis[0])]
        if self.anchor is None:return
        for side in ('left','right'):
            pts=road.get(side+'_boundary_xy') or []
            if road.get('boundary_source',{}).get(side) in ('observed','observed_with_history') and len(pts)>=2:
                self.support[side]=dict(points=world(pts,pose),measurement_s=measured)

    def bounds(self, now):
        if self.anchor is None:return None
        curves={}
        for side in ('left','right'):
            item=self.support.get(side)
            if item is None or not 0<=now-item['measurement_s']<self.cfg.road_support_max_age_s:return None
            curves[side]=body(item['points'],self.anchor)
        low=max(p[1] for p in curves['right']);high=min(p[1] for p in curves['left'])
        start=max(min(p[0] for p in v) for v in curves.values());end=min(max(p[0] for p in v) for v in curves.values())
        return dict(low=low,high=high,start=start,end=end) if low<high and start<end else None

    def update(self, obstacles, road, pose, epoch, now):
        cfg=self.cfg;self.pose=list(pose);self.stamp=now
        if self.epoch is not None and epoch!=self.epoch:
            if self.active:return self.fail('maneuver_motion_epoch_changed')
            self.anchor=None;self.support={}
        self.epoch=epoch
        if not self.active:self.anchor=None;self.support={}
        self.road_snapshot(road,pose,now);bounds=self.bounds(now)
        if self.active and bounds is None:return self.fail('maneuver_road_support_expired')
        if bounds is None:self.describe();return None
        sd=body([pose[:2]],self.anchor)[0];yaw=wrap_angle(pose[2]-self.anchor[2])
        points=body(world(obstacles['points_xyz_m'],pose),self.anchor);measured=obstacles['measurement_timestamp_s']
        relevant=[p for p in points if bounds['low']+cfg.road_edge_exclusion_m<p[1]<bounds['high']-cfg.road_edge_exclusion_m
                  and sd[0]-cfg.rear_extent_m-.5<p[0]<sd[0]+cfg.trigger_lookahead_m]
        if not self.active:
            blocking=[g for g in clusters(relevant,cfg.target_cluster_gap_m)
                if min(p[0] for p in g)>sd[0]+cfg.front_extent_m
                and min(p[1] for p in g)<=cfg.width_m/2+cfg.obstacle_margin_m
                and max(p[1] for p in g)>=-cfg.width_m/2-cfg.obstacle_margin_m]
            if blocking:
                group=min(blocking,key=lambda g:min(p[0] for p in g))
                s_min,s_max=min(p[0] for p in group),max(p[0] for p in group)
                d_min,d_max=min(p[1] for p in group)-cfg.target_lateral_allowance_m,max(p[1] for p in group)+cfg.target_lateral_allowance_m
                side=cfg.maneuver_side
                clearance=cfg.width_m/2+cfg.obstacle_margin_m+cfg.maneuver_clearance_m+cfg.tracking_margin_m
                goal=d_max+clearance if side==1 else d_min-clearance
                margin=cfg.width_m/2+cfg.boundary_margin_m+cfg.tracking_margin_m
                if not bounds['low']+margin<goal<bounds['high']-margin:return self.fail('maneuver_no_side_clearance')
                length=max(cfg.shift_length_m,math.sqrt(6.*abs(goal-sd[1])/cfg.max_curvature_1pm))
                if sd[0]+length+cfg.front_extent_m+cfg.obstacle_margin_m>s_min:return self.fail('maneuver_too_close_to_start')
                self.next_id+=1;self.revision+=1;self.admitted=False
                self.task=dict(id=self.next_id,anchor=list(self.anchor),side=side,entry_s=sd[0],entry_d=sd[1],
                    shift_end_s=sd[0]+length,goal_d=goal,target_s_min=s_min,
                    target_s_max=s_max+cfg.target_depth_allowance_m,target_d_min=d_min,target_d_max=d_max,
                    return_start_s=None,return_length_m=cfg.return_length_m,source_measurement_s=measured,started_at_s=now)
                self.phase='SHIFT'
        elif measured!=self.last_measurement:
            t=self.task
            matches=[p for p in relevant if t['target_s_min']-cfg.target_match_distance_m<=p[0]<=t['target_s_max']+cfg.target_match_distance_m
                     and t['target_d_min']-cfg.target_match_distance_m<=p[1]<=t['target_d_max']+cfg.target_match_distance_m]
            if matches:
                t['target_s_min']=min(t['target_s_min'],min(p[0] for p in matches));t['target_s_max']=max(t['target_s_max'],max(p[0] for p in matches))
                t['target_d_min']=min(t['target_d_min'],min(p[1] for p in matches));t['target_d_max']=max(t['target_d_max'],max(p[1] for p in matches))
                t['source_measurement_s']=measured
        self.last_measurement=measured
        if self.active:
            t=self.task
            if self.phase=='SHIFT' and abs(sd[1]-t['goal_d'])<=cfg.lateral_tolerance_m and abs(yaw)<=cfg.maneuver_heading_tolerance_rad:self.phase='PASSING'
            # Minimum projection of ALL rotated body corners, past the FAR face.
            rear=min(sd[0]+math.cos(yaw)*x-math.sin(yaw)*y for x in (-cfg.rear_extent_m,cfg.front_extent_m) for y in (-cfg.width_m/2,cfg.width_m/2))
            if self.phase in ('SHIFT','PASSING') and rear>t['target_s_max']+cfg.pass_margin_m:
                if abs(sd[1]-t['goal_d'])>cfg.lateral_tolerance_m*2:return self.fail('maneuver_lateral_goal_not_reached')
                self.phase='RETURN';self.revision+=1;t['return_start_s']=sd[0]
            if self.phase=='RETURN' and sd[0]>=t['return_start_s']+t['return_length_m']-.1 and abs(sd[1])<=cfg.lateral_tolerance_m and abs(yaw)<=cfg.maneuver_heading_tolerance_rad:
                self.last_completed_id=t['id'];self.phase='NORMAL';self.task=None;self.admitted=False;self.revision+=1
        self.describe(sd,yaw);return self.error

    def describe(self, sd=None, yaw=None):
        self.diagnostics=dict(phase=self.phase,task_id=None if not self.task else self.task['id'],revision=self.revision,
            goal_d_m=None if not self.task else self.task['goal_d'],vehicle_sd_m=sd,heading_error_rad=yaw,
            last_completed_task_id=self.last_completed_id,target=deepcopy(self.task),error=self.error)

    def collision_points(self, live):
        points=[list(p[:2]) for p in live if math.hypot(*p[:2])<=self.cfg.sensing_radius_m]
        if self.task:
            t=self.task;step=.04
            ss=np.linspace(t['target_s_min'],t['target_s_max'],max(2,math.ceil((t['target_s_max']-t['target_s_min'])/step)+1))
            ds=np.linspace(t['target_d_min'],t['target_d_max'],max(2,math.ceil((t['target_d_max']-t['target_d_min'])/step)+1))
            if len(ss)*len(ds)>self.cfg.memory_capacity:self.fail('maneuver_target_capacity');return points
            points.extend(body(world([[float(s),float(d)] for s in ss for d in ds],self.anchor),self.pose))
        return points

    def context(self):
        return dict(phase=self.phase,task=deepcopy(self.task),revision=self.revision,
            bounds=self.bounds(self.stamp) if self.stamp is not None else None,
            boundaries={side:body(item['points'],self.pose) for side,item in self.support.items()},error=self.error)

    def result_current(self, info):
        return info.get('maneuver_revision')==self.revision and info.get('maneuver_task_id')==(None if not self.task else self.task['id'])


def plan_maneuver(job, cfg):
    """Dedicated avoidance reference; normal lattice candidates never compete."""
    from .avoidance import collision,curvature_peak,path_poses
    from .lattice import ShadowLatticePlanner,ShadowSettings
    started=time.perf_counter();context=job['lidar_avoidance'];state=context['maneuver'];t=state['task']
    info=dict(enabled=True,maneuver_task_id=None if t is None else t['id'],maneuver_revision=state['revision'],
        phase=state['phase'],selected_side=0 if t is None else t['side'],blocking_obstacle=t is not None,
        reason=state.get('error'),memory_point_count=len(context['points_xy_m']))
    if t is None:
        result=ShadowLatticePlanner(ShadowSettings(**job['settings'])).plan(job);result['avoidance']=info;return result
    result=dict(status='NO_AVOIDANCE_PATH',selected=None,candidates=[],candidate_count=0,feasible_count=0,
        input_stamp_s=job['stamp_s'],measurement_stamp_s=job['road'].get('measurement_timestamp_s'),
        shadow_settings=job['settings'],reference=job['road'].get('centerline_xy') or [],check_length_m=cfg.rolling_check_m,avoidance=info)
    try:
        if state.get('error'):raise ValueError(state['error'])
        limits=state['bounds']
        if limits is None:raise ValueError('maneuver_road_support_expired')
        pose=job['shadow_pose'];current_s=body([pose[:2]],t['anchor'])[0][0]
        start=max(t['entry_s'],current_s-.10)
        end=min(current_s+job['settings']['length_m'],limits['end']-cfg.front_extent_m-cfg.boundary_margin_m)
        if end-start<job['preview_m']+.2:raise ValueError('maneuver_insufficient_forward_support')
        n=max(2,math.ceil((end-start)/job['settings']['path_spacing_m']))
        sd=[[start+(end-start)*k/n,task_offset(t,start+(end-start)*k/n)] for k in range(n+1)]
        path=path_poses(body(world(sd,t['anchor']),pose));peak=curvature_peak(path)
        if peak>cfg.max_curvature_1pm:raise ValueError('maneuver_curvature_limit')
        hit=collision(path,context['points_xy_m'],cfg,boundaries=state['boundaries'],plan_tracking=True)
        if hit:result['collision']=hit;raise ValueError('maneuver_'+hit['kind'])
        candidate=dict(id=0,family='maneuver_'+state['phase'].lower(),path=path,cost=0.,collision=None,peak_curvature_1pm=peak)
        result.update(status='VALID',selected=candidate,candidates=[candidate],candidate_count=1,feasible_count=1,generated_length_m=end-start)
    except (ValueError,TypeError,KeyError,ArithmeticError) as error:info['reason']=str(error)
    elapsed=time.perf_counter()-started
    result.update(elapsed_s=elapsed,timing_s=dict(total=elapsed),late_for_requested_period=elapsed>1/job['settings']['rate_hz'])
    return result
