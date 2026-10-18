"""Ports around owning planner/perception/C* modules; never reimplement algorithms."""
from copy import deepcopy
import math
import time
from types import SimpleNamespace

import numpy as np

from mvp21_policy.candidates import from_lattice
from mvp21_policy.frenet import ShadowFrame
from mvp21_policy.lattice import ShadowSettings,_shadow_collision
from offline.lattice_training.cstar import CStarSolver,SolverSettings
from offline.lattice_training.objective import CStarTrackingSettings
from .delivery import SimulatedPlannerWorker
from .simulation import ideal_observation


def mvp1_candidate(job):
    """MVP1 fixed straight baseline: one body-forward line, no search or road fit."""
    cfg=ShadowSettings(**job['settings'])
    result=dict(status='NO_ROAD',selected=None,candidates=[],candidate_count=0,feasible_count=0,
        input_stamp_s=job['stamp_s'],planning_input=dict(road=job['road'],state=job['state']),
        planner_backend='mvp1_straight')
    count=max(1,math.ceil(cfg.length_m/cfg.path_spacing_m))
    points=[[float(x),0.,0.] for x in np.linspace(0.,cfg.length_m,count+1)]
    road=job['road'];sources=road.get('boundary_source') or {}
    boundaries={s:road[s+'_boundary_xy'] for s in ('left','right') if road.get(s+'_boundary_xy')
        and sources.get(s) in ('observed','observed_with_history')}
    collision=_shadow_collision(points,float('inf'),boundaries,cfg)
    candidate=dict(id=0,family='mvp1_straight',path=points,collision=collision,cost=0.,costs={},offsets_m=[])
    frame=ShadowFrame([p[:2] for p in points]);s0,d0=frame.project((0.,0.))
    result.update(status='VALID' if collision is None else 'ALL_COLLIDE',selected=candidate,
        candidates=[candidate],candidate_count=1,feasible_count=int(collision is None),reference_chart_xy=[p[:2] for p in points],
        reference_origin='mvp1_straight',start_s_m=s0,start_d_m=d0,known_boundary_sides=list(boundaries),
        measurement_stamp_s=road.get('measurement_timestamp_s'),shadow_settings=vars(cfg),
        generated_length_m=frame.end,collision_method='shared_native_boundary_check')
    return result


class ModularPlannerWorker(SimulatedPlannerWorker):
    """Generation delivery and latest-pool selection have separate causal clocks.

    Selection ticks without a new pool are idle; a faster selector never fabricates
    new candidates or combines a delayed path with a mismatched current frame.
    """
    def __init__(self,config,selector=None):
        super().__init__(delay_s=config['planner_latency_s'],candidate_selector=selector,
            candidate_planner=mvp1_candidate if config['modules']['planner']=='mvp1_straight' else None)
        self.period=1/config['frequencies_hz']['policy'];self.next_selection=None
        self.selection_waiting=None;self.selection_events=[]
        self.generated=[]

    def submit(self,job):
        job=deepcopy(job);job['policy_observation']=deepcopy(self.policy_observation)
        if self.next_selection is None:self.next_selection=self.now_s+self.delay_s
        super().submit(job)

    def _start(self,job,start_s):
        selector=self.candidate_selector;self.candidate_selector=None
        try:super()._start(job,start_s)
        finally:self.candidate_selector=selector

    def poll(self,now_s):
        packet=super().poll(now_s)
        if packet is not None:
            self.generated.append(deepcopy(packet));self.selection_waiting=packet
        if self.next_selection is None or now_s+1e-9<self.next_selection:return None
        tick=self.next_selection
        while self.next_selection<=now_s+1e-9:self.next_selection+=self.period
        packet,self.selection_waiting=self.selection_waiting,None
        self.selection_events.append(dict(timestamp_s=now_s,tick_s=tick,job_id=packet['job']['job_id'] if packet else None))
        if packet is None:return None
        started=time.perf_counter()
        if self.candidate_selector is not None:
            packet['result']=self.candidate_selector.select(packet['result'],packet['job']['policy_observation'])
        wall=time.perf_counter()-started
        packet.update(planner_ready_s=packet['ready_s'],selection_tick_s=tick,ready_s=now_s,delivered_s=now_s,
            policy_wall_s=wall,planner_wall_s=packet['native_wall_s']+wall,
            round_trip_s=now_s-packet['job']['submitted_monotonic_s'])
        return packet


class PerfectPerception:
    def __init__(self,period):self.period=period;self.cached=None;self.pose=None;self.stamp=None
    def apply(self,env,observation,step):
        now=observation['timestamp_s'];pose=list(env._backend.car._state_values()[:3])
        if self.cached is None or step%self.period==0:
            ideal=ideal_observation(env,observation);self.cached=deepcopy(ideal['estimation']['road'])
            # Perfect obstacle polygons are an explicit oracle port, never hidden
            # in the observed-perception or policy-default inputs.
            polygons=env._backend._obstacle_polygons_m[env._backend._obstacle_active_mask]
            self.obstacles=[p.tolist() for polygon in polygons for p in polygon]
            self.pose=pose;self.stamp=now
        from .diagnose_perception import world_points,body_points
        result=deepcopy(observation);road=deepcopy(self.cached)
        for key in ('centerline_xy','left_boundary_xy','right_boundary_xy'):
            road[key]=body_points(world_points(road[key],self.pose),pose)
        road.update(timestamp_s=now,measurement_timestamp_s=self.stamp,source_age_s=now-self.stamp,
            time_aligned=True,road_input_source='explicit_perfect_perception')
        result['estimation']['road']=road
        points=body_points(self.obstacles,pose)
        result['estimation']['obstacles'].update(valid=True,available=True,status='explicit_perfect_perception',
            points_xyz_m=[[x,y,0.] for x,y in points],timestamp_s=now,source_age_s=now-self.stamp)
        return result


def timed_candidate(reference,speed):
    """Authored execution timing only; never a learned-policy feature."""
    points=reference['path']['samples_xy'];headings=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(points,points[1:])]
    path=[[p[0],p[1],headings[min(i,len(headings)-1)]] for i,p in enumerate(points)]
    frame=ShadowFrame(points);s0,_=frame.project((0.,0.))
    contract=from_lattice(dict(path=path),dict(reference_chart_xy=points,input_stamp_s=reference['timestamp_s'],start_s_m=s0))
    length=contract['samples'][-1][0];target=reference['target_speed_mps'];duration=2*length/(speed+target)
    ref=deepcopy(reference);ref.update(frenet_path=dict(type='SAMPLED_FRENET',candidate=contract),planned_stop=False,
        longitudinal_profile=dict(type='P_PATH_OF_TIME',duration_s=duration,
            coeffs_low_to_high=[0.,speed*duration,(target-speed)*duration/2],stop=False),
        speed_profile=[[0.,speed],[duration,target]])
    return dict(candidate_id='control_reference',reference=ref)


class CStarController:
    """Solve each new reference with existing C*; dispatch its sequence at control Hz."""
    def __init__(self,env,settings,config):
        self.env,self.settings,self.config=env,settings,config;self.clear()

    def __deepcopy__(self,memo):
        result=type(self)(self.env,self.settings,self.config)
        for key in ('key','controls','position','previous','solves','last_diagnostics'):
            if hasattr(self,key):setattr(result,key,deepcopy(getattr(self,key),memo))
        return result

    def clear(self):self.key=None;self.controls=[];self.position=0;self.previous=[0.,0.];self.solves=getattr(self,'solves',0)

    def calculate(self,reference,state,vehicle_params,now_s,dt):
        period=1/self.config['frequencies_hz']['control']
        key=(reference.get('planner_version'),reference.get('trajectory_id'))
        if reference.get('planner_version')!='lattice_execution':key+=(math.floor(now_s*self.config['frequencies_hz']['planner']),)
        if key!=self.key or self.position>=len(self.controls):
            hold=round(period/self.env.dt_s)
            original=self.env._backend
            class HoldBackend:
                effective_integration_substep_count=original.effective_integration_substep_count*hold
                def __getattr__(self,name):return getattr(original,name)
            backend=HoldBackend()
            proxy=SimpleNamespace(_backend=backend,dt_s=period,config=self.env.config)
            objective=CStarTrackingSettings(**{**self.config['cstar_objective'],'dt_s':period})
            solver=CStarSolver(objective,SolverSettings(**self.config['cstar_solver']))
            candidate=timed_candidate(reference,max(0.,float(state['speed_mps'])))
            solution=solver.solve(proxy,candidate,previous_action=self.previous,seed=self.solves)
            self.solves+=1;self.last_diagnostics=solution['diagnostics']
            if solution['status']!='approximate':
                return 0.,0.,dict(valid=False,stop_requested=True,reason='cstar_'+solution['reason'],cstar=solution['diagnostics'])
            self.key=key;self.controls=solution['controls'].tolist();self.position=0
        drive,plant_steer=self.controls[self.position];self.position+=1;self.previous=[drive,plant_steer]
        return drive,-plant_steer,dict(valid=True,stop_requested=False,reason='cstar_control_sequence',
            cstar=self.last_diagnostics,sequence_index=self.position-1,reference_source='selected_execution_reference')
