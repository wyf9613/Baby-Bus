"""Full candidate execution and whole-road evaluation shared by run and visualization."""
from copy import deepcopy
import math
import time

import numpy as np

from offline.planner_sim import PlannerSimulationEnv, SensorTiming, load_notebook
from .cstar import CStarSolver, NativePrediction, performance_profile, PerformanceActuator
from .dataset import json_safe
from .objective import CStarTrackingSettings, reference_plan
from .roads import road_length, road_identity
from .features import encode_candidate


def road_environment(spec, seed=0, *, progress=.25, offset=0.0, heading_error=0.0, speed=.2, max_steps=30000):
    config = load_notebook()
    config.road_spec = deepcopy(spec)
    for key,value in (("progress",progress),("lateral_offset_m",offset),("heading_error_rad",heading_error)):
        config.initial_state_config["pose"][key].update(lower=float(value),upper=float(value))
    config.initial_state_config["body_motion"]["longitudinal_velocity"]["value_mps"].update(lower=speed,upper=speed)
    env = PlannerSimulationEnv(config,max_steps=max_steps,sensor_timing=SensorTiming(
        period_steps={"cone_detections":2},delay_steps={"cone_detections":1}),
        estimation_settings=_training_estimation_settings())
    env.reset(seed=seed)
    env.step((0,0))
    return env


def _training_estimation_settings():
    from offline.mvp21_sim.source import parameters
    settings=dict(parameters()['estimation'])
    settings.update(left_colour=2,right_colour=1)
    return settings


def world_points(points, pose):
    points = np.asarray(points,dtype=float)
    c,s = math.cos(pose[2]),math.sin(pose[2])
    return points @ np.array([[c,s],[-s,c]])+np.asarray(pose[:2])


def candidate_panel(planner, observation, pose, *, model=None, progress_m=None, objective=None):
    from .pipeline import candidates_at
    ref,diag = candidates_at(planner,observation,model)
    rows=[]
    for candidate in diag.get("candidate_references",[]):
        score = float(model.predict(encode_candidate(observation,candidate)[None,:])[0]) if model else candidate['descriptor']['score']
        plan=reference_plan(candidate,objective)
        rows.append({"candidate_id":candidate['candidate_id'],"admitted":True,"score":score,"reason":"feasible",
                     "descriptor":candidate['descriptor'],"points":world_points(candidate['trajectory_xy_m'],pose).tolist(),
                     "selected":candidate['candidate_id']==diag.get('selected_candidate_id'),
                     "speed_profile":candidate['reference']['speed_profile'],
                     "full_path_duration_s":plan['duration_s'],"spatial_span_m":plan['spatial_span_m']})
    if rows:
        best=max(rows,key=lambda row:row['score'])
        for row in rows: row['selected']=row['candidate_id']==best['candidate_id']
        diag['native_selected_candidate_id']=diag.get('selected_candidate_id')
        diag['selected_candidate_id']=best['candidate_id']
        ref=next(c['reference'] for c in diag['candidate_references'] if c['candidate_id']==best['candidate_id'])
    for proposal in (diag.get('result') or {}).get('candidates',[]):
        if proposal['collision'] is None:continue
        rows.append(dict(candidate_id=str(proposal['id']),admitted=False,selected=False,
            score=None,descriptor=proposal,reason='collision',
            points=world_points([p[:2] for p in proposal['path']],pose).tolist()))
    return {"progress_m":progress_m,"pose":list(pose),"timestamp_s":observation['timestamp_s'],
            "prediction_space":model.prediction_space if model else "native_rule_score",
            "valid":bool(ref.get('valid')),"reason":ref.get('reason'),"candidates":rows,
            "selected_candidate_id":diag.get('selected_candidate_id'),
            "admitted_count":sum(row['admitted'] for row in rows),
            "rejected_count":sum(not row['admitted'] for row in rows)},ref,diag


class RoadEvaluation:
    def __init__(self, environment, *, model=None, objective=None, solver_settings=None,
                 profile='standard', seed=0, performance=None, cruise_speed=.2):
        from .pipeline import make_planner
        self.env,self.model,self.seed=environment,model,seed
        self.objective=objective or (CStarTrackingSettings.from_specification(model.provenance['objective'])
                                     if model is not None else CStarTrackingSettings())
        if model is not None and self.objective.specification()!=model.provenance['objective']:
            raise ValueError('Evaluation C* objective differs from the policy checkpoint')
        self.solver=CStarSolver(self.objective,solver_settings)
        self.planner=make_planner(profile, cruise_speed=cruise_speed)
        self.cruise_speed=cruise_speed
        self.profile=profile
        self.performance=performance or performance_profile(seed,nominal=True)
        self.actuator=PerformanceActuator(self.performance,(0,0))
        self.snapshots={}
        self.report=None

    def run(self, *, max_segments=100, max_control_steps=None, goal_progress=None, progress=None):
        from .pipeline import simulate_segment,road_goal_progress
        started=time.perf_counter(); env=self.env
        spec=env.config.road_spec; length=road_length(spec)
        goal=goal_progress if goal_progress is not None else road_goal_progress(env)
        initial=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        start_pose=list(env._backend.car._state_values()[:3])
        trace,segments,panels=[],[],[]
        previous=(0.0,0.0); fault=None; completion=None
        initial_margins=NativePrediction(env,self.solver.settings).margins(env._backend.car._state_values())
        for number in range(max_segments):
            if min(initial_margins.values()) < -1e-7:
                fault='illegal_initial_state';break
            if env._done:
                fault='environment_finished_during_initialization';break
            if max_control_steps is not None and len(trace)>=max_control_steps:
                completion='complete_segment_budget_reached';break
            current=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
            if current>=goal:
                completion='road_goal_reached';break
            self.snapshots[len(trace)]=env.snapshot()
            observation=env._frame
            panel,ref,diag=candidate_panel(self.planner,observation,env._backend.car._state_values()[:3],
                                          model=self.model,progress_m=current,objective=self.objective)
            panels.append(panel)
            if not ref.get('valid'):
                fault='planning:'+str(ref.get('reason'));break
            candidate=next(c for c in diag['candidate_references'] if c['candidate_id']==diag['selected_candidate_id'])
            solve=self.solver.solve(env,candidate,previous,seed=self.seed+number*100)
            snapshot,transition=simulate_segment(env,candidate,solve,previous,self.objective,self.solver.settings,
                                                performance=self.performance,actuator=self.actuator)
            for row in transition.get('trace',[]):
                row['step']=len(trace);row['segment']=number;row['candidate_id']=candidate['candidate_id'];trace.append(row)
            plan=reference_plan(candidate,self.objective)
            segments.append({'index':number,'candidate_id':candidate['candidate_id'],'score':panel['candidates'][
                next(i for i,r in enumerate(panel['candidates']) if r['candidate_id']==candidate['candidate_id'])]['score'],
                'start_progress_m':current,'pose':list(panel['pose']),
                'reference_points':world_points(plan['states'][:,:2],panel['pose']).tolist(),
                'full_path_duration_s':plan['duration_s'],'spatial_span_m':plan['spatial_span_m'],
                'actual_loss':transition.get('actual_loss'),'completion':transition.get('completion'),
                'fault':transition.get('reason'),'solver_diagnostics':solve['diagnostics']})
            if progress:
                progress({'stage':'evaluate_segment','segment':number,'progress_m':trace[-1]['progress_m'] if trace else current,
                          'steps':transition['steps'],'duration_s':plan['duration_s'],'fault':transition.get('reason')})
            if snapshot is None:
                fault=transition['reason'];break
            env.restore(snapshot);previous=tuple(transition['final_control'])
            if ref['planned_stop']:
                completion='planned_stop_reference_complete';break
        else:
            fault='maximum_segments_reached'
        final=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        if trace: final=trace[-1]['progress_m']
        if final>=goal and fault is None: completion='road_goal_reached'
        collision=any(row['constraint_margins']['obstacle_contact_margin']<0 for row in trace)
        offroad=any(row['constraint_margins']['road_clearance_m']<0 for row in trace)
        costs=[s['actual_loss'] for s in segments if s['actual_loss'] is not None]
        self.report={'format':'lattice_full_road_evaluation_v1','road_id':road_identity(spec),'road_spec':spec,
            'seed':self.seed,'selector':'learned' if self.model else 'rule','lattice_profile':self.profile,
            'cruise_speed_mps':self.cruise_speed,
            'planner_profile':{'length_m':self.planner.cfg.length_m,
                'collision_coverage':self.planner.cfg.collision_coverage,'score_coverage':self.planner.cfg.score_coverage},
            'candidate_representation':'policy_frenet_candidate_v1',
            'objective':self.objective.specification(),'performance':self.performance,'trace':trace,'segments':segments,
            'candidate_panels':panels,'initial_pose':start_pose,'initial_progress_m':initial,'final_progress_m':final,
            'initial_constraint_margins':initial_margins,
            'goal_progress_m':goal,'road_length_m':length,'progress_m':final-initial,'steps':len(trace),
            'return':-sum(costs)-(self.objective.failure_loss if fault else 0),
            'fault':fault,'completion':completion,'road_completed':completion=='road_goal_reached',
            'planned_stop':completion=='planned_stop_reference_complete',
            'stop_completed':completion=='planned_stop_reference_complete' and bool(trace and trace[-1]['speed_mps']<=self.objective.stop_speed_tolerance_mps),
            'collision':collision,'offroad':offroad,'wall_s':time.perf_counter()-started,'runtime_qualified':False,
            'environment_configuration':env.configuration()}
        return self.report

    def probe(self, progress_m, *, speed=.2, offset=0., heading_error=0.,use_trace=True):
        from .pipeline import make_planner
        if not 0<=progress_m<=road_length(self.env.config.road_spec): raise ValueError('Point lies beyond this road')
        trace=self.report['trace'] if self.report else []
        nearest=min(range(len(trace)),key=lambda i:abs(trace[i]['progress_m']-progress_m)) if trace else None
        if use_trace and nearest is not None and abs(trace[nearest]['progress_m']-progress_m)<=.15:
            base=max(k for k in self.snapshots if k<=nearest)
            branch=PlannerSimulationEnv(self.env.config,max_steps=self.env.max_steps)
            branch.restore(self.snapshots[base])
            for row in trace[base:nearest+1]: branch.step(row['action'])
            source='executed_trace'
        else:
            branch=road_environment(self.env.config.road_spec,self.seed,progress=progress_m,speed=speed,
                                    offset=offset,heading_error=heading_error)
            source='local_initial_observation'
        with branch:
            current=float(branch._info['evaluation']['ground_truth']['reference_line_progress_m'])
            planner=make_planner(self.profile, cruise_speed=self.cruise_speed)
            panel,_,_=candidate_panel(planner,branch._frame,branch._backend.car._state_values()[:3],
                                      model=self.model,progress_m=current,objective=self.objective)
            panel['requested_progress_m']=progress_m;panel['state_source']=source
            raw=branch._backend.car._state_values()
            cx,cy,_,_,_,tangent,_=branch._backend.road.find_closest_point_to(float(raw[0]),float(raw[1]))
            panel['state']={'speed_mps':math.hypot(raw[3],raw[4]),
                'lateral_offset_m':-(raw[0]-cx)*math.sin(tangent)+(raw[1]-cy)*math.cos(tangent),
                'heading_error_rad':math.atan2(math.sin(raw[2]-tangent),math.cos(raw[2]-tangent))}
            return json_safe(panel)
