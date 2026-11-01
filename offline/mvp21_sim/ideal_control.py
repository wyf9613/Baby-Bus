"""Offline oracle tracking: exact execution pose and constant arc speed.

No hidden road enters selection. No PID, actuator dynamics, steering limits or
path smoothing: this is a path-quality diagnostic, not a physical vehicle.
"""
from copy import deepcopy
import math
import numpy as np
from mvp21_policy.execution import PlannerExecutionManager
from mvp21_policy.geometry import _path_lengths,_path_project,_path_sample


def world_points(points,pose):
    points=np.asarray(points,dtype=float)[:,:2]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return points@np.array([[c,s],[-s,c]])+pose[:2]


def walk(points,distance):
    """Interpolate arc length, without snapping to the beginning of a path."""
    points=np.asarray(points,dtype=float)
    lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
    arc=np.r_[0.,np.cumsum(lengths)]
    distance=min(max(0.,distance),float(arc[-1]))
    index=max(0,min(int(np.searchsorted(arc,distance,side='right'))-1,len(lengths)-1))
    tangent=(points[index+1]-points[index])/lengths[index]
    return points[index]+tangent*(distance-arc[index]),math.atan2(tangent[1],tangent[0]),distance


def tracking_step(points,distance,join_length,join_heading):
    xy,heading,arc=walk(points,distance)
    # A holonomic ideal correction must not rotate the camera sideways just
    # because the vehicle is centimetres off its reference. Follow the road's
    # tangent during the finite lateral join; leave true path bends unchanged.
    if arc<=join_length+1e-10:heading=join_heading
    return xy,heading,arc


class IdealExecutionManager(PlannerExecutionManager):
    """Shared admission/cropping, with private oracle execution anchors.

    Perception and planner fusion retain their measured motion histories.
    """
    def __init__(self,env,**kwargs):
        self.env=env;self.true_poses={}
        super().__init__(**kwargs)

    def record_pose(self,stamp):
        self.true_poses[round(stamp,9)]=list(self.env._backend.car._state_values()[:3])

    def accept(self,result):
        mapped=deepcopy(result)
        if mapped.get('status')=='VALID':
            anchor=mapped.get('execution_anchor') or {}
            anchor['pose']=self.true_poses[round(mapped['input_stamp_s'],9)]
            mapped['execution_anchor']=anchor
        return super().accept(mapped)

    def select(self,road,state,pose,epoch,*args):
        result=super().select(road,state,list(self.env._backend.car._state_values()[:3]),epoch,*args)
        self.diagnostics['execution_pose_source']='explicit_simulation_truth'
        return result


class IdealPathController:
    def __init__(self,env,settings,config):
        self.env,self.settings,self.config=env,settings,config
        self.clear()

    def __deepcopy__(self,memo):
        result=type(self)(self.env,self.settings,self.config)
        result.points=None if self.points is None else self.points.copy()
        result.progress=self.progress;result.speed=self.speed
        result.join_length=self.join_length;result.join_heading=self.join_heading
        result.last_diagnostics=deepcopy(self.last_diagnostics)
        return result

    def clear(self):
        self.points=None;self.progress=0.;self.speed=0.;self.last_diagnostics={}
        self.join_length=0.;self.join_heading=0.

    def calculate(self,reference,state,vehicle_params,now_s,dt):
        points=np.asarray((reference.get('path') or {}).get('samples_xy',[]),dtype=float)
        if points.ndim!=2 or points.shape[0]<2 or points.shape[1]!=2 or not np.isfinite(points).all():
            return 0.,0.,dict(valid=False,stop_requested=True,reason='ideal_invalid_path')
        points=points[np.r_[True,np.linalg.norm(np.diff(points,axis=0),axis=1)>1e-10]]
        if len(points)<2:return 0.,0.,dict(valid=False,stop_requested=True,reason='ideal_empty_path')
        arc=_path_lengths(points);nearest=_path_project(points,arc)
        if nearest is None:return 0.,0.,dict(valid=False,stop_requested=True,reason='ideal_forward_branch_unavailable')
        start=nearest[1]
        cropped=[_path_sample(points,arc,start)]+[p for p,s in zip(points,arc) if s>start+1e-10]
        if len(cropped)<2:return 0.,0.,dict(valid=False,stop_requested=True,reason='ideal_coverage_exhausted')
        raw=list(self.env._backend.car._state_values());world=world_points(cropped,raw)
        # Traverse a straight connector to a delayed/new route; never teleport.
        route=np.vstack([raw[:2],world])
        route=route[np.r_[True,np.linalg.norm(np.diff(route,axis=0),axis=1)>1e-10]]
        if len(route)<2:return 0.,0.,dict(valid=False,stop_requested=True,reason='ideal_empty_path')
        self.points=route;self.progress=0.
        self.join_length=float(np.linalg.norm(world[0]-raw[:2]))
        self.join_heading=math.atan2(world[1,1]-world[0,1],world[1,0]-world[0,0])
        self.speed=0. if reference.get('stop_requested') else float(reference['target_speed_mps'])
        diagnostics=dict(valid=True,stop_requested=bool(reference.get('stop_requested')),
            reason='ideal_arc_length_tracking',ideal=True,actuator_command_applied=False,
            execution_pose_source='explicit_simulation_truth',connector_length_m=float(np.linalg.norm(world[0]-raw[:2])),
            target_speed_mps=self.speed,steering_limits_enforced=False,path_smoothing=False)
        diagnostics['join_mode']='finite_holonomic_translation_with_reference_tangent'
        self.last_diagnostics=diagnostics
        return 0.,0.,diagnostics

    def advance(self):
        """Move on the selected reference; reuse the original sensor producer."""
        env=self.env;backend=env._backend
        if env._done:raise RuntimeError('Cannot advance a finished ideal run')
        previous=backend.get_current_ground_truth();raw=list(backend.car._state_values())
        dt=env.dt_s;travel=0.;heading=raw[2];xy=np.asarray(raw[:2])
        if self.points is not None and self.speed>0:
            xy,heading,following=tracking_step(self.points,self.progress+self.speed*dt,self.join_length,self.join_heading)
            travel=following-self.progress;self.progress=following
        delta=math.atan2(math.sin(heading-raw[2]),math.cos(heading-raw[2]))
        heading=raw[2]+delta
        # Report actual translational velocity, including explicit lateral joins.
        velocity=(xy-np.array(raw[:2]))/dt;c,s=math.cos(heading),math.sin(heading)
        backend.car.reset(state=dict(world_pose=dict(x_m=float(xy[0]),y_m=float(xy[1]),heading_rad=heading),
            body_motion=dict(longitudinal_velocity_mps=float(c*velocity[0]+s*velocity[1]),
                lateral_velocity_mps=float(-s*velocity[0]+c*velocity[1]),yaw_rate_rad_per_s=delta/dt),
            front_wheel_steering_angle_rad=0.))
        env._step_count+=1;backend._simulation_time_s=env.elapsed_time_s
        # Match the host's step-local projection window, without a map oracle.
        backend.reference_line_progress_search_radius_m=2*float(np.sum(np.abs(xy-raw[:2])))+1e-6
        native,native_info=backend._get_observation_and_info_and_update_ground_truth()
        truth=backend.get_current_ground_truth()
        backend._previous_reference_line_progress_m=truth['reference_line_progress_m']
        backend._previous_reference_line_road_element_index=truth['reference_line_road_element_index']
        backend._latest_observation,backend._latest_info=native,native_info
        env._frame=env._perceive(native)
        evaluation=env._evaluation(previous,[0.,0.])
        terminated=bool(env.terminate_on_collision and evaluation['collision'])
        truncated=env._step_count>=env.max_steps and not terminated
        reasons={'ideal_physical_contact':True} if terminated else {}
        evaluation.update(terminated=terminated,truncated=truncated,termination_reasons=reasons)
        env._done=terminated or truncated
        env._info=dict(evaluation=evaluation,native_info=native_info,termination_reasons=reasons,
            truncation_reason='step_limit' if truncated else None,policy_debug=(None,None))
        self.last_diagnostics.update(step_arc_m=travel,heading_step_deg=abs(delta)*180/math.pi)
        return deepcopy(env._frame),0.,terminated,truncated,deepcopy(env._info)
