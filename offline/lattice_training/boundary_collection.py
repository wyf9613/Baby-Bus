"""Dynamic native-plant collection on frozen, independent physical boundaries."""
from collections import deque
from copy import deepcopy
import hashlib,json,math
import numpy as np
from dreamgym.envs.road_core.cones import ResolvedCone,ResolvedConeSet
from offline.planner_sim import SensorTiming,load_notebook
from .screen_roads import PrescribedSensorEnvironment
from .robust_roads import centerline_samples
from .boundary_roads import geometry
from .pipeline import with_terminal_support,terminal_region


class BoundaryCollectionEnvironment(PrescribedSensorEnvironment):
    """Only sensing, true geometry and initialization ports differ from legacy.

    Native RK4 dynamics, C*, candidate generation and legal continuation are shared.
    The derived native carrier remains the reference-station coordinate for progress.
    """
    def __init__(self,case,cones,*,sigma,episode_seed,start_index,start_count,max_steps,safety,start_mode='stratified_random'):
        if start_mode not in ('stratified_random','road_start'):raise ValueError('Unknown boundary initial mode')
        self.collection_start_mode=start_mode
        self.boundary_case=deepcopy(case);self.collection_physical_cones=deepcopy(cones)
        self.collection_physical_geometry=geometry(case,case.get('terminal_sensor_support_m',2.))
        self.collection_scene_id=case['boundary_sha256']
        self.collection_route_length_m=case['mission_road_length_m']
        self.collection_terminal_support_m=case.get('terminal_sensor_support_m',2.)
        self.initial_episode_seed=episode_seed;self.initial_start_index=start_index;self.initial_start_count=start_count
        self.collection_safety=safety;self.noise_sigma=sigma
        config=load_notebook(with_obstacles=False)
        config.road_spec=with_terminal_support(case['road_spec'],self.collection_terminal_support_m)
        config.integration_config['environment_step_duration_s']=.05
        config.initial_state_config['pose']['progress'].update(lower=.25,upper=.25)
        config.initial_state_config['pose']['lateral_offset_m'].update(lower=0.,upper=0.)
        config.initial_state_config['pose']['heading_error_rad'].update(lower=0.,upper=0.)
        config.initial_state_config['body_motion']['longitudinal_velocity']['value_mps'].update(lower=0.,upper=0.)
        super().__init__(config,max_steps=max_steps,terminate_on_collision=True,
            sensor_timing=SensorTiming(period_steps={'cone_detections':1},delay_steps={'cone_detections':1}))

    def configuration(self):
        result=super().configuration()
        result['boundary_fixture']=deepcopy(self.boundary_case)
        result['physical_boundary_source']='original_independent_curves_not_native_host_lane'
        result['physical_cones_sha256']=hashlib.sha256(json.dumps(self.collection_physical_cones,sort_keys=True).encode()).hexdigest()
        result['physical_cones']=deepcopy(self.collection_physical_cones)
        result['measurement_noise']=dict(distribution='independent_gaussian_radial',sigma_m=self.noise_sigma,
            camera_fov_deg=80.,camera_near_clip=False,delay_steps=1,seed=self.initial_episode_seed,
            fixed_cone_offsets_reused=True)
        return result

    def _evaluation(self,previous_truth=None,action=None):
        result=super()._evaluation(previous_truth,action)
        raw=self._backend.car._state_values();car=self._backend.car
        model=dict(front_m=car._body_front_extent_from_center_of_mass_m,rear_m=car._body_rear_extent_from_center_of_mass_m,width_m=car._body_width_m)
        left,right=self.collection_physical_geometry.footprint_margins(model,np.array(raw[:2]),raw[2])
        points=np.array([[c['x_m'],c['y_m']] for c in self.collection_physical_cones]);radii=np.array([c['radius_m'] for c in self.collection_physical_cones])
        co,si=math.cos(raw[2]),math.sin(raw[2]);delta=points-np.array(raw[:2]);bx=delta[:,0]*co+delta[:,1]*si;by=-delta[:,0]*si+delta[:,1]*co
        dx=np.maximum(np.maximum(-model['rear_m']-bx,bx-model['front_m']),0);dy=np.maximum(abs(by)-model['width_m']/2,0)
        cone_margin=float(np.min(np.hypot(dx,dy)-radii))
        result.update(collision=bool(result['collision'] or min(left,right)<0 or cone_margin<0),
            physical_road_clearance_m=min(left,right),physical_cone_clearance_m=cone_margin,
            physical_geometry_source='original_independent_boundaries_and_frozen_cones')
        return result

    def reset(self,*,seed=0):
        self.screen_noise=False
        super().reset(seed=seed)
        samples=centerline_samples(self.boundary_case);model=samples['model']
        rng=np.random.default_rng(self.initial_episode_seed)
        low=float(samples['station'][0]);goal=self.collection_route_length_m-model['front_m']-.2
        maximum=goal-1.5
        if maximum<=low:raise ValueError('Insufficient route length for multi-start collection')
        a=low+(maximum-low)*self.initial_start_index/self.initial_start_count
        b=low+(maximum-low)*(self.initial_start_index+1)/self.initial_start_count
        station=low if self.collection_start_mode=='road_start' else float(rng.uniform(a,b))
        x,y=[float(np.interp(station,samples['station'],samples['xy'][:,j])) for j in (0,1)]
        heading=float(np.interp(station,samples['station'],samples['heading']))
        beta=float(np.interp(station,samples['station'],samples['beta']));steering=float(np.interp(station,samples['station'],samples['steering']))
        expanded=dict(front_m=model['front_m']+self.collection_safety.safety_margin_m,
            rear_m=model['rear_m']+self.collection_safety.safety_margin_m,width_m=model['width_m']+2*self.collection_safety.safety_margin_m)
        left,right=self.collection_physical_geometry.footprint_margins(expanded,np.array([x,y]),heading)
        offset_low=max(-.12,-right);offset_high=min(.12,left)
        offset=float(rng.uniform(offset_low,offset_high)) if offset_low<=offset_high else 0.
        heading_error=float(rng.uniform(-.035,.035));speed=float(rng.uniform(.08,.25))
        if self.collection_start_mode=='road_start':
            a=b=low;offset=heading_error=speed=0.
        tangent=heading+beta;x-=math.sin(tangent)*offset;y+=math.cos(tangent)*offset;heading+=heading_error
        self.collection_initial_sampling=dict(mode=self.collection_start_mode,stratum_index=self.initial_start_index,
            strata=self.initial_start_count,stratum_bounds_m=[a,b],drawn_progress_m=station,
            lateral_offset_m=offset,lateral_support_m=[offset_low,offset_high],heading_error_rad=heading_error,
            initial_speed_mps=speed,draws=1,filtered_by_planner=False)
        records=[]
        for i,c in enumerate(self.collection_physical_cones):
            xy=(c['x_m'],c['y_m']);nominal=(c.get('nominal_x_m',xy[0]),c.get('nominal_y_m',xy[1]))
            records.append(ResolvedCone(xy,nominal,c['colour'],1 if c['colour']=='blue' else 0,c['radius_m'],
                c['side'],'boundary_collection',0,i,c['station_m'],c['station_m'],c.get('outward_displacement_m',0.)))
        self._backend._resolved_cones=ResolvedConeSet(np.array([r.position_world_m for r in records]),
            np.array([r.color_id for r in records]),np.array([r.radius_m for r in records]),tuple(records))
        self.screen_noise=True;self.screen_paired_noise=True;self.screen_seed=self.initial_episode_seed
        self.screen_sigma=self.noise_sigma;self.screen_rng=np.random.default_rng(self.initial_episode_seed)
        self.screen_bias={};self.screen_cones=np.array([r.position_world_m for r in records])
        self.screen_source_ids=[c.get('source_id',i) for i,c in enumerate(self.collection_physical_cones)]
        self._pending={key:deque() for key in self._pending};self._latest={}
        def refresh(tick,moving):
            self._step_count=tick;self._backend._simulation_time_s=tick*self.dt_s
            velocity=speed if moving else 0.
            self._backend.car.reset(state=dict(world_pose=dict(x_m=x,y_m=y,heading_rad=heading),
                body_motion=dict(longitudinal_velocity_mps=velocity*math.cos(beta),lateral_velocity_mps=velocity*math.sin(beta),
                    yaw_rate_rad_per_s=velocity/model['rear_axle_from_cg_m']*math.sin(beta)),front_wheel_steering_angle_rad=steering))
            self._backend.car.set_action_request(0.,steering)
            self._backend._previous_reference_line_progress_m=station
            pending=self._backend._pending_reference_line_progress_search
            self._backend._pending_reference_line_progress_search={
                'lower_bound_m':max(0.,station-.1),'upper_bound_m':station+.1}
            try:native,info=self._backend._get_observation_and_info_and_update_ground_truth()
            finally:self._backend._pending_reference_line_progress_search=pending
            self._backend._latest_observation,self._backend._latest_info=native,info
            self._frame=self._perceive(native,initial=tick==0)
        for tick in range(8):refresh(tick,False)
        refresh(8,True)
        self._done=False
        self._info={'evaluation':self._evaluation(),'native_info':self._backend._latest_info,'seed':seed}
        return deepcopy(self._frame),deepcopy(self._info)


def make_boundary_initializer(jobs,pool_manifest_sha256):
    def initialize(kind,seed,initial_index,settings,objective,road_settings,solver_settings):
        job=jobs[seed];case=job['case'];episode_seed=int(np.random.SeedSequence([seed,initial_index,61031]).generate_state(1)[0])
        limit=(settings.max_segments+1)*math.ceil(objective.max_horizon_s/objective.dt_s)+10
        env=BoundaryCollectionEnvironment(case,job['cones'],sigma=job['sigma'],episode_seed=episode_seed,
            start_index=initial_index,start_count=settings.starts_per_road,max_steps=limit,safety=solver_settings)
        env.collection_episode_metadata=dict(fixture_id=case['fixture_id'],physical_kind=case['kind'],
            split_group_id=case['split_group'],source_road_seed=case['road_seed'],repetition=job['repetition'])
        env.collection_label_contract=dict(boundary_geometry_contract=dict(version='independent_boundary_cstar_v1',
            hard_constraints='original_boundary_polygon_and_frozen_physical_cones',
            progress_reference='derived_native_reference_fitted_between_original_boundaries',
            pool_manifest_sha256=pool_manifest_sha256,noise_distribution='independent_gaussian_radial',sigma_range_m=[.08,.12]),
            road_randomization=dict(preset='frozen_qualified_boundary_pool',pool_manifest_sha256=pool_manifest_sha256,
                physical_maps_unchanged=True,extra_dropout_applied=False))
        observation,_=env.reset(seed=episode_seed)
        raw=env._backend.car._state_values()
        initialization=dict(episode_seed=episode_seed,initial_state_draws=1,warmup_steps=8,filtered_by_planner=False,
            sampling_domain='independent_boundary_physical_free_space',warmup_terminated=False,warmup_truncated=False,
            route_length_m=env.collection_route_length_m,physical_road_length_m=env.collection_route_length_m+env.collection_terminal_support_m,
            terminal_support_m=env.collection_terminal_support_m,known_lane_width_prior_m=env.estimator.settings.known_lane_width_m,
            initial_sampling=env.collection_initial_sampling,initial_region=terminal_region(env,solver_settings),
            initial_control=[0.,float(raw[6]/env._backend.car._requested_front_wheel_angle_magnitude_limit_rad)])
        return env,observation,initialization
    return initialize
