"""Prescribed-centerline sensor and native-planner screening; no policy training.

Run with ``python -m offline.lattice_training.screen_roads --output NEW_DIRECTORY``.
The virtual vehicle follows model-consistent prescribed poses. This is not PID,
C* execution, or evidence that a learned selector can drive the road.
"""
from collections import Counter,deque
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import argparse,json,math
import numpy as np
from dreamgym.envs.road_core.cones import ResolvedCone,ResolvedConeSet
from offline.mvp21_sim.simulation import MVP21SimulationEnv
from offline.mvp21_sim.source import parameters,runtime_provenance,verify_preserved_files
from offline.mvp21_sim.validation_workspace import source_snapshot
from offline.planner_sim import SensorTiming,load_notebook
from mvp21_policy.lattice import ShadowSettings,ShadowLatticePlanner
from mvp21_policy.planning import PlanningSettings
from mvp21_policy.transport import ShadowJobSource
from .dataset import write_json,json_safe
from .robust_roads import FAMILIES,generate_case,cone_layout,centerline_samples,physical_screen,vehicle_model


def filter_extra_cones(cones,exclude=False):
    """Keep stable source identities and remove only the adjacent-road treatment."""
    records=[dict(c,source_id=c.get('source_id',i)) for i,c in enumerate(cones)]
    return [c for c in records if not exclude or c['role']!='adjacent_road']


def paired_cone_noise(seed,tick,source_id,radius,sigma,previous):
    """IID Gaussian radial measurement error; placement offsets live in the map."""
    rng=np.random.default_rng(np.random.SeedSequence([seed+3091,tick,source_id]))
    bias=rng.normal(0,sigma)
    return bias,bool(rng.random()<.12),float(rng.normal(0,.01))


class PrescribedSensorEnvironment(MVP21SimulationEnv):
    def _perceive(self,native,*,initial=False):
        if getattr(self,'screen_noise',False):
            native=deepcopy(native);count=int(native['cone_detections_count'][0])
            forward=native['cone_detections_forward_positions_in_body_frame_m'];left=native['cone_detections_left_positions_in_body_frame_m']
            colours=native['cone_detections_color_ids'];n=count;capacity=len(forward)
            px,py,heading,*_=self._backend.car._state_values();co,si=math.cos(heading),math.sin(heading)
            paired={}
            if self.screen_paired_noise:
                for identity,point in enumerate(self.screen_cones):
                    radius=float(np.linalg.norm(point-[px,py]));source_id=self.screen_source_ids[identity]
                    values=paired_cone_noise(self.screen_seed,self._step_count,source_id,radius,self.screen_sigma,
                        self.screen_bias.get(identity,0.))
                    self.screen_bias[identity]=values[0];paired[identity]=values
            for i in range(count):
                x,y=float(forward[i]),float(left[i]);world=np.array([px+co*x-si*y,py+si*x+co*y])
                identity=int(np.argmin(np.sum((self.screen_cones-world)**2,axis=1)))
                radius=math.hypot(x,y);sigma=self.screen_sigma
                if self.screen_paired_noise:bias,duplicate_requested,duplicate_offset=paired[identity]
                else:
                    bias=self.screen_rng.normal(0,sigma)
                    duplicate_requested=False;duplicate_offset=0.
                self.screen_bias[identity]=bias;scale=max(.001,radius+bias)/radius if radius else 1.
                forward[i],left[i]=x*scale,y*scale
                if n<capacity and (duplicate_requested if self.screen_paired_noise else self.screen_rng.random()<.12):
                    offset=duplicate_offset if self.screen_paired_noise else self.screen_rng.normal(0,.01)
                    duplicate=max(.001,radius+bias+offset)/radius if radius else 1.
                    forward[n],left[n],colours[n]=x*duplicate,y*duplicate,colours[i];n+=1
            native['cone_detections_count'][0]=n
        return super()._perceive(native,initial=initial)


def sensor_probe(case,cones,view,output,*,paired_noise=False):
    from .pipeline import with_terminal_support
    config=load_notebook(with_obstacles=False)
    config.road_spec=with_terminal_support(case['road_spec'],case.get('terminal_sensor_support_m',0.))
    samples=centerline_samples(case);config.integration_config['environment_step_duration_s']=samples['dt_s']
    cfg=parameters();settings=ShadowSettings(**cfg['shadow']);planning=PlanningSettings(**cfg['planning'])
    messages=[];planner=ShadowLatticePlanner(settings);statuses=Counter();road_statuses=Counter();frames=[]
    start=samples['station'][0]
    config.initial_state_config['pose']['progress'].update(lower=float(start),upper=float(start))
    source=ShadowJobSource();source.shadow_settings=settings;source.planning_settings=planning
    source.shadow_last_submit_s=-math.inf;source.shadow_handoff_estimate_s=settings.handoff_allowance_s
    source.shadow_generation=1;source.shadow_job_id=0;source.shadow_worker=SimpleNamespace(submit=messages.append)
    with PrescribedSensorEnvironment(config,max_steps=10000,terminate_on_collision=False,
            sensor_timing=SensorTiming(delay_steps={'cone_detections':1})) as env:
        env.reset(seed=case['road_seed']);backend=env._backend
        records=[]
        for i,c in enumerate(cones):
            xy=(c['x_m'],c['y_m']);colour=c['colour']
            records.append(ResolvedCone(xy,xy,colour,1 if colour=='blue' else 0,c['radius_m'],
                c['side'],'screen',0,i,c['station_m'],c['station_m'],0.))
        backend._resolved_cones=ResolvedConeSet(np.array([r.position_world_m for r in records]),
            np.array([r.color_id for r in records]),np.array([r.radius_m for r in records]),tuple(records))
        env._pending={key:deque() for key in env._pending};env._latest={}
        env.screen_noise=view=='noisy';env.screen_rng=np.random.default_rng(case['road_seed']+3091)
        env.screen_bias={};env.screen_cones=np.array([r.position_world_m for r in records])
        env.screen_source_ids=[c.get('source_id',i) for i,c in enumerate(cones)]
        env.screen_paired_noise=paired_noise;env.screen_seed=case['road_seed']
        env.screen_sigma=float(np.random.default_rng(case['road_seed']+11).uniform(*case['generation']['frame_noise']['sigma_range_m']))
        source.motion_history=env.estimator.motion
        with (output/'trace.jsonl').open('w',encoding='utf-8') as stream:
            for tick,index in enumerate([0]*8+list(range(len(samples['station'])))):
                warm=tick<8;station=float(samples['station'][index]);x,y=samples['xy'][index]
                heading=float(samples['heading'][index]);beta=float(samples['beta'][index]);speed=0. if warm else samples['speed_mps']
                backend.car.reset(state=dict(world_pose=dict(x_m=float(x),y_m=float(y),heading_rad=heading),
                    body_motion=dict(longitudinal_velocity_mps=speed*math.cos(beta),lateral_velocity_mps=speed*math.sin(beta),
                        yaw_rate_rad_per_s=0. if warm else float(samples['yaw_rate'][index])),
                    front_wheel_steering_angle_rad=float(samples['steering'][index])))
                backend._simulation_time_s=tick*samples['dt_s'];env._step_count=tick
                pending=backend._pending_reference_line_progress_search
                backend._pending_reference_line_progress_search={'lower_bound_m':max(0.,station-.1),'upper_bound_m':station+.1}
                try:native,info=backend._get_observation_and_info_and_update_ground_truth()
                finally:backend._pending_reference_line_progress_search=pending
                backend._previous_reference_line_progress_m=station
                backend._latest_observation,backend._latest_info=native,info
                observation=env._perceive(native,initial=tick==0);env._frame=observation
                source.motion_history=env.estimator.motion;source.estimation_output=observation['estimation']
                now=observation['timestamp_s'];source._monotonic=lambda:now
                # Boundary-first screening starts the 1 Hz clock only AFTER the
                # static sensor warmup. Every actual job is then a screened job.
                if not warm or not case.get('boundary_geometry'):source._shadow_offer()
                result=None
                if messages:
                    job=messages.pop(0);result=planner.plan(job)
                    if not warm:statuses[result['status']]+=1
                road=observation['estimation']['road'];geom=road if any(road.get(k) for k in
                    ('left_boundary_xy','right_boundary_xy','centerline_xy')) else road.get('forward_geometry') or {}
                if not warm:road_statuses[road['status']]+=1
                sensors=observation['sensors']['cone_detections'] or {};detected=sensors.get('detections') or []
                frame=dict(tick=tick,warmup=warm,station_m=station,time_s=now,pose=[float(x),float(y),heading],
                    detected_cones=detected,road=road,state=observation['estimation']['state'],planner=result,
                    exported_boundary=bool(geom.get('left_boundary_xy') or geom.get('right_boundary_xy')),
                    centerline=bool(geom.get('centerline_xy')))
                stream.write(json.dumps(json_safe(frame),allow_nan=False)+'\n');frames.append(frame)
        write_json(output/'environment.json',env.configuration())
    moving=[f for f in frames if not f['warmup']];planned=[f for f in moving if f['planner'] is not None]
    output_count=sum(f['exported_boundary'] for f in moving);center_count=sum(f['centerline'] for f in moving)
    valid=sum(f['planner']['status']=='VALID' for f in planned)
    # Strict complete coverage is reported separately from partial availability.
    summary=dict(view=view,frames=len(moving),exported_boundary_frames=output_count,centerline_frames=center_count,
        road_statuses=dict(road_statuses),planner_statuses=dict(statuses),planner_jobs=len(planned),valid_planner_jobs=valid,
        complete_boundary_coverage=output_count==len(moving),complete_planner_coverage=bool(planned) and valid==len(planned),
        warmup_planner_jobs=sum(f['planner'] is not None for f in frames if f['warmup']),
        planner_first_tick=next((f['tick'] for f in frames if f['planner'] is not None),None),
        radial_sigma_base_m=env.screen_sigma if view=='noisy' else 0.,
        radial_correlation=0. if view=='noisy' else None,
        radial_noise_distribution='gaussian_iid' if view=='noisy' else None,
        duplicate_detection_probability=.12 if view=='noisy' else 0.,
        noise_sampling='per_cone_per_tick_paired' if paired_noise else 'legacy_visible_detection_sequence',
        prescribed_centerline_motion=True,policy_used=False,control_used=False,
        prescribed_reference_kind=samples['reference_kind'],
        maximum_reference_shift_m=samples['maximum_reference_shift_m'],
        camera_fov_deg=config.observation_config['cone_detections']['acquisition']['horizontal_field_of_view_deg'],
        camera_near_clip=False,radar='dummy')
    write_json(output/'summary.json',summary)
    # Compact immutable playback, planner geometry retained only at its 1 Hz ticks.
    write_json(output/'frames.json',frames)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seeds',default='3200,3201,3202,3203');parser.add_argument('--families',default=','.join(FAMILIES))
    parser.add_argument('--no-extra-cones',action='store_true',help='Remove only adjacent-road physical cone records')
    parser.add_argument('--paired-noise',action='store_true',help='Match radial noise by source cone and tick across ablations')
    args=parser.parse_args();seeds=[int(s) for s in args.seeds.split(',')];families=args.families.split(',')
    if args.output.exists():raise FileExistsError('Choose a new screening output directory')
    args.output.mkdir(parents=True);verify_preserved_files()
    write_json(args.output/'source.json',source_snapshot());write_json(args.output/'vehicle.json',vehicle_model())
    from .robust_roads import GENERATION_VERSION,CONE_PLACEMENT
    write_json(args.output/'experiment.json',dict(exclude_adjacent_road_cones=args.no_extra_cones,paired_noise=args.paired_noise,
        generation_version=GENERATION_VERSION,clear_width_range_m=[.5,1.5],cone_placement=CONE_PLACEMENT,
        frame_noise=dict(distribution='gaussian_iid_radial',sigma_range_m=[.08,.12],correlation=0.)))
    entries=[]
    for i,seed in enumerate(seeds):
        split='test' if i==len(seeds)-1 else 'validation' if i==len(seeds)-2 else 'train'
        for family in families:
            case=generate_case(family,seed,split);directory=args.output/case['fixture_id'];directory.mkdir()
            write_json(directory/'road.json',case);samples=centerline_samples(case);views=[]
            for view in ('regular','irregular','noisy'):
                cones,layout=cone_layout(case,irregular=view!='regular')
                original_count=len(cones);cones=filter_extra_cones(cones,args.no_extra_cones)
                layout.update(excluded_adjacent_road_cones=args.no_extra_cones,extra_cones_removed=original_count-len(cones))
                output=directory/view;output.mkdir()
                write_json(output/'cones.json',dict(cones=cones,layout=layout))
                physical=physical_screen(case,cones,samples);write_json(output/'physical.json',physical)
                summary=sensor_probe(case,cones,view,output,paired_noise=args.paired_noise);summary['physical']=physical;views.append(summary)
                print(json.dumps(dict(case=case['fixture_id'],split=split,view=view,physical=physical,
                    frames=summary['frames'],boundary_frames=summary['exported_boundary_frames'],
                    planner_jobs=summary['planner_jobs'],valid_jobs=summary['valid_planner_jobs'])),flush=True)
            entry=dict(case=case,views=views);entries.append(entry);write_json(args.output/'index.json',dict(cases=entries))
    verify_preserved_files()


if __name__=='__main__':main()
