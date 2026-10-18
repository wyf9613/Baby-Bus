"""Replay fixed U-turn validation roads through perception, native, optional policy and C*."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np

from .cstar import CStarSolver, SolverSettings
from .dataset import write_json
from .evaluation import world_points
from .features import encode_candidate, scene_features
from .model import SpatialPathPolicy
from .objective import CStarTrackingSettings
from .pipeline import (PipelineSettings, initialize_episode, make_planner, candidates_at,
                       road_goal_progress, simulate_segment)
from .roads import RoadRandomizationSettings
from .scenarios import road_input
from .source import planner_source_sha256, policy_encoder_sha256
from .validation_roads import VALIDATION_KINDS, validation_case, DIRECTORY
from .visualize import scene_data


def run_case(kind, *, model=None, selector='native', mode='perception', start_mode='default',
             reference_scale=None, max_segments=30, seed=713):
    case=validation_case(kind)
    objective=CStarTrackingSettings()
    if model is not None and model.provenance['objective'] != objective.specification():
        raise ValueError('Frozen policy objective differs from validation execution')
    settings=PipelineSettings(kinds=(kind,),train_seeds=(seed,),validation_seeds=(),test_seeds=(),
        train_only=True,starts_per_road=1,planner_profiles=('standard',),until_road_end=True,
        max_segments=max_segments,planner_sample_count=8,planner_rate_hz=20,terminal_support_m=2.,
        road_input_mode=mode,initial_start_mode=start_mode,use_scenario_width_prior=False)
    safety=SolverSettings(population=6,iterations=1,max_evaluations=12,max_wall_s=30)
    env,_,initialization=initialize_episode(kind,seed,0,settings,objective,RoadRandomizationSettings(),safety)
    if env.estimator.settings.known_lane_width_m != 0:
        env.close()
        raise ValueError('No-prior validation requires estimation.known_lane_width_m=0')
    planner=make_planner('standard',sample_count=settings.planner_sample_count,rate_hz=settings.planner_rate_hz)
    if reference_scale is not None:
        planner.cfg.unknown_width_reference_scale=reference_scale
        planner.cfg.__post_init__()
    nominal_length,nominal_preview=planner.cfg.length_m,planner.settings.preview_m
    solver=CStarSolver(objective,safety)
    started=time.perf_counter()
    trace,segments=[],[]
    previous=(0.,0.)
    fault=None
    with env:
        initial_state=list(env._backend.car._state_values())
        initial_progress=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
        initial_stamp=env._frame['timestamp_s']
        current=initial_progress
        goal=road_goal_progress(env)
        if not initialization['initial_region']['legal']:
            fault='illegal_initial_state'
        elif initialization['warmup_terminated'] or initialization['warmup_truncated']:
            fault='environment_finished_during_initialization'
        else:
            for index in range(max_segments):
                current=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])
                if goal-current <= objective.completion_tolerance_m:break
                remaining=goal-current
                planner.cfg.length_m=remaining/math.ceil(remaining/nominal_length)
                planner.settings.preview_m=min(nominal_preview,planner.cfg.length_m)
                observation=road_input(env._frame,mode)
                reference,diagnostics=candidates_at(planner,observation)
                result=diagnostics['result']
                choices=diagnostics['candidate_references']
                native_id=diagnostics.get('selected_candidate_id')
                inputs=dict(road=deepcopy(observation['estimation']['road']),
                    state=deepcopy(observation['estimation']['state']),native_status=result['status'],
                    known_boundary_sides=result.get('known_boundary_sides',[]),
                    native_reference_origin=result.get('reference_origin'),
                    reference_boundary_offset_m=result.get('reference_boundary_offset_m'),
                    unknown_width_reference_scaled=result.get('unknown_width_reference_scaled',False),
                    native_fusion_status=(result.get('fusion') or {}).get('status'))
                pose=list(env._backend.car._state_values()[:3])
                if not reference.get('valid') or not choices:
                    fault='planning:'+str(reference.get('reason','no_candidates'))
                    segments.append(dict(index=index,pose=pose,start_progress_m=current,
                        start_timestamp_s=env._frame['timestamp_s'],candidates=[],fault=fault,
                        accepted=False,input_diagnostics=inputs))
                    break
                context=scene_features(observation) if model is not None else None
                learned=[float(model.score(encode_candidate(observation,c,context=context)))
                    if model is not None else None for c in choices]
                if selector=='policy':
                    if model is None:raise ValueError('Policy selection requires an explicit model')
                    chosen=int(np.argmax(learned))
                else:
                    chosen=next(i for i,c in enumerate(choices) if c['candidate_id']==native_id)
                selected=choices[chosen]
                seed_solve=(initialization['episode_seed']+index*100+17)%2**32
                solution=solver.solve(env,selected,previous,seed=seed_solve)
                snapshot,transition=simulate_segment(env,selected,solution,previous,objective,safety)
                first_trace=len(trace)
                for row in transition.get('trace',[]):
                    env.step(row['action'])
                    raw=list(env._backend.car._state_values())
                    np.testing.assert_allclose(raw[:3],[row['x_m'],row['y_m'],row['body_heading_rad']],rtol=0,atol=1e-9)
                    trace.append(dict(row,raw_state=raw,segment=index,candidate_id=selected['candidate_id'],
                        segment_accepted=snapshot is not None))
                panels=[]
                for i,c in enumerate(choices):
                    native_score=c['descriptor']['score']
                    score=learned[i] if selector=='policy' else native_score
                    panels.append(dict(candidate_id=c['candidate_id'],score=score,policy_score=learned[i],
                        native_cost=-native_score,family=c['descriptor']['family'],selected=i==chosen,
                        trajectory_xy_m=c['trajectory_xy_m'],world_xy_m=world_points(c['trajectory_xy_m'],pose).tolist()))
                segments.append(dict(index=index,pose=pose,start_progress_m=current,
                    start_timestamp_s=transition['start_timestamp_s'],candidates=panels,
                    native_selected_candidate_id=native_id,selected_candidate_id=selected['candidate_id'],
                    selected_score=panels[chosen]['score'],solver_status=solution['status'],
                    solver_reason=solution.get('reason'),solver_diagnostics=solution['diagnostics'],
                    accepted=snapshot is not None,fault=transition['reason'],completion=transition.get('completion'),
                    actual_loss=transition.get('actual_loss'),first_trace_index=first_trace,
                    trace_count=len(trace)-first_trace,input_diagnostics=inputs))
                if trace:current=trace[-1]['progress_m']
                if snapshot is None:
                    fault=transition['reason'];break
                env.restore(snapshot)
                np.testing.assert_allclose(env._backend.car._state_values(),transition['end_state'],rtol=0,atol=1e-9)
                previous=tuple(transition['final_control'])
            else:
                if goal-current > objective.completion_tolerance_m:fault='maximum_segments_reached'
        completed=fault is None and goal-current <= objective.completion_tolerance_m
        return dict(format='fixed_uturn_validation_rollout_v1',episode_id=':'.join((kind,str(seed),mode,selector,start_mode)),
            kind=kind,variant=case['variant'],direction=case['direction'],selector=selector,
            road_input_mode=mode,initial_start_mode=start_mode,road_seed=seed,
            fixture_kind=kind,fixture_road_id=case['physical_road_id'],full_width_m=case['full_width_m'],
            fixture_suite_version=case.get('suite_version',1),
            fixture_sha256=hashlib.sha256((DIRECTORY/(kind.removeprefix('validation_')+'.json')).read_bytes()).hexdigest(),
            initial_state=initial_state,initial_timestamp_s=initial_stamp,initial_progress_m=initial_progress,
            final_progress_m=current,goal_progress_m=goal,mission_road_length_m=case['mission_road_length_m'],
            road_completed=completed,completion='road_goal_reached' if completed else None,fault=fault,
            explicit_scenario_width_prior=False,known_lane_width_m=env.estimator.settings.known_lane_width_m,
            unknown_width_reference_scale=planner.cfg.unknown_width_reference_scale,
            cruise_speed_mps=.2,fallback=False,policy_connected_to_main=False,training_performed=False,
            replanning='after_complete_spatial_path',runtime_qualified=False,
            pipeline_settings=asdict(settings),solver_settings=asdict(safety),objective=objective.specification(),
            initialization=initialization,road_spec=env.config.road_spec,scene=scene_data(env.config.road_spec),
            body_geometry=env.configuration()['bicycle_model_config']['body_geometry'],
            axle_geometry=env.configuration()['bicycle_model_config']['axle_geometry'],
            trace=trace,segments=segments,wall_s=time.perf_counter()-started)


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--kinds',type=lambda s:tuple(s.split(',')),default=VALIDATION_KINDS)
    parser.add_argument('--modes',type=lambda s:tuple(s.split(',')),default=('perception','no_centerline'))
    parser.add_argument('--selectors',type=lambda s:tuple(s.split(',')),default=('native',))
    parser.add_argument('--model',type=Path)
    parser.add_argument('--start-mode',choices=('default','road_start'),default='default')
    parser.add_argument('--reference-scale',type=float)
    parser.add_argument('--max-segments',type=int,default=30)
    args=parser.parse_args()
    if not args.kinds or not set(args.kinds)<=set(VALIDATION_KINDS):parser.error('Unknown fixed road kind')
    if not set(args.modes)<= {'perception','no_centerline'}:parser.error('Unknown input mode')
    if not set(args.selectors)<= {'native','policy'}:parser.error('Unknown selector')
    if 'policy' in args.selectors and args.model is None:parser.error('Policy validation requires --model')
    if args.output.exists():raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    model=SpatialPathPolicy.load(args.model) if args.model is not None else None
    before_hash=hashlib.sha256(args.model.read_bytes()).hexdigest() if args.model is not None else None
    manifest_bytes=(DIRECTORY/'manifest.json').read_bytes()
    rows=[]
    for kind in args.kinds:
        for mode in args.modes:
            for selector in args.selectors:
                report=run_case(kind,model=model,selector=selector,mode=mode,start_mode=args.start_mode,
                    reference_scale=args.reference_scale,max_segments=args.max_segments)
                report['model_sha256']=before_hash
                name=report['episode_id'].replace(':','_')+'.json'
                write_json(args.output/'rollouts'/name,report)
                row={k:report[k] for k in ('episode_id','kind','variant','direction','selector','road_input_mode',
                    'road_completed','fault','initial_progress_m','final_progress_m','goal_progress_m','wall_s')}
                row.update(file='rollouts/'+name,steps=len(report['trace']),
                    accepted_segments=sum(s['accepted'] for s in report['segments']),
                    unknown_width_decisions=sum(s['input_diagnostics']['unknown_width_reference_scaled'] for s in report['segments']))
                rows.append(row)
                write_json(args.output/'summary.json',dict(complete=False,episodes=rows))
                print(json.dumps(dict(stage='case_complete',**row)),flush=True)
    if model is not None:assert before_hash==hashlib.sha256(args.model.read_bytes()).hexdigest()
    assert manifest_bytes==(DIRECTORY/'manifest.json').read_bytes()
    write_json(args.output/'summary.json',dict(complete=True,total=len(rows),
        completed=sum(r['road_completed'] for r in rows),episodes=rows,model_sha256=before_hash,
        fixture_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        planner_source_sha256=planner_source_sha256(),policy_encoder_sha256=policy_encoder_sha256(),
        reference_scale_override=args.reference_scale,start_mode=args.start_mode,
        fixture_sources_unchanged=True,model_unchanged=True,training_performed=False,
        policy_connected_to_main=False,width_prior=False))


if __name__=='__main__':main()
