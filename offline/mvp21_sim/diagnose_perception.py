"""Read-only cone/road audit of recorded MVP runtime episodes.

Reconstruct the pinned episode's resolved cones, verify the sensor query against
the recorded packets, and retain acquisition-frame coordinates for visualization.
No planner, controller, estimator algorithm or recorded motion is changed.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np

from mvp21_policy.estimation import EstimationPipeline, EstimationSettings
from offline.lattice_training.dataset import write_json, json_safe
from .simulation import fixed_road_environment
from .validate_roads import read_rows, protected_hashes


def world_points(points, pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return [[pose[0]+c*p[0]-s*p[1],pose[1]+s*p[0]+c*p[1]] for p in points]


def body_points(points, pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return [[c*(p[0]-pose[0])+s*(p[1]-pose[1]),-s*(p[0]-pose[0])+c*(p[1]-pose[1])] for p in points]


def acquisition_pose(rows, summary, stamp_ns):
    """Use the pose at acquisition, never the later delivery pose."""
    lookup={round(r['timestamp_s']*1e9):r['raw_state_before'][:3] for r in rows}
    if stamp_ns in lookup:
        return list(lookup[stamp_ns]),'recorded_acquisition_pose'
    if stamp_ns < round(summary['initial_timestamp_s']*1e9) and (summary.get('validation_config') or summary['initial_timestamp_s']-stamp_ns/1e9 < .2):
        initial=summary['initial_raw_state']
        if max(abs(v) for v in initial[3:]) < 1e-12:
            return list(initial[:3]),'stationary_warmup_pose'
    raise ValueError('Missing acquisition pose: '+str(stamp_ns))


def query_cones(env, pose, maximum):
    mount=env._backend._cone_sensor_mount
    sensor=world_points([mount[:2]],pose)[0]
    cfg=env.config.observation_config['cone_detections']['acquisition']
    return env._backend.road.cone_info_for_resolved_cones_at_given_pose_and_fov(
        env._backend._resolved_cones,px=sensor[0],py=sensor[1],theta=pose[2]+mount[2],
        fov_horizontal_degrees=cfg['horizontal_field_of_view_deg'],
        body_x_upper_bound=cfg['forward_position_upper_bound_m'],max_results=maximum)


def summarize_rows(rows):
    return dict(frames=len(rows),
        packet_query_max_error_m=max((r['query_error_m'] for r in rows),default=0.),
        truth_match_max_error_m=max((r['truth_match_error_m'] for r in rows),default=0.),
        sensor_capacity_frames=sum(r['visible_without_cap']>r['detected_count'] for r in rows),
        opposite_side_omitted_by_capacity_frames=sum(bool(r['capacity_omitted_sides']) for r in rows),
        both_chains_fit_but_no_raw_corridor_frames=sum(r['both_fit_no_corridor'] for r in rows),
        both_chains_fit_but_no_exported_boundaries_frames=sum(r['both_fit_no_export'] for r in rows),
        current_frame_statuses=dict(Counter(r['current_status'] for r in rows)),
        exported_statuses=dict(Counter(r['export_status'] for r in rows)))


def diagnose_case(directory):
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    case=json.loads((directory/'road.json').read_text(encoding='utf-8'))
    rows=read_rows(directory/'trace.jsonl')
    by_stamp={round(r['timestamp_s']*1e9):r['raw_state_before'][:3] for r in rows}
    recorded=summary['environment']
    initial_progress=recorded['initial_state_config']['pose']['progress']['lower']
    seed=case.get('runtime_seed',713)
    profile=summary.get('validation_config')
    options=dict(physics_hz=profile['frequencies_hz']['physics'],perception_hz=profile['frequencies_hz']['perception'],
        terminal_support_m=profile['terminal_support_m']) if profile else {}
    delay=round(profile['camera_latency_s']*profile['frequencies_hz']['physics']) if profile else 1
    with fixed_road_environment(case,seed=seed,progress=initial_progress,latency_steps=delay,**options) as env:
        actual=env.configuration()
        for key in ('road_spec','observation_config','bicycle_model_config','initial_state_config','integration_config'):
            if recorded[key]!=actual[key]:
                raise ValueError('Reconstructed environment differs: '+key)
        if env._backend._camera_pan_config['enabled']:
            raise ValueError('This audit expects the recorded fixed camera')
        records=env._backend._resolved_cones.records
        cones=[dict(id=i,xy=list(r.position_world_m),colour=r.color_id+1,radius_m=r.radius_m,
            path=r.path_name,progress_m=r.reference_progress_m) for i,r in enumerate(records)]
        truth=np.asarray([r['xy'] for r in cones]); colours=np.asarray([r['colour'] for r in cones])
        raw_estimator=EstimationPipeline(EstimationSettings(**recorded['estimation_settings']),'base_link')
        result=[]
        for index,row in enumerate(rows):
            obs=row['observation']; stamp=obs['sensor_stamp_ns']['cone_detections']
            pose=by_stamp.get(stamp)
            basis='recorded_acquisition_pose'
            if pose is None:pose,basis=acquisition_pose(rows,summary,stamp)
            batch=obs['sensors']['cone_detections']; detections=batch['detections']
            world=world_points(detections,pose)
            query=query_cones(env,pose,recorded['observation_config']['cone_detections']['maximum_detection_count'])
            unlimited=query_cones(env,pose,None)
            expected=np.column_stack((query['px_in_body_frame'],query['py_in_body_frame']))
            measured=np.asarray([p[:2] for p in detections])
            if len(expected)!=len(measured):raise ValueError('Reconstructed camera packet count differs')
            query_error=float(np.max(np.abs(expected-measured))) if len(measured) else 0.
            if query_error>3e-5 or list(np.asarray(query['color'])+1)!=[p[3] for p in detections]:
                raise ValueError('Reconstructed camera packet differs')
            matched=[]; residuals=[]
            for p,raw in zip(world,detections):
                distance=np.linalg.norm(truth-np.asarray(p),axis=1)
                distance[colours!=raw[3]]=np.inf
                identifier=int(np.argmin(distance));matched.append(identifier);residuals.append(float(distance[identifier]))
            raw=raw_estimator.road(batch,stamp,obs['sensor_age_s']['cone_detections'])
            chains={side:[p[:2] for p in (raw.get('_boundary_fits',{}).get(colour) or {}).get('support_points',[])]
                for side,colour in (('left',2),('right',1))}
            road=row.get('perception_road',obs['estimation']['road']);current=road.get('current_frame') or {}
            output=road if any(road.get(k) for k in ('left_boundary_xy','right_boundary_xy','centerline_xy')) else road.get('forward_geometry') or road
            diag=current.get('boundary_diagnostics') or road.get('boundary_diagnostics') or (row.get('raw_estimator_road')or{}).get('boundary_diagnostics') or raw['boundary_diagnostics']
            both_fit=all((diag.get(side) or {}).get('fit_valid') for side in ('left','right'))
            unbounded_counts={side:int(np.sum(np.asarray(unlimited['color'])+1==colour)) for side,colour in (('left',2),('right',1))}
            detected_counts={side:sum(p[3]==colour for p in detections) for side,colour in (('left',2),('right',1))}
            omitted=[s for s in ('left','right') if unbounded_counts[s]>=2 and detected_counts[s]<2]
            result.append(dict(index=index,timestamp_s=row['timestamp_s'],acquisition_timestamp_s=stamp/1e9,
                acquisition_pose=pose,acquisition_pose_basis=basis,detected_raw=detections,
                detected_world=world,detected_current=body_points(world,row['raw_state_before']),matched_cone_ids=matched,
                query_error_m=query_error,truth_match_error_m=max(residuals,default=0.),
                detected_count=len(detections),visible_without_cap=int(unlimited['num_cones']),
                detected_counts=detected_counts,visible_without_cap_counts=unbounded_counts,
                capacity_omitted_sides=omitted,
                raw_chains_world={side:world_points(points,pose) for side,points in chains.items()},
                raw_chains_current={side:body_points(world_points(points,pose),row['raw_state_before']) for side,points in chains.items()},
                boundary_diagnostics=diag,current_status=current.get('status'),raw_stateless_status=raw['status'],
                export_status=output.get('status'),
                both_fit_no_corridor=bool(both_fit and not current.get('fit_valid')),
                both_fit_no_export=bool(both_fit and not any(output.get(k) for k in ('left_boundary_xy','right_boundary_xy')))))
        if max(r['truth_match_error_m'] for r in result)>3e-5:raise ValueError('Truth matching failed')
        metadata=dict(format='runtime_perception_diagnosis_v1',cones=cones,
            truth_source=f'resolved from identical pinned episode configuration and seed {seed}; verified against every recorded sensor packet',
            camera_mount=list(env._backend._cone_sensor_mount),camera=recorded['observation_config']['cone_detections'],
            raw_chain_note='Fresh-frame chain audit using the actual estimator; no width/history injected. Final estimator output remains the original recorded output.',
            summary=summarize_rows(result))
    write_json(directory/'perception-cones.json',metadata)
    with (directory/'perception-frames.jsonl').open('w',encoding='utf-8') as stream:
        for row in result:stream.write(json.dumps(json_safe(row),allow_nan=False)+'\n')
    return metadata['summary']


def orientation_reproduction():
    # Both boundaries and all ten cones are available. x is constant on each
    # boundary; swapping chain travel direction should not erase the boundary.
    settings=EstimationSettings(left_colour=2,right_colour=1,road_geometry='ordered_xy',
        road_forward_max_m=4.,known_lane_width_m=0.,single_side_from_learned_width=True)
    batch=dict(detections=[(x,y,0.,colour,1.) for x,colour in ((1.,2),(2.,1)) for y in (-.7,-.35,0.,.35,.7)])
    estimator=EstimationPipeline(settings,'base_link');result=estimator.road(batch,1_000_000_000,0.)
    fits=result['_boundary_fits']
    # A local counterfactual only: reverse both chain directions, keeping all
    # points, settings and pairing equations. No source/module file is edited.
    from unittest.mock import patch
    from mvp21_policy.estimation import _estimation_order_boundary
    def reverse_chain(points,config):
        fit=_estimation_order_boundary(points,config)
        if fit is not None:
            fit=deepcopy(fit);length=fit['lengths'][-1]
            fit['support_points'].reverse()
            fit['lengths']=[length-s for s in reversed(fit['lengths'])]
        return fit
    with patch('mvp21_policy.estimation._estimation_order_boundary',side_effect=reverse_chain):
        reversed_result=EstimationPipeline(settings,'base_link').road(batch,1_000_000_000,0.)
    return dict(input=batch,settings=vars(settings),status=result['status'],valid=result['valid'],
        boundary_diagnostics=result['boundary_diagnostics'],
        chains={str(colour):fit['support_points'] for colour,fit in fits.items()},
        exported_left=result['left_boundary_xy'],exported_right=result['right_boundary_xy'],
        counterfactual_reversing_both_chains=dict(status=reversed_result['status'],valid=reversed_result['valid'],
            centerline_xy=reversed_result['centerline_xy'],lane_width_m=reversed_result['lane_width_m']))


def input_hashes(root):
    return {p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for pattern in ('*/trace.jsonl','*/shadow.jsonl','*/summary.json','*/road.json','*/failure.json','suite.json')
        for p in root.glob(pattern)}


def annotate_branches(root):
    """Truth route positions are display-only evidence, never estimator input."""
    from offline.lattice_training.roads import road_length
    for directory in sorted(Path(root).glob('*')):
        if not (directory/'perception-cones.json').exists():continue
        meta=json.loads((directory/'perception-cones.json').read_text(encoding='utf-8'))
        case=json.loads((directory/'road.json').read_text(encoding='utf-8'))
        ends=[];distance=0.
        for element in case['road_spec']['elements']:
            distance+=road_length(dict(elements=[element]));ends.append(distance)
        for cone in meta['cones']:
            progress=cone['progress_m'];segment=next((i for i,end in enumerate(ends) if progress<=end+1e-8),len(ends))
            terminal=progress>case['mission_road_length_m']+1e-8
            if terminal:label='terminal_support'
            elif case['variant']=='half_width_double':
                label=('entry_straight','first_uturn','return_straight','second_uturn','final_straight')[min(segment,4)]
            else:label='road_segment_'+str(segment+1)
            cone.update(route_segment=segment,route_segment_label=label,terminal_support=terminal)
        rows=read_rows(directory/'perception-frames.jsonl')
        for row in rows:
            ranges={}
            for side,colour in (('left',2),('right',1)):
                selected=[meta['cones'][identifier] for identifier,raw in zip(row['matched_cone_ids'],row['detected_raw']) if raw[3]==colour]
                ranges[side]=dict(minimum=min((q['progress_m'] for q in selected),default=None),
                    maximum=max((q['progress_m'] for q in selected),default=None),
                    segments=sorted(set(q['route_segment_label'] for q in selected)),
                    terminal_support_count=sum(q['terminal_support'] for q in selected))
            row['detected_route_ranges']=ranges
        write_json(directory/'perception-cones.json',meta)
        with (directory/'perception-frames.jsonl').open('w',encoding='utf-8') as stream:
            for row in rows:stream.write(json.dumps(json_safe(row),allow_nan=False)+'\n')


def run(root):
    root=Path(root);before=input_hashes(root);protected=protected_hashes()
    suite=json.loads((root/'suite.json').read_text(encoding='utf-8'))
    for file in ('viewer.html','viewer-screenshot.png'):
        original=root/file;backup=root/(original.stem+'-before-perception'+original.suffix)
        if original.exists() and not backup.exists():shutil.copyfile(original,backup)
    reports={}
    for entry in suite['cases']:
        reports[entry['name']]=diagnose_case(root/entry['name'])
        print(entry['name'],json.dumps(reports[entry['name']]),flush=True)
    write_json(root/'perception-orientation-reproduction.json',orientation_reproduction())
    result=dict(cases=reports,input_sha256=before,recorded_inputs_unchanged=before==input_hashes(root),
        owning_modules_parameters_model_unchanged=protected==protected_hashes())
    write_json(root/'perception-diagnosis.json',result)
    if not result['recorded_inputs_unchanged'] or not result['owning_modules_parameters_model_unchanged']:
        raise AssertionError('Read-only diagnosis changed runtime inputs or protected code')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True)
    run(parser.parse_args().input)
