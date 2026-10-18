"""Untimed single-path policy, learned geometry, partial roads and strict archives."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from mvp21_policy.candidates import validate_candidate
from offline.lattice_training.features import (encode_candidate, candidate_features, INPUT_GROUPS,
    ROAD_ROLES, OBSTACLE_SECTORS)
from offline.lattice_training.spatial import input_matrices
from offline.lattice_training.reference import trajectory_samples
from offline.lattice_training.objective import reference_plan, CStarTrackingSettings, CSTAR_TARGET
from offline.lattice_training.dataset import (import_dataset, load_dataset, fixed_execution_condition,
                                             append_training_dataset)
from offline.lattice_training.model import SpatialPathPolicy
from offline.lattice_training.pipeline import make_environment, make_planner, candidates_at
from tests.test_lattice_training import candidate, decision_records


class PolicyInputContract(unittest.TestCase):
    def test_inputs_are_three_structured_groups_and_one_candidate(self):
        obs=decision_records()[0]['observation']
        encoded=encode_candidate(obs,candidate())
        self.assertEqual(set(encoded), {'schema_version', *INPUT_GROUPS})
        self.assertIsInstance(encoded['current_road_and_obstacles']['road'], dict)
        self.assertNotIn('time_s', json.dumps(encoded))
        with self.assertRaisesRegex(ValueError,'Exactly one'):
            encode_candidate(obs,[candidate(),candidate(.1)])
        model=SpatialPathPolicy(hidden=4)
        with self.assertRaisesRegex(ValueError,'Use score'):
            model.predict(encoded)

    def test_scores_and_inputs_ignore_timing_planner_scores_history_and_labels(self):
        row=decision_records()[0];obs=row['observation'];proposal=row['candidates'][0]['candidate']
        expected=encode_candidate(obs,proposal);changed=deepcopy(proposal);other=deepcopy(obs)
        changed['descriptor']={'costs':{'center':1e9},'score':-1e9,'length_m':1e9}
        changed.update(minimum_clearance_m=-1e9,label={'return':1e9},previous_path=[[999,999]])
        for field in ('speed_profile','longitudinal_profile','target_speed_mps'):
            changed['reference'].pop(field,None)
        changed['reference'].update(valid_for_s=999,planned_stop=True)
        other.update(ground_truth={'x':1e9},controller_context={'previous_action':[1,1]},previous_path=[[999,999]])
        other['estimation']['road'].update(confidence=0.,recommended_speed_scale=0.)
        self.assertEqual(encode_candidate(other,changed),expected)
        model=SpatialPathPolicy(hidden=4)
        self.assertEqual(model.score(expected),model.score(encode_candidate(other,changed)))
        # Labels still require the explicitly timed C* contract.
        with self.assertRaises(KeyError):reference_plan(changed)

    def test_each_requested_group_changes_the_input(self):
        row=decision_records()[0];base=encode_candidate(row['observation'],candidate())
        obs=deepcopy(row['observation']);obs['estimation']['state']['speed_mps']+=.1
        self.assertNotEqual(base['current_state'],encode_candidate(obs,candidate())['current_state'])
        obs=deepcopy(row['observation']);obs['estimation']['road']['left_boundary_xy'][0]=(0.,.7)
        self.assertNotEqual(base['current_road_and_obstacles'],
            encode_candidate(obs,candidate())['current_road_and_obstacles'])
        self.assertNotEqual(base['candidate_path'],
            encode_candidate(row['observation'],candidate(.1))['candidate_path'])

    def test_native_curves_are_retained_without_resampling_or_flattening(self):
        obs=decision_records()[0]['observation']
        points=[(0.,0.),(1.,.4),(2.,1.)]
        obs['estimation']['road']['centerline_xy']=points
        result=encode_candidate(obs,candidate())
        self.assertEqual(result['current_road_and_obstacles']['road']['centerline']['points_xy'],
                         [list(p) for p in points])
        matrices=input_matrices(result)
        self.assertEqual(matrices['path'].shape[0],len(candidate()['reference']['frenet_path']['candidate']['samples'])-1)
        self.assertEqual(matrices['scene'].shape[0],4)  # two center + one left + one right segments

    def test_sectors_preserve_nearest_xy_and_are_order_independent(self):
        obs=decision_records()[0]['observation']
        obs['estimation']['obstacles']['points_xyz_m']=[[1.,.2,0.],[.5,.1,0.],[.5,-.3,0.],[-.1,0.,0.],[5.,0.,0.]]
        first=encode_candidate(obs,candidate())
        obs['estimation']['obstacles']['points_xyz_m'].reverse()
        self.assertEqual(first,encode_candidate(obs,candidate()))
        sectors=first['current_road_and_obstacles']['obstacles']['sectors_xy_present']
        self.assertEqual(len(sectors),OBSTACLE_SECTORS)
        self.assertIn([.5,.1,1.],sectors)
        self.assertIn([.5,-.3,1.],sectors)
        self.assertNotIn([1.,.2,1.],sectors)
        self.assertEqual(sum(row[2] for row in sectors),2)

    def test_unavailable_obstacles_and_missing_center_or_both_sides_can_be_scored(self):
        obs=decision_records()[0]['observation']
        model=SpatialPathPolicy(hidden=4)
        observed=encode_candidate(obs,candidate())
        obs['estimation']['obstacles']['available']=False
        self.assertNotEqual(observed,encode_candidate(obs,candidate()))
        for roles in (('centerline',),('centerline','left_boundary'),ROAD_ROLES):
            changed=deepcopy(obs)
            changed['estimation']['road']['valid']=False
            for role in roles:changed['estimation']['road'][role+'_xy']=None
            value=encode_candidate(changed,candidate())
            self.assertTrue(np.isfinite(model.score(value)))
            self.assertTrue(all(not value['current_road_and_obstacles']['road'][role]['points_xy'] for role in roles))

    def test_boundary_provenance_changes_the_observable_input(self):
        obs=decision_records()[0]['observation']
        obs['estimation']['road']['boundary_source']={'left':'observed','right':'inferred_width'}
        first=encode_candidate(obs,candidate())
        obs['estimation']['road']['boundary_source']['right']='observed'
        self.assertNotEqual(first,encode_candidate(obs,candidate()))

    def test_frame_and_spatial_arc_geometry_are_checked(self):
        base=candidate()['reference']['frenet_path']['candidate']
        for key,value in (('frame_id','map'),('vehicle_reference_point','rear_axle'),('sample_columns',['x','y'])):
            changed=deepcopy(base);changed[key]=value
            with self.assertRaises(ValueError):validate_candidate(changed)
        changed=deepcopy(base);changed['samples'][1][0]/=2
        with self.assertRaisesRegex(ValueError,'arc distance'):validate_candidate(changed)

    def test_current_export_and_cstar_timing_remain_original_but_policy_ignores_time(self):
        with make_environment('straight',0,max_steps=5000) as env:
            env.reset(seed=0);obs,*_=env.step((0,0));ref,diag=candidates_at(make_planner(),obs)
            self.assertTrue(ref['valid'])
            original={str(c['id']):c for c in diag['result']['candidates']}
            for proposal in diag['candidate_references']:
                self.assertEqual(proposal['reference']['frenet_path']['candidate']['geometry_xy_heading'],
                    original[proposal['candidate_id']]['path'])
            proposal=diag['candidate_references'][0];samples=trajectory_samples(proposal,16)
            self.assertAlmostEqual(samples[0,4],obs['estimation']['state']['speed_mps'])
            before=encode_candidate(obs,proposal)
            del proposal['reference']['longitudinal_profile'];del proposal['reference']['speed_profile']
            self.assertEqual(before,encode_candidate(obs,proposal))

    def test_native_adapter_does_not_require_a_valid_centerline(self):
        road=dict(valid=False,centerline_xy=[],left_boundary_xy=[(-.4,.5),(2.,.5)],
            right_boundary_xy=[],boundary_source=dict(left='observed',right='absent'),lane_width_m=1.)
        state=dict(timestamp_s=1.,frame_id='base_link',speed_mps=.1,yaw_rate_rps=0.,
                   speed_valid=True,yaw_rate_valid=True)
        ref,diag=make_planner().plan(road,state,{}, {},1.,collect_candidates=True)
        self.assertTrue(ref['valid'],diag)
        self.assertTrue(diag['candidate_references'])
        self.assertEqual(diag['result']['reference_origin'],'boundary_offset_inferred')
        obs=dict(estimation=dict(road=road,state=state,obstacles=dict(available=False)))
        self.assertTrue(np.isfinite(SpatialPathPolicy(hidden=4).score(encode_candidate(obs,diag['candidate_references'][0]))))

    def test_single_and_batched_independent_predictions_match(self):
        obs=decision_records()[0]['observation'];model=SpatialPathPolicy(hidden=4)
        first,second=encode_candidate(obs,candidate()),encode_candidate(obs,candidate(.1))
        self.assertEqual(model.score(first),model.predict([second,first])[1])

    def test_training_refuses_hidden_mixed_target_speed_conditions(self):
        records=decision_records()
        entry=records[0]['candidates'][1]
        proposal=entry['candidate'];reference=proposal['reference']
        length=reference['frenet_path']['candidate']['samples'][-1][0]
        duration=2*length/(.1+.2)
        reference['speed_profile']=[[0.,.1],[duration,.2]]
        reference['longitudinal_profile'].update(duration_s=duration,
            coeffs_low_to_high=[0.,.1*duration,.05*duration])
        from tests.test_lattice_training import execution
        entry['cstar_trials']=[execution(proposal,identity='noise-0'),execution(proposal,identity='noise-1')]
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'input.jsonl'
            source.write_text('\n'.join(json.dumps(r) for r in records),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'hidden target speeds'):
                import_dataset(source,root/'data')

    def test_offline_scoring_refuses_a_different_cruise_condition(self):
        model=SpatialPathPolicy(hidden=4)
        model.provenance['execution_condition']=fixed_execution_condition(candidate())
        with self.assertRaisesRegex(ValueError,'cruise-speed condition'):
            candidates_at(make_planner(),decision_records()[0]['observation'],model)

    def test_ragged_training_append_preserves_holdout_inputs_and_labels(self):
        base=decision_records();addition=deepcopy(base[:1])
        addition[0].update(group_id='added-decision',scene_id='added-scene')
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name,records in (('base',base),('addition',addition)):
                source=root/(name+'.jsonl')
                source.write_text('\n'.join(json.dumps(r) for r in records),encoding='utf-8')
                metadata=import_dataset(source,root/name,required_splits={'train'} if name=='addition' else None)
                metadata['collection_coverage']=dict(roads=[dict(scene_id=r['scene_id']) for r in records])
                (root/name/'metadata.json').write_text(json.dumps(metadata),encoding='utf-8')
            original,_=load_dataset(root/'base')
            metadata=append_training_dataset(root/'base',root/'addition',root/'merged')
            merged,_=load_dataset(root/'merged')
            self.assertEqual(metadata['rows'],12)
            for split in ('validation','test'):
                self.assertTrue(metadata['augmentation']['holdout_arrays_unchanged'][split])
                for key in original:
                    np.testing.assert_array_equal(original[key][original['splits']==split],
                                                  merged[key][merged['splits']==split])

    def test_encoder_gradients_match_finite_differences_including_empty_scene(self):
        for empty in (False,True):
            obs=deepcopy(decision_records()[0]['observation'])
            if empty:
                for role in ROAD_ROLES:obs['estimation']['road'][role+'_xy']=[]
            model=SpatialPathPolicy(hidden=3,seed=4)
            matrices=input_matrices(encode_candidate(obs,candidate(.1)))
            value,cache=model.network.forward(matrices)
            gradients=model.network.backward(cache,1.)
            for name,parameter in model.parameters.items():
                index=np.unravel_index(np.argmax(np.abs(gradients[name])),parameter.shape)
                old=parameter[index];epsilon=1e-5
                parameter[index]=old+epsilon;plus=model.network.forward(matrices)[0]
                parameter[index]=old-epsilon;minus=model.network.forward(matrices)[0]
                parameter[index]=old
                self.assertAlmostEqual((plus-minus)/(2*epsilon),gradients[name][index],delta=2e-7,
                                       msg=str((empty,name,index)))

    def test_old_formats_rejected_and_new_model_and_ragged_dataset_roundtrip(self):
        old=candidate();old['reference']['frenet_path']={'type':'FRENET','reference_curve':{},'lateral':{}}
        with self.assertRaisesRegex(ValueError,'Only current'):candidate_features(old)
        model=SpatialPathPolicy(hidden=4)
        model.provenance=dict(objective=CStarTrackingSettings().specification(),target=CSTAR_TARGET,
            training_target_transform=model.target_specification,
            execution_condition=fixed_execution_condition(candidate()))
        records=decision_records()
        for i,row in enumerate(records):row['planner_source_sha256']=str(i)*64
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);path=root/'model.json';model.save(path)
            record=json.loads(path.read_text());record['planner_source_sha256']='a'*64
            path.write_text(json.dumps(record));SpatialPathPolicy.load(path)
            record['format']='state_road_trajectory_mlp_v1';path.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError,'format/schema'):SpatialPathPolicy.load(path)
            source=root/'input.jsonl';source.write_text('\n'.join(json.dumps(r) for r in records),encoding='utf-8')
            meta=import_dataset(source,root/'data');data,_=load_dataset(root/'data')
            self.assertEqual(meta['producer_planner_sha256'],[str(i)*64 for i in range(3)])
            self.assertEqual(data['X'].shape,(9,))
            self.assertEqual(json.loads(data['X'][0])['candidate_path'],
                             encode_candidate(records[0]['observation'],records[0]['candidates'][0]['candidate'])['candidate_path'])
            meta['policy_encoder_sha256']='f'*64;(root/'data/metadata.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError,'encoder/schema'):load_dataset(root/'data')


if __name__=='__main__':unittest.main()
