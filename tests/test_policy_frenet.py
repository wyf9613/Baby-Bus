"""Three-group scorer inputs and independence from planner scores/history."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from mvp21_policy.candidates import validate_candidate
from offline.lattice_training.features import encode_candidate,candidate_features,FEATURE_NAMES,INPUT_GROUPS,OBSTACLE_POINTS
from offline.lattice_training.reference import trajectory_samples
from offline.lattice_training.objective import reference_plan
from offline.lattice_training.dataset import import_dataset,load_dataset
from offline.lattice_training.model import ReturnMLP
from offline.lattice_training.pipeline import make_environment,make_planner,candidates_at
from tests.test_lattice_training import candidate,decision_records


class PolicyInputContract(unittest.TestCase):
    def test_inputs_have_exactly_the_three_requested_groups(self):
        self.assertEqual(tuple(INPUT_GROUPS),('current_state','current_road_and_obstacles','candidate_trajectory'))
        names=' '.join(FEATURE_NAMES)
        for word in ('cost','clearance','consistency','previous','reward','confidence','recommended','sector'):
            self.assertNotIn(word,names)

    def test_planner_costs_history_and_labels_cannot_change_prediction(self):
        row=decision_records()[0];obs=row['observation'];proposal=row['candidates'][0]['candidate']
        expected=encode_candidate(obs,proposal);changed=deepcopy(proposal);other=deepcopy(obs)
        changed['descriptor']={'costs':{'center':1e9},'score':-1e9,'length_m':1e9}
        changed.update(minimum_clearance_m=-1e9,label={'return':1e9},previous_path=[[999,999]])
        changed['reference'].update(valid_for_s=999,planned_stop=True)
        other.update(ground_truth={'x':1e9},controller_context={'previous_action':[1,1]},previous_path=[[999,999]])
        other['estimation']['road'].update(confidence=0.,recommended_speed_scale=0.)
        np.testing.assert_array_equal(encode_candidate(other,changed),expected)
        model=ReturnMLP(hidden=4)
        np.testing.assert_array_equal(model.predict(expected[None,:]),model.predict(encode_candidate(other,changed)[None,:]))

    def test_all_three_raw_groups_affect_the_input(self):
        row=decision_records()[0];base=encode_candidate(row['observation'],candidate())
        obs=deepcopy(row['observation']);obs['estimation']['state']['speed_mps']+=.1
        a,b=INPUT_GROUPS['current_state'];self.assertFalse(np.array_equal(base[a:b],encode_candidate(obs,candidate())[a:b]))
        obs=deepcopy(row['observation']);obs['estimation']['road']['left_boundary_xy'][0]=(0.,.7)
        a,b=INPUT_GROUPS['current_road_and_obstacles'];self.assertFalse(np.array_equal(base[a:b],encode_candidate(obs,candidate())[a:b]))
        obs=deepcopy(row['observation']);obs['estimation']['obstacles']['points_xyz_m']=[[1.,.2,0.]]
        self.assertFalse(np.array_equal(base[a:b],encode_candidate(obs,candidate())[a:b]))
        a,b=INPUT_GROUPS['candidate_trajectory'];self.assertFalse(np.array_equal(base[a:b],encode_candidate(row['observation'],candidate(.1))[a:b]))

    def test_raw_road_shape_is_retained_and_obstacle_order_is_canonical(self):
        row=decision_records()[0];obs=row['observation']
        obs['estimation']['road']['centerline_xy']=[(0.,0.),(1.,.4),(2.,1.)]
        features=encode_candidate(obs,candidate())
        self.assertGreater(features[FEATURE_NAMES.index('centerline_15_y_m')],.9)
        obs['estimation']['obstacles']['points_xyz_m']=[[1.,.2,0.],[.5,-.3,0.]]
        first=encode_candidate(obs,candidate());obs['estimation']['obstacles']['points_xyz_m'].reverse()
        np.testing.assert_array_equal(first,encode_candidate(obs,candidate()))
        obs['estimation']['obstacles']['points_xyz_m']=[[1.,0.,0.]]*(OBSTACLE_POINTS+1)
        with self.assertRaisesRegex(ValueError,'silent truncation'):encode_candidate(obs,candidate())

    def test_missing_obstacles_are_distinct_from_an_empty_observation(self):
        obs=decision_records()[0]['observation'];observed=encode_candidate(obs,candidate())
        obs['estimation']['obstacles']['available']=False
        unavailable=encode_candidate(obs,candidate())
        index=FEATURE_NAMES.index('obstacles_available')
        self.assertEqual((observed[index],unavailable[index]),(1.,0.))

    def test_frame_units_and_arc_distance_are_checked(self):
        base=candidate()['reference']['frenet_path']['candidate']
        for key,value in (('frame_id','map'),('vehicle_reference_point','rear_axle'),('sample_columns',['x','y'])):
            changed=deepcopy(base);changed[key]=value
            with self.assertRaises(ValueError):validate_candidate(changed)
        changed=deepcopy(base);changed['samples'][1][0]/=2
        with self.assertRaisesRegex(ValueError,'arc distance'):validate_candidate(changed)

    def test_current_native_export_is_unchanged_geometry_and_explicit_timing(self):
        with make_environment('straight',0,max_steps=5000) as env:
            env.reset(seed=0);obs,*_=env.step((0,0));ref,diag=candidates_at(make_planner(),obs)
            self.assertTrue(ref['valid']);original={str(c['id']):c for c in diag['result']['candidates']}
            for proposal in diag['candidate_references']:
                self.assertEqual(proposal['reference']['frenet_path']['candidate']['geometry_xy_heading'],original[proposal['candidate_id']]['path'])
                self.assertNotIn('costs',proposal['descriptor']);self.assertNotIn('minimum_clearance_m',proposal)
            proposal=diag['candidate_references'][0];samples=trajectory_samples(proposal,16)
            self.assertAlmostEqual(samples[0,4],obs['estimation']['state']['speed_mps'])
            self.assertEqual(samples.shape,(16,7));self.assertAlmostEqual(samples[-1,0],proposal['reference']['longitudinal_profile']['duration_s'])
            self.assertFalse(hasattr(make_planner(),'previous'))

    def test_old_candidate_and_model_formats_are_rejected(self):
        old=candidate();old['reference']['frenet_path']={'type':'FRENET','reference_curve':{},'lateral':{}}
        with self.assertRaisesRegex(ValueError,'Only current'):candidate_features(old)
        with self.assertRaisesRegex(ValueError,'Only current'):reference_plan(old)
        partial=candidate();partial['reference']['longitudinal_profile']['coeffs_low_to_high']=[0.,.3]
        with self.assertRaisesRegex(ValueError,'full path'):candidate_features(partial)
        with self.assertRaisesRegex(ValueError,'full path'):reference_plan(partial)
        model=ReturnMLP(hidden=4)
        from offline.lattice_training.objective import CStarTrackingSettings,CSTAR_TARGET
        model.provenance=dict(objective=CStarTrackingSettings().specification(),target=CSTAR_TARGET,
            training_target_transform=model.target_specification)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.json';model.save(path);record=json.loads(path.read_text())
            record['planner_source_sha256']='a'*64;path.write_text(json.dumps(record));ReturnMLP.load(path)
            record['format']='lattice_return_mlp_v3';path.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError,'format/schema'):ReturnMLP.load(path)

    def test_dataset_schema_is_new_and_preserves_producer_provenance(self):
        records=decision_records()
        for i,row in enumerate(records):row['planner_source_sha256']=str(i)*64
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'input.jsonl'
            source.write_text('\n'.join(json.dumps(r) for r in records),encoding='utf-8')
            meta=import_dataset(source,root/'data');data,_=load_dataset(root/'data')
            self.assertEqual(meta['producer_planner_sha256'],[str(i)*64 for i in range(3)])
            self.assertEqual(data['X'].shape,(9,len(FEATURE_NAMES)))
            meta['policy_encoder_sha256']='f'*64;(root/'data/metadata.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError,'encoder/schema'):load_dataset(root/'data')


if __name__=='__main__':unittest.main()
