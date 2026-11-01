import unittest
import math
import numpy as np
from offline.lattice_training.robust_roads import (
    FAMILIES,generate_case,cone_layout,centerline_samples,physical_screen,vehicle_model)


class ShortRoadGeometry(unittest.TestCase):
    def test_failure_intervals_keep_planner_sample_times_and_reason_changes(self):
        from offline.lattice_training.screen_roads_failures import failure_intervals
        frames=[dict(time_s=i*.05,station_m=i*.025,pose=[i*.025,0.,0.]) for i in range(61)]
        events=failure_intervals(frames,[0,20,40,60],'planner',{20:'NO_ROAD',40:'NO_ROAD'})
        self.assertEqual(len(events),1);self.assertEqual(events[0]['sample_frames'],[20,40])
        self.assertEqual(events[0]['start_frame'],20);self.assertEqual(events[0]['end_frame'],40)
        self.assertEqual(events[0]['sampling'],'actual_1hz_jobs')
        changed=failure_intervals(frames,[0,20,40,60],'planner',{20:'NO_ROAD',40:'ALL_COLLIDE'})
        self.assertEqual(len(changed),2)
        dense=failure_intervals(frames,list(range(6)),'perception',{1:'empty',2:'empty',4:'empty'})
        self.assertEqual([e['sample_frames'] for e in dense],[[1,2],[4]])

    def test_extra_cone_removal_preserves_all_boundary_records_and_noise_identities(self):
        from offline.lattice_training.screen_roads import filter_extra_cones,paired_cone_noise
        case=generate_case('s_left',3203,'test');cones,_=cone_layout(case,irregular=True)
        full=filter_extra_cones(cones);removed=filter_extra_cones(cones,True)
        self.assertEqual(removed,[c for c in full if c['role']!='adjacent_road'])
        self.assertGreater(len(full),len(removed))
        self.assertEqual([c for c in removed if c['role']=='boundary'],[c for c in full if c['role']=='boundary'])
        state={};filtered_state={}
        for tick in range(10):
            for c in full:
                key=c['source_id'];radius=math.hypot(c['x_m'],c['y_m'])
                value=paired_cone_noise(3203,tick,key,radius,.03,state.get(key,0.))
                state[key]=value[0]
                if c['role']!='adjacent_road':
                    other=paired_cone_noise(3203,tick,key,radius,.03,filtered_state.get(key,0.))
                    filtered_state[key]=other[0];self.assertEqual(value,other)

    def test_each_seed_group_covers_width_regimes_without_split_severity_bias(self):
        for seed,split in ((3200,'train'),(3201,'train'),(3202,'validation'),(3203,'test')):
            cases=[generate_case(family,seed,split) for family in FAMILIES]
            self.assertEqual({c['generation']['width_regime'] for c in cases},{0,1,2})
            self.assertEqual({c['split_group'] for c in cases},{f'robust_seed_{seed}'})

    def test_seeded_geometry_preserves_variable_width_contract(self):
        for family in FAMILIES:
            case=generate_case(family,3200,'train')
            self.assertEqual(case,generate_case(family,3200,'train'))
            previous=None
            for element in case['road_spec']['elements']:
                width=element['lanes']['reference_lane']['width_m']
                for v in width.values():self.assertTrue(.5-1e-9<=v-.08<=1.5+1e-9)
                if previous is not None:self.assertAlmostEqual(previous,width['start'])
                previous=width['end']
            self.assertEqual(case['width_convention'],'cone_inner_edge_clear_width')
            if case['generation']['width_regime']==0:
                self.assertEqual(case['generation']['width_controls_m'],[1.,.5,1.5,.8])

    def test_fixed_outward_layout_is_dense_after_random_omissions(self):
        case=generate_case('u_left',3203,'test')
        cones,meta=cone_layout(case,irregular=True)
        from dreamgym.envs import Road
        from offline.lattice_training.pipeline import with_terminal_support
        road=Road(road_spec=with_terminal_support(case['road_spec'],2.))
        for side in ('left','right'):
            records=[c for c in cones if c['side']==side]
            self.assertEqual(records[0]['station_m'],0.)
            self.assertAlmostEqual(records[-1]['station_m'],case['mission_road_length_m']+2.)
            for a,b in zip(records,records[1:]):
                self.assertLessEqual(math.hypot(a['x_m']-b['x_m'],a['y_m']-b['y_m']),.5+1e-9)
            for c in records:
                _,heading=road.convert_progression_to_coordinates(np.array([c['station_m']]))
                normal=np.array([-math.sin(heading[0]),math.cos(heading[0])])*(1 if side=='left' else -1)
                delta=np.array([c['x_m']-c['nominal_x_m'],c['y_m']-c['nominal_y_m']])
                self.assertGreaterEqual(np.dot(delta,normal),-1e-9)
                self.assertLessEqual(np.linalg.norm(delta),.08+1e-9)
        self.assertLessEqual(meta['maximum_actual_cone_gap_m'],.5+1e-9)
        self.assertGreater(meta['gap_fill_cones'],0)

    def test_frame_errors_are_gaussian_independent_and_do_not_move_physical_cones(self):
        from offline.lattice_training.screen_roads import paired_cone_noise
        values=np.array([paired_cone_noise(3200,tick,7,2.,.1,100.)[0] for tick in range(4000)])
        self.assertLess(abs(values.mean()),.006)
        self.assertAlmostEqual(values.std(),.1,delta=.004)
        self.assertLess(abs(np.corrcoef(values[:-1],values[1:])[0,1]),.05)
        self.assertEqual(paired_cone_noise(3200,5,7,2.,.1,0.),paired_cone_noise(3200,5,7,2.,.1,100.))

    def test_boundary_arc_spacing_does_not_crush_cones_on_tight_inner_bends(self):
        for family,seed in (('compound_left',3200),('s_left',3202),('u_right',3203)):
            case=generate_case(family,seed,'test')
            for irregular in (False,True):
                cones,meta=cone_layout(case,irregular=irregular)
                points=np.array([[c['x_m'],c['y_m']] for c in cones if c['role']!='adjacent_road'])
                distances=np.linalg.norm(points[:,None]-points[None,:],axis=2)
                np.fill_diagonal(distances,np.inf)
                self.assertGreaterEqual(float(distances.min()),.08-1e-9)
                self.assertLessEqual(meta['maximum_nominal_boundary_gap_m'],.5+1e-9)

    def test_adjusted_reference_is_kinematic_and_improves_full_body_clearance(self):
        case=generate_case('u_left',3203,'test');samples=centerline_samples(case)
        result=physical_screen(case,cone_layout(case)[0],samples)
        self.assertTrue(result['centerline_passable'],result)
        self.assertGreater(samples['maximum_reference_shift_m'],.001)
        self.assertGreaterEqual(result['minimum_body_road_clearance_m'],samples['baseline_minimum_body_road_clearance_m']-1e-8)
        velocity=np.gradient(samples['xy'],samples['dt_s'],axis=0)
        velocity_direction=np.unwrap(np.arctan2(velocity[:,1],velocity[:,0]))
        np.testing.assert_allclose(velocity_direction[1:-1],(samples['heading']+samples['beta'])[1:-1],atol=1e-8)
        np.testing.assert_allclose(np.linalg.norm(velocity,axis=1)[1:-1],.5,atol=.003)

    def test_mirrors_share_holdout_group_and_true_motion_is_model_bounded(self):
        for seed in (3200,3201,3202,3203):
            a=generate_case('s_left',seed,'test');b=generate_case('s_right',seed,'test')
            self.assertEqual(a['split_group'],b['split_group'])
            left,right=centerline_samples(a),centerline_samples(b)
            np.testing.assert_allclose(left['xy'][:,0],right['xy'][:,0],atol=1e-8)
            np.testing.assert_allclose(left['xy'][:,1],-right['xy'][:,1],atol=1e-8)
            self.assertLessEqual(np.max(abs(left['steering'])),vehicle_model()['steering_limit_rad'])
            self.assertAlmostEqual(left['dt_s'],.05)

    def test_irregular_views_freeze_physical_cones_and_keep_terminal_support_separate(self):
        case=generate_case('u_left',3201,'validation')
        a,meta=cone_layout(case,irregular=True);b,_=cone_layout(case,irregular=True)
        self.assertEqual(a,b);self.assertGreater(meta['removed_boundary_cones'],0)
        self.assertTrue(any(c['role']=='adjacent_road' for c in a))
        self.assertTrue(any(c['role']=='terminal_support' and c['station_m']>case['mission_road_length_m'] for c in a))
        self.assertEqual(case['terminal_sensor_support_m'],2.)
        result=physical_screen(case,a,centerline_samples(case))
        self.assertEqual(result['centerline_passable'],not result['reasons'])


if __name__=='__main__':unittest.main()
