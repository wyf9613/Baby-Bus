from copy import deepcopy
import unittest
import numpy as np
from offline.lattice_training.boundary_roads import cubic_curve,generate_boundary_case,geometry,boundary_geometry_audit
from offline.lattice_training.robust_roads import centerline_samples,cone_layout,physical_screen,body_road_margins
from offline.lattice_training.screen_boundaries import usable_candidate,system_pass


class BoundaryFirstRoads(unittest.TestCase):
    def test_clamped_cubic_preserves_lines_and_knots(self):
        p=np.array([[0.,0.],[1.,2.],[2.,4.],[3.,6.]])
        result=cubic_curve(p,301)
        np.testing.assert_allclose(result[::100],p,atol=1e-12)
        np.testing.assert_allclose(result[:,1],2*result[:,0],atol=1e-12)

    def test_boundary_controls_are_seeded_mirrored_and_not_a_shared_offset_curve(self):
        a=generate_boundary_case('u_left',4103,'test');b=generate_boundary_case('u_right',4103,'test')
        self.assertEqual(a,generate_boundary_case('u_left',4103,'test'))
        self.assertEqual(a['split_group'],b['split_group'])
        left=np.array(a['boundary_geometry']['left_xy']);right=np.array(a['boundary_geometry']['right_xy'])
        np.testing.assert_allclose(left*np.array([1.,-1.]),b['boundary_geometry']['right_xy'],atol=1e-8)
        tangent_l=np.gradient(left,axis=0);tangent_r=np.gradient(right,axis=0)
        tangent_l/=np.linalg.norm(tangent_l,axis=1)[:,None];tangent_r/=np.linalg.norm(tangent_r,axis=1)[:,None]
        self.assertGreater(np.linalg.norm(tangent_l-tangent_r,axis=1).max(),.2)
        self.assertEqual(a['generation']['construction_order'][0],'independent_left_and_right_controls')

    def test_u_turn_pool_has_reversed_width_timing(self):
        entries=[(generate_boundary_case('u_left',seed,'train'),seed) for seed in range(4100,4104)]
        measured={c['generation']['width_profile']:boundary_geometry_audit(c) for c,_ in entries}
        self.assertGreater(measured['wide_entry']['entry_clear_width_m'],measured['wide_entry']['exit_clear_width_m']+.3)
        self.assertGreater(measured['wide_exit']['exit_clear_width_m'],measured['wide_exit']['entry_clear_width_m']+.3)

    def test_physical_margins_use_original_boundaries_instead_of_native_host_lane(self):
        case=dict(boundary_sha256='unit-straight-original-boundaries',mission_road_length_m=4.,
            road_spec={'deliberately_not_used_as_physical_geometry':True},
            boundary_geometry=dict(reference_station_m=[0.,1.,4.],reference_xy=[[-1.,0.],[0.,0.],[3.,0.]],
                pair_progress_m=[0.,1.,4.],left_xy=[[-1.,.5],[0.,.5],[3.,.5]],right_xy=[[-1.,-.5],[0.,-.5],[3.,-.5]]))
        road=geometry(case);model=dict(front_m=.3,rear_m=.2,width_m=.25)
        np.testing.assert_allclose(body_road_margins(road,model,np.array([1.,0.]),0.,2.),[.375,.375],atol=1e-8)
        self.assertLess(min(body_road_margins(road,model,np.array([1.,.45]),0.,2.)),-.07)
        self.assertAlmostEqual(road.get_reference_lane_bounds_at_progress(2.)['left_bound_m'],.5)

    def test_sharper_compound_is_physically_admitted_without_relaxing_vehicle_limits(self):
        case=generate_boundary_case('compound_left',4103,'test')
        self.assertTrue(boundary_geometry_audit(case)['passed'])
        samples=centerline_samples(case);cones,_=cone_layout(case)
        result=physical_screen(case,cones,samples)
        self.assertTrue(result['centerline_passable'],result)
        self.assertGreater(result['max_required_steering_deg'],25.)
        self.assertLessEqual(result['max_required_steering_deg'],45.)

    def test_native_carrier_has_no_generated_offset_cones_on_sharp_curves(self):
        from dreamgym.envs import Road
        for seed in (4100,4103,4106,4107,4108):
            case=generate_boundary_case('compound_left',seed,'test')
            self.assertEqual(case['road_spec']['cone_paths'],{})
            self.assertTrue(all(e['active_cone_paths']=={} for e in case['road_spec']['elements']))
            Road(road_spec=case['road_spec'])

    def test_admission_ignores_boundary_gaps_and_rejects_missing_invalid_candidates(self):
        candidate=dict(path=[[0.,0.],[1.,0.]],collision=False)
        self.assertTrue(usable_candidate(dict(status='VALID',candidates=[candidate])))
        self.assertFalse(usable_candidate(dict(status='NO_ROAD',candidates=[candidate])))
        self.assertFalse(usable_candidate(dict(status='VALID',candidates=[])))
        invalid=deepcopy(candidate);invalid['path'][1][1]=float('nan')
        self.assertFalse(usable_candidate(dict(status='VALID',candidates=[invalid])))
        view=dict(physical=dict(centerline_passable=True),planner_jobs=3,valid_planner_jobs=3,
            valid_candidate_jobs=3,complete_boundary_coverage=False,exported_boundary_frames=0)
        self.assertTrue(system_pass(view))
        self.assertFalse(system_pass(dict(view,valid_candidate_jobs=2)))
        self.assertFalse(system_pass(dict(view,runtime_exception='ValueError')))
        self.assertFalse(system_pass(dict(view,warmup_planner_jobs=1)))


if __name__=='__main__':unittest.main()
