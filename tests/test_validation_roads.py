"""Fixed-road geometry and the scope of the unknown-width reference adjustment."""
from copy import deepcopy
import math
import unittest
import numpy as np
from dreamgym.envs import Road
from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings
from offline.lattice_training.validation_roads import VALIDATION_KINDS, VARIANTS, validation_case
from offline.lattice_training.pipeline import make_environment, PipelineSettings
from offline.lattice_training.roads import road_length, road_identity


class UnknownWidthReferenceChecks(unittest.TestCase):
    def run_geometry(self, road, scale):
        config=ShadowSettings(sample_count=8, fusion_enabled=False, unknown_width_reference_scale=scale)
        return ShadowLatticePlanner(config).plan(dict(road=deepcopy(road),state=dict(speed_mps=.2),preview_m=.35))

    def test_only_unknown_width_single_boundary_is_scaled_symmetrically(self):
        for side, sign in [('left',1),('right',-1)]:
            road=dict(centerline_xy=[],lane_width_m=None,boundary_source={side:'observed'},
                      **{side+'_boundary_xy':[(0,sign),(3,sign)]})
            before=self.run_geometry(road,1.)
            after=self.run_geometry(road,4/3)
            self.assertTrue(after['unknown_width_reference_scaled'])
            self.assertAlmostEqual(before['reference_boundary_offset_m'],.45)
            self.assertAlmostEqual(after['reference_boundary_offset_m'],.60)
            np.testing.assert_allclose(np.asarray(after['reference_chart_xy'])[:,1],sign*.4)
            self.assertEqual(after['shadow_settings']['search_half_width_m'],.45)
            self.assertEqual(after['known_boundary_sides'],[side])
            self.assertEqual(after['planning_input']['road'],road)
            known=dict(road,lane_width_m=1.4)
            a,b=self.run_geometry(known,1.),self.run_geometry(known,4/3)
            self.assertFalse(b['unknown_width_reference_scaled'])
            self.assertAlmostEqual(b['reference_boundary_offset_m'],.7)
            self.assertEqual(a['reference_chart_xy'],b['reference_chart_xy'])
            self.assertEqual(a['candidates'],b['candidates'])

    def test_supplied_centerline_and_two_sides_are_unchanged(self):
        road=dict(centerline_xy=[],lane_width_m=None,left_boundary_xy=[(0,1),(3,1)],
            right_boundary_xy=[(0,-1),(3,-1)],boundary_source=dict(left='observed',right='observed'))
        for center in ([],[(0,0),(3,0)]):
            road['centerline_xy']=center
            a,b=self.run_geometry(road,1.),self.run_geometry(road,4/3)
            self.assertFalse(b['unknown_width_reference_scaled'])
            self.assertEqual(a['reference_chart_xy'],b['reference_chart_xy'])
            self.assertEqual(a['candidates'],b['candidates'])

    def test_invalid_reference_scale_is_rejected(self):
        for value in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):ShadowSettings(unknown_width_reference_scale=value)


class FixedValidationRoadChecks(unittest.TestCase):
    def test_fixtures_are_concrete_independent_and_match_their_road_identity(self):
        self.assertEqual(len(VALIDATION_KINDS),8)
        for kind in VALIDATION_KINDS:
            case=validation_case(kind)
            self.assertEqual(road_identity(case['road_spec']),case['physical_road_id'])
            self.assertAlmostEqual(road_length(case['road_spec']),case['mission_road_length_m'])
            self.assertFalse(case['scenario_width_prior_default'])
            case['road_spec']['elements'].clear()
            self.assertTrue(validation_case(kind)['road_spec']['elements'])

    def test_half_width_also_halves_turn_radius_and_retains_straight_lengths(self):
        for side in ('left','right'):
            base=validation_case('validation_uturn_original_'+side)
            narrow=validation_case('validation_uturn_half_width_'+side)
            self.assertAlmostEqual(narrow['full_width_m'],base['full_width_m']/2)
            for a,b in zip(base['road_spec']['elements'],narrow['road_spec']['elements']):
                if a['geometry']['type']=='straight':self.assertEqual(a['geometry'],b['geometry'])
                else:
                    self.assertEqual(a['geometry']['sweep_angle_deg'],b['geometry']['sweep_angle_deg'])
                    self.assertAlmostEqual(b['geometry']['curvature_per_m'],2*a['geometry']['curvature_per_m'])
            self.assertEqual(narrow['reference_radius_scale'],.5)

    def test_curved_turn_has_changed_radii_and_curved_entry_exit(self):
        case=validation_case('validation_uturn_curved_left')
        elements=case['road_spec']['elements'];turn=case['main_turns'][0]
        arcs=elements[turn['first_element']:turn['last_element']+1]
        self.assertAlmostEqual(sum(e['geometry']['sweep_angle_deg'] for e in arcs),180.)
        self.assertGreater(len({e['geometry']['curvature_per_m'] for e in arcs}),1)
        self.assertTrue(any(e['geometry']['type']=='circular_arc' for e in elements[:turn['first_element']]))
        self.assertTrue(any(e['geometry']['type']=='circular_arc' for e in elements[turn['last_element']+1:]))
        self.assertTrue(all(1/abs(e['geometry']['curvature_per_m']) > case['full_width_m']/2
                            for e in elements if e['geometry']['type']=='circular_arc'))

    def test_double_turn_is_half_width_connected_and_turns_in_opposite_directions(self):
        case=validation_case('validation_uturn_half_width_double_left')
        self.assertEqual([t['signed_sweep_deg'] for t in case['main_turns']],[180.,-180.])
        self.assertAlmostEqual(case['full_width_m'],case['base_full_width_m']/2)
        for turn in case['main_turns']:
            curve=case['road_spec']['elements'][turn['first_element']]['geometry']
            self.assertAlmostEqual(1/abs(curve['curvature_per_m']),case['base_reference_radius_m']/2)
        road=Road(road_spec=case['road_spec'])
        s=np.linspace(0,road_length(case['road_spec']),500)
        xy,headings=road.convert_progression_to_coordinates(s)
        self.assertTrue(np.isfinite(xy).all())
        self.assertLess(abs(math.atan2(math.sin(headings[-1]),math.cos(headings[-1]))),1e-8)
        for element in case['road_spec']['elements']:
            self.assertTrue(element['active_cone_paths'])

    def test_left_right_are_reflections_including_obstacle_placements(self):
        for variant in VARIANTS:
            left=validation_case('validation_uturn_'+variant+'_left')
            right=validation_case('validation_uturn_'+variant+'_right')
            s=np.linspace(0,left['mission_road_length_m'],401)
            lx,lh=Road(road_spec=left['road_spec']).convert_progression_to_coordinates(s)
            rx,rh=Road(road_spec=right['road_spec']).convert_progression_to_coordinates(s)
            # Road's public coordinate arrays use float32.
            np.testing.assert_allclose(lx*np.asarray([1,-1]),rx,atol=1e-6)
            np.testing.assert_allclose(np.cos(lh),np.cos(rh),atol=1e-6)
            np.testing.assert_allclose(np.sin(lh),-np.sin(rh),atol=1e-6)
            for a,b in zip(left['road_spec']['obstacles'],right['road_spec']['obstacles']):
                self.assertEqual(a['placement']['lateral_offset_m'],-b['placement']['lateral_offset_m'])

    def test_existing_pipeline_loads_fixed_geometry_and_keeps_width_prior_off(self):
        PipelineSettings(kinds=VALIDATION_KINDS)
        for kind in VALIDATION_KINDS:
            case=validation_case(kind)
            for seed in (713,714):
                with make_environment(kind,seed,max_steps=10) as env:
                    self.assertEqual(env.config.road_spec,case['road_spec'])
                    self.assertEqual(env.estimator.settings.known_lane_width_m,0.)
                    self.assertAlmostEqual(env.config.initial_state_config['pose']['progress']['lower'],case['cold_start_progress_m'])


if __name__=='__main__':unittest.main()
