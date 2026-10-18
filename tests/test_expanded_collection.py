from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from dreamgym.envs import Road
from offline.lattice_training.scenarios import expanded_road_spec, road_input, SUITE_KINDS
from offline.lattice_training.repartition import repartition_records
from offline.lattice_training.dataset import import_dataset, load_dataset
from offline.lattice_training.failure_audit import failure_selection_metrics
from offline.lattice_training.pipeline import make_environment, make_planner, candidates_at
from tests.test_lattice_training import decision_records


class ExpandedCollectionChecks(unittest.TestCase):
    def test_near_center_obstacle_variant_keeps_same_road_group_and_has_side_passage(self):
        from offline.lattice_training.roads import road_identity
        with make_environment('suite_straight', 710, max_steps=10) as base, make_environment(
                'suite_straight', 710, max_steps=10, obstacle_layout='near_center_alternating') as hard:
            self.assertEqual(road_identity(base.config.road_spec), road_identity(hard.config.road_spec))
            self.assertEqual(len(hard.config.road_spec['obstacles']), 2)
            for obstacle in hard.config.road_spec['obstacles']:
                offset = abs(obstacle['placement']['lateral_offset_m'])
                half = obstacle['geometry']['width_m']/2
                self.assertLess(offset-half, .125)
                width = hard.config.road_spec['elements'][0]['lanes']['reference_lane']['width_m']['start']
                self.assertGreater(width/2+offset-half, .25+.04)

    def test_width_prior_is_explicit_and_unblocks_observed_uturn_boundary(self):
        with make_environment('uturn_left', 710, max_steps=10, use_scenario_width_prior=True) as env:
            env.reset(seed=710)
            obs, *_ = env.step((0., 0.))
            expected = env.config.road_spec['elements'][0]['lanes']['reference_lane']['width_m']['start']
            self.assertEqual(env.estimator.settings.known_lane_width_m, expected)
            self.assertTrue(obs['estimation']['road']['forward_geometry']['right_boundary_xy'])
            ref, info = candidates_at(make_planner(sample_count=8, rate_hz=20), obs)
            self.assertTrue(ref['valid'], info)
            self.assertTrue(info['candidate_references'])

    def test_uturn_has_single_straight_inner_divider_and_outer_semicircle(self):
        for kind, sign in [('uturn_left', 1), ('uturn_right', -1)]:
            spec = expanded_road_spec(kind, 713)
            radius = 1/abs(spec['elements'][1]['geometry']['curvature_per_m'])
            entry = spec['elements'][0]['geometry']['length_m']
            cones = Road(road_spec=spec).resolve_cones(0)
            inner = 'left_boundary' if sign > 0 else 'right_boundary'
            inside = np.asarray([c.position_world_m for c in cones.records if c.path_name == inner])
            outside = np.asarray([c.position_world_m for c in cones.records if c.path_name != inner])
            self.assertGreater(len(inside), 5)
            np.testing.assert_allclose(inside[:, 1], sign*radius, atol=1e-6)
            self.assertLessEqual(inside[:, 0].max(), entry+1e-6)
            cap = outside[outside[:, 0] > entry+1e-4]
            self.assertGreater(len(cap), 5)
            np.testing.assert_allclose(np.linalg.norm(cap-[entry, sign*radius], axis=1), 2*radius, atol=1e-5)
            self.assertEqual(spec['elements'][1]['geometry']['sweep_angle_deg'], 180.)

    def test_missing_centerline_removes_both_sources_and_preserves_obstacles_boundaries(self):
        original = decision_records()[0]['observation']
        original['estimation']['road']['forward_geometry'] = deepcopy(original['estimation']['road'])
        changed = road_input(original, 'no_centerline')
        for source in (changed['estimation']['road'], changed['estimation']['road']['forward_geometry']):
            self.assertEqual(source['centerline_xy'], [])
        self.assertTrue(original['estimation']['road']['centerline_xy'])
        self.assertEqual(changed['estimation']['obstacles'], original['estimation']['obstacles'])
        self.assertEqual(changed['estimation']['road']['left_boundary_xy'], original['estimation']['road']['left_boundary_xy'])

    def test_repartition_keeps_views_and_reflections_together_and_preserves_raw_labels(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            records = []
            template = decision_records()[0]
            for seed in range(5):
                for kind in SUITE_KINDS:
                    for mode in ('perception', 'no_centerline'):
                        r = deepcopy(template)
                        r.update(group_id=f'{kind}:{seed}:{mode}', scene_id=f'road:{kind}:{seed}')
                        r['episode'] = dict(kind=kind, road_seed=seed, road_input_mode=mode)
                        r['label_source']['collection_settings'] = dict(kinds=[kind], road_input_mode=mode)
                        records.append(r)
            path = root/'raw.jsonl'
            before = '\n'.join(json.dumps(r) for r in records)
            path.write_text(before, encoding='utf-8')
            metadata = repartition_records([path], root/'merged')
            data, _ = load_dataset(root/'merged/dataset')
            manifest = metadata['repartition']
            self.assertEqual(set(data['splits']), {'train', 'validation', 'test'})
            self.assertEqual(len(manifest['assignments']), 5)
            for seed in range(5):
                selected = {manifest['assignments'][manifest['scene_groups'][f'road:{kind}:{seed}']] for kind in SUITE_KINDS}
                self.assertEqual(len(selected), 1)
            self.assertEqual(path.read_text(encoding='utf-8'), before)
            merged = [json.loads(line) for line in (root/'merged/decisions.jsonl').read_text().splitlines()]
            for a, b in zip(records, merged):
                self.assertEqual(a['candidates'], b['candidates'])
                self.assertEqual(json.loads(json.dumps(a['observation'])), b['observation'])

    def test_varying_collection_does_not_admit_changed_solver_semantics(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = decision_records()
            for i, row in enumerate(rows):
                row['label_source']['solver_settings'] = {'max_evaluations': 12 if i < 2 else 64}
            path = Path(tmp)/'records.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in rows), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'cannot mix'):
                import_dataset(path, Path(tmp)/'data', allow_collection_variation=True)

    def test_explicit_merge_accepts_equivalent_numeric_settings_and_different_roads(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = decision_records()
            for i, row in enumerate(rows):
                row['label_source']['solver_settings'] = {'max_wall_s': 30. if i == 0 else 30}
                row['label_source']['road_randomization'] = {'seed': i}
                row['label_source']['collection_settings'] = {'kind': str(i)}
            path = Path(tmp)/'records.jsonl'
            path.write_text('\n'.join(json.dumps(r) for r in rows), encoding='utf-8')
            metadata = import_dataset(path, Path(tmp)/'data', allow_collection_variation=True)
            self.assertEqual(metadata['rows'], 9)
            self.assertEqual(len(metadata['label_sources']), 3)

    def test_failure_metrics_separate_unavoidable_and_perturbed_failures(self):
        base = dict(split='test', kind='uturn_left', mode='no_centerline', candidate_ids=['1', '0'],
                    returns=[-1e6, .2], failure_reasons=[['no_solution'], []])
        rows = [dict(base, group_id='avoidable', scores=[.9, .1], nominal_failed=[True, False], any_failed=[True, False]),
                dict(base, group_id='all_failed', scores=[.9, .1], nominal_failed=[True, True], any_failed=[True, True]),
                dict(base, group_id='perturbed', scores=[.9, .1], nominal_failed=[False, False], any_failed=[True, False]),
                dict(base, group_id='tie', scores=[.5, .5], nominal_failed=[True, False], any_failed=[True, False])]
        result = failure_selection_metrics(rows)['metrics']['test']
        self.assertEqual(result['decisions'], 4)
        self.assertEqual(result['all_nominal_failed'], 1)
        self.assertEqual(result['selected_nominal_failed'], 2)
        self.assertEqual(result['avoidable_nominal_failed'], 1)
        self.assertEqual(result['avoidable_any_trial_failed'], 2)


if __name__ == '__main__':
    unittest.main()
