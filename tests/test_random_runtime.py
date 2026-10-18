import json
from pathlib import Path
import tempfile
import unittest

from offline.mvp21_sim.random_roads import saved_random_roads, fingerprint
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.policy_selection import DEFAULT_MODEL
from offline.mvp21_sim.validate_roads import read_rows, timing_audit
from offline.mvp21_sim.diagnose_perception import diagnose_case
from offline.lattice_training.dataset import write_json


class RandomRoadRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.catalog=saved_random_roads()

    def test_restores_empty_episodes_and_changed_obstacle_variants(self):
        cases=self.catalog['cases']
        self.assertEqual((len(cases),self.catalog['source_scene_ids'],self.catalog['source_episodes']),(33,28,66))
        self.assertTrue(any(e['recorded_decisions']==0 for c in cases for e in c['source_episodes']))
        for c in cases:
            self.assertEqual(fingerprint(c['road_spec']),c['road_sha256'])
            self.assertFalse(c['scenario_width_prior_used'])
        by_scene={}
        for c in cases:
            for scene in c['saved_scene_ids']:by_scene.setdefault(scene,[]).append(c)
        variants=[v for v in by_scene.values() if len(v)>1]
        self.assertEqual(len(variants),5)
        self.assertTrue(all(v[0]['road_spec']['obstacles']!=v[1]['road_spec']['obstacles'] for v in variants))
        self.assertTrue(all(len({c['split_group'] for c in v})==1 for v in variants))

    def test_missing_centerline_is_only_a_runtime_input_mask(self):
        case=next(c for c in self.catalog['cases'] if c['kind']=='randomized' and c['road_seed']==0)
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp)/'run'
            summary=run_runtime(case,directory,seed=case['runtime_seed'],road_input='no_centerline',
                latency_steps=1,max_steps=12,detailed_trace=True,policy_model_path=DEFAULT_MODEL,stop_tail_s=.1)
            write_json(directory/'road.json',case)
            rows=read_rows(directory/'trace.jsonl');tracking=[r for r in rows if r['phase']=='tracking']
            self.assertFalse(tracking)
            self.assertEqual(summary['fault'],'planning:invalid_centerline_samples')
            self.assertFalse(summary['learned_scorer_used'])
            self.assertFalse(read_rows(directory/'submissions.jsonl'))
            failure=json.loads((directory/'failure.json').read_text())
            raw=failure['perception_road']
            self.assertTrue(raw.get('centerline_xy') or (raw.get('forward_geometry')or{}).get('centerline_xy'))
            supplied=failure['observation']['estimation']['road']
            self.assertFalse(supplied['centerline_xy'])
            self.assertFalse((supplied.get('forward_geometry')or{}).get('centerline_xy'))
            self.assertEqual(supplied['left_boundary_xy'],failure['perception_road']['left_boundary_xy'])
            self.assertTrue(timing_audit(directory)['checks_passed'])
            self.assertLess(diagnose_case(directory)['packet_query_max_error_m'],3e-5)
            self.assertEqual(summary['perception_rate_hz'],20)
            self.assertEqual(summary['lattice_generation_rate_hz'],1)


if __name__=='__main__':unittest.main()
