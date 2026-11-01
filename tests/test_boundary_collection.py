from pathlib import Path
import json,unittest
import numpy as np
from offline.lattice_training.boundary_collection import make_boundary_initializer
from offline.lattice_training.cstar import NativePrediction,SolverSettings,frozen_state
from offline.lattice_training.objective import CStarTrackingSettings
from offline.lattice_training.pipeline import PipelineSettings
from offline.lattice_training.repartition import split_unit


class BoundaryCollectionPort(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(__file__).resolve().parents[1]/'.verification/boundary-first-reviewed-20261031'
        if not root.exists():raise unittest.SkipTest('Frozen qualified road evidence not present')
        entry=json.loads((root/'index.json').read_text(encoding='utf-8'))['cases'][0]
        case=entry['case'];cones=json.loads((root/case['fixture_id']/'irregular/cones.json').read_text(encoding='utf-8'))['cones']
        sigma=next(v['radial_sigma_base_m'] for v in entry['views'] if v['view']=='noisy')
        cls.jobs={731000:dict(case=case,cones=cones,sigma=sigma,repetition=0)}
        cls.settings=PipelineSettings(kinds=('boundary_fixture',),train_seeds=(731000,),validation_seeds=(),test_seeds=(),
            train_only=True,starts_per_road=4,cruise_speed_mps=.5,initial_start_mode='stratified_random')

    def make(self):
        return make_boundary_initializer(self.jobs,'unit-pool')('boundary_fixture',731000,0,self.settings,
            CStarTrackingSettings(road_progress_weight=1.),None,SolverSettings())

    def test_frozen_map_noise_snapshot_and_native_motion_remain_consistent(self):
        env,observation,initialization=self.make()
        with env:
            self.assertEqual(env._step_count,8)
            self.assertEqual(env.screen_sigma,self.jobs[731000]['sigma'])
            self.assertTrue(env.screen_noise and env.screen_paired_noise)
            self.assertEqual(env.collection_physical_cones,self.jobs[731000]['cones'])
            self.assertEqual(initialization['initial_state_draws'],1)
            self.assertEqual(env.estimator.settings.known_lane_width_m,0.)
            predictor=NativePrediction(env)
            controls=[(.05,initialization['initial_control'][1])]*3
            predicted,_=predictor.predict(controls)
            with env.fork() as rollout:
                actual=[frozen_state(rollout._backend.car._state_values(),predictor.anchor,rollout._backend.car)]
                for control in controls:
                    rollout.step(control)
                    actual.append(frozen_state(rollout._backend.car._state_values(),predictor.anchor,rollout._backend.car))
                np.testing.assert_allclose(predicted,actual,atol=1e-11,rtol=0)
            with env.fork() as fork:
                a=env.step((.05,initialization['initial_control'][1]))
                b=fork.step((.05,initialization['initial_control'][1]))
                np.testing.assert_allclose(env._backend.car._state_values(),fork._backend.car._state_values(),atol=0,rtol=0)
                self.assertEqual(a[0]['sensors']['cone_detections'],b[0]['sensors']['cone_detections'])

    def test_cstar_checks_original_boundary_and_actual_cones_not_host_lane(self):
        env,observation,initialization=self.make()
        with env:
            checker=NativePrediction(env,SolverSettings())
            self.assertIsNotNone(checker.physical_road)
            raw=np.array(env._backend.car._state_values())
            cone=env.collection_physical_cones[5];raw[:2]=[cone['x_m'],cone['y_m']]
            margins=checker.margins(raw)
            self.assertLess(margins['physical_cone_clearance_m'],0.)
            self.assertLess(margins['obstacle_contact_margin'],0.)
            self.assertEqual(env.collection_label_contract['boundary_geometry_contract']['hard_constraints'],
                'original_boundary_polygon_and_frozen_physical_cones')

    def test_repartition_groups_all_mirrors_starts_and_repetitions(self):
        a=dict(scene_id='left',episode=dict(split_group_id='boundary_seed_4100'))
        b=dict(scene_id='right',episode=dict(split_group_id='boundary_seed_4100',repetition=1,initial_index=3))
        self.assertEqual(split_unit(a),split_unit(b))


if __name__=='__main__':unittest.main()
