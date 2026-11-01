from pathlib import Path
import json,tempfile,unittest
import numpy as np
from offline.mvp21_sim.simulation import fixed_road_environment
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.validation_config import configuration
from offline.mvp21_sim.boundary_evaluation import sensor_view


class NoisyBoundaryRuntimeChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(__file__).resolve().parents[1]/'offline/lattice_training/artifacts/2026-10-31-boundary-rnn/inputs'
        files=list(root.glob('boundary_s_left_4102_*.json'))
        if not files:raise unittest.SkipTest('Frozen Test maps unavailable')
        f=json.loads(files[0].read_text(encoding='utf-8'));cls.case=f['case']
        cls.case.update(frozen_physical_cones=f['cones'],measurement_noise=dict(sigma_m=f['sigma']))

    def test_fixed_pose_and_delayed_sensor_packet_match_exact_producer(self):
        with fixed_road_environment(self.case,seed=1101) as env:
            self.assertEqual(env._step_count,8)
            self.assertEqual(env.estimator.settings.known_lane_width_m,0.)
            self.assertEqual(np.linalg.norm(env._backend.car._state_values()[3:6]),0.)
            for _ in range(4):
                obs,*_=env.step((.02,0.))
                packet=env.sensor_audit[obs['sensor_stamp_ns']['cone_detections']]
                self.assertEqual(obs['sensors']['cone_detections']['detections'],packet['detected_raw'])

    def test_paired_gaussian_draws_do_not_depend_on_vehicle_path(self):
        with fixed_road_environment(self.case,seed=1101) as a,fixed_road_environment(self.case,seed=1101) as b:
            for _ in range(8):
                a.step((.08,.05));b.step((.08,-.05))
            first={(r['tick'],d['source_id']):d['radial_error_m'] for r in a.sensor_audit.values() for d in r['noise']}
            second={(r['tick'],d['source_id']):d['radial_error_m'] for r in b.sensor_audit.values() for d in r['noise']}
            common=first.keys()&second.keys();self.assertGreater(len(common),20)
            self.assertTrue(all(first[k]==second[k] for k in common))

    def test_shared_runtime_uses_mvp_and_exports_the_noise_audit(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)/'run'
            profile=configuration(dict(cruise_speed_mps=.5,max_steps=30,stop_tail_s=0.))
            summary=run_runtime(self.case,directory,seed=1101,detailed_trace=True,validation_config=profile)
            (directory/'road.json').write_text(json.dumps(self.case),encoding='utf-8')
            self.assertFalse(summary['cstar_controls_used'])
            self.assertEqual(summary['shadow']['effective_settings']['sample_count'],48)
            self.assertEqual(summary['perception_rate_hz'],20.)
            audit=sensor_view(directory)
            self.assertGreater(audit['exact_delivered_packets'],0)


if __name__=='__main__':unittest.main()
