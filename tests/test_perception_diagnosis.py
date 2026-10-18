"""Protect acquisition-time alignment and read-only cone reconstruction."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from offline.mvp21_sim.diagnose_perception import world_points, body_points, acquisition_pose, query_cones
from offline.mvp21_sim.simulation import fixed_road_environment
from offline.lattice_training.validation_roads import validation_case


class PerceptionAuditChecks(unittest.TestCase):
    def test_delayed_packet_matches_truth_only_at_acquisition_pose(self):
        with fixed_road_environment(validation_case('validation_uturn_original_left')) as env:
            rows=[]
            summary=dict(initial_timestamp_s=env._frame['timestamp_s'],initial_raw_state=list(env._backend.car._state_values()))
            for _ in range(5):
                rows.append(dict(timestamp_s=env._frame['timestamp_s'],raw_state_before=list(env._backend.car._state_values())))
                env.step((.3,.2))
            stamp=env._frame['sensor_stamp_ns']['cone_detections']
            pose,basis=acquisition_pose(rows,summary,stamp)
            batch=env._frame['sensors']['cone_detections']['detections']
            query=query_cones(env,pose,24)
            np.testing.assert_allclose(np.asarray(batch)[:,:2],np.column_stack((query['px_in_body_frame'],query['py_in_body_frame'])),atol=1e-7)
            actual=world_points(batch,pose);wrong=world_points(batch,env._backend.car._state_values())
            self.assertGreater(np.max(np.abs(np.asarray(actual)-np.asarray(wrong))),.001)
            self.assertEqual(basis,'recorded_acquisition_pose')
            np.testing.assert_allclose(body_points(actual,pose),np.asarray(batch)[:,:2],atol=1e-12)

    def test_pose_gaps_fail_instead_of_inventing_current_pose(self):
        with self.assertRaises(ValueError):
            acquisition_pose([],dict(initial_timestamp_s=1.,initial_raw_state=[0.]*7),2_000_000_000)


if __name__=='__main__':unittest.main()
