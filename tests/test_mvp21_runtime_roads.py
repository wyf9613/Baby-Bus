"""Check the runtime ports against acquisition clocks and the actual MVP PID."""
from pathlib import Path
import tempfile
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from mvp21_policy.control import PolicyController, ControlSettings
from offline.lattice_training.validation_roads import validation_case
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.source import parameters
from offline.mvp21_sim.validate_roads import read_rows, timing_audit


class RuntimeRoadChecks(unittest.TestCase):
    def test_independent_clocks_mid_path_replanning_actual_pid_and_zero_hold(self):
        case = validation_case('validation_uturn_original_left')
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'run'
            summary = run_runtime(case, output, max_steps=50, latency_steps=1, seed=713,
                                  detailed_trace=True, stop_tail_s=.2)
            audit = timing_audit(output)
            self.assertTrue(audit['checks_passed'])
            self.assertEqual(audit['submissions'],3)
            self.assertEqual(audit['camera_acquisitions'],50)
            self.assertEqual(audit['camera_acquisitions_including_zero_hold'],54)
            self.assertGreaterEqual(audit['native_job_switches'],2)
            rows = read_rows(output/'trace.jsonl')
            tracking = [r for r in rows if r['phase']=='tracking']
            self.assertEqual(len(rows)-len(tracking),4)
            self.assertFalse(summary['cstar_controls_used'])
            self.assertFalse(summary['learned_scorer_used'])
            # These exact actions must be reproducible by the shared controller,
            # including its integrated error and steering-rate/filter history.
            controller = PolicyController(ControlSettings(**parameters()['control']))
            for index,row in enumerate(tracking):
                drive, steer, diagnostics = controller.calculate(row['reference'],row['state'],
                    row['observation']['estimation']['vehicle_params'],row['timestamp_s'],0. if index==0 else .05)
                self.assertTrue(diagnostics['valid'])
                self.assertAlmostEqual(drive,row['raw_action'][0],places=12)
                self.assertAlmostEqual(steer,row['raw_action'][1],places=12)
            # First delivery arrives while the vehicle moves; next job is not
            # gated by full execution of the previous spatial candidate.
            delivered = read_rows(output/'shadow.jsonl')
            second = next(r for r in tracking if abs(r['timestamp_s']-delivered[1]['delivery']['delivered_s'])<1e-8)
            previous = next(r for r in tracking if r['step']==second['step']-1)
            self.assertGreater(previous['execution']['remaining_length_m'],.1)
            self.assertGreater(second['x_m'],tracking[0]['x_m'])

    def test_single_side_cold_start_keeps_actual_node_order_and_zero_publication(self):
        case = validation_case('validation_uturn_original_left')
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'cold'
            summary = run_runtime(case, output, progress=case['cold_start_progress_m'], max_steps=20,
                                  latency_steps=1, seed=713, detailed_trace=True, stop_tail_s=.2)
            audit = timing_audit(output)
            self.assertEqual(summary['fault'],'planning:invalid_road_single_side_width_unknown')
            self.assertEqual(summary['delivery']['submitted'],0)
            self.assertEqual(audit['control_ticks'],0)
            self.assertEqual(audit['camera_acquisitions_including_zero_hold'],4)
            self.assertEqual(audit['camera_intervals_checked'],3)
            self.assertTrue(audit['checks_passed'])
            self.assertTrue(all(r['raw_action']==[0.,0.] for r in read_rows(output/'trace.jsonl')))


if __name__=='__main__':
    unittest.main()
