from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from offline.lattice_training.validation_roads import VALIDATION_KINDS,validation_case
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.validation_config import configuration
from offline.mvp21_sim.validate_roads import read_rows


class ModuleValidationTests(unittest.TestCase):
    def case(self):return validation_case(VALIDATION_KINDS[0])
    def run_case(self,cfg):
        with tempfile.TemporaryDirectory() as tmp:
            summary=run_runtime(self.case(),Path(tmp)/'run',seed=713,detailed_trace=True,validation_config=cfg)
            return summary,read_rows(Path(tmp)/'run/trace.jsonl'),read_rows(Path(tmp)/'run/shadow.jsonl')

    def test_default_matches_existing_shared_runtime(self):
        cfg=configuration(dict(max_steps=35,stop_tail_s=0.))
        with tempfile.TemporaryDirectory() as tmp:
            old=Path(tmp)/'old';new=Path(tmp)/'new'
            run_runtime(self.case(),old,seed=713,max_steps=35,latency_steps=1,detailed_trace=True,stop_tail_s=0.)
            run_runtime(self.case(),new,seed=713,detailed_trace=True,validation_config=cfg)
            a,b=read_rows(old/'trace.jsonl'),read_rows(new/'trace.jsonl')
            self.assertEqual(len(a),len(b))
            for x,y in zip(a,b):
                for key in ('raw_action','raw_state','active_job_id','road_status'):self.assertEqual(x[key],y[key])

    def test_separate_camera_control_generation_selection_clocks(self):
        cfg=configuration(dict(max_steps=45,stop_tail_s=0.,frequencies_hz=dict(perception=10,control=10,planner=2,policy=1)))
        summary,rows,deliveries=self.run_case(cfg)
        updates=[r for r in rows if r['control_updated']]
        self.assertEqual(len(updates),(len(rows)+1)//2)
        self.assertTrue(all(abs(b['timestamp_s']-a['timestamp_s']-.1)<1e-8 for a,b in zip(updates,updates[1:])))
        for i,r in enumerate(rows):
            if not r['control_updated']:self.assertEqual(r['raw_action'],rows[i-1]['raw_action'])
        self.assertEqual(summary['perception_rate_hz'],10)
        self.assertGreater(len(deliveries),1)
        self.assertTrue(all(abs(b['delivery']['delivered_s']-a['delivery']['delivered_s']-1.)<1e-8 for a,b in zip(deliveries,deliveries[1:])))

    def test_real_mvp1_has_exactly_one_straight_candidate(self):
        summary,rows,deliveries=self.run_case(configuration(dict(max_steps=12,stop_tail_s=0.,modules=dict(planner='mvp1_straight'))))
        self.assertTrue(deliveries)
        for packet in deliveries:
            result=packet['result'];self.assertEqual(result['candidate_count'],1,result)
            points=result['candidates'][0]['path'];a,b=points[0],points[-1]
            self.assertTrue(all(abs((p[0]-a[0])*(b[1]-a[1])-(p[1]-a[1])*(b[0]-a[0]))<1e-8 for p in points))

    def test_perfect_port_and_real_cstar_solver(self):
        cfg=configuration(dict(max_steps=4,stop_tail_s=0.,modules=dict(perception='perfect',control='cstar'),
            cstar_solver=dict(population=4,iterations=1,max_evaluations=6,max_wall_s=10.)))
        summary,rows,packets=self.run_case(cfg)
        self.assertTrue(rows)
        self.assertTrue(summary['cstar_controls_used'])
        self.assertEqual(rows[0]['control']['reason'],'cstar_control_sequence')
        self.assertGreater(rows[0]['control']['cstar']['evaluations'],0)
        self.assertEqual(rows[0]['observation']['estimation']['road']['road_input_source'],'explicit_perfect_perception')
        self.assertTrue(rows[0]['raw_estimator_road'])

    def test_unknown_settings_and_unrepresentable_rates_fail(self):
        for cfg in (dict(modules=dict(control='other')),dict(frequencies_hz=dict(control=3)),dict(max_steps=0),dict(bogus=True)):
            with self.assertRaises(ValueError):configuration(cfg)

    def test_cstar_control_hold_uses_native_substeps_at_lower_frequency(self):
        cfg=configuration(dict(max_steps=4,stop_tail_s=0.,frequencies_hz=dict(control=10),
            modules=dict(perception='perfect',control='cstar'),
            cstar_solver=dict(population=4,iterations=1,max_evaluations=6,max_wall_s=10.)))
        summary,rows,_=self.run_case(cfg)
        self.assertEqual(summary['control_updates'],2)
        self.assertEqual(rows[1]['raw_action'],rows[0]['raw_action'])
        self.assertEqual(rows[3]['raw_action'],rows[2]['raw_action'])

    def test_zero_step_failure_is_a_pose_snapshot_not_fake_motion(self):
        cfg=configuration(dict(max_steps=1,stop_tail_s=0.,modules=dict(perception='no_centerline')))
        summary,rows,_=self.run_case(cfg)
        self.assertEqual(summary['steps'],0)
        self.assertEqual(rows[0]['phase'],'fault_snapshot')
        self.assertFalse(rows[0]['action_applied'])
        self.assertEqual(rows[0]['raw_state_before'],rows[0]['raw_state'])


if __name__=='__main__':unittest.main()
