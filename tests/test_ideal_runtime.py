from pathlib import Path
import json,tempfile,unittest
import numpy as np
from offline.mvp21_sim.ideal_control import walk
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.validation_config import configuration
from offline.mvp21_sim.boundary_evaluation import sensor_view


class IdealRuntimeChecks(unittest.TestCase):
    def test_lateral_join_does_not_turn_camera_to_the_connector(self):
        from offline.mvp21_sim.ideal_control import tracking_step
        path=np.array([[0.,0.],[0.,.2],[2.,.2]])
        xy,heading,_=tracking_step(path,.025,.2,0.)
        np.testing.assert_allclose(xy,[0.,.025]);self.assertEqual(heading,0.)

    def test_full_baseline_projects_current_pose_before_tracking(self):
        from types import SimpleNamespace
        from offline.mvp21_sim.ideal_control import IdealPathController
        env=SimpleNamespace(_backend=SimpleNamespace(car=SimpleNamespace(_state_values=lambda:[0.,0.,0.,0.,0.,0.,0.])))
        controller=IdealPathController(env,None,{})
        reference=dict(path=dict(samples_xy=[[-2.,0.],[2.,0.]]),target_speed_mps=.5)
        _,_,diagnostics=controller.calculate(reference,{},None,0.,.05)
        self.assertTrue(diagnostics['valid']);self.assertEqual(diagnostics['connector_length_m'],0.)
        np.testing.assert_allclose(controller.points,[[0.,0.],[2.,0.]])

    def test_arc_walk_follows_corner_and_clamps_end(self):
        path=np.array([[0.,0.],[1.,0.],[1.,1.]])
        p,yaw,s=walk(path,1.25)
        np.testing.assert_allclose(p,[1.,.25]);self.assertAlmostEqual(yaw,np.pi/2)
        p,_,s=walk(path,5.)
        np.testing.assert_allclose(p,[1.,1.]);self.assertEqual(s,2.)

    def test_ideal_loop_has_no_teleport_and_native_reference_stays_on_source(self):
        from offline.mvp21_sim.ideal_control import world_points
        root=Path(__file__).resolve().parents[1]/'offline/lattice_training/artifacts/2026-10-31-boundary-rnn/inputs'
        files=list(root.glob('boundary_s_left_4102_*.json'))
        if not files:self.skipTest('Frozen maps unavailable')
        frozen=json.loads(files[0].read_text(encoding='utf-8'));case=frozen['case']
        case.update(frozen_physical_cones=frozen['cones'],measurement_noise=dict(sigma_m=frozen['sigma']))
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)/'run'
            summary=run_runtime(case,directory,seed=1101,detailed_trace=True,
                validation_config=configuration(dict(modules=dict(control='ideal'),cruise_speed_mps=.5,max_steps=45,stop_tail_s=0.)))
            self.assertTrue(summary['ideal_controls_used']);self.assertFalse(summary['cstar_controls_used'])
            self.assertLessEqual(summary['maximum_executed_speed_mps'],.5+1e-10)
            self.assertGreater(summary['final_progress_m'],summary['initial_progress_m']+.3)
            def rows(name):return [json.loads(s) for s in (directory/name).read_text().splitlines()]
            trace=rows('trace.jsonl');self.assertTrue(trace)
            sources={r['job']['job_id']:r for r in rows('submissions.jsonl')}
            packets={r['result']['job_id']:r for r in rows('shadow.jsonl')}
            count=0
            for row in trace:
                self.assertLessEqual(np.linalg.norm(np.array(row['raw_state'])[:2]-row['raw_state_before'][:2]),.025+1e-10)
                self.assertEqual(row['raw_action'],[0.,0.])
                if row['execution']['mode']!='planner':continue
                count+=1;j=row['execution']['job_id']
                source=world_points(packets[j]['result']['selected']['path'],sources[j]['decision_pose'])
                reference=world_points(row['reference']['path']['samples_xy'],row['raw_state_before'])
                a=source[:-1];d=source[1:]-a;den=np.sum(d*d,axis=1)
                for point in reference:
                    u=np.clip(np.sum((point-a)*d,axis=1)/den,0,1)
                    self.assertLess(np.min(np.linalg.norm(point-a-u[:,None]*d,axis=1)),1e-9)
            self.assertGreater(count,5)
            (directory/'road.json').write_text(json.dumps(case),encoding='utf-8')
            self.assertEqual(sensor_view(directory)['exact_delivered_packets'],len(trace))


if __name__=='__main__':unittest.main()
