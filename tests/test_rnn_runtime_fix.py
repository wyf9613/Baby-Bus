from test_user_mpc_bridge import adapter, ref, state, calculate_selected_mpc
from types import SimpleNamespace
from dataclasses import replace
import unittest
from mvp21_policy.transport import ShadowJobSource
import numpy as np

class RuntimeFixChecks(unittest.TestCase):
    def curved_reference(self):
        r=ref()
        r['path']['samples_xy']=[(.6,.05),(.8,.05),(1.,.25),(1.5,.3),(2.,.3)]
        r['path']['range']=[0.,2.]
        return r

    def test_bounded_prefix_retains_tracking_error_and_deadline(self):
        a=adapter()
        a.estimates=dict(state=state(),road=dict(frame_id='base_link',time_aligned=True,timestamp_s=10.,
            boundary_source=dict(left='observed',right='observed'),
            left_boundary_xy=[(0.,.8),(3.,.8)],right_boundary_xy=[(0.,-.8),(3.,-.8)]))
        t,reason=a.prepare(self.curved_reference(),state(),10.)
        self.assertIsNone(reason)
        self.assertEqual(t['join_repair']['method'],'bounded_tangent_prefix')
        self.assertEqual(t['timestamp_s'],10.)
        self.assertEqual(t['valid_for_s'],.3)
        self.assertAlmostEqual(t['points'][0]['y_m'],.05)
        self.assertLess(t['points'][0]['x_m'],0.)
        self.assertLessEqual(t['join_repair']['extension_m'],a.mpc_settings.max_backward_extension_m)

    def test_prefix_cannot_hide_observed_body_collision(self):
        a=adapter()
        a.estimates=dict(state=state(),road=dict(frame_id='base_link',time_aligned=True,timestamp_s=10.,
            boundary_source=dict(left='observed',right='observed'),
            left_boundary_xy=[(-.5,.04),(3.,.04)],right_boundary_xy=[(-.5,-.8),(3.,-.8)]))
        t,reason=a.prepare(self.curved_reference(),state(),10.)
        self.assertIsNone(t)
        self.assertIsNotNone(reason)

    def test_rejected_gap_has_debug_without_advancing_controller(self):
        a=adapter(False);r=ref()
        r['path']['samples_xy']=[(2.,0.),(3.,0.),(4.,0.)]
        got=calculate_selected_mpc(a,r,None,state(),{},10.,0.)
        self.assertTrue(got[3]['stop_requested'])
        self.assertEqual(got[3]['mpc']['reject_reason'],'origin_before_path_start')
        self.assertEqual(got[3]['mpc']['first_xy_m'],[2.,0.])
        self.assertEqual(a.mpc_controller.clock_s,0.)

    def test_measured_speed_jacobian_equals_full_finite_difference(self):
        for measured in (False,True):
            a=adapter();solver=a.mpc_controller.solver
            solver.cfg=replace(solver.cfg,speed_feedback_measured_motion=measured)
            z=[.1,-.2,.15,.5,.03];u=[.33,-.1];h=.1
            base,A,B=solver._linearise(z,u,h)
            for j in range(2):
                shifted=list(u);shifted[j]+=1e-6
                expected=(np.array(solver._motion_step(z,shifted,h,solver.vehicle,rate_limited=False))-base)/1e-6
                np.testing.assert_allclose(B[:,j],expected,atol=1e-12)

    def test_priming_does_not_consume_planner_submission_slot(self):
        source=ShadowJobSource()
        source._monotonic=lambda:1.
        source.shadow_settings=SimpleNamespace(rate_hz=1.,fusion_enabled=True)
        source.estimation_output=dict(road=dict(status='missing_motion_history',time_aligned=False),state=state())
        source.shadow_last_submit_s=-float('inf')
        source._shadow_offer()
        self.assertEqual(source.shadow_last_submit_s,-float('inf'))

if __name__=='__main__':unittest.main()
