from dataclasses import replace
from types import SimpleNamespace
import unittest,math
import test_control as c
import test_mpc as m
import test_current_centerline_profile as profile
P=profile.P
class SupervisionTiming(unittest.TestCase):
    def node(self):
        n=c.make_node(mvp=True)
        n.control_settings=c.ns['ControlSettings'](**P['control'])
        n.estimation_settings=c.ns['EstimationSettings'](**P['estimation'])
        n.mpc_settings=SimpleNamespace(enabled=True,shadow=False,reference_source='planning')
        n.planning_settings=c.ns['PlanningSettings'](algorithm='road_centerline')
        n.distance_limiter=c.ns['RunDistanceLimiter'](n.control_settings.max_distance_m,n.control_settings.max_dt_s)
        c.feed(n);c.start(n);n.control_has_run=True
        n.previous_policy_step_at=n.test_clock.monotonic
        n.last_control_reference_deadline_s=n.test_clock.ros_ns/1e9+.2
        return n
    def test_debug_odometry_skips_gap_without_inventing_distance(self):
        n=self.node();n.test_clock.advance(.21);c.feed(n,speed=.2)
        n.last_control_reference_deadline_s=n.test_clock.ros_ns/1e9+1
        n.supervision_callback()
        self.assertEqual(n.fsm_state,3)
        self.assertEqual(n.distance_limiter.skipped_gap_count,1)
        self.assertEqual(n.distance_limiter.distance_m,0)
    def test_real_distance_budget_retains_gap_stop(self):
        n=self.node();n.control_settings=replace(n.control_settings,max_distance_m=3)
        n.distance_limiter.maximum_m=3
        n.test_clock.advance(.21);c.feed(n);n.supervision_callback()
        self.assertEqual(n.fsm_state,2);self.assertIn('clock/gap',n.state_reason)
    def test_stale_motion_still_stops_without_distance_budget(self):
        n=self.node();n.test_clock.advance(.26)
        n.supervision_callback();self.assertEqual(n.fsm_state,2)
        self.assertIn('motion sensor',n.state_reason)
    def test_backward_clock_is_never_skipped(self):
        limiter=c.ns['RunDistanceLimiter'](0,.2);limiter.reset(2.,.1)
        self.assertIn('clock/gap',limiter.advance(.1,1.,skip_unmeasured_gap=True))
    def test_fresh_inputs_can_refresh_expired_reference_but_zero_first(self):
        n=self.node();n.test_clock.advance(.21);c.feed(n,speed=.1)
        n.supervision_callback()
        self.assertEqual(n.fsm_state,3);self.assertEqual(n.action_publisher.messages[-1].drive,0)
        def fresh(*args):
            n.last_control_reference_deadline_s=n.test_clock.ros_ns/1e9+.2
            return .25,.03,None,None,None
        n.calculate_policy_actions=fresh;n.run_policy_step()
        self.assertEqual(n.fsm_state,3,n.state_reason)
        self.assertEqual(n.action_publisher.messages[-1].drive,.25)
    def test_no_new_motion_is_not_treated_as_refresh(self):
        n=self.node();n.test_clock.advance(.21);n.supervision_callback()
        self.assertEqual(n.fsm_state,2);self.assertIn('expired',n.state_reason)
    def test_refresh_window_is_not_renewed_by_supervisor_calls(self):
        n=self.node();n.test_clock.advance(.21);c.feed(n);n.supervision_callback()
        self.assertEqual(n.fsm_state,3)
        n.test_clock.advance(.05);c.feed(n);n.supervision_callback()
        self.assertEqual(n.fsm_state,2);self.assertIn('expired',n.state_reason)
    def test_new_result_must_still_be_valid_after_computation(self):
        n=self.node();n.test_clock.advance(.21);c.feed(n)
        def too_late(*args):
            n.last_control_reference_deadline_s=n.test_clock.ros_ns/1e9+.02
            n.test_clock.advance(.03)
            return .25,.03,None,None,None
        n.calculate_policy_actions=too_late;n.run_policy_step()
        self.assertEqual(n.fsm_state,2);self.assertIn('expired',n.state_reason)
        self.assertTrue(all(msg.drive==0 for msg in n.action_publisher.messages))
    def test_zero_during_refresh_is_included_in_mpc_command_history(self):
        n=self.node();ctrl=m.Controller(m.Settings(**P['mpc']),m.Vehicle(**P['vehicle']))
        ctrl.clock_s=1.;ctrl.history=[(1.,.35,.1)];ctrl.applied_drive=.35;ctrl.applied_steer=.1
        n.mpc_controller=ctrl;n.test_clock.advance(.21);c.feed(n);n.supervision_callback()
        self.assertEqual(n.fsm_state,3)
        self.assertEqual(ctrl.command_at(1.22),(0.,0.))
        self.assertEqual((ctrl.applied_drive,ctrl.applied_steer),(0.,0.))
        self.assertEqual(ctrl.feedback_command,0.)
        self.assertEqual(ctrl.clock_s,1.)

    def test_default_refresh_remains_disabled(self):
        self.assertEqual(c.ns['ControlSettings']().reference_refresh_grace_s,0)
        with self.assertRaises(ValueError):c.ns['ControlSettings'](reference_refresh_grace_s=.2)
class StartGeometry(unittest.TestCase):
    def test_noisy_straight_start_uses_the_same_fitted_heading_in_solver(self):
        helper=profile.StartBridgeChecks();traj=helper.trajectory()
        for i,p in enumerate(traj['points'][:9]):
            p['y_m']=.008*(1 if i%2 else -1)
            p['curvature_1pm']=4. if i%2 else -4.
        cfg=m.Settings(**P['mpc']);ctrl=m.Controller(cfg,m.Vehicle(**P['vehicle']))
        out=ctrl.step(traj,0.,0.,10.,True)
        self.assertEqual(out['debug']['branch'],'solved')
        heading=out['debug']['bridge_fit_heading_rad']
        detail=m.namespace['mpc_path_projector'](traj['points'],1.2,heading).details(0,0)
        self.assertAlmostEqual(out['debug']['e_psi_rad'],-detail['projection'][2])
        self.assertEqual(detail['projection'][3],0)
        self.assertLess(out['debug']['bridge_fit_max_deviation_m'],.025)
    def test_expiry_budget_still_cannot_outlive_motion_source(self):
        f=profile.f;pipe=f.EstimationPipeline(f.EstimationSettings(**P['estimation']))
        detections=[[x,y,0,color,.95] for x in (.4,.7,1.,1.4,1.8,2.2) for y,color in ((.45,P['estimation']['left_colour']),(-.45,P['estimation']['right_colour']))]
        road=pipe._history_road({'detections':detections},10_000_000_000,0.,10.);road['time_aligned']=True
        state=dict(valid=True,speed_valid=True,yaw_rate_valid=True,speed_mps=.1,yaw_rate_rps=0.,frame_id='base_link',timestamp_s=10.,source_age_s={'wheel_speed':.1,'imu_angular_velocity':.1})
        planner=f.RoadCenterlinePlanner(f.PlanningSettings(**{k:v for k,v in P['planning'].items() if k!='lattice'}))
        ref,_=planner.plan(road,state,{}, {},10.,{'road':1.5,'wheel_speed':.25,'imu_angular_velocity':.25},mvp=True)
        self.assertTrue(ref['valid']);self.assertAlmostEqual(ref['valid_for_s'],.15)
if __name__=='__main__':unittest.main()
