from pathlib import Path
import unittest,yaml
from types import SimpleNamespace
import test_mpc_real_frames as f
import test_mpc as m
import test_control as c
R=Path(__file__).resolve().parents[1]
P=yaml.safe_load((R/'config/ai4r_policy_speed_feedback.yaml').read_text(encoding='utf-8'))['/**/ai4r_policy']['ros__parameters']
class CurrentCenterline(unittest.TestCase):
    def test_selected_profile_has_one_controller_and_no_lattice_cache(self):
        self.assertEqual(P['planning']['algorithm'],'road_centerline')
        self.assertFalse(P['planning']['lattice']['async_enabled'])
        self.assertEqual(P['estimation']['road_geometry'],'ordered_xy')
        self.assertFalse(P['control']['enabled']);self.assertTrue(P['mpc']['enabled'])
        self.assertFalse(P['control']['robustness_enabled'])
        self.assertEqual(P['control']['max_distance_m'],0)
        self.assertEqual(P['control']['max_run_time_s'],0)
        self.assertEqual((P['estimation']['left_colour'],P['estimation']['right_colour']),(1,2))
    def test_actual_constructor_model_pairing_accepts_partial_profile(self):
        import ast
        source=ast.parse((R/'scripts/policy_node.py').read_text(encoding='utf-8'))
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='PolicyNode')
        init=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        block=next(n for n in init.body if isinstance(n,ast.If) and any(isinstance(x,ast.Name) and x.id=='simulated_mpc' for x in ast.walk(n)))
        code=compile(ast.Module(body=[block],type_ignores=[]),'<actual constructor pairing>','exec')
        node=SimpleNamespace(mpc_settings=m.Settings(**P['mpc']),planning_settings=f.PlanningSettings(**{k:v for k,v in P['planning'].items() if k!='lattice'}))
        exec(code,{'self':node})
        node.planning_settings=f.PlanningSettings(vehicle_limits_source='upstream')
        with self.assertRaises(ValueError):exec(code,{'self':node})

    def test_original_partial_model_and_feedback_are_selected(self):
        self.assertEqual(P['vehicle']['source'],'newcar20_partial_identification_20261012')
        self.assertFalse(P['vehicle']['valid'])
        self.assertEqual(P['vehicle']['steering_gain_rad'],-.3142)
        self.assertEqual(P['mpc']['vehicle_params_source'],'partial_vehicle')
        self.assertTrue(P['mpc']['speed_feedback_enabled']);self.assertFalse(P['mpc']['shadow'])
        self.assertFalse(P['mpc']['overspeed_stop_enabled'])
    def test_selected_cone_layout_to_centerline_to_execution_feedback(self):
        pipe=f.EstimationPipeline(f.EstimationSettings(**P['estimation']))
        detections=[[x,y,0,color,.95] for x in (.95,1.25,1.55,1.9,2.3,2.7,3.1,3.5) for y,color in ((.45,P['estimation']['left_colour']),(-.45,P['estimation']['right_colour']))]
        road=pipe._history_road({'detections':detections},10_000_000_000,0.,10.)
        self.assertTrue(road['valid'],road)
        road['time_aligned']=True
        state=dict(valid=True,speed_valid=True,yaw_rate_valid=True,speed_mps=0.,yaw_rate_rps=0.,frame_id='base_link',timestamp_s=10.,source_age_s={'wheel_speed':.01,'imu_angular_velocity':.01})
        plan_values={k:v for k,v in P['planning'].items() if k!='lattice'}
        planner=f.RoadCenterlinePlanner(f.PlanningSettings(**plan_values))
        ref,diag=planner.plan(road,state,{}, {},10.,{'road':1.5,'wheel_speed':.25,'imu_angular_velocity':.25},mvp=True)
        self.assertTrue(ref['valid'],(ref,diag));self.assertEqual(ref['path']['type'],'ORDERED_CENTERLINE')
        traj=f.adapt(ref,spacing_m=.05);ctrl=m.Controller(m.Settings(**P['mpc']),m.Vehicle(**P['vehicle']))
        result=ctrl.step(traj,0.,0.,10.,True)
        self.assertEqual(result['debug']['branch'],'solved',result)
        self.assertEqual(result['debug']['longitudinal_mode'],'speed_feedback')
        self.assertGreater(result['drive'],0);self.assertLessEqual(result['drive'],.36)
    def test_reference_expiry_independently_stops_centerline_mpc(self):
        n=c.make_node(mvp=True);n.control_settings=c.ns['ControlSettings'](**P['control'])
        n.mpc_settings=SimpleNamespace(enabled=True,shadow=False,reference_source='planning')
        n.planning_settings=c.ns['PlanningSettings'](algorithm='road_centerline')
        n.distance_limiter=c.ns['RunDistanceLimiter'](0,.2)
        c.feed(n);c.start(n);n.control_has_run=True
        n.last_control_reference_deadline_s=n.test_clock.ros_ns/1e9+.2
        n.test_clock.advance(.1);c.feed(n);n.supervision_callback()
        n.test_clock.advance(.11);c.feed(n);n.supervision_callback()
        self.assertEqual(n.fsm_state,2);self.assertIn('expired',n.state_reason)
class StartBridgeChecks(unittest.TestCase):
    def trajectory(self,start=.95,y=0.,slope=0.):
        ref=f.ordered_centerline_reference(3.,speed=.2)
        ref['path']['samples_xy']=[[start+x,y+slope*x] for x,_ in ref['path']['samples_xy']]
        return f.adapt(ref,spacing_m=.05)
    def ctrl(self,**changes):
        values=dict(P['mpc']);values.update(changes)
        return m.Controller(m.Settings(**values),m.Vehicle(**P['vehicle']))
    def test_old_limit_reproduces_and_bounded_bridge_solves(self):
        traj=self.trajectory()
        old=self.ctrl(max_backward_extension_m=.6)
        with self.assertRaisesRegex(m.Stop,'origin_before_path_start'):old.step(traj,0.,0.,10.,True)
        new=self.ctrl();out=new.step(traj,0.,0.,10.,True)
        self.assertEqual(out['debug']['branch'],'solved')
        self.assertTrue(out['debug']['start_bridge_active'])
        self.assertAlmostEqual(out['debug']['start_extension_m'],.95)
        self.assertGreater(out['drive'],0)
    def test_gap_beyond_limit_is_not_unconditionally_accepted(self):
        with self.assertRaisesRegex(m.Stop,'origin_before_path_start'):
            self.ctrl().step(self.trajectory(start=1.25),0.,0.,10.,True)
    def test_bad_heading_and_lateral_offset_are_refused(self):
        for traj in (self.trajectory(y=.3),self.trajectory(slope=.5)):
            with self.assertRaisesRegex(m.Stop,'start_bridge_alignment'):
                self.ctrl().step(traj,0.,0.,10.,True)
    def test_metadata_curvature_spike_on_straight_geometry_is_not_a_false_stop(self):
        traj=self.trajectory()
        for point in traj['points'][:3]:point['curvature_1pm']=3.
        out=self.ctrl().step(traj,0.,0.,10.,True)
        self.assertEqual(out['debug']['branch'],'solved')
    def test_genuinely_bent_start_geometry_is_refused(self):
        import math
        traj=self.trajectory()
        for point in traj['points']:
            distance=point['x_m']-.95
            if distance<=.4:point['y_m']=.07*math.sin(2*math.pi*distance/.4)
        with self.assertRaisesRegex(m.Stop,'start_bridge_geometry'):
            self.ctrl().step(traj,0.,0.,10.,True)
    def test_real_path_end_and_expiry_remain_refused(self):
        traj=f.adapt(f.ordered_centerline_reference(1.,speed=.2))
        for point in traj['points']:point['x_m']-=2.
        with self.assertRaisesRegex(m.Stop,'origin_past_path_end'):
            self.ctrl().step(traj,0.,0.,10.,True)
        with self.assertRaisesRegex(m.Stop,'ref_expired_or_future'):
            self.ctrl().step(self.trajectory(),0.,0.,11.,True)
    def test_bridge_cannot_be_enabled_with_an_unbounded_distance(self):
        with self.assertRaises(ValueError):self.ctrl(max_backward_extension_m=2.)
    def test_mpc_rejection_keeps_actual_reason_in_state(self):
        n=c.make_node(mvp=True);c.feed(n);c.start(n)
        def reject(*args):raise m.Stop('ref_invalid:start_bridge_alignment')
        n.calculate_policy_actions=reject;n.run_policy_step()
        self.assertEqual(n.fsm_state,2)
        self.assertIn('start_bridge_alignment',n.state_reason)

if __name__=='__main__':unittest.main()
