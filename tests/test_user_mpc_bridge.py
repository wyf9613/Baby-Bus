from pathlib import Path
import sys,json,yaml,unittest
import runpy
from dataclasses import replace
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from mvp21_policy.local_mpc import MPCSettings,VehicleParamsSettings,MPCController,reference_trajectory_from_planning
from mvp21_policy.mpc_adapter import MPCPolicyController,calculate_selected_mpc
from mvp21_policy.control import ControlSettings
from mvp21_policy.lattice import ShadowSettings,ShadowLatticePlanner
from mvp21_policy.learned import LearnedPolicySettings,FrozenPathPolicy,PolicyExecutionMemory
from mvp21_policy.execution import PlannerExecutionManager

PARAMS=next(iter(yaml.safe_load((ROOT/'config/ai4r_policy.yaml').read_text()).values()))['ros__parameters']

def state():
    return dict(valid=True,speed_valid=True,yaw_rate_valid=True,speed_mps=.2,yaw_rate_rps=0.,timestamp_s=10.,frame_id='base_link')
def ref():
    return dict(schema_version='planning_reference_v1',valid=True,status='VALID',reason='test',
        frame_id='base_link',vehicle_reference_point='cg_ground_projection',timestamp_s=10.,valid_for_s=.3,
        planner_version='lattice_execution',trajectory_id=5,target_speed_mps=.5,stop_requested=False,
        simulation_only=False,path=dict(type='ORDERED_CENTERLINE',independent_variable='s_m',
            samples_xy=[(0.,0.),(.5,0.),(1.,0.),(1.5,0.),(2.,0.)],range=[0.,2.],preview_m=.35))
def adapter(shadow=True):
    return MPCPolicyController(ControlSettings(**PARAMS['control']),
        replace(MPCSettings(**PARAMS['mpc']),shadow=shadow),
        VehicleParamsSettings(**PARAMS['mpc_vehicle']),ShadowSettings(**PARAMS['shadow']))

class MPCBridgeChecks(unittest.TestCase):
    def test_speed_condition_and_model_checksum(self):
        model=FrozenPathPolicy(LearnedPolicySettings(**PARAMS['learned_policy']),.5)
        self.assertEqual(model.sha256,'a3116bb7bc4bb46ccb6e208fa3de628eca940820810943409c3157e5b12aa66b')
        with self.assertRaisesRegex(ValueError,'cruise-speed'):
            FrozenPathPolicy(LearnedPolicySettings(**PARAMS['learned_policy']),.3)

    def test_planner_body_matches_user_model(self):
        model=PARAMS['mpc_vehicle'];planner=PARAMS['shadow']
        self.assertEqual(planner['body_front_m'],model['body_front_extent_from_cg_m'])
        self.assertEqual(planner['body_rear_m'],model['body_rear_extent_from_cg_m'])
        self.assertEqual(planner['body_width_m'],model['body_width_m'])

    def test_explicit_half_meter_gate_only(self):
        c=adapter(False).mpc_controller;self.assertIsNone(c.gate_problem())
        cfg=replace(c.cfg,learned_planner_speed_match_enabled=False)
        self.assertIsNotNone(MPCController(cfg,c.vehicle).gate_problem())

    def test_prepare_preserves_identity_and_deadline_without_stepping(self):
        a=adapter();r=ref();before={k:deepcopy(getattr(a.mpc_controller,k)) for k in ('clock_s','feedback_integral')}
        with patch.object(a.mpc_controller,'step',side_effect=AssertionError('admission stepped')):
            t,reason=a.prepare(r,state(),10.)
        self.assertIsNone(reason);self.assertEqual(t['path_id'],5)
        self.assertEqual(t['timestamp_s'],10.);self.assertEqual(t['valid_for_s'],.3)
        self.assertEqual(a.mpc_controller.clock_s,before['clock_s'])
        self.assertEqual(a.mpc_controller.feedback_integral,before['feedback_integral'])

    def test_bad_frames_and_expiry_cannot_reach_mpc(self):
        a=adapter()
        for key,value in [('frame_id','map'),('timestamp_s',9.),('stop_requested',True)]:
            r=ref();r[key]=value
            with self.subTest(key=key):self.assertIsNotNone(a.prepare(r,state(),10.)[1])
        self.assertIsNotNone(a.prepare(ref(),state(),10.4)[1])

    def test_fallback_before_step_and_once_only(self):
        a=adapter();bad=ref();bad['frame_id']='map';execution=SimpleNamespace(diagnostics={'mode':'planner'})
        good=ref();good['planner_version']='road_centerline'
        result={'drive':.31,'steer':.1,'debug':{'branch':'solved'}}
        with patch.object(a.mpc_controller,'step',return_value=result) as step:
            got=calculate_selected_mpc(a,good,bad,state(),{},10.,0.,execution)
        self.assertEqual(step.call_count,1);self.assertEqual(got[-1]['planner_version'],'road_centerline')
        self.assertEqual(execution.diagnostics['mode'],'baseline')

    def test_no_double_step_after_solver_failure(self):
        a=adapter(False);e=SimpleNamespace(diagnostics={'mode':'planner'})
        from mvp21_policy.local_mpc import MPCStop
        with patch.object(a.mpc_controller,'step',side_effect=MPCStop('over_budget')) as step:
            got=calculate_selected_mpc(a,ref(),ref(),state(),{},10.,0.,e)
        self.assertEqual(step.call_count,1);self.assertTrue(got[3]['stop_requested'])
        self.assertFalse(got[3]['mpc_reference_executed'])

    def test_actual_solver_matches_direct_user_controller(self):
        a=adapter(True);r=ref();t,reason=a.prepare(r,state(),10.);self.assertIsNone(reason)
        direct=MPCController(a.mpc_settings,a.mpc_controller.vehicle)
        out=direct.step(deepcopy(t),.2,0.,10.,True)
        drive,steer,d=a.execute(r,t,state(),10.,0.)
        self.assertEqual(d['mpc']['branch'],out['debug']['branch'])
        self.assertAlmostEqual(d['mpc']['candidate_steer_action'],out['debug']['candidate_steer_action'])
        self.assertEqual((drive,steer),(out['drive'],out['steer']))
        self.assertFalse(d['mpc_reference_executed'])

    def test_execution_uses_user_pi_and_steering_sign(self):
        a=adapter(False);r=ref()
        r['path']['samples_xy']=[(0.,0.),(.5,.05),(1.,.10),(1.5,.15),(2.,.20)]
        from mvp21_policy.geometry import _path_lengths
        r['path']['range']=[0.,_path_lengths(r['path']['samples_xy'])[-1]]
        t,reason=a.prepare(r,state(),10.);self.assertIsNone(reason)
        drive,steer,d=a.execute(r,t,state(),10.,0.)
        self.assertTrue(d['valid'],d);self.assertTrue(d['mpc_reference_executed'],d)
        self.assertGreater(drive,0.);self.assertLessEqual(drive,PARAMS['mpc']['speed_feedback_drive_max'])
        self.assertLess(steer,0.) # user gain negative: normalized negative turns left.

    def test_new_rnn_to_execution_to_mpc_end_to_end(self):
        st=state();road=dict(valid=True,time_aligned=True,timestamp_s=10.,frame_id='base_link',
            centerline_xy=[(0.,0.),(1.,0.),(2.,0.)],left_boundary_xy=[(0.,.7),(1.,.7),(2.,.7)],
            right_boundary_xy=[(0.,-.7),(1.,-.7),(2.,-.7)],
            boundary_source=dict(left='observed',right='observed'))
        model=FrozenPathPolicy(LearnedPolicySettings(**PARAMS['learned_policy']),.5)
        memory=PolicyExecutionMemory(model.network.hidden,model.sha256)
        job=dict(settings=vars(ShadowSettings(enabled=True,sample_count=8)),road=road,state=st,
            generation=1,job_id=7,stamp_s=10.,shadow_pose=[0.,0.,0.],shadow_odometry_epoch=1,
            preview_m=.35,submitted_monotonic_s=1.,planner_control_enabled=True)
        result=ShadowLatticePlanner(ShadowSettings(**job['settings'])).plan(job)
        observation=dict(frame_id='base_link',timestamp_s=10.,estimation=dict(state=st,road=road,
            obstacles=dict(available=True,points_xyz_m=[])))
        result=model.select(result,observation,memory.snapshot())
        self.assertEqual(result['policy_selection']['status'],'SELECTED')
        result.update(job_id=7,generation=1,input_stamp_s=10.,measurement_stamp_s=10.,
            execution_anchor=dict(pose=[0.,0.,0.],odometry_epoch=1))
        e=PlannerExecutionManager('base_link');self.assertTrue(memory.validate_result(result));e.accept(result)
        r=e.select(road,st,[0.,0.,0.],1,ShadowSettings(**job['settings']),.35,.5,.05,.3)
        self.assertIsNotNone(r,e.diagnostics)
        a=adapter(True);a.estimates=dict(road=road,state=st)
        t,reason=a.prepare(r,st,10.)
        self.assertIsNone(reason)
        memory.stage(result,True)
        drive,steer,d=a.execute(r,t,st,10.,0.)
        self.assertTrue(d['valid'],d);self.assertEqual((drive,steer),(0.,0.))
        self.assertEqual(memory.revision,0) # shadow never commits recurrent state.

    def test_actual_node_cycle_calls_mpc_and_publishes_shadow_zero(self):
        api=runpy.run_path(str(ROOT/'tests/test_control.py'))
        import mvp21_policy.mpc_adapter as bridge
        from mvp21_policy.local_mpc import mpc_debug_json
        api['ns'].update(MPCPolicyController=bridge.MPCPolicyController,
            calculate_selected_mpc=bridge.calculate_selected_mpc,mpc_debug_json=mpc_debug_json)
        node=api['make_node'](simulated=False,mvp=True)
        node.mpc_settings=replace(MPCSettings(**PARAMS['mpc']),shadow=True)
        node.mpc_vehicle_settings=VehicleParamsSettings(**PARAMS['mpc_vehicle'])
        node.shadow_settings=ShadowSettings(**PARAMS['shadow'])
        node.planning_settings=__import__('mvp21_policy.planning',fromlist=['PlanningSettings']).PlanningSettings(**PARAMS['planning'])
        node.control_settings=ControlSettings(**PARAMS['control'])
        node.estimation_settings=__import__('mvp21_policy.estimation',fromlist=['EstimationSettings']).EstimationSettings(**PARAMS['estimation'])
        node.motion_history=__import__('mvp21_policy.motion',fromlist=['EstimationMotionHistory']).EstimationMotionHistory(node.estimation_settings)
        node.controller=adapter(True)
        node.planner_control_settings=SimpleNamespace(enabled=True)
        node.planner_execution=PlannerExecutionManager('base_link')
        node.mpc_debug_publisher=api['Recorder']()
        def select(estimates):
            r=ref();r['timestamp_s']=estimates['state']['timestamp_s']
            node.planner_execution.diagnostics=dict(mode='planner',reason='planner_tracking')
            return r
        node._select_planner_reference=select
        api['feed'](node,center=0.,speed=.2);api['start'](node)
        node.run_policy_step()
        self.assertFalse(node.logged_errors,node.logged_errors)
        self.assertTrue(node.control_diagnostics['valid'],node.control_diagnostics)
        self.assertEqual(node.control_diagnostics['controller'],'user_mpc_speed_feedback')
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)
        self.assertTrue(node.mpc_debug_publisher.messages)

if __name__=='__main__':unittest.main()
