import unittest,runpy,os,json,sys,subprocess
from pathlib import Path
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch
from test_user_mpc_bridge import adapter,ref,state,ROOT,PARAMS
from mvp21_policy.control import ControlSettings,PolicyStopRequest
from mvp21_policy.learned import FrozenPathPolicy,LearnedPolicySettings,PolicyExecutionMemory
from mvp21_policy.lattice import ShadowSettings
from mvp21_policy.local_mpc import MPCSettings,VehicleParamsSettings,mpc_debug_json
from mvp21_policy.mpc_adapter import MPCPolicyController,calculate_selected_mpc
from mvp21_policy.execution import PlannerExecutionManager

def node_fixture():
    api=runpy.run_path(str(ROOT/'tests/test_control.py'))
    api['ns'].update(MPCPolicyController=MPCPolicyController,calculate_selected_mpc=calculate_selected_mpc,mpc_debug_json=mpc_debug_json)
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
    node.shadow_generation=0
    node.shadow_worker=SimpleNamespace(status='computing')
    node.offers=[]
    node._shadow_offer=lambda:node.offers.append(node.estimation_output)
    return api,node

class DecoupledChecks(unittest.TestCase):
    def test_submission_happens_even_if_controller_rejects(self):
        api,node=node_fixture()
        def select(estimates):
            r=ref();r['timestamp_s']=estimates['state']['timestamp_s'];return r
        node._select_planner_reference=select
        api['feed'](node,center=0.,speed=.2);api['start'](node)
        def fail(*args,**kw):raise PolicyStopRequest('injected MPC rejection')
        with patch.dict(api['ns'],calculate_selected_mpc=fail):node.run_policy_step()
        self.assertEqual(len(node.offers),1)
        self.assertIn('injected MPC rejection',node.state_reason)

    def test_pending_planner_has_zero_startup_and_keeps_submitting(self):
        api,node=node_fixture()
        model=FrozenPathPolicy(LearnedPolicySettings(**PARAMS['learned_policy']),.5)
        node.learned_policy_memory=PolicyExecutionMemory(model.network.hidden,model.sha256)
        node._select_planner_reference=lambda estimates:None
        api['feed'](node,center=0.,speed=.2);api['start'](node)
        node.run_policy_step()
        self.assertEqual(len(node.offers),1)
        self.assertEqual(node.control_diagnostics['reason'],'waiting_for_first_planner_result')
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)
        self.assertFalse(node.control_has_run)
        self.assertEqual(node.learned_policy_memory.commits,0)

    def test_refresh_window_is_neutral_and_finite(self):
        api,node=node_fixture()
        api['feed'](node,center=0.,speed=.2);api['start'](node)
        node.control_has_run=True
        node.last_control_reference_deadline_s=10.05
        node.test_clock.advance(.06)
        problem=node._control_problem(node.test_clock.monotonic,node.test_clock.ros_ns)
        self.assertIsNone(problem)
        self.assertEqual(node.last_control_reference_deadline_s,10.05)
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)
        node.test_clock.advance(.11)
        self.assertIn('reference expired',node._control_problem(node.test_clock.monotonic,node.test_clock.ros_ns))

    def test_legacy_pid_expiry_still_latches_without_grace(self):
        api,node=node_fixture()
        api['feed'](node,center=0.,speed=.2);api['start'](node)
        node.control_settings=replace(node.control_settings,reference_refresh_grace_s=0.)
        node.control_has_run=True;node.last_control_reference_deadline_s=10.05
        node.test_clock.advance(.06)
        self.assertIn('reference expired',node._control_problem(node.test_clock.monotonic,node.test_clock.ros_ns))

    def test_settings_reject_unbounded_or_late_startup(self):
        with self.assertRaises(ValueError):ControlSettings(reference_refresh_grace_s=.3)
        with self.assertRaises(ValueError):ControlSettings(planner_startup_wait_s=1.)

    def test_entry_sets_blas_limits_before_loading_node(self):
        import mvp21_policy.__main__ as entry
        fake=SimpleNamespace(main=lambda:None)
        with patch.dict(os.environ,OPENBLAS_NUM_THREADS='32'):
            with patch.dict(sys.modules,{'mvp21_policy.node':fake}):
                with patch.object(sys,'argv',['policy']):entry.main()
            self.assertEqual(os.environ['OPENBLAS_NUM_THREADS'],'1')

if __name__=='__main__':unittest.main()
