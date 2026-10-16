"""Actual opt-in runtime adapter and same-tick baseline fallback, without ROS."""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

shadow = runpy.run_path(str(Path(__file__).with_name('test_lattice.py')))
ns, control = shadow['ns'], shadow['control']
Manager, Settings = ns['PlannerExecutionManager'], ns['PlannerControlSettings']
Cfg = ns['ShadowSettings']


def result(stamp=10., epoch=1, y=0., length=1., job_id=1, pose=(0.,0.,0.)):
    return dict(status='VALID', input_stamp_s=stamp, measurement_stamp_s=stamp-.1,
        job_id=job_id, execution_anchor=dict(pose=list(pose), odometry_epoch=epoch),
        selected=dict(path=[(length*i/40, y, 0.) for i in range(41)]))


def state(stamp=10., speed=.2):
    return dict(timestamp_s=stamp, frame_id='base_link', speed_valid=True,
                yaw_rate_valid=True, speed_mps=speed, yaw_rate_rps=0.)


def road(stamp=10.):
    return dict(timestamp_s=stamp, frame_id='base_link', time_aligned=True,
        boundary_source=dict(left='observed', right='observed'),
        left_boundary_xy=[(-.4,.5),(2.,.5)], right_boundary_xy=[(-.4,-.5),(2.,-.5)])


class GeometryExecutionChecks(unittest.TestCase):
    def select(self, manager, stamp=10., pose=(0.,0.,0.), epoch=1, boundaries=None, speed=.2):
        return manager.select(road(stamp) if boundaries is None else boundaries, state(stamp,speed),
                              pose, epoch, Cfg(), .35, .2, .05, .25)

    def test_default_is_baseline_and_switch_rejects_non_boolean(self):
        self.assertFalse(Settings().enabled)
        with self.assertRaises(ValueError):
            Settings(enabled='true')

    def test_actual_worker_retains_original_execution_anchor_and_mode(self):
        import json
        import subprocess
        import sys
        value=shadow['job']()
        value.update(shadow_pose=[0.,0.,0.],shadow_odometry_epoch=1,planner_control_enabled=True)
        process=subprocess.run([sys.executable,str(shadow['worker_source']),'--shadow-worker'],
            input=json.dumps(value)+'\n',text=True,capture_output=True,timeout=15)
        self.assertEqual(process.returncode,0,process.stderr)
        response=json.loads(process.stdout)
        self.assertEqual(response['status'],'VALID')
        self.assertEqual(response['execution_anchor'],{'pose':[0.,0.,0.],'odometry_epoch':1})
        self.assertFalse(response['shadow_only'])
        manager=Manager();manager.accept(response)
        self.assertIsNotNone(self.select(manager,stamp=100.2,pose=(.04,0.,0.)))

    def test_delayed_result_aligns_and_retains_source_age_in_fresh_execution_reference(self):
        manager=Manager(); manager.accept(result())
        ref=self.select(manager, stamp=13.2, pose=(.3,0.,0.))
        self.assertTrue(ref['valid'])
        self.assertEqual(ref['timestamp_s'],13.2)
        self.assertEqual(ref['original_input_stamp_s'],10.)
        self.assertEqual(ref['valid_for_s'],.25)
        self.assertAlmostEqual(manager.diagnostics['source_age_s'],3.2)
        self.assertAlmostEqual(manager.diagnostics['remaining_length_m'],.7)
        self.assertAlmostEqual(ref['path']['samples_xy'][0][0],0.)
        self.assertEqual(manager.diagnostics['mode'],'planner')

    def test_alignment_rotation_and_passed_arc_crop_preserve_future_negative_x(self):
        manager=Manager()
        value=result(length=2.)
        value['selected']['path']=[(0.,0.,0.),(.5,0.,0.),(1.,.3,1.),(.5,.7,2.),(-.3,.8,3.)]
        manager.accept(value)
        ref=self.select(manager,pose=(.1,0.,0.))
        self.assertIsNotNone(ref)
        self.assertTrue(any(p[0]<0 for p in ref['path']['samples_xy'][1:]))
        ref=self.select(manager,stamp=10.05,pose=(.2,0.,.2))
        self.assertIsNotNone(ref)
        expected=ns['_shadow_pose_points']([(-.3,.8)],(0,0,0),(.2,0,.2))[0]
        for a,b in zip(ref['path']['samples_xy'][-1],expected): self.assertAlmostEqual(a,b)

    def test_rolling_check_covers_preview_beyond_original_planner_prefix(self):
        manager=Manager(); manager.accept(result())
        boundaries=road()
        # At x=.6 it is outside the initial 0.02 m CG execution check plus body
        # front .25 m, but inside the .35 m preview plus body front.
        boundaries['left_boundary_xy']=[(.6,-.1),(.6,.1)]
        self.assertIsNone(self.select(manager,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['reason'],'known_boundary_collision')
        self.assertGreaterEqual(manager.diagnostics['check_length_m'],.35)

    def test_new_known_wall_rechecked_every_tick_but_inferred_side_is_not_hard_wall(self):
        manager=Manager(); manager.accept(result())
        self.assertIsNotNone(self.select(manager))
        boundaries=road(10.05)
        boundaries.update(right_boundary_xy=[(.3,-.1),(.3,.1)],
                          boundary_source={'left':'absent','right':'width_inferred'})
        self.assertIsNotNone(self.select(manager,stamp=10.05,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['known_boundary_sides'],[])
        boundaries['boundary_source']['right']='observed'
        self.assertIsNone(self.select(manager,stamp=10.05,boundaries=boundaries))

    def test_coverage_failure_and_recovery_without_fixed_age_timeout(self):
        manager=Manager(); manager.accept(result())
        self.assertIsNotNone(self.select(manager,stamp=15.))
        self.assertIsNone(self.select(manager,stamp=15.05,pose=(.7,0.,0.)))
        self.assertEqual(manager.diagnostics['reason'],'preview_coverage_exhausted')
        manager.accept(result(stamp=15.05,job_id=2,pose=(.7,0.,0.)))
        self.assertIsNotNone(self.select(manager,stamp=15.1,pose=(.71,0.,0.)))

    def test_epoch_change_malformed_and_planning_failures_cannot_reuse_old_success(self):
        for status in ('NO_ROAD','ALL_COLLIDE','WORKER_ERROR','TRANSPORT_ERROR'):
            manager=Manager(); manager.accept(result())
            manager.accept(dict(status=status,job_id=2))
            self.assertIsNone(self.select(manager))
            self.assertEqual(manager.diagnostics['reason'],'planner_'+status.lower())
        manager=Manager(); manager.accept(result())
        self.assertIsNone(self.select(manager,epoch=2))
        self.assertEqual(manager.diagnostics['reason'],'motion_epoch_changed')
        manager.accept({**result(),'execution_anchor':None})
        self.assertIsNone(self.select(manager))
        self.assertEqual(manager.diagnostics['reason'],'planner_invalid_execution_payload')

    def test_unaligned_known_boundary_falls_back_without_using_wrong_frame(self):
        manager=Manager(); manager.accept(result())
        boundaries=road(9.9)
        self.assertIsNone(self.select(manager,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['reason'],'known_boundary_alignment_unavailable')

    def test_geometric_connection_checks_actual_body_to_offset_path(self):
        manager=Manager(); manager.accept(result(y=.5))
        boundaries=road()
        boundaries.update(left_boundary_xy=[(-.1,.25),(.1,.25)],right_boundary_xy=None)
        self.assertIsNone(self.select(manager,boundaries=boundaries))
        self.assertEqual(manager.diagnostics['reason'],'known_boundary_collision')


class RuntimeExecutionChecks(unittest.TestCase):
    def node(self, enabled=True):
        helper=shadow['ShadowIntegrationChecks']()
        node=helper.node()
        helper.configure_shipped_baseline(node)
        node.planner_control_settings=Settings(enabled=enabled)
        node.planner_execution=Manager()
        node._last_execution_mode=None
        node.planner_execution_publisher=control['Recorder']()
        node._shadow_publish_path=lambda *_: None
        node._publish_planner_execution=node._publish_planner_execution
        helper.feed_shipped_baseline(node)
        control['start'](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state,3,node.logged_errors)
        return node,helper

    def tick(self,node,helper,speed=.2,yaw=0.):
        node.test_clock.advance(.05)
        helper.feed_shipped_baseline(node,speed=speed,yaw=yaw)
        node.run_policy_step()
        self.assertEqual(node.fsm_state,3,node.logged_errors)
        self.assertGreater(node.action_publisher.messages[-1].drive,0)

    def accept(self,node,y=0.):
        node.planner_execution.accept(result(stamp=node.test_clock.ros_ns/1e9,
            epoch=node.shadow_odometry_epoch,pose=node.shadow_pose,y=y))

    def test_default_actions_match_existing_baseline_even_with_valid_planner_result(self):
        baseline,helper=self.node(enabled=False)
        other,helper2=self.node(enabled=False)
        self.accept(other,y=.15)
        for _ in range(24):
            self.tick(baseline,helper); self.tick(other,helper2)
            self.assertEqual(vars(baseline.action_publisher.messages[-1]),
                             vars(other.action_publisher.messages[-1]))
            self.assertEqual(baseline._trajectory_reference,other._trajectory_reference)

    def test_pid_receives_aligned_planner_path_then_failure_falls_back_same_tick(self):
        node,helper=self.node()
        self.assertEqual(node.planner_execution.diagnostics['mode'],'baseline')
        self.accept(node,y=.03)
        self.tick(node,helper)
        self.assertEqual(node._trajectory_reference['planner_version'],'lattice_execution')
        self.assertEqual(node.planner_execution.diagnostics['mode'],'planner')
        self.assertGreater(node.control_diagnostics['preview_x_m'],0.)
        node.planner_execution.accept(dict(status='ALL_COLLIDE',job_id=2))
        self.tick(node,helper)
        self.assertEqual(node._trajectory_reference['planner_version'],'road_centerline')
        self.assertEqual(node.planner_execution.diagnostics['reason'],'planner_all_collide')
        self.accept(node)
        self.tick(node,helper)
        self.assertEqual(node._trajectory_reference['planner_version'],'lattice_execution')

    def test_adapter_exception_and_preview_exhaustion_fall_back_without_state_two(self):
        node,helper=self.node(); self.accept(node)
        with patch.object(node.planner_execution,'select',side_effect=RuntimeError('injected')):
            self.tick(node,helper)
        self.assertIn('execution_exception',node.planner_execution.diagnostics['reason'])
        node.planner_execution.accept(result(stamp=node.test_clock.ros_ns/1e9,length=.1,
            epoch=node.shadow_odometry_epoch,pose=node.shadow_pose))
        self.tick(node,helper)
        self.assertEqual(node.planner_execution.diagnostics['reason'],'preview_coverage_exhausted')
        self.assertEqual(node._trajectory_reference['planner_version'],'road_centerline')

    def test_controller_rejection_restores_pid_and_runs_baseline_same_cycle(self):
        node,helper=self.node(); self.accept(node)
        controller_type=type(node.controller)
        original=controller_type.calculate
        def injected(controller,ref,*args):
            if ref.get('planner_version')=='lattice_execution':
                controller.clear()
                return 0.,0.,dict(valid=False,stop_requested=True,reason='injected')
            return original(controller,ref,*args)
        with patch.object(controller_type,'calculate',injected):
            self.tick(node,helper)
        self.assertEqual(node._trajectory_reference['planner_version'],'road_centerline')
        self.assertEqual(node.planner_execution.diagnostics['reason'],'planner_control_rejected: injected')

    def test_planner_controller_exception_also_falls_back_and_diagnostics_describe_executed_action(self):
        node,helper=self.node(); self.accept(node)
        controller_type=type(node.controller)
        original=controller_type.calculate
        def injected(controller,ref,*args):
            if ref.get('planner_version')=='lattice_execution':
                raise RuntimeError('injected_exception')
            return original(controller,ref,*args)
        with patch.object(controller_type,'calculate',injected):
            self.tick(node,helper)
        import json
        data=json.loads(node.planner_execution_publisher.messages[-1].data)
        self.assertEqual(data['mode'],'baseline')
        self.assertIn('planner_controller_exception',data['reason'])
        self.assertEqual(data['reference_version'],'road_centerline')
        self.assertAlmostEqual(data['steering_action'],node.action_publisher.messages[-1].steer)

    def test_result_poll_passes_input_anchor_to_execution_and_records_requested_source(self):
        node,helper=self.node()
        value=result(stamp=node.test_clock.ros_ns/1e9,
            epoch=node.shadow_odometry_epoch,pose=node.shadow_pose)
        value.update(generation=node.shadow_generation,check_length_m=.22,reference=[],
                     submitted_monotonic_s=node.test_clock.monotonic)
        node.shadow_worker.poll=lambda:value
        node._shadow_poll()
        self.assertIsNotNone(node.planner_execution.active)
        self.tick(node,helper)
        self.assertEqual(node._trajectory_reference['planner_version'],'lattice_execution')
        offered=[];node.shadow_worker.submit=offered.append
        node.shadow_last_submit_s=-math.inf
        node._shadow_offer()
        self.assertTrue(offered[-1]['planner_control_enabled'])
        self.assertFalse(offered[-1]['shadow_only'])
        self.assertIn('frame_id',offered[-1]['road_history'][-1]['road'])
        self.assertIn('speed_mps',offered[-1]['road_history'][-1]['state'])

    def test_switch_retains_operator_stop_sensor_watchdog_and_generation_fence(self):
        node,helper=self.node(); self.accept(node)
        self.tick(node,helper)
        old=result(stamp=node.test_clock.ros_ns/1e9,epoch=node.shadow_odometry_epoch,pose=node.shadow_pose)
        old.update(generation=node.shadow_generation-1)
        before=deepcopy(node.planner_execution.active)
        node.shadow_worker.poll=lambda:old
        node._shadow_poll()
        self.assertEqual(node.planner_execution.active,before)
        node.fsm_transition_request_callback(SimpleNamespace(data=2))
        self.assertEqual(node.fsm_state,2)
        self.assertIsNone(node.planner_execution.active)
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)
        node,helper=self.node(); self.accept(node)
        node.test_clock.advance(2.)
        node.supervision_callback()
        self.assertEqual(node.fsm_state,2)
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)

    def test_transport_failure_and_failed_diagnostic_publication_leave_baseline_driving(self):
        node,helper=self.node(); self.accept(node)
        node.planner_execution.accept({'status':'TRANSPORT_ERROR'})
        node.planner_execution_publisher.publish=lambda _: (_ for _ in ()).throw(RuntimeError('transport'))
        self.tick(node,helper)
        self.assertEqual(node._trajectory_reference['planner_version'],'road_centerline')


if __name__=='__main__':
    unittest.main()
