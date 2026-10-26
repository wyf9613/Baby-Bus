"""Portable detector and actual modular-node lifecycle checks; no devices."""
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

api = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
ns = api["ns"]
Settings, detect = ns["LidarObstacleSettings"], ns["detect_lidar_obstacles"]


def scan_fixture(hits=None, yaw=0.0, origin=(0.0, 0.0, 0.12), count=720):
    """Synthetic full-ring returns, not physical measurements or a vehicle model."""
    ranges = [4.0]*count
    for index, distance in (hits or {}).items():
        ranges[index] = distance
    increment = 2*math.pi/count
    axes = ((math.cos(yaw), math.sin(yaw), 0.0), (-math.sin(yaw), math.cos(yaw), 0.0))
    points, indices = [], []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and 0.05 <= r <= 10:
            points.append((origin[0]+r*math.cos(yaw+i*increment),
                           origin[1]+r*math.sin(yaw+i*increment), origin[2]))
            indices.append(i)
    raw = {"ranges": ranges, "intensities": [], "angle_min": 0.0,
           "angle_max": (count-1)*increment, "angle_increment": increment,
           "time_increment": 0.0, "scan_time": 0.1, "frame_id": "laser",
           "range_min": 0.05, "range_max": 10.0}
    cart = {"points_xyz": points, "scan_indices": indices, "frame_id": "base_link",
            "ray_origin_xyz_m": origin, "ray_axes_xyz": axes}
    return raw, cart


class DetectorChecks(unittest.TestCase):
    def detect(self, hits=None, **kwargs):
        return detect(*scan_fixture(hits, **kwargs), Settings(enabled=True), 10_000_000_000)

    def test_finite_clear_scene_preserves_source_and_bounds(self):
        raw, cart = scan_fixture()
        before = deepcopy((raw, cart))
        result = detect(raw, cart, Settings(), 10_000_000_000)
        self.assertEqual(result["status"], "CLEAR")
        self.assertEqual(result["measurement_timestamp_s"], 10.0)
        self.assertEqual(result["roi_xy_m"], ((0.297, 1.297), (-0.145, 0.145)))
        self.assertEqual(result["known_fraction"], 1.0)
        self.assertEqual((raw, cart), before)

    def test_front_cluster_has_geometry_and_original_ray_indices(self):
        result = self.detect({0: 0.8, 1: 0.8, 2: 0.8})
        self.assertEqual(result["status"], "OBSTACLE")
        cluster = result["clusters"][0]
        self.assertEqual(cluster["scan_indices"], [0, 1, 2])
        self.assertEqual(cluster["point_count"], 3)
        self.assertAlmostEqual(cluster["bbox_xy_m"][0], 0.8*math.cos(math.pi/180))
        self.assertAlmostEqual(result["nearest_bumper_distance_m"], cluster["bbox_xy_m"][0]-0.297)

    def test_side_and_rear_hits_are_not_front_obstacles(self):
        result = self.detect({180: 0.6, 181: 0.6, 360: 0.6, 361: 0.6})
        self.assertEqual(result["status"], "CLEAR")
        self.assertEqual(result["points_xyz_m"], [])

    def test_single_hit_is_unknown_not_removed_as_free_space(self):
        result = self.detect({1: 0.8})
        self.assertEqual(result["status"], "UNKNOWN")
        self.assertEqual(result["reason"], "sparse_hit_in_stop_roi")
        self.assertEqual(len(result["candidate_clusters"]), 1)

    def test_missing_ray_splits_clusters(self):
        result = self.detect({0: 0.8, 1: math.nan, 2: 0.8})
        self.assertEqual(result["clusters"], [])
        self.assertEqual(len(result["candidate_clusters"]), 2)
        self.assertEqual(result["status"], "UNKNOWN")

    def test_jump_splits_even_adjacent_rays(self):
        result = self.detect({0: 0.5, 1: 1.1})
        self.assertEqual(len(result["candidate_clusters"]), 2)

    def test_no_returns_are_unknown_not_zero_distance_or_clear(self):
        for value in (math.nan, math.inf, -math.inf, 0.01, 11.0):
            with self.subTest(value=value):
                result = self.detect(dict.fromkeys(range(720), value))
                self.assertEqual(result["status"], "UNKNOWN")
                self.assertEqual(result["roi_ray_count"] > 3, True)
                self.assertEqual(result["points_xyz_m"], [])

    def test_rear_returns_do_not_hide_forward_sensor_blindness(self):
        hits = {i: math.nan for i in list(range(70))+list(range(650, 720))}
        self.assertEqual(self.detect(hits)["reason"], "insufficient_roi_returns")

    def test_self_occlusion_does_not_count_as_clear_roi(self):
        hits = {i: 0.1 for i in list(range(70))+list(range(650, 720))}
        result = self.detect(hits)
        self.assertEqual(result["status"], "UNKNOWN")
        self.assertEqual(result["points_xyz_m"], [])

    def test_real_mount_yaw_moves_raw_rear_angles_to_vehicle_front(self):
        result = self.detect({359: 0.6, 360: 0.6, 361: 0.6}, yaw=math.pi, origin=(0.2, 0.0, 0.12))
        self.assertEqual(result["status"], "OBSTACLE")
        self.assertEqual(result["scan_indices"], [359, 360, 361])
        self.assertAlmostEqual(result["nearest_bumper_distance_m"], 0.2+0.6*math.cos(math.pi/360)-0.297)

    def test_front_and_side_boundaries_inclusive_outside_excluded(self):
        raw, cart = scan_fixture()
        cart["points_xyz"], cart["scan_indices"] = [(0.297, 0.145, 0.12), (1.297, -0.145, 0.12),
                                                  (1.297001, 0.0, 0.12), (0.8, 0.145001, 0.12)], [0, 1, 2, 3]
        result = detect(raw, cart, Settings(min_cluster_points=1))
        self.assertEqual(result["scan_indices"], [0, 1])
        self.assertEqual(result["nearest_bumper_distance_m"], 0.0)

    def test_partial_scan_wrong_frame_tilt_and_invalid_shape_are_unknown(self):
        for mode in ("partial", "frame", "tilt", "indices", "no_mount", "invalid_point"):
            raw, cart = scan_fixture()
            if mode == "partial":
                raw["ranges"] = raw["ranges"][:360]
            elif mode == "frame":
                cart["frame_id"] = "laser"
            elif mode == "tilt":
                cart["ray_axes_xyz"] = ((0.8, 0.0, 0.6), (0.0, 1.0, 0.0))
            elif mode == "indices":
                cart["scan_indices"][1] = 0
            elif mode == "no_mount":
                cart.pop("ray_origin_xyz_m")
            else:
                cart["points_xyz"][1] = (math.nan, 0.0, 0.0)
            with self.subTest(mode=mode):
                self.assertEqual(detect(raw, cart, Settings())["status"], "UNKNOWN")

    def test_invalid_configuration_rejected(self):
        for values in ({"enabled": 1}, {"body_width_m": 0}, {"stop_distance_m": math.inf},
                       {"side_margin_m": -0.1}, {"min_cluster_points": True},
                       {"min_cluster_points": 0}, {"min_known_fraction": 0},
                       {"range_min_m": -0.1}, {"allow_no_return_clear": 1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Settings(**values)


class AggressiveProfileChecks(unittest.TestCase):
    def detect(self, hits=None, **settings):
        return detect(*scan_fixture(hits, yaw=math.pi, origin=(0.2,0.0,0.12)),
                      Settings(allow_no_return_clear=True, **settings))

    def test_sensor_minimum_02_does_not_reject_visible_far_returns_due_to_blind_prefix(self):
        raw, cart = scan_fixture(yaw=math.pi,origin=(0.2,0.0,0.12))
        raw['range_min'] = .2
        result = detect(raw,cart,Settings())
        self.assertEqual(result['status'],'CLEAR')
        self.assertEqual(result['effective_range_min_m'],.2)
        self.assertEqual(result['known_fraction'],1.)
        self.assertGreater(result['blind_roi_ray_count'],0)

    def test_positive_infinity_only_clears_with_explicit_assumption(self):
        result=self.detect(dict.fromkeys(range(720),math.inf))
        self.assertEqual(result['status'],'CLEAR')
        self.assertEqual(result['known_fraction'],0.)
        self.assertEqual(result['usable_fraction'],1.)
        self.assertGreater(result['assumed_clear_ray_count'],0)
        self.assertEqual(result['reason'],'no_hits_in_observable_roi_assumed_clear')
        self.assertEqual(detect(*scan_fixture(dict.fromkeys(range(720),math.inf)),Settings())['status'],'UNKNOWN')

    def test_nan_negative_infinity_zero_and_invalid_returns_never_count_as_clear(self):
        for value in (math.nan,-math.inf,0.,.01,11.):
            with self.subTest(value=value):
                self.assertEqual(self.detect(dict.fromkeys(range(720),value))['status'],'UNKNOWN')

    def test_inf_background_does_not_hide_real_cluster_or_sparse_hit(self):
        hits=dict.fromkeys(range(720),math.inf)
        hits.update({359:.6,360:.6,361:.6})
        self.assertEqual(self.detect(hits)['status'],'OBSTACLE')
        hits[359]=hits[361]=math.inf
        self.assertEqual(self.detect(hits)['reason'],'sparse_hit_in_stop_roi')

    def test_valid_near_obstacle_is_not_deleted_to_enforce_policy_lower_bound(self):
        # Sensor metadata admits .15 m, even though policy visibility starts at .2.
        result=self.detect({359:.15,360:.15,361:.15})
        self.assertEqual(result['status'],'OBSTACLE')
        self.assertLess(result['nearest_bumper_distance_m'],.06)

    def test_reduced_margin_excludes_old_padding_but_keeps_new_boundary(self):
        raw,cart=scan_fixture()
        cart['points_xyz'],cart['scan_indices']=[(.8,.16,.12),(.8,.16,.12)],[0,1]
        self.assertEqual(detect(raw,cart,Settings())['status'],'CLEAR')
        self.assertEqual(detect(raw,cart,Settings(side_margin_m=.05))['status'],'OBSTACLE')

    def test_insufficient_sensor_reach_is_not_assumed_clear(self):
        raw,cart=scan_fixture(dict.fromkeys(range(720),math.inf))
        raw['range_max']=.1
        self.assertEqual(detect(raw,cart,Settings(allow_no_return_clear=True))['reason'],
                         'range_min_exceeds_sensor_max')


class StopFeedbackChecks(unittest.TestCase):
    def monitor(self):
        value = ns['StopFeedback']()
        value.reset(2,1.0)
        return value

    def test_zero_effort_does_not_mean_stopped_when_speed_is_nonzero(self):
        result=self.monitor().update(2,.4,1.05,1.05,True)
        self.assertEqual(result['phase'],'STOPPING')
        self.assertFalse(result['confirmed'])

    def test_new_zero_samples_spanning_hold_confirm_exact_zero(self):
        value=self.monitor()
        self.assertEqual(value.update(2,0.,1.05,1.05,True)['phase'],'STOPPING')
        result=value.update(2,0.,1.25,1.25,True)
        self.assertEqual(result['phase'],'STOPPED')
        self.assertTrue(result['confirmed'])
        self.assertEqual(result['speed_mps'],0.)

    def test_cached_zero_does_not_accumulate_hold(self):
        value=self.monitor()
        value.update(2,0.,1.05,1.05,True)
        result=value.update(2,0.,1.05,1.4,True)
        self.assertEqual(result['phase'],'STOPPING')
        self.assertEqual(result['zero_samples'],1)

    def test_pre_stop_zero_is_not_new_stop_confirmation(self):
        result=self.monitor().update(2,0.,.99,1.2,True)
        self.assertEqual(result['phase'],'STOPPING')
        self.assertEqual(result['zero_samples'],0)

    def test_small_nonzero_speed_is_not_rounded_to_zero(self):
        value=self.monitor()
        value.update(2,0.,1.05,1.05,True)
        self.assertEqual(value.update(2,.001,1.3,1.3,True)['phase'],'STOPPING')

    def test_invalid_or_stale_feedback_is_unknown_not_zero(self):
        for speed,fresh in ((None,False),(0.,False),(math.nan,True),(-.1,True)):
            with self.subTest(speed=speed,fresh=fresh):
                result=self.monitor().update(2,speed,1.1,1.1,fresh)
                self.assertEqual(result['phase'],'UNKNOWN')
                self.assertIsNone(result['speed_mps'])

    def test_motion_after_confirmation_revokes_stopped_report(self):
        value=self.monitor()
        value.update(2,0.,1.05,1.05,True);value.update(2,0.,1.3,1.3,True)
        self.assertEqual(value.update(2,.1,1.35,1.35,True)['phase'],'STOPPING')

    def test_state_one_and_driving_are_not_stop_confirmations(self):
        value=self.monitor()
        self.assertEqual(value.update(1,0.,1.1,1.1,True)['phase'],'NOT_PUBLISHING')
        self.assertEqual(value.update(3,0.,1.2,1.2,True)['phase'],'NOT_REQUESTED')

    def test_explicit_new_stop_resets_old_confirmation(self):
        value=self.monitor()
        value.update(2,0.,1.05,1.05,True);value.update(2,0.,1.3,1.3,True)
        value.reset(2,1.4)
        self.assertEqual(value.update(2,0.,1.3,1.45,True)['phase'],'STOPPING')

    def test_long_sample_gap_or_clock_reset_restarts_hold(self):
        value=self.monitor()
        value.update(2,0.,1.05,1.05,True)
        self.assertEqual(value.update(2,0.,1.6,1.6,True)['phase'],'STOPPING')
        self.assertEqual(value.update(2,0.,.5,.5,True)['phase'],'STOPPING')

    def test_invalid_hold_rejected(self):
        with self.assertRaises(ValueError):
            ns['StopFeedback'](0.)


def policy_fixture():
    node = api["make_node"](simulated=False, mvp=True)
    node.control_settings.enabled = False  # Isolate guard, not road/planning validity.
    node.lidar_obstacle_settings = Settings(enabled=True)
    node.lidar_obstacle_output = None
    node._lidar_obstacle_stamp_ns = None
    node._lidar_obstacle_publication_key = None
    node.lidar_obstacle_publisher = api["Recorder"]()
    node.calculate_policy_actions = lambda *_: (0.3, 0.1, None, None, None)
    return node


def feed_lidar(node, hits=None):
    api["feed"](node)
    raw, cart = scan_fixture(hits)
    for name, value in (("lidar_scan", raw), ("lidar_cartesian", cart)):
        node._store(name, value, node.test_clock.ros_ns, node.test_clock.monotonic, node.test_clock.ros_ns)


class PolicyChecks(unittest.TestCase):
    def feedback_node(self):
        node=policy_fixture()
        node.stop_feedback=ns['StopFeedback']()
        node.stop_status_publisher=api['Recorder']()
        node.stop_state_publisher=api['Recorder']()
        node._last_stop_feedback_phase=None
        node._last_stop_feedback_publish_s=-math.inf
        return node

    def test_obstacle_stop_is_reported_complete_only_after_new_zero_speed_samples(self):
        node=self.feedback_node()
        feed_lidar(node);api['start'](node);node.run_policy_step()
        node.test_clock.advance(.05);feed_lidar(node,{0:.8,1:.8})
        node._store('wheel_speed',.4,None,node.test_clock.monotonic,node.test_clock.ros_ns)
        node.supervision_callback()
        self.assertEqual(node.fsm_state,2)
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)
        self.assertEqual(json.loads(node.stop_status_publisher.messages[-1].data)['phase'],'STOPPING')
        for gap in (.05,.2):
            node.test_clock.advance(gap)
            node._store('wheel_speed',0.,None,node.test_clock.monotonic,node.test_clock.ros_ns)
            node.supervision_callback()
        result=json.loads(node.stop_status_publisher.messages[-1].data)
        self.assertEqual(result['phase'],'STOPPED')
        self.assertEqual(result['speed_mps'],0.)
        self.assertEqual(node.fsm_state,2)  # Reporting does not resume driving.

    def test_missing_wheel_feedback_never_reports_stopped(self):
        node=self.feedback_node()
        node.supervision_callback()
        self.assertEqual(node.stop_state_publisher.messages[-1].data,3)
        self.assertEqual(json.loads(node.stop_status_publisher.messages[-1].data)['phase'],'UNKNOWN')

    def test_aggressive_inf_scan_can_start_but_stale_data_still_latches_zero(self):
        node = policy_fixture()
        node.lidar_obstacle_settings = Settings(enabled=True, allow_no_return_clear=True)
        api['feed'](node)
        raw,cart = scan_fixture(dict.fromkeys(range(720),math.inf))
        raw['range_min'] = .2
        for name,value in (('lidar_scan',raw),('lidar_cartesian',cart)):
            node._store(name,value,node.test_clock.ros_ns,node.test_clock.monotonic,node.test_clock.ros_ns)
        api['start'](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state,3)
        self.assertEqual(node.action_publisher.messages[-1].drive,.3)
        self.assertEqual(node.lidar_obstacle_output['known_fraction'],0.)
        self.assertEqual(node.lidar_obstacle_output['usable_fraction'],1.)
        node.test_clock.advance(.5)
        node.supervision_callback()
        self.assertEqual(node.fsm_state,2)
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)

    def test_start_refused_when_missing_or_obstructed(self):
        node = policy_fixture()
        api["feed"](node)
        api["start"](node)
        self.assertEqual(node.fsm_state, 2)
        feed_lidar(node, {0: 0.8, 1: 0.8})
        api["start"](node)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Lidar stop", node.state_reason)

    def test_supervisor_stops_without_control_callback_and_requires_explicit_resume(self):
        node = policy_fixture()
        feed_lidar(node)
        api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.3)
        node.test_clock.advance(0.05)
        feed_lidar(node, {0: 0.8, 1: 0.8})
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)
        self.assertEqual(node.action_publisher.messages[-1].steer, 0.0)
        self.assertEqual(node.pan_publisher.messages, [])
        node.test_clock.advance(0.05)
        feed_lidar(node)
        node.supervision_callback()
        node.run_policy_step()
        self.assertEqual(node.lidar_obstacle_output["status"], "CLEAR")
        self.assertEqual(node.fsm_state, 2)
        api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.3)

    def test_new_scan_processed_once_and_original_timestamp_retained(self):
        node = policy_fixture()
        feed_lidar(node)
        node.supervision_callback()
        result = node.lidar_obstacle_output
        node.test_clock.advance(0.1)
        node.supervision_callback()
        self.assertIs(node.lidar_obstacle_output, result)
        self.assertEqual(len(node.lidar_obstacle_publisher.messages), 1)
        self.assertEqual(result["measurement_timestamp_s"], 10.0)
        self.assertAlmostEqual(result["source_age_s"], 0.1)
        published = json.loads(node.lidar_obstacle_publisher.messages[-1].data)
        self.assertEqual(published["frame_id"], "base_link")

    def test_stale_or_failed_tf_or_unmatched_stamps_stop(self):
        for mode in ("stale", "tf", "stamp"):
            with self.subTest(mode=mode):
                node = policy_fixture()
                feed_lidar(node)
                api["start"](node)
                if mode == "stale":
                    node.test_clock.advance(0.5)
                elif mode == "tf":
                    node.observations["lidar_cartesian"] = None
                else:
                    node.observations["lidar_cartesian"].stamp_ns -= 1
                node.supervision_callback()
                self.assertEqual(node.fsm_state, 2)
                self.assertEqual(node.lidar_obstacle_output["status"], "UNKNOWN")
                self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)

    def test_bounded_cone_recovery_cannot_bypass_guard(self):
        node = policy_fixture()
        node.control_settings.enabled = True
        node.control_settings.robustness_enabled = True
        feed_lidar(node)
        api["start"](node)
        node.test_clock.advance(0.05)
        feed_lidar(node, {0: 0.8, 1: 0.8})
        self.assertIn("Lidar stop", node._control_health_problem(node.test_clock.monotonic, node.test_clock.ros_ns))
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)

    def test_detector_error_fails_closed(self):
        node = policy_fixture()
        feed_lidar(node)
        node.observations["lidar_scan"].value["ranges"] = None
        api["start"](node)
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.lidar_obstacle_output["status"], "UNKNOWN")


class ScanCallbackChecks(unittest.TestCase):
    def node(self):
        from mvp21_policy.geometry import quaternion_xyzw, valid_frame
        tree = ast.parse(api['api']['SOURCE'].read_text(encoding='utf-8'))
        original = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PolicyNode')
        methods = [n for n in original.body if isinstance(n, ast.FunctionDef)
                   and n.name in {'lidar_callback', '_convert_lidar_scan', '_stamp_ns'}]
        scope = dict(ns, valid_frame=valid_frame, quaternion_xyzw=quaternion_xyzw,
                     Time=SimpleNamespace(from_msg=lambda stamp: stamp), TransformException=RuntimeError)
        subject = ast.ClassDef(name='Callbacks', bases=[], keywords=[], body=methods, decorator_list=[])
        exec(compile(ast.fix_missing_locations(ast.Module(body=[subject], type_ignores=[])),
                     str(api['api']['SOURCE']), 'exec'), scope)
        node = policy_fixture()
        node.__class__ = type('CallbackNode', (node.__class__, scope['Callbacks']), {})
        node.policy_update_mode = 'timer'
        node.tf_buffer = SimpleNamespace(lookup_transform=Mock(side_effect=RuntimeError('no TF')))
        return node

    def message(self, node):
        raw, _ = scan_fixture()
        return SimpleNamespace(**raw, header=SimpleNamespace(frame_id='base_link',
            stamp=SimpleNamespace(sec=node.test_clock.ros_ns//1_000_000_000,
                                  nanosec=node.test_clock.ros_ns%1_000_000_000)))

    def test_invalid_metadata_invalidates_previously_clear_scan_immediately(self):
        node = self.node()
        api['feed'](node)
        node.lidar_callback(self.message(node))
        api['start'](node)
        self.assertEqual(node.fsm_state, 3)
        broken = self.message(node)
        broken.angle_increment = 0.
        node.lidar_callback(broken)
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIsNone(node.observations['lidar_scan'])

    def test_tf_failure_does_not_reuse_old_cartesian_result(self):
        node = self.node()
        api['feed'](node)
        node.lidar_callback(self.message(node))
        api['start'](node)
        node.test_clock.advance(.05)
        msg = self.message(node)
        msg.header.frame_id = 'laser'
        node.lidar_callback(msg)
        node.supervision_callback()
        self.assertIsNone(node.observations['lidar_cartesian'])
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)

    def test_conversion_reuses_one_acquisition_transform_for_points_and_rays(self):
        node = self.node()
        msg = self.message(node)
        msg.header.frame_id = 'laser'
        msg.ranges[359:362] = [.5, .5, .5]
        translation = SimpleNamespace(x=.2, y=0., z=.12)
        rotation = SimpleNamespace(x=0., y=0., z=1., w=0.)
        node.tf_buffer.lookup_transform = Mock(return_value=SimpleNamespace(
            transform=SimpleNamespace(translation=translation, rotation=rotation)))
        node.lidar_callback(msg)
        node.tf_buffer.lookup_transform.assert_called_once_with('base_link', 'laser', msg.header.stamp)
        node._lidar_stop_problem(node.test_clock.monotonic, node.test_clock.ros_ns)
        self.assertEqual(node.lidar_obstacle_output['status'], 'OBSTACLE')
        self.assertAlmostEqual(node.lidar_obstacle_output['nearest_bumper_distance_m'],
                               .2 + .5*math.cos(math.pi/360) - .297)


class RNNMPCStopChecks(unittest.TestCase):
    def node(self, enabled=True):
        from test_rnn_decoupled import node_fixture
        from test_user_mpc_bridge import PARAMS, adapter, ref
        runtime, node = node_fixture()
        node.mpc_settings = adapter(False).mpc_settings
        node.controller = adapter(False)
        node.shadow_worker = None
        node.lidar_obstacle_settings = Settings(**dict(PARAMS['lidar_obstacle'], enabled=enabled))
        node.lidar_obstacle_output = None
        node._lidar_obstacle_stamp_ns = node._lidar_obstacle_publication_key = None
        node.lidar_obstacle_publisher = runtime['Recorder']()

        def select(estimates):
            selected = ref()
            selected['timestamp_s'] = estimates['state']['timestamp_s']
            node.planner_execution.diagnostics = dict(mode='planner', reason='planner_tracking')
            return selected
        node._select_planner_reference = select
        return runtime, node

    def feed(self, runtime, node, hits=None):
        runtime['feed'](node, center=0., speed=.2)
        raw, cart = scan_fixture(hits)
        for name, value in (('lidar_scan', raw), ('lidar_cartesian', cart)):
            node._store(name, value, node.test_clock.ros_ns,
                        node.test_clock.monotonic, node.test_clock.ros_ns)

    def test_selected_half_metre_profile_accepts_outside_and_stops_inside(self):
        runtime, node = self.node()
        self.assertEqual(node.lidar_obstacle_settings.stop_distance_m, .5)
        self.feed(runtime, node, {0: .85, 1: .85})
        runtime['start'](node)
        node.run_policy_step()
        self.assertFalse(node.logged_errors, node.logged_errors)
        self.assertEqual(node.fsm_state, 3)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0.)
        self.assertTrue(node.control_diagnostics['mpc_reference_executed'])
        for actual, expected in zip(node.lidar_obstacle_output['roi_xy_m'][0], (.297, .797)):
            self.assertAlmostEqual(actual, expected)
        node.test_clock.advance(.05)
        self.feed(runtime, node, {0: .7, 1: .7})
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn('Lidar stop', node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        self.assertIsNone(node._trajectory_reference)
        node.test_clock.advance(.05)
        self.feed(runtime, node)
        node.supervision_callback()
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)

    def test_obstacle_during_mpc_solve_discards_action_and_rnn_commit(self):
        runtime, node = self.node()
        self.feed(runtime, node)
        runtime['start'](node)
        node.run_policy_step()
        self.assertGreater(node.action_publisher.messages[-1].drive, 0.)
        memory = SimpleNamespace(commit=Mock(), reset=Mock())
        node.learned_policy_memory = memory
        original = node.calculate_policy_actions
        def solve_then_obstruct(*args):
            result = original(*args)
            node.test_clock.advance(.01)
            self.feed(runtime, node, {0: .6, 1: .6})
            return result
        node.calculate_policy_actions = solve_then_obstruct
        node.test_clock.advance(.05)
        self.feed(runtime, node)
        before = len(node.action_publisher.messages)
        node.run_policy_step()
        self.assertFalse(node.logged_errors, node.logged_errors)
        self.assertEqual(node.fsm_state, 2)
        self.assertTrue(all(m.drive == 0. and m.steer == 0.
                            for m in node.action_publisher.messages[before:]))
        memory.commit.assert_not_called()
        memory.reset.assert_called_once()

    def test_status_publication_failure_cannot_suppress_stop(self):
        runtime, node = self.node()
        self.feed(runtime, node)
        runtime['start'](node)
        node.run_policy_step()
        node.lidar_obstacle_publisher.publish = Mock(side_effect=RuntimeError('no telemetry'))
        node.test_clock.advance(.05)
        self.feed(runtime, node, {0: .6, 1: .6})
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)

    def test_lidar_loss_stops_while_motion_and_camera_are_fresh(self):
        runtime, node = self.node()
        self.feed(runtime, node)
        runtime['start'](node)
        node.run_policy_step()
        node.test_clock.advance(.5)
        runtime['feed'](node, center=0., speed=.2)
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn('missing_stale_or_unmatched_lidar', node.state_reason)

    def test_clear_guard_matches_disabled_checkpoint_controller(self):
        outputs = []
        for enabled in (False, True):
            runtime, node = self.node(enabled)
            self.feed(runtime, node)
            runtime['start'](node)
            node.run_policy_step()
            self.assertFalse(node.logged_errors, node.logged_errors)
            self.assertEqual(node.fsm_state, 3)
            message = node.action_publisher.messages[-1]
            outputs.append((message.drive, message.steer))
        self.assertEqual(outputs[0], outputs[1])


if __name__ == "__main__":
    unittest.main()
