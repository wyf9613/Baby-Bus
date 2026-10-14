"""Portable tests of actual single-file control and lifecycle methods.

Uses the installed-source selector from test_estimation. The transport ports
below record actions; they do not simulate rclpy, DDS or physical actuators.
The installed ROS gate separately runs test_policy_node with real rclpy peers.
"""
import ast
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import traceback
import unittest


api = runpy.run_path(str(Path(__file__).with_name("test_estimation.py")))
ns = dict(api["namespace"])
tree = ast.parse(api["SOURCE"].read_text(encoding="utf-8"))
subject = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "PolicyNode")
method_names = {"_times", "_age", "_fresh", "health_problem", "_store", "_warn", "run_policy_step",
                "_robust_control_enabled", "_control_health_problem", "_select_control_reference", "_report_control_status",
                "calculate_policy_actions", "fsm_transition_request_callback", "_change_state",
                "_control_problem", "supervision_callback", "publish_state", "publish_zero_actions", "_publish_actions"}
method_names.add("_async_planning_cycle")
method_names.update({"_receive_async_result", "_ready_async_result_usable"})
method_names.update({"_warmstart_refresh_usable", "_warmstart_job_state", "_validate_warmstart_path"})
method_names.add("_merge_warmstart_planner_state")
method_names.add("_observe_stationary_gyro")
method_names.update({"_log_planning_rejection", "_startup_lattice_wait", "_log_planned_trajectory"})
method_names.update({"_startup_road_ready", "_straight_startup_reference", "_drive_straight_startup",
                     "_straight_startup_problem"})
methods = [item for item in subject.body if isinstance(item, ast.FunctionDef) and item.name in method_names]
constants = [item for item in tree.body if isinstance(item, ast.Assign) and
             any(isinstance(target, ast.Name) and (target.id.startswith("FSM_STATE") or
                 target.id in ("STATE_NAMES", "SENSORS", "STAMPED_SENSORS")) for target in item.targets)]
node_class = ast.ClassDef(name="SourceNode", bases=[], keywords=[], body=methods, decorator_list=[])


class DriveMessage(SimpleNamespace):
    UNITS_NORMALIZED = 1


ns.update(Float32=SimpleNamespace, Int8=SimpleNamespace, String=SimpleNamespace,
          DriveAndSteer=DriveMessage, traceback=traceback)
exec(compile(ast.fix_missing_locations(ast.Module(body=constants+[node_class], type_ignores=[])),
             str(api["SOURCE"]), "exec"), ns)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Clock:
    def __init__(self):
        self.monotonic, self.ros_ns = 1.0, 10_000_000_000

    def now(self):
        return SimpleNamespace(nanoseconds=self.ros_ns)

    def advance(self, seconds):
        self.monotonic += seconds
        self.ros_ns += round(seconds*1e9)


def make_node(simulated=True, mvp=False):
    node = ns["SourceNode"]()
    node.test_clock = Clock()
    node._monotonic = lambda: node.test_clock.monotonic
    node.get_clock = lambda: node.test_clock
    node.logged_errors = []
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, warning=lambda *_: None,
                                               error=node.logged_errors.append)
    node.policy_frame_id = "base_link"
    node.required_sensors = ["cone_detections", "wheel_speed", "imu_angular_velocity"]
    node.sensor_timeout_s = {name: 0.5 for name in ns["SENSORS"]}
    node.timestamp_tolerance_s, node.cone_empty_timeout_s = 0.05, 1.0
    node.status_period_s = 0.5
    node.estimation_settings = ns["EstimationSettings"]()
    node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
    node.planning_settings = ns["PlanningSettings"](vehicle_limits_source="course_simulation" if simulated else "upstream")
    node.control_settings = ns["ControlSettings"](enabled=True, mode="mvp" if mvp else "calibrated",
        vehicle_params_source="course_simulation" if simulated else "upstream")
    node.vehicle_settings = ns["VehicleCalibrationSettings"]()
    node.controller = ns["PolicyController"](node.control_settings)
    node.distance_limiter = ns["RunDistanceLimiter"](3.0, node.control_settings.max_dt_s)
    node.estimation_output = node.planning_output = node.planning_diagnostics = node.control_diagnostics = None
    node.last_control_reference_deadline_s = None
    node.control_has_run = False
    node.fsm_state, node.state_reason = 2, "Startup"
    node.observations = {name: None for name in ns["SENSORS"]}
    node.last_nonempty_cones = None
    node.heading_reference = node.policy_started_at = node.previous_policy_step_at = None
    node._last_ros_time_ns = None
    node._last_status_at = -math.inf
    node._last_warning_at = {}
    for name in ("action", "pan", "state", "state_text", "heading", "debug1", "debug2"):
        setattr(node, name+"_publisher", Recorder())
    return node


def feed(node, center=0.1, speed=0.0, yaw=0.0, delay=0.0, empty=False):
    now = node.test_clock.ros_ns
    mono = node.test_clock.monotonic
    node._store("wheel_speed", speed, None, mono, now)
    node._store("imu_angular_velocity", (0, 0, yaw), now, mono, now)
    # A fully observed corridor brackets the origin under fused-road validity.
    xs = [-0.1+0.3*i for i in range(10)]
    batch = api["cones"]([(x, center+0.5) for x in xs], [(x, center-0.5) for x in xs])
    if empty:
        batch["detections"] = []
    node._store("cone_detections", batch, now-round(delay*1e9), mono, now)
    if not empty:
        node.last_nonempty_cones = node.observations["cone_detections"]


def start(node):
    node.fsm_transition_request_callback(SimpleNamespace(data=3))


class RobustControlChecks(unittest.TestCase):
    def node(self, speed=0.2, yaw=0.0):
        node = make_node(simulated=False, mvp=True)
        # Isolate the control bridge; combined estimator prediction is tested
        # separately so it cannot conceal the control-side failure injection.
        node.estimation_settings = replace(node.estimation_settings, road_hold_s=0)
        node.control_settings = ns["ControlSettings"](enabled=True, mode="mvp", robustness_enabled=True,
            drive_max=0.35, mvp_drive_feedforward=0.30)
        node.controller = ns["PolicyController"](node.control_settings)
        feed(node, center=0.05, speed=speed, yaw=yaw)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        return node

    def tick(self, node, speed=0.2, yaw=0.0, cones=False, empty=False):
        node.test_clock.advance(0.05)
        if cones or empty:
            feed(node, center=0.05, speed=speed, yaw=yaw, empty=empty)
        else:
            now, ros = node.test_clock.monotonic, node.test_clock.ros_ns
            node._store("wheel_speed", speed, None, now, ros)
            node._store("imu_angular_velocity", (0.0, 0.0, yaw), ros, now, ros)
        node.supervision_callback()
        node.run_policy_step()

    def test_empty_frame_degrades_and_recovers_without_reset(self):
        node = self.node()
        manager = node.control_reference_manager
        controller = node.controller
        node.controller.speed.integral = 0.4
        self.tick(node, empty=True)
        self.assertEqual(node.fsm_state, 3)
        self.assertEqual(manager.mode, "DEGRADED")
        self.assertEqual(node.controller.speed.integral, 0.4)
        self.assertIs(node.controller, controller)
        self.assertFalse(node.planning_output["valid"])
        self.assertEqual(node.control_diagnostics["upstream_rejection_reason"], "invalid_estimates")
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)
        self.tick(node, cones=True)
        self.assertEqual(manager.mode, "TRACKING")
        self.assertIs(node.controller, controller)
        self.assertGreater(node.distance_limiter.distance_m, 0)
        self.assertEqual(node.policy_started_at, 1.0)

    def test_same_frame_and_missing_cones_never_renew_time_budget(self):
        node = self.node(speed=0.0)
        accepted = node.control_reference_manager.accepted_at_s
        for _ in range(19):
            self.tick(node, speed=0.0)
            self.assertEqual(node.fsm_state, 3)
            self.assertEqual(node.control_reference_manager.accepted_at_s, accepted)
        self.assertEqual(node.control_reference_manager.mode, "DEGRADED")
        self.tick(node, speed=0.0)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("hold time exhausted", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        self.assertIsNone(node.control_reference_manager.reference)
        self.tick(node, speed=0.0, cones=True)
        self.assertEqual(node.fsm_state, 2)

    def test_distance_budget_has_independent_supervision(self):
        node = self.node(speed=0.5)
        for _ in range(9):
            self.tick(node, speed=0.5, empty=True)
            self.assertEqual(node.fsm_state, 3)
        node.test_clock.advance(0.05)
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("hold distance exhausted", node.state_reason)

    def test_fresh_frame_renews_budget_but_stop_and_restart_clear_cache(self):
        node = self.node(speed=0.0)
        for _ in range(12):
            self.tick(node, speed=0.0)
        self.tick(node, speed=0.0, cones=True)
        self.assertAlmostEqual(node.control_reference_manager.accepted_at_s, 1.65)
        node.fsm_transition_request_callback(SimpleNamespace(data=2))
        self.assertIsNone(node.control_reference_manager.reference)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.test_clock.advance(0.05)
        feed(node, speed=0.0, empty=True)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)  # No usable initial path to predict.

    def test_execution_deadline_is_separate_from_short_upstream_lifetime(self):
        node = self.node(speed=0.0)
        node.planning_settings.reference_lifetime_s = 0.01
        for _ in range(4):
            self.tick(node, speed=0.0, cones=True)
            self.assertEqual(node.fsm_state, 3)
            self.assertEqual(node.planning_output["valid_for_s"], 0.01)
            self.assertAlmostEqual(node.last_control_reference_deadline_s,
                                   node.estimation_output["state"]["timestamp_s"]+0.2)

    def test_execution_gap_stops_even_with_trusted_cache(self):
        node = self.node(speed=0.0)
        node.test_clock.advance(0.2)
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("reference expired", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_motion_loss_is_not_hidden_by_cone_loss(self):
        node = self.node(speed=0.0)
        node.test_clock.advance(0.05)
        feed(node, speed=0.0, empty=True)
        node.observations["imu_angular_velocity"] = None
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("imu_angular_velocity", node.state_reason)

    def test_soft_planning_rejections_use_cache_and_hard_rejections_stop(self):
        for reason in ("v1_requires_both_boundaries", "unsupported_road_curvature",
                       "insufficient_near_or_far_coverage", "not_straight_enough",
                       "outside_v1_offset_or_heading_domain", "stale_road"):
            with self.subTest(reason=reason):
                node = self.node(speed=0.0)
                node.planner.plan = lambda *args, **kwargs: (
                    {"valid": False, "reason": reason}, {})
                self.tick(node, speed=0.0, empty=True)
                self.assertEqual(node.fsm_state, 3)
                self.assertEqual(node.control_reference_manager.mode, "DEGRADED")
        for reason in ("frame_mismatch", "invalid_geometry", "stale_wheel_speed", "invalid_clock"):
            with self.subTest(reason=reason):
                node = self.node(speed=0.0)
                node.planner.plan = lambda *args, **kwargs: (
                    {"valid": False, "reason": reason}, {})
                self.tick(node, speed=0.0, empty=True)
                self.assertEqual(node.fsm_state, 2)
                self.assertIn(reason, node.state_reason)

    def test_known_road_conflict_cannot_be_hidden_by_generic_planning_failure(self):
        node = self.node(speed=0.0)
        original = node.estimator.update
        def conflict(*args, **kwargs):
            out = original(*args, **kwargs)
            out["road"].update(valid=False, status="observed_history_conflict")
            return out
        node.estimator.update = conflict
        node.planner.plan = lambda *args, **kwargs: ({"valid": False, "reason": "invalid_estimates_or_boundaries"}, {})
        self.tick(node, speed=0.0, empty=True)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("observed_history_conflict", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_current_road_conflict_blocks_an_older_valid_worker_snapshot(self):
        node = self.node(speed=0.0)
        snapshot = deepcopy(node.estimation_output)
        node.estimation_output["road"].update(valid=False, status="observed_history_conflict")
        with self.assertRaisesRegex(ValueError, "observed_history_conflict"):
            node._select_control_reference(snapshot)
        self.assertIsNone(node.control_reference_manager.reference)

    def test_incremental_prediction_matches_total_motion_and_updates_actions(self):
        node = self.node(speed=0.2, yaw=0.1)
        original = deepcopy(node.control_reference_manager.endpoints)
        initial_steer = node.action_publisher.messages[-1].steer
        for _ in range(6):
            self.tick(node, speed=0.2, yaw=0.1, empty=True)
            self.assertEqual(node.fsm_state, 3)
        angle = 0.03
        dx, dy = 2*math.sin(angle), 2*(1-math.cos(angle))
        for (x, y), actual in zip(original, node.control_reference_manager.endpoints):
            expected = (math.cos(angle)*(x-dx)+math.sin(angle)*(y-dy),
                        -math.sin(angle)*(x-dx)+math.cos(angle)*(y-dy))
            for value, oracle in zip(actual, expected):
                self.assertAlmostEqual(value, oracle, places=10)
        self.assertNotEqual(node.action_publisher.messages[-1].steer, initial_steer)
        self.assertAlmostEqual(node.control_diagnostics["original_source_age_s"], 0.3)

    def test_motion_gap_and_exhausted_support_stop(self):
        node = self.node()
        node.motion_history.samples["imu_angular_velocity"].clear()
        self.tick(node, empty=True)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("missing_motion_history", node.state_reason)
        node = self.node()
        node.control_reference_manager.endpoints = [(0.0, 0.05), (0.001, 0.05)]
        self.tick(node, empty=True)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("support exhausted", node.state_reason)

    def test_runtime_distance_and_clock_limits_still_stop(self):
        node = self.node()
        node.distance_limiter.maximum_m = 0.01
        self.tick(node, empty=True)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Distance limit", node.state_reason)
        node = self.node()
        node.policy_started_at -= 30
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Run time limit", node.state_reason)
        node = self.node()
        node.test_clock.ros_ns -= 1
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("clock moved backwards", node.state_reason)


class RobustLatticeIntegrationChecks(unittest.TestCase):
    def test_learned_single_side_keeps_tracking_beyond_control_hold_then_stops_when_empty(self):
        node = self.node(direct=True, speed=0.0)
        node.estimation_settings = replace(node.estimation_settings,
            single_side_width_memory_enabled=True, history_conflict_check_enabled=False)
        node.estimator.settings = node.estimation_settings
        node.test_clock.advance(.05)
        feed(node, center=.1, speed=0)
        node.run_policy_step()
        for _ in range(30):
            node.test_clock.advance(.05)
            feed(node, center=.1, speed=0)
            batch = node.observations["cone_detections"].value
            batch["detections"] = [p for p in batch["detections"] if p[3] == 1]
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertEqual(node.estimation_output["road"]["visibility"], "right_only")
            self.assertTrue(node.estimation_output["road"]["single_side_reconstructed"])
            self.assertEqual(node.control_reference_manager.mode, "TRACKING")
            self.assertGreater(node.action_publisher.messages[-1].drive, 0)
        for _ in range(25):
            node.test_clock.advance(.05)
            feed(node, speed=0, empty=True)
            node.run_policy_step()
            if node.fsm_state == 2:
                break
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def straight_node(self):
        node = make_node(simulated=False, mvp=True)
        node.control_settings = ns["ControlSettings"](enabled=True, mode="mvp", robustness_enabled=True,
            straight_startup_enabled=True, mvp_drive_feedforward=.3, drive_max=.35,
            max_distance_m=0, startup_grace_s=3.0)
        node.controller = ns["PolicyController"](node.control_settings)
        node.distance_limiter = ns["RunDistanceLimiter"](0, .2)
        node.planning_settings = replace(node.planning_settings, algorithm="lattice_v2", max_source_age_s=.6)
        node.lattice_settings = ns["LatticeSettings"](horizon_m=.6, preview_m=.2, minimum_output_m=.2,
            durations_s="1.0", score_time_s=1.0, transition_lengths_m="0.3,0.6", budget_s=5,
            obstacle_check_enabled=False, direct_sample_output=True, clear_start_assumed=True)
        self.front_feed(node, speed=0.0)
        start(node)
        return node

    def front_feed(self, node, speed=.2):
        node._store("wheel_speed", speed, None, node.test_clock.monotonic, node.test_clock.ros_ns)
        node._store("imu_angular_velocity", (0, 0, 0), node.test_clock.ros_ns,
                    node.test_clock.monotonic, node.test_clock.ros_ns)
        xs = [.7+i*.2 for i in range(14)]
        batch = api["cones"]([(x, .5) for x in xs], [(x, -.5) for x in xs])
        node._store("cone_detections", batch, node.test_clock.ros_ns,
                    node.test_clock.monotonic, node.test_clock.ros_ns)
        node.last_nonempty_cones = node.observations["cone_detections"]

    def test_straight_start_switches_on_one_road_and_never_reenters(self):
        node = self.straight_node()
        for _ in range(10):
            self.front_feed(node)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertEqual(node.planning_output["planner_version"], "straight_startup")
            self.assertGreater(node.action_publisher.messages[-1].drive, 0)
            self.assertEqual(node.action_publisher.messages[-1].steer, 0)
            self.assertTrue(all(point[1:] == (0.0, 0.0, 0.0)
                for point in node.planning_output["path"]["samples_xy_heading_curvature"]))
            self.assertFalse(getattr(node, "straight_startup_road_seen", False))
            node.test_clock.advance(.05)
        # One fully observed road switches immediately, without a frame gate.
        feed(node, center=0, speed=.2)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.straight_startup_complete)
        self.assertEqual(node.planning_output["planner_version"], "frenet_lattice_v2")
        node.straight_startup_started_at -= 3.0
        node.distance_limiter.distance_m = .6
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        estimates = deepcopy(node.estimation_output)
        estimates["road"]["valid"] = False
        self.assertIsNone(node._straight_startup_reference(estimates,
            {"road": 1.5, "wheel_speed": .25, "imu_angular_velocity": .25}))

    def test_straight_start_time_budget_stops_without_new_sensor_callback(self):
        node = self.straight_node()
        node.run_policy_step()
        node.straight_startup_started_at -= 3.0
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Straight startup time limit", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        self.assertIsNone(node.straight_startup_started_at)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)

    def test_straight_start_distance_budget_even_with_total_distance_disabled(self):
        node = self.straight_node()
        node.run_policy_step()
        node.distance_limiter.distance_m = .5
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Straight startup distance limit", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_straight_start_budget_checked_before_each_reference_and_during_handoff(self):
        node = self.straight_node()
        node.run_policy_step()
        node.straight_startup_started_at -= 3.0
        with self.assertRaisesRegex(ns["PolicyStopRequest"], "Straight startup time limit"):
            node._straight_startup_reference(node.estimation_output,
                {"road": 1.5, "wheel_speed": .25, "imu_angular_velocity": .25})
        node = self.straight_node()
        node.run_policy_step()
        node.test_clock.advance(.05)
        feed(node, center=0, speed=.2)
        node.async_planner = SimpleNamespace(poll=lambda: None)
        def pending(estimates, timeouts):
            node.planning_output = {"valid": False, "reason": "planning_pending"}
            return estimates
        node._async_planning_cycle = pending
        node.run_policy_step()
        self.assertTrue(node.straight_startup_road_seen)
        node.straight_startup_started_at -= 3.0
        node.supervision_callback()
        self.assertIn("Straight startup time limit", node.state_reason)

    def test_straight_start_bounds_must_be_finite_positive(self):
        for name in ("straight_startup_max_time_s", "straight_startup_max_distance_m"):
            for value in (0, -1, math.inf, math.nan):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    ns["ControlSettings"](**{name: value})

    def test_straight_start_keeps_motion_failure_and_run_time_stop(self):
        node = self.straight_node()
        node.run_policy_step()
        node.test_clock.advance(.3)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        node = self.straight_node()
        node.run_policy_step()
        node.policy_started_at -= 30
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Run time limit", node.state_reason)

    def test_straight_start_bridges_only_initial_pending_handoff(self):
        node = self.straight_node()
        node.run_policy_step()
        node.test_clock.advance(.05)
        feed(node, center=0, speed=.2)
        node.async_planner = SimpleNamespace(poll=lambda: None)
        def pending(estimates, timeouts):
            node.planning_output = {"valid": False, "reason": "planning_pending"}
            return estimates
        node._async_planning_cycle = pending
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.straight_startup_road_seen)
        self.assertFalse(node.straight_startup_complete)
        self.assertEqual(node.planning_output["planner_version"], "straight_startup")
        node.async_planner = None
        node.test_clock.advance(.05)
        feed(node, center=0, speed=.2)
        node.run_policy_step()
        self.assertTrue(node.straight_startup_complete)

    def test_predicted_first_road_and_stale_motion_cannot_start_handoff(self):
        node = self.straight_node()
        node.run_policy_step()
        estimates = deepcopy(node.estimation_output)
        stamp = estimates["state"]["timestamp_s"]
        estimates["road"].update(valid=True, visibility="both", time_aligned=True,
            frame_id="base_link", timestamp_s=stamp, measurement_timestamp_s=stamp,
            source_age_s=0, degraded=True)
        timeouts = {"road": 1.5, "wheel_speed": .25, "imu_angular_velocity": .25}
        self.assertFalse(node._startup_road_ready(estimates, timeouts))
        self.assertIsNotNone(node._straight_startup_reference(estimates, timeouts))
        estimates["state"]["source_age_s"]["imu_angular_velocity"] = .251
        with self.assertRaisesRegex(ns["PolicyStopRequest"], "stale imu"):
            node._straight_startup_reference(estimates, timeouts)

    def test_straight_start_hands_off_to_actual_worker(self):
        import time
        node = self.straight_node()
        node.run_policy_step()
        node.async_planner = ns["AsyncLatticePlanner"]()
        node.planning_generation = getattr(node, "planning_generation", 0)+1
        node.last_planning_submit_s = -math.inf
        node.last_async_fault, node.last_async_diagnostics = None, {}
        try:
            deadline = time.monotonic()+5
            while not node.straight_startup_complete and time.monotonic() < deadline:
                node.test_clock.advance(.05)
                feed(node, center=0, speed=.2)
                node.run_policy_step()
                self.assertEqual(node.fsm_state, 3, node.state_reason)
                time.sleep(.01)
            self.assertTrue(node.straight_startup_complete)
            self.assertEqual(node.planning_output["planner_version"], "frenet_lattice_v2")
        finally:
            node.async_planner.close()

    def node(self, direct=False, recovery_frames=1, speed=0.2):
        node = make_node(simulated=False, mvp=True)
        node.estimation_settings = replace(node.estimation_settings, road_hold_s=0)
        node.planning_settings = replace(node.planning_settings, algorithm="lattice_v2",
                                        max_source_age_s=0.4, reference_lifetime_s=0.2)
        node.lattice_settings = ns["LatticeSettings"](obstacle_check_enabled=False, budget_s=5.0,
                                                     clear_start_assumed=True, direct_sample_output=direct)
        node.control_settings = ns["ControlSettings"](enabled=True, mode="mvp", robustness_enabled=True,
            drive_max=0.35, mvp_drive_feedforward=0.30, recovery_confirm_frames=recovery_frames)
        node.controller = ns["PolicyController"](node.control_settings)
        feed(node, center=0.1, speed=speed)
        start(node)
        # Numerical integration test uses deterministic planner time; separate
        # watchdog tests retain real elapsed-control and source deadline checks.
        node.planner = ns["FrenetLatticePlanner"](node.planning_settings, node.lattice_settings,
                                                  clock=lambda: 0.0)
        node.previous_policy_step_at = node.test_clock.monotonic
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.planning_output["planner_version"], "frenet_lattice_v2")
        return node

    def test_first_observed_lattice_reference_drives_without_recovery_frame_gate(self):
        node = self.node(direct=True, recovery_frames=9, speed=0.0)
        self.assertTrue(node.control_has_run)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)
        self.assertEqual(node.control_settings.recovery_confirm_frames, 9)

    def test_startup_wait_keeps_deadline_and_excludes_stop_or_prediction(self):
        node = self.node(direct=True, speed=0.0)
        node.control_has_run = False
        estimates = deepcopy(node.estimation_output)
        cruise = deepcopy(node.planning_output)
        for reason in ("planning_pending", "no_feasible_lattice_trajectory"):
            node.planning_output = {"valid": False, "reason": reason}
            self.assertTrue(node._startup_lattice_wait(estimates, 0.1))
        node.planning_output = {**cruise, "planned_stop": True}
        self.assertTrue(node._startup_lattice_wait(estimates, 0.1))
        node.planning_output = cruise
        estimates["road"]["degraded"] = True
        self.assertTrue(node._startup_lattice_wait(estimates, 0.1))
        estimates["road"]["degraded"] = False
        self.assertFalse(node._startup_lattice_wait(estimates, 0.1))
        node.planning_output = {"valid": False, "reason": "planning_pending"}
        with self.assertRaisesRegex(ns["PolicyStopRequest"], "startup history deadline"):
            node._startup_lattice_wait(estimates, node.control_settings.startup_grace_s)

    def test_actual_quintic_reference_degrades_recovers_with_preview(self):
        node = self.node()
        original = deepcopy(node.control_reference_manager.curve_samples)
        self.assertIsNotNone(original)
        self.assertEqual(len(node.planning_output["path"]["coeffs_low_to_high"]), 6)
        self.assertGreater(abs(node.control_diagnostics["path_error_m"]), 1e-5)
        for _ in range(3):
            node.test_clock.advance(0.05)
            feed(node, speed=0.2, empty=True)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertEqual(node.control_diagnostics["tracking_mode"], "DEGRADED")
            self.assertEqual(node.control_diagnostics["path_degree"], 5)
            self.assertIn("control_preview_x_m", node.control_diagnostics)
        for initial, current in zip(original, node.control_reference_manager.curve_samples):
            self.assertAlmostEqual(current[0], initial[0]-0.03, places=8)
            self.assertAlmostEqual(current[1], initial[1], places=8)
            self.assertAlmostEqual(current[3], initial[3], places=8)
        node.test_clock.advance(0.05)
        feed(node, center=0.1, speed=0.2)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.control_diagnostics["tracking_mode"], "TRACKING")
        accepted = node.control_reference_manager.accepted_at_s
        node.planner.cfg = replace(node.planner.cfg, budget_s=0.035)
        ticks = iter(i*0.01 for i in range(1000))
        node.planner.clock = lambda: next(ticks)
        node.test_clock.advance(0.05)
        feed(node, center=0.1, speed=0.2)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.planning_output["reason"], "lattice_time_budget_exceeded")
        self.assertEqual(node.control_diagnostics["tracking_mode"], "DEGRADED")
        self.assertEqual(node.control_reference_manager.accepted_at_s, accepted)

    def test_direct_sample_cache_bridges_motion_and_recovers_without_refitting(self):
        node = self.node(direct=True)
        original = deepcopy(node.control_reference_manager.curve_samples)
        self.assertEqual(node.planning_output["path"]["type"], "CARTESIAN_SAMPLES")
        accepted = node.control_reference_manager.accepted_at_s
        for _ in range(3):
            node.test_clock.advance(0.05)
            feed(node, speed=0.2, empty=True)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertEqual(node.control_diagnostics["tracking_mode"], "DEGRADED")
            self.assertGreater(node.action_publisher.messages[-1].drive, 0)
            self.assertEqual(node.control_reference_manager.accepted_at_s, accepted)
        for old, new in zip(original, node.control_reference_manager.curve_samples):
            self.assertAlmostEqual(new[0], old[0]-0.03, places=8)
            self.assertEqual(new[3], old[3])
        node.test_clock.advance(0.05)
        feed(node, center=0.1, speed=0.2)
        node.run_policy_step()
        self.assertEqual(node.control_diagnostics["tracking_mode"], "TRACKING")
        node.planner.plan = lambda *args, **kwargs: ({"valid": False, "reason": "no_feasible_lattice_trajectory"}, {})
        node.test_clock.advance(0.05)
        feed(node, center=0.1, speed=0.2)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_upstream_prediction_cannot_renew_trust_or_undo_slowdown(self):
        node = self.node()
        manager = node.control_reference_manager
        accepted = manager.accepted_at_s
        cached = deepcopy(manager.reference)
        source = manager.source_stamp_s
        node.test_clock.advance(0.05)
        feed(node, speed=0.2)
        now = node.test_clock.ros_ns/1e9
        reference = deepcopy(cached)
        reference.update(timestamp_s=now, target_speed_mps=0.05)
        state = dict(node.estimation_output["state"], timestamp_s=now)
        # Even a misleading new timestamp on a prediction cannot seed trust.
        road = {"measurement_timestamp_s": now, "degraded": True}
        selected = manager.select(reference, road, state, node.motion_history, now,
                                  node.test_clock.monotonic, 0.01, 0.2)
        self.assertEqual(manager.accepted_at_s, accepted)
        self.assertEqual(manager.source_stamp_s, source)
        self.assertEqual(manager.mode, "DEGRADED")
        self.assertEqual(selected["target_speed_mps"], 0.05)
        rejected = {"valid": False, "reason": "expired_road_prediction"}
        selected = manager.select(rejected, road, state, node.motion_history, now,
                                  node.test_clock.monotonic, 0.01, 0.2)
        self.assertLessEqual(selected["target_speed_mps"], 0.05)

    def test_lattice_stop_and_collision_rejection_never_replay_cruise(self):
        node = self.node()
        manager = node.control_reference_manager
        reference = deepcopy(node.planning_output)
        reference["planned_stop"] = True
        selected = manager.select(reference, node.estimation_output["road"],
            node.estimation_output["state"], node.motion_history, reference["timestamp_s"], 1.0, 0.0, 0.2)
        self.assertTrue(selected["planned_stop"])
        self.assertIsNone(manager.reference)
        for reason in ("no_feasible_lattice_trajectory", "no_valid_cartesian_output", "initial_frenet_domain"):
            node = self.node()
            node.planner.plan = lambda *args, **kwargs: ({"valid": False, "reason": reason}, {})
            node.test_clock.advance(0.05)
            feed(node, speed=0.2)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 2)
            self.assertIn(reason, node.state_reason)


class ControlChecks(unittest.TestCase):
    def test_trajectory_log_uses_forward_input_and_matching_startup_details(self):
        import json
        node = make_node()
        lines = []
        node.get_logger = lambda: SimpleNamespace(info=lines.append)
        reference = {"valid": True, "trajectory_id": 3, "timestamp_s": 10.0,
            "frame_id": "base_link", "path": {"type": "CARTESIAN_SAMPLES",
            "samples_xy_heading_curvature": [[0, 0, 0, 0], [.6, .02, -.1, -.2]]}}
        forward = {"timestamp_s": 10.0, "measurement_timestamp_s": 9.7,
            "status": "near_field_unobserved", "geometry_valid": True,
            "centerline_xy": [[.6+i*.01, .03] for i in range(201)],
            "left_boundary_xy": [[.6, .5], [2.6, .5]],
            "right_boundary_xy": [[.6, -.5], [2.6, -.5]]}
        road = {"valid": False, "status": "near_field_unobserved", "centerline_xy": [],
                "forward_geometry": forward}
        diag = {"startup_connection": {"selected_join": {"x_m": .7, "y_m": .03}},
                "selected": {"startup_join_x_m": .7}}
        state = {"timestamp_s": 10.0, "speed_mps": 0.0, "yaw_rate_rps": .1}
        before = deepcopy((reference, road, diag, state))
        node._log_planned_trajectory(reference, "planner_worker", road, diag, state)
        record = json.loads(lines[0].split("Planned trajectory: ", 1)[1])
        self.assertEqual(record["trajectory_id"], 3)
        self.assertEqual(record["planning_input"]["geometry_source"], "forward_geometry")
        self.assertEqual(record["planning_input"]["measurement_timestamp_s"], 9.7)
        center = record["planning_input"]["centerline_xy"]
        self.assertEqual(center["point_count"], 201)
        self.assertEqual(center["logged_point_count"], 128)
        self.assertEqual(center["point_indices"][0], 0)
        self.assertEqual(center["point_indices"][-1], 200)
        self.assertEqual(record["planning_details"]["startup_connection"], diag["startup_connection"])
        self.assertEqual(record["planning_state"]["yaw_rate_rps"], .1)
        self.assertEqual((reference, road, diag, state), before)

    def test_generated_trajectory_logging_preserves_path_and_bounds_output(self):
        import json
        node = make_node()
        lines = []
        node.get_logger = lambda: SimpleNamespace(info=lines.append)
        reference = {"valid": True, "trajectory_id": 1, "timestamp_s": 10.0,
                     "frame_id": "base_link", "target_speed_mps": .2,
                     "path": {"type": "CARTESIAN_SAMPLES", "range": [0, 2],
                              "samples_xy_heading_curvature": [[i*.01, -i*.002, -.1, -.2]
                                                                 for i in range(201)]}}
        before = deepcopy(reference)
        node._log_planned_trajectory(reference, "planner_worker")
        node._log_planned_trajectory(reference, "planner_worker")
        self.assertEqual(len(lines), 1)
        record = json.loads(lines[0].split("Planned trajectory: ", 1)[1])
        self.assertEqual(record["stage"], "generated_before_control_selection")
        self.assertEqual(record["sample_count"], 201)
        self.assertEqual(record["logged_sample_count"], 128)
        self.assertEqual(record["sample_indices"][0], 0)
        self.assertEqual(record["sample_indices"][-1], 200)
        self.assertEqual(record["samples"][-1], [2, -.4, -.1, -.2])
        self.assertEqual(reference, before)
        node.planning_generation = getattr(node, "planning_generation", 0)+1
        node._log_planned_trajectory(reference, "planner_worker")
        self.assertEqual(len(lines), 2)
        reference["valid"] = False
        node._log_planned_trajectory(reference, "planner_worker")
        self.assertEqual(len(lines), 2)

    def test_mvp_sustaining_effort_keeps_fault_and_operator_stops_neutral(self):
        node = make_node(simulated=False, mvp=True)
        node.estimation_settings = replace(node.estimation_settings, road_hold_s=0)
        node.control_settings = ns["ControlSettings"](
            enabled=True, mode="mvp", drive_max=0.35, mvp_drive_feedforward=0.30)
        node.controller = ns["PolicyController"](node.control_settings)
        feed(node, center=0.0, speed=node.planning_settings.cruise_speed_mps)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertGreaterEqual(node.action_publisher.messages[-1].drive, 0.25)
        node.test_clock.advance(0.05)
        feed(node, empty=True)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.test_clock.advance(0.05)
        feed(node, center=0.0, speed=0.0)
        start(node)
        node.run_policy_step()
        self.assertLessEqual(node.action_publisher.messages[-1].drive, 0.35)
        node.fsm_transition_request_callback(SimpleNamespace(data=2))
        node.supervision_callback()
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        for invalid in (-0.01, 0.36, math.nan):
            with self.assertRaises(ValueError):
                ns["ControlSettings"](drive_max=0.35, mvp_drive_feedforward=invalid)

    def test_actual_policy_entry_produces_limited_actions_and_debug(self):
        for center in (-0.1, 0.1):
            node = make_node()
            feed(node, center=center)
            start(node)
            node.run_policy_step()
            action = node.action_publisher.messages[-1]
            self.assertEqual(node.fsm_state, 3)
            self.assertTrue(0 < action.drive <= 0.15)
            self.assertTrue(0 < abs(action.steer) <= 0.5)
            self.assertGreater(action.steer*center, 0)
            self.assertEqual(node.pan_publisher.messages, [])
            self.assertEqual(len(node.debug1_publisher.messages), 1)
            self.assertEqual(node.debug2_publisher.messages[-1].data, 0)
            self.assertEqual(node.logged_errors, [])

    def test_unknown_car_limits_stop_without_simulation_fallback(self):
        node = make_node(simulated=False)
        feed(node)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("vehicle_limits_unavailable", node.state_reason)
        self.assertEqual((node.action_publisher.messages[-1].drive, node.action_publisher.messages[-1].steer), (0, 0))
        self.assertIsNone(node.planning_output)
        self.assertEqual(node.controller.speed.integral, 0)

    def test_invalid_road_latches_stop_and_fresh_data_does_not_resume(self):
        node = make_node()
        node.estimation_settings = replace(node.estimation_settings, road_hold_s=0)
        feed(node)
        start(node)
        node.run_policy_step()
        node.controller.speed.integral = 0.4
        node.test_clock.advance(0.05)
        feed(node, empty=True)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.controller.speed.integral, 0)
        self.assertIsNone(node.last_control_reference_deadline_s)
        node.test_clock.advance(0.05)
        feed(node)
        count = len(node.action_publisher.messages)
        node.run_policy_step()
        self.assertEqual(len(node.action_publisher.messages), count)
        node.supervision_callback()
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)
        self.assertEqual(node.controller.speed.integral, 0)  # First dt=0.

    def test_reference_expiry_stops_without_sensor_callbacks(self):
        node = make_node()
        feed(node)
        start(node)
        node.run_policy_step()
        node.test_clock.advance(0.1)
        self.assertIsNone(node.health_problem(node.test_clock.monotonic, node.test_clock.ros_ns))
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("reference expired", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        count = len(node.action_publisher.messages)
        node.test_clock.advance(0.05)
        node.supervision_callback()
        self.assertEqual(len(node.action_publisher.messages), count+1)
        self.assertEqual(node.pan_publisher.messages, [])

    def test_late_new_reference_does_not_hide_previous_expiry(self):
        node = make_node()
        feed(node)
        start(node)
        node.run_policy_step()
        node.test_clock.advance(0.11)
        feed(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("reference expired", node.state_reason)

    def test_calculation_past_reference_deadline_cannot_publish_drive(self):
        node = make_node()
        feed(node)
        start(node)
        method = node.calculate_policy_actions
        def slow(*args):
            result = method(*args)
            node.test_clock.advance(0.11)
            return result
        node.calculate_policy_actions = slow
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertTrue(all(msg.drive == 0 for msg in node.action_publisher.messages))

    def test_motion_priming_is_neutral_bounded_and_only_before_tracking(self):
        node = make_node()
        feed(node, delay=0.05)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertEqual(node.control_diagnostics["reason"], "priming_motion_history")
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.test_clock.advance(0.05)
        feed(node)  # New acquisition has covered motion history.
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertTrue(node.control_has_run)
        timeout = make_node()
        feed(timeout, delay=0.05)
        start(timeout)
        timeout.run_policy_step()
        for _ in range(10):
            timeout.test_clock.advance(0.05)
            feed(timeout)
            timeout.supervision_callback()
        self.assertEqual(timeout.fsm_state, 2)
        self.assertIn("startup history deadline", timeout.state_reason)

    def test_mvp_tracks_without_vehicle_calibration(self):
        for center in (-0.1, 0.1):
            node = make_node(simulated=False, mvp=True)
            feed(node, center=center)
            start(node)
            node.run_policy_step()
            self.assertFalse(node.vehicle_settings.valid)
            self.assertEqual(node.fsm_state, 3)
            self.assertEqual(node.planning_output["target_speed_mps"], 0.2)
            self.assertEqual(node.planning_output["operating_profile"], "mvp_low_speed")
            self.assertTrue(node.control_diagnostics["valid"])
            action = node.action_publisher.messages[-1]
            self.assertTrue(0 < action.drive <= 0.15)
            self.assertGreater(action.steer*center, 0)
            self.assertLessEqual(abs(action.steer), 0.5)
            self.assertEqual(node.pan_publisher.messages, [])
            # A quintic from planning is consumed without a car-geometry record.
            ref = deepcopy(node.planning_output)
            ref["path"]["coeffs_low_to_high"] = [center, 0.02, 0.01, -0.002, 0.001, 0.0002]
            drive, steer, info = node.controller.calculate(ref, node.estimation_output["state"], {}, 10.0, 0.05)
            self.assertTrue(info["valid"], info["reason"])
            self.assertGreater(steer*center, 0)
            self.assertTrue(0 < drive <= 0.15)
            ref["simulation_only"] = True
            self.assertFalse(node.controller.calculate(ref, node.estimation_output["state"], {}, 10.0, 0.05)[2]["valid"])

    def test_three_metre_cutoff_latches_and_explicit_restart_resets(self):
        node = make_node(simulated=False, mvp=True)
        feed(node, speed=0.2)
        start(node)
        node.run_policy_step()
        for step in range(1, 301):
            node.test_clock.advance(0.05)
            feed(node, speed=0.2)
            if step % 11 == 0:
                start(node)  # Repeated start while running must not reset mileage.
            node.run_policy_step()
            if step < 300:
                self.assertEqual(node.fsm_state, 3)
        self.assertAlmostEqual(node.distance_limiter.distance_m, 3.0)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Distance limit reached", node.state_reason)
        self.assertAlmostEqual(node.debug2_publisher.messages[-1].data, 3.0)
        self.assertEqual(node.logged_errors, [])  # A normal stop, no exception trace.
        self.assertEqual((node.action_publisher.messages[-1].drive, node.action_publisher.messages[-1].steer), (0, 0))
        for _ in range(3):
            node.test_clock.advance(0.05)
            feed(node)
            node.run_policy_step()
            node.supervision_callback()
            self.assertEqual(node.fsm_state, 2)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        start(node)
        self.assertEqual(node.distance_limiter.distance_m, 0)
        node.run_policy_step()
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)

    def test_distance_supervision_and_post_calculation_cutoff(self):
        for during_calculation in (False, True):
            node = make_node(simulated=False, mvp=True)
            feed(node, speed=0.2)
            start(node)
            node.run_policy_step()
            node.distance_limiter.distance_m = 2.995
            if during_calculation:
                original = node.calculate_policy_actions
                def slow(*args):
                    result = original(*args)
                    node.test_clock.advance(0.05)
                    return result
                node.calculate_policy_actions = slow
                count = len(node.action_publisher.messages)
                node.run_policy_step()
                self.assertTrue(all(msg.drive == 0 for msg in node.action_publisher.messages[count:]))
            else:
                node.test_clock.advance(0.05)
                node.supervision_callback()  # No new sensors or policy invocation.
            self.assertEqual(node.fsm_state, 2)
            self.assertIn("Distance limit reached", node.state_reason)
            self.assertAlmostEqual(node.distance_limiter.distance_m, 3.005)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_distance_uses_raw_speed_and_operator_stop_keeps_reading(self):
        node = make_node(simulated=False, mvp=True)
        feed(node, speed=0.2)
        start(node)
        node.run_policy_step()
        node.test_clock.advance(0.05)
        feed(node, speed=1.0)
        node.run_policy_step()
        self.assertLess(node.estimation_output["state"]["speed_mps"], 1.0)
        self.assertAlmostEqual(node.distance_limiter.distance_m, 0.03)
        node.fsm_transition_request_callback(SimpleNamespace(data=2))
        node.test_clock.advance(0.05)
        node.supervision_callback()
        self.assertAlmostEqual(node.distance_limiter.distance_m, 0.03)

    def test_fresh_zero_wheel_speed_has_run_time_cutoff(self):
        node = make_node(simulated=False, mvp=True)
        feed(node)
        start(node)
        for _ in range(601):
            feed(node)
            node.run_policy_step()
            if node.fsm_state != 3:
                break
            node.test_clock.advance(0.05)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Run time limit", node.state_reason)
        self.assertEqual(node.distance_limiter.distance_m, 0)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_mvp_missing_and_stale_wheel_speed_stop(self):
        for missing in (False, True):
            node = make_node(simulated=False, mvp=True)
            feed(node)
            start(node)
            node.run_policy_step()
            if missing:
                node.observations["wheel_speed"] = None
            else:
                node.test_clock.advance(0.5)
                wheel = node.observations["wheel_speed"]
                feed(node)
                node.observations["wheel_speed"] = wheel
            node.supervision_callback()
            self.assertEqual(node.fsm_state, 2)
            self.assertIn("wheel_speed", node.state_reason)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_gyro_loss_and_ros_clock_rollback_reset_control(self):
        for change in ("gyro", "clock"):
            node = make_node()
            feed(node)
            start(node)
            node.run_policy_step()
            if change == "gyro":
                node.observations["imu_angular_velocity"] = None
            else:
                node.test_clock.ros_ns -= 1_000_000_000
            node.supervision_callback()
            self.assertEqual(node.fsm_state, 2)
            self.assertEqual(node.controller.speed.integral, 0)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_quintic_geometry_scale_and_multiple_local_minima(self):
        path = {"type": "CARTESIAN_Y_OF_X", "independent_variable": "x_m", "origin": 0.0,
                "scale": 1.0, "coeffs_low_to_high": [0.1, 0.03, 0.02, -0.01, 0.002, 0.005], "range": [0.5, 2.0]}
        geometry = ns["control_path_geometry"](path)
        x = 0.5
        slope = 0.03+0.04*x-0.03*x*x+0.008*x**3+0.025*x**4
        second = 0.04-0.06*x+0.024*x*x+0.1*x**3
        self.assertAlmostEqual(geometry["closest_x_m"], x)
        self.assertAlmostEqual(geometry["curvature_1pm"], second/(1+slope*slope)**1.5)
        # y(u)=(u-.2)(u-.7)(u-1.3)(u-1.8)(u-2.4)*6, independent oracle samples.
        coeffs = [1.0]
        for root in (0.2, 0.7, 1.3, 1.8, 2.4):
            next_coeffs = [0.0]*(len(coeffs)+1)
            for i, coefficient in enumerate(coeffs):
                next_coeffs[i] -= root*coefficient
                next_coeffs[i+1] += coefficient
            coeffs = next_coeffs
        path.update(coeffs_low_to_high=[6*v for v in coeffs], range=[0.0, 2.7])
        geometry = ns["control_path_geometry"](path)
        xs = [i*2.7/10000 for i in range(10001)]
        def cost(value):
            y = 6*math.prod(value-root for root in (0.2, 0.7, 1.3, 1.8, 2.4))
            return value*value+y*y
        oracle = min(xs, key=cost)
        self.assertAlmostEqual(geometry["closest_x_m"], oracle, delta=3e-4)
        # Nonzero origin and scale, physical y(x)=.1+.05*x.
        path.update(coeffs_low_to_high=[0.16, 0.035, 0, 0, 0, 0], origin=1.2, scale=0.7, range=[0.4, 2])
        geometry = ns["control_path_geometry"](path)
        self.assertAlmostEqual(geometry["closest_y_m"], 0.12)
        self.assertAlmostEqual(geometry["path_heading_rad"], math.atan(0.05))

    def test_invalid_settings_and_incomplete_calibration_are_rejected(self):
        for values in ({"drive_max": 2.0}, {"enabled": 1}, {"max_dt_s": 0}, {"speed_ki": float("nan")},
                       {"mode": "unknown"}, {"max_distance_m": -1}, {"max_run_time_s": float("inf")},
                       {"mvp_steering_direction": 0}):
            with self.assertRaises(ValueError):
                ns["ControlSettings"](**values)
        with self.assertRaises(ValueError):
            ns["VehicleCalibrationSettings"](valid=True)
        with self.assertRaises(ValueError):
            ns["VehicleCalibrationSettings"](steering_direction=0.0)

    def test_measured_record_inlet_drives_actual_policy(self):
        node = make_node(simulated=False)
        # Synthetic measurement fixture only, not a statement about a real car.
        node.vehicle_settings = ns["VehicleCalibrationSettings"](valid=True, source="synthetic_measurement_fixture",
            wheelbase_m=0.33, rear_axle_from_cg_m=0.132, steering_limit_rad=math.pi/4,
            front_extent_m=0.297, rear_extent_m=0.198, width_m=0.25, speed_max_mps=0.5,
            braking_deceleration_mps2=0.5, actuation_delay_s=0.1, safety_margin_m=0.05)
        feed(node)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertEqual(node.control_diagnostics["vehicle_params_source"], "synthetic_measurement_fixture")
        self.assertFalse(node.control_diagnostics["simulation_only"])
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)

    def test_stop_request_and_simulation_reference_never_issue_reverse(self):
        node = make_node()
        feed(node)
        start(node)
        node.run_policy_step()
        ref, state = deepcopy(node.planning_output), deepcopy(node.estimation_output["state"])
        ref["stop_requested"] = True
        drive, steer, info = node.controller.calculate(ref, state, {}, 10.0, 0.05)
        self.assertEqual((drive, steer), (0, 0))
        self.assertTrue(info["stop_requested"])
        ref["stop_requested"] = False
        live = ns["PolicyController"](ns["ControlSettings"](enabled=True))
        drive, steer, info = live.calculate(ref, state, {}, 10.0, 0.05)
        self.assertEqual((drive, steer), (0, 0))
        self.assertFalse(info["valid"])
        self.assertIn("simulation_reference_rejected", info["reason"])
        ref["simulation_only"] = False
        params = {"valid": True, "source": "course_simulation"}
        drive, steer, info = live.calculate(ref, state, params, 10.0, 0.05)
        self.assertEqual((drive, steer), (0, 0))
        self.assertFalse(info["valid"])
        self.assertIn("simulation_geometry_rejected", info["reason"])


class DistanceChecks(unittest.TestCase):
    def test_disabled_distance_limit_counts_travel_and_keeps_gap_checks(self):
        self.assertEqual(ns["ControlSettings"](max_distance_m=0).max_distance_m, 0)
        limiter = ns["RunDistanceLimiter"](0.0, 0.2)
        limiter.reset(0.0, 1.0)
        limiter.distance_m = 3.1
        self.assertIsNone(limiter.advance(1.0, .1))
        self.assertAlmostEqual(limiter.distance_m, 3.2)
        self.assertIsNone(limiter.advance(1.0, .2))
        self.assertAlmostEqual(limiter.distance_m, 3.3)
        self.assertIn("clock/gap", limiter.advance(1.0, .5))
        self.assertEqual(limiter.last_check["reason"], "integration_gap")
        self.assertAlmostEqual(limiter.last_check["dt_s"], .3)

    def test_variable_speed_cached_checks_and_exact_threshold(self):
        limiter = ns["RunDistanceLimiter"](3.0, 0.2)
        limiter.reset(0.0, 0.0)
        self.assertIsNone(limiter.advance(1.0, 0.1))
        self.assertAlmostEqual(limiter.distance_m, 0.05)
        self.assertIsNone(limiter.advance(1.0, 0.1))  # Same clock never counts twice.
        self.assertIsNone(limiter.advance(1.0, 0.2))  # Cached speed still represents travel.
        self.assertAlmostEqual(limiter.distance_m, 0.15)
        limiter.distance_m = 2.9
        self.assertIn("Distance limit reached", limiter.advance(1.0, 0.3))
        self.assertAlmostEqual(limiter.distance_m, 3.0)

    def test_invalid_wheel_input_and_clock_gap_fail_closed(self):
        for speed, now in ((-0.1, 0.1), (float("nan"), 0.1), (float("inf"), 0.1),
                           (0.2, float("nan")), (0.2, -0.1), (0.2, 0.201)):
            limiter = ns["RunDistanceLimiter"](3.0, 0.2)
            limiter.reset(0.0, 0.2)
            self.assertIsNotNone(limiter.advance(speed, now))
            self.assertEqual(limiter.distance_m, 0)


class Mvp2RecoveryChecks(unittest.TestCase):
    def test_initial_prediction_stays_neutral_until_observed_control_trust(self):
        node = make_node(simulated=False, mvp=True)
        node.control_settings = replace(node.control_settings, robustness_enabled=True)
        node.async_planner = SimpleNamespace(poll=lambda: None)
        def cycle(estimates, timeouts):
            if not hasattr(node, "first_request_seen"):
                node.first_request_seen = True
                node.planning_output = {"valid": False, "reason": "planning_pending"}
                node.planning_diagnostics = {}
            else:
                node.planning_output, node.planning_diagnostics = node.planner.plan(
                    estimates["road"], estimates["state"], estimates["obstacles"],
                    estimates["vehicle_limits"], estimates["state"]["timestamp_s"], timeouts, mvp=True)
            return estimates
        node._async_planning_cycle = cycle
        feed(node, center=0.05)
        start(node)
        node.run_policy_step()
        node.test_clock.advance(0.05)
        feed(node, empty=True)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.control_diagnostics["reason"], "awaiting_observed_reference")
        self.assertFalse(node.control_has_run)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.test_clock.advance(0.05)
        feed(node, center=0.05)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.control_has_run)

    def test_stationary_gyro_bias_learns_freezes_and_keeps_raw_source(self):
        node = make_node(simulated=False, mvp=True)
        node.control_settings = replace(node.control_settings, gyro_bias_learning_enabled=True)
        node.gyro_bias_samples, node.gyro_bias_rps = [], 0.0
        for _ in range(13):
            feed(node, yaw=0.07)
            node._observe_stationary_gyro(0.07, node._monotonic(), node.test_clock.ros_ns)
            node.test_clock.advance(0.1)
        self.assertAlmostEqual(node.gyro_bias_rps, 0.07)
        feed(node, yaw=0.07)
        start(node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertAlmostEqual(node.estimation_output["state"]["yaw_rate_rps"], 0.0)
        self.assertEqual(node.observations["imu_angular_velocity"].value[2], 0.07)
        node._observe_stationary_gyro(0.1, node._monotonic(), node.test_clock.ros_ns)
        self.assertAlmostEqual(node.gyro_bias_rps, 0.07)

    def test_noisy_or_moving_stationary_window_cannot_change_gyro_bias(self):
        for moving in (False, True):
            node = make_node(simulated=False, mvp=True)
            node.control_settings = replace(node.control_settings, gyro_bias_learning_enabled=True)
            node.gyro_bias_samples, node.gyro_bias_rps = [], 0.0
            for i in range(20):
                rate = 0.06 if moving else 0.06+0.02*(-1)**i
                feed(node, speed=0.1 if moving else 0, yaw=rate)
                node._observe_stationary_gyro(rate, node._monotonic(), node.test_clock.ros_ns)
                node.test_clock.advance(0.1)
            self.assertEqual(node.gyro_bias_rps, 0)

    def test_compute_source_expiry_uses_cache_but_motion_loss_stops(self):
        node = RobustControlChecks().node()
        node.test_clock.advance(0.05)
        feed(node, center=0.05, speed=0.2)
        original = node.planner.plan
        def expired(*args, **kwargs):
            ref, diag = original(*args, **kwargs)
            ref.update(valid=False, reason="source_deadline_during_planning")
            return ref, diag
        node.planner.plan = expired
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.control_diagnostics["tracking_mode"], "DEGRADED")
        node.observations["imu_angular_velocity"] = None
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)

    def test_degrade_speed_cap_confirm_distinct_frames_and_ramp_up(self):
        node = RobustControlChecks().node()
        manager = node.control_reference_manager
        manager.settings = replace(manager.settings, degraded_speed_mps=0.1, recovery_confirm_frames=3)
        node.test_clock.advance(0.05)
        feed(node, empty=True, speed=0.2)
        node.run_policy_step()
        self.assertAlmostEqual(manager.last_target_speed_mps, 0.1)
        for i in range(3):
            node.test_clock.advance(0.05)
            feed(node, center=0.05, speed=0.2)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertEqual(manager.mode, "DEGRADED" if i < 2 else "TRACKING")
            self.assertLessEqual(manager.last_target_speed_mps, 0.1+0.15*0.05+1e-9)

    def test_async_poll_never_queues_more_than_one_job_and_rejects_old_generation(self):
        node = RobustControlChecks().node()
        node.lattice_settings = ns["LatticeSettings"](async_enabled=True)
        node.planning_generation = 10
        node.last_planning_submit_s = -math.inf
        node.last_async_fault = None
        class Worker:
            ready, pending_at, result = True, None, None
            process = SimpleNamespace(poll=lambda: None)
            def poll(self):
                result, self.result = self.result, None
                if result:
                    self.pending_at = None
                return result
            def submit(self, job, now):
                self.pending_at = now
                self.job = job
        worker = node.async_planner = Worker()
        estimates = deepcopy(node.estimation_output)
        node._async_planning_cycle(estimates, {})
        self.assertEqual(worker.job["generation"], 10)
        old_job = worker.job
        node._async_planning_cycle(estimates, {})
        self.assertIs(worker.job, old_job)
        worker.result = {"generation": 9, "reference": {"valid": True}}
        node._async_planning_cycle(estimates, {})
        self.assertFalse(node.planning_output["valid"])
        self.assertEqual(node.planning_output["reason"], "planning_pending")
        node._change_state(2, "Operator request")
        self.assertEqual(node.planning_generation, 11)
        self.assertIsNone(node.control_reference_manager.reference)


class WarmstartChecks(unittest.TestCase):
    def test_control_gap_stop_reports_actual_distance_and_execution_intervals(self):
        node = self.node()
        warnings = []
        node.get_logger = lambda: SimpleNamespace(info=lambda *_:None, warning=warnings.append, error=lambda *_:None)
        node.test_clock.advance(.25)
        mono, ros = node._times()
        node._store("wheel_speed", .2, None, mono, ros)
        node._store("imu_angular_velocity", (0,0,0), ros, mono, ros)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("Control execution gap exceeded", node.state_reason)
        record = ast.literal_eval(next(s.split("Control timing: ",1)[1] for s in warnings if "Control timing: " in s))
        self.assertAlmostEqual(record["distance_check"]["dt_s"], .25)
        self.assertEqual(record["distance_check"]["reason"], "integration_gap")
        self.assertAlmostEqual(record["execution_check"]["control_gap_s"], .25)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_feedback_jitter_uses_active_half_second_budget_but_real_timeout_stops(self):
        node = self.node()
        node.sensor_timeout_s.update(wheel_speed=.5, imu_angular_velocity=.5)
        node.estimation_settings = replace(node.estimation_settings, motion_max_gap_s=.5)
        node.estimator.settings = node.estimation_settings
        node.motion_history.settings = node.estimation_settings
        self.fail_planning(node)
        # Keep the 20 Hz control cadence while both motion streams pause.
        for _ in range(7):
            node.test_clock.advance(.05)
            node.supervision_callback()
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
        diag = node.estimation_output["state"]["motion_feedback"]
        self.assertGreater(diag["imu_angular_velocity"]["age_s"], .25)
        self.assertEqual(diag["imu_angular_velocity"]["reason"], "valid")
        self.tick(node)
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        for _ in range(11):
            node.test_clock.advance(.05)
            node.supervision_callback()
            node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("stale", node.state_reason.lower())

    def node(self, front=False, progress=False):
        node = make_node(simulated=False, mvp=True)
        node.estimation_settings = replace(node.estimation_settings, max_source_age_s=1.5,
            motion_max_gap_s=.25, motion_max_interval_s=.5, road_hold_s=0)
        node.sensor_timeout_s.update(cone_detections=1.5, wheel_speed=.25, imu_angular_velocity=.25)
        node.planning_settings = replace(node.planning_settings, algorithm="lattice_v2", max_source_age_s=1.5)
        node.lattice_settings = ns["LatticeSettings"](horizon_m=1, minimum_output_m=.2, preview_m=.2,
            durations_s="1.0", score_time_s=1, transition_lengths_m="0.3,0.6", budget_s=5,
            obstacle_check_enabled=False, direct_sample_output=True, clear_start_assumed=True,
            startup_connection_enabled=True, startup_corridor_check_enabled=False,
            startup_progress_enabled=progress,
            boundary_collision_only_enabled=True, recovery_enabled=True)
        node.control_settings = ns["ControlSettings"](enabled=True, mode="mvp", robustness_enabled=True,
            warmstart_enabled=True, max_distance_m=0, startup_grace_s=3,
            drive_max=.35, mvp_drive_feedforward=.3)
        node.controller = ns["PolicyController"](node.control_settings)
        node.distance_limiter = ns["RunDistanceLimiter"](0, .2)
        if front:
            mono, ros = node._times()
            node._store("wheel_speed", 0, None, mono, ros)
            node._store("imu_angular_velocity", (0, 0, 0), ros, mono, ros)
            xs = [.2+i*.1 for i in range(9)]
            node._store("cone_detections", api["cones"]([(x, .5) for x in xs], [(x, -.5) for x in xs]), ros, mono, ros)
            node.last_nonempty_cones = node.observations["cone_detections"]
        else:
            feed(node, center=0, speed=0)
        start(node)
        node.planner = ns["FrenetLatticePlanner"](node.planning_settings, node.lattice_settings, clock=lambda: 0)
        node.previous_policy_step_at = node.test_clock.monotonic
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.control_has_run, node.state_reason)
        return node

    def fail_planning(self, node):
        original = node.planner.plan
        def rejected(*args, **kwargs):
            args = list(args)
            args[0] = {**args[0], "valid": False, "status": "insufficient_geometry"}
            return original(*args, **kwargs)
        node.planner.plan = rejected
        return original

    def tick(self, node, speed=.2, cones=False):
        node.test_clock.advance(.05)
        if cones:
            feed(node, center=0, speed=speed)
        else:
            mono, ros = node._times()
            node._store("wheel_speed", speed, None, mono, ros)
            node._store("imu_angular_velocity", (0, 0, 0), ros, mono, ros)
        node.supervision_callback()
        node.run_policy_step()

    def test_failure_survives_road_age_and_legacy_hold_budgets_then_replans(self):
        node = self.node()
        manager = node.control_reference_manager
        original = self.fail_planning(node)
        cached_id = manager.reference["trajectory_id"]
        accepted, endpoint = manager.accepted_at_s, manager.curve_samples[-1][0]
        for _ in range(60):
            self.tick(node)
            self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(manager.mode, "WARMSTART")
        self.assertEqual(manager.reference["trajectory_id"], cached_id)
        self.assertEqual(manager.accepted_at_s, accepted)
        self.assertGreater(node.control_diagnostics["original_source_age_s"], 1.5)
        self.assertGreater(node.control_diagnostics["prediction_distance_m"], .25)
        self.assertGreater(endpoint-manager.curve_samples[-1][0], .5)
        self.assertLess(endpoint-manager.curve_samples[-1][0], .6)
        node.planner.plan = original
        self.tick(node, cones=True)
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(manager.mode, "TRACKING")
        self.assertNotEqual(manager.reference["trajectory_id"], cached_id)
        self.assertGreater(manager.accepted_at_s, accepted)

    def test_cache_brakes_for_endpoint_and_never_extends_it(self):
        node = self.node()
        self.fail_planning(node)
        slowed = False
        for _ in range(110):
            self.tick(node)
            if node.fsm_state == 2:
                break
            slowed |= node.control_reference_manager.last_target_speed_mps < .19
        self.assertTrue(slowed)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("exhausted", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        self.assertLess(node.distance_limiter.distance_m, 1.0)

    def test_controlled_start_can_seed_short_path_and_continue_without_restarting(self):
        node = self.node(front=True)
        self.assertFalse(node.planner.startup_complete)
        self.assertLess(node.control_reference_manager.curve_samples[-1][0], 1.0)
        corridor = node.planner.startup_corridor
        self.fail_planning(node)
        for _ in range(8):
            self.tick(node)
            self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertIs(node.planner.startup_corridor, corridor)
        self.assertGreater(corridor["pose"][0], .03)
        self.assertFalse(node.planner.startup_complete)
        self.assertIsNotNone(node.control_reference_manager.reference)

    def test_accepted_startup_join_does_not_move_away_on_success_or_failure(self):
        node = self.node(front=True, progress=True)
        manager = node.control_reference_manager
        first = manager.startup_join_remaining_m
        self.assertIsNotNone(first)
        for _ in range(8):
            previous = manager.startup_join_remaining_m
            self.tick(node)
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertLessEqual(manager.startup_join_remaining_m, previous+1e-9)
            if node.planning_output["valid"]:
                self.assertLessEqual(node.planning_diagnostics["startup_join_space"]["preferred_join_x_m"],
                    max(previous, node.planning_diagnostics["startup_join_space"]["observed_start_x_m"], .05)+1e-8)
        self.assertLess(manager.startup_join_remaining_m, first-.03)
        self.fail_planning(node)
        cached_id = manager.reference["trajectory_id"]
        before = manager.startup_join_remaining_m
        for _ in range(8):
            self.tick(node)
            self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(manager.reference["trajectory_id"], cached_id)
        self.assertLess(manager.startup_join_remaining_m, before-.03)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)

    def test_delayed_worker_state_cannot_undo_verified_control_cadence_motion(self):
        node = self.node(front=True, progress=True)
        snapshot = {"startup_corridor": deepcopy(node.planner.startup_corridor),
                    "startup_motion": tuple(node.planner.startup_motion), "startup_complete": False,
                    "observed_origin_seen": False, "stop_latched": False}
        node.warmstart_planner_state = deepcopy(snapshot)
        for _ in range(6):
            self.tick(node)
            state = node.estimation_output["state"]
            current = node._warmstart_job_state(state["timestamp_s"], state)
            self.assertTrue(current["startup_motion_verified"])
            node.warmstart_planner_state = current
            node._merge_warmstart_planner_state(snapshot)
            self.assertEqual(node.warmstart_planner_state["startup_motion"][0], state["timestamp_s"])
            self.assertGreaterEqual(node.warmstart_planner_state["startup_corridor"]["pose"][0], 0)
        self.assertGreater(node.warmstart_planner_state["startup_corridor"]["pose"][0], .02)
        node.warmstart_planner_state["startup_complete"] = True
        node._merge_warmstart_planner_state(snapshot)
        self.assertTrue(node.warmstart_planner_state["startup_complete"])
        node._change_state(2, "Operator request")
        self.assertIsNone(node.warmstart_planner_state)
        self.assertIsNone(node.control_reference_manager.startup_join_remaining_m)

    def test_first_start_has_no_cache_and_retains_original_age_and_wait_gate(self):
        node = self.node()
        node._change_state(2, "Operator request")
        self.assertIsNone(node.control_reference_manager.reference)
        start(node)
        self.assertIsNone(node.control_reference_manager.reference)
        node.previous_policy_step_at = node.test_clock.monotonic
        seen_limits = []
        def pending(*args, **kwargs):
            seen_limits.append(args[5]["road"])
            return {"valid": False, "reason": "no_feasible_lattice_trajectory"}, {}
        node.planner.plan = pending
        node.control_has_run = False
        for _ in range(8):
            self.tick(node, speed=0, cones=True)
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0)
            self.assertIsNone(node.control_reference_manager.reference)
        self.assertEqual(set(seen_limits), {.6})
        for _ in range(56):
            self.tick(node, speed=0, cones=True)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("startup history deadline", node.state_reason)

    def test_stale_motion_and_control_gap_still_stop(self):
        for fault in ("gyro", "gap"):
            with self.subTest(fault=fault):
                node = self.node()
                if fault == "gyro":
                    node.observations["imu_angular_velocity"] = None
                else:
                    node.test_clock.advance(.25)
                    feed(node, speed=.2)
                node.supervision_callback()
                self.assertEqual(node.fsm_state, 2, node.state_reason)
                self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_all_planning_failure_reasons_keep_the_accepted_path(self):
        node = self.node()
        manager = node.control_reference_manager
        cached_id, accepted = manager.reference["trajectory_id"], manager.accepted_at_s
        for reason in ("planning_pending", "no_feasible_lattice_trajectory", "no_valid_cartesian_output",
                       "initial_frenet_domain", "lattice_time_budget_exceeded", "planning_worker_timeout",
                       "planning_worker_unavailable", "source_deadline_during_planning", "invalid_estimates_or_boundaries"):
            with self.subTest(reason=reason):
                node.planner.plan = lambda *args, _reason=reason, **kwargs: (
                    {"valid": False, "reason": _reason, "stop_requested": True}, {})
                self.tick(node)
                self.assertEqual(node.fsm_state, 3, node.state_reason)
                self.assertEqual(manager.reference["trajectory_id"], cached_id)
                self.assertEqual(manager.accepted_at_s, accepted)
                self.assertEqual(manager.diagnostics["upstream_rejection_reason"], reason)
                self.assertGreater(node.action_publisher.messages[-1].drive, 0)

    def collision_fixture(self, node, left_y, points):
        node.control_reference_manager.last_collision_key = None
        selected = deepcopy(node._trajectory_reference)
        selected["control_cached_path_samples"] = points
        road = deepcopy(node.estimation_output["road"])
        xs = [-.5+i*.05 for i in range(41)]
        road.update(valid=True, geometry_valid=True, time_aligned=True,
            left_boundary_xy=[(x, left_y(x)) for x in xs],
            right_boundary_xy=[(x, left_y(x)-1) for x in xs],
            boundary_source={"left": "width_inferred", "right": "observed_with_history"},
            visibility="right_only", status="single_side_inferred")
        model = node.planner._model(node.estimation_output["vehicle_limits"])
        return selected, road, model

    def test_logged_passed_pose_collision_does_not_reject_remaining_path(self):
        node = self.node()
        # Reproduce the measured retained CG/rear-left pose and inferred-left
        # boundary contact. Its past rear corner crosses by about 7.4 mm, while
        # the actual current rear corner is already forward of that conflict.
        old = (-.028189591294579965, -.00004314644702826714, .00331504327235717, -.15319165673866275)
        points = [old, (.02, 0, 0, 0), (.1, 0, 0, 0), (.2, 0, 0, 0)]
        selected, road, model = self.collision_fixture(node,
            lambda x: .11573892993616514+.58*(x+.22660288298397044), points)
        left, right = [ns["LatticeCurve"](road[side+"_boundary_xy"], 0, .05) for side in ("left", "right")]
        clearance = node.planner._boundary_edge_clearance(
            {"x": old[0], "y": old[1], "heading": old[2]}, left, right, model, math.inf, {})
        self.assertAlmostEqual(clearance, -.0074, delta=.0002)
        actual = node.planner._boundary_edge_clearance(
            {"x": 0, "y": 0, "heading": 0}, left, right, model, math.inf, {})
        self.assertGreater(actual, 0)
        original_check = node.planner._boundary_edge_clearance
        checked = []
        def capture(geom, *args, **kwargs):
            checked.append(dict(geom))
            return original_check(geom, *args, **kwargs)
        node.planner._boundary_edge_clearance = capture
        for stage in ("candidate", "selected"):
            node.control_reference_manager.last_collision_key = None
            node._validate_warmstart_path(selected, road, node.estimation_output, model, stage=stage)
        self.assertEqual(checked[0], {"x": 0, "y": 0, "heading": 0})
        self.assertTrue(all(p["x"] >= 0 for p in checked))
        self.assertEqual(points[0], old)  # Keep the bracket for controller interpolation.
        self.assertFalse(node.warmstart_collision_diagnostics)

    def test_actual_current_body_is_checked_even_when_the_same_path_was_validated(self):
        node = self.node()
        selected, road, model = self.collision_fixture(node, lambda x: .05,
            [(-.02, -.2, 0, 0), (0, -.2, 0, 0), (.2, -.2, 0, 0)])
        manager = node.control_reference_manager
        manager.last_collision_key = ((selected.get("trajectory_id"), selected.get("generated_at_s")),
                                      road.get("measurement_timestamp_s"))
        with self.assertRaisesRegex(ValueError, "conflicts with current boundaries"):
            node._validate_warmstart_path(selected, road, node.estimation_output, model)
        collision = node.warmstart_collision_diagnostics["selected"]
        self.assertEqual(collision["check_pose_type"], "current_body")
        self.assertEqual(collision["path_pose_xy_heading_curvature"], [0, 0, 0, 0])
        self.assertEqual(collision["ignored_consumed_path_samples"], 1)

    def test_future_body_collision_still_rejects_a_currently_clear_path(self):
        node = self.node()
        selected, road, model = self.collision_fixture(node, lambda x: .5-.9*x,
            [(-.02, 0, 0, 0), (0, 0, 0, 0), (.2, 0, 0, 0)])
        with self.assertRaisesRegex(ValueError, "conflicts with current boundaries"):
            node._validate_warmstart_path(selected, road, node.estimation_output, model)
        collision = node.warmstart_collision_diagnostics["selected"]
        self.assertEqual(collision["check_pose_type"], "future_path")
        self.assertEqual(collision["path_sample_index"], 2)
        self.assertGreater(collision["path_distance_from_origin_m"], .1)
        self.assertFalse(collision["path_pose_behind_origin"])

    def test_new_real_boundary_collision_stops_cached_path(self):
        node = self.node()
        warnings = []
        node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, warning=warnings.append,
                                                  error=node.logged_errors.append)
        self.fail_planning(node)
        node.test_clock.advance(.05)
        feed(node, center=-.45, speed=.2)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2, node.state_reason)
        self.assertIn("conflicts with current boundaries", node.state_reason)
        stop = ast.literal_eval(next(s.split("Control timing: ", 1)[1] for s in warnings if s.startswith("Control timing: ")))
        record = stop["warmstart_collision"]["selected"]
        self.assertEqual(record["stage"], "selected")
        self.assertEqual(record["contact"]["side"], "left")
        self.assertEqual(record["boundary_source"], "observed")
        self.assertLess(record["signed_clearance_m"], 0)
        self.assertAlmostEqual(record["penetration_m"], -record["signed_clearance_m"])
        self.assertAlmostEqual(record["boundary_point_xy_m"][1], .05)
        self.assertEqual(record["body_model_m"]["width_m"], .25)
        self.assertEqual(len(record["contact"]["body_point_xy_m"]), 2)
        self.assertTrue(record["boundary_near_contact_xy_m"])
        self.assertTrue(any(s.startswith("Warmstart collision: ") for s in warnings))

    def test_discontinuous_replan_keeps_previous_reference(self):
        node = self.node()
        manager = node.control_reference_manager
        cached_id = manager.reference["trajectory_id"]
        candidate = deepcopy(manager.reference)
        candidate.pop("control_cached_path_samples", None)
        candidate["trajectory_id"] = 999
        candidate["path"]["samples_xy_heading_curvature"] = [
            (x, y+.25, h, k) for x, y, h, k in candidate["path"]["samples_xy_heading_curvature"]]
        def replanned(*args, **kwargs):
            candidate.update(timestamp_s=args[4], valid_for_s=.2)
            return candidate, {}
        node.planner.plan = replanned
        self.tick(node, cones=True)
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(manager.reference["trajectory_id"], cached_id)
        self.assertEqual(manager.diagnostics["upstream_rejection_reason"], "warmstart_candidate_discontinuous")

    def test_expired_execution_window_refreshes_cache_through_normal_supervision(self):
        node = self.node()
        self.fail_planning(node)
        node.test_clock.advance(.05)
        feed(node, center=0, speed=.2)
        node.last_control_reference_deadline_s = node.test_clock.ros_ns/1e9-.001
        previous = len(node.action_publisher.messages)
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertGreater(len(node.action_publisher.messages), previous)
        self.assertGreater(node.last_control_reference_deadline_s, node.test_clock.ros_ns/1e9)

    def test_async_invalid_error_old_generation_and_expired_results_keep_cache_and_dispatch(self):
        for fault in ("invalid", "error", "generation", "expired"):
            with self.subTest(fault=fault):
                node = self.node()
                cached_id = node.control_reference_manager.reference["trajectory_id"]
                result = {"generation": node.planning_generation,
                    "reference": deepcopy(node.planning_output), "road": deepcopy(node.estimation_output["road"]),
                    "diagnostics": {}}
                result["reference"]["trajectory_id"] = 999
                if fault == "invalid":
                    result["reference"].update(valid=False, reason="no_feasible_lattice_trajectory")
                elif fault == "error":
                    result["error"] = "synthetic worker fault"
                elif fault == "generation":
                    result["generation"] -= 1
                else:
                    result["reference"].update(valid_for_s=.001)
                class Worker:
                    pending_at, ready = None, True
                    process = SimpleNamespace(poll=lambda: None)
                    def poll(self):
                        value, self.result = self.result, None
                        self.pending_at = None
                        return value
                    def submit(self, job, now):
                        self.job, self.pending_at = job, now
                        return True
                node.async_planner = Worker()
                node.async_planner.result = result
                node.last_planning_submit_s = -math.inf
                node.lattice_settings = replace(node.lattice_settings, async_enabled=True)
                self.tick(node, cones=True)
                self.assertEqual(node.fsm_state, 3, node.state_reason)
                self.assertEqual(node.control_reference_manager.reference["trajectory_id"], cached_id)
                self.assertIn("warmstart", node.async_planner.job)
                self.assertEqual(node.async_planner.job["warmstart"]["timestamp_s"], node.estimation_output["state"]["timestamp_s"])

    def test_candidate_collision_is_rejected_before_replacing_safe_cache(self):
        node = self.node()
        manager = node.control_reference_manager
        cached_id = manager.reference["trajectory_id"]
        node.test_clock.advance(.05)
        feed(node, center=0, speed=.2)
        estimates = node.estimator.update(
            {name: obs.value if obs else None for name, obs in node.observations.items()},
            {name: 0 for name in node.observations},
            {name: obs.stamp_ns if obs else None for name, obs in node.observations.items()},
            {name: obs.received_at if obs else None for name, obs in node.observations.items()},
            node.test_clock.ros_ns/1e9,
            {name: obs.received_ros_ns if obs else None for name, obs in node.observations.items()})
        candidate = deepcopy(manager.reference)
        candidate.update(trajectory_id=999, timestamp_s=estimates["state"]["timestamp_s"], valid_for_s=.2)
        candidate.pop("control_cached_path_samples", None)
        candidate["path"]["samples_xy_heading_curvature"] = [
            (x, y+(.6*x*x), h, k) for x, y, h, k in candidate["path"]["samples_xy_heading_curvature"]]
        node.planning_output, node.estimation_output = candidate, estimates
        selected = node._select_control_reference(estimates)
        self.assertEqual(selected["trajectory_id"], cached_id)
        self.assertIn("conflicts with current boundaries", manager.diagnostics["upstream_rejection_reason"])
        collision = node.warmstart_collision_diagnostics["candidate"]
        self.assertEqual(collision["trajectory_id"], 999)
        self.assertEqual(collision["cached_trajectory_id"], cached_id)
        self.assertGreater(collision["path_distance_from_origin_m"], .2)
        self.assertNotIn("selected", node.warmstart_collision_diagnostics)

    def test_background_log_severities_have_distinct_caller_locations(self):
        import sys
        import threading
        event, calls, locations = threading.Event(), [], {}
        class Logger:
            def emit(self, level, message):
                frame = sys._getframe(2)
                key = (frame.f_code.co_filename, frame.f_lineno)
                if key in locations and locations[key] != level:
                    raise ValueError("Logger severity changed at one caller location")
                locations[key] = level
                calls.append((level, message))
                event.set()
            def info(self, message):
                self.emit("info", message)
            def warning(self, message):
                self.emit("warning", message)
        sink = ns["AsyncPolicyLog"](Logger())
        try:
            for level in ("warning", "info", "warning", "info"):
                event.clear()
                sink.submit(level, "record: ", {"value": 1})
                self.assertTrue(event.wait(1), sink.status())
            self.assertEqual([level for level, _ in calls], ["warning", "info", "warning", "info"])
            self.assertEqual(sink.status()["errors"], 0)
            self.assertEqual(len(locations), 2)
        finally:
            sink.close()
            sink.thread.join(1)

    def test_background_logger_failure_is_visible_and_next_record_still_writes(self):
        import contextlib
        import io
        import threading
        entered, recovered = threading.Event(), threading.Event()
        def fails(message):
            entered.set()
            raise RuntimeError("synthetic logger failure")
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            sink = ns["AsyncPolicyLog"](SimpleNamespace(info=fails, warning=lambda _: recovered.set()))
            try:
                sink.submit("info", "bad: ", {})
                self.assertTrue(entered.wait(1))
                sink.submit("warning", "good: ", {})
                self.assertTrue(recovered.wait(1))
                status = sink.status()
                self.assertEqual(status["errors"], 1)
                self.assertIn("synthetic logger failure", status["last_error"])
                self.assertTrue(status["thread_alive"])
                self.assertIn("Policy diagnostic log error:", stderr.getvalue())
            finally:
                sink.close()
                sink.thread.join(1)

    def test_large_log_can_block_without_blocking_control_and_queue_is_bounded(self):
        import threading
        entered, release = threading.Event(), threading.Event()
        def blocked(message):
            entered.set()
            release.wait(2)
        node = self.node()
        sink = ns["AsyncPolicyLog"](SimpleNamespace(info=blocked, warning=blocked))
        node.diagnostic_log = sink
        try:
            sink.submit("info", "test", {})
            self.assertTrue(entered.wait(.5))
            began = ns["time"].perf_counter()
            for _ in range(3):
                self.tick(node, cones=True)
                self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertLess(ns["time"].perf_counter()-began, .8)
            self.assertEqual(sink.records.qsize(), 1)
            self.assertGreater(sink.dropped, 0)
            self.assertFalse(release.is_set())
        finally:
            release.set()
            sink.close()
            sink.thread.join(1)


class AsyncDeadlineHandoffChecks(unittest.TestCase):
    def ready_node(self, delay=.05):
        node = RobustControlChecks().node(speed=0)
        reference = deepcopy(node.planning_output)
        reference["trajectory_id"] = 123
        reference["expires_at_s"] = reference["timestamp_s"]+reference["valid_for_s"]
        result = {"generation": node.planning_generation, "reference": reference,
                  "road": deepcopy(node.estimation_output["road"]),
                  "state": deepcopy(node.estimation_output["state"]), "diagnostics": {}}
        class Worker:
            ready, pending_at = False, None
            process = SimpleNamespace(poll=lambda: None)
            def poll(self):
                value, self.result = self.result, None
                return value
        node.async_planner = Worker()
        node.async_planner.result = result
        node.lattice_settings = ns["LatticeSettings"](async_enabled=True)
        node.test_clock.advance(delay)
        feed(node, center=.05, speed=0)
        node.last_control_reference_deadline_s = node.test_clock.ros_ns/1e9-.001
        return node, result

    def test_ready_result_refreshes_before_policy_preflight_expiry(self):
        node, _ = self.ready_node()
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.last_control_timing["trajectory_id"], 123)
        self.assertGreater(node.last_control_reference_deadline_s, node.test_clock.ros_ns/1e9)
        self.assertIsNone(node.ready_async_result)

    def test_supervisor_validates_ready_result_before_stopping(self):
        node, _ = self.ready_node()
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.last_control_timing["trajectory_id"], 123)
        self.assertGreater(node.last_control_reference_deadline_s, node.test_clock.ros_ns/1e9)

    def test_bad_or_late_results_cannot_bypass_expiry(self):
        for fault in ("generation", "expired", "invalid", "error", "control_gap", "motion"):
            with self.subTest(fault=fault):
                node, result = self.ready_node(.21 if fault == "control_gap" else .05)
                if fault == "generation":
                    result["generation"] -= 1
                elif fault == "expired":
                    result["reference"]["expires_at_s"] = node.test_clock.ros_ns/1e9
                elif fault == "invalid":
                    result["reference"]["valid"] = False
                elif fault == "error":
                    result["error"] = "worker failed"
                elif fault == "motion":
                    node.observations["imu_angular_velocity"] = None
                node.supervision_callback()
                self.assertEqual(node.fsm_state, 2, node.state_reason)
                self.assertEqual(node.action_publisher.messages[-1].drive, 0)
                self.assertIsNone(node.ready_async_result)


if __name__ == "__main__":
    unittest.main(verbosity=2)
