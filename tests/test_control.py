"""Portable tests of actual single-file control and lifecycle methods.

Uses the installed-source selector from test_estimation. The transport ports
below record actions; they do not simulate rclpy, DDS or physical actuators.
The installed ROS gate separately runs test_policy_node with real rclpy peers.
"""
import ast
from copy import deepcopy
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
    xs = [0.1+0.3*i for i in range(9)]
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


class ControlChecks(unittest.TestCase):
    def test_mvp_sustaining_effort_keeps_fault_and_operator_stops_neutral(self):
        node = make_node(simulated=False, mvp=True)
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
                       {"mode": "unknown"}, {"max_distance_m": 0}, {"max_run_time_s": float("inf")},
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
