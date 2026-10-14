"""Focused portable checks of the simple follower using actual policy methods.

No ROS/device connection or actuator publisher is created. Recorder ports are
the same source-method harness used by test_control, not a vehicle simulation.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
import unittest

api = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
ns = api["ns"]


def fixture(offset=0.0, bend=0.0):
    points = [(i*0.1, offset+bend*(i*0.1)**2) for i in range(21)]
    road = {"valid": True, "status": "observed", "visibility": "both", "time_aligned": True,
            "frame_id": "base_link", "timestamp_s": 10.0, "measurement_timestamp_s": 9.75,
            "source_age_s": 0.25, "centerline_xy": points, "recommended_speed_scale": 1.0}
    state = {"valid": True, "speed_valid": True, "yaw_rate_valid": True,
             "speed_mps": 0.0, "yaw_rate_rps": 0.0, "frame_id": "base_link",
             "timestamp_s": 10.0, "source_age_s": {"wheel_speed": 0.01, "imu_angular_velocity": 0.01}}
    return road, state, {}, {}, 10.0, {"road": 1.5, "wheel_speed": 0.25, "imu_angular_velocity": 0.25}


def planner(**values):
    return ns["RoadCenterlinePlanner"](ns["PlanningSettings"](
        algorithm="road_centerline", max_source_age_s=1.5, reference_lifetime_s=0.2,
        allow_forward_only=values.pop("allow_forward_only", True), **values))


class CenterlineFollowChecks(unittest.TestCase):
    def end_straight_node(self, active=True, distance=0.30):
        node = self.learned_width_node(active=False)
        node.control_settings = ns["ControlSettings"](
            enabled=True, mode="mvp", drive_max=0.35, mvp_drive_feedforward=0.30,
            max_distance_m=0.0, max_run_time_s=0.0,
            end_straight_distance_m=distance, end_straight_drive_normalized=0.30,
            end_straight_max_time_s=8.0 if distance == 1.0 else 3.0)
        node.controller = ns["PolicyController"](node.control_settings)
        node.distance_limiter = ns["RunDistanceLimiter"](0.0, 0.2)
        if active:
            api["feed"](node, center=0.15, speed=0.2)
            api["start"](node)
            node.run_policy_step()
            self.assertNotEqual(node.action_publisher.messages[-1].steer, 0.0)
        return node

    def end_straight_tick(self, node, speed=0.2, empty=True, seconds=0.05):
        node.test_clock.advance(seconds)
        api["feed"](node, center=0.15, speed=speed, empty=empty)
        node.supervision_callback()
        node.run_policy_step()

    def test_geometry_loss_finishes_configured_straight_distance_then_latches_stop(self):
        for distance in (0.30, 1.0):
            with self.subTest(distance=distance):
                node = self.end_straight_node(distance=distance)
                self.end_straight_tick(node)
                self.assertEqual(node.fsm_state, 3)
                self.assertEqual((node.action_publisher.messages[-1].drive,
                                  node.action_publisher.messages[-1].steer), (0.3, 0.0))
                start_distance = node._end_straight_start_distance_m
                for _ in range(round(distance/0.01)-1):
                    self.end_straight_tick(node)
                    self.assertEqual(node.fsm_state, 3, node.state_reason)
                self.assertAlmostEqual(node.distance_limiter.distance_m-start_distance, distance-0.01)
                self.end_straight_tick(node)
                self.assertEqual(node.fsm_state, 2)
                self.assertIn("End straight distance reached", node.state_reason)
                self.assertAlmostEqual(node.distance_limiter.distance_m-start_distance, distance)
                self.assertEqual((node.action_publisher.messages[-1].drive,
                                  node.action_publisher.messages[-1].steer), (0.0, 0.0))
                self.end_straight_tick(node, empty=False)
                self.assertEqual(node.fsm_state, 2)  # Fresh cones never resume a stop.

    def test_end_straight_does_not_follow_returning_cones_or_reset_its_distance(self):
        node = self.end_straight_node()
        self.end_straight_tick(node)
        start_distance = node._end_straight_start_distance_m
        started_at = node._end_straight_started_at_s
        self.end_straight_tick(node, empty=False)
        self.assertEqual((node.action_publisher.messages[-1].drive,
                          node.action_publisher.messages[-1].steer), (0.3, 0.0))
        self.assertEqual(node._end_straight_start_distance_m, start_distance)
        self.assertEqual(node._end_straight_started_at_s, started_at)
        node.supervision_callback()
        self.assertAlmostEqual(node.distance_limiter.distance_m-start_distance, 0.01)

    def test_end_straight_operator_stop_and_stop_publishing_remain_immediate(self):
        for requested in (1, 2):
            node = self.end_straight_node()
            self.end_straight_tick(node)
            node.fsm_transition_request_callback(api["SimpleNamespace"](data=requested))
            count = len(node.action_publisher.messages)
            self.end_straight_tick(node, empty=False)
            self.assertEqual(node.fsm_state, requested)
            self.assertIsNone(node._end_straight_start_distance_m)
            if requested == 1:
                self.assertEqual(len(node.action_publisher.messages), count)
            else:
                self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)
            api["start"](node)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3)
            self.assertIsNone(node._end_straight_start_distance_m)
            self.assertNotEqual(node.action_publisher.messages[-1].steer, 0.0)

    def test_end_straight_stops_independently_on_motion_or_camera_loss(self):
        for missing in ("wheel_speed", "imu_angular_velocity", "cone_detections"):
            node = self.end_straight_node()
            self.end_straight_tick(node)
            node.test_clock.advance(0.05)
            api["feed"](node, speed=0.2, empty=True)
            node.observations[missing] = None
            node.supervision_callback()  # No selected policy callback required.
            self.assertEqual(node.fsm_state, 2, missing)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)
        for stale in ("wheel_speed", "imu_angular_velocity"):
            node = self.end_straight_node()
            self.end_straight_tick(node)
            previous = node.observations[stale]
            for _ in range(4):
                node.test_clock.advance(0.05)
                api["feed"](node, empty=True)
                node.observations[stale] = previous
                node.supervision_callback()
            self.assertEqual(node.fsm_state, 2, stale)
            self.assertIn(stale, node.state_reason)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)

    def test_end_straight_stalled_wheel_timeout_and_execution_gap_stop(self):
        node = self.end_straight_node()
        self.end_straight_tick(node, speed=0.0)
        for _ in range(61):
            self.end_straight_tick(node, speed=0.0)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("End straight time limit reached", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)
        node = self.end_straight_node()
        self.end_straight_tick(node)
        self.end_straight_tick(node, seconds=0.21)
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("clock/gap invalid", node.state_reason)

    def test_end_straight_cannot_start_without_prior_tracking_or_on_other_faults(self):
        node = self.end_straight_node(active=False)
        node.test_clock.advance(0.05)
        api["feed"](node, empty=True)
        api["start"](node)
        node.run_policy_step()
        self.assertIsNone(node._end_straight_start_distance_m)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)
        self.end_straight_tick(node)
        self.assertEqual(node.fsm_state, 2)
        self.assertIsNone(node._end_straight_start_distance_m)
        for fault in ("wheel_speed", "imu_angular_velocity", "reference_expiry", "unknown_width"):
            node = self.end_straight_node()
            node.test_clock.advance(0.05)
            api["feed"](node, empty=True)
            if fault in ("wheel_speed", "imu_angular_velocity"):
                node.observations[fault] = None
            elif fault == "reference_expiry":
                node.last_control_reference_deadline_s = node.test_clock.ros_ns/1e9
            else:
                node.estimator._clear_road_history()
                node.observations["cone_detections"].value["detections"] = [
                    (0.5, 0.5, 0.0, 2, 0.95), (1.0, 0.5, 0.0, 2, 0.95)]
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 2, fault)
            self.assertIsNone(node._end_straight_start_distance_m)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)

    def test_end_straight_parameters_reject_invalid_effort_distance_and_time(self):
        baseline = dict(mode="mvp", drive_max=0.35, end_straight_distance_m=0.3,
                        end_straight_drive_normalized=0.3)
        for changes in ({"end_straight_distance_m": -0.1},
                        {"end_straight_distance_m": math.nan},
                        {"end_straight_drive_normalized": 0.0},
                        {"end_straight_drive_normalized": 0.36},
                        {"end_straight_max_time_s": 0.0},
                        {"end_straight_max_time_s": math.inf}, {"mode": "calibrated"}):
            with self.assertRaises(ValueError):
                ns["ControlSettings"](**{**baseline, **changes})
        ns["ControlSettings"](drive_max=0.05)  # Disabled fallback preserves old profiles.

    def learned_width_node(self, active=True):
        node = api["make_node"](simulated=False, mvp=True)
        node.estimation_settings = ns["EstimationSettings"](
            road_hold_s=0.0, single_side_from_learned_width=True)
        node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
        node.planning_settings = planner().settings
        api["feed"](node)
        if active:
            api["start"](node)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
        return node

    def test_fresh_either_side_tracks_with_fixed_width_then_both_sides_restore(self):
        for visible_colour in (1, 2):
            node = self.learned_width_node()
            learned_at = node.estimator._learned_lane_width_stamp_s
            # Fresh single-side frames continue beyond the old 0.8 s hold;
            # the controller follows their current geometry, not a cached path.
            for _ in range(30):
                node.test_clock.advance(0.05)
                api["feed"](node, center=0.15)
                batch = node.observations["cone_detections"].value
                batch["detections"] = [p for p in batch["detections"] if p[3] == visible_colour]
                node.run_policy_step()
                self.assertEqual(node.fsm_state, 3, node.state_reason)
                self.assertTrue(node.planning_diagnostics["inferred_single_side"])
                self.assertAlmostEqual(node.planning_diagnostics["inward_offset_m"], 0.5)
                self.assertEqual(node.estimator._learned_lane_width_stamp_s, learned_at)
                self.assertGreater(node.action_publisher.messages[-1].drive, 0.0)
                for _, y in node.planning_output["path"]["samples_xy"]:
                    self.assertAlmostEqual(y, 0.15, places=6)
            node.test_clock.advance(0.05)
            api["feed"](node, center=0.15)
            batch = node.observations["cone_detections"].value
            batch["detections"] = [(x, y+(0.2 if colour == 2 else -0.2), z, colour, confidence)
                                   for x, y, z, colour, confidence in batch["detections"]]
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertFalse(node.planning_diagnostics["inferred_single_side"])
            self.assertAlmostEqual(node.estimator._learned_lane_width_m, 1.4)

    def test_single_side_cold_start_and_restart_do_not_reuse_a_previous_width(self):
        for previously_running in (False, True):
            node = self.learned_width_node(active=previously_running)
            if previously_running:
                node.fsm_transition_request_callback(api["SimpleNamespace"](data=2))
            node.test_clock.advance(0.05)
            api["feed"](node)
            batch = node.observations["cone_detections"].value
            batch["detections"] = [p for p in batch["detections"] if p[3] == 2]
            api["start"](node)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 2)
            self.assertIn("single_side_width_unknown", node.state_reason)
            self.assertIsNone(node.estimator._learned_lane_width_m)

    def test_single_side_extension_still_stops_on_missing_both_or_motion_feedback(self):
        for missing in ("both_boundaries", "wheel_speed", "imu_angular_velocity"):
            node = self.learned_width_node()
            if missing == "both_boundaries":
                node.test_clock.advance(0.05)
                api["feed"](node, empty=True)
            else:
                node.test_clock.advance(0.6)
                mono, ros = node.test_clock.monotonic, node.test_clock.ros_ns
                batch = node.observations["cone_detections"].value
                node._store("cone_detections", batch, ros, mono, ros)
                node.last_nonempty_cones = node.observations["cone_detections"]
                if missing == "wheel_speed":
                    node._store("imu_angular_velocity", (0, 0, 0), ros, mono, ros)
                else:
                    node._store("wheel_speed", 0.0, None, mono, ros)
            node.supervision_callback()
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 2, missing)
            if missing != "both_boundaries":
                self.assertIn(missing, node.state_reason)
            action = node.action_publisher.messages[-1]
            self.assertEqual((action.drive, action.steer), (0.0, 0.0))

    def test_recorded_dropout_policy_continues_only_with_learned_width_extension(self):
        for filename in ("car15_single_side_dropout.json", "car15_disjoint_boundary_return.json"):
            record = json.loads((Path(__file__).parent / "fixtures" / filename).read_text())
            for enabled in (False, True):
                node = self.learned_width_node(active=False)
                node.estimation_settings = ns["EstimationSettings"](**dict(
                    record["estimation_parameters_at_capture"], single_side_from_learned_width=enabled))
                node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
                for i, frame in enumerate(record["frames"]):
                    if i:
                        node.test_clock.advance(0.05)
                    api["feed"](node)
                    node.observations["cone_detections"].value["detections"] = frame["detections"]
                    if i == 0:
                        api["start"](node)
                    node.run_policy_step()
                    if i == 0:
                        self.assertEqual(node.fsm_state, 3, node.state_reason)
                self.assertEqual(node.fsm_state, 3 if enabled else 2, node.state_reason)
                if enabled:
                    self.assertTrue(node.planning_diagnostics["inferred_single_side"])
                    self.assertGreater(node.action_publisher.messages[-1].drive, 0.0)
                else:
                    self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)

    def test_recorded_curve_start_with_reduced_window_and_relaxed_width(self):
        record = json.loads((Path(__file__).parent / "fixtures" / "car15_curve_start.json").read_text())
        for limit, width_min in ((4.0, 0.3), (3.0, 0.3), (4.0, 0.15)):
            with self.subTest(forward_max_m=limit, width_min_m=width_min):
                node = api["make_node"](simulated=False, mvp=True)
                settings = dict(record["estimation_parameters_at_capture"],
                                road_forward_max_m=limit, road_width_min_m=width_min)
                node.estimation_settings = ns["EstimationSettings"](**settings)
                node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
                node.planning_settings = planner().settings
                node.control_settings = ns["ControlSettings"](
                    enabled=True, mode="mvp", mvp_lateral_kp=2.5,
                    mvp_heading_kp=1.0, mvp_steering_direction=-1.0,
                    steering_max_normalized=1.0, mvp_steering_slew_rate_per_s=2.0,
                    drive_max=0.35, mvp_drive_feedforward=0.30)
                node.controller = ns["PolicyController"](node.control_settings)
                api["feed"](node)
                node.observations["cone_detections"].value["detections"] = record["detections"]
                api["start"](node)
                node.run_policy_step()
                action = node.action_publisher.messages[-1]
                if limit == 4.0 and width_min == 0.3:
                    self.assertEqual(node.fsm_state, 2)
                    self.assertIn("invalid_width_or_order", node.state_reason)
                    self.assertEqual((action.drive, action.steer), (0.0, 0.0))
                else:
                    self.assertEqual(node.fsm_state, 3, node.state_reason)
                    self.assertGreater(action.drive, 0.0)
                    self.assertTrue(node.planning_output["valid"])
                    self.assertEqual(node.control_diagnostics["error_basis"], "observed_forward_target")
                    self.assertEqual(node.estimation_settings.road_width_min_m, width_min)
                    self.assertEqual(node.estimation_settings.road_width_max_m, 2.5)

    def test_distant_bend_uses_observed_target_without_false_local_offset(self):
        for side in (-1, 1):
            controller = ns["PolicyController"](ns["ControlSettings"](
                enabled=True, mode="mvp", mvp_lateral_kp=2.5,
                mvp_heading_kp=1.0, steering_max_normalized=1.0))
            near = fixture(offset=side*0.15)
            ref, _ = planner().plan(*near, mvp=True)
            _, before, _ = controller.calculate(ref, near[1], {}, 10.0, 0.05)
            args = list(fixture())
            xs = [0.95+i*0.05 for i in range(31)]
            points = [(x, side*(0.25-0.35*(x-0.95)-0.2*(x-0.95)**2)) for x in xs]
            args[0]["centerline_xy"] = points
            ref, _ = planner().plan(*args, mvp=True)
            original = deepcopy(ref["path"])
            _, after, info = controller.calculate(ref, args[1], {}, 10.0, 0.05)
            self.assertTrue(info["valid"], info["reason"])
            self.assertGreater(abs(info["path_error_m"]), 0.5)
            self.assertLess(abs(info["control_lateral_error_m"]), 0.15)
            self.assertEqual(info["error_basis"], "observed_forward_target")
            self.assertGreater(after*side, 0)
            self.assertLess(abs(after-before), 0.15)
            self.assertEqual(ref["path"], original)

    def test_vehicle_steering_profiles_track_both_bends_better(self):
        def run(radius, candidate):
            cfg = ns["ControlSettings"](enabled=True, mode="mvp",
                mvp_lateral_kp=2.5 if candidate else 1.5,
                mvp_heading_kp=1.0, mvp_steering_direction=-1.0,
                mvp_steering_slew_rate_per_s=2.0 if candidate else 0.0,
                steering_max_normalized=1.0 if candidate else 0.5)
            controller, follower = ns["PolicyController"](cfg), planner()
            x, y, heading, yaw = 0.0, 0.3, 0.0, 0.0
            for step in range(300):
                now, travelled = 10.0+step*0.05, step*0.01
                points = []
                for i in range(41):
                    s = travelled-0.5+i*0.05
                    px, py = radius*math.sin(s/radius), radius*(1-math.cos(s/radius))
                    dx, dy = px-x, py-y
                    points.append((math.cos(heading)*dx+math.sin(heading)*dy,
                                   -math.sin(heading)*dx+math.cos(heading)*dy))
                road, state, obstacles, limits, _, timeouts = fixture()
                road.update(timestamp_s=now, centerline_xy=points)
                state.update(timestamp_s=now, speed_mps=0.2, yaw_rate_rps=yaw)
                ref, _ = follower.plan(road, state, obstacles, limits, now, timeouts, mvp=True)
                self.assertTrue(ref["valid"], ref["reason"])
                _, steer, info = controller.calculate(ref, state, {}, now, 0.05)
                self.assertTrue(info["valid"], info["reason"])
                # Actual old 0.5 / candidate 0.8 interface scale; angle map and wheelbase remain
                # explicit synthetic assumptions, not car calibration.
                angle = cfg.mvp_steering_direction*steer*(0.8 if candidate else 0.5)*math.pi/4
                yaw = 0.2*math.tan(angle)/0.33
                x += 0.2*math.cos(heading)*0.05
                y += 0.2*math.sin(heading)*0.05
                heading += yaw*0.05
            return abs(math.hypot(x, y-radius)-abs(radius))
        for radius in (-1.8, 1.8):
            old, new = run(radius, False), run(radius, True)
            # The same initial offset is inside one bend and outside the other;
            # require improvement in both plus an absolute error bound.
            self.assertLess(new, old, (radius, old, new))
            self.assertLess(new, 0.15, (radius, old, new))

    def test_steering_changes_are_smooth_but_stop_is_immediate(self):
        controller = ns["PolicyController"](ns["ControlSettings"](
            enabled=True, mode="mvp", mvp_lateral_kp=2.5,
            steering_max_normalized=1.0, mvp_steering_slew_rate_per_s=2.0))
        last = 0.0
        for i, offset in enumerate([0.5]*10+[-0.5]*4):
            args = list(fixture(offset=offset))
            now = 10.0+i*0.05
            args[0]["timestamp_s"] = args[1]["timestamp_s"] = args[4] = now
            ref, _ = planner().plan(*args, mvp=True)
            _, steer, info = controller.calculate(ref, args[1], {}, now, 0.05)
            self.assertTrue(info["valid"], info["reason"])
            self.assertLessEqual(abs(steer-last), 0.1+1e-12)
            last = steer
        self.assertGreater(abs(last), 0.5)
        ref.update(stop_requested=True, target_speed_mps=0.0)
        drive, steer, info = controller.calculate(ref, args[1], {}, now, 0.05)
        self.assertTrue(info["stop_requested"])
        self.assertEqual((drive, steer), (0.0, 0.0))

    def test_yellow_left_blue_right_reaches_control_through_actual_policy(self):
        node = api["make_node"](simulated=False, mvp=True)
        node.planning_settings = planner().settings
        node.estimation_settings = ns["EstimationSettings"](left_colour=1, right_colour=2)
        node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
        api["feed"](node, center=0.1)
        batch = node.observations["cone_detections"].value
        batch["detections"] = [(x, y, z, 3-colour, confidence)
                               for x, y, z, colour, confidence in batch["detections"]]
        api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.estimation_output["road"]["valid"])
        self.assertAlmostEqual(node.estimation_output["road"]["centerline_xy"][0][1], 0.1)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)
        self.assertGreater(node.action_publisher.messages[-1].steer, 0)
        for left, right in ((1, 1), (2, 2), (0, 2), (1, 3), (True, 2), (1.0, 2)):
            with self.assertRaises(ValueError):
                ns["EstimationSettings"](left_colour=left, right_colour=right)

    def test_straight_and_both_bends_preserve_estimated_samples_and_fixed_speed(self):
        for bend in (0.0, -0.25, 0.25):
            with self.subTest(bend=bend):
                args = fixture(bend=bend)
                before = deepcopy(args)
                ref, diag = planner().plan(*args, mvp=True)
                self.assertTrue(ref["valid"], ref["reason"])
                self.assertEqual(ref["path"]["samples_xy"], args[0]["centerline_xy"])
                self.assertEqual(ref["target_speed_mps"], 0.2)
                self.assertFalse(diag["candidate_search_enabled"])
                self.assertEqual(args, before)
                evaluated = ns["evaluate_planning_path"](ref, 0.4, 10.0)
                self.assertAlmostEqual(evaluated["position_xy_m"][1], bend*0.4**2)
                self.assertIsNone(ns["evaluate_planning_path"](ref, 2.1, 10.0))
                self.assertIsNone(ns["evaluate_planning_path"](ref, 0.4, 10.0+ref["valid_for_s"]))
                geom = ns["control_path_geometry"](ref["path"])
                if bend:
                    self.assertGreater(geom["path_heading_rad"]*bend, 0)
                    self.assertGreater(geom["curvature_1pm"]*bend, 0)
                else:
                    self.assertAlmostEqual(geom["path_heading_rad"], 0)

    def test_large_offset_is_left_for_feedback_not_recovery_search(self):
        for offset in (-0.55, 0.55):
            args = fixture(offset=offset)
            ref, _ = planner().plan(*args, mvp=True)
            self.assertTrue(ref["valid"], ref["reason"])
            cfg = ns["ControlSettings"](enabled=True, mode="mvp", drive_max=0.35,
                mvp_drive_feedforward=0.30, speed_kp=0.35, speed_ki=0.08)
            drive, steer, info = ns["PolicyController"](cfg).calculate(ref, args[1], {}, 10.0, 0.05)
            self.assertTrue(info["valid"], info["reason"])
            self.assertGreater(steer*offset, 0)
            self.assertAlmostEqual(drive, 0.35)

    def test_forward_only_uses_observed_geometry_without_extending_or_mutating(self):
        args = list(fixture(bend=0.2))
        forward = deepcopy(args[0])
        forward.update(valid=False, geometry_valid=True, status="near_field_unobserved")
        forward["centerline_xy"] = [(x, y) for x, y in forward["centerline_xy"] if x >= 0.6]
        args[0] = {**deepcopy(forward), "centerline_xy": [], "time_aligned": False,
                   "forward_geometry": forward}
        before = deepcopy(args[0])
        ref, diag = planner().plan(*args, mvp=True)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertTrue(diag["forward_only"])
        self.assertEqual(ref["path"]["samples_xy"], forward["centerline_xy"])
        self.assertEqual(args[0], before)
        geom = ns["control_path_geometry"](ref["path"])
        self.assertGreaterEqual(geom["preview_x_m"], 0.6)
        self.assertFalse(planner(allow_forward_only=False).plan(*args, mvp=True)[0]["valid"])
        forward["geometry_valid"] = False
        self.assertFalse(planner().plan(*args, mvp=True)[0]["valid"])

    def test_prediction_retains_constant_speed_but_does_not_renew_lifetime(self):
        args = list(fixture(bend=0.2))
        args[0].update(degraded=True, status="predicted_history",
                       recommended_speed_scale=0.5, prediction_remaining_s=0.07)
        ref, diag = planner().plan(*args, mvp=True)
        self.assertTrue(ref["valid"], ref["reason"])
        self.assertEqual(ref["target_speed_mps"], 0.2)
        self.assertEqual(ref["valid_for_s"], 0.07)
        self.assertEqual(diag["speed_policy"], "constant")
        args[0]["prediction_remaining_s"] = 0
        self.assertEqual(planner().plan(*args, mvp=True)[0]["reason"], "expired_road_prediction")

    def test_source_deadlines_and_invalid_feedback_still_stop(self):
        for name in ("road", "wheel_speed", "imu_angular_velocity"):
            args = list(fixture())
            if name == "road":
                args[0]["source_age_s"] = 1.5
            else:
                args[1]["source_age_s"][name] = 0.25
            ref, _ = planner().plan(*args, mvp=True)
            self.assertFalse(ref["valid"])
            self.assertEqual(ref["reason"], "stale_"+name)
            self.assertEqual(ref["target_speed_mps"], 0)
        for key in ("speed_valid", "yaw_rate_valid"):
            args = list(fixture()); args[1][key] = False
            self.assertFalse(planner().plan(*args, mvp=True)[0]["valid"])

    def test_malformed_frame_samples_and_non_mvp_are_rejected(self):
        for change in ("frame", "timestamp", "nan", "reverse", "gap", "behind"):
            args = list(fixture())
            if change == "frame": args[0]["frame_id"] = "map"
            elif change == "timestamp": args[0]["timestamp_s"] = 9.9
            elif change == "nan": args[0]["centerline_xy"][1] = (0.1, math.nan)
            elif change == "reverse": args[0]["centerline_xy"].reverse()
            elif change == "gap": args[0]["centerline_xy"] = [(0, 0), (2, 0)]
            elif change == "behind": args[0]["centerline_xy"] = [(-0.2, 0), (-0.1, 0)]
            self.assertFalse(planner().plan(*args, mvp=True)[0]["valid"], change)
        self.assertEqual(planner().plan(*fixture(), mvp=False)[0]["reason"], "road_centerline_requires_mvp")

    def test_real_policy_methods_start_without_three_frames_and_keep_controller(self):
        node = api["make_node"](simulated=False, mvp=True)
        node.planning_settings = planner().settings
        node.estimation_settings = ns["EstimationSettings"](history_propagation="continuous",
            continuous_road_output=True, motion_max_gap_s=0.25, motion_max_interval_s=0.5)
        node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
        node.control_settings = ns["ControlSettings"](enabled=True, mode="mvp", drive_max=0.35,
            speed_kp=0.35, speed_ki=0.08, mvp_drive_feedforward=0.30,
            mvp_steering_direction=-1.0, mvp_lateral_kp=1.5, mvp_heading_kp=1.0)
        api["feed"](node, center=0.3)
        api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertIsInstance(node.planner, ns["RoadCenterlinePlanner"])
        self.assertEqual(node.planning_output["target_speed_mps"], 0.2)
        self.assertAlmostEqual(node.action_publisher.messages[-1].drive, 0.35)
        self.assertLess(node.action_publisher.messages[-1].steer, 0)
        controller = node.controller
        for _ in range(3):
            node.test_clock.advance(0.05)
            api["feed"](node, center=0.25)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            self.assertIs(node.controller, controller)
            self.assertEqual(node.planning_output["target_speed_mps"], 0.2)
        node.observations["imu_angular_velocity"] = None
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_closed_loop_straight_and_gentle_arcs_correct_initial_offset(self):
        # Synthetic bicycle sensitivity check, not identified car dynamics.
        # Ideal centerline/feedback; no claim about real sensing, ESC or latency.
        for radius in (None, -3.0, 3.0):
            with self.subTest(radius=radius):
                cfg = ns["ControlSettings"](enabled=True, mode="mvp", drive_max=0.35,
                    mvp_drive_feedforward=0.30, mvp_steering_direction=-1.0,
                    mvp_lateral_kp=1.5, mvp_heading_kp=1.0)
                controller, follower = ns["PolicyController"](cfg), planner()
                x, y, heading, yaw_rate = 0.0, 0.3, 0.0, 0.0
                for step in range(200):
                    now, travelled = 10.0+step*0.05, step*0.01
                    points = []
                    for i in range(61):
                        s = travelled-0.5+i*0.05
                        px, py = ((s, 0.0) if radius is None else
                                  (radius*math.sin(s/radius), radius*(1-math.cos(s/radius))))
                        dx, dy = px-x, py-y
                        points.append((math.cos(heading)*dx+math.sin(heading)*dy,
                                       -math.sin(heading)*dx+math.cos(heading)*dy))
                    road, state, obstacles, limits, _, timeouts = fixture()
                    road.update(timestamp_s=now, centerline_xy=points)
                    state.update(timestamp_s=now, speed_mps=0.2, yaw_rate_rps=yaw_rate)
                    ref, _ = follower.plan(road, state, obstacles, limits, now, timeouts, mvp=True)
                    self.assertTrue(ref["valid"], ref["reason"])
                    _, steer, info = controller.calculate(ref, state, {}, now, 0.05)
                    self.assertTrue(info["valid"], info["reason"])
                    self.assertEqual(ref["target_speed_mps"], 0.2)
                    angle = cfg.mvp_steering_direction*steer*math.pi/4
                    yaw_rate = 0.2*math.tan(angle)/0.33
                    x += 0.2*math.cos(heading)*0.05
                    y += 0.2*math.sin(heading)*0.05
                    heading += yaw_rate*0.05
                error = abs(y) if radius is None else abs(math.hypot(x, y-radius)-abs(radius))
                self.assertLess(error, 0.15)


class OrderedRoadChecks(unittest.TestCase):
    def batch(self, left, right):
        return api["api"]["cones"](left, right)

    def test_recorded_car05_false_nearest_blue_cluster_does_not_stop_valid_visible_side(self):
        record = json.loads((Path(__file__).parent/"fixtures"/"car05_ordered_chain_false_seed.json").read_text())
        pipeline = ns["EstimationPipeline"](self.settings(
            left_colour=1, right_colour=2, min_cone_confidence=0.25,
            road_max_gap_m=2., huber_delta_m=0.2, road_max_abs_slope=3.))
        for frame in record["frames"]:
            road = pipeline._history_road({"detections": frame["detections"]},
                round(frame["stamp_s"]*1e9), 0., frame["stamp_s"])
            self.assertTrue(road["valid"], road["status"])
        self.assertEqual(road["visibility"], "right_only")
        self.assertTrue(road["single_side_inferred_from_learned_width"])
        # The short blue-labelled group beside yellow is closer to the car.
        # The real right-side support starts below y=-0.3 instead.
        blue = road["_boundary_fits"][2]["support_points"]
        self.assertTrue(all(p[1] < -0.3 for p in blue[:4]))
        self.assertGreater(len(blue), 10)

    def test_recorded_short_noisy_near_boundary_uses_longer_opposite_support(self):
        record = json.loads((Path(__file__).parent/"fixtures"/"car05_ordered_short_near_boundary.json").read_text())
        pipeline = ns["EstimationPipeline"](self.settings(
            left_colour=1, right_colour=2, min_cone_confidence=0.25,
            road_max_gap_m=2., huber_delta_m=0.2, road_max_abs_slope=3.))
        for frame in record["frames"]:
            road = pipeline._history_road({"detections": frame["detections"]},
                round(frame["stamp_s"]*1e9), 0., frame["stamp_s"])
            self.assertTrue(road["valid"], road["status"])
        self.assertEqual(road["visibility"], "right_only")
        self.assertTrue(road["single_side_inferred_from_learned_width"])
        self.assertGreater(ns["_path_lengths"](road["centerline_xy"])[-1], 2.)

    def settings(self, **changes):
        values = dict(road_geometry="ordered_xy", road_min_coverage_m=0.3,
                      road_forward_max_m=4.0, road_rear_support_m=1.0,
                      road_width_min_m=0.15, road_width_max_m=2.5,
                      single_side_from_learned_width=True,
                      max_source_age_s=1.5, motion_max_gap_s=0.25, motion_max_interval_s=0.5,
                      history_propagation="continuous", continuous_road_output=True,
                      road_hold_s=0.8, road_prediction_max_distance_m=0.5)
        return ns["EstimationSettings"](**{**values, **changes})

    def hairpin(self, turn=1):
        # Entry, concentric semicircle and a returning leg. Colours describe
        # travel-side boundaries, including after the heading rotates 180 deg.
        sides = []
        for radius in (0.7, 1.3):
            points = [(-0.6, 1-radius), (-0.3, 1-radius)]
            points += [(radius*math.cos(-math.pi/2+math.pi*i/24),
                        1+radius*math.sin(-math.pi/2+math.pi*i/24)) for i in range(25)]
            points += [(-0.3, 1+radius), (-0.6, 1+radius)]
            sides.append([(x, turn*y) for x, y in points])
        return sides if turn == 1 else list(reversed(sides))

    def reference(self, points, **changes):
        lengths = ns["_path_lengths"](points)
        count = math.ceil(lengths[-1]/0.1)
        points = [ns["_path_sample"](points, lengths, lengths[-1]*i/count) for i in range(count+1)]
        args = list(fixture())
        args[0].update(centerline_xy=points, geometry_encoding="ordered_xy", **changes)
        reference, diagnostic = planner().plan(*args, mvp=True)
        self.assertTrue(reference["valid"], reference["reason"])
        self.assertEqual(reference["target_speed_mps"], 0.2)
        return reference, args[1], diagnostic

    def test_unordered_hairpin_keeps_entry_arc_and_return_without_closing_loop(self):
        import random
        for turn in (-1, 1):
            left, right = self.hairpin(turn)
            random.Random(17).shuffle(left)
            random.Random(32).shuffle(right)
            road = ns["EstimationPipeline"](self.settings()).road(self.batch(left, right), 10_000_000_000, 0.0)
            self.assertTrue(road["valid"], road["status"])
            self.assertAlmostEqual(road["lane_width_m"], 0.6, delta=0.04)
            points = road["centerline_xy"]
            self.assertLess(points[-1][0], -0.3)
            self.assertAlmostEqual(points[-1][1], turn*2.0, delta=0.04)
            self.assertTrue(any(b[0] < a[0] for a, b in zip(points, points[1:])))
            for x, y in points:
                if x > 0.1:
                    self.assertAlmostEqual(math.hypot(x, y-turn), 1.0, delta=0.08)
            for fit in road["_boundary_fits"].values():
                chain = fit["support_points"]
                self.assertLess(turn*chain[0][1], 0.5)
                self.assertGreater(turn*chain[-1][1], 1.5)
            self.reference(points)

    def test_right_angle_can_contain_vertical_samples_and_preview_around_corner(self):
        left = [(x, 0.3) for x in (-0.6, -0.3, 0., 0.3, 0.5, 0.7)]
        left += [(0.7, y) for y in (0.5, 0.7, 0.9, 1.1, 1.3, 1.5)]
        right = [(x, -0.3) for x in (-0.6, -0.3, 0., 0.3, 0.6, 0.9, 1.1, 1.3)]
        right += [(1.3, y) for y in (0., 0.3, 0.6, 0.9, 1.2, 1.5)]
        road = ns["EstimationPipeline"](self.settings()).road(self.batch(left, right), 10_000_000_000, 0.)
        self.assertTrue(road["valid"], road["status"])
        self.assertGreater(road["centerline_xy"][-1][1], 1.2)
        points = [(x-0.85, y) for x, y in road["centerline_xy"]]
        ref, state, _ = self.reference(points)
        geometry = ns["control_path_geometry"](ref["path"])
        self.assertGreater(geometry["preview_y_m"], 0.1)
        self.assertGreater(geometry["path_heading_rad"], 0.7)
        self.assertAlmostEqual(geometry["preview_s_m"]-geometry["closest_s_m"], 0.35)
        controller = ns["PolicyController"](ns["ControlSettings"](
            mode="mvp", mvp_steering_direction=-1, mvp_lateral_kp=2.5, mvp_heading_kp=1.0))
        drive, steer, diagnostic = controller.calculate(ref, state, {}, 10., 0.05)
        self.assertTrue(diagnostic["valid"])
        self.assertLess(steer, 0.)
        self.assertGreater(drive, 0.)

    def test_single_side_hairpin_offsets_normally_and_two_sides_return(self):
        for missing in (0, 1):
            left, right = self.hairpin()
            pipeline = ns["EstimationPipeline"](self.settings())
            both = pipeline._history_road(self.batch(left, right), 10_000_000_000, 0., 10.)
            partial = [left, right]
            partial[missing] = []
            road = pipeline._history_road(self.batch(*partial), 10_100_000_000, 0., 10.1)
            self.assertTrue(road["valid"], road["status"])
            self.assertTrue(road["single_side_inferred_from_learned_width"])
            self.assertAlmostEqual(road["inward_offset_m"], both["lane_width_m"]/2)
            for x, y in road["centerline_xy"]:
                if x > 0.1:
                    self.assertAlmostEqual(math.hypot(x, y-1), 1., delta=0.08)
            restored = pipeline._history_road(self.batch(left, right), 10_200_000_000, 0., 10.2)
            self.assertTrue(restored["valid"], restored["status"])
            self.assertEqual(restored["visibility"], "both")

    def test_recorded_dropout_and_far_return_remain_usable_in_ordered_mode(self):
        for filename in ("car15_single_side_dropout.json", "car15_disjoint_boundary_return.json"):
            record = json.loads((Path(__file__).parent/"fixtures"/filename).read_text())
            pipeline = ns["EstimationPipeline"](ns["EstimationSettings"](**dict(
                record["estimation_parameters_at_capture"], road_geometry="ordered_xy",
                single_side_from_learned_width=True)))
            for i, frame in enumerate(record["frames"]):
                road = pipeline._history_road({"detections": frame["detections"]},
                                             10_000_000_000+i*100_000_000, 0., 10+i*0.1)
                self.assertTrue(road["valid"], (filename, i, road["status"]))
            self.assertTrue(road["single_side_inferred_from_learned_width"])

    def test_projection_does_not_jump_to_nearer_rear_facing_exit(self):
        # Exit passes nearer to the origin than the entry. Its tangent faces
        # backward, so acquiring it would skip the entire U turn.
        points = [(-0.3, 0.2), (0.6, 0.2), (0.9, 0.5), (0.6, 0.8),
                  (0.3, 0.05), (-0.3, 0.05)]
        ref, _, _ = self.reference(points)
        geometry = ns["control_path_geometry"](ref["path"])
        self.assertAlmostEqual(geometry["closest_y_m"], 0.2)
        self.assertLess(geometry["closest_s_m"], 1.)

    def test_delay_alignment_preserves_order_after_x_turns_back(self):
        pipeline = ns["EstimationPipeline"](self.settings())
        left, right = self.hairpin()
        road = pipeline.road(self.batch(left, right), 10_000_000_000, 0.)
        for stamp in (10., 10.1):
            pipeline.motion.observe("wheel_speed", 0.2, stamp, stamp, stamp)
            pipeline.motion.observe("imu_angular_velocity", 0.3, stamp, stamp, stamp)
        aligned = pipeline._align_road(road, 10.1, True)
        self.assertTrue(aligned["valid"], aligned["status"])
        self.assertTrue(aligned["time_aligned"])
        self.assertEqual(aligned["measurement_timestamp_s"], 10.)
        self.assertTrue(any(b[0] < a[0] for a, b in zip(aligned["centerline_xy"], aligned["centerline_xy"][1:])))

    def test_pid_completes_left_and_right_hairpins_in_labelled_bicycle_fixture(self):
        # Explicit OFFLINE model: L=0.33 m, normalized steering maps linearly
        # to +/- pi/4 wheel angle, perfect speed/pose, no servo/ESC dynamics.
        # This verifies path progression and feedback, not real-car feasibility.
        raw = [(-0.6, 0.), (-0.3, 0.)]
        raw += [(math.cos(-math.pi/2+math.pi*i/80),
                 1+math.sin(-math.pi/2+math.pi*i/80)) for i in range(81)]
        raw += [(-0.3, 2.), (-0.6, 2.)]
        lengths = ns["_path_lengths"](raw)
        world = [ns["_path_sample"](raw, lengths, lengths[-1]*i/100) for i in range(101)]
        for turn in (-1, 1):
            x, y, yaw, errors = -0.4, 0., 0., []
            controller = ns["PolicyController"](ns["ControlSettings"](
                mode="mvp", mvp_lateral_kp=2.5, mvp_heading_kp=1.,
                mvp_steering_direction=-1., steering_max_normalized=1.,
                mvp_steering_slew_rate_per_s=2., mvp_error_filter_tau_s=0.1))
            for tick in range(600):
                now = 10+tick*0.05
                c, s = math.cos(yaw), math.sin(yaw)
                body = [(c*(px-x)+s*(turn*py-y), -s*(px-x)+c*(turn*py-y)) for px, py in world]
                args = list(fixture())
                args[0].update(centerline_xy=body, geometry_encoding="ordered_xy", timestamp_s=now)
                args[1].update(timestamp_s=now, speed_mps=0.2)
                args[4] = now
                ref, _ = planner().plan(*args, mvp=True)
                _, steer, diagnostic = controller.calculate(ref, args[1], {}, now, 0.05)
                self.assertTrue(diagnostic["valid"], diagnostic)
                self.assertEqual(ref["target_speed_mps"], 0.2)
                errors.append(abs(diagnostic["path_error_m"]))
                wheel_angle = -steer*math.pi/4
                x += 0.01*math.cos(yaw)
                y += 0.01*math.sin(yaw)
                yaw += 0.01*math.tan(wheel_angle)/0.33
                if x < -0.3 and turn*y > 1.8:
                    break
            self.assertLess(tick, 599)
            self.assertGreater(turn*y, 1.8)
            self.assertAlmostEqual(turn*y, 2., delta=0.12)
            self.assertLess(max(errors), 0.1)

    def test_error_filter_smooths_source_switch_but_stop_bypasses_filter(self):
        controller = ns["PolicyController"](ns["ControlSettings"](
            mode="mvp", mvp_error_filter_tau_s=0.1, mvp_lateral_kp=1.0, mvp_heading_kp=0.0))
        for y in (0.1, -0.1):
            ref, state, _ = self.reference([(-0.3, y), (0., y), (0.3, y), (0.6, y)])
            _, _, diagnostic = controller.calculate(ref, state, {}, 10., 0.05)
        self.assertAlmostEqual(diagnostic["raw_lateral_error_m"], -0.1)
        self.assertGreater(diagnostic["control_lateral_error_m"], 0.)
        ref["stop_requested"] = True
        self.assertEqual(controller.calculate(ref, state, {}, 10., 0.05)[:2], (0., 0.))
        self.assertIsNone(controller._mvp_filtered_errors)

    def test_corner_entry_anticipates_preview_tangent_in_both_directions(self):
        # Static controller calculation only: no vehicle/dynamics model.
        for turn in (-1, 1):
            ref, state, _ = self.reference([(0.25, 0.), (0.4, 0.),
                                           (0.4, turn*0.4), (0.4, turn*0.8)])
            outputs = []
            for enabled in (False, True):
                controller = ns["PolicyController"](ns["ControlSettings"](
                    mode="mvp", mvp_ordered_cornering_enabled=enabled,
                    mvp_lateral_kp=0., mvp_heading_kp=1.,
                    mvp_steering_direction=-1., steering_max_normalized=1.))
                _, steer, info = controller.calculate(ref, state, {}, 10., 0.05)
                self.assertTrue(info["valid"], info)
                self.assertLess(turn*steer, 0.)
                outputs.append(info)
            self.assertGreater(turn*(outputs[1]["raw_heading_error_rad"]
                                     -outputs[0]["raw_heading_error_rad"]), 0.3)
            self.assertEqual(outputs[1]["error_basis"], "blended_ordered_corner")
            self.assertGreater(outputs[1]["corner_local_weight"], 0.)
            self.assertLess(outputs[1]["corner_local_weight"], 1.)

    def test_corner_feedback_is_continuous_at_observed_entry_projection(self):
        values = []
        for start in (-0.0501, -0.0499):
            ref, state, _ = self.reference([(x, x+0.1) for x in (start, start+0.5, start+1.)])
            controller = ns["PolicyController"](ns["ControlSettings"](
                mode="mvp", mvp_ordered_cornering_enabled=True,
                mvp_lateral_kp=1., mvp_heading_kp=0.5, steering_max_normalized=1.))
            _, steer, info = controller.calculate(ref, state, {}, 10., 0.05)
            self.assertTrue(info["valid"], info)
            self.assertEqual(info["error_basis"], "blended_ordered_corner")
            values.append((steer, info["raw_lateral_error_m"], info["raw_heading_error_rad"]))
        for before, after in zip(*values):
            self.assertLess(abs(before-after), 0.002)

    def test_distant_corner_support_has_less_tangent_weight(self):
        weights = []
        for advance in (0., 2.):
            ref, state, _ = self.reference([(0.25+advance, 0.), (0.4+advance, 0.),
                                           (0.4+advance, 0.4), (0.4+advance, 0.8)])
            controller = ns["PolicyController"](ns["ControlSettings"](
                mode="mvp", mvp_ordered_cornering_enabled=True))
            _, _, info = controller.calculate(ref, state, {}, 10., 0.05)
            self.assertTrue(info["valid"], info)
            weights.append(info["corner_local_weight"])
        self.assertLess(weights[1], weights[0]/3)

    def test_corner_control_keeps_straight_zero_and_stops_without_slew_delay(self):
        controller = ns["PolicyController"](ns["ControlSettings"](
            mode="mvp", mvp_ordered_cornering_enabled=True,
            mvp_steering_direction=-1., mvp_lateral_kp=2.5, mvp_heading_kp=1.5,
            steering_max_normalized=1., mvp_steering_slew_rate_per_s=4.,
            mvp_error_filter_tau_s=0.1))
        ref, state, _ = self.reference([(0.25, 0.), (0.6, 0.), (1., 0.)])
        self.assertEqual(controller.calculate(ref, state, {}, 10., 0.05)[1], 0.)
        ref, state, _ = self.reference([(0.25, 0.), (0.4, 0.), (0.4, 0.4), (0.4, 0.8)])
        drive, steer, info = controller.calculate(ref, state, {}, 10., 0.05)
        self.assertTrue(info["valid"], info)
        self.assertGreater(drive, 0.)
        self.assertLess(steer, 0.)
        self.assertLessEqual(abs(steer), 0.2)
        ref["stop_requested"] = True
        self.assertEqual(controller.calculate(ref, state, {}, 10., 0.05)[:2], (0., 0.))
        self.assertIsNone(controller._mvp_filtered_errors)
        ref["stop_requested"] = False
        self.assertEqual(controller.calculate(ref, state, {}, 10.3, 0.05)[:2], (0., 0.))
        self.assertEqual(controller._mvp_previous_steer, 0.)

    def test_corner_mode_requires_boolean_setting(self):
        for value in (1, 0.6, "true", None):
            with self.assertRaises(ValueError):
                ns["ControlSettings"](mvp_ordered_cornering_enabled=value)

    def test_clock_sensor_loss_and_prediction_budget_still_stop_without_terminal_straight(self):
        for fault in ("both", "wheel_speed", "imu_angular_velocity"):
            node = CenterlineFollowChecks().learned_width_node(active=False)
            node.estimation_settings = self.settings()
            node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
            node.control_settings.end_straight_distance_m = 0.
            api["feed"](node, speed=0.2)
            api["start"](node)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3, node.state_reason)
            for _ in range(19):
                node.test_clock.advance(0.05)
                api["feed"](node, speed=0.2, empty=True)
                if fault != "both":
                    node.observations[fault] = None
                node.supervision_callback()
                node.run_policy_step()
            self.assertEqual(node.fsm_state, 2, node.state_reason)
            self.assertIsNone(node._end_straight_start_distance_m)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0.)

    def test_short_blind_gap_after_single_side_keeps_real_source_epoch_then_expires(self):
        node = CenterlineFollowChecks().learned_width_node(active=False)
        node.sensor_timeout_s["cone_detections"] = 1.5
        node.estimation_settings = self.settings()
        node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
        api["feed"](node, center=0.15, speed=0.2)
        api["start"](node)
        node.run_policy_step()
        width_stamp = node.estimator._learned_lane_width_stamp_s
        node.test_clock.advance(0.05)
        api["feed"](node, center=0.15, speed=0.2)
        batch = node.observations["cone_detections"].value
        batch["detections"] = [p for p in batch["detections"] if p[3] == node.estimation_settings.left_colour]
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertTrue(node.planning_diagnostics["inferred_single_side"])
        single_stamp = node.test_clock.ros_ns/1e9
        for tick in range(1, 19):
            node.test_clock.advance(0.05)
            api["feed"](node, speed=0.2, empty=True)
            node.supervision_callback()
            node.run_policy_step()
            if tick <= 15:
                self.assertEqual(node.fsm_state, 3, node.state_reason)
                road = node.estimation_output["road"]
                self.assertTrue(road["degraded"])
                self.assertEqual(road["measurement_timestamp_s"], single_stamp)
                self.assertEqual(node.estimator._learned_lane_width_stamp_s, width_stamp)
                self.assertIn("inferred_width", road["boundary_source"].values())
        self.assertEqual(node.fsm_state, 2)
        self.assertIsNone(node._end_straight_start_distance_m)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)


if __name__ == "__main__":
    unittest.main(verbosity=2)
