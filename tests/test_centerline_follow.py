"""Focused portable checks of the simple follower using actual policy methods.

No ROS/device connection or actuator publisher is created. Recorder ports are
the same source-method harness used by test_control, not a vehicle simulation.
"""
from copy import deepcopy
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
