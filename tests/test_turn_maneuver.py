"""Static geometry/intent and controlled motion-history checks; no vehicle plant."""
from copy import deepcopy
import math
from pathlib import Path
import runpy
import unittest

api = runpy.run_path(str(Path(__file__).with_name("test_centerline_follow.py")))
ns = api["ns"]


def hairpin(turn=1, radius=0.55, entrance=0.0):
    points = [(entrance-0.3, 0.), (entrance, 0.)]
    points += [(entrance+radius*math.cos(-math.pi/2+math.pi*i/40),
                turn*radius*(1+math.sin(-math.pi/2+math.pi*i/40))) for i in range(1, 41)]
    points.append((entrance-0.3, turn*2*radius))
    return points


class TurnManeuverChecks(unittest.TestCase):
    def setUp(self):
        self.settings = ns["TurnManeuverSettings"](enabled=True)
        self.planning = api["planner"]().settings
        self.subject = ns["TurnManeuverPlanner"](self.settings, self.planning)
        self.motion = ns["EstimationMotionHistory"](ns["EstimationSettings"](
            motion_max_gap_s=0.25, motion_max_interval_s=0.5))
        self.now = 10.
        self.limits = {"road": 1.5, "wheel_speed": 0.25, "imu_angular_velocity": 0.25}
        self.observe()

    def observe(self, speed=0.2, gyro=0.):
        for name, value in (("wheel_speed", speed), ("imu_angular_velocity", gyro)):
            self.motion.observe(name, value, self.now, self.now, self.now)

    def step(self, points, visibility="both", source=None, reason=None, seconds=0.1, gyro=0.):
        self.now += seconds
        self.observe(gyro=gyro)
        ref, state, _ = api["OrderedRoadChecks"]().reference(points)
        ref.update(timestamp_s=self.now)
        state.update(timestamp_s=self.now, speed_mps=0.2)
        source = self.now-0.2 if source is None else source
        road = {"valid": reason is None, "visibility": visibility, "lane_width_m": 0.85,
                "measurement_timestamp_s": source, "timestamp_s": self.now, "degraded": False}
        if reason:
            ref.update(valid=False, path=None, stop_requested=True, reason=reason, valid_for_s=0.)
        out, diag = self.subject.update(ref, {}, road, state, self.motion, self.limits)
        return out, diag

    def confirm(self, points=None):
        for _ in range(3):
            result = self.step(hairpin() if points is None else points)
        return result

    def test_missing_one_colour_on_straight_does_not_trigger_a_turn(self):
        for side in ("left_only", "right_only"):
            for _ in range(5):
                _, diag = self.step([(-0.3, 0.), (0.6, 0.), (1.2, 0.)], side)
                self.assertEqual(diag["turn_phase"], "FOLLOW")
                self.assertEqual(diag["turn_candidate_frames"], 0)

    def test_ninety_degree_prefix_stays_unknown_and_both_hairpins_are_recognised(self):
        right_angle = [(-0.3, 0.), (0.6, 0.), (0.6, 0.5), (0.6, 1.2)]
        shape = self.subject.shape(right_angle)
        self.assertTrue(shape["sharp"])
        self.assertEqual(shape["turn_type"], "UNKNOWN")
        for direction in (-1, 1):
            shape = self.subject.shape(hairpin(direction))
            self.assertTrue(shape["sharp"])
            self.assertEqual(shape["turn_type"], "UTURN")
            self.assertEqual(shape["direction"], direction)

    def test_wide_curve_and_s_bend_do_not_trigger_uturn(self):
        gentle = [(x/10, 0.04*(x/10)**2) for x in range(-3, 26)]
        s_bend = [(-0.3, 0.), (0.3, 0.), (0.6, 0.3), (0.9, 0.3), (1.2, 0.), (1.5, 0.)]
        for points in (gentle, s_bend):
            feature = self.subject.shape(points)
            self.assertTrue(feature is None or not feature["sharp"])
            self.assertTrue(feature is None or feature["turn_type"] == "UNKNOWN")

    def test_repeated_camera_batch_cannot_confirm_intent(self):
        for _ in range(5):
            _, diag = self.step(hairpin(), source=9.9)
        self.assertEqual(diag["turn_phase"], "FOLLOW")
        self.assertEqual(diag["turn_candidate_frames"], 1)

    def test_prediction_and_interrupted_candidate_do_not_confirm_intent(self):
        self.step(hairpin())
        self.step(hairpin())
        self.step(hairpin(), reason="invalid_road_insufficient_geometry")
        _, diag = self.step(hairpin())
        self.assertEqual(diag["turn_candidate_frames"], 1)
        self.assertEqual(diag["turn_phase"], "FOLLOW")

    def test_confirmed_direction_does_not_depend_on_missing_side(self):
        for direction in (-1, 1):
            self.subject.clear()
            for _ in range(3):
                _, diag = self.step(hairpin(direction), visibility="right_only")
            self.assertEqual(diag["turn_type"], "UTURN")
            self.assertEqual(diag["turn_direction"], direction)
            self.assertIn(diag["turn_phase"], ("PREPARE", "TURN"))

    def test_unknown_calibration_keeps_outward_setup_off(self):
        _, diag = self.confirm(hairpin(entrance=0.8))
        self.assertEqual(diag["turn_phase"], "PREPARE")
        self.assertEqual(diag["setup_unavailable"], "radius_or_body_width_unknown")
        self.assertEqual(self.subject.outward_offset, 0.)

    def test_calibrated_narrow_turn_offsets_entrance_outwards_then_commits(self):
        self.subject.settings = ns["TurnManeuverSettings"](
            enabled=True, minimum_turn_radius_m=0.8, body_half_width_m=0.12,
            setup_distance_m=0.04)
        out, diag = self.confirm(hairpin(entrance=0.9))
        self.assertEqual(diag["turn_phase"], "SETUP")
        self.assertLess(self.subject.outward_offset, 0.)
        self.assertLess(out["path"]["samples_xy"][0][1], 0.)
        for _ in range(3):
            out, diag = self.step(hairpin(entrance=0.9))
        self.assertEqual(diag["turn_phase"], "TURN")
        self.assertAlmostEqual(out["path"]["samples_xy"][0][1], 0.)

    def test_outward_setup_requires_both_sides_and_available_room(self):
        self.subject.settings = ns["TurnManeuverSettings"](
            enabled=True, minimum_turn_radius_m=0.8, body_half_width_m=0.45)
        _, diag = self.confirm(hairpin(entrance=0.8))
        self.assertNotEqual(diag["turn_phase"], "SETUP")
        self.subject.clear()
        self.subject.settings.body_half_width_m = 0.12
        for _ in range(3):
            _, diag = self.step(hairpin(entrance=0.8), visibility="left_only")
        self.assertNotEqual(diag["turn_phase"], "SETUP")

    def test_opposite_branch_is_rejected_and_original_source_age_is_preserved(self):
        self.confirm()
        source = self.subject.cached_source
        accepted = self.subject.accepted_time
        out, diag = self.step(hairpin(-1))
        self.assertTrue(out["valid"])
        self.assertEqual(out["reason"], "bounded_turn_reference")
        self.assertEqual(diag["turn_direction"], 1)
        self.assertIn("rejected_branch", diag)
        self.assertEqual(diag["retained_source_s"], source)
        self.assertEqual(self.subject.accepted_time, accepted)
        self.assertGreater(diag["retained_source_age_s"], 0.)

    def test_missing_geometry_hold_expires_without_rearming(self):
        self.confirm()
        source = self.subject.cached_source
        for _ in range(9):
            out, diag = self.step(hairpin(), reason="invalid_road_insufficient_geometry")
            if not out["valid"]:
                break
            self.assertEqual(diag["retained_source_s"], source)
        self.assertFalse(out["valid"])
        self.assertEqual(out["reason"], "turn_reference_hold_exhausted")
        self.assertEqual(self.subject.phase, "FOLLOW")

    def test_hard_geometry_fault_cannot_be_hidden_by_turn_cache(self):
        self.confirm()
        out, _ = self.step(hairpin(), reason="invalid_road_invalid_boundary_pairing")
        self.assertFalse(out["valid"])
        self.assertIsNone(self.subject.cached)

    def test_motion_gap_and_clear_discard_turn_intent(self):
        self.confirm()
        out, _ = self.step(hairpin(), seconds=0.6)
        self.assertFalse(out["valid"])
        self.assertEqual(self.subject.phase, "FOLLOW")
        self.subject.direction, self.subject.phase = 1, "TURN"
        self.subject.clear()
        self.assertEqual(self.subject.direction, 0)
        self.assertIsNone(self.subject.cached)

    def test_uturn_exit_requires_yaw_progress_and_distinct_paired_exit_frames(self):
        self.confirm()
        # Supply an already aligned exit branch to isolate the exit decision.
        points = [(-0.3, 0.), (0.6, 0.), (1.2, 0.)]
        ref, _, _ = api["OrderedRoadChecks"]().reference(points)
        self.subject.cached = deepcopy(ref)
        self.subject.phase, self.subject.turn_type, self.subject.direction = "TURN", "UTURN", 1
        self.subject.yaw_progress = 1.5
        _, diag = self.step(points)
        self.assertEqual(diag["turn_phase"], "TURN")
        self.subject.yaw_progress = 2.8
        _, diag = self.step(points, visibility="right_only")
        self.assertEqual(diag["turn_phase"], "TURN")
        self.step(points)
        _, diag = self.step(points)
        self.assertEqual(diag["turn_phase"], "EXIT")
        _, diag = self.step(points)
        self.assertEqual(diag["turn_phase"], "FOLLOW")

    def test_disabled_planner_is_exact_pass_through_and_settings_are_bounded(self):
        self.subject.settings.enabled = False
        reference, diag = {"valid": False}, {"existing": 1}
        out, metadata = self.subject.update(reference, diag, {}, {}, self.motion, {})
        self.assertIs(out, reference)
        self.assertIs(metadata, diag)
        for change in ({"enabled": 1}, {"confirmation_frames": 1}, {"confirmation_frames": 3.0},
                       {"uturn_heading_rad": 0.4}, {"outward_fraction": 0.4},
                       {"hold_max_s": float("nan")}, {"minimum_turn_radius_m": -1.}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                ns["TurnManeuverSettings"](**change)

    def test_fov_prefix_loss_matches_common_support_instead_of_nearest_endpoints(self):
        previous = [(0.3, 0.), (1.2, 0.), (1.2, 0.6)]
        clipped = [(0.9, 0.), (1.2, 0.), (1.2, 0.6)]
        self.assertIsNone(self.subject._continuity_problem(clipped, previous))
        self.assertIsNone(self.subject._continuity_problem(previous, clipped))
        displaced = [(x, y+0.5) for x, y in clipped]
        self.assertEqual(self.subject._continuity_problem(displaced, previous), "turn_branch_position_jump")

    def test_unknown_prepare_keeps_new_observed_corner_reference(self):
        corner = [(-0.3, 0.), (0.6, 0.), (0.6, 0.6), (0.6, 1.2)]
        _, diag = self.confirm(corner)
        self.assertEqual(diag["turn_type"], "UNKNOWN")
        self.assertEqual(diag["turn_phase"], "PREPARE")
        changed = [(0.8, 0.), (1.2, 0.), (1.2, 0.7), (1.2, 1.2)]
        out, diag = self.step(changed, visibility="right_only")
        self.assertTrue(out["valid"], diag)
        self.assertEqual(out["reason"], "road_centerline_available")
        self.assertAlmostEqual(out["path"]["samples_xy"][0][0], 0.8)
        self.assertEqual(out["valid_for_s"], 0.2)
        self.assertNotIn("rejected_branch", diag)
        self.assertEqual(self.subject.accepted_time, self.now)

    def test_hold_tail_does_not_emit_a_reference_shorter_than_a_control_tick(self):
        self.confirm()
        for _ in range(7):
            out, _ = self.step(hairpin(), reason="invalid_road_insufficient_geometry")
            self.assertTrue(out["valid"])
        out, _ = self.step(hairpin(), reason="invalid_road_insufficient_geometry", seconds=0.075)
        self.assertFalse(out["valid"])
        self.assertEqual(out["reason"], "turn_reference_hold_exhausted")
        self.assertEqual(out["valid_for_s"], 0.)

    def test_actual_policy_entry_integrates_turn_planner_and_stop_clears_it(self):
        node_api = api["api"]
        node = api["CenterlineFollowChecks"]().learned_width_node(active=False)
        node.estimation_settings = api["OrderedRoadChecks"]().settings()
        node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
        node.turn_maneuver_settings = ns["TurnManeuverSettings"](enabled=True)
        node_api["feed"](node, speed=0.2)
        node_api["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.planning_diagnostics["turn_phase"], "FOLLOW")
        node.turn_maneuver_planner.phase = "TURN"
        node.turn_maneuver_planner.direction = 1
        node.fsm_transition_request_callback(node_api["SimpleNamespace"](data=2))
        self.assertEqual(node.turn_maneuver_planner.phase, "FOLLOW")
        self.assertEqual(node.turn_maneuver_planner.direction, 0)
        self.assertIsNone(node.turn_maneuver_planner.cached)
        self.assertEqual((node.action_publisher.messages[-1].drive, node.action_publisher.messages[-1].steer), (0., 0.))


if __name__ == "__main__":
    unittest.main(verbosity=2)
