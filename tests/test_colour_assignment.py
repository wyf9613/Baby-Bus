"""estimation.colour_assignment: auto (both track colour conventions), without ROS.

Ported on 2026-10-21 from the sim-integration branch
(origin/feature/mvp21-sim-integration-20261015@2c41240, tests/test_colour_assignment.py).
The estimator tests are the sim branch's, on this branch's EstimationPipeline. Its
priming_motion_history helper does not exist here; the startup wait is instead
checked on the two call sites in calculate_policy_actions (MPC and PID), with the
car 05 deployed layers (control.startup_grace_s 3.0).
"""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import runpy
import unittest
from unittest.mock import patch

mpc = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
ns = mpc["estimation_ns"]
EstimationPipeline, EstimationSettings = ns["EstimationPipeline"], ns["EstimationSettings"]


def settings(**changes):
    values = dict(colour_assignment="auto", road_geometry="ordered_xy",
                  road_min_coverage_m=0.3, single_side_from_learned_width=True)
    return EstimationSettings(**{**values, **changes})


def batch(left_colour=2, center=0.0, width=0.8):
    return dict(detections=[(x, center+offset, 0.0, colour, 0.95)
        for colour, offset in ((left_colour, width/2), (3-left_colour, -width/2))
        for x in (0.3, 0.6, 0.9, 1.2, 1.5, 1.8)])


class ColourAssignmentChecks(unittest.TestCase):
    def test_both_conventions_ignore_the_configured_fixed_preference(self):
        for geometry in ("ordered_xy", "y_of_x"):
            for physical_left in (1, 2):
                for configured_left in (1, 2):
                    with self.subTest(geometry=geometry, physical_left=physical_left,
                                      configured_left=configured_left):
                        pipeline = EstimationPipeline(settings(road_geometry=geometry,
                            left_colour=configured_left, right_colour=3-configured_left))
                        road = pipeline.road(batch(physical_left), 10_000_000_000, 0.0)
                        self.assertTrue(road["valid"], road["status"])
                        self.assertEqual(road["visibility"], "both")
                        self.assertEqual(road["colour_assignment"]["left_colour"], physical_left)
                        self.assertAlmostEqual(road["lane_width_m"], 0.8)
                        self.assertTrue(all(abs(p[1]) < 1e-6 for p in road["centerline_xy"]))

    def test_two_boundaries_on_same_body_side_use_geometry_not_y_sign(self):
        for left in (1, 2):
            pipeline = EstimationPipeline(settings())
            road = pipeline.road(batch(left, center=-1.0), 10_000_000_000, 0.0)
            self.assertTrue(road["valid"], road["status"])
            self.assertEqual(pipeline.left_colour, left)

    def test_single_side_cannot_lock_even_with_a_configured_width(self):
        pipeline = EstimationPipeline(settings(known_lane_width_m=0.8))
        one = batch()
        one["detections"] = [p for p in one["detections"] if p[3] == 2]
        road = pipeline.road(one, 10_000_000_000, 0.0)
        self.assertFalse(road["valid"])
        self.assertEqual(road["status"], "boundary_colour_assignment_pending")
        self.assertIsNone(road["colour_assignment"]["left_colour"])
        self.assertFalse(pipeline._colour_assignment_locked)
        road = pipeline.road(batch(1), 10_100_000_000, 0.0)
        self.assertTrue(road["valid"])
        self.assertEqual(pipeline.left_colour, 1)

    def test_assignment_survives_partial_frames_and_history_clears(self):
        for left in (1, 2):
            pipeline = EstimationPipeline(settings())
            road = pipeline._history_road(batch(left), 10_000_000_000, 0.0, 10.0)
            self.assertTrue(road["valid"])
            learned = pipeline._learned_lane_width_m
            one = batch(left)
            one["detections"] = [p for p in one["detections"] if p[3] == left]
            road = pipeline.road(one, 10_100_000_000, 0.0)
            self.assertTrue(road["single_side_inferred_from_learned_width"])
            self.assertEqual(road["visibility"], "left_only")
            self.assertAlmostEqual(road["lane_width_m"], learned)
            pipeline._clear_road_history()
            opposite = pipeline.road(batch(3-left), 10_200_000_000, 0.0)
            self.assertFalse(opposite["valid"])
            self.assertEqual(pipeline.left_colour, left)
            self.assertEqual(opposite["colour_assignment"]["status"], "locked")
            new_run = EstimationPipeline(settings())
            self.assertTrue(new_run.road(batch(3-left), 11_000_000_000, 0.0)["valid"])
            self.assertEqual(new_run.left_colour, 3-left)

    def test_width_gate_is_not_relaxed_to_force_assignment(self):
        road = EstimationPipeline(settings()).road(batch(width=0.1), 10_000_000_000, 0.0)
        self.assertFalse(road["valid"])
        self.assertEqual(road["colour_assignment"]["status"], "pending")

    def test_equally_plausible_two_sided_hypotheses_remain_unassigned(self):
        template = EstimationPipeline(settings(colour_assignment="fixed")).road(
            batch(), 10_000_000_000, 0.0)
        with patch.object(EstimationPipeline, "_road", return_value=deepcopy(template)):
            road = EstimationPipeline(settings()).road(batch(), 10_000_000_000, 0.0)
        self.assertEqual(road["status"], "boundary_colour_assignment_ambiguous")
        self.assertFalse(road["valid"])
        self.assertEqual(road["visibility"], "unassigned")
        self.assertFalse(road["centerline_xy"])
        self.assertIsNone(road["colour_assignment"]["left_colour"])

    def test_nearest_observed_corridor_wins_only_with_clear_distance_margin(self):
        template = EstimationPipeline(settings(colour_assignment="fixed")).road(
            batch(), 10_000_000_000, 0.0)
        def candidate(probe, *_):
            result = deepcopy(template)
            offset = 0.0 if probe.left_colour == 2 else 1.0
            result["centerline_xy"] = [(p[0], p[1]+offset) for p in result["centerline_xy"]]
            return result
        pipeline = EstimationPipeline(settings(left_colour=1, right_colour=2))
        with patch.object(EstimationPipeline, "_road", candidate):
            road = pipeline.road(batch(), 10_000_000_000, 0.0)
        self.assertTrue(road["valid"])
        self.assertEqual(pipeline.left_colour, 2)

    def test_missing_or_stale_data_does_not_acquire_a_mapping(self):
        pipeline = EstimationPipeline(settings())
        for value, stamp, age in ((None, None, None), (batch(), 10_000_000_000, 0.6)):
            road = pipeline.road(value, stamp, age)
            self.assertEqual(road["status"], "unavailable")
            self.assertFalse(pipeline._colour_assignment_locked)
        with self.assertRaises(ValueError):
            replace(settings(), colour_assignment="unknown")

    def test_fixed_default_reconstructs_exactly_as_before(self):
        self.assertEqual(EstimationSettings().colour_assignment, "fixed")
        for left in (1, 2):
            pipeline = EstimationPipeline(settings(colour_assignment="fixed", left_colour=left, right_colour=3-left))
            road = pipeline.road(batch(left), 10_000_000_000, 0.0)
            raw = EstimationPipeline(settings(colour_assignment="fixed", left_colour=left,
                                              right_colour=3-left))._road(batch(left), 10_000_000_000, 0.0)
            self.assertEqual(road.pop("colour_assignment"), dict(mode="fixed", status="fixed",
                left_colour=left, right_colour=3-left, hypotheses=[]))
            self.assertEqual(road, raw)


def feed(node, sides=("left", "right"), physical_left=2, speed=0.12):
    """mpc["feed"]'s straight 0.7 m road, with physical colours and optionally one side."""
    now, mono = node.test_clock.ros_ns, node.test_clock.monotonic
    node._store("wheel_speed", speed, None, mono, now)
    node._store("imu_angular_velocity", (0.0, 0.0, 0.0), now, mono, now)
    xs = [0.15+0.15*i for i in range(15)]
    detections = []
    if "left" in sides:
        detections += [(x, 0.35, 0.0, physical_left, 0.95) for x in xs]
    if "right" in sides:
        detections += [(x, -0.35, 0.0, 3-physical_left, 0.95) for x in xs]
    node._store("cone_detections", {"detections": detections, "acquisition_to_publish_latency_s": 0.05},
                now, mono, now)
    node.last_nonempty_cones = node.observations["cone_detections"]


def auto_node(mode):
    node = mpc["make_node"](mode=mode)
    # Configured blue-left, but the road fed below is yellow-left.
    node.estimation_settings = replace(node.estimation_settings, colour_assignment="auto",
                                       left_colour=2, right_colour=1)
    return node


def step(node, ticks, **feed_args):
    for _ in range(ticks):
        node.test_clock.advance(0.05)
        feed(node, **feed_args)
        node.run_policy_step()


class StartupWaitChecks(unittest.TestCase):
    """The road is invalid until the colours lock: wait neutral within startup_grace_s."""

    def test_mpc_waits_for_the_lock_then_tracks_the_physical_mapping(self):
        node = auto_node("mpc-exec-centerline")
        self.assertEqual(node.control_settings.startup_grace_s, 3.0)
        feed(node, sides=("left",), physical_left=1)
        mpc["start"](node)
        node.run_policy_step()
        step(node, 20, sides=("left",), physical_left=1)          # 1 s, one side only
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.estimation_output["road"]["status"], "boundary_colour_assignment_pending")
        self.assertTrue(all(m.drive == 0 and m.steer == 0 for m in node.action_publisher.messages))
        self.assertEqual(node.mpc_debug_publisher.messages, [])
        step(node, 5, physical_left=1)                             # both sides: locks, MPC runs
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual((node.estimator.left_colour, node.estimator.right_colour), (1, 2))
        debug = [json.loads(m.data) for m in node.mpc_debug_publisher.messages]
        self.assertTrue(debug)
        self.assertEqual(debug[-1]["colour_assignment"], {"status": "locked", "left_colour": 1, "right_colour": 2})

    def test_mpc_stops_once_the_grace_expires_without_a_lock(self):
        node = auto_node("mpc-exec-centerline")
        feed(node, sides=("left",))
        mpc["start"](node)
        node.run_policy_step()
        step(node, 55, sides=("left",))                            # 2.8 s
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        step(node, 10, sides=("left",))                            # past 3.0 s
        self.assertEqual(node.fsm_state, 2)

    def test_pid_waits_with_its_own_reason(self):
        node = mpc["control"]["make_node"]()
        node.estimation_settings = replace(node.estimation_settings, colour_assignment="auto")
        feed(node, sides=("left",), speed=0.0)
        mpc["start"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3, node.state_reason)
        self.assertEqual(node.control_diagnostics["reason"], "priming_colour_assignment")
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)

    def test_fixed_mpc_debug_has_no_colour_field(self):
        records = mpc["run"](mpc["make_node"](mode="mpc-exec-centerline"), 10)
        self.assertTrue(records)
        self.assertTrue(all("colour_assignment" not in r for r in records))


if __name__ == "__main__":
    unittest.main()
