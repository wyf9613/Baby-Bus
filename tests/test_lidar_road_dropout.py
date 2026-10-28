"""Checked-path continuity across short road glitches, using actual node/MPC code."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import runpy
import unittest


def checked_node(deliver=True):
    m = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
    original_feed = m["feed"]
    def feed(node, **kwargs):
        original_feed(node, **kwargs)
        stamp, mono = node.test_clock.ros_ns, node.test_clock.monotonic
        node._store("lidar_scan", dict(ranges=[], intensities=[]), stamp, mono, stamp)
        node._store("lidar_cartesian", dict(frame_id="base_link", points_xyz=[(1.1, .55, 0.)],
            scan_indices=[0]), stamp, mono, stamp)
        node._lidar_obstacle_stamp_ns = stamp
        node.lidar_obstacle_output = dict(measurement_timestamp_s=stamp/1e9,
                                         status="OBSTACLE", reason="fixture_obstacle")
        node._lidar_obstacle_publication_key = (stamp/1e9, "OBSTACLE", "fixture_obstacle")
    m["run"].__globals__["feed"] = feed
    node = m["make_node"](shadow_enabled=True, planner=True, mode="mpc-exec-lattice21",
                           worker=m["SyncWorker"](deliver=deliver))
    node.shadow_settings = replace(node.shadow_settings, obstacle_avoidance_enabled=True,
        obstacle_check_enabled=True, obstacle_margin_m=0., collision_coverage="generated", length_m=1.4)
    node.lidar_obstacle_settings = m["cns"]["LidarObstacleSettings"](
        enabled=True, stop_enabled=False, stop_distance_m=1.4, roi_half_width_m=.6)
    m["run"](node, 6)
    return m, node


def road_glitch(node, status="invalid_ordered_geometry"):
    original = node.estimator.update
    def update(*args, **kwargs):
        result = original(*args, **kwargs)
        result["road"].update(valid=False, geometry_valid=False, status=status,
            centerline_xy=[], left_boundary_xy=[], right_boundary_xy=[], forward_geometry={},
            boundary_source={"left":"absent", "right":"absent"})
        return result
    node.estimator.update = update
    return original


class RoadDropoutChecks(unittest.TestCase):
    def test_observed_glitches_keep_checked_path_and_mpc_without_zero_pulses(self):
        for status in ("invalid_ordered_geometry", "motion_transformed_geometry_invalid", "insufficient_geometry"):
            with self.subTest(status=status):
                m, node = checked_node()
                manager = node.planner_execution
                self.assertTrue(manager.active["obstacle_checked"])
                job_id, source_stamp = manager.last_job_id, manager.active["stamp_s"]
                controller = node.mpc_controller
                road_glitch(node, status)
                count = len(node.action_publisher.messages)
                result = m["run"](node, 3)[-1]
                self.assertEqual(result["reference_selection"], "planner")
                self.assertEqual(result["reference_selection_reason"], "planner_tracking_road_dropout")
                self.assertTrue(all(msg.drive > 0 for msg in node.action_publisher.messages[count:]))
                self.assertIs(node.mpc_controller, controller)
                self.assertEqual(manager.last_job_id, job_id)
                self.assertEqual(manager.active["stamp_s"], source_stamp)
                self.assertGreater(manager.diagnostics["progress_m"], 0.)
                self.assertEqual(manager.diagnostics["road_support"], "motion_transported_cached")

    def test_no_road_worker_results_preserve_identity_without_renewing_deadline(self):
        m, node = checked_node()
        node.shadow_settings = replace(node.shadow_settings, fusion_enabled=False)
        manager = node.planner_execution
        job_id, source_stamp = manager.last_job_id, manager.active["stamp_s"]
        original = road_glitch(node)
        for _ in range(11):
            node.shadow_last_submit_s = -float("inf")  # force a failure result at each test tick
            m["run"](node, 1)
        self.assertEqual(manager.last_job_id, job_id)
        self.assertEqual(manager.active["stamp_s"], source_stamp)
        self.assertIn("retained_no_road_job_id", manager.active)
        self.assertEqual(manager.diagnostics["reason"], "road_dropout_timeout")
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        self.assertGreaterEqual(manager.diagnostics["road_dropout_elapsed_s"], .5-1e-9)
        node.estimator.update = original
        result = m["run"](node, 1)[-1]
        self.assertEqual(result["reference_selection"], "planner")
        self.assertIsNone(manager.road_dropout_started_s)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0.)

    def test_obstacle_blocked_result_still_stops_during_a_road_glitch(self):
        m, node = checked_node()
        road_glitch(node)
        m["run"](node, 1)
        node.shadow_worker.results.append(dict(status="ALL_COLLIDE", stop_required=True,
            generation=node.shadow_generation, job_id=999))
        node._shadow_poll()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("no passable path", node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        self.assertIsNone(node.planner_execution.active)
        self.assertIsNone(node.planner_execution.last_good_road)

    def test_epoch_change_and_real_boundary_collision_do_not_use_cached_support(self):
        m, node = checked_node()
        estimates = deepcopy(node.estimation_output)
        now = estimates["state"]["timestamp_s"]
        estimates["road"].update(right_boundary_xy=[(-.1, -.1), (.1, .1)],
            boundary_source={"left":"absent", "right":"observed"}, time_aligned=True,
            frame_id="base_link", timestamp_s=now)
        self.assertIsNone(node._select_planner_reference(estimates))
        self.assertEqual(node.planner_execution.diagnostics["reason"], "known_boundary_collision")
        self.assertIsNone(node.planner_execution.last_good_road)
        m["run"](node, 1)
        road_glitch(node)
        node.shadow_odometry_epoch += 1
        m["run"](node, 1)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        self.assertEqual(node.planner_execution.diagnostics["mode"], "waiting")

    def test_no_checked_startup_path_never_gains_dropout_permission(self):
        m, node = checked_node(deliver=False)
        self.assertIsNone(node.planner_execution.active)
        self.assertIsNone(node.planner_execution.last_good_road)
        self.assertTrue(all(msg.drive == 0 for msg in node.action_publisher.messages))
        road_glitch(node)
        m["run"](node, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)


if __name__ == "__main__":
    unittest.main()
