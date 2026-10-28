"""Real sampler/worker/MPC handoff checks for soft LiDAR ranking; no vehicle."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import runpy
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
shadow = runpy.run_path(str(Path(__file__).with_name("test_shadow_lattice.py")))


def record(points=((.9, 0., 0.),)):
    job = shadow["job"](speed=.5)
    job["settings"].update(sample_count=48, length_m=1.4, obstacle_avoidance_enabled=True)
    job["obstacles"] = dict(available=True, time_aligned=True, timestamp_s=100.,
        measurement_timestamp_s=99.9, source_age_s=.1,
        roi_xy_m=[[.297, 1.697], [-.6, .6]], points_xyz_m=list(points))
    return job


class SoftAvoidanceChecks(unittest.TestCase):
    def test_central_and_mirrored_obstacles_change_selection_without_new_paths(self):
        baseline = shadow["plan"](record(), obstacle_avoidance_enabled=False)
        for y in (0., .1, -.1):
            with self.subTest(y=y):
                result = shadow["plan"](record(((.9, y, 0.),)))
                selected = result["selected"]
                self.assertEqual(result["status"], "VALID")
                self.assertNotEqual(selected["id"], baseline["selected"]["id"])
                self.assertGreater(selected["obstacle_clearance_m"], .02)
                near = min(selected["path"], key=lambda p: abs(p[0]-.9))
                self.assertGreater(abs(near[1]), .2)
                if y:
                    self.assertLess(y*near[1], 0.)
                # Candidate generation, road collisions and all native costs are intact.
                self.assertEqual(result["feasible_count"], baseline["feasible_count"])
                for actual, original in zip(result["candidates"], baseline["candidates"]):
                    for key in ("id", "family", "offsets_m", "path", "collision", "scoring_samples"):
                        self.assertEqual(actual[key], original[key])
                    self.assertEqual({k:v for k,v in actual["costs"].items() if k != "obstacle"},
                                     original["costs"])

    def test_obstacle_beyond_native_scoring_prefix_still_changes_path(self):
        job = record(((1.25, 0., 0.),))
        result = shadow["plan"](job)
        self.assertLess(result["score_length_m"]+.25, 1.25)
        self.assertNotEqual(result["selected"]["id"], 0)
        self.assertGreater(result["candidates"][0]["costs"]["obstacle"], 0.)

    def test_empty_outside_roi_missing_and_unaligned_lidar_preserve_native_choice(self):
        original = shadow["plan"](record(), obstacle_avoidance_enabled=False)
        jobs = [record(()), record(((1.8, 0., 0.), (.9, .61, 0.)))]
        for field, value in (("available", False), ("time_aligned", False), ("timestamp_s", 99.)):
            job = record()
            job["obstacles"][field] = value
            jobs.append(job)
        job = record()
        job.pop("obstacles")
        jobs.append(job)
        for job in jobs:
            result = shadow["plan"](job)
            self.assertEqual(result["status"], original["status"])
            self.assertEqual(result["selected"]["path"], original["selected"]["path"])
            self.assertEqual(result["selected"]["cost"], original["selected"]["cost"])
        self.assertEqual(shadow["plan"](record(), weight_obstacle=0.)["selected"]["path"],
                         original["selected"]["path"])

    def test_blocking_wall_is_a_finite_cost_not_a_new_rejection(self):
        job = record(tuple((.75, -.6+i*.02, 0.) for i in range(61)))
        result = shadow["plan"](job)
        original = shadow["plan"](job, obstacle_avoidance_enabled=False)
        self.assertEqual(result["status"], original["status"])
        self.assertEqual(result["feasible_count"], original["feasible_count"])
        self.assertTrue(all(math.isfinite(c["cost"]) and c["costs"]["obstacle"] > 0.
                            for c in result["candidates"]))
        self.assertFalse(result["obstacle_avoidance"]["hard_rejection"])

    def test_existing_road_collision_rejection_is_preserved(self):
        job = record()
        job["road"].update(left_boundary_xy=[(0., .18), (2., .18)],
                           right_boundary_xy=[(0., -.18), (2., -.18)], lane_width_m=.36)
        result = shadow["plan"](job)
        original = shadow["plan"](job, obstacle_avoidance_enabled=False)
        self.assertEqual([c["collision"] for c in result["candidates"]],
                         [c["collision"] for c in original["candidates"]])
        self.assertGreater(sum(c["collision"] is not None for c in result["candidates"]), 0)
        self.assertIsNone(result["selected"]["collision"])

    def test_duplicate_returns_do_not_amplify_proximity_weight(self):
        a = shadow["plan"](record())
        b = shadow["plan"](record(((.9, 0., 0.),)*40))
        self.assertEqual(a["selected"]["id"], b["selected"]["id"])
        self.assertEqual([c["cost"] for c in a["candidates"]],
                         [c["cost"] for c in b["candidates"]])

    def test_real_worker_roundtrip_preserves_scoring_and_input_epoch(self):
        job = record()
        expected = shadow["plan"](job)
        process = subprocess.run([sys.executable, str(shadow["source"]), "--shadow-worker"],
            input=json.dumps(job)+"\n", capture_output=True, text=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        self.assertEqual(result["selected"]["id"], expected["selected"]["id"])
        self.assertEqual(result["obstacle_avoidance"], expected["obstacle_avoidance"])
        self.assertEqual(result["planning_input"]["obstacles"], json.loads(json.dumps(job["obstacles"])))

    def test_deployment_changes_ranking_and_passability_without_changing_mpc(self):
        import yaml
        current = yaml.safe_load((ROOT/"checkpoints/fusion-mpc-lidar-car14-20261028/ai4r_policy.yaml").read_text())
        seed = yaml.safe_load((ROOT/"checkpoints/car01-uturn-noprior-20261028/v050-5m-noprior/ai4r_policy.yaml").read_text())
        current, seed = (p["/**/ai4r_policy"]["ros__parameters"] for p in (current, seed))
        section = current["shadow"]
        self.assertTrue(section.pop("obstacle_avoidance_enabled"))
        self.assertTrue(section.pop("obstacle_check_enabled"))
        self.assertEqual(section.pop("weight_obstacle"), 30.)
        self.assertEqual(section.pop("obstacle_margin_m"), 0.)
        self.assertEqual(section["body_width_m"], .3)
        self.assertEqual(section.pop("collision_coverage"), "generated")
        seed["shadow"].pop("collision_coverage")
        self.assertEqual(section, seed["shadow"])
        self.assertTrue(current["mpc"].pop("geometry_glitch_tolerable"))
        seed["mpc"].pop("geometry_glitch_tolerable", None)
        self.assertEqual(current["mpc"], seed["mpc"])
        self.assertEqual(current["planning"]["cruise_speed_mps"], .5)
        self.assertFalse(current["lidar_obstacle"]["stop_enabled"])
        self.assertEqual(current["lidar_obstacle"]["roi_half_width_m"], .6)
        self.assertEqual(current["control"]["max_distance_m"], 0.)
        self.assertEqual(current["control"]["max_run_time_s"], 0.)


class MPCHandoffChecks(unittest.TestCase):
    def test_expanded_roi_forwards_both_new_lateral_strips(self):
        m = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
        node = m["make_node"](shadow_enabled=True, planner=True)
        m["run"](node, 2)
        node.shadow_settings = replace(node.shadow_settings, obstacle_avoidance_enabled=True)
        node.lidar_obstacle_settings = m["cns"]["LidarObstacleSettings"](
            enabled=True, stop_enabled=False, stop_distance_m=1.4, roi_half_width_m=.6)
        stamp = node.estimation_output["state"]["timestamp_s"]
        node.estimation_output["obstacles"] = dict(available=True, time_aligned=True,
            frame_id="base_link", timestamp_s=stamp,
            points_xyz_m=[(1.1, .5, 0.), (1.1, -.5, 0.), (1.1, .61, 0.), (1.1, -.61, 0.)])
        node.shadow_last_submit_s = -math.inf
        node._shadow_offer()
        self.assertEqual(node.shadow_worker.jobs[-1]["obstacles"]["points_xyz_m"],
                         [[1.1, .5, 0.], [1.1, -.5, 0.]])

    def test_real_node_passes_aligned_roi_and_mpc_tracks_the_selected_path(self):
        m = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
        node = m["make_node"](shadow_enabled=True, planner=True, mode="mpc-exec-lattice21")
        before = m["run"](node, 6)[-1]
        node.shadow_settings = replace(node.shadow_settings,
            obstacle_avoidance_enabled=True, length_m=1.4, sample_count=48)
        node.lidar_obstacle_settings = m["cns"]["LidarObstacleSettings"](
            enabled=True, stop_enabled=False, stop_distance_m=1.4, roi_half_width_m=.6)
        # This fixture injects estimator output; raw-scan telemetry is tested separately.
        node._lidar_obstacle_publication_key = (None, "UNKNOWN", "missing_stale_or_unmatched_lidar")
        stamp = node.estimation_output["state"]["timestamp_s"]
        source = dict(available=True, time_aligned=True, frame_id="base_link", timestamp_s=stamp,
            measurement_timestamp_s=stamp-.1, source_age_s=.1,
            points_xyz_m=[(1.1, 0., 0.), (2., 0., 0.), (1.1, .7, 0.)])
        node.estimation_output["obstacles"] = deepcopy(source)
        node.shadow_last_submit_s = -math.inf
        node._shadow_offer()
        submitted = node.shadow_worker.jobs[-1]
        self.assertEqual(submitted["obstacles"]["points_xyz_m"], [[1.1, 0., 0.]])
        self.assertEqual(submitted["obstacles"]["timestamp_s"], stamp)
        self.assertEqual(node.estimation_output["obstacles"], source)
        result = node.shadow_worker.results[-1]
        self.assertNotEqual(result["selected"]["id"], 0)
        node._shadow_poll()
        after = m["run"](node, 2)[-1]
        self.assertEqual(after["reference_selection"], "planner")
        self.assertEqual(after["path_id"], submitted["job_id"])
        self.assertEqual(after["branch"], "solved")
        self.assertNotEqual(after["candidate_steer_action"], before["candidate_steer_action"])
        self.assertGreater(abs(node.action_publisher.messages[-1].steer), .01)
        self.assertEqual(node.fsm_state, 3)
        self.assertGreater(max(abs(p[1]) for p in node._trajectory_reference["path"]["samples_xy"]), .1)


def checked_record(points):
    job = record(points)
    job["settings"].update(obstacle_check_enabled=True, obstacle_margin_m=0.,
                           collision_coverage="generated")
    job["road"].update(left_boundary_xy=[(0., .5), (2., .5)],
                       right_boundary_xy=[(0., -.5), (2., -.5)])
    return job


class PassabilityChecks(unittest.TestCase):
    def test_narrow_gap_example_is_blocked_instead_of_a_valid_prefix(self):
        job = checked_record(tuple((.9, -.2+.02*i, 0.) for i in range(21)))
        result = shadow["plan"](job)
        self.assertEqual(result["status"], "ALL_COLLIDE")
        self.assertEqual(result["feasible_count"], 0)
        self.assertTrue(result["stop_required"])
        self.assertEqual(result["reason"], "no_passable_obstacle_path")
        self.assertEqual(result["check_length_m"], result["generated_length_m"])
        self.assertEqual(result["obstacle_avoidance"]["minimum_straight_gap_m"], .3)
        self.assertTrue(all(c["collision"] for c in result["candidates"]))

    def test_a_wide_road_still_produces_a_checked_bypass(self):
        job = checked_record(((.9, 0., 0.),))
        job["road"].update(left_boundary_xy=[(0., .7), (2., .7)],
                           right_boundary_xy=[(0., -.7), (2., -.7)], lane_width_m=1.4)
        result = shadow["plan"](job)
        self.assertEqual(result["status"], "VALID")
        self.assertFalse(result["stop_required"])
        self.assertNotEqual(result["selected"]["id"], 0)
        self.assertIsNone(result["selected"]["collision"])
        self.assertGreater(result["selected"]["obstacle_clearance_m"], 0.)
        # Hard feasibility remains active even if soft obstacle weight is zero.
        self.assertEqual(shadow["plan"](job, weight_obstacle=0.)["candidates"][0]["collision"]["side"], "lidar")

    def test_zero_extra_width_and_rotated_body_occupancy(self):
        cfg = shadow["Settings"](obstacle_avoidance_enabled=True, obstacle_check_enabled=True,
                                 obstacle_margin_m=0.)
        context = shadow["ns"]["_shadow_obstacle_context"]
        collide = shadow["ns"]["_shadow_obstacle_collision"]
        for half_gap, blocked in ((.149, True), (.151, False)):
            job = checked_record(((.8, half_gap, 0.), (.8, -half_gap, 0.)))
            result = collide([(0., 0., 0.), (1.4, 0., 0.)], context(job, cfg), cfg)
            self.assertEqual(result is not None, blocked)
        job = checked_record(((.9, .18, 0.), (.7, -.18, 0.)))
        self.assertIsNone(collide([(.8, 0., 0.)], context(job, cfg), cfg))
        self.assertIsNotNone(collide([(.8, 0., math.pi/6)], context(job, cfg), cfg))

    def test_hard_blocked_result_survives_real_worker_transport(self):
        job = checked_record(tuple((.9, -.2+.02*i, 0.) for i in range(21)))
        process = subprocess.run([sys.executable, str(shadow["source"]), "--shadow-worker"],
            input=json.dumps(job)+"\n", capture_output=True, text=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        self.assertTrue(result["stop_required"])
        self.assertEqual(result["feasible_count"], 0)

    def test_known_obstacles_wait_for_a_checked_path_then_block_latched(self):
        m = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
        node = m["make_node"](shadow_enabled=True, planner=True, mode="mpc-exec-lattice21")
        m["run"](node, 6)  # previously accepted road-only path
        node.shadow_settings = replace(node.shadow_settings, obstacle_avoidance_enabled=True,
            obstacle_check_enabled=True, obstacle_margin_m=0., collision_coverage="generated", length_m=1.4)
        node.lidar_obstacle_settings = m["cns"]["LidarObstacleSettings"](
            enabled=True, stop_enabled=False, stop_distance_m=1.4, roi_half_width_m=.6)
        node._lidar_obstacle_publication_key = (None, "UNKNOWN", "missing_stale_or_unmatched_lidar")
        node.test_clock.advance(.05)
        m["feed"](node)
        stamp, mono = node.test_clock.ros_ns, node.test_clock.monotonic
        points = [(.9, -.2+.02*i, 0.) for i in range(21)]
        node._store("lidar_scan", dict(ranges=[], intensities=[]), stamp, mono, stamp)
        # Supply the detector cache as well as matching raw/cartesian sensor epochs.
        node._lidar_obstacle_stamp_ns = stamp
        node.lidar_obstacle_output = dict(measurement_timestamp_s=stamp/1e9,
                                         status="OBSTACLE", reason="fixture_obstacle")
        node._lidar_obstacle_publication_key = (stamp/1e9, "OBSTACLE", "fixture_obstacle")
        node._store("lidar_cartesian", dict(frame_id="base_link", points_xyz=points,
            scan_indices=list(range(len(points)))), stamp, mono, stamp)
        node.shadow_last_submit_s = -math.inf
        count = len(node.mpc_debug_publisher.messages)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 3)
        self.assertEqual(node.planner_execution.diagnostics["mode"], "waiting")
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        self.assertEqual(len(node.mpc_debug_publisher.messages), count)  # baseline was not solved
        self.assertTrue(node.shadow_worker.results[-1]["stop_required"])
        for _ in range(3):  # a delayed worker must not release the old centerline
            node.test_clock.advance(.05)
            m["feed"](node)
            node.run_policy_step()
            self.assertEqual(node.fsm_state, 3)
            self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        node._shadow_poll()
        self.assertEqual(node.fsm_state, 2)
        self.assertIn("no passable path", node.state_reason)
        self.assertIsNone(node.planner_execution.active)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        m["feed"](node)
        node.run_policy_step()
        self.assertEqual(node.fsm_state, 2)  # no automatic restart

    def test_checked_bypass_resumes_mpc_after_waiting_on_a_wide_road(self):
        m = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
        original_feed = m["feed"]
        def wide_feed(node, **kwargs):
            original_feed(node, **kwargs)
            batch = node.observations["cone_detections"].value
            batch["detections"] = [(x, 2*y, z, colour, conf)
                                   for x,y,z,colour,conf in batch["detections"]]
        m["run"].__globals__["feed"] = wide_feed
        node = m["make_node"](shadow_enabled=True, planner=True, mode="mpc-exec-lattice21")
        m["run"](node, 6)
        node.shadow_settings = replace(node.shadow_settings, obstacle_avoidance_enabled=True,
            obstacle_check_enabled=True, obstacle_margin_m=0., collision_coverage="generated", length_m=1.4)
        node.lidar_obstacle_settings = m["cns"]["LidarObstacleSettings"](
            enabled=True, stop_enabled=False, stop_distance_m=1.4, roi_half_width_m=.6)
        node.test_clock.advance(.05)
        wide_feed(node)
        stamp, mono = node.test_clock.ros_ns, node.test_clock.monotonic
        node._store("lidar_scan", dict(ranges=[], intensities=[]), stamp, mono, stamp)
        node._store("lidar_cartesian", dict(frame_id="base_link", points_xyz=[(1.1, 0., 0.)],
            scan_indices=[0]), stamp, mono, stamp)
        node._lidar_obstacle_stamp_ns = stamp
        node.lidar_obstacle_output = dict(measurement_timestamp_s=stamp/1e9,
                                         status="OBSTACLE", reason="fixture_obstacle")
        node._lidar_obstacle_publication_key = (stamp/1e9, "OBSTACLE", "fixture_obstacle")
        node.shadow_last_submit_s = -math.inf
        node.run_policy_step()
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.)
        self.assertEqual(node.shadow_worker.results[-1]["status"], "VALID")
        node._shadow_poll()
        self.assertTrue(node.planner_execution.active["obstacle_checked"])
        result = m["run"](node, 2)[-1]
        self.assertEqual(result["reference_selection"], "planner")
        self.assertEqual(result["branch"], "solved")
        self.assertGreater(node.action_publisher.messages[-1].drive, 0.)
        self.assertGreater(abs(node.action_publisher.messages[-1].steer), .01)

    def test_old_generation_blocked_result_cannot_stop_a_new_run(self):
        m = runpy.run_path(str(Path(__file__).with_name("test_mpc_planner_control.py")))
        node = m["make_node"](shadow_enabled=True, planner=True)
        m["run"](node, 2)
        node.shadow_settings = replace(node.shadow_settings, obstacle_check_enabled=True)
        node.shadow_worker.results.append(dict(status="ALL_COLLIDE", stop_required=True,
                                               generation=node.shadow_generation-1))
        node._shadow_poll()
        self.assertEqual(node.fsm_state, 3)


if __name__ == "__main__":
    unittest.main()
