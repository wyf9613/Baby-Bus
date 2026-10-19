"""Portable checks of actual detector + policy lifecycle source; no ROS mocks/devices."""
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest


api = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
ns = api["ns"]
tree = ast.parse(api["api"]["SOURCE"].read_text(encoding="utf-8"))
definitions = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))
               and item.name in {"LidarObstacleSettings", "_lidar_ray_rectangle", "detect_lidar_obstacles",
                                 "lidar_obstacle_display_geometry"}]
subject = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "PolicyNode")
helpers = [item for item in subject.body if isinstance(item, ast.FunctionDef)
           and item.name in {"_lidar_stop_problem", "_publish_lidar_obstacle_display"}]
ns["json"] = json
exec(compile(ast.Module(body=definitions+helpers, type_ignores=[]), str(api["api"]["SOURCE"]), "exec"), ns)
for helper in helpers:
    setattr(ns["SourceNode"], helper.name, ns[helper.name])
Settings, detect = ns["LidarObstacleSettings"], ns["detect_lidar_obstacles"]


# Message-construction ports only: exercise the actual publisher method without
# claiming ROS message validation, DDS transport or a rendered Foxglove session.
def display_pose():
    return SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                           orientation=SimpleNamespace(w=0.0))


def display_message():
    return SimpleNamespace(header=SimpleNamespace(frame_id="", stamp=None), poses=[])


ns.update(Pose=display_pose, PoseArray=display_message, Path=display_message,
          PoseStamped=lambda header: SimpleNamespace(header=header, pose=display_pose()),
          Time=lambda nanoseconds: SimpleNamespace(to_msg=lambda: SimpleNamespace(
              sec=nanoseconds//1_000_000_000, nanosec=nanoseconds % 1_000_000_000)))


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
    raw = {"ranges": ranges, "angle_min": 0.0, "angle_increment": increment,
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
        self.assertEqual(result["roi_xy_m"], ((0.297, 1.297), (-0.175, 0.175)))
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
        cart["points_xyz"], cart["scan_indices"] = [(0.297, 0.175, 0.12), (1.297, -0.175, 0.12),
                                                  (1.297001, 0.0, 0.12), (0.8, 0.175001, 0.12)], [0, 1, 2, 3]
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
                       {"min_cluster_points": 0}, {"min_known_fraction": 0}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Settings(**values)


class DisplayGeometryChecks(unittest.TestCase):
    def test_confirmed_and_sparse_hits_are_separate_and_keep_measured_coordinates(self):
        result = detect(*scan_fixture({0: 0.8, 1: 0.8, 10: 0.8}), Settings())
        before = deepcopy(result)
        geometry = ns["lidar_obstacle_display_geometry"](result, Settings())
        self.assertEqual(len(geometry["hits"]), 2)
        self.assertEqual(len(geometry["candidates"]), 1)
        self.assertEqual(geometry["hits"], result["clusters"][0]["points_xyz_m"])
        self.assertEqual(result, before)  # Display does not alter detection or stopping.

    def test_roi_is_closed_and_tracks_configured_dimensions(self):
        cfg = Settings(body_width_m=0.3, side_margin_m=0.1, stop_distance_m=1.5)
        geometry = ns["lidar_obstacle_display_geometry"]({}, cfg)
        self.assertEqual(geometry["roi"], [(0.297, -0.25, 0.0), (1.797, -0.25, 0.0),
                                           (1.797, 0.25, 0.0), (0.297, 0.25, 0.0),
                                           (0.297, -0.25, 0.0)])

    def test_missing_stale_and_clear_results_clear_old_hits_but_keep_roi(self):
        result = detect(*scan_fixture({0: 0.8, 1: 0.8}), Settings())
        result.update(available=False, status="UNKNOWN")
        clear = detect(*scan_fixture(), Settings())
        for frame in (result, clear, {}):
            geometry = ns["lidar_obstacle_display_geometry"](frame, Settings())
            self.assertEqual(geometry["hits"], [])
            self.assertEqual(geometry["candidates"], [])
            self.assertEqual(len(geometry["roi"]), 5)

    def test_layout_uses_matching_xy_fields_and_scatter_for_both_point_series(self):
        layout_path = Path(__file__).resolve().parents[1] / "tools/foxglove_road_visualization/layout.json"
        # Source-only asset: installed ROS gate need not install offline layout.
        if not layout_path.exists():
            return
        layout = json.loads(layout_path.read_text(encoding="utf-8"))
        plot = next(p for p in layout["configById"].values()
                    if any("policy_trajectory" in s.get("value", "") for s in p.get("paths", [])))
        series = {s["value"].split(".poses")[0]: s for s in plot["paths"] if ".poses" in s["value"]}
        for topic in ("/car/lidar_obstacle_points", "/car/lidar_obstacle_candidates"):
            self.assertFalse(series[topic]["showLine"])
            self.assertEqual(series[topic]["value"], topic+".poses[:].position.y")
            self.assertEqual(series[topic]["xValuePath"], topic+".poses[:].position.x")
        self.assertTrue(series["/car/lidar_stop_roi"]["showLine"])
        self.assertTrue(series["/car/policy_trajectory"]["enabled"])


def policy_fixture():
    node = api["make_node"](simulated=False, mvp=True)
    node.control_settings.enabled = False  # Isolate guard, not road/planning validity.
    node.required_sensors += ["lidar_scan", "lidar_cartesian"]
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

    def test_cone_recovery_and_terminal_straight_cannot_bypass_guard(self):
        for terminal in (False, True):
            node = policy_fixture()
            node.control_settings.enabled = True
            node.control_settings.robustness_enabled = True
            feed_lidar(node)
            api["start"](node)
            if terminal:
                node._end_straight_start_distance_m = 0.0
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


class DisplayPublicationChecks(unittest.TestCase):
    def node(self):
        node = policy_fixture()
        node.trajectory_enabled = True
        for name in ("lidar_obstacle_points", "lidar_obstacle_candidates", "lidar_stop_roi"):
            setattr(node, name+"_publisher", api["Recorder"]())
        return node

    def test_actual_publisher_preserves_stamp_then_clears_stale_hits(self):
        node = self.node()
        feed_lidar(node, {0: 0.8, 1: 0.8, 10: 0.8})
        node.test_clock.advance(0.1)
        node.supervision_callback()
        message = node.lidar_obstacle_points_publisher.messages[-1]
        self.assertEqual(len(message.poses), 2)
        self.assertEqual(message.header.frame_id, "base_link")
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (10, 0))
        self.assertEqual(message.poses[0].position.x, 0.8)
        self.assertEqual(len(node.lidar_obstacle_candidates_publisher.messages[-1].poses), 1)
        roi = node.lidar_stop_roi_publisher.messages[-1]
        self.assertEqual(roi.header.stamp.nanosec, 100_000_000)  # ROI is body geometry now.
        self.assertEqual(len(roi.poses), 5)
        node.supervision_callback()
        self.assertEqual(len(node.lidar_obstacle_points_publisher.messages), 1)
        node.test_clock.advance(0.4)
        node.supervision_callback()
        self.assertEqual(node.lidar_obstacle_points_publisher.messages[-1].poses, [])
        self.assertEqual(node.lidar_obstacle_candidates_publisher.messages[-1].poses, [])

    def test_visualization_disabled_does_not_disable_detection_or_stop(self):
        node = self.node()
        node.trajectory_enabled = False
        feed_lidar(node)
        api["start"](node)
        node.test_clock.advance(0.05)
        feed_lidar(node, {0: 0.8, 1: 0.8})
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.lidar_obstacle_points_publisher.messages, [])
        self.assertEqual(node.lidar_stop_roi_publisher.messages, [])

    def test_display_retains_exact_epoch_nanoseconds(self):
        node = self.node()
        node.test_clock.ros_ns = 1_800_000_000_123_456_789
        feed_lidar(node)
        node.supervision_callback()
        stamp = node.lidar_obstacle_points_publisher.messages[-1].header.stamp
        self.assertEqual(stamp.sec*1_000_000_000+stamp.nanosec, node.test_clock.ros_ns)

    def test_display_failure_does_not_prevent_zero_publication(self):
        node = self.node()
        feed_lidar(node)
        api["start"](node)

        class FailingDisplay:
            def publish(self, _):
                raise RuntimeError("display failed")

        node.lidar_obstacle_points_publisher = FailingDisplay()
        node.test_clock.advance(0.05)
        feed_lidar(node, {0: 0.8, 1: 0.8})
        node.supervision_callback()
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
