"""Real ROS types, controlled time for policy contracts, and an isolated graph.

Run through tools/verify_fast.sh on Ubuntu/ROS Jazzy. No sensor driver, serial
device, vehicle process, or physical actuator is started by these tests.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, UInt16
from dream_interfaces.msg import (ConeDetection, ConeDetections, DriveAndSteer,
                                  FiducialDetection, FiducialDetections)
import yaml


SHARE = Path(get_package_share_directory("ai4r_policy"))
SCRIPT = Path(get_package_prefix("ai4r_policy")) / "lib/ai4r_policy/policy_node.py"
spec = importlib.util.spec_from_file_location("ai4r_policy_test_subject", SCRIPT)
policy = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = policy
spec.loader.exec_module(policy)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class ControlledClock:
    def __init__(self):
        self.monotonic = 10.0
        self.ros_ns = 100_000_000_000

    def advance(self, seconds):
        self.monotonic += seconds
        self.ros_ns += round(seconds * 1e9)

    def now(self):
        return Time(nanoseconds=self.ros_ns)


@pytest.fixture
def make_node(monkeypatch):
    context = Context()
    rclpy.init(context=context)
    nodes = []

    def create(mode="timer", required=(), **extra):
        settings = {"policy_update_mode": mode, "required_sensors": list(required), **extra}
        overrides = [Parameter(name, value=value) if value != [] else
                     Parameter(name, Parameter.Type.STRING_ARRAY, [])
                     for name, value in settings.items()]
        node = policy.PolicyNode(context=context, namespace="contract_test",
                                 parameter_overrides=overrides)
        nodes.append(node)
        node.supervision_timer.cancel()
        if node.policy_timer:
            node.policy_timer.cancel()
        clock = ControlledClock()
        node._monotonic = lambda: clock.monotonic
        monkeypatch.setattr(node, "get_clock", lambda: clock)
        node._last_status_at = -math.inf
        for name in ("action", "pan", "state", "state_text", "heading", "debug1", "debug2"):
            setattr(node, name + "_publisher", Recorder())
        node.test_clock = clock
        return node

    yield create
    for node in reversed(nodes):
        node.destroy_node()
    context.shutdown()


def stamp(node, msg, offset=0.0, frame="base_link"):
    msg.header.stamp = Time(nanoseconds=node.test_clock.ros_ns + round(offset * 1e9)).to_msg()
    msg.header.frame_id = frame
    return msg


def cones(node, empty=False, **kwargs):
    msg = stamp(node, ConeDetections(), **kwargs)
    if not empty:
        cone = ConeDetection()
        cone.position.x, cone.position.y = 1.0, 0.5
        cone.color = ConeDetection.COLOR_YELLOW
        cone.classification_confidence = 0.8
        msg.detections = [cone]
    return msg


def fiducials(node, empty=False, pose_valid=True, frame="camera_optical", **kwargs):
    msg = stamp(node, FiducialDetections(), frame=frame, **kwargs)
    msg.dictionary_name = "DICT_4X4_50"
    if not empty:
        detection = FiducialDetection()
        detection.id = 7
        detection.marker_size_m = 0.25
        for corner, values in zip(detection.corners,
                                  ((10.0, 20.0), (30.0, 20.0),
                                   (30.0, 40.0), (10.0, 40.0))):
            corner.u, corner.v = values
        detection.pose_valid = pose_valid
        detection.pose.orientation.w = 1.0
        detection.reprojection_error_px = 0.4 if pose_valid else math.nan
        if pose_valid:
            detection.pose.position.x = 1.0
            detection.pose.position.z = 2.0
        msg.detections = [detection]
    return msg


def scan(node, **kwargs):
    kwargs.setdefault("frame", "laser")
    msg = stamp(node, LaserScan(), **kwargs)
    msg.angle_min, msg.angle_max, msg.angle_increment = 0.0, 0.1, 0.1
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [1.0, float("inf")]
    return msg


def imu(node, yaw=0.0, orientation=True, **kwargs):
    msg = stamp(node, Imu(), **kwargs)
    msg.orientation.z = math.sin(yaw / 2)
    msg.orientation.w = math.cos(yaw / 2)
    msg.orientation_covariance[0] = 0.0 if orientation else -1.0
    msg.angular_velocity_covariance[0] = -1.0
    msg.linear_acceleration_covariance[0] = -1.0
    return msg


def request(node, state):
    node.fsm_transition_request_callback(UInt16(data=state))


def driving_policy(*args):
    return 0.4, -0.2, None, None, None


def test_integrated_control_real_ros_callbacks_stop_and_resume(make_node):
    node = make_node(required=("cone_detections", "wheel_speed", "imu_angular_velocity"),
        **{"control.enabled": True, "control.vehicle_params_source": "course_simulation",
           "planning.vehicle_limits_source": "course_simulation", "estimation.road_hold_s": 0.0})

    def feed_road(empty=False):
        node.wheel_speed_callback(Float32(data=0.0))
        gyro = imu(node, orientation=False)
        gyro.angular_velocity_covariance[0] = 0.0
        node.imu_callback(gyro)
        batch = stamp(node, ConeDetections())
        batch.acquisition_to_publish_latency_s = 0.0
        if not empty:
            for colour, y in ((ConeDetection.COLOR_BLUE, 0.6), (ConeDetection.COLOR_YELLOW, -0.4)):
                for i in range(10):
                    cone = ConeDetection()
                    cone.position.x, cone.position.y = -0.1+0.3*i, y
                    cone.color, cone.classification_confidence = colour, 0.95
                    batch.detections.append(cone)
        node.cone_detection_callback(batch)

    feed_road()
    request(node, 3)
    node.run_policy_step()
    assert node.fsm_state == 3
    assert 0 < node.action_publisher.messages[-1].drive <= 0.15
    assert 0 < node.action_publisher.messages[-1].steer <= 0.5
    assert node.pan_publisher.messages == []
    node.test_clock.advance(0.05)
    feed_road(empty=True)
    node.run_policy_step()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0
    assert node.controller.speed.integral == 0
    node.test_clock.advance(0.05)
    feed_road()
    node.run_policy_step()
    assert node.fsm_state == 2
    request(node, 3)
    node.run_policy_step()
    assert node.fsm_state == 3
    node.test_clock.advance(0.1)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert "reference expired" in node.state_reason
    assert node.action_publisher.messages[-1].drive == 0


def test_integrated_control_requires_feedback_and_matching_profiles(make_node):
    with pytest.raises(ValueError, match="requires cones"):
        make_node(required=("cone_detections",), **{"control.enabled": True})
    with pytest.raises(ValueError, match="profiles must be selected together"):
        make_node(required=("cone_detections", "wheel_speed", "imu_angular_velocity"),
                  **{"control.enabled": True, "planning.vehicle_limits_source": "course_simulation"})


@pytest.mark.parametrize("algorithm", ["centerline", "lattice_v2"])
def test_mvp_robustness_real_ros_callbacks_degrade_recover_and_stop(make_node, algorithm):
    node = make_node(required=("cone_detections", "wheel_speed", "imu_angular_velocity"),
        **{"control.enabled": True, "control.mode": "mvp", "control.robustness_enabled": True,
           "estimation.road_hold_s": 0.0,
           "planning.algorithm": algorithm, "planning.lattice.obstacle_check_enabled": False,
           "planning.lattice.budget_s": 5.0, "planning.lattice.clear_start_assumed": True,
           "control.drive_max": 0.35, "control.mvp_drive_feedforward": 0.30})

    def feed_road(empty=False):
        node.wheel_speed_callback(Float32(data=0.2))
        gyro = imu(node, orientation=False)
        gyro.angular_velocity_covariance[0] = 0.0
        node.imu_callback(gyro)
        batch = stamp(node, ConeDetections())
        if not empty:
            for colour, y in ((ConeDetection.COLOR_BLUE, 0.6), (ConeDetection.COLOR_YELLOW, -0.4)):
                for i in range(10):
                    cone = ConeDetection()
                    cone.position.x, cone.position.y = -0.1+0.3*i, y
                    cone.color, cone.classification_confidence = colour, 0.95
                    batch.detections.append(cone)
        node.cone_detection_callback(batch)

    feed_road()
    request(node, 3)
    if algorithm == "lattice_v2":
        node.planner = policy.FrenetLatticePlanner(node.planning_settings, node.lattice_settings,
                                                  clock=lambda: 0.0)
        node.previous_policy_step_at = node.test_clock.monotonic
    node.run_policy_step()
    assert node.fsm_state == 3
    node.controller.speed.integral = 0.4
    node.test_clock.advance(0.05)
    feed_road(empty=True)
    node.run_policy_step()
    assert node.fsm_state == 3
    assert node.control_diagnostics["tracking_mode"] == "DEGRADED"
    # V2 speed preview may adjust the integral slightly; recovery must retain
    # accumulated controller state rather than reinitialize it to zero.
    assert node.controller.speed.integral == pytest.approx(0.4, abs=1e-4)
    assert node.action_publisher.messages[-1].drive > 0
    assert not node.planning_output["valid"]
    node.test_clock.advance(0.05)
    feed_road()
    node.run_policy_step()
    assert node.control_diagnostics["tracking_mode"] == "TRACKING"
    assert node.distance_limiter.distance_m > 0
    request(node, 2)
    assert node.control_reference_manager.reference is None
    assert node.action_publisher.messages[-1].drive == 0
    node.supervision_callback()
    assert node.action_publisher.messages[-1].steer == 0


def test_mvp_real_ros_callbacks_distance_stop_and_restart(make_node):
    node = make_node(required=("cone_detections", "wheel_speed", "imu_angular_velocity"),
        **{"control.enabled": True, "control.mode": "mvp"})

    def feed_road(speed=0.2):
        node.wheel_speed_callback(Float32(data=speed))
        gyro = imu(node, orientation=False)
        gyro.angular_velocity_covariance[0] = 0.0
        node.imu_callback(gyro)
        batch = stamp(node, ConeDetections())
        for colour, y in ((ConeDetection.COLOR_BLUE, 0.6), (ConeDetection.COLOR_YELLOW, -0.4)):
            for i in range(10):
                cone = ConeDetection()
                cone.position.x, cone.position.y = -0.1+0.3*i, y
                cone.color, cone.classification_confidence = colour, 0.95
                batch.detections.append(cone)
        node.cone_detection_callback(batch)

    assert node.vehicle_settings.valid is False
    feed_road()
    request(node, 3)
    node.run_policy_step()
    assert node.fsm_state == 3
    assert node.planning_output["target_speed_mps"] == 0.2
    assert node.control_diagnostics["operating_profile"] == "mvp_low_speed"
    node.distance_limiter.distance_m = 2.995
    node.test_clock.advance(0.05)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert "Distance limit reached" in node.state_reason
    assert node.action_publisher.messages[-1].drive == 0
    assert node.action_publisher.messages[-1].steer == 0
    node.test_clock.advance(0.05)
    feed_road(speed=0.0)
    node.run_policy_step()
    node.supervision_callback()
    assert node.fsm_state == 2
    request(node, 3)
    assert node.distance_limiter.distance_m == 0
    node.run_policy_step()
    assert node.fsm_state == 3
    assert node.action_publisher.messages[-1].drive > 0
    assert node.pan_publisher.messages == []


def test_startup_zero_and_not_publishing_meanings(make_node):
    node = make_node()
    assert node.fsm_state == 2
    node.supervision_callback()
    assert [(m.drive, m.steer) for m in node.action_publisher.messages] == [(0.0, 0.0)]
    assert node.action_publisher.messages[0].units == DriveAndSteer.UNITS_NORMALIZED
    assert node.pan_publisher.messages == []
    request(node, 1)
    count = len(node.action_publisher.messages)
    node.supervision_callback()
    node.run_policy_step()
    node.stop_before_shutdown()
    assert len(node.action_publisher.messages) == count
    request(node, 99)
    assert node.fsm_state == 1
    request(node, 2)
    assert len(node.action_publisher.messages) == count + 1


@pytest.mark.parametrize("mode,required", [("cone_detection", ["cone_detections"]),
                                          ("fiducial_detection", ["fiducial_detections"]),
                                          ("lidar", ["lidar_scan"]), ("timer", [])])
def test_only_selected_trigger_executes_policy(make_node, mode, required):
    node = make_node(mode, required)
    camera_tf = TransformStamped()
    camera_tf.header.frame_id, camera_tf.child_frame_id = "base_link", "camera_optical"
    camera_tf.transform.rotation.w = 1.0
    node.tf_buffer.set_transform_static(camera_tf, "test")
    node.cone_detection_callback(cones(node))
    node.fiducial_detection_callback(fiducials(node))
    node.lidar_callback(scan(node))
    request(node, 3)
    calls = []
    node.calculate_policy_actions = lambda *args: (calls.append(args) or driving_policy())
    node.test_clock.advance(0.05)
    node.cone_detection_callback(cones(node))
    node.fiducial_detection_callback(fiducials(node))
    node.lidar_callback(scan(node))
    node.wheel_speed_callback(Float32(data=0.5))
    node.imu_callback(imu(node))
    node.supervision_callback()
    assert len(calls) == (0 if mode == "timer" else 1)
    assert (node.policy_timer is not None) == (mode == "timer")
    if mode == "timer":
        node.run_policy_step()
        assert len(calls) == 1
    assert calls[0][-1] is True
    assert calls[0][-3] == 0.0
    # Repeated sensor stamps do not trigger another calculation.
    node.cone_detection_callback(cones(node))
    node.fiducial_detection_callback(fiducials(node))
    node.lidar_callback(scan(node))
    assert len(calls) == 1


def test_fiducial_records_transform_and_fresh_empty_batches_trigger(make_node):
    node = make_node("fiducial_detection", ["fiducial_detections"])
    camera_tf = TransformStamped()
    camera_tf.header.frame_id, camera_tf.child_frame_id = "base_link", "camera_optical"
    camera_tf.transform.translation.x = 1.0
    camera_tf.transform.translation.y = 2.0
    camera_tf.transform.translation.z = 3.0
    camera_tf.transform.rotation.z = math.sin(math.pi / 4)
    camera_tf.transform.rotation.w = math.cos(math.pi / 4)
    node.tf_buffer.set_transform_static(camera_tf, "test")

    batch = fiducials(node)
    duplicate = FiducialDetection()
    duplicate.id = batch.detections[0].id
    duplicate.marker_size_m = 0.10
    for target, source in zip(duplicate.corners, batch.detections[0].corners):
        target.u, target.v = source.u, source.v
    duplicate.pose.orientation.w = 1.0
    duplicate.reprojection_error_px = math.nan
    batch.detections.append(duplicate)
    node.fiducial_detection_callback(batch)
    value = node.observations["fiducial_detections"].value
    assert value["dictionary_name"] == "DICT_4X4_50"
    assert value["source_frame_id"] == "camera_optical"
    assert [record["id"] for record in value["detections"]] == [7, 7]
    assert value["detections"][0]["corners"] == (
        (10.0, 20.0), (30.0, 20.0), (30.0, 40.0), (10.0, 40.0))
    assert value["detections"][0]["position_xyz"] == pytest.approx((1.0, 3.0, 5.0))
    assert value["detections"][0]["orientation_xyzw"] == pytest.approx(
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    assert value["detections"][1]["pose_valid"] is False
    assert value["detections"][1]["position_xyz"] is None
    assert math.isnan(value["detections"][1]["reprojection_error_px"])

    request(node, 3)
    calls = []
    node.calculate_policy_actions = lambda *args: (calls.append(args) or driving_policy())
    node.test_clock.advance(0.1)
    node.fiducial_detection_callback(fiducials(node, empty=True))
    assert node.fsm_state == 3
    assert len(calls) == 1
    student_value = calls[0][0]["fiducial_detections"]
    assert student_value["detections"] == []
    assert calls[0][1]["fiducial_detections"] == pytest.approx(0.0)


def test_bad_fiducial_or_missing_tf_does_not_refresh_observation(make_node):
    node = make_node(required=["fiducial_detections"])
    node.fiducial_detection_callback(fiducials(node))
    assert node.observations["fiducial_detections"] is None

    bad_dictionary = fiducials(node, frame="base_link")
    bad_dictionary.dictionary_name = "DICT_APRILTAG_36h11"
    node.fiducial_detection_callback(bad_dictionary)
    assert node.observations["fiducial_detections"] is None

    out_of_range = fiducials(node, frame="base_link")
    out_of_range.detections[0].id = 50
    node.fiducial_detection_callback(out_of_range)
    assert node.observations["fiducial_detections"] is None

    valid = fiducials(node, frame="base_link")
    node.fiducial_detection_callback(valid)
    accepted = node.observations["fiducial_detections"]
    node.test_clock.advance(0.1)
    invalid = fiducials(node, frame="base_link")
    invalid.detections[0].corners[0].u = math.nan
    node.fiducial_detection_callback(invalid)
    assert node.observations["fiducial_detections"] is accepted

    node.test_clock.advance(0.4)
    node.supervision_callback()
    request(node, 3)
    assert node.fsm_state == 2
    assert "missing or stale" in node.state_reason


def test_fiducial_transform_overflow_rejects_batch_without_refresh_or_trigger(make_node):
    node = make_node("fiducial_detection", ["fiducial_detections"])
    node.fiducial_detection_callback(fiducials(node, frame="base_link"))
    accepted = node.observations["fiducial_detections"]
    request(node, 3)
    calls = []
    node.calculate_policy_actions = lambda *args: (calls.append(args) or driving_policy())
    action_count = len(node.action_publisher.messages)

    camera_tf = TransformStamped()
    camera_tf.header.frame_id, camera_tf.child_frame_id = "base_link", "camera_optical"
    camera_tf.transform.rotation.w = 1.0
    camera_tf.transform.translation.x = 1.5e308
    node.tf_buffer.set_transform_static(camera_tf, "test")
    node.test_clock.advance(0.1)
    batch = fiducials(node)
    overflowing = fiducials(node).detections[0]
    overflowing.id = 8
    overflowing.pose.position.x = 1.5e308
    # The first marker can transform; only the second overflows. Neither may
    # replace the previously accepted batch or trigger a policy calculation.
    batch.detections.append(overflowing)
    node.fiducial_detection_callback(batch)
    assert node.observations["fiducial_detections"] is accepted
    assert calls == []
    assert len(node.action_publisher.messages) == action_count

    node.test_clock.advance(0.401)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert "missing or stale" in node.state_reason
    assert (node.action_publisher.messages[-1].drive,
            node.action_publisher.messages[-1].steer) == (0.0, 0.0)

    node.fiducial_detection_callback(fiducials(node, frame="base_link"))
    assert node.observations["fiducial_detections"] is not accepted
    assert node.fsm_state == 2  # Fresh data still needs an explicit resume.
    assert calls == []


@pytest.mark.parametrize("change", [
    lambda detection: setattr(detection, "id", -1),
    lambda detection: setattr(detection, "marker_size_m", 0.0),
    lambda detection: setattr(detection.pose.orientation, "w", 2.0),
    lambda detection: setattr(detection, "reprojection_error_px", math.nan),
    lambda detection: setattr(detection, "pose_valid", False),
])
def test_invalid_fiducial_fields_reject_whole_batch(make_node, change):
    node = make_node()
    msg = fiducials(node, frame="base_link")
    change(msg.detections[0])
    node.fiducial_detection_callback(msg)
    assert node.observations["fiducial_detections"] is None


@pytest.mark.parametrize("required", [["lidar_scan"], ["wheel_speed"]])
def test_exercises_need_no_cone_publisher_and_recovery_is_explicit(make_node, required):
    mode = "lidar" if required == ["lidar_scan"] else "timer"
    node = make_node(mode, required)
    feed = (lambda: node.lidar_callback(scan(node))) if mode == "lidar" else (
        lambda: node.wheel_speed_callback(Float32(data=0.0)))
    request(node, 3)
    assert node.fsm_state == 2
    feed()
    node.calculate_policy_actions = driving_policy
    request(node, 3)
    assert node.fsm_state == 3
    node.run_policy_step()
    assert node.action_publisher.messages[-1].drive > 0.0
    node.test_clock.advance(0.5)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0.0
    feed()
    node.supervision_callback()
    assert node.fsm_state == 2
    request(node, 3)
    assert node.fsm_state == 3


def lidar_mount(node):
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "laser"
    transform.transform.translation.x = 0.2
    transform.transform.translation.z = 0.12
    transform.transform.rotation.w = 1.0
    node.tf_buffer.set_transform_static(transform, "test")
    return transform


def test_lidar_conversion_filters_preserves_indices_and_transforms_all_axes(make_node, monkeypatch):
    node = make_node("lidar", ["lidar_cartesian"])
    transform = lidar_mount(node)
    transform.transform.translation.y = -0.1
    transform.transform.rotation.y = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    node.tf_buffer.set_transform_static(transform, "test")
    calls = []
    lookup = node.tf_buffer.lookup_transform

    def record_lookup(*args, **kwargs):
        calls.append((args, kwargs))
        return lookup(*args, **kwargs)

    monkeypatch.setattr(node.tf_buffer, "lookup_transform", record_lookup)
    message = scan(node)
    message.range_min, message.range_max = 0.5, 2.0
    message.ranges = [0.5, math.inf, math.nan, -math.inf, 0.1, 3.0, 2.0]
    message.intensities = [10.0 + i for i in range(7)]
    message.angle_max = message.angle_min + 6 * message.angle_increment
    node.lidar_callback(message)
    raw = node.observations["lidar_scan"]
    cartesian = node.observations["lidar_cartesian"]
    assert math.isnan(raw.value["ranges"][2])
    assert raw.value["ranges"][1] == math.inf
    assert raw.value["intensities"] == list(message.intensities)
    assert cartesian.stamp_ns == raw.stamp_ns == node._stamp_ns(message.header)
    assert cartesian.received_at == raw.received_at
    assert cartesian.value["frame_id"] == "base_link"
    assert cartesian.value["scan_indices"] == [0, 6]
    assert cartesian.value["points_xyz"][0] == pytest.approx((0.2, -0.1, -0.38))
    angle = 6 * message.angle_increment
    assert cartesian.value["points_xyz"][1] == pytest.approx(
        (0.2, -0.1 + 2 * math.sin(angle), 0.12 - 2 * math.cos(angle)))
    assert len(calls) == 1
    assert calls[0][0][:2] == ("base_link", "laser")
    assert calls[0][0][2].nanoseconds == raw.stamp_ns
    assert calls[0][1] == {}  # No blocking timeout.


@pytest.mark.parametrize("mode,required", [
    ("lidar", ["lidar_scan"]), ("lidar", ["lidar_cartesian"]),
    ("timer", ["lidar_cartesian"])])
def test_lidar_conversion_failure_keeps_raw_and_invalidates_points_before_policy(
        make_node, monkeypatch, mode, required):
    node = make_node(mode, required)
    transform = lidar_mount(node)
    node.lidar_callback(scan(node))
    request(node, 3)
    snapshots = []
    node.calculate_policy_actions = lambda *args: (snapshots.append(args[0]) or driving_policy())

    def missing(*args, **kwargs):
        raise policy.TransformException("mount unavailable")

    monkeypatch.setattr(node.tf_buffer, "lookup_transform", missing)
    node.test_clock.advance(0.1)
    message = scan(node)
    node.lidar_callback(message)
    node.supervision_callback()
    assert node.observations["lidar_scan"].stamp_ns == node._stamp_ns(message.header)
    assert node.observations["lidar_cartesian"] is None
    if "lidar_cartesian" in required:
        assert not snapshots
        assert node.fsm_state == 2
        assert node.action_publisher.messages[-1].drive == 0.0
    else:
        assert node.fsm_state == 3
        assert snapshots[-1]["lidar_scan"] is not None
        assert snapshots[-1]["lidar_cartesian"] is None
    monkeypatch.setattr(node.tf_buffer, "lookup_transform", lambda *args: transform)
    node.test_clock.advance(0.1)
    node.lidar_callback(scan(node))
    assert node.observations["lidar_cartesian"] is not None
    if "lidar_cartesian" in required:
        assert node.fsm_state == 2
        request(node, 3)
        assert node.fsm_state == 3


@pytest.mark.parametrize("bad_rotation", [False, True])
def test_lidar_invalid_transform_preserves_raw_but_clears_cartesian(make_node, monkeypatch, bad_rotation):
    node = make_node()
    transform = lidar_mount(node)
    node.lidar_callback(scan(node))
    if bad_rotation:
        transform.transform.rotation.w = 0.0
    else:
        transform.transform.translation.x = math.nan
    monkeypatch.setattr(node.tf_buffer, "lookup_transform", lambda *args: transform)
    node.test_clock.advance(0.1)
    node.lidar_callback(scan(node))
    assert node.observations["lidar_scan"] is not None
    assert node.observations["lidar_cartesian"] is None


def test_lidar_empty_converted_scan_is_valid_and_identity_needs_no_tf(make_node, monkeypatch):
    node = make_node("lidar", ["lidar_cartesian"])
    monkeypatch.setattr(node.tf_buffer, "lookup_transform",
                        lambda *args: pytest.fail("identity must not need TF"))
    message = scan(node, frame="base_link")
    message.ranges = [math.inf, math.nan]
    node.lidar_callback(message)
    request(node, 3)
    assert node.fsm_state == 3
    value = node.observations["lidar_cartesian"].value
    assert value == {"points_xyz": [], "scan_indices": [], "frame_id": "base_link"}
    node.test_clock.advance(0.1)
    node.lidar_callback(scan(node, frame="base_link"))
    assert node.observations["lidar_cartesian"].value["points_xyz"] == [(1.0, 0.0, 0.0)]


def test_lidar_rejected_scan_does_not_clear_matching_cartesian_or_retry_conversion(make_node, monkeypatch):
    node = make_node()
    lidar_mount(node)
    node.lidar_callback(scan(node))
    raw, cartesian = node.observations["lidar_scan"], node.observations["lidar_cartesian"]
    monkeypatch.setattr(node.tf_buffer, "lookup_transform",
                        lambda *args: pytest.fail("rejected scan must not convert"))
    node.lidar_callback(scan(node))  # Same stamp.
    node.test_clock.advance(0.1)
    invalid = scan(node)
    invalid.angle_increment = 0.0
    node.lidar_callback(invalid)
    node.lidar_callback(scan(node, offset=-1.0))
    assert node.observations["lidar_scan"] is raw
    assert node.observations["lidar_cartesian"] is cartesian


@pytest.mark.parametrize("scan_timeout,cartesian_timeout", [(0.2, 0.5), (0.5, 0.2)])
def test_lidar_cartesian_snapshots_are_copies_and_expire_at_both_deadlines(
        make_node, scan_timeout, cartesian_timeout):
    node = make_node(**{"sensor_timeout_s.lidar_scan": scan_timeout,
                        "sensor_timeout_s.lidar_cartesian": cartesian_timeout})
    lidar_mount(node)
    node.lidar_callback(scan(node))
    request(node, 3)
    captured = []

    def student(values, ages, stamps, *args):
        captured.append((values, ages, stamps))
        if values["lidar_cartesian"] is not None:
            values["lidar_cartesian"]["points_xyz"].clear()
            values["lidar_cartesian"]["scan_indices"].clear()
        return driving_policy()

    node.calculate_policy_actions = student
    node.run_policy_step()
    assert len(node.observations["lidar_cartesian"].value["points_xyz"]) == 1
    assert node.observations["lidar_cartesian"].value["scan_indices"] == [0]
    assert captured[-1][2]["lidar_cartesian"] == captured[-1][2]["lidar_scan"]
    node.test_clock.advance(0.201)
    node.run_policy_step()
    assert captured[-1][0]["lidar_cartesian"] is None
    assert (captured[-1][0]["lidar_scan"] is None) == (scan_timeout < cartesian_timeout)
    assert captured[-1][1]["lidar_cartesian"] == pytest.approx(0.201)


def test_empty_cones_and_missing_stream_have_independent_deadlines(make_node):
    node = make_node("cone_detection", ["cone_detections"])
    node.cone_detection_callback(cones(node, empty=True))
    request(node, 3)
    assert node.fsm_state == 2  # Never had a nonempty frame.
    node.test_clock.advance(0.1)
    node.cone_detection_callback(cones(node))
    request(node, 3)
    for _ in range(4):
        node.test_clock.advance(0.2)
        node.cone_detection_callback(cones(node, empty=True))
        assert node.fsm_state == 3
    node.test_clock.advance(0.201)
    node.cone_detection_callback(cones(node, empty=True))
    assert node.fsm_state == 2
    assert "nonempty" in node.state_reason
    node.test_clock.advance(0.01)
    node.cone_detection_callback(cones(node))
    assert node.fsm_state == 2
    request(node, 3)
    node.test_clock.advance(0.5)
    node.supervision_callback()
    assert node.fsm_state == 2
    assert "missing or stale" in node.state_reason


@pytest.mark.parametrize("offset", [-0.5, 0.051, -100.0])
def test_rejected_stamps_do_not_refresh_data(make_node, offset):
    node = make_node("lidar", ["lidar_scan"])
    node.lidar_callback(scan(node, offset=offset))
    assert node.observations["lidar_scan"] is None
    node.lidar_callback(scan(node))
    received = node.observations["lidar_scan"].received_at
    node.test_clock.advance(0.1)
    node.lidar_callback(scan(node, offset=-0.1))
    assert node.observations["lidar_scan"].received_at == received


def test_optional_stale_values_are_unavailable_and_snapshots_are_copies(make_node):
    node = make_node()
    node.cone_detection_callback(cones(node))
    node.lidar_callback(scan(node))
    request(node, 3)
    captured = []

    def student(values, ages, stamps, *args):
        captured.append((values, ages, stamps))
        if values["lidar_scan"] is not None:
            values["lidar_scan"]["ranges"][0] = 99.0
        return driving_policy()

    node.calculate_policy_actions = student
    node.run_policy_step()
    assert node.observations["lidar_scan"].value["ranges"][0] == 1.0
    node.test_clock.advance(0.5)
    node.run_policy_step()
    assert captured[-1][0]["lidar_scan"] is None
    assert captured[-1][0]["cone_detections"] is None
    assert captured[-1][1]["lidar_scan"] == pytest.approx(0.5)
    assert node.fsm_state == 3


@pytest.mark.parametrize("empty", [False, True])
def test_cone_publication_latency_is_batch_metadata_not_added_to_age(make_node, empty):
    node = make_node()
    message = cones(node, empty=empty, offset=-0.2)
    message.acquisition_to_publish_latency_s = 0.15
    node.cone_detection_callback(message)
    request(node, 3)
    captured = []

    def student(values, ages, stamps, *args):
        captured.append((values, ages, stamps))
        if values["cone_detections"] is not None:
            assert values["cone_detections"]["acquisition_to_publish_latency_s"] == 0.15
            values["cone_detections"]["acquisition_to_publish_latency_s"] = 99.0
        return driving_policy()

    node.calculate_policy_actions = student
    node.test_clock.advance(0.1)
    node.run_policy_step()
    accepted = node.observations["cone_detections"]
    assert accepted.value["acquisition_to_publish_latency_s"] == 0.15
    assert bool(accepted.value["detections"]) is not empty
    assert captured[-1][1]["cone_detections"] == pytest.approx(0.3)
    assert captured[-1][2]["cone_detections"] == node._stamp_ns(message.header)

    node.test_clock.advance(0.201)
    node.run_policy_step()
    assert captured[-1][0]["cone_detections"] is None
    assert captured[-1][1]["cone_detections"] == pytest.approx(0.501)


@pytest.mark.parametrize("latency", [-0.1, math.nan, math.inf])
def test_invalid_cone_latency_does_not_refresh_or_trigger(make_node, latency):
    node = make_node("cone_detection", ["cone_detections"])
    node.cone_detection_callback(cones(node))
    accepted = node.observations["cone_detections"]
    request(node, 3)
    calls = []
    node.calculate_policy_actions = lambda *args: (calls.append(args) or driving_policy())
    node.test_clock.advance(0.1)
    message = cones(node)
    message.acquisition_to_publish_latency_s = latency
    node.cone_detection_callback(message)
    assert node.observations["cone_detections"] is accepted
    assert calls == []


def test_imu_mounting_rotation_and_partial_field_freshness(make_node):
    node = make_node(required=["imu_orientation"])
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "imu_sensor"
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    node.tf_buffer.set_transform_static(transform, "test")
    msg = imu(node, yaw=math.pi / 2, frame="imu_sensor")
    msg.angular_velocity_covariance[0] = 0.0
    msg.angular_velocity.x = 1.0
    msg.linear_acceleration_covariance[0] = 0.0
    msg.linear_acceleration.z = 9.81
    node.imu_callback(msg)
    assert policy.roll_pitch_yaw(node.observations["imu_orientation"].value)[2] == pytest.approx(0.0)
    assert node.observations["imu_angular_velocity"].value == pytest.approx((0.0, 1.0, 0.0))
    assert node.observations["imu_specific_force"].value == pytest.approx((0.0, 0.0, 9.81))
    request(node, 3)
    node.test_clock.advance(0.5)
    partial = imu(node, orientation=False, frame="imu_sensor")
    partial.angular_velocity_covariance[0] = 0.0
    partial.angular_velocity.x = 2.0
    node.imu_callback(partial)
    now, ros_now = node._times()
    assert node._fresh("imu_angular_velocity", now, ros_now)
    assert not node._fresh("imu_orientation", now, ros_now)
    node.supervision_callback()
    assert node.fsm_state == 2


def test_missing_imu_transform_or_bad_quaternion_cannot_satisfy_required_heading(make_node):
    node = make_node(required=["imu_orientation"])
    node.imu_callback(imu(node, frame="missing_mount"))
    request(node, 3)
    assert node.fsm_state == 2
    msg = imu(node)
    msg.orientation.w = 0.0
    node.imu_callback(msg)
    assert node.observations["imu_orientation"] is None


def test_older_imu_field_does_not_replace_newer_but_new_peer_is_accepted(make_node):
    node = make_node()
    node.imu_callback(imu(node, yaw=0.4))
    orientation = node.observations["imu_orientation"]
    msg = imu(node, yaw=1.0, offset=-0.01)
    msg.angular_velocity_covariance[0] = 0.0
    msg.angular_velocity.x = 0.2
    node.imu_callback(msg)
    assert node.observations["imu_orientation"] is orientation
    assert node.observations["imu_angular_velocity"].value[0] == pytest.approx(0.2)


def test_heading_and_time_reset_only_on_actual_state_entry(make_node):
    node = make_node(required=["imu_orientation"])
    node.imu_callback(imu(node, yaw=0.4))
    request(node, 3)
    start = node.policy_started_at
    node.run_policy_step()
    node.test_clock.advance(0.1)
    node.imu_callback(imu(node, yaw=0.9))
    request(node, 3)
    assert node.heading_reference == pytest.approx(0.4)
    assert node.policy_started_at == start
    assert node.previous_policy_step_at is not None
    assert node.heading_publisher.messages[-1].data == pytest.approx(math.degrees(0.5))
    request(node, 2)
    request(node, 3)
    assert node.heading_reference == pytest.approx(0.9)
    assert node.previous_policy_step_at is None
    assert node.policy_started_at > start


def test_optional_delayed_heading_does_not_silently_tare(make_node):
    node = make_node()
    request(node, 3)
    node.imu_callback(imu(node, yaw=0.5))
    assert node.heading_reference is None
    assert not node.heading_publisher.messages


@pytest.mark.parametrize("required", [["lidar_scan"], ["wheel_speed"]])
def test_backward_clock_invalidates_stamped_data_only(make_node, required):
    node = make_node(required=required)
    node.lidar_callback(scan(node))
    node.wheel_speed_callback(Float32(data=0.0))
    request(node, 3)
    node.test_clock.ros_ns -= 1_000_000_000
    node.supervision_callback()
    assert node.observations["lidar_scan"] is None
    assert node.fsm_state == (2 if "lidar_scan" in required else 3)


@pytest.mark.parametrize("result", [(math.nan, 0, None, None, None),
                                    (0.4, 0, math.inf, None, None),
                                    ("0.4", 0, None, None, None),
                                    (True, 0, None, None, None)])
def test_invalid_result_stops_before_any_pan_or_nonzero_publication(make_node, result):
    node = make_node()
    request(node, 3)
    node.calculate_policy_actions = lambda *args: result
    node.run_policy_step()
    assert node.fsm_state == 2
    assert all(msg.drive == 0.0 and msg.steer == 0.0 for msg in node.action_publisher.messages)
    assert not node.pan_publisher.messages


def test_clipping_pan_hold_exception_and_overrun(make_node):
    node = make_node(required=["wheel_speed"])
    node.wheel_speed_callback(Float32(data=0.0))
    request(node, 3)
    node.calculate_policy_actions = lambda *args: (3.0, -4.0, 2.0, None, None)
    node.run_policy_step()
    msg = node.action_publisher.messages[-1]
    assert (msg.drive, msg.steer) == (1.0, -1.0)
    assert node.pan_publisher.messages[-1].data == 1.0

    def slow_student(*args):
        node.test_clock.advance(0.5)
        return driving_policy()

    node.calculate_policy_actions = slow_student
    node.run_policy_step()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0.0
    node.supervision_callback()
    assert len(node.pan_publisher.messages) == 1
    node.wheel_speed_callback(Float32(data=0.0))
    request(node, 3)

    def broken_student(*args):
        raise ZeroDivisionError("student computation")

    node.calculate_policy_actions = broken_student
    node.run_policy_step()
    assert node.fsm_state == 2
    assert node.action_publisher.messages[-1].drive == 0.0


def test_shipped_student_calculation_is_zero_and_handles_missing_observations(make_node):
    node = make_node()
    request(node, 3)
    node.run_policy_step()
    assert node.action_publisher.messages[-1].drive == 0.0
    assert node.action_publisher.messages[-1].steer == 0.0
    assert not node.pan_publisher.messages
    assert node.estimation_output is not None
    assert not node.estimation_output["state"]["valid"]
    # Missing wheel speed is represented as None even though actuator output
    # is zero. This must not manufacture a stopped-state measurement.
    assert node.estimation_output["state"]["speed_mps"] is None


@pytest.mark.parametrize("buffer_history", [True, False])
def test_student_geometry_aligns_to_state_only_with_motion_history(make_node, buffer_history):
    node = make_node()
    request(node, 3)
    first_stamp = node.test_clock.ros_ns
    for index in range(3):
        if index:
            node.test_clock.advance(0.05)
        if not buffer_history and index != 2:
            continue
        node.wheel_speed_callback(Float32(data=0.5))
        gyro = imu(node, orientation=False)
        gyro.angular_velocity_covariance[0] = 0.0
        gyro.angular_velocity.z = 0.0
        node.imu_callback(gyro)
    batch = stamp(node, ConeDetections(), offset=-0.1)
    batch.acquisition_to_publish_latency_s = 0.08
    for colour, lateral in ((ConeDetection.COLOR_BLUE, 0.5), (ConeDetection.COLOR_YELLOW, -0.5)):
        for x in (-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8):
            cone = ConeDetection()
            cone.position.x, cone.position.y = x, lateral
            cone.color, cone.classification_confidence = colour, 0.95
            batch.detections.append(cone)
    node.cone_detection_callback(batch)
    node.run_policy_step()
    out = node.estimation_output
    assert out["state"]["valid"]
    assert out["road"]["measurement_timestamp_s"] == first_stamp/1e9
    assert out["alignment"]["valid"] == buffer_history
    if buffer_history:
        assert out["road"]["timestamp_s"] == out["state"]["timestamp_s"]
        assert out["road"]["centerline_xy"][0][0] == pytest.approx(-0.35)
        assert out["road"]["near_field"]["valid"]
    else:
        assert not out["road"]["valid"]
        assert out["road"]["timestamp_s"] is None
        assert out["road"]["status"] == "missing_motion_history"
    assert node.action_publisher.messages[-1].drive == 0.0
    assert node.action_publisher.messages[-1].steer == 0.0


def test_student_empty_frame_bridge_preserves_acquisition_and_rejects_missing_gyro(make_node):
    node = make_node()
    request(node, 3)
    node.wheel_speed_callback(Float32(data=0.5))
    gyro = imu(node, orientation=False)
    gyro.angular_velocity_covariance[0] = 0.0
    gyro.angular_velocity.z = 0.0
    node.imu_callback(gyro)
    batch = stamp(node, ConeDetections())
    original_stamp_s = node.test_clock.ros_ns/1e9
    for colour, lateral in ((ConeDetection.COLOR_BLUE, 0.5), (ConeDetection.COLOR_YELLOW, -0.5)):
        for x in (-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8):
            cone = ConeDetection()
            cone.position.x, cone.position.y = x, lateral
            cone.color, cone.classification_confidence = colour, 0.95
            batch.detections.append(cone)
    node.cone_detection_callback(batch)
    node.run_policy_step()
    assert node.estimation_output["road"]["valid"]

    node.test_clock.advance(0.05)
    node.wheel_speed_callback(Float32(data=0.5))
    gyro = imu(node, orientation=False)
    gyro.angular_velocity_covariance[0] = 0.0
    gyro.angular_velocity.z = 0.0
    node.imu_callback(gyro)
    node.cone_detection_callback(cones(node, empty=True))
    node.run_policy_step()
    road = node.estimation_output["road"]
    assert road["valid"] and road["degraded"]
    assert road["measurement_timestamp_s"] == original_stamp_s
    assert road["timestamp_s"] == node.estimation_output["state"]["timestamp_s"]
    assert road["source_age_s"] == pytest.approx(0.05)
    assert road["current_frame"]["visibility"] == "none"
    assert road["centerline_xy"][0][0] == pytest.approx(-0.325)

    # A partial IMU message does not erase the previously accepted gyro.
    # Let that actual gyro sample exceed the estimator's 0.15 s motion limit.
    node.test_clock.advance(0.16)
    node.wheel_speed_callback(Float32(data=0.5))
    missing = imu(node, orientation=False)
    missing.angular_velocity_covariance[0] = -1.0
    node.imu_callback(missing)
    node.run_policy_step()
    assert not node.estimation_output["state"]["yaw_rate_valid"]
    assert not node.estimation_output["road"]["valid"]
    assert node.estimator._trusted_road is None
    assert node.action_publisher.messages[-1].drive == 0.0
    assert node.action_publisher.messages[-1].steer == 0.0


def test_invalid_sensor_content_and_runtime_parameter_changes(make_node):
    node = make_node()
    bad_cone = cones(node)
    bad_cone.detections[0].position.x = math.nan
    node.cone_detection_callback(bad_cone)
    node.cone_detection_callback(cones(node, frame="optical"))
    bad_scan = scan(node)
    bad_scan.angle_increment = 0.0
    node.lidar_callback(bad_scan)
    node.wheel_speed_callback(Float32(data=-0.1))
    assert all(value is None for value in node.observations.values())
    assert not node.set_parameters_atomically([Parameter("use_sim_time", value=True)]).successful
    assert not node.set_parameters_atomically([Parameter("policy_update_mode", value="lidar")]).successful


@pytest.mark.parametrize("mode,required,extra", [
    ("automatic", [], {}), ("lidar", [], {}), ("cone_detection", [], {}),
    ("fiducial_detection", [], {}),
    ("timer", ["unknown"], {}), ("timer", ["lidar_scan", "lidar_scan"], {}),
    ("lidar", ["lidar"], {}),
    ("timer", [], {"supervision_rate_hz": 0.0}),
    ("timer", [], {"timestamp_tolerance_s": -1.0}),
    ("timer", [], {"policy_frame_id": "/base_link"}),
    ("timer", [], {"use_sim_time": True}),
])
def test_invalid_startup_settings_fail(make_node, mode, required, extra):
    with pytest.raises(ValueError):
        make_node(mode, required, **extra)


def test_installed_configs_and_namespaced_loading(tmp_path):
    assert yaml.safe_load((SHARE / "config/lidar_mount.yaml").read_text()) == {}
    for name in ("ai4r_policy", "traxxas_vehicle_interface", "oakd_cone_detector",
                 "bno08x_imu_interface", "aruco_detector"):
        document = yaml.safe_load((SHARE / "config" / f"{name}.yaml").read_text())
        assert list(document) == [f"/**/{name}"]
        assert isinstance(document[f"/**/{name}"]["ros__parameters"], dict)
    aruco_parameters = yaml.safe_load(
        (SHARE / "config/aruco_detector.yaml").read_text())["/**/aruco_detector"]["ros__parameters"]
    assert aruco_parameters == {
        "dictionary_name": "DICT_4X4_50", "default_marker_size_m": 0.25,
        "allowed_marker_ids": [], "min_consecutive_frames": 3}
    context = Context()
    rclpy.init(context=context)
    node = policy.PolicyNode(context=context, namespace="namespaced",
        cli_args=["--ros-args", "--params-file", str(SHARE / "config/ai4r_policy.yaml")])
    try:
        assert node.get_fully_qualified_name() == "/namespaced/ai4r_policy"
        assert node.policy_update_mode == "timer"
        assert node.policy_update_rate_hz == 20.0
        assert node.required_sensors == ["cone_detections", "wheel_speed", "imu_angular_velocity"]
        assert node.control_settings.enabled is True
        assert node.control_settings.mode == "mvp"
        assert node.planning_settings.algorithm == "lattice_v2"
        assert node.lattice_settings.obstacle_check_enabled is False
        assert node.estimation_settings.road_history_s == 5.0
        assert node.estimation_settings.road_hold_s == 0.2

        assert node.control_settings.robustness_enabled is True
        assert node.control_settings.reference_hold_max_s == 1.0
        assert node.control_settings.reference_hold_max_distance_m == 0.25
        assert node.control_settings.max_distance_m == 3.0
        assert node.control_settings.max_run_time_s == 30.0
        assert node.lattice_settings.direct_sample_output is True
        assert node.lattice_settings.async_enabled is True
        assert node.lattice_settings.recovery_enabled is True
        assert node.lattice_settings.update_rate_hz == 10.0
        assert node.control_settings.mvp_steering_direction == -1.0
        assert node.control_settings.mvp_drive_feedforward == 0.30
        assert node.control_settings.drive_max == 0.35
        assert node.control_settings.degraded_speed_mps == 0.1
        assert node.vehicle_settings.valid is False
        assert node.fsm_state == 2
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("failure", ["paused", "killed"])
def test_mvp2_actual_worker_does_not_block_control_and_restarts(make_node, failure):
    node = make_node(required=("cone_detections", "wheel_speed", "imu_angular_velocity"),
        **{"control.enabled": True, "control.mode": "mvp", "control.robustness_enabled": True,
           "planning.algorithm": "lattice_v2", "planning.lattice.async_enabled": True,
           "planning.lattice.recovery_enabled": False, "planning.lattice.direct_sample_output": True,
           "planning.lattice.clear_start_assumed": True,
           "planning.lattice.obstacle_check_enabled": False, "planning.lattice.update_rate_hz": 10.0,
           "planning.reference_lifetime_s": 0.2, "control.startup_grace_s": 3.0,
           "estimation.motion_max_gap_s": 0.25, "control.degraded_speed_mps": 0.1,
           "control.drive_max": 0.35, "control.mvp_drive_feedforward": 0.30})

    def step():
        node.test_clock.advance(0.05)
        node.wheel_speed_callback(Float32(data=0.0))
        gyro = imu(node, orientation=False)
        gyro.angular_velocity_covariance[0] = 0.0
        node.imu_callback(gyro)
        batch = stamp(node, ConeDetections())
        for colour, shift in ((ConeDetection.COLOR_BLUE, 0.6), (ConeDetection.COLOR_YELLOW, -0.6)):
            for i in range(13):
                cone = ConeDetection()
                x = -0.3+0.25*i
                cone.position.x, cone.position.y = x, 0.1+0.04*x*x+shift
                cone.color, cone.classification_confidence = colour, 0.95
                batch.detections.append(cone)
        node.cone_detection_callback(batch)
        began = time.monotonic()
        node.run_policy_step()
        return time.monotonic()-began

    step()
    request(node, 3)
    deadline = time.monotonic()+3.0
    while not node.control_has_run and time.monotonic() < deadline:
        step()
        time.sleep(0.05)
    assert node.control_has_run, node.state_reason
    assert node.planning_output["path"]["type"] == "CARTESIAN_SAMPLES"
    assert node.control_diagnostics["curvature_1pm"] > 0
    assert node.action_publisher.messages[-1].steer > 0
    child = node.async_planner.process
    timings = []
    if failure == "paused":
        child.send_signal(signal.SIGSTOP)
        try:
            for _ in range(4):
                timings.append(step())
                time.sleep(0.05)
                assert node.fsm_state == 3, node.state_reason
                assert node.action_publisher.messages[-1].drive > 0
        finally:
            child.send_signal(signal.SIGCONT)
    else:
        child.kill()
        child.wait(timeout=0.5)
        timings.append(step())
        assert node.async_planner.process.pid != child.pid
        assert node.fsm_state == 3, node.state_reason
    deadline = time.monotonic()+0.9
    while time.monotonic() < deadline:
        timings.append(step())
        assert node.fsm_state == 3, node.state_reason
        time.sleep(0.05)
        if node.planning_output.get("valid"):
            break
    assert node.planning_output.get("valid"), node.state_reason
    assert max(timings) < 0.2, timings
    evidence = Path(os.environ["ROS_LOG_DIR"])/f"mvp2-worker-{failure}.json"
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_text(json.dumps({"failure": failure, "control_call_ms": [t*1000 for t in timings],
        "max_control_call_ms": max(timings)*1000, "last_planner_elapsed_s": node.last_planner_elapsed_s}, indent=2)+"\n")
    request(node, 2)
    assert node.action_publisher.messages[-1].drive == 0
    old_generation = node.planning_generation
    for _ in range(3):
        step()
        node.supervision_callback()
    assert node.fsm_state == 2
    assert node.planning_generation == old_generation
    assert node.action_publisher.messages[-1].drive == 0

def test_open_loop_yaml_empty_sensor_array(tmp_path):
    # [] has no element from which the ROS YAML parser can infer an array type.
    # Confirm the deliberate open-loop configuration also works through YAML.
    overlay = tmp_path / "open_loop.yaml"
    overlay.write_text("/**/ai4r_policy:\n  ros__parameters:\n"
                       "    policy_update_mode: timer\n    required_sensors: []\n")
    context = Context()
    rclpy.init(context=context)
    node = policy.PolicyNode(context=context, namespace="namespaced",
        cli_args=["--ros-args", "--params-file", str(overlay)])
    try:
        assert node.policy_update_mode == "timer"
        assert node.required_sensors == []
    finally:
        node.destroy_node()
        context.shutdown()


def test_installed_launch_starts_executable_and_shuts_down(tmp_path):
    assert os.access(SCRIPT, os.X_OK), "Installed policy script is not executable"
    environment = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    output_path = tmp_path / "launch-output.log"
    with output_path.open("w", encoding="utf-8") as output_file:
        process = subprocess.Popen(
            ["ros2", "launch", "ai4r_policy", "ai4r_policy.launch.py", "namespace:=launch_check"],
            cwd=tmp_path, env=environment, stdout=output_file, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        try:
            marker = "Policy update source: timer"
            deadline = time.monotonic() + 10.0
            output = ""
            while time.monotonic() < deadline:
                output_file.flush()
                output = output_path.read_text(encoding="utf-8", errors="replace")
                if marker in output:
                    break
                if process.poll() is not None:
                    pytest.fail("Policy launch exited before readiness:\n" + output)
                time.sleep(0.05)
            else:
                pytest.fail("Policy launch did not become ready within 10 seconds:\n" + output)

            assert process.poll() is None, output
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5.0)
            output_file.flush()
            output = output_path.read_text(encoding="utf-8", errors="replace")
            assert process.returncode == 0, output
            assert marker in output
            assert "[ERROR]" not in output and "Traceback" not in output
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)


@pytest.mark.parametrize("mode", ["lidar", "timer"])
def test_real_ros_graph_zero_handshake_sensor_qos_and_watchdog(mode):
    """Exercise DDS/timers with synthetic peers, not a simulated physical car."""
    context = Context()
    rclpy.init(context=context)
    required = "lidar_scan" if mode == "lidar" else "wheel_speed"
    node = policy.PolicyNode(context=context, namespace="graph_test",
        parameter_overrides=[Parameter("policy_update_mode", value=mode),
                             Parameter("required_sensors", value=[required])])
    peer = Node("synthetic_peer", context=context, namespace="graph_test")
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(peer)
    actions, pans = [], []
    peer.create_subscription(DriveAndSteer, "drive_and_steer_set_point_normalized", actions.append, 10)
    peer.create_subscription(Float32, "pan_set_point_normalized", pans.append, 10)
    request_pub = peer.create_publisher(UInt16, "policy_fsm_transition_request", 10)
    # A reliable scan publisher must match the policy's best-effort subscription.
    sensor_pub = peer.create_publisher(LaserScan if mode == "lidar" else Float32,
                                      "scan" if mode == "lidar" else "wheel_speed_m_per_sec",
                                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    node.calculate_policy_actions = driving_policy

    def spin_until(condition, timeout=3.0, feed=False, start=False):
        deadline = time.monotonic() + timeout
        next_send = 0.0
        while time.monotonic() < deadline:
            if feed and time.monotonic() >= next_send:
                if mode == "lidar":
                    msg = LaserScan()
                    msg.header.stamp = peer.get_clock().now().to_msg()
                    msg.header.frame_id = "laser"
                    msg.angle_max, msg.angle_increment = 0.1, 0.1
                    msg.range_min, msg.range_max, msg.ranges = 0.05, 12.0, [1.0, math.inf]
                else:
                    msg = Float32(data=0.0)
                sensor_pub.publish(msg)
                if start:
                    request_pub.publish(UInt16(data=3))
                next_send = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.01)
            if condition():
                return
        pytest.fail("ROS graph did not reach expected state within its bounded deadline")

    try:
        spin_until(lambda: len(actions) >= 2)
        assert all(m.drive == 0.0 and m.units == DriveAndSteer.UNITS_NORMALIZED for m in actions)
        # Fresh zeros keep arriving after an operator's separate Enable boundary.
        # This observes the policy side of that handshake, not MCU acceptance.
        count_at_enable = len(actions)
        spin_until(lambda: len(actions) > count_at_enable)
        spin_until(lambda: any(m.drive > 0.0 for m in actions), feed=True, start=True)
        spin_until(lambda: node.fsm_state == 2 and actions[-1].drive == 0.0)
        zero_count = len(actions)
        spin_until(lambda: len(actions) > zero_count + 2, feed=True)
        assert node.fsm_state == 2
        assert not pans
        assert not any(name.endswith("/request") for name, _ in node.get_topic_names_and_types())
    finally:
        executor.remove_node(peer)
        executor.remove_node(node)
        node.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        context.shutdown()
