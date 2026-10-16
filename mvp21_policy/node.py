"""MVP2.1 node; synced from upstream 3652e9b into the modular implementation."""

from numbers import Real
from copy import deepcopy
import json
import math
import signal
import time
import traceback
from .contracts import (
    required_sensor_problem,
    FIDUCIAL_DICTIONARY_SIZES,
    FSM_STATE_NOT_PUBLISHING_ACTIONS,
    FSM_STATE_PUBLISHING_POLICY_ACTION,
    FSM_STATE_PUBLISHING_ZERO_ACTIONS,
    Observation,
    SENSORS,
    STAMPED_SENSORS,
    STATE_NAMES,
)
from .geometry import (
    finite_number,
    quaternion_inverse,
    quaternion_product,
    quaternion_xyzw,
    roll_pitch_yaw,
    rotate_vector,
    transform_position,
    valid_frame,
    wrap_angle,
)
from .motion import (
    EstimationMotionHistory,
)
from .estimation import (
    EstimationPipeline,
    EstimationSettings,
)
from .planning import (
    CenterlinePlanner,
    PlanningSettings,
    RoadCenterlinePlanner,
    evaluate_planning_path,
)
from .control import (
    ControlReferenceManager,
    ControlSettings,
    PolicyController,
    PolicyStopRequest,
    RunDistanceLimiter,
    VehicleCalibrationSettings,
)
from .lattice import (
    ShadowSettings,
    _shadow_at,
    _shadow_lengths,
)
from .transport import (
    ShadowJobSource,
    ShadowWorker,
)
from .execution import PlannerControlSettings, PlannerExecutionManager, calculate_selected_control, priming_motion_history
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Int8, String, UInt16
from dream_interfaces.msg import (ConeDetection, ConeDetections, DriveAndSteer,
                                  FiducialDetections)
from tf2_ros import Buffer, TransformException, TransformListener


class PolicyNode(ShadowJobSource, Node):
    def __init__(self, **kwargs):
        super().__init__("ai4r_policy", **kwargs)

        # All supplied policy settings are startup-only. Restart this node after
        # editing its YAML; it always restarts in the zero-action state.
        defaults = {
            "policy_update_mode": "cone_detection",
            "policy_update_rate_hz": 50.0,
            "required_sensors": ["cone_detections"],
            "cone_empty_timeout_s": 1.0,
            "supervision_rate_hz": 20.0,
            "status_period_s": 0.5,
            "timestamp_tolerance_s": 0.05,
            "policy_frame_id": "base_link",
        }
        defaults.update({f"sensor_timeout_s.{name}": 0.5 for name in SENSORS})
        for name, default in defaults.items():
            # YAML cannot infer a string-array type from []. Allow that startup
            # representation, then check its contents explicitly below.
            self.declare_parameter(name, default, ParameterDescriptor(
                read_only=True, dynamic_typing=(name == "required_sensors")))

        self.policy_update_mode = self.get_parameter("policy_update_mode").value
        self.policy_update_rate_hz = self.get_parameter("policy_update_rate_hz").value
        required_parameter = self.get_parameter("required_sensors")
        # Jazzy's C YAML parser represents an explicit [] as NOT_SET, while a
        # Python parameter override can carry a typed empty string array. An
        # omitted key still uses our declared default ["cone_detections"].
        self.required_sensors = ([] if required_parameter.type_ == Parameter.Type.NOT_SET
                                 else required_parameter.value)
        self.cone_empty_timeout_s = self.get_parameter("cone_empty_timeout_s").value
        self.supervision_rate_hz = self.get_parameter("supervision_rate_hz").value
        self.status_period_s = self.get_parameter("status_period_s").value
        self.timestamp_tolerance_s = self.get_parameter("timestamp_tolerance_s").value
        self.policy_frame_id = self.get_parameter("policy_frame_id").value
        self.sensor_timeout_s = {
            name: self.get_parameter(f"sensor_timeout_s.{name}").value for name in SENSORS}
        self._validate_parameters()
        self.add_on_set_parameters_callback(self._reject_clock_change)

        # TO ADD A STUDENT PARAMETER, follow these THREE steps:
        # 1. Declare it here, for example:
        # self.declare_parameter('speed_kp', 0.2, ParameterDescriptor(read_only=True))
        # 2. Add speed_kp: 0.2 under ros__parameters in ai4r_policy.yaml.
        # 3. Read it here, then use self.speed_kp in your policy below:
        # self.speed_kp = self.get_parameter('speed_kp').value
        # YAML alone does not declare a parameter. A value such as 0.2 is a
        # floating-point number; 0 is an integer, which is a different ROS type.

        estimation_defaults = EstimationSettings()
        estimation_values = {}
        for name, default in vars(estimation_defaults).items():
            parameter_name = f"estimation.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            estimation_values[name] = self.get_parameter(parameter_name).value
        self.estimation_settings = EstimationSettings(**estimation_values)
        self.estimation_output = None
        self.motion_history = EstimationMotionHistory(self.estimation_settings)

        planning_values = {}
        for name, default in vars(PlanningSettings()).items():
            parameter_name = f"planning.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            planning_values[name] = self.get_parameter(parameter_name).value
        self.planning_settings = PlanningSettings(**planning_values)
        self.planning_output = None
        self.planning_diagnostics = None

        for prefix, settings_type in (("control", ControlSettings), ("vehicle", VehicleCalibrationSettings)):
            values = {}
            for name, default in vars(settings_type()).items():
                parameter_name = f"{prefix}.{name}"
                self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
                values[name] = self.get_parameter(parameter_name).value
            setattr(self, f"{prefix}_settings", settings_type(**values))
        shadow_values = {}
        for name, default in vars(ShadowSettings()).items():
            parameter_name = f"shadow.{name}"
            self.declare_parameter(parameter_name, default, ParameterDescriptor(read_only=True))
            shadow_values[name] = self.get_parameter(parameter_name).value
        self.shadow_settings = ShadowSettings(**shadow_values)
        self.declare_parameter("planner_control.enabled", False, ParameterDescriptor(read_only=True))
        self.planner_control_settings = PlannerControlSettings(
            enabled=self.get_parameter("planner_control.enabled").value)
        if self.planner_control_settings.enabled and (
                not self.shadow_settings.enabled or not self.control_settings.enabled
                or self.control_settings.mode != "mvp"
                or self.planning_settings.algorithm != "road_centerline"
                or self.policy_update_mode != "timer"):
            raise ValueError("Planner control requires shadow, timer updates and MVP road_centerline control")
        self.planner_execution = PlannerExecutionManager(self.policy_frame_id)
        self.planner_execution_publisher = self.create_publisher(String, "planner_execution", 1)
        self._last_execution_mode = None
        if self.control_settings.enabled:
            if (self.estimation_settings.road_geometry == "ordered_xy"
                    and self.planning_settings.algorithm != "road_centerline"):
                raise ValueError("Ordered XY geometry requires planning.algorithm=road_centerline")
            if (self.control_settings.end_straight_distance_m > 0
                    and self.planning_settings.algorithm != "road_centerline"):
                raise ValueError("End straight requires planning.algorithm=road_centerline")
            if (self.planning_settings.algorithm == "road_centerline"
                    and self.control_settings.mode != "mvp"):
                raise ValueError("Road centerline following requires control.mode=mvp")
            if not {"cone_detections", "wheel_speed", "imu_angular_velocity"}.issubset(self.required_sensors):
                raise ValueError("Integrated control requires cones, wheel_speed and imu_angular_velocity")
            simulation_planning = self.planning_settings.vehicle_limits_source == "course_simulation"
            simulation_control = self.control_settings.vehicle_params_source == "course_simulation"
            if self.control_settings.mode == "calibrated" and simulation_planning != simulation_control:
                raise ValueError("Planning and control simulation profiles must be selected together")
            if (self.policy_update_mode == "timer" and
                    1.0/self.policy_update_rate_hz >= self.planning_settings.reference_lifetime_s):
                raise ValueError("Integrated control timer period must be shorter than the planning reference lifetime")
        self.controller = PolicyController(self.control_settings, self.policy_frame_id)
        self.control_reference_manager = ControlReferenceManager(self.control_settings)
        self.last_control_tracking_mode = None
        self.distance_limiter = RunDistanceLimiter(self.control_settings.max_distance_m, self.control_settings.max_dt_s)
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False
        self._end_straight_start_distance_m = None
        self._end_straight_started_at_s = None

        self.fsm_state = FSM_STATE_PUBLISHING_ZERO_ACTIONS
        self.state_reason = "Startup: waiting for an explicit policy request"
        self.observations = {name: None for name in SENSORS}
        self.last_nonempty_cones = None
        self.heading_reference = None
        self.policy_started_at = None
        self.previous_policy_step_at = None
        self._monotonic = time.monotonic
        self._last_ros_time_ns = None
        self._last_status_at = -math.inf
        self._last_warning_at = {}

        reliable_one = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        sensor_one = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        imu_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.action_publisher = self.create_publisher(
            DriveAndSteer, "drive_and_steer_set_point_normalized", reliable_one)
        self.pan_publisher = self.create_publisher(Float32, "pan_set_point_normalized", reliable_one)
        self.state_publisher = self.create_publisher(Int8, "policy_fsm_state_value", 10)
        self.state_text_publisher = self.create_publisher(String, "policy_fsm_state_string", 10)
        self.heading_publisher = self.create_publisher(Float32, "imu_heading_angle", 10)
        self.debug1_publisher = self.create_publisher(Float32, "debug1", 10)
        self.debug2_publisher = self.create_publisher(Float32, "debug2", 10)
        self.declare_parameter("visualization.enabled", True, ParameterDescriptor(read_only=True))
        self.trajectory_enabled = self.get_parameter("visualization.enabled").value
        self.trajectory_publisher = self.create_publisher(Path, "policy_trajectory", 1)
        self._trajectory_reference = None
        self._trajectory_last_publish_s = -math.inf
        self._publish_trajectory(self.get_clock().now().nanoseconds, force=True)
        self.shadow_generation = self.shadow_job_id = 0
        self.shadow_last_submit_s = -math.inf
        self.shadow_handoff_estimate_s = self.shadow_settings.handoff_allowance_s
        self.shadow_last_result_s = None
        self.shadow_road_history = []
        self.shadow_pose = [0.0, 0.0, 0.0]
        self.shadow_pose_stamp_s = None
        self.shadow_odometry_epoch = 0
        self.shadow_previous_reference = None
        self.shadow_collection_events = []
        self.shadow_collection_reset_id = 0
        self.shadow_worker = None
        self.shadow_timer = None
        if self.shadow_settings.enabled:
            self.shadow_path_publisher = self.create_publisher(Path, "shadow_trajectory", 1)
            self.shadow_prefix_publisher = self.create_publisher(Path, "shadow_checked_prefix", 1)
            self.shadow_reference_publisher = self.create_publisher(Path, "shadow_reference", 1)
            self.shadow_diagnostics_publisher = self.create_publisher(String, "shadow_planning", 1)
            self.shadow_worker = ShadowWorker()
            self.shadow_timer = self.create_timer(1/self.supervision_rate_hz, self._shadow_poll)
            self._shadow_clear()

        self.create_subscription(ConeDetections, "cone_detections",
                                 self.cone_detection_callback, reliable_one)
        self.create_subscription(FiducialDetections, "fiducial_detections",
                                 self.fiducial_detection_callback, reliable_one)
        self.create_subscription(LaserScan, "scan", self.lidar_callback, sensor_one)
        self.create_subscription(Float32, "wheel_speed_m_per_sec",
                                 self.wheel_speed_callback, reliable_one)
        self.create_subscription(Imu, "imu/data", self.imu_callback, imu_qos)
        self.create_subscription(UInt16, "policy_fsm_transition_request",
                                 self.fsm_transition_request_callback, 10)

        # DREAM owns mounting TF, including the camera's fixed zero-pan pose
        # relative to ground-level base_link. Physical panning needs measured
        # dynamic TF; this policy only looks up transforms and does not
        # broadcast a competing tree.
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # These timers use a steady clock so a ROS clock adjustment cannot stop
        # supervision. Only timer mode creates a timer that runs student code.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.supervision_timer = self.create_timer(
            1.0 / self.supervision_rate_hz, self.supervision_callback, clock=self.steady_clock)
        self.policy_timer = None
        if self.policy_update_mode == "timer":
            self.policy_timer = self.create_timer(
                1.0 / self.policy_update_rate_hz, self.run_policy_step, clock=self.steady_clock)
        self.publish_zero_actions()
        self.publish_state()
        self.get_logger().info(
            f"Policy update source: {self.policy_update_mode}; required sensors: "
            f"{list(self.required_sensors)}; namespace: {self.get_namespace()}")

    def _validate_parameters(self):
        if self.policy_update_mode not in ("cone_detection", "fiducial_detection", "lidar", "timer"):
            raise ValueError(
                "policy_update_mode must be cone_detection, fiducial_detection, lidar, or timer")
        required = self.required_sensors
        if not isinstance(required, (list, tuple)) or not all(isinstance(s, str) for s in required):
            raise ValueError("required_sensors must be a list of sensor names")
        if len(set(required)) != len(required) or any(s not in SENSORS for s in required):
            raise ValueError(f"required_sensors must contain unique names from {SENSORS}")
        trigger = {"cone_detection": "cone_detections",
                   "fiducial_detection": "fiducial_detections"}.get(self.policy_update_mode)
        if trigger is not None and trigger not in required:
            raise ValueError(f"{self.policy_update_mode} mode requires {trigger} in required_sensors")
        if self.policy_update_mode == "lidar" and not {"lidar_scan", "lidar_cartesian"}.intersection(required):
            raise ValueError("lidar mode requires lidar_scan or lidar_cartesian in required_sensors")
        positive = {"policy_update_rate_hz": self.policy_update_rate_hz,
                    "cone_empty_timeout_s": self.cone_empty_timeout_s,
                    "supervision_rate_hz": self.supervision_rate_hz,
                    "status_period_s": self.status_period_s, **self.sensor_timeout_s}
        for name, value in positive.items():
            if not finite_number(value) or value <= 0.0 or not math.isfinite(1.0 / value):
                raise ValueError(f"{name} must be finite and positive")
        # ROS timers have nanosecond resolution: reject a period that truncates
        # to zero, rather than accidentally creating a continuously-ready timer.
        if max(self.policy_update_rate_hz, self.supervision_rate_hz) > 1e9:
            raise ValueError("timer frequencies cannot exceed nanosecond resolution")
        if not finite_number(self.timestamp_tolerance_s) or self.timestamp_tolerance_s < 0.0:
            raise ValueError("timestamp_tolerance_s must be finite and nonnegative")
        if not valid_frame(self.policy_frame_id):
            raise ValueError("policy_frame_id must be a nonempty frame without whitespace or leading /")
        if self.get_parameter("use_sim_time").value:
            raise ValueError("use_sim_time=true is unsupported for this live-robot policy")

    @staticmethod
    def _reject_clock_change(parameters):
        if any(p.name == "use_sim_time" for p in parameters):
            return SetParametersResult(successful=False, reason="Live policy clock is startup-only")
        return SetParametersResult(successful=True)

    # ---- Measurement acceptance and freshness --------------------------------
    def _times(self):
        monotonic_now = self._monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        self.motion_history.check_clock(ros_now_ns/1e9)
        if self._last_ros_time_ns is not None and ros_now_ns < self._last_ros_time_ns:
            for name in STAMPED_SENSORS:
                self.observations[name] = None
            self.last_nonempty_cones = None
            if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION
                    and any(s in STAMPED_SENSORS for s in self.required_sensors)):
                self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, "ROS clock moved backwards")
        self._last_ros_time_ns = ros_now_ns
        return monotonic_now, ros_now_ns

    @staticmethod
    def _stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    @staticmethod
    def _age(observation, monotonic_now, ros_now_ns):
        age = max(0.0, monotonic_now - observation.received_at)
        if observation.stamp_ns is not None:
            age = max(age, (ros_now_ns - observation.stamp_ns) / 1e9)
        return age

    def _store(self, name, value, stamp_ns, monotonic_now, ros_now_ns):
        if stamp_ns is not None:
            stamp_age = (ros_now_ns - stamp_ns) / 1e9
            previous = self.observations[name]
            if (stamp_ns <= 0 or stamp_age >= self.sensor_timeout_s[name]
                    or stamp_age < -self.timestamp_tolerance_s
                    or (previous is not None and stamp_ns <= previous.stamp_ns)):
                self._warn(name, f"Ignoring stale, future, repeated, or out-of-order {name} sample")
                return False
        self.observations[name] = Observation(value, monotonic_now, stamp_ns, ros_now_ns)
        # Record EVERY accepted wheel/gyro sample, including those between
        # camera triggers. Current snapshots cannot reconstruct past motion.
        if name in ("wheel_speed", "imu_angular_velocity"):
            scalar = value if name == "wheel_speed" else value[2]
            self.motion_history.observe(
                name, scalar, stamp_ns if stamp_ns is not None else monotonic_now,
                stamp_ns/1e9 if stamp_ns is not None else monotonic_now,
                stamp_ns/1e9 if stamp_ns is not None else ros_now_ns/1e9)
        return True

    def _fresh(self, name, monotonic_now, ros_now_ns):
        observation = self.observations[name]
        if name == "lidar_cartesian" and not self._fresh("lidar_scan", monotonic_now, ros_now_ns):
            return False  # Derived points cannot outlive their matching raw scan.
        return (observation is not None
                and self._age(observation, monotonic_now, ros_now_ns) < self.sensor_timeout_s[name])

    def health_problem(self, monotonic_now, ros_now_ns):
        age = None if self.last_nonempty_cones is None else self._age(self.last_nonempty_cones, monotonic_now, ros_now_ns)
        return required_sensor_problem(self.required_sensors,
            {name: self._fresh(name, monotonic_now, ros_now_ns) for name in self.required_sensors},
            age, self.cone_empty_timeout_s)

    def _warn(self, key, message):
        # One message per cause per three seconds, so a 50 Hz error does not
        # flood the terminal. State changes are reported immediately separately.
        now = self._monotonic()
        if now - self._last_warning_at.get(key, -math.inf) >= 3.0:
            self.get_logger().warning(message)
            self._last_warning_at[key] = now

    def cone_detection_callback(self, msg):
        now, ros_now = self._times()
        if msg.header.frame_id != self.policy_frame_id:
            self._warn("cone_frame", f"Cone frame must be {self.policy_frame_id}; got {msg.header.frame_id!r}")
            return
        latency = msg.acquisition_to_publish_latency_s
        if not finite_number(latency) or latency < 0.0:
            self._warn("cone_invalid", "Ignoring cone batch with invalid acquisition-to-publication latency")
            return
        # The detector already rejects bad depth. Validate the public message
        # at this boundary too, so malformed data does not refresh its watchdog.
        cones = []
        for cone in msg.detections:
            point = (cone.position.x, cone.position.y, cone.position.z)
            confidence = cone.classification_confidence
            if (not all(finite_number(v) for v in point)
                    or not finite_number(confidence) or not 0.0 <= confidence <= 1.0
                    or cone.color not in (ConeDetection.COLOR_YELLOW, ConeDetection.COLOR_BLUE)):
                self._warn("cone_invalid", "Ignoring cone batch with invalid position, colour, or confidence")
                return
            cones.append((*point, cone.color, confidence))
        value = {"detections": cones, "acquisition_to_publish_latency_s": float(latency)}
        if self._store("cone_detections", value, self._stamp_ns(msg.header), now, ros_now):
            if cones:
                self.last_nonempty_cones = self.observations["cone_detections"]
            if self.policy_update_mode == "cone_detection":
                self.run_policy_step()

    def fiducial_detection_callback(self, msg):
        now, ros_now = self._times()
        source_frame = msg.header.frame_id
        dictionary_name = msg.dictionary_name
        if not valid_frame(source_frame):
            self._warn("fiducial_frame", "Fiducial batch has no valid source frame")
            return
        if dictionary_name not in FIDUCIAL_DICTIONARY_SIZES:
            self._warn("fiducial_dictionary", "Fiducial batch has an invalid dictionary name")
            return
        dictionary_size = FIDUCIAL_DICTIONARY_SIZES[dictionary_name]

        # Validate the whole camera-frame batch before looking up one transform
        # at its acquisition time. A malformed member rejects the whole batch;
        # repeated IDs are retained because association belongs to the student.
        validated = []
        for detection in msg.detections:
            marker_id = detection.id
            marker_size = detection.marker_size_m
            pose_valid = detection.pose_valid
            corners = tuple((corner.u, corner.v) for corner in detection.corners)
            error = detection.reprojection_error_px
            if (not isinstance(marker_id, int) or isinstance(marker_id, bool)
                    or not 0 <= marker_id < dictionary_size
                    or not finite_number(marker_size) or marker_size <= 0.0
                    or len(corners) != 4
                    or not all(finite_number(value) for corner in corners for value in corner)
                    or not isinstance(pose_valid, bool)
                    or not isinstance(error, Real) or isinstance(error, bool)
                    or math.isinf(error) or (math.isfinite(error) and error < 0.0)):
                self._warn("fiducial_invalid", "Ignoring fiducial batch with invalid marker data")
                return
            position = detection.pose.position
            orientation = detection.pose.orientation
            if pose_valid:
                try:
                    camera_orientation = quaternion_xyzw(orientation)
                except ValueError as exc:
                    self._warn("fiducial_invalid", f"Ignoring fiducial batch with invalid pose: {exc}")
                    return
                supplied_norm = math.hypot(orientation.x, orientation.y, orientation.z, orientation.w)
                if (not math.isclose(supplied_norm, 1.0, rel_tol=0.0, abs_tol=1e-3)
                        or not all(finite_number(v) for v in
                                   (position.x, position.y, position.z))
                        or not math.isfinite(error)):
                    self._warn("fiducial_invalid", "Ignoring fiducial batch with invalid valid-pose data")
                    return
            else:
                camera_orientation = None
                placeholder = (position.x, position.y, position.z,
                               orientation.x, orientation.y, orientation.z, orientation.w)
                if (not all(finite_number(v) for v in placeholder)
                        or placeholder != (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)):
                    self._warn("fiducial_invalid", "Ignoring invalid-pose marker without its placeholder pose")
                    return
            validated.append((marker_id, marker_size, corners, pose_valid,
                              position, camera_orientation, float(error)))

        stamp_ns = self._stamp_ns(msg.header)
        try:
            if source_frame == self.policy_frame_id:
                translation = None
                policy_from_camera = (0.0, 0.0, 0.0, 1.0)
            else:
                # Nonblocking and exactly one lookup per batch at acquisition.
                transform = self.tf_buffer.lookup_transform(
                    self.policy_frame_id, source_frame, Time.from_msg(msg.header.stamp))
                translation = transform.transform.translation
                policy_from_camera = quaternion_xyzw(transform.transform.rotation)
                if not all(finite_number(v) for v in
                           (translation.x, translation.y, translation.z)):
                    raise ValueError("TF translation contains a nonfinite value")
        except (TransformException, ValueError) as exc:
            self._warn("fiducial_tf", f"Fiducial camera transform unavailable: {exc}")
            return

        records = []
        for marker_id, marker_size, corners, pose_valid, position, camera_orientation, error in validated:
            if pose_valid:
                if translation is None:
                    policy_position = (position.x, position.y, position.z)
                else:
                    try:
                        policy_position = transform_position(
                            translation, policy_from_camera, position)
                    except ValueError as exc:
                        # Finite inputs can still overflow during the transform.
                        # Reject the whole batch without refreshing its watchdog.
                        self._warn("fiducial_invalid",
                                   f"Ignoring fiducial batch with invalid transformed pose: {exc}")
                        return
                policy_orientation = quaternion_product(policy_from_camera, camera_orientation)
                policy_orientation = tuple(
                    value / math.hypot(*policy_orientation) for value in policy_orientation)
            else:
                policy_position = None
                policy_orientation = None
            records.append({
                "id": marker_id,
                "marker_size_m": marker_size,
                "corners": corners,
                "pose_valid": pose_valid,
                "position_xyz": policy_position,
                "orientation_xyzw": policy_orientation,
                "reprojection_error_px": error,
            })
        value = {"dictionary_name": dictionary_name, "source_frame_id": source_frame,
                 "detections": records}
        if self._store("fiducial_detections", value, stamp_ns, now, ros_now):
            if self.policy_update_mode == "fiducial_detection":
                self.run_policy_step()

    def lidar_callback(self, msg):
        now, ros_now = self._times()
        metadata = (msg.angle_min, msg.angle_max, msg.angle_increment,
                    msg.time_increment, msg.scan_time, msg.range_min, msg.range_max)
        if (not valid_frame(msg.header.frame_id) or not msg.ranges
                or not all(finite_number(v) for v in metadata)
                or msg.angle_increment <= 0.0 or msg.angle_max < msg.angle_min
                or msg.time_increment < 0.0 or msg.scan_time < 0.0
                or msg.range_min < 0.0 or msg.range_max <= msg.range_min
                or (msg.intensities and len(msg.intensities) != len(msg.ranges))
                or abs(msg.angle_min + (len(msg.ranges)-1)*msg.angle_increment - msg.angle_max)
                > max(0.001, 1.1*msg.angle_increment)):
            self._warn("lidar_invalid", "Ignoring lidar scan with invalid shape, frame, or metadata")
            return
        # Infinity/NaN and out-of-range individual rays are normal possibilities
        # in LaserScan. Preserve them for the student to filter, not as zeros.
        scan = {"ranges": list(msg.ranges), "intensities": list(msg.intensities),
                "frame_id": msg.header.frame_id, "angle_min": msg.angle_min,
                "angle_max": msg.angle_max, "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment, "scan_time": msg.scan_time,
                "range_min": msg.range_min, "range_max": msg.range_max}
        stamp_ns = self._stamp_ns(msg.header)
        if not self._store("lidar_scan", scan, stamp_ns, now, ros_now):
            return
        # One executor thread: finish both representations before any policy
        # step. A new accepted scan always invalidates the old Cartesian result;
        # failed conversion must never pair old indices with new raw ranges.
        self.observations["lidar_cartesian"] = None
        try:
            cartesian = self._convert_lidar_scan(scan, msg.header.stamp)
            self._store("lidar_cartesian", cartesian, stamp_ns, now, ros_now)
        except (TransformException, ValueError, OverflowError) as exc:
            self._warn("lidar_cartesian", f"Lidar Cartesian conversion unavailable: {exc}")
        if self.policy_update_mode == "lidar":
            self.run_policy_step()

    def _convert_lidar_scan(self, scan, stamp):
        """Convert one scan with a single nonblocking acquisition-time TF lookup."""
        offsets = (0.0, 0.0, 0.0)
        x_axis, y_axis = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
        if scan["frame_id"] != self.policy_frame_id:
            transform = self.tf_buffer.lookup_transform(
                self.policy_frame_id, scan["frame_id"], Time.from_msg(stamp))
            translation = transform.transform.translation
            x, y, z, w = quaternion_xyzw(transform.transform.rotation)
            offsets = (translation.x, translation.y, translation.z)
            if not all(finite_number(v) for v in offsets):
                raise ValueError("TF translation contains a nonfinite value")
            # The scan plane has z=0. Rotate its two basis vectors once, then
            # reuse them for every ray instead of repeating quaternion work.
            x_axis = (1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y))
            y_axis = (2*(x*y - w*z), 1 - 2*(x*x + z*z), 2*(y*z + w*x))
        points, indices = [], []
        for index, distance in enumerate(scan["ranges"]):
            if not finite_number(distance) or not scan["range_min"] <= distance <= scan["range_max"]:
                continue
            angle = scan["angle_min"] + index * scan["angle_increment"]
            local_x, local_y = distance * math.cos(angle), distance * math.sin(angle)
            point = tuple(offsets[axis] + local_x*x_axis[axis] + local_y*y_axis[axis]
                          for axis in range(3))
            if not all(math.isfinite(v) for v in point):
                raise ValueError("Cartesian lidar point is not finite")
            points.append(point)
            indices.append(index)
        return {"points_xyz": points, "scan_indices": indices, "frame_id": self.policy_frame_id}

    def wheel_speed_callback(self, msg):
        now, ros_now = self._times()
        if finite_number(msg.data) and msg.data >= 0.0:
            self._store("wheel_speed", msg.data, None, now, ros_now)
        else:
            self._warn("wheel_invalid", "Ignoring invalid unsigned wheel-speed measurement")

    def imu_callback(self, msg):
        now, ros_now = self._times()
        if not valid_frame(msg.header.frame_id):
            self._warn("imu_frame", "IMU measurement has no valid sensor frame")
            return
        try:
            if msg.header.frame_id == self.policy_frame_id:
                body_from_sensor = (0.0, 0.0, 0.0, 1.0)
            else:
                # Nonblocking: never wait for TF inside a sensor callback.
                transform = self.tf_buffer.lookup_transform(
                    self.policy_frame_id, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                body_from_sensor = quaternion_xyzw(transform.transform.rotation)
        except (TransformException, ValueError) as exc:
            self._warn("imu_tf", f"IMU mounting transform unavailable: {exc}")
            return

        stamp = self._stamp_ns(msg.header)
        # A partial IMU message may contain gyro but no orientation. Covariance
        # [0] == -1 marks an ABSENT field; its numeric zeros are not measurements.
        fields = (("imu_orientation", msg.orientation_covariance, msg.orientation),
                  ("imu_angular_velocity", msg.angular_velocity_covariance, msg.angular_velocity),
                  ("imu_specific_force", msg.linear_acceleration_covariance, msg.linear_acceleration))
        for name, covariance, measurement in fields:
            if covariance[0] == -1.0:
                continue
            try:
                if name == "imu_orientation":
                    # BNO reports q_ENU_from_sensor. We need q_ENU_from_body:
                    # q_ENU_from_sensor * inverse(q_body_from_sensor).
                    value = quaternion_product(quaternion_xyzw(measurement), quaternion_inverse(body_from_sensor))
                else:
                    value = rotate_vector(body_from_sensor, measurement)
                accepted = self._store(name, value, stamp, now, ros_now)
                if accepted and name == "imu_orientation" and self.heading_reference is not None:
                    heading = wrap_angle(roll_pitch_yaw(value)[2] - self.heading_reference)
                    self.heading_publisher.publish(Float32(data=math.degrees(heading)))
            except ValueError as exc:
                self._warn(name, f"Ignoring invalid {name}: {exc}")

    # ---- Main policy step: only the selected trigger calls this ---------------
    def run_policy_step(self):
        now, ros_now = self._times()
        if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
            return
        problem = self._control_health_problem(now, ros_now) or self._control_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return

        # Each field is a snapshot: optional stale observations become None.
        # ages/stamps remain available for understanding why a field is absent.
        values, ages, stamps = {}, {}, {}
        for name in SENSORS:
            observation = self.observations[name]
            ages[name] = None if observation is None else self._age(observation, now, ros_now)
            stamps[name] = None if observation is None else observation.stamp_ns
            values[name] = deepcopy(observation.value) if self._fresh(name, now, ros_now) else None
        first = self.previous_policy_step_at is None
        dt = 0.0 if first else now - self.previous_policy_step_at
        elapsed = now - self.policy_started_at
        self.previous_policy_step_at = now

        try:
            if self._end_straight_start_distance_m is not None:
                # Terminal mode never resumes cone following, even if cones
                # return. The independent supervisor stops it at its budget.
                result = (self.control_settings.end_straight_drive_normalized,
                          0.0, None, None, self.distance_limiter.distance_m)
            else:
                result = self.calculate_policy_actions(values, ages, stamps, dt, elapsed, first)
            drive, steer, pan, debug1, debug2 = result
            # Validate ALL outputs before publishing ANY of them.
            outputs = [drive, steer] + [v for v in (pan, debug1, debug2) if v is not None]
            if not all(finite_number(v) for v in outputs):
                raise ValueError("Policy outputs must be finite numbers; only pan/debug may be None")
            for debug in (debug1, debug2):
                if debug is not None and abs(debug) > 3.4028234663852886e38:
                    raise ValueError("Debug output does not fit a Float32")
            if any(abs(v) > 1.0 for v in (drive, steer) + (() if pan is None else (pan,))):
                self._warn("saturation", "Clipping policy action to normalized [-1, 1]")
            drive, steer = max(-1.0, min(1.0, drive)), max(-1.0, min(1.0, steer))
            pan = None if pan is None else max(-1.0, min(1.0, pan))
        except PolicyStopRequest as stop:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, str(stop))
            return
        except Exception:
            self.get_logger().error("Student policy failed:\n" + traceback.format_exc())
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, "Policy calculation or output was invalid")
            return

        # A calculation may have taken longer than a sensor's deadline. Check
        # again after it returns; never refresh stale driving actions on a timer.
        now, ros_now = self._times()
        if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
            return
        problem = self._control_health_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return
        problem = self._control_problem(now, ros_now)
        if problem:
            self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
            return
        self._publish_actions(drive, steer, pan)
        if hasattr(self, "planner_execution_publisher"):
            self._publish_planner_execution()
        # Offer AFTER the baseline action, never on its calculation/exception path.
        # This is a diagnostic side channel; failures cannot change the action/FSM.
        if getattr(self, "shadow_worker", None) is not None:
            try:
                self._shadow_offer()
            except Exception as error:
                self._warn("shadow_submit", f"Shadow submission unavailable: {error}")
        if hasattr(self, "trajectory_publisher"):
            self._publish_trajectory(ros_now)
        if debug1 is not None:
            self.debug1_publisher.publish(Float32(data=float(debug1)))
        if debug2 is not None:
            self.debug2_publisher.publish(Float32(data=float(debug2)))

    def calculate_policy_actions(self, observations, sensor_age_s, sensor_stamp_ns,
                                 dt, policy_elapsed_s, is_first_policy_step):
        """
        STUDENT STARTING POINT

        This function:
        > Calculates one action from the most recent observations.
        > Is called repeatedly at the specified frequency (hence why it is important
          that you do NOT block this function with sleep or while loops or similar).
        > Is called only when this policy_node is in the publishing-policy state and
          only while the required observations are healthy.
        > Does NOT need to publish ROS messages or manage the state machine itself
          (that is all taken care of in other functions).
        """

        # CONE DETECTOR OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        cone_batch = observations["cone_detections"]
        cone_data_available = cone_batch is not None
        cones = [] if cone_batch is None else cone_batch["detections"]
        cone_acquisition_to_publish_latency_s = (None if cone_batch is None else
                                                 cone_batch["acquisition_to_publish_latency_s"])
        cone_measurement_age_s = sensor_age_s["cone_detections"]
        num_cones = len(cones)
        x_coords = [cone[0] for cone in cones]
        y_coords = [cone[1] for cone in cones]
        z_coords = [cone[2] for cone in cones]
        cone_colour = [cone[3] for cone in cones]
        cone_confidence = [cone[4] for cone in cones]

        # FIDUCIAL MARKER OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        fiducial_data = observations["fiducial_detections"]
        fiducials_available = fiducial_data is not None
        fiducial_dictionary_name = (None if fiducial_data is None
                                    else fiducial_data["dictionary_name"])
        fiducial_source_frame_id = (None if fiducial_data is None
                                    else fiducial_data["source_frame_id"])
        fiducials = [] if fiducial_data is None else fiducial_data["detections"]

        # WHEEL SPEED OBSERVATION: EXTRACT INTO LOCAL VARIABLE
        wheel_speed_in_meters_per_second = observations["wheel_speed"]

        # 2D LIDAR SCAN OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        lidar_scan = observations["lidar_scan"]
        lidar_scan_available = lidar_scan is not None
        lidar_ranges = [] if lidar_scan is None else lidar_scan["ranges"]
        lidar_intensities = [] if lidar_scan is None else lidar_scan["intensities"]
        lidar_cartesian = observations["lidar_cartesian"]
        lidar_cartesian_available = lidar_cartesian is not None
        lidar_points_xyz = [] if lidar_cartesian is None else lidar_cartesian["points_xyz"]
        lidar_scan_indices = [] if lidar_cartesian is None else lidar_cartesian["scan_indices"]

        # IMU OBSERVATIONS: EXTRACT INTO LOCAL VARIABLES
        orientation_xyzw = observations["imu_orientation"]
        roll_angle_in_radians = pitch_angle_in_radians = heading_angle_in_radians = None
        if orientation_xyzw is not None:
            roll_angle_in_radians, pitch_angle_in_radians, yaw = roll_pitch_yaw(orientation_xyzw)
            if self.heading_reference is not None:
                heading_angle_in_radians = wrap_angle(yaw - self.heading_reference)
        angular_velocity_rad_per_sec = observations["imu_angular_velocity"]
        specific_force_m_per_sec_squared = observations["imu_specific_force"]

        # ===============================
        # EXPLANATION OF THE OBSERVATIONS
        # ===============================
        # NOTE: All lengths use meters and all angles use radians
        #
        # CONE DETECTOR OBSERVATIONS:
        # - LOCAL VARIABLE NAMES: x_coords/y_coords/z_coords and cone_colour/confidence
        #   - These are all parallel lists.
        #   - Index i describes one cone.
        #   - In the normal base_link frame, +x is forwards, +y left, +z upwards
        #     from nominal ground directly below the vehicle CG.
        #   - Hence, z_coords is the detected point's nominal height above that
        #     ground plane (it is NOT the cone's total height).
        #   - The fixed camera pose does NOT compensate for terrain or vehicle pitch.
        #   - The (x,y,z) coords are CAR coordinates (they are NOT world positions).
        #   - Use ConeDetection.COLOR_YELLOW / COLOR_BLUE (1/2).
        #   - A valid empty frame gives num_cones == 0 and cone_data_available True.
        #   - If cone detections are configured to be optional for this policy_node
        #     and no fresh cone detection message is available, then num_cones is 0
        #     and cone_data_available is False.
        # - LOCAL VARIABLE NAME: cone_acquisition_to_publish_latency_s
        #   - This is the fixed time from camera acquisition (end-of-exposure for OAK-D)
        #     to immediately before the detector called publish().
        #   - Exposure duration is excluded: this is not exposure-start-to-publication
        #     latency.
        #   - It includes device/host processing and queues, excludes downstream transport,
        #     and is None when no fresh cone batch is available.
        # - LOCAL VARIABLE NAME: cone_measurement_age_s
        #   - This is how old the acquisition is NOW, including transport and time spent
        #     waiting for this policy step.
        #   - It uses the same end-of-exposure reference for OAK-D.
        #   - Use this for measurement age; DO NOT add the publication latency again.
        #   - Age remains available for a stale batch, and is None before any batch.
        #   - Empty batches have timing too; zero cones does not mean zero latency.
        #
        # WHEEL SPEED OBSERVATION
        # - LOCAL VARIABLE NAME: wheel_speed_in_meters_per_second
        #   - This is the wheel speed estimate based on the measurement of the
        #     angular speed of the gearbox.
        #   - The vehicle's Traxxas node performs the calculations to estimate wheel
        #     speed from sparse encoder periods.
        #   - Sparse encoder periods means that low speeds take longer to measure.
        #   - Wheel speed is UNSIGNED, i.e., it does not tell you forward versus reverse.
        #   - The encoder_timeout_seconds for the vehicle's Traxxas node controls
        #     when it concludes that no encoder ticks means "wheels are stopped".
        #   - The sensor timeout here in this policy_node detects missing wheel speed
        #     telemetry.
        #   - None is not a measured zero.
        #
        # FIDUCIAL MARKER OBSERVATIONS
        # - LOCAL VARIABLE NAME: fiducials
        #   - This is a list of marker dictionaries. Each dictionary has:
        #     - id (int)
        #     - marker_size_m (float)
        #     - corners (four canonical-order (u,v) pixel pairs)
        #     - pose_valid (bool)
        #     - position_xyz (tuples in policy_frame_id; or None when pose_valid is false)
        #     - orientation_xyzw (tuples in policy_frame_id; or None when pose_valid is false)
        #     - reprojection_error_px (float; NaN means unavailable).
        # - LOCAL VARIABLE NAME: fiducial_dictionary_name
        #   - This scopes every marker ID in this batch.
        # - LOCAL VARIABLE NAME: fiducial_source_frame_id
        #   - This is the camera optical frame for corners; corners remain source-image pixels
        #     after the poses are transformed.
        # - ADDITIONAL NOTES:
        #   - Repeated IDs remain separate records in their received order.
        #   - An accepted empty batch gives fiducials == [] and fiducials_available True.
        #     It is fresh data and can trigger a step; marker absence has no automatic
        #     timeout or remembered pose.
        #   - Optional missing/stale data gives fiducials_available False.
        #
        # 2D LIDAR SCAN OBSERVATIONS
        # - LOCAL VARIABLE NAME: lidar_scan_available
        #   - This is False when the raw scan is missing/stale.
        # - LOCAL VARIABLE NAME: lidar_ranges[i]
        #   - This is the distance measured at angle_min + i * angle_increment.
        #   - It is measured in the frame: lidar_scan['frame_id'] (which is the
        #     lidar's original frame).
        #   - The raw scan and its origin are unchanged by the base_link
        #     ground-origin convention.
        #   - The lidar data is NOT rotated, it is in the frame of the lidar device.
        # - LOCAL VARIABLE NAME: lidar_intensities[i]
        #   - When supplied, this is matched to the i-th entry in lidar_ranges.
        #   - The list is empty if the lidar supplies no intensities; check before indexing.
        #   - This is the intensity of the ray when received back at the detector.
        #   - Intensity is device-specific; it is not a calibrated accuracy estimate.
        # - LOCAL VARIABLE NAME: lidar_scan
        #   - This is the dictionary with all details of the lidar scan.
        #   - Most relevant is that it contains:
        #     - angle_min / angle_max / angle_increment
        #     - time_increment
        #     - scan_time
        #     - range_min / range_max
        #   - You should retain only finite rays between range_min and range_max
        #     when your algorithm needs to detect real hits.
        #   - Infinity can mean no return; NaN is not a zero-distance obstacle.
        # - LOCAL VARIABLE NAME: lidar_cartesian_available
        #   - This is False when the conversion of the lidar scan ranges into
        #     cartesian coordinates is missing/stale.
        #   - Hence, this distinguishes unavailable conversion from a fresh scan
        #     with no usable returns (True with an empty list).
        # - LOCAL VARIABLE NAME: lidar_points_xyz
        #   - This is a list of (x, y, z) tuples in metres in policy_frame_id,
        #     which is normally body x forward, y left, z up.
        #   - Only finite returns within the scan's inclusive range limits are
        #     included. Every tuple is a usable point; there are no placeholders.
        # - LOCAL VARIABLE NAME: lidar_scan_indices[j]
        #   - This is matched to the length of lidar_points_xyz
        #   - This is the original ray index for point j, hence the original range
        #     can be extracted as: lidar_ranges[lidar_scan_indices[j]]
        # - ADDITIONAL NOTES:
        #   - Both forms are prepared once per accepted scan, before a policy
        #     update. Conversion is always enabled, including for radial-only
        #     policies. Use either representation or both in your code below.
        #   - Edit config/lidar_mount.yaml to change the mounting pose, then run
        #     dream runtime restart rplidar_c1. This affects Cartesian points;
        #     the raw scan stays in its original lidar frame.
        #   - Cartesian points always match the latest accepted raw scan. If
        #     its transform/conversion fails, raw data remains available and the
        #     old Cartesian result is discarded immediately.
        #   - Require lidar_scan and/or lidar_cartesian in required_sensors to
        #     stop driving when the data your policy uses is unavailable.
        #     Cartesian data expires at either representation's timeout.
        #   - The scan stamp is the first ray's acquisition time.
        #   - One mounting transform is used for the whole scan; points have no
        #     per-ray motion correction or IMU levelling.
        #   - The scan's timing fields are retained.
        #
        # IMU OBSERVATIONS:
        # - LOCAL VARIABLE NAME: orientation_xyzw
        #   - This is the orientation quaternion as an (x, y, z, w) tuple, or None.
        # - LOCAL VARIABLE NAMES: roll_angle_in_radians, pitch_angle_in_radians
        #   - A +x roll is forwards-axis rotation (i.e., car body tilts to the right).
        #   - A +y pitch is left-axis rotation (i.e., car nose dips down).
        # - LOCAL VARIABLE NAME: heading_angle_in_radians
        #   - This is the yaw of the car.
        #   - This is a relative heading that is re-zeroed ONLY when entering policy state.
        #   - It is wrapped to [-pi, pi].
        # - LOCAL VARIABLE NAME: angular_velocity_rad_per_sec
        #   - This is the 3-axis gyroscope measurement, i.e., angular velocity about
        #     each axis.
        # - LOCAL VARIABLE NAME: specific_force_m_per_sec_squared
        #   - This is the 3-axis accelerometer measurement, i.e., linear acceleration
        #     along each axis.
        #   - Specific force means that it INCLUDES GRAVITY; hence it is NOT pure
        #     driving acceleration.
        # - ADDITIONAL NOTES:
        #   - IMU quaternion/roll/pitch/vectors are in the policy body frame after
        #     the external mounting TF is applied.
        #   - Angles can drift.
        #   - Absolute orientation uses magnetic ENU (east/north/up).
        #   - Partial messages are normal and each IMU field may independently be None.
        #
        # OTHER RELEVANT INFORMATION:
        # - LOCAL VARIABLE NAME: sensor_age_s[name]
        #   - This is the age in seconds of the last accepted sample of that "name"
        #     sensor; or None if none exists.
        # - LOCAL VARIABLE NAME: sensor_stamp_ns[name]
        #   - This holds the respective ROS stamp in ns (None for wheel speed).
        #   - If an optional observations is expired, then its value is None.
        #
        # - LOCAL VARIABLE NAME: dt
        #   - This is the ACTUAL monotonic seconds between policy steps.
        #   - It is 0.0 on the first step.
        #   - Hence, if you use dt in your policy code, then you need to have a
        #     guard in your code to avoid division by dt.
        #   - If you operate this policy_node in timer mode, then it can reuse a
        #     sensor sample across several steps. In other words, a policy step is
        #     not necessarily a new measurement.
        #
        # - LOCAL VARIABLE NAME: policy_elapsed_s
        #   - This is the time (in seconds) since entering policy state.
        #
        # - LOCAL VARIABLE NAME: is_first_policy_step
        #   - This flag allows you reset an integrator (or similar) once per run.

        # ===============================
        # EXPLANATION OF THE ACTIONS
        # ===============================
        # ACTIONS are NORMALIZED values in [-1, 1]:
        #
        # - LOCAL VARIABLE NAME: drive_action
        #   - This requests motor effort (ESC), NOT speed in m/s.
        #
        # - LOCAL VARIABLE NAME: steering_action
        #   - This requests steering position; zero is centre.
        #   - This is NOT an angle in radians; it is a request normalized
        #     over the configured steering interval, with steering trim applied.
        #
        # - LOCAL VARIABLE NAME: camera_pan_action
        #   - This is the pan-servo target position.
        #   - This is NOT an angle in radians; it is a request normalized
        #     over the full pan-servo range available.
        #   - None means send no target and retain its current position.
        #   - It can move even while vehicle drive is disabled.
        #   - Zero means centre.
        #
        # Values outside [-1,1] are clipped. NaN/infinity/programming errors stop
        # the policy. Invalid data never becomes a motor command.

        # Initialize drive and steering to zero; None holds the camera pan position.
        drive_action = 0.0
        steering_action = 0.0
        camera_pan_action = None
        debug1 = None
        debug2 = None

        # =======================================
        # START OF: INSERT POLICY CODE BELOW HERE
        # =======================================

        # Preserve course framework inputs/actions/timing. The Word draft fills
        # internal group-record gaps only; screenshot spellings are aliases.
        # Downstream code may consume self.estimation_output["state"/"road"/
        # "obstacles"/"vehicle_params"/"vehicle_limits"]. Valid geometry is aligned
        # to state.timestamp_s; measurement_timestamp_s keeps source time. No
        # ReferenceTrajectory or target speed is generated by the estimator.
        if is_first_policy_step or not hasattr(self, "estimator"):
            self.estimator = EstimationPipeline(
                self.estimation_settings, self.policy_frame_id,
                motion_history=self.motion_history)
        sample_receipts = {name: None if obs is None else obs.received_at
                           for name, obs in self.observations.items()}
        receipt_ros_ns = {name: None if obs is None else obs.received_ros_ns
                          for name, obs in self.observations.items()}
        self.estimation_output = self.estimator.update(
            observations, sensor_age_s, sensor_stamp_ns, sample_receipts,
            self.get_clock().now().nanoseconds / 1e9, receipt_ros_ns)

        # Supply explicit car calibration at the integration boundary without
        # changing the estimator or substituting simulation for missing facts.
        calibration = getattr(self, "vehicle_settings", VehicleCalibrationSettings())
        if calibration.valid:
            params, limits = calibration.records()
            self.estimation_output["vehicle_params"] = params
            self.estimation_output["vehicle_limits"] = limits

        if is_first_policy_step or not hasattr(self, "planner"):
            planner_type = (RoadCenterlinePlanner if self.planning_settings.algorithm == "road_centerline"
                            else CenterlinePlanner)
            self.planner = planner_type(self.planning_settings, self.policy_frame_id)
        estimates = self.estimation_output
        source_timeouts = {"road": min(self.estimation_settings.max_source_age_s,
                                      self.sensor_timeout_s["cone_detections"]),
                           "wheel_speed": min(self.estimation_settings.max_source_age_s,
                                              self.estimation_settings.motion_max_gap_s,
                                              self.sensor_timeout_s["wheel_speed"]),
                           "imu_angular_velocity": min(self.estimation_settings.max_source_age_s,
                                                       self.estimation_settings.motion_max_gap_s,
                                                       self.sensor_timeout_s["imu_angular_velocity"])}
        self.planning_output, self.planning_diagnostics = self.planner.plan(
            estimates["road"], estimates["state"], estimates["obstacles"],
            estimates["vehicle_limits"], estimates["state"]["timestamp_s"], source_timeouts,
            mvp=getattr(self, "control_settings", ControlSettings()).mode == "mvp")

        control = getattr(self, "control_settings", ControlSettings())
        if control.enabled:
            if is_first_policy_step or not hasattr(self, "controller"):
                self.controller = PolicyController(control, self.policy_frame_id)
                self.control_has_run = False
            planner_requested = bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False))
            planner_reference = self._select_planner_reference(estimates) if planner_requested else None
            # Delayed camera data may predate the reset motion history at start.
            # Bounded neutral priming is allowed only BEFORE the first valid
            # control step; any fault after tracking requires explicit resume.
            if planner_reference is None and priming_motion_history(
                    self.planning_output, estimates, self.control_has_run, policy_elapsed_s, control):
                self.control_diagnostics = {"valid": False, "stop_requested": False,
                                            "reason": "priming_motion_history"}
                return 0.0, 0.0, None, None, None
            selected_reference = planner_reference if planner_reference is not None else self.planning_output
            if planner_reference is not None and hasattr(self, "control_reference_manager"):
                self.control_reference_manager.clear()
            if planner_reference is None and self._robust_control_enabled():
                try:
                    selected_reference = self._select_control_reference(estimates)
                except (ValueError, TypeError, ArithmeticError) as error:
                    raise PolicyStopRequest("Control reference: " + str(error)) from error
            if not selected_reference["valid"]:
                if (control.end_straight_distance_m > 0 and self.control_has_run
                        and control.mode == "mvp"
                        and self.planning_settings.algorithm == "road_centerline"
                        and self.planning_output["reason"] == "invalid_road_insufficient_geometry"
                        and estimates["state"]["valid"]
                        and estimates["state"]["speed_valid"]
                        and estimates["state"]["yaw_rate_valid"]):
                    # Authorized geometry-loss fallback: once per explicit
                    # run, finish straight instead of restarting on new cones.
                    self._end_straight_start_distance_m = self.distance_limiter.distance_m
                    self._end_straight_started_at_s = self._monotonic()
                    self.last_control_reference_deadline_s = None
                    self._trajectory_reference = None
                    self.state_reason = (f"End straight: {control.end_straight_distance_m:.3f} m "
                                         "after insufficient boundary geometry")
                    self.get_logger().info(self.state_reason)
                    self.publish_state()
                    return control.end_straight_drive_normalized, 0.0, None, None, self.distance_limiter.distance_m
                reason = "Planning: " + str(self.planning_output["reason"])
                self.get_logger().warning("Planning rejection details: " + str({
                    "reason": self.planning_output["reason"],
                    "road_status": estimates["road"].get("status"),
                    "road_valid": estimates["road"].get("valid"),
                    "road_visibility": estimates["road"].get("visibility"),
                    "road_source_age_s": estimates["road"].get("source_age_s"),
                    "state_valid": estimates["state"].get("valid"),
                    "speed_valid": estimates["state"].get("speed_valid"),
                    "yaw_rate_valid": estimates["state"].get("yaw_rate_valid"),
                    "planning_diagnostics": self.planning_diagnostics,
                }))
                raise PolicyStopRequest(reason)
            baseline = selected_reference if planner_reference is None else self.planning_output
            baseline_provider = (lambda: self._select_control_reference(estimates)) if self._robust_control_enabled() else None
            (self.controller, drive_action, steering_action, self.control_diagnostics,
             selected_reference) = calculate_selected_control(self.controller, baseline,
                planner_reference, estimates["state"], estimates["vehicle_params"],
                self.get_clock().now().nanoseconds/1e9, dt,
                self.planner_execution if planner_requested else None, baseline_provider)
            if not self.control_diagnostics["valid"] or self.control_diagnostics["stop_requested"]:
                reason = "Control: " + self.control_diagnostics["reason"]
                raise PolicyStopRequest(reason)
            self.control_has_run = True
            self._trajectory_reference = selected_reference
            self.last_control_reference_deadline_s = (
                selected_reference["timestamp_s"]+selected_reference["valid_for_s"])
            if self._robust_control_enabled() and selected_reference.get("planner_version") != "lattice_execution":
                self._report_control_status()
            # Existing debug topics: lateral error (m), per-run distance (m).
            # The existing state string reports planning/control stop reasons.
            debug1 = self.control_diagnostics["path_error_m"]
            debug2 = self.distance_limiter.distance_m
            # Bounded diagnostic logging: expose the actual controller inputs,
            # including forward-only targeting, instead of inferring them from
            # separately delivered debug/action topics. No extra ROS interface.
            log_now = self._monotonic()
            if log_now-getattr(self, "_last_tracking_log_at", -math.inf) >= 0.5:
                self._last_tracking_log_at = log_now
                info = self.control_diagnostics
                self.get_logger().info("Tracking: " + str({
                    "timestamp_s": selected_reference["timestamp_s"],
                    "path_error_m": info["path_error_m"],
                    "control_lateral_error_m": info.get("control_lateral_error_m", info["path_error_m"]),
                    "heading_error_rad": info["heading_error_rad"],
                    "error_basis": info.get("error_basis", "calibrated"),
                    "path_encoding": info.get("path_encoding"),
                    "closest_s_m": info.get("closest_s_m"),
                    "preview_s_m": info.get("preview_s_m"),
                    "raw_heading_error_rad": info.get("raw_heading_error_rad"),
                    "corner_local_weight": info.get("corner_local_weight"),
                    "preview_tangent_heading_rad": info.get("preview_tangent_heading_rad"),
                    "target_bearing_rad": info.get("target_bearing_rad"),
                    "preview_xy_m": [info.get("preview_x_m"), info.get("preview_y_m")],
                    "steer_requested": steering_action,
                    "steer_before_slew": info.get("steer_before_slew", steering_action),
                    "steer_slew_limited": info.get("steer_slew_limited", False),
                    "curvature_1pm": info["curvature_1pm"],
                    "speed_mps": estimates["state"]["speed_mps"],
                    "yaw_rate_rps": estimates["state"]["yaw_rate_rps"],
                    "road_status": estimates["road"].get("status"),
                    "centerline_source": self.planning_diagnostics.get("centerline_source"),
                    "inferred_single_side": self.planning_diagnostics.get("inferred_single_side"),
                    "inward_offset_m": self.planning_diagnostics.get("inward_offset_m"),
                    "lane_width_observed_at_s": self.planning_diagnostics.get("lane_width_observed_at_s"),
                    "reference_valid_for_s": selected_reference["valid_for_s"],
                }))

        # =====================================
        # END OF: INSERT POLICY CODE ABOVE HERE
        # =====================================
        return drive_action, steering_action, camera_pan_action, debug1, debug2

    # ---- Control-only reference recovery; upstream algorithms stay unchanged --
    def _select_planner_reference(self, estimates):
        return self.planner_execution.prepare(estimates, self.planning_output, self,
            self.shadow_settings, self.planning_settings, self.control_settings, self.policy_update_rate_hz)

    def _publish_planner_execution(self):
        """Record every executed control tick; transport faults cannot stop it."""
        try:
            enabled = self.planner_control_settings.enabled
            info = dict(self.planner_execution.diagnostics) if enabled else {
                "requested_source": "baseline", "mode": "baseline", "fallback": False,
                "reason": "baseline_selected"}
            ref = getattr(self, "_trajectory_reference", None) or {}
            control = self.control_diagnostics or {}
            info.update(timestamp_s=ref.get("timestamp_s"), generation=self.shadow_generation,
                reference_version=ref.get("planner_version"),
                baseline_valid=bool(self.planning_output and self.planning_output.get("valid")),
                baseline_reason=(self.planning_output or {}).get("reason"),
                speed_mps=(self.estimation_output or {}).get("state", {}).get("speed_mps"),
                preview_xy_m=[control.get("preview_x_m"), control.get("preview_y_m")],
                heading_error_rad=control.get("heading_error_rad"),
                steer_before_slew=control.get("steer_before_slew"),
                steer_after_slew=info.get("steering_action"),
                reference_valid_for_s=ref.get("valid_for_s"))
            if self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION:
                info.update(mode="idle", fallback=False, reason=self.state_reason)
            self.planner_execution_publisher.publish(String(data=json.dumps(info, allow_nan=False)))
            mode = (info["mode"], info["reason"])
            if mode != self._last_execution_mode:
                self.get_logger().info("Reference selection: "+json.dumps(info, separators=(",", ":")))
                self._last_execution_mode = mode
        except Exception as error:
            self._warn("planner_execution_diagnostics", "Execution diagnostics unavailable: "+str(error))

    def _robust_control_enabled(self):
        cfg = self.control_settings
        return cfg.enabled and cfg.mode == "mvp" and cfg.robustness_enabled

    def _control_health_problem(self, monotonic_now, ros_now_ns):
        problem = self.health_problem(monotonic_now, ros_now_ns)
        if (getattr(self, "_end_straight_start_distance_m", None) is not None
                and problem == "No recent nonempty cone detections"):
            # Fresh empty camera frames are expected outside the cone FOV.
            # health_problem already checked EVERY required source's freshness.
            return None
        if not problem or not self._robust_control_enabled():
            return problem
        # Only cone availability may be bridged. Never exempt wheel/gyro or any
        # other required field, even if the cone fault is the first in the list.
        for name in self.required_sensors:
            if name != "cone_detections" and not self._fresh(name, monotonic_now, ros_now_ns):
                return f"Required sensor {name} is missing or stale"
        manager = getattr(self, "control_reference_manager", None)
        if manager is None or manager.reference is None:
            return problem
        cache_problem = manager.problem(monotonic_now, self.distance_limiter.distance_m)
        if cache_problem:
            return cache_problem + "; explicit resume required"
        return None

    def _select_control_reference(self, estimates):
        if not hasattr(self, "control_reference_manager"):
            self.control_reference_manager = ControlReferenceManager(self.control_settings)
        state = estimates["state"]
        # Execution cadence is independent of the planner's remaining ROAD
        # lifetime. Fresh motion is checked each cycle; the original source
        # timestamps and hold budgets are kept independently in the cache.
        ages = state.get("source_age_s") or {}
        remaining = [self.sensor_timeout_s[name]-ages.get(name, math.inf)
                     for name in ("wheel_speed", "imu_angular_velocity")]
        return self.control_reference_manager.select(
            self.planning_output, estimates["road"], state, self.motion_history,
            state["timestamp_s"], self._monotonic(), self.distance_limiter.distance_m,
            min(0.2, self.control_settings.max_dt_s, *remaining))

    def _report_control_status(self):
        manager = self.control_reference_manager
        self.control_diagnostics.update(manager.diagnostics)
        if manager.reference is None:
            return
        if manager.mode != getattr(self, "last_control_tracking_mode", None):
            self.last_control_tracking_mode = manager.mode
            self.state_reason = "Control: " + str(manager.diagnostics)
            self.get_logger().info(self.state_reason)
            self.publish_state()

    # ---- State requests, watchdog, and publication ---------------------------
    def fsm_transition_request_callback(self, msg):
        requested = msg.data
        now, ros_now = self._times()
        if requested not in STATE_NAMES:
            self._warn("state_request", f"Ignoring invalid policy state request: {requested}")
            return
        if requested == self.fsm_state:
            return  # No accidental heading/integrator/time reset on a repeat.
        if requested == FSM_STATE_PUBLISHING_POLICY_ACTION:
            problem = self.health_problem(now, ros_now)
            if problem:
                self.state_reason = f"Policy start refused: {problem}"
                self.get_logger().warning(self.state_reason)
                self.publish_state()
                return
            self.heading_reference = None
            if self._fresh("imu_orientation", now, ros_now):
                self.heading_reference = roll_pitch_yaw(self.observations["imu_orientation"].value)[2]
            # Optional missing heading stays unavailable until the next real
            # policy-state entry; a later IMU sample does not silently re-tare.
            self.policy_started_at = now
            self.previous_policy_step_at = None
        self._change_state(requested, "Operator request")

    def _change_state(self, state, reason):
        self.fsm_state = state
        self.state_reason = reason
        # An internal consumer must not mistake the previous run's estimate
        # for an active result after a stop, source expiry, or explicit restart.
        self.estimation_output = None
        self.motion_history.clear()
        self.planning_output = None
        self.planning_diagnostics = None
        self.controller.clear()
        if hasattr(self, "control_reference_manager"):
            self.control_reference_manager.clear()
        self.last_control_tracking_mode = None
        self.control_diagnostics = None
        self.last_control_reference_deadline_s = None
        self.control_has_run = False
        if hasattr(self, "planner_execution"):
            self.planner_execution.clear()
            self._last_execution_mode = None
        self._end_straight_start_distance_m = None
        self._end_straight_started_at_s = None
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_generation += 1
            self.shadow_last_submit_s = -math.inf
            self.shadow_handoff_estimate_s = self.shadow_settings.handoff_allowance_s
            self.shadow_last_result_s = None
            self.shadow_road_history = []
            self.shadow_pose_stamp_s = None
            self.shadow_pose = [0.0, 0.0, 0.0]
            self.shadow_odometry_epoch = getattr(self, "shadow_odometry_epoch", 0)+1
            self.shadow_previous_reference = None
            self.shadow_collection_events = []
            self.shadow_collection_reset_id = 0
        if state == FSM_STATE_PUBLISHING_POLICY_ACTION:
            wheel = self.observations.get("wheel_speed")
            speed = wheel.value if wheel is not None else None
            self.distance_limiter.reset(self._monotonic(), speed)
        if state == FSM_STATE_PUBLISHING_ZERO_ACTIONS:
            if self.control_settings.enabled:
                self.debug2_publisher.publish(Float32(data=float(self.distance_limiter.distance_m)))
            self.publish_zero_actions()
        self.get_logger().info(f"{STATE_NAMES[state]}: {reason}")
        self.publish_state()
        self._trajectory_reference = None
        if hasattr(self, "trajectory_publisher"):
            self._publish_trajectory(self.get_clock().now().nanoseconds, force=True)
            self._trajectory_last_publish_s = -math.inf
        # Baseline state/zero publications have priority over shadow diagnostics.
        if getattr(self, "shadow_worker", None) is not None:
            self._shadow_clear()
        if hasattr(self, "planner_execution_publisher"):
            self._publish_planner_execution()

    def _shadow_clear(self):
        """Clear shadow displays on every run boundary; old results are fenced."""
        if hasattr(self, "planner_execution"):
            self.planner_execution.clear(generation=self.shadow_generation)
        try:
            stamp = self.get_clock().now().nanoseconds/1e9
            for publisher in (self.shadow_path_publisher, self.shadow_prefix_publisher,
                              self.shadow_reference_publisher):
                self._shadow_publish_path(publisher, [], stamp)
            self.shadow_diagnostics_publisher.publish(String(data=json.dumps({
                "status": "IDLE", "generation": self.shadow_generation,
                "shadow_only": not bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False))})))
        except Exception as error:
            self._warn("shadow_clear", f"Shadow display clearing unavailable: {error}")



    def _shadow_publish_path(self, publisher, points, stamp_s):
        message = Path()
        message.header.frame_id = self.policy_frame_id
        message.header.stamp = Time(nanoseconds=round(stamp_s*1e9)).to_msg()
        for point in points:
            pose = PoseStamped(header=message.header)
            pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
            heading = point[2] if len(point) > 2 else 0.0
            pose.pose.orientation.z = math.sin(heading/2)
            pose.pose.orientation.w = math.cos(heading/2)
            message.poses.append(pose)
        publisher.publish(message)

    def _shadow_poll(self):
        try:
            result = self.shadow_worker.poll()
            if result is None:
                return
            if result.get("status") == "TRANSPORT_ERROR":
                self._warn("shadow_worker", "Shadow worker failed: "+result.get("error", "unknown"))
                self._shadow_clear()
                if getattr(getattr(self, "planner_control_settings", None), "enabled", False):
                    self.planner_execution.accept(result)
                self.shadow_diagnostics_publisher.publish(String(data=json.dumps(result)))
                self.shadow_worker.close()
                self.shadow_worker = ShadowWorker()
                return
            if (self.fsm_state != FSM_STATE_PUBLISHING_POLICY_ACTION
                    or result.get("generation") != self.shadow_generation):
                return
            if hasattr(self, "planner_execution") and not self.planner_execution.accept(result):
                return
            guide = (result.get("fusion") or {}).get("guide")
            if (result.get("status") == "VALID" and guide
                    and guide.get("odometry_epoch") == getattr(self, "shadow_odometry_epoch", 0)):
                self.shadow_previous_reference = guide
            round_trip = self._monotonic()-result.get("submitted_monotonic_s", self._monotonic())
            self.shadow_handoff_estimate_s = max(self.shadow_handoff_estimate_s, round_trip)
            received = self._monotonic()
            previous = getattr(self, "shadow_last_result_s", None)
            interval = received-previous if previous is not None else None
            self.shadow_last_result_s = received
            transport = {"round_trip_s": round_trip, "completed_interval_s": interval,
                "completed_rate_hz": 1/interval if interval is not None and interval > 0 else None,
                "handoff_exceeded_estimate": round_trip > result.get("handoff_estimate_s", round_trip)}
            # Header is the INPUT frame time, never the completion/publication time.
            stamp = result.get("input_stamp_s")
            stamp = stamp if finite_number(stamp) else self.get_clock().now().nanoseconds/1e9
            selected = result.get("selected")
            safe = result.get("status") == "VALID"
            path = selected["path"] if selected and safe else []
            prefix = []
            if path:
                arc = _shadow_lengths([p[:2] for p in path])
                end = min(result["check_length_m"], arc[-1])
                prefix = [p for p, s in zip(path, arc) if s <= end]
                if end > 0:
                    p = _shadow_at([p[:2] for p in path], arc, end)
                    prefix.append((*p, prefix[-1][2]))
            self._shadow_publish_path(self.shadow_path_publisher, path, stamp)
            self._shadow_publish_path(self.shadow_prefix_publisher, prefix, stamp)
            self._shadow_publish_path(self.shadow_reference_publisher, result.get("reference", []), stamp)
            wire = result.get("wire_json") or json.dumps(result)
            # The large candidate JSON was encoded off the executor. Append only
            # small transport metadata here, instead of re-encoding all points.
            self.shadow_diagnostics_publisher.publish(String(data=wire[:-1]+',"transport":'+json.dumps(transport)+'}'))
            # Retain pending faults until a diagnostic carrying them was published.
            acknowledged = ((result.get("fusion") or {}).get("collection") or {}).get("events") or []
            if acknowledged:
                reset_id = max(event.get("id", 0) for event in acknowledged)
                self.shadow_collection_events = [event for event in getattr(self, "shadow_collection_events", [])
                                                 if event.get("id", 0) > reset_id]
            summary = {key: result.get(key) for key in ("status", "job_id", "candidate_count", "feasible_count",
                "speed_mps", "hold_time_s", "check_length_m", "score_length_m", "elapsed_s", "late_for_requested_period", "timing_s")}
            summary.update(selected_id=selected.get("id") if selected else None,
                selected_family=selected.get("family") if selected else None,
                result_age_s=self.get_clock().now().nanoseconds/1e9-stamp,
                **transport)
            if result.get("fusion"):
                summary["fusion"] = {k: result["fusion"].get(k) for k in
                    ("status", "reason", "degraded", "raw_fallback_used", "selected_source",
                     "fused_status", "raw_status", "history_frames", "prior_used", "removed_guidance_loops", "reference_crop", "collection")}
            if result.get("error"):
                summary["error"] = result["error"]
            self.get_logger().info("Shadow planning: "+json.dumps(summary, separators=(",", ":")))
        except Exception as error:
            self._warn("shadow_publish", f"Shadow diagnostics unavailable: {error}")

    def _publish_trajectory(self, ros_now_ns, force=False):
        """Display the accepted reference at 10 Hz; existing stops clear it."""
        if not self.trajectory_enabled:
            return
        now = self._monotonic()
        if not force and now-self._trajectory_last_publish_s < 0.1:
            return
        message = Path()
        message.header.frame_id = self.policy_frame_id
        message.header.stamp = Time(nanoseconds=ros_now_ns).to_msg()
        try:
            ref = self._trajectory_reference
            if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION and ref
                    and ref.get("valid") is True and ref.get("frame_id") == self.policy_frame_id
                    and ref["timestamp_s"] <= ros_now_ns/1e9 < ref["timestamp_s"]+ref["valid_for_s"]):
                path = ref["path"]
                if path.get("type") in ("CARTESIAN_CENTERLINE", "ORDERED_CENTERLINE"):
                    points = path["samples_xy"]
                else:
                    cached = ref.get("control_cached_path_samples")
                    lo, hi = (cached[0][0], cached[-1][0]) if cached else path["range"]
                    points = []
                    for i in range(81):
                        x = lo if i == 0 else hi if i == 80 else lo+(hi-lo)*i/80
                        sample = evaluate_planning_path(ref, x, ros_now_ns/1e9)
                        if sample is None:
                            points = []
                            break
                        points.append(sample["position_xy_m"])
                if (2 <= len(points) <= 201
                        and all(len(p) == 2 and all(finite_number(v) for v in p) for p in points)):
                    message.header.stamp = Time(nanoseconds=round(ref["timestamp_s"]*1e9)).to_msg()
                    for x, y in points:
                        pose = PoseStamped(header=message.header)
                        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                        pose.pose.orientation.w = 1.0
                        message.poses.append(pose)
        except Exception as error:
            message.poses = []
            self._warn("trajectory_display", f"Trajectory display unavailable: {error}")
        try:
            self.trajectory_publisher.publish(message)
            self._trajectory_last_publish_s = now
        except Exception as error:
            self._warn("trajectory_display", f"Trajectory display publication unavailable: {error}")

    def _control_problem(self, monotonic_now, ros_now_ns):
        """Distance/time budgets and reference expiry need no sensor callback."""
        if not self.control_settings.enabled:
            return None
        wheel = self.observations.get("wheel_speed")
        if wheel is None or not self._fresh("wheel_speed", monotonic_now, ros_now_ns):
            return "Wheel speed unavailable for distance limit"
        problem = self.distance_limiter.advance(wheel.value, monotonic_now)
        if problem:
            return problem
        if (self.control_settings.max_run_time_s > 0 and self.policy_started_at is not None and
                monotonic_now-self.policy_started_at >= self.control_settings.max_run_time_s):
            return "Run time limit reached; explicit resume required"
        start_distance = getattr(self, "_end_straight_start_distance_m", None)
        if start_distance is not None:
            for name in ("wheel_speed", "imu_angular_velocity"):
                observation = self.observations.get(name)
                timeout = min(self.sensor_timeout_s[name], self.estimation_settings.max_source_age_s,
                              self.estimation_settings.motion_max_gap_s)
                if (observation is None
                        or self._age(observation, monotonic_now, ros_now_ns) >= timeout):
                    return f"End straight required sensor {name} is missing or stale"
            if self.distance_limiter.distance_m-start_distance+1e-9 >= self.control_settings.end_straight_distance_m:
                return "End straight distance reached; explicit resume required"
            if monotonic_now-self._end_straight_started_at_s >= self.control_settings.end_straight_max_time_s:
                return "End straight time limit reached; explicit resume required"
            return None  # No road reference is consumed in fixed-command mode.
        deadline = self.last_control_reference_deadline_s
        manager = getattr(self, "control_reference_manager", None)
        if (self._robust_control_enabled() and manager is not None
                and manager.mode == "DEGRADED"):
            problem = manager.problem(monotonic_now, self.distance_limiter.distance_m)
            if problem:
                return problem + "; explicit resume required"
        if deadline is not None and ros_now_ns/1e9 >= deadline:
            return "Control reference expired; explicit resume required"
        if (not self.control_has_run and self.policy_started_at is not None
                and monotonic_now-self.policy_started_at >= self.control_settings.startup_grace_s):
            return "Control startup history deadline exceeded; explicit resume required"
        return None

    def supervision_callback(self):
        now, ros_now = self._times()
        if self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION:
            problem = self._control_health_problem(now, ros_now) or self._control_problem(now, ros_now)
            if problem:
                self._change_state(FSM_STATE_PUBLISHING_ZERO_ACTIONS, problem)
        elif self.fsm_state == FSM_STATE_PUBLISHING_ZERO_ACTIONS:
            self.publish_zero_actions()
        if now - self._last_status_at >= self.status_period_s:
            self.publish_state()

    def publish_state(self):
        self.state_publisher.publish(Int8(data=self.fsm_state))
        self.state_text_publisher.publish(String(data=f"{STATE_NAMES[self.fsm_state]}: {self.state_reason}"))
        self._last_status_at = self._monotonic()

    def publish_zero_actions(self):
        self._publish_actions(0.0, 0.0, None)

    def _publish_actions(self, drive, steer, pan):
        command = DriveAndSteer()
        command.units = DriveAndSteer.UNITS_NORMALIZED
        command.drive, command.steer = float(drive), float(steer)
        self.action_publisher.publish(command)
        if pan is not None:
            self.pan_publisher.publish(Float32(data=float(pan)))

    def stop_before_shutdown(self):
        # Best effort while ROS is still usable. State 1 continues to mean no
        # publication; process loss is handled by the independent vehicle node.
        if self.fsm_state != FSM_STATE_NOT_PUBLISHING_ACTIONS and self.context.ok():
            self.publish_zero_actions()
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_worker.close()

    def destroy_node(self):
        if getattr(self, "shadow_worker", None) is not None:
            self.shadow_worker.close()
        return super().destroy_node()


def main(args=None):
    # Keep the context valid until the final zero command has been published.
    # ROS's asynchronous signal handler can otherwise invalidate it mid-publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    previous_handlers = {sig: signal.signal(sig, request_stop)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PolicyNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_before_shutdown()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
