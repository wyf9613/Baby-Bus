"""Headless notebook plant with Baby-Bus sensor/estimation observations."""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from importlib.metadata import version
import hashlib
import json
import math
from numbers import Real
from pathlib import Path

import gymnasium as gym
import numpy as np
import dreamgym  # Registers the pinned Gymnasium environment.

from .notebook import DREAMGYM_VERSION, load_notebook
from .perception import make_estimator, native_sensor_packets
from .snapshot import clone_runtime

_STREAMS = ("cone_detections", "wheel_speed", "imu_angular_velocity", "lidar")
_SENSORS = ("cone_detections", "wheel_speed", "imu_angular_velocity", "lidar_scan",
            "lidar_cartesian", "fiducial_detections", "imu_orientation", "imu_specific_force")


@dataclass(frozen=True)
class SensorTiming:
    """Acquisition period and delivery delay in environment steps.

    Defaults reproduce the notebook's instantaneous sample each step. A 10 Hz
    camera at dt=.05 with 50 ms latency uses period_steps={"cone_detections":2}
    and delay_steps={"cone_detections":1}. LiDAR is one instantaneous scan.
    """
    period_steps: dict = field(default_factory=dict)
    delay_steps: dict = field(default_factory=dict)
    timeout_s: float = 0.5

    def __post_init__(self):
        for name, values, minimum in (("period_steps", self.period_steps, 1),
                                     ("delay_steps", self.delay_steps, 0)):
            if not isinstance(values, dict) or set(values) - set(_STREAMS):
                raise ValueError(f"{name} must name streams in {_STREAMS}")
            if any(isinstance(v, bool) or not isinstance(v, int) or not minimum <= v <= 512
                   for v in values.values()):
                raise ValueError(f"{name} values must be integers in [{minimum},512]")
            object.__setattr__(self, name, dict(values))
        if not _finite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")


@dataclass(frozen=True)
class SimulationSnapshot:
    """Opaque in-memory checkpoint; use restore() or fork(), not car.reset()."""
    _environment: object = field(repr=False)


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, (bool, np.bool_)) and math.isfinite(value)


def _action_request(output):
    """Accept [drive,steer] or the policy's (drive,steer,pan,debug1,debug2)."""
    try:
        values = list(output)
    except TypeError as error:
        raise ValueError("Action must be a two- or five-component sequence") from error
    if len(values) not in (2, 5):
        raise ValueError("Action must contain two or five components")
    drive, steering = values[:2]
    if any(not _finite(v) or not -1 <= v <= 1 for v in (drive, steering)):
        raise ValueError("Drive and steering must be finite normalized scalars in [-1,1]")
    debug = (None, None)
    if len(values) == 5:
        if values[2] is not None:
            raise ValueError("This fixed-camera simulation supports pan hold (None) only")
        debug = tuple(values[3:])
        if any(v is not None and not _finite(v) for v in debug):
            raise ValueError("Debug values must be finite scalars or None")
    return np.asarray([drive, steering], dtype=np.float32), debug


class PlannerSimulationEnv:
    """Simulate external policy actions; no policy, planner or scorer is built in.

    reset()/step() follow the Gymnasium tuple convention. The observation is a
    rich Python record, not a Gym observation space: sensors, ages, availability
    and actual Baby-Bus estimation output. Truth is in info["evaluation"] only.
    A stateless reward_function(evaluation) may be supplied later; default is
    the notebook's zero reward. No ROS or hardware services are imported.
    """
    def __init__(self, config=None, *, with_obstacles=True, sensor_timing=None,
                 estimation_settings=None, max_steps=1000, terminate_on_collision=True,
                 synthetic_cone_confidence=1.0, clock_start_s=1.0, reward_function=None):
        installed = version("dreamgym")
        if installed != DREAMGYM_VERSION:
            raise RuntimeError(f"Dream Gym {DREAMGYM_VERSION} required; found {installed}. "
                               "Install offline/planner_sim/requirements.txt in an isolated environment.")
        if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps < 1:
            raise ValueError("max_steps must be a positive integer")
        if not isinstance(terminate_on_collision, bool):
            raise ValueError("terminate_on_collision must be boolean")
        if not _finite(synthetic_cone_confidence) or not 0 <= synthetic_cone_confidence <= 1:
            raise ValueError("synthetic_cone_confidence must be in [0,1]")
        if not _finite(clock_start_s) or clock_start_s <= 0:
            raise ValueError("clock_start_s must be positive (zero ROS stamps are invalid)")
        if reward_function is not None and not callable(reward_function):
            raise ValueError("reward_function must be callable or None")
        self.config = deepcopy(config if config is not None else load_notebook(with_obstacles=with_obstacles))
        self.timing = deepcopy(sensor_timing if sensor_timing is not None else SensorTiming())
        if not isinstance(self.timing, SensorTiming):
            raise ValueError("sensor_timing must be SensorTiming")
        self.max_steps, self.terminate_on_collision = max_steps, terminate_on_collision
        self.synthetic_cone_confidence, self.clock_start_s = synthetic_cone_confidence, clock_start_s
        self.reward_function = reward_function
        self.estimation_settings = deepcopy(estimation_settings or {})
        self.dt_s = self.config.integration_config["environment_step_duration_s"]
        if not _finite(self.dt_s) or self.dt_s <= 0:
            raise ValueError("Environment dt must be positive and finite")
        self._observation_config = deepcopy(self.config.observation_config)
        # The notebook routes only cones/LiDAR. Add ideal measured telemetry,
        # keeping sensor noise/bias overrides when the caller supplies them.
        self._observation_config.setdefault("body_longitudinal_velocity_mps", {
            "destination": "observation", "scaling_factor": 1.0,
            "bias_mps": 0.0, "noise_standard_deviation_mps": 0.0})
        self._observation_config.setdefault("yaw_rate_rad_per_s", {
            "destination": "observation", "scaling_factor": 1.0,
            "bias_rad_per_s": 0.0, "noise_standard_deviation_rad_per_s": 0.0,
            "sample_count_per_environment_step": 1})
        for name in ("body_longitudinal_velocity_mps", "yaw_rate_rad_per_s"):
            record = self._observation_config[name]
            if record.get("destination") != "observation" or record.get("scaling_factor", 1.0) != 1.0:
                raise ValueError(f"{name} must be observed unscaled in SI units")
        if self._observation_config["yaw_rate_rad_per_s"].get("sample_count_per_environment_step", 1) != 1:
            raise ValueError("The adapter requires one end-of-step yaw-rate sample")
        cones = self._observation_config.get("cone_detections", {})
        for name in ("positions_in_body_frame_m", "color_ids", "count"):
            if cones.get(name, {}).get("destination") != "observation":
                raise ValueError(f"cone_detections.{name} must be routed to observation")
        if cones["positions_in_body_frame_m"].get("scaling_factor", 1.0) != 1.0:
            raise ValueError("Cone positions must be unscaled in metres")
        self._backend = gym.make(
            "dreamgym/autonomous_driving_env", road_spec=self.config.road_spec,
            bicycle_model_config=self.config.bicycle_model_config,
            observation_config=self._observation_config,
            initial_state_config=self.config.initial_state_config,
            integration_config=self.config.integration_config,
            termination_config=self.config.termination_config,
            observation_format="dict", render_mode=None, disable_env_checker=True).unwrapped
        self.estimator, self.estimator_sha256 = make_estimator(self.estimation_settings)
        self._closed, self._has_reset, self._done = False, False, False
        self._step_count = 0
        self._pending = {name: deque() for name in _STREAMS}
        self._latest = {}
        self._frame, self._info = None, None

    @property
    def elapsed_time_s(self):
        return self._step_count * self.dt_s

    def configuration(self):
        """Detached resolved configuration and source hashes for experiment logs."""
        package_root = Path(dreamgym.__file__).resolve().parent
        package_hashes = {name: hashlib.sha256((package_root / "envs" / name).read_bytes()).hexdigest()
                          for name in ("autonomous_driving_env.py", "bicycle_model_dynamic.py", "road.py")}
        return deepcopy({
            "provenance": {**self.config.provenance, "estimator_sha256": self.estimator_sha256},
            "installed_dependency": {"version": version("dreamgym"),
                                     "package_path": str(package_root), "source_sha256": package_hashes},
            "road_spec": self.config.road_spec, "bicycle_model_config": self.config.bicycle_model_config,
            "observation_config": self._observation_config,
            "initial_state_config": self.config.initial_state_config,
            "integration_config": self.config.integration_config,
            "termination_config": self.config.termination_config,
            "estimation_settings": vars(self.estimator.settings),
            "adapter": {"clock_start_s": self.clock_start_s, "frame_id": "base_link",
                        "synthetic_cone_confidence": self.synthetic_cone_confidence,
                        "wheel_speed_model": "unsigned_COM_forward_velocity_proxy",
                        "period_steps": self.timing.period_steps, "delay_steps": self.timing.delay_steps,
                        "sensor_timeout_s": self.timing.timeout_s, "max_steps": self.max_steps,
                        "terminate_on_collision": self.terminate_on_collision,
                        "reward": "notebook_zero" if self.reward_function is None else "caller_supplied"}})

    def _perceive(self, native, *, initial=False):
        packets = native_sensor_packets(native, self._observation_config, self.synthetic_cone_confidence)
        for stream, payload in packets.items():
            if stream == "lidar" and payload["lidar_scan"] is None:
                continue
            if self._step_count % self.timing.period_steps.get(stream, 1) == 0:
                delay = self.timing.delay_steps.get(stream, 0)
                if stream == "cone_detections":
                    payload[stream]["acquisition_to_publish_latency_s"] = delay * self.dt_s
                self._pending[stream].append((self._step_count+delay, self._step_count, payload))
            while self._pending[stream] and self._pending[stream][0][0] <= self._step_count:
                delivery, acquisition, delivered = self._pending[stream].popleft()
                self._latest[stream] = (acquisition, delivery, delivered)
        sensors = dict.fromkeys(_SENSORS)
        ages, stamps, receipts, receipt_ros_ns = (dict.fromkeys(_SENSORS) for _ in range(4))
        for acquisition, delivery, payload in self._latest.values():
            received = self.clock_start_s + delivery*self.dt_s
            acquired_ns = round((self.clock_start_s+acquisition*self.dt_s)*1e9)
            for name, value in payload.items():
                # Wheel messages are unstamped, matching the live receipt proxy.
                age = (self._step_count-(delivery if name == "wheel_speed" else acquisition))*self.dt_s
                ages[name] = age
                stamps[name] = None if name == "wheel_speed" else acquired_ns
                receipts[name], receipt_ros_ns[name] = received, round(received*1e9)
                sensors[name] = value if age < self.timing.timeout_s else None
        now = self.clock_start_s+self.elapsed_time_s
        estimates = self.estimator.update(sensors, ages, stamps, receipts, now, receipt_ros_ns)
        return {"timestamp_s": now, "elapsed_time_s": self.elapsed_time_s,
                "dt_s": 0.0 if initial else self.dt_s, "frame_id": "base_link",
                "sensors": sensors, "sensor_age_s": ages, "sensor_stamp_ns": stamps,
                "sensor_available": {name: value is not None for name, value in sensors.items()},
                "estimation": estimates}

    def _evaluation(self, previous_truth=None, action=None):
        truth = dict(self._backend.get_current_ground_truth())
        contacts = dict(self._backend.get_obstacle_contact_state())
        collision = bool(contacts["obstacle_indices_overlapping_current_pose"]
                         or contacts["obstacle_indices_contacted_during_step"])
        delta = 0.0 if previous_truth is None else (
            truth["reference_line_progress_m"] - previous_truth["reference_line_progress_m"])
        return {"ground_truth": truth, "vehicle_state": deepcopy(self._backend.car.state),
                "contacts": contacts, "collision": collision, "progress_delta_m": delta,
                "elapsed_time_s": self.elapsed_time_s, "dt_s": self.dt_s,
                "action": None if action is None else [float(v) for v in action]}

    def reset(self, *, seed=0):
        if self._closed:
            raise RuntimeError("Environment is closed")
        native, native_info = self._backend.reset(seed=seed)
        self.estimator, self.estimator_sha256 = make_estimator(self.estimation_settings)
        self._pending = {name: deque() for name in _STREAMS}
        self._latest, self._step_count, self._done = {}, 0, False
        self._has_reset = True
        self._frame = self._perceive(native, initial=True)
        config = self.configuration()
        self._info = {"evaluation": self._evaluation(), "native_info": native_info,
                      "configuration": config, "seed": seed,
                      "configuration_sha256": hashlib.sha256(
                          json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()}
        return deepcopy(self._frame), deepcopy(self._info)

    def step(self, policy_output):
        if self._closed or not self._has_reset or self._done:
            raise RuntimeError("Call reset() before stepping a new or finished environment")
        action, debug = _action_request(policy_output)
        previous = self._backend.get_current_ground_truth()
        native, native_reward, terminated, truncated, native_info = self._backend.step(action)
        self._step_count += 1
        self._frame = self._perceive(native)
        evaluation = self._evaluation(previous, action)
        reasons = dict(native_info.get("termination_reasons", {}))
        if self.terminate_on_collision and evaluation["collision"]:
            terminated, reasons["obstacle_contact"] = True, True
        truncated = bool(truncated or (self._step_count >= self.max_steps and not terminated))
        evaluation.update(terminated=bool(terminated), truncated=truncated, termination_reasons=reasons)
        self._done = bool(terminated or truncated)
        self._info = {"evaluation": evaluation, "native_info": native_info,
                      "termination_reasons": reasons,
                      "truncation_reason": "step_limit" if truncated else None,
                      "policy_debug": debug}
        reward = native_reward if self.reward_function is None else self.reward_function(deepcopy(evaluation))
        if not _finite(reward):
            raise ValueError("Reward function must return a finite scalar")
        return deepcopy(self._frame), float(reward), bool(terminated), truncated, deepcopy(self._info)

    def snapshot(self):
        if not self._has_reset or self._closed:
            raise RuntimeError("Reset an open environment before taking a snapshot")
        return SimulationSnapshot(clone_runtime(self))

    def restore(self, snapshot):
        if not isinstance(snapshot, SimulationSnapshot):
            raise TypeError("Expected SimulationSnapshot")
        restored = clone_runtime(snapshot._environment)
        self._backend.close()
        self.__dict__ = restored.__dict__
        return deepcopy(self._frame), deepcopy(self._info)

    def fork(self):
        """Independent branch at this exact state, including sensor/estimator history."""
        if not self._has_reset or self._closed:
            raise RuntimeError("Reset an open environment before forking")
        return clone_runtime(self)

    def close(self):
        if not self._closed:
            self._backend.close()
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
