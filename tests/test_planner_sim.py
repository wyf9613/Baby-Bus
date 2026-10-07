"""Portable simulator contracts against the pinned notebook and Dream Gym plant."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import gymnasium as gym
import numpy as np

from offline.planner_sim import PlannerSimulationEnv, SensorTiming, load_notebook
from offline.planner_sim.notebook import NOTEBOOK


def fixed_start(config, *, progress=0.0, speed=1.0):
    config = deepcopy(config)
    pose = config.initial_state_config["pose"]
    pose["progress"].update(lower=progress, upper=progress)
    pose["lateral_offset_m"].update(lower=0.0, upper=0.0)
    config.initial_state_config["body_motion"]["longitudinal_velocity"]["value_mps"].update(
        lower=speed, upper=speed)
    return config


class NotebookConfiguration(unittest.TestCase):
    def test_physics_sensors_and_obstacles_are_extracted_without_running_cells(self):
        config = load_notebook()
        self.assertEqual(config.bicycle_model_config["mass_properties"]["mass_kg"], 3.0)
        self.assertAlmostEqual(config.bicycle_model_config["mass_properties"]["yaw_moment_of_inertia_kg_m2"],
                               0.055625)
        self.assertEqual(config.integration_config["environment_step_duration_s"], 0.05)
        self.assertEqual(len(config.road_spec["obstacles"]), 2)
        self.assertEqual(config.observation_config["obstacle_lidar"]["acquisition"]["beam_count"], 61)
        notebook = json.loads(NOTEBOOK.read_text(encoding="utf-8"))
        notebook["cells"].append({"cell_type": "code", "source": ["raise AssertionError('notebook executed')"]})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "course.ipynb"
            path.write_text(json.dumps(notebook), encoding="utf-8")
            self.assertEqual(load_notebook(path).road_spec, config.road_spec)
            notebook["cells"].append({"cell_type": "code",
                                       "source": ["CONE_NOISE_SEED = forbidden_function()"]})
            path.write_text(json.dumps(notebook), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Unsupported"):
                load_notebook(path)


class SimulationContracts(unittest.TestCase):
    def test_sensor_contract_rejects_scaling_and_multiple_yaw_epochs(self):
        configs = [load_notebook() for _ in range(3)]
        configs[0].observation_config["body_longitudinal_velocity_mps"] = {
            "destination": "observation", "scaling_factor": 2.0}
        configs[1].observation_config["cone_detections"]["positions_in_body_frame_m"]["scaling_factor"] = 2.0
        configs[2].observation_config["yaw_rate_rad_per_s"] = {
            "destination": "observation", "sample_count_per_environment_step": 2}
        for config in configs:
            with self.subTest(config=config.observation_config), self.assertRaises(ValueError):
                PlannerSimulationEnv(config)

    def test_full_dynamics_match_notebook_gym_for_acceleration_turning_and_braking(self):
        config = fixed_start(load_notebook())
        with PlannerSimulationEnv(config, max_steps=300, terminate_on_collision=False) as simulator:
            _, setup = simulator.reset(seed=7)
            native = gym.make("dreamgym/autonomous_driving_env", road_spec=config.road_spec,
                              bicycle_model_config=config.bicycle_model_config,
                              observation_config=setup["configuration"]["observation_config"],
                              initial_state_config=config.initial_state_config,
                              integration_config=config.integration_config,
                              termination_config=config.termination_config,
                              observation_format="dict", render_mode=None, disable_env_checker=True)
            try:
                native.reset(seed=7)
                actions = [(0.2, 0.1)]*15 + [(-0.4, -0.1)]*25 + [(0.0, 0.0)]*3 + [(-0.2, 0.0)]*4
                for action in actions:
                    _, _, _, _, info = simulator.step(action)
                    native.step(np.array(action, dtype=np.float32))
                    actual, expected = info["evaluation"]["vehicle_state"], native.unwrapped.car.state
                    for group in ("world_pose", "body_motion"):
                        np.testing.assert_allclose(list(actual[group].values()),
                                                   list(expected[group].values()), rtol=0, atol=1e-12)
                    self.assertEqual(actual["front_wheel_steering_angle_rad"],
                                     expected["front_wheel_steering_angle_rad"])
            finally:
                native.close()

    def test_perception_maps_colours_and_no_return_without_privileged_road(self):
        with PlannerSimulationEnv(fixed_start(load_notebook())) as env:
            observation, info = env.reset(seed=0)
            sensors = observation["sensors"]
            self.assertEqual({cone[3] for cone in sensors["cone_detections"]["detections"]}, {1, 2})
            self.assertTrue(all(cone[2] == 0.0 and cone[4] == 1.0
                                for cone in sensors["cone_detections"]["detections"]))
            self.assertEqual(len(sensors["lidar_scan"]["ranges"]), 61)
            self.assertTrue(all(np.isinf(v) for v in sensors["lidar_scan"]["ranges"]))
            self.assertEqual(sensors["lidar_cartesian"]["points_xyz"], [])
            self.assertTrue(observation["sensor_available"]["lidar_cartesian"])
            self.assertTrue(observation["estimation"]["state"]["valid"])
            self.assertTrue(observation["estimation"]["road"]["valid"])
            self.assertNotIn("ground_truth", observation)
            self.assertIn("ground_truth", info["evaluation"])
            self.assertFalse(observation["estimation"]["vehicle_limits"]["valid"])

    def test_missing_lidar_is_distinct_from_a_fresh_empty_scan(self):
        with PlannerSimulationEnv(with_obstacles=False) as env:
            observation, _ = env.reset()
            self.assertIsNone(observation["sensors"]["lidar_cartesian"])
            self.assertFalse(observation["sensor_available"]["lidar_cartesian"])
            self.assertFalse(observation["estimation"]["obstacles"]["available"])

    def test_lidar_hits_and_collision_use_vehicle_footprint(self):
        config = fixed_start(load_notebook())
        obstacle = deepcopy(config.road_spec["obstacles"][1])
        obstacle["placement"].update(reference_line_progress_m=2.0, lateral_offset_m=0.0)
        config.road_spec["obstacles"] = [obstacle]
        with PlannerSimulationEnv(config) as env:
            observation, _ = env.reset()
            self.assertAlmostEqual(observation["sensors"]["lidar_scan"]["ranges"][30], 1.8, places=6)
            cartesian = observation["sensors"]["lidar_cartesian"]
            point = cartesian["points_xyz"][cartesian["scan_indices"].index(30)]
            np.testing.assert_allclose(point, [1.8, 0.0, 0.0], atol=1e-6)
            for _ in range(60):
                _, _, terminated, _, info = env.step((0.1, 0.0))
                if terminated:
                    break
            self.assertTrue(terminated)
            self.assertTrue(info["evaluation"]["collision"])
            self.assertTrue(info["termination_reasons"]["obstacle_contact"])
            self.assertLess(info["evaluation"]["vehicle_state"]["world_pose"]["x_m"], 1.8)
            with self.assertRaises(RuntimeError):
                env.step((0.0, 0.0))

    def test_delayed_cached_cones_keep_acquisition_time_and_align_geometry(self):
        timing = SensorTiming(period_steps={"cone_detections": 2}, delay_steps={"cone_detections": 1})
        with PlannerSimulationEnv(fixed_start(load_notebook()), sensor_timing=timing) as env:
            observation, _ = env.reset()
            self.assertIsNone(observation["sensors"]["cone_detections"])
            first, *_ = env.step((0.1, 0.0))
            second, *_ = env.step((0.1, 0.0))
            third, *_ = env.step((0.1, 0.0))
            self.assertEqual(first["sensor_stamp_ns"]["cone_detections"], 1_000_000_000)
            self.assertEqual(second["sensor_stamp_ns"]["cone_detections"], 1_000_000_000)
            self.assertEqual(third["sensor_stamp_ns"]["cone_detections"], 1_100_000_000)
            self.assertAlmostEqual(second["sensor_age_s"]["cone_detections"], 0.1)
            road = second["estimation"]["road"]
            self.assertTrue(road["valid"], road["status"])
            self.assertEqual(road["measurement_timestamp_s"], 1.0)
            self.assertEqual(road["timestamp_s"], second["timestamp_s"])
            self.assertTrue(road["motion_compensated"])

    def test_stale_sensor_is_withheld_at_deadline(self):
        timing = SensorTiming(period_steps={"cone_detections": 11})
        with PlannerSimulationEnv(sensor_timing=timing) as env:
            env.reset()
            for _ in range(10):
                observation, *_ = env.step((0.0, 0.0))
            self.assertEqual(observation["sensor_age_s"]["cone_detections"], 0.5)
            self.assertIsNone(observation["sensors"]["cone_detections"])
            self.assertFalse(observation["estimation"]["road"]["valid"])

    def test_noisy_snapshot_restore_and_fork_are_repeatable_and_independent(self):
        config = fixed_start(load_notebook())
        config.observation_config["fixed_noise_seed"] = None  # shared reset/noise RNG
        noise = config.observation_config["cone_detections"]["acquisition"]["position_error_in_camera_frame"]
        for axis in ("forward", "left"):
            noise[axis]["noise_standard_deviation_m"] = 0.01
        timing = SensorTiming(period_steps={"cone_detections": 2}, delay_steps={"cone_detections": 1})
        with PlannerSimulationEnv(config, sensor_timing=timing) as env:
            env.reset(seed=9)
            for _ in range(5):
                env.step((0.1, 0.05))
            checkpoint = env.snapshot()
            with env.fork() as branch:
                self.assertIsNot(branch._backend.road, env._backend.road)
                expected = [env.step((0.15, -0.1)) for _ in range(6)]
                actual = [branch.step((0.15, -0.1)) for _ in range(6)]
                np.testing.assert_equal(actual, expected)
                branch.step((0.3, 0.4))
                self.assertNotEqual(branch.elapsed_time_s, env.elapsed_time_s)
            env.restore(checkpoint)
            replay = [env.step((0.15, -0.1)) for _ in range(6)]
            np.testing.assert_equal(replay, expected)

    def test_snapshot_preserves_stopped_forward_direction_latch(self):
        with PlannerSimulationEnv(fixed_start(load_notebook(with_obstacles=False)), max_steps=300) as env:
            env.reset()
            for _ in range(50):
                _, _, _, _, info = env.step((-0.8, 0.0))
            self.assertEqual(info["evaluation"]["vehicle_state"]["body_motion"]["longitudinal_velocity_mps"], 0.0)
            checkpoint = env.snapshot()
            expected = env.step((-0.2, 0.0))
            self.assertEqual(expected[-1]["evaluation"]["vehicle_state"]["body_motion"]["longitudinal_velocity_mps"], 0.0)
            env.step((0.0, 0.0))  # release direction latch
            reverse = env.step((-0.2, 0.0))
            self.assertLess(reverse[-1]["evaluation"]["vehicle_state"]["body_motion"]["longitudinal_velocity_mps"], 0.0)
            self.assertGreater(reverse[0]["sensors"]["wheel_speed"], 0.0)
            env.restore(checkpoint)
            np.testing.assert_equal(env.step((-0.2, 0.0)), expected)

    def test_reward_hook_policy_tuple_validation_and_time_limit(self):
        with PlannerSimulationEnv(max_steps=2, reward_function=lambda e: e["progress_delta_m"]) as env:
            env.reset()
            for invalid in ((float("nan"), 0), (1.1, 0), (0, True), (0, 0, 0, None, None), (0,)):
                with self.assertRaises(ValueError):
                    env.step(invalid)
            observation, reward, terminated, truncated, info = env.step((0.1, 0.0, None, 1.0, None))
            self.assertEqual(reward, info["evaluation"]["progress_delta_m"])
            self.assertEqual(info["policy_debug"], (1.0, None))
            self.assertFalse(terminated or truncated)
            self.assertEqual(observation["elapsed_time_s"], 0.05)
            _, _, _, truncated, info = env.step((0, 0))
            self.assertTrue(truncated)
            self.assertEqual(info["truncation_reason"], "step_limit")
            with self.assertRaises(RuntimeError):
                env.step((0, 0))


if __name__ == "__main__":
    unittest.main()
