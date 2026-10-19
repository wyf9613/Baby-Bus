"""LiDAR timing/fault ports and stop priority in the actual shared runtime."""
import gc
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from offline.lidar_stop.__main__ import scenario, SIMULATION_CONTROL
from offline.lidar_stop.sensors import LidarSensorSettings
from offline.mvp21_sim.runtime import run_runtime
from offline.mvp21_sim.simulation import fixed_road_environment
from offline.mvp21_sim.validation_config import configuration
from offline.planner_sim import PlannerSimulationEnv, load_notebook


class LidarSimulationChecks(unittest.TestCase):
    def test_explicit_default_sensor_port_matches_original_observations_and_plant(self):
        case = scenario('central')
        with fixed_road_environment(case) as original, fixed_road_environment(case, lidar_sensor_config={}) as experiment:
            for _ in range(8):
                self.assertEqual(original._frame, experiment._frame)
                self.assertEqual(list(original._backend.car._state_values()),
                                 list(experiment._backend.car._state_values()))
                original.step((.01, 0.)); experiment.step((.01, 0.))

    def test_dropout_does_not_block_already_queued_delivery(self):
        with fixed_road_environment(scenario('central'), lidar_sensor_config=dict(delay_steps=2,
                dropout_windows_s=((.2, .3),))) as env:
            self.assertEqual(env._step_count, 3)
            # During the two dropped acquisitions, packets from before the
            # outage must still be delivered with their original source time.
            for source_step in (2, 3):
                env.step((0., 0.))
                self.assertEqual(env._frame['sensor_stamp_ns']['lidar_cartesian'],
                                 round((env.clock_start_s+source_step*env.dt_s)*1e9))
            self.assertEqual(env.configuration()['lidar_sensor_experiment']['dropped_acquisition_steps'], [4, 5])

    def test_noise_is_seeded_and_preserves_no_return_beams(self):
        config = dict(noise_std_m=.005, noise_seed=7)
        with fixed_road_environment(scenario('central')) as clean, \
                fixed_road_environment(scenario('central'), lidar_sensor_config=config) as noisy:
            sample = noisy._frame['sensors']['lidar_scan']['ranges']
            original = clean._frame['sensors']['lidar_scan']['ranges']
            self.assertEqual([v==float('inf') for v in sample], [v==float('inf') for v in original])
            self.assertNotEqual(sample, original)
            noisy.reset(seed=713)
            for _ in range(3): noisy.step((0., 0.))
            self.assertEqual(noisy._frame['sensors']['lidar_scan']['ranges'], sample)

    def test_sensor_queue_and_noise_rng_survive_snapshot_fork_and_restore(self):
        with fixed_road_environment(scenario('central'), lidar_sensor_config=dict(
                delay_steps=1, noise_std_m=.005, dropout_windows_s=((.3, .4),))) as env:
            checkpoint = env.snapshot()
            with env.fork() as branch:
                expected = [env.step((.01, 0.))[0] for _ in range(4)]
                actual = [branch.step((.01, 0.))[0] for _ in range(4)]
                self.assertEqual(actual, expected)
            env.restore(checkpoint)
            gc.collect()
            self.assertEqual([env.step((.01, 0.))[0] for _ in range(4)], expected)

    def test_original_environment_restore_survives_garbage_collection(self):
        with PlannerSimulationEnv(load_notebook(with_obstacles=False)) as env:
            env.reset(seed=3)
            checkpoint = env.snapshot()
            expected = env.step((.01, 0.))
            env.restore(checkpoint)
            gc.collect()
            actual = env.step((.01, 0.))
            self.assertEqual(actual[0], expected[0])
            self.assertEqual(actual[-1]['evaluation']['ground_truth']['reference_line_progress_m'],
                             expected[-1]['evaluation']['ground_truth']['reference_line_progress_m'])

    def test_missing_lidar_stops_before_planning_or_startup_priming(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'run'
            with patch('offline.mvp21_sim.runtime.RoadCenterlinePlanner.plan',
                       side_effect=AssertionError('A pending stop cannot enter planning')), \
                    patch('offline.mvp21_sim.runtime.priming_motion_history', return_value=True):
                summary = run_runtime(scenario('central'), output, detailed_trace=True,
                    validation_config=configuration(dict(max_steps=20)),
                    simulation_control_overrides=SIMULATION_CONTROL,
                    obstacle_stop_config=dict(enabled=True), simulation_brake_drive=-.1,
                    lidar_sensor_config=dict(dropout_windows_s=((0., 1.),)))
            self.assertEqual(summary['fault'], 'control:lidar_unavailable')
            self.assertEqual(summary['steps'], 0)
            self.assertEqual(summary['delivery']['submitted'], 0)
            self.assertGreaterEqual(summary['obstacle_stop']['stopped_hold_s']+1e-9, 1.)
            failure = json.loads((output/'failure.json').read_text())
            rows = [json.loads(line) for line in (output/'trace.jsonl').read_text().splitlines()]
            self.assertEqual(failure['zero_publication_simulated_cycles'],
                             sum(r['raw_action']==[0., 0.] for r in rows))

    def test_moving_fault_stop_has_separate_brake_and_zero_counters(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'run'
            summary = run_runtime(scenario('central'), output, initial_speed=.2, detailed_trace=True,
                validation_config=configuration(dict(max_steps=20)),
                simulation_control_overrides=SIMULATION_CONTROL,
                obstacle_stop_config=dict(enabled=True), simulation_brake_drive=-.1,
                lidar_sensor_config=dict(dropout_windows_s=((0., 1.),)))
            failure = json.loads((output/'failure.json').read_text())
            rows = [json.loads(line) for line in (output/'trace.jsonl').read_text().splitlines()]
            self.assertGreater(failure['simulation_braking_cycles'], 0)
            self.assertEqual(failure['simulation_braking_cycles']+failure['zero_publication_simulated_cycles'], len(rows))
            self.assertEqual(summary['zero_publication_tail_cycles'], failure['zero_publication_simulated_cycles'])
            self.assertEqual(summary['maximum_executed_speed_mps'], max(r['speed_mps'] for r in rows))

    def test_sensor_settings_reject_ambiguous_or_invalid_clocks(self):
        for config in (dict(period_steps=0), dict(period_steps=True), dict(delay_steps=-1),
                       dict(noise_std_m=float('nan')), dict(noise_seed=-1),
                       dict(dropout_windows_s=((1., .5),)),
                       dict(dropout_windows_s=((0., 2.), (1., 3.)))):
            with self.subTest(config=config), self.assertRaises(ValueError):
                LidarSensorSettings(**config)


if __name__ == '__main__':
    unittest.main()
