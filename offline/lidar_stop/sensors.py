"""Explicit offline LiDAR perturbations at the existing acquisition queue.

The plant, ray casting, packet conversion and estimator remain unchanged.
Dropouts omit acquisitions; previously queued packets still arrive on time.
Noise affects finite hits before Cartesian conversion, never no-return beams.
"""
from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
import math

import numpy as np

from offline.mvp21_sim.simulation import MVP21SimulationEnv
from offline.planner_sim import SensorTiming


@dataclass(frozen=True)
class LidarSensorSettings:
    period_steps: int = 1
    delay_steps: int = 0
    noise_std_m: float = 0.
    noise_seed: int = 0
    # Simulation elapsed time, INCLUDING stationary sensor warmup.
    dropout_windows_s: tuple = ()

    def __post_init__(self):
        for name, minimum in (('period_steps', 1), ('delay_steps', 0)):
            value = getattr(self, name)
            if type(value) is not int or not minimum <= value <= 512:
                raise ValueError(name+' must be an integer in ['+str(minimum)+',512]')
        if (isinstance(self.noise_std_m, bool) or not isinstance(self.noise_std_m, (int, float))
                or not math.isfinite(self.noise_std_m) or self.noise_std_m < 0):
            raise ValueError('Finite nonnegative LiDAR noise required')
        if type(self.noise_seed) is not int or not 0 <= self.noise_seed < 2**32:
            raise ValueError('LiDAR noise seed must be an unsigned 32-bit integer')
        windows = []
        for window in self.dropout_windows_s:
            if not isinstance(window, (list, tuple)) or len(window) != 2:
                raise ValueError('Dropout windows require [start, end] pairs')
            start, end = window
            if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                    for v in window) or not 0 <= start < end
                    or (windows and start < windows[-1][1])):
                raise ValueError('Dropout windows must be finite, ordered and nonoverlapping')
            windows.append((float(start), float(end)))
        object.__setattr__(self, 'dropout_windows_s', tuple(windows))

    def drops(self, step, dt_s):
        elapsed = step*dt_s
        return any(start-1e-9 <= elapsed < end-1e-9 for start, end in self.dropout_windows_s)


class _AcquisitionQueue(deque):
    """Filter only newly acquired packets; preserve the parent's delivery loop."""
    def __init__(self, settings, dt_s):
        super().__init__()
        self.settings, self.dt_s = settings, dt_s
        self.dropped_steps = []

    def append(self, packet):
        if self.settings.drops(packet[1], self.dt_s):
            self.dropped_steps.append(packet[1])
        else:
            super().append(packet)

    def __deepcopy__(self, memo):
        clone = type(self)(self.settings, self.dt_s)
        memo[id(self)] = clone
        for packet in self:
            deque.append(clone, deepcopy(packet, memo))
        clone.dropped_steps = deepcopy(self.dropped_steps, memo)
        return clone


class LidarExperimentEnv(MVP21SimulationEnv):
    def __init__(self, *args, lidar_sensor_config=None, sensor_timing=None, **kwargs):
        self.lidar_experiment = LidarSensorSettings(**(lidar_sensor_config or {}))
        timing = sensor_timing or SensorTiming()
        cfg = self.lidar_experiment
        timing = SensorTiming(period_steps={**timing.period_steps, 'lidar': cfg.period_steps},
            delay_steps={**timing.delay_steps, 'lidar': cfg.delay_steps}, timeout_s=timing.timeout_s)
        super().__init__(*args, sensor_timing=timing, **kwargs)

    def reset(self, *, seed=0):
        self._lidar_rng = np.random.default_rng(self.lidar_experiment.noise_seed)
        self._noisy_acquisitions = 0
        return super().reset(seed=seed)

    def _perceive(self, native, *, initial=False):
        cfg = self.lidar_experiment
        if initial:
            # Parent reset has just made its stream queues. Install this one
            # before the first packet, without copying the sensor pipeline.
            self._pending['lidar'] = _AcquisitionQueue(cfg, self.dt_s)
        if (cfg.noise_std_m and self._step_count % cfg.period_steps == 0
                and not cfg.drops(self._step_count, self.dt_s)
                and 'obstacle_lidar_ranges_m' in native):
            native = deepcopy(native)
            lidar = self._observation_config['obstacle_lidar']
            acquisition = lidar['acquisition']
            minimum, maximum = acquisition['minimum_range_m'], acquisition['maximum_range_m']
            scale = maximum if lidar.get('range_encoding') == 'fraction_of_maximum_range' else 1.
            ranges = np.asarray(native['obstacle_lidar_ranges_m'], dtype=float)*scale
            hits = np.isfinite(ranges) & (ranges >= minimum) & (ranges < maximum)
            ranges[hits] = np.clip(ranges[hits]+self._lidar_rng.normal(0., cfg.noise_std_m, hits.sum()),
                                  minimum, np.nextafter(maximum, 0.))
            native['obstacle_lidar_ranges_m'] = ranges/scale
            self._noisy_acquisitions += 1
        return super()._perceive(native, initial=initial)

    def configuration(self):
        result = super().configuration()
        queue = self._pending.get('lidar')
        result['lidar_sensor_experiment'] = dict(settings=asdict(self.lidar_experiment),
            lidar_rate_hz=1/(self.dt_s*self.lidar_experiment.period_steps),
            delay_s=self.dt_s*self.lidar_experiment.delay_steps,
            dropped_acquisition_steps=list(getattr(queue, 'dropped_steps', [])),
            noisy_acquisitions=self._noisy_acquisitions,
            noise_model='Gaussian SI range noise, clipped finite hits; no-return unchanged')
        return result
