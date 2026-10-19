"""Adapt the existing simulator's sensor/plant ports; never replace its algorithms."""
from collections import deque
from copy import deepcopy
import math

import numpy as np

from offline.planner_sim import PlannerSimulationEnv, SensorTiming, load_notebook
from offline.lattice_training.native_replay import map_road_input
from .source import api, parameters, runtime_provenance, SOURCE_SHA256, UPSTREAM_COMMIT


class MVP21SimulationEnv(PlannerSimulationEnv):
    def __init__(self, *args, estimation_settings=None, **kwargs):
        self.mvp21_estimation_settings = deepcopy(estimation_settings or parameters()['estimation'])
        # The parent constructor makes a throwaway default estimator. The hook
        # below selects upstream BEFORE the first reset observation is processed.
        super().__init__(*args, estimation_settings={}, **kwargs)

    def _perceive(self, native, *, initial=False):
        if initial:
            active = api()
            settings = active.EstimationSettings(**self.mvp21_estimation_settings)
            self.estimator = active.EstimationPipeline(settings, 'base_link')
            self.estimator_sha256 = SOURCE_SHA256
        return super()._perceive(native, initial=initial)

    def configuration(self):
        result = super().configuration()
        result['mvp21_source_commit'] = UPSTREAM_COMMIT
        result['mvp21_source_sha256'] = SOURCE_SHA256
        result['mvp21_runtime_modules'] = runtime_provenance()
        return result


def recorded_environment(report, *, seed=0, progress=None, speed=0., latency_steps=4, max_steps=1220):
    """Use the checkpoint's identical road, 42 measured cones and pinned plant."""
    from dreamgym.envs.road_core.cones import ResolvedCone, ResolvedConeSet
    config = load_notebook(with_obstacles=False)
    config.road_spec = deepcopy(report['road_spec'])
    position = report['original_initial_progress_m'] if progress is None else progress
    for name, value in (('progress', position), ('lateral_offset_m', 0.), ('heading_error_rad', 0.)):
        config.initial_state_config['pose'][name].update(lower=value, upper=value)
    config.initial_state_config['body_motion']['longitudinal_velocity']['value_mps'].update(lower=speed, upper=speed)
    environment = MVP21SimulationEnv(config, max_steps=max_steps, with_obstacles=False,
        sensor_timing=SensorTiming(period_steps={'cone_detections':2},delay_steps={'cone_detections':latency_steps}))
    environment.reset(seed=seed)
    records = []
    for index, cone in enumerate(report['cones']):
        xy = (cone['x_m'],cone['y_m'])
        station = environment._backend.road.find_closest_point_to(*xy)[4]
        records.append(ResolvedCone(xy, xy, 'yellow' if cone['colour']==1 else 'blue',cone['colour']-1,.04,
            'measured','recorded',0,index,float(station),float(station),0.))
    environment._backend._resolved_cones = ResolvedConeSet(np.array([r.position_world_m for r in records]),
        np.array([r.color_id for r in records]),np.array([r.radius_m for r in records]),tuple(records))
    # Discard generated-cone packets before priming the measured map, as in log_road.
    environment._pending = {key:deque() for key in environment._pending}
    environment._latest = {}
    active = api()
    environment.estimator = active.EstimationPipeline(active.EstimationSettings(**environment.mvp21_estimation_settings),'base_link')
    for _ in range(latency_steps+2):
        environment.step((0.,0.))
    return environment


def fixed_road_environment(case, *, seed=713, progress=None, speed=0., latency_steps=1,
                           max_steps=4000, terminal_support_m=2.,physics_hz=20.,perception_hz=20.,
                           lidar_sensor_config=None):
    """20 Hz camera/telemetry port for saved road fixtures.

    Warm up only at rest. No oracle geometry or scenario width enters estimation;
    the colour mapping is the generated-cone sensor adapter convention.
    """
    from offline.lattice_training.pipeline import with_terminal_support
    config = load_notebook(with_obstacles=True)
    config.integration_config['environment_step_duration_s']=1/physics_hz
    config.road_spec = with_terminal_support(case['road_spec'], terminal_support_m)
    body = config.bicycle_model_config['body_geometry']
    position = body['rear_extent_from_center_of_mass_m']+.04 if progress is None else progress
    for name, value in (('progress', position), ('lateral_offset_m', 0.), ('heading_error_rad', 0.)):
        config.initial_state_config['pose'][name].update(lower=value, upper=value)
    config.initial_state_config['body_motion']['longitudinal_velocity']['value_mps'].update(lower=speed, upper=speed)
    estimation = parameters()['estimation']
    estimation.update(left_colour=2, right_colour=1)
    if estimation['known_lane_width_m'] != 0:
        raise ValueError('Fixed no-prior suite requires known_lane_width_m=0')
    environment_class, experiment_options = MVP21SimulationEnv, {}
    if lidar_sensor_config is not None:
        from offline.lidar_stop.sensors import LidarExperimentEnv
        environment_class = LidarExperimentEnv
        experiment_options['lidar_sensor_config'] = lidar_sensor_config
    env = environment_class(config, max_steps=max_steps, estimation_settings=estimation,
        **experiment_options,
        sensor_timing=SensorTiming(period_steps={'cone_detections': round(physics_hz/perception_hz)},
                                   delay_steps={'cone_detections': latency_steps}))
    env.collection_route_length_m = case['mission_road_length_m']
    env.collection_terminal_support_m = terminal_support_m
    env.fixed_validation_case = deepcopy(case)
    env.reset(seed=seed)
    for _ in range(max(latency_steps+2,round(physics_hz/perception_hz)+1)):
        env.step((0., 0.))
    return env


def ideal_observation(environment, observation):
    result = map_road_input(environment, observation, state_input='measured')
    road = result['estimation']['road']
    progress = environment._info['evaluation']['ground_truth']['reference_line_progress_m']
    road.update(geometry_encoding='ordered_xy',boundary_source={'left':'observed','right':'observed'},
        centerline_source='recorded_map_ground_truth',
        # Oracle geometry is sampled NOW, not from the delayed camera packet.
        measurement_timestamp_s=result['timestamp_s'],source_age_s=0.,
        local_curvature_1pm=float(environment._backend.road.convert_progression_to_curvature([progress])[0]))
    return result


def plant_action(drive, steering, *, actuator_steering_sign=-1.):
    """Only coordinate/sign conversion; preserve upstream gains and drive effort."""
    if actuator_steering_sign not in (-1.,1.) or not all(math.isfinite(v) and abs(v)<=1 for v in (drive,steering)):
        raise ValueError('Invalid normalized actuator request/sign')
    return float(drive), float(actuator_steering_sign*steering)
