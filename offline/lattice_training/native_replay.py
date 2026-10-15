"""Recorded-road map input and compatibility CLI for the latest planner only."""
import math
import numpy as np


def map_road_input(env,observation,*,lookahead_m=2.0,state_input='measured'):
    """Explicit map oracle to isolate planner failures from camera reconstruction.

    This is ground-truth geometry from the recorded-map plant, NOT a perception
    pass. Measured wheel/gyro state and all native planner checks remain.
    """
    from copy import deepcopy
    result=deepcopy(observation);raw=env._backend.car._state_values();road=env._backend.road
    current=env._info['evaluation']['ground_truth']['reference_line_progress_m']
    length=sum(e['geometry'].get('length_m',0.) if e['geometry']['type']=='straight' else
        math.radians(e['geometry']['sweep_angle_deg'])/abs(e['geometry']['curvature_per_m']) for e in env.config.road_spec['elements'])
    stations=np.linspace(max(0.,current-.5),min(length,current+lookahead_m),max(3,math.ceil(lookahead_m/.05)+11))
    center,angles=road.convert_progression_to_coordinates(stations)
    normals=np.column_stack((-np.sin(angles),np.cos(angles)))
    bounds=[road.get_reference_lane_bounds_at_progress(float(s)) for s in stations]
    left=center+normals*np.array([b['left_bound_m'] for b in bounds])[:,None]
    right=center+normals*np.array([b['right_bound_m'] for b in bounds])[:,None]
    c,s=math.cos(raw[2]),math.sin(raw[2]);rotation=np.array([[c,-s],[s,c]])
    paths=[(p-np.array(raw[:2]))@rotation for p in (center,left,right)]
    record=dict(timestamp_s=observation['timestamp_s'],frame_id='base_link',valid=True,geometry_valid=True,
        time_aligned=True,visibility='both',status='recorded_map_oracle',source_age_s=.2,
        recommended_speed_scale=1.,degraded=False,centerline_xy=paths[0].tolist(),
        left_boundary_xy=paths[1].tolist(),right_boundary_xy=paths[2].tolist(),
        lane_width_m=float(np.mean([b['left_bound_m']-b['right_bound_m'] for b in bounds])),
        local_curvature_1pm=float(road.find_closest_point_to(float(raw[0]),float(raw[1]))[-1]),
        road_input_source='recorded_map_ground_truth')
    result['estimation']['road']=record
    if state_input=='plant_truth':
        result['estimation']['state'].update(timestamp_s=observation['timestamp_s'],frame_id='base_link',
            speed_mps=float(abs(raw[3])),v_mps=float(abs(raw[3])),yaw_rate_rps=float(raw[5]),yaw_rate_radps=float(raw[5]),
            speed_valid=True,yaw_rate_valid=True,
            source_age_s={'wheel_speed':0.,'imu_angular_velocity':0.},state_input_source='plant_truth')
    return result

def main():
    import sys
    from offline.mvp21_sim.__main__ import main as latest_main
    if '--path-source' not in sys.argv:sys.argv.extend(['--path-source','lattice'])
    return latest_main()


if __name__=='__main__':raise SystemExit(main())
