# 当前 policy 的完整 raise 清单

来源：`mvp2.0` 工作树当前 `scripts/policy_node.py`；已与车端 SHA-256 核对一致。
源码 SHA-256：`1f41919536674fe5fc0200a38c4337acfb38eb194d6daae8419c36e3fa37a3b0`。
共有 **209 个 raise 位置**，同一消息可能有多个位置；这不是独立故障种类计数。

多数异常在内部捕获，转成候选淘汰或无效参考，不会直接令 ROS 进程退出。
最终停车入口为 calculate_policy_actions 中的 3 个 PolicyStopRequest；
run_policy_step 捕获它后切 state 2、持续零输出。启动参数校验则会阻止节点启动。

当前配置：lattice_v2；centerline_first_enabled=false；recovery_enabled=true；
direct_sample_output=true；obstacle_check_enabled=false；control.mode=mvp。
因此 MvpRecoveryPlanner、legacy Cartesian 拟合、障碍相关及 calibrated 专用分支
仍在源码，但当前配置不走对应执行路径。

## 所有 raise（保留原始表达式）

| 行号 | 函数 / 方法 | 异常类型 | 消息或表达式 |
| --- | --- | --- | --- |
| 116 | `quaternion_xyzw` | `ValueError` | `"quaternion contains a nonfinite value"` |
| 119 | `quaternion_xyzw` | `ValueError` | `"quaternion has no usable orientation"` |
| 140 | `rotate_vector` | `ValueError` | `"IMU vector contains a nonfinite value"` |
| 143 | `rotate_vector` | `ValueError` | `"rotated IMU vector is not finite"` |
| 151 | `transform_position` | `ValueError` | `"pose position or TF translation contains a nonfinite value"` |
| 156 | `transform_position` | `ValueError` | `"transformed pose position is not finite"` |
| 228 | `EstimationSettings.__post_init__` | `ValueError` | `"estimation.history_propagation must be camera or continuous"` |
| 230 | `EstimationSettings.__post_init__` | `ValueError` | `"estimation.road_history_mode must be time or distance"` |
| 232 | `EstimationSettings.__post_init__` | `ValueError` | `"estimation.history_merge_single_side must be bool"` |
| 234 | `EstimationSettings.__post_init__` | `ValueError` | `"estimation.continuous_road_output must be bool"` |
| 236 | `EstimationSettings.__post_init__` | `ValueError` | `"continuous road output requires continuous history propagation"` |
| 242 | `EstimationSettings.__post_init__` | `ValueError` | `f"estimation.{name} must be finite"` |
| 245 | `EstimationSettings.__post_init__` | `ValueError` | `"min_cone_confidence must be in [0,1]"` |
| 248 | `EstimationSettings.__post_init__` | `ValueError` | `"known width must be 0 (unknown) or inside width bounds"` |
| 251 | `EstimationSettings.__post_init__` | `ValueError` | `"estimation.history_merge_radius_m must be >= 0"` |
| 257 | `EstimationSettings.__post_init__` | `ValueError` | `f"road prediction hold must be in [0, {hold_limit}] seconds"` |
| 259 | `EstimationSettings.__post_init__` | `ValueError` | `f"estimation.{name} must be positive"` |
| 261 | `EstimationSettings.__post_init__` | `ValueError` | `"road width bounds must increase"` |
| 263 | `EstimationSettings.__post_init__` | `ValueError` | `"road coverage exceeds the forward range"` |
| 265 | `EstimationSettings.__post_init__` | `ValueError` | `"road spacing exceeds minimum coverage"` |
| 267 | `EstimationSettings.__post_init__` | `ValueError` | `"road sampling is limited to 201 points"` |
| 269 | `EstimationSettings.__post_init__` | `ValueError` | `"local road history is limited to 5 s and 1 m behind the car"` |
| 271 | `EstimationSettings.__post_init__` | `ValueError` | `"road_history_max_s must be in [road_history_s, 30] s"` |
| 273 | `EstimationSettings.__post_init__` | `ValueError` | `"road prediction travel is limited to 0.5 m"` |
| 276 | `EstimationSettings.__post_init__` | `ValueError` | `"motion timing limits cannot exceed the source-age limit"` |
| 1241 | `PlanningSettings.__post_init__` | `ValueError` | `"planning.algorithm must be centerline or lattice_v2"` |
| 1244 | `PlanningSettings.__post_init__` | `ValueError` | `"planning vehicle limits source must be upstream or course_simulation"` |
| 1247 | `PlanningSettings.__post_init__` | `ValueError` | `"planning compensation flag must be bool"` |
| 1249 | `PlanningSettings.__post_init__` | `ValueError` | `f"planning.{name} must be finite and positive"` |
| 1251 | `PlanningSettings.__post_init__` | `ValueError` | `"planning near range must precede far range"` |
| 1650 | `LatticeSettings.__post_init__` | `ValueError` | `"invalid lattice model source"` |
| 1654 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice sampling must have 1..9 distinct values"` |
| 1656 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice lengths, durations and speeds must be positive"` |
| 1659 | `LatticeSettings.__post_init__` | `ValueError` | `"invalid lattice boolean"` |
| 1661 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice flags must be booleans"` |
| 1663 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice numeric settings must be finite and positive: " + name` |
| 1665 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice proportions must be in [0,1]"` |
| 1667 | `LatticeSettings.__post_init__` | `ValueError` | `"invalid lattice vehicle geometry"` |
| 1669 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice body must cover both axles"` |
| 1671 | `LatticeSettings.__post_init__` | `ValueError` | `"lattice checking resolution/range exceeds bounded domain"` |
| 1673 | `LatticeSettings.__post_init__` | `ValueError` | `"recovery must stay forward and worker timeout <= 1 s"` |
| 1675 | `LatticeSettings.__post_init__` | `ValueError` | `"planning rate must not exceed 20 Hz"` |
| 1677 | `LatticeSettings.__post_init__` | `ValueError` | `"all longitudinal candidates must cover the common scoring window"` |
| 1681 | `LatticeSettings.__post_init__` | `ValueError` | `"candidate product exceeds lattice cap"` |
| 1683 | `LatticeSettings.__post_init__` | `ValueError` | `"invalid obstacle cap"` |
| 1689 | `LatticeSettings.samples` | `ValueError` | `"invalid lattice CSV: " + name` |
| 1691 | `LatticeSettings.samples` | `ValueError` | `"nonfinite lattice sample"` |
| 1703 | `_lattice_interp` | `ValueError` | `"outside_supported_range"` |
| 1725 | `LatticeCurve.__init__` | `ValueError` | `"invalid_reference_points"` |
| 1738 | `LatticeCurve.__init__.tangent` | `ValueError` | `"endpoint_tangent_singular"` |
| 1767 | `LatticeCurve.__init__` | `ValueError` | `"reference_grid_cap"` |
| 1789 | `LatticeCurve.at_x` | `ValueError` | `"curve_range"` |
| 1838 | `LatticeCurve.project_origin` | `ValueError` | `"projection_at_support_end"` |
| 1841 | `LatticeCurve.project_origin` | `ValueError` | `"nonorthogonal_projection"` |
| 1884 | `_lattice_lateral` | `ValueError` | `"lateral_solve"` |
| 1900 | `_lattice_geometry` | `ValueError` | `"frenet_degenerate"` |
| 1911 | `_lattice_steering` | `ValueError` | `"unreachable_cg_curvature"` |
| 1937 | `FrenetLatticePlanner._model` | `ValueError` | `"lattice_vehicle_limits_unavailable"` |
| 1943 | `FrenetLatticePlanner._model` | `ValueError` | `"incomplete_lattice_vehicle_limits"` |
| 2004 | `FrenetLatticePlanner.plan.budget` | `TimeoutError` | `"lattice_time_budget_exceeded"` |
| 2008 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_lattice_input"` |
| 2020 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_estimates_or_boundaries"` |
| 2023 | `FrenetLatticePlanner.plan` | `ValueError` | `"lattice_frame_or_time_mismatch"` |
| 2026 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_motion_state"` |
| 2034 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_source_timeout"` |
| 2038 | `FrenetLatticePlanner.plan` | `ValueError` | `"stale_"+name` |
| 2044 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_road_speed_advisory"` |
| 2048 | `FrenetLatticePlanner.plan` | `ValueError` | `"expired_road_prediction"` |
| 2058 | `FrenetLatticePlanner.plan` | `ValueError` | `"obstacles_unavailable_or_unaligned"` |
| 2062 | `FrenetLatticePlanner.plan` | `ValueError` | `"stale_obstacles"` |
| 2065 | `FrenetLatticePlanner.plan` | `ValueError` | `"obstacle_point_cap_or_missing"` |
| 2068 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_obstacle_point"` |
| 2080 | `FrenetLatticePlanner.plan` | `ValueError` | `"initial_speed_exceeds_model"` |
| 2084 | `FrenetLatticePlanner.plan` | `ValueError` | `"invalid_road_geometry"` |
| 2089 | `FrenetLatticePlanner.plan` | `ValueError` | `"insufficient_lattice_coverage"` |
| 2091 | `FrenetLatticePlanner.plan` | `ValueError` | `"near_gap_requires_clear_start_assumption"` |
| 2094 | `FrenetLatticePlanner.plan` | `ValueError` | `"road_sample_gap"` |
| 2105 | `FrenetLatticePlanner.plan` | `ValueError` | `"extension_limit"` |
| 2112 | `FrenetLatticePlanner.plan` | `ValueError` | `"initial_frenet_domain"` |
| 2118 | `FrenetLatticePlanner.plan` | `ValueError` | `"low_speed_yaw_inconsistent"` |
| 2134 | `FrenetLatticePlanner.plan` | `ValueError` | `"insufficient_body_supported_horizon"` |
| 2151 | `FrenetLatticePlanner.plan` | `ValueError` | `"planned_stop_requires_explicit_restart"` |
| 2189 | `FrenetLatticePlanner.plan` | `ValueError` | `"steering_limit"` |
| 2192 | `FrenetLatticePlanner.plan` | `ValueError` | `"blind_correction_limit"` |
| 2195 | `FrenetLatticePlanner.plan` | `ValueError` | `"body_boundary_or_obstacle_collision"` |
| 2197 | `FrenetLatticePlanner.plan` | `ValueError` | `"nonmonotonic_cartesian_path"` |
| 2222 | `FrenetLatticePlanner.plan` | `TimeoutError` | `"lattice_candidate_cap"` |
| 2235 | `FrenetLatticePlanner.plan` | `ValueError` | `"longitudinal_exceeds_support"` |
| 2294 | `FrenetLatticePlanner.plan` | `TimeoutError` | `"source_deadline_during_planning"` |
| 2347 | `FrenetLatticePlanner._motion` | `ValueError` | `"longitudinal_sampling_cap"` |
| 2370 | `FrenetLatticePlanner._motion` | `ValueError` | `"longitudinal_reverses_or_exceeds_support"` |
| 2381 | `FrenetLatticePlanner._motion` | `ValueError` | `"speed_limit"` |
| 2383 | `FrenetLatticePlanner._motion` | `ValueError` | `"acceleration_limit"` |
| 2385 | `FrenetLatticePlanner._motion` | `ValueError` | `"stop_trajectory_accelerates"` |
| 2387 | `FrenetLatticePlanner._motion` | `ValueError` | `"lateral_acceleration_limit"` |
| 2389 | `FrenetLatticePlanner._motion` | `ValueError` | `"steering_rate_limit"` |
| 2391 | `FrenetLatticePlanner._motion` | `ValueError` | `"visibility_stopping_limit"` |
| 2395 | `FrenetLatticePlanner._motion` | `ValueError` | `"initial_speed_mismatch"` |
| 2460 | `FrenetLatticePlanner._sampled_output` | `ValueError` | `"sampling_cap"` |
| 2471 | `FrenetLatticePlanner._sampled_output` | `ValueError` | `"short_output"` |
| 2486 | `FrenetLatticePlanner._compatibility` | `ValueError` | `"short_output"` |
| 2501 | `FrenetLatticePlanner._compatibility` | `ValueError` | `"fit_singular"` |
| 2520 | `FrenetLatticePlanner._compatibility` | `ValueError` | `"fit_error"` |
| 2522 | `FrenetLatticePlanner._compatibility` | `ValueError` | `"fit_steering_limit"` |
| 2528 | `FrenetLatticePlanner._compatibility` | `ValueError` | `"fit_motion_limit"` |
| 2530 | `FrenetLatticePlanner._compatibility` | `ValueError` | `"fit_collision"` |
| 2552 | `MvpRecoveryPlanner.plan` | `ValueError` | `"recovery_requires_cone_only_profile"` |
| 2558 | `MvpRecoveryPlanner.plan` | `ValueError` | `"invalid_recovery_road"` |
| 2560 | `MvpRecoveryPlanner.plan` | `ValueError` | `"invalid_recovery_road"` |
| 2565 | `MvpRecoveryPlanner.plan` | `ValueError` | `"recovery_frame_or_motion_invalid"` |
| 2576 | `MvpRecoveryPlanner.plan` | `ValueError` | `"source_deadline_during_planning"` |
| 2580 | `MvpRecoveryPlanner.plan` | `ValueError` | `"invalid_recovery_geometry"` |
| 2583 | `MvpRecoveryPlanner.plan` | `ValueError` | `"insufficient_lattice_coverage"` |
| 2586 | `MvpRecoveryPlanner.plan` | `ValueError` | `"invalid_recovery_speed"` |
| 2588 | `MvpRecoveryPlanner.plan` | `ValueError` | `"low_speed_yaw_inconsistent"` |
| 2591 | `MvpRecoveryPlanner.plan` | `ValueError` | `"extension_limit"` |
| 2602 | `MvpRecoveryPlanner.plan` | `ValueError` | `"recovery_fit_error"` |
| 2606 | `MvpRecoveryPlanner.plan` | `ValueError` | `"insufficient_body_supported_horizon"` |
| 2614 | `MvpRecoveryPlanner.plan` | `ValueError` | `"initial_frenet_domain"` |
| 2620 | `MvpRecoveryPlanner.plan` | `ValueError` | `"current_speed_cannot_stop_in_horizon"` |
| 2625 | `MvpRecoveryPlanner.plan` | `ValueError` | `"initial_curvature_outside_model"` |
| 2636 | `MvpRecoveryPlanner.plan` | `ValueError` | `"recovery_body_boundary_conflict"` |
| 2653 | `MvpRecoveryPlanner.plan` | `ValueError` | `"recovery_not_forward"` |
| 2657 | `MvpRecoveryPlanner.plan` | `ValueError` | `"lattice_time_budget_exceeded"` |
| 2659 | `MvpRecoveryPlanner.plan` | `ValueError` | `"recovery_rollout_cap"` |
| 2662 | `MvpRecoveryPlanner.plan` | `ValueError` | `"lattice_time_budget_exceeded" if cfg.budget_s <= lifetime                                  else "source_deadline_during_planning"` |
| 2807 | `transport_control_reference` | `ValueError` | `"reference_expired"` |
| 2810 | `transport_control_reference` | `ValueError` | `"Control delayed path motion: " + problem` |
| 2821 | `transport_control_reference` | `ValueError` | `"invalid_delayed_path"` |
| 2826 | `transport_control_reference` | `ValueError` | `"Control delayed path support exhausted"` |
| 2882 | `ControlSettings.__post_init__` | `ValueError` | `"control.enabled must be bool"` |
| 2884 | `ControlSettings.__post_init__` | `ValueError` | `"control.robustness_enabled must be bool"` |
| 2886 | `ControlSettings.__post_init__` | `ValueError` | `"control.gyro_bias_learning_enabled must be bool"` |
| 2888 | `ControlSettings.__post_init__` | `ValueError` | `"control.mode must be mvp or calibrated"` |
| 2890 | `ControlSettings.__post_init__` | `ValueError` | `"control.vehicle_params_source must be upstream or course_simulation"` |
| 2894 | `ControlSettings.__post_init__` | `ValueError` | `f"control.{name} must be finite and nonnegative"` |
| 2899 | `ControlSettings.__post_init__` | `ValueError` | `f"control.{name} must be finite and positive"` |
| 2903 | `ControlSettings.__post_init__` | `ValueError` | `"invalid control speed recovery settings"` |
| 2905 | `ControlSettings.__post_init__` | `ValueError` | `"control action limits must not exceed 1"` |
| 2907 | `ControlSettings.__post_init__` | `ValueError` | `"control.mvp_steering_direction must be -1.0 or 1.0"` |
| 2910 | `ControlSettings.__post_init__` | `ValueError` | `"control.mvp_drive_feedforward must be within [0, drive_max]"` |
| 2975 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `"vehicle calibration requires a boolean valid and a source label"` |
| 2978 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `"vehicle calibration values must be finite"` |
| 2980 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `"vehicle.steering_direction must be -1.0 or 1.0"` |
| 2982 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `"vehicle dimensions and limits must be nonnegative"` |
| 2986 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `"valid vehicle calibration requires a measured source label"` |
| 2990 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `f"vehicle.{name} must be positive when calibration is valid"` |
| 2994 | `VehicleCalibrationSettings.__post_init__` | `ValueError` | `"vehicle geometry or steering range is inconsistent"` |
| 3034 | `ControlPID.update` | `ValueError` | `"nonfinite_control_state"` |
| 3057 | `_control_poly_roots` | `ValueError` | `"nonfinite_path_geometry"` |
| 3094 | `control_path_geometry` | `ValueError` | `"unsupported_path_encoding"` |
| 3099 | `control_path_geometry` | `ValueError` | `"invalid_polynomial"` |
| 3104 | `control_path_geometry` | `ValueError` | `"invalid_path_range_or_normalization"` |
| 3120 | `control_path_geometry` | `ValueError` | `"nonfinite_path_geometry"` |
| 3132 | `control_path_geometry` | `ValueError` | `"nonfinite_path_geometry"` |
| 3145 | `_planning_control_geometry` | `ValueError` | `"invalid_lattice_control_preview"` |
| 3149 | `_planning_control_geometry` | `ValueError` | `"lattice_preview_unavailable"` |
| 3166 | `_control_sample_points` | `ValueError` | `"unsupported_path_encoding"` |
| 3175 | `_control_sample_points` | `ValueError` | `"invalid_control_cached_curve"` |
| 3181 | `_control_sample_points` | `ValueError` | `"invalid_sample_path_range"` |
| 3188 | `_control_sample_at` | `ValueError` | `"outside_supported_range"` |
| 3233 | `_control_cached_geometry` | `ValueError` | `"invalid_lattice_control_preview"` |
| 3309 | `ControlReferenceManager.select` | `ValueError` | `"Control recovery motion feedback unavailable"` |
| 3316 | `ControlReferenceManager.select` | `ValueError` | `"Invalid upstream control reference timestamp"` |
| 3320 | `ControlReferenceManager.select` | `ValueError` | `"control_frame_or_time_mismatch"` |
| 3329 | `ControlReferenceManager.select` | `ValueError` | `"Planning: " + str(reason)` |
| 3351 | `ControlReferenceManager.select` | `ValueError` | `"Invalid trusted road timestamp"` |
| 3353 | `ControlReferenceManager.select` | `ValueError` | `"Control road sample moved backwards"` |
| 3372 | `ControlReferenceManager.select` | `ValueError` | `"Control cache curve sampling limit"` |
| 3378 | `ControlReferenceManager.select` | `ValueError` | `"Control cache curve sample unavailable"` |
| 3385 | `ControlReferenceManager.select` | `ValueError` | `problem` |
| 3389 | `ControlReferenceManager.select` | `ValueError` | `"Control cache motion: " + problem` |
| 3394 | `ControlReferenceManager.select` | `ValueError` | `"Control cached path support exhausted"` |
| 3402 | `ControlReferenceManager.select` | `ValueError` | `"Control cached curve no longer y(x)"` |
| 3467 | `PolicyController.calculate` | `ValueError` | `"invalid_planning_or_speed"` |
| 3472 | `PolicyController.calculate` | `ValueError` | `"reference_expired_or_control_gap"` |
| 3477 | `PolicyController.calculate` | `ValueError` | `"control_frame_or_time_mismatch"` |
| 3482 | `PolicyController.calculate` | `ValueError` | `"invalid_speed"` |
| 3485 | `PolicyController.calculate` | `ValueError` | `"invalid_stop_request"` |
| 3491 | `PolicyController.calculate` | `ValueError` | `"simulation_reference_rejected_for_mvp"` |
| 3510 | `PolicyController.calculate` | `ValueError` | `"simulation_control_requires_simulation_reference"` |
| 3516 | `PolicyController.calculate` | `ValueError` | `"simulation_reference_rejected_for_live_control"` |
| 3520 | `PolicyController.calculate` | `ValueError` | `"simulation_geometry_rejected_for_live_control"` |
| 3524 | `PolicyController.calculate` | `ValueError` | `"vehicle_control_geometry_unavailable"` |
| 3529 | `PolicyController.calculate` | `ValueError` | `"invalid_vehicle_control_geometry"` |
| 3533 | `PolicyController.calculate` | `ValueError` | `"curvature_outside_vehicle_geometry"` |
| 3636 | `PolicyNode.__init__` | `ValueError` | `"Integrated control requires cones, wheel_speed and imu_angular_velocity"` |
| 3639 | `PolicyNode.__init__` | `ValueError` | `"Lattice obstacle checking requires lidar_cartesian"` |
| 3645 | `PolicyNode.__init__` | `ValueError` | `"Planning and control simulation profiles must be selected together"` |
| 3648 | `PolicyNode.__init__` | `ValueError` | `"Integrated control timer period must be shorter than the planning reference lifetime"` |
| 3667 | `PolicyNode.__init__` | `ValueError` | `"Async planning requires robust MVP lattice control in timer mode"` |
| 3729 | `PolicyNode._validate_parameters` | `ValueError` | `"policy_update_mode must be cone_detection, fiducial_detection, lidar, or timer"` |
| 3733 | `PolicyNode._validate_parameters` | `ValueError` | `"required_sensors must be a list of sensor names"` |
| 3735 | `PolicyNode._validate_parameters` | `ValueError` | `f"required_sensors must contain unique names from {SENSORS}"` |
| 3739 | `PolicyNode._validate_parameters` | `ValueError` | `f"{self.policy_update_mode} mode requires {trigger} in required_sensors"` |
| 3741 | `PolicyNode._validate_parameters` | `ValueError` | `"lidar mode requires lidar_scan or lidar_cartesian in required_sensors"` |
| 3748 | `PolicyNode._validate_parameters` | `ValueError` | `f"{name} must be finite and positive"` |
| 3752 | `PolicyNode._validate_parameters` | `ValueError` | `"timer frequencies cannot exceed nanosecond resolution"` |
| 3754 | `PolicyNode._validate_parameters` | `ValueError` | `"timestamp_tolerance_s must be finite and nonnegative"` |
| 3756 | `PolicyNode._validate_parameters` | `ValueError` | `"policy_frame_id must be a nonempty frame without whitespace or leading /"` |
| 3758 | `PolicyNode._validate_parameters` | `ValueError` | `"use_sim_time=true is unsupported for this live-robot policy"` |
| 3937 | `PolicyNode.fiducial_detection_callback` | `ValueError` | `"TF translation contains a nonfinite value"` |
| 4025 | `PolicyNode._convert_lidar_scan` | `ValueError` | `"TF translation contains a nonfinite value"` |
| 4039 | `PolicyNode._convert_lidar_scan` | `ValueError` | `"Cartesian lidar point is not finite"` |
| 4122 | `PolicyNode.run_policy_step` | `ValueError` | `"Policy outputs must be finite numbers; only pan/debug may be None"` |
| 4125 | `PolicyNode.run_policy_step` | `ValueError` | `"Debug output does not fit a Float32"` |
| 4521 | `PolicyNode.calculate_policy_actions` | `PolicyStopRequest` | `"Control reference: " + str(error)` |
| 4535 | `PolicyNode.calculate_policy_actions` | `PolicyStopRequest` | `reason` |
| 4541 | `PolicyNode.calculate_policy_actions` | `PolicyStopRequest` | `reason` |

## 不通过 raise 直接产生的停止/拒绝

`no_feasible_lattice_trajectory`、`no_valid_cartesian_output` 是 planner 的 fail 返回，
不属于上表的显式 raise；之后仍可被 ControlReferenceManager 转换为停车异常。
候选还可被 `transition_exceeds_common_horizon`、`current_speed_cannot_stop_in_horizon`
直接标记淘汰，而无需 raise。输出复核会产生 `sample_output_*` / `compatibility_*`。

框架和控制的 watchdog 也通过返回 reason 停车，包括：

- Required sensor <name> is missing or stale；No recent nonempty cone detections。
- Wheel speed unavailable for distance limit。
- Invalid wheel-speed distance input；Wheel-speed distance clock/gap invalid；Wheel-speed distance estimate invalid。
- Distance limit reached: ... m / ... m; explicit resume required。
- Run time limit reached; explicit resume required。
- Control reference expired; explicit resume required。
- Control startup history deadline exceeded; explicit resume required。
- No trusted control reference；Control cache clock/distance invalid。
- Control reference hold time exhausted；Control reference hold distance exhausted。

运动/估计器还会返回 missing_motion_history、motion_history_gap、motion_interval_too_long、
near_field_unobserved、invalid_width_or_order、observed_history_conflict、prediction_deadline_expired 等
上游状态。这些不是 Python 异常，但可使道路无效并导致下游拒绝。

## 当前诊断上的信息丢失

1. _async_planning_cycle 读到 worker diagnostics 后，若参考无效，会重新赋值为
   {async, reason, pending_age_s}，丢掉 candidates 的逐条淘汰原因。
2. robust control 的 _select_control_reference 可先抛出 ValueError；
   calculate_policy_actions 后面的 Planning rejection details 日志因此未执行。
3. _change_state 会清空 estimation_output、planning_output、planning_diagnostics。
因此最后的 no_feasible_lattice_trajectory 不足以确认究竟是净空、转向、盲区纠偏还是运动条件失败。
