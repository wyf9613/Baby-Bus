# PID baseline offline results

Generated from validation_metrics.csv; simulation only.

The selected gains are the best of the tested coarse grids, not a global optimum.
Speed PI is preferred when its training score is within 2% of the best PID.

## Selected gains

- Speed Kp / Ki / Kd: 1.4 / 0.6 / 0.0
- Lateral Kp / Ki / Kd: 1.2 / 0.1 / 0.0
- Heading Kp: 0.8

## Validation

| Scenario | Completed / runs | Cross-track RMSE [m] | Maximum offset [m] | Steady speed RMSE [m/s] |
| --- | --- | --- | --- | --- |
| notebook_05 | 3 / 3 | 0.0267 | 0.1502 | 0.0001 |
| notebook_10 | 3 / 3 | 0.0318 | 0.1502 | 0.0053 |
| s_bend_05 | 3 / 3 | 0.0384 | 0.2014 | 0.0001 |
| s_bend_12 | 3 / 3 | 0.0479 | 0.2014 | 0.0108 |
| noisy_reference | 3 / 3 | 0.0317 | 0.1502 | 0.0075 |
| plant_mismatch | 3 / 3 | 0.0454 | 0.1502 | 0.0069 |
| command_delay_100ms | 3 / 3 | 0.0308 | 0.1502 | 0.0029 |
| dt_100ms | 3 / 3 | 0.0306 | 0.1502 | 0.0021 |
| reference_dropout | 3 / 3 | 0.0309 | 0.1502 | 0.0111 |
| selected | 1 / 1 | 0.0000 | 0.0000 | 0.0036 |
| speed_p_only | 1 / 1 | 0.0000 | 0.0000 | 0.0313 |

Selected speed step/stop run: speed <= 0.03 m/s after 0.60 s and 0.093 m following the zero-speed request from 0.4 m/s.

## Limits

- Ground-truth road geometry supplies the cubic mock planner; no cone-based planning is validated.
- Speed feedback is simulated body longitudinal velocity, with optional added noise.
- Seeds 101/202/303 alter noise; noiseless repetitions are deterministic.
- Completion means reaching within 0.3 m of the road end without a local footprint-margin failure.
- The footprint margin uses local tangent geometry; it is not an obstacle collision test.
- Stop/hold uses the Dream Gym direction latch. Physical braking and steering calibration are untested.
- The ROS policy, required-sensor contract, explicit recovery and vehicle watchdog have not been ported or tested.
