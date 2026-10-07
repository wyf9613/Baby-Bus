# Car .27 in simulation: MVP vs joint MPC

Simulation only: Dream Gym geometry and cones; longitudinal response, delay, static friction and steering sign of car .27 imposed from `offline/vehicle_identification/results/newcar27_drive_fit.json`; 20 Hz timer, 0.19-0.22 s camera latency, 5 % dropped cone batches. Not car evidence.

## Calibration (MVP, car settings) against run B12

| | start delay s | peak m/s | steady m/s |
|---|---:|---:|---:|
| car B12 | 0.53 | 0.333 | 0.2 |
| simulation | 0.5 | 0.261 | 0.202 |

## Matrix

| controller | completed 3 m | stops | max lateral m | mean lateral RMSE m | peak m/s | mean speed RMSE m/s | drive jumps | MPC step p95 ms |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| MVP | 0/25 | Planning: insufficient_near_or_far_coverage, Planning: stale_road | 0.15 | 0.061 | 0.261 | 0.006 | 1 | - |
| MPC N=10 | 24/25 | ref_invalid:invalid_estimates | 0.15 | 0.034 | 0.293 | 0.005 | 1 | 3.0 |
| MPC N=8 | 24/25 | ref_invalid:invalid_estimates | 0.15 | 0.036 | 0.285 | 0.005 | 1 | 3.1 |
| MPC N=5 | 24/25 | ref_invalid:invalid_estimates | 0.15 | 0.039 | 0.284 | 0.004 | 1 | 1.8 |

**Read the completion column with care:** the MVP locks on ANY rejected planning frame, while the MPC holds its last plan for up to 0.3 s. Most of the gap is that fault-handling difference (the same rejections ended every car run on 2026-10-06), not tracking quality. The simulation drops more cone batches than the car showed, so absolute distances are pessimistic for both.

Drive jumps: steps whose requested drive changed by more than 0.05.
Scenarios: start pose (0 / +0.15 m / -0.15 m with 0.08 rad), cone noise, deadband drift (battery), steering offset, 20 % dropped cone batches. Per-run rows are in the JSON.
