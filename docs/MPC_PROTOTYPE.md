# Vehicle-model MPC: running and release gates (v3)

Updated 2026-10-07 for `control-mpc-v0` at `3acd32d` (v3). For status, interfaces and where to look first, read [MPC_README.md](MPC_README.md) (Chinese). This document keeps the run-time conventions, the `mpc_debug` log, the release gates G0–G4 and the Jetson → ROS → car procedure.

The code is in `scripts/policy_node.py`:

- `VehicleParamsSettings`, `course_model_step`, `MPCSettings`;
- `reference_trajectory_from_planning`, `mpc_reference_errors`;
- `VehicleModelMPC`, `MPCController`.

The tests are in `tests/test_mpc.py`. The prediction model is the course model of `offline/mpc_prediction_model/` with the v3 drive deadband and ESC drag brake. The vehicle record is `vehicle.*`, described in [VEHICLE_PARAMS_INTEGRATION.md](VEHICLE_PARAMS_INTEGRATION.md).

The MPC is off in the shipped YAML. For car 10.43.254.27 (".27") it is enabled by layering three files:

1. `config/ai4r_policy.yaml`;
2. `config/ai4r_policy_newcar27.yaml`: the car's field settings from 2026-10-06;
3. `config/ai4r_policy_mpc_newcar27.yaml`: MVP off, MPC on, shadow; the `vehicle.*` record with the drive response fitted from the logs, `valid: false`.

`config/ai4r_policy_mpc_prototype.yaml` and `config/ai4r_policy_mpc_bypass.yaml` remain for the course (Dream Gym) vehicle only.

## Data flow

1. Estimation (`estimation_output`).
2. Planning V1: straight line `y = a0 + a1·x`. With `control.mode: mvp` (the .27 configuration) Planning uses its `mvp_low_speed` profile: geometry, freshness and domain checks only, with no footprint or stopping-distance model.
3. `reference_trajectory_from_planning`: a v0.1 point list that keeps Planning's original timestamp and validity period.
4. `MPCController.step` → `drive_action`, `steering_action`.

The MPC reads three things: the point list, the filtered speed, and the vehicle record selected by `mpc.vehicle_params_source`:

- `vehicle`: the `vehicle.*` record;
- `course_simulation`: the notebook values, shadow only.

A valid `vehicle.*` record also replaces the estimator's unmeasured `vehicle_params` / `vehicle_limits`. Its `records()` also derives `steering_limit_rad` and `steering_direction` for the MVP's calibrated profile.

## Starting it

On the car the policy runs with all three parameter files, in order (see the [babysitter guide](dream-car-babysitter-guide.md) §5.2 for building one merged YAML for DREAM). A manual launch looks like this:

```
ros2 launch ai4r_policy ai4r_policy.launch.py params_file:=<merged newcar27 MPC YAML>
```

**Shadow.** The default. The MPC computes and logs every step, but the applied drive and steering are always 0, and a rejected step does not stop the policy. The distance and time budgets are not active in shadow.

**Execution** (`shadow: false`) requires all of:

- `mpc.vehicle_params_source: vehicle`;
- `vehicle.valid: true` (a measured record);
- `v_exec_max_mps > 0`.

Otherwise the first step is refused (`gate_refused`). An inconsistent record shows `vehicle_invalid`.

The node refuses to start if any of these holds:

- `control.enabled` and `mpc.enabled` are both true;
- `id_test.mode` is not `off` while the MPC is enabled;
- in Planning reference mode, `mpc.vehicle_params_source` and `planning.vehicle_limits_source` do not select `course_simulation` together.

## Conventions

- **Errors.** `e_y > 0` means the car is left of the path; `e_psi = car heading − path heading`; positive steering is left.
- **Prediction.** State `[x, y, ψ, v, δ]` at the CG in base_link; input `[drive, steering_action]` (normalized).
  - Steering: `δ_target = clip(gain·action + offset, min, max)`, approached at the steering rate limit.
  - Drive (v3): `m·v̇ = gain·(drive − drive_deadband) − drag·v·abs(v)`, with gain = `motor_gain_n` at or above the deadband and `brake_gain_n` below it (the ESC drag brake).
  - No lateral slip. Static friction is not part of the prediction model.
  - One RK4 step per `dt_pred_s = 0.1`. This step is fixed: the measured dt only advances the command-history clock and is range-checked.
- **Optimisation.** One real-time SQP iteration per step. The model, without its clips, is linearised along the previous solution shifted by one step. An OSQP QP bounds the steering range and rate, the action range and the drive. The OSQP object is set up once and updated with warm start on later steps.
- **Drive bounds.** Drive is never negative. While the car moves (phase `tracking`) its lower bound is the effective deadband `drive_deadband + drive_bias`, so the QP stays where the model is linear; a drive at that bound means no drive force, i.e. coasting.
- **Bias estimate.** `drive_bias` integrates the model's one-step speed residual (`mpc.drive_bias_gain`, clipped to ±`drive_bias_max`) and shifts the effective deadband. This gives the MPC integral action on speed.
- **Start from rest.** Phase `starting` applies only when `vehicle.drive_breakaway > 0`. The MPC still chooses the steering, but the drive is `drive_breakaway`, rising at `mpc.breakaway_ramp_per_s` up to `drive_max`. The controller switches to `tracking` once the wheel speed reaches `moving_speed_mps`. It locks if:
  - there is no motion within `breakaway_timeout_s` (`no_motion_after_breakaway`);
  - the car returns to rest more than `max_restarts` times (`too_many_restarts`).
- **Steering estimate.** The zero steering command maps to `offset`, not 0 rad. There is no wheel-angle sensor: `delta_est_rad` is the model replay of the commands actually applied (zero in shadow mode), not a measured angle. Identified actuator delays are bridged the same way: the state is propagated over the delay with the commands already sent.
- **Overspeed.** Execution stops (`stop:overspeed`) when the measured speed exceeds `v_exec_max_mps + overspeed_margin_mps`.
- **Failure handling (v3).** In execution mode, rejections fall into two classes:
  - **Tolerable**: the reference is unavailable or expired, an update is late (`dt_max_s < dt ≤ dt_hard_max_s`, `dt_late`), or a step is over budget. For at most `reference_dropout_tolerance_s` the controller keeps executing the last solved input sequence (`plan_hold`), with the drive never above the last applied value. Without a plan yet, it coasts with the steering held (`ref_hold`). Longer than the tolerance, it locks.
  - **Immediate lock**: a Planning stop request (`valid` with `stop_requested`; always the `stop` branch, never tolerable), overspeed, invalid speed, `dt` above `dt_hard_max_s`, a solver failure, out-of-bounds actions, a start failure, or a failed gate.

  A lock raises `MPCStop`; the framework goes to state 2 and needs an explicit state-3 request, which rebuilds the controller from zero.

## Log

Topic `mpc_debug` (String, JSON): one message for every step, including early returns and rejections.

`branch` takes these values:

| Group | Values |
| --- | --- |
| Accepted | `shadow`, `solved`, `zero_target` |
| Tolerable dropouts | `plan_hold`, `ref_hold` |
| Locked | `ref_invalid`, `stop`, `solver_fail`, `over_budget`, `gate_refused`, `vehicle_invalid` |

The other fields:

- `reject_reason`;
- `phase`, `drive_bias`, `plan_index`, `dropout_s`;
- `candidate_drive`, `candidate_steer_action`, `candidate_delta_rad`, and the `applied_*` values;
- `e_y_m`, `e_psi_rad`, `speed_mps`, `v_ref_mps`, `delta_est_rad`, `pred_e_y_end_m`, `pred_v_end_mps`;
- `vehicle_source`, `vehicle_valid`, `path_id`, `ref_age_s`, `status`, `iterations`;
- `solve_s`, `step_s`, `step_p95_s` (the last 200 accepted steps);
- `dt`, `shadow`, `simulation_only`, `reference_source`, `planning_bypassed`.

`debug1 = e_y`, `debug2 = candidate_delta` (rad).

## Release gates

Simulation evidence:

- **M6 comparison (v2).** The same MPC code against the PID baseline in Dream Gym, under the PID experiment's exact scenarios, reference, seeds and metrics: `results/comparison.md`.
- **Car conditions (v3).** The full estimation → Planning → MPC chain against the fitted .27 plant at the car's timing: `results/newcar27_comparison.md`.

Both are in [offline/mpc_gym/](../offline/mpc_gym/README.md).

| Gate | Content | Evidence |
|---|---|---|
| G0 | `python -m unittest tests.test_control tests.test_mpc tests.test_planning tests.test_estimation` (22 + 70 + 17 + 28) and the offline suites in `offline/mpc_prediction_model`, `offline/vehicle_identification`, `offline/mpc_gym`, `offline/control_pid` (6 + 18 + 9 + 20); commands in [MPC_README.md §7](MPC_README.md#7-怎么验证) | All pass on the laptop (Python 3.12/3.13, osqp 1.1.3); the ROS gate (`tests/test_policy_node.py`) must be re-run after the v3 merge |
| G1 | On the Jetson, import numpy/scipy/osqp and run the same suite, read the timing output (laptop: mean about 2.6 ms, p95 about 2.7 ms at N = 10); start the node and load parameters | Mean / P95 / max step time and over-budget count; confirm the OSQP `time_limit` takes effect; decide N (10, 8 or 5) |
| G2 (T9) | With the newcar27 layers and shadow on, vehicle not enabled: push the car by hand through left/right offsets and heading errors, then request state 3. Shadow cannot run while the MVP drives: the two controllers are mutually exclusive | Signs of e_y, e_psi and the candidate steering; share and reasons of `ref_invalid`; rosbag; confirm wheel speed reads while pushing; `candidate_drive` near 0.30 when pushed at about 0.2 m/s |
| G3 | Complete the `vehicle.*` record. The drive response is already fitted from the 2026-10-06 logs (`drive_deadband` 0.289, `motor_gain_n`, `brake_gain_n`, `drag_kg_per_m`, `drive_breakaway` 0.30, `drive_delay_s` 0.1); re-check it with one short MVP run. Measure on site: steering gain magnitude and offset (the sign is known: negative), steering range, wheelbase and body; also check the 0.5 s command timeout | `vehicle.valid: true` with a source label naming the runs, then `v_exec_max_mps` (0.2 for the first runs) |
| G4 (T10, conditional) | Fix the route, speed, stop conditions and who holds the stop button first; one run to check direction, then three in a row | Records of every run, including failures |

## Running it: Jetson, ROS, then the car

None of the MPC commands below has been run on the Jetson or the car yet (the MVP, not the MPC, ran on .27 on 2026-10-06). Names such as `car`, the workspace path and the DREAM commands come from the repository READMEs and the vehicle-identification branch; check them against the real setup first. Stop at the first step that fails.

### 0. Before going to the car

1. **Find out how the policy node is started on the car.** Ask whoever has car access whether DREAM manages the policy node (`dream runtime restart ai4r_policy`) or it is started by hand with `ros2 launch`. This decides which route to use in section 2. DREAM reads the source YAML in the workspace and may not take extra parameter files, so build one merged YAML from the three newcar27 layers (babysitter guide §5.2). A manual launch takes `params_file:=`. Never run both, or two nodes will publish actions.
2. **Run the framework's ROS tests if at all possible.** `tests/test_policy_node.py` needs ROS. It passed on WSL before the v3 merge (2026-10-06) and must be re-run: start-up changed again (the merged `control.*` parameters, the mutual-exclusion check and new `mpc.*` / `vehicle.*` parameters). On a machine with ROS Jazzy and the build tools (the Jetson, if it has them), prepare the pinned `dream_interfaces` as in `CONTRIBUTING.md`, then:

```bash
AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" \
  bash tools/verify_fast.sh
```

   If this cannot be run, the node starting cleanly in section 2 is only indirect evidence and covers far less (no watchdog, state-machine or timeout tests). Record which of the two was done.
3. Commit and push the branch so the Jetson can pull it.

### 1. Jetson: dependencies and the offline suite (G1)

Why: the tests need no ROS, so this isolates "does the solver work on this machine and how long does a step take" from everything else.

One command does all of this section and saves the evidence to `~/ai4r-evidence/g1-<time>/`: `bash tools/jetson_g1_check.sh` (machine and power mode, dependency versions, the suites below, then `tools/mpc_platform_timing.py`: closed-loop step time for N = 10, 8 and 5 on a straight, a new-car start from rest and a bypass S-bend, plus a check that OSQP stops at `time_limit`). The manual steps below remain the reference for reading the results.

1. Put the branch on the Jetson (`git pull`, or copy it).
2. In the `Baby-Bus` directory, with the `python3` the node uses (not a venv):

```bash
python3 -c "import sys, numpy, scipy, osqp, yaml; print(sys.executable, numpy.__version__, scipy.__version__, osqp.__version__)"
python3 -m unittest tests.test_control tests.test_mpc tests.test_planning tests.test_estimation
```

What to look at:

| Look at | Good | If not |
|---|---|---|
| Import line | Prints a path and four versions | `osqp` missing: try `pip3 install osqp` (needs network). If it cannot be installed, stop; only collect data that day |
| Last lines of the test run | `OK`, no failures | A failure on the Jetson but not on the laptop is a platform difference; record it and do not go on |
| Line `MPC step time (...) mean / p95 / max / over 50 ms budget` | p95 well below 50 ms and `0/200` over budget | Over budget: cut `horizon_n` or the solver iterations before anything else |
| Bend scenarios printed above it (`left y0=...: max |e_y| ... tail RMSE ...`) | Close to the laptop numbers (peak about 0.10 m) | Large differences mean a numeric or version problem |
| Counts | control 22, mpc 70, planning 17, estimation 28 | A different count means a different source snapshot |

Save the timing line as the Jetson timing evidence.

### 2. ROS node: launch, parameters, shadow run (G1/G2)

Shadow mode only computes: applied drive and steering are always 0, so the car cannot be driven by this node. Keep the vehicle **not enabled** throughout this step.

**Getting the node to run with the MPC settings.** Two routes; use only one, so that exactly one node publishes actions.

- DREAM-managed (the route the vehicle-identification instructions use): copy `scripts/policy_node.py` and the merged YAML (the three newcar27 layers, built as in babysitter guide §5.2) into `~/ai4r_student_workspace/src/ai4r_policy/`, the YAML as `config/ai4r_policy.yaml`, then

```bash
dream build ros student            # needed because policy_node.py changed
dream runtime restart ai4r_policy  # reads the source YAML on each start
```

- Manual launch (only if DREAM is not already running the policy node), with the ROS environment loaded:

```bash
ros2 launch ai4r_policy ai4r_policy.launch.py namespace:=car \
  params_file:=$HOME/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml   # the merged newcar27 YAML
```

  For the bend-capable bypass mode, also set `mpc.reference_source: estimation_centerline` (shadow only until Planning V2).

**Checks before moving anything:**

```bash
ros2 topic info /car/drive_and_steer_set_point_normalized    # exactly one publisher
ros2 topic hz /car/wheel_speed_m_per_sec                      # wheel speed arriving, also at rest
ros2 param get /car/ai4r_policy control.enabled               # False (MVP off)
ros2 param get /car/ai4r_policy mpc.enabled                   # True
ros2 param get /car/ai4r_policy mpc.shadow                    # True
ros2 param get /car/ai4r_policy policy_update_mode            # timer (20 Hz)
ros2 param get /car/ai4r_policy mpc.vehicle_params_source     # vehicle
ros2 param get /car/ai4r_policy vehicle.source                # newcar27_logfit_20261006
ros2 param get /car/ai4r_policy planning.vehicle_limits_source   # upstream
ros2 param get /car/ai4r_policy mpc.reference_source          # planning, or estimation_centerline for bypass
```

**Run it:** with the vehicle disabled, request policy state 3 (State 2 stops it):

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
ros2 topic echo /car/mpc_debug          # one JSON line per step
```

If a required sensor is missing the framework refuses state 3 and says why (`policy_fsm_state_string`). Record a bag for the pushing test:

```bash
ros2 bag record /car/mpc_debug /car/debug1 /car/debug2 /car/policy_fsm_state_string \
  /car/drive_and_steer_set_point_normalized
```

**What to look at in `mpc_debug` while the car sits and while you push it by hand** (centred, then offset left, offset right, then turned left and right by a known angle; measure the real offsets with a tape):

| Field | Expect | Meaning of a miss |
|---|---|---|
| `branch` | Mostly `shadow` (`plan_hold` never appears in shadow) | `ref_invalid` with a `reject_reason` (for example `stale_road`, `insufficient_near_or_far_coverage`, `ref_expired_or_future`) means the upstream reference is not usable; count how often and why. `solver_fail` or `over_budget` must be zero |
| `e_y_m` | Positive when the car is left of the road centre; close to the tape measurement (offset 10 cm gives about 0.10) | Wrong sign: the estimator or coordinate convention differs from the assumption; do not go on |
| `e_psi_rad` | Positive when the car heading is left of the road direction | Same |
| `candidate_delta_rad` | Opposite sign to `e_y_m` and to `e_psi_rad` (left of the road, so steer right: negative) | Wrong sign is a model/convention bug |
| `candidate_steer_action`, `candidate_drive` | Steering follows `candidate_delta_rad` through the placeholder gain (negative: a left angle is a negative request; only the sign is meaningful until measured). Drive ≥ 0; at about 0.2 m/s pushed, near the fitted steady value 0.297 | A drive far from 0.3 at a steady push points at the drive record |
| `phase`, `drive_bias` | `starting` at rest, `tracking` once pushed above 0.05 m/s; bias stays 0 in shadow | |
| `applied_steer_action`, `applied_drive` | Always 0 in shadow | Anything else is a bug |
| `step_s`, `solve_s` | p95 under 0.05 s (half of the 0.1 s period) | Over budget: reduce `horizon_n` |
| `dt` | Steady, about 0.05 s (20 Hz timer) | Gaps above 0.2 s become `dt_late` (plan hold) in execution, above 0.4 s a lock |
| `ref_age_s` | Small and below the validity period | Old references mean a timing or alignment problem |
| `delta_est_rad`, `vehicle_source` | The steering offset (shadow applies zero), `newcar27_logfit_20261006` | Model replay of the applied commands, not a measured angle |
| `simulation_only`, `planning_bypassed`, `reference_source` | `false`, `false`, `planning` (unless bypass was chosen) | |

Also confirm in Foxglove that wheel speed reads while you push (if it reads 0, the speed is invalid or too low for the estimator and the MPC will reject), and that `policy_fsm_state_string` stays in policy state 3 for the whole test. Plot `debug1` (e_y) and `debug2` (candidate angle) against time.

Pass for this step: signs and magnitudes right, the share of `ref_invalid` understood, no solver failures, timing inside the budget, bag saved.

### 3. On the car: measurements first, then execution (G3, G4)

Execution is refused (`gate_refused`) until `mpc.vehicle_params_source: vehicle`, a valid measured `vehicle.*` record and a positive `v_exec_max_mps` are set, and in bypass mode also `bypass_acknowledged`. The newcar27 layer already sets the source and the fitted drive response; the measurements below complete the rest.

**3a. Measurements (vehicle enabled only for these tests; keep an RC stop to hand).** The identification sequence is part of this node: follow `offline/vehicle_identification/README.md` and [vehicle-params-test-plan.md](vehicle-params-test-plan.md), with `id_test.mode` set and `mpc.enabled: false` (the node refuses both at once). Afterwards set `id_test.mode: "off"` again.

| Measure | How | Write to |
|---|---|---|
| Geometry and footprint | Tape and axle loads (`vehicle_test_tools.py geometry`) | `vehicle.wheelbase_m`, `rear_axle_from_cg_m`, `body_*` |
| Steering mapping, direction and range | `id_test.mode: steering` at rest; equivalent front-wheel angle for several requests both ways (`steering` tool) | `vehicle.steering_gain_rad` (negative if a positive request turns right), `steering_offset_rad`, `steering_min_rad`, `steering_max_rad` |
| Steering rate and delay | Synchronised angle video and requests (`steering-dynamics` tool); keep the rate below the Traxxas slew limit (10 normalized units per second) times the gain | `vehicle.steering_rate_limit_rad_s`, `steering_delay_s` |
| Drive response | Already fitted from the 2026-10-06 MVP runs (`offline/vehicle_identification/fit_drive_from_logs.py`). Re-check with one short MVP run on the day (battery): steady request at 0.2 m/s about 0.30; optionally `id_test.mode: drive` steps to refit | `vehicle.drive_deadband`, `motor_gain_n`, `brake_gain_n`, `drag_kg_per_m`, `drive_breakaway`, `breakaway_wait_s`, `drive_delay_s`, `drive_response_model` (text) |
| Coast distance and braking | Run to the test speed, request state 2, measure distance and time to rest; note how long wheel speed takes to read zero (encoder timeout 3 s); ESC rules from `brake` mode | `vehicle.braking_deceleration_mps2` (conservative, about v² / 2d), `braking_behavior` (text), `speed_max_mps`, `safety_margin_m` |
| Command timeout | With the car stopped, confirm what happens when the node stops publishing (vehicle command timeout 0.5 s) | Notes only |

Then set `vehicle.valid: true` and `vehicle.source` to a label naming the record (date, run ids). The node refuses to start if the record is inconsistent.

**3b. Switch to execution.** In the merged workspace YAML set `vehicle.valid: true` (with the measured values and a source label), `mpc.shadow: false` and `mpc.v_exec_max_mps: 0.2` for the first runs (`mpc.bypass_acknowledged: true` only in bypass mode, knowing the clearance and stopping checks are gone). Then `dream runtime restart ai4r_policy` and repeat the parameter checks in section 2.

**3c. Run order** (from the repository README): the policy starts publishing zeros, then request the vehicle Enable in Foxglove and wait for Enabled, then request policy state 3. Stop by requesting policy state 2 (zeros) and use the RC stop; zero commands are not active braking.

1. Fix beforehand, in writing: route length (short, straight, empty), start pose, target speed, stop distance, who holds the stop, and the pass criteria. Do not change them after seeing results.
2. First run: car centred, speed at `v_exec_max_mps`. Watch the direction of steering and the speed loop. Abort by state 2 or RC at any unexpected steering, speed overshoot or loss of the road.
3. If the first run is sane, three consecutive runs without manual steering correction, touching a cone, or leaving the road. Any intervention counts as a failure; keep every run, including failures.
4. A short reference dropout (up to `reference_dropout_tolerance_s`, 0.3 s) is bridged by `plan_hold`; anything longer, and every immediate-lock reason in "Conventions", stops the policy (state 2). It does not resume by itself: read `mpc_debug` (`branch`, `reject_reason`), fix the cause, and request state 3 again, which rebuilds the controller from zero. The 3 m / 30 s budgets also stop an MPC run.
5. After each run save the bag and the `mpc_debug` lines, and write down the battery state, surface and the settings used.

What to look at during execution: `branch` should be `solved` (occasional `plan_hold`), `phase` should go from `starting` to `tracking` within about 1 s, `applied_steer_action` should move smoothly and stay well inside [-1, 1], `applied_drive` should stay near the steady value for the target speed (about 0.30 on .27) and `drive_bias` small, `e_y_m` should stay small and shrink, `pred_v_end_mps` and `speed_mps` near the target, `step_s` inside the budget. Only a straight, empty route is in scope; three clean runs at 0.5 m, 1 m and 3 m are the MPC subteam's M0 (see [MPC_PLAN_OVERVIEW.md §8](MPC_PLAN_OVERVIEW.md#8-阶段与当前状态)).

## Bends: a reference source that bypasses Planning

`mpc.reference_source: estimation_centerline` (overlay `config/ai4r_policy_mpc_bypass.yaml`) skips Planning V1 and takes the estimator's centerline directly. It fits a quadratic over `x ≤ bypass_fit_max_x_m` (default 1.5 m) and produces the same v0.1 point list, so the MPC and its interface are unchanged. Curvature is positive for a left bend; the fitted curve is extrapolated back to x = -0.1 m at the car.

- Checks that are lost: Planning's body clearance, stopping distance and speed cap no longer apply; the target speed is fixed at `bypass_target_speed_mps` (then capped by `v_exec_max_mps`). Executing (`shadow: false`) therefore also needs `bypass_acknowledged: true`, otherwise `gate_refused`. The log marks `reference_source` and `planning_bypassed`.
- Rejections (handled by the existing rules: shadow only logs; in execution they are tolerable reference dropouts, bridged by `plan_hold` up to the tolerance): road not aligned to the state timestamp, stale data (≥ `bypass_max_source_age_s`), missing near coverage (first point x > `max_backward_extension_m`), short forward coverage, fit residual > `bypass_max_fit_error_m`, curvature > `bypass_max_curvature_1pm`.
- Offline evidence: left, right and S bends at R = 2.5 m (κ = 0.4), initial lateral offsets 0 and ±0.1 m, on the course prediction model at 0.05 s with drive and steering both from the MPC. Peak |e_y| about 0.10 m (the initial offset) and tail RMSE 0.005–0.05 m (largest for the S bend), inside the pre-fixed E_LIMIT = 0.325 m and E_LIMIT/2. Also passes with steering gain ×0.85, a −3° offset, a 100 ms unmodelled delay and 1 cm centerline noise.
- Limits: at κ = 0.4 the quadratic's extrapolation to the car is off by about 1.6 cm / 2.7°; the centerline comes only from the estimator and has not been checked against real-car data; speed is not lowered automatically in bends; the reference is still single-frame and local, with no path fusion across frames.

## Fit to Interface Contract v0.1

Checked against the interface-freeze proposal by reading the code; nothing here has run on ROS or the car.

**Matches**

- Frame and units: `base_link`, +x forward, +y left, metres, m/s, rad; curvature positive for a left bend.
- `ReferenceTrajectory` is a spatial path with per-point `target_speed_mps`, not a timed trajectory. Points carry `s_m, x_m, y_m, yaw_rad, curvature_1pm, target_speed_mps`; the top level carries `timestamp_s, frame_id, valid, stop_required, status, reason, points`. The MPC resamples internally.
- Default sample spacing 0.05 m. The per-step time budget (0.05 s) is half of a 0.1 s period, as in the appendix.
- Failure handling: invalid input or a solver failure gives (0, 0) in shadow mode. In execution mode a short reference dropout holds the last solved plan (bounded in time, drive never raised); every other failure raises `MPCStop`, which the framework turns into zero actions (state 2). There is no hidden fallback to the PID and no clamped NaN.
- Both actions are limited to [-1, 1]; saturation is counted and repeated saturation stops the run.

**Deviations (to record in the Decision Log)**

| Item | Contract v0.1 | This MPC |
|---|---|---|
| Stale rule | Age above 2 nominal control periods, computed by the consumer | Uses the validity period `valid_for_s` given by Planning (`planning.reference_lifetime_s`, 0.2 s on .27), or fixed 0.2 s / 0.1 s in bypass mode. `valid_for_s` is an extra top-level field not in the contract |
| Actual dt | Algorithms use the actual dt | Prediction step is fixed at 0.1 s; the measured dt is only range-checked (0 to 0.2 s) and advances the command-history clock used for the steering/delay replay |
| Stop semantics | With `stop_required`, target speed ramps to 0 along the path | A Planning stop request is a locked stop (zero drive, so the ESC drag brake decelerates the car at about 0.5 m/s² on .27), not a controlled deceleration along the path |
| Parameter source | Steering mapping and limits come from Identification via `VehicleParams`; constraints are read from it | Read from the `vehicle.*` record (fields as `VehicleParams`, plus the numeric drive response and Planning limits), not from the estimator's `vehicle_params` dict; a valid record overwrites that dict for consumers. Course-simulation values are allowed in shadow only |
| Planning output | Planning outputs a point list | Planning V1 outputs a polynomial `[a0, a1]`; an adapter converts it |
| Bypass mode | Estimation should not output the final reference | In bypass mode the MPC side builds the reference from the estimator centerline, which does Planning's job. It is marked `planning_bypassed` |

**Not done**

- `ControllerOutput` fields `timestamp_s`, `valid` and `controller_mode`. The node returns the framework's `(drive, steer, pan, debug1, debug2)`; MPC information is only in `mpc_debug`.
- The `controller_mode = baseline | mpc` selector (#14). There are the `control.enabled` / `mpc.enabled` switches, mutually exclusive at start-up.
- The minimal common log of section 10 (no `run_id`, `controller_mode`, config version or scenario id).
- Decision Log entries for the deviations above.

## Known limits

- The drive response is fitted from one day on one car (.27, 2026-10-06); steering magnitude/offset and geometry are placeholders, so `vehicle.valid` is false and only shadow runs are possible until G3.
- The default Planning reference supports straight roads only (curvature always 0); bends need the Planning-bypass mode above.
- The v2 closed-loop simulation uses the course prediction model as the plant. The v3 car-conditions simulation imposes the fitted .27 response on the Gym car; its calibration matches the start delay and steady speed of run B12 but underestimates the start peak by about 0.07 m/s. Neither is car evidence. Unresolved modelling gaps are listed in [VEHICLE_PARAMS_INTEGRATION.md](VEHICLE_PARAMS_INTEGRATION.md).
- A blocked solve also stops the single-threaded executor's watchdog timer; the fallbacks are the OSQP `time_limit` and the vehicle's 0.5 s command timeout, whose real effect must be checked in G3.
- The step time (laptop mean about 2.6 ms, p95 about 2.7 ms with the reused OSQP object) has not been measured on the Jetson (G1).
