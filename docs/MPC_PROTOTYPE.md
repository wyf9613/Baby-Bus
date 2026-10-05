# Minimal lateral MPC prototype: running and release gates

For a first low-speed, straight-road car test only. This is not M0. The code is in `scripts/policy_node.py` (`MPCSettings`, `reference_trajectory_from_planning`, `mpc_reference_errors`, `MinimalLateralMPC`, `MPCController`, `SpeedPI`) and the tests are in `tests/test_mpc.py`. It is off by default and enabled by the overlay `config/ai4r_policy_mpc_prototype.yaml`.

## Data flow

Estimation (`estimation_output`) → Planning V1 (straight line `y = a0 + a1·x`, reference marked `simulation_only`) → `reference_trajectory_from_planning` (v0.1 point list, keeping Planning's original timestamp and validity period) → `MPCController.step` → `drive_action`, `steering_action`. The MPC reads only the point list.

## Starting it

```
ros2 launch ai4r_policy ai4r_policy.launch.py params_file:=<share>/config/ai4r_policy_mpc_prototype.yaml
```

The overlay only turns on shadow mode. With `shadow: true` the applied drive and steering are always 0, whatever is computed, and a rejected step does not stop the policy. `shadow: false` requires `mapping_verified` and `limits_verified` both true and `v_exec_max_mps > 0`; otherwise the first step is refused (`gate_refused`).

## Conventions

- Errors: `e_y > 0` means the car is left of the path; `e_psi = car heading − path heading`; positive steering is left.
- `δ = steering_sign · steering_gain_rad_per_action · action + steering_offset_rad`. The zero command maps to `offset`, not 0 rad. There is no wheel-angle sensor, so `prev_delta` is "the estimated angle for the last command sent", not a measured wheel angle.
- The prediction step is fixed at `dt_pred_s = 0.1`; the measured dt is only range-checked (a first-step dt of 0 is accepted).
- In execution mode any rejection raises `MPCStop`; the framework goes to state 2 and needs an explicit request to leave it. There is no automatic recovery.
- Below `v_steer_min_mps` the controller does not solve and holds the current steering. In shadow mode it substitutes that speed into the model and flags `model_speed_substituted` in the log.

## Log

Topic `mpc_debug` (String, JSON), one message for every branch, including early returns and rejections. Key fields: `branch` (shadow / solved / low_speed / zero_target / ref_invalid / stop / solver_fail / over_budget / gate_refused), `reject_reason`, `candidate_*` and `applied_*`, `e_y_m`, `e_psi_rad`, `speed_mps`, `v_ref_mps`, `path_id`, `ref_age_s`, `solve_s`, `step_s`, `clip_count`, `simulation_only`. `debug1 = e_y`, `debug2 = candidate_delta` (rad).

## Release gates

| Gate | Content | Evidence |
|---|---|---|
| G0 | `python -m unittest tests/test_mpc.py tests/test_planning.py tests/test_estimation.py` | All pass (laptop, Python 3.13 / osqp 1.1.3) |
| G1 | On the Jetson, import numpy/scipy/osqp and run the same suite, read the timing output; start the node and load parameters | Mean / P95 / max step time and over-budget count; confirm the OSQP `time_limit` takes effect |
| G2 (T9) | Vehicle not enabled; push the car by hand through left/right offsets and heading errors; request state 3 | Signs of e_y, e_psi and the candidate steering; share and reasons of `ref_invalid`; rosbag; confirm wheel speed reads while pushing |
| G3 | Measure the steering mapping with the vehicle-identification `id_test`; measure the steering rate; test the speed PI on its own; measure coast distance, wheel-speed decay delay and the 0.5 s command timeout; write the measured braking deceleration back to `planning.simulation_braking_deceleration_mps2` | Write the values to `mpc.*`, set both verified flags true and set `v_exec_max_mps` |
| G4 (T10, conditional) | Fix the route, speed, stop conditions and who holds the stop button first; one run to check direction, then three in a row | Records of every run, including failures |

## Running it: Jetson, ROS, then the car

None of the commands below has been run on the Jetson or the car. Names such as `car`, the workspace path and the DREAM commands come from the repository READMEs and the vehicle-identification branch; check them against the real setup first. Stop at the first step that fails.

### 0. Before going to the car

1. **Find out how the policy node is started on the car.** Ask whoever has car access whether DREAM manages the policy node (`dream runtime restart ai4r_policy`) or it is started by hand with `ros2 launch`. This decides which route to use in section 2: DREAM reads the source YAML in the workspace and may not take the overlay file, while a manual launch takes `params_file:=`. Never run both, or two nodes will publish actions.
2. **Run the framework's ROS tests if at all possible.** `tests/test_policy_node.py` needs ROS and has not been run on these changes (only the offline suites ran, on Windows). The node's start-up changed: new `mpc.*` parameters and a new `mpc_debug` publisher. On a machine with ROS Jazzy and the build tools (the Jetson, if it has them), prepare the pinned `dream_interfaces` as in `CONTRIBUTING.md`, then:

```bash
AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" \
  bash tools/verify_fast.sh
```

   If this cannot be run, the node starting cleanly in section 2 is only indirect evidence and covers far less (no watchdog, state-machine or timeout tests). Record which of the two was done.
3. Commit and push the branch so the Jetson can pull it.

### 1. Jetson: dependencies and the offline suite (G1)

Why: the tests need no ROS, so this isolates "does the solver work on this machine and how long does a step take" from everything else.

1. Put the branch on the Jetson (`git pull`, or copy it).
2. In the `Baby-Bus` directory, with the `python3` the node uses (not a venv):

```bash
python3 -c "import sys, numpy, scipy, osqp, yaml; print(sys.executable, numpy.__version__, scipy.__version__, osqp.__version__)"
python3 -m unittest tests/test_mpc.py tests/test_planning.py tests/test_estimation.py
```

What to look at:

| Look at | Good | If not |
|---|---|---|
| Import line | Prints a path and four versions | `osqp` missing: try `pip3 install osqp` (needs network). If it cannot be installed, stop; only collect data that day |
| Last lines of the test run | `OK`, no failures | A failure on the Jetson but not on the laptop is a platform difference; record it and do not go on |
| Line `MPC step time (...) mean / p95 / max / over 50 ms budget` | p95 well below 50 ms and `0/200` over budget | Over budget: cut `horizon_n` or the solver iterations before anything else |
| Bend scenarios printed above it (`left y0=...: max |e_y| ... tail RMSE ...`) | Close to the laptop numbers (peak about 0.10 m) | Large differences mean a numeric or version problem |

Save the timing line as the Jetson timing evidence.

### 2. ROS node: launch, parameters, shadow run (G1/G2)

Shadow mode only computes: applied drive and steering are always 0, so the car cannot be driven by this node. Keep the vehicle **not enabled** throughout this step.

**Getting the node to run with the MPC settings.** Two routes; use only one, so that exactly one node publishes actions.

- DREAM-managed (the route the vehicle-identification instructions use): copy the changed files into `~/ai4r_student_workspace/src/ai4r_policy/`, merge the overlay's entries into that copy of `config/ai4r_policy.yaml` (`mpc.enabled: true`, the `required_sensors` list, and `planning.vehicle_limits_source: course_simulation` for the Planning reference), then

```bash
dream build ros student            # needed because policy_node.py changed
dream runtime restart ai4r_policy  # reads the source YAML on each start
```

- Manual launch (only if DREAM is not already running the policy node), with the ROS environment loaded:

```bash
ros2 launch ai4r_policy ai4r_policy.launch.py namespace:=car \
  params_file:=$HOME/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy_mpc_prototype.yaml
```

  Use `config/ai4r_policy_mpc_bypass.yaml` instead for the bend-capable bypass mode.

**Checks before moving anything:**

```bash
ros2 topic info /car/drive_and_steer_set_point_normalized    # exactly one publisher
ros2 topic hz /car/wheel_speed_m_per_sec                      # wheel speed arriving, also at rest
ros2 param get /car/ai4r_policy mpc.enabled                   # True
ros2 param get /car/ai4r_policy mpc.shadow                    # True
ros2 param get /car/ai4r_policy planning.vehicle_limits_source   # course_simulation (planning mode only)
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
| `branch` | Mostly `shadow` | `ref_invalid` with a `reject_reason` (for example `stale_road`, `motion_alignment_required`, `forward_coverage_short`) means the upstream reference is not usable; count how often and why. `solver_fail` or `over_budget` must be zero |
| `e_y_m` | Positive when the car is left of the road centre; close to the tape measurement (offset 10 cm gives about 0.10) | Wrong sign: the estimator or coordinate convention differs from the assumption; do not go on |
| `e_psi_rad` | Positive when the car heading is left of the road direction | Same |
| `candidate_delta_rad` | Opposite sign to `e_y_m` and to `e_psi_rad` (left of the road, so steer right: negative) | Wrong sign is a model/convention bug |
| `candidate_steer_action` | Follows `candidate_delta_rad` through the placeholder mapping; only its sign relation is meaningful until the mapping is measured | |
| `applied_steer_action`, `applied_drive` | Always 0 in shadow | Anything else is a bug |
| `step_s`, `solve_s` | p95 under 0.05 s (half of the 0.1 s period) | Over budget: reduce `horizon_n` |
| `dt` | Steady, about the cone-detection period (the node triggers on cone batches) | Large jitter or gaps above 0.2 s will lock a real run |
| `ref_age_s` | Small and below the validity period | Old references mean a timing or alignment problem |
| `model_speed_substituted` | `true` while the car is at rest or pushed slowly | Expected in shadow only; execution mode never substitutes |
| `simulation_only`, `planning_bypassed`, `reference_source` | Match the mode you started | |

Also confirm in Foxglove that wheel speed reads while you push (if it reads 0, the speed is invalid or too low for the estimator and the MPC will reject), and that `policy_fsm_state_string` stays in policy state 3 for the whole test. Plot `debug1` (e_y) and `debug2` (candidate angle) against time.

Pass for this step: signs and magnitudes right, the share of `ref_invalid` understood, no solver failures, timing inside the budget, bag saved.

### 3. On the car: measurements first, then execution (G3, G4)

Execution is refused (`gate_refused`) until `mapping_verified`, `limits_verified` and a positive `v_exec_max_mps` are set, and in bypass mode also `bypass_acknowledged`. Those flags are only to be set from the measurements below.

**3a. Measurements (vehicle enabled only for these tests; keep an RC stop to hand).** Use the vehicle-identification branch and its instructions (`offline/vehicle_identification/README.md` there). It has its own policy file with an `id_test` mode, so run it **instead of** this node, not together with it. Afterwards return to this branch.

| Measure | How | Write to |
|---|---|---|
| Steering mapping and direction | `id_test.mode: steering` with a few small steering requests at rest; measure the front-wheel angle (or turning radius R, then angle = atan(L / R)) for left and right | `mpc.steering_gain_rad_per_action`, `mpc.steering_offset_rad`, `mpc.steering_sign` (positive action must give a left turn when the sign is +1), `mpc.steering_limit_rad` |
| Steering rate | Time the wheel from one extreme to the other, or use the step test in that README | `mpc.steering_rate_limit_rad_s`; also keep it below the Traxxas slew limit (10 normalized units per second) times the gain |
| Drive needed for the test speed | `id_test.mode: drive` on the straight; find the request that holds the test speed (start at 0.15 m/s) | `mpc.drive_max` (a little above that request), `mpc.v_exec_max_mps`, then check the speed PI gains |
| Coast distance and braking | Run to the test speed, request state 2, measure the distance and time to rest; also note how long wheel speed takes to read zero (encoder timeout 3 s) | `planning.simulation_braking_deceleration_mps2` (use a conservative value, about v squared over twice the distance) |
| Command timeout | With the car stopped, confirm what happens when the node stops publishing (vehicle command timeout 0.5 s) | Notes only |

Not implemented: a speed-only test of the PI with steering held at zero. Until it exists, the speed loop is first exercised in the short straight run below, with the stop in hand.

**3b. Switch to execution.** In the workspace YAML set `mpc.shadow: false`, `mpc.mapping_verified: true`, `mpc.limits_verified: true`, `mpc.v_exec_max_mps` to the measured cap (and `mpc.bypass_acknowledged: true` only in bypass mode, knowing the clearance and stopping checks are gone), then `dream runtime restart ai4r_policy` and repeat the parameter checks in section 2.

**3c. Run order** (from the repository README): the policy starts publishing zeros, then request the vehicle Enable in Foxglove and wait for Enabled, then request policy state 3. Stop by requesting policy state 2 (zeros) and use the RC stop; zero commands are not active braking.

1. Fix beforehand, in writing: route length (short, straight, empty), start pose, target speed, stop distance, who holds the stop, and the pass criteria. Do not change them after seeing results.
2. First run: car centred, speed at `v_exec_max_mps`. Watch the direction of steering and the speed loop. Abort by state 2 or RC at any unexpected steering, speed overshoot or loss of the road.
3. If the first run is sane, three consecutive runs without manual steering correction, touching a cone, or leaving the road. Any intervention counts as a failure; keep every run, including failures.
4. Any rejection in execution mode stops the policy (state 2). It does not resume by itself: read `mpc_debug` (`branch`, `reject_reason`), fix the cause, and request state 3 again, which rebuilds the controller from zero.
5. After each run save the bag and the `mpc_debug` lines, and write down the battery state, surface and the settings used.

What to look at during execution: `branch` should be `solved` (or `low_speed` during the start-up), `applied_steer_action` should move smoothly and stay well inside [-1, 1] (`clip_count` stays 0), `e_y_m` should stay small and shrink, `step_s` inside the budget, `speed_mps` near the target. Only a straight, empty route is in scope; the result is a "first straight-line takeover prototype", not M0.

## Bends: a reference source that bypasses Planning

`mpc.reference_source: estimation_centerline` (overlay `config/ai4r_policy_mpc_bypass.yaml`) skips Planning V1 and takes the estimator's centerline directly. It fits a quadratic over `x ≤ bypass_fit_max_x_m` (default 1.5 m) and produces the same v0.1 point list, so the MPC and its interface are unchanged. Curvature is positive for a left bend; the fitted curve is extrapolated back to x = -0.1 m at the car.

- Checks that are lost: Planning's body clearance, stopping distance and speed cap no longer apply; the target speed is fixed at `bypass_target_speed_mps` (then capped by `v_exec_max_mps`). Executing (`shadow: false`) therefore also needs `bypass_acknowledged: true`, otherwise `gate_refused`. The log marks `reference_source` and `planning_bypassed`.
- Rejections (handled by the existing rules: shadow only logs, execution mode locks the stop): road not aligned to the state timestamp, stale data (≥ `bypass_max_source_age_s`), missing near coverage (first point x > `max_backward_extension_m`), short forward coverage, fit residual > `bypass_max_fit_error_m`, curvature > `bypass_max_curvature_1pm`.
- Offline evidence: left, right and S bends at R = 2.5 m (κ = 0.4), initial lateral offsets 0 and ±0.1 m, on the nonlinear plant. Peak |e_y| about 0.10 m (the initial offset) and tail RMSE 0.01–0.07 m (largest for the S bend), inside the pre-fixed E_LIMIT = 0.325 m and E_LIMIT/2. Also passes with ±30 % steering gain, a 2-step delay and 1 cm centerline noise.
- Limits: at κ = 0.4 the quadratic's extrapolation to the car is off by about 1.6 cm / 2.7°; the centerline comes only from the estimator and has not been checked against real-car data; speed is not lowered automatically in bends; the reference is still single-frame and local, with no path fusion across frames.

## Fit to Interface Contract v0.1

Checked against the interface-freeze proposal by reading the code; nothing here has run on ROS or the car.

**Matches**

- Frame and units: `base_link`, +x forward, +y left, metres, m/s, rad; curvature positive for a left bend.
- `ReferenceTrajectory` is a spatial path with per-point `target_speed_mps`, not a timed trajectory. Points carry `s_m, x_m, y_m, yaw_rad, curvature_1pm, target_speed_mps`; the top level carries `timestamp_s, frame_id, valid, stop_required, status, reason, points`. The MPC resamples internally.
- Default sample spacing 0.05 m. The per-step time budget (0.05 s) is half of a 0.1 s period, as in the appendix.
- Failure handling: invalid input or a solver failure gives (0, 0) in shadow mode, and `MPCStop` in execution mode, which the framework turns into zero actions (state 2). There is no hidden fallback to the PID and no clamped NaN.
- Both actions are limited to [-1, 1]; saturation is counted and repeated saturation stops the run.

**Deviations (to record in the Decision Log)**

| Item | Contract v0.1 | This prototype |
|---|---|---|
| Stale rule | Age above 2 nominal control periods, computed by the consumer | Uses the validity period `valid_for_s` given by Planning (at most 0.1 s), or fixed 0.2 s / 0.1 s in bypass mode. `valid_for_s` is an extra top-level field not in the contract |
| Actual dt | Algorithms use the actual dt | Prediction step is fixed at 0.1 s; the measured dt is only range-checked (0 to 0.2 s) and used by the speed PI |
| Stop semantics | With `stop_required`, target speed ramps to 0 along the path | The adapter sets every point speed to 0; in execution mode a stop is a locked stop, not a controlled deceleration |
| Parameter source | Steering mapping and limits come from Identification via `VehicleParams`; constraints are read from it | `vehicle_params` is not read (the estimator still reports it as invalid/unmeasured). Mapping and limits are placeholders in `mpc.*` |
| Planning output | Planning outputs a point list | Planning V1 outputs a polynomial `[a0, a1]`; an adapter converts it |
| Bypass mode | Estimation should not output the final reference | In bypass mode the MPC side builds the reference from the estimator centerline, which does Planning's job. It is marked `planning_bypassed` |

**Not done**

- `ControllerOutput` fields `timestamp_s`, `valid` and `controller_mode`. The node returns the framework's `(drive, steer, pan, debug1, debug2)`; MPC information is only in `mpc_debug`.
- The `controller_mode = baseline | mpc` selector (#14). There is only the `mpc.enabled` switch.
- The minimal common log of section 10 (no `run_id`, `controller_mode`, config version or scenario id).
- Decision Log entries for the deviations above.

## Known limits

- Placeholder values: steering gain/offset/limit/rate, speed PI gains, `drive_max`, and Planning's braking deceleration and delay are all unmeasured.
- The default Planning reference supports straight roads only (curvature always 0); bends need the Planning-bypass mode above.
- The closed-loop simulation only shows the implementation is self-consistent (same model family, limited gain/delay mismatch); it does not predict the real car.
- A blocked solve also stops the single-threaded executor's watchdog timer; the fallbacks are the OSQP `time_limit` and the vehicle's 0.5 s command timeout, whose real effect must be checked in G3.
- The speed PI is a temporary copy of the baseline branch's PID and should be replaced by the shared module.
