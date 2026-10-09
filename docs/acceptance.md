# AI4R policy acceptance

Status: `mvp2.0` extends the 2026-10-09 V2/fused-road/control candidate with
bounded centerline-first recovery and asynchronous planning. The current
software/model evidence is recorded below: 149 portable tests passed, and the
installed ROS gate passed 229 checks with zero errors/failures/skips. Physical
runs are not run.
The 2026-10-06 car candidate's installed gate and physical runs below are
historical evidence, not evidence for the new V2 candidate. A complete 3 m
physical run remains unverified.
Upstream gates on `jah` and the earlier entries below are historical evidence.
Release identity requires a published annotated tag and successful release CI.

## MVP2 completion-first candidate, 2026-10-09

20 Hz control/supervision remains in the ROS executor; a single isolated child
process runs planning at <=10 Hz. At most one request is in flight. Request/run
generations reject results from before an explicit stop/restart. Delayed valid
paths are transported using actual motion history; arrival never renews their
original absolute expiry. Worker failure or >=0.3 s job timeout triggers bounded
process reconstruction (<=2 Hz), while trusted geometry can bridge the gap.
Shutdown terminates the child. The child loads only stdlib/pure policy code,
avoiding ROS/DDS/TF imports during reconstruction. No worker publishes hardware
commands.

The cone-only profile first tries a quadratic centerline with a bounded rollout
of the existing lateral P/heading P controller and full approximate body boundary
checks. Lattice runs only when needed; total compute budget stays 35 ms. Sampling
is now two lengths, center/current offset and two speeds plus stop, capped at 16
candidates; geometry/time steps 0.05 m / 0.1 s. Polynomial position/heading and
curvature are retained, including when the car advances/rotates during a delayed
result. Control snapshots omit rich Frenet/speed profiles instead of deep-copying
them each cycle. Robust fitting initializes from <=3N slope pairs and exits IRLS
at convergence rather than generating all N(N-1)/2 pairs. Input/memory caps stay.

Fixed 0.4 m / 0.5 rad initialization gates no longer decide admission in this
profile: predicted correction must fit the observed/explicitly extended corridor.
The forward y(x) domain remains bounded below 1.22 rad (~70 degrees), and body
conflicts, backwards motion, missing support and invalid motion stay failures.
This is approximate model validation, not measured trajectory feasibility.

Short road/coverage failures, compute expiry, source expiry during calculation,
worker faults and purely numerical Cartesian conversion failures can use the
trusted line/curve cache. No-feasible-path/body conflicts and explicit stop paths
are not blanket exemptions. Predictions cannot seed trust or renew it. Hold is
still <=1 s AND <=0.25 m, and ends earlier on motion/support failure. Degraded
target <=0.1 m/s; recovery requires three distinct trusted camera frames with
upward target change <=0.15 m/s². Initial neutral priming is <=3 s; a predicted
first worker result stays neutral until observed control trust exists, preventing
a one-cycle start followed by a no-cache stop. Fresh wheel/
gyro and per-field source timestamps remain required; motion gap is 0.25 s.

Same-car Tuesday values are now in the active YAML: direction -1, feedforward
0.30, drive cap 0.35, speed PI 0.35/0.08, lateral/heading P 1.5/1.0. In state 2,
fresh near-zero wheel observations plus >=1 s stable gyro samples may learn a
small bias (abs<=0.12 rad/s, stddev<=0.003); it freezes during each run. The body
must remain stationary during this neutral interval. Raw messages and their
independent field freshness are never altered. Run limits remain 3 m / 30 s.

Model study: `/opt/anaconda3/bin/python -B tools/study_mvp2.py` exercises actual
YAML, estimation, planning, reference management and controller code. Eight
synthetic 1.2 m wide corridors completed the 3 m software distance limit: straight
with 0.15 m offset, ±30 degrees with ±0.15 m offset, 45 degrees, ±gentle quadratic
bends, a larger gentle bend, and a 30-degree curved start with two 0.4 s empty-frame
intervals plus a 0.2 s worker-result delay. Final lateral errors were 0.005–0.024 m.
Planning medians 5.2–6.4 ms, per-scene p95 6.5–9.9 ms, max 14.4 ms; no planning
call exceeded 35 ms. Parent computation medians 5.1–6.0 ms, p95 6.6–10.1 ms;
two wall-time spikes exceeded 35 ms, max 91.6 ms, during the concurrent ROS gate.
These are local Python 3.12
wall-time observations, not target-device CPU utilization or guaranteed deadlines.

The model uses an ideal first-order target-speed response and approximate bicycle
steering, not a calibrated ESC/grip model; synthetic observations lack camera
noise and occlusion. The worker port in this study uses deterministic delayed
delivery. Actual child-process pause/kill/restart is tested separately with ROS.
Artifacts are `.verification/mvp2-study.json` and `mvp2-study-final.log`.

Final installed gate: `.verification/mvp2-fast-gate-final-pass.log`, 229 reported
checks = 149 portable cases + 75 actual ROS cases + 5 CTest wrappers; no skips.
The actual subprocess tests paused and killed the child while continuing sensor
callbacks/control, observed cache bridging and successful reconstruction, and
confirmed a subsequent explicit stop remains stopped. Captured maximum control
call times were below 20 ms (including process reconstruction). Evidence is in
`.verification/mvp2-worker-paused.json` / `mvp2-worker-killed.json`; installed
policy/config and tested snapshot match current source byte-for-byte, hashes in
`.verification/mvp2-tested-source-sha256.json`.

For next week's physical test, load this branch's installed YAML on the same car,
leave state 2 stationary for >=1 s, then request state 3. Test a continuous
two-sided gentle curve, offset ±0.15 m and headings ±15/30 degrees before the
45-degree case. Record run completion, maximum command gap, gyro bias, planning
elapsed time, TRACKING/DEGRADED transitions and each stop reason. Wider angle
support is conditional on corridor width and available support. No car deployment,
hardware enable, main merge or physical run was performed here.

## Control integration after lattice/fused-road, 2026-10-09

Based on `origin/feature/rule-lattice-planner` revision `6245537`, preserving
perception commit `db312ab`, V2 planning, the cone-only profile, preview-based
lateral tracking and the existing speed PI/drive feedforward. The control-only
robustness baseline is retained separately at `138f90d` on `fix/newcar-control`.
No upstream estimator/planner algorithm, threshold or controller gain changed.

The 1 s / 0.25 m control hold now supports Cartesian polynomials through degree
five. Curves are sampled at <=0.025 m with <=201 points, transported by incremental
wheel/gyro motion and queried using V2's existing body-x preview. Tangent headings
rotate with the body and curvature is preserved; cached curves are not flattened
into straight lines or fitted beyond their original support. The private
`control_cached_path_samples` record belongs only to the selected control
reference; upstream `planning_output` and its public path encoding remain intact.

Upstream predicted roads cannot seed or renew control trust. Their speed
advisory/preview target is retained, and a later control fallback cannot raise
the last valid target. V2 stop trajectories clear the cruise cache. New boundary
or coverage failures, expired road prediction and planner computation-budget
expiry may use the trusted control cache; no-feasible-path, incompatible-output,
Frenet-domain, obstacle-input and frame/motion failures retain stopping behavior.
Neither prediction layer can reset the control hold budget by repeating a frame.

Validation:

- Portable: estimation 53, centerline planning 18, lattice 30, control 35; all
  136 passed. New integration cases use the actual V2 quintic output, check
  moving-curve prediction/preview, recovery, upstream prediction non-renewal,
  speed-advisory retention and stop/collision rejection behavior.
- ROS Jazzy installed fast gate: 213 checks, zero errors/failures/skips, launch
  arguments checked against clean pinned interfaces
  `5f50902ccee44e8370d6e2be85607b3054ffbf98`. Log:
  `.verification/lattice-control-fast-gate-final.log`. The installed suite tests
  both V1 and V2 callback paths and the combined default YAML.
- Old ROS fixtures were updated to provide observed support around the origin
  and disable estimator bridging when checking legacy control-stop behavior.
  Partial IMU messages keep the last valid gyro until it expires; tests now
  exercise that real deadline. Numerical lattice deadline tests use a controlled
  clock, with advancing-clock timeout tests retained separately.
- Historical stationary cone replay remains reproducible using
  `python tools/replay_control_robustness.py --source-ref 138f90d`.
  That V1 replay is not evidence of V2 physical performance.
- The unchanged wall-clock lattice study was also exercised locally. On Python
  3.12, MVP straight and curved studies completed 200 steps; blind-gap and
  calibrated studies stopped at steps 159 and 21 on source deadlines. All four
  had calls exceeding the nominal 35 ms budget. Python 3.10 under concurrent
  container-build load stopped earlier. These measurements do not qualify
  Jetson timing; the planner budget/settings are preserved for upstream owners.
  Logs: `.verification/lattice-control-study-py312.log` and
  `.verification/lattice-control-study.log`. Unit tests separately verify that
  a real planner computation timeout can transition to the trusted curve cache.

## Fused-road merge into V2, 2026-10-08

Merged `origin/feature/state-road-estimation` at `db312ab` into the local
`feature/rule-lattice-planner` branch, preserving V2, the cone-only obstacle
profile, control settings and 0.4 s motion alignment interval. Fusion defaults
are 5 s observed history, up to 0.5 m rear support and 3 m forward support,
and at most 0.2 s quality-failure bridging; these are bounds, not guaranteed
observed coverage. No live configuration or remote branch was updated.

Integration fixes accept negative-x support without a negative extension or
zero-length projection interval. Both planners consume the degraded speed
advisory and prediction deadline. V2 scales longitudinal terminal samples and
motion score targets, caps controller targets, and rejects expired prediction.
Estimator-invalid unseen startup still stays invalid; a separate aligned
`forward_geometry` record is available only to the explicitly assumed V2
controlled-start extension. Once observed origin support has been acquired,
loss cannot fall back to that startup assumption. Other estimation failures
remain rejected. No previous invalid trajectory is executed as a fallback.

Final portable checks: estimation 53, legacy planning 18, lattice 30 and control
21, total 122 passing. New integration coverage includes observed rear support
and serialization, explicit startup acceptance/rejection without input mutation,
degraded speed/deadline consumption by V1/V2, and actual policy brief-failure
slowdown, fixed prediction expiry and latched stop. Legacy controller stop tests
explicitly disable the optional bridge where they test immediate fault handling.

After merge, four 200-frame numerical planner/controller studies passed with
the relaxed 5 s budget, reproducing the prior final errors. Four 50-frame studies
also passed at 35 ms; Windows maximum was 29.05 ms. These numerical studies use
ideal road records, not live fused perception; estimator/policy integration is
covered by the portable tests. They do not qualify Jetson timing or physical
stopping. All repository Python files parse and diff whitespace checks pass.

The required ROS fast gate was retried and stopped at the absent pinned
`dream_interfaces` checkout. ROS/Jazzy, installed-node transport/configuration,
real fused-road replay and vehicle validation remain not run.

## V2 Frenet lattice local candidate, 2026-10-08

The selected YAML uses `planning.algorithm: lattice_v2`. The current controlled
cone-only trial disables `planning.lattice.obstacle_check_enabled` and removes
Cartesian lidar from required sensors. Lidar collection remains enabled; its
points and availability do not affect this trial's planning. Body/boundary and
motion checks remain enabled. This profile does not avoid lidar obstacles;
restore both the flag and required sensor before obstacle-response trials.
Runtime code remains in the single policy script. It adds
bounded near-road extension, spatial quintic lateral and time quartic/quintic
longitudinal candidates, approximate whole-body/static-obstacle checks,
physical steering/motion limits, fixed normalized scores, persistent planned
stops, original Frenet output and revalidated Cartesian compatibility output.
Both controllers use a preview for V2. Model dimensions are explicitly course
approximations; no vehicle enablement or calibrated motor/brake map changed.
See [PLANNING_V2_CN.md](PLANNING_V2_CN.md) for contracts and restrictions.

Before the fusion merge, portable checks: estimation 28, legacy planning 17, V2 lattice 26 and
control 21 tests, total 92 passing. The cone-only configuration follow-up reran
the 26 lattice, 17 planning and 21 control tests successfully. Added checks
verify absent/noisy lidar is ignored when disabled, narrow boundaries still
reject trajectories, and the actual policy follows a road without optional lidar.
The required fast gate was retried and still stops at the missing pinned
interfaces checkout. V2 cases cover analytic geometry, C2 near
extension, serialization, startup assumptions, obstacle bypass/stop, stop
persistence, stale/misaligned inputs, deterministic deadline rejection,
controller preview, and the actual policy's lidar/stop/restart integration.
These tests extract pure definitions and use stubbed ROS; they do not verify DDS.

`python -B tools/study_lattice.py --steps 200` passed four 200-step numerical
studies using the actual planner/controller and teacher approximate plant:
MVP straight, MVP curved, MVP with a 0.9 m blind gap, calibrated simulated curved.
Initial lateral error was -0.1 m; final errors were approximately -0.0343,
-0.0444, -0.0343 and +0.00247 m. This uses a relaxed 5 s numerical budget;
full-search Windows p95 times were approximately 27–45 ms, so it is not a
35 ms real-time qualification.

An earlier 35 ms run rejected one curve frame at 35.995 ms. Reserving 40% of
the budget for output checks then passed four 50-step studies at the actual
35 ms budget: p95 approximately 22.1–26.9 ms, maximum 33.1 ms, no timeout.
The planner may truncate search after finding feasible candidates; it reports
that fact and selects the best of evaluated feasible candidates. OS scheduling,
more complex scenes and Jetson execution remain unqualified. Hard timeout
rejection is intentional and requests a latched framework stop.

Required `tools/verify_fast.sh` was attempted but cannot run here: the pinned
`.verification/dependencies/dream_interfaces` checkout is absent, and this
Windows workspace has no provisioned ROS Jazzy environment. Installed YAML,
real rclpy/DDS timing, message-build and launch checks remain not run for V2.
Physical steering response, TF, wheel-speed calibration, actual brake/deceleration,
obstacle accuracy and controlled blind-start assumption also remain not run.

## Control-only robustness candidate, 2026-10-08

Implemented on `fix/newcar-control`; not deployed or merged into main. The
estimation/planning algorithms, their thresholds, lateral feedback, speed PI
and sustaining drive feedforward are unchanged. The control boundary handles
temporary road rejection using a cached, previously trusted straight reference.

`control.robustness_enabled` defaults to false in code and is enabled in the
supplied MVP YAML. The cache has a 1.0 s / 0.25 m budget from acceptance of the
last **distinct** trusted cone frame; repeated references from the same sample
cannot renew either budget. Predictions use incremental wheel/gyro motion,
retain the original measurement timestamp, and end earlier if motion history
or observed forward path support is unavailable. New valid road references
restore tracking without clearing controller integrals or resetting run budgets.
Other polynomial paths retain the original controller and expiry behavior.

The internal `TRACKING` / `DEGRADED` states preserve the external FSM interface.
Motion-feedback loss, malformed references, clock anomalies, invalid actions,
operator stop, execution timeout and run distance/time limits still latch zero
outputs with explicit restart. Cone missing/empty deadlines may be bridged only
while the cache is usable and all other required fields remain fresh. Before
the first trusted path, the original startup checks and neutral-history priming
apply. Control execution leases are at most 0.2 s and bounded by wheel/gyro
freshness; upstream road deadlines are not rewritten. Transition diagnostics
include the rejection cause, source/cache ages, prediction distance and remaining
path range; periodic status retains the last transition snapshot. To restore
legacy behavior set `control.robustness_enabled: false` and restart policy.

Verification of the runtime/configuration/test sources:

- Portable tests: estimation 28, planning 17, control 32; all 77 passed.
- Required installed fast gate passed in an isolated Docker ROS Jazzy environment
  against clean pinned `dream_interfaces` commit
  `5f50902ccee44e8370d6e2be85607b3054ffbf98`: 151 checks, zero errors/failures/skips,
  installed launch arguments checked. Log: `.verification/control-robustness-fast-gate.log`.
- The new cases exercise dropout/recovery, exact cache budgets, stale cones,
  unchanged-frame non-renewal, reference leases, independent supervision,
  incremental translation/rotation, exhausted support/history, callback behavior,
  operator stop and explicit restart. Legacy disabled-mode cases remain passing.
- `tools/replay_control_robustness.py` replays 27 full-confidence recorded cone
  frames with **synthetic stationary motion and deterministic 20 Hz timing**,
  applying the final Tuesday thresholds. Legacy first rejection is at 1.0 s
  (`unsupported_road_curvature`); the robustness candidate processes 40 tracking
  and 30 degraded cycles, stopping at 3.5 s when the hold-time budget expires.
  Output: `.verification/control-robustness-replay.json`. Run with PyYAML:
  `python -B tools/replay_control_robustness.py --source-ref 138f90d --output /tmp/control-replay.json`.
- This partial replay does not reconstruct actual driving: the main driving
  JSONL records omit per-cone confidence and height. Exact B11/B12/B14 trajectories,
  physical lateral error and improved physical run length remain unverified;
  no missing measurements were replaced with guessed confidence values.

## New-car MVP control trial, 2026-10-06

The isolated `fix/newcar-control` candidate adds optional normalized sustaining
drive feedforward (default zero, bounded by `drive_max`) and planning-rejection
diagnostics. Stop, invalid-feedback and reference-expiry paths still publish
zero; the calibrated controller is unchanged. Feedforward needs measured car
tuning and is not a universal output calibration.

On car 10.43.254.27, positive steering was observed to turn right. The car-only
configuration uses MVP steering direction -1, drive feedforward 0.30, drive cap
0.35, speed gains 0.35/0.08, and target 0.2 m/s. These overrides are retained in
the external diagnostic bundle rather than the shipped default YAML. The
operator confirmed forward motion and stopping, and subsequently reported
normal steering/throttle in the approximately 1.11 m wheel-integral trial.

With the curvature gate increased from 0.15 to 0.25 1/m, a subsequent bounded
run recorded 2.015 m wheel-integral distance in 10.54 s, peak wheel speed
0.333 m/s, and stopped on invalid road estimates (`right_only`, unknown width).
The operator confirmed this approximately 2 m run was normal and stopped.
Increasing cone confidence from 0.75 to 0.85 recovered some invalid replay
frames but reduced the next live run to 0.425 m due to insufficient coverage;
the deployed confidence threshold was restored to 0.75. One intervening start
was refused for stale gyro and issued no nonzero drive. The test then requested
policy state 2 and vehicle Disable. Wheel integration
is not an independent measurement of physical displacement. This run does not
establish 3 m completion, obstacle response, braking calibration, or general
turning performance. Boundary/freshness checks and explicit resume remain.

Verification:

- Portable estimation/planning/control tests: 28 + 17 + 21 = 66 passed.
- Student overlay build on the car: passed.
- `AI4R_INTERFACES_SOURCE=/home/ai4r/dream_system/ros2_ws/src/dream_interfaces
  bash tools/verify_fast.sh`: passed in an isolated temporary checkout;
  exact clean dependency `5f50902ccee44e8370d6e2be85607b3054ffbf98`;
  139 tests, zero failures/errors/skips, installed launch arguments passed.
  This gate uses synthetic localhost peers in test domain 218.
- An earlier Disabled idle probe recorded only zero applied drive/steering and
  zero wheel speed. A later post-run probe lost wheel/applied telemetry after
  a vehicle reconnection, so that later probe alone does not prove a physical
  stop. The final six-second check after restoring confidence 0.75 recorded
  Disabled/state 2, 271 zero wheel samples, 270 zero applied commands, and 88
  zero requested commands. Recorded results, sampled telemetry, profiles and gate logs are archived in
  `docs/experiments/2026-10-06/`; original rollback backups remain in the parent
  project workspace.
- The runtime-runner CPU problem is not claimed fully resolved.

## Evidence ownership

The offline gate in CONTRIBUTING.md builds and tests the installed policy with
the exact units-bearing `dream_interfaces` revision in ci/dependencies.repos.
Synthetic observations and a synthetic ROS peer establish software behavior,
not actual sensor/vehicle response.
Keep revision, command, result and limitations here; retain detailed logs in
CI artifacts or merge requests rather than a tracked evidence directory.

## MVP revision and 3 m run limit, 2026-10-05

The shipped YAML now selects `control.mode=mvp`, one 20 Hz timer, target speed
0.2 m/s, normalized drive cap 0.15 and absolute steering cap 0.5. All runtime
code stays in `scripts/policy_node.py`. The direct normalized lateral/heading
controller and speed PI require no complete vehicle calibration. The planner's
MVP branch keeps state/road validity, source age, time alignment, supported
coverage, both boundaries and the straight/small-error domain. It skips the
calibrated footprint and braking qualification. The optional calibrated mode
and measured parameter inlet remain available. There is no simulation fallback.
Control supports degree-five paths; the present upstream V1 emits straight paths.

Each explicit entry into policy state 3 resets raw unsigned wheel-speed
trapezoidal odometry to zero. At estimated travel >=3.0 m, the framework latches
state 2 and continuously publishes zero drive/steer with pan held. Policy
pre-calculation, pre-publication and independent supervision share one monotonic
integration clock. Repeated checks at the same time do not add distance; a
repeated state-3 request during a run does not reset it. Fresh sensor data cannot
resume a stopped run. Another explicit start resets the budget. A 30 s run-time
limit also stops a run with fresh but continuously zero wheel speed. Missing or
stale feedback, invalid/expired references and timing gaps retain latched stops.
debug1 is path normal error (m); debug2 is per-run distance (m), including the
final stop reading. This is estimated travel, not guaranteed physical stopping
position: encoder scale, slip, sampling and neutral-effort coasting still matter.

The local teacher PDFs require a single policy/YAML, finite normalized actions,
framework publication/FSM, sensor freshness and explicit recovery. They permit
local editing without ROS and describe copying to the car, building the student
overlay and restarting policy. Complete physical identification is not a first
MVP prerequisite. The earlier calibration requirement below belongs to the
optional calibrated controller. See the Chinese MVP integration note for PDF
page references and the minimal on-car sequence.

Verification of this revision:

- `python -B tests/test_estimation.py -v`: 28/28 passed.
- `python -B tests/test_planning.py -v`: 17/17 passed.
- `python -B tests/test_control.py -v`: 20/20 passed against actual single-file
  definitions/methods with recording ports, not rclpy/DDS. Includes exact 3 m,
  variable speed, repeated checks/start, raw versus filtered wheel speed,
  supervisor/post-calculation stops, latched zeros, explicit restart and 30 s.
- `conda run -n AI4R python -B offline/control_pid/test_controller.py`: 20/20
  passed (Python 3.12.13 / Dream Gym v0.4.0). Actual policy/model tests include
  both calibrated and no-calibration MVP, +/-0.1 m and +/-0.04 rad initial
  errors, 20 Hz control, 10 Hz cones and 50 ms acquisition delay. MVP runs reach
  3 m and request zero within the next check (<3.02 m estimated); final-window
  lateral offset stays <0.03 m. The model is the plant, not supplied MVP
  calibration. Independent NumPy comparisons cover 72 polynomial cases.
- Python syntax and YAML setting construction passed; whitespace checks passed.
  Original offline PID gains/models/results were not regenerated or modified.

Real-rclpy MVP callback/stop/restart and installed-YAML assertions were added,
but remain NOT RUN. The required CONTRIBUTING fast command was attempted in
WSL Ubuntu and failed at the absent pinned `dream_interfaces` directory.
The expected revision is `5f50902ccee44e8370d6e2be85607b3054ffbf98`; ROS Jazzy
and colcon are also not provisioned. No live configuration, car build, vehicle
Enable, physical test or remote push was performed. The teacher's car-side
build/restart and first physical response check remain necessary.

Tested policy SHA-256 (local file bytes):
`b1a2293e2bf5c09a0332aa2b5dc8828b0394502f25763f004093e6ed2dcd1570`.

## Earlier calibrated single-file control, 2026-10-05

Earlier local candidate: the policy executes estimation, V1 planning and
forward-only PID control in `scripts/policy_node.py`. No offline-directory,
NumPy or Gym import is needed by the installed ROS node. The control consumer
supports Cartesian y(x) through degree five, producer origin/scale/range,
CG-path feedforward and heading/normal-error feedback. The V1 planner itself
still produces a straight path; quintics are contract fixtures, not an observed
new upstream planning implementation.

The existing YAML selects one 20 Hz timer and requires cones, wheel speed and
body yaw rate. Planning recomputes each cycle from motion-aligned cached road
data; this does not claim a 20 Hz camera. Candidate normalized caps are drive
0.15 and abs(steering) 0.5. Parameter loading and configuration validation are
startup-only. Bare-node defaults leave the teaching starter disabled; the
shipped YAML explicitly enables the integrated code. The separate vehicle
Enable and explicit policy state-3 request remain necessary.

Missing calibration, invalid planning, control gaps, expired references and
feedback faults latch state 2, clear controller state and publish zeros.
Reference expiry is checked by supervision, before calculation and before
publication, including when sensor callbacks cease or calculation runs late.
Fresh data cannot automatically resume tracking. Initial missing motion history
has a bounded neutral priming period before the first successful control only.
Pan continues to hold. debug1/debug2 expose normal error (m) and filtered speed
(m/s); the existing state string reports the stop cause.

`vehicle` is the measured-parameter inlet: CG-referenced footprint, wheelbase,
CG-to-rear-axle distance, effective steering mapping, speed/deceleration/delay
limits and margin. Defaults remain invalid/unmeasured; no physical measurements
were supplied or fabricated. Effective wheel steering is measured AFTER the
vehicle interface scaling, and assumes a symmetric approximately linear map.
The stopping-domain deceleration must describe the current neutral-effort stop,
not a negative-drive brake experiment. Runtime never issues negative effort and
does not inherit the offline simulator's brake/hold assumption. Simulation
planning/control profiles must match; live control rejects simulation references
and known simulation geometry. No live vehicle configuration was changed.

Local verification:

- `python -B tests/test_estimation.py -v`: 28/28 passed.
- `python -B tests/test_planning.py -v`: 16/16 passed.
- `python -B tests/test_control.py -v`: 12/12 passed, exercising actual policy
  methods with recording transport ports (not rclpy or DDS).
- `conda run -n AI4R python -B offline/control_pid/test_controller.py`: 20/20
  passed on Python 3.12.13 / Dream Gym v0.4.0. Includes the actual policy entry,
  20 Hz control, 10 Hz synthetic cones, 50 ms acquisition delay, both initial
  offsets +/-0.1 m and headings +/-0.04 rad, 200 model steps each. Final-window
  absolute lateral offset stays below 0.03 m, final speed within 0.3+/-0.05 m/s.
  Also compares the single-file bounded root solver with the independent NumPy
  solver on 72 seeded polynomial/normalization cases. These are model checks.
- YAML was parsed and both new parameter groups instantiated; syntax checks and
  `git diff --check` passed. Original PID gain/evidence files are unchanged.

New control cases are registered with CMake; two real-rclpy integration cases
were added to `test_policy_node.py`, but those installed ROS tests are NOT RUN.
The required CONTRIBUTING fast command was attempted in WSL Ubuntu and failed
at the missing `.verification/dependencies/dream_interfaces` path. ROS Jazzy
and colcon are also not provisioned there. The required interface revision is
`5f50902ccee44e8370d6e2be85607b3054ffbf98`. Prior upstream gate results do not
qualify this changed candidate. Physical steering mapping, neutral stopping,
sensor/frame accuracy and actual-car closed loop remain NOT RUN.

Final control-tested policy SHA-256 (local file bytes):
`e9bdd6717b94f11e423a2b5815ff98daec5427aeae8e179ff3bc38e4bc25a1ce`.
Earlier zero-action/pre-integration descriptions below retain their historical
scope; the MVP section above describes the current local candidate.

## V1 planning implementation, 2026-10-05

Latest follow-up: V1 now has an explicit `course_simulation` limits profile,
with geometry from the course notebook and separately labelled configurable
motion assumptions. Default `upstream` remains invalid without calibration;
references mark simulation-only provenance. Near observed coverage is checked
before footprint support trimming, and motion-source expiry caps reference life.
Local Python: 16 planning tests and 28 estimation tests passed. A 100-frame
noisy/delayed straight-road replay yielded 100 valid frames, mean/max frame
path RMSE 0.000922/0.002202 m, planning median/p95/max 0.496/0.658/1.367 ms.
Final tested policy SHA-256:
`9e95275ee8fb12ed6ed1a5d368ae267dd6ace16f0cd668c48bf50b82aec1680d`.
These synthetic timings exclude estimation and ROS. Reports/logs are in ignored
`.verification`, and `tools/study_planning.py` reproduces the study with hashes.
The required fast gate was attempted but could not start because Bash, ROS
Jazzy and colcon are unavailable here; model/controller closed-loop and physical
checks remain not run. Earlier no-test notes below describe the initial request.

The single-file policy now includes a straight-centerline planner, internal
reference/diagnostics, startup-only planning settings and a shared evaluator.
It checks freshness, limited time alignment, straightness, coverage, body
clearance and model stopping distance. Constant-twist alignment is explicitly
opt-in; unknown vehicle limits remain invalid. Actions remain zero.
Offline cases and CMake registration are supplied, but **no tests were run**
for this change at the user's request. ROS gate, controller/model closed-loop
and physical-car evidence are also **not run**. Earlier estimation passes do
not verify this change. See [PLANNING_V1_CN.md](PLANNING_V1_CN.md).

Remote estimation commit `144ab8a` was fast-forwarded locally and the uncommitted
V1 changes restored on top. Conflict resolution preserves motion-history reset
and planning-output reset together, plus both offline loaders' definitions.
Static inspection confirms aligned road/state timestamps bypass the planner's
optional fallback compensation and retain original source age. An integration
regression is supplied, but no tests were run during this integration.

## Team estimation prototype, 2026-10-03

The initial implementation based on Baby-Bus `be46dc3` adds causal, sample-aware
low-pass filters for unsigned wheel speed and body yaw rate, and bounded Huber
road-boundary fitting in the existing single policy file. GitHub course framework
contracts take priority over the Word meeting draft. Only additional internal
group records use draft state names `speed_mps` and `yaw_rate_rps`;
`v_mps` and `yaw_rate_radps` are additive screenshot aliases. Settings are added
under `estimation` in the existing policy YAML. Physical vehicle parameters stay
unmeasured/invalid. The policy still returns zero drive/steering and pan hold.

The submission branch is based on GitHub main `4e86a43` (2026-10-04); that newer
baseline only adds course materials. Team-readable implementation/interface/
tuning notes are in [ESTIMATION_V1_CN.md](ESTIMATION_V1_CN.md).

Read-only comparison against GitHub main on 2026-10-03 confirmed that all
original policy parameter lines and the entire vehicle interface YAML are
unchanged. All original PolicyNode method signatures and 22 complete framework
methods match; estimator initialization, reset and calculation preserve the
original statements in their original order. This static comparison does not
replace the ROS gate. The outer-workspace evidence is
`tmp/project_review/framework_contract_check.json`.

On the local Windows workspace, bundled Python 3.12.14 ran
`python -B tests/test_estimation.py -v`: 15 checks passed. Checks cover duplicate
source assimilation, irregular timing/reset, noise versus lag, missing versus
zero/stale data, draft fields and YAML defaults, offset/sloped/curved roads,
outlier resistance, single-sided normal offsets, support/gap/width failures,
empty versus unavailable lidar, finite outputs and the actual student entry.
AST-based offline loading executes the estimator from policy_node.py; in a ROS
gate the same test selects the installed script. CMake registers these checks
alongside the existing ROS framework suite; one framework test additionally
asserts that absent wheel speed does not become a measured zero.

A synthetic single-frame comparison used five seeds (0-4), 20 frames each,
0.03 m lateral Gaussian noise and one +/-0.8 m misplaced cone per frame.
All 100 robust frames were valid. Mean per-frame centreline RMSE was 0.0115 m
versus 0.0668 m for ordinary quadratic least squares at the same sample points.
Windows fitting time median/p95/max was approximately 0.445/0.543/0.701 ms;
these are short local measurements, not Jetson or worst-case timing bounds.
Reproduce the numerical study with `python -B tools/study_estimation.py` from
the repository root; it prints a JSON report including the exact source hash.
The original local report remains in the outer workspace's
`tmp/project_review/estimation_study.json`. Runtime varies between runs.

The CONTRIBUTING synthetic ROS fast gate was **not run**: this Windows workspace
has no ROS/colcon/rclpy and WSL is not installed. ROS parameter loading, installed
package execution, DDS/watchdogs and the modified framework test remain to be
verified on Ubuntu 24.04 / ROS 2 Jazzy with the pinned interfaces revision.
No real-car, Gym closed-loop, or powered-actuator experiment ran. Geometry keeps
its original source-frame stamp; current-frame ego-motion compensation and
cross-frame/Kalman fusion are subsequent investigations. Single-sided estimates
require an explicitly supplied road width, unknown by default. Road-relative
errors and unimplemented single-side curvature remain None, not fabricated.

## Estimation time-alignment fix, 2026-10-05

The original draft paired current-time filtered state with acquisition-time
road geometry. A fresh frame could be valid despite belonging to an older body
frame. The fix records every accepted wheel/gyro sample in bounded history,
including those between camera triggers. Filtered motion is integrated causally
over the source-to-output interval and its inverse planar rigid transform is
applied to road/boundary and lidar points. Valid geometry and state now share
`timestamp_s`; `measurement_timestamp_s` and source ages retain original timing.

The model assumes forward, planar, no-slip motion. Wheel acquisition time is
unknown; its ROS receipt is a proxy, explicitly exposed in alignment metadata.
The low-pass and encoder delay have not been calibrated away. Wheel speed is
approximated as base_link forward speed with zero lateral speed; the CG/rear-axle
offset is unmeasured and can add turning error. Missing coverage, history gaps,
excessive interval, future geometry, unavailable motion state or
invalid transformed support reject geometry rather than merely relabel it.
Current state is a causal filtered hold. Stop/restart and ROS clock rollback
clear motion history. Lidar uses its first-ray stamp for a whole-scan rigid
approximation and is **not** individually deskewed.

Defaults add `motion_max_gap_s=0.15` and `motion_max_interval_s=0.3`; neither can
exceed `max_source_age_s`. Each source retains at most 512 motion samples. The
original framework input/action signatures, selected trigger, required-sensor
settings, supervisor and vehicle YAML are preserved. Internal acceptance hooks
record motion and clear it on clock/state resets; no driving controller is added.
Static comparison against framework baseline `4e86a43` confirms all original
PolicyNode signatures, 20 complete framework methods, original policy settings
and the entire vehicle YAML are preserved. All seven Python files parse.

Local Python 3.12.14: `python -B tests/test_estimation.py -v` passed 28 checks.
New checks use the actual `_store` and student calculation entry; verify analytic
straight/turn geometry, asynchronous causal motion, a 0.2 s delayed road with
and without history, gaps, future data, ROS rollback, stop reset, lidar metadata,
clipping and bounded history. A new two-case ROS callback regression is included
in test_policy_node.py but **not run** in this Windows environment. The full
CONTRIBUTING ROS gate and physical accuracy tests remain unperformed.

## Near-field observed support fix, 2026-10-08

On `feature/state-road-estimation`, the estimator now keeps at most 64 validated
cones per side, original observation ages, and an acquisition-frame cache.
Camera-to-camera filtered odometry transports history incrementally; current
two-sided geometry remains mandatory, nearby new detections replace old support,
and overlapping corridor conflicts discard history. Original observations expire
after at most 5 s; rear support is bounded to 0.5 m by default. No polynomial is
extrapolated into unobserved space and no repeated prediction renews point age.

Valid output now also requires the centerline and both boundaries to bracket the
body origin. Front-only observations return `near_field_unobserved`, invalid
road/alignment and empty consumable geometry, retaining coverage diagnostics.
Near-field validity does not assert footprint/swept-path clearance or an interior
Frenet projection; those are downstream planning checks using real geometry.
First-ever blind startup cannot be solved by historical observations. The branch
keeps zero actuator actions and does not import the new-car drive compensation.

Portable `python -B tests/test_estimation.py -v`: **40 checks passed**, including
the original 28 checks and 12 near-field cases. These exercise a moving 0.9 m
camera blind zone, original ages/expiry, stationary startup rejection, duplicate
batches, overlap conflicts, missing motion, clock/stop resets, empty frames,
bounded storage, analytic rotation, and a noisy curved road. The simulated blind
zone requires actual prior motion; it is not accepted before reaching the origin.
The ROS acquisition-time callback fixture now includes observed support across
the body origin; the ROS suite and CONTRIBUTING fast gate are **not run** here
(Windows lacks ROS/colcon and an installed WSL distribution). Physical camera,
odometry drift, startup procedures and full vehicle footprint remain unverified.

A local Windows measurement of the complete estimator update, 320 frames per
scenario with the first 20 excluded, gave median/p95 1.003/1.038 ms for 10 cones
per side and 4.356/4.421 ms for 64 per side (maximum 1.353/4.691 ms respectively).
The retained caches contained 20/128 points. These are local timing samples,
not Jetson CPU utilization, a worst-case deadline, or physical performance.
The source SHA256 was `65d83abc6b18ccda2c86cd3fc0cb4dba0636619f2e1f9aac2bce8b1570598f51`;
the complete report is retained outside the repository at
`tmp/project_review/near_field_benchmark.json`.

## Bounded road quality-drop bridge, 2026-10-08

The same local estimation branch adds `estimation.road_hold_s=0.2` (0 disables,
maximum 0.25 s). Fresh empty/one-sided/insufficient-common-range frames may
briefly transport the last fully valid double-sided corridor to the current
state epoch. Predictions preserve original measurement/near-support ages and
never enter the trusted cache. Failure elapsed time, source deadline, motion
interval and near observation expiry jointly bound the output. Missing or stale
sources, broken motion, width/order faults, observed overlap conflicts,
out-of-order camera timestamps, clock/reset and expiry reject reuse. Current
failed-frame diagnostics are separate from historical geometry.

Portable `python -B tests/test_estimation.py -v`: **53 checks passed** (40 previous
checks with the empty-frame case updated, and 13 new dropout checks). They cover
analytic turning transport, consecutive failures and timer reuse, recovery,
unchanged original timestamps, empty startup, width/conflict faults, missing
motion, gaps, unavailable/stale camera, stop/clock/out-of-order resets, acquisition
delay consuming the available budget, near-support expiry, disabled hold and
bounded parameters. Repeated failed batches fit only once. A ROS callback case
for empty-frame transport and invalid gyro was added but is **not run** here.

An in-memory offline integration used the actual new-car branch's extracted
`calculate_policy_actions`, planner and PID source at `6c80c51`, replacing only
its estimator definitions/settings with this candidate. Valid output at 10.0 s,
one-sided failures at 10.1/10.2, recovery at 10.3, and failures at 10.4/10.5 were
accepted by planning/control; the continuous failure at 10.6 requested a stop.
Road/state/reference epochs agreed; predictions retained 10.0/10.3 as the real
measurement stamps. The existing planner does NOT yet consume the new speed
advisory or prediction lifetime: at 10.2 its 0.15 s reference exceeded the 0.10 s
prediction remainder. An offline two-field adapter (multiply target speed by
`recommended_speed_scale`, cap reference lifetime by `prediction_remaining_s`)
produced accepted references at 0.1 m/s with the bounded lifetime. This was only
in-memory validation; no other team's source or live car was modified. Planning
owns that adapter, startup strategy, negative-x reference preservation and body
checks. No claim of automatic slowdown or full vehicle qualification is made.

Before integrating the remote planning update, the whole PolicyNode AST matched
branch HEAD `144ab8a`; input, trigger, startup/lifecycle, action and hardware-enable
behavior remained unchanged. All
repository Python source parsed and `git diff --check` passed. The required
CONTRIBUTING ROS/Jazzy fast gate and vehicle tests remain **not run**: this
Windows environment has no ROS, colcon or installed WSL distribution.

A local Windows complete-update sample (320 fresh frames; first 20 excluded)
gave median/p95 1.214/1.254 ms for 10 cones per side and 4.760/4.901 ms for 64
per side, with maxima 1.389/5.770 ms and 20/128 retained cones. These are local
measurements, not Jetson CPU utilization or real-time guarantees. Measured
policy source SHA256:
`c4879b749950da592433a7bf35a62260a670b9c48d8b9db18464ec8aadc7edcb`.
Report outside the repository: `tmp/project_review/road_dropout_benchmark.json`.

## Follow-up feedback audit and submission check, 2026-10-08

The teammate's 2026-10-08 report and `control-mpc-v0` replay log at `52af5e8`
describe sustained common-range failure (19%), one-sided visibility (14%), and
an unconfirmed approximately -0.06 rad road-relative heading offset. This
candidate only bridges brief quality loss; partial fits failing the joint-range
requirement are not assimilated into cone history. Sustained common-range or
one-sided loss still expires. Known lane width alone does not satisfy this
candidate's near-field two-boundary checks or the downstream two-boundary gate.
No fixed heading correction was applied without verified mounting/placement.
The report's seven G2 bags are not available in this workspace, so no new
real-bag replay or claim of >=90% validity / 0.3 s dropout tolerance is made.

Before submission, the remote branch had advanced to `43c3e96` with V1 planning.
The estimation patch was rebased onto that commit, preserving all remote work.
All **53 estimator checks and 16 planning checks passed**. One planning fixture
was updated to explicitly reject front-only unobserved startup before supplying
observed origin support for its same-epoch test; no planning logic changed.
All ten repository Python files parsed and `git diff --check` passed. PolicyNode
and all planning definitions remain AST-identical to `43c3e96`. The integrated
policy source SHA256 is
`54e884bf97baabc58fc767994e23ecdfae74a7b5f7d911fcf7559db824f0ee29`;
the timing report above predates this planning integration, so its timings do
not measure the new planner. ROS/Jazzy dependencies and an
installed WSL distribution remain unavailable: required ROS gate and physical
tests are **not run**. Startup and downstream advisory/lifetime consumption
remain planning-team responsibilities. No live configuration or hardware changed.

## Student-facing review, 2026-09-28

The owner reviewed `scripts/policy_node.py`, `config/ai4r_policy.yaml` and
`config/aruco_detector.yaml` at commit
`5d81db2353e798d687b0a65dfb6a0d3862805e80` and approved them without changes in
the development conversation. The reviewed Git blobs, respectively, are
`f6b2eee08e259fe08f77aab9a4f0eafbc45f84f8`,
`35b77406b6af2c938f8999cd2809b0ec6b1fe197` and
`c47dbe135b816282a82977d307a956ec7299c9b7`.
This records approval of those three student-facing files. Review of the wider
cross-repository changes and final MR verification remains separate.

## Observed software evidence

### Student lidar mounting configuration, 2026-09-30

The uncommitted candidate on `feature/student-lidar-mount`, based on
`d244b246984d0cc6d64876ffd9e506b610494da5`, was checked on the supplied Jetson
Orin Nano (Ubuntu 24.04.5 LTS, aarch64, Python 3.12.3, ROS Jazzy), using exact
clean `dream_interfaces` revision `5f50902ccee44e8370d6e2be85607b3054ffbf98`.
`AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` passed all 64 pytest
cases / 65 colcon checks, with no errors, failures or skips. Installed launch
arguments were checked, including installation of the empty `lidar_mount.yaml`
mapping. The node changes are comments only; executable behavior is unchanged.

The normalized SHA-256 of `config/lidar_mount.yaml` was
`2e4463e748064ef2a3e13916e285f1d88bb819123efa0f165ac4c3e745d7348f`;
that of `scripts/policy_node.py` was
`cd5b5a4fafc14afe09d410633edd98b45b1cc2f70f3e48219fde99541ecbcdee`.
Logs and a source manifest are retained under
`/tmp/student-lidar-verify.3P2Yuw3r` on the test host and in the local
`.test-artifacts/student-lidar-mount` directory. Subsequent acceptance edits
change documentation only. DREAM's matching acceptance record owns the
configuration/restart contracts and live lidar/TF checks. The policy gate used
synthetic peers in localhost domain 218; no policy-controlled vehicle test was
performed. Deployment and physical mounting accuracy remain **not tested**.

The implementation was subsequently committed as
`ffe7d3d0233af745b0d8a19ac1a6210a893882d4`. A fresh live scan captured at
2026-09-30 01:29:56 AEST was converted by calling that source's
`PolicyNode._convert_lidar_scan` directly, producing 600 valid points from 720
rays. The owner explicitly confirmed that the 180-degree plot matches the
car's current physical surroundings, supporting the unchanged default yaw.
DREAM's acceptance record retains the capture identity and operator evidence.
This records the default orientation; metric calibration and MR diff review
remain separate.

### Student Traxxas settings and policy comments, 2026-09-29/30

The working-tree Traxxas YAML was checked on Windows with Python 3.12 and
PyYAML 6.0.2 against the matching DREAM parameter resolver. All 19 active
settings (the original four plus 15 newly delegated settings) were admitted,
retained their types and existing defaults, and resolved to the baseline when
omitted. This was a source-configuration check, not an installed ROS test.

The assistant subsequently checked exact clean policy commit
`a6b7cbfef839adf140da43efe356b900514fee76` on the supplied Jetson Orin Nano,
Ubuntu 24.04.5 LTS / aarch64, Python 3.12.3 and ROS Jazzy. The required installed
fast gate passed: 64 pytest cases / 65 colcon checks, zero errors, failures or
skips, and installed launch arguments checked. All four release-contract tests
and source packaging passed. The installed Traxxas YAML was byte-identical to
source; the matching DREAM resolver admitted all 19 settings, preserved their
types/defaults in launch YAML and inherited the baseline for omitted values.

The owner's comment edits were committed as `4ada8d0`, followed by a small
comment correction/whitespace pass. An AST comparison confirmed unchanged
executable behavior; the only docstring change is a grammar correction.
Exact clean candidate `4c8b55480a2dcc84618982784d56d635df2aea39` was then
verified again on that Jetson using exact clean `dream_interfaces`
`5f50902ccee44e8370d6e2be85607b3054ffbf98` from `ci/dependencies.repos`:

- `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh`: 64 pytest cases /
  65 colcon checks passed, no errors, failures or skips; installed launch
  arguments checked.
- `python3 -B tests/test_release_contract.py -v`: all four tests passed.
- `python3 -B tools/package_source.py --output ...`: source-package preview
  passed in a fresh output directory. The first attempt correctly refused to
  overwrite the previous run's existing `build/release` directory.

Logs and the verification manifest are retained under
`/tmp/student-traxxas-verify.NpwVqxx9/final` on the test host and attached to the
implementation MR. Subsequent acceptance edits change documentation only.
Tests used localhost domain 218 and synthetic peers; no physical driver or
actuator ran. Deployment and physical vehicle checks remain **not run**.
Install the matching DREAM delegation update before using the new active
student YAML; older DREAM versions reject these newly admitted settings.

### Cartesian lidar observations, 2026-09-27

The candidate based on `890857e` prepares body-frame Cartesian points for every
accepted raw scan, with original ray indices and separate freshness requirements.
The assistant ran `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` on jah
against exact-clean interfaces `d95da38bbea66ff2a91f6f52ab24ea18ba116f8c`:
64 pytest cases / 65 colcon checks passed, no failures or skips. Both packages
built and installed launch arguments were checked. Cases cover three-axis TF,
invalid-ray filtering, index correspondence, empty results, failed conversion,
rejected scans, copied snapshots, both freshness deadlines and explicit recovery.

The public student build passed. A synthetic check used the installed policy
and actual DREAM mounting-TF launch action in isolated localhost domain 219,
omitting the physical driver. For 720 rays the callback median was 3.182 ms,
p95 3.291 ms and maximum 33.623 ms across 200 samples on jah. This includes raw
validation, storage and conversion, but excludes DDS delivery and student policy
execution. It is a short observation, not a worst-case timing bound. Runtime,
configuration and test hashes matched the local feature source after CRLF
normalization. The accompanying commit identifies the implementation; only
acceptance documentation was added after verification. Logs and the probe remain
under `/home/poi/lidar_cartesian_20260927/evidence/` and the matching DREAM system
evidence record. Physical lidar, mounting accuracy and driving were **not run**.

### Cone publication latency, 2026-09-27

The candidate based on `bed86cf` consumes interface
`d95da38bbea66ff2a91f6f52ab24ea18ba116f8c` and keeps cone publication latency
attached to the corresponding batch. Student coordinate lists and the function
signature remain unchanged; the internal cone observation is now a dictionary.
The assistant ran the required `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh`
on jah against the exact clean interface checkout: 53 pytest cases / 54 colcon
checks passed, no failures or skips, and installed launch arguments were checked.
The new cases cover nonempty/empty batches, independent acquisition-based age,
copied snapshots, stale availability and rejection of invalid latency without
refreshing data or triggering a policy step. The earlier overflow regression
also remains in the passing suite. Source hashes matched the local changes
after CRLF normalization; the accompanying feature commit identifies the tested
implementation. The log is `/home/poi/cone_latency_20260927/evidence/policy-fast.log`.
Only synthetic ROS peers ran; physical driving remains untested.

### Fiducial overflow and comment audit fixes, 2026-09-27

The assistant verified the executable, configuration and test sources committed
in `d6395cc2dddf3cff69a18c670fa67a7723b16619` on jah, using a disposable workspace
and exact-clean interfaces `6711f6ce7799f0d97dfa4bf9f6bc40c52b39857b`.
The required `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` passed:
48 pytest cases / 49 colcon checks, zero failures or skips, and installed launch
arguments checked. Source hashes matched the local files after CRLF normalization;
the disposable script's executable mode was restored from Git's source archive.

The new regression failed against the preceding runtime source with the expected
unhandled transform `ValueError`. With the fix it confirms whole-batch rejection,
unchanged cached freshness, no policy trigger, expiry to zero publication, and
continued explicit-resume requirements after fresh data returns. The two YAML
comment omissions are corrected. Logs are retained on jah under
`/home/poi/aruco_test_20260926/evidence/policy-audit-fast.log` and
`policy-audit-regression-before.log`, with local development copies. These checks
use synthetic ROS data; no camera, vehicle interface or physical actuator ran.

### ArUco filter configuration, 2026-09-27

Policy source `0adf7d5` adds the detector-owned allowed-ID and consecutive-frame
settings to the student YAML and its installed-configuration check. The required
`AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` passed on jah against
exact-clean interfaces `6711f6ce7799f0d97dfa4bf9f6bc40c52b39857b`: 47 pytest
cases / 48 colcon checks, no failures or skips. The log is retained at
`/home/poi/aruco_test_20260926/evidence/policy-filters-fast.log`. The policy runtime
Python source is unchanged by this filter configuration update; these are
synthetic checks, not physical driving evidence.

### ArUco development snapshot, 2026-09-26

The uncommitted 0.2.0 snapshot based on `ff6049d` built on jah against the new
working-tree fiducial interfaces and passed all 47 installed pytest cases.
Synthetic ROS cases cover camera-to-policy pose transformation, dictionary and
batch validation, fresh empty detections, stale required-input stopping,
recovery and the fiducial trigger. They establish software behavior only.

Logs remain in `/home/poi/aruco_test_20260926/evidence/policy-tests.log` and the
system development evidence record. The standalone gate subsequently passed
against exact-clean interface commit `6711f6ce7799f0d97dfa4bf9f6bc40c52b39857b`:
47 pytest cases / 48 colcon checks, zero failures or skips, with both packages
built and installed launch arguments inspected. Four portable release/provenance
regressions passed after source packaging was changed to record the dependency
manifest's exact commit without inferring the old v0.1.0 release tag. No physical
vehicle response or camera mounting accuracy was tested.

### Release preparation, 2026-09-22

The isolated `jah` checkout at
`4d16bc025a47a0b287d801977c5eee92b405bf1b` passed the full installed-package
gate with exact standalone `dream_interfaces` v0.1.0. Both packages built;
all 38 pytest cases / 39 colcon checks passed, with no failures or skips.
The installed public launch arguments were present. Four portable release
contract tests passed, and source packaging produced the archive, provenance
manifest and matching checksums. Logs are retained outside this repository in
`/home/poi/build/dream-release-20260922/logs`; MR !2 holds CI and review evidence.

The first run at `135f4efe4b008d393b56c27b5e93d4dc0976c0f9` exposed a fixed
two-second wait in the launch smoke test: it interrupted the node before its
startup message under parallel build load. The test now waits for that actual
startup event with a bounded deadline, then checks graceful shutdown. Policy
behavior was unchanged. All ROS peers in this gate were synthetic and isolated
from the ordinary robot domain.

### Ground-origin camera documentation and settings, 2026-09-22

On `jah`, the required `AI4R_INTERFACES_SOURCE=PATH bash tools/verify_fast.sh`
gate passed for the executable and configuration files committed in
`7c0ca0c050a2aefc24a68a7e12d6099692216d4a`, using exact-clean `dream_interfaces`
v0.1.0 at `9b6ef917c0b8bc31efe6ca07b8a3d25f29c35fdd`. Verification used a
working-tree snapshot before final documentation-only edits; tested source
hashes were checked before committing.

Both packages built successfully. All 38 pytest cases passed (39 colcon checks,
zero errors, failures or skips). Installed launch arguments remained `namespace`
and `params_file`, and installed `camera_mount.yaml` matched its source.
Only synthetic ROS peers ran; physical sensor, vehicle and camera mounting
accuracy were not tested.

### Initial policy gate, 2026-09-21

On 2026-09-21 (Australia/Sydney), the assistant ran `tools/verify_fast.sh` on
`jah`: ARM64 Ubuntu 24.04.4, ROS 2 Jazzy, Python 3.12.3. The isolated build used
`dream_interfaces` v0.1.0 commit `9b6ef917c0b8bc31efe6ca07b8a3d25f29c35fdd` and
policy commit `f7bf564bf3c3b6a2fda98318dc643954be9f0e98` (before this evidence
update; tree `6fc93b1fcc9b0cd234cdbb77db8d64cd3dab5bb3`). The consumed cone and
actuator messages are unchanged from the earlier tested development dependency.
Tracked executable modes were preserved by transferring a Git archive.

- Both packages built and installed successfully.
- All 38 pytest cases passed in 6.50 seconds. Colcon reports 39 checks because
  it also counts the enclosing CTest case: zero failures, errors or skips.
- The tests exercised real ROS messages, selected-trigger behavior, sensor
  expiry and recovery, state controls, IMU transforms/tare, invalid outputs and
  calculation overruns, namespaced YAML loading, synthetic DDS/timers, and the
  installed launch process starting and shutting down successfully.
- The installed launch arguments check passed. Local staged whitespace checks
  passed, and public component parameter names were checked against source.
- GitLab CI lint accepted `.gitlab-ci.yml` without errors or warnings.

The remote log is `/tmp/ai4r-policy-20260921.v1YztL/verify-fast.log`; a copy was
retained outside this repository as `tmp/ai4r-policy-jah-verify-fast.log` in the
development collection. These temporary logs are supporting evidence, not
release artifacts. No sensor driver, vehicle interface or physical car ran.
GitLab pipeline results are linked from the implementation merge request.

### Source-live runtime observation, 2026-09-22

Policy commit `b8591507b9bdcf8faa4f13da78aa673aa6e0a333` was checked from source
against the DREAM system using `dream_interfaces` v0.1.1. The observation found
the managed AI4R policy and Foxglove services starting, and Foxglove decoded
zero drive/steer actions with normalized units in policy state 2. Duplicate
start was idempotent, the detached services persisted, and graceful stop was
observed. The standalone behavior gate was not rerun during this deployment;
no calculation request, vehicle enable, firmware flash, sensor exercise, or
powered-vehicle acceptance occurred.

## Initial GitLab setup audit

Read back on 2026-09-21 for `dream/ai4r_policy` (project 7128): default branch
`dev`, semi-linear merges, encouraged squash, required successful pipelines and
resolved discussions, source deletion by default, obsolete-pipeline cancellation,
and the DREAM squash/merge message templates.

The project protection list contains exactly `main` and `dev`, with no wildcard
branch rule. Neither permits direct or force pushes. Developers/Maintainers may
merge to `dev`; only Maintainers may merge to `main` or create `v*` tags. The
instance reports GitLab 19.4.0, `enterprise: false`; group-protection and approval
APIs return 404. No separate second-person approval rule was configured. Human
diff review for this safety/public-interface/CI change remains a recorded MR
requirement, not a claim of server-enforced selective approval.

## Robot acceptance checklist

All currently **not run** with a physical car. Record source/dependency commits,
tester/date, robot configuration and observed result when these are performed:

- Zero startup, explicit vehicle/policy controls, neutral handshake and stop.
- Lidar-only and wheel-speed-only operation with no cone detector running.
- Empty/missing cones and other required-sensor loss, explicit recovery request.
- Actual command timing/vehicle watchdog and physical stop behavior.
- Pan hold on policy stop, correct pan-dependent cone frame and measured pose.
- Correct IMU mounting, body axes, relative heading and reset semantics.
- Admitted component YAML loading and component-specific restart through DREAM.

The DREAM runtime/configuration source contract now exists, but its behavior and
physical frame accuracy have not been accepted on a robot. Measured dynamic
camera TF is needed before physical pan motion. GUI workflow remains a separate
follow-up. Release publication requires promotion, final-tag CI, immutable
assets and recorded human review; their completion is recorded in GitLab MRs
and the release record.
