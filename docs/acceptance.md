# AI4R policy acceptance

Status: the implementation passes the software gate on `jah`. Version 0.1.0
identifies a release only through its published annotated tag and successful
release CI. Physical vehicle acceptance remains unperformed.

## Evidence ownership

The offline gate in CONTRIBUTING.md builds and tests the installed policy with
the exact units-bearing `dream_interfaces` revision in ci/dependencies.repos.
Synthetic observations and a synthetic ROS peer establish software behavior,
not actual sensor/vehicle response.
Keep revision, command, result and limitations here; retain detailed logs in
CI artifacts or merge requests rather than a tracked evidence directory.

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
