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

### Vehicle-identification test workflow, local candidate, 2026-10-04

Uncommitted work on `feature/vehicle-identification`, based on Baby-Bus main
`4e86a436475cdfe4b525f6801d598b10696385d9`, adds a disabled-by-default finite
identification sequence in the existing policy script, a subscription-only
JSONL recorder, offline measurement tools and operator instructions.

On Windows / Python 3.12, `python -B
offline/vehicle_identification/test_vehicle_identification.py` passed 17 tests.
These cover the actual scheduler, selected actual policy methods with replaced
transport/clock, request limits, explicit restart, stale data, speed/timing
abort, geometry, steering conversion/fitting, angle-response summaries and log
export. They are not ROS/DDS or physical-device evidence. YAML preview passed
using PyYAML 6.0.2 and confirmed mode `off` with all request limits at zero.
Three additional installed-ROS regression tests were added to the existing
suite but have not been executed on this host.

The required `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh` was attempted
and stopped because the pinned `dream_interfaces` checkout is absent. This host
also lacks a provisioned Ubuntu/ROS Jazzy environment. The full installed gate,
recorder ROS subscriptions, deployment and physical tests are **not run**.
Run the gate in the course development environment before nonzero trials.
No vehicle was enabled, no firmware or live robot configuration was changed,
and no measured parameter values were generated. See the concise
[operator guide](../offline/vehicle_identification/README.md).

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
