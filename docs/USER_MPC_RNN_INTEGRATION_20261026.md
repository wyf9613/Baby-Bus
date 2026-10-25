# Trained lattice selector + Zhenyu's MPC — 2026-10-26

## Source and actual behavior

Upstream: feature/mvp21-sim-integration-20261015, commit
2c41240eebd834599d1e0dc1b8715625ef4350c3, astune, 2026-10-21.
Remote branch heads were read on 2026-10-26. The separate training-pipeline
branch is f1a6c20 from 2026-10-11, not a newer runtime model.
RNN resource SHA256 a3116bb7bc4bb46ccb6e208fa3de628eca940820810943409c3157e5b12aa66b.
The multi-start recurrent model ranks collision-admitted native lattice paths;
it does not directly generate actuator commands or eliminate road-reconstruction
failures. Learned weights, encoder, recurrent inference, lattice generation,
worker protocol and common execution manager are unchanged from upstream.

Control is Zhenyu's MPC with wheel-speed PI/feedforward, extracted from local
checkpoint-reference-repair030-20261022, SHA256
2566da03e18fe24e61cfd51a63bdbef52d30b2a89f0f2f479d2bcaef79613aef.
Original numerical solver, vehicle dynamics, MPC step and speed controller remain
unchanged. An explicit learned_planner_speed_match_enabled gate allows 0.50m/s.
This matches the RNN training target speed, but MPC differs from training C* and
upstream PID execution. No retraining or physical equivalence is claimed.

## Ownership and interfaces

- Upstream estimation -> native candidates -> frozen RNN -> shared execution
  frame/epoch/progress/collision checks -> MPC reference adapter -> original MPC.
- Module local_mpc.py owns the extracted original MPC. mpc_adapter.py is the
  integration boundary. node.py orchestrates, publishes and commits RNN memory.
- Planner paths retain base_link, CG reference, metres/radians/m/s, trajectory ID,
  source age and original per-cycle deadline; no rejected path TTL renewal.
- Read-only admission/fallback occurs before the only MPC.step per control tick.
  No deep-copy/reset/second-step retry after MPC failure. PI state is continuous
  through reference-source transitions. Actual solver failure remains a stop.
- Existing local bounded join adapter can repair only rejected near-car joins,
  checking actual MPC limits and observed body collision. Displayed reference
  includes an accepted connector; original selected ID remains its parent.
- RNN memory commits only after publication of a successfully solved MPC action
  on its admitted lattice job. No commit for baseline, rejected/failed solve,
  shadow, plan hold or duplicate job. Reset and stale-memory fencing retained.
- Shared execution checks still use their upstream geometric preview; the MPC
  performs its own full reference-support and model checks. No controller-rollout
  collision assurance or real-time latency guarantee has been added.

## Profile

config/ai4r_policy.yaml is the only selected configuration in this NEW checkout.
Do not deploy an old ai4r_policy_lattice_feedback.yaml from the previous directory.
Target planning+MPC 0.50m/s, fixed left blue2/right yellow1, 20Hz control/1Hz
generation+ranking, original normalized MPC steering bound +/-0.50. Existing
drive PI/start/max remain the user's (drive max0.36), not upstream PID0.50.
Shadow planner and execution footprint aligned with original model:
front0.297m, rear0.198m, width0.25m. Camera/chassis/IMU/LiDAR YAMLs are not copied
to the car by this task. Those upstream component files belong to another car.

Original MPC vehicle parameters live under mpc_vehicle.*, avoiding collision
with upstream optional vehicle.* calibration. Values equal the original local
vehicle record. This is reuse, not identification/qualification for car13 (.26).
Strict IMU remains; camera-only gyro fallback is disabled.

mpc.shadow=true is the initial profile. It emits zero drive/steer while checking
the integration. To select an authorized physical test later, set mpc.shadow=false
in this file, rebuild the single-file bundle with the command below, upload BOTH
the generated script and this YAML, build, restart, and verify the runtime value.
No remote upload, vehicle Enable, run request, commit, push or merge performed.

## Checks and packaging

11 focused MPC bridge checks passed: model/footprint/target alignment, immutable
reference deadline and frame, one-step fallback, no re-step on failure, original
solver comparison, actual executable node cycle, original drive and steering
sign, and RNN->execution->MPC synthetic path. Nine learned-policy tests passed,
including unchanged score/hidden state, stale-memory rejection, publication
commit guards, frozen checksum, and isolated single-file worker inference.
Windows temporary-file creation required a workspace-local TEMP/TMP for the
isolated packaging test; no production timing/algorithm changes were used.

Required ROS fast gate attempted but blocked before build: missing pinned
.verification/dependencies/dream_interfaces. No full regression, ROS/Jetson or
physical validation claimed. These checks do not prove acute-turn completion.

```powershell
python -B tools/bundle_python.py scripts/mvp21_policy_node.py --root . --output scripts/policy_node.py --no-auto-resources --resource config/ai4r_policy.yaml --resource mvp21_policy/models/path_policy.json
```

The generated scripts/policy_node.py contains 23 source files, the common YAML
snapshot and the exact frozen weight bytes. The separate ROS YAML still must be
uploaded as the car's config/ai4r_policy.yaml. No separate model upload is needed.
NumPy is required; original OSQP path remains optional with NumPy dense fallback.

Operator-visible revision: user-mpc-trained-rnn-20261026. Monitor learned-policy
selection and state commits on planner_execution, and actual MPC results on
mpc_debug. Native-only success, baseline recovery and shadow-only checks are
different outcomes and must be reported separately.
