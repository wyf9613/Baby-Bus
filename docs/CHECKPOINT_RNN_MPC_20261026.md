# RNN planner + user MPC real-car checkpoint — 26 October 2026

Branch: `checkpoint/rnn-mpc-car13-20261026`.
Runtime revision: `user-mpc-rnn-narrow-admission-20261026`.

## Operator acceptance

Zhenyu Zhang reported that deployment now works and this new planner is more
stable than the previous planner, and explicitly requested preserving this
version as the new-planner checkpoint. This is operator-reported real-car
feedback on car13 (10.43.254.26), not a measured completion-rate or safety
qualification. No claim is made that all narrow roads or sharp corners pass.

## Frozen deployment

Upload BOTH `scripts/policy_node.py` and `config/ai4r_policy.yaml`. The bundle
includes the frozen recurrent model; no separate weights upload is necessary.
The authoritative hashes are `docs/user-mpc-rnn-manifest-20261026.json`:

| File | SHA256 |
| --- | --- |
| scripts/policy_node.py | f282dcc271f64da779913ec71ec812cd4f6b227a75d8db75c826c731d8f7d0b2 |
| config/ai4r_policy.yaml | 8f01613954ef72189da7cefe5c8e26c7021160f0cbc09a0eede2de945429a4d9 |
| mvp21_policy/models/path_policy.json | a3116bb7bc4bb46ccb6e208fa3de628eca940820810943409c3157e5b12aa66b |

Target0.5 m/s; fixed LEFT BLUE2 / RIGHT YELLOW1; MPC shadow=false;
normalized MPC steering +/-0.50; original vehicle model and speed-feedback PI;
6 x0.1 s prediction, 50 ms computation cap. Planner/RNN1 Hz, control20 Hz.
The model remains partially identified; car13 has not gained a new calibration.

## Integration and narrow-road changes

- Upstream recurrent model and encoding are preserved; MPC replaces PID control.
- Planner submission is independent of MPC success. First-result waiting is
  zero-output and bounded8 s; sensor validity, generation fencing and explicit
  resume remain. RNN memory commits only on published, solved selected paths.
- Reference expiry emits zero at the original deadline, with a bounded0.1 s
  neutral refresh interval. It never extends an expired trajectory or replays
  an expired driving action. BLAS pools are limited and worker payload compacted.
- Native collision checks now cover at least0.5 m, including at rest, matching
  the consumer preview. Full body dimensions0.297/0.198/0.25 m are preserved.
- Width-aware candidate offsets and a0.48 /m model-curvature screen prevent
  selecting obviously incompatible paths. Nine of48 candidates are geometric
  curvature primitives. Requested native length is1.4 m for overlap.
- RNN weights are unchanged, but candidate distribution/length differ from
  upstream training/runtime. Geometric screening is not a nonlinear tracking
  or collision-sweep guarantee; current observed boundaries remain authoritative.

## Evidence and remaining work

37 focused checks pass, including standalone bundle/model equivalence, actual
node neutral startup/refresh/dispatch, narrow straight and gentle bend, too-small
road rejection and two numerical boundary-collision replays. Fixture inputs are
portable and contain geometry only. Full ROS gate remains blocked locally by
the missing pinned dream_interfaces dependency; no full-regression pass is claimed.

Earlier logs showed selected-path/current-boundary conflicts and rejected
baseline joins, steering saturation and direction changes. The operator accepts
this version as more stable; upstream boundary association, very sharp turns,
unsupported geometry and physically too-narrow roads remain areas to verify.
Real/new boundary collisions and hard faults still stop; no collision/freshness
bypass is part of this checkpoint. Camera pan/TF and firmware were not modified.

## Restore

Check out this branch at its checkpoint commit and deploy the two frozen files
above together. Verify their hashes and runtime revision BEFORE manually
enabling/starting the car. The previous original-MPC checkpoint remains in its
existing branch; this new branch is separate. Local backup copies are retained
under backups/checkpoint-rnn-mpc-car13-20261026 and intentionally not uploaded.
