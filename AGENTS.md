# AI4R policy guidance

This repository is the student policy integration consumer. Read README.md,
CONTRIBUTING.md and docs/acceptance.md for contracts, checks and current evidence.
The canonical [DREAM guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/main/docs/development-guide.md)
governs repository workflow; preserve explicit user authority and Solo/Co-op
preferences. Ordinary implementation requests authorize in-scope edits/checks.

- On this simulation branch the user authorized splitting the imported MVP2.1
  implementation into mvp21_policy/. Its separate entry is
  scripts/mvp21_policy_node.py; tools/bundle_python.py can produce a single-file
  delivery. Keep algorithm edits in their owning modules and preserve the
  insertion markers in mvp21_policy/node.py. The user subsequently authorized
  retiring the old V2 planner: both script names now use MVP2.1, and offline
  training scores only current state, current road/obstacles and candidate
  trajectory. The user authorized removing legacy feature/model/data readers
  and planner-cost/history inputs. Preserve historical model/data artifacts and
  the immutable upstream source snapshot; do not add compatibility loaders.
- Current policy scores one untimed spatial path per call. Preserve native
  curve structure, learned segment/scene encoders, eight-sector nearest XY
  obstacle encoding and independently missing road curves. Do not restore a
  fixed sampled-point vector or require a centerline. Keep target-speed curves
  confined to C* labels/execution and record their common execution condition.
- Collection follows the user's candidate -> C* -> random legal full-path
  execution -> next-candidate loop, independently of the real-time scheduler.
  Keep failed candidates as labels, exclude solver failures only from state
  continuation, and preserve full terminal snapshots and previous controls.
  Record explicit sensor-support extensions separately from the mission goal.
- User authorized a new combined-data partition for expanded collection,
  retiring the previous batch's holdouts. Keep physical-road variants and
  mirrored same-seed suite families together; never split candidate rows.
  Select models on a development dataset containing no Test rows. Report
  observed-perception failures and explicit scenario-width-prior experiments
  separately; the latter must not change the shared/main configuration.
- User authorized a compatibility-floor merge with MVP2.1 3652e9b. ROS and
  simulation share execution.py selection and PID fallback. Do not restore the
  retired reference.py/one_hz.py execution path. Preserve MVP2.1 partial-road,
  boundary-provenance and fallback behavior; do not add a source-path TTL or
  require baseline centerline success. Use the common config/ai4r_policy.yaml;
  full-path collision/ranking and 1.3 m length remain explicit experiments.
  Record actual planner/baseline use; fallback completion is not native-only success.
- Keep one selected trigger, explicit resume, per-field IMU validity and
  freshness, zero publication independent of sensor callbacks, and pan hold.
- User authorized frozen learned-policy selection in offline runtime evaluation
  and visualization at 20 Hz perception/control and 1 Hz native generation plus
  policy ranking. Keep ROS/default native execution unchanged. Record and retain
  the known perception/branch/direction/partial-output limits in
  docs/SIMULATION_LIMITATIONS_CN.md; policy integration does not repair them.
  Preserve three-layer cone/detection/road views, shared execution admission and
  actual MVP control/fallback. No retraining or hidden truth/width inputs.
- User authorized the offline modular validation workspace and working-tree
  bundle. Keep one shared runtime; selectable observed/perfect perception,
  native/fixed-one-straight planner, native/frozen-model selection and MVP/C*
  control are explicit offline ports. Individual module clocks default to the
  current 20 Hz perception/control and 1 Hz generation/selection. Replay reads
  immutable indexed evidence only; source changes trigger fresh-process
  validation, never reinterpret old trajectories. C* plant/map truth and perfect
  perception must be labelled. Policy quality is not this round's goal.

- The policy never enables the vehicle. DREAM owns hardware services,
  calibrated facts and admitted component overrides. Its camera TF is fixed at
  zero pan; physical panning needs a measured dynamic TF before use.
- Preserve existing edits. Do not edit other repositories, flash hardware,
  change a live robot configuration, merge, or release without task authority.
- Run the CONTRIBUTING.md fast command against the pinned message definition.
  Hardware-free ROS tests do not establish physical stopping or frame accuracy.
- Implement/review with Astra in Solo mode by default. Bounded read-only
  explorers are welcome when useful; ask before delegating implementation.
- Use skills proportionately, not as automatic approval ceremonies or test
  expansion. Complete authorized work and report actual evidence concisely.

Safety, public-interface and CI-policy MRs require a human's recorded diff
review before merge. Leave a concrete, tested MR for that review.
