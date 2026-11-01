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
  actual MVP control/fallback. That frozen evaluation did not retrain or add hidden truth/width inputs.
- User authorized the offline modular validation workspace and working-tree
  bundle. Keep one shared runtime; selectable observed/perfect perception,
  native/fixed-one-straight planner, native/frozen-model selection and MVP/C*
  control are explicit offline ports. Individual module clocks default to the
  current 20 Hz perception/control and 1 Hz generation/selection. Replay reads
  immutable indexed evidence only; source changes trigger fresh-process
  validation, never reinterpret old trajectories. C* plant/map truth and perfect
  perception must be labelled. The 2026-10-18 framework packaging did not target policy quality.
- On 2026-10-19 the user authorized a new recurrent policy training round:
  random physical road widths and partial single-side cone gaps, left/right
  bends and U-turns, a common 0.5 m/s C* execution condition and an additional
  signed road-reference progress gain. Keep external scene/path features
  unchanged. All candidates at a decision share one preceding hidden state;
  commit only the actually executed candidate's next state. Train by replaying
  the fixed collected execution sequence with current weights and BPTT, never
  cache numerical hidden states across optimizer steps. Collection independently
  draws 50% random / 50% native choice at each decision within a road. Preserve
  failed candidates, report coverage and selection failures, and keep generated
  traces local. No ROS/main policy enable or automatic push is requested.
  The user subsequently authorized additional collection from multiple randomized
  starting positions on each physical road. Keep all starts of a road in one
  split, reset recurrent state per independent rollout, record physical support
  and failed initial/planning cases, and distinguish suffix completion from a
  complete entrance-to-goal drive. Preserve the earlier batch and model weights.
  The user then authorized retraining on the new multi-start batch. Produce a
  separate checkpoint, keep previous artifacts and main/default execution intact,
  and select only on Validation. Failed-selection priority and early stopping are
  explicit training options; they must not change the external feature contract.

- On 2026-10-21 the user authorized real-car learned-policy integration,
  superseding the earlier offline-only/default-native limits for this task.
  Use the unchanged latest multi-start RNN, a 0.5 m/s target matching training,
  0.50 drive feedforward/cap, strict IMU admission, and the single executable
  name scripts/policy_node.py. The current target is 10.43.254.19 (superseding
  .22); the user requested direct overwrite without a backup. Include its
  separate ROS YAML.
  The user subsequently required both colour conventions. The shared YAML now
  uses estimation.colour_assignment=auto: evaluate both from fresh measured
  two-sided geometry, lock an unambiguous assignment for the run, and reacquire
  on explicit Stop/Run. Do not revert to a fixed physical left/right colour or
  flip it on partial frames/turns. Unresolved association remains neutral only
  within the existing startup grace, then stops. An ordered_xy left_only failure
  flag can mean failed two-sided pairing, not a missing camera boundary.
  The user subsequently requested full [-1,1] steering, initially held upload,
  then authorized uploading the restored RNN policy, its YAML, and the Traxxas
  component YAML to .19. Back up ONLY the original chassis-interface settings.
  The files were uploaded/built without an agent-issued restart. The user then
  reported that the car drives as expected and authorized committing/pushing
  this policy version to the current feature branch. Record that as operator
  feedback, not independently measured physical qualification. The user ended
  remote-car work; do not reconnect or manage vehicle services for this task.
  Native collision admission and shared execution/PID remain authoritative;
  commit recurrent memory only after the selected path's action is published.
  Vehicle Enable and policy Run remain the operator's actions.

- On 2026-10-30 the user requested short-fragment S/U/compound roads with clear
  cone-inner-edge widths 0.3–1.5 m, nonuniform/gapped/overlapping cone observations
  and radial measurement drift, retaining camera FOV and no near-distance cutoff.
  Use the loaded vehicle model for steering/footprint/rate feasibility. First
  screen prescribed centerline motion through the unchanged perception and
  native planner; separate geometry, exported-road and candidate-generation
  outcomes. Preserve rejected cases and grouped seeded holdouts. This prescribed
  sensing screen is not autonomous control or policy training; radar is dummy.
  The user then raised the minimum clear width to 0.5 m, requested a full-body
  balanced prescribed reference, denser cones with a 0.5 m maximum remaining
  gap, fixed per-cone outward displacement independent of measurement noise,
  and larger per-frame Gaussian radial noise. Keep old evidence immutable and
  screen a separate revised batch; the reference remains environment-only.
  Cone spacing is measured on each boundary's own arc length; check both the
  post-omission gap cap and noninterpenetrating physical cone placement.
  On 2026-10-31 the user removed boundary-output coverage as an acceptance
  requirement: every actual planner job must still produce valid candidates;
  NO_ROAD, no feasible candidate and runtime exceptions reject a road. Generate
  a larger boundary-first pool, independently vary both physical boundaries
  before fitting a middle reference, increase compound bend diversity and
  counterbalance U-turn entry/apex/exit width timing. The user explicitly
  authorized curating suitable runtime-passing roads from this pool. Preserve
  the full pool, rejected cases and seeded split groups; report qualification
  separately from full-pool results, without training or changing main modules.

- On 2026-10-31 the user then authorized fresh collection and RNN training on
  all 39 qualified boundary-first roads, superseding the screening-only limit.
  Preserve their frozen physical cones, per-road Gaussian radial noise settings,
  multi-start/repeated sampling, 0.5 m/s C* and fixed executed-sequence replay.
  Freeze a new construction-group partition before labels are observed; keep
  all mirrors, starts and repetitions together. Physical boundary/cone truth
  belongs only to C* constraints and evaluation, never policy features. Preserve
  prior datasets/checkpoints and current main/default policy. The user explicitly
  requires staged work: start each stage, end the turn without ongoing checks,
  and wait for user feedback before starting the next stage. Do not auto-chain
  consolidation/training or create progress-monitoring automations.
  After the first collection was interrupted, the user authorized diagnosing
  the interruption and resuming the remaining 241 episodes. Keep all original
  data immutable; skip 71 finished episodes, deterministically replay executed
  prefixes without re-solving or duplicating labels, and append only new rows
  in a separate continuation directory. Start the independent hidden workers,
  then end the turn; no continuous polling or automatic next stage.
  On 2026-11-01 the user authorized completing consolidation and starting
  training in the same turn. Join all original and resumed decisions by the
  recorded episode/segment IDs, preserve the frozen grouped split and recompute
  labels from saved executions before import. Start a separate RNN fine-tune
  from the current checkpoint with unchanged inputs/target/architecture;
  select on Validation only. End the turn after starting the independent
  hidden training process and wait for feedback; no main-model replacement.

- On 2026-11-01 the user authorized a noisy closed-loop comparison of the old
  and newly trained RNN policies against native selection on the seven frozen
  Test roads. Reuse original cones/noise, current MVP control and shared
  execution/fallback at 20 Hz perception/control and 1 Hz generation/ranking.
  Preserve models, training data and main configuration; evaluation is not
  retraining or vehicle deployment. Physical truth belongs only to the plant,
  passive stop checks and labelled visualization. Keep actual speed, failure
  positions, fallback use and paired-noise/clock audits in local evidence.
  The user subsequently authorized an explicit ideal-control diagnostic:
  constant 0.5 m/s arc tracking and exact execution-pose transforms replace PID,
  actuator dynamics and execution dead reckoning only. Keep perception/fusion
  motion histories measured, shared admission/fallback active, and expose all
  connector/heading discontinuities without smoothing or lateral teleportation.
  The finite ideal join is holonomic: retain reference-tangent heading while
  approaching the curve and report actual longitudinal/lateral velocities.
  Oracle poses remain in the offline control port, never policy features or
  training labels. This does not change the physical/main control path.

- On 2026-11-01 the user authorized training the same RNN from scratch on the
  frozen noisy boundary dataset. Initialize from seed 42 without checkpoint
  weights or normalization; fit normalization on new Train inputs only. Preserve
  the 16-hidden/1841-parameter architecture, features, C* target, fixed executed
  sequences and grouped split. A separate scratch budget is lr 0.001, up to
  1000 epochs with 150-epoch Validation rank patience. Validation alone selects
  the checkpoint; Test is evaluated afterwards. Keep all existing models and
  main configuration unchanged. Start the independent hidden training stage,
  confirm startup only, then end the turn and wait for feedback. Do not chain
  runtime evaluation, replace the model or monitor epochs automatically.

- At the close of this round on 2026-11-01, the user authorized installing the
  epoch-200 fine-tuned noisy-boundary model in the local repository and pushing
  the current feature branch. This supersedes the no-main-model-replacement
  restriction for that selected checkpoint only. Pin its exact SHA-256 in the
  inference settings/YAML, regenerate the single-file delivery and preserve the
  old archived checkpoint. Keep scratch, raw datasets and verification evidence
  local. No vehicle upload, service action, merge or release is authorized.

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
