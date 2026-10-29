# AI4R policy acceptance

## Task-maneuver r5 comparison candidate, 2026-10-30

The r4 task planner is retained on its own branch. The separate r5 delivery
reduces extra obstacle/tracking/boundary margins without changing the 30cm
body, native following, PID, public YAML, curvature or swept-body checks.
Measured-point channel admission is approximately 40cm rather than 44cm when
a roadside LiDAR surface constrains the passing side. The unseen lateral
allowance and extra maneuver clearance remain 2cm each.

A separate diagnostic YAML disables only the forward proximity zone. Sensor
watchdogs, body intrusion, candidate/execution collision checks and explicit
resume remain active. `obstacle_stop.enabled` stays true. Diagnostics expose
the proximity switch and raw/required channel sizes.

Current installed Jazzy gate: 442 pytest cases plus 14 CTest groups (456 checks),
zero failures/errors/skips. Both actual single-file/YAML DDS checks pass.
Observed 80cm-road/25cm-side-obstacle closed loops complete with and without
the proximity zone, without contact; a 3cm more central placement also completes.
The identical standard/diagnostic trace in the first fixture shows its prior
rejection was not the 15cm gate. These are synthetic results, with no vehicle
deployment or physical qualification. Evidence: [r5 record](baselines/lidar_maneuver_20261030_r5.json).

Current policy input is limited to current state, current road/obstacles and
candidate trajectory. Old 140/148-feature and polynomial-candidate compatibility
code was removed at the user's request. The old migration claim is superseded.
C* remains a constrained numerical approximation, not a proven global optimum.
The ROS fast gate remains blocked by the absent pinned dream_interfaces checkout.
Historical planner/controller/physical evidence below remains separate.

## Observed single-boundary output, 2026-10-17

The simulation branch now retains a successfully fitted ordered single boundary
when width is unknown, and exposes it through existing forward_geometry only
after state/motion alignment. Full-corridor and near-field validity stay false;
no centerline, opposite boundary or scenario width is fabricated. Existing
stale/out-of-order, motion-gap and contradictory-pair rejection remains intact.

With frozen policy and the original C* budget/0.2 m/s condition, all eight
left/right U-turn attempts produce eight initial candidates without a width
prior. Two reach the original goal: the right-turn original-start/no-centerline
case and the left-turn road-entrance/no-centerline case. Four stop at C* solve
failure, one at insufficient common boundary support, and one at insufficient
detections near the goal. No fallback, endpoint relaxation or model updates.

The 133 estimation/contract checks and 39 module/environment/execution checks
pass. The required ROS fast gate still stops before build because the pinned
dream_interfaces checkout is absent. Policy inputs/weights, labels, planner,
plant, C* and shared YAML are unchanged; no main merge/deployment occurred.
See [before/after plots, actual traces and verification](../offline/lattice_training/artifacts/2026-10-17-single-boundary-fix/REPORT_CN.md).

## Expanded collection and failure selection, 2026-10-17

Collected 25 new physical roads across 63 explicitly recorded experiments,
including left/right curves, obstacle layouts, controlled missing centerlines
and outer-semicircle/single-inner-straight U-turns. Width-prior U-turn runs
remain separate from the original perception configuration. Thirty-seven
new sequences complete; all 26 incomplete/empty sequences remain reported.

Combined with the previous 120 native-protocol rows: 2111 candidates, 258
decisions and 4222 C* trials (3725 successful, 497 failures retained).
Physical-road and mirrored same-seed views remain grouped. Train/Validation/
Test rows are 1091/476/544; development fitting/model choice contains no Test.
Validation-only selection chooses hidden 16, seed 0, epoch 175 of 180.
Avoidable nominal failure selections are 1/130 Train, 0/58 Validation and
1/66 Test; four all-failed decisions are reported separately. The remaining
test error is on the previous batch, despite no avoidable failure on the
62 feasible-option decisions from fresh test roads. No learned closed loop
or main-runtime integration is claimed.

All 2111 scores survive save/load and archive extraction exactly. The 208
state/time/control links and 35424 accepted simulation steps pass integrity
checks; original record content/labels are unchanged except split/provenance.
Sixty-eight portable checks pass. The pinned ROS dependency is still missing,
so the fast gate stops before build. Runtime/configuration/plant/C* modules
and 57 protected files remain unchanged. See [report and evidence](../offline/lattice_training/artifacts/2026-10-17-expanded-collection/REPORT_CN.md).

## First persistent spatial-policy batch, 2026-10-16

Fresh collection, C* labels, training, save/load and frozen-candidate scoring
completed. Three isolated roads produce 19 decisions, 120 candidates and 240
trials (210 successful; 30 failures retained). Random continuation uses only
C*-feasible candidates followed by full legal execution, without reward/policy
ranking. All 19 segments pass replay constraints; all 16 continuation links
preserve the exact seven-state vector, timestamp and previous controls.

An explicit 2 m sensor-support runout fixes terminal cone loss without moving
the original mission goal or initial sampling domain. Collection finishes
within the declared 0.15 m progress tolerance, without a stop requirement.
All three sampled mission roads are straight, each with one obstacle; no
centerline-missing observation occurred in this batch.

Training uses 44/52/24 Train/Validation/Test rows, with validation checkpoint
44 of 60 epochs. The reloaded model reproduces all 120 scores. It selects a
C* failure in 1/8 frozen validation decisions and 0/4 test decisions: this is
workflow evidence, not learned closed-loop qualification. Native runtime,
common configuration, plant, C* solver/objective and 57 protected files remain
unchanged. Current portable checks: 32 input/model/sampling, two actual C*
continuation checks and 34 runtime/compatibility checks pass. ROS fast gate
still stops before build on the missing pinned dream_interfaces checkout.
See [batch report](../offline/lattice_training/artifacts/2026-10-16-new-protocol-batch/REPORT_CN.md).

## Untimed spatial policy and historical data audit, 2026-10-16

The sole offline scorer now accepts current state, current road/obstacles and
one untimed spatial candidate. Native polyline segments feed learned encoders
and candidate-to-scene attention; no fixed sampled-point vector is retained.
Obstacles use eight front sectors with nearest observed XY and availability.
Centerline and either boundary may be absent independently. The training
adapter no longer requires baseline centerline success before native planning.
Policy inputs do not read speed profiles, timing, planner costs or history.

C* still uses its existing explicit timing and speed references for supervision.
Datasets/checkpoints declare a common execution condition, preventing mixed
hidden target speeds. The numerical C* solver, objective, plant, runtime
planner/PID and common YAML are unchanged. No learned scorer is installed in
ROS or the native replay runtime.

Validation: the full 65-check training/input suite passed before final execution
condition checks; final focused input/model/dataset checks and a second real
C* CLI roundtrip cover those changes. All 38 module/simulation/execution and
upstream-compatibility checks passed. ROS fast gate again stopped before build
because the pinned dream_interfaces checkout is absent. Exact results and
fingerprints are in [verification.json](../offline/lattice_training/artifacts/2026-10-16-spatial-policy/verification.json).

Full scan of the retained merged data found 217 decisions, 4936 candidates and
9872 C* trial records. Three frozen NPZ files and the merged raw-record hash
match their metadata. All scene records can enter the new scene encoder;
all candidate records still use FRENET_SPATIAL/S_REF_OF_TIME, so none is
natively compatible with the new sampled spatial contract. Historical labels
mix 0.1/0.2/0.3 m/s and stop conditions, and their terminal-speed-extension
protocol differs from the current explicit-full-timing contract. No records
were converted, labels recomputed, or historical weights/data overwritten.
See [data-audit.json](../offline/lattice_training/artifacts/2026-10-16-spatial-policy/data-audit.json).

## Shared execution merge, 2026-10-16

This supersedes the dual-adapter state at acc5324. ROS and simulation now share
PlannerExecutionManager.prepare/select and calculate_selected_control. The local
bounded-reference adapter and literal handoff paths were removed. The shared
YAML enables planner execution and retains MVP2.1 1.0 m/prefix defaults, partial
road support, boundary provenance and same-tick baseline fallback. Local run/job/
timestamp fencing is checked before replacing a path. Collision and ranking
coverage are independent explicit experiment options.

All 299 final portable checks pass, including six paired compatibility checks
against immutable 3652e9b source. Twenty recorded planner inputs keep status,
selected candidate, path and hard boundaries; edge input and execution matrices
show no added rejection. This evidence is bounded to the tested inputs, not a
universal real-car or ROS compatibility proof. Old cache-specific tests were
retired with that implementation; current motion/delivery and execution tests remain.
The real-C* CLI collection/training/save/load/evaluation smoke check also passes
after freezing the final configuration. No persistent production model was trained.

With fixed 100 ms delivery, default perception/map runs complete in 147/148
cycles, using planner for 141/143 and baseline for 6/5 cycles. Two baseline
cycles are startup; the other 4/3 are preview exhaustion. Native-only acceptance
fails in both runs. Explicit 1.3 m/full-check/full-score perception has 147 planner
cycles and two startup baseline cycles, also failing strict native-only acceptance.
The centerline control remains 152 cycles. The strict acceptance switch changes
only exit status and was checked to return failure on a completed fallback run.
ROS gate still stops before build on the absent pinned dream_interfaces checkout.
No hardware or deployment occurred; learned scoring is disconnected.
See [shared execution](../offline/mvp21_sim/SHARED_EXECUTION_CN.md).

## Upstream module sync, 2026-10-16

Synced MVP2.1 `3652e9b` temporal guidance, bounded 20 Hz collection, worker
execution anchors, and its opt-in ROS execution adapter into the existing modules.
The ROS switch stays off locally. Offline native execution keeps its generated
path coverage, 1.3 m minimum length, bounded reference cache and zero-action
failure behavior. Synced `mpc-car-run` `d155a80` ordered-pairing configuration
with the existing 0.03 m default. No MPC controller or learned scorer is active.

All 302 selected portable checks pass, including 20 added temporal cases,
18 execution cases, five ordered-pairing cases, and policy input checks.
Final native perception/map runs reach 4.756241/4.754804 m in 151/150 cycles,
each accepting eight VALID fused results.
The centerline baseline still completes in 152 cycles at 4.742971 m.
The final ROS gate stops before build because its pinned dream_interfaces
checkout is absent. No deployment occurred; speed tracking remains unqualified.
See [sync details](../offline/mvp21_sim/UPSTREAM_SYNC_20261016.md).

## Native planner default, 2026-10-16

The offline `offline.mvp21_sim` entry now defaults to native lattice selection
at 1 Hz with cached, motion-aligned paths for 20 Hz control. No learned scorer
is connected. The historical centerline baseline requires
`--path-source perception`; ROS configuration remains shadow-only.

Twenty-three module/reference/simulation checks pass. Both CLI runs without a
path-source override complete with fixed 100 ms delivery: perception 151 cycles
and 4.747474 m, ideal map 149 cycles and 4.737976 m, versus the 4.736800 m goal.
Each accepts eight VALID native results. These runs retain the previously
documented speed-tracking limitation. The ROS gate remains unavailable because
the pinned dream_interfaces checkout is absent.
See [run evidence](../offline/mvp21_sim/artifacts/2026-10-16/native-direct.json).

## Three-input learned policy, 2026-10-15

The scorer now reads only current state, current road/obstacles and candidate
trajectory. Raw point/trajectory sampling and padding replace planner cost,
clearance-statistic, sector-aggregation and previous-path features. The old
model loaders and polynomial-candidate decoder are deleted. Native scores are
baseline diagnostics only. Current models/datasets have a distinct schema and
cannot silently load historical 140/148-feature checkpoints.

The complete 61-case input/training suite passed. Final removal of implicit
time-profile extension was followed by 40 passing targeted input/objective/model/
C* checks, including real-environment CLI collection, training, save/load and
evaluation. Forty worker/reference/simulation/bundle checks also passed. Neural
input and prediction invariance under altered planner scores, past paths and
labels is tested; the three raw input groups retain their geometric information.
The final time contract requires full explicit candidate coverage. Objective
weights and C* optimizer are retained; no automatic terminal-speed path tail is
added. The source and configuration identify this stricter objective explicitly.

The unchanged native-ranked one-Hz framework still completes ideal/perception
recorded-road runs: 149/151 cycles and 4.737976/4.747474 m final station. These are
framework regressions, not learned-model road qualification. Training validation
is a small real-C* smoke run; no production model training or deployment occurred.
The ROS gate stops before build because the pinned dream_interfaces checkout is
missing. Fifty-seven protected artifacts/plant/solver files pass checkpoint
hashes; seventeen authorized source/config/document migrations are listed apart.

See [input contract](../offline/lattice_training/POLICY_INPUT.md) and
[fingerprinted checks](../offline/mvp21_sim/artifacts/2026-10-15/policy-three-inputs.json).

## MVP2.1 one-Hz offline control, 2026-10-15

Added bounded ordered-path reference reuse and segmented wheel/gyro motion
alignment in the modular MVP2.1 implementation. Offline simulation submits at
1 Hz and controls at 20 Hz while a causal service model delays planner delivery.
An opt-in generated-path collision/scoring profile and explicit 1.3/1.5 m
minimum lengths address the previously unchecked tail and acceleration-related
preview exhaustion. The original shadow prefix profile and PID remain unchanged.

Ideal/perception runs with measured planner CPU time plus an assumed 50 ms
delivery overhead both complete: 8 valid results each, 147/149 tracking cycles,
4.739302/4.754692 m final station, 0.179403/0.173813 m minimum corner margin.
Fixed 100 ms delivery runs also complete. All 152 baseline actions match the
previous centerline run; 74 original files and the pinned source/config remain
unchanged. Eleven reference/delivery, eight simulation and four module/process/
bundle checks pass (23 total), including expiry, old-run fencing and zero-action
stop requests. This is not physical stopping or continuous-sweep qualification.

Speed tracking still fails (about 0.78 m/s maximum versus 0.2 m/s target).
ROS entry remains shadow-only; no deployment, hardware or executor real-time
qualification. The ROS gate was not run: its pinned dependency is still absent.
Details: offline/mvp21_sim/ONE_HZ_CONTROL.md and its fingerprinted artifact.

## Continuous boundary / full body clearance, 2026-10-15

Replaced the native planner's ten footprint samples, global slope conversion,
edge_guard and subtracted sweep_guard with continuous cubic-boundary distance
to the full oriented filled rectangle, corridor containment and support checks.
The configured .05 m hard margin and vehicle/motion limits remain unchanged.
Unseen backward extension retains its existing uncertainty allowance; it is zero
on observed body support. Static-point obstacle margins remain unchanged.

Intervals between path poses use body travel bounds to certify clearance or
subdivide and check actual intermediate poses. Bounded unresolved intervals,
collisions and timeouts reject the candidate. The Cartesian compatibility route
also checks intervals on its actual fitted polynomial; recorded-road experiments
continue to use direct Frenet export. This is numerical geometric validation,
not a formal or physical safety qualification.

All three recorded failure poses admit native paths with unchanged ideal input.
The original default-sampling geometric run reached 1.815711 m (281 valid paths),
and the prior parameter candidate reached 1.939313 m (359 valid paths), then
failed low_speed_yaw_inconsistent at .049881 m/s and -.060220 rad/s. No run
completed the 4.736800 m goal. Default 35 ms execution and a shorter/coarser
sampling experiment still observed lattice_time_budget_exceeded; isolated
successful frame timing does not establish whole-road real-time execution.

42 lattice checks, 53 training/integration checks, 53 estimation checks, 18 legacy
planning checks and 21 control checks pass (187 total). The final full pipeline
suite completed in 624.207 seconds. Results and source fingerprints are recorded in
offline/lattice_training/artifacts/2026-10-15-continuous-body/verification.json.
The ROS fast gate was retried through WSL and stops before building because its
pinned dream_interfaces checkout is absent. No deployment or hardware test.
Details: offline/lattice_training/CONTINUOUS_BODY_CN.md.

Follow-up diagnosis separates centerline progress from world x and wall time from
simulation time. At s=1.939313 m, x=1.965656 m, the low-speed gate rejects a turn
consistent with the pinned bicycle model. A process-local initialization
counterfactual resumes the exact state but fails again after three valid paths.
The former timed-out input passes eight isolated repetitions in 25.778–27.422 ms;
this is neither whole-road nor target-hardware timing qualification. A fresh native
run of the short-range parameter group with wall-clock isolation, and unchanged
low-speed logic, produces 312 paths and fails at s=1.946846 m on steering/motion
constraints. No planner source or physical threshold changed in this diagnosis.
Details: offline/lattice_training/REMAINING_DIAGNOSIS_CN.md.

## Native planner ideal-input diagnosis before geometry change, 2026-10-15

Recorded cone logs were extracted byte-for-byte from fix/newcar-control commit
5e8d1b8e4bec54d1306f769315e57246f9fff496. The Car 15 end-straight1 recording
contains 270 cone frames. Parameter readback is a 20 Hz policy timer; measured
camera acquisition interval/latency medians are .099986/.192415 seconds.
The reconstructed common bilateral corridor is 4.234 m, with a separately
declared unobserved 1 m entry buffer. Raw gyro sign is inferred from cone-map
consistency, not surveyed mounting calibration.

The offline runner uses direct native Frenet export (Cartesian fitting disabled),
ideal mapped road input, optional exact plant state, and the existing C* solver.
Native paths are queried at 20 Hz. C* is updated every .5 simulated seconds;
solver wall time does not advance the simulated clock. This does not qualify
the complete system for real-time execution or establish a perception pass.

The geometric baseline from rest still failed after 107 valid paths at .739 m,
with no_feasible_lattice_trajectory. Its wall-budget counterpart also observed
lattice_time_budget_exceeded. A local derivative-bound counterfactual admits a
path on the original failure frame; it has not changed production source.
Separate shorter-range/time-window settings reached 1.334/1.512 m before motion
or supported-transition rejection. No experiment completed the road. Thus the
condition for attributing the issue solely to an environment mismatch is unmet.

Existing test_lattice.py:34 passed; test_lattice_training.py:53 passed. The
hardware-free ROS fast gate was attempted in WSL but its pinned dream_interfaces
checkout is absent, so the gate did not run. No hardware or deployed configuration
was changed. Commands, limits and preserved evidence are in
offline/lattice_training/NATIVE_REPLAY_CN.md and its compact artifact directory.

## Retraining on expanded Train, 2026-10-11

The user authorized retraining. Actual frozen corpus:3295 Train candidates/145
states/11 labelled roads,942 validation/37/two and699 test/35/two. Same from-scratch
seed0,140-32-1 tanh/sigmoid,Adam/.005,150 epochs,pair weight.25,L2.0001,T=1 target
sigmoid(mean(raw C* returns)). Validation score RMSE selects epoch150; test is
excluded from fitting/model selection/hyperparameter changes. No coverage override
is required; provenance records candidate readiness=true,legacy reported flag=false,
road coverage=false and candidate_supervision. Labels/rewards/data remain unchanged.

Validation old/new selected raw mean return .229130/.299922; pair accuracy
.649886/.684919; score RMSE .103350/.097947. Test old/new return .445127/.424509;
pair accuracy .797230/.768405; score RMSE .074287/.059393. Calibration improves on
both holdouts and validation selection improves; test selection/ranking regresses
slightly. Both models are preserved. Best validation is at this run's final epoch;
no convergence or whole-road performance claim. Train score RMSE .041580.

Saved-model/source/schema/target checks, finite0-1 predictions, expanded-Train-only
normalization, matching saved metrics and unchanged old holdout metrics are checked.
Every original validation/test array, frozen input/previous model file,205 protected
files and Git refs remain unchanged. Public score CLI ran on a new-road recorded
decision and its scores/argmax match frozen-feature inference. No unit suite or new
policy closed-loop run. Artifacts under .verification/augmented-sigmoid-training-20261011:
model.json,model.training.json,evaluation.json,training.png,comparison.png,
comparison.json,verification.json and REPORT.md.
Later fixed-label ranking diagnosis:Test has8014 non-tied pairs;1078 correct→wrong,
847 wrong→correct,net231 lost. The three largest selected-return regressions,all
seed201,account for212 of those net lost pairs. In an initial-state candidate pair
with the same1.5m transition and-.221226m offset,the .3m/s candidate's raw return
.517249 exceeds .1m/s candidate's .212864. Old scores .608090/.577746 order them
correctly; new scores .426740/.608027 reverse them. Test road200 selection improves
slightly (.477012→.477864) while road201 drops (.430514→.400055). Regression includes
medium return gaps and persists in state-averaged ranking; it is a real local
generalization regression. The current score/difference-MSE objective and RMSE
checkpoint criterion do not guarantee maximizing ordering accuracy or selected
return. A single fit cannot isolate data distribution,scaling,capacity or training
length as the cause. No algorithm/target change or new fit was made for diagnosis.
Details:augmented-sigmoid-training-20261011/ranking_diagnosis.json and .md.

## Candidate label readiness separated from whole-road coverage, 2026-10-11

The user questioned rejection of labelled candidates when the random episode
does not reach the road end. That gate conflated collection coverage with sample
validity. Candidate supervision targets the full C* execution of each candidate,
not an episode-to-road-end return. A later planning failure does not invalidate
previous labels; a requested road with no candidates supplies no rows. Declared
failure labels remain valid penalized supervision and are not filtered out.

The shared training entry now validates the existing source/label/objective/hash/
schema/split contract, then requires at least two Train rows and a nonempty
Validation split. It recomputes readiness for legacy frozen metadata without
changing it; provenance records both the original reported flag and computed
readiness. The old override option remains accepted for command compatibility
but no exception is needed. Collection and append also store sample readiness
separately from road_coverage_complete; absent requested road goals/labels stay
visible as diagnostics and do not prevent valid rows reaching training. Missing
required label splits still stop collection/training. C* trial completion,
constraints, failure penalties and objective checks are unchanged.

The current merged corpus loads with3295/942/699 Train/validation/test rows and is
trainable despite legacy training_eligible=false,seed8's missing labels and zero
road goals. The previous existing coverage assertion was updated to the new
contract. No new tests or unit suite, recollection, relabeling or policy training
were run for this correction. Frozen datasets/checkpoints and non-feature files
are protected by .verification/candidate-label-readiness-20261011 snapshots.

## Train-only augmentation, 2026-10-11

The user authorized adding another random-road batch entirely to Train. The
augment CLI reuses collect_dataset and its existing full C* label generation;
PipelineSettings permits empty validation/test seeds only for explicit train-only
collection. Import still requires all three splits by default. New labels retain
their full producer provenance; compatibility ignores only batch seed lists and
split scope, retaining solver/plant/objective/sampling/other settings checks.
Encoding now streams one decision at a time. The merge concatenates already
validated NPZ rows, checks road/group disjointness, and compares every holdout
array. It writes a separate dataset; existing datasets and models are preserved.
Coverage/eligibility flags remain truthful; augmentation does not train a model.

Actual preflight confirms eight new road identities at seeds 4-11, lengths
28.63-37.97m, disjoint from all original splits, and matching C* plant/sampling
contracts. Plan: two initial states and standard/wide lattice per road, 32 chains,
two full trials per candidate, same population4/iteration1/evaluations8/30s solver
budget, near1.2/extension1.6m and authorized origin override. Original validation
942 rows/37 states/two roads and test699/35/two roads remain fixed. Output:
.verification/train-augmentation-20261011. This is a preflight/launch record;
completion, actual additions and preservation checks belong in the new run.json,
augmentation.json and verification.json. No unit tests were added or run.
The background job is running (actual Python PID74504; venv launcher PID46088).
The first two actual decisions completed 36 candidates/72 successful trials and
continued from 6.898m to 10.910m in about56s. This confirms new Train collection
and full-state continuation have started, not completion of encoding/merge.
Startup hashes for the original datasets/models,205 protected files and Git refs
are unchanged. The runner will audit again after automatic merge.
The job has now finished all32 sequences and merged successfully in35.21min:
67 new decisions,1392 candidates,2784 trials (2776 successful). Seven of the
eight new roads produced labels; seed8's four sequences all stopped at
insufficient_lattice_coverage before a decision. Endings:25 planning failures,
seven completed planned stops,zero road goals. Merged Train is3295 rows/145
decisions/11 labelled roads; validation942/37/two and test699/35/two are unchanged.
All holdout arrays, original datasets/models,205 protected files and Git refs
passed the final preservation audit; merged source/schema/hash checks load.
No retraining yet. At merge time the historical training_eligible flag remained
false,with missing-label seed8 and missing road goals recorded. The later
candidate-readiness correction above supersedes that coverage-based rejection;
the frozen metadata and collection evidence were not rewritten.
Actual artifacts: augmentation.json, verification.json, REPORT.md, dataset/ and
addition/road_coverage.png under .verification/train-augmentation-20261011.

## Sigmoid-target fit on the same frozen data, 2026-10-11

The user authorized retraining with sigmoid targets. The shared train_model now
uses sigmoid(mean(raw C* trial returns)), T=1, offset=0, after trial aggregation.
The original objective, failure penalties, frozen labels, splits and previous
checkpoint remain unchanged. The MLP uses a sigmoid output, no raw-y scaler;
decision-weighted MSE and candidate-difference MSE operate in score space.
V3 checkpoints and inference explicitly record the output space and transform;
the preceding v2 raw-return checkpoint remains readable with identical metrics.

Same 140-32-1 tanh/Adam/.005/seed 0/150 epochs. Validation score RMSE chooses epoch
49, test excluded from fitting and model selection. Test score RMSE .074287;
raw-label selected mean return .445127 versus previous .295319 and rule .279209
(oracle .484249); pairwise accuracy .797230 versus previous .524582; mean regret
.039122 versus previous .188930. Calibration RMSE in different output spaces is
not directly comparable. The six large failure labels remain, saturated to score
zero; no such candidate was selected in the frozen train/validation/test groups.

Reloaded model/source/schema/objective and finite bounded predictions checked;
reported metrics match the saved checkpoint. Frozen NPZ/metadata/records and
previous training outputs, 205 protected files and Git refs remain unchanged.
Partial-road experimental override remains explicit, training_eligible=false,
zero collected road goals. This is labelled-candidate evaluation on test's 35
existing decisions from two roads; no new policy closed-loop run or unit suite.
The public score CLI also ran on one recorded decision; scores/argmax agree with
the frozen-feature predictions and declare predicted_score. The existing model
learning/roundtrip check was updated to the new target space without adding or
running tests.
Artifacts: .verification/eight-road-sigmoid-training-20261011/model.json,
model.training.json, evaluation.json, training.png, comparison.png,
comparison.json, verification.json and REPORT.md.

## First policy fit on the frozen eight-road data, 2026-10-11

The user authorized starting training despite the incomplete road-goal coverage.
The shared train_model now supports an explicit allow_incomplete_coverage option;
the new train CLI reuses it without collecting/relabeling. Default coverage refusal
remains. The exception is limited to missing goals with labelled roads, legal
initial states and all three splits present; source/label/objective/hash/schema and
split checks remain. Original dataset metadata stays training_eligible=false.

Actual run: 140-32-1 tanh MLP, Adam/.005, seed 0, 150 epochs; 1903/942/699 rows
at 78/37/35 train/validation/test decisions. Validation RMSE selects epoch 14;
test never participates in training or checkpoint selection. The saved model marks
partial_road_coverage_experiment and retains coverage and the explicit override.
Test label selection: learned .295319 versus rule .279209 mean return; oracle
.484249; learned pairwise accuracy .524582 and mean regret .188930 versus rule
.205040. This is a weak first fit, not evidence of completed roads or robust control.
Five train labels and one test label carry approximately -500000 failure penalties;
validation labels have no such penalties, while validation prediction RMSE is
5783.16. The extreme training targets affect score calibration; no failure labels
were dropped or reward terms changed.

Checkpoint load/source/schema checks and finite predictions passed. Frozen NPZ and
metadata hashes, 197 protected non-training files and Git refs remain unchanged.
No closed-loop evaluation or unit-test suite was run in this training turn.
Artifacts: .verification/eight-road-training-20261011/model.json, model.training.json,
evaluation.json, training.png, verification.json and REPORT.md.

## New eight-road collection started, 2026-10-11

The user requested deleting old data before recollection. Nine obsolete training,
model and contract-check directories (100 files, 448.25MiB) were removed from this
feature's .verification directory after resolved-path checks. Diagnostic reports
and figures remain; historical dataset/model output links below no longer resolve.
The active output is .verification/eight-road-collection-20261011, with cleanup.json,
run.json and collection.log. It requests eight fixed random roads at train seeds
0/1/2/3, validation 100/101 and test 200/201; two physical initial draws and
standard/wide profiles per road, 32 random chains until goal/STOP/explicit failure.
Native Frenet export and the authorized origin-support override are enabled at
1.2/1.6m limits; vehicle, 80-degree camera, road width and .5m cone spacing retain
the current defaults. Two actual C* trials per candidate, population 4, one CEM
iteration and at most 8 evaluations match the prior full collection budget.
This section records launch, not successful completion. Final metrics/eligibility
must come from the new summary; no policy training was requested for this run.
The collection subsequently finished 32 chains / 150 decisions / 3544 candidates /
7088 trials (7082 successful) in 136.58min plus 63.03s encoding. All eight roads
have labels but zero sequences reached the road goal (25 planning failures, seven
planned stops); metadata remains incomplete_coverage and training_eligible=false.
The later explicit experimental training authorization is recorded above.

## Native Frenet training export, 2026-10-10

The user selected the smaller change: consume the existing native Frenet output
instead of truncating a Cartesian reference and changing its timing/labels.
Only the offline training factory bypasses the Cartesian approximation stage;
the native caller still performs all original Frenet body/obstacle/motion/range
checks. C* fully executes the original supported curve, restores its terminal
snapshot, then samples the next native candidates. No reference truncation,
new fitting algorithm, objective change, original env edit or live/native change.
References mark frenet_native, path=None and output_errors=None.
--require-cartesian-compatibility restores the legacy adapter in the same pipeline
for comparison. Export semantics native_frenet_export_v1 enter the source signature;
old corpus/checkpoint signatures are rejected rather than silently mixed.

Exact replay of the illustrated 20.344m failure: old mode has no valid Cartesian
output; native mode returns 9 candidates. Candidate 1|-0.125|3|0.2|0 completes its
full 2m reference in 10.25s with .009693m endpoint error and all step constraints.
At its actual terminal progress 22.354m, the next rollout returns 39 candidates.
The public highest-rule-score visualize CLI then executes 17 complete segments /
3418 steps from the original start, reaches 34.362m on the 35.081m road, and
explicitly stops at insufficient_geometry (goal 34.584m not reached). No complete
eight-road dataset claim. Full-path and segment continuation behavior are verified
through the actual plant; no unit-test suite was added or rerun for this change.
Artifacts: .verification/frenet-export-20261010/case.json and rule-evaluation/.
The real collect CLI also froze three straight scenes with one decision each:
54 candidates / 54 successful full-C* trials, imported train/validation/test NPZ,
96.60s collection plus 1.26s encoding. Native references have path=None and no
fabricated Cartesian errors; dataset loading validates the new export signature.
This is contract verification, not eight-road training or a policy-training run.
Final scope check: 197 protected file hashes and all Git refs unchanged. Detailed
report and continuation plot: .verification/frenet-export-20261010/REPORT.md.

## Temporary offline origin-support override, 2026-10-10

User-authorized --allow-unobserved-origin is now available in collect/run and
visualize, default off. It applies only to course_simulation with an explicit
clear-start assumption, after observed_origin_seen is true, and only for
near_field_unobserved with validated, aligned, two-sided forward_geometry.
The offline query adapter admits that bounded geometry to native planning;
raw estimation remains invalid, history/observed-origin/STOP state is retained.
Native gap/extension, freshness, frame/time, body/obstacle/motion checks and actual
C* step constraints remain. No native planner, original env or live YAML edits.
Diagnostics and references explicitly mark assumed origin support and override use.

The saved seed 0 failure was replayed exactly: max state error 0; raw observation
unchanged, history age 2.4s and support beginning at .404608m. Default rejects;
enabled returns 27 marked candidates. Smaller near/extension bounds, stale road,
single-side or unaligned forward geometry still reject.
Eight paired uniform-random full-C* sequences at 1.2/1.6m support limits completed
24 segments / 46.496m with the guard versus 53 / 104.660m with the offline override.
Both completed 0 roads. The accepted common-prefix terminal states match exactly;
all 77 accepted segments completed their references and passed each-step constraints.
Remaining endings include no_feasible_lattice_trajectory, no_valid_cartesian_output,
insufficient_body_supported_horizon, planned STOP, and one insufficient_geometry.
The experiment took 174.22s, including 48.27s guarded and 124.30s enabled.

The public visualize CLI additionally executed the rule maximum on seed 0 with
the option: 10 complete segments / 2009 steps, final progress 20.344m, explicit
no_valid_cartesian_output failure. Two candidate probes, PNG and HTML generated.
An actual blocked collect CLI recorded the flag and rejected empty/incomplete data.
No complete labelled corpus or policy training was produced. Scope checks confirm
197 protected hashes, vehicle/dynamics and Git refs unchanged. This did not rerun
the unit-test suite; the native ROS fast gate limitation above remains.
Evidence: .verification/origin-support-20261010/REPORT.md, replay.json, chains.json,
verification.json, comparison.png, random_chains.png and rule-evaluation/.

## Environment settings comparison, 2026-10-10

Fetched origin/feature/rule-lattice-planner: the upstream tip remains 5068e9b.
Kept the notebook .25m width / .495m body length and all vehicle dynamics,
80-degree camera, 24 detections and original sensor timing. No native planner,
estimator, original planner_sim or live configuration edits were made.
The training package now exposes cone_spacing_m in road settings and explicit
offline max_near_gap_m / max_extension_m options shared by collection and evaluation.
Original defaults remain .5m cones and .9/1.3m support limits.

Six configurations were probed with the same eight road geometry seeds and 16
identical legal initial states, at standard/wide profiles (32 queries each).
Admitted initial queries: original 12; .25m cones 22; .9-1.0m road 12;
1.2/1.6m support limits 29; dense cones plus support limits 28; all three 28.
All 48 road/config passage checks passed with the unchanged inflated vehicle.
Each configuration then ran eight actual full-C* uniform-random candidate chains
using initial 0/wide. Original: 9 complete segments; dense: 11; narrower: 6;
larger near limit: 24; dense plus near: 15; all three: 19.
All 84 accepted segments completed their references and passed step constraints.
The larger near setting executed an initial segment on 8/8 roads and reached a
second valid decision on 7/8, but no configuration completed any road (0/48).
Later near_field_unobserved and finite end-support limitations remain visible.

These are diagnostics, not a labelled corpus: only randomly selected candidates
were executed. No policy was trained and no costly all-candidate collection was
restarted against a configuration already observed to fail whole-road coverage.
Static probes took 25.73s; continuous diagnostics took 129.97s. Defaults stay
unchanged. Evidence: .verification/environment-settings-20261010/REPORT.md,
comparison.json, chains.json, comparison.png, random_chains.png and verification.json.
Scope audit confirmed 197 protected file hashes, Git refs and vehicle/dynamics
unchanged. No physical vehicle activity. The prior 53-test result below precedes
these option additions; this experiment did not rerun that test suite.
The public RoadEvaluation entry additionally executed the rule maximum on seed 0:
3 complete segments / 618 steps, then an explicit invalid_estimates_or_boundaries
fault. Two point probes and PNG/HTML outputs were produced with the same support
settings. An actual collect CLI blocked-scene check recorded 1.2/1.6m options,
reported incomplete coverage with dataset=null and produced no model. The required
ROS fast gate was attempted and stopped at the still-missing pinned dream_interfaces.

## Eight-road coverage bug audit and corrected safety checks, 2026-10-10

The earlier v5 corpus below is retained as diagnostic history, not current
training evidence. Train seed 0's nine decisions are four chains (3/3/2/1),
154 alternatives at nine states. Their merged visited progress is 9.369m,
26.7% of the 35.081m road. The other three train geometries have no labels.

Reproduction found two physically illegal initial draws: seed 0/I1 extends
its inflated rear body .0306m before the finite road, and seed 2/I2 intersects
an obstacle. The old C* predictor/trial checks omitted finite start/end planes.
V6 now enforces those bounds for the complete body at initialization and every
prediction/replay step. Initial progress is drawn once from conservative physical
free-space intervals, with a body-aware start and obstacle cross-sections
excluded. No planner/perception acceptance filter or replacement draw is used.
Twenty-four seeded initial-draw checks passed. Warmup termination is explicit;
road evaluation checks initial safety before declaring the goal reached.

Candidate enumeration now queries a planner copy rather than prematurely
committing the native rule choice; actual selected execution commits the path.
The observed-origin guard is preserved. Coverage figures draw actual intermediate
trace points per chain, with failures, goals, decision counts and visited fraction.
Collect/run share a coverage gate; missing requested road labels or missing goals
in until-road-end mode save diagnostics, mark data training_eligible=false, and
stop training. Independent train_model also rejects that metadata. V5 internal
datasets/checkpoints are rejected instead of silently reusing incomplete safety.

The 18 startup failures have common forward support .939-1.190m versus the
native .9m near-gap limit. A diagnostic-only 80/120-degree camera comparison
with identical old poses/actions changed initial candidate availability from
14/32 to 30/32, with physical state replay error 0. Runtime remains 80 degrees;
planner/perception modules were not changed. The eight subsequent
invalid_estimates_or_boundaries cases are four near_field_unobserved, three
insufficient_geometry and one single_side_width_unknown. Three of the four
near-field cases would admit a fresh planner, but resetting observed_origin_seen
would bypass a native safety guard and was not used as a repair. All three STOP
endings had STOP-only candidate sets, not random picks from mixed cruise/STOP.

A separate end-support probe starts at road_length-2.5m with zero lateral/heading
error, .2m/s and diagnostic 120-degree FOV. Seven roads provide candidates whose
maximum reference endpoint is .210-.564m short of the current goal; seed 100
has no valid Cartesian output. The shared last-cone progress is .157-.517m short
of physical road end. This identifies an unvalidated goal/support configuration,
not proof of global impossibility. No goal shortening, tolerance relaxation,
ground-truth policy input or top-reward transition was introduced.

Final 53 tests passed in 420.034s, including native actual CLI, complete-path
labels, whole-road visualization and the new safety/coverage/query/rendering
regressions. A first regression run exposed the old 1.4m obstacle example's
insufficient .6m transition space after correcting the initial rear footprint;
the controlled example was moved to 1.6m and the whole suite rerun successfully.
Source/vehicle parameters are unchanged. Evidence and corrected figures are in
.verification/eight-road-bug-audit-20261010. The ROS fast gate still stops at
the absent pinned dream_interfaces checkout. No physical vehicle activity.

## Historical fixed eight-road continuous collection, 2026-10-10

The single shared collector now supports --until-road-end, with a separately
reported 100-segment ceiling. Collection and evaluation use the same body-safe
road goal, road_length minus front_extent minus .2m. The collect command only
collects, validates and freezes data; it refuses to overwrite an existing corpus.
Random choice, complete C* execution, full terminal snapshot inheritance and
retry-only-after-completed-illegal-endpoint semantics are preserved. Planner,
solver, replay and serialization phases are timed; each episode checkpoints its
real ending. No native planner/perception or vehicle configuration was changed.

The package README's eight-road command exited 0: train seeds 0/1/2/3,
validation 100/101, test 200/201, two stratified initial draws per road and
standard/wide profiles. It used the previous smoke run's 8-evaluation C* budget,
two trials per candidate, and complete spatial/timed STOP paths. Benchmark
collection ran without concurrent regression tests.

- Eight generated geometries, 32 attempted sequences; five geometries provided labels.
- 30 decisions, 537 candidates; train/validation/test rows 154/190/193.
- 1074 trials, 1072 complete successful executions; two execution-constraint
  violations remain explicit failed labels.
- Collection/labeling 909.2849s (15.15 minutes); encoding 9.3930s.
- Full trial replay 533.9503s (58.7%), C* solving 345.7088s (38.0%);
  average 1.6933s per candidate. The maximum decision took 84.3990s.
- Sixteen adjacent decision pairs were checked for identical inherited terminal
  vehicle state/control and observation timestamp. Nominal replay max state error 0.
- Longest sequence four complete segments, approximately 8m. No segment ceiling hit.
- Zero sequences reached the road goal: 29 native planning failures and three
  completed planned STOP references. These are not successful whole-road drives.

The 29 planning failures were insufficient_lattice_coverage (18),
invalid_estimates_or_boundaries (8), insufficient_body_supported_horizon (1),
and no_valid_cartesian_output (2). Train roads 1/2/3 produced no candidates from
either initial draw, so the current train rows come only from road 0. Initial
draws were not redrawn to select an accepted start. This is a measured partial
coverage corpus, not eight complete road sequences or sufficient training coverage.
The measured cost cannot be reported as the cost of fully traversing eight roads.

Evidence: .verification/eight-road-collection-20261010 contains immutable measured
decisions, the frozen NPZ/metadata, collection log, phase/road analysis, REPORT.md,
road_coverage.png and collection_timing.png. The report figures were inspected.
No policy was trained in this collection run; repeated epochs can consume the
same saved dataset without new C* solves or simulator labels.

All 45 training/integration cases passed in 388.557s after collection, including
multi-segment terminal-state inheritance, until-road-end versus an explicit
segment-ceiling failure, overwrite refusal and the existing run/visualization
CLIs. Frozen data hashes and current pipeline source fingerprints match. The
scope audit found no changes to 201 protected files, the copied fragment's two
upskill source files or any of 15 Git refs. The required ROS gate was attempted
with local Git Bash and stopped at the missing pinned dream_interfaces path;
no ROS or hardware acceptance is claimed.

## Full spatial-path scoring, passable random obstacles and visualization, 2026-10-10

The shared target is now cstar_full_path_loss_v2 / negative_expected_full_path_cost.
Each cruise keeps the native authored time prefix, then follows the original
Frenet geometry at its terminal physical speed until the entire admitted spatial
path ends (normally 2m). STOP executes its complete timed stop and settling hold.
Duration is aligned to .05s and checked against the explicit 60s maximum. A 3s
prefix is never passed off as the full spatial score. Costs use time-average
tracking/control terms, positive along-reference speed utility (weight .5,
scale .2m/s), terminal position/speed costs, and endpoint completion constraints.
Position tolerance is .15m, stop speed tolerance .02m/s. Optimization, actual
trial labels and evaluation share that definition; old 3s checkpoints are rejected.

Random generation adds 3-6 rectangles (.20-.35m long, .16-.28m wide), at least
3.5m apart, with 3m clear end zones. One random obstacle side per road preserves
an opposite-side connected corridor, minimum .55m. Native geometry checks an
inflated .35m full car along the reserved route at .1m intervals; failed layouts
are rejected. Ten seeded checks passed. This does not prove continuous swept
safety or guarantee that the native planner finds the geometric corridor.
Obstacle/speed annotation variants share geometry IDs for partition isolation.

The actual full-path training command in the current package README exited 0.
It produced 156 candidates in eight decisions, 312 successful nominal/perturbed
full env trials, durations 7.05-20.20s and maximum nominal replay error 0. Train/
validation/test are 78/36/42 candidates. Collection took about 349s while the
integration suite ran concurrently. The 60-epoch checkpoint reloads and scores;
held-out RMSE .183702, pairwise accuracy .634033, learned/rule regret
.006855/.004376. The corpus establishes pipeline operation, not ranking superiority.
Only layout changes followed collection; measured records were re-encoded and
deterministically retrained to retain current code fingerprints, with matched
evaluation repeated. Native inverse-heading transient and curvature feedforward
are optimization initializers only; constrained CEM remains approximate.

evaluation.py shares complete candidate execution between run and visualize.py.
Evaluation strictly executes argmax of the requested rule/policy numeric score,
including overriding native near-best tie ranking; training transitions remain
uniform random. visualize.py accepts given road JSON, exports the complete actual
trace/reference/speed/candidate figures and serves a localhost interactive page.
The page adds arbitrary road positions, restores actual execution history when
available, or generates declared local initial observations, and shows admitted
and rejected native geometries with reasons. It never fills missing perception.

The 16m given-road example with two randomly placed obstacles reached the body-safe
road endpoint at progress 15.667m: eight complete segments, 1038 actual steps,
51.9s simulated driving, no collision/offroad, about 5.5s wall time with 8-evaluation
solver budget. Its evaluation.png and point-candidate figures were inspected.
Adding a 7m position through the page returned 31 admitted and 10 rejected
candidates; score maximum selection and real-state display were verified.
The broader 35.081m random curved case stopped at 12.315m with native
no_valid_cartesian_output; that failure remains visible rather than showing a
successful whole-road drive. Original planner/perception admission was preserved.

Full-path scoring costs more: sampled 7.75-7.8s candidates took .3-.4s at 8 evaluations
and about 1.5-2.1s at the 64 budget (40 actual evaluations), plus env replay.
Default large collection can take tens of minutes to hours; no 3s truncation was
introduced as a shortcut. All 43 tests passed, including complete spatial endpoint,
speed preference/standing-still rejection, passage checks, full actual CLI and
visualization/inspection export. The single training test file is retained.

## Historical uniform 3s C* segment collection, 2026-10-09

The following 3s target and completion evidence are historical. They are superseded
by full spatial-path duration and the versioned objective above.

Collection now follows the requested state-transition rule: independently label
all admitted candidates, uniformly sample one without consulting policy/reward,
execute its complete C* horizon (default 3s/60 steps), and inherit the actual full
terminal env snapshot. Position, body/motion state, steering, hidden direction
latch, last applied action, sensor queues, estimator history and RNG are retained.

Only a completed C* terminal state outside the spatially legal region triggers
another candidate draw from the same original snapshot, without replacement.
The endpoint check covers the inflated full body, lateral boundaries, finite road
endpoints and obstacles. Low reward and tracking error do not trigger resampling.
Solve failure or incomplete execution has no full successor: it ends that episode
with an explicit fault, rather than trying a higher-scoring candidate or redrawing
the initial state. Bounded exhausted illegal-endpoint attempts are also explicit.

Initial positions are drawn once and no longer filtered by native planner
availability. Missing perception/candidates remain reported failures. Earlier
figures' accepted starts meant valid native planning after camera warmup; they
were not C* terminal legality checks. New coverage figures distinguish unfiltered
Initial draw from Legal C* endpoint. Rule/learned choice remains only in inference
and closed-loop evaluation; neither selects collection successors.

The same single run entry/test file remains. Checks cover full 3s state/action
continuity, retained hidden/sensor/RNG history, reward-independent random draws,
only completed illegal endpoints causing retries, bounded retry exhaustion,
unfiltered startup, finite-road endpoint guards and actual CLI execution.

The complete run in .verification/segment-random-run exited 0 using the current
README command (seeds 0/1, 100, 200; two initial draws/profiles; two C* segments;
two trials; 4/1/8 solve budget; 40 epochs; six evaluation steps). It produced 16
decision groups, 188 candidate labels and 376 successful execution trials. All
16 sampled successors executed the full 60 controls. Eight actual segment links
have exactly zero state and previous-action mismatch; their time intervals are
3s within 9e-16. Fourteen of 16 chosen candidates were not the maximum measured
reward candidate, and legal endpoints were retained regardless.

All eight requested initial positions were kept. Eight of the 16 profile episodes
had no native candidate due to insufficient coverage and remain explicit empty
episodes, without an initialization retry. No completed illegal endpoint occurred
in this nominal smoke run; the only-illegal-endpoint retry predicate and bounded
exhaustion are exercised by controlled unit cases. No C* safety constraints were
removed to create invalid endpoint examples. Nominal prediction/replay error is 0.
Collection took about 67s; current dataset/model code hashes match final source.

All 38 training checks passed on final source. Scope audit again confirms 201
protected non-training files, the upskill sources and 15 branch refs unchanged.
This small model has test RMSE .064113, pairwise accuracy .332298 and learned/rule
regret .123634/.011571; it establishes sampling/state continuity, not improved
model quality. Existing ROS provisioning/physical qualification limits remain.

## Historical random roads with first-action collection, 2026-10-09

The following run describes the earlier sampler, superseded by complete random
C* segment transitions above. Its accepted-start filter, behavior-model option,
decision stride and code fingerprints are historical, not current defaults.

The same feature now adapts the upskill notebook's clean-random-road-distribution
cell and Dream Gym 0.2.2 random_generation.py fragment into roads.py. Attribution,
source hashes and adapted settings are recorded; the source project is never
read at training runtime. The existing pinned Dream Gym 0.4 accepts the generated
nested road/lane/cone definitions. Vehicle dynamics/integration are unchanged.

Default roads have 8-10 straight/arc segments, a 6-8m first straight, 2-4m other
straights, 4.5-8m radii, 20-35 degree arcs, and 1-1.2m width. Initial progress is
stratified across [.12m, road_length-3m]; lateral/heading/speed also vary. The
native lattice local horizon remains 2m (the old full straight road was 10m).
The heading bound reduces loops without proving non-intersection or continuous
curvature. Random obstacles are not generated; deterministic obstacle/blocked
cases remain available through the same pipeline.

Collection combines rule/random/optional checkpoint selection with standard,
conservative or wide sampling profiles of the same native planner. Every actual
step consumes fresh env feedback, replans, solves C* and applies its first action.
Decision stride only controls the expensive all-candidate labeling frequency.
All variants of a road share its geometry hash and one data split. The checkpoint
fingerprint is retained when learned collection is enabled; native rule baseline
IDs are preserved independently of the behavior choice.

Initial perception/planning rejection triggers bounded deterministic retries
within the same progress stratum. Failed initialization remains an empty
collection episode or explicit failed evaluation, not a fabricated observation.
This conditions the observed data on current native perception/planner coverage.
The default seed set is still finite; epochs do not generate new roads.

The random-road run exited 0 with:

    python -B -u -m offline.lattice_training run --output .verification/random-road-run --kinds randomized --train-seeds 0,1 --validation-seeds 100 --test-seeds 200 --starts-per-road 2 --rollout-behaviors rule,random --planner-profiles standard,conservative --decisions 2 --decision-stride 2 --trials 2 --population 4 --iterations 1 --max-evaluations 8 --epochs 60 --evaluation-steps 6

Its four roads are 28.233/31.353/35.081/36.433m long. Of eight requested initial
positions, six were accepted, spanning .190-32.141m. Two exhausted native coverage
retries, producing eight empty episodes across the actor/profile combinations.
The 24 accepted episodes produced 48 decision groups and 496 candidate rows:
train 248/24, validation 160/16, test 88/8. All 992 nominal/performance trials
passed recorded constraints, and maximum nominal prediction/replay error was 0.
Collection took about 347s with the deliberately small numerical solve budget.

The current export reloads and scores successfully; its dataset/model code hashes
match source. Whole-road coverage, training and local execution PNGs were visually
inspected; the coverage titles were adjusted and the unchanged measured records
were re-encoded/retrained with matching provenance. Matched closed-loop evaluation
was repeated against that final export.

Test RMSE is .013394 and pairwise accuracy .770270. Test learned/rule regret is
.007892/.008934, while validation learned/rule regret is .010856/.005136. The
small corpus therefore does not establish a general ranking improvement. Its
accepted training starts happen to be on straights while validation starts are
on arcs; train/validation RMSE is .008447/.067563. A separate default-seed start
probe found 13/16 initial states accepted on training seeds 0-7, including curved
starts on seed 2. Larger road coverage still requires generating actual labels.

The two selectors each have three matched test evaluations: one initialization
failure, one six-step actual execution, and the blocked planning failure. The
executed case has rule/learned return -.00002624/-.00005074, with no recorded
collision or road departure. Both failure cases retain -1e6 encoding, so the mean
return is dominated by failures. Six steps is a smoke check, not a completed
long-road drive or physical safety qualification.

An existing C*-trained checkpoint was also used for learned data collection on
road seeds 2/100/200. It produced five decisions and 58 candidates; four decisions
had behavior selections differing from their separately retained native rule
baseline. Its model fingerprint is stored in learned-rollout-check.collection.json
under the same ignored output directory. This is a collection check, not another
training entry point or a second implementation.

All 33 tests in tests/test_lattice_training.py pass. New checks cover native road
construction, reproducibility, long-road start stratification, retained notebook
physics, different candidate profiles with unchanged local horizon, deterministic
retries, retained failures, base-road split isolation, and actual per-step feedback
producing different subsequent states under rule/random behavior.

Scope audit found all 201 protected non-training files, both upskill source files,
and all 15 existing branch refs unchanged. The pre-existing tracked changes from
earlier feature work remain preserved. The original ROS dependency limitation
below still applies.

## Native env, C* and policy training, 2026-10-09

feature/lattice-training-pipeline contains the training C* solver and calls the
existing planner_sim through reset/fork/step. The flow is native observations ->
actual compatible lattice candidates -> constrained native-plant optimization ->
full env execution -> negative shared tracking-loss labels -> supervised scoring
policy -> ranking and actual closed-loop evaluation -> model/data/report export.

C* performs multi-start CEM direct shooting over all 120 normalized control
variables for 3s/.05s. Each prediction clones the full pinned BicycleModelDynamic
instance including hidden direction latch, invokes its integration kernel and
checks full body/road/obstacle and operating-domain constraints. Every successful
solve is replayed in the full env. Predicted loss never replaces measured labels.
Solver diagnostics report evaluation/time limits and approximate/failed status;
global_optimum_proven is false. No PI controller was restored or pretrained.

Performance trials record seeds, gains/bias and request-delay history. All
candidates at a decision fork the same full state; nominal prediction/replay
maximum state error was 0 in the verified corpus. Plant/Notebook/integration
identity, code hashes and source settings are attached to the model/data.
Current 140-feature inputs exclude future/ground-truth/controller state.

The earlier four-template command exited 0 (historical run before random-road
collection and its expanded defaults):

    python -B -u -m offline.lattice_training run --output .verification/cstar-training-run --kinds straight,left_curve,right_curve,obstacle --train-seeds 0,1 --validation-seeds 100 --test-seeds 200 --decisions 2 --trials 2 --population 8 --iterations 3 --max-evaluations 28 --epochs 120 --evaluation-steps 60

It produced 241 real candidates / 28 decision groups: train 111/14, validation
74/7, test 56/7. All 482 nominal/performance execution trials passed recorded
constraints. Outputs in ignored .verification/cstar-training-run include actual
decisions/configurations, hash-checked NPZ, model.json, ranking/evaluation reports,
closed_loop.json, summary.json and training/closed-loop PNGs. Saved pipeline code
hashes describe that earlier implementation; the exported model reloads and
scores against the same unchanged native planner source.

Held-out return RMSE is 0.038049 and pairwise accuracy 0.577586. Learned selection
regret 0.026310 is worse than rule regret 0.023285; this initial checkpoint does
not establish a ranking improvement. Actual matched 60-step evaluations on seed
200 use straight/left/right/obstacle plus blocked. The three clear-scene returns
are rule -0.147843/-0.058446/-0.074549 versus learned
-0.022129/-0.001438/-0.011461; obstacle is -0.163747 for both. These limited
examples do not establish general driving quality.

STOP commits its original timed trajectory instead of requesting a lattice
restart mid-stop. Both obstacle episodes executed all 60 steps without collision
or road departure. With the recorded performance deviation, endpoint speed is
0.008816m/s, above the declared .005m/s stop threshold: stop_completed is false.
Blocked returns no native candidate and is explicitly a planning failure for
both selectors with -1e6 encoding. Zero simulated collisions is not successful
completion, especially for the blocked episode that terminates before actuation.

All 25 tests/test_lattice_training.py checks pass, including actual env/prediction
equivalence through braking/reverse latch, physical wheel-rate bounds, parent
state isolation, deterministic solving, full-body constraints, wall-budget
failure, persistent delay, committed STOP and the complete actual CLI. Renderer
plots were inspected. Requirements pin the tested NumPy/Gymnasium/Matplotlib
and inherit the original Dream Gym git pin; no new dependency install was needed.

The CONTRIBUTING fast command was attempted. WSL initially denied startup; the
escalated retry reached the actual script and stopped at realpath for the missing
pinned dream_interfaces checkout. No installed ROS or physical qualification is
claimed. Simulated contact/boundary checks are discrete, not continuous swept
safety proof, and offline solver/planner timing is not the live 35ms admission.

Implementation is restricted to the training package/test and its own README/
acceptance sections. Original native planner/controller source, planner_sim,
control_pid, live configuration, other worktrees and branch references are
outside this change. No commit, push, merge, release or hardware operation was
performed.

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

### Fixed U-turn validation roads and unknown-width reference, 2026-10-17

The simulation branch adds `shadow.unknown_width_reference_scale=4/3`: only a
single observed boundary with no supplied centreline and no width uses a 0.60 m
CG guide offset instead of 0.45 m. Search half-width remains 0.45 m. Known-width,
two-sided and supplied-centreline cases keep their existing rules; collision
geometry, C*, policy encoder, frozen model and historical data are unchanged.

Eight concrete JSON validation roads now cover original, half-width/half-radius,
curved-entry/variable-radius-turn/curved-exit, and half-width/half-radius double
U-turns in both directions. Per the user's final clarification both full lane
width AND U-turn reference radius are halved; straight lengths stay fixed. Current
suite version is 2, with SHA-256 checked loading. The early width-only v1
construction and runs are retained separately as a construction comparison.

`offline.lattice_training.validate_uturns` uses the existing perception, native
candidate, optional frozen-policy and C* path. It does not train, add fallback,
inject scenario width or deploy. Cold-start perception/no-centreline and full
road-entry perception runs are recorded separately. Completion/failure, actual
seven-state traces, observed boundary inputs, static road-space checks and the
original-road 0.45/0.60 m offset ablation are in the
[v2 validation report](../offline/lattice_training/artifacts/2026-10-17-fixed-uturn-suite-v2/REPORT_CN.md).
Portable checks cover the parameter's narrow scope, mirrored geometry, concrete
fixture loading and upstream availability/provenance. Exact legacy path/cost
comparison for the intentionally changed branch uses scale 1; default-scale
availability/provenance and unaffected geometry remain checked. The ROS fast
gate stops before build because the pinned dream_interfaces checkout is absent.
No ROS runtime or physical vehicle acceptance is claimed.

### Clock-driven MVP control on fixed U-turn roads, 2026-10-18

`offline.mvp21_sim.validate_roads` reuses the existing shared runtime loop,
`mvp21_policy.execution` and actual `PolicyController`. The fixed-road camera
acquires at 20 Hz with 50 ms delivery latency; estimation/control run at 20 Hz
and native jobs are submitted at 1 Hz. A causal 100 ms simulated planner service
keeps control moving between submissions/deliveries. It does not wait for a
candidate's end. Actual CPU durations are logged separately; deterministic
delivery is not a ROS executor deadline measurement.

Eight unchanged v2 road fixtures are tested from the road entrance, plus eight
separate stationary pre-bend diagnostics. Scenario width is not injected.
Learned scoring and C* control are not used. Common planner/control YAML and
owning MVP modules remain unchanged. The existing C* geometry helper is only a
passive truth monitor. Input packets, acquisition times, full seven-state plant
traces, actual control/reference source, native candidates and delayed handoffs
are retained; faults publish zero in the simulation tail without auto-resume.

Entrance completion is 3/8 with baseline use (strict native-only 0/8). Three
runs contact a physical obstacle; two double-turn runs stop after the rolling
execution check detects a known boundary collision and the single-side baseline
is unavailable. All eight pre-bend starts stop before submitting the first job:
the actual node submits only after a successful control action. Startup speed
peaks at about 0.776 m/s despite the unchanged 0.2 m/s target.

The [runtime report](../offline/lattice_training/artifacts/2026-10-18-mvp-runtime-validation/REPORT_CN.md)
and interactive viewer separate these failures from the earlier C* training
loop. Recorded 50 ms control/camera periods, 1 s submission periods, delivery
causality and zero-action tails pass all sixteen trace audits. Portable checks
also replay logged commands through the real stateful MVP PID and reproduce
its actions exactly. The required ROS gate still stops before build because
the pinned dream_interfaces checkout is absent. Hardware acceptance: not run.

### Frozen-policy runtime evaluation, 2026-10-18

The user authorized frozen learned-policy selection in offline visualization and
evaluation. Known perception/branch/chain-direction/partial-output limits are
recorded in SIMULATION_LIMITATIONS_CN.md and remain unfixed. ROS/default native
selection, common parameters, owning MVP algorithms, input encoder, model and
historical data are unchanged. The optional simulation worker scores exactly
one admitted native candidate per model call using the submission-frame current
state, estimator road and obstacles. Authored geometry is preserved; native cost,
history, truth, width injection and speed curves do not enter the model.

Eight fixed roads at 20 Hz perception/control and 1 Hz generation/ranking have
0/8 entrance completion versus the preserved same-condition native baseline's
3/8. Eight pre-bend starts stop before any policy inference. Independent replay
verifies 3,387 delivered candidate inputs/scores/argmax and 1,409 stateful shared
MVP PID action frames; no policy inference error occurred. The existing baseline
is reused only after exact module/parameter/environment/initial-state checks.

The fixed 100 ms causal service model remains explicit. Actual combined native
and ranking CPU wall time exceeded 1 s in 11 deliveries; this does not qualify
real 1 Hz execution. The training labels use C* at 0.2 m/s while evaluation uses
actual MVP control. No retraining or tuning is hidden in this regression run.
The comparison viewer retains three-layer perception and per-candidate scores.
Evidence: offline/lattice_training/artifacts/2026-10-18-current-frequency-policy-evaluation/REPORT_CN.md.

### Archived random-road runtime evaluation, 2026-10-18

The current acceptance target is road integration, clocked replay and shared
runtime verification. Per the user's latest direction, learned-policy quality
is observational only and is not a target for this round; no tuning/retraining.

Restored exact road/obstacle specifications from the first new-protocol batch and
expanded collection archives, including empty collection episodes. The 28 old
scene IDs contain 33 physical variants because five obstacle revisions reused
scene IDs. All variants retain their original split group; no model was trained.

Ran 132 entrance cases: 33 variants, original perception / controlled missing
centerline, native / frozen policy. Camera and actual MVP PID run at 20 Hz;
generation and policy selection run at 1 Hz with the same simulated delays and
shared execution/fallback. No scenario width prior or main/ROS policy enablement.
Original perception reaches the mission goal in 22/33 native and 10/33 policy
runs, allowing actual baseline fallback. Both missing-centerline groups stop
33/33 before first planner submission/inference; these are startup gate failures,
not evidence that the learned scorer cannot encode a missing centerline.

55 relevant portable tests pass. The ROS fast gate stops at preflight because
the pinned dream_interfaces checkout is absent. CPU timing is measured separately
from simulated delivery and is not a real-time qualification. Independent input,
score, argmax, PID and causal-clock replay and browser evidence are recorded in
the new report. Historical fixed-road raw evidence and trained weights remain
unchanged.

Evidence: [random-road report](../offline/lattice_training/artifacts/2026-10-18-current-frequency-policy-evaluation/random-road-evaluation/REPORT_CN.md).

### Configurable validation workspace and replay index, 2026-10-18

The new offline CLI is `python -m offline.mvp21_sim.validation`. It initializes
saved road fixtures, ingests additional JSON roads, runs the same shared runtime,
indexes immutable records, serves a data-only replay and watches source/config
changes. Defaults reproduce the previous runtime frame-by-frame. Module choices
are observed/perfect perception, current native/fixed single straight planning,
native/frozen model selection and actual MVP/existing C* control. Independent
integer physics clocks default to 20/20/1/1/20 Hz for physics/perception/planning/
selection/control; changing them is explicit offline configuration.

64 portable tests pass, including default action/state/job parity, independent
clocks, lower-frequency held C* control, original single-candidate geometry,
zero-step failure snapshots, road ingestion and immutable re-indexing. All 16
module combinations produced inspectable records with causal-clock checks;
solver/mission failures remain failures. A new short road reached its mission
goal using the default runtime, with baseline fallback retained. A live config
change triggered fresh-process C*/single-line validation and a different actual
trajectory; restoring defaults generated a new MVP record which the browser
selected automatically without manual refresh. No policy-quality target,
retraining or ROS enablement. The ROS gate still stops at missing pinned
dream_interfaces preflight. Evidence is in `offline/mvp21_sim/workspace/evidence`.

See [workflow](../offline/mvp21_sim/VALIDATION_WORKFLOW_CN.md). The working-tree
bundle includes current uncommitted source and official evidence with per-file
hashes; no Git commit/push or release is performed.


### Local ARM64 ROS Jazzy environment, 2026-10-20

An isolated Colima profile on Apple Silicon runs the official Ubuntu Noble ARM64 ROS Jazzy image, fixed by digest in `.verification/ros2-jazzy/base-image-ref.txt`. The exact `dream_interfaces` pin and current policy build successfully. The CONTRIBUTING fast gate passes: 353 reported tests (344 pytest cases plus 9 CTest groups), zero errors/failures/skips, and installed launch arguments verified. The test-only lattice fixture now locates installed config through the ament share directory, retaining source-tree lookup for portable runs; no controller/planner behavior or YAML changed.

The real generated message types import, two synthetic LaserScan messages round-trip through MCAP with acquisition stamps and ranges preserved, and 8 obstacle-stop monitor checks pass in ROS Python. Sixty affected portable fixture tests pass on Mac. A stop/restart retained compiled interfaces. Receipts and XML are under `.verification/ros2-jazzy/`; usage is in `tools/ros2_jazzy/README_CN.md`. This confirms a local synthetic ROS development environment, not ROS-node integration of the new obstacle-stop monitor, live-sensor calibration, deployment, real-time qualification or physical braking. The VM was stopped after verification to release memory.


### Opt-in LiDAR stop through ROS, 2026-10-20

The ROS node now applies the shared latched ObstacleStopMonitor before planning, startup motion priming, terminal commands or controller fallback. Defaults remain disabled; the common YAML is unchanged. Source freshness and TF availability are checked by the independent supervisor. Only a fresh explicit operator resume resets the trigger. The estimator and gate share one snapshot time, and post-calculation freshness is checked again before publication. Bounded initial alignment holds zero without running a planner or offering native jobs.

The full synthetic ROS gate passes 380 reported tests (369 pytest cases plus 11 CTest groups), no failures/errors/skips. Seventeen new ROS checks include a seven-scenario real DDS/timer run using actual LaserScan, static TF, wheel/IMU/cone messages, current common MVP2.1 native/baseline execution and normalized action subscriptions. The central case used native execution, continued at 16 cm, requested zero at 14 cm, stayed zero after obstacle removal and resumed only on request. Dropouts, failed TF, stopped control timers and startup obstacles stop safely at the command/FSM layer. Tests and demos use explicit synthetic dimensions and stationary wheel telemetry; no plant dynamics or physical braking is represented.

Related Mac portable checks pass 250 cases. Four offline closed-loop cases retain all 697 prior state/action frames including the coast contact failure. Evidence is indexed by `.verification/ros-lidar-stop/acceptance.json`; review and limitations are in ROS_LIDAR_STOP_CN.md. These changes are local/uncommitted and are not a hardware deployment or physical stop-distance qualification.

## LiDAR source-timeout diagnostic candidate, 2026-10-21

Stationary car observations include a real-scan obstacle stop and latch retention,
followed by a separate source-age stop on explicit resume. The root cause of that
car timeout remains unresolved. A subsequent source/effective-parameter/log snapshot
belongs to a different policy run started at 16:14:22 Melbourne time, without the
obstacle_stop parameters; it is not evidence of the earlier timeout's cause.
The local candidate adds bounded raw/Cartesian age, check-phase and calculation-time
logging without changing stop admission, default thresholds or resume behavior.
Three additional ROS regressions cover 10 Hz scans with 100 ms source delay under
an explicit 0.25 s limit, a late-calculation stop with source diagnostics, and
diagnostic-logger failure retaining the watchdog stop. The required fast gate
passes 383 reported tests (372 pytest cases plus 11 CTest groups), no failures,
errors or skips; 41 focused portable control/monitor checks pass. The freshly
bundled candidate starts in an isolated local ROS namespace, remains zero, and
returns the intended enabled/0.25 s/0.2 m/s parameters via the parameter service.
This is a diagnostic candidate, not an established root-cause fix or driving
qualification. Gate log: `.verification/ros2-jazzy/checks/20261021-162531/verify-fast.log`.

The later 16:38 snapshot is the matching diagnostic bundle/run. It records a
0.253753087 s source age against the 0.25 s limit, 0.113141551 s receipt age and
a 0.025266238 s previous calculation. The logged stop was a genuine configured
age-limit exceedance; the record does not attribute upstream latency or scheduling.
The timing candidate adds an independent positive finite receipt-age setting
(default 0.15 s), retains the source-age default of 0.15 s, and changes neither
public YAML nor obstacle geometry. The Disabled stationary candidate configuration
uses 0.30 s source age and 0.15 s receipt age. Both deadlines remain fail-closed.
Five ROS cases replay the measured timing under old/new source budgets, verify
receipt dropout still stops with larger source allowances, and reject old stamps
despite fresh receipt; one pure validation case covers invalid receipt limits.
The required gate passes 389 reported tests (378 pytest cases plus 11 CTest groups),
no failures/errors/skips. Gate log:
`.verification/ros2-jazzy/checks/20261021-164136/verify-fast.log`.
This is a locally verified timing-budget correction candidate, not a claim of
physical stopping, a hard deadline, or resolution of the separate recorded road
support rejection. The higher-level driver/transport/executor latency remains
unattributed by the available snapshot.

## Integrated car19 LiDAR profile and invalid-scan fix, 2026-10-21

The operator reports successful physical driving and obstacle stopping with source
age 0.30 s and receipt age 0.20 s, without further timing changes. This supersedes
the earlier lack of driving evidence for that trial, but provides no measured
speed, braking distance or repeatability qualification. The feature branch common
configuration now explicitly enables the same profile: target 0.2 m/s, 15 cm
front-bumper trigger, front/rear extents 0.275/0.198 m and width 0.30 m. Parsed
values match the reconstructed successful-profile record exactly. Bare-node
defaults remain disabled; loading this branch's common YAML enables the feature.

Conversion now refuses a scan consisting entirely of NaN, negative infinity or
finite out-of-range readings instead of producing an apparently clear point set.
Raw values remain available for diagnostics; the previous Cartesian observation
is invalidated immediately. Enabled stopping refuses admission or latches zero
through the existing guard. Positive-infinity no-return rays and partly invalid
scans remain supported; this is not a guarantee of coverage in every direction.
Five added ROS cases cover running failure, startup refusal, preserved zero after
data recovery and explicit restart. Control/planning algorithms are unchanged.

The required pinned-interface fast gate passes 383 pytest cases and 11 CTest
groups (394 reported), no errors/failures/skips, including the seven DDS scenarios.
The config-loading test verifies the installed profile. Focused Mac control and
monitor checks pass 42 cases. Gate log:
`.verification/ros2-jazzy/checks/20261021-214922/verify-fast.log`.
No car was connected or changed during this consolidation. This new scan-quality
fix still needs physical revalidation. A recorded 118.5 ms calculation exceeds
the nominal 50 ms policy period; executor latency remains an open performance
issue, with pre/post-calculation freshness checks retained. The new local
`tools/package_lidar_stop.py` bundles current source and common configuration with
fingerprints, Git dirty-state provenance and an archive checksum; it never deploys.

## Static LiDAR avoidance offline experiment, 2026-10-24

Branch `ying/mvp21-lidar-avoidance` starts from preserved MVP2.1 stop snapshot
`8c75519`, with the workspace entry recorded at `9e4a9dc`. Original dirty MVP/MPC
worktrees, historical models, configuration and upstream snapshots are retained.
No vehicle was connected, deployed, enabled or changed.

The opt-in offline experiment uses native lattice generation plus smooth lateral
hold families in the current perceived road chart. Full generated-path checks
reject LiDAR/body and observed-boundary intersections and excessive curvature.
Measured-motion static-point memory retains original measurement times until
the rear passes; a fresh scan remains mandatory. Side commitment avoids changing
bypass direction. The shared control function now accepts an optional guard:
both selected-reference execution and same-tick baseline fallback are checked.
The existing emergency-stop latch has priority. ROS/default execution does not
construct the new guard and does not enable active avoidance.

Seven scenarios meet their explicitly separate acceptance criteria: clear road;
central, left-offset and right-offset bypass with return; fully blocked-road
protective stop; LiDAR dropout protective stop; and original stop-only control.
All have no obstacle contact. The central case peaks at 0.2041 m/s, finishes
0.0675 m from the centerline and retains 0.1358 m minimum plant body/road margin.
Two startup baseline cycles remain counted in each case; native-only acceptance
is not claimed. Stops retain the existing simulator-only negative braking and
at least one second of stopped hold, not a real-car braking result.

The fixture is a 5 m straight, 1.4 m wide road with an obstacle at 2.8 m; existing
mission completion is near 4.5 m progress, not terminal stopping. Explicit
offline settings are 2.5 m full-path checks, 5 Hz generation, 20 Hz control and
10 Hz LiDAR with 50 ms delay. Common YAML and learned models are unchanged.
Development trials with the closer obstacle/narrower road stopped protectively
and are retained, not relabelled as successful bypasses.

Measured host planning median is about 0.49–0.51 s in the driving cases, exceeding
the modelled 0.20 s generation period. Fixed 100 ms delivery is a discrete-event
assumption, not measured processing latency. Real-time behavior, general road
scenarios, sensor coverage, calibrated dimensions/steering and physical braking
remain unqualified. CPU optimization must precede ROS avoidance integration.

132 portable regressions pass. A separate source-bound, network-disabled local
ROS container runs the required pinned-interface gate: 395 pytest cases plus
12 CTest groups, 407 reported tests, zero errors/failures/skips. This is installed
software/default-node evidence, not enabled ROS avoidance or physical driving.
Evidence: `.verification/lidar-avoidance-20261024-first-suite/` (immutable raw
records, suite/source manifests, report, measured CPU analysis and replay) and
`.verification/ros-avoidance-20261024/` (gate log, XML and build logs).

## Opt-in ROS LiDAR avoidance deployment candidate, 2026-10-26

The node now declares startup-only lidar_avoidance parameters and enables the
guard only in the dedicated complete candidate profile. That profile uses the
existing MVP control, 20 Hz control and 2 Hz worker requests with a 2.5 m horizon.
The common YAML is unchanged. Planning/selection stays in the existing separate
worker process; current-snapshot LiDAR/road/motion checks, source watchdogs,
final zero publication and explicit admitted resume remain on the node path.
Worker errors, malformed results, motion epoch loss and reference expiry stop.

The optimized numerical sweep broadcasts pose/point and segment tests in NumPy
and is checked against the established sweep over randomized cases. Tangents
are compared against the original geometric reader. Measured near-straight
centerlines may be smoothed only with <=0.02 m fit residual; raw observed road
boundaries stay hard constraints. Sensing margin remains 0.08 m. Additional
future tracking uncertainty grows to 0.08 m over 0.65 m, rather than incorrectly
rejecting the measured starting pose for future tracking error.

A nonblocking temporary replan miss can retain a still-checked bypass, without
renewing its pose/epoch/progress or source time. Known blocking results still
latch zero. Retained paths undergo current boundaries, LiDAR, preview coverage
and the 1.5 s original plan-age deadline. Actual reference job IDs remain distinct
from the latest planning result; default non-avoidance failure behavior is kept.

The final seven-scenario suite passes with measured planner CPU time included
in causal delivery plus the explicit 100 ms transport-delay assumption. All
scenarios avoid contact; central/left/right complete with return, while blocked,
dropout and original-stop control meet their separate stop acceptance. The
central case uses 429 native and 20 baseline frames (6 at startup), not native-only
execution. Median host planning time is roughly 0.15–0.20 s; the maximum in the
final suite is 0.464 s. These are Mac measurements, not Jetson timing guarantees.
Earlier failed timing/geometry trials remain unchanged in separate directories.

156 portable checks pass, including bundle checks. The required ROS gate passes
411 pytest cases plus 13 CTest groups, 424 reported tests, zero failures/errors/
skips. The bundled node and complete candidate YAML also pass actual synthetic
DDS checks: startup zero, worker-selected guarded native control with steering,
close-obstacle stop, zero retained after clear scan, and explicit restart.
The smoke uses stationary wheel telemetry and proves messages/control requests,
not physical bypass motion. No vehicle driver or Enable service is used.

Evidence: `.verification/lidar-avoidance-ros-20261026-final/` and
`.verification/ros-avoidance-20261026/`. Delivery is a source/config candidate;
vehicle dimensions, physical stopping and hardware acceptance remain unverified.

## Task-owned MVP static bypass, 2026-10-28

Branch `ying/mvp21-lidar-maneuver`, based on preserved A2 `36dcad6`.
`lidar_avoidance.maneuver_enabled` selects a new control-owned target/task rather
than adding more competing A2 candidates. Normal operation retains native
selection. A fixed observed-road chart, target envelope, stable lateral goal,
SHIFT/PASSING/RETURN phases and rotated whole-body rear clearance own progress.
Other returns remain collision inputs but cannot keep a task alive indefinitely.
Worker results carry task/revision markers; obsolete jobs cannot replace a new
phase. The executed path and baseline fallback are checked with current LiDAR
and bounded original-age road support. Hard stops still require explicit resume.
Bare/default configuration retains legacy A2 behavior for existing replays;
new deployment packages opt in, fixed left, left blue/right yellow, 0.20 m/s.

Validation: 115 portable checks pass, including the original MVP compatibility
and bundle tests; 38 focused final-reference checks pass after the final gate
addition. Full installed Jazzy gate: 425 pytest cases + 14 CTest groups = 439
reported checks, zero failures/errors/skips. An actual standalone bundled node
and complete YAML pass synthetic DDS startup zero, dedicated maneuver worker
selection, near-obstacle stop, retained zero on clear scan and explicit resume.
DDS smoke uses stationary wheel messages; it is not moving-car acceptance.

An observed-perception closed loop on an 8 m, 1.8 m wide straight with a 0.2 m
static obstacle at 4 m completes NORMAL/SHIFT/PASSING/RETURN/NORMAL, with measured
planner CPU plus 100 ms transport delay. Simulation body dimensions explicitly
match the provisional delivery rectangle (0.275/0.198/0.30 m). No obstacle
contact; peak speed 0.2012 m/s; maximum lateral excursion 0.4541 m; minimum body
road margin 0.2883 m. Seven initial baseline frames are retained and reported;
this is not native-only execution. Simulation longitudinal settings are explicit
and differ from the real-car drive profile. Negative simulated braking is not
packaged. Target unseen-depth/lateral allowances are assumptions, not measured
object dimensions. Earlier too-late-start attempts remain in their own folders.

Evidence: `docs/baselines/lidar_maneuver_20261028.json` and
`.verification/maneuver-20261028/`. No hardware, Enable service or vehicle deploy
was used. First scope remains fixed-side static bypass with observed near-straight
road support; dynamic obstacles and physical stopping/timing are unqualified.


## MVP maneuver r2 normal-following regression repair, 2026-10-28

The r1 field log showed three original-input-age stops after 2.176–2.284 s
and five current-pose collisions with a rear-left point outside the original
stop corridor. NORMAL now retains the successful forward stop and geometry
execution admission, without the task-only source deadline or inflated LiDAR
sweep. Active maneuvers keep swept-body checks, stale-task rejection and the
independent stop/watchdogs. Normal planning requests use 1 Hz / 1 m / prefix;
only active tasks request the dedicated 2 Hz / 2.5 m reference.

62 focused portable cases pass. The required ROS fast gate reports 443 checks
(429 pytest cases plus 14 CTest groups), no failures/errors/skips. A real node
regression holds an actual selected reference beyond 3 s with fresh feedback
and the field rear return, then confirms front-stop latching. The bundled DDS
smoke covers 4 s ordinary following, task selection, front stop and explicit
resume. Observed closed-loop simulation completes task 1 and returns to NORMAL
in 754 steps, no obstacle contact, 751 native and 3 startup-baseline frames.
This is synthetic software evidence, not physical-car acceptance.

Evidence: `docs/baselines/lidar_maneuver_20261028_r2.json` and
`.verification/maneuver-r2-20261028/`. Original r1 artifacts remain unchanged.

## MVP maneuver r3 local narrow-road passage, 2026-10-28

The user reported an 0.80 m straight with a roughly 0.25 m obstacle on the
right. r2 rejected this geometry before creating a task. r3 computes passing
room over the observed obstacle interval using local measured boundary
intersections and any measured roadside surface on the chosen passing side.
The chosen centre is inside the feasible interval and avoids unnecessary
lateral travel. Newly visible road is rechecked by the worker; no unobserved
width is extrapolated. Full swept-body and local road-containment checks
remain active. Admission/rejection diagnostics retain target bounds, support,
required/available width, centre interval and the provisional margin budgets.

The delivery profile uses 0.03 m obstacle, 0.02 m tracking, 0.02 m boundary,
0.01 m extra-clearance and 0.02 m unseen-lateral allowances. These are explicit
experimental settings, not physically measured error bounds. Original stop,
normal 1 Hz / 1 m execution, controller and partial-road inference are unchanged.
The field curve-following problem is not claimed fixed.

66 focused portable cases pass. The final installed ROS gate reports 448
checks (434 pytest cases plus 14 CTest groups), no failures/errors/skips.
Actual bundle DDS smoke uses an 0.80 m road, 0.25 m right-offset obstacle and
roadside returns; ordinary following, task execution, front stop, no automatic
resume and explicit resume pass. The fresh observed-perception closed loop
uses the delivery profile except explicit simulation longitudinal overrides,
provisional matching body dimensions, and square roadside cone stand-ins in
the ordinary ray caster/contact checker. It completes task 1 and returns to
NORMAL in 743 steps with no obstacle/roadside contact. 730 native frames and
13 startup-baseline frames are reported, with no fallback after the first
native reference; this is not native-only acceptance.

Evidence: docs/baselines/lidar_maneuver_20261028_r3.json and
.verification/maneuver-r3-20261028/. Earlier failed tests and r1/r2 outputs
remain preserved. No vehicle was connected, deployed or enabled by these tools.

## MVP maneuver r4 adaptive shift length, 2026-10-28

The r3 field record rejected a feasible-side start at 2.283 m because 2 m was
an unconditional minimum shift length. The r4 delivery enables an adaptive
quintic length: keep a further 0.10 m longitudinal buffer beyond front extent
and obstacle inflation; use a conservative 80 percent curvature budget and
0.8 m minimum; prefer 2 m when available. The worker still checks actual
curvature, full body collision and observed road containment. Extra lateral
clearance increases from 0.01 to 0.02 m; the target stays toward the minimum
necessary shift to retain room for the rotated front corner beside cones.
Legacy adaptive=false behavior, original stop and normal planning are retained.
No repair to field road-estimation failures or curve following is claimed.

69 focused portable cases pass, including the logged start-distance geometry
in an explicit straight fixture and genuine insufficient-distance rejection.
The installed ROS gate reports 451 checks (437 pytest plus 14 CTest groups),
no errors/failures/skips. Actual final-bundle DDS smoke uses a 2.28 m obstacle,
0.80 m road, right-offset 0.25 m box and roadside returns; task reference,
near stop, latched zero and explicit resume pass. A fresh observed closed loop
limits LiDAR range to 2.28125 m (an exact binary no-return sentinel), with the
same narrow geometry and square cone stand-ins. Task 1 completes and returns
in 743 steps without obstacle contact; startup baseline use is reported.
Simulation longitudinal overrides and provisional body dimensions remain
explicit and do not establish real-car acceptance or physical safety margins.
Evidence: docs/baselines/lidar_maneuver_20261028_r4.json and
.verification/maneuver-r4-20261028/. Earlier failed probes remain preserved.
