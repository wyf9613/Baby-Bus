# AI4R policy acceptance

## Main bend-version integration, 2026-10-14

The operator explicitly selected aggressive-bend checkpoint 5e8d1b8 as main's
default and requested preservation of the gentle-bend version. The integration
first merged 271bfb9 without squash (merge node 55d50ff), then 5e8d1b8 without
squash (f86561d). Policy code and both vehicle/policy YAML files match 5e8d1b8.
All original commits remain reachable; CONTROL_VERSION_HISTORY_CN.md documents
settings, evidence, known limitations and non-destructive retrieval of old code.

The state machine and final undeployed repairs are saved separately as 3aa06e8
on origin/feature/newcar-turn-fsm-experiment and are excluded from main. Their
initial right-angle failures and undeployed-fix status remain explicit there.

The exact merged baseline was archived into a separate Docker workspace and
checked against the pinned dream_interfaces revision with tools/verify_fast.sh.
Result: 274 checks, zero errors/failures/skips, three existing vehicle-dynamics
cases deselected at the user's request. Build, installed package, ROS callbacks,
configuration and launch checks passed. Log:
experiments/2026-10-14/evidence/main-merge/main-bend-merge-no-simulation.log.
This Git integration does not deploy code or change any vehicle runtime.

## Car 05 corner-entry feedback candidate, 2026-10-14

The operator requested sharper turns after selecting a single-side full-width
offset fraction of 0.6. Live readback confirmed that fraction and vehicle
steering_max_normalized=1.0. Recent logs showed alternating endpoint/interior
feedback conventions and output lag under the 2.0/s policy slew limiter.

With control.mvp_ordered_cornering_enabled=true, ordered-XY MVP feedback blends
local normal error with observed preview-point y, and preview tangent with
target bearing. The local/tangent weight is preview/(preview+entry_distance),
where entry_distance is the nonnegative tangential distance to the observed
first point, zero for an interior projection. The formula is continuous as
support reaches the vehicle projection; far support has less tangent weight.
PI + heading P remains active, with no physical-angle map or curvature law.
Heading Kp is 1.5 and policy steering slew is 4.0/s. The 0.10 s error filter,
0.35 m preview, 0.2 m/s target and 0.6 boundary offset are retained.

Five static new controller checks, 33 existing control/lifecycle checks and
the real-ROS installed/namespaced configuration check passed (39 checks total).
The vehicle-dynamics simulation and full gate containing it were not run at
the user's request. This is a candidate for operator-controlled hardware tests;
corner clearance and physical tracking improvement remain unverified.
Uploaded policy_node.py and ai4r_policy.yaml to Car 05 (10.43.254.18); student
build and policy restart succeeded. Runtime readback confirmed cornering=true,
heading Kp=1.5, slew=4.0/s, single-side offset=0.6, policy state 2 and vehicle
Disabled. No Run/Enable was sent; the operator performs the physical trial.

## Car 05 autonomous steering range, 2026-10-14

The operator reported successful gentle-bend repeats and a passed right-angle
turn with insufficient steering margin. Read-only logs showed sustained -1.0
policy requests while the vehicle interface still scaled steering into +/-0.5.
At the user's explicit request the local and Car 05 student vehicle YAML now
select steering_min_normalized=-1.0 and steering_max_normalized=1.0. Only those
two live YAML values changed; policy gains, 0.2 m/s target and actuator PWM/RC
calibration settings were preserved. Student build and vehicle-interface restart
succeeded; parameter readback confirmed -1.0/+1.0, trim 0.0, policy 2, Disarmed,
wheel 0 and applied drive 0. Manual neutral steer reported 0.004 (1502 us).
The final pinned ROS gate passed 272 checks, zero errors/failures/skips. No
physical run was initiated after this range change; the user tests through UI.
Readback: project diagnostics/car05-20261014/steering-full/readback.json.
Gate: .verification/car05-steering-full-fast-gate.log. Follow-up changes remain
local and deployed; no additional push was performed after the user's correction.

## Ordered XY / PID bend candidate, 2026-10-14

The local `fix/newcar-control` candidate selects `estimation.road_geometry=ordered_xy`:
bounded directional cone chains, local normal boundary pairing, ordered
arc-length centreline samples, and 0.35 m arc-length PID preview with atan2
headings. The previous y(x) implementation remains selectable. Current-frame
lateral/heading errors use a 0.10 s low-pass filter; immediate faults and Stop
clear it. Speed remains 0.2 m/s with the existing PI/feedforward settings.
Single-boundary width inference is preserved, including disjoint far-side
return. Its actually observed visible chain can now bridge a short subsequent
blind gap under the original source/hold/travel deadlines. Predictions do not
refresh acquisition ages or relearn lane width. The complex-track YAML disables
automatic terminal straight (`end_straight_distance_m=0.0`); the optional old
patch remains available.

Pinned Jazzy fast gate passed 270 checks, zero errors/failures/skips:
181 portable, 84 ROS policy cases and 5 colcon entries. Coverage includes left/
right U turns, vertical right-angle segments, avoiding a nearer rear-facing
exit, wheel/gyro alignment, single-side dropout/restoration, recorded Car 15
dropouts, blind-gap expiry, PID smoothing and immediate zero/Path clear.
An explicitly labelled ideal bicycle fixture completes both 1 m radius
hairpins with maximum internal cross-track error about 0.0484 m. This uses an
assumed 0.33 m wheelbase and +/- pi/4 steering map; neither is physical car
calibration. Two extra source probes reject unknown-width cold starts and
swapped overlapping boundaries. The initial ROS cases correctly stopped when
their 0.11 s fixture update exceeded the default 0.1 s reference; setting the
fixture to the production 0.2 s lifetime fixed the test setup.

Gate and provenance: `experiments/2026-10-14/evidence/ordered-road/`.
Candidate deployment, actual corner clearance and physical turn trials: **not run**.
The pushed gentle-bend baseline is `271bfb9`; the evidence below describes that
and earlier physical trials, not physical acceptance of the new candidate.

## Archived terminal profile

Archived terminal setting: automatic 0.80 m terminal straight on tracked-road
`invalid_road_insufficient_geometry`, requested and clarified by the user.
0.30 m and 1.0 m settings were deployed and physically exercised; both stopped
on the terminal distance condition with zero/Disabled telemetry confirmed.
The user selected 0.80 m and requested no additional tests. That final setting
is deployed and statically read back, without another gate or physical run.

Status: `fix/newcar-control` contains the simple curved-centerline follower,
configured yellow left / blue right. Car 15 (10.43.254.28) completed the initial
3.002 m wheel-integral trial. Continuous baseline and first correction ran
4.543 m and 5.042 m, stopping on reference expiry; tracking remains unaccepted.
The wider-steering/smooth candidate passed the installed ROS gate (232 checks)
and was deployed, but its only physical attempt stopped on invalid estimated
road width before moving. Offline replay reproduces that curve-start fault.
The car was subsequently reconnected. Current deployed profile keeps the 4 m
fit window, width minimum 0.15 m, slope limit 3 and Huber delta 0.20 m, and
enables learned-width single-boundary tracking. Its first run lasted
20.195 s / 3.932 m wheel integral; the operator reported improved bend tracking
and a generally normal trajectory. Both-to-single-to-both transitions are
logged, with a reference jump on two-sided recovery still unresolved.
Final stop was insufficient geometry, with state 2 / vehicle Disabled and zero
wheel/applied drive confirmed. Repeated-lap and external-position acceptance
remain outstanding. Visualization was later deployed and observed in Foxglove.
The latest common-range fallback candidate passed 250 software checks but its
physical trial had an operator-reported early blue-cone collision. Live steering
trim 0.5 was confirmed, despite startup parameter 0.0, and restored to 0.0 while
policy 2 / Disarmed. The unchanged candidate was then retested with live trim
guarding: 19.746 s / 3.817 m, operator feedback "完美完美" and explicit push
authorization. Trim stayed 0.0; median applied/requested residual returned near
zero. Final stop was insufficient geometry, with zero/Disabled feedback confirmed.
This run did not confirm Path clear; repeated-lap acceptance remains outstanding.
Car 09's IMU and car 13's RC faults are below.
The 2026-10-06 car candidate's gate and bounded physical runs are historical.
Wheel-integral distance is not an independent physical distance measurement.
Upstream gates on `jah` and the earlier entries below are historical evidence.
Release identity requires a published annotated tag and successful release CI.

## Geometry-loss terminal straight patch, 2026-10-14

The user requested fixed-command forward travel of 0.30 m before stopping when
end-of-track cones leave the camera FOV, and explicitly selected automatic
activation on geometry insufficiency rather than a manual endpoint command.
That trigger cannot distinguish a finish from geometry loss within a bend.
After trials at 0.30 m and 1.0 m, the shipped YAML now enables
0.80 m / drive 0.30 / maximum 8.0 s; code default
distance 0 disables the fallback. It requires prior successful control in the
same explicit run and exactly `invalid_road_insufficient_geometry` with valid
motion state in the MVP road-centerline planner. Other faults do not activate it.

It stays in policy state 3, clears the old road-reference deadline and display,
publishes fixed drive and zero steering using the existing selected trigger,
and consumes no stale road/reference or speed-PID output. Returning cones do
not restart following or reset the manoeuvre. Independent supervision uses
the existing raw wheel integral relative to activation to latch zero state 2
at the configured distance. Fresh camera messages are still required; empty frames are allowed
only during the terminal mode. Wheel/gyro keep their original motion freshness
limits. Operator Stop, sensor faults, clock/integration gaps, positive overall
run budgets and the configured stalled-motion time fallback can stop it sooner. Stop/restart
clear the terminal state; no additional ROS trigger or vehicle Enable is added.
Zero requested steering assumes the vehicle's independently verified live
trim zero. Physical travel/coasting is not established by encoder integration.

Focused checks cover distance completion, no double integration, returning
cones, explicit restart, Stop/stop-publishing semantics, independent source
loss/freshness, stalled-motion time cap, execution gaps, cold start, other
faults, and parameter bounds. A real PolicyNode/ROS-type controlled-clock test
also exercises trajectory clear and independent-supervisor completion.
The initial full gate exposed a test-fixture timestamp mistake: its empty
frame reused the previous stamp and was correctly rejected. Advancing the
controlled clock fixes that setup; the portable focused suite then passed 24
checks, with legacy control 33 passing. Final pinned Jazzy gate passed 258
checks, zero errors/failures/skips; portable 171 (97 estimation, 17 planning,
33 control, 24 follower), ROS policy 82 plus 5 colcon test entries. Log:
`.verification/car15-end-straight-fast-gate.log`; runtime/source hashes:
`.verification/car15-end-straight-manifest.json`.
The 1.0 m / 8.0 s update then passed the same pinned gate: 258 checks, zero
errors/failures/skips, including the installed YAML and completion at both
0.30 m and 1.0 m. Log/manifest: `.verification/car15-end-straight-1m-fast-gate.log`
and `.verification/car15-end-straight-1m-manifest.json`.

`end-straight1` ran 21.669613 s / 4.246473 m total wheel integral. Terminal
receipt duration was 1.4963 s, observed extra integral about 0.290–0.319 m;
11 interior command samples were drive 0.30 / steer 0, and 13 empty Paths were
recorded in terminal mode. The operator found 0.30 m's effect too small and
requested 1.0 m. `end-straight2-1m` ran 24.594377 s / 4.862829 m total, terminal
receipt duration 4.6394 s, observed extra integral about 1.003–1.034 m;
35 interior samples were drive 0.30 / steer 0 and 36 empty Paths were recorded.
Both ended with `End straight distance reached`, live trim 0.0, policy 2 /
Disabled and zero wheel/applied drive confirmed. Neighboring asynchronous
debug2/phase receipts bound the observer estimates; they are not independent
physical travel measurements. Fresh post-stop Path clear was not confirmed
by the runner, despite receiving empty Paths during each terminal mode.

The operator requested final 0.80 m and no more tests, with a more complex
track planned later. Only distance was reduced; fixed effort 0.30 and timeout
8.0 s remain. The car was observed stationary, then YAML copied, student ROS
built and only policy restarted. Static parameter readback confirmed
0.80 / 0.30 / 8.0 and policy 2 / Disabled, trim 0.0, wheel/applied commands zero.
No Enable or policy-3 command was issued for this final settings change, and
no additional tests were run. Final configuration SHA:
`b5c475e29955ee05cd2d9e49129f972041b3eb3f481e6bdaf919b093c665c92f`;
runtime source stayed `79505042755c81f0434370ccd7f7275887125d5dd09d6eec9b82da71fb454d2f`.
Report and raw evidence: `docs/experiments/2026-10-14/CAR15_END_STRAIGHT_REPORT_CN.md`
and `evidence/car15/end-straight{1,2-1m}/`. The final 0.80 m tuning has static
deployment confirmation, not its own physical trial or exact-config gate.

## Visualization deployment, returning far boundary and live trim fault, 2026-10-14

The zero-trim retest (`visualization4`) ran 19.746045 s / 3.816904 m without a
distance/time cutoff, with peak wheel speed 0.309682 m/s. Before motion the
runner verified the unchanged tested profile, vehicle endpoints and fresh live
trim zero. Trim remained zero and no set/increment command was recorded.
178 matched active-run actuation samples had median
`applied_steer - 0.8*last_requested_steer` = 0.000036343. The operator reported
"完美完美，直接push这版本" in response to the first-bend tracking question.
37 Tracking samples contained 27 two-boundary, 8 yellow inward-offset and
2 blue inward-offset references. 147 nonempty Paths were recorded, at most
34 points in base_link; the recorder did not confirm the stop empty Path.
The final fault was `invalid_road_insufficient_geometry`, with state/motion
valid. The last recorder frame before that fault had yellow/blue x spans
0.209420 / 0.202841 m, both below 0.30 m fit coverage; recorder/policy frame
identity is not proven. Final policy 2 / Disabled, zero applied drive and zero
wheel speed were confirmed. The requested push preserves the tested runtime
source and parameters. Self-contained report, current raw recorder, policy log,
guarded runner, result and software gate are archived under
`docs/experiments/2026-10-14/CAR15_NEW_CONTROL_REPORT.md` and `evidence/car15/`.

Authorized deployment copied policy source/package, built student ROS and
restarted only policy. First readback (`visualization1`) caught default settings
instead of the tested profile and aborted before motion. The verified local
YAML was copied, built and restarted; its SHA stayed unchanged through build.
The source of the earlier remote YAML change is unknown. Subsequent runner
readback verified timer 20 Hz, cones/wheel/gyro required, all current controller
settings, learned-width enable, visualization enable, and vehicle endpoints.

`visualization2` ran 8.492393 s / 1.590816 m, recorded 63 nonempty Paths (up to
34 points, base_link), and the operator confirmed a visible green Foxglove line.
Stop was `invalid_road_insufficient_common_range`; Path clear and zero/Disabled
feedback were confirmed. Last recorder frame had yellow fit x=0.598–2.823 m,
blue fit x=3.419–3.816 m: both individual fits passed but shared range was absent.
The fix retains the nearest independently valid side using learned half-width
until shared support is sufficient. Raw both-side frames alone still teach
width; unknown-width cold start remains rejected. Recorded-frame and symmetric
left/right checks plus the actual policy-method regression passed. Final pinned
Jazzy gate: 250 checks, zero errors/failures/skips; portable 164 (97 estimation,
17 planning, 33 control, 17 follower). Logs and manifest:
`.verification/car15-disjoint-return-fast-gate.log`,
`.verification/car15-disjoint-return-manifest.json`.

The fix was deployed with source SHA
`39b39c55e7d3786a60ab342dbb61b57029b7a21fb0c7b60951388d5fc5ae2bb8`;
YAML `8a41cab3dcdf041d21109b0689fd1e710a64cf56f7ee8f44c2d15ca8f2ace49a`
and package `d8b3ae10c4dfc938b94c9704866ab288a36670846285dbfb77d16dc58fb367f9`
matched the uploaded profile. `visualization3` recorded 18.229736 s / 3.465967 m,
139 nonempty Paths, and ended Disabled. The operator reported a blue-cone
collision in the first five seconds, so this is not tracking acceptance or
validated physical distance. This run's recorder did not confirm Path clear.

Actuation review found a persistent 0.5 residual in
`applied_steer - 0.8*last_requested_steer`; medians in successful learned1 and
visualization2 were -0.000047 and -0.000387. All logged control parameters
matched. Around 2 s, request -0.592 produced applied +0.026 instead of -0.474;
around 4 s, request -0.820 produced -0.156 instead of -0.656. This large positive
offset opposes the intended negative correction. Live trim topic and diagnostics
confirmed 0.5; startup `steering_trim_normalized` still returned 0.0. Set/increment
commands and sender were not recorded, so attribution to a particular operator
or GUI action is unavailable. Provided layout defaults for both trim buttons
are zero. The live trim admission check was absent from the earlier runner.

After stop confirmation, the car was observed Disarmed / policy 2. Two early
observation attempts failed their state/discovery guards and did not write trim.
The final attempt set live trim to 0.0 through the public topic while Disarmed;
topic and diagnostics both confirmed zero, and manual source/state were preserved.
No Enable or drive was requested during that restoration. The runner now logs live trim and
set/increment commands, checks the tested zero baseline before Enable, and ends
a run if trim changes. That updated runner was used for the authorized
`visualization4` retest described above; its final state was policy 2 / Disabled.

Parent evidence: `diagnostics/car15-20261014/evidence/visualization{1,2,3}/`,
`correction/visualization{2,3}-policy.log`, `steering-trim-restore-final.log`,
`COLLISION_TRIM_REVIEW_CN.md`, and `collision-trim-comparison.json`.

## Local control-reference visualization review, 2026-10-14

The user supplied a local visualization addition after the completed driving
trial. It publishes `/car/policy_trajectory` as `nav_msgs/msg/Path` from the
reference actually accepted by control, at most 10 Hz; expiry/state changes
publish an empty path. The supplied Foxglove layout uses independent x/y
message fields for that line. No secondary policy trigger or driving publisher
was added. Comparison with the deployed car file confirms 61 runtime lines
added, none removed; ASTs of estimation settings/pipeline, road planner and
controller are unchanged. Runtime dependencies `nav_msgs`/`geometry_msgs`
were verified by read-only imports on car 15. Source hash before visualization:
`c4714c53ac80d62647329e4147c826418f5f196efc067f94354795f526298be0`.

The supplied gate reports 246 checks. Added two real DDS subscriber cases
covering each missing colour after learning width: Path geometry equals the
accepted single-boundary reference, and stop delivers an empty Path. The first
added test advanced past the default 0.1 s reference deadline; corrected test
timing to 0.11 s with the deployed 0.2 s reference lifetime. Both focused cases
then passed, followed by the final required installed Jazzy gate: 248 checks,
zero errors/failures/skips. Existing trajectory disable, reference expiry,
display-publication failure and cached-reference checks passed. Evidence:
`.verification/car15-trajectory-dds-focused.log`,
`.verification/car15-trajectory-review-fast-gate.log`, and source/layout hashes
in `.verification/car15-trajectory-review-manifest.json`.

At that initial review the visualization increment was local only; no car
build/restart or driving request occurred. Later deployment and operator GUI
confirmation are recorded above. Latest raw cones and current compensated reference can refer
to different body epochs; overlay is not external tracking ground truth.
Usage: `tools/foxglove_road_visualization/README.md` and `layout.json`.

## Car 15 learned-width single-boundary trial and review, 2026-10-14

After reconnection, width relaxation alone stopped before motion because the
yellow boundary exceeded slope limit 2 (`relaxed1`). Increasing slope to 3
admitted all nine recorded frames in raw replay. The subsequent `relaxed2` ran
4.813 s / 0.905 m and stopped on `observed_history_conflict` as blue support
dropped. Increasing Huber delta 0.15→0.20 m (`relaxed3`) ran 5.122 s / 0.941 m
and stopped for the same reason. Approximate replay with decimated motion
telemetry had suggested improvement; the physical test did not resolve it.

The user explicitly requested remembering the centre-to-boundary distance
from both observed sides and following the visible boundary with that offset
when the other side is missing. `single_side_from_learned_width` defaults false
and is true in the shipped profile. Only independently valid, fresh RAW
two-sided frames teach width and its acquisition timestamp. A valid single
boundary is offset inward along its local normal by half that remembered width.
Old opposite-side cones are removed from this branch; inferred or predicted
geometry cannot relearn width. A fresh single-side frame provides fresh
geometry, subject to existing camera/motion/reference deadlines, rather than
extending the old 0.8 s path hold. Width is fixed during a one-sided segment,
under the explicitly requested constant local-width assumption. Stop/restart
and motion reset clear it; a single-side cold start remains unavailable.
Two-sided return updates width and restores the ordinary centreline. Missing
boundaries remain marked absent; MVP explicitly admits the learned-offset
reference without claiming near-body clearance. Other planning profiles retain
their previous rejection behavior.

Required installed Jazzy gate against interfaces pin
`5f50902ccee44e8370d6e2be85607b3054ffbf98`: 241 checks, zero failures/errors/skips;
log `.verification/car15-learned-width-fast-gate.log`. Portable checks total
162: estimation 95, planning 17, control 33, follower 17. New checks cover both
missing colours, normal offset on sloped boundaries, fixed width/timestamp in
single-side operation, two-sided recovery, reset/cold-start behavior, complete
geometry loss and wheel/gyro expiry, and the recorded physical dropout sample.
The first focused follower run exposed width being cleared by the generic
prediction branch; the final source preserves fresh single-side width memory
without starting a two-boundary prediction. All final checks passed before
deployment. Student build, policy restart and runtime parameter readback
succeeded; camera, IMU and vehicle continued running. Controller gains, slew,
vehicle endpoints and speed target match `relaxed3`.

`learned1`: 20.195086 s, 3.932476 m wheel integral, peak wheel speed 0.340523 m/s.
~2 Hz Tracking logs show both sides until about 4.98 s, yellow-only offset
0.446989 m until about 9.23 s, both again until about 17.73 s, then blue-only
offset 0.438285 m. Logged sample counts are 25 both / 8 left-offset / 5
right-offset; these are sampled modes, not exact-duration statistics.
The operator reported "能继续走，轨迹基本正常" and later confirmed better bend
performance. This establishes the requested mode switching in one physical
run, not independent tracking accuracy or completed repeated-lap qualification.

At two-sided recovery around 9.23 s, preview x changed from 0.742 to 2.263 m
between Tracking samples; actual lateral input changed -0.123→-0.943 m.
Pre-slew steer reached 1.0 while that published sample was 0.376. This is a
reference/support transition, not evidence of a physical 0.94 m position jump.
Recovery continuity is the next quality issue; increasing lateral Kp is not
supported by this event. Raw debug1 and actual controller lateral input have
different geometry and must not be treated as a common ground-truth metric.

Final stop: `Planning: invalid_road_insufficient_geometry`; policy logged
camera age 0.212 s with valid wheel/gyro. The last recorder frame before fault
contains one yellow and two blue cones, with blue x span 0.198 m < current
0.30 m minimum. Raw replay reports insufficient yellow points and insufficient
blue x coverage. Recorder identity with the exact consumed fault frame is not
independently proven. Stop feedback confirms policy 2, Disabled, zero wheel and
applied drive. Driving distance/time budgets remain zero/disabled. No further
motion was requested during the user's post-run review.

Parent-workspace evidence: `diagnostics/car15-20261014/evidence/{relaxed1,relaxed2,relaxed3,learned1}/`,
`correction/learned1-policy.log`, `learned1-analysis.json`,
`LEARNED_WIDTH_REVIEW_CN.md`, `learned-width-review.json`, and
`learned-width-review.png`; regenerate the latter with `analyse_learned_width.py`.

## Minimal trajectory patch on fix/newcar-control, 2026-10-14

Per the owner's request, the tested 61-line visualization patch is now applied
directly to the local `fix/newcar-control` working tree, together with standard
`nav_msgs`/`geometry_msgs` runtime dependency declarations, focused ROS
regressions and the one-line Foxglove layout. Existing uncommitted algorithms,
configuration and test edits were preserved; AST comparison confirms unchanged
estimation/planning/control definitions and only one read-only reference record
in the control calculation. There are no preview threads or extra planner runs.
The topic is `policy_trajectory` (`nav_msgs/msg/Path`); the original cone plot
adds its position array as a green line. Existing stop/expiry transitions clear it.

The CONTRIBUTING fast gate passed for this exact working-tree source with
exact-clean dream_interfaces `5f50902ccee44e8370d6e2be85607b3054ffbf98`, in
network-disabled local Docker with ROS Jazzy/Python 3.12.3 and localhost domain
218: 241 pytest cases / 246 colcon checks, zero errors, failures or skips;
installed launch arguments passed. Source hashes were unchanged during the gate.
Evidence and pre-patch backups are in the parent system project's
`diagnostics/fix-newcar-trajectory-20261014`. No vehicle deployment, restart,
Enable or driving request was performed.

## Car 15 curve-start rejection and local correction, 2026-10-14

The first correction increased lateral Kp to 2.5, policy steering limit to 1.0
and used the actual forward target instead of a far endpoint's normal distance
when near-field support is missing. It ran 25.693 s / 5.0423 m wheel integral,
peak measured speed 0.3162 m/s, then stopped on `Control reference expired`.
The operator still reported large centre deviation and unsmooth correction.
One steering request changed by 0.438 within 37 ms while raw path error was
nearly unchanged; missing near support and reference changes remain relevant.
The final frame contained only two blue cones, no yellow: prediction exhausted.

The second candidate changes history conflict checking to compare actual
observed support, adds ordinary steering slew limit 2/s (stop still immediate),
and widens only the car's existing autonomous normalized endpoints from ±0.5
to ±0.8. Existing PWM/output bounds, drive and wheel settings were preserved
by preparing the deployment YAML from the actual car profile. Student build
and policy/vehicle restarts succeeded. The preflight parameter readback
confirmed the new controller and vehicle endpoint values. Supervisor wrappers
were given process nice=10 after observing high CPU load; ROS child priorities
were preserved. This temporary mitigation does not establish the cause of
earlier prediction expiry. No firmware or other repository was edited.

The required Docker Jazzy gate against clean interfaces pin
`5f50902ccee44e8370d6e2be85607b3054ffbf98` passed with 232 checks and zero
errors/failures/skips (`.verification/car15-smooth-fast-gate.log`). It includes
shutdown via launch SIGINT and direct SIGTERM: main now retains a valid ROS
context until final zero publication, avoiding the shutdown race exposed by
the gate. Portable checks then comprised 91 estimation, 17 planning, 33 control
and 12 follower cases. Synthetic steering-angle mapping is not a car calibration.

The single `smooth1` physical attempt became active for 0.189 s, commanded
no nonzero drive, registered zero wheel speed/distance, and stopped with
`Planning: invalid_road_invalid_width_or_order`. Stop feedback confirmed state 2,
Disabled, zero wheel speed and applied drive. The user then disconnected the
car and requested offline analysis; connections resumed only at the user's later
reconnection request, recorded above.

Offline geometry replay of all 21 recorded stationary frames found 8 width
rejections with the deployed 4 m fit window, versus none with a 3 m window.
The last camera message before the fault was received at recorder time 1.8209 s;
fault status arrived at 1.8690 s. Its estimated width fell to 0.259 m at the far
end of the 4 m fit, while the 3 m fit retained 0.811–0.922 m over measured common
support x=0.592–2.148 m. These are estimator widths, not surveyed track widths.
The same-x/slope approximation and global quadratic fit are limited in tight
bends. One invalid far sample rejects the whole road, explaining the cold start.

The first local alternative selected `estimation.road_forward_max_m=3.0`, retaining width
0.3–2.5 m, slope, confidence and timing protections. The actual policy-method
regression uses the recorded fault frame: 4 m rejects and zeros; 3 m obtains
valid forward geometry and a positive drive request. Replaying earlier raw
frames introduced no extra rejection: correction1 gains one valid frame;
continuous2 has identical counts. This is geometry evidence, not a closed-loop
driving replay. The newly prepared 3 m profile remains undeployed. Distance and
time driving budgets remain disabled; the fit range is a separate parameter.

Final local verification: 91 estimation, 17 planning, 33 control and 13 follower
portable cases (154 total); installed Docker Jazzy fast gate reports 233 checks,
zero errors/failures/skips, with installed launch arguments also checked.
Log: `.verification/car15-curve-fast-gate.log`. The first range-change gate
identified two old 4 m profile expectations; these were updated to the new
3 m bounds and the affected estimation suite passed before the final gate.
That undeployed profile was subsequently replaced by the user's requested
width relaxation, as recorded below.

### User-requested width relaxation, offline

After the user requested fewer geometry rejections, compared minimum estimated
width 0.3 / 0.2 / 0.15 m with the original 4 m fit range and all other capture
settings unchanged. The 21 stationary frames produced 8 / 2 / 0 width rejections.
Continuous2 and correction1 geometry status counts were unchanged by this
width-only adjustment. Current local YAML therefore selects 4 m and a 0.15 m
minimum, replacing the undeployed 3 m alternative. The existing 2.5 m maximum,
positive ordering, sensor/motion freshness, finite-output, reference deadlines
and explicit restart controls remain. This is an estimator plausibility bound,
not a claim of measured vehicle clearance or guaranteed perception recovery.

The recorded cold-start policy regression now checks all three profiles:
4 m / 0.3 m stops at zero, 3 m / 0.3 m and 4 m / 0.15 m produce valid forward
references and positive drive requests. Existing reversed-boundary and extreme
width rejection checks still pass with the relaxed shipped profile. Geometry
replay is saved in parent workspace `correction/width-threshold-comparison.json`
and is reproducible using `replay_width_threshold.py` in that directory.
At that offline stage the car remained disconnected and this configuration was
undeployed. Subsequent deployment and physical trials are recorded above.

Verification of the final relaxed profile: 154 portable cases and the required
installed Jazzy gate passed; 233 checks, zero errors/failures/skips, plus launch
argument validation. Log: `.verification/car15-relaxed-fast-gate.log`.

Detailed evidence in the parent workspace:
`diagnostics/car15-20261014/correction/CURVE_START_ANALYSIS_CN.md`,
`curve-start-analysis.json`, `curve-start-analysis.png`, `deploy-smooth.log`,
and `diagnostics/car15-20261014/evidence/smooth1/`.

## Car 15 runs and continuous mode, 2026-10-14

Deployed the verified yellow-left / blue-right policy to 10.43.254.28, with
source/settings backups, successful student build and matching installed hashes.
Initial trial: 15.576 s, 3.0024 m wheel integral, peak measured wheel speed
0.3148 m/s, stopped on the 3 m limit, then Disabled with fresh zero feedback.
The initial source/settings hashes were the same as the earlier car 09 candidate.

The user then requested removal of the 3 m limit and continuous driving. Zero
now disables `control.max_distance_m` and `max_run_time_s` individually;
defaults retain 3 m / 30 s for bare-node compatibility, while shipped YAML
selects zero for both. Raw wheel odometry, clock/gap validity, sensor/path
deadlines, operator stop and explicit recovery remain active. A new actual-
method test continues beyond 6 m and 30 s, then confirms missing-gyro stop.
149 portable checks and the required local Docker Jazzy installed gate passed;
colcon reports 227 checks with zero failures/errors/skips. Log:
`.verification/car15-continuous-fast-gate.log`.

After restoring a stalled vehicle telemetry stream by restarting its unit,
the continuous trial ran 23.240 s and 4.5431 m wheel integral, peak wheel speed
0.3138 m/s, then stopped with `Control reference expired`. Subsequent independent
observation confirmed policy state 2, vehicle Disabled and zero wheel/applied
drive. No automatic restart followed. The operator reported hitting a blue cone
on the right after the bend; the exact collision time was not recorded.

The control log shows policy left request saturated at -0.5 from 20.878 to
23.183 s, with firmware output -0.25 while estimated right offset grew from
0.071 to 0.188 m. The vehicle interface's 0.5 scale combines with the policy's
0.5 limit; these are normalized outputs, not measured wheel angles. At about
10 s, a +0.330 m estimated error produces only -0.091 policy request because
the heading term opposes the lateral term. At 7.33 s, estimated error jumps
0.414 m in 43 ms although adjacent raw cone positions remain smooth. This is
consistent with a change in historical/forward-only geometry, but the exact
internal transition was not logged. No control gain or steering limit was
changed based on this analysis. Detailed evidence and plot: parent workspace
`diagnostics/car15-20261014/continuous/CONTROL_LOG_ANALYSIS_CN.md`.

## Replacement car 13, 2026-10-14

The same verified source/settings were backed up, copied and built successfully
on 10.43.254.26. Source/installed hashes matched. Cones, wheel feedback and valid
gyro messages were present; the IMU reports CALIBRATING/accuracy_below_low while
host publication is ready. Yellow-left / blue-right layout was observed.

Both RC statuses remained BAD=0, with raw receiver PWM values 0/0. The operator
reported no response after turning on the transmitter. The trial remained at
preflight, before Enable or state 3. After an explicit manual handover using
policy state 2 and Disarm request 4, Disarmed was confirmed; the operator still
reported no steering response. Subsequent telemetry confirmed zero wheel speed
and applied drive, with RC still 0/0. The vehicle was left Disarmed and the
policy in state 2. No autonomous motion was requested. Receiver/power/pairing
or other vehicle-side diagnosis and a physical run remain pending. Evidence:
parent project workspace `diagnostics/car13-20261014/`.

## Car 09 deployment and colour layout, 2026-10-14

Added startup-only `estimation.left_colour` / `right_colour` parameters. Defaults
preserve the historical blue-left layout; the supplied YAML selects yellow=1
left and blue=2 right. Both must be distinct supported integer IDs. The policy
now constructs its estimator from these settings. Live camera samples confirmed
the requested layout: within 2.5 m, the final idle sample contained seven yellow
cones (median y +0.465 m) and five blue cones (median y -0.357 m).

Verification: 90 estimation, 17 planning, 32 control and 9 follower portable
tests passed. The required `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh`
passed in a local Docker ROS Jazzy snapshot against exact clean interfaces pin
`5f50902ccee44e8370d6e2be85607b3054ffbf98`: colcon reports 226 checks with zero
errors/failures/skips, including installed configuration and launch checks.
The first gate exposed the new integer settings in a float-only test parser
and an existing motion-alignment fixture missing observed origin support. The
fixtures were corrected; runtime geometry/health checks were not relaxed.
Logs: `.verification/car09-fast-gate-retry.log`.

On the authorized car, Foxglove, vehicle, OAK-D, IMU and the updated policy were
started using DREAM. Old policy source/settings were backed up under
`~/car09-test-20261014/before/`. `dream build ros student` passed; source and
installed policy/settings SHA-256 hashes matched. Other students' package
modules, launch files and component configuration were preserved.

The wheel and camera streams were healthy, but no IMU messages arrived during
the eight-second read-only observation after `dream runtime restart
bno08x_imu_interface`. The IMU MCU continued to report
`sensor_initialization_failed`, `FAULT_LATCHED`, and publication not ready.
The requested driving trial aborted at sensor preflight. No vehicle Enable or
nonzero driving action was requested. The final check confirmed policy state 2,
vehicle Disabled, fresh zero wheel speed and zero applied drive. Hardware
power-cycle recovery and actual motion remain pending. Detailed probe/build/gate evidence
is in the parent project workspace `diagnostics/car09-20261014/` and on the car
under `~/car09-test-20261014/`.

## Simple constant-speed road-centerline follower, 2026-10-13

Imported only the estimation settings, motion history, pipeline and bounded
robust fitter from `feature/road-estimation-v2@6552033`, with the matching
perception profile/tests. The main control PID and framework are retained.
The selected `planning.algorithm=road_centerline` passes observed curved samples
directly to the controller; there is no straight fit, lattice search, worker,
model-based recovery, offset/heading rejection or multi-frame startup gate.
The estimator can expose validated forward-only geometry at startup; the
follower uses those observed points without extending road boundaries.

At the user's explicit follow-up, all usable references, including valid
predictions, request exactly 0.2 m/s. Curve and prediction speed advisories do not
reduce this target. Prediction expiry, input freshness, finite-output checks,
operator stop and existing 3 m/30 s budgets remain. Constant target speed is not
constant throttle or a guarantee of constant actual speed.

The controller takes nearest-segment lateral error and a 0.35 m forward tangent,
then uses the existing lateral PI/heading P and speed PI+feedforward. It retains
its integrators between reference updates. Active gains/sign/effort use main's
archived car profile; the independent straight reference cache is disabled.
Camera age limits are aligned at 1.5 s for estimation/planning; wheel/gyro retain
their separate 0.25 s limits, references at most 0.2 s, predictions at most 0.8 s
and 0.5 m travel. Coordinates and original observation ages are preserved.

Portable verification: estimation 90, legacy planning 17, control 32, new
centerline follower 8: **147 passed**. The new suite checks direct curved sample
delivery, both turn directions, offset correction, forward-only input, fixed
speed in predictions, expiry/invalid feedback stops, first drive without three
frames, controller continuity and ideal bicycle tracking. In the ideal model,
straight and radius ±3 m arcs reduce a 0.3 m initial offset to less than 0.15 m
after 2 m travel. This uses ideal observations and assumed bicycle dynamics,
not physical sensing, calibrated steering, ESC response or actual stopping.

Legacy control fixtures now provide observed origin support required by the
new estimator; their estimator dropout bridge is disabled to isolate old
controller stop/cache contracts. Matching imported estimator tests separately
cover history and predictions. Installed configuration assertions and ROS
callback tests were updated, but cannot run in the local macOS environment.
The ROS Jazzy installed gate remains pending. Evidence and runtime hashes:
[simple-follower report](experiments/2026-10-13/SIMPLE_CENTERLINE_REPORT.md).

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
  `python -B tools/replay_control_robustness.py --output /tmp/control-replay.json`.
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
