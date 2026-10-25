# Narrow-road candidate admission — 2026-10-26

Changes are local and opt-in in the supplied YAML. Frozen RNN weights and
original MPC dynamics/PI/steering +/-0.50 remain. Backups preserve the prior
operator-tested decoupled runtime under backups/before-narrow-admission-*.

Actual operator log: some paths were valid at generation, then rejected against
new current boundaries; baseline fallback could not join. At rest native prefix
collision length was zero, while execution checks at least preview0.35+0.15 m.
This admission mismatch is corrected with execution_min_check_m=0.5, retaining
larger speed/latency checks and adding the actual CG body pose to admission.

Width-aware sampling limits lateral exploration to (observed/retained road width
- body width)/2. The complete 0.297 front / 0.198 rear / 0.25 wide rectangle and
known-boundary sweep remain. Global width is only a sampling hint, never a claim
of local clearance or permission to ignore a measured boundary.

MPC compatibility is now screened before frozen scoring over the consumed path
prefix. Limit0.48 /m matches tan(0.3142*0.50)/0.33; this partially identified
model is still not a new physical steering calibration. Rejected curvature is
reported separately from collision; no-control-feasible differs from all-collide.
Because native splines can spike even on gentle bends, nine of 48 candidates
are replaced with constant-curvature primitives at 0 and mirrored fractions of
0.98*limit. RNN scores admitted authored geometry with unchanged weights; this
is an explicit changed candidate distribution, not the original frozen runtime.
Geometric compatibility is necessary, not a nonlinear tracking guarantee.

Native requested path length1.0->1.4 m increases overlap at 0.5 m/s with 1 Hz
generation and observed handoff latency. This is longer native generation, not
evidence of additional observed road or permission to reuse expired geometry.
The executor still checks current known boundaries and can reject on updates.

37 focused tests passed, including straight0.35 m corridor with a0.25 m body,
gentle radius3 m bend, road narrower than the body, stopped-car obstacle ahead,
MPC-incompatible candidate exclusion from RNN scores, and replay of two logged
collisions. Those logged collisions remain detected: no evidence yet proves
them false or fixes their upstream boundary association. Real/new boundary
conflicts, physically impossible sharp turns and failed fallback can still stop.

Final standalone-worker compact/full path, score and recurrent-memory equivalence
checked at the actual48-candidate profile. ROS fast gate is blocked by missing
pinned dream_interfaces locally. Real-car completion and timing remain untested.

Deploy BOTH scripts/policy_node.py and config/ai4r_policy.yaml from the RNN
checkout. Revision user-mpc-rnn-narrow-admission-20261026. Target0.5 m/s,
left blue2/right yellow1; original body, model and steering limits retained.
