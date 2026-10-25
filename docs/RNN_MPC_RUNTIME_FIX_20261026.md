# RNN + MPC runtime repair — 2026-10-26

Operator log: repeated over_budget:deadline_exceeded and final
mpc_admission:origin_before_path_start; shown RNN jobs NO_ROAD/SKIPPED.
This is a software repair, not evidence of successful physical RNN execution.

Changes:
- Skip startup jobs lacking motion-history alignment without consuming the 1 Hz
  submission slot. Collect/submit immediately when eligible; no fabricated road.
- MPC horizon 10 -> 6 at dt=0.1 s: nominal preview 0.6 s (0.30 m at 0.5 m/s),
  reducing work, with less anticipation. Adaptive minimum and original deadline
  remain. Target stays 0.5 m/s; PI, physical model and steering +/-0.50 unchanged.
- Exact zero drive Jacobian in measured-speed mode removes a redundant RK4 call;
  ordinary dynamic mode still computes both control derivatives.
- Rejected near paths first try a bounded tangent prefix retaining lateral error,
  instead of demanding a zero-error connector. Same 1.2 m, 0.20 m, 0.35 rad bounds;
  supported measured-boundary body-sweep check retained. No far-end extrapolation,
  TTL renewal or bypass of observed collisions. Missing unobserved geometry is
  still uncertain; this numerical check does not certify a physical corridor.
- Fix missing wrap_angle import on Hermite fallback, cache path lengths there.
- Publish admission debug (first point, gap/limit, join outcome), log rejection
  timing before FSM clears diagnostics, tolerate pending execution-log fields.

Checks: 25 focused checks passed, including frozen model/bundled-worker checks,
actual node shadow publication, collision rejection, prefix TTL/error preservation,
finite-difference equivalence and startup scheduling. Required ROS gate cannot run
without the pinned dream_interfaces dependency and ROS Jazzy environment locally.
No Jetson timing bound or continuous real-car run has been established. Actual
NO_ROAD, large gaps, unsupported geometry and hard computation overruns still stop.

Deploy BOTH scripts/policy_node.py and config/ai4r_policy.yaml from THIS checkout.
Revision user-mpc-rnn-runtime-fix-20261026; left blue2/right yellow1;
mpc.shadow=false. Operator retains manual activation. Backup in backups/.
