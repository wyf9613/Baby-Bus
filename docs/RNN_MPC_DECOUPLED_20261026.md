# Decoupled RNN/MPC runtime — 2026-10-26

Operator logs: new runtime loaded, but fallback centerline, no completed planner
job shown; deadline_exceeded 129 ms for a 50 ms budget, reference expiry, and
34.7 cm / 30.5 deg start-bridge alignment rejection. These do not establish
that the RNN algorithm itself failed, or that every wall-time spike was BLAS.

Root integration differences from upstream PID:
- PID reads current forward-reference geometry without an MPC start bridge or
  nonlinear prediction/QP. Original MPC retains its model/steering/support gates.
- Job submission was after successful action publication, coupling planning to
  controller success. It now occurs immediately after estimation, including
  neutral warmup and rejected control ticks, while 1 Hz cadence remains.
- First learned result is now awaited at zero output, bounded by 8 s (startup
  grace 8.5 s). Failure to return stops explicitly rather than repeatedly
  resetting the run under baseline driving. Returned NO_ROAD can still fall
  back to centerline; that is not RNN execution and remains visible in logs.
- Short references previously latched before the next tick could replace them.
  New 0.1 s refresh window emits ZERO at the original deadline; no old command
  replay or TTL extension. A still-expired reference cannot publish an action,
  and prolonged expiry still locks. Freshness, body collisions and explicit
  operator resume after hard faults remain. Speculative unpublished MPC state
  is cleared while estimator/planner/RNN history survives neutral refresh.

Runtime workload changes:
- Set BLAS/OMP pools to one thread before NumPy import, including child workers.
- Rank the same 48 candidates and frozen model first; runtime compact-result
  mode omits unused candidate geometry from the wire. Selected path, collision
  result, anchors and recurrent hidden state remain exact. Offline/full payload
  is unchanged by default. This reduces serialization/decode work, not planner
  candidate count. Logs expose submitted job ID, worker PID and worker status.
- Solver now records thread CPU vs wall time to distinguish CPU calculation
  from scheduling/waiting. 50 ms hard budget remains, horizon stays 6 x 0.1 s.

Checks: 31 targeted tests passed, plus actual final standalone-worker full vs
compact equivalence at 48 candidates. YAML rejects duplicate mapping keys.
Pinned dream_interfaces and ROS Jazzy runtime are unavailable locally; the
required fast gate remains blocked. No physical success or Jetson timing
guarantee is claimed. Large misalignment/unsupported turns remain legitimate
rejections: no collision or curvature bypass was introduced.

Revision user-mpc-rnn-decoupled-20261026. Deploy BOTH scripts/policy_node.py and
config/ai4r_policy.yaml from this RNN checkout, not the original Baby-Bus.
0.5 m/s, left blue2/right yellow1, +/-0.50 MPC steering, original vehicle/PI.
All prior files backed up under backups/before-decoupled-runtime-*.
