# Synchronized feedback prediction/support optimization — 2026-10-19

Revision: local-feedback-lattice21-motion-20261019

## Two coordinated changes
1. Before Jacobians/QP assembly, cheaply roll the same nominal controls through the clipped prediction model and shared finite-path projector. Reduce the first optimization horizon to the supported prefix (minimum two steps). The final optimized nonlinear prediction still must satisfy the original endpoint margin. Deadlines are shared, no PI/history/time is advanced twice, and no endpoint extrapolation occurs. Internal probe timing and supported horizon are logged.
2. Add opt-in mpc.speed_feedback_measured_motion (default false, requires speed feedback). Selected centreline/lattice feedback profiles set true. In this mode the steering preview uses zero-order-held fresh wheel speed through both the actuator delay and horizon. An internal force-balancing input through the SAME lateral RK4 model holds speed; it is NOT a throttle command. Actual drive remains the unchanged speed PI/feedforward. Measured body geometry, gain, delay, steering limits/rates, 0.25 m/s target, 0.36 start/max drive, 0.30 sustain and 0.5 s start setting remain unchanged. Disabled switch keeps original force prediction. Measured hold does not supply a braking model; profile-braking requests in that mode are refused.

This is deliberately a changed longitudinal prediction assumption, NOT vehicle identification and NOT proof the car stays at constant speed. It avoids treating the uncalibrated Gym force gain as a real-car acceleration model. Speed changes within the preview remain a modelling error; fresh feedback updates the preview every control tick. Freshness, collision, explicit-resume and compute-budget stop guards remain active.

## Evidence
Reproduced example: measured speed 0.25 m/s, historical throttle 0.30, delay 0.26 s: legacy predicts 0.4974 m/s and delay travel 0.0975 m; hold predicts 0.25 m/s and 0.065 m. On a 0.70 m reference, force mode's nominal support probe reduces an 8-step initial request to 6 before assembly and solves once; hold mode solves 10 steps once at 0.25 m/s. A real too-short path still stops. Eight seconds of an assumed delayed FOPDT wheel plant with rolling references keeps solving and moving; no real vehicle claim.

Portable final checks: 268 distinct cases, 266 pass and 2 OSQP skips; regression.log retains initial legacy optional-field assertion failure and recheck.log confirms the corrected configuration/default contract. Core physical course_model_step and vehicle parameter record remain unchanged, and actual speed_feedback_drive method is AST-identical. The base shadow YAML remains byte-identical. Only one new boolean key is added to both selected feedback YAML profiles.

ROS fast gate is blocked by the absent pinned dream_interfaces checkout / ROS environment. No live car changes, Enable, GitHub changes or push. This does not repair genuinely invalid road geometry, all start-bridge problems, sensor outages or all OS scheduling delays.

## Deployment requirement
Upload BOTH scripts/policy_node.py and config/ai4r_policy_lattice_feedback.yaml (remote selected ai4r_policy.yaml), then build/restart on the car. Confirm revision and mpc.speed_feedback_measured_motion=True. Other expected parameters remain mpc.v_exec_max_mps=0.25, mpc.shadow=False, mpc.overspeed_stop_enabled=False. Hashes in preservation.json. Original versions are backed up before installation.
