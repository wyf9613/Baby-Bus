# Experiment evidence, 2026-10-06

This directory contains the recorded result summaries, per-message JSONL telemetry,
CPU samples, cProfile statistics, timing probes, configuration snapshots and logs
from the three local diagnostic directories. The CSV log index lists sizes and
SHA256 hashes. Original car/workstation backups and transfer archives remain local.

Key files:

- `runner-fix-20261006/before-runner.txt`, `after-runner.txt` and corresponding
  `.pstats`: runner profiles. Nested cumulative times must not be added together;
  profiler overhead and different loop counts prevent a direct total-time A/B.
- `timing-fix-20261006/policy-timing.jsonl`: measured policy calculation duration,
  update intervals and estimation/planning validity in the recorded windows.
- `newcar-27-20261006/newcar-test-evidence/extended-2m.jsonl`: per-message telemetry
  for the approximately 2 m run, including requested/applied actions and wheel speed.
- `newcar-27-20261006/newcar-test-evidence/extended-curvature.jsonl`: raw cone
  batches with reconstructed road curvature and coverage around the longer test.
- `newcar-27-20261006/newcar-test-evidence/policy-logs.txt`: recorded node logs.

JSONL telemetry is sampled by the diagnostic scripts; it is not a complete rosbag.
Wheel-integral distance is not an independent measurement of chassis displacement.
Some result files are aliases/copies; use the result CSV before counting runs.
The deployed YAML is a car-specific experiment snapshot, not the package default.
Process commands and filesystem paths inside logs describe the historical car.
Probe scripts are archived for analysis and are not launched by this documentation.
