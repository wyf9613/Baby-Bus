# AI4R policy guidance

This repository is the student policy integration consumer. Read README.md,
CONTRIBUTING.md and docs/acceptance.md for contracts, checks and current evidence.
The canonical [DREAM guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/main/docs/development-guide.md)
governs repository workflow; preserve explicit user authority and Solo/Co-op
preferences. Ordinary implementation requests authorize in-scope edits/checks.

- Keep all policy code in scripts/policy_node.py. The comments, five ROS parameter
  files and optional camera_mount.yaml/lidar_mount.yaml are the student documentation. Preserve
  the insertion markers.
- Keep one selected trigger, explicit resume, per-field IMU validity and
  freshness, zero publication independent of sensor callbacks, and pan hold.
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
