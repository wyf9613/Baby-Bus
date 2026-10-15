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
  insertion markers in mvp21_policy/node.py. Keep the original training policy,
  scripts/policy_node.py, its config/models and the immutable upstream snapshot
  intact unless the user explicitly requests a change to them.
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
