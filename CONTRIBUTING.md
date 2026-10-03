# Development and verification

Follow the [DREAM development guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/main/docs/development-guide.md).
Use an Ubuntu 24.04 / ROS 2 Jazzy environment. The policy remains a single
commented Python file; keep student explanations beside code/configuration.
Do not split it into a framework or create extra example YAML files.

## Build dependencies

The compatible `dream_interfaces` commit is recorded in
[ci/dependencies.repos](ci/dependencies.repos). Provision it separately from the
offline gate. The current feature pin includes the new fiducial messages; until
published, transfer it from the provider's local repository. Source packaging
records that exact commit without inferring an unverified release tag.
From the policy repository on a development machine with ROS Jazzy installed:

```bash
mkdir -p .verification/dependencies
git clone --no-checkout https://gitlab.unimelb.edu.au/dream/dream_interfaces.git \
  .verification/dependencies/dream_interfaces
git -C .verification/dependencies/dream_interfaces checkout --detach \
  "$(python3 -c 'import yaml; print(yaml.safe_load(open("ci/dependencies.repos"))["repositories"]["dream_interfaces"]["version"])')"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . .verification/dependencies/dream_interfaces \
  --ignore-src -r -y --rosdistro jazzy
```

The build requires Git, CMake, a C++ toolchain for generated messages, colcon,
pytest, PyYAML, and the ROS dependencies in `package.xml`. Installation and
network access belong to provisioning, not to a test run.

## Required fast gate

```bash
AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" \
  bash tools/verify_fast.sh
```

CI invokes this same command. It verifies the exact clean dependency revision,
builds both packages in `.verification/ws`, runs the installed Python node's
tests, and checks installed launch arguments. It uses localhost discovery in
reserved test domain 218, protected by a local lock, and starts only synthetic
ROS peers. No device interface, physical sensor or vehicle is launched.

Tests use controlled clocks for exact watchdog/state behavior and bounded
executor tests for DDS/timer behavior. Re-run focused tests when fixing failures;
complete the gate once for the final relevant source. Do not add tests that
freeze teaching-comment wording or duplicate implementation logic.

## Review and repository conventions

`dev` is the default development branch; `main` is the release-ready line. Both
are protected against direct/force pushes. Create `feature/`, `fix/`, or
`maintenance/` branches from `dev`; ordinary MRs squash into a semi-linear merge.
Pipelines must succeed and discussions must be resolved. A human must inspect
and record review for safety, public-interface, CI-policy, release and hotfix
changes; an assistant's technical review does not fulfill that requirement.
Final `v*` tags are Maintainer-controlled and immutable by convention.

The portable release-contract check exercises tag identity, promotion and sync
topology, and deterministic source packaging without ROS:

```bash
python3 -B tests/test_release_contract.py -v
```

MR CI runs the full synthetic fast gate and creates a source-package preview.
Protected-branch checks validate topology without repeating that ROS suite.
Annotated `vX.Y.Z-rc.N` tags may identify the exact frozen `dev` candidate;
annotated final `vX.Y.Z` tags identify the matching two-parent promotion on
`main`. Tagged CI builds the installed policy and pinned interfaces package,
checks launch/IDL, and retains the deterministic source archive,
`release.json`, and `SHA256SUMS`. Candidate assets are evidence only. After
successful final-tag CI and recorded human review, a maintainer publishes the
exact final-tag files as durable GitLab Release assets. Tools here never create
or move tags, merge branches, or publish releases. See
[the v0.2.0 release preparation](docs/release-v0.2.0.md) and the historical
[v0.1.0 release record](docs/release-v0.1.0.md).

Keep sensor parameter comments aligned with the selected component contracts.
Configuration eligibility, robot calibration, component selection and TF belong
in DREAM integration; changes here cannot broaden that authority.

Record current verification and any manual robot results in
[docs/acceptance.md](docs/acceptance.md). Unobserved hardware is `not run`.
