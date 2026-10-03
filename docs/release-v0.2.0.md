# Release preparation: v0.2.0

`ai4r_policy` 0.2.0 prepares the next source release of the standalone student
policy. Publication is established by an annotated tag on the verified promotion
commit, successful final-tag CI, and published release assets; this preparation
alone does not establish publication.

## Scope and compatibility

The release adds calibrated ArUco observations and filtering settings, measured
cone-publication latency, and body-frame Cartesian lidar observations. The lidar
configuration renames `lidar` to `lidar_scan` and adds `lidar_cartesian`; policy
updates still use the `lidar` mode. Students migrating configuration must update
the required-sensor and timeout keys accordingly.

The policy consumes the exact `dream_interfaces` commit in
`ci/dependencies.repos`. It does not infer an interface release tag. The source
package remains external to DREAM system selection, and hardware behaviour or
vehicle acceptance is not claimed here.

## Release gate

The frozen `dev` candidate must pass the existing synthetic ROS gate and the
portable release-contract check. A two-parent `release: v0.2.0` promotion with
an identical candidate tree is then tagged with annotated `v0.2.0`; final-tag
CI builds the policy against its exact interface pin and packages deterministic
source artifacts. This preparation creates neither the tag nor release assets.
