# Baby Bus · ai4r_policy

团队策略仓库，基于 DREAM 课程框架。当前分工见 [四组项目计划](docs/TEAM_PROJECT_PLAN.md)。接口优先沿用 GitHub 中的课程框架代码和配置；框架未定义的组间数据结构再按 Word 接口会议草案补充。已加入首版车辆状态低通滤波和单帧鲁棒道路估计，新增内部状态采用 `speed_mps`、`yaw_rate_rps` 等草案字段，并提供截图名称别名。结果在 `self.estimation_output`，参数位于现有 `config/ai4r_policy.yaml` 的 `estimation` 节；规划及控制尚未接入，驱动和转向仍为零。算法、字段、调参和联调说明见 [估计模块首版说明](docs/ESTIMATION_V1_CN.md)。离线检查命令为 `python3 -B tests/test_estimation.py -v`；完整 ROS gate 与实车验证见 [验收记录](docs/acceptance.md)。下文为上游使用说明。

2026-10-05 更新：有效道路/雷达点通过轮速与偏航率历史补偿到自车状态的参考时刻，并保留原测量时间；历史不足或断档时拒绝使用。该补偿采用前进、平面无侧滑近似，轮速时间为接收时刻代理，未完成实车精度验证。详情见上方模块说明。

Student ROS 2 policy package for the DREAM robot, targeting Ubuntu 24.04 and
ROS 2 Jazzy. Version 0.2.0 adds ArUco observations to the first source release;
a published annotated tag and successful release CI establish release identity.
It is not a physically qualified driving controller.
[Acceptance](docs/acceptance.md) records observed software checks and remaining
robot checks, and [the v0.2.0 release preparation](docs/release-v0.2.0.md)
defines promotion, tag, artifact, and publication evidence.

**Start with [scripts/policy_node.py](scripts/policy_node.py)** and its
`INSERT POLICY CODE` section. It contains the observation/action explanations
and a zero-action starter. The [policy YAML](config/ai4r_policy.yaml) explains
every setting and includes lidar-only and wheel-speed-controller examples.

| Update mode | Executes your policy when |
| --- | --- |
| `cone_detection` | A fresh cone-detection batch arrives |
| `fiducial_detection` | A fresh fiducial-detection batch arrives |
| `lidar` | A fresh lidar scan arrives |
| `timer` | The configured policy timer runs |

Other callbacks cache observations. Declare `required_sensors` for your exercise;
missing/stale required data stops the policy. Required cones also have an
independent empty-detection timeout. Recovery needs an explicit start request.
Cone batches carry `acquisition_to_publish_latency_s`, measured from OAK-D's
end-of-exposure timestamp until immediately before the detector's `publish()`
call. Exposure duration is excluded; this is not exposure-start-to-publication
latency. The student starter exposes
`cone_acquisition_to_publish_latency_s` and `cone_measurement_age_s` alongside
the existing coordinate/colour/confidence lists. The latter is the current age
from the same end-of-exposure acquisition reference, already including latency;
never add the two together.
Internally, `observations["cone_detections"]` is now a batch dictionary with
`detections` (the existing five-value tuples) and
`acquisition_to_publish_latency_s`, or `None` when unavailable/stale. This
replaces the previous raw-list value; the student function signature is unchanged.
Rebuild the OAK-D producer and policy against the same updated cone message.
Fiducial batches include dictionary/source-frame metadata and marker records;
valid poses are transformed into `policy_frame_id` at image acquisition time.
A fresh empty marker list is valid data, with no automatic visibility timeout.
The exact student Python record fields are documented beside
`calculate_policy_actions` in the policy script. A valid pose is a successful
geometric solve, not proof that the marker is real or safe to drive toward.
Your policy selects expected IDs and evaluates distance, reprojection error
and consistency across observations; background false detections are possible.

## Build and run

The robot must first have a compatible DREAM underlay and a built student
overlay at `~/ai4r_student_workspace`. In particular, `dream_interfaces` v0.2.0
must provide `FiducialDetections` and the units-bearing `DriveAndSteer`;
older definitions are incompatible. The compatible dependency is pinned in
[ci/dependencies.repos](ci/dependencies.repos). Staff/developer standalone build
and test instructions are in [CONTRIBUTING.md](CONTRIBUTING.md).

For managed DREAM runs, keep this package in the external student workspace and
set `~/.config/dream/ai4r.yaml` to:

```yaml
student_workspace: ~/ai4r_student_workspace
```

DREAM uses that workspace's installed overlay only for `ai4r_policy`. Its
[AI4R runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/feature/initial-autonomous-car-stack/docs/ai4r-runtime.md)
describes the explicit build, source-YAML validation and restart commands.

With the compatible underlay and student overlay sourced:

```bash
ros2 launch ai4r_policy ai4r_policy.launch.py namespace:=car
```

The launch supports optional `namespace` and `params_file` arguments. Shipped
policy settings load first, then the explicit overlay. It starts only the
`ai4r_policy` node and does not respawn it. DREAM owns hardware services and
mounting transforms separately. Its camera transform describes the fixed
zero-pan pose; physically panning the camera invalidates that transform until a
measured dynamic TF is supplied.

The policy starts in state **2: publishing zero drive and steering**. It holds
the pan target by sending no pan command. Vehicle Enable/Disable/Disarm is
separate: wait for the vehicle's Enabled state after its explicit Enable request,
then request policy state 3. Ongoing zero commands satisfy the policy side of
the vehicle's fresh-zero pre-enable handshake. The policy never enables it.

```bash
# Start policy calculations after required sensors and the vehicle are ready:
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
# Stop policy driving actions and publish zeros continuously:
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
```

State **1** ceases all action publication; it is neither Disarm nor an immediate
vehicle stop. A blocked Python calculation also blocks this node's watchdog;
the independent vehicle command watchdog remains necessary. Sending a zero
command does not prove physical stopping or take control away from manual RC.

## Interfaces and configuration

All names below are relative to the selected namespace. The fixed node name is
`ai4r_policy`. Sensor subscriptions keep the latest observations; IMU fields have
independent validity/freshness. Cone positions are metres in `policy_frame_id`
(normally `base_link`). DREAM places the normal `base_link` origin on nominal
ground directly below the agreed vehicle centre of gravity: +x forwards, +y
left, +z up. A cone position's `z` is the detected point's nominal height above
that ground plane, not the cone's total height. The fixed-pose transform does
not compensate for terrain or vehicle pitch. Raw lidar remains in its message
frame with its existing origin; Cartesian points use mounting TF to reach the
body frame. Body IMU vectors use mounting TF, and relative
heading resets only when entering policy state 3.

Every accepted scan prepares `lidar_scan` and attempts `lidar_cartesian` in the
subscriber callback, before a lidar-triggered policy step. There is no conversion
switch. Students receive `lidar_ranges`, `lidar_intensities`, `lidar_points_xyz`
and `lidar_scan_indices`, with separate `lidar_scan_available` and
`lidar_cartesian_available` flags. Cartesian points are finite, in-range `(x,y,z)`
tuples in metres; index `j` corresponds to raw ray `lidar_scan_indices[j]`.
The Cartesian dictionary contains `points_xyz`, `scan_indices` and `frame_id`.
Missing TF or failed conversion leaves the raw scan usable and immediately
discards the previous Cartesian result. A fresh empty Cartesian list is valid.
Snapshots retain matching acquisition stamps and cannot expose Cartesian data
beyond either its own timeout or the raw scan timeout. One nonblocking transform
lookup uses the first ray's timestamp for the entire scan; there is no per-ray
motion correction or IMU levelling.

**Configuration migration:** rename the old observation/required-sensor name
`lidar` to `lidar_scan`, and `sensor_timeout_s.lidar` to
`sensor_timeout_s.lidar_scan`. The update mode remains `lidar`. That mode must
require `lidar_scan`, `lidar_cartesian`, or both. Cartesian driving policies
should require `lidar_cartesian`; its timeout defaults to 0.5 s. Timer and other
sensor modes reuse the same cached representations and freshness handling.

| Topic | Type | Direction / QoS |
| --- | --- | --- |
| `cone_detections` | `dream_interfaces/ConeDetections` | Input; reliable, depth 1 |
| `fiducial_detections` | `dream_interfaces/FiducialDetections` | Input; reliable, depth 1 |
| `scan` | `sensor_msgs/LaserScan` | Input; best effort, depth 1 |
| `wheel_speed_m_per_sec` | `std_msgs/Float32` | Input; reliable, depth 1; unsigned m/s |
| `imu/data` | `sensor_msgs/Imu` | Input; best effort, depth 5 |
| `policy_fsm_transition_request` | `std_msgs/UInt16` | Input; reliable, depth 10; states 1/2/3 |
| `drive_and_steer_set_point_normalized` | `dream_interfaces/DriveAndSteer` | Output; reliable, depth 1; explicit normalized units |
| `pan_set_point_normalized` | `std_msgs/Float32` | Output; reliable, depth 1; optional normalized target |
| `policy_fsm_state_value`, `policy_fsm_state_string` | `std_msgs/Int8`, `std_msgs/String` | Output; reliable, depth 10; current state and reason |
| `imu_heading_angle` | `std_msgs/Float32` | Output; reliable, depth 10; valid tared heading in degrees |
| `debug1`, `debug2` | `std_msgs/Float32` | Output; reliable, depth 10; optional student scalars |

All QoS is volatile. Standard ROS parameter/introspection services and TF
subscriptions also exist. Finite actions are clipped to `[-1,1]`; invalid
outputs or student exceptions stop the policy. A pan target of `None` holds;
zero recentres. Drive is motor effort, not a speed setpoint. Full meanings and
examples are beside the variables in the Python file.

Five commented ROS parameter files are installed:

- [ai4r_policy.yaml](config/ai4r_policy.yaml): triggers, required sensors and timing.
- [traxxas_vehicle_interface.yaml](config/traxxas_vehicle_interface.yaml): editable steering limits/trim, timeouts, slew rates, RC calibration and wheel/encoder settings; commented identity and pan-joint references.
- [oakd_cone_detector.yaml](config/oakd_cone_detector.yaml): perception and debug settings.
- [bno08x_imu_interface.yaml](config/bno08x_imu_interface.yaml): selected IMU products and accuracy.
- [aruco_detector.yaml](config/aruco_detector.yaml): dictionary, marker sizes and detection filtering.

ArUco filtering runs in `aruco_detector`, before observations reach the policy.
Set `allowed_marker_ids: [4, 5]` for an exercise using only those IDs; the shipped
empty list allows every dictionary ID. `min_consecutive_frames: 3` requires an
ID in three consecutive processed images before publishing the current result,
adding about 0.2 s at 10 Hz. Set it to `1` to disable confirmation. A missed
detection removes the ID immediately; reacquisition must confirm again. A gap
over 0.5 s between image acquisitions resets confirmation, so rates below 2 Hz
require confirmation to be disabled. This filters IDs, not individual copies
of the same ID, and cannot exclude a persistent false detection of an allowed
ID. Repeated-ID observations remain separate and poses are never replayed.
An image with no accepted markers still supplies a fresh empty batch; the
student continues to decide the driving response to marker absence.

The optional [camera mount file](config/camera_mount.yaml) is also installed,
but it is DREAM source configuration, not ROS parameters. The shipped `{}` uses
the defaults: camera optical centre `0.30 m` above nominal ground, `0.10 m`
behind the CG (`forward_from_cg_m: -0.10`), centred at `y = 0`, and tilted
`20 degrees` down. It accepts only `height_above_ground_m` (finite and
nonnegative), `forward_from_cg_m` (finite and signed), and
`downward_tilt_deg` (finite, strictly between `-90` and `90`, positive down).
Fields may be omitted to inherit their defaults; a missing file also uses all
defaults.

The optional [lidar mount file](config/lidar_mount.yaml) likewise ships `{}` to
inherit DREAM's baseline: scan origin 0.20 m forward of the CG, centred laterally,
0.12 m above nominal ground, level with 180-degree yaw. It admits
`forward_from_cg_m`, `left_from_cg_m`, `height_above_ground_m`, `roll_deg`,
`pitch_deg` and `yaw_deg`. Distances are finite metres, height is nonnegative,
and angles are finite degrees in [-180, 180]. Rotations are absolute,
right-handed `Rz(yaw) * Ry(pitch) * Rx(roll)` from the lidar frame into
`base_link`, not offsets added to the default. The file documents signs and
measurement references. Apply edits using `dream runtime restart rplidar_c1`;
policy restart alone does not reload the pose. Invalid settings leave the
current lidar run intact. Cartesian points use the updated TF; raw scan ranges
are unchanged. Install the matching DREAM lidar-mount support first; older
DREAM versions ignore this optional file. No dynamic/IMU compensation is added.

Policy settings are startup-only. DREAM reads the current source YAML under
`~/ai4r_student_workspace/src/ai4r_policy/config/` on each new start or restart,
so editing these files needs no student-workspace rebuild. Restart only the
affected unit: `dream runtime restart ai4r_policy` for policy settings, or
`dream runtime restart traxxas_vehicle_interface`, `oakd_cone_detector`,
`bno08x_imu_interface`, or `aruco_detector` for that component's YAML.
Camera-mount changes take effect through `dream runtime restart oakd_cone_detector`. DREAM validates and
snapshots the effective camera configuration before stopping a running detector;
malformed YAML, unknown keys, or invalid values therefore leave it running with
its previous snapshot. An idempotent `dream runtime start oakd_cone_detector`
does not restart or reload an already-running unit. Restarting policy alone does
not reload a hardware unit. The standalone policy launch loads only its own ROS
parameter file and never consumes `camera_mount.yaml` or `lidar_mount.yaml`; DREAM admits component
requests before passing them to independently launched units. The active vehicle
settings override DREAM's baselines; use measured calibration and geometry for
your car. Omitted settings inherit the baseline. This requires DREAM's expanded
Traxxas delegation; older versions reject the newly exposed settings. Commented
reference values remain system-owned. See the [runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/feature/classroom-ros-environment/docs/ai4r-runtime.md)
for the complete source-YAML workflow.

Direct-node equivalent for integrations that need one (Python ROS launch):

```python
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path
policy = Node(
    package="ai4r_policy", executable="policy_node.py", name="ai4r_policy",
    namespace="car", output="screen", respawn=False,
    parameters=[str(Path(get_package_share_directory("ai4r_policy")) /
                    "config" / "ai4r_policy.yaml")],
)
# Append one explicit policy overlay after the shipped file if needed.
```

## Integration status and development

This is the standalone source repository. DREAM's AI4R integration defines the
managed policy and component runtime units and admits source-YAML overrides.
Its `base_link` origin is nominal ground directly below the agreed CG. The
default zero-pan camera optical pose is `[-0.10, 0.00, 0.30]` m relative to that
origin, with a 20-degree downward tilt; the optional camera mount file overrides
height, forward offset and tilt. The nominal IMU translation now has
`z = 0.10 m` in this ground-level frame, with its existing rotation unchanged.
Sensor observations retain their contracts except that transformed
cone `z` is rebased to the ground-level origin; raw lidar data is unchanged.
Physical pan motion requires a measured dynamic transform before cone positions
can be trusted. The fixed flat-ground pose does not compensate for terrain or
vehicle pitch, and physical frame validation remains separate. The policy does
not own those services, perform hardware discovery, or install student
dependencies.

`dev` is the active development branch; `main` is the release-ready line.
Annotated immutable tags identify candidates and releases. Use feature branches
and reviewed MRs; safety, public-interface and CI-policy changes require recorded
human review.
See [contribution checks](CONTRIBUTING.md), [change history](CHANGELOG.md),
[acceptance](docs/acceptance.md), and the [MIT license](LICENSE).

Traxxas output mapping and final PWM bounds are documented beside the eight
integer microsecond settings in `config/traxxas_vehicle_interface.yaml`. DREAM
admits and validates them before vehicle-interface restart. They affect manual
RC fallback too; restarting only the policy does not reload them. Matching
firmware/interface 0.5.0 is required.
