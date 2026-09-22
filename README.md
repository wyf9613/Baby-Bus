# Baby Bus · ai4r_policy

团队策略仓库，基于 DREAM 课程框架。先看 [三个子组与代码的对应关系](docs/baby-bus-team.md)。当前保留老师的零动作起始策略，团队算法尚未实现。下文为上游使用说明。

Student ROS 2 policy package for the DREAM robot, targeting Ubuntu 24.04 and
ROS 2 Jazzy. Version 0.1.0 describes the first source release; its published
annotated tag and successful release CI establish the release identity.
It is not a physically qualified driving controller.
[Acceptance](docs/acceptance.md) records observed software checks and remaining
robot checks, and [the release record](docs/release-v0.1.0.md) defines promotion,
tag, artifact, and publication evidence.

**Start with [scripts/policy_node.py](scripts/policy_node.py)** and its
`INSERT POLICY CODE` section. It contains the observation/action explanations
and a zero-action starter. The [policy YAML](config/ai4r_policy.yaml) explains
every setting and includes lidar-only and wheel-speed-controller examples.

| Update mode | Executes your policy when |
| --- | --- |
| `cone_detection` | A fresh cone-detection batch arrives |
| `lidar` | A fresh lidar scan arrives |
| `timer` | The configured policy timer runs |

Other callbacks cache observations. Declare `required_sensors` for your exercise;
missing/stale required data stops the policy. Required cones also have an
independent empty-detection timeout. Recovery needs an explicit start request.

## Build and run

The robot must first have a compatible DREAM underlay and a built student
overlay at `~/ai4r_student_workspace`. In particular, `dream_interfaces` must
provide `DriveAndSteer.units` and `UNITS_NORMALIZED`; older definitions are
incompatible. The compatible `dream_interfaces` v0.1.0 dependency is pinned in
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
not compensate for terrain or vehicle pitch. Lidar remains raw in its message
frame with its existing origin. Body IMU vectors use mounting TF, and relative
heading resets only when entering policy state 3.

| Topic | Type | Direction / QoS |
| --- | --- | --- |
| `cone_detections` | `dream_interfaces/ConeDetections` | Input; reliable, depth 1 |
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

Four commented ROS parameter files are installed:

- [ai4r_policy.yaml](config/ai4r_policy.yaml): triggers, required sensors and timing.
- [traxxas_vehicle_interface.yaml](config/traxxas_vehicle_interface.yaml): vehicle tuning and commented calibration/identity references.
- [oakd_cone_detector.yaml](config/oakd_cone_detector.yaml): perception and debug settings.
- [bno08x_imu_interface.yaml](config/bno08x_imu_interface.yaml): selected IMU products and accuracy.

The optional [camera mount file](config/camera_mount.yaml) is also installed,
but it is DREAM source configuration, not ROS parameters. The shipped `{}` uses
the defaults: camera optical centre `0.30 m` above nominal ground, `0.10 m`
behind the CG (`forward_from_cg_m: -0.10`), centred at `y = 0`, and tilted
`20 degrees` down. It accepts only `height_above_ground_m` (finite and
nonnegative), `forward_from_cg_m` (finite and signed), and
`downward_tilt_deg` (finite, strictly between `-90` and `90`, positive down).
Fields may be omitted to inherit their defaults; a missing file also uses all
defaults.

Policy settings are startup-only. DREAM reads the current source YAML under
`~/ai4r_student_workspace/src/ai4r_policy/config/` on each new start or restart,
so editing these files needs no student-workspace rebuild. Restart only the
affected unit: `dream runtime restart ai4r_policy` for policy settings, or
`dream runtime restart traxxas_vehicle_interface`, `oakd_cone_detector`, or
`bno08x_imu_interface` for that component's YAML. Camera-mount changes take
effect through `dream runtime restart oakd_cone_detector`. DREAM validates and
snapshots the effective camera configuration before stopping a running detector;
malformed YAML, unknown keys, or invalid values therefore leave it running with
its previous snapshot. An idempotent `dream runtime start oakd_cone_detector`
does not restart or reload an already-running unit. Restarting policy alone does
not reload a hardware unit. The standalone policy launch loads only its own ROS
parameter file and never consumes `camera_mount.yaml`; DREAM admits component
requests before passing them to independently launched units. Commented
reference values do not override robot calibration. See the [runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/feature/classroom-ros-environment/docs/ai4r-runtime.md)
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
