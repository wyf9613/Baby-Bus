# Baby Bus · ai4r_policy

2026-10-20：新增前方 **3 m × 3 m** 雷达局部栅格：`x∈[0,3)`、
`y∈[-1.5,1.5)`，默认 5 cm/格、10 Hz 更新。`local_grid` 配置默认开启；
发布 `/car/local_occupancy_grid`、`/car/local_inflated_grid` 和
`/car/local_grid_status`，本机内部结果为 `self.local_grid_output`。
新 Foxglove 布局增加 **Local grid** 标签，见
[显示及接口说明](tools/foxglove_road_visualization/README.md)。
地图在停车状态也可查看，不需要 Enable / Run；当前只提供地图，不参与控制或绕行。
已有轮速/gyro 历史时重投影最近 0.3 秒扫描；缺少运动历史时只显示最新单帧，
保留采样时间并标记 `SCAN_ONLY_NO_MOTION`，不能当成当前时刻的运动补偿地图。

`mvp2.1` 增加独立 lattice，默认 1 Hz，`shadow.rate_hz` 可调。当前分支随车测试配置
已设 `planner_control.enabled: true`，让同一 PID 跟踪 lattice 轨迹；设为 false 可回到基线，
执行层随 20 Hz 运动对齐、滚动检查和管理预瞄覆盖，不可用时同周期退回基线。
开关、执行语义和直道录制指令见[控制接入说明](docs/PLANNER_CONTROL_CN.md)。
新算法不使用旧 lattice 的代码或参数。现场话题、近似尺寸与验证范围见
[影子规划说明](docs/SHADOW_LATTICE_CN.md)。

影子侧新增过去 1 秒道路的运动补偿与稳健参考融合，收集随 20 Hz 基线周期，
规划频率仍可独立调整。融合不可用时重试原始规划；降级原因和完整历史输入通过
`shadow_planning` 记录。稳定控制与本轮硬碰撞边界保持原样。
见[多帧融合验证](docs/experiments/2026-10-15/SHADOW_TEMPORAL_FUSION_CN.md)。

`main` 默认采用 **弯道激进版 `5e8d1b8`**：航向 Kp=1.5、转向变化率 4/s、
连续混合急弯反馈，策略和车辆转向范围均为 ±1.0，单侧偏移为完整宽度的 0.6 倍。
用户已反馈急弯改进正常，U 弯仍待改进。配置及验证范围见
[弯道激进版实测基线](docs/experiments/2026-10-14/AGGRESSIVE_BEND_BASELINE_CN.md)。
温和转弯版 `271bfb9` 已保留完整合并历史，版本差异和取回方法见
[控制版本记录](docs/CONTROL_VERSION_HISTORY_CN.md)。状态机及末次未上车修复另存
[实验分支](https://github.com/wyf9613/Baby-Bus/tree/feature/newcar-turn-fsm-experiment)，不合入 main。

已加入最小 Foxglove 轨迹显示：`/car/policy_trajectory`（标准 ROS Path），
表示控制器实际采用的参考路径，停止时清线。
导入 [布局](tools/foxglove_road_visualization/layout.json)，见[使用说明](tools/foxglove_road_visualization/README.md)。

当前策略选择 `estimation.road_geometry=ordered_xy`，
边界按相邻锥桶和行驶方向构成二维有序链，双侧沿局部法线匹配，单侧偏移记忆完整宽度的 0.6 倍。
中线按弧长采样，支持竖直及向后折返的局部路段；PID 沿道路预瞄 0.35 m，
用 `atan2` 计算航向，增加 0.10 s 当前误差平滑。沿用横向 PI、航向 P、速度 PI，
目标恒定 0.2 m/s；弯道和有效短时预测
均不自动降速。偏离中心线时由横向 PI 与航向 P 纠正，不搜索回正轨迹。
有序二维道路初版的软件检查记录为 270 项；后续激进控制已部署并获得用户正面反馈，
当前验证范围见[验收记录](docs/acceptance.md)。
设计、证据与回退设置见 [急弯候选说明](docs/experiments/2026-10-14/ORDERED_ROAD_PID_CN.md)。
单一 20 Hz timer 同步执行，无 lattice、规划子进程、候选搜索或三帧起步确认；
控制侧的旧直线路径缓存关闭，连续道路历史只由感知负责。支持已观测前方中线
起步；双侧观测学习宽度后，缺一侧或共同范围不足时，
可用有效边界向内偏移完整宽度的 0.6 倍继续跟踪，
双侧恢复后更新宽度并回到双侧中线。配置沿用 main 归档车的前馈 0.30、驱动上限 0.35、
速度 PI 0.35/0.08、转向方向 -1。当前候选横向 Kp=2.5、策略转向上限 1、
转向变化率 4/s，航向 Kp=1.5，车辆接口转向区间 ±1.0；U 弯仍待改进。
实现和参数见 [最简中心线跟踪说明](docs/SIMPLE_CENTERLINE_FOLLOW_CN.md)。
原 V1 直线规划仍可选，本文后面的 V1 描述属于历史版本。
每次显式启动累计原始轮速里程。当前 `control.max_distance_m` 与
`max_run_time_s` 均为 0，按用户要求关闭 3 m / 30 s 测试上限；设回正数可恢复。
复杂赛道候选将 `control.end_straight_distance_m` 设为 0，关闭几何不足后的自动直行，
避免急弯缺桶被当成终点。短时预测仍受原时间、源年龄和里程预算约束。
已 push 的缓弯基线为 `271bfb9`，其末段补丁在本轮已有有效跟踪后，`invalid_road_insufficient_geometry` 自动转入
固定油门 0.30、转向 0 的直行，额外轮速积分达到 0.80 m 后锁存停止；每次起跑只触发一次。
该原因也可能出现在弯道，不自动判定终点。
直行仍要求新鲜相机消息（允许空帧）、轮速和 gyro，8 s 未完成则停车；手动停止立即生效。
其他无效/过期路径或反馈断流锁存停止，重新请求状态 3 才重置里程并恢复。
`control.end_straight_distance_m=0.0` 可关闭补丁。0.30 m 和 1.0 m 配置均通过
258 项完整 ROS 检查并已实测按距离截止。最终按用户要求改为 0.80 m，已部署并读回确认，
不再追加检查或试跑。详见 [末段补偿记录](docs/experiments/2026-10-14/CAR15_END_STRAIGHT_REPORT_CN.md)。
里程是编码器估计值，实际位移受轮速尺度、打滑和滑行影响。
可选 `calibrated` 模式仍支持有来源的 `vehicle.*` 参数。上车步骤及老师要求见
[MVP 接入说明](docs/MVP_INTEGRATION_REVIEW_2026-10-05_CN.md)。
当前配置为左黄右蓝，可用 `estimation.left_colour/right_colour`（1/2）切换。
15 号车首段完成 3 m 轮速积分试跑；连续模式的基线与首轮修复分别运行
4.543 m / 5.042 m，均因参考过期停止，用户报告仍偏蓝侧且修正不顺。
扩大转向和平滑后的试跑在移动前因弯道远端宽度估计无效停止。
离线复现后，按用户要求保留 4 m 感知拟合窗口，放宽估计宽度下限
0.3→0.15 m：同一组 21 帧全部有效，原下限有 8 帧被拒绝。
重新连接后，斜率上限放宽至 3、Huber 容差放宽至 0.20 m，单侧宽度记忆功能已部署。
单纯调整门限的两次短测仍在约 0.90/0.94 m 停止；启用单侧偏移后运行
20.195 s / 3.932 m，日志确认左侧偏移→双侧→右侧偏移，现场反馈弯道改善、轨迹基本正常。
最终因两侧均不能拟合停止：1 黄桶、2 蓝桶且蓝侧前向覆盖约 0.20 m，小于 0.30 m 门限。
双侧恢复时仍记录到参考跳变，需后续优化衔接。估计宽度与内部误差均未经外部定位验真。
参考轨迹可视化已部署，用户确认绿线可见。共同范围不足的回归衔接已修补，
末段补丁之前的版本通过 164 项 portable 与 250 项完整 ROS 检查，零失败/错误/跳过。
前一轮用户报告前段碰蓝桶；发现实际转向多加了 +0.5 实时 trim，抵消左修正。
已在策略 2 / 遥控器模式下恢复实时 trim 为 0.0，并补充试跑脚本的实时核对。
零偏置复测运行 19.746 s / 3.817 m，用户反馈“完美完美”；实时 trim 始终为 0，
执行转向偏差中位数约 0.000036。最终因双侧可用几何不足停车，已确认归零/Disabled。
复测记录见 [15 号车当前版本实测](docs/experiments/2026-10-14/CAR15_NEW_CONTROL_REPORT.md)。
导入布局与说明见 [参考轨迹可视化](tools/foxglove_road_visualization/README.md)。
详细当前证据见 [验收记录](docs/acceptance.md)。

团队策略仓库，基于 DREAM 课程框架。当前分工见 [四组项目计划](docs/TEAM_PROJECT_PLAN.md)。接口优先沿用 GitHub 中的课程框架代码和配置；框架未定义的组间数据结构再按 Word 接口会议草案补充。状态、道路和规划分别在 `self.estimation_output`、`self.planning_output`；控制诊断在 `self.control_diagnostics`，debug1/debug2 为路径误差（m）和本次累计里程（m），停止原因沿用 policy 状态字符串。算法、字段、调参和联调说明见 [估计模块首版说明](docs/ESTIMATION_V1_CN.md) 与 [V1 规划说明](docs/PLANNING_V1_CN.md)。离线检查命令为 `python3 -B tests/test_estimation.py -v`、`python3 -B tests/test_planning.py -v`、`python3 -B tests/test_control.py -v`；模型测试为 `python offline/control_pid/test_controller.py`（AI4R 环境）。下文为上游使用说明。

2026-10-05 更新：有效道路/雷达点通过轮速与偏航率历史补偿到自车状态的参考时刻，并保留原测量时间；历史不足或断档时拒绝使用。该补偿采用前进、平面无侧滑近似，轮速时间为接收时刻代理，未完成实车精度验证。详情见上方模块说明。

以下 2026-10-08/12 条目保留为旧估计分支的历史记录；当前配置和控制行为以上方 MVP2.1 说明与实际 YAML 为准。

2026-10-08 更新：道路估计增加短时历史锥桶缓存（最多 128 点），逐帧运动补偿后补足已经观测过的近端盲区。旧点保留原始年龄，5 秒内过期；首次启动的未观测近处道路仍拒绝使用。有效道路还要求双侧边界与中心线覆盖车体原点，增加 `geometry_valid` / `near_field` 诊断。该检查不替代规划组的完整车身和起步轨迹检查；本分支仍输出零动作。离线回归、参数和下游接入要求见模块说明。

同日补充：新鲜相机帧偶发空帧、单侧拟合失败或共同覆盖不足时，可短暂运动补偿最近一条完整有效道路，标记 `degraded=True` / `predicted_history`；默认失败延续上限 0.2 秒，原始源年龄、运动跨度和近端历史期限可能使其更早结束。预测不续期，当前失败原因与每侧点数/拟合范围另行保留。过期、运动缺失、异常宽度或观测冲突仍拒绝使用。规划组需读取减速建议及剩余有效时间；启动策略由规划组负责，本文实现未修改起步、规划、PID 或车辆动作。

2026-10-12 更新：仅同步 `mpc-car-run` 的感知/道路修复（来源 `af2e1fb`），现有规划代码保留。当前 estimation 配置开启逐周期推进历史、按局部位置保留历史（原始年龄上限 20 秒）、0.08 m 去重半径，以及当前单侧/短共同区间观测与真实双侧历史融合。类默认值保留旧模式，配置说明见估计模块文档。相机断档期间仍可输出无效道路，但运动连续时保留历史供恢复；未见过的边界不凭空生成。同步修正了上游运动时间预算校验不可达的问题。长期单侧弯道、航向偏差、启动及完整 ROS/实车验证仍需对应工作，不能把上游整车回放结果视为本分支验收。

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
