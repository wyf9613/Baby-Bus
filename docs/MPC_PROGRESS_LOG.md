# MPC 进度日志

对应计划：[MPC_PLAN_OVERVIEW.md](MPC_PLAN_OVERVIEW.md)（v3）。新条目加在最上面。状态标记：✅ 完成，🟡 部分完成，⏳ 待做，❌ 阻塞。

## 2026-10-12（夜，再续）：MPC 相对 PID 的目标（提议）

依据 10-10 组会讨论和课程技术要求，写进 [MPC_PLAN_OVERVIEW.md](MPC_PLAN_OVERVIEW.md) §1 的 M3：**在跟住 Planning 轨迹、不出界的前提下，MPC 的最高稳定速度和用时比 PID 更好。** 文中包括指标、公平对比的条件、不能做 SLAM 的红线、前提（速度上限归属、刹车实测、Planning 用实测转角上限）、环线只能用 R ≥ 5 m 的弯，以及风险。待组会确认。

## 2026-10-12（夜，续）：WSL 门禁通过；G2 回放改前改后对比

**门禁**（源码 `2cecbc5`）：ROS 门禁和离线测试全部 PASS。证据：`~/ai4r-evidence/ros-20261012-165857/`。

**G2 的 7 个包用 V2 回放**，参数为 `make_deploy_yaml --mode mpc-shadow`；“改前”用 `--set` 把所有修复开关恢复成默认值。逐帧结果：`car-evidence/g2-20261008-162044/replay-v2-{nofix,fixes}.json`。表中是 MPC 有效（`shadow` 分支）的比例，括号里是硬停车（`stop`）的帧数：

| 包 | 改前 | **改后** | 主要变化 |
| --- | --- | --- | --- |
| A 居中 | 81% | **87%** | |
| B 左偏 10 cm（实测 +0.18 m） | 4%（5） | **71%（41）** | 近处空缺上限 1.0 m 生效；静止阶段出现车身冲突硬停 |
| D 车头左偏 | 77% | **79%** | |
| A2 居中（起点后移） | 47% | **99%** | 共同覆盖不足的帧与历史融合 |
| B2 左偏（起点后移） | 7%（50） | **86%（21）** | 同上 |
| C 右偏 10 cm | 65% | **88%** | |
| E 车头右偏 | 10%（6） | **77%（54）** | 单侧帧与历史融合；静止阶段出现车身冲突硬停 |

- B2 和 E 有 68–70% 的帧达到 `observed_with_history`，改前一帧都没有。多帧融合第一次在实车数据上生效。
- **新问题：起步时车身贴边的硬停车。** B 和 E 的 `stop` 帧全部出现在推车之前、车静止的阶段（1.7–5.4 s），道路为 `near_field_unobserved`，也就是 V2 用起步假设补出近处道路后，判定车身贴边。改前这些帧更早就因为覆盖不足被拒，冲突没有暴露。这两个包的实际摆放比名义值偏得多：B 实测 `e_y` +0.18 m；E 叠加了未查明的约 −0.06 rad 航向偏差。Gym 里偏 5 cm、车头偏 5° 都是 10/10。
- **对周四的要求**：起步时车居中 ±5 cm、车头 ±3°，用尺子量；shadow 复测时确认起步阶段没有 `stop`；同时带尺子查航向偏差。

## 2026-10-12（夜）：分支 `mpc-car-run` —— 自己修上游，准备周四上车

目标：周四（10-15）用 MPC 低速跑完直道，最好再跑一个缓弯。改动都加了开关、代码默认不变，只在车上 MPC 配置里打开；同时作为给估计组和 Planning 组的参考：[UPSTREAM_FIXES_FOR_CAR_CN.md](UPSTREAM_FIXES_FOR_CAR_CN.md)。

| 提交 | 内容 |
| --- | --- |
| `0a6ff8f` | Gym 支持直道、缓弯、课程赛道、S 弯，以及起步位置；新增 `car_readiness.py` 评估矩阵，跑出基线 |
| `1b8c56c` | 估计：历史逐拍推进，相机断档不清空；历史按位置保留，不按 5 s；单独的去重半径（**新发现**：0.16 m 去重半径会删掉 0.2 m 间距的相邻锥桶） |
| `c01d18e` | 估计：单侧帧和两侧历史融合 |
| `f0e4866` | V2：盲区平行外推（沿中心线直线外推，保持观测路宽） |
| `705775b` | 节点和规划子进程启动后 `gc.freeze()` |
| `a88745e` | `id_test` 新增 `arc` 模式和 `steering-arcs` 分析工具（现场不能量轮角，靠短弧推算）；车上配置打开所有修复；部署工具新增 `steer-arc` 模式、弯道 5 m、`--v-exec-max`；Gym 配置分层改为按叶子键合并（和 ROS 一致） |
| `06ec124` | 完整评估结果 `offline/mpc_gym/results/car_readiness.md` |

**Gym 结果**（car .27 条件，10 个种子，MPC，改前 → 改后）：
- 直道 1.0 m：居中、偏 5 cm、车头偏 5° 都从 5/10 → **10/10**；
- 直道 0.72 m：2、0、1 → **10、8、10**；
- 缓弯（R 5 m 45°，两侧锥桶，跑完整个弯 5 m）：1.0 m 0 → **10**，0.72 m 0 → **9**；
- MVP 的改善一样（说明修的是上游）；
- 课程赛道弯道和 S 弯仍然 0/10：弯道上只有左侧锥桶，要用路宽推断看不到的一侧，没有做，交给 Planning 组决定。

**试过、没采用**：MPC 模式下关掉回正规划（结果完全相同，保留默认）；`max_near_gap_m` 1.0 在 Gym 里没有差异（仍然放进车上配置，用 G2 回放验证）。

**还没做**：WSL ROS 门禁；G2 回放的改前改后对比（需要在 WSL 里跑）。周四现场清单：[THURSDAY_CAR_RUN_CN.md](THURSDAY_CAR_RUN_CN.md)。

## 2026-10-12（晚）：G2 包用 V2 回放 —— 有效率基本没变

`tools/replay_policy_bag.py`，参数为 `make_deploy_yaml.py --mode mpc-shadow` 生成的 V2 配置（源码 `d61da16`），V2 同步调用，不含异步延迟和暂存。逐帧结果：`car-evidence/g2-20261008-162044/replay-v2.json`。

| 包 | V1 回放有效（10-08） | **V2 回放有效** | 主要原因（V2） |
| --- | --- | --- | --- |
| A 居中 | 74% | 81% | `invalid_recovery_road` 15% |
| B 左偏 10 cm | 4% | 4% | `insufficient_lattice_coverage` 75% |
| D 车头左偏 | 76% | 77% | `insufficient_lattice_coverage` 18% |
| A2 居中（起点后移） | 47% | 47% | 两侧共同覆盖不足 52% |
| B2 左偏（起点后移） | 12% | 7% | 共同覆盖不足 77%；**硬停车** `recovery_body_boundary_conflict` 11% |
| C 右偏 10 cm | 65% | 65% | `insufficient_lattice_coverage` 21% |
| E 车头右偏 | 10% | 10% | 只看到一侧 76% |

结论：
1. **V2 加新估计器没有改善这批数据的有效率**，三个主因和 10-08 一样：两侧共同覆盖不足（20%，估计组）；只看到一侧（13%，估计组 + Planning 组）；车偏位时近处空缺超过 0.9 m（B 75%，Planning 组的 `max_near_gap_m`）。
2. **多帧融合在这批数据上没被用到**：7 个包没有一帧达到 `observed_with_history` 或 `observed`。推车只有约 0.8 m，没走过约 0.9 m 的相机近处盲区，V2 全程靠起步假设补出的近处道路（`near_field_unobserved`）。要评估融合效果，下次上车得推或开过 1.5 m 以上。
3. **新增的硬停车**：B2 有 50 帧、E 有 6 帧、B 有 5 帧是 `recovery_body_boundary_conflict`（车身离边界太近）。这不属于瞬时原因，执行模式下会立即锁停。车左偏 10 cm 时会触发，需要和 Planning 组核对 1 m 路宽下的车身余量。

下次 G2 复测要加上：推或开过 1.5 m 以上的场景，以及偏位 5 cm 的场景（看车身余量的边界）。

## 2026-10-12（傍晚）：门禁通过；Gym 测试改为 V2 口径

**WSL 门禁**（源码 `cb9d746`）：ROS 门禁 242 项、离线 166 + 6 + 18 项全部通过。证据 `~/ai4r-evidence/ros-20261012-155219/`。

途中两次失败都是测试进程的垃圾回收停顿落进墙钟计时：
- `test_lattice` 的节点起步检查：截止时间是 0.15 s 的传感器有效期，规划约 40 ms，回收停顿 80–150 ms；
- `test_mpc` 闭环的 50 ms 单步预算：一步 52.3 ms。

修复（`f6124b0`、`cb9d746`）是在计时前 `gc.collect()` + `gc.freeze()`，只改测试。**实车同样会有回收停顿**，建议节点初始化后 `gc.freeze()`，由 Planning / 集成负责人决定。

**Gym 测试**（`offline/mpc_gym`，现在 10 项：8 通过、2 项预期失败）：
- `PipelineFlow` 原来同时开着 MVP 和 MPC（实车节点禁止这种组合），现关掉 `control.enabled`。
- 默认 Gym 场地最近能看到的锥桶约在 1.0 m 处，超过 V2 起步补足上限 0.9 m，起不了步；改用 G2 推车场地（路宽 1 m、锥桶间距 0.3 m）。正常起步和带 2 cm 锥桶噪声的起步都能跑完，不锁停。
- 偏位和航向起步（+0.15 m；−0.15 m 加 0.08 rad）单独成一项，标为预期失败：−0.15 m 加航向在起步时被拒（`insufficient_lattice_coverage`）；锥桶间距 0.2 m 时中途出现 `near_field_unobserved`；0.72 m 窄路上偏位起步触发 `recovery_body_boundary_conflict`。V1 时这些都能跑完。
- V1 专有的覆盖门槛测试换成 V2 的安全规则：`clear_start_assumed: false` 时不起步，道路为 `near_field_unobserved`。
- newcar27 跑完 3 m 的测试标为预期失败（相机断档后道路恢复不了，见下一条）。
- 课程驱动模型测试的门槛从 0.25 改为拟合死区 0.289。
- `newcar27_comparison.md` 注明是 V1 时期的结果（`3acd32d`，复现用 `52af5e8`）。`comparison.md` 用的是理想参考，不经过估计和 Planning，不受影响。

**交给估计组 / Planning 组的问题**（Gym 中复现）：
1. 相机断档超过 `motion_max_interval_s` 后历史清空，近处道路再也恢复不了；
2. 偏位或航向起步在 V2 下被拒，或者中途失效；
3. Gym 默认场地最近锥桶约 1.0 m，超过 0.9 m 起步上限；
4. 回收停顿对实时性的影响。

## 2026-10-12（下午）：合入 Planning V2（mvp2.0），修复 MPC 对接

**合并**：`f10e15d` 合入 `488dc91`（= `origin/mvp2.0` `ae74be4` + 适配器 `28c0b82`，冲突由 ying 那边解好；`policy_node.py` 与 `488dc91` 逐字相同）。基础 YAML 改为 `lattice_v2`、异步规划、`clear_start_assumed: true`、多帧道路历史（5 s，`road_hold_s` 0.2）、`motion_max_gap_s` 0.25、`road_max_abs_slope` 2.75 和周二 MVP 参数；`mpc:` 段未变。

**修复**（每项一个提交）：

| 提交 | 内容 |
| --- | --- |
| `da3e111` | 异步 V2 允许配合 MPC（`mpc.reference_source: planning`），不再只限 MVP 鲁棒控制；MPC 也执行三传感器、障碍需雷达、timer 周期短于参考有效期这几条启动检查；执行模式首次有效步前，异步规划在算或首帧道路是预测时保持零动作（同 MVP） |
| `ae4ded7` | **方案 A**：`mpc_planning_reference()`。只在 `planning_pending` 时，把最近一次接受的 V2 结果用 `transport_control_reference` 逐拍补偿到当前时刻给 MPC；不延长原始有效期；其它失效、过期、运动历史问题都不暂存。不改 `planning_output`，不碰 MVP 的 `ControlReferenceManager`。`mpc_debug` 新增 `reference_held`、`reference_held_age_s` |
| `59baac3` | 补偿后保留 `speed_profile_timestamp_s`（原始规划时刻），MPC 用它查速度表；修正 shadow 速度目标偏晚 |
| `00d7ee0` | **新发现的回归**：V1、V2 每次失败都置 `stop_requested`，`28c0b82` 的适配器把它们都当明确停车，一帧 `stale_road` 就锁停，`reference_dropout_tolerance_s` 失效（`28c0b82` 起 Gym 3 m 测试已失败，当时没跑 Gym）。改为：原因在 MVP 的瞬时原因集合（`SOFT_REASONS`）里的按可容忍失效处理，其余（如 `no_feasible_lattice_trajectory`）仍立即停车 |
| `ad2d9d4` | 新增 `tools/policy_offline.py`：按异步 worker 的方式无 ROS 加载整个 `policy_node.py`，替代各工具的白名单。`replay_policy_bag.py`、`offline/mpc_gym/pipeline_sim.py` 改用它，V2 同步调用（不含异步延迟、MPC 暂存、静止 gyro 零偏学习） |

**验证（笔记本，dense/osqp）**：mpc 95/95（两种求解器），planning 18，estimation 53，control 43，lattice 43，预测模型 6，辨识 18（补了合并后缺的方法白名单）；N=10 单步 p95 3.6 ms（dense），无超预算，ying 记录的 63–170 ms 未复现。`replay_v2_mpc_wall.py` 两种编码执行分支均为 `stop`。ROS 测试新增启动检查两项，待 WSL 门禁。

**Gym（`offline/mpc_gym`）5/9 通过**，4 项失败未改，原因：
1. **V1 在新估计器下起不了步**：`db312ab` 后，近处没看到的道路一律 `near_field_unobserved`、`road.valid=False`，只有 V2 的起步假设能用。依赖 V1 拒绝原因或 V1 起步的测试（`PipelineFlow` 两项）不再成立。
2. **相机断档后道路恢复不了**：newcar27 MPC 跑 V2 走到 2.14 m（修复前 0.6 s 锁停），在 11.4 s 停下。约 0.3 s 的相机断档加 0.19 s 延迟超过 `motion_max_interval_s` 0.4 s，估计器清空锥桶历史（`motion_interval_too_long`），之后近处一直 `near_field_unobserved`；V2 设计上获得过真实近端支持后不再回退到起步假设，所以再也拿不到有效道路，MPC 的 0.3 s 容忍到期锁停。这是估计 / Planning 的行为，G2 包里也有 0.3 s 级断档，实车大概率会遇到。
3. `test_course_drive_model_never_starts_the_car`：峰值速度 0.2502 对门槛 0.25，临界。

**下一步**：
1. WSL ROS 门禁（含新增启动检查测试）；
2. 用 V2 配置回放 G2 的 7 个包，与 10-08 的有效率表对比：`python3 tools/make_deploy_yaml.py --mode mpc-shadow --out /tmp/v2-shadow`，再 `python3 tools/replay_policy_bag.py --params /tmp/v2-shadow/ai4r_policy.yaml $E/g2-*/ --json-out /tmp/replay-v2.json`；
3. 第 2 点（断档后无法恢复）找估计组和 Planning 组：是放宽 `motion_max_interval_s`，还是允许中途重新进入有界的近端补足；
4. 决定 Gym 测试怎么改：改写成 V2 的期望，或者把 V1 对比固定在合并前的提交上作为历史证据。

## 2026-10-12：合入 Planning V2 适配器；其它分支盘点

**合并**：`ying/mpc-planning-v2-adapter`（`28c0b82`）快进合入 `control-mpc-v0`。`reference_trajectory_from_planning` 现在接受 V2 的 `CARTESIAN_SAMPLES` 点列（不重新拟合、不重采样）、带 `origin`/`scale` 的 1–6 阶 y(x) 多项式和 `control_cached_path_samples`；`stop_requested` 在 `valid=False` 时也走停车分支（`planning_pending` 除外）；`speed_profile` 只在 shadow 中按时间插值成每步速度目标；执行模式遇到 `planned_stop` 仍立即锁停。新增 `tools/replay_v2_mpc_wall.py`（读本地 `origin/mvp2.0` 的真实 V2 代码做障碍墙离线联调）。**本分支的 Planning 仍是 V1**，V2 只在离线工具和测试中使用。

**验证**：
- 笔记本：`test_mpc` 87/87（dense 与 osqp 各一遍），planning 17/17，estimation 28/28，control 22/22；N=10 单步 dense p95 5.8 ms、osqp p95 4.7 ms，无超预算。墙场景两种编码都通过适配器，执行分支为 `stop`；没有制动模型，不能证明能在 0.15 m 处停住。
- WSL ROS 门禁（源码 `28c0b82`，`dream_interfaces` `5f50902`）：`verify_fast.sh` PASS，143 项，0 错误/失败/跳过；离线 132 + 6 + 18 全部通过，MPC p95 3.95 ms。证据 `~/ai4r-evidence/ros-20261012-151006/`。

**其它分支盘点**（针对车辆标定、道路参考、Planning 接口）：

| 项 | 现状 | 来源 |
| --- | --- | --- |
| 车辆标定 | **没有分支在做**。只有 `mvp2.0` 的周二经验 MVP 参数（方向 −1、前馈 0.30、`drive_max` 0.35）和静止时的 gyro 零偏在线学习；航向偏差 −0.06 rad 估计组明确未处理；V2 用课程近似车辆模型 | `mvp2.0`、`feature/state-road-estimation` |
| 道路参考 | 部分改善：多帧锥桶历史、0.2 s 质量失败桥接、退化降速（`db312ab`）；V2 起步近端补足 0.9 m 取代 V1 覆盖门槛；控制侧参考保持 ≤1 s/0.25 m（只给 MVP 控制器）。**单侧（14%）与持续共同覆盖不足（19%）仍未解决**，也未用 G2 包回放 | `feature/state-road-estimation`、`mvp2.0`、`fix/newcar-control` |
| Planning 接口 | V2 `planning_reference_v2` 已在 `mvp2.0`（`ae74be4`）；MPC 适配器本次已合入 | `mvp2.0`、`ying/*` |

**`ying/planning-v2-mpc-integration` 记录的未决问题**（合 `mvp2.0` 前要处理）：
1. 异步 V2 的启动条件只允许配合 MVP 控制器，MPC 模式启动不了；
2. MPC 没有接 `ControlReferenceManager`，`planning_pending` 与 MPC 自身的参考失效容忍需对接；
3. 运动补偿推进参考时间戳，但没有平移 `speed_profile`，shadow 速度目标时间错位。

**下一步**：合入 `mvp2.0`（或集成分支）并修上述 3 项 → 用 `replay_policy_bag.py` 让 G2 的 7 个包走新估计和 V2，对比 10-08 的有效率 → 单侧问题和车辆标定分别和估计组、Planning 组、辨识组排期。

## 2026-10-08：实车包回放工具，查清 G2 道路无效的原因

**工具**：新增 `tools/replay_policy_bag.py`。从 `scripts/policy_node.py` 抽出真实的估计器、Planning、MPC、`calculate_policy_actions` 和 `_store`（做法同 `offline/mpc_gym/pipeline_sim.py`，但不依赖 Gym），把录包里的 `/car/cone_detections`、`/car/wheel_speed_m_per_sec`、`/car/imu/data` 按接收顺序、按车上回调的校验规则喂进去，按 20 Hz 定时器逐步运行。每个包输出估计器道路状态、可见侧、路宽、Planning 原因、回放的 MPC 分支，并与包里车上录到的 `/car/mpc_debug` 分支对比。`--set section.key=value` 做假设分析，`--json-out` 输出逐帧结果。需要 ROS（rosbag2_py）和已构建的 `dream_interfaces`，在 WSL 里运行：

```bash
source /opt/ros/jazzy/setup.bash; source ~/ai4r-dev/Baby-Bus/.verification/ws/install/setup.bash
E=/mnt/e/26b/AI4R/car-evidence/g2-20261008-162044
python3 tools/replay_policy_bag.py --params $E/ai4r_policy.yaml $E/g2-*/ --json-out /tmp/replay-base.json
```

先在 Windows 上用合成数据（静止车前 1 m 宽直线，每 3 批锥桶有 1 批只有一侧）验证了逻辑：两侧都在时有效；单侧帧报 `single_side_width_unknown`；设 `known_lane_width_m=1.0` 后估计器变为 `single_side_inferred`，但 Planning 改报 `v1_requires_both_boundaries`。

**回放与车上一致**（忠实度检查）：

| 包 | 回放：有效 / 无效 | 车上录到：有效 / 无效 |
| --- | --- | --- |
| A 居中 | 74% / 26% | 79% / 21% |
| B 左偏 10 cm | 4% / 96% | 5% / 95% |
| D 车头左偏 | 76% / 24% | 79% / 21% |
| A2 居中（起点 0.45 m） | 47% / 53% | 44% / 56% |
| B2 左偏（起点 0.45 m） | 12% / 88% | 14% / 86% |
| C 右偏 10 cm | 65% / 35% | 68% / 32% |
| E 车头右偏 | 10% / 90% | 13% / 87% |

相差 2–5 个百分点，主要拒绝原因的分布一致。差异来自回放从空的运动历史开始（开头约 0.4 s 多出 `missing_motion_history` / `motion_state_unavailable`）和假定的 IMU 安装姿态（包里没有 `/tf_static`，用 10-08 IMU 日志里打印的 base_link ← fsm300_sensor 绕 y 轴 180°）。

**道路无效的原因**（7 个包合计 3,172 步，估计器状态）：

| 状态 | 占比 | 主要出现在 |
| --- | --- | --- |
| `observed`（有效） | 60% | — |
| `insufficient_common_range`（两侧都拟合出边界，但前后重叠不到 0.5 m） | 19% | A2 52%、B2 77%：起点后移到 0.45 m 的两次 |
| `single_side_width_unknown`（只看到一侧，路宽未知） | 14% | E 76%：车头右偏，可见侧 73% 为 `right_only` |
| `motion_state_unavailable`（运动状态不可用） | 4% | A、B、E；部分是回放开头造成的 |
| 其他（`invalid_width_or_order`、`insufficient_geometry`、`missing_motion_history` 等） | 3% | — |

道路有效但被 Planning 覆盖门槛（`insufficient_near_or_far_coverage`，`max_near_x_m` 0.9 / `min_forward_x_m` 1.0）拒绝的，场景 B 占 76%，C 21%，D 18%。估计路宽中位数 0.99–1.06 m，和实际 1 m 一致。

**假设分析：`--set estimation.known_lane_width_m=1.0`**。单侧帧被估计器接受（E 有 357 帧变成 `single_side_inferred`），但 Planning 全部改报 `v1_requires_both_boundaries`（`policy_node.py` 第 875 行，写在代码里），7 个包的有效比例**一个都没变**。合成数据上的结论在真实数据上得到确认：单侧兜底必须估计组和 Planning 组一起改。

**结论与下一步**（已写入全队文档和计划 §10）：
1. 两侧共同覆盖太短（19%）→ 估计组：查清重叠不够的原因（远处锥桶因置信度 < 0.75 被丢，还是检测距离有限）。可以给回放工具加“每帧左右两侧各自的前后范围和锥桶数”的输出。
2. 只看到一侧（14%）→ 估计组 + Planning 组：已知路宽兜底**加上** Planning 接受单侧推出的道路（改代码），或锥桶记忆。
3. 覆盖门槛（场景 B 76%）→ Planning 组：用回放工具试放宽（`max_near_x_m` 必须小于 `min_forward_x_m`），同时看路径误差。

证据：逐帧结果在 WSL 的 `/tmp/replay-base.json`（建议复制到 `car-evidence/g2-20261008-162044/`）。

## 2026-10-08：G2 shadow 推车 —— MPC 一侧通过，上游有两个问题

- 车 .27，部署 `434cedd`（`policy_node.py` sha256 `c7b44b0a…`，部署 YAML `0dd3ca3b…`，mpc-shadow，`qp_solver: dense`），车辆 Disarmed。按 [G2 清单](G2_SHADOW_CHECKLIST_CN.md) 单人流程，一个人手摆、**没有卷尺**；道路 2.2 m × 1 m，每排 8 个锥桶、间距 0.3 m，每个场景静止约 3 s 后推约 0.8 m，录包 25 s。
- 部署前备份了车上原有的策略文件（另一组的 `TeamPolicy`），收尾时可按清单第 10 步恢复。

| 场景 | 有效 | `e_y` m | `e_psi` rad | 候选转角 rad | 修正方向 | step p95 / max ms | 主要拒绝原因 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A 居中 | 79% | +0.064 | −0.077 | +0.015 | 89% | 17.0 / 24.5 | invalid_estimates 50，coverage 45 |
| B 左偏 10 cm | 5% | +0.178 | −0.059 | −0.046 | 90% | 1.9 / 20.6 | coverage 403 |
| D 车头左偏 | 79% | +0.124 | −0.041 | −0.026 | 100% | 17.2 / 23.2 | coverage 81 |
| A2 居中（起点后移） | 43% | +0.050 | −0.121 | +0.043 | 85% | 15.8 / 25.2 | invalid_estimates 235 |
| B2 左偏（车头略偏右） | 14% | +0.093 | −0.023 | −0.026 | 100% | 12.3 / 21.6 | invalid_estimates 363 |
| C 右偏 10 cm | 68% | −0.019 | −0.021 | +0.017 | 90% | 16.2 / 22.7 | coverage 88，invalid_estimates 60 |
| E 车头右偏 | 13% | +0.039 | −0.068 | +0.030 | 97% | 12.8 / 24.0 | invalid_estimates 384 |

（中位数取自有效样本，大部分是静止时的样本；“修正方向”= 候选转角与误差反号的比例；coverage = `insufficient_near_or_far_coverage`。）

- **通过（MPC 一侧）**：7 个场景应用动作全为 0，无 `solver_fail` / `over_budget`；整步 p95 12–17 ms、最大 25 ms；`dt` 中位数 50 ms；配置来源正确（newcar27、planning、非仿真、shadow）。候选转角始终朝减小误差的方向（85–100%）。`e_y` 正负号正确：B − A = +0.114，B2 − C = +0.112（实际都约左偏 10 cm）。推车时候选油门 0.24–0.29，接近拟合的稳态值 0.297。
- **未确认：航向偏差**。7 个场景的 `e_psi` 全为负，平均约 −0.06 rad（−3.4°）；D − E = +0.027，方向对但小于手摆误差。可能是摆放误差，也可能是相机安装角带来的固定偏差（10-06 MVP 略向右漂可能同源）。需带尺子精确摆放复测，并与估计组核对相机安装。
- **上游问题：有效参考比例 5–79%**。`invalid_estimates`（估计器判道路无效，例如只看到一侧时 `known_lane_width_m = 0` 推不出中线）和 `insufficient_near_or_far_coverage`（近处锥桶在相机视野外，车偏时远侧超过 0.9 m）。起点后移到 0.45 m 后 `invalid_estimates` 明显增多。两个原因都由 Planning（`CenterlinePlanner`）报出：前者是上游道路估计判了无效（估计组），后者是 Planning 自己的覆盖门槛（`max_near_x_m` 0.9、`min_forward_x_m` 1.0）加上估计给出的覆盖范围（Planning 组 + 估计组）。已写入计划 §10。
- 证据：本机 `E:\26b\AI4R\car-evidence\g2-20261008-162044\`（部署文件与 SHA、原文件备份、7 个包、`g2-summary.txt/json`、`g2-policy.log`）。
- 阶段状态：**G2 中 MPC 自身部分 ✅**；进入执行（G3/G4）前还需：转向标定、航向偏差复测、道路有效率改善。

## 2026-10-08：车上 IMU 无数据 —— 固件故障锁定，已恢复

- 现象：车 .27 的 `bno08x_imu_interface` 显示 running，`/car/imu/data` 有 1 个发布者，但 `ros2 topic echo` 8 s 收不到消息；重启服务（`dream runtime restart bno08x_imu_interface`）无效。车上 IMU 配置与仓库一致；`lsusb` 能看到 `DREAM BNO08x IMU`。
- 原因在 `/car/diagnostics`（`/diagnostics` 里没有 IMU 条目）：level ERROR，`firmware.lifecycle_state: FAULT_LATCHED`，`firmware.reason_code: dynamic_calibration_failed`，`connection_state: fault`，`host_publication_ready: false`。固件 0.3.0，`expectations_match: true`（不是版本不匹配）。故障锁在 IMU 板（RP2040）固件里，只重启 ROS 服务清不掉。
- 处理：停服务、给 IMU 板断电再上电（拔插 USB）、车静止放平后再启动服务。之后诊断为 `connection_state: streaming`、`host_publication_ready: true`、`lifecycle_state: CALIBRATING`（`accuracy_below_low`，门槛是 `unreliable`，所以照常发布）。注意：处理后 `lsusb` 的设备号没变（仍为 005），板子是否真正断电重启没有确认，恢复的确切原因不确定。
- 验证：静止时角速度为 0.0（BNO08x 输出已扣零偏，分辨率约 0.002 rad/s）；用手原地转车 10 s，收到 350 条消息（约 35 条每秒，部分消息不带角速度，协方差首项为 −1），z 在 −1.27 到 +1.41 rad/s，277 条非零。
- 下次排查顺序：`ros2 topic echo /car/diagnostics --once` 看 `lifecycle_state` 和 `reason_code` → 若为 `FAULT_LATCHED`，停服务、拔插 IMU 的 USB、车静止后再启动 → 仍不行交 demonstrator。
- 小坑：`timeout ... ros2 topic echo ... | grep` 会因 Python 输出缓冲丢掉全部输出，要加 `PYTHONUNBUFFERED=1` 或先写入文件。
- 状态：A 运行链中的 IMU 问题已解决；可以进行 shadow 推车（G2）。

## 2026-10-08：车上 G1 通过（dense 求解器，Jetson Orin Nano）

- 车 10.43.254.27：Jetson Orin Nano Super，L4T R39.2.1，aarch64，6 核，25W 模式；numpy 1.26.4、scipy 1.11.4、PyYAML 6.0.1，无 osqp。源码快照 `b6016b8`（代码与 `5b3d30e` 相同），scp 到 `~/ai4r_mpc_check/20261008-154505/`，未动 student workspace。
- 负载：foxglove_bridge、traxxas_vehicle_interface（Disabled，`traxxas_state_value` 0）、oakd_cone_detector、bno08x_imu_interface 在运行；ai4r_policy 未运行。4 个 DREAM runner（python3）各占约 58–74% CPU，load 约 4（6 核）。
- `bash tools/jetson_g1_check.sh`：**offline PASS、timing PASS**。没有 osqp，测试和计时自动用 dense。

| N | 步数 | mean | p95 | p99 | max | 首步 | 超 50 ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | 1650 | 10.59 ms | 11.83 ms | 12.25 ms | 12.94 ms | 11.59 ms | 0 |
| 8 | 1650 | 8.77 ms | 9.80 ms | 10.10 ms | 10.55 ms | 9.65 ms | 0 |
| 5 | 1650 | 6.23 ms | 6.58 ms | 6.73 ms | 7.23 ms | 6.80 ms | 0 |

- 求解超时保护生效：极小上限返回 `run time limit reached`；默认 30 ms 上限下正常求解（含线性化等，墙钟 13.3 ms）。
- 约为笔记本的 3 倍（N=10 笔记本 mean 3.4 ms）；N=10 的最坏步占 50 ms 预算的 26%。
- **决定：保持 N = 10**，`max_step_time_s` 0.05、`solver_time_limit_s` 0.03 不变。理由：最坏步仍有约 4 倍余量；Gym 中 N=5 跟踪效果相近，作为余量不足时的退路。
- 局限：policy 节点没有运行，所以估计、规划、ROS 回调和执行器调度的开销不在这组数里（10-06 MVP 整步计算 p99 约 6 ms），实车运行时的整步耗时仍需在 shadow（G2）中由 `mpc_debug` 的 `step_s` 和 `step_p95_s` 复核。
- 同时发现：`/car/imu/data` 有发布者，但 5 s 内 `ros2 topic echo` 收不到消息；MPC 需要 IMU 角速度，shadow 前必须解决。
- tegrastats（1 s 采样）：只开上述服务时（15:30–15:43）6 核平均占用 57%；G1 运行中（15:45–15:50）64%，最忙的核到 100%。CPU 频率 729–1344 MHz，测试前后不变；结温 48–49.5 °C，没有过热降频。所以这组耗时是在接近实车的后台负载下测得的。
- 证据：车上 `~/ai4r-evidence/g1-20261008-154540/`（machine、dependencies、offline-tests、mpc-timing.txt/json、summary）和 tegrastats 日志；已复制到本机 `E:\26b\AI4R\car-evidence\g1-20261008-154540\`（G1 时段在 `tegrastats-g1-151717.log`）。
- 阶段状态：**B3 ✅**（ROS 门禁 + Jetson G1）；A 中求解器依赖已解决。下一步：解决 IMU 无数据，然后 shadow 推车（G2）。

## 2026-10-08：不依赖 OSQP 的 QP 求解器（`mpc.qp_solver: dense`）

车上没有 osqp（见下一条），所以在 `VehicleModelMPC` 里加了只用 numpy 的求解器 `_solve_dense`：Goldfarb–Idnani 对偶积极集法，适用于正定 P（每个输入都带权重，所以成立），有限步得到精确解。新参数 `mpc.qp_solver`：默认 `osqp`，行为不变；`config/ai4r_policy_mpc_newcar27.yaml` 设为 `dense`。状态字与 OSQP 一致（`solved`、`run time limit reached`、`maximum iterations reached`、`primal infeasible`），`solver_time_limit_s` 和 `solver_max_iter` 同样生效，失败时同样锁停。

验证（笔记本 Windows，Python 3.12）：
- 300 个同结构随机 QP 与高精度 OSQP 对比：解的最大差 4e-16，平均 11 次迭代，最多 17 次。
- `tests/test_mpc.py` 新增 `DenseSolver` 4 项与一项配置检查：控制器真实 QP 上与 OSQP 的输入差 ≤1e-4；直线与新车起步闭环的分支序列相同、状态差 <1e-3；时间和迭代上限按 OSQP 的状态字停止；屏蔽 osqp 和 scipy 后仍能求解。
- `MPC_TEST_QP_SOLVER=dense` 时全部 MPC 测试用 dense 运行：75/75 通过；默认 osqp 也是 75/75。control、planning、estimation 67/67，Gym 9/9（newcar27 实验已经走 dense）。
- **车上同版本环境**（干净 venv：numpy 1.26.4、scipy 1.11.4、PyYAML 6.0.1，无 osqp）跑 `tools/jetson_g1_check.sh`：142 项通过、2 项对照测试因无 osqp 跳过；N=10 单步 mean 4.4 ms、max 6.0 ms，无超预算。第一次运行时弯道测试出现过一次 55 ms 的单步（`over_budget`），重跑和单独分析都没有复现：900 次弯道求解最多 9 次迭代、单次 ≤2.8 ms，判断为 Windows 调度抖动。
- 笔记本上两种求解器 N=10 单步 mean：dense 3.4 ms，osqp 3.2 ms。

G1 脚本同步修改：osqp 变为可选；没有 osqp 时测试和计时自动用 dense，两种都有时都测。

WSL ROS 门禁（`5b3d30e`，ROS Jazzy）：`verify_fast.sh` PASS，143 项，0 错误、0 失败；离线测试 PASS（证据 `~/ai4r-evidence/ros-20261008-153613/`）。

未完成：Jetson 上的 G1（dense）还没跑。

## 2026-10-08：车上 G1 —— Jetson 缺少 OSQP，MPC 目前无法在车上运行

- 车：Jetson Orin Nano（Super 开发套件），L4T R39.2.1，aarch64，6 核，功耗模式 25W；源码快照 `209ca4d`，按上车指南 §6.2 用 scp 复制到 `~/ai4r_mpc_check/20261008-151042/`，没有动 student workspace。运行时 foxglove_bridge、traxxas_vehicle_interface（未 Enable）、oakd_cone_detector、bno08x_imu_interface 在运行，ai4r_policy 未运行。
- `bash tools/jetson_g1_check.sh` 在第 1 步停止：**`No module named 'osqp'`**。车上所有 python3 都一样：DREAM 的 venv `~/.local/share/dream/venvs/ros2_jazzy`（策略节点 shebang 为 `#!/usr/bin/env python3`，DREAM 下解析到这个 venv）和 `/usr/bin/python3`，都是 numpy 1.26.4、scipy 1.11.4、PyYAML 6.0.1，没有 osqp。
- PyPI 有对应的现成包：`pip3 download` 得到 `osqp-1.1.3-cp312-cp312-manylinux_2_24_aarch64.manylinux_2_28_aarch64.whl`（只下载到 /tmp，没有安装）。
- 按课程规则不在车上自行安装软件。离线测试和计时都没有执行，所以 **G1 没有得到耗时数据**。
- 同时观察到：3–4 个 `python3`（DREAM runner）各占约 60–74% CPU，load 3.8（6 核），与 10-06 复盘一致。
- 证据：车上 `~/ai4r-evidence/g1-20261008-151825/`（machine、dependencies、summary）。
- 另外发现：车上 numpy/scipy 版本（1.26/1.11）比笔记本和 WSL（2.5/1.18）旧，我们的 MPC 代码和测试还没有在这个版本组合上跑过。
- 阶段状态：**A 运行链 ❌（求解器依赖缺失）**；B3 的 G1 被阻塞。下一步：请 demonstrator 在 DREAM venv 里装 `osqp==1.1.3`；同时在 WSL 用 numpy 1.26.4 / scipy 1.11.4 / osqp 1.1.3 复现车上版本组合，先离线确认兼容性。如果不允许加装，需要评估不依赖 OSQP 的求解方案。

## 2026-10-08：合并后 ROS 门禁重跑通过

- WSL（Ubuntu 24.04，ROS Jazzy）运行 `.verification/run_ros_check.sh`，测试源码为 `26a140e`（包含 `3acd32d` 合入的 MVP 和 newcar27 改动），`dream_interfaces` 为 `5f50902`。
- **`tools/verify_fast.sh`：PASS**，143 项测试，0 失败、0 错误、0 跳过；launch 参数检查通过。比 10-06 的 116 项多出的是合入的 MVP/control 测试。只使用合成的 ROS 节点，不代表实车行为。
- 离线测试全部通过：主测试（MPC、规划、估计）115/115，预测模型 6/6，辨识 18/18。
- WSL 上 MPC 每步耗时 mean 2.88 ms，p95 3.14 ms，max 3.77 ms，200 步中没有超出 50 ms 预算的步（10-06 为 p95 3.8 ms；下降应来自 v3 的 OSQP 只建一次加热启动）。这是笔记本数据，不能代替 Jetson 计时。
- 证据：WSL 中的 `~/ai4r-evidence/ros-20261008-135127/`（summary、verify-fast、offline-tests、rosdep 日志）。
- 阶段状态：**B3 中的 ROS 门禁（合并后）✅**；Jetson 计时（G1）仍待完成。下一步：Jetson 依赖检查和计时脚本。

## 2026-10-07（v3）：按新车 .27 的实车日志适配 MPC

提交 `3acd32d`（合并 `origin/fix/newcar-control`），已在 `control-mpc-v0` 上。

**起因**：实车 MVP 跑出了 2.015 m。对照它的日志，原来的 MPC 上车会遇到三个问题：
- 纵向模型过原点，0.2 m/s 只给 0.004 的油门，实车需要约 0.30，车会一直不动；
- 配置还是旧门限，第一帧就锁停；
- 缺少 3 m 距离停车。

**完成的工作**：

| 阶段 | 内容 | 状态 |
|---|---|---|
| 0 合并 | 合入 MVP；`vehicle.*` 统一使用 `VehicleParamsSettings`，并给 PolicyController 推导出 `steering_limit_rad` 和 `steering_direction`；`control.enabled` 与 `mpc.enabled` 互斥；3 m/30 s 预算同样约束 MPC 执行；MPC 起步时也有动作历史预热；新增 overlay `config/ai4r_policy_newcar27.yaml`（实车参数） | ✅ |
| 1 纵向辨识 | `offline/vehicle_identification/fit_drive_from_logs.py` 用输出误差法拟合日志，结果在 `results/newcar27_drive_fit.json`：死区 0.289，死区以上加速度斜率 10.0 m/s² 每单位油门，拖刹 1.84，阻力 1.94 1/m，延迟 0.10 s，起步需要请求 ≥0.30 并保持 0.2 s。速度 RMSE：B10/B11/B12 为 0.024–0.032（拟合集），B14 为 0.026（验证集）；B06/B07/B08 走走停停的工况为 0.06–0.09。预测模型两处副本同步加入 `drive_deadband` 和 `brake_gain_n`，一致性误差 ≤1e-12 | ✅ |
| 2 MPC 纵向 | 车在动时油门下界为"死区 + 偏置"；偏置估计 `drive_bias_gain`；起步状态机（`drive_breakaway` 加爬升，超时或回到静止次数过多都锁停）；短暂失效时沿用上一步计划；Planning 的停车请求走 stop 分支（修了 bug）；更新迟到 0.2–0.4 s 时可容忍；OSQP 只建一次（单步 4.3→2.6 ms） | ✅ |
| 3 仿真 | `pipeline_sim.py`：`NewcarPlant`、timer 20 Hz、0.19–0.22 s 延迟、丢帧、0.72 m 车道和 0.2 m 锥桶间距；`newcar27_experiment.py` 跑场景矩阵 | ✅ |
| 4 转向零偏 | ±0.03 rad 零偏只造成约 0.02 m 稳态偏移，低于 0.1 m 门槛，不加横向积分，只做现场实测 | ✅（不需要额外实现） |
| 5 上车准备 | `config/ai4r_policy_mpc_newcar27.yaml`（shadow，`valid: false`） | 🟡 需要现场测量 |

**仿真结果**（[newcar27_comparison.md](../offline/mpc_gym/results/newcar27_comparison.md)，只是仿真，不是实车证据）：
- 标定：MVP 起步延迟 0.50 s（实车 0.53）、稳态 0.202 m/s（实车 0.20）；**峰值 0.261，实车 0.333**。拟合低估了脱离静摩擦时的加速度，所以评估超速时要多留约 0.07 m/s 的余量。
- 25 个场景跑完 3 m 的数量：MVP 0，MPC 24（N=10/8/5 相同）。差距主要来自失效处理方式（MVP 遇到任一帧规划失败就锁停），而不是跟踪精度。MPC 唯一失败的是 20% 丢帧场景。
- 死区漂移到 0.27–0.305 时都能完成，起步爬升和偏置估计都在起作用。MPC 峰值速度 ≤0.293 m/s。
- 笔记本上单步 p95：N=10 为 3.0 ms，N=5 为 1.8 ms。N=10 的横向 RMSE 最好（0.034 m），先保持 10，等 Jetson 实测后再定。

**测试**：主测试 control 22、mpc 70、planning 17、estimation 28；离线测试 prediction 6、identification 18、Gym 9、control_pid 20，全部通过。**`tests/test_policy_node.py`（ROS）未运行**，需要在 WSL 上跑 `.verification/run_ros_check.sh`。

**下一步**：
1. WSL 上跑 ROS 门禁；Jetson 上做 G1 计时（N=10 和 N=5）。
2. 审阅后提交合并；`vehicle.*` 字段的变化通知辨识组，新增 `drive_deadband`、`brake_gain_n`、`drive_breakaway`、`breakaway_wait_s`。
3. 现场按顺序进行：遥控确认动力和零输出 → 原地转向，测零偏和 gain 大小 → 用尺子量轴距 → MVP 跑 0.5 m 复测 → MPC shadow 推车测试（检查符号、拒绝原因和耗时；推车约 0.2 m/s 时候选油门应接近 0.297。shadow 时 MVP 不能同时开车）→ 把 `vehicle.valid` 改为 true → MPC 依次执行 0.5 m、1 m、3 m。

按周的详细计划见 [MPC_PLAN_OVERVIEW.md §9](MPC_PLAN_OVERVIEW.md#9-后续开发计划按周)。

**分工调整（待组内确认）**：团队计划 §3.4 把"实时部署"写在机械（Zhenyu Zhang）名下。现调整为：
- IT2（Yuzaiyang Fan）负责车载实时运行：求解器、耗时、ROS 门禁、Jetson；
- 机械负责车辆侧参数和验证；
- 真车部署和测试由两人共同完成，IT1（Ying Xu）负责测试场景和指标。

**未决问题**：

| # | 问题 | 对象 | 影响 |
|---|---|---|---|
| 1 | Planning V2（弯道路径）的时间和格式 | Planning（Xinkai Zhang、YueShan Li） | M1 的开始时间 |
| 2 | 遇到障碍只停车就达标，还是必须绕行（团队计划第 315 行） | Planning 牵头，全队决定 | M2 的工作量 |
| 3 | 停车请求的格式（`stop_requested=True`）是否会变 | Planning | 停车分支 |
| 4 | 新增的 4 个 `vehicle.*` 字段是否接受 | Identification（Zhengxi Chen） | `vehicle.*` 能否冻结 |
| 5 | 速度上限和安全余量由谁定 | Planning 与 MPC | 超速阈值的归属 |
| 6 | 分工调整（见上） | MPC 三人，然后全队 | 团队计划 §3.4 和决策表 |

**建议提交到团队决策表（TEAM_PROJECT_PLAN 附录 B）的条目，组会确认后再写入**：

| 日期 | 决策 | 备选方案 | 理由和影响 |
|---|---|---|---|
| 2026-10-06 | MPC 改为纵横向联合控制 | 横向 MPC + 共享速度 PI | 预测模型已有纵向方程，可以利用速度和转向的耦合；与 Baseline 的比较因此是整套架构的比较 |
| 2026-10-07 | 纵向模型加入死区和拖刹，参数从实车日志拟合 | 过原点模型；等辨识实验结果 | 过原点模型在 0.2 m/s 只给 0.004 的油门，实车需要约 0.30 |
| 2026-10-07 | 短暂失效时沿用上一步计划（≤0.3 s），油门不增加 | 立即锁停；零油门滑行 | 实车每一轮都停在单帧规划失败上；这台车零油门时是拖刹 |
| 2026-10-07 | MVP 和 MPC 放在同一个部署文件里，二者只能开一个；`vehicle.*` 统一成一套字段 | 两个分支分开维护 | 车上只部署一个文件，避免配置互相覆盖 |
| 2026-10-07 | MPC 的实时部署由 IT2 和机械共同负责 | 团队计划原分工（机械负责） | IT2 负责求解器和耗时，机械负责车辆参数，两人在现场互补 |

## 2026-10-07：感知 → 规划 → MPC 对齐，完成最小流程闭环测试

**问题**（车上试跑前用真实代码检查发现）：按原来的配置，执行模式下 MPC 在**第一帧就锁停**。
1. 相机视场约 80°，1 m 宽车道的边界锥桶最近要到约 0.6–1.3 m 才能看到；Planning V1 要求道路从 0.5 m 以内开始（`max_near_x_m`），所以每帧都拒绝。
2. Planning 为车身后部留边界支撑，参考线从车前约 0.7–1.5 m 才开始；MPC 只允许往回延长 0.6 m，拒绝为 `origin_before_path_start`。
3. 锥桶噪声达到 2 cm 时，估计出的曲率超过 0.05 1/m，中线拟合误差超过 3 cm，规划拒绝。
4. 执行模式下，任何一帧拒绝都立即锁停。

**新增 `offline/mpc_gym/pipeline_sim.py`**：用 Dream Gym 的相机锥桶（80° 视场，0.05 s 延迟，10 Hz）驱动节点里真实的 `calculate_policy_actions`（估计 → 规划 → MPC）和 `_store`，形成闭环。时间线和真车一致。

**对齐修正**（尚未提交）：
- `config/ai4r_policy_mpc_prototype.yaml`：
  - `planning.max_near_x_m` 1.45、`max_curvature_1pm` 0.3、`max_fit_error_m` 0.06（**这三项是 Planning 组的参数，需要和他们商定**）；
  - `mpc.max_backward_extension_m` 2.0（Planning V1 输出直线，往回延长是精确的）。
- 新增 MPC 参数 `mpc.reference_dropout_tolerance_s`，默认 0，即保持立即锁停。设为正值时，执行中短暂的参考失效先滑行（驱动 0、保持转向）；超过这个时长，或遇到超速、`stop_required`、dt 异常、求解失败，仍然锁停。
- 试跑配置 `.verification/car-trial/mpc_exec.yaml`（仿真车辆参数，0.2 m/s，容忍 0.3 s）。

**闭环结果**（直线课程道路，15 s）：
- 名义、偏移 ±0.15 m、偏移 −0.15 m 加航向 0.08 rad、噪声 2 cm（3 个种子）、噪声 5 cm（3 个种子，含 1–2 次短暂滑行）：**全部执行到底，无锁停**；p95 约 5 ms。
- 起步就超出 Planning V1 工作范围（偏移 0.2 m 加航向 −0.1 rad）时会拒绝，这是正确行为。**车要摆在中线 ±0.15 m、航向 ±5° 以内。**
- 对照：修正前的配置在第 0.1 s 锁停。
- 测试：主测试 99/99，Gym 测试 6/6（新增两项闭环测试）。

**下一步**：车上先用 `mpc_shadow_check.yaml` 统计拒绝原因，再用 `mpc_exec.yaml` 低速执行；三项 Planning 阈值的改动要和 Planning 组确认。

## 2026-10-06（下午）：ROS 门禁通过

- WSL（Ubuntu 24.04，ROS Jazzy，Python 3.12.3）运行 `.verification/run_ros_check.sh`，测试源码为 `1578021`，`dream_interfaces` 为 `5f50902`。
- **`tools/verify_fast.sh`：PASS**，116 项测试，0 失败、0 错误。覆盖 `test_policy_node`、`test_estimation`、`test_planning`，只使用合成的 ROS 节点。这是联合 MPC 改动后 `test_policy_node` 的第一次运行。
- 离线测试：97/97、6/6、17/17 全部通过。WSL 上 MPC 每步耗时 mean 3.5 ms，p95 3.8 ms，max 7.7 ms，没有超出 50 ms 预算的步。
- 证据：WSL 中的 `~/ai4r-evidence/ros-20261006-114822/`（summary、verify-fast、offline-tests、rosdep 日志）。
- 阶段状态：**B3 中的 ROS 门禁 ✅**；Jetson 计时（G1）和车上 shadow（G2）仍待完成。下一步从第 2 项开始（参数需求和决策表），然后是第 4 项（Jetson G1）。

## 2026-10-06：联合 MPC 落地，完成 Gym 对比

分支 `control-mpc-v0`，已 push 到 origin（HEAD `111190c`）。

### 今天完成的工作
| 提交 | 内容 |
| --- | --- |
| `6a9fa31` | 合并 `feature/mpc-prediction-model`：移到 `offline/mpc_prediction_model/`；类改名为 `CourseModelParams`；转向映射支持 offset 和不对称范围；Dream Gym 验证重跑，结果与原分支一致 |
| `0b93b1b` | 合并 `feature/vehicle-identification`：辨识实验在 estimator 之前执行；启动时拒绝 `id_test` 与 MPC 同时开启 |
| `430f998` | **联合 LTV-MPC**：内联预测模型（与离线模型逐行一致，测试误差 ≤ 1e-12）；驱动和转向一起优化；删除复制来的速度 PI；新增 `vehicle.*` 参数记录与执行门禁 |
| `7ce1eb9` | 文档：`VEHICLE_PARAMS_INTEGRATION.md`，更新 `MPC_PROTOTYPE.md`、测试计划、运维指南 |
| `e85bc3e` | 原样引入 PID 组的 `offline/control_pid/`（只读复用，不含 main 上的运行时 MVP） |
| `111190c` | `offline/mpc_gym/`：MPC 与 PID 在 Dream Gym 中的条件一致对比（M6 证据） |

### 测试状态（笔记本，Windows，Python 3.12，osqp 1.1.3）
- `tests/test_mpc.py`、`test_planning.py`、`test_estimation.py`：97/97 通过。MPC 每步耗时 mean 4.7 ms，p95 5.7 ms，max 8.5 ms。
- `offline/mpc_prediction_model/test_vehicle_model.py`：6/6 通过。`offline/vehicle_identification/test_vehicle_identification.py`：17/17 通过。
- `offline/mpc_gym/test_mpc_gym.py`：4/4 通过。PID 走共享仿真循环时，指标和逐步轨迹与 `tune.simulate` 完全相同。
- `offline/control_pid/test_controller.py`：17/20。另外 3 项依赖 main 上的运行时 MVP，本分支没有，属于预期。
- **`tests/test_policy_node.py`（ROS）未运行。**

### Gym 对比结果（[comparison.md](../offline/mpc_gym/results/comparison.md)，**整套架构对比**）
- 62 次运行全部完成。所有场景中 MPC 的横向 RMSE 都低 15–35%，最大偏移相同；**转向变化率是 PID 的 3–6 倍**。
- 弱项：
  - 驱动/阻力失配时有稳态速度误差（0.027 m/s，PID 为 0.007 m/s）；
  - 不能主动制动；
  - 参考丢失时每次运行触发 6 次锁停。
- 选出的权重大多落在网格边界上：PID 的评分公式对平滑性惩罚很轻。
- 已知 100 ms 延迟的补偿没有带来改善；N = 5 与 N = 10 效果相近，N = 5 耗时约少 40%。

### 对照计划的阶段状态
| 阶段 | 状态 | 说明 |
| --- | --- | --- |
| A 运行链 | 🟡 | Jetson 求解器验证、车上运行链、监督逻辑负责人都没完成；参数字段扩展待辨识组确认 |
| B0 模块测试 | ✅ | 缺少一项单独的"线性化与原模型局部一致性"测试 |
| B1 闭环仿真 | ✅ | 自写被控对象 + Dream Gym |
| B2 扰动与故障 | 🟡 | 求解器限时在 Jetson 上是否生效未验证 |
| B3 ROS 与目标平台 | ⏳ | ROS 门禁、Jetson G1、shadow G2 都没做 |
| C（M0） | ⏳ | 需要 B3 完成，以及实测的 `vehicle.*` |
| D–F | ⏳ | Gym 对比是 F 的仿真部分；实车部分未开始 |

### 未决问题
1. **路线变更（横向 → 联合）需要记入团队决策表**，并告知 PID 组：对比将是架构对比。
2. `vehicle_params.py` 和测试计划中新增的字段（m、k、c、速度上限、制动减速度、安全余量）是**提案**，需要辨识组（Zhengxi Chen）同意。速度上限和安全余量的归属要和 Planning 组商定。
3. `origin/main` 的 MVP 也使用 `vehicle.*` 前缀，但字段名不同。将来合回 main 前要和 PID 组统一。
4. 参考短暂丢失时，是"立即锁停"还是"沿用上一帧参考 0.1–0.2 s"，需要团队决定。
5. 运维指南有两处过时：第 6.2 节的 scp 缺少 `offline/`；第 8.3 节的期望值描述过时。

### 下一步
| # | 动作 | 负责 | 产出 |
| --- | --- | --- | --- |
| 1 | WSL 运行 `.verification/run_ros_check.sh`（ROS 门禁 + 离线测试），失败就修 | IT2 | `~/ai4r-evidence/ros-*/summary.txt` |
| 2 | 向辨识组提交 MPC 参数需求（方程、字段、单位、符号），确认新增字段 | 机械 | 双方确认的字段清单 |
| 3 | 决策表记录路线变更；与 PID 组确认对比口径和监督逻辑负责人 | 三人 | 决策表条目 |
| 4 | Jetson：依赖检查、离线测试和计时（G1），快照要带上 `offline/` | IT2 | `jetson-offline-tests.log`、p95 |
| 5 | 车上 shadow 推车（G2，不 Enable），场景 A–E | 三人 | bag、`mpc_debug`、符号检查记录 |
| 6 | 辨识组完成实测后，填入 `vehicle.*`，用实测参数再跑 shadow | 机械 + 辨识组 | `vehicle.valid: true` 的记录 |
| 7 | （可选）速度积分或扰动估计；评分加重平滑性后扩展网格重整定 | IT1 / IT2 | 新的 Gym 结果，与现有结果并列 |
| 8 | 修正运维指南中过时的两处 | 任一 | 文档 |
