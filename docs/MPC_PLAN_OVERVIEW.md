# MPC Plan Overview：从最小真车闭环到系统集成与对比实验（v3）

日期：2026-10-07（v2：2026-10-06，v1：2026-10-03）。适用对象：MPC 子组三人，即机械（Zhenyu Zhang）、IT1（Ying Xu）、IT2（Yuzaiyang Fan），对应团队计划 [TEAM_PROJECT_PLAN.md](TEAM_PROJECT_PLAN.md) §3.4。每天的进展和证据记录在 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md)。其他组想快速了解现状和接口，先看 [MPC_README.md](MPC_README.md)。

**版本变化**

- v2（10-06）：技术路线从"横向 MPC + 共享速度 PI"改为**纵横向联合 MPC**。MPC 同时输出驱动和转向，预测模型使用 `offline/mpc_prediction_model`。
- v3（10-07）：依据 10-06 新车 .27 的实车日志做了以下调整：
  - 纵向模型加入油门死区和电调拖刹；
  - 加入偏置估计和起步逻辑；
  - 短暂失效时沿用上一步计划；
  - 合入 MVP，两种控制器互斥；
  - 仿真改为实车条件。
  
  本版同时补充了分工调整、各组交付时间线、阶段状态，以及按周排的后续开发计划。

---

## 1. 目标与边界

子组目标：设计、实现并验证一个 MPC controller。它要能在车载计算平台上及时运行，并跟踪共享 Planner 给出的路径；同时要形成模型、代价与约束、优化与运行三个方面的个人贡献证据。

按三个层次推进：

- **V0：最小虚拟闭环。** 用仿真车辆运行和真车相同的 MPC 代码，在直线和缓弯上闭环；完成异常输入测试和车载平台试运行后进入 M0。
- **M0：最小真车闭环。** 低速、无障碍、直线；MPC 同时控制驱动和转向；有结果检查、停止逻辑和日志。
- **M1：可靠的路径跟踪。** 扩展到弯道和更多速度条件，验证模型误差、控制平滑性、参考更新和实时运行；在条件一致的前提下与 Baseline 比较（这是整套控制架构的比较，见 §5）。
- **M2：整体任务集成。** 跟踪 Planning 组生成的避障路径和速度参考，支持停止、恢复和到达终点。

这里的 M0/M1/M2 是本子组的阶段。团队计划里的 M0–M8 是全队里程碑，对应关系见 §8。

**本子组不负责的内容**（团队计划 §3.2、§3.4）：

- 弯道参考路径的生成：归 Planning；
- 障碍是否相关、停车还是绕行的决策：归 Planning；
- 车辆参数辨识实验：归 Estimation/Identification。

MPC 负责的是跟踪 Planning 的输出，以及正确执行停车请求。障碍前停车可以作为中间验收行为；如果团队目标包含绕过障碍到达终点，只停车不算完成。

## 2. 分工

| 成员 | 持续负责 | 对应代码和证据 |
| --- | --- | --- |
| **机械：Zhenyu Zhang** | 预测模型、离散化和线性化、仿真被控对象、参数解释、动作映射；**车辆侧参数**：和辨识组对接，现场测量转向 gain、零偏、轴距，复核纵向拟合 | `offline/mpc_prediction_model/`、`course_model_step`、模型一致性测试、`offline/vehicle_identification/fit_drive_from_logs.py` |
| **IT1：Ying Xu** | 参考适配、代价函数、约束、权重整定、仿真场景与指标、对比评估；和 Estimation、Planning 协调 | `reference_trajectory_from_*`、`mpc_path_projector`、代价项、`offline/mpc_gym/`（场景、指标、权重） |
| **IT2：Yuzaiyang Fan** | 问题组装、优化与求解器（RTI + OSQP）、控制循环、失效处理、日志、时间预算；**实时运行**：ROS 门禁、Jetson 上的依赖和耗时 | `VehicleModelMPC`、`MPCController`、`mpc_debug`、`tests/test_mpc.py`、ROS 门禁记录、Jetson 计时 |
| **共同负责** | 接口和集成、**真车部署与测试**（部署清单、shadow、低速执行），以及最终对比 | `docs/`、`config/*newcar27*.yaml`、实车记录、对比图表 |

**分工调整说明（待组内确认，并记入团队决策表）**：团队计划 §3.4 把"实时部署"写在机械名下。现调整为：

- IT2 负责车载实时运行，包括求解器、耗时和 ROS 门禁；
- 机械负责车辆侧参数和验证；
- 真车部署和测试由两人共同完成，IT1 负责测试场景和指标。

## 3. 组间边界与依赖

数据流：Estimation 提供状态和道路几何；Planning 提供路径、速度参考和停车请求；MPC 输出驱动和转向。MPC 自带门禁：拒绝会让框架进入状态 2（零动作），必须显式请求 state 3 才能恢复。v3 起有一个例外：时间不超过 `reference_dropout_tolerance_s` 的可容忍失效，会继续执行上一步的计划。

| 小组 | 负责 | 给 MPC 的内容 | MPC 的需求和现状 |
| --- | --- | --- | --- |
| Estimation（Yiming Zhang） | 状态估计、道路几何 | `state`（速度、yaw rate、时间戳和有效性）、`road`（中线和边界，已对齐到状态时刻） | ✅ 已接入；实车需要 0.4 s 的相机源时效 |
| Identification（Zhengxi Chen） | 车辆参数实验 | `VehicleParams`，运行时对应 `vehicle.*` | 🟡 v3 新增 4 个字段（`drive_deadband`、`brake_gain_n`、`drive_breakaway`、`breakaway_wait_s`），**待确认**；转向 gain 和零偏待实测 |
| Planning（Xinkai Zhang、YueShan Li） | 局部参考、弯道、障碍决策 | V1：直线中线（`mvp_low_speed` 或 `upstream` 两种配置）；`stop_requested` | ⏳ 需要 **V2 弯道路径**（M1 的前提）；⏳ 需要**障碍响应方案**（M2 的前提）；**停车还是绕行待定**（团队计划第 315 行） |
| Baseline（Yifan Wu、JiaHao Ni） | Pure Pursuit/PID 和 MVP | 实车 MVP 参数、实车日志、Gym 对比基线 | ✅ MVP 已合入同一个文件，`control.enabled` 和 `mpc.enabled` 二选一 |

**弯道的过渡方案**：Planning V2 出来之前，弯道测试用 bypass 模式（`reference_source: estimation_centerline`），直接拟合估计器给出的中线。这个模式没有 Planning 的车身间距和停车距离检查，**只用于仿真和 shadow，不作为验收依据**。

## 4. 时间线：到目前为止各组交付了什么

日期取自 git 提交记录。

| 日期 | 小组 / 成员 | 交付内容 | 对 MPC 的意义 |
| --- | --- | --- | --- |
| 09-21 至 09-30 | 课程团队 | policy 框架 v0.1–v0.3（锥桶延迟、激光雷达、Traxxas 输出限制） | 运行环境 |
| 09-30 | 全队 | 分为四个子组，MPC 三人；先做轨迹跟踪 MPC，不直接做 MPCC | 确定范围 |
| 10-03 | 全队 | 团队项目计划 | 分工、接口草案 |
| 10-03 | Baseline（Yifan Wu） | 离线 PID 基线整定（Dream Gym） | 后来的对比基线 |
| 10-04 | Estimation（Yiming Zhang） | 状态滤波和鲁棒道路估计 | MPC 的状态和道路输入 |
| 10-04 | Identification（Zhengxi Chen） | 车辆辨识测试流程 | `vehicle.*` 字段的来源 |
| 10-05 | Estimation | 道路几何对齐到状态时刻 | 预测起点和道路一致 |
| 10-05 | Planning（Xinkai Zhang） | V1 直线中线规划 | MPC 的参考路径（仅直线） |
| 10-05 | 机械（Zhenyu Zhang） | 预测模型和 Dream Gym 验证（误差 ≤1e-6 m） | MPC 的预测模型 |
| 10-05 | IT2（Yuzaiyang Fan） | 横向 MPC 原型、shadow 模式、弯道 bypass | 第一版 MPC |
| 10-05 | Baseline | MVP 集成，3 m 自动停车 | 实车基线 |
| 10-06 | MPC 子组 | 合并预测模型和辨识流程；**联合 MPC**；`vehicle.*` 参数；Gym 与 PID 对比（M6 的仿真证据）；WSL 上 ROS 门禁通过；感知、规划、MPC 的阈值对齐 | v2 |
| 10-06 | Baseline 和集成 | **实车实验**：新车 .27 上 MVP 跑了 2.015 m；转向符号 −1，前馈 0.30；日志归档 | v3 的依据 |
| 10-07 | MPC 子组 | **v3**：合入 MVP；从日志拟合纵向模型；死区、偏置估计、起步逻辑、沿用计划；按实车条件仿真（MPC 24/25 跑完 3 m，MVP 0/25） | 当前版本 |

## 5. 技术路线（v3）

**纵横向联合 LTV-MPC。**

| 项目 | 选择 |
| --- | --- |
| 预测模型 | 课程 notebook 车辆模型的运动学版本，状态 `[x, y, ψ, v, δ]`（CG，base_link），输入 `[drive, steering_action]`（归一化）。两份代码逐行一致（`course_model_step` 与 `vehicle_model.step`，误差 ≤1e-12） |
| 纵向（v3） | `m·v̇ = gain·(drive − d0) − c·v·abs(v)`；drive ≥ d0 时 gain = `motor_gain_n`，低于 d0 时 gain = `brake_gain_n`（电调拖刹）。新车拟合值：d0 = 0.289，死区以上 10.0 m/s² 每单位油门，拖刹 1.84，阻力 1.94 1/m，延迟 0.1 s。静摩擦（起步）不在预测模型里，由控制器的起步阶段处理 |
| 转向 | `δ_target = clip(gain·u + offset, min, max)`，按速率限制逼近目标。新车 gain 为负（正请求向右转），大小和零偏待测 |
| 优化 | 每步沿上一步解的平移序列线性化一次（RTI，1 次 SQP），组成凸 QP，用 OSQP 求解。v3 起 OSQP 只建一次，之后每步更新并热启动。N = 10，dt_pred = 0.1 s。笔记本上单步 p95 约 3 ms |
| 约束 | 转角范围和变化率；动作范围；**车在动时油门下界 = d0 + 偏置**（v3）。静止或起步阶段下界为 0。不下发负油门 |
| 代价 | 横向误差、航向误差、速度误差；转角相对曲率前馈的偏差；油门相对稳态值的偏差（稳态值包含死区）；两个输入的变化量；终端加权 |
| 积分作用（v3） | 油门偏置估计：用模型一步预测的速度残差积分（`drive_bias_gain`），在 ±`drive_bias_max` 内平移有效死区 |
| 起步（v3） | 静止时先开环给 `drive_breakaway`，按 `breakaway_ramp_per_s` 逐步加大，转向仍用 MPC 的解；速度达到 `moving_speed_mps` 后交给 MPC。超时没动，或者回到静止的次数超过上限，都会锁停 |
| 失效处理（v3） | 可容忍：参考不可用、更新迟到（不超过 `dt_hard_max_s`）、超出计算预算。这些情况下继续执行上一步的输入序列，油门不增加，最长 `reference_dropout_tolerance_s`。立即锁停：Planning 停车请求、超速、非有限值、求解失败、更新间隔过长、起步超时 |
| 参数来源 | `vehicle.*` 记录，字段与辨识组的 `VehicleParams` 一一对应。`valid: false` 时只能跑 shadow |

**为什么用联合 MPC**：预测模型已经包含纵向方程；联合优化可以利用速度和转向的耦合。MVP 的共享速度 PI 已经合入同一个文件，作为基线（`control.enabled` 和 `mpc.enabled` 二选一）。

**代价和风险（报告里必须说明）**：

1. 和 Baseline 的比较是**整套控制架构的比较**，差异不能全部归因于横向控制器。仿真里跑完 3 m 的差距（MPC 24/25，MVP 0/25），主要来自失效处理方式不同：MVP 遇到任意一帧规划失败就锁停。
2. 偏置估计在仿真里能把稳态速度误差收敛到约 0（死区偏差 0.011 时）。实车效果和电池漂移的影响**待验证**。
3. 零油门或低油门时实测是**拖刹**（约 0.5 m/s²），不是滑行。不下发负油门，所以没有主动制动。
4. 纵向拟合只来自一台车、一天、同一个电量状态。仿真标定显示起步峰值被低估约 0.07 m/s（仿真 0.261，实车 0.333），超速余量已经按这个差留出。
5. 转向 gain 的大小、零偏和轴距目前是课程占位值。仿真里 ±0.03 rad 的零偏会造成约 0.02 m 的稳态偏移。

**可选扩展（未实现）**：横向 MPC + 共享速度 PI 的对照配置，用来做"只换横向控制器"的比较；横向偏置估计（只有现场零偏导致偏移超过 0.1 m 时才需要）。

## 6. 接口（已实现）

| 接口 | 当前实现 | 状态 |
| --- | --- | --- |
| 当前状态 | estimator 输出的 `state`（`speed_mps`、`timestamp_s`、`speed_valid`，base_link） | ✅ |
| Planner 路径 | `ReferenceTrajectory v0.1` 点列（`s_m, x_m, y_m, yaw_rad, curvature_1pm, target_speed_mps`），带 `timestamp_s`、`valid_for_s`、`stop_required`。Planning 的停车请求在点列里表现为 `valid=True` 且 `stop_required=True`，走 stop 分支（v3 修复） | ✅（Planning V1 只有直线） |
| 车辆参数 | `vehicle.*`（`VehicleParamsSettings`），带 `valid` 和 `source`；同时给 MVP 的 calibrated 模式提供 `steering_limit_rad` 和 `steering_direction` | ✅，新增字段待辨识组确认 |
| 预测模型 | `course_model_step(state, control, dt, params)` | ✅ |
| MPC 输出 | `drive`、`steer` 和 `mpc_debug`（branch、拒绝原因、阶段、偏置、计划序号、候选值、`delta_est_rad`、预测终点、耗时和 p95） | ✅ |
| 安全预算 | 3 m 距离和 30 s 时间（`control.max_distance_m` 和 `control.max_run_time_s`），MPC 执行时同样生效 | ✅ |

坐标系为车辆局部 base_link。warm start 只沿用**输入序列**（平移一步），不沿用旧坐标下的状态。

## 7. 几何路径如何供 MPC 使用

在路径上投影每个预测步的名义位置，取投影点的位置、航向、曲率和目标速度作为参考。预测超出路径末端时直接拒绝（`reference_too_short`），不重复最后一个点。

起点之前允许向后延长：默认 0.6 m，**实车配置为 2.0 m**。原因是 MVP 配置下路径从车前 0.9 m 左右才开始，而 Planning V1 输出的是直线，向后延长是精确的。弯道路径上不能沿用这个延长长度，Planning V2 出来后要重新评估。

## 8. 阶段与当前状态

| 阶段 | 内容 | 状态 | 对应团队里程碑 |
| --- | --- | --- | --- |
| A 运行链 | 求解器依赖、转向正负和映射、纵向参数需求、监督逻辑 | 🟡 转向符号已知（−1）；纵向已从日志拟合；**Jetson 上 OSQP 未验证** | M0（运行约定） |
| B0 模块测试 | 预测模型、参考、求解器、控制器 | ✅ 主测试中 MPC 70 项 | M2 |
| B1 闭环仿真 | 自写被控对象；Dream Gym 与 PID 对比 | ✅ | M6（仿真部分） |
| B2 扰动和故障 | 噪声、偏移、丢帧、死区漂移、转向零偏、延迟 | ✅ v3 场景矩阵 | M6 |
| B3 ROS 与车载平台 | ROS 门禁；Jetson 计时（G1） | 🟡 ROS 门禁合并后重跑通过（10-08，`26a140e`，143 项）；**G1 未做** | M6（计时部分） |
| C → M0 真车 | shadow（G2）；低速执行 0.5 m → 1 m → 3 m 直线 | ⏳ 需要 B3，以及现场测量后把 `vehicle.valid` 改为 true | M7 |
| D → M1 弯道 | 弯道跟踪，在条件一致的前提下与 Baseline 对比 | ⏳ 依赖 Planning V2 | M7 |
| E → M2 障碍 | 跟踪避障路径或执行停车 | ⏳ 依赖 Planning 的障碍输出，以及"停车还是绕行"的决定 | M5、M7 |
| F 证据 | 仿真和实车分开报告；架构对比的说明；局限 | 持续进行 | M8 |

**当前位置：V0（虚拟闭环）基本完成，ROS 门禁已通过，B3 只剩 Jetson 计时（G1），还没进入真车 M0。**

## 9. 后续开发计划（按周）

日期留空，由各人在 Planner 中填写。"上车时段"指团队能用车的那一次现场时间；每周的工作都以能否用车为分界。

### 本周（10-07 这一周）：上车前全部离线完成

| 负责 | 任务 | 交付物 | 完成标准 | 日期 |
| --- | --- | --- | --- | --- |
| IT2 | 合并后在 WSL 重跑 ROS 门禁 ✅ | `~/ai4r-evidence/ros-20261008-135127/summary.txt` | `verify_fast.sh` PASS | 10-08 |
| IT2 | 准备 Jetson 依赖检查和计时脚本（numpy、scipy、osqp 版本；N=10 和 N=5） | 脚本和预期输出 | 能一条命令运行 | |
| IT2 + 机械 | 部署清单：脚本 SHA256、生效参数 dump、启动命令、run ID、overlay 层次 | `docs/` 里的清单 | 复盘文档 P0 要求的每一项都有对应 | |
| 机械 | 和辨识组确认 4 个新字段；写现场测量步骤（转向 gain、零偏、轴距） | 字段确认记录；测量步骤 | 双方确认 | |
| 机械 | 改进起步拟合：峰值低估 0.07 m/s | 更新后的 `newcar27_drive_fit.json` | 仿真峰值误差 ≤0.05 m/s | |
| IT1 | 用 `NewcarPlant` 场景矩阵重新整定权重，评分加重平滑性；确认 N | `offline/mpc_gym/results/` 中新的对比 | 跑完数不低于现有权重，转向变化率下降 | |
| IT1 | 设计 M1 弯道场景（bypass 仿真）和指标 | 场景表 | 有明确的通过线 | |
| 三人 | 把分工调整和 v2、v3 的关键决策提交给团队决策表；向 Planning 发出 §10 的问题 | 决策表条目；消息记录 | 组会确认 | |

### 下周：第一次上车，目标是 M0（直线）

现场顺序，每一步通过后才进入下一步：

1. 遥控确认动力和零输出（共同）；
2. 原地转向：测符号、零偏、gain 的大小（机械）；用尺子量轴距（机械）；
3. Jetson：依赖检查和 G1 计时，车不使能（IT2）；
4. 用 MVP 跑 0.5 m，作为基线复测，也确认电池状态（共同）；
5. MPC shadow（G2）：**推车**检查横向误差和航向误差的符号、拒绝原因、耗时；以约 0.2 m/s 推车时，`candidate_drive` 应接近拟合的稳态值 0.297（IT2，IT1 记录指标）。shadow 时不能让 MVP 同时开车，因为两者互斥；和 MVP 油门（约 0.30）的对比已经由离线拟合完成；
6. 填入实测值，把 `vehicle.valid` 改为 true，`v_exec_max_mps` 设为 0.2；
7. MPC 执行 0.5 m → 1 m → 3 m 直线（共同）。

交付物：实车记录（每轮一个 run ID）、`mpc_debug` 日志、G1 耗时、shadow 统计、现场测得的参数。

### 第 3 周：M1 弯道，以及和 Baseline 的实车对比

- 前提：Planning V2 能输出弯道路径。如果没有，弯道只做 bypass 仿真和 shadow，实车只做直线对比。
- IT1：在条件一致的前提下与 Baseline 对比（同一辆车、同一赛道、同一电量窗口、相同运行次数）。
- IT2：根据实车耗时决定最终的 N 和 `max_step_time_s`。
- 机械：根据实车数据更新模型（转向、纵向、电量漂移）。

### 第 4 周：M2 障碍和证据整理

- 前提：Planning 的障碍输出，以及"停车还是绕行"的决定。
- 如果只停车：验证停车请求的响应时间和停车距离（拖刹）。
- 如果要绕行：跟踪 Planning 给的绕行路径，并重新评估向后延长长度和参考长度。
- 三人：整理证据（团队 M8）——仿真和实车分开报告，写明架构对比的说明和局限。

**范围收缩的取舍**：

- 优先：可靠的直线联合 MPC，加上充分的 shadow 和低速实车验证；
- 其次：弯道；
- 可以取消：横向 MPC + PI 的对照配置、横向偏置估计、更复杂的动力学模型。

## 10. 需要其他组回答的问题

| 问题 | 对象 | 影响 |
| --- | --- | --- |
| Planning V2（弯道路径）什么时候能出？输出格式是否沿用 `CARTESIAN_Y_OF_X` 多项式？ | Planning | M1 的开始时间；MPC 参考适配器是否要改 |
| 遇到障碍，只停车就算达标，还是必须绕行？ | Planning 牵头，全队决定 | M2 的工作量 |
| 停车请求的格式（目前是 `stop_requested=True`）是否会变？ | Planning | 停车分支 |
| `drive_deadband`、`brake_gain_n`、`drive_breakaway`、`breakaway_wait_s` 这 4 个字段是否接受？ | Identification | `vehicle.*` 能否冻结 |
| 速度上限和安全余量由谁定？ | Planning 与 MPC | 超速阈值的归属 |

## 11. 代码组织（单文件部署约束下的实际位置）

车上只安装 `scripts/policy_node.py`，所以运行时代码都在这一个文件里，离线目录保存基准实现和实验。

| 模块 | 位置 |
| --- | --- |
| 预测模型 | `policy_node.py: course_model_step`；基准实现在 `offline/mpc_prediction_model/vehicle_model.py` |
| 参考构建 | `reference_trajectory_from_planning` / `_from_road`、`mpc_path_projector`、`mpc_reference_errors` |
| 目标、约束、优化 | `VehicleModelMPC` |
| 控制器 | `MPCController`（门禁、起步、偏置估计、沿用计划、日志） |
| MVP 基线 | `ControlSettings`、`PolicyController`、`RunDistanceLimiter` |
| 车辆参数 | `VehicleParamsSettings`（`vehicle.*`）；离线记录在 `offline/vehicle_identification/vehicle_params.py`；日志拟合在 `fit_drive_from_logs.py` |
| 测试 | `tests/test_mpc.py`、`tests/test_control.py`、`offline/mpc_prediction_model/test_vehicle_model.py`、`offline/mpc_gym/test_mpc_gym.py`、`offline/vehicle_identification/test_vehicle_identification.py` |
| 仿真与对比 | `offline/mpc_gym/`：`pipeline_sim.py`（实车条件和 `NewcarPlant`）、`newcar27_experiment.py`（场景矩阵） |
| 配置层次 | `ai4r_policy.yaml` → `ai4r_policy_newcar27.yaml`（实车 MVP）→ `ai4r_policy_mpc_newcar27.yaml`（MPC，默认 shadow）；仿真原型用 `ai4r_policy_mpc_prototype.yaml` 和 `_bypass.yaml` |

文档：入口 [MPC_README.md](MPC_README.md)；运行约定和放行门槛 [MPC_PROTOTYPE.md](MPC_PROTOTYPE.md)；车辆参数 [VEHICLE_PARAMS_INTEGRATION.md](VEHICLE_PARAMS_INTEGRATION.md)；上车操作 [dream-car-babysitter-guide.md](dream-car-babysitter-guide.md)；仿真 [offline/mpc_gym/README.md](../offline/mpc_gym/README.md)。

课程要求的 Teams、Task Board、Weekly Summary 照常维护；关键选择要记入团队决策表。
