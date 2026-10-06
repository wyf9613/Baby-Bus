# MPC Plan Overview：从最小真车闭环到系统集成与对比实验（v2）

日期：2026-10-06（v1：2026-10-03）。适用对象：MPC 子组三人——机械（预测模型与部署）、IT1（目标、约束与评估）、IT2（优化、求解与控制循环）。分工对应团队计划 3.4：Zhenyu Zhang、Ying Xu、Yuzaiyang Fan。

**v2 主要变更：第一版技术路线由"横向 MPC＋共享 PI/PID 速度控制"改为纵横向联合 MPC**：预测模型使用 `offline/mpc_prediction_model` 的车辆模型，MPC 同时输出驱动和转向。各节的相应调整用"v2"标出。当前进度和下一步见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md)。

## 1. 目标与边界

子组目标：设计、实现并验证一个能在目标车辆计算平台上及时运行、跟踪共享 Planner 路径的 MPC controller，形成模型、代价与约束、优化与运行三个方面的个人贡献证据。

按三个层次推进：

- **V0：最小虚拟闭环（M0 的前置验证）。** 用仿真车辆运行与真车相同的 MPC 控制代码，在直线和缓弯上闭环；完成基本异常输入测试与目标平台试运行后进入 M0。
- **M0：最小真车闭环。** 低速、无障碍、直线与缓弯；**v2：MPC 同时控制驱动和转向**；有结果检查、停止逻辑和日志。
- **M1：可靠的路径跟踪。** 扩展弯道与速度条件，验证模型误差、控制平滑性、参考更新与实时运行；与 Baseline 做匹配条件的比较。**v2：这一比较是整套控制架构的比较**（见第 2 节）。
- **M2：整体任务集成。** 跟踪 Planning 组生成的避障路径与速度参考，支持停止、恢复和到达目标；验证包含弯道与障碍的完整任务。

障碍前停止可以作为中间验收行为；如果团队目标包含绕过障碍后到达终点，仅停止不代表最终目标完成。阶段和验收数字是建议，不是课程规定。

## 2. 技术路线（v2）

**当前版本：纵横向联合 LTV-MPC。**

| 项目 | 选择 |
| --- | --- |
| 预测模型 | 课程 notebook 车辆模型的运动学版本（`offline/mpc_prediction_model/vehicle_model.py`）。已用 Dream Gym 开环验证：直行、转弯、蛇形工况误差 ≤ 1e-6 m。状态 `[x, y, ψ, v, δ]`（CG，base_link），输入 `[drive, steering_action]`（归一化） |
| 纵向 | `m·v̇ = k·drive − c·v|v|` |
| 转向 | `δ_target = clip(gain·u + offset, min, max)`，按转向速率限制逼近目标值 |
| 优化 | 每步沿上一步解的平移序列做一次线性化（RTI，1 次 SQP），组成凸 QP，用 OSQP 求解；N = 10，dt_pred = 0.1 s |
| 约束 | 转角范围和变化率、动作范围；驱动只取正值 `[max(0, drive_min), drive_max]`（模型没有倒车锁存） |
| 代价 | 横向误差、航向误差、速度误差；转角相对曲率前馈的偏差；驱动相对稳态值的偏差；两个输入的变化量；终端加权 |
| 参数来源 | `vehicle.*` 记录，字段与辨识组 `VehicleParams` 一一对应；未实测时只允许 shadow 模式，使用 `course_simulation` 参数 |

**为什么改**：预测模型已经包含经过验证的纵向方程；联合优化可以直接利用速度与转向的耦合；`origin/main` 上的共享速度 PI 属于 PID 组的 MVP，没有合入本分支。

**代价和风险（必须在报告中说明）**：

1. 与 Baseline（横向 PID＋纵向 PI）的比较是**整套控制架构的比较**，差异不能都归因于横向控制器（v1 第 6F 节的要求）。
2. MPC 没有积分作用：驱动或阻力与模型不符时会有稳态速度误差。Gym 失配场景中为 0.027 m/s，PID 为 0.007 m/s。
3. 只能滑行停车，不能主动制动；停车距离依赖实测的滑行特性。
4. 纵向模型的参数（m、k、c）必须由辨识组实测。ESC 死区和静摩擦不在模型内。

**备选（未实现，按需评估）**：横向 MPC＋共享速度 PI 的对照配置，用于"只换横向控制器"的比较；在 MPC 中加入速度扰动估计或积分作用。

## 3. 架构与组间边界

Estimation 提供状态与时间戳；Planning 提供几何路径、速度参考和行为模式；IT1 把路径转成时域参考，并定义误差、代价和约束；机械提供预测模型和参数映射；IT2 组装并求解优化问题，**v2：输出驱动和转向候选**。MPC 自带门禁和拒绝逻辑：任何拒绝都进入框架的状态 2（零动作），需要显式请求 state 3 才能恢复。

| 模块/小组 | 主要责任 | 向 MPC 提供或接收的内容 |
| --- | --- | --- |
| Estimation/Identification | 状态估计、参数测量 | 速度、yaw rate、道路几何、有效性、时间戳；**`VehicleParams`（运行时为 `vehicle.*`）** |
| Planning | 路径选择、障碍相关性、停止/目标行为 | 路径（V1 为一次多项式，经适配器转成 v0.1 点列）、速度参考、有效期、stop 状态 |
| MPC 机械 | 预测模型、动作映射、参数解释、车辆侧验证 | `course_model_step`、转向和驱动映射、模型验证 |
| MPC IT1 | 参考适配、误差、代价和约束、评估 | 时域参考、权重、约束、场景与指标 |
| MPC IT2 | 问题组装、求解、控制循环、时间预算 | 驱动和转向候选、预测序列、状态、耗时、`mpc_debug` 日志 |
| Baseline（PID 组） | 横向 PID＋纵向 PI | 对比基线；**v2：MPC 不调用共享速度控制** |

## 4. 接口（v2：已实现的部分）

| 接口 | 当前实现 | 状态 |
| --- | --- | --- |
| 当前状态 | estimator 输出的 `state`（`speed_mps`、`timestamp_s`、`speed_valid`，base_link） | 已接入 |
| Planner 路径 | `ReferenceTrajectory v0.1` 点列（`s_m, x_m, y_m, yaw_rad, curvature_1pm, target_speed_mps`），带 `timestamp_s`、`valid_for_s`、`stop_required` | 已接入；Planning V1 只支持直线，弯道走 bypass（估计器中线二次拟合） |
| 车辆参数 | `vehicle.*`（`VehicleParamsSettings`），加上 `valid` 和 `source` | 已实现；新增字段**待辨识组确认** |
| 预测模型 | `course_model_step(state, control, dt, params)`，与离线模型逐行一致（有测试） | 已实现 |
| MPC 输出 | `drive`、`steer` 和 `mpc_debug`（branch、拒绝原因、候选值、`delta_est_rad`、预测终点、耗时） | 已实现 |
| 上次实际输入 | 控制器记录已应用指令的历史，用来回放转角估计和延迟推演 | 已实现 |

坐标系为车辆局部 base_link。warm start 只沿用**输入序列**（平移一步），不沿用旧坐标下的状态；拒绝或重启时会清空。

## 5. 几何路径如何供 MPC 使用

在路径上投影每个预测步的名义位置，取投影点的位置、航向、曲率和目标速度作为参考。预测超出路径末端时直接拒绝（`reference_too_short`），**不重复最后一个点**；起点之前最多向后延长 0.6 m。已有测试覆盖短路径、重复点、非有限值、过期和未来时间戳、`stop_required`、零目标速度。预测窗口受参考长度约束：增加 N 前要先确认参考覆盖长度足够。

## 6. 分阶段执行与验收

阶段划分与 v1 相同，下面只列 v2 的调整。各阶段的完成情况记录在进度日志里。

- **阶段 A（运行链）**：IT2 在 Jetson 上验证 OSQP 和耗时（G1）。机械与辨识组确认转向正负、偏置、映射，以及纵向参数需求（m、k、c、驱动范围、延迟）。还需明确监督逻辑的负责人：目前是 MPC 自带门禁加框架的状态机。
- **阶段 B（V0）**：
  - B0 模块测试 ✅；
  - B1 闭环仿真：自写被控对象，加上 Dream Gym 与 PID 的条件一致对比 ✅；
  - B2 扰动和故障注入：大部分 ✅；
  - B3 ROS 门禁和 Jetson 无动作运行：待完成。
- **阶段 C（M0）**：先 shadow 运行（G2），再在低速下**接管驱动和转向**。前提是 `vehicle.*` 已实测并设为 `valid`。停止依靠零指令滑行加 RC 急停，路线长度按实测滑行距离预留。
- **阶段 D–F**：与 v1 相同。对比报告必须写明是架构对比；虚拟测试和实车测试分开报告。

## 7. 职责与交付物

| 成员 | 持续负责 | 当前对应代码和证据 |
| --- | --- | --- |
| 机械 | 模型、离散化和线性化、仿真被控对象、参数解释、动作映射 | `offline/mpc_prediction_model/`、`course_model_step`、模型一致性测试；**向辨识组提交参数需求** |
| IT1 | 参考适配、代价、约束、权重、场景与指标 | 参考适配器、`mpc_path_projector`、代价项；`offline/mpc_gym` 的场景、指标和权重整定 |
| IT2 | 优化组装、求解器、控制循环、日志、时间预算 | `VehicleModelMPC`、`MPCController`、`mpc_debug`；ROS 门禁、Jetson 计时 |
| 三人 | 接口、集成、现场测试、最终对比 | `docs/`、对比图表、实车记录 |

## 8. 代码组织（单文件部署约束下的实际位置）

车上只安装 `scripts/policy_node.py`，所以运行时代码都在这一个文件里，离线目录保存基准实现和实验。

| 模块 | 位置 |
| --- | --- |
| vehicle_model | `policy_node.py: course_model_step`；基准实现在 `offline/mpc_prediction_model/vehicle_model.py` |
| reference_builder | `reference_trajectory_from_planning` / `_from_road`、`mpc_path_projector`、`mpc_reference_errors` |
| objective_constraints / optimizer | `VehicleModelMPC` |
| controller | `MPCController`（门禁、拒绝、回放、日志） |
| 车辆参数 | `VehicleParamsSettings`（`vehicle.*`）；离线记录在 `offline/vehicle_identification/vehicle_params.py` |
| 测试 | `tests/test_mpc.py`、`offline/mpc_prediction_model/test_vehicle_model.py`、`offline/mpc_gym/test_mpc_gym.py` |
| 仿真与对比 | `offline/mpc_gym/`（复用 `offline/control_pid/` 的 Gym 实验，未作改动） |
| 配置 | `config/ai4r_policy.yaml`（`mpc.*`、`vehicle.*`）及 overlay |

## 9. 排期与范围收缩

先完成 B3（ROS 门禁、Jetson G1/G2），再在最早可用的车辆时段完成 M0。时间不足时的取舍：

- 优先：可靠的联合 MPC，加上充分的 shadow 和低速验证；
- 可取消：速度积分或扰动估计、横向 MPC＋共享 PI 的对照配置、更复杂的动力学模型。

课程要求的 Teams、Task Board、Weekly Summary 照常维护，关键选择（例如本次路线变更）要记入决策表。

## 10. 立即任务

见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 中的"下一步"。
