# MPC 子组 README

本文是 MPC（Model Predictive Control）子组的入口文档。读完可以知道：做到哪一步、和你们组之间的接口、代码和配置在哪、怎么验证，以及细节去哪里看。正文用中文，代码名、字段名、参数名保持英文。

> 更新于 2026-10-07，对应 `control-mpc-v0` 分支提交 `3acd32d`（v3）。目标车辆：新车 10.43.254.27（下文简称 .27）。

## 1. 现状

**V0（虚拟闭环）基本完成；ROS 门禁合并后已重跑通过（10-08）；车上 Jetson 没有 osqp，改用只依赖 numpy 的求解器（`mpc.qp_solver: dense`），10-08 在 Jetson 上通过 G1（N=10 单步 max 12.9 ms）；下一步车上 IMU 排查和 shadow；还没上真车。**

| 项目 | 状态 |
| --- | --- |
| 联合 MPC（同时输出驱动和转向）实现与单元测试 | ✅ 主测试中 MPC 70 项通过 |
| 纵向模型按 .27 实车日志拟合（死区、拖刹、静摩擦） | ✅ 速度 RMSE 0.024–0.032 m/s |
| 实车条件仿真（20 Hz timer、0.2 s 相机延迟、丢帧） | ✅ 25 个场景中 MPC 24 个跑完 3 m，MVP 0 个（原因见 §8） |
| ROS 门禁（`tools/verify_fast.sh`） | 🟡 合并前通过；合并后需在 WSL 重跑 |
| Jetson 计时（G1）、车上 shadow（G2） | ⏳ 需要用车时段 |
| 真车执行（M0 直线 → M1 弯道 → M2 障碍） | ⏳ |

- 详细阶段表：[MPC_PLAN_OVERVIEW.md §8](MPC_PLAN_OVERVIEW.md#8-阶段与当前状态)
- 按周计划：[MPC_PLAN_OVERVIEW.md §9](MPC_PLAN_OVERVIEW.md#9-后续开发计划按周)
- 每天的进展和证据：[MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md)

## 2. MPC 做什么、不做什么

**做：**
- 跟踪 Planning 给出的路径，**同时**输出驱动（drive）和转向（steer）；
- 执行 Planning 的停车请求；
- 自己检查输入是否可用，拒绝不可用的输入，并在 `mpc_debug` 里记录原因。

**不做**（依据团队计划 [TEAM_PROJECT_PLAN.md](TEAM_PROJECT_PLAN.md) §3.2、§3.4）：
- 生成弯道参考路径：归 Planning；
- 判断障碍是否相关、决定停车还是绕行：归 Planning；
- 车辆参数辨识实验：归 Identification；
- 状态估计、道路几何：归 Estimation。

所以 **MPC 不会自己避障**。障碍怎么处理由 Planning 决定，MPC 执行 Planning 给的停车请求，或者跟踪 Planning 给的绕行路径。

## 3. 和各组的接口

所有运行时代码都在 [scripts/policy_node.py](../scripts/policy_node.py)。下文中的函数名和类名都在这个文件里搜得到。

### 3.1 Estimation（Yiming Zhang）

MPC 读取 `self.estimation_output`：

| 记录 | 字段 | 用途 |
| --- | --- | --- |
| `state` | `speed_mps`、`speed_valid` | 预测的初始速度；`speed_valid` 为 False 时这一步拒绝（`speed_invalid`） |
| `state` | `timestamp_s` | 参考时间：MPC 用它检查参考有没有过期 |
| `state` | `yaw_rate_valid` | 只在 bypass 模式检查 |
| `road`（只在 bypass 模式用） | `valid`、`time_aligned`、`frame_id`、`timestamp_s`、`source_age_s`、`centerline_xy` | 直接拟合中线作为参考（`reference_trajectory_from_road`） |

约定：坐标系 `base_link`（x 向前、y 向左，米、弧度）；道路几何必须已经对齐到 `state.timestamp_s`。实车相机延迟约 0.2 s，所以实车配置的源时效是 `planning.max_source_age_s: 0.4`。

### 3.2 Planning（Xinkai Zhang、YueShan Li）

MPC 读取 `self.planning_output`，经适配器 `reference_trajectory_from_planning` 转成点列（`ReferenceTrajectory v0.1`）：

| Planning 字段 | MPC 怎么用 |
| --- | --- |
| `valid` | False 时属于**可容忍失效**：在 `reference_dropout_tolerance_s` 内继续执行上一步的计划，超时就锁停 |
| `stop_requested` | 和 `valid=True` 同时出现时是**停车请求**：立即走 `stop` 分支，零动作并锁停。不会被当成可容忍失效 |
| `frame_id`、`timestamp_s` | 必须是 `base_link`；时间戳用来判断参考是否过期 |
| `valid_for_s` | 参考的有效期；过期也属于可容忍失效（`ref_expired_or_future`） |
| `path` | 目前只认 `CARTESIAN_Y_OF_X` 多项式（`coeffs_low_to_high`、`range`），按 `mpc.reference_spacing_m` = 0.05 m 采样成点列 |
| `target_speed_mps` | 每个点的目标速度；为 0 时零油门（`zero_target`） |
| `simulation_only` | 记录到日志 |

两种规划配置：
- **mvp**（实车配置使用，`control.mode: mvp`）：只检查几何、时效和工作范围，不要求实测的车身轮廓和制动参数；路径从实际观测到的最近点开始（实车约 0.9 m 处）。
- **upstream**：额外检查车身间距和停车距离，需要有效的 `vehicle.*` 记录。

路径起点在车前方，MPC 会把第一段**向后延长**到车的位置（实车配置为 2.0 m）。这只对直线精确；Planning 给出弯道路径后需要重新评估。

**我们需要 Planning 提供**：
- V2 弯道路径（M1 的前提）；
- 障碍响应方案，以及"只停车还是要绕行"的决定（M2 的前提）；
- 如果停车请求的格式要变，提前告知。

详见 [MPC_PLAN_OVERVIEW.md §10](MPC_PLAN_OVERVIEW.md#10-需要其他组回答的问题)。

### 3.3 Identification（Zhengxi Chen）

车辆参数通过 ROS 参数 `vehicle.*` 传入（类 `VehicleParamsSettings`），字段和离线的 [vehicle_params.py](../offline/vehicle_identification/vehicle_params.py) 一一对应（有测试检查）。

- 单位：米、弧度（左为正）、秒；drive 和 steering 都是归一化请求。
- 转向映射：`δ = steering_gain_rad · u + steering_offset_rad`。gain 为负表示正请求向右转（.27 就是这样）。
- 纵向：`m·v̇ = gain·(drive − drive_deadband) − drag_kg_per_m·v·abs(v)`；drive 不低于死区时 gain = `motor_gain_n`，低于死区时 gain = `brake_gain_n`（电调拖刹）。
- 静止起步：请求至少 `drive_breakaway` 并保持 `breakaway_wait_s` 后车轮才转。
- `valid: false` 表示记录未获准上车，只能跑 shadow。`valid: true` 需要 `source` 写明可追溯的来源，并且通过一致性检查（`problem()`），否则节点拒绝启动。

**待确认**：v3 新增的 4 个字段 `drive_deadband`、`brake_gain_n`、`drive_breakaway`、`breakaway_wait_s`。

完整字段对照见 [VEHICLE_PARAMS_INTEGRATION.md](VEHICLE_PARAMS_INTEGRATION.md)。

### 3.4 Baseline 和集成（Yifan Wu、JiaHao Ni）

- MVP（PI 控速 + P 控转向）和 MPC 在同一个文件里：`control.enabled` 和 `mpc.enabled` **只能开一个**，同时开节点拒绝启动。MPC 也不能和 `id_test` 同时开。
- 3 m 距离和 30 s 时间限制（`control.max_distance_m`、`control.max_run_time_s`，由 `RunDistanceLimiter` 和 `_control_problem` 实现）对 **MPC 执行**同样生效；shadow 模式下不生效（`_budgets_active`）。
- `vehicle.*` 只有一套。MVP 的 calibrated 模式需要的 `steering_limit_rad`、`steering_direction` 由 `VehicleParamsSettings.records()` 推导。
- `debug1`/`debug2`：MVP 模式下是路径误差（m）和累计里程（m）；MPC 模式下是 `e_y_m` 和 `candidate_delta_rad`。

## 4. MPC 的输出

### 4.1 动作

- `drive`：归一化油门，**不下发负值**。车在动时不低于"死区 + 偏置估计"（约 0.29），这一点对应的是零驱动力，也就是开始滑行；更低的请求在 .27 上会触发拖刹。
- `steer`：归一化转向，范围由 `vehicle.steering_min_rad`、`steering_max_rad` 和 gain 换算。
- shadow 模式下两者恒为 0。

### 4.2 `mpc_debug` 话题

类型 `std_msgs/String`，内容是 JSON，每个 policy 步一条（包括被拒绝的步）。

| 字段 | 含义 |
| --- | --- |
| `branch` | 这一步走了哪个分支，见下表 |
| `reject_reason` | 拒绝原因，例如 `stale_road`、`ref_expired_or_future`、`dt_late`、`overspeed`、`no_motion_after_breakaway`、`too_many_restarts`、`reference_too_short`、`origin_before_path_start` |
| `phase` | `starting`（起步阶段）或 `tracking` |
| `drive_bias` | 油门偏置估计值 |
| `plan_index`、`dropout_s` | 沿用计划时用的是第几步、失效持续了多久 |
| `candidate_drive`、`candidate_steer_action`、`candidate_delta_rad` | MPC 解出的候选值 |
| `applied_drive`、`applied_steer_action` | 实际下发的值（shadow 下为 0） |
| `e_y_m`、`e_psi_rad` | 横向误差（车在路径左侧为正）、航向误差 |
| `speed_mps`、`v_ref_mps` | 实测速度、参考速度 |
| `pred_e_y_end_m`、`pred_v_end_mps` | 预测时域末端的横向误差和速度 |
| `delta_est_rad` | 用已下发指令回放出的前轮角估计（没有轮角传感器） |
| `solve_s`、`step_s`、`step_p95_s`、`iterations` | 求解耗时、单步耗时、最近 200 步的 p95、OSQP 迭代数 |
| `dt`、`ref_age_s`、`path_id`、`status` | 时间间隔、参考的年龄、路径编号、求解状态 |
| `vehicle_source`、`vehicle_valid` | 用的是哪条车辆记录、是否有效 |
| `shadow`、`simulation_only`、`reference_source`、`planning_bypassed` | 运行模式标记 |

| `branch` | 含义 | 执行模式下的后果 |
| --- | --- | --- |
| `solved` | 正常求解 | 下发 |
| `shadow` | 正常求解，但处于 shadow | 下发 0 |
| `zero_target` | 目标速度为 0 | 零油门 |
| `plan_hold` | 可容忍失效，沿用上一步计划（油门不增加） | 下发；超过容忍时长则锁停 |
| `ref_hold` | 可容忍失效，但还没有可沿用的计划 | 零油门，保持转向 |
| `ref_invalid` | 参考或输入不可用 | 锁停 |
| `stop` | 停车请求、超速、起步失败 | 锁停 |
| `solver_fail`、`over_budget` | 求解失败、超出计算预算（超预算属于可容忍失效） | 锁停 |
| `gate_refused`、`vehicle_invalid` | 执行门槛不满足、车辆记录不一致 | 锁停 |

"锁停"指框架进入状态 2（持续零动作），必须显式请求 state 3 才能恢复，恢复时控制器从零重建。

**排查顺序**：先看 `branch` 和 `reject_reason`；再看 `phase`、`speed_mps` 和 `candidate_drive`，判断是起步问题还是跟踪问题；最后看 `step_p95_s`，判断是否超时。

## 5. 配置和运行

### 5.1 三层配置

ROS 按顺序加载多个参数文件，后面的覆盖前面的：

| 层 | 文件 | 内容 |
| --- | --- | --- |
| 1 | [ai4r_policy.yaml](../config/ai4r_policy.yaml) | 包的通用默认值：MVP 开启，MPC 关闭，`vehicle.valid: false` |
| 2 | [ai4r_policy_newcar27.yaml](../config/ai4r_policy_newcar27.yaml) | .27 的实车参数（10-06 实测）：timer 20 Hz、时效和门限、MVP 增益、转向方向 −1 |
| 3 | [ai4r_policy_mpc_newcar27.yaml](../config/ai4r_policy_mpc_newcar27.yaml) | 切到 MPC：关闭 MVP，开启 MPC（**默认 shadow**）；`vehicle.*` 填入日志拟合的纵向参数，几何和转向大小是占位值，`valid: false` |

`ai4r_policy_mpc_prototype.yaml` 和 `ai4r_policy_mpc_bypass.yaml` 用于课程仿真车辆（Gym），不用于 .27。

### 5.2 从 shadow 到执行

- **shadow**（默认）：计算并记录，下发零动作；拒绝不会锁停。车不能被 MPC 驱动。
- **执行**需要同时满足（`MPCController.gate_problem`）：
  - `mpc.shadow: false`；
  - `mpc.vehicle_params_source: vehicle`；
  - `vehicle.valid: true`（转向 gain、零偏、轴距必须先在现场实测）；
  - `mpc.v_exec_max_mps > 0`；
  - bypass 模式另需 `bypass_acknowledged: true`。

上车的具体操作见 [dream-car-babysitter-guide.md](dream-car-babysitter-guide.md)，放行门槛（G0–G4）见 [MPC_PROTOTYPE.md](MPC_PROTOTYPE.md)。

### 5.3 常用参数

| 参数 | .27 取值 | 作用 |
| --- | --- | --- |
| `mpc.v_exec_max_mps` | 0.25（首次执行建议 0.2） | 执行时的速度上限 |
| `mpc.overspeed_margin_mps` | 0.15 | 超过 `v_exec_max_mps` 加这个余量就锁停 |
| `mpc.reference_dropout_tolerance_s` | 0.3 | 可容忍失效最长持续多久 |
| `mpc.drive_bias_gain` / `drive_bias_max` | 2.0 / 0.05 | 偏置估计的增益和上限（0 表示关闭） |
| `mpc.breakaway_ramp_per_s` | 0.05 | 起步油门从 `drive_breakaway` 起每秒增加多少 |
| `mpc.moving_speed_mps` / `breakaway_timeout_s` / `max_restarts` | 0.05 / 1.5 / 3 | 判定已起步的速度、起步超时、回到静止的最多次数 |
| `mpc.dt_max_s` / `dt_hard_max_s` | 0.2 / 0.4 | 更新间隔超过前者沿用计划，超过后者锁停 |
| `mpc.horizon_n` / `dt_pred_s` | 10 / 0.1 | 预测步数和步长（1 s 预测时域） |
| `mpc.max_backward_extension_m` | 2.0 | 路径起点向后延长的最大距离 |
| `mpc.max_step_time_s` | 0.05 | 单步计算预算 |

## 6. 代码地图

车上只部署 `scripts/policy_node.py` 这一个文件，所以运行时代码全部在里面；`offline/` 存放基准实现、拟合脚本和实验。

| 模块 | 位置 | 负责 |
| --- | --- | --- |
| 预测模型 | `course_model_step`；基准实现在 [offline/mpc_prediction_model/vehicle_model.py](../offline/mpc_prediction_model/vehicle_model.py) | 机械 |
| 车辆参数 | `VehicleParamsSettings`、`course_simulation_vehicle_params`；拟合脚本 [fit_drive_from_logs.py](../offline/vehicle_identification/fit_drive_from_logs.py) | 机械 |
| 参考构建 | `reference_trajectory_from_planning`、`reference_trajectory_from_road`、`mpc_path_projector`、`mpc_reference_errors` | IT1 |
| 代价、约束、优化 | `VehicleModelMPC`（`solve`、`input_bounds`、`steady_drive`） | IT1（代价和约束）、IT2（组装和求解） |
| 控制器 | `MPCController`（门禁、起步、偏置估计、沿用计划、日志） | IT2 |
| 参数 | `MPCSettings`（`mpc.*`） | IT2 |
| 节点接入 | `PolicyNode.calculate_policy_actions` 中的 MPC 分支 | IT2 |
| MVP 基线 | `ControlSettings`、`PolicyController`、`RunDistanceLimiter` | Baseline |
| 仿真 | [offline/mpc_gym/](../offline/mpc_gym/README.md) | IT1 |

## 7. 怎么验证

在仓库根目录执行。本环境用 `unittest`，不需要 pytest：

```powershell
# 主测试（预期：control 22、mpc 70、planning 17、estimation 28，全部 OK）
python -m unittest tests.test_control tests.test_mpc tests.test_planning tests.test_estimation

# 离线测试（预期：prediction 6、identification 18、Gym 9、control_pid 20）
cd offline/mpc_prediction_model;  python -m unittest discover -p "test_*.py"; cd ../..
cd offline/vehicle_identification; python -m unittest discover -p "test_*.py"; cd ../..
cd offline/mpc_gym;               python -m unittest discover -p "test_*.py"; cd ../..   # 需要 Dream Gym 0.4.0
cd offline/control_pid;           python -m unittest discover -p "test_*.py"; cd ../..

# 从实车日志重新拟合纵向模型，输出 results/newcar27_drive_fit.json
python offline/vehicle_identification/fit_drive_from_logs.py

# 实车条件仿真：MVP 与 MPC 的场景矩阵，约 1.5 分钟，输出 results/newcar27_comparison.md
python offline/mpc_gym/newcar27_experiment.py --horizons 10 8 5
```

- `test_mpc.py` 会打印笔记本上的单步耗时：目前 mean 约 2.6 ms，p95 约 2.7 ms。
- `tests/test_policy_node.py` 需要 ROS Jazzy，用 WSL 或 Jetson 上的 `.verification/run_ros_check.sh`（即 `tools/verify_fast.sh`）运行。

## 8. 已知限制

- 纵向参数只来自**一台车（.27）、一天（10-06）、一个电量状态**。死区会随电池漂移，偏置估计和起步爬升就是为此设计的，但还没有实车验证。
- 仿真标定：起步延迟（0.50 s，实车 0.53 s）和稳态速度（0.202，实车 0.20）都吻合，**起步峰值低估约 0.07 m/s**（仿真 0.261，实车 0.333）。
- 转向 gain 大小、零偏、轴距是课程占位值。
- 仿真里 MPC 24/25、MVP 0/25 跑完 3 m，差距**主要来自失效处理方式**：MVP 遇到任意一帧规划失败就锁停，MPC 会沿用上一步计划。这不代表跟踪精度差这么多。仿真的丢帧也比实车多。
- 所有仿真结果都不是实车证据；Jetson 上的耗时还没测。
- 只有直线经过 Planning 正式流程；弯道只能用 bypass 模式在仿真和 shadow 中测试。

## 9. 常见问题

**规划失败一帧会怎样？**
执行模式下，在 0.3 s 内继续执行上一步解出的输入序列，并且油门不增加（`plan_hold`）；参考恢复后正常求解。超过 0.3 s 仍未恢复就锁停。Planning 的停车请求不属于这种情况，会立即停车。

**为什么默认是 shadow？**
`vehicle.*` 里的转向参数和轴距还没实测。shadow 可以先在车上检查符号、拒绝原因和耗时，而车不会被 MPC 驱动。

**为什么车在动时油门不会低于 0.289？**
这是 .27 的油门死区：低于它没有驱动力，而且会触发电调拖刹。MPC 只在死区以上做优化，因为那一段模型是线性的。想减速时，MPC 会把油门压到死区边界，相当于滑行。

**弯道现在能测吗？**
Planning V1 只输出直线。弯道可以用 bypass 模式（`reference_source: estimation_centerline`）在仿真和 shadow 中测试，但它没有 Planning 的车身间距和停车距离检查，不能作为验收依据。正式的弯道路径要等 Planning V2。

**MPC 会不会自己避障？**
不会。见 §2。

**shadow 时能不能让 MVP 同时开车，对比两者的油门？**
不能。两种控制器互斥，shadow 时没有控制器在驱动车。车上 shadow 只做推车测试；和 MVP 油门的对比用离线方式：MVP 在 10-06 维持 0.2 m/s 用的油门约 0.30，MPC 模型算出的稳态油门是 0.297（`VehicleModelMPC.steady_drive(0.2)`）。

## 10. 文档索引和联系人

| 文档 | 内容 |
| --- | --- |
| [MPC_PLAN_OVERVIEW.md](MPC_PLAN_OVERVIEW.md) | 计划：分工、组间边界、时间线、技术路线、阶段、按周计划、待回答的问题 |
| [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) | 每天的进展、证据、未决问题、建议写入决策表的条目 |
| [MPC_PROTOTYPE.md](MPC_PROTOTYPE.md) | 英文：运行约定、`mpc_debug`、放行门槛 G0–G4、Jetson/ROS/上车步骤 |
| [VEHICLE_PARAMS_INTEGRATION.md](VEHICLE_PARAMS_INTEGRATION.md) | `vehicle.*` 字段对照、valid/source 规则、模型相关的未解决问题 |
| [dream-car-babysitter-guide.md](dream-car-babysitter-guide.md) | 上车操作手册：连接、部署、shadow、Foxglove、执行 |
| [offline/mpc_gym/README.md](../offline/mpc_gym/README.md) | 仿真实验：与 PID 的 Gym 对比（v2），以及实车条件仿真（v3） |
| [experiments/2026-10-06/](experiments/2026-10-06/实车实验复盘与本周改进计划.md) | 10-06 实车实验复盘和原始日志（v3 的依据） |

| 模块 | 联系人 |
| --- | --- |
| MPC 预测模型、车辆参数 | Zhenyu Zhang（机械） |
| MPC 参考、代价和约束、评估、仿真场景 | Ying Xu（IT1） |
| MPC 求解器、控制器、实时运行、日志 | Yuzaiyang Fan（IT2） |
| 真车部署与测试 | 机械和 IT2 共同负责 |
| Estimation / Identification | Yiming Zhang / Zhengxi Chen |
| Planning | Xinkai Zhang、YueShan Li |
| Baseline / MVP | Yifan Wu、JiaHao Ni |
