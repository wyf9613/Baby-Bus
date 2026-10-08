# MPC 进度日志

对应计划：[MPC_PLAN_OVERVIEW.md](MPC_PLAN_OVERVIEW.md)（v3）。新条目加在最上面。状态标记：✅ 完成，🟡 部分完成，⏳ 待做，❌ 阻塞。

## 2026-10-08：不依赖 OSQP 的 QP 求解器（`mpc.qp_solver: dense`）

车上没有 osqp（见下一条），所以在 `VehicleModelMPC` 里加了只用 numpy 的求解器 `_solve_dense`：Goldfarb–Idnani 对偶积极集法，适用于正定 P（每个输入都带权重，所以成立），有限步得到精确解。新参数 `mpc.qp_solver`：默认 `osqp`，行为不变；`config/ai4r_policy_mpc_newcar27.yaml` 设为 `dense`。状态字与 OSQP 一致（`solved`、`run time limit reached`、`maximum iterations reached`、`primal infeasible`），`solver_time_limit_s` 和 `solver_max_iter` 同样生效，失败时同样锁停。

验证（笔记本 Windows，Python 3.12）：
- 300 个同结构随机 QP 与高精度 OSQP 对比：解的最大差 4e-16，平均 11 次迭代，最多 17 次。
- `tests/test_mpc.py` 新增 `DenseSolver` 4 项与一项配置检查：控制器真实 QP 上与 OSQP 的输入差 ≤1e-4；直线与新车起步闭环的分支序列相同、状态差 <1e-3；时间和迭代上限按 OSQP 的状态字停止；屏蔽 osqp 和 scipy 后仍能求解。
- `MPC_TEST_QP_SOLVER=dense` 时全部 MPC 测试用 dense 运行：75/75 通过；默认 osqp 也是 75/75。control、planning、estimation 67/67，Gym 9/9（newcar27 实验已经走 dense）。
- **车上同版本环境**（干净 venv：numpy 1.26.4、scipy 1.11.4、PyYAML 6.0.1，无 osqp）跑 `tools/jetson_g1_check.sh`：142 项通过、2 项对照测试因无 osqp 跳过；N=10 单步 mean 4.4 ms、max 6.0 ms，无超预算。第一次运行时弯道测试出现过一次 55 ms 的单步（`over_budget`），重跑和单独分析都没有复现：900 次弯道求解最多 9 次迭代、单次 ≤2.8 ms，判断为 Windows 调度抖动。
- 笔记本上两种求解器 N=10 单步 mean：dense 3.4 ms，osqp 3.2 ms。

G1 脚本同步修改：osqp 变为可选；没有 osqp 时测试和计时自动用 dense，两种都有时都测。

未完成：`policy_node.py` 改动后需要重跑 WSL ROS 门禁；Jetson 上的 G1（dense）还没跑。

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
