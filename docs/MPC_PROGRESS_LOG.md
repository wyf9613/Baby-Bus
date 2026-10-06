# MPC 进度日志

对应计划：[MPC_PLAN_OVERVIEW.md](MPC_PLAN_OVERVIEW.md)（v2）。新条目加在最上面。状态标记：✅ 完成，🟡 部分完成，⏳ 待做，❌ 阻塞。

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
