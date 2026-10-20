# 计划：lattice 规划时的碰撞检查覆盖执行检查窗口（`mpc-car-run`，5 号车）

2026-10-20。给在云端实施的会话，也给组里（Planning 组）看。**云端不需要跑 Gym**，Gym 数据已在本机做完（见第 1 节）；只做代码、单元测试、部署包和文档。

## 1. 背景和依据

- **现象**：急弯（U 弯、直角弯）里，lattice 和中心线来回切换。Gym 里每组 160–320 次切换，20–30% 的拍用中心线。实车上表现为"一段一段走"，参考切换时转向跳变。
- **原因**（[MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 2026-10-20"再续四"）：
  - 急弯里 92–97% 的中心线拍，原因是执行检查 `known_boundary_collision`；
  - lattice 搜索本身几乎都是 `VALID`；
  - 车自身几乎不撞（≤ 1%），撞的是路径前方 0.3–0.5 m，和路径新旧无关。
- **根本原因**：两处碰撞检查的长度不一样。
  - **规划时**（`ShadowLatticePlanner._plan_geometry`）只检查 `check = speed × hold`。其中 `hold = 1/rate_hz + max(handoff_allowance_s, 观测到的往返时间)`，5 号车约 0.20 × 1.1 = **0.22 m**。
  - **执行时**（`PlannerExecution.select`）每拍从车当前在路径上的位置往前，检查到 `max(needed, closest + preview_m + 0.15)`，约 0.35 + 0.15 = **0.5 m**。`preview_m` 是 `PolicyNode._planner_preview_m()`，规划任务里的 `job["preview_m"]` 是同一个值。
  - 一条路径要用约 1.1 s，车在这期间前进 `speed × hold`，所以执行检查最远到 `speed × hold + preview_m + 0.15`，约 **0.72 m**。
  - 规划时只查了 0.22 m，后面那段一到执行就可能撞。这是 mvp2.1 lattice 原本的写法。
- **Gym 验证**（5 号车仿真车，联合 MPC，0.20 m/s，每组 60 次；用 `shadow.handoff_allowance_s` 模拟规划检查长度）：

  | 规划时检查长度 | 对应下面的模式 | 完成 | 急弯 lattice 占比 | U 弯 / R1.5 / R1 切换 | 最大横向误差（U 弯 / R1.5） |
  | --- | --- | --- | --- | --- | --- |
  | 0.22 m | `travel`（现在） | 38/60 | 77–82% | 294 / 164 / 216 | 0.29 / 0.25 m |
  | 0.52 m | ≈ `max` | 37/60 | 97–99% | 83 / 27 / 40 | 0.25 / 0.16 m |
  | 0.72 m | `travel_plus_window` | 34/60 | 98–99% | **22 / 30 / 30** | **0.17 / 0.07 m** |

  完成率仍由估计层失败决定（`invalid_ordered_geometry`），差别在重跑波动附近，实车 A/B 时要留意。
- **CPU**：本机测了 223 次规划，0.22 m 和 0.52 m 的每次规划耗时中位数分别是 93.8 / 91.4 ms，没有可测差别。
  - 路径生成是大头，碰撞检查遇到第一处碰撞就停。
  - `travel_plus_window` 会让路径总长从 1.0 m 变为约 1.07 m，生成多约 7%。lattice 每秒只跑一次，影响可以忽略。

## 2. 设计

新增设置 `shadow.check_length_mode`（字符串），只改规划时的检查长度：

| 值 | `check` | 用途 |
| --- | --- | --- |
| `"travel"`（默认） | `speed × hold`，和现在**完全一样** | 所有非 5 号车的配置、mvp2.1 原行为 |
| `"max"` | `max(speed × hold, preview_m + EXECUTION_TAIL_M)` | 只覆盖规划那一刻的执行窗口 |
| `"travel_plus_window"` | `speed × hold + preview_m + EXECUTION_TAIL_M` | 覆盖整段使用期间每一拍的执行窗口。**5 号车用这个** |

- **`EXECUTION_TAIL_M = 0.15`**：新建模块级常量，替换 `PlannerExecution.select` 里写死的 `0.15`（`check = min(lengths[-1], max(needed, closest[1]+preview_m+0.15))`），让规划和执行用同一个数。执行检查的行为不变。
- **`preview_m`**：用规划任务里已有的 `job["preview_m"]`，和 `_plan_geometry` 里现在取 `preview` 的方式一样，包括无效时退回 `score_min_length_m`。
- **`check` 的下游**：现在 `score_length = max(score_min_length_m, check)`、`horizon = max(length_m, score_length, check + preview)` 都从 `check` 推出，**保持这个推导不变**，第 1 节的 Gym 验证就是这样做的。车速为 0 时，新模式下 `check` = 执行窗口，原来是 0。
- **诊断**：规划结果里已有 `check_length_m`、`score_length_m`、`generated_length_m`、`hold_time_s`。再加：
  - `check_length_mode`；
  - `travel_check_m`：`speed × hold`；
  - `execution_window_m`：`preview + EXECUTION_TAIL_M`。

  ROS 摘要（policy_node 约 8794 行那份 key 列表）里也加上 `check_length_mode`。
- **校验**：`ShadowSettings.__post_init__` 现在把所有非布尔字段当数字检查，要把新字段加进跳过列表，再单独检查它是不是这三个值之一。
- **不要改**：
  - 执行检查的其他逻辑（车自身、connector、sweep、边界来源）；
  - lattice 的转向参数、评分权重、融合；
  - MPC。

## 3. 配置

- `config/ai4r_policy_mpc_lattice21.yaml` 的 `shadow:` 段：加 `check_length_mode: "travel"`，注释说明三个值和第 1 节的依据（用链接，不复制表格）。
  - 先确认仓库里有没有测试要求"yaml 和 dataclass 默认值一致"，比如 `tests/test_release_contract.py`、`tests/test_shadow_lattice.py` 或 `tests/test_mpc.py` 的 `ConfigChecks`。有的话，基础配置里也要按它的要求声明。
- `config/ai4r_policy_mpc_car05.yaml`：加
  ```yaml
  shadow:
    check_length_mode: "travel_plus_window"
  ```
  带一行注释，指向本文档。合并顺序上，car05 覆盖在 lattice21 之后，所以会生效。

## 4. 测试（单元测试，不跑 Gym）

放在 `tests/test_shadow_lattice.py`，沿用它现有的规划任务、道路构造方式：

1. **默认 `travel` 不变**：速度 0.20、`rate_hz` 1、`handoff_allowance_s` 0.10、`preview_m` 0.35 时，`check_length_m` 等于 0.22；同一个任务的选中路径和改动前完全相同。现有测试全部原样通过，就是这一点的主要证明。
2. **三种模式的长度**：同样输入下分别是 0.22 / 0.50 / 0.72。速度为 0 时是 0 / 0.50 / 0.50。`preview_m` 缺失或无效时，按 `score_min_length_m` 计算。
3. **规划拒绝"前方 0.3–0.6 m 才撞"的候选**：构造直道，在中心线前方约 0.45 m 处让一侧边界向内凸，使车身扫过时相撞。
   - `travel` 下，至少一个从中间穿过的候选被判为可行；
   - `travel_plus_window` 下，同一个候选被判为碰撞（`feasible_count` 变少，或选中的路径绕开）。
4. **规划和执行一致**（`tests/test_planner_execution.py`，或在 shadow 测试里直接用 `PlannerExecution`）：用 `travel_plus_window` 规划出的路径，在车沿路径前进 0、`speed×hold/2`、`speed×hold` 这几个位置调用 `select`，同一份边界下都不会返回 `known_boundary_collision`。
5. **校验**：`check_length_mode` 为其他字符串或非字符串时报 `ValueError`。
6. **执行检查不变**：`select` 改用 `EXECUTION_TAIL_M` 后，现有 `test_planner_execution.py` 全部通过。
7. **部署配置**（`tests/test_mpc_planner_control.py`）：
   - `deployed("mpc-exec-lattice21")`（5 号车）里 `shadow.check_length_mode == "travel_plus_window"`，包括 `-fb` 变体；
   - 用 `deploy["build"](..., car="27")` 生成的配置是 `"travel"`。

要跑的测试：

```text
python -m unittest tests.test_shadow_lattice tests.test_planner_execution tests.test_lattice \
  tests.test_mpc_planner_control tests.test_mpc tests.test_mpc_real_frames
python -m unittest discover -s tests
```

`test_policy_node` 需要 pytest，没有就跳过并说明。`test_lattice` 对 CPU 负载敏感，失败时先单独重跑。

## 5. 部署包和文档

1. 重新生成 7 个包（`--mode mpc-exec-lattice21 --car 05 --cones yellow-left --v-exec-max 0.2`）：

   | 包 | 参数 |
   | --- | --- |
   | `v020-1m` | `--distance 1.0 --run-time-s 60` |
   | `v020-5m` | `--distance 5.0 --run-time-s 60` |
   | `v020-nolimit` | `--distance 0 --run-time-s 0` |
   | `v020-1m-fb` | `--distance 1.0 --run-time-s 60 --longitudinal feedback` |
   | `v020-nolimit-fb` | `--distance 0 --run-time-s 0 --longitudinal feedback` |
   | `v020-nolimit-fb-nomm` | `--distance 0 --run-time-s 0 --longitudinal feedback-nomm` |
   | `v020-nolimit-fb-soft` | `--distance 0 --run-time-s 0 --longitudinal feedback-soft` |

   输出目录 `.verification/car05-lattice21-exec-<包名>`。把 yaml 和 policy_node 的 sha256（前 8 位加 …）填进 [CAR05_LATTICE21_EXEC_RUN_CN.md](CAR05_LATTICE21_EXEC_RUN_CN.md) 的 10-20 哈希表。

2. **换行符**：仓库是 LF（`.gitattributes` 为 `eol=lf`）。生成包之前，用 `git ls-files --eol scripts/policy_node.py` 确认工作副本是 `w/lf`；否则哈希和别人 checkout 出来的不一致（10-20 出过一次）。

3. 在 `CAR05_LATTICE21_EXEC_RUN_CN.md` 加一段"10-20：规划检查长度"：
   - 说明改了什么；
   - 参数核对：`ros2 param get "$N" shadow.check_length_mode`，预期 `travel_plus_window`；
   - 看包时看什么：`/car/shadow_planning` 里的 `check_length_m` 约 0.7 m；急弯里 `reference_selection` 很少再变成 `baseline`；`known_boundary_collision` 明显减少。

4. [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 最上面加一条：改了什么、测试结果、新哈希，标 🟡（未上车）。

5. [MPC_BRANCH_COMPARISON_CN.md](MPC_BRANCH_COMPARISON_CN.md) 的 2.3 节，在"lattice ↔ 中心线切换"一行补一句：本分支的规划检查已覆盖执行窗口。

## 6. 提交

- 分支 `mpc-car-run`。提交信息末尾加：`Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。
- 代码和测试一个提交；部署包哈希和文档一个提交。
- 推送到 `origin/mpc-car-run`。
- 本机由使用者运行 WSL 门禁：`BRANCH=mpc-car-run bash /mnt/e/26b/AI4R/Baby-Bus/.verification/run_ros_check.sh`。

## 7. 给 Planning 组的说明（要点）

- 这是 mvp2.1 lattice 的行为：规划只检查 `speed × hold`，执行检查到 `preview + 0.15`，并且执行窗口随车前进。
- 本分支加了可选模式，默认不变；5 号车用 `travel_plus_window`。
- Gym 里急弯切换减少 70–90%，最大横向误差下降，每次规划的耗时没有可测增加。
- 建议在 mvp2.1 里也采用。
