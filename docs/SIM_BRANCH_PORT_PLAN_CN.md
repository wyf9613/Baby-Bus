# mvp21-sim-integration 分支 → mpc-car-run：分析结论、验证清单与执行顺序

> 交接文档，2026-10-21，作者 yuzaiyang(Lucien)，在另一台电脑上执行。
> 文中的代码行号都以 `origin/mpc-car-run@bb0bef2` 的 `scripts/policy_node.py` 为准；`origin/feature/mvp21-sim-integration-20261015@2c41240` 简称"sim 分支"。
> 原则：**所有移植都默认关闭**，关闭时行为和现在逐位一致；每一步完成后在 `docs/MPC_PROGRESS_LOG.md` 顶部加一条记录；没有明确要求时不 push。

> **2026-10-21 审阅修正（执行前）**。下面原文保留，有错的地方用"〔修正〕"标出。执行结果见 `docs/MPC_PROGRESS_LOG.md` 2026-10-21 条目，分支 `mpc-car-run-sim-port`。
> 1. **P3 已经部分过时**：`score_length = max(score_min_length_m, check)`。car05 用 `travel_plus_window`，0.20 m/s 时 `check` ≈ 0.72 m，所以评分长度已经是 0.72 m，不是 0.2 m；`horizon` = max(1.0, 0.72, 0.72+0.35) = 1.07 m。③ 的 `generated` 在 car05 上只把评分（或检查）长度从 0.72 拉到 1.07。第 3 节"collision 基本重叠、score 才是真正新增"的说法不对，两者都是 0.72 → 1.07。③ 也只针对 10-16 列的第一个原因（评分短），"弯外准备约 0.14 m"不受影响。
> 2. **工具跑的不是计划以为的配置**：`replay_run_jsonl.py --planner-control` 原来先应用 `--set` 再合并 lattice21.yaml（`travel`），所以回放永远是 `travel`，也改不了；Gym（`car05_variants.py`）只取 car05 的 mpc/control/vehicle，`shadow` 来自 lattice21.yaml（`travel`），第 2 步的 A 组其实和 C 组相同。另外两个工具里 lattice 结果的可见时间都取本机实测耗时，同一份代码跑两遍，回放 398 拍里有 355 拍不同。已修：`--set` 在 overlay 之后再应用一次；两个工具都加了 `--lattice-planning-s`（固定耗时，结果确定）。
> 3. **① 的 import**：worker 用固定命名空间执行 AST 抽取的定义，`bisect` 写在模块顶层会 `WORKER_ERROR`，必须写在函数体内（sim 分支就是这样）。
> 4. **`length_m=1.3` 的理由不成立**：`horizon` 会自动延长到 `check+preview`，0.5 m/s 时已经是 1.40 m。D 组在 0.20 m/s 下改的是所有候选的形状（knot 位置是 horizon 的比例），不是"为提速做准备"。
> 5. **② 的两个担心**：car05 实际是 `road_geometry: ordered_xy`（和 sim 分支相同）、`control.startup_grace_s: 3.0`，都没问题。
> 6. **执行顺序改为**：0 修工具 → 1 基线（先回答 `travel_plus_window` 本身对 P3 有多大作用）→ 2 ① → 3 准备上车材料 → 4 ③ → 5 ②。

---

## 0. 结论速览

| 编号 | sim 分支里的内容 | 判断 | 对应 MPC 日志里的问题 |
|---|---|---|---|
| ① | lattice 二分弧长查找 + 每次规划内缓存采样点坐标 | **采用**，低风险 | P4（CPU），给 P2 留出计算余量 |
| ③ | `shadow.score_coverage` / `collision_coverage: generated` | **先评估**（开环回放 + Gym） | P3（评分只看前 0.2 m 导致转向反号）、P2 |
| ② | 锥桶颜色自动识别 `estimation.colour_assignment: auto` | 可选，看场地是否经常换颜色方向 | P5 |
| ④ | 单侧观测边界交给 lattice（`_align_observed_boundary` + `forward_geometry`） | **搁置**，先做决定 | P1 的一部分 |
| — | RNN 学习选路、0.5 m/s 配置、陀螺仪兜底、模块拆分、单文件打包 | 不采用（理由见第 3 节） | — |

执行顺序：**第 0 步环境 → 第 1 步 ① → 第 2 步 ③ → 第 3 步 ②（可选）→ 第 4 步 ④（只讨论）→ 第 5 步 0.5 m/s 扫描（可选）**

---

## 1. sim 分支是什么

- 由其他小组（Steve10086、astune）维护，基于 mvp2.1。他们把 `policy_node.py` 拆成了 `mvp21_policy/` 模块包，再用 `tools/bundle_python.py` 打包回单文件；下游控制器是 **MVP PID**。
- 和 mpc-car-run 的共同祖先是 `6245537`。mpc-car-run 比它多 111 个提交，它比 mpc-car-run 多 14 个。`policy_node.py` 差了上万行，**不能用 git merge，只能按功能手动移植**。
- 最新提交 `2c41240`（10-21）：接入冻结的多起点 RNN 选路、颜色自动识别、目标 0.5 m/s（前馈和上限都是 0.50）、traxxas 转向 ±1.0；已部署到 .19 车，验收只有一句操作员反馈"车辆能按预计行驶"。
- 两边的估计层几乎一样：`EstimationSettings` 只差 `colour_assignment` 一个字段，`EstimationPipeline` 只差 43 行（就是 ④ 那项改动）。lattice、shadow worker、`PlannerExecutionManager` 也是同一套代码，都来自 `mvp2.1@3652e9b`；我们在 10-16 移植过，改动处标了 `mpc-car-run adaptation`。

## 2. MPC 日志里还没解决的问题（来自 `docs/MPC_PROGRESS_LOG.md`）

| 编号 | 问题 | 出处 |
|---|---|---|
| P1 | 估计层停车（`invalid_boundary_pairing`、`invalid_ordered_geometry`、`insufficient_common_range`，以及别组锥桶误导），是失败的主因 | 10-18 起几乎每条 |
| P2 | lattice 规划时检查 0.22 m，执行时检查约 0.5 m；已经做了 `check_length_mode`，还没上车，Jetson 上的耗时也没测 | 10-20 再续四/五 |
| P3 | lattice 评分只看前 0.2 m（`score_min_length_m`），MPC 转向和 PID 反号（例如 end-straight1 上 203 对 46）；日志写的是"需和 Planning 组商量"。〔修正〕这是 `travel` 下的情况；car05 的 `travel_plus_window` 下评分已是约 0.72 m | 10-16 |
| P4 | 车上 CPU 满载，节拍被拖到 0.1–0.15 s 后报 `motion_state_unavailable` | 10-19 现场 |
| P5 | 每个场地都要手动确认锥桶颜色（`--cones`、`cone_layout_check`） | 10-14 |
| P6 | 本分支 `traxxas_vehicle_interface.yaml` 仍是 ±0.5，没定要不要统一 | 10-19 现场 |
| P7 | IMU 故障 | 10-08；sim 分支 10-21 在 .22 上也遇到 |

## 3. 逐项分析

### ① lattice 性能优化（采用）
- **改什么**（参照 sim 分支 `92ef303` 的 diff，只有几行；源头是 Zhenyu 的 `feature/mpc-model@15724bc`，不过那个提交很大，从 92ef303 取更清楚）：
  - `_shadow_at`（第 6188 行起，第 6197 行现在是 `next(i for i in range(...) if s <= lengths[i+1])` 线性查找）改成 `bisect_left(lengths, s) - 1`；
  - `ShadowLatticePlanner._plan_geometry` 里 `p, _, n = frame.at(s0+distance)`（第 6771 行）改成每次规划内用 `knot_frames` 字典缓存同一个 `distance` 的结果。
- **已确认** mpc-car-run 还没有这两项。
- 局限：lattice 在独立的 worker 子进程里运行，这项优化降低的是整机 CPU 争用，**不会直接缩短 policy 节拍**；在 Jetson 上没有测过。

### ③ 评分和检查范围（先评估）
- sim 分支的 `ShadowSettings` 比我们多两个选项：`collision_coverage`、`score_coverage`，取值是 `prefix`（默认，和上游一致）或 `generated`。实现在 sim 分支 `mvp21_policy/lattice.py` 第 560–565 行：`generated` 时把检查或评分长度设成 `horizon = max(length_m, score_length, check+preview)`。
- 我们的 `check_length_mode: travel_plus_window` 在 0.2 m/s 下检查约 0.72 m，已经接近 1.0 m 的整条路径，所以 `collision_coverage=generated` 和它基本重叠。**真正新增的变量是 `score_coverage`**，它对应的就是 P3。〔修正〕评分长度也由 `check` 推出，同样已是 0.72 m；路径长 1.07 m。两个选项都是 0.72 → 1.07。
- sim 分支自己的仿真：1.3 m 路径 + 全长检查/评分，退回中心线的周期从 6 降到 2（`offline/mvp21_sim/SHARED_EXECUTION_CN.md`）。样本很小，下游是 PID。
- 另外还有一个字段 `unknown_width_reference_scale`（4/3），只在单侧边界、路宽未知时起作用，和 ④ 一起考虑，这一步不移植。

### ② 锥桶颜色自动识别（可选）
- 原理：两种颜色假设（黄左蓝右、蓝左黄右）各自用"不靠路宽推断、真实看到两侧"的几何去试。颜色反了会算出负路宽，过不了宽度门限，所以通常只有一种假设能通过。如果两种都通过，选中心线离车更近的那个，而且要近出一个采样间隔以上。选定后在这次 Run 内锁定，Stop/Run 后重新识别。
- 实现：sim 分支 `mvp21_policy/estimation.py` 的 `road()` 包装加 `_road()`，以及 `EstimationSettings.colour_assignment`；测试在 `tests/test_colour_assignment.py`。
- **移植要点**：mpc-car-run 有两处起步等待判断只认 `missing_motion_history`，分别是 PID 路径第 8206 行和 MPC 路径第 8294 行。两处都要加上 `boundary_colour_assignment_pending` 和 `boundary_colour_assignment_ambiguous`，否则颜色还没识别出来就会停车。
- 局限：解决不了 ea03 那种"别组锥桶被当成边界"的问题；起步时附近如果有别组锥桶，还可能锁错。

### ④ 单侧观测边界交给 lattice（搁置）
- 只看到一侧、路宽未知时，把对齐后的那一侧边界放进 `road["forward_geometry"]`，交给 lattice 规划；中心线基线仍然判无效。
- 我们设了 `planner_control.require_valid_baseline: true`，所以**改了也完全不起作用**。要起作用，还得允许中心线无效时由 lattice 驱动、估计层无效时不立即停车。这已经超出移植范围，而且是估计组的代码。

### 不采用的部分
| 内容 | 原因 |
|---|---|
| RNN 学习选路（`learned.py` + `models/path_policy.json`） | 见下面专门一节 |
| 0.5 m/s、前馈/上限 0.50 | 这是 PID 纵向的参数；他们实际上是固定 0.50 油门、没有实测车速，而且 .19 车 0.40 油门都起不了步，和 5 号车不能比（5 号车 0.35 油门稳态就有约 0.5 m/s） |
| 陀螺仪兜底 | 要求 `control.mode=mvp` 加 PID；MPC 的预测和运动对齐都离不开偏航角速度 |
| 模块拆分、`bundle_python.py` | 我们本来就是单文件，部署走 `make_deploy_yaml` |
| `execution.py` | 10-16 已经移植并做了 MPC 适配 |
| `offline/mvp21_sim`、`lattice_training`、`planner_sim` | 我们有自己的 Gym，这些只作参考 |

### RNN 能不能借鉴到 MPC
- **接入位置是通的**：RNN 只在已经通过碰撞检查的 lattice 候选里排序，挂在 worker 里 `ShadowLatticePlanner.plan()` 之后。我们的 worker 和执行管理器是同一套代码，1 Hz、48 条候选也一样。它的训练标签 C* 是在 Gym 车辆模型上用约束优化求出的全路径跟踪代价，思路上更接近 MPC；方向上正好对着 P3。
- **现在这个模型不能用**：
  1. 代码强制要求巡航速度等于 0.5，否则节点起不来；我们是 0.20，硬停线 0.40。
  2. C* 用的是 Dream Gym 课程车，不是 5 号车。
  3. 训练时每次 8 条候选，车上是 48 条。
  4. 证据弱：评分器选中 C* 最优的次数从 18/71 提高到 35/71，但闭环通过数和 native 一样都是 3/6，U 弯失败；清零记忆不改变选路，说明 RNN 记忆没有贡献；多起点重训后最优命中只从 66/108 到 67/108。
- **结论**：先用 ③ 的手写全路径评分解决 P3。如果不够，再请对方组按我们的条件重训（0.20 m/s、5 号车参数、48 条候选，最好用我们的 MPC 在 `offline/mpc_gym` 里闭环生成标签），先做 shadow，再做 exec A/B。

### MPC 能不能跑 0.5 m/s
- MPC 算法本身没有卡在 0.2，限制来自 `config/ai4r_policy_mpc_car05.yaml` 里的保护参数：`v_exec_max_mps` 0.20、软超速线 0.25、`overspeed_hard_mps` 0.40（持续 0.2 s 就锁停）、`high_speed_drive_max` 0.35 和 `drive_max` 0.38。
- 需要验证的问题：
  1. 联合 MPC（joint）超调明显：目标 0.25 时最高 0.46，目标 0.30 时最高 0.45，取消限制后到 0.60。应该用 `--feedback`（PI-MPC）。
  2. lattice 路径 1.0 m 不够：0.5 m/s 下一个规划周期走约 0.55 m，`travel_plus_window` 要检查约 1.05 m，需要把 `shadow.length_m` 调到约 1.3。〔修正〕不需要：`horizon` 自动取 `check+preview` ≈ 1.40 m。要看的是估计层前方能不能给出 1.4 m 的路，以及 lattice 耗时（检查越长越慢，见进度日志 10-21）。
  3. 出弯冲速（a5da）。
  4. CPU 节拍。
- 现有证据：Gym 只扫到 0.30（.27 车模型、joint 模式）；实车最高只跑过目标 0.20。

---

## 4. 数据和环境盘点（本机只读检查的结果）

- **本机没有原始 ROS 车包**（没有 `.mcap` 或 `.db3`）。10-19 的 5 号车车包都在仓库外的 `car-evidence/` 目录。
- **能完整回放的只有** `docs/experiments/2026-10-14/evidence/car15/{end-straight1,end-straight2-1m,visualization4}/run.jsonl`：3 段，每段约 20 s，含原始锥桶、轮速和陀螺仪；车是 15 号车，下游是 PID；以直道和缓弯为主，只有一种颜色布置。
- `tests/fixtures/car05_20261019_runs/*.json.gz`（09bc、a5da、ea03 共 4 个）**只有每拍的 MPC 诊断摘要，没有锥桶原始数据，不能回放**，只能给 `tools/car05_run_stats.py` 算指标。
- 本机 WSL 只有 `docker-desktop`，没有 Ubuntu，不能跑 ROS 门禁。

**另一台电脑需要准备：**
1. 仓库切到 `mpc-car-run`，确认 `git log -1` 是 `bb0bef2` 或更新的版本；先 `git fetch` 拿到 sim 分支 `origin/feature/mvp21-sim-integration-20261015`。
2. Dream Gym 环境（Gym 脚本依赖 `dreamgym`），Python 3.12 和 NumPy。
3. WSL Ubuntu 和 ROS 门禁脚本（按以往的方式：`BRANCH=mpc-car-run bash .verification/run_ros_check.sh`）。
4. **如果要做 ②，或者想用 5 号车数据验证 ③**：从 `car-evidence/` 拿 10-19 的 mcap，至少 ea03-run1 和 a5da-run1，放在仓库外单独的目录里。

---

## 5. 执行顺序与验证清单

### 第 0 步：建立基线（所有改动之前）
- 新建分支，例如 `mpc-car-run-sim-port`，不要直接在 `mpc-car-run` 上改。
- 跑一次全部单元测试，记录通过数：`python -m unittest discover -s tests`。日志里的基线是 414 个，唯一的错误是 `test_policy_node` 需要 `rclpy`。
- 跑一次 Gym 基线，后面所有对比都和它比：
  `python offline/mpc_gym/car05_variants.py --plant-turn-drag 0.06 --seeds 6 --workers 8`
  共 60 次（5 种路 × 2 种车道宽 × 6 个种子）。**跑两遍**，记录完成数、lattice 占比、`reference_switches`、`selection_reasons`、最大横向误差和最高车速。两遍之间差 1–3 次属于正常波动。
- 跑 3 段 run.jsonl 开环回放的基线（每段都跑）：
  `python tools/replay_run_jsonl.py docs/experiments/2026-10-14/evidence/car15/end-straight1/run.jsonl --planner-control --json-out <out>.json`

### 第 1 步：① 性能优化
- **改**：上面 ① 里说的两处。
- **同步**：worker 子进程入口、`offline/shadow_lattice` 里按 AST 抽取定义的地方，以及测试替身，凡是抽取了 `_shadow_at` 或 `_plan_geometry` 的都要检查。10-20 加 `EXECUTION_TAIL_M` 时漏了这一步，报过 `WORKER_ERROR`。
- **验证**：
  1. 新增等价测试：仿照 10-20 的做法，用改动前后的代码对比 28 个规划任务（`tests/fixtures/shadow_temporal_recorded_run.json` 的 20 帧 + 合成路）。`travel`、`max`、`travel_plus_window` 三种模式下，候选、评分、选中的候选都要**逐位相同**（耗时字段除外）。另外测弧长查找在节点上、相邻浮点值上和外推段上的结果都和原实现一致。
  2. 现有测试全部原样通过：`test_shadow_lattice`、`test_lattice`、`test_planner_execution`、`test_mpc_planner_control`。
  3. 计时：本机用同一批任务对比规划耗时的均值和最大值；上车时测 Jetson 上的 lattice 耗时，同时回答 P2 留下的"travel_plus_window 在 Jetson 上耗时多少"。
- **不要用 Gym 判断这一步。** Gym 里 lattice 结果的可见时间取决于本机实测耗时，结果会有波动，但这不代表行为变了。
- **通过标准**：逐位等价，规划耗时下降。

### 第 2 步：③ 评分范围
- **改**：在我们的 `ShadowSettings` 里加入 `score_coverage` 和 `collision_coverage`，默认 `prefix`，校验方式照抄 sim 分支；在 `_plan_geometry` 里按 sim 分支 560–565 行的逻辑实现，要和我们已有的 `check_length_mode` 共存。规划结果和 ROS 摘要里加上这两个字段。部署配置先不改。
- **验证（先单元测试）**：
  1. 默认 `prefix` 下，第 1 步那 28 个任务的结果逐位不变。
  2. `generated` 下评分长度等于整条生成路径；对一个"前段好、后段偏出"的候选，`prefix` 会选它，`generated` 不会。
  3. 取值校验；worker 往返测试。
- **验证（开环回放，回答 P3）**：3 段 run.jsonl 各跑两遍，第二遍加 `--set shadow.score_coverage=generated`。用 10-16 的统计口径：两边转向绝对值都大于 0.05 时，统计 MPC 转向和车上 PID 反号的拍数，以及和中心线 MPC 反号的拍数。10-16 的基线是 end-straight1 上 203 / 145。工具只打印 RMSE，反号数要从 `--json-out` 的逐拍数据自己统计。同时看退回中心线的次数、切换次数和规划耗时。
- **验证（Gym 闭环对比）**：同一个仿真车（`--plant-turn-drag 0.06`），每组 60 次，**每组跑两遍**：

  | 组 | 参数 | 作用 |
  |---|---|---|
  | A | car05 配置原样（`check_length_mode: travel_plus_window`，评分 prefix） | 基线 |
  | B | A 加 `--set shadow.score_coverage=generated` | 看 P3 |
  | C | `--set shadow.check_length_mode=travel` | 原始版本，作参照 |
  | D（可选） | B 加 `--set shadow.length_m=1.3` | 为提速做准备 |

  〔修正〕A 组必须显式加 `--set shadow.check_length_mode=travel_plus_window`，否则 Gym 用的是 lattice21.yaml 的 `travel`，和 C 组相同。D 组改的是所有候选的形状，不是提速准备。各组都加 `--lattice-planning-s 0.10`，结果确定，每组跑一遍即可。

  **指标不能只看完成率**（失败大多停在估计层，会掩盖规划层的差别）。要看：lattice 占比、`reference_switches`、`selection_reasons`、急弯（U 弯、直角弯 R1、R1.5）的最大横向误差、规划耗时、`ALL_COLLIDE` 次数。
- **通过标准**：B 相对 A，开环反号数明显下降；Gym 里急弯切换次数和横向误差不变差，完成数在正常波动范围内；规划耗时可以接受。满足这些后，才考虑把 `score_coverage: generated` 写进 car05 配置并生成新的部署包，然后上车做 A/B。
- 结果整理后拿去和 Planning 组讨论，这本来就是 P3 待商量的事项。

### 第 3 步：② 颜色自动识别（可选，只在场地经常换颜色方向时做）
- **改**：`EstimationSettings.colour_assignment`（默认 `fixed`）；`EstimationPipeline.road()` 包装和 `_road()` 照抄 sim 分支；第 8206 行和第 8294 行两处起步等待判断都加上两种颜色等待状态；节点只在锁定时打一次日志；`mpc_debug` 加上 `colour_assignment`。
- **验证**：
  1. 移植 sim 分支的 `tests/test_colour_assignment.py`；另外补一个 MPC 路径的起步等待测试：颜色识别期间不报 stop，超过 `startup_grace_s` 才停。
  2. Gym 回归：auto 和 fixed 的结果应该一致（锁定后完全相同，只是起步多几拍中性等待）。再把 `offline/mpc_gym/pipeline_sim.py:48` 的 `GYM_TO_ROS_COLOUR` 临时反过来跑一组，确认另一种布置也能识别。这只是本地实验，改动不要提交。
  3. **真实车包回放（必须有 mcap）**：`python tools/replay_policy_bag.py <ea03 车包目录>`，看起步锁定的映射对不对、有没有被右侧别组黄锥误导。没有 mcap 就不能判断能不能上车。
- **通过标准**：Gym 结果和 fixed 一致，两种布置都能锁定，ea03 和 a5da 回放都锁定为正确的映射。

### 第 4 步：④ 只讨论，不改代码
需要和估计组、Planning 组定两件事：中心线无效时是否允许 lattice 驱动，以及估计层无效时是否不立即停车。定下来之后再按 ③ 的方式评估。

### 第 5 步（可选）：0.5 m/s 的 Gym 扫描
`python offline/mpc_gym/car05_variants.py --plant-turn-drag 0.06 --feedback --speed 0.30`，然后 0.40、0.50 各跑一遍。同时用 `--set` 抬高超速线（硬停线始终高于目标速度并留余量），`shadow.length_m` 设为 1.3。看最高车速、急弯横向误差、退回中心线的比例和超速锁停次数。仿真通过后，实车再按 0.30 → 0.40 → 0.50 逐级上，不要直接取消硬停。

---

## 6. 通用注意事项
- 每项都要做到"默认关闭时逐位不变"，用 28 个规划任务和 Gym 两遍基线来证明。
- 改动处统一标注 `mpc-car-run adaptation (from sim-branch <commit>)`，方便以后对比。
- 上车前必须过 WSL ROS 门禁，并重新生成部署包、把哈希记进现场清单。
- 每一步完成后在 `docs/MPC_PROGRESS_LOG.md` 顶部加一条记录，写清楚改了什么、测了什么、结果、哈希、还没验证什么。

## 7. 要和其他组沟通的事项
- **Planning 组**：P3 的评分范围（第 2 步的结果），以及 P2 的检查长度。
- **sim 分支组（Steve10086、astune）**：如果 ③ 不够，请他们按 0.20 m/s、5 号车参数、48 条候选重训评分器；另外问清楚他们 0.5 m/s 的实际车速有没有实测。
- **估计组**：④，以及 P1"只接受和上一帧边界连续的锥桶"（ea03 的问题）。
- **助教**：车上 CPU 负载（P4）。
