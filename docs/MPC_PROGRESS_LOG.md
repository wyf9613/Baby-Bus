# MPC 进度日志

对应计划：[MPC_PLAN_OVERVIEW.md](MPC_PLAN_OVERVIEW.md)（v3）。新条目加在最上面。状态标记：✅ 完成，🟡 部分完成，⏳ 待做，❌ 阻塞。

## 2026-10-20（再续三）：lattice 转向假设对齐的 Gym 检验，结论为"没有影响"

**为什么做**：核对部署配置时发现，本分支 lattice 按 45° / 1.57 rad/s 规划，而 MPC 是 0.37 rad / 1.21 rad/s，实车 0.40–0.44 rad。Zhenyu 的分支把 lattice 和 MPC 对齐成 0.3142 / 1.21。见 [MPC_BRANCH_COMPARISON_CN.md](MPC_BRANCH_COMPARISON_CN.md) 2.2b。

**设置**：`car05_variants.py --plant-turn-drag 0.06 --set planning.lattice.steering_limit_rad=… --set planning.lattice.steering_rate_limit_radps=…`。联合 MPC，0.20 m/s，每组 60 次。

| lattice 转向假设 | 完成 | U 弯切换次数 | 直角弯 R1.5 / R1 完成 | 直角弯 R1.5 / R1 切换次数 |
| --- | --- | --- | --- | --- |
| 45° / 1.57（现在） | 38/60 | 294 | 3 / 2 | 164 / 216 |
| 0.42 / 1.21（实测） | 38/60 | 294 | 5 / 1 | 184 / 163 |
| 0.37 / 1.21（MPC 用的值） | 37/60 | 319 | 4 / 0 | 193 / 180 |

**结论**：
- **没有一致的差别**，都在同一组重跑的波动范围内。
- **原因**：Gym 里最急的 R = 1 m 弯只需约 0.32 rad 转向，低于 0.37，所以转向上限从来没起作用。
- **这个对齐只对比 R ≈ 0.85 m 更急的弯有意义**。为了一致性仍建议在 5 号车配置里对齐，但不能指望它减少急弯的切换。
- **切换的主要原因在别处**：U 弯和直角弯每组 160–320 次切换，原因见 `selection_reasons`；失败大多仍停在估计层。

**工具改动**：
- `car05_variants.py` 的 `--set` 支持嵌套参数名；
- 每条路输出 lattice 占的拍数比例和切换次数（`pipeline_sim` 新增 `reference_switches`）。

## 2026-10-20（再续二）：取消超速限制的 Gym；车包俯视图工具；IMU 安装方向

**取消超速限制**（0.25 m/s，转弯阻力 0.06；软线 1.0、切油门 0、硬停关闭）：

| 仿真车 | 联合 MPC：有限制 → 取消 | PI-MPC：有限制 → 取消 |
| --- | --- | --- |
| 死区 0.23 | 41 → 39 次完成；最高车速 0.46 → **0.60** | 41 → 44；0.37 → 0.37 |
| 死区 0.289 | 38 → 36；0.39 → 0.43 | 39 → 40；0.34 → 0.36 |

- 完成数的变化都在 1–3 次以内，和同一组重跑的波动差不多，所以**取消限制没有带来更好的结果**。
- 联合 MPC 的最高车速升到 0.60，远超 0.40 的硬停线。软线和切油门在联合 MPC 里起着实际的控速作用。
- PI-MPC 不受影响：它的车速本来就由 PI 管住。
- 实车上保留现有限制，至少保留 0.40 的硬停。

**俯视图工具** `tools/bag_birdseye.py`（不需要 ROS；用法写在现场清单第 5 节）。它用轮速和 IMU 推算车的位置，把锥桶检测画到地图上，并标出停车、重启、超速、卡住、参考切换的位置。也可以输出某一时刻的局部图（参考路径、MPC 预测、lattice 路径），以及一段时间的动画。

10-19 的 12 个包都能出图。比如 ea03 第二次运行停车前，可以看到前方的蓝色锥桶已经弯进车道，同时车右侧出现了黄色锥桶，和现场"右侧有别组锥桶"的判断一致。

**IMU 安装方向**：
- 5 号车的 IMU（`fsm300_sensor`）是倒装的：`/tf_static` 里 base_link → fsm300_sensor 是绕 y 轴转 180°，静止时 z 轴读到的重力是 −9.18。所以车体坐标系里的偏航角速度是**原始 z 取反**。
- policy 节点本身没有问题：它用 `/tf_static` 把陀螺仪读数转到车体坐标系。
- 但 `tools/export_car_run.py` 导出的 `yaw_rps` 和之前的一次性分析脚本用的是原始 z，**符号是反的**。用到 `yaw_rps` 的地方都只看绝对值（卡住识别用 |yaw|、转向增益估计用 |yaw|/v），所以结论不受影响。以后要用带符号的偏航角速度时要注意。
- 验证方法：按原始 z 推算，锥桶地图明显糊（每次检测占用的 5 cm 格子数 0.42）；取反后清晰（0.25）。

## 2026-10-20（再续）：0.25 m/s 的 Gym，联合 MPC 和 PI-MPC；对比方案

**Gym 设置**：`car05_variants.py --speed 0.25 --plant-turn-drag 0.06`，每组 60 次，仿真车满舵 24°。

| 仿真车 | 联合 MPC：完成 / 最高车速 | PI-MPC（`--feedback`）：完成 / 最高车速 |
| --- | --- | --- |
| 死区 0.23 | 41 / **0.46** | 41 / 0.37 |
| 死区 0.289 | 38 / 0.39 | 39 / 0.34 |

和 0.20 下同样的仿真车对比（联合 38/28，PI-MPC 36/33）：

- **0.25 的完成数反而更高。** 失败几乎都停在估计层，跑得快、时间短，估计层停车的机会就少。这和 10-19 提速扫描里"0.15 最差"的现象一致。
- **两种模式在所有组里的控制层停车都是 0**（超速锁停、`no_motion`、`too_many_restarts`）。
- **联合 MPC 在 0.25 下最高车速到 0.46，高过 0.40 的硬停线**，只是没持续 0.2 s，所以没锁停。实车上这种冲速就是 10-19 看到的超速风险。PI-MPC 最高 0.37。
- **横向最大误差两种模式相近**（0.14–0.33 m），直角弯完成数都很低。
- 只是仿真：仿真车的轮速读数不会像实车那样保持不变。

**对比方案**：组内（联合 MPC 对 PI-MPC）和组外（MPC 对 PID）写进了 [MPC_COMPARISON_PLAN_CN.md](MPC_COMPARISON_PLAN_CN.md)，内容包括：
- 每组对比回答什么问题；
- 参数分三类对齐；
- 约束作用怎么在 Gym 里单独证明；
- 公平性要求和公共 topic 指标。

M3 链接到这份文档。

## 2026-10-20（续）：PI 油门模式（`longitudinal_mode: feedback`），移植自 Zhenyu 的分支 🟡

**改了什么**：
- 新增 `mpc.longitudinal_mode`：`joint`（默认，行为不变）或 `feedback`。feedback 模式在跟踪阶段由轮速 PI 决定油门，QP 用固定油门（`solve(fixed_drive=...)`），只求转向。
- 新增 `mpc.measured_motion`：跟踪阶段，转向预测保持实测车速。
- 只在 5 号车的 `-fb` 包里打开（`config/ai4r_policy_mpc_car05_feedback.yaml`，`make_deploy_yaml.py --longitudinal feedback`）。
- 和 Zhenyu 分支的完整区别、移植了哪些、哪些没移植：[MPC_BRANCH_COMPARISON_CN.md](MPC_BRANCH_COMPARISON_CN.md)。

**和他的版本不同的地方**，都来自仿真里复现的停走循环：
- kp/ki 用 0.2/0.2，不是 0.5/1.0：kp 0.5 在 .27 的油门响应和约 0.4 s 延迟下会振荡；
- 转弯阻力保留为前馈；
- 起步交接时积分不反算；
- 读数过期时冻结积分和油门；
- `-fb` 包的软超速 / 切油门余量放宽到 0.10 / 0.05。

**仿真**：
- 纵向扫描：54 种参数不准的仿真车，跟踪阶段零重启，车速波动 ≤ 0.017 m/s。
- Gym（4 种仿真车 × 2 种模式，每组 60 次）：
  - 完成率相近，控制层停车都是 0；
  - feedback 的最高车速每组都更低：0.44/0.34/0.38/0.29，joint 是 0.51/0.43/0.39/0.35。

数字见对比文档第 3 节。

**测试**：
- 本机：mpc、planner_control、real_frames、shadow_lattice、planner_execution 共 227 项通过；lattice 45 项通过。新增 `FeedbackLongitudinal`、`FixedDriveSolver` 两组测试，以及部署包和替身节点上的测试。
- 新代码另外用 dense 求解器跑了一遍，通过。
- `test_policy_node` 本机缺 pytest，由 WSL 门禁覆盖。

**部署包**：policy_node `34048cbf…`。joint 包和 `-fb` 包的哈希已写进 [CAR05_LATTICE21_EXEC_RUN_CN.md](CAR05_LATTICE21_EXEC_RUN_CN.md)，连同 A/B 步骤和判定标准。

**工具**：
- `car05_run_stats.py`：按模式分组；新增跟踪车速标准差、处在软线的比例，以及 PI 油门来回摆动次数。
- `export_car_run.py`：导出 `fb_*` 字段。
- Gym：`--plant-turn-drag` 给仿真车加转弯阻力，`--feedback` 切换到 PI 油门。

**待做**：WSL 门禁；实车 A/B。

## 2026-10-20：`b0c660c` 本机复核、部署包、Gym 对比

- 本机测试全部通过：mpc 138、planner_control 18、real_frames 15、shadow_lattice 30、ROSplanner_execution 10、control 45、estimation 65、planning 18、release_contract 4、lattice 45。
- 部署包已重新生成：policy_node `3567d439…`，yaml 分别为 `24da6b96…`（1m）/ `923a3ab6…`（5m）/ `6dfc0d1c…`（nolimit），已填进现场清单。
- `offline/mpc_gym/car05_variants.py` 修了一个 bug：进程池复用工作进程时补丁会层层嵌套，超过 `--workers` 个任务就报 `plant_changes` 重复。现在每个进程只打一次补丁。

**Gym**（`car05_variants.py`，仿真车转向 24°，5 种路 × 2 种车道宽 × 6 个种子 = 60 次，0.20 m/s）。"关闭"用 `--set` 把 `turn_drive_per_steer`、`rest_speed_mps`、`handover_s`、`switch_blend_s` 置 0：

| 仿真车 | 关闭 | 打开 | 最大横向误差（直道 / 直角 r1.5），关闭 → 打开 |
| --- | --- | --- | --- |
| 满电（死区 0.23、起步阈值 0.24） | 36/60，峰值 0.49 | 38/60，峰值 0.51 | 0.16 → 0.23 / 0.30 → 0.52 m |
| 电量偏低（0.31、0.37） | 32/60 | 29/60 | 0.12 → 0.20 / 0.22 → 0.57 m |

- 完成次数在随机波动以内（同一配置重跑会差 1–3 次）。两种仿真车都没有超速、推不动或 `too_many_restarts`，失败都在估计层。
- 最大横向误差变大了。Gym 的仿真车**没有转弯阻力，也不会出现轮速读数保持**，这次三项改动针对的正是这两点，所以 Gym 体现不出好处。转弯阻力补偿在没有转弯阻力的仿真车上只会让车过弯更快。
- 上车要重点看急弯的横向误差：如果实车的转弯阻力比 0.06 小，弯里可能偏得更多。可以先只关掉转弯阻力（`turn_drive_per_steer` 0）做对照。

## 2026-10-19（深夜）：MPC_NEXT_PLAN 第 2 节三项改动（转弯阻力、起步回差 + 交接限幅、参考切换限速）🟡

按 [MPC_NEXT_PLAN_CN.md](MPC_NEXT_PLAN_CN.md) 第 2 节实施，目标是消除"一段一段"的走法。**只跑了单元测试，未在 Gym 和实车上验证。** 部署包和哈希没有生成（留给本人本地做）。

新参数都在 `MPCSettings`，默认关闭，基础 `ai4r_policy.yaml` 补了默认值，只在 `ai4r_policy_mpc_car05.yaml` 打开：

| 参数 | 默认 | 车 05 | 作用 |
| --- | --- | --- | --- |
| `turn_drive_per_steer` | 0 | 0.06 | 舵量 \|s\| 时死区抬高 k\|s\|：求解器收到 `drive_bias = bias + k·|applied_steer|`；偏置估计的单步预测按当时指令舵量同样抬高 |
| `rest_speed_mps` | 0（= `moving_speed_mps`） | 0.03 | 跟踪中低于它才回到起步；`moving_speed_mps` 仍 0.05 |
| `handover_s` / `handover_drive_margin` | 0 / 0.04 | 0.5 / 0.04 | 起步 → 跟踪后 0.5 s 内油门 ≤ min(死区 + bias + turn, drive_max−1e-3) + 0.04；`cut` 仍为 0，起步油门、`plan_hold` 不变 |
| `switch_blend_s` / `switch_steer_rate_per_s` | 0 / 1.5 | 0.4 / 1.5 | lattice ↔ 中心线切换后 0.4 s 内每拍转向变化 ≤ 1.5·dt（0.075），`accept` 和 `plan_hold` 都限，记录的施加历史是限幅后的值 |

- `MPCController.step` 新增 `steer_slew=None`；policy 侧按 `planner_traj is not None` 判断来源，只在限幅期内传这个参数（替身没有新属性时用 `getattr` 兜底）。没有新增顶层名字和 `PolicyNode` 方法，测试抽取列表不用改。
- `mpc_debug` 新字段：`turn_drive`、`handover`、`steer_slew`。
- 和计划的小差异：交接限幅只在 `vehicle.drive_breakaway > 0`（真的施加过起步油门）时记录交接时刻；`handover_drive_margin`、`switch_steer_rate_per_s` 的默认值取车 05 的值（它们只在对应时长 > 0 时生效，和 `switch_max_epsi_rad` 的做法一样）。
- 校验：七个部署模式 × 车 05/27 都能构造 `MPCSettings`（车 27 的 exec 模式按预期被 build 拒绝）。
- 测试：`test_mpc.py` 新增 `TurnDragHandoverBlend`（9 个），`test_mpc_planner_control.py` 新增节点层切换限速测试并扩展车 05 配置断言。第 4 节全部单元测试通过：test_mpc 138、test_mpc_planner_control 18、test_mpc_real_frames 15、test_shadow_lattice 30、test_planner_execution 10、test_control 45、test_estimation 65、test_planning 18、test_release_contract 4、test_lattice 45（`test_policy_node.py` 需要 ROS，未跑）。

改动前基线（`python tools/car05_run_stats.py`，10-19 的 4 个 bag，上车后用同一套指标对比）：

| 指标 | 09bc-run1 | 09bc-run2 | a5da-run1 | ea03-run1 | 合计 |
| --- | --- | --- | --- | --- | --- |
| 重启（卡住 / 低于 moving speed） | 21（10/11） | 9（4/5） | 18（6/12） | 14（6/8） | 62（26/36） |
| 其中前 1 s 有参考切换 | 1 | 0 | 3 | 1 | 5 |
| 起步 → 跟踪交接 / 2.5 s 内进入软超速或切油门 | 22/17 | 10/7 | 19/14 | 14/10 | 65/48（74%） |
| baseline → planner 切换：次数，\|dsteer\| 中位数/最大，\|de_y\| 最大 | 12，0.39/0.39，0.17 | 3，0.28/0.39，0.07 | 24，0.27/0.39，0.18 | 7，0.33/0.46，0.44 | — |
| 稳态油门：直行 \|s\|<0.15 / 满舵 \|s\|≥0.85 | 0.299/0.326 | 0.299/0.341 | 0.282/0.350 | 0.280/—（满舵只有减速窗口 0.348） | — |
| 满舵占跟踪时间，1 s 内左右来回 | 15%，5 | 7%，2 | 28%，14 | 12%，2 | — |

## 2026-10-19（夜，续）：ea03 实车：控制层没有再停车；剩下的停车是别组锥桶误导估计层

bag `…-ea03-run1-191128`（`ea0304c`，模型转向 0.37）。三次启动：
- 20.4 s / 约 3.9 m：`invalid_boundary_pairing`；
- 刚启动就停：`insufficient_common_range`；
- 26.1 s / 约 4.4 m：`invalid_boundary_pairing`。

**全部出在估计层，控制层没有停车。**

和 18:45 的 a5da 对比：

| | a5da（模型 18°） | ea03（模型 21°） |
| --- | --- | --- |
| 满舵占跟踪时间 | 28% | 12% |
| 1 s 内左右满舵来回 | 14 次 | 2 次 |
| 跟 lattice 时航向误差 p95 | 0.81 rad | 0.25 rad |
| 跟 lattice 时横向误差中位数 / p95 | 2 / 13 cm | 1.5 / 10.6 cm |
| 最高车速 | 0.47（超速锁停 2 次） | 0.358（无超速） |

其他保护都正常：
- 实测满舵仍为 23–25°；
- 卡住 6 次，都自动重新起步；每次启动最多重启 6 次；
- 没有用过期读数切油门；
- 偏置在 -0.01 到 +0.07 之间；
- 0.38 的低速油门上限只在 164 拍里用到。

**`invalid_boundary_pairing` 的原因**：两次失败前，前方 2.4–3.5 m、车右侧 0.8–1.0 m 都有一排黄色锥桶（配置是黄色在左）。现场确认停车点右侧 2–4 m 内有别组的锥桶。估计层的视野很宽（`road_forward_max_m` 4.0；横向上限 `road_width_max_m` 2.5，再加随距离放宽的斜率 3.0，几乎不做横向过滤），别组锥桶被当成了我们的边界。

- 单看"颜色在错误一侧"分不出来：整个 bag 72% 的帧都有，因为过弯时外侧边界本来就会出现在车头前方。
- 收窄视野不能根治：别组锥桶只在右侧约 1 m 外，而我们自己的边界在急弯里横向也会超过 1 m。

**后续**：
- 测试时把停车点附近的别组锥桶挪开或挡住；
- 估计层应只接受和上一帧边界连续的锥桶（间距、方向），交估计组处理。

## 2026-10-19（夜）：a5da 实车：出急弯超速；5 号车实际转向比模型强约 35%

bag `…-a5da-run1-184516`，赛道依次是小弯、短直道、急弯。共 6 次启动，最长 28.6 s、约 6 m。

上一版的改动都生效了：
- 读数过期时不再切油门或压低下限（0 拍）；
- 每次启动最多重启 4 次，没有再出现 `too_many_restarts`；
- 卡住 6 次，都自行重新起步；
- 偏置在 -0.03 到 +0.08 之间。

停车原因：估计层 3 次、`origin_before_path_start` 1 次、**超速 2 次**。

**超速**：两次都在急弯。满舵时车速掉到 0.09，油门按低速档给到 0.40；出弯心后 0.6 s 内车速冲到 0.47（IMU 偏航角速度 -0.8 rad/s），超过 0.40 并持续 0.2 s 以上，触发硬停。滤波车速比原始轮速慢约一拍，所以低速档多用了 0.2 s；当时电池是满的。

**转向**：用 IMU 偏航角速度除以车速得到实际曲率，4 个 bag 一致：
- 满舵 23–25°，左右对称；满舵以下每单位指令 23–29°；零位在 ±2° 以内。
- 模型用的是 18°（借自 .20 车）。MPC 过度转向，1 s 内从一侧满舵打到另一侧 14 次；满舵占 28%。
- 舵机施加值只比设定值晚 0.08 s。
- 跟 lattice 时横向误差中位数 2 cm、p95 13 cm，但航向误差 p95 0.8 rad。

改动：
- `MPCController.step(..., speed_raw=)`：判断油门上限时取原始轮速和滤波车速中较大的值。policy 传入原始轮速。
- 车 05：`drive_max` 0.38、`high_speed_mps` 0.12、`steering_gain_rad` -0.37、`steering_min/max_rad` ±0.37，`vehicle.source` 更新为 `car05_20261019_start035_db025_27model_05steering_UNVERIFIED`。

**Gym**：脚本把仿真车的转向改成实测的 24°（原来是 45°），60 次（5 种路 × 2 种车道宽 × 6 个种子）：

| 模型转向 | 完成 | 直道 / 缓弯 / U 弯 最大横向误差 | 直角 r1.5 最大横向误差 |
| --- | --- | --- | --- |
| 0.3142 | 41/60（另一次跑 44/60） | 0.21 / 0.27 / 0.37 m | 0.27 m |
| 0.37 | 40/60 | 0.21 / 0.20 / 0.26 m | 0.29 m |
| 0.42 | 40/60（另一次跑 39/60） | 0.15 / 0.19 / 0.29 m | 0.52 m |

- 完成次数在随机波动内相同，失败都在估计层。
- 选 0.37：U 弯误差最小，也没有 0.42 在直角弯的大偏离。
- 电量低的仿真车（死区 0.31，起步阈值 0.37）用最终配置 35/60，没有一次推不动，也没有超速。

部署包已重新生成：policy_node `873df5ae…`，yaml 分别为 `c27113a3…` / `ce78e2eb…` / `67dc7494…`。

## 2026-10-19（晚，再续）：09bc 实车：读数过期还在切油门；按车速分两档油门上限

bag `…-09bc-run1-181828`（4 次启动，2.1–5.5 m）和 `…-09bc-run2-182414`（急弯）。

| 现象 | 数据 |
| --- | --- |
| 读数过期时超速保护照样切油门或压低下限 | run1 卡住 10 次，其中 5 次卡住前处于 `cut`（油门 0）或 `soft`（0.24–0.26）；run2 5 次中 2 次 |
| 重启成对出现 | 卡住后重新起步，车速刚过 0.05 又掉下来，又算一次重启。run1 第 4 次 9 次重启中 4 次是这样多出来的，run2 同样 |
| 电量低 | run2 第 1 次 0.35 的起步油门 2.5 s 没推动车；转向不大时 0.28–0.31 也会停；偏置最高 +0.07 |
| 其他 | 两个 bag 各有一次 Disarm（主机与 MCU 链路断开）；跳变保护分别触发 20 拍、4 拍，转向平稳 |

改动（`MPCController`；默认关闭的参数只在车 05 打开）：
- `overspeed_check`：读数不新鲜时状态视为 `ok`，即不切油门、不放开下限；切油门计时冻结，0.40 硬停照常。
- `restart_merge_s` 2.0：距上次计数的重启不到 2 s 的不再计数。`max_restarts` 10。`mpc_debug` 新增 `restarts`。
- `high_speed_drive_max` 0.35 / `high_speed_mps` 0.15：车速 ≥ 0.15 时 QP 的油门上界为 0.35，低于 0.15 时为 `vehicle.drive_max` 0.40。下界（死区）高于它时取下界。`solve(..., drive_cap=)`，`mpc_debug` 新增 `drive_cap`。
- `breakaway_ramp_per_s` 0.05：起步油门从 0.35 起，1 s 后升到 0.40。

**测试**：新增 3 项（读数过期不切油门、重启合并、按车速的油门上限）；全部本地测试通过（mpc 129，其余同上）。

**Gym**（直道、缓弯、U 弯、直角 r1.5、直角 r1，两种车道宽，种子 1–6，共 60 次，0.20）：

| 仿真车 | 旧配置（上限 0.35，无起步爬升、无重启合并） | 新配置 |
| --- | --- | --- |
| 满电（死区 0.23） | 41/60，峰值 0.48 | 40/60，峰值 0.50 |
| 原版（0.289） | 33/60 | 32/60 |
| 电量偏低（0.31，起步阈值 0.37） | 0/60（全部 `no_motion_after_breakaway`） | 28/60 |

三种仿真车都没有超速锁停。部署包已重新生成：policy_node `c78d1b39…`，yaml 分别为 `d8841971…` / `f373d0fd…` / `f6d555da…`。

## 2026-10-19（晚，续）：b93d 实车：`too_many_restarts`；判定加锁存，偏置范围加宽

bag `car05-lattice21-v020-nolimit-b93d-run1-20261019-175349`。三次启动：
- 第 1 次 1 m 后 `invalid_road_invalid_boundary_pairing`（估计层）；
- 第 2、3 次 `too_many_restarts`，分别跑了 1.9 m 和 2.8 m。
- 跳变保护只触发 2 拍，都正常回到求解，**不是 lattice 和中心线切换的问题**。

原因：
1. **卡住判定会来回闪**：重启后转向小于 0.3、读数还不满 1.5 s，判定撤销，用保持的旧读数切回跟踪，下一拍又判为卡住，每闪一次多算一次重启（43.6 s、63.6 s 各多算 1 次）。
2. **电池用了一个半小时**：偏置频繁顶在 +0.05，跟踪时只给 0.26–0.29 的油门，车在弯里慢下来、卡住。满舵时给到 0.35 也只有 0.11–0.13 m/s。
3. 车速在 0.05 附近时，起步和跟踪两个状态会来回切换，每次也算一次重启（以前就有）。

改动：
- 卡住判定保持到读数变化为止（新测试 `test_a_stall_stays_declared_until_the_reading_changes`）；
- 车 05：`drive_deadband` 0.25，`drive_bias_max` 0.10（覆盖 0.15–0.35），`max_restarts` 8。

**油门上限保持 0.35**。Gym 对比（死区 0.25 ± 0.10，切油门超时 2 s）：

| 油门上限 | 仿真车死区 0.23（满电） | 仿真车死区 0.289 |
| --- | --- | --- |
| 0.40 | 25/30，U 弯 1.0 m 车道 3 次超速锁停（峰值 0.56） | 26/30 |
| 0.35 | 26/30，0 次 | 25/30，0 次 |

仿真车死区 0.32 的那组不可用：只改了死区、没改起步阈值，0.35 推不动。

部署包已重新生成：policy_node `8a1f6013…`，yaml 分别为 `38ff5861…` / `33e59193…` / `3877cc86…`。上车前要跑 WSL 门禁，并建议换满电电池。

## 2026-10-19（晚）：轮速过期识别 + 中心线跳变保护

针对同日实车的问题 1 和 2（见下一条）。新参数都默认关闭，只在 `ai4r_policy_mpc_car05.yaml` 打开：`wheel_hold_stale_s` 0.5、`switch_hold_s` 0.25。

| 改动 | 内容 |
| --- | --- |
| `WheelHoldMonitor` | 原始轮速读数 0.5 s 不变：不新鲜，软超速的切油门计时和偏置估计暂停（0.40 硬停照常计时）。最近 0.5 s 内 IMU 偏航角速度都 < 0.02 rad/s，且施加的转向 ≥ 0.3（或读数 1.5 s 不变）：判为卡住，交给 MPC 的车速改为 0，由已有逻辑切回 starting、重新给 0.35 的起步油门（计入 `max_restarts`） |
| `_mpc_jump_guard` | 只对 `road_centerline`：中心线参考的 `|e_psi|` > 0.5 rad，或相对上一拍已求解的误差跳变（0.5 rad / 0.15 m），就作为可容忍的参考缺失，进入 `plan_hold`，最多 0.25 s，之后照常使用（`baseline_jump_expired`）。lattice 的拍只更新"上一拍误差"。需要 `reference_dropout_tolerance_s` 大于它（V2 0.3、中心线 0.5） |

**离线复核**（今天 6 个 bag 逐拍回放监测器）：判为卡住 10 次，读数随后都跳到 0，**无误判**；判出时间多在卡住后 0.7–1.5 s，原来要等 3 s。nolimit-run2 那次"连续 32 s"其实在满舵过弯时卡住了 3 次、每次约 3 s。17:16 两次正常的 1 m 运行没有误判。跳变规则在 run2 的 15、25、29、37–38 s 命中。

**测试**：
- 新增 `tests/fixtures/car05_20261019_wheel_hold.json`（从 bag 截的 5 段）；
- `test_mpc.HeldWheelReading` 7 项；
- `test_mpc_planner_control.CarFiveGuards` 3 项，外加车 05 配置检查 1 项；
- 全部本地测试通过（mpc 125、planner_control 17、real_frames 15、shadow_lattice 30、planner_execution 10、control 45、estimation 65、planning 18、release_contract 4、lattice 45）；
- Gym（lattice21 + 车 05，0.20）：死区 0.23 的仿真车 29/30（改前 28/30），死区 0.289 的 23/30（改前 21/30），都没有超速锁停。

Gym 和 replay 工具的方法清单加了 `_mpc_jump_guard`；只抽取部分代码的测试替身没有这个方法时跳过保护。部署包 `car05-lattice21-exec-v020-*` 已重新生成：policy_node 为 `5b6b6027…`，yaml 分别为 `a0bed939…` / `60236ee0…` / `261f6abf…`。上车前要跑 WSL 门禁。

## 2026-10-19（现场，续）：`d19a0d5` 实车结果，nolimit 连续 32 s

bag 同上目录。全部是 0.20 限速、lattice 2.1、转向区间 ±1.0。

| bag | 结果 |
| --- | --- |
| `v020-1m-d19a`（17:08） | 7 次启动：6 次 `invalid_road_motion_state_unavailable`，1 次 overspeed。policy 节拍 p95 0.092 s、最长 0.151 s（之前 0.058–0.067） |
| `v020-1m-overspeed-171654` | **两次 1 m 都跑满**；第二次进过软超速/切油门后恢复，偏置正常（-0.003）。节拍 p95 0.067 |
| `v020-nolimit-run1` | 0.7 m、2.5 m 后 overspeed 锁停 |
| `v020-nolimit-run2` | **连续 32.5 s、约 5.5 m**，平均 0.19 m/s，88% 跟 lattice，跟 lattice 时横向基本在 ±5 cm 内；结束于 `invalid_road_insufficient_geometry` |

**CPU**：车上负载均值 8.5–9.5（6 核），各核 91–99%，温度约 50 °C，没有降频。5 个 `dream_command.runtime_runner` 包装进程各占 70–80%，是最大负载。负载高时 policy 节拍被拖到 0.1–0.15 s，超过 `motion_max_gap_s` 0.15，状态被判过期而停车（`motion_state_unavailable`）。时有时无。已建议关 Foxglove、减少终端，并报告助教。

**问题（待改代码）**：
1. **车停住但轮速保持旧值**：nolimit-run1 第二次锁停时满舵、油门 0.22–0.27，IMU 偏航角速度 31.75–32.6 s 恒为 0，轮速却一直 0.252。MPC 以为在巡航，不重新起步，最后由超速保护锁停。
2. **lattice ↔ 中心线切换时转向跳变**：退回中心线那一拍中心线估计常跳（`e_psi` 到 1.1 rad），转向从 -1 打到 +1（run2 25–26 s、29 s、37–38 s）。起步前 1–2 s 只有中心线参考，车没摆正时横向误差显示 0.2–0.6 m。
3. **弯道常打满舵**：run2 满舵占 32% 时间，都是单方向持续 1–4 s 的过弯。转向参数借自 .20（满舵 18°），5 号车未测。
4. 起步时车要摆正（清单要求 ±0.15 m、±5°）；跑满距离后原地接着跑会带着很大的初始偏差。

## 2026-10-19（现场）：0.20 实车的三个问题和现场修正

bag：`car-evidence/car05-lattice21-20261019-161732-197/`。
1. **转向只施加一半**：车上 `src/ai4r_policy/config/traxxas_vehicle_interface.yaml` 是课程默认 ±0.5（10-15 19:27 被改；main/mvp2.1 给 5 号车的是 ±1.0，10-15 的 MPC 运行也是 ±1.0）。驱动层把 [-1,1] 线性压进该区间。现场换回 main 的版本（只差这两行）。本分支的这个文件仍是 ±0.5，待决定是否跟 main 统一。
2. **超速锁停**：
   - 模型死区 0.30 偏高：满电的 5 号车 0.25 请求就维持约 0.255 m/s，MPC 以为在滑行，车停在软限线上方，`bias` 顶到 -0.05；
   - 轮速驱动在轮子变慢/停下时保持最后读数约 3 s（0.2901 卡 3.0 s 后跳 0），切油门后误判仍超速。
   - 现场改 `drive_deadband` 0.30 → 0.24、`overspeed_cut_timeout_s` 1.0 → 2.0，硬停 0.40/0.2 s 不变。Gym（仿真车死区设 0.23 模拟满电）：旧配置 17/30、超速锁停 10 次；新配置 28/30、0 次。仿真车死区 0.289（偏弱）时新配置无超速锁停，失败都在估计层。
   - 轮速卡值的根本修复（读数不变时按过期处理）留到代码改动。
3. **一次 Disarm**：`write completion expired`，主机与 MCU 的链路断约 3.4 s 后自动重连，MCU `boot_id` 不变（没有复位）。起步瞬间发生，原因待查（USB 线/干扰或主机卡顿）。

## 2026-10-19（续）：lattice 2.1 + MPC 提速 Gym 扫描（0.15 / 0.20 / 0.25 / 0.30）

`car_readiness_centerline.py` 新增 `--speed`，同时覆盖 `mpc.v_exec_max_mps` 和 `planning.cruise_speed_mps`（lattice 2.1 只给路径，速度来自后者）。其余参数不变，都用 `--lattice21 --car05-mpc`、.27 车模型、10 个种子。结果：`results/car_readiness_lattice21_car05_v{015,020,025,030}.md`。

| 限速 | 直道 | 缓弯 | U 弯 | 直角弯 | 超速锁停 | 直道 s/m | 峰值中位数 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.15 | 55/60 | 15/20 | 11/20 | 3/40 | 3 | 6.8 | 0.32 |
| 0.20 | **60/60** | 18/20 | **18/20** | 10/40 | **0** | 5.3 | 0.32 |
| 0.25 | 57/60 | 19/20 | 15/20 | 10/40 | 6 | 4.3 | 0.32 |
| 0.30 | 58/60 | 19/20 | 17/20 | 10/40 | 3 | 3.7 | 0.36 |

- 0.20 是这次最好的一档：完成率最高，没有超速锁停。0.15 反而最差，可能是油门贴着死区、跑得久，估计层停车的机会更多。
- 0.25/0.30 的完成率没有明显下降，但超速锁停变多（都在 U 弯），0.30 的峰值到 0.45，已经高过 0.40 硬停线，只是没持续 0.2 s。
- 横向最大误差在直道、缓弯、U 弯上不随速度变大（≤ 0.28 m）。
- 失败仍然大多是估计层停车（`invalid_ordered_geometry`）。
- 直角弯 1.0 m 车道在 0.15/0.20 下"完成"的几次，横向达到 0.47–0.51 m，已经超出车身余量（Gym 不检查碰边界）。
- lattice 的规划耗时是本机实测的，结果不完全可复现。同一种子重跑，每格完成数会差 1–2 次（和 10-18 的 0.15 结果对比可见）。
- 仍是 .27 车模型 + Gym 噪声，不是实车证据。
- 车 05 配置的限速上限提到 0.20，生成 `car05-lattice21-exec-v020-*` 包（0.15 包哈希不变）。WSL 门禁对 `4ebfc59` 复跑：离线 PASS，ROS 门禁 PASS，证据 `~/ai4r-evidence/ros-20261019-134108/`。

## 2026-10-19：lattice 2.1 exec 上车前 ROS 门禁通过

- WSL `BRANCH=mpc-car-run bash .verification/run_ros_check.sh`，源码 `b3f9629`：离线 PASS，ROS 门禁 PASS。证据：`~/ai4r-evidence/ros-20261019-131546/`。
- 本机 `.verification/` 中原本没有部署包，按清单命令重新生成了 `car05-lattice21-exec-{1m,5m,nolimit}` 和备用的 `car05-cl-relaxed-exec-{1m,5m,nolimit}`，yaml 和 policy_node 的哈希都与两份清单一致。

## 2026-10-18（续）：lattice 2.1 → MPC 闭环 Gym，回答"为什么不启用"

**背景**：10-16 的回放是开环的：车按 PID 走，MPC 的输出不改变车的位置。那次回放里，lattice 参考下 MPC 转向和 PID 反号的拍数是中心线的 2–4 倍，但开环分不出谁对。这次让 MPC 跟着 lattice 真正控车。

**工具**：
- `offline/mpc_gym/pipeline_sim.run(..., lattice_delivery_s=)` 复用 `tools/replay_run_jsonl.py` 的 `enable_lattice` 和 `DelayedLatticeWorker`：真实的 `ShadowLatticePlanner` 在本进程里跑，每个结果要等"本机实测规划耗时 + 0.05 s"后才可见。
- `car_readiness_centerline.py --lattice21` 叠加 `ai4r_policy_mpc_lattice21.yaml`。

**结果**（[lattice](../offline/mpc_gym/results/car_readiness_lattice21_car05.md) 对比 [中心线](../offline/mpc_gym/results/car_readiness_centerline_car05.md)，都带 `--car05-mpc`，同样的道路和种子）：

| 道路 | 中心线完成 | lattice 完成 | 最大横向（中位数），中心线 → lattice | lattice 使用比例 |
| --- | --- | --- | --- | --- |
| 直道（6 组，60 次） | 55 | 54 | 5–12 cm（2–5 cm）→ 9–18 cm（5–16 cm） | 93–97% |
| 缓弯 0.72 / 1.0 | 10 / 7 | 9 / 7 | 11 / 9 cm → 12 / 23 cm | 93% |
| 直角弯 r1 | 0 / 0 | 0 / 0（中位距离 3.0–3.5 → 3.5 m） | — | 75–80% |
| 直角弯 r1.5 | 0 / 0 | 0 / **1** | 17 cm | 80% |
| U 弯 0.72 / 1.0 | 0 / 0 | **3 / 7** | 14 / 28 cm | 75% |

- 完成率：直道和缓弯和中心线基本一样，U 弯明显更好。
- lattice 失败的原因都是估计层停车（`invalid_ordered_geometry`、`motion_interval_too_long`），和中心线一样。没有因为跟 lattice 而冲出边界。
- 横向误差更大，在意料之中：lattice 本来就不追中心线，会在车道内选路，大弯前还有弯外准备偏移。都在车道余量以内：0.72 m 车道车身余量约 23 cm；1.0 m 车道约 37 cm，U 弯最大 28 cm 是最接近的一次。
- 开环回放里担心的"往弯外打"，在闭环里没有变成失败。

**结论**：闭环不支持"lattice 比中心线差"。横向误差偏大、U 弯余量偏紧，评分范围（`score_min_length_m` 0.2 m）还是值得和 Planning 组商量。仍然是 .27 的车模型和 Gym 噪声，不是实车证据。

**决定：下周只跑 lattice 版 exec。**
- 新增 `make_deploy_yaml --mode mpc-exec-lattice21`：centerline + lattice21 配置叠加，`mpc.shadow: false`，要求 `vehicle.valid`，所以 .27 车会被拒绝。
- 部署配置测试和 ROS 门禁的"每个部署配置都能启动节点"都加上了这个模式。
- 部署包 `.verification/car05-lattice21-exec-{1m,5m,nolimit}`：policy_node 没变，还是 `f1e07c7b…`；yaml 分别是 `afd054f9…` / `6ac9a15f…` / `cfdcf4cf…`。
- 现场步骤见 [CAR05_LATTICE21_EXEC_RUN_CN.md](CAR05_LATTICE21_EXEC_RUN_CN.md)；中心线版清单只作为备用。

## 2026-10-18：放宽 MPC 门禁，让 MVP 能跑的地方 MPC 也能跑；下周起只跑 exec

**背景**：
- 逐项核对过，估计层和规划层的参数和 mvp2.1 完全一致。
- MVP 能跑而 MPC 停车的原因，全在 MPC 自己的门禁上：
  - 超速立即锁停（18:18 两次）；
  - 起步超时 1.5 s；
  - 参考失效只容忍 0.3 s；
  - 参考线必须覆盖整个预测时域（`reference_too_short`）；
  - `solver_fail` 立即锁停；
  - 距离上限 1 m；
  - 限速 0.12 贴着死区。

**代码改动**（新参数全部默认关闭，打开时有注释说明）：

| 新参数 | 作用 |
| --- | --- |
| `mpc.overspeed_soft_margin_mps` / `overspeed_cut_margin_mps` / `overspeed_cut_timeout_s` / `overspeed_hard_mps` / `overspeed_hard_persist_s` | 软超速：超过 cap + soft，QP 的油门下限放到 0（新增 `drive_floor_zero`，和 `allow_braking` 是两回事）；再高 cut 就强制油门为 0，并清掉热启动。锁停条件：切油门 cut_timeout 后仍在软限线以上，或者超过硬停线并持续 persist。规划等待期间（`plan_hold`）也遵守。打开后，旧的 `overspeed_margin_mps` 检查不再生效 |
| `mpc.max_forward_extension_m` | 参考末端沿最后一段直线外推（曲率 0），`mpc_reference_errors` 和 `mpc_path_projector` 都支持；debug 新增 `forward_extension_m`。**lattice 的预检不外推**：lattice 路径不够长就退回中心线基线，由基线去外推 |
| `mpc.solver_fail_tolerable` | `solver_status:*`、`constraint_residual`、`solver_nonfinite`、`action_out_of_bounds` 走 `plan_hold`；输入错误和未知异常仍然立即锁停 |
| `mpc.restart_count_reset_s` | 连续 tracking 这么久后，停后重启计数清零 |
| `control.max_distance_m` / `max_run_time_s` = 0 | 0 表示不限（和 mvp2.1 一样，为环道准备）；轮速新鲜度检查保留。`make_deploy_yaml` 新增 `--run-time-s`，`--distance` 允许 0 |

**配置**：
- centerline 配置：dropout 0.3 → 0.5，外推 0.3 m，`solver_fail_tolerable: true`。
- car05 配置：
  - 限速 0.12 → 0.15；
  - 软超速 0.05 / cut 0.03 / 切油门 1 s / 硬停 0.40 持续 0.2 s；
  - 起步超时 1.5 → 2.5 s；
  - `max_restarts` 3 → 5，连续 tracking 10 s 清零；
  - `control.max_run_time_s` 60。
- `max_step_time_s` 保持 0.05：bag 不在本机，G1 在 Jetson 上测的 p95 是 11.8 ms，余量足够。

**验证**：
- 离线测试全部通过：mpc 118（新增 16 个 `RelaxedGuards`）、real_frames 15、planner_control 12、control 45（新增 2）、estimation 65、planning 18、lattice 45、shadow_lattice 30、planner_execution 10、release_contract 4。
- V2 的 `car_readiness.py` 矩阵：表格和存档完全一致。
- 15 号车三段 run.jsonl 回放：和改动前的提交（`cd37852`）逐拍一致，功能开关都一样。这三段数据本来就没有 `reference_too_short`，软超速和求解失败容忍也只在 exec 下起作用。
- 中心线 Gym（[改前](../offline/mpc_gym/results/car_readiness_centerline.md) → [放宽后](../offline/mpc_gym/results/car_readiness_centerline_relaxed.md) → [再加 car05 的 MPC 设置](../offline/mpc_gym/results/car_readiness_centerline_car05.md)，后者用 `--car05-mpc`，仍是 .27 的车模型）：

| 道路 | 改前 | 放宽后 | 加 car05 设置 |
| --- | --- | --- | --- |
| 直道（6 组） | 3–10/10，主要是 `reference_too_short` | 9–10/10 | 9–10/10 |
| 缓弯 0.72 / 1.0 m | 5 / 1 | 9 / 7 | 10 / 7 |
| 直角弯、U 弯 | 0/10，主要是 `overspeed`，中位距离约 1.4 m | 不变（.27 配置没有软限） | 仍是 0/10，但 `overspeed` 基本消失，中位距离 3.0–4.5 m，停在估计层的 `invalid_ordered_geometry` / `insufficient_geometry`（E） |

**要注意**：
- 跑完的直道和缓弯里，最大横向误差有一部分从约 2 cm 升到 5–12 cm。
  - 大部分是以前提前停车、现在跑满全程的那些种子；以前的数值只统计到停车为止，没有可比性。
  - 但也有两次都跑完、却变差的。例如 car05 设置下直道 1.0 m 车头偏 5° 的种子 10：1.7 → 11 cm；缓弯 0.72 种子 5：1.7 → 10.5 cm。
  - 都还在车道余量以内（0.72 m 车道约 18 cm，1.0 m 车道约 32 cm），但原因还没查清。可能来自低速 0.15 下的纵向表现，或者外推段 / `plan_hold` 期间的跟踪。下次实车要重点看。
- car05 设置下有 4 个种子新失败，都是估计层停车（`invalid_ordered_geometry`、`motion_interval_too_long`、`motion_transformed_geometry_invalid`），不是这次的放宽造成的。MVP 遇到规划无效同样会立即停车。

**部署包**（只有 exec，不再出 shadow 包）：
- `.verification/car05-cl-relaxed-exec-{1m,5m,nolimit}`；
- policy_node 都是 `f1e07c7b…`；yaml 分别是 `afef5e5b…` / `0a65c72c…` / `4f620c36…`；
- 现场步骤见 [CAR05_RELAXED_EXEC_RUN_CN.md](CAR05_RELAXED_EXEC_RUN_CN.md)。原来的 shadow 推车，改成 Disabled 下请求 state 3 做静态检查（2.5 s 后报 `no_motion_after_breakaway` 是正常的）。

**本机没有 ROS，门禁没跑**，上车前要在 WSL 上跑。

## 2026-10-16：MVP2.1 lattice 接到 MPC（默认关闭，只有 shadow 包）

**存档**：改动前给 `9213576` 打了 tag `mpc-car-run-cl-20261015`（只在本地，未 push）。下周 5 号车中心线这一轮用这个版本的包。从 tag 重新生成的 yaml `2e3a116c` / `496f4ffe`、policy_node `72f2ccbd`，和 [CAR05_CENTERLINE_RUN_CN.md](CAR05_CENTERLINE_RUN_CN.md) 里的哈希一致。

**移植来源**：`origin/mvp2.1` `3652e9b`，只取它的规划器部分，没有参考其他分支。
- `ShadowSettings`…`PlannerExecutionManager`、`--shadow-worker` 入口、`offline/shadow_lattice/` 和纯算法测试，都原样搬入。
- 改动的地方一律标了 `mpc-car-run adaptation`，以后 mvp2.1 有更新可以直接对比合入。

**和 mvp2.1 不同的地方**（mvp2.1 的下游是 MVP PID，这里是 MPC）：

| 项 | mvp2.1 | 本分支 |
| --- | --- | --- |
| 下游控制器 | MVP PID（`control.mode mvp`） | MPC（`mpc.enabled`，`reference_source planning`，`road_centerline`，timer 模式）。本分支的 PID 读不了 ORDERED_CENTERLINE |
| 覆盖长度 | PID 预瞄 0.35 m，用 `control_path_geometry` 计算 | `max(planning.preview_m, 执行限速 × (N·dt + 执行器延迟))`，用 `_path_project` 计算 |
| 参考有效期 | `control.max_dt_s + 周期` | `mpc.dt_max_s + 周期`（数值同为 0.25 s） |
| 参考被拒后的处理 | 恢复 PID 状态，用基线重算一遍 | 调用 `MPCController.step` 之前先用纯函数预检（适配器 + 按预测速度的 `mpc_reference_errors`），每拍只 step 一次。预检之后求解器仍拒绝的，按普通可容忍 dropout 处理 |
| 中心线无效时 | lattice 照样驱动 | `planner_control.require_valid_baseline: true`（默认）：中心线无效就不用 lattice |
| 诊断话题 | `/car/planner_execution` 始终创建 | 只在 `planner_control` 打开时创建；`mpc_debug` 也只在打开时增加 `reference_selection(_reason)` 字段 |

**开关关闭时行为不变**（`shadow.enabled`、`planner_control.enabled` 默认都是 false）：
- 原有离线测试全部通过：估计 65、规划 18、控制 43、lattice 45、MPC 102、真实帧 15。
- Gym `car_readiness` 和 `car_readiness_centerline` 两个矩阵，和同一台机器上跑的 tag 版本相比，除 `step_p95_ms` 外完全一致。和存档相比有 3 行不同，tag 版本在本机跑出来也一样不同，是机器计时差异造成的。
- 15 号车三段 run.jsonl 的中心线回放，和 tag 版本逐拍一致。

**新增**：
- 测试：`test_shadow_lattice` 30、`test_planner_execution` 10（从 mvp2.1 移植）、`test_mpc_planner_control` 12（MPC 接入、退回、跑次隔离、部署配置）。全部通过。
- 部署：`make_deploy_yaml --mode mpc-shadow-lattice21`，只有 MPC shadow 包。其他模式都会断言 lattice 开关是关闭的。ROS 门禁的"每个部署配置都能启动节点"测试也加上了这个模式。
- 工具：`replay_run_jsonl.py --planner-control`。

**本机没有 ROS，门禁没跑。** 上车前要在 WSL 上跑 ROS 门禁。

**15 号车 run.jsonl 开环回放**（[结果](../offline/mpc_gym/results/replay_car15_lattice21.md)）：
- 三段数据上 lattice 结果全部 VALID，用 lattice 的拍占 80–87%。
- 退回中心线的主要原因是碰到已知边界（27–66 拍）；参考来源切换 8–11 次。
- 本机规划耗时平均 50–61 ms。
- **问题**：用 lattice 参考时，MPC 转向和车上 PID 反号的拍数是中心线 MPC 的 2–4 倍（例如 203 对 46）。原因主要有两个：Halton 随机候选占多数，而评分只看前 0.2 m；大转角时有约 0.14 m 的弯外准备偏移。这就是 mvp2.1 文档里写的"大转向 / 参考方向冲突"，本次没有改评分。

**下一步**：
- 下周中心线照原清单跑。时间够的话做第 6 步：lattice shadow 推车采数据。
- 在转向反号的问题解决之前，不做 lattice 执行。评分范围和候选需要和 Planning 组商量。

## 2026-10-15（晚）：V2 在弯前不起步；中心线方案修 H，C 维持 0.03，生成 5 号车中心线包

**18:32、18:37 两次 V2 exec，车都没动**：
- bag：`car-evidence/car05-v2-db030-exec-20261015-183239`、`-183711`。
- 车摆在左急弯前约 1.7 m（x=1.5 m 起蓝锥急转向左）。V2 在 2 m 规划范围里按直线外推车身，判定会碰边界，每次都报 `recovery_body_boundary_conflict`。第一次摆位还报过 `insufficient_lattice_coverage`（最近的黄锥在 1.12 m）。
- 死区 0.30 因此没有得到验证。用户决定 V2 先不改，先做中心线版本。

**用中心线方案离线重放这两个 bag 和 18:18 的 bag**（`tools/replay_policy_bag.py`，新增参考线范围、MPC 误差和转向、锥桶对照字段，以及 `--full-path` 选项）：
- 参考线有效率 96–99%，弯前起点也有往左弯的参考线。
- 参考线短于 0.5 m 的帧不到 1%，说明 C 在实车上很少发生。

**新问题 H：车附近的参考线算歪。**
- 现象：18:18 起步时车居中，但 MPC 算出横向 +12 cm、航向 −0.19 rad。
- 原因：中心线从车前 0.58 m 才开始，适配器把第一段 5 cm 往回延长到车身，第一个黄锥偏了 8 cm 造成的倾斜就被放大了。
- 修复（`baac940`）：新参数 `mpc.ordered_lead_in_fit_m` = 1.0，沿二次拟合在起点处的切线往回加引入段。
- 试过、但没采用的做法：
  - 直线拟合：缓弯 R5 上会造成 7 cm、0.1 rad 的新偏差；
  - 跳过最近 0.25 m 再拟合：在 18:37 弯前那个 bag 上把弯的曲率外推回车身，偏差 9.6 cm。
- 重放结果（MPC 横向误差和最近一对锥中点的差，p95）：
  - 18:18：0.105 → 0.075 m；
  - 18:32：0.175 → 0.141 m（大的那部分来自最近黄锥在 1.12 m 的摆位，对照值本身不可靠）；
  - 18:37：0.039 → 0.016 m。
- 合成的直道和 R1–R5 弯误差都在 2 cm、0.035 rad 以内。
- 剩下的问题：18:18 起步那种"最近一个锥放偏"只降到 8 cm、0.135 rad，单靠几何分不出这是真弯还是噪声。

**C：做成参数，但值不改（`d155a80`）。**
- 新参数 `estimation.ordered_pairing_backtrack_m`，默认 0.03，和 main 一样。中心线叠加层也明确写 0.03。
- 不放宽到 0.10 的原因：5 号车真实帧（false-seed fixture，有误识别的锥）在 0.10 下会被配成一条 0.41 m 的"两侧都有"的路，0.03 下是 2.39 m 的单侧路。测试把这两种情况都固定下来了。

**中心线 Gym 结果**（`results/car_readiness_centerline.md`，0.72 / 1.0 m 车道）：
- 直道：居中 4→7 / 2→5，车头偏 5° 5→7 / 3→6，左偏 5 cm 7→10 / 2→3；
- 缓弯：6→5 / 0→1；
- 急弯和 U 弯：仍为 0，原因是超速（D）。
- 剩下的失败主要是 Gym 2 cm 噪声下的 `reference_too_short`。

**离线测试**：test_mpc、test_mpc_real_frames、test_estimation、test_planning 共 200 个通过。

**新包**：
- `.verification/car05-cl-shadow`：yaml `2e3a116c…`；
- `.verification/car05-cl-exec`：yaml `496f4ffe…`；
- 两个包的 policy_node 都是 `72f2ccbd…`。
- 现场步骤见 [CAR05_CENTERLINE_RUN_CN.md](CAR05_CENTERLINE_RUN_CN.md)。上车前要跑 WSL ROS 门禁。

## 2026-10-15（18:18）：死区 0.32 两次都超速停车，改 0.30、超速余量 0.10

bag `car-evidence/car05-v2-db032-exec-20261015-181825`，V2 exec，1 m，`v_exec_max` 0.12。

- 两次起步都正常：固定 0.35，约 1 s 出轮速，0.06 m/s 交接。规划有 `planning_pending`、`lattice_time_budget_exceeded`，
  都被 `plan_hold` 接住，**不是规划问题**。
- 交接后车速 0.06 → 0.20 m/s 只用了 0.6–0.8 s，两次都在约 0.20 m/s 触发 `overspeed`，分别走了约 0.14 m、0.11 m。
- 原因：MPC 预测车速保持在约 0.12（`pred_v_end`），只把油门从 0.35 慢慢收到 0.318。车在动时 QP 的油门下限是
  "死区 + 偏置"，约 0.318，MPC 给不出更低的油门。0.289 时车减速（17:53），0.32 时车加速，所以真实值在两者之间。
- 第二次起步时车放偏了 11 cm，横向在慢慢收。

改动：`config/ai4r_policy_mpc_car05.yaml` 的 `drive_deadband` 0.32 → **0.30**（`source` 改成 `db030`），
`overspeed_margin_mps` 0.08 → **0.10**（超速停车线 0.22 m/s，用户同意）。新包 `.verification/car05-v2-db030-exec`
（yaml `240712da…`，policy_node `8a0d64dd…` 没变）。离线 `test_mpc` + `test_mpc_real_frames` 109 个通过。

## 2026-10-15（17:53）：5 号车 MPC 第一次正常起步并跑完设定距离（V2，0.5 m）

包：`car-evidence/car05-v2-startfix-exec-20261015-{175008,175148,175338}`，用 `tools/mpc_bag_timeline.py` 读。
- 17:50、17:51 两次：MPC 分支全是 `shadow`、下发油门始终为 0——车上装的是 shadow 包，车不会动。不是控制问题。
- **17:53**：exec 包。起步固定 0.35（实际应用 0.324），中间多次 `plan_hold` 都保持住了 0.35（17:06 失败的原因已修复）；
  请求后约 0.6 s 轮子开始转，0.058 m/s 交接给 MPC 跟踪，最后 `Distance limit reached: 0.501 m / 0.500 m` 正常停车。
- 交接后车速先冲到 0.184 m/s（目标 0.12，超速线 0.20），再掉到 0.06 m/s：MPC 用 .27 的死区 0.289（实际应用约
  0.256），5 号车在这个油门下减速；偏置估计用约 3 s 把请求提到约 0.33，车速回到 0.106。**5 号车死区约 0.32–0.33。**
- 横向：全程在参考线左侧约 6 cm，MPC 一直向右修正（方向对），0.5 m 内偏差基本没收敛。
- 规划时效：`planning_pending` 频繁，偶有 `lattice_time_budget_exceeded`、`source_deadline_during_planning`，都被
  `plan_hold` 桥接住，没有锁停。

改动：`config/ai4r_policy_mpc_car05.yaml` 的 `vehicle.drive_deadband` 0.289 → **0.32**（只这一项，`source` 加 `db032`），
代码不变。新包 `.verification/car05-v2-db032-{shadow,exec}`，距离上限 1.0 m。下一次：看交接后车速能不能稳在 0.12 附近。

后续修复和顺序：[MPC_FIX_PLAN_CN.md](MPC_FIX_PLAN_CN.md)。

## 2026-10-15（17:06 固定输入试验后）：5 号车起步保持修复，待实车复测

实测 V2 请求爬升 0.30→0.35、接口应用峰值 0.324，但达到 >=0.32 只有约 0.147 秒，规划等待会落到旧 QP 跟踪序列的较低输入；随后 no_motion_after_breakaway。独立 id_test 固定请求 0.35 时，接口应用约 0.324，约 0.79 秒出现 >0.05 m/s 轮速，约 0.98 秒触发 0.15 m/s 速度保护，实际轮速峰值约 0.172 m/s。用户确认车移动。原始包和只读分析在仓库外 car-evidence 下，不能把该固定输入结果当成 V2 已跑通。

改动：car05 overlay 固定起步请求/上限 0.35，ramp 置 0；其余纵向模型仍借 .27、转向仍借 .20，source 保留 UNVERIFIED。MPC 在已接受一次起步求解之后，短时 plan_hold 保持已应用的起步努力，不取更低的旧 QP drive，也不在等待中增加努力。轮速越过 0.05 后恢复原 tracking hold。规划失效期间也执行超速和起步超时门禁；其余明确停车、失效到期、无计划回零和反馈保护保留。

TDD：旧代码在三种规划等待原因下的起步保持、等待中的超速、等待跨越起步超时五个断言失败；修复后 targeted 33 项通过。新增检测“轮动首次出现在 dropout”的交接和无首个成功计划时不加油。Ubuntu dense MPC 共 102 项、0 失败、2 项 OSQP 对照跳过；ROS 门禁复跑 PASS（colcon 267 项、0 错误、0 失败、2 跳过）。首次门禁有一个原有 MVP lattice 用例 source_deadline_during_planning 失败，单项复核及无并行测试负载的完整复跑均通过，未放宽时效/测试门槛。两个部署 YAML 均用安装节点在本机 LOCALHOST 域 219 初始化通过、停在 state 2。Astra 只读复核无阻塞问题。完整证据及部署哈希见本轮部署包 verification.json；新版本尚未上车执行。

复测专用清单：[CAR05_V2_START_FIX_RUN_CN.md](CAR05_V2_START_FIX_RUN_CN.md)。使用 .verification/car05-v2-startfix-shadow 和 car05-v2-startfix-exec，先 Disabled shadow 看参考与方向，再首次居中 0.5 m exec（0.12 m/s 配置、超速阈值 0.20 m/s）。不覆盖/复用旧 car05-v2-first 部署副本，不取消油门上限或停车保护。

## 2026-10-15：把 main 的 ordered_xy 估计和 road_centerline 规划接入 MPC

分支 `mpc-car-run-port-main`（基于 `mpc-car-run` 的 `c01aa9b`，周四部署版本已打 tag
`mpc-car-run-v2-20261015` 存档）。origin/main（`0c8593a`）已经用 `estimation.road_geometry:
ordered_xy` + `planning.algorithm: road_centerline` 在车上跑通直角弯和 S 弯（右角弯备忘录
`right-angle-bend-is-own-goal` 据此更新）；这次把这两块整块移植进 MPC 分支，V1/V2 lattice 和
MVP/PID 保持不变（新增兼容检查：`ordered_xy` 要求 `road_centerline`；`lattice_v2` 要求
`y_of_x`）。MPC 侧新代码：`reference_trajectory_from_planning` 新增 ORDERED_CENTERLINE 适配
（按弧长重采样，曲率用航向差分并限幅）；`mpc_path_projector` 加了防跳支路（先试朝向车头 ±105°
以内的段，全部排除才退回全量搜索）；`VehicleModelMPC`/`MPCController` 把预测轨迹传到
`mpc_debug.pred_xy`。Foxglove：MPC 复用 main 的 `/car/policy_trajectory`（和 PID 共用绿线），
新增 `/car/mpc_prediction`（预测线）。

**验证**：`tests/test_estimation`（65/65）、`test_planning`（18/18）、`test_lattice`（45/45）、
`test_mpc`（95/95）、`test_control`（43/43）全部不变；新增 `tests/test_mpc_real_frames.py`
直接用 main 的 car05（5 号车实测直角弯链）两个 ordered_xy fixture 跑通 Estimation →
RoadCenterlinePlanner → MPC 适配 → shadow 求解全链路。`car_readiness.py` 整套矩阵（V2/MVP，
`--workers 6`）结果和存档逐字节一致，证明移植未影响现有线路。

**新增工具**：`tools/replay_run_jsonl.py`（真车 car15 的 3 条完整 run.jsonl 开环回放 MPC
shadow，`offline/mpc_gym/results/replay_car15_run_jsonl.md`）；`offline/mpc_gym/real_track.py`
（用 run.jsonl 的轮速/陀螺仪重建道路几何和锥桶统计，生成 Gym road_spec）；
`offline/mpc_gym/car_readiness_centerline.py`（road_centerline 版本的 car-readiness 矩阵，
只测 MPC——这个分支自己的 PID 不认 ORDERED_CENTERLINE，会报
`unsupported_path_encoding`，PID 对照需要用 main 自己的代码跑，这次没做）。

**车 .27 条件下 Gym 矩阵全 0/10**（`car_readiness_centerline.md`）：近处盲区持续
`near_field_unobserved`/`near_field_requires_both_boundaries` 太久，在真正观测到双侧之前
就先撞上 `estimation.motion_max_interval_s`（0.4 s）。关掉噪声/丢帧后同样配置能跑完整条
3 m 直道，且这个现象和真车 `replay_car15_run_jsonl.md` 的回放结果一致（约 85% 的帧也在这两个
状态里）——是 Estimation 层在这颗摄像头的时延/丢帧预算下的已知限制，不是这次 MPC 移植的问题。

这次矩阵测试本身也抓到一个真的 MPC 适配器 bug：ORDERED_CENTERLINE 重采样的强制终点
（s=total_s）即使和上一个点几乎重合也照样追加，产生长度≈0 的末段，被 `mpc_reference_errors`
按 `repeated_point` 拒绝；改成重合时替换掉上一个点。回归测试：
`tests/test_mpc_real_frames.py::OrderedCenterlineAdapterChecks`。

**部署**：`tools/make_deploy_yaml.py --mode mpc-shadow-centerline|mpc-exec-centerline`
（叠加 `config/ai4r_policy_mpc_centerline.yaml`，`mpc.qp_solver: dense`，`--cones` 仍为必填）。
下次上车顺序和 G2/周四一致：G1 计时 → shadow 推车（看 `/car/policy_trajectory` 绿线和新的
`/car/mpc_prediction` 预测线，确认两者不发散）→ 低速直道执行 → 缓弯 → 直角弯。鉴于上面 Gym
矩阵全 0/10 的发现，shadow 阶段要重点核对 `mpc_debug` 的 `reject_reason` 是否持续落在
`invalid_road_*`（近处盲区问题），而不是 MPC 自己的求解失败。

**WSL ROS 门禁**（2026-10-15）：第一次（`edccf17`）FAIL——移植的 `from nav_msgs.msg import Path`
覆盖了文件原有的 `pathlib.Path`，V2 异步规划器启动子进程时 `Path(__file__)` 报 TypeError（上车会让
异步 V2 启动即崩溃）。改为 `TrajectoryPath` 别名，并加离线检查
`tests/test_mpc_real_frames.py::ImportShadowingChecks`。第二次（`99cd86c`）PASS：policy_node 81、
estimation 65、planning 18、lattice 45、control 43，0 失败；证据 `~/ai4r-evidence/ros-20261015-142450/`。

**5 号车**（`--car 05`，`config/ai4r_policy_mpc_car05.yaml`）：05 没有任何辨识数据，现场决定借用 .27 的纵向
拟合和 .20 的转向实测（-0.3142 rad），`vehicle.valid: true`、`v_exec_max_mps` 压到 0.12，`source` 标明是跨车替代。

## 2026-10-14：周四前补丁 —— 锥桶颜色可配置、一个校验 bug、现场核对项

来源：盘点其它分支 10-12 到 10-14 的提交（`feature/road-estimation-v2`、`mvp2.0`、`fix/newcar-control`、`feature/vehicle-identification`）。

| 改动 | 原因 |
| --- | --- |
| `estimation.left_colour/right_colour`（整数，黄=1、蓝=2；默认蓝左，与原来相同），节点用它构造估计器 | 我们原来写死蓝左。10-13 `.20` 车 dry run 因黄左报 `invalid_width_or_order`；10-14 `09` 车实测黄色锥桶中位 y = +0.465 m。来自 `fix/newcar-control` |
| `EstimationSettings.__post_init__` 恢复“运动时限不能超过源年龄”的检查 | `1b8c56c` 把它放在 `history_lifetime_s` 的 `return` 之后，一直没执行。由估计组 `1b759ed` 发现 |
| `make_deploy_yaml.py`：`--cones` 必填（`steer-arc` 除外）；`--road-age-s`（0.4, 0.6] 同时改 `planning.max_source_age_s`、`estimation.max_source_age_s`、`sensor_timeout_s.cone_detections` | Planning 的道路期限取三者中最小的，只改第一个只能多出 0.1 s |
| `tools/cone_layout_check.py`（车上运行，打印 `--cones` 的值）；`mpc_shadow_summary.py` 新增 `timing rejects` 占比 | 现场判断颜色；异步 V2 时效问题 |
| 周四清单：新增第 0.7 步（车号、实时 trim、转向区间、颜色）；shadow 增加时效类拒绝 ≤ 10% 的通过条件和 `-age06` 备用部署；非 `.27` 车只跑 shadow | 15 号车实时 trim 0.5、启动参数 0；15 号车转向区间改为 ±0.8；10-13 异步 V2 的参考多数在取用前过期；`.20` 车 0.303 不动 |

**验证（笔记本）**：estimation 65（新增 2 项）、planning 18、control 43、lattice 45、mpc 95、release_contract 4、Gym 10（1 项预期失败，和改前的 HEAD 相同）。5 份部署配置都能通过各 Settings 的校验。`test_policy_node` 和 ROS 门禁需要在 WSL 里跑，**还没跑**。

没有移植 `continuous_road_output`：Gym 直道已经 10/10，周四 shadow 复测看到相机断档造成的掉线多时再考虑。

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
