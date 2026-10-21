# 5 号车 lattice 2.1 + MPC 实车（2026-10-19 这周起，只跑 exec）

**这周只跑这一版。** 规划器用 mvp2.1（`3652e9b`）的 lattice 2.1：时间融合，1 Hz 子进程。MPC 优先跟 lattice 选出的路径；lattice 不可用时，同一拍退回 `road_centerline` 中心线。

MPC 的放宽和 [CAR05_RELAXED_EXEC_RUN_CN.md](CAR05_RELAXED_EXEC_RUN_CN.md) 完全相同：限速 0.15、软超速、末端外推、求解失败可容忍、起步超时 2.5 s 等。那份清单现在只作为备用，lattice 版有问题时可以换回中心线版。

**为什么现在可以跑 lattice**（[MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 2026-10-18 续）：
- 10-16 的开环回放里，MPC 跟 lattice 时和 PID 反号的拍数是中心线的 2–4 倍。但开环分不出谁对。
- 这次闭环 Gym（[结果](../offline/mpc_gym/results/car_readiness_lattice21_car05.md)）：
  - 直道和缓弯的完成率和中心线基本一样；
  - U 弯从 0/10 提高到 3/10 和 7/10；
  - 失败都停在估计层，没有一次是跟 lattice 冲出边界。
- **代价**：横向误差更大。直道最大 9–18 cm，1.0 m 车道的 U 弯最大 28 cm。lattice 本来就不追中心线，大弯前还会先往弯外偏。

## 部署包

```text
.verification\car05-lattice21-exec-1m        yaml afd054f9…  policy_node f1e07c7b…   1 m / 60 s
.verification\car05-lattice21-exec-5m        yaml 6ac9a15f…  policy_node f1e07c7b…   5 m / 60 s
.verification\car05-lattice21-exec-nolimit   yaml cfdcf4cf…  policy_node f1e07c7b…   不限 / 不限
```

**0.20 m/s 版**（2026-10-19：`ai4r_policy_mpc_car05.yaml` 的限速上限从 0.15 提到 0.20，0.15 包的哈希不变；依据是同日 Gym 扫描中 0.20 是最好的一档，见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md)）：

```text
.verification\car05-lattice21-exec-v020-1m        yaml 24da6b96…  policy_node 3567d439…   1 m / 60 s
.verification\car05-lattice21-exec-v020-5m        yaml 923a3ab6…  policy_node 3567d439…   5 m / 60 s
.verification\car05-lattice21-exec-v020-nolimit   yaml 6dfc0d1c…  policy_node 3567d439…   不限 / 不限
```

同日现场改了 5 号车模型（16:55 后）：`vehicle.drive_deadband` 0.30 → 0.24（满电时 0.25 的请求就能维持约 0.255 m/s），`mpc.overspeed_cut_timeout_s` 1.0 → 2.0；0.40/0.2 s 硬停不变。上表是改后的哈希；0.15 包也随之变化，用前重新生成。车上 `traxxas_vehicle_interface` 的转向区间必须是 ±1.0（10-19 发现被改回 ±0.5，转向只施加一半）。

10-19 晚（上表是这一版的哈希，policy_node 也变了）：新增两项保护，只在 5 号车配置里打开。
- **轮速读数过期**（`mpc.wheel_hold_stale_s` 0.5）：
  - 轮速驱动在轮子变慢或停下后会保持最后读数约 3 s。读数 0.5 s 不变就视为过期，此时软超速计时和偏置估计暂停。
  - 如果同时 IMU 偏航角速度 < 0.02 rad/s、转向 ≥ 0.3（或读数 1.5 s 不变、IMU 静止），判为车卡住：车速按 0 交给 MPC，重新给起步油门。
  - `mpc_debug` 新字段：`wheel_held_s`、`wheel_fresh`、`speed_override`（`stall`）、`speed_raw_mps`。
- **中心线跳变保护**（`mpc.switch_hold_s` 0.25）：中心线参考的 `e_psi` 超过 0.5 rad，或误差相对上一拍跳变（`e_psi` 0.5 rad / `e_y` 0.15 m），就继续执行上一个已求解的输入序列（`plan_hold`），最多 0.25 s，之后照常使用中心线。`mpc_debug` 新字段：`reference_gate`（`baseline_jump` / `baseline_jump_expired`）、`reference_gate_s`。
- 17:55 那次之后（同一个表的哈希）：
  - 判为卡住后保持住，直到出现新的轮速读数（原来会来回闪，多算了重启次数）；
  - `vehicle.drive_deadband` 0.24 → 0.25，`mpc.drive_bias_max` 0.05 → 0.10，覆盖 0.15–0.35，满电和用过一阵的电池都在范围内；
  - `mpc.max_restarts` 5 → 8；
  - `vehicle.drive_max` 仍是 0.35。
- 18:20 那两次之后（同一个表的哈希）：
  - 读数过期时，超速保护不再切油门，也不再放开油门下限（只保留 0.40 硬停）；
  - 重新起步后 2 s 内再掉回起步状态，不重复计数（`restart_merge_s` 2.0），`max_restarts` 10；
  - 车速低于 0.15 时油门上限 0.40，高于 0.15 时 0.35（`vehicle.drive_max` 0.40、`mpc.high_speed_drive_max` 0.35）；
  - 起步油门从 0.35 开始，每秒加 0.05，最多到 0.40。
  - `mpc_debug` 新字段：`restarts`、`drive_cap`。
- 18:45 那次之后（同一个表的哈希）：
  - 低速档油门上限 0.40 → **0.38**，切换车速 0.15 → **0.12**；判断用原始轮速和滤波车速中较大的那个；
  - 转向 `steering_gain_rad` -0.3142 → **-0.37**，上限 ±0.37。实测满舵 23–25°，0.37 取的是和原来 18° 之间的折中值。
  - 核对参数：`vehicle.drive_max` 0.38、`mpc.high_speed_mps` 0.12、`vehicle.steering_gain_rad` -0.37。
- 看包时：弯道里出现 `speed_override=stall` 之后应该是 `phase=starting`、油门 0.35，车随即重新动起来；`reference_gate=baseline_jump` 期间分支是 `plan_hold`，转向不应再从 -1 打到 +1。

10-19 深夜（[MPC_NEXT_PLAN_CN.md](MPC_NEXT_PLAN_CN.md) 第 2 节，**还没在实车上验证**；上表已是这一版 `b0c660c` 的哈希）：新增三项，只在 5 号车配置里打开。
- **转弯阻力**（`mpc.turn_drive_per_steer` 0.06）：舵量 |s| 时，MPC 和偏置估计都把死区抬高 0.06×|s|（满舵 +0.06）。弯里油门下限随之变高，偏置不再把转弯阻力当误差学进去。
- **起步回差 + 平顺交接**（`mpc.rest_speed_mps` 0.03，`mpc.handover_s` 0.5，`mpc.handover_drive_margin` 0.04）：
  - 起步 → 跟踪仍在 0.05 m/s；跟踪中车速低于 **0.03** 才回到起步，0.03–0.05 之间保持原状态；
  - 起步切到跟踪后 0.5 s 内，油门不超过"死区 + 偏置 + 转弯阻力 + 0.04"；超速切油门（`cut`）照样为 0，起步油门不受影响。
- **参考来源切换限速**（`mpc.switch_blend_s` 0.4，`mpc.switch_steer_rate_per_s` 1.5）：lattice 和中心线之间每切换一次（两个方向），之后 0.4 s 内转向每拍最多变 1.5×dt（20 Hz 下 0.075）。原有的 `switch_hold_s` 跳变保护不变。
- 核对参数（车 1，Disabled 下）：

  ```bash
  for p in mpc.turn_drive_per_steer mpc.rest_speed_mps mpc.moving_speed_mps mpc.handover_s \
           mpc.handover_drive_margin mpc.switch_blend_s mpc.switch_steer_rate_per_s; do
    printf "%-32s " "$p"; ros2 param get "$N" "$p"
  done
  ```

  预期依次为 0.06、0.03、0.05、0.5、0.04、0.4、1.5。前四项里出现 0（`turn_drive_per_steer`、`rest_speed_mps`、`handover_s`、`switch_blend_s` 的默认值）说明部署的不是新包。
- `mpc_debug` 新字段：
  - `turn_drive`：这一拍加到死区上的转弯阻力（0–0.06）；直行接近 0，满舵 0.06。
  - `handover`：`true` 表示处在起步后的交接限幅里。每次 `phase` 从 `starting` 变成 `tracking` 之后应连续约 10 拍为 `true`，这期间 `applied_drive` ≤ 死区 + `drive_bias` + `turn_drive` + 0.04。
  - `steer_slew`：这一拍允许的最大转向变化，没有限幅时为 `null`。`reference_selection` 每次变化后约 8 拍为 0.075，期间相邻两拍 `applied_steer_action` 之差不超过它；`candidate_steer_action` 仍是 MPC 原始解，可能更大。
  - 车 3 观察时可以在 grep 的字段里加上 `turn_drive|handover|steer_slew|applied_drive|applied_steer_action`。
- 看包时（`tools/car05_run_stats.py`，和 10-19 对比）：重启次数里"低于 moving speed"那一类、"起步后 2.5 s 内进入软超速/切油门"的比例应该明显下降；满舵弯里不应再减速到停；baseline → planner 的 `|dsteer|` 中位数应 ≤ 0.075。

**10-20：PI 油门（feedback）A/B**（**还没在实车上验证**）。新增 `mpc.longitudinal_mode: feedback`：
- 跟踪阶段，油门由轮速 PI 决定，MPC 只决定转向。
- 起步、卡住识别、硬停都不变。
- 和 Zhenyu 分支的完整区别、移植了什么、为什么增益是 0.2，见 [MPC_BRANCH_COMPARISON_CN.md](MPC_BRANCH_COMPARISON_CN.md)。

上面三个 joint 包和下面的 `-fb` 包都已按这一版重新生成。joint 包的 yaml 哈希也变了，因为基础配置多了新参数的默认值；行为不变。

下表已按"10-20：规划检查长度"（见下）重新生成，7 个包都带 `shadow.check_length_mode: travel_plus_window`。和上一版（policy_node `34048cbf…`）相比，yaml 只多了这一行。

```text
.verification\car05-lattice21-exec-v020-1m               yaml c81faa12…  policy_node 9f136686…   joint     1 m / 60 s
.verification\car05-lattice21-exec-v020-5m               yaml a96a60b8…  policy_node 9f136686…   joint     5 m / 60 s
.verification\car05-lattice21-exec-v020-nolimit          yaml d35fc3b1…  policy_node 9f136686…   joint     不限
.verification\car05-lattice21-exec-v020-1m-fb            yaml 1c605ece…  policy_node 9f136686…   feedback  1 m / 60 s
.verification\car05-lattice21-exec-v020-nolimit-fb       yaml dd3d8870…  policy_node 9f136686…   feedback  不限
.verification\car05-lattice21-exec-v020-nolimit-fb-nomm  yaml 3f699029…  policy_node 9f136686…   feedback，measured_motion 关
.verification\car05-lattice21-exec-v020-nolimit-fb-soft  yaml e9ac3ac9…  policy_node 9f136686…   feedback，kp 0.15 / ki 0.1
```

生成命令：在 joint 包的命令后面加 `--longitudinal feedback`（或 `feedback-nomm`、`feedback-soft`）。

- **`-fb` 包的超速线**：软线 0.30、切油门 0.35，硬停仍是 0.40 / 0.2 s。现场"明显超过 0.25 就 Disable"对 `-fb` 包仍然适用。
- **核对参数**（Disabled 下）：

  ```bash
  for p in mpc.longitudinal_mode mpc.fb_kp mpc.fb_ki mpc.measured_motion mpc.overspeed_soft_margin_mps mpc.turn_drive_per_steer; do
    printf "%-32s " "$p"; ros2 param get "$N" "$p"
  done
  ```

  `-fb` 包预期依次为 feedback、0.2、0.2、true、0.1、0.06。joint 包是 joint、0.2、0.2、false、0.05、0.06。
- **`mpc_debug` 新字段**：
  - `longitudinal_mode`；
  - `prediction_mode`：起步阶段是 `force_model`，跟踪阶段是 `measured_speed_hold`；
  - `fb_target`、`fb_error`、`fb_integral`、`fb_command`：跟踪时 `applied_drive` 等于 `fb_command`；起步时 `fb_command` 为 null；
  - `fb_frozen`：读数过期时为 true，期间油门和积分都不变。
- **步骤**：
  1. 先跑 `-1m-fb`：确认能起步；交接后油门从 0.35 平滑下降；`fb_command` 没有周期性来回摆动。
  2. 再做 A/B：同一条赛道（小弯 + 直线 + 急弯），`v020-nolimit` 和 `v020-nolimit-fb` 交替各跑 2 次，至少 3 轮，记下电量或时间，全部录包。
- **判定**（`tools/car05_run_stats.py`，按模式分组）。10-19 joint 的基线是：重启 21 次；起步后 2.5 s 内冲到软线 17/22；跟踪车速标准差 0.063；21% 的时间处在软线或切油门状态。

  `-fb` 包要同时满足下面四条，5 号车才改用 feedback：
  - 重启次数减半以上；
  - 车速标准差和处在软线的比例都下降；
  - 没有新的停车原因；
  - planner/baseline 的 e_y、e_psi p95 不变差。
- **出问题时**：
  - `fb_command` 来回摆（统计里 `drive reversals` 明显偏高）：换 `-fb-soft`。
  - 转向比 joint 差：换 `-fb-nomm`，看是不是实测车速预测（`measured_motion`）造成的。
  - 都不行就回到 joint 包。

**10-20：规划检查长度**（**还没在实车上验证**；计划和 Gym 数据见 [LATTICE_CHECK_LENGTH_PLAN_CN.md](LATTICE_CHECK_LENGTH_PLAN_CN.md)）：
- **改了什么**：lattice 规划时的碰撞检查原来只查 `speed × hold`（约 0.22 m），执行时每拍查到车前方 `preview_m + 0.15`（约 0.5 m），急弯里 0.22 m 之后那段一到执行就撞，于是退回中心线。新增 `shadow.check_length_mode`，5 号车用 `travel_plus_window`：规划时查 `speed × hold + preview_m + 0.15`（约 0.72 m），覆盖路径使用期间每一拍的执行窗口。默认 `travel` 和原来一样，其他车不变。执行检查、lattice 其他参数和 MPC 都没改。
- **Gym（每组 60 次）**：急弯切换从 294 / 164 / 216 次降到 22 / 30 / 30 次（U 弯 / R1.5 / R1），lattice 占比 98–99%，最大横向误差 0.29 → 0.17 m（U 弯）、0.25 → 0.07 m（R1.5）。完成 38 → 34/60，仍由估计层失败决定，差别在重跑波动附近，实车 A/B 时要留意。
- **包**：上表 7 个包都已包含这项改动，不另出包。
- **参数核对**（Disabled 下，在第 2 步的检查后面加）：

  ```bash
  ros2 param get "$N" shadow.check_length_mode
  ```

  预期 `travel_plus_window`。
- **看包时看什么**：
  - `/car/shadow_planning` 里 `check_length_m` 约 0.7 m（0.20 m/s 时；静止约 0.5 m），`check_length_mode` 为 `travel_plus_window`，`travel_check_m` 和 `execution_window_m` 是它的两部分；
  - 急弯里 `/car/mpc_debug` 的 `reference_selection` 很少再变成 `baseline`；
  - `/car/planner_execution` 的 `reason` 里 `known_boundary_collision` 明显减少；
  - 窄处查得更长，可能更常出现 `ALL_COLLIDE`（同样退回中心线），要和 `known_boundary_collision` 分开数。

用 `--v-exec-max 0.2`、`--out .verification\car05-lattice21-exec-v020-*` 生成。0.20 版下软超速线是 0.25，0.28 以上强制油门为 0，硬停 0.40；第 3 步"车速在 0.10–0.20 之间"改为 0.15–0.25，"明显超过 0.25 就 Disable"改为 0.30。

重新生成的命令（距离和时间分别换成 `1.0 60`、`5.0 60`、`0 0`）：

```powershell
python tools/make_deploy_yaml.py --mode mpc-exec-lattice21 --car 05 --cones yellow-left --distance 1.0 --run-time-s 60 --v-exec-max 0.15 --out .verification\car05-lattice21-exec-1m
```

上车前，WSL ROS 门禁要对这一版代码跑通（`.verification/run_ros_check.sh`）。门禁里"每个部署配置都能启动节点"的测试已经包括 `mpc-exec-lattice21`。

**10-21：sim 分支移植后重新生成**（分支 `mpc-car-run-sim-port`；**还没在实车上验证，WSL 门禁待跑**）。见 [SIM_BRANCH_PORT_PLAN_CN.md](SIM_BRANCH_PORT_PLAN_CN.md) 和进度日志 10-21。
- **改了什么**：lattice 弧长查找改二分、knot 缓存（结果逐位不变，本机快 6–9%）；新增 `shadow.collision_coverage`/`score_coverage`、`estimation.colour_assignment`，**包里全部是默认值**（`prefix`/`prefix`/`fixed`），行为和上一版相同。
- 7 个包用和上面相同的命令重新生成（本机 `.verification` 里原来是 `34048cbf…` 那一版）。yaml 和上一版相比只多了 `colour_assignment: fixed`、`collision_coverage: prefix`、`score_coverage: prefix`（以及 10-20 的 `check_length_mode`）。

```text
.verification\car05-lattice21-exec-v020-1m               yaml 3fd7181c…  policy_node 3cfdf86b…   joint     1 m / 60 s
.verification\car05-lattice21-exec-v020-5m               yaml 0eb7b4b8…  policy_node 3cfdf86b…   joint     5 m / 60 s
.verification\car05-lattice21-exec-v020-nolimit          yaml b72f7f5c…  policy_node 3cfdf86b…   joint     不限
.verification\car05-lattice21-exec-v020-1m-fb            yaml 33b59a1e…  policy_node 3cfdf86b…   feedback  1 m / 60 s
.verification\car05-lattice21-exec-v020-nolimit-fb       yaml 49097dd6…  policy_node 3cfdf86b…   feedback  不限
.verification\car05-lattice21-exec-v020-nolimit-fb-nomm  yaml ba348449…  policy_node 3cfdf86b…   feedback，measured_motion 关
.verification\car05-lattice21-exec-v020-nolimit-fb-soft  yaml e5ffd700…  policy_node 3cfdf86b…   feedback，kp 0.15 / ki 0.1
```

- **参数核对**（Disabled 下，接在上面的检查后面）：

  ```bash
  for p in shadow.check_length_mode shadow.collision_coverage shadow.score_coverage estimation.colour_assignment; do
    printf "%-32s " "$p"; ros2 param get "$N" "$p"
  done
  ```

  预期依次为 travel_plus_window、prefix、prefix、fixed。
- **这次最要紧的是 lattice 耗时**：本机上 `travel_plus_window` 比 `travel` 慢约 56%（82 → 128 ms/次，碰撞检查和评分变长），① 只追回 6–9%。`/car/shadow_planning` 的 `elapsed_s` 和 `timing_s` 要记下来；`late_for_requested_period` 为 true（超过 1 s）或 `/car/planner_execution` 里 `waiting_for_planner` 明显变多，就说明 Jetson 上不够用，回到 `travel` 的包对比。同时看整机 CPU（P4）。

## 1. SSH、Foxglove、停车、备份

和 [CAR05_V2_START_FIX_RUN_CN.md](CAR05_V2_START_FIX_RUN_CN.md) 第 1、2 步一样。

## 2. 部署 1 m 包，Disabled 下做静态检查

```powershell
Deploy-StartFix .verification\car05-lattice21-exec-1m
```

车 1：核对哈希、构建、重启，和中心线版的第 2 步一样。然后检查参数：先按中心线版第 2 步的表全部核对一遍，再加查下面几项：

```bash
for p in shadow.enabled shadow.rate_hz shadow.score_min_length_m planner_control.enabled \
         planner_control.require_valid_baseline mpc.shadow; do
  printf "%-40s " "$p"; ros2 param get "$N" "$p"
done
dream runtime logs ai4r_policy --tail 50 | grep -i "shadow worker"
```

预期：
- 依次为 True、1.0、0.2、True、True、**False**；
- 日志里**不应该**出现 `Shadow worker failed`。

车 3 观察：

```bash
PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data |
grep --line-buffered -oE '"(branch|phase|reject_reason|reference_selection|reference_selection_reason|speed_mps|e_y_m|candidate_steer_action|overspeed_state)": ?[^,}]*'
```

**车保持 Disabled**，请求 state 3：

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
```

- 开头约 1 s 是 `reference_selection=baseline`、`waiting_for_planner`，这是正常的：lattice 每秒出一次结果。
- 之后应该切到 `planner`。
- 2.5 s 后报 `no_motion_after_breakaway` 并回到 state 2，这也是正常的，因为车 Disabled 不会动。

| 摆法 | 通过标准 |
| --- | --- |
| 直道居中 | 切到 `planner`；Foxglove 里蓝线（lattice）和绿线（MPC 用的参考）从车附近开始、顺着路；`candidate_steer_action` 小 |
| 左偏 5 cm | 候选转向往右（正值是右转） |
| 车头左偏 5° | 候选转向往右 |

一直停在 `baseline`，或者转向方向不对：不上路跑，录 30 s 的包带回来。

## 3. Exec：1 m → 5 m → 不限

车 2 录包，比中心线版多了 lattice 的几个话题：

```bash
ros2 bag record -o "$HOME/ai4r-car-bags/car05-lattice21-$(date +%Y%m%d-%H%M%S)" --topics \
  /car/mpc_debug /car/mpc_prediction /car/policy_trajectory /car/planner_execution \
  /car/shadow_trajectory /car/shadow_checked_prefix /car/shadow_reference /car/shadow_planning \
  /car/policy_fsm_state_string /car/traxxas_state \
  /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized \
  /car/wheel_speed_m_per_sec /car/cone_detections /car/imu/data /car/diagnostics /tf /tf_static
```

每一次的流程和中心线版第 3 步一样：Enable → state 3 → 停车后马上 Disable → 读状态和日志。

| 阶段 | 包 | 赛道 | 进入下一阶段的条件 |
| --- | --- | --- | --- |
| A | `-1m` | 直道居中 | 两次都正常报 `Distance limit reached`；车速在 0.10–0.20 之间；转向方向对；大部分时间是 `planner` |
| B | `-5m` | 直道，然后缓弯 | **连续 3 次**跑满 5 m，没有锁停 |
| C | `-nolimit` | 整条赛道或环道 | 一直有人跟车，随时 Disable |

以下情况立即 Disable：
- 转向往弯外打，而且越打越大（大弯前先往弯外偏约 14 cm 是 lattice 的设计，但不应该接近边界）；
- 车蛇行；
- 车接近边界；
- 车速明显超过 0.25。

## 4. 判定

中心线版第 4 步的表全部适用。lattice 另外要看：

| 现象 | 说明 | 下一步 |
| --- | --- | --- |
| `reference_selection` 频繁在 planner 和 baseline 之间切换，切换时转向跳变 | 两种参考差得太远 | 记下切换次数和原因（`reference_selection_reason`），把 bag 带回来 |
| 退回中心线的原因多半是 `known_boundary_collision` | lattice 认为自己的路径会碰到已知边界，属于正常保护 | 记下比例 |
| 弯里横向偏得多，但没碰边界 | lattice 的选路和弯外准备 | 记下来，交给 Planning 组（评分只看前 0.2 m） |
| 日志里出现 `Shadow worker failed` | 子进程挂了，MPC 会一直跟中心线 | 停止 exec，把日志带回来 |
| 规划耗时（日志里的 `elapsed_s`、`round_trip_s`）接近 1 s | 车上算得太慢，lattice 结果会过期 | 记下数值 |

同一个停车原因出现两次，就停止 exec。不在现场改阈值。lattice 版不行时，可以换中心线版（`car05-cl-relaxed-exec-*`）继续跑，每次换包都要重新核对哈希。

## 5. 下载与恢复

和 V2 文档第 6 步一样。包名用 `car05-lattice21-*`，bag 拷到 `E:\26b\AI4R\car-evidence\`。

**看包：俯视图**（`tools/bag_birdseye.py`，需要 `pip install mcap mcap-ros2-support matplotlib`，不需要 ROS）：

```powershell
python tools/bag_birdseye.py ..\car-evidence\<目录>\<包名>                 # 每次运行一个面板
python tools/bag_birdseye.py ..\car-evidence\<目录>\<包名> --at 103.8      # 某一时刻的局部图
python tools/bag_birdseye.py ..\car-evidence\<目录>\<包名> --gif 98 108    # 一段时间的动画
```

时间是从包里第一条消息开始的秒数，和 `mpc_debug` 时间线一致。输出在 `.verification\birdseye\`：

- **每次运行一个面板**：车从原点出发，朝 +x。路线按车速着色；画出 2 m 内看到的所有锥桶；标出停车（附原因）、重启、超速、卡住、lattice/中心线切换的位置。
- **局部图**（`--at`）：这一帧的锥桶检测、接受的参考路径、MPC 预测、lattice 路径，以及当时的分支、阶段、车速、油门、转向。

注意：位置是用轮速和 IMU 推算的，不是真值。卡住附近的距离会偏长；每次运行单独画一个面板，是因为两次运行之间车是被搬回起点的。

