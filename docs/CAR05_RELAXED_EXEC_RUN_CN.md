# 5 号车 中心线 + MPC 放宽版实车（2026-10-19 这周起，只跑 exec）

目标：MVP 能跑的地方，MPC 也能跑。估计层和规划层与 mvp2.1 完全一致（`ordered_xy` + `road_centerline`）；这一版只放宽 MPC 自己的门禁。改动说明见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 2026-10-18 条目。

**这周起不再跑 shadow 推车。** 原来靠 shadow 查的两件事，改到下面第 2 步检查：
- 参考线从车附近开始、方向顺着路；
- 转向修正的方向对。

第 2 步是车 Disabled 时请求 state 3 做的静态检查。

## 和上一版（`car05-cl-exec`）的区别

| 项 | 上一版 | 这一版 |
| --- | --- | --- |
| 限速 | 0.12 | **0.15** |
| 超速 | 超过 0.22 立即锁停 | **软限**：超过 0.20 时 MPC 可以把油门降到死区以下；超过 0.23 油门强制为 0，转向照常。**硬停**：切油门 1 s 后仍高于 0.20，或者超过 0.40 并持续 0.2 s |
| 起步超时 | 1.5 s | **2.5 s** |
| 停后重新起步 | 每次 run 最多 3 次 | 最多 5 次；连续 tracking 10 s 后计数清零 |
| 参考失效容忍 | 0.3 s | **0.5 s** |
| 参考线不够长 | `reference_too_short`，容忍 0.3 s 后锁停 | 末端沿切线直线外推，最多 0.3 m |
| 求解器数值失败 | 立即锁停 | 当作参考失效处理（`plan_hold`），超过 0.5 s 才锁停 |
| 距离 / 时间上限 | 1 m / 30 s | 分三档：1 m、5 m、不限 |

没变的：
- 死区 0.30，还没在车上验证过；
- 起步油门 0.35；
- 转向借 .20 的数据；
- 估计层和规划层的参数；
- 传感器时效 1.5 s；
- 操作员 Stop。

## 部署包

三个包的 policy_node 相同，只有距离和时间上限不同：

```text
.verification\car05-cl-relaxed-exec-1m        yaml afef5e5b…  policy_node f1e07c7b…   1 m / 60 s
.verification\car05-cl-relaxed-exec-5m        yaml 0a65c72c…  policy_node f1e07c7b…   5 m / 60 s
.verification\car05-cl-relaxed-exec-nolimit   yaml 4f620c36…  policy_node f1e07c7b…   不限 / 不限
```

重新生成的命令（在仓库根目录执行；距离和时间分别换成 `1.0 60`、`5.0 60`、`0 0`）：

```powershell
python tools/make_deploy_yaml.py --mode mpc-exec-centerline --car 05 --cones yellow-left --distance 1.0 --run-time-s 60 --v-exec-max 0.15 --out .verification\car05-cl-relaxed-exec-1m
```

上车前，WSL ROS 门禁要对这一版代码跑通（`.verification/run_ros_check.sh`）。

## 1. SSH、Foxglove、停车、备份

和 [CAR05_V2_START_FIX_RUN_CN.md](CAR05_V2_START_FIX_RUN_CN.md) 第 1、2 步一样：三个 SSH 窗口、启动 units、导入 Foxglove 布局、回零并 Disabled、本机 PS 里定义 `Deploy-StartFix`。

## 2. 部署 1 m 包，Disabled 下做静态检查（代替 shadow）

```powershell
Deploy-StartFix .verification\car05-cl-relaxed-exec-1m
```

车 1：核对哈希，构建、重启：

```bash
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
```

等约 5 秒后检查参数：

```bash
timeout 5 ros2 topic echo /car/policy_fsm_state_string --once
N=/car/ai4r_policy
for p in control.enabled mpc.enabled mpc.shadow mpc.qp_solver vehicle.drive_deadband \
         mpc.v_exec_max_mps mpc.overspeed_soft_margin_mps mpc.overspeed_hard_mps \
         mpc.max_forward_extension_m mpc.reference_dropout_tolerance_s mpc.solver_fail_tolerable \
         mpc.breakaway_timeout_s mpc.max_restarts control.max_distance_m control.max_run_time_s \
         planning.algorithm estimation.road_geometry estimation.left_colour estimation.right_colour; do
  printf "%-40s " "$p"; ros2 param get "$N" "$p"
done
```

| 参数 | 预期 |
| --- | --- |
| state | 2 |
| `control.enabled` / `mpc.enabled` / `mpc.shadow` | False / True / False |
| `mpc.qp_solver` | dense |
| `vehicle.drive_deadband` | 0.3 |
| 限速 / 软超速余量 / 硬停线 | 0.15 / 0.05 / 0.4 |
| 末端外推 / 参考失效容忍 / 求解失败可容忍 | 0.3 / 0.5 / True |
| 起步超时 / 最多重启次数 | 2.5 / 5 |
| 距离 / 时间上限 | 1.0 / 60.0 |
| `planning.algorithm` / `estimation.road_geometry` | road_centerline / ordered_xy |
| 左 / 右颜色 | 1 / 2 |

车 3 观察：

```bash
PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data |
grep --line-buffered -oE '"(branch|phase|reject_reason|speed_mps|e_y_m|e_psi_rad|candidate_steer_action|overspeed_state|forward_extension_m)": ?[^,}]*'
```

**车保持 Disabled**，车 1 请求 state 3，然后看 2.5 s 内的输出：

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
```

- 车 Disabled，油门不会生效，车不会动。2.5 s 后会报 `no_motion_after_breakaway` 并回到 state 2，这是**正常的**。
- 每摆一次车，就请求一次 state 3。

| 摆法 | 通过标准 |
| --- | --- |
| 直道居中，车头沿路 | `branch=solved`，`phase=starting`；`|e_y_m|` ≤ 0.05，`|e_psi_rad|` ≤ 0.08，`|candidate_steer_action|` 小；Foxglove 里绿线从车附近开始、顺着路，橙色预测线方向合理 |
| 左偏 5 cm | `e_y_m` 约 +0.05，候选转向往右（正值是右转） |
| 车头左偏 5° | `e_psi_rad` 约 +0.09，候选转向往右 |

任何一项不对就不上路跑，录 30 s 的包带回来。

## 3. Exec：1 m → 5 m → 不限

车 2 先开录包，确认在录再继续：

```bash
ros2 bag record -o "$HOME/ai4r-car-bags/car05-relaxed-$(date +%Y%m%d-%H%M%S)" --topics \
  /car/mpc_debug /car/mpc_prediction /car/policy_trajectory \
  /car/policy_fsm_state_string /car/traxxas_state \
  /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized \
  /car/wheel_speed_m_per_sec /car/cone_detections /car/imu/data /car/diagnostics
```

每一次的流程：
1. 车摆好，有人守着 Disable。
2. Foxglove 里 Vehicle Request Enable。
3. 车 1 发一次 state 3。
4. 车停下后马上 Disable，读状态和日志：

```bash
timeout 5 ros2 topic echo /car/policy_fsm_state_string --once
dream runtime logs ai4r_policy --tail 100
```

分三阶段：

| 阶段 | 包 | 赛道 | 进入下一阶段的条件 |
| --- | --- | --- | --- |
| A | `-1m` | 直道居中 | 两次都正常报 `Distance limit reached`，交接后车速在 0.10–0.20 之间，转向方向对 |
| B | `-5m` | 直道，然后缓弯 | **连续 3 次**跑满 5 m，没有锁停 |
| C | `-nolimit` | 整条赛道或环道 | 一直有人跟车，随时 Disable |

换包时，重做第 2 步的部署、哈希核对和参数检查（距离和时间上限要对应换成 5.0 / 60.0 或 0.0 / 0.0）。

以下情况立即 Disable，不要等程序自己停：
- 转向往弯外打；
- 车蛇行；
- 车接近边界；
- 车速明显超过 0.25。

## 4. 判定

| 结果 | 说明 | 下一步 |
| --- | --- | --- |
| 车速超过 0.20 时 `overspeed_state` 变成 soft 或 cut，车速回落，没有停车 | 软限按设计起作用 | 记下出现的频率 |
| `overspeed` 锁停 | 切油门 1 s 都没降下来，或者超过 0.40：死区明显偏高，或者 ESC 不响应 0 油门 | 不在现场改，把 bag 带回来 |
| 车速掉到 0.05 以下，反复重新起步 | 死区 0.30 偏低 | 记下重启次数，把 bag 带回来 |
| `invalid_road_*` 停车 | 估计层的问题（修复计划里的 E），MVP 遇到同样情况也会停 | 把 bag 带回来，用 `tools/replay_policy_bag.py` 重放 |
| `forward_extension_m` 经常大于 0.2 | 中心线经常不够长，靠外推在跑 | 记下来；弯里要特别注意 |
| `solver_fail` 锁停 | 求解器连续 0.5 s 失败 | 把 bag 带回来 |

同一个停车原因出现两次，就停止 exec。不在现场改阈值。

## 5. 下载与恢复

和 V2 文档第 6 步一样。包名用 `car05-relaxed-*`，bag 拷到 `E:\26b\AI4R\car-evidence\`。
