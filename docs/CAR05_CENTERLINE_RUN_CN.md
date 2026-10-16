# 5 号车 中心线 + MPC 实车（2026-10-15 起）

本轮第一次在实车上用中心线方案：估计层 `ordered_xy`，规划层 `road_centerline`（main 的有序中心线），后面接 MPC，代替 V2。原因是 V2 在弯前起步时报 `recovery_body_boundary_conflict`，车根本不动（18:32、18:37 两个 bag）。用同样两个 bag 离线重放中心线方案，参考线有效率是 97–99%。

本轮只测直道 1 m。弯前起步只做 shadow 检查，不 exec。

## 和 V2 包的区别

- 规划：`planning.algorithm road_centerline`，同步规划（`planning.lattice.async_enabled false`），没有 V2 的 `plan_hold` 等待。
- 估计：`estimation.road_geometry ordered_xy`；相机数据允许的年龄是 1.5 s，用的是 main 车上的数值。
- 车附近的参考线（`mpc.ordered_lead_in_fit_m 1.0`）：
  - 中心线常常从车前 0.4–0.75 m 才开始。以前 MPC 是把第一段 5 cm 往回延长到车身，现在改为沿二次拟合在起点处的切线往回延长。
  - 18:18 那个 bag 里，原来的做法让一辆居中的车算出 12 cm、0.19 rad 的偏差；现在是 8 cm、0.135 rad。改善了但没有消除，原因是最近那个黄锥本身放偏了 8 cm。
  - 所以这次 shadow 要专门看这一点：静止时转向是否接近 0。
- `estimation.ordered_pairing_backtrack_m 0.03`：和 main 一样。放宽到 0.10 会在 5 号车的真实帧上配错锥，所以没放宽。
- 纵向和 V2 用同一份 5 号车配置：
  - 起步 0.35，起步超时 1.5 s；
  - 死区 0.30，**还没上车验证过**；
  - 限速 0.12，超速余量 0.10（超过 0.22 m/s 停车）；
  - 距离上限 1.0 m。

部署目录：

```text
.verification\car05-cl-shadow     yaml 2e3a116c…  policy_node 72f2ccbd…
.verification\car05-cl-exec       yaml 496f4ffe…  policy_node 72f2ccbd…
```

**这两个包必须从 tag `mpc-car-run-cl-20261015`（`9213576`）生成，不要从分支最新代码生成。**
2026-10-16 起，分支上已经合入 MVP2.1 lattice（默认关闭，见第 6 步），policy_node 哈希变成了 `78f133e1…`。
已核对：从 tag 生成的两个 yaml 和 policy_node 哈希都和上表一致。如果本机已经有上表哈希的包，直接用，不用重新生成。

从 tag 生成（在仓库根目录执行）：

```powershell
git worktree add ..\Baby-Bus-cl mpc-car-run-cl-20261015
python ..\Baby-Bus-cl\tools\make_deploy_yaml.py --mode mpc-shadow-centerline --car 05 --cones yellow-left --distance 1.0 --v-exec-max 0.12 --out .verification\car05-cl-shadow
python ..\Baby-Bus-cl\tools\make_deploy_yaml.py --mode mpc-exec-centerline  --car 05 --cones yellow-left --distance 1.0 --v-exec-max 0.12 --out .verification\car05-cl-exec
git worktree remove ..\Baby-Bus-cl
```

上车前要看到 WSL ROS 门禁对 tag 版本通过（`.verification/run_ros_check.sh`，里面包括每个部署配置都能真正启动节点的测试）。
如果脚本的 `BRANCH` 只接受分支名，就先执行 `git branch cl-20261015 mpc-car-run-cl-20261015`，再传 `BRANCH=cl-20261015`。

## 1. SSH、Foxglove、停车、备份

和 [CAR05_V2_START_FIX_RUN_CN.md](CAR05_V2_START_FIX_RUN_CN.md) 第 1、2 步完全一样：三个 SSH 窗口、启动 units、导入 Foxglove 布局、回零并 Disabled。本机 PS 里定义好 `Deploy-StartFix`。如果当天已经备份过，就不用再备份，`before` 里的备份还在。

## 2. Shadow，车 Disabled

本机 PS：

```powershell
Deploy-StartFix .verification\car05-cl-shadow
```

车 1：核对哈希，必须和上表一致；然后构建、重启：

```bash
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
```

等约 5 秒后检查参数：

```bash
timeout 5 ros2 topic echo /car/policy_fsm_state_string --once
N=/car/ai4r_policy
for p in control.enabled mpc.enabled mpc.shadow mpc.qp_solver id_test.mode vehicle.source \
         vehicle.drive_breakaway vehicle.drive_max vehicle.drive_deadband mpc.v_exec_max_mps \
         mpc.overspeed_margin_mps control.max_distance_m planning.algorithm estimation.road_geometry \
         planning.lattice.async_enabled mpc.ordered_lead_in_fit_m estimation.ordered_pairing_backtrack_m \
         estimation.left_colour estimation.right_colour; do
  printf "%-42s " "$p"; ros2 param get "$N" "$p"
done
```

预期值：

| 参数 | 预期 |
| --- | --- |
| state | 2 |
| `control.enabled` | False |
| `mpc.enabled` | True |
| `mpc.shadow` | True |
| `mpc.qp_solver` | dense |
| `id_test.mode` | off |
| `vehicle.source` | 包含 `db030` |
| `vehicle.drive_breakaway` / `drive_max` / `drive_deadband` | 0.35 / 0.35 / 0.3 |
| 限速 / 超速余量 | 0.12 / 0.1 |
| `control.max_distance_m` | 1.0 |
| `planning.algorithm` | road_centerline |
| `estimation.road_geometry` | ordered_xy |
| `planning.lattice.async_enabled` | False |
| `mpc.ordered_lead_in_fit_m` | 1.0 |
| `estimation.ordered_pairing_backtrack_m` | 0.03 |
| 左 / 右颜色 | 1 / 2 |

车 3 观察：

```bash
PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data |
grep --line-buffered -oE '"(branch|phase|reject_reason|speed_mps|e_y_m|e_psi_rad|candidate_steer_action)": ?[^,}]*'
```

保持 Disabled，车 1 请求 state 3（`ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'`），然后逐项检查：

| 摆法 | 通过标准 |
| --- | --- |
| 直道居中，车头沿路 | `branch=shadow`；`|e_y_m|` ≤ 0.05，`|e_psi_rad|` ≤ 0.08；`|candidate_steer_action|` ≤ 0.05；Foxglove 里绿色参考线从车附近开始、顺着路，橙色预测线方向合理 |
| 左偏 5 cm（每次摆车前先回 state 2） | `e_y_m` 约 +0.05，候选转向往右修正（转向符号：正值是右转） |
| 车头左偏 5° | `e_psi_rad` 约 +0.09，候选转向往右修正 |
| 弯前（18:37 的位置） | 有参考线，没有 `recovery_body_boundary_conflict`；参考线跟着弯走 |

- 直道居中时如果 `e_y` 超过 5 cm、但锥桶看起来是正的：先看最近那一对锥是不是有一个放偏了，把它摆正再看。还是不对就不进 exec，录 30 s 包带回来。
- 检查完回 state 2，确认仍是 Disabled。

## 3. Exec，直道 1 m

本机 PS：

```powershell
Deploy-StartFix .verification\car05-cl-exec
```

车辆 Disabled、state 2 下，重做第 2 步的哈希核对、构建、参数检查。唯一不同的参数是 `mpc.shadow` 应为 False。

车 2 先开录包，确认在录再继续：

```bash
ros2 bag record -o "$HOME/ai4r-car-bags/car05-cl-exec-$(date +%Y%m%d-%H%M%S)" --topics \
  /car/mpc_debug /car/mpc_prediction /car/policy_trajectory \
  /car/policy_fsm_state_string /car/traxxas_state \
  /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized \
  /car/wheel_speed_m_per_sec /car/cone_detections /car/imu/data /car/diagnostics
```

准备：
- 车摆在直道中间，车头沿路；前方至少 2 m 平直，并留出滑行余量；
- 有人守着 Disable；
- 读到 trim 为 0、静止轮速接近 0 之后再开始。

步骤：
1. Foxglove 里 Vehicle Request Enable，确认状态是 Enabled。
2. 车 1 发一次 state 3。
3. `phase=starting` 时油门 0.35；轮速到 0.05 m/s 后进入 `tracking`。
4. 停车后马上 Disable，读状态和日志，再按 Ctrl+C 停止录包。

如果转向朝外偏、车蛇行、车速明显超过 0.22 m/s，或者车接近边界，立即 Disable，不要等程序自己停。

```bash
timeout 5 ros2 topic echo /car/policy_fsm_state_string --once
timeout 5 ros2 topic echo /car/traxxas_state --once
dream runtime logs ai4r_policy --tail 100
```

## 4. 判定

| 结果 | 说明 | 下一步 |
| --- | --- | --- |
| 交接后车速稳在 0.10–0.15 m/s，转向方向对，最后报 Distance limit reached | 通过 | 同样摆车再跑一次；两次都过了，再考虑弯前起步 1 m（会在弯前停） |
| 冲过 0.22 m/s，`overspeed` 停车 | 死区 0.30 还是高了 | 不在现场改，把 bag 带回来 |
| 车速掉到 0.05 以下，或出现 `no_motion` | 死区 0.30 低了 | 不在现场改，把 bag 带回来 |
| `reference_too_short`、`invalid_road_*` 等提前停车 | 估计或规划的问题 | 把 bag 带回来，用 `tools/replay_policy_bag.py` 重放 |
| 起步时转向明显往一侧打 | 车附近参考线还是歪的（见上面"和 V2 包的区别"的第 3 条） | 把 bag 带回来 |

同一个停车原因出现两次，就停止 exec。不在现场改阈值。

## 5. 下载与恢复

和 V2 文档第 6 步一样。包名换成 `car05-cl-*`，日志存成 `car05-cl-policy.log`。bag 拷到 `E:\26b\AI4R\car-evidence\`。

## 6. 可选：MVP2.1 lattice shadow，车 Disabled，推车采数据

前 5 步全部做完、bag 已经拷下来以后才做。时间不够就跳过，不影响本轮结论。**只做 shadow，不 exec。**

这一步看的是：lattice 选出的路径交给 MPC 以后，MPC 的转向是否合理。离线回放里，用 lattice 参考时 MPC 转向和 PID 反号的拍数，是中心线 MPC 的 2–4 倍，见[回放结果](../offline/mpc_gym/results/replay_car15_lattice21.md)。

这个包从分支最新代码生成（`e6c13a6` 及以后）：

```text
.verification\car05-lattice21-shadow   yaml 3048ea1c…  policy_node 78f133e1…
```

```powershell
python tools/make_deploy_yaml.py --mode mpc-shadow-lattice21 --car 05 --cones yellow-left --distance 1.0 --v-exec-max 0.12 --out .verification\car05-lattice21-shadow
Deploy-StartFix .verification\car05-lattice21-shadow
```

车 1：核对哈希、构建、重启，和第 2 步一样。第 2 步的参数检查完后，再查下面几项：

```bash
for p in shadow.enabled shadow.rate_hz planner_control.enabled planner_control.require_valid_baseline mpc.shadow; do
  printf "%-42s " "$p"; ros2 param get "$N" "$p"
done
```

预期依次为 True、1.0、True、True、True。启动日志里不应出现 `Shadow worker failed`。

车 2 录包：

```bash
ros2 bag record -o "$HOME/ai4r-car-bags/car05-lattice21-shadow-$(date +%Y%m%d-%H%M%S)" --topics \
  /car/mpc_debug /car/mpc_prediction /car/policy_trajectory /car/planner_execution \
  /car/shadow_trajectory /car/shadow_checked_prefix /car/shadow_reference /car/shadow_planning \
  /car/policy_fsm_state_string /car/wheel_speed_m_per_sec /car/cone_detections /car/imu/data /tf /tf_static
```

车 3 观察：

```bash
PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data |
grep --line-buffered -oE '"(branch|reference_selection|reference_selection_reason|e_y_m|candidate_steer_action)": ?[^,}]*'
```

保持 Disabled，请求 state 3，然后：

1. 车静止放在直道中间，等 10 s。
2. 慢慢推车，沿直道走 1–2 m。
3. 回 state 2，把车摆到 18:37 那个弯前位置，再请求 state 3。静止 10 s 后，推车慢慢入弯。

每一段结束后都回 state 2。Foxglove 里三条线的含义：

- 绿线：MPC 实际用的参考；
- 蓝线：lattice 选中的路径，坐标是规划那一刻的，推车时和绿线不重合是正常的；
- 橙线：MPC 的预测轨迹。

要记下的：

| 看什么 | 记录 |
| --- | --- |
| `reference_selection` | planner 和 baseline 各占多少；退回的主要原因（`known_boundary_collision`、`baseline_invalid: ...`、`preview_coverage_exhausted`） |
| 弯前位置 | `/car/shadow_planning` 的 `status` 是否为 VALID；选中的候选类型 `selected_family`；有没有 `ALL_COLLIDE` |
| 转向方向 | `reference_selection=planner` 时，`candidate_steer_action` 的方向是否合理，有没有明显往弯外打 |
| 切换 | planner 和 baseline 来回切换时，转向有没有跳变 |
| 日志 | `dream runtime logs ai4r_policy --tail 300` 里的 `Shadow planning:` 和 `Reference selection:` 两种行；看 `elapsed_s` 和 `round_trip_s`，这是车上的规划耗时 |

做完回 state 2，确认仍是 Disabled。bag 和日志按第 5 步的方式拷回，目录名用 `car05-lattice21-shadow-*`。
之后要恢复中心线，用 `Deploy-StartFix .verification\car05-cl-shadow`（或 `car05-cl-exec`）换回去，并重新核对哈希。
