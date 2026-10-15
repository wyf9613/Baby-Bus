# 5 号车 MPC 实车清单（2026-10-15）：shadow 对比 V2 与中心线 → 直道 → 缓弯

分支 `mpc-car-run`。5 号车 IP **10.43.254.18**（main 的 `AGGRESSIVE_BEND_BASELINE_CN.md`；以车身标签为准）。
通用操作（窗口约定、Foxglove、IMU 故障处理）见 [G2 清单](G2_SHADOW_CHECKLIST_CN.md)，备份和恢复的写法与
[周四清单](THURSDAY_CAR_RUN_CN.md) 相同。本文只写 5 号车今天的步骤，以及每一步的**通过条件**和**停止条件**。

## 总原则
- **5 号车只有起步油门是自己测的**（17:06 固定 0.35 实测能动，见 [起步修复复测](CAR05_V2_START_FIX_RUN_CN.md)）。其余油门模型借 .27（死区 0.289），转向借 .20（满舵 18°，`steering_gain_rad -0.3142`），
  见 `config/ai4r_policy_mpc_car05.yaml`。执行速度上限 0.12 m/s，超过 0.20 m/s 锁停。第 1 步的短弧标定能把转向换成
  5 号车自己的数，**有时间就做**：5 号车的转向区间是 ±1.0，和 .20 量的时候不一定一样。
- **两个版本都先 shadow，按数据选一个去执行**，不要直接认定 V2：
  - V2（`lattice_v2`）：Gym 直道和缓弯 10/10，但 Planning 组反馈它在实车上效果不好；时序参数是按 .27 调的。
  - 中心线（`road_centerline` + `ordered_xy`）：**今天 main 的 PID 在 5 号车上用的就是这套估计和规划**。
    Gym 里直道 2–7/10、缓弯 0–6/10，原因是 MPC 参考线偶尔断在车前 0.1 m（`reference_too_short`），修复在计划里，还没做。
- **今天不跑直角弯和 U 弯。** MPC 在急弯会因为转向打满而提速，最后超速锁停（Gym 0/10，修复在计划里）。
- 执行时始终有一个人站在车旁守着 **Vehicle Request Disable**。每次运行都录包，记下场景、起点姿态、结果和停车原因。
- 车上原有策略文件**全天只备份一次**（第 0.5 步）。

## 0. 出发前（本机）

```powershell
cd E:\26b\AI4R\Baby-Bus
git checkout mpc-car-run; git pull; git log --oneline -1
$cones = 'yellow-left'   # 左黄右蓝；第 0.7 步现场确认，不一致就改这里，再重新生成
python tools/make_deploy_yaml.py --mode steer-arc --out .verification\car05-arc
python tools/make_deploy_yaml.py --mode mpc-shadow --car 05 --cones $cones --out .verification\car05-v2-shadow
python tools/make_deploy_yaml.py --mode mpc-shadow-centerline --car 05 --cones $cones --out .verification\car05-cl-shadow
python tools/make_deploy_yaml.py --mode mpc-exec --car 05 --cones $cones --distance 3 --out .verification\car05-v2-exec
python tools/make_deploy_yaml.py --mode mpc-exec --car 05 --cones $cones --distance 5 --out .verification\car05-v2-curve
python tools/make_deploy_yaml.py --mode mpc-exec-centerline --car 05 --cones $cones --distance 3 --out .verification\car05-cl-exec
python tools/make_deploy_yaml.py --mode mpc-exec-centerline --car 05 --cones $cones --distance 5 --out .verification\car05-cl-curve
```

每条命令都要打印 `cones yellow-left (left_colour 1, right_colour 2 ...)` 和
`vehicle car05_20261015_cross_car_substitution_27drive_20steering_UNVERIFIED valid True`。

Foxglove 布局：导入 `tools/foxglove_road_visualization/layout.json`。“Cone locations” 图里有锥桶（红点）、
控制器采用的参考线（绿）、MPC 预测（橙）。

## 0.5 备份车上原有文件（全天只做一次）[本机 PS]

同一个 PowerShell 窗口做完全天；换了窗口就重新设置这 4 个变量，`$taskEvidenceDir` 填**同一个**目录。

```powershell
$taskCarIp = '10.43.254.18'
$taskWs = '~/ai4r_student_workspace/src/ai4r_policy'
$taskStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$taskEvidenceDir = "E:\26b\AI4R\car-evidence\car05-mpc-$taskStamp"
New-Item -ItemType Directory -Force "$taskEvidenceDir\original" | Out-Null
scp "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py" "$taskEvidenceDir\original\policy_node.py"
scp "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml" "$taskEvidenceDir\original\ai4r_policy.yaml"
ssh "ai4r@$taskCarIp" "mkdir -p ~/ai4r-original-$taskStamp && cp $taskWs/scripts/policy_node.py $taskWs/config/ai4r_policy.yaml ~/ai4r-original-$taskStamp/ && sha256sum ~/ai4r-original-$taskStamp/*"
Get-FileHash "$taskEvidenceDir\original\*" -Algorithm SHA256 | Format-Table Hash, Path -AutoSize | Out-String -Width 300 | Tee-Object "$taskEvidenceDir\original\sha256.txt"

function Deploy-Car($dir) {
  scp "$dir\policy_node.py" "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py"
  scp "$dir\ai4r_policy.yaml" "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml"
  Copy-Item $dir "$taskEvidenceDir\$(Split-Path $dir -Leaf)" -Recurse -Force
  Get-FileHash "$dir\policy_node.py", "$dir\ai4r_policy.yaml" -Algorithm SHA256 | Format-Table Hash, Path -AutoSize
}
ssh "ai4r@$taskCarIp" "mkdir -p ~/ai4r_mpc_check/$taskStamp/offline ~/ai4r-car-bags"
scp -r tools "ai4r@${taskCarIp}:~/ai4r_mpc_check/$taskStamp/"
scp -r offline/vehicle_identification "ai4r@${taskCarIp}:~/ai4r_mpc_check/$taskStamp/offline/"
```

检查：两个原文件都拷回来了、不是空的，本机和车上的 SHA-256 一致。
**注意**：车上现在跑的是 main 的 PID 版本（今天能跑的那个），收尾时要恢复成它。

每次 `Deploy-Car` 之后，在车上（车辆 Disabled，策略 state 2）：

```bash
# [车 1]
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml   # 和 Deploy-Car 打印的一致
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
sleep 5; timeout 3 ros2 topic echo /car/policy_fsm_state_string --once        # 起来了，state 2
```

**节点起不来**（`state_string` 读不到，或 `dream runtime logs ai4r_policy --tail 50` 有 `ValueError`）：把那行错误发回来，
不要改车上的文件。

## 0.7 现场核对 [车 1]

服务按 G2 第 4 步启动（`foxglove_bridge`、`traxxas_vehicle_interface`、`oakd_cone_detector`、`bno08x_imu_interface`），
先看 IMU 诊断和传感器频率。然后：

1. **实时转向 trim 必须是 0**（10-14 的 15 号车启动参数是 0、实时 trim 却是 0.5）：
   ```bash
   timeout 5 ros2 topic echo /car/steering_trim_current_normalized --once   # 期望 data: 0.0
   ```
   不是 0：`ros2 topic pub --once /car/steering_trim_set_normalized std_msgs/msg/Float32 '{data: 0.0}'`，再读一次。
   **每次 Enable 前都读一次。**
2. **转向区间**（main 给 5 号车设的是 ±1.0）：
   ```bash
   ros2 param get /car/traxxas_vehicle_interface steering_min_normalized
   ros2 param get /car/traxxas_vehicle_interface steering_max_normalized
   ```
   记下来。全天不要改。
3. **锥桶颜色**：车居中摆在直道起点、车头沿路：
   ```bash
   python3 ~/ai4r_mpc_check/<stamp>/tools/cone_layout_check.py   # 最后一行应是 --cones yellow-left
   ```
   不是 `yellow-left`：本机改 `$cones`，重新跑第 0 步的生成命令。

## 1.（推荐，约 30 分钟）转向短弧标定

把借来的 .20 转向换成 5 号车自己的。不做就直接跳到第 2 步，用借来的值。

1. 量轴距 `L`（前后轮轴中心距离，米）。
2. `Deploy-Car .verification\car05-arc`，车上构建、重启。检查：
   ```bash
   N=/car/ai4r_policy
   for p in id_test.mode id_test.steering_values id_test.drive_values control.enabled mpc.enabled; do
     printf "%-28s " $p; ros2 param get $N $p; done
   # 期望：arc、[0.0, 0.25, -0.25, 0.4, -0.4, 0.0]、全是 0.32、False、False
   ```
3. 另开一个车上窗口录制：
   ```bash
   python3 ~/ai4r_mpc_check/<stamp>/offline/vehicle_identification/record_vehicle_test.py \
     --namespace car --car-id 05 --notes "arc L=<L> steer_range=<0.7 第 2 项> trim=0" \
     --output ~/ai4r-car-bags/arc-$(date +%H%M%S).jsonl
   ```
4. 车放空地中央 → 读 trim → **Enable** → state 3。序列 16 s，结束后自动回 state 2 → **Disable** → 录制窗口 `Ctrl+C`。
5. 分析：
   ```bash
   python3 ~/ai4r_mpc_check/<stamp>/offline/vehicle_identification/vehicle_test_tools.py steering-arcs \
     --log ~/ai4r-car-bags/arc-*.jsonl --wheelbase <L> --output /tmp/steering-arcs.json
   ```
   通过：`max_abs_residual_rad` < 0.03，4 个转弯段都没有 `skipped`，增益为负。段落被跳过（车速不到 0.08 m/s）就是
   5 号车的死区比 0.32 高：本机 `python tools/make_deploy_yaml.py --mode steer-arc --arc-drive 0.34 --out .verification\car05-arc`，
   重新部署再跑一次。**这也是 0.289 的油门死区对 5 号车不适用的早期信号**，记下来。
6. 本机改 `config/ai4r_policy_mpc_car05.yaml` 的 `vehicle` 段（增益 `k`、零偏 `b` 用工具输出的 `vehicle_yaml`）：
   ```yaml
   wheelbase_m: <L>
   steering_gain_rad: <k>
   steering_offset_rad: <b>
   steering_min_rad: <b - |k|>    # 5 号车请求范围是 ±1.0
   steering_max_rad: <b + |k|>
   source: "car05_20261015_arcs_27drive_UNVERIFIED"   # 油门仍是借 .27 的
   ```
   然后重新跑第 0 步里所有 `--car 05` 的生成命令（arc 那条不用），之后再部署。

## 2. Shadow 对比：V2 和中心线各推一遍（不 Enable）

两个版本按同样的顺序做，**先 V2，后中心线**：

1. `Deploy-Car .verification\car05-v2-shadow`（第二轮换成 `car05-cl-shadow`），车上构建、重启。
2. 参数检查：
   ```bash
   N=/car/ai4r_policy
   for p in control.enabled mpc.enabled mpc.shadow mpc.qp_solver vehicle.source vehicle.valid mpc.v_exec_max_mps \
            planning.algorithm estimation.road_geometry planning.max_source_age_s planning.lattice.async_enabled \
            estimation.left_colour estimation.right_colour; do
     printf "%-34s " $p; ros2 param get $N $p; done
   ```

   | 参数 | V2 shadow | 中心线 shadow |
   | --- | --- | --- |
   | `control.enabled` / `mpc.enabled` / `mpc.shadow` | False / True / True | 同左 |
   | `mpc.qp_solver` | `dense` | `dense` |
   | `vehicle.source` / `vehicle.valid` | `car05_20261015_...` / True | 同左 |
   | `mpc.v_exec_max_mps` | 0.12 | 0.12 |
   | `planning.algorithm` | `lattice_v2` | `road_centerline` |
   | `estimation.road_geometry` | `y_of_x` | `ordered_xy` |
   | `planning.max_source_age_s` | 0.4 | 1.5 |
   | `planning.lattice.async_enabled` | True | False |
   | `estimation.left_colour` / `right_colour` | 1 / 2 | 1 / 2 |

   有任何一项不符就停下来查部署文件。
3. 进 state 3，另开窗口实时看：
   ```bash
   # [车 1]
   ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
   # [车 3]
   PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data | grep --line-buffered -oE '"(branch|reject_reason|e_y_m|e_psi_rad|candidate_steer_action|candidate_drive|step_s)": [^,]*'
   ```
4. 三个推车场景，每个录一个包，**推 2 m 以上**（走过约 0.9 m 的相机盲区），约 0.2 m/s：

   | 场景 | 摆放 | 预期 |
   | --- | --- | --- |
   | A | 居中，车头沿路 | `e_y`、`e_psi` 接近 0 |
   | B | 左偏 5 cm | `e_y` 约 +0.05，候选转向修正向右 |
   | D | 车头左偏 5° | `e_psi` 约 +0.087，候选转向修正向右 |

   ```bash
   # [车 2]，<场景> 换成 v2-A、cl-B 之类
   cd ~/ai4r-car-bags && ros2 bag record -o car05-$(date +%H%M%S)-<场景> --topics \
     /car/mpc_debug /car/mpc_prediction /car/policy_trajectory /car/debug1 /car/debug2 \
     /car/policy_fsm_state_string /car/drive_and_steer_set_point_normalized /car/wheel_speed_m_per_sec \
     /car/cone_detections /car/imu/data /car/traxxas_state /car/diagnostics
   ```
   开始录 → 静止 2 s → 推 2 m → 静止 2 s → `Ctrl+C`。确认还在 state 3；掉回 state 2 就记下原因再请求 state 3。
5. 汇总：
   ```bash
   python3 ~/ai4r_mpc_check/<stamp>/tools/mpc_shadow_summary.py ~/ai4r-car-bags/car05-*-v2-*   # 第二轮换成 *-cl-*
   ```

**每个版本的通过条件**：
- 推车阶段 `shadow` 分支 ≥ 80%；
- 没有 `stop` 分支，静止起步阶段也不能有；
- 候选转向的修正方向 ≥ 85% 正确；
- `step_p95` < 30 ms；
- Foxglove 里橙色预测线贴着绿色参考线，没有往反方向甩。

**选哪个去执行**：只有一个通过就用它；两个都通过就用**中心线**（实车验证过的就是这套估计和规划）；
两个都不通过就**今天不执行**，把包和汇总发回来。中心线常见的拒绝原因是 `reference_too_short`，
V2 常见的是 `insufficient_lattice_coverage`、`recovery_body_boundary_conflict`、时效类拒绝。

## 3. 直道执行（MPC 动车）

`Deploy-Car .verification\car05-cl-exec`（选了 V2 就用 `car05-v2-exec`），车上构建、重启。参数检查同第 2 步，
另外 `mpc.shadow` 为 False、`control.max_distance_m` 为 3.0。

1. 车居中摆在直道起点 → 读 trim（必须 0.0）→ 开始录包（同第 2 步，场景名 `exec-1`）→ **Enable** → 看到 Enabled → state 3。
2. 预期：先固定给 0.35 的油门起步（`breakaway_ramp_per_s: 0`），轮速到 0.05 m/s 后交给 MPC，以约 0.12 m/s 前进，走满 3 m 自动停下，回 state 2。
3. 结束 → **Disable** → 停止录包。
4. 第一次成功后，再做一次左偏 5 cm 起步。

**立即 Disable**：
- 车往偏离的方向打（转向符号错）；
- 转向明显过猛或过弱，车开始蛇行（借来的转向增益不对）；
- 车速明显超过 0.2 m/s；
- 压到锥桶。

**给了油门车不动**：1.5 s（`mpc.breakaway_timeout_s`）后策略会以 `no_motion_after_breakaway` 锁停，这是安全的。
**起步后又停下**（交给 MPC 后失速）：MPC 跟踪时的油门下限是 .27 的死区 0.289，5 号车起步要请求 0.35，
很可能维持不了。两种情况都不要硬调，把包发回来。

锁停（掉回 state 2）本身是安全的：记下 `state_string` 的原因，**同一个原因连续两次就不要再跑**。
通过：两次都走满 3 m，没有锁停，横向偏差目测不超过 10 cm。

## 4. 缓弯执行（直道两次都成功后）

`Deploy-Car .verification\car05-cl-curve`（或 `car05-v2-curve`），5 m 距离上限。弯道场地是另外摆的，先按第 0.7 步第 3 项
再查一次颜色。

1. 车放在弯前至少 2 m 的直道上，居中 → 读 trim → 录包 → Enable → state 3。
2. 预期：先直行，入弯开始转向，沿弯走满 5 m 或走出弯道。
3. 通过条件和停止条件同第 3 步。入弯时锁停，记下原因：`reference_too_short` 是参考线断链（计划第 1 步），
   `overspeed` 是急弯提速（计划第 2 步），`invalid_road_*` 是估计层（计划第 3 步）。

## 5. 收尾：停车、取证据、恢复原文件

```bash
# [车 1]
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
timeout 5 ros2 topic echo /car/traxxas_state --once      # 必须是 Disabled
dream runtime logs ai4r_policy --tail 300 > ~/ai4r-car-bags/car05-policy.log 2>&1
```
```powershell
# [本机 PS]
scp -r "ai4r@${taskCarIp}:~/ai4r-car-bags/*" $taskEvidenceDir
scp "ai4r@${taskCarIp}:/tmp/steering-arcs.json" $taskEvidenceDir   # 做了第 1 步才有
scp "$taskEvidenceDir\original\policy_node.py" "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py"
scp "$taskEvidenceDir\original\ai4r_policy.yaml" "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml"
Get-Content "$taskEvidenceDir\original\sha256.txt"
```
```bash
# [车 1]
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml   # 必须和 original\sha256.txt 一致
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
timeout 3 ros2 topic echo /car/policy_fsm_state_string --once   # 原来的 PID 策略，state 2
```
SHA 对不上就从车上的 `~/ai4r-original-<stamp>/` 复制回去再核对。交接前告诉下一组：车已 Disabled，策略已恢复。

**要发回来的**：`$taskEvidenceDir` 整个目录（包、汇总输出、`car05-policy.log`、第 1 步的 `steering-arcs.json`）和现场记录表。
