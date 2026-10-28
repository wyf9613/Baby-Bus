# 5 号车 V2 起步修复复测（2026-10-15）

本轮验证 V2 + MPC 起步及起步→跟踪交接。固定油门辨识已经证明，在该轮现场条件下请求 0.35 / 接口报告应用约 0.324 能在约 0.79 秒内产生轮速。它没有验证 V2 跟踪、转向参数或借来的纵向模型。

## 修改内容与不变的门禁

- 5 号车 `vehicle.drive_breakaway=0.35`、`vehicle.drive_max=0.35`、`mpc.breakaway_ramp_per_s=0.0`。
- 已经成功求解并进入起步后，短时 `plan_hold` 保持上一条已应用的起步请求，不被旧 QP 跟踪序列中的较小油门拉低；等待期间不增加油门。
- 第一次成功求解之前保持零输出；轮速达到 0.05 m/s 后交给原来的跟踪/plan_hold 行为。
- 起步超时仍 1.5 s；规划失效容忍窗仍 0.3 s；等待阶段也必须执行超速和起步超时停车。
- 速度上限配置 0.12 m/s、超速余量 0.08 m/s，首次距离上限 0.5 m。速度目标不是瞬时物理限速，距离是编码器累计上限，不保证精确停车位置。
- 明确停车、参考失效到期、反馈丢失、超长时序间隔、求解失败仍回零锁停，需要显式请求才能重启。
- `id_test.mode` 必须为 off。这轮使用 V2，不使用 `car05-drive-minimal` 固定油门辨识配置。

本轮部署目录：

```text
.verification\car05-v2-startfix-shadow
.verification\car05-v2-startfix-exec
```

旧 `car05-v2-first`、`car05-v2-shadow` 文件夹不会自动更新，不能用于本次验证。

## 0. 本机软件检查

执行门禁与已验证结果记录在部署包的 verification.json；物理复测仍未进行。生成命令：

```powershell
python tools/make_deploy_yaml.py --mode mpc-shadow --car 05 --cones yellow-left --distance 0.5 --out .verification\car05-v2-startfix-shadow
python tools/make_deploy_yaml.py --mode mpc-exec --car 05 --cones yellow-left --distance 0.5 --v-exec-max 0.12 --out .verification\car05-v2-startfix-exec
```

需要 Python + PyYAML；已准备的文件可直接使用。代码或 YAML 之后再改，必须重新生成并重新完成对应检查。

## 1. SSH、Foxglove、停车

三窗口各执行 `ssh ai4r@10.43.254.18`，以车身 IP 标签为准。使用带 ros2_jazzy 环境的交互 SSH；不要从未加载 ROS 环境的远程单行命令执行 ROS 脚本。

车 1：

```bash
dream runtime status
dream runtime start foxglove_bridge
dream runtime start traxxas_vehicle_interface
dream runtime start oakd_cone_detector
dream runtime start bno08x_imu_interface
```

已运行的 units 无需重启。Foxglove WebSocket：`ws://10.43.254.18:1234`，导入本机 `tools/foxglove_road_visualization/layout_v2.0.json`。

原策略正在运行时，车 1 回零，然后请求车辆 Disabled：

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
ros2 topic pub --once /car/request std_msgs/msg/UInt8 '{data: 0}'
timeout 5 ros2 topic echo /car/traxxas_state --once
```

实际看到 Disabled，确认车已停稳，再部署。不要因为 policy restart 就认为车辆已停用。

## 2. 本机 PowerShell：本轮备份只做一次

```powershell
cd E:\26b\AI4R\Baby-Bus
$taskStartIp = '10.43.254.18'
$taskStartWs = '/home/ai4r/ai4r_student_workspace/src/ai4r_policy'
$taskStartStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$taskStartEvidence = "E:\26b\AI4R\car-evidence\car05-v2-startfix-$taskStartStamp"
New-Item -ItemType Directory -Force "$taskStartEvidence\before" | Out-Null
scp "ai4r@${taskStartIp}:$taskStartWs/scripts/policy_node.py" "$taskStartEvidence\before\policy_node.py"
if ($LASTEXITCODE -ne 0) { throw 'policy 备份失败' }
scp "ai4r@${taskStartIp}:$taskStartWs/config/ai4r_policy.yaml" "$taskStartEvidence\before\ai4r_policy.yaml"
if ($LASTEXITCODE -ne 0) { throw '配置备份失败' }
Get-FileHash "$taskStartEvidence\before\policy_node.py", "$taskStartEvidence\before\ai4r_policy.yaml" -Algorithm SHA256 |
  Format-Table Hash,Path -AutoSize | Out-String -Width 300 | Tee-Object "$taskStartEvidence\before\sha256.txt"

function Deploy-StartFix($dir) {
  scp "$dir\policy_node.py" "ai4r@${taskStartIp}:$taskStartWs/scripts/policy_node.py"
  if ($LASTEXITCODE -ne 0) { throw 'policy 上传失败' }
  scp "$dir\ai4r_policy.yaml" "ai4r@${taskStartIp}:$taskStartWs/config/ai4r_policy.yaml"
  if ($LASTEXITCODE -ne 0) { throw '配置上传失败' }
  Copy-Item $dir "$taskStartEvidence\$(Split-Path $dir -Leaf)" -Recurse -Force
  Get-FileHash "$dir\policy_node.py", "$dir\ai4r_policy.yaml" -Algorithm SHA256 | Format-Table Hash,Path -AutoSize
}
```

备份非空并记录好本轮前是什么版本。不要覆盖当天最早的 PID 备份。保留本机 PS 窗口以保留变量和恢复路径。

## 3. 先做 Disabled shadow，检查路径和方向

本机 PS：

```powershell
Deploy-StartFix .verification\car05-v2-startfix-shadow
```

每次部署之后，车 1 核对哈希，必须和本机本轮输出逐文件一致：

```bash
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml
```

一致后构建并加载：

```bash
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
```

等约 5 秒后：

```bash
timeout 5 ros2 topic echo /car/policy_fsm_state_string --once
N=/car/ai4r_policy
for p in control.enabled mpc.enabled mpc.shadow mpc.qp_solver id_test.mode vehicle.source \
         vehicle.drive_breakaway vehicle.drive_max mpc.breakaway_ramp_per_s \
         mpc.breakaway_timeout_s mpc.moving_speed_mps mpc.v_exec_max_mps \
         mpc.overspeed_margin_mps control.max_distance_m planning.algorithm \
         estimation.road_geometry planning.lattice.async_enabled estimation.left_colour estimation.right_colour; do
  printf "%-35s " "$p"; ros2 param get "$N" "$p"
done
ros2 topic info /car/drive_and_steer_set_point_normalized
```

预期 state 2；PID False；MPC True；shadow True；dense；id_test off；source 包含 `start035_27model_20steering_UNVERIFIED`；起步/上限 0.35/0.35；ramp 0；timeout 1.5；moving 0.05；限速/余量 0.12/0.08；距离 0.5；V2/y_of_x/async True；颜色 1/2。动作 topic 只有一个发布者。

传感器频率、IMU 诊断和 trim 按 CAR05_MPC_RUN_CN.md 第 0.7 步检查。实时 trim 必须为 0；车停稳后等待旧轮速回到接近 0。转向区间保留本车现有 ±1.0。

车摆在两侧锥桶中间，车头沿直道。保持 Disabled，请求 state 3：

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
```

车 3 观察（此正则兼容紧凑 JSON，不依赖冒号后有空格）：

```bash
PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data |
grep --line-buffered -oE '"(branch|phase|reject_reason|speed_mps|e_y_m|e_psi_rad|applied_drive|candidate_steer_action)": ?[^,}]*'
```

先静止观察 10 秒：应该能产生有效 shadow 求解，实际动作始终为零，绿色参考线和橙色预测线方向合理。可静止左偏 5 cm / 车头左偏 5°，检查候选转向向右修正；每次摆车先回 state 2。已有条件时按原推车流程补测三场景。静止 shadow 不验证这次 exec 起步保持逻辑；若没有有效参考、存在 stop 或方向不对，先查问题，不进 exec。

结束回 state 2，确认仍 Disabled。

## 4. 换 exec，仅测 0.5 m

本机 PS：

```powershell
Deploy-StartFix .verification\car05-v2-startfix-exec
```

车辆 Disabled、state 2 下，再执行第 3 步的哈希核对、构建、重启、参数检查。唯一 shadow 参数改为 False；仍然是 V2，id_test off，起步 0.35，上限 0.35，限速 0.12，距离 0.5。

车 2 先录包，等待 Recording 和实际 topic 订阅成功：

```bash
ros2 bag record -o "$HOME/ai4r-car-bags/car05-v2-startfix-exec-$(date +%Y%m%d-%H%M%S)" --topics \
  /car/mpc_debug /car/mpc_prediction /car/policy_trajectory \
  /car/policy_fsm_state_string /car/traxxas_state \
  /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized \
  /car/wheel_speed_m_per_sec /car/cone_detections /car/imu/data /car/diagnostics
```

车 3 保持上面的 mpc_debug 观察，或在 Foxglove 画请求油门、实际应用油门和轮速。不要把 mpc_debug.applied_drive 当成车辆接口独立上报的 applied topic。

车居中、车头沿直道，前方有停车滑行余量，有人守 Disable。读 trim=0 和静止轮速接近 0 后：

1. Foxglove Vehicle Request Enable，确认 Enabled。
2. 车 1 发一次 state 3。
3. `phase=starting` 的已接受求解输出应为 0.35；短时 `branch=plan_hold` 且仍 starting 时，MPC 输出也保持已有起步值，不向上增长。接口实际油门本次可能约 0.324，不能硬要求遥测等于请求。
4. 轮速达到 0.05 后进入 tracking，油门由 MPC 跟踪求解决定；此时油门变小是预期，不要求全程 0.35。
5. 预期到达累计 0.5 m 回 state 2。道路不足、时效、超速等也可能提前停车；不能依赖锥桶末端自动停车。
6. 结束立即 Disable，现场确认停稳，读完整状态和日志，再 Ctrl+C 停录包。

如果转向朝偏离方向、蛇行、明显窜动、速度明显超过 0.20 m/s 或接近边界，立即 Disable，不等程序停车。遇到同一锁停原因两次停止 exec；给油不动不在现场加大上限。

```bash
timeout 5 ros2 topic echo /car/policy_fsm_state_string --once
timeout 5 ros2 topic echo /car/traxxas_state --once
dream runtime logs ai4r_policy --tail 100
```

框架的 `Policy calculation or output was invalid` 是通用文字；具体看日志中的 MPCStop 与 mpc_debug.reject_reason。

## 5. 判定与下一轮

- 本轮起步成功：轮速越过 0.05，phase 从 starting 到 tracking；起步无 no_motion_after_breakaway。
- 本轮 0.5 m 跟踪成功：保持方向正确、无明显蛇行/越界，最终 Distance limit reached，车辆 Disable 后已停稳。
- 若起步成功但 tracking 后失速/反复重启，这是下一层纵向模型/交接问题，不取消停车保护，不直接加到 3 m。
- 若 overspeed，记录峰值及 starting→tracking 时序，不现场提高超速阈值。
- 一次成功后同样居中再做一次 0.5 m。两次都通过且场地足够，再生成 1 m 配置；暂不跑缓弯、直角或 U 弯。

## 6. 下载、恢复与交接

确认 Disabled、state 2，保存日志：

```bash
dream runtime logs ai4r_policy --tail 300 > ~/ai4r-car-bags/car05-v2-startfix-policy.log 2>&1
```

本机 PS：

```powershell
scp -r "ai4r@${taskStartIp}:~/ai4r-car-bags/car05-v2-startfix-*" $taskStartEvidence
scp "$taskStartEvidence\before\policy_node.py" "ai4r@${taskStartIp}:$taskStartWs/scripts/policy_node.py"
scp "$taskStartEvidence\before\ai4r_policy.yaml" "ai4r@${taskStartIp}:$taskStartWs/config/ai4r_policy.yaml"
Get-Content "$taskStartEvidence\before\sha256.txt"
```

车端哈希必须与 before 记录一致，再构建、重启。确认 state 2、Disabled 后交接，说明恢复的具体版本；如果恢复的是固定油门辨识配置，明确告知它仍是辨识模式，不当成道路跟踪策略交接。当天最早的原 PID 备份仍保留。
