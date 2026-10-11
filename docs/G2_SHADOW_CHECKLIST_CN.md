# G2 现场清单：MPC shadow 推车（车 .27）

日期：2026-10-08。适用于 `control-mpc-v0`（≥ `5b3d30e`，dense 求解器）和车 10.43.254.27。背景和字段含义的完整说明见 [上车指南 §5、§7–§11](dream-car-babysitter-guide.md) 与 [MPC_PROTOTYPE.md §2](MPC_PROTOTYPE.md)；本文只按现场顺序列出命令，并纳入 10-08 在车上确认过的情况（车上无 osqp、IMU 故障的处理、`timeout` 的输出缓冲）。

## G2 要回答什么

在**车辆不使能**的前提下，让 MPC 在真车的感知、估计、规划数据上运行，只算不发（shadow：应用动作恒为 0）。用手推车，检查：

1. 横向误差 `e_y_m`、航向误差 `e_psi_rad` 的符号和大小与卷尺、角度记录一致；
2. 候选转角 `candidate_delta_rad` 朝修正方向；
3. 有效参考的比例和各类拒绝原因；
4. 实车上 MPC 控制器一步的耗时（`step_s`）在 50 ms 预算内，没有 `solver_fail`、`over_budget`；
5. 以约 0.2 m/s 推车时，候选油门 `candidate_drive` 接近拟合的稳态值约 0.30。

shadow 不会让车动。但部署会替换车上 student workspace 里的策略文件，共用车辆时先和其他组协调。

**默认做精简版**：场景 A（居中）、B（左偏 10 cm）、D（车头左偏 5°），覆盖横向和航向误差的符号。C、E 是 B、D 的镜像，可选；B 或 D 有疑问时再补。

**一个人做**：第 1–7 步照做，第 8–9 步换成文末的[单人流程](#单人流程)。

## 窗口约定

| 标记 | 在哪运行 |
| --- | --- |
| **[本机 PS]** | 笔记本的 PowerShell，目录 `E:\26b\AI4R\Baby-Bus` |
| **[车 1] [车 2] [车 3]** | 三个 SSH 窗口：`ssh ai4r@10.43.254.27`。车 1 操作和检查，车 2 录包，车 3 看 `mpc_debug`（单人流程只用车 1） |
| **[Foxglove]** | 浏览器里的 Foxglove，`ws://10.43.254.27:1234` |

看 topic 时一律用 `timeout`。要手动停就按 `Ctrl+C`，**不要按 `Ctrl+Z`**（只会挂起，进程还在）。把 `ros2 topic echo` 接到管道时加 `PYTHONUNBUFFERED=1`，否则 `timeout` 结束时输出会丢失。

## 0. 出发前准备

- 人员：理想是三人（拿遥控器、推车、操作终端）。shadow 加 Disabled 时车不会自己动，一个人也能做，遥控器放在手边即可。
- 器材：卷尺、量角参照（量角器或手机指南针）、胶带标记、锥桶。
- 道路：**直线**，宽约 1 m，两侧锥桶，从车前约 0.5 m 一直摆到 2.5 m 以外，两侧都要在相机视野内。Planning V1 只支持直线。
- 记录表：每个场景的实测横向偏移（m）、航向角（°）、推车距离、开始时间和备注。

## 1. 生成部署文件 [本机 PS]

```powershell
cd E:\26b\AI4R\Baby-Bus
git status --short                 # scripts/ config/ 不应有未提交改动
git log --oneline -1
$taskCarIp = '10.43.254.27'
$taskStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$taskDeployDir = 'E:\26b\AI4R\Baby-Bus\.verification\car-deploy'
$taskEvidenceDir = "E:\26b\AI4R\car-evidence\g2-$taskStamp"
New-Item -ItemType Directory -Force $taskEvidenceDir | Out-Null
python tools/make_deploy_yaml.py --mode mpc-shadow --distance 3.0 --out $taskDeployDir
git rev-parse HEAD | Set-Content -Encoding ascii (Join-Path $taskEvidenceDir 'source-sha.txt')
Copy-Item "$taskDeployDir\*" $taskEvidenceDir
```

脚本输出中应看到：`control.enabled False`、`mpc` 为 `enabled True / shadow True / qp_solver dense`、`vehicle newcar27_logfit_20261006 valid False`，以及两个 SHA-256。记下这两个值。

## 2. 确认车辆停用 [车 1] + [Foxglove]

```bash
timeout 5 ros2 topic echo /car/traxxas_state --once        # 应为 data: Disabled
timeout 5 ros2 topic echo /car/traxxas_state_value --once  # 应为 data: 0
dream runtime status
```

如果不是 Disabled，就在 Foxglove 里点 **Vehicle Request Disable**，等状态变成 Disabled 再继续。如果 `ai4r_policy` 正在运行，先让它持续发零：

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
```

## 3. 备份车上旧文件，复制新文件 [本机 PS]

```powershell
$taskWs = '~/ai4r_student_workspace/src/ai4r_policy'
scp "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py" (Join-Path $taskEvidenceDir 'previous-policy_node.py')
scp "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml" (Join-Path $taskEvidenceDir 'previous-ai4r_policy.yaml')
scp "$taskDeployDir\policy_node.py" "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py"
scp "$taskDeployDir\ai4r_policy.yaml" "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml"

# 汇总工具单独放进检查目录，不进 student workspace
$taskRemoteCheck = "ai4r_mpc_check/$taskStamp"
ssh "ai4r@$taskCarIp" "mkdir -p ~/$taskRemoteCheck ~/ai4r-car-bags"
scp -r tools "ai4r@${taskCarIp}:~/$taskRemoteCheck/"
Write-Host "检查目录：~/$taskRemoteCheck"
```

两个备份必须成功。失败的话先核对路径，备份没拿到就不要覆盖。

## 4. 核对、构建、启动 [车 1]

```bash
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml   # 与第 1 步的两个值一致
dream build ros student                                                     # 必须成功
dream runtime start foxglove_bridge
dream runtime start traxxas_vehicle_interface      # 启动但不 Enable
dream runtime start oakd_cone_detector
dream runtime start bno08x_imu_interface
dream runtime status
```

**IMU 先看诊断**（10-08 遇到过固件故障锁定）：

```bash
timeout 8 ros2 topic echo /car/diagnostics --once 2>&1 | grep -A1 -E "lifecycle_state|connection_state|publication_ready|reason_code"
```

- 应看到 `connection_state: streaming`、`host_publication_ready: 'true'`。`lifecycle_state` 为 `CALIBRATING` 也可以接受。
- 如果是 `FAULT_LATCHED`：`dream runtime stop bno08x_imu_interface` → 拔插 IMU 板的 USB 线（`lsusb` 里的 `DREAM BNO08x IMU`，别拔错）→ 车静止放平 → `dream runtime start bno08x_imu_interface` → 重新看诊断。仍然不行就交给 demonstrator。

**传感器有数据**：

```bash
timeout 6 ros2 topic hz /car/wheel_speed_m_per_sec     # 约 48 Hz
timeout 6 ros2 topic hz /car/cone_detections           # 约 10 Hz
PYTHONUNBUFFERED=1 timeout 5 ros2 topic echo /car/imu/data --field angular_velocity > /tmp/gyro.txt; grep -c "z:" /tmp/gyro.txt   # 应有上百条
```

**启动 policy**。`ai4r_policy` 已在运行就用 `restart`，否则用 `start`：

```bash
dream runtime restart ai4r_policy || dream runtime start ai4r_policy
sleep 5
dream runtime status | grep -E "UNIT|ai4r_policy"
```

## 5. 运行时参数检查 [车 1]

```bash
N=/car/ai4r_policy
ros2 topic info /car/drive_and_steer_set_point_normalized | grep "Publisher count"   # 1
for p in policy_update_mode required_sensors control.enabled mpc.enabled mpc.shadow mpc.qp_solver \
         mpc.reference_source mpc.vehicle_params_source planning.vehicle_limits_source \
         vehicle.source vehicle.valid vehicle.drive_deadband mpc.v_exec_max_mps; do
  printf "%-32s " $p; ros2 param get $N $p
done
```

| 参数 | 期望 |
| --- | --- |
| `policy_update_mode` | `timer` |
| `required_sensors` | `cone_detections, wheel_speed, imu_angular_velocity` |
| `control.enabled` | False（MVP 关闭） |
| `mpc.enabled` / `mpc.shadow` | True / True |
| `mpc.qp_solver` | `dense` |
| `mpc.reference_source` | `planning` |
| `mpc.vehicle_params_source` / `planning.vehicle_limits_source` | `vehicle` / `upstream` |
| `vehicle.source` / `vehicle.valid` | `newcar27_logfit_20261006` / False |
| `vehicle.drive_deadband` | 0.289 |
| `mpc.v_exec_max_mps` | 0.25（shadow 下不起作用） |

有任何一项不符，就停下来查部署文件，不要继续。

## 6. Foxglove 检查 [Foxglove]

导入 `docs/AI4R_Foxglove_UI_v2026-09-22.json` 布局后确认：

- **Vehicle state** 为 Disabled；
- 推车时 **Wheel speed** 有读数；
- **Cone locations** 里两侧锥桶都看得到；
- **Policy state** 先是 state 2；
- **Drive/steer commands** 一直为 0。

## 7. 进入 state 3 [车 1] + 观察 [车 3]

```bash
# [车 1]
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
timeout 3 ros2 topic echo /car/policy_fsm_state_string --once
```

```bash
# [车 3]：实时看关键字段
PYTHONUNBUFFERED=1 ros2 topic echo /car/mpc_debug --field data | grep --line-buffered -oE '"(branch|reject_reason|e_y_m|e_psi_rad|candidate_delta_rad|candidate_drive|speed_mps|step_s)": [^,]*'
```

如果进不了 state 3，看 `policy_fsm_state_string` 的原因（常见的是某个必需传感器缺失或过期）。处理好后再请求一次 state 3，策略不会自己恢复。

## 8. 推车场景，每个场景录一个包 [车 2]

每个场景的步骤相同：

1. 把车摆到起点，用卷尺和角度参照**量好并记下**实际姿态；
2. [车 2] 开始录包，把 `<场景名>` 换成下表里的名字：
   ```bash
   cd ~/ai4r-car-bags && ros2 bag record -o g2-$(date +%H%M%S)-<场景名> --topics \
     /car/mpc_debug /car/debug1 /car/debug2 /car/policy_fsm_state_string \
     /car/drive_and_steer_set_point_normalized /car/wheel_speed_m_per_sec \
     /car/cone_detections /car/imu/data /car/traxxas_state /car/diagnostics
   ```
3. 等显示 `Recording...` 后，先静止约 2 s，再**保持姿态**，以约 0.2 m/s（5 秒推 1 m）沿道路推 1–1.5 m。估计器需要运动历史才能对齐道路，所以必须推，不能只摆着；
4. 推完静止约 2 s，[车 2] 按 `Ctrl+C` 结束录包；
5. 确认 [车 1] 的策略仍在 state 3。如果掉回 state 2，记下原因，再请求 state 3。

| 场景 | 精简版 | `<场景名>` | 摆放（车相对道路中心线） | 预期 `e_y_m` / `e_psi_rad` | 预期候选转角 |
| --- | --- | --- | --- | --- | --- |
| A 居中平行 | ✅ | `A-centre` | 居中，车头沿路 | 都接近 0 | 接近 0 |
| B 左偏 | ✅ | `B-left10` | 向左 0.10 m，车头沿路 | 约 +0.10 / 接近 0 | 负（向右修正） |
| C 右偏 | 可选 | `C-right10` | 向右 0.10 m，车头沿路 | 约 −0.10 / 接近 0 | 正（向左修正） |
| D 车头偏左 | ✅ | `D-headleft5` | 居中，车头向左约 5°（0.087 rad） | 接近 0 / 约 +0.087 | 负 |
| E 车头偏右 | 可选 | `E-headright5` | 居中，车头向右约 5° | 接近 0 / 约 −0.087 | 正 |

不要超出 Planning 的工作范围：横向偏移最多 ±0.15 m（上限 0.2 m），航向最多约 ±5°（上限 0.25 rad）。超出范围时参考被拒绝是正常的，不能算作 MPC 符号错误。

## 9. 在车上汇总 [车 1]

```bash
cd ~/ai4r_mpc_check/$(ls -t ~/ai4r_mpc_check | head -1)
python3 tools/mpc_shadow_summary.py ~/ai4r-car-bags/g2-* --json-out ~/ai4r-car-bags/g2-summary.json | tee ~/ai4r-car-bags/g2-summary.txt
```

每个场景一行，各列含义：

- `valid`：有效参考（branch 为 `shadow`）的比例；
- `e_y`、`e_psi`、`delta`：有效样本的中位数；
- `fix%`：候选转角朝修正方向的比例；
- `drive@.2`：推车速度 0.15–0.25 m/s 时候选油门的中位数；
- `step95`、`stepmax`：MPC 控制器一步的耗时（ms），不含估计和规划；
- `applied`：应用了非零动作的步数，必须为 0。

最后一行 `Safety and solver checks` 应为 OK。如果车上报 `rosbag2_py` 导入失败，就先把包取回笔记本，在 WSL（已 source ROS）里运行同一命令。

## 10. 收尾 [车 1] + [Foxglove]

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
timeout 5 ros2 topic echo /car/traxxas_state --once      # Disabled
dream runtime logs ai4r_policy --tail 200 > ~/ai4r-car-bags/g2-policy.log 2>&1
ls -d ~/ai4r-car-bags/g2-*
```

- 共用车辆时，问清下一组需要什么。需要恢复原来的策略，就把第 3 步备份的两个文件复制回去，再运行 `dream build ros student` 和 `dream runtime restart ai4r_policy`。
- 不要关 Foxglove 来“停车”；停止就是 state 2 加 Disabled。

## 11. 取回证据 [本机 PS]

```powershell
scp -r "ai4r@${taskCarIp}:~/ai4r-car-bags/g2-*" $taskEvidenceDir
Get-ChildItem $taskEvidenceDir
```

把第 0 步的现场记录表（每个场景的实测姿态）也放进这个目录。

## 12. 通过条件

- [ ] B（及可选的 C）的 `e_y` 符号正确，大小与卷尺读数相差不超过约 3 cm；D（及可选的 E）的 `e_psi` 符号正确
- [ ] 做过的偏置场景，`fix%` 绝大多数为修正方向
- [ ] 所有场景 `applied` = 0，`Safety and solver checks: OK`（没有 `solver_fail`、`over_budget`）
- [ ] `step95` < 50 ms（`step_s` 是 MPC 控制器一步的耗时，含参考处理和求解，不含估计和规划；G1 中最慢 12.9 ms）
- [ ] 推车时有轮速，`drive@.2` 大致在 0.30 附近
- [ ] 每个场景的 `valid` 比例和主要 `reject_reason` 都记下并能解释（没有预设阈值，团队判断）
- [ ] 部署文件、SHA-256、备份、包、汇总、policy 日志、现场记录表都在 `car-evidence/g2-<时间>/`

任何一个场景的符号不对，就停止，不要进入执行阶段（G3/G4）。

## 要发回给我的内容

1. 第 9 步的完整输出（`g2-summary.txt`）；
2. 每个场景的实测姿态；
3. 过程中的异常，例如掉出 state 3 的原因，或推车时没有轮速。

## 单人流程

一个人、一台笔记本、**一个 SSH 窗口**。思路：录包用 `timeout -s INT 25` 定时 25 s 自动正常结束（与按 `Ctrl+C` 等效，会写好 `metadata.yaml`），人在这 25 s 里走过去推车；每个场景录完立刻汇总，确认正确再做下一个，不需要实时盯 `mpc_debug`。

**前提**：第 1–7 步已完成（车 Disabled，策略在 state 3，参数检查通过）。遥控器放在手边，Foxglove 可以不开。

**1. 在 SSH 窗口里定义快捷命令 `g2rec`**（复制粘贴一次即可）：

```bash
CHK=~/ai4r_mpc_check/$(ls -t ~/ai4r_mpc_check | head -1)
G2_TOPICS="/car/mpc_debug /car/debug1 /car/debug2 /car/policy_fsm_state_string
  /car/drive_and_steer_set_point_normalized /car/wheel_speed_m_per_sec
  /car/cone_detections /car/imu/data /car/traxxas_state /car/diagnostics"
g2rec() {   # 用法：g2rec B-left10   录 25 s 后自动结束
  # < /dev/null：录包程序不读键盘，否则在 timeout 下会被系统暂停（Stopped），什么都录不到
  cd ~/ai4r-car-bags && timeout -s INT 25 ros2 bag record -o g2-$(date +%H%M%S)-$1 \
    --topics $G2_TOPICS < /dev/null > /dev/null 2>&1
  B=$(ls -td ~/ai4r-car-bags/g2-*-$1 | head -1); echo "录好：$B"
  python3 $CHK/tools/mpc_shadow_summary.py "$B"
  timeout 3 ros2 topic echo /car/policy_fsm_state_string --once
}
```

**2. 每个场景**（按 A → B → D 的顺序）：

1. 摆车，用卷尺量好横向偏移、用量角参照摆好航向，记在记录表上；
2. 回到电脑，输入 `g2rec A-centre`（B 用 `g2rec B-left10`，D 用 `g2rec D-headleft5`），回车；
3. 走到车旁，**静止约 3 s**，然后保持姿态，用约 5 s 沿道路推 1 m（约 0.2 m/s），推完停住别动；
4. 等到 25 s，回到电脑看输出：先是一行汇总表，再是策略状态。

**3. 每个场景录完当场判断**：

| 场景 | 当场要看到 | 不对时 |
| --- | --- | --- |
| A | `valid` 不为 0；`e_y`、`delta` 接近 0；`applied` 为 0；最后一行 OK | `valid` 为 0：看下一行的 `reject reasons`，常见是道路覆盖不足或数据过期，调整锥桶或车的位置后重录 |
| B | `e_y` 约 +0.10，`delta` 为负，`fix%` 接近 100 | `e_y` 为负：**正负号约定有问题，停止**，把输出发给我 |
| D | `e_psi` 约 +0.087，`delta` 为负，`fix%` 接近 100 | 同上 |
| 每次 | 状态仍为 policy state 3；`drive@.2` 有数时大致 0.30 | 掉回 state 2：看状态字符串里的原因，处理后 `ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'` 再重录 |

推得太快或太慢时，`drive@.2` 会显示 `-`（没有 0.15–0.25 m/s 的样本），不影响正负号判断。

**4. 三个都通过后**：

```bash
python3 $CHK/tools/mpc_shadow_summary.py ~/ai4r-car-bags/g2-* --json-out ~/ai4r-car-bags/g2-summary.json | tee ~/ai4r-car-bags/g2-summary.txt
```

然后做第 10 步（收尾）和第 11 步（取回证据），把 `g2-summary.txt` 和记录表发给我。
