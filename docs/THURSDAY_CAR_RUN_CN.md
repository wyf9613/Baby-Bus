# 周四（2026-10-15）实车清单：转向标定 → shadow 复测 → MPC 直道 → 缓弯

分支 `mpc-car-run`，车 10.43.254.27。部署、IMU 诊断、传感器检查、Foxglove、录包的通用命令见 [G2 清单](G2_SHADOW_CHECKLIST_CN.md) 第 1–8 步；本文只写和 G2 不同的地方，以及每一步的**通过条件**和**停止条件**。

Gym 依据：`offline/mpc_gym/results/car_readiness.md`（直道 1.0 m 10/10，缓弯 10/10）。改了哪些上游问题：[UPSTREAM_FIXES_FOR_CAR_CN.md](UPSTREAM_FIXES_FOR_CAR_CN.md)。

## 总原则
- **起步时车要摆正：居中 ±5 cm、车头 ±3°，用卷尺量。** G2 回放显示，偏位约 18 cm 或车头偏右再叠加航向偏差时，V2 在起步阶段会判定车身贴边，直接硬停车（`recovery_body_boundary_conflict`）。
- 每一步通过了才进下一步。任何一步出现**方向反了**（车往偏离的方向打）、**无法解释的锁停**连续两次，或者 IMU 故障，就停止执行，改回 shadow 查原因。
- 执行时始终有一个人站在车旁，手放在遥控器上或守着 Foxglove 的 **Vehicle Request Disable**。
- 每次运行都录包（命令见 G2 第 8 步，加上 `/car/identification_sample`），并在记录表里写下场景、起点姿态、结果和停车原因。
- **车上原有的策略文件全天只备份一次**（第 0.5 步），之后的 4 次部署都只复制、不备份。否则第二次“备份”拿到的已经是我们第一次部署的文件，原文件就丢了。收尾时按第 6 步恢复，并核对 SHA-256。

## 0. 出发前（本机）

```powershell
cd E:\26b\AI4R\Baby-Bus
git checkout mpc-car-run; git pull; git log --oneline -1
python tools/make_deploy_yaml.py --mode steer-arc  --out .verification\car-deploy-arc
python tools/make_deploy_yaml.py --mode mpc-shadow --out .verification\car-deploy-shadow
```

`mpc-exec` 要等标定数值填进去之后才能生成（第 2 步）。所以**笔记本要带到现场**，并装好 Python 和 PyYAML。

**器材**：卷尺（量轴距和摆放位置）、胶带、锥桶。场地需要三块：
- 短弧标定：约 3 m × 2 m 的空地，不放锥桶；
- 直道：路宽约 1 m，两侧锥桶间距 0.2–0.3 m，从车前 0.5 m 一直摆到 4 m 以外；
- 缓弯：**两侧都要有锥桶**，弯前至少 2 m 直道，半径约 3–5 m。如果弯道只有一侧锥桶，目前不支持，只能跑到弯口。

## 0.5 备份车上原有文件（全天只做一次）[本机 PS]

在同一个 PowerShell 窗口里做完全天，变量才不会丢。换了窗口，就重新设置开头这 4 个变量；`$taskEvidenceDir` 要填**同一个**目录。

```powershell
cd E:\26b\AI4R\Baby-Bus
$taskCarIp = '10.43.254.27'
$taskWs = '~/ai4r_student_workspace/src/ai4r_policy'
$taskStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$taskEvidenceDir = "E:\26b\AI4R\car-evidence\car-run-$taskStamp"
New-Item -ItemType Directory -Force "$taskEvidenceDir\original" | Out-Null

# 1) 拷回本机
scp "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py" "$taskEvidenceDir\original\policy_node.py"
scp "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml" "$taskEvidenceDir\original\ai4r_policy.yaml"
# 2) 车上另存一份（本机文件出问题时的第二份）
ssh "ai4r@$taskCarIp" "mkdir -p ~/ai4r-original-$taskStamp && cp $taskWs/scripts/policy_node.py $taskWs/config/ai4r_policy.yaml ~/ai4r-original-$taskStamp/ && sha256sum ~/ai4r-original-$taskStamp/*"
# 3) 记下 SHA-256，收尾时核对
Get-FileHash "$taskEvidenceDir\original\*" -Algorithm SHA256 | Format-Table Hash, Path -AutoSize | Out-String -Width 300 | Tee-Object "$taskEvidenceDir\original\sha256.txt"
```

**检查**：
- 两个文件都拷回来了，而且不是空的；
- 本机的两个 SHA-256 和车上 `sha256sum` 打印的一致；
- `original\policy_node.py` 的 SHA-256 **不应该**等于我们 `.verification\car-deploy-*\policy_node.py` 的。如果相等，说明车上已经是我们上次部署的版本，原文件要找 10-08 的备份（`car-evidence\g2-20261008-162044\previous-*`），或者问其他组。

**每次部署用这个函数**（只复制、不备份；同时把部署文件存进证据目录）：

```powershell
function Deploy-Car($dir) {
  scp "$dir\policy_node.py" "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py"
  scp "$dir\ai4r_policy.yaml" "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml"
  Copy-Item $dir "$taskEvidenceDir\$(Split-Path $dir -Leaf)" -Recurse -Force
  Get-FileHash "$dir\policy_node.py", "$dir\ai4r_policy.yaml" -Algorithm SHA256 | Format-Table Hash, Path -AutoSize
}
# 检查工具和标定工具单独放，不进 student workspace
ssh "ai4r@$taskCarIp" "mkdir -p ~/ai4r_mpc_check/$taskStamp/offline ~/ai4r-car-bags"
scp -r tools "ai4r@${taskCarIp}:~/ai4r_mpc_check/$taskStamp/"
scp -r offline/vehicle_identification "ai4r@${taskCarIp}:~/ai4r_mpc_check/$taskStamp/offline/"
```

每次 `Deploy-Car` 之后，在车上执行：

```bash
# [车 1] 车辆必须是 Disabled，策略在 state 2
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml   # 和 Deploy-Car 打印的一致
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
```

第一次部署前，按 G2 第 2 步和第 4 步启动其他服务、检查 IMU 诊断和传感器。

## 1. 转向短弧标定（约 30 分钟）

**目的**：测出转向增益 `steering_gain_rad` 和零偏 `steering_offset_rad`。MPC 执行模式要求这两个值可信（`vehicle.valid: true`）。

1. **量轴距**：前后轮轴中心距离，记为 `L`（米）。
2. **部署**：`Deploy-Car .verification\car-deploy-arc`，然后在车上核对 SHA、构建、重启（见第 0.5 步）。运行时检查：
   ```bash
   N=/car/ai4r_policy
   for p in id_test.mode id_test.steering_values id_test.drive_values control.enabled mpc.enabled planning.lattice.async_enabled; do
     printf "%-30s " $p; ros2 param get $N $p; done
   ```
   期望依次为 `arc`、`[0.0, 0.25, -0.25, 0.4, -0.4, 0.0]`、全是 0.32、False、False、False。
3. **开始记录**（另开一个车上窗口；`<stamp>` 是第 0.5 步的 `$taskStamp`）：
   ```bash
   python3 ~/ai4r_mpc_check/<stamp>/offline/vehicle_identification/record_vehicle_test.py \
     --namespace car --car-id 27 --notes "arc L=<L> battery=<电压>" --output ~/ai4r-car-bags/arc-$(date +%H%M%S).jsonl
   ```
4. 车放在空地中央、车头朝空地较长的方向，**Vehicle Request Enable** → 看到 Enabled → 请求 state 3。序列共 16 s：先静止 2 s，然后依次是直行、右小弯、左小弯、右大弯、左大弯、直行，每段 2 s，最后静止 2 s，之后自动回到 state 2。
5. 车停稳后，**Vehicle Request Disable**；在录制窗口按 `Ctrl+C`。
6. **分析**（车上或拷回本机都可以）：
   ```bash
   python3 ~/ai4r_mpc_check/<stamp>/offline/vehicle_identification/vehicle_test_tools.py steering-arcs \
     --log ~/ai4r-car-bags/arc-*.jsonl --wheelbase <L> --output /tmp/steering-arcs.json
   ```
   **通过条件**：
   - `max_abs_residual_rad` < 0.03；
   - 4 个转弯段都没有标 `skipped`；
   - 增益为负（正指令向右转，和 10-06 的结论一致）。

   段落被跳过（车速不到 0.08 m/s）时，用 `make_deploy_yaml.py --arc-drive 0.34` 重新生成部署文件再跑一次。残差大，就再跑一次，看两次的结果是否一致。
7. **把结果填进 `config/ai4r_policy_mpc_newcar27.yaml` 的 `vehicle` 段**：
   - `wheelbase_m: <L>`；
   - `steering_gain_rad`、`steering_offset_rad`：用工具输出的 `vehicle_yaml`；
   - `steering_min_rad = b − 0.5·|k|`，`steering_max_rad = b + 0.5·|k|`，让请求上限仍是 ±0.5，和 MVP 一样；
   - `source: "newcar27_arcs_20261015"`，`valid: true`。

   填完提交到分支。

## 2. 生成执行部署文件（本机）

```powershell
python tools/make_deploy_yaml.py --mode mpc-shadow --out .verification\car-deploy-shadow   # 重新生成，带上标定值
python tools/make_deploy_yaml.py --mode mpc-exec --distance 3 --out .verification\car-deploy-exec
python tools/make_deploy_yaml.py --mode mpc-exec --distance 5 --v-exec-max 0.15 --out .verification\car-deploy-curve
```

`mpc-exec` 会检查 `vehicle.valid: true`，没填会直接报错。

## 3. Shadow 复测（不 Enable，推车）

`Deploy-Car .verification\car-deploy-shadow`，在车上核对、构建、重启，然后按 G2 第 5–7 步操作。G2 第 5 步的参数检查**额外加上**这几个：

```bash
for p in estimation.history_propagation estimation.road_history_mode estimation.history_merge_radius_m \
         estimation.history_merge_single_side planning.lattice.blind_extension_mode planning.lattice.max_near_gap_m \
         planning.lattice.async_enabled vehicle.valid vehicle.steering_gain_rad; do
  printf "%-40s " $p; ros2 param get /car/ai4r_policy $p; done
```

期望依次为 `continuous`、`distance`、0.08、True、`parallel`、1.0、True、True，以及第 1 步测出的增益。

推车场景，在直道上每个都**推 2 m 以上**，这样才能走过约 0.9 m 的相机盲区，用到融合历史：

| 场景 | 摆放 | 预期 |
| --- | --- | --- |
| A | 居中 | `e_y`、`e_psi` 接近 0 |
| B | 左偏 5 cm | `e_y` 约 +0.05，候选转角为负（向右修正） |
| D | 车头左偏 5° | `e_psi` 约 +0.087，候选转角为负 |

**通过条件**（用 `python3 tools/mpc_shadow_summary.py ~/ai4r-car-bags/<包>` 统计）：
- 推车阶段有效（`shadow` 分支）≥ 80%；
- 没有 `stop` 分支，也就是没有硬停车，**静止起步阶段也不能有**；
- 居中静止时 `e_psi` 的中位数记下来：如果明显不为 0（10-08 约 −0.06 rad），就是相机安装角偏差，要记录并告诉估计组；
- 候选转角的修正方向 ≥ 85%；
- `step_p95` < 30 ms；
- 推过 1 m 以后，`mpc_debug` 里能看到 `reference_held`，同时 `/car/policy_fsm_state_string` 没有掉回 state 2。

不通过时**不要执行**。记下拒绝原因，拷回包，用 `tools/replay_policy_bag.py` 离线查。

## 4. 直道执行（MPC 动车）

`Deploy-Car .verification\car-deploy-exec`，在车上核对、构建、重启。参数检查：`mpc.shadow` 为 False，`mpc.v_exec_max_mps` 为 0.2，`control.max_distance_m` 为 3.0。

1. 车居中摆在直道起点 → **Enable** → 看到 Enabled → 请求 state 3。
2. 预期：1–2 s 内起步（要先等异步规划的第一个结果，启动缓冲最长 3 s），以约 0.2 m/s 前进，走满 3 m 后自动停下（距离上限），回到 state 2。
3. 结束后 **Disable**。
4. 第一次成功后，再做一次**左偏 5 cm 起步**。

**通过条件**：两次都走满 3 m，没有锁停；横向偏差目测不超过 10 cm。

**立即 Disable 的情况**：
- 车往偏离的方向打（转向符号错了）；
- 车速明显超过 0.3 m/s；
- 车身压到锥桶。

锁停（掉回 state 2）本身是安全的：记下 `state_string` 的原因，看有没有规律，**同一个原因连续两次就不要再跑**。

## 5. 缓弯执行（直道两次都成功后）

`Deploy-Car .verification\car-deploy-curve`（5 m 距离上限，限速 0.15 m/s），在车上核对、构建、重启。

1. 车放在弯前至少 2 m 的直道上，居中。
2. Enable → state 3。预期：先直行，在入弯处开始转向，沿弯道走满 5 m 或走出弯道。
3. 通过条件和停止条件同第 4 步。另外，入弯时如果锁停，记下原因，并拷回包回放：如果是 `single_side_width_unknown` 或 `no_feasible_lattice_trajectory`，基本可以确定是锥桶只在一侧。

## 6. 收尾：停车、取证据、恢复原文件

**6.1 停下** [车 1] + [Foxglove]
```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
timeout 5 ros2 topic echo /car/traxxas_state --once      # 必须是 Disabled；不是就在 Foxglove 点 Disable
dream runtime logs ai4r_policy --tail 300 > ~/ai4r-car-bags/car-run-policy.log 2>&1
```

**6.2 取回证据** [本机 PS]
```powershell
scp -r "ai4r@${taskCarIp}:~/ai4r-car-bags/*" $taskEvidenceDir
scp "ai4r@${taskCarIp}:/tmp/steering-arcs.json" $taskEvidenceDir
Get-ChildItem $taskEvidenceDir
```
现场记录表（每次运行的场景、起点姿态、结果、停车原因）也放进这个目录。

**6.3 恢复车上原有文件** [本机 PS] → [车 1]
```powershell
scp "$taskEvidenceDir\original\policy_node.py" "ai4r@${taskCarIp}:$taskWs/scripts/policy_node.py"
scp "$taskEvidenceDir\original\ai4r_policy.yaml" "ai4r@${taskCarIp}:$taskWs/config/ai4r_policy.yaml"
Get-Content "$taskEvidenceDir\original\sha256.txt"
```
```bash
# [车 1]
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py \
          ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml   # 必须和 original\sha256.txt 一致
dream build ros student && (dream runtime restart ai4r_policy || dream runtime start ai4r_policy)
dream runtime status | grep -E "UNIT|ai4r_policy"
timeout 3 ros2 topic echo /car/policy_fsm_state_string --once   # 原来的策略在运行，state 2
```
- SHA 对不上，就从车上第二份 `~/ai4r-original-<stamp>/` 复制回去，再核对一次。
- 恢复成功后，车上那份备份可以保留；需要清理时再删 `~/ai4r-original-<stamp>` 和 `~/ai4r_mpc_check/<stamp>`。
- 共用车辆时，交接前告诉下一组：车已经 Disabled，策略已经恢复成原来的。

**6.4 记录**：进度日志写一条，包括标定结果、每次运行的结果和停车原因、和 Gym 的对比，以及“原文件已恢复，SHA 已核对”。
