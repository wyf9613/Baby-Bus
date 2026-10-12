# 实车参数测试：运行说明

前向辨识已取得实车数据，见 [交付清单](../../docs/vehicle-identification-start.md) 和 [驱动结果](../../docs/vehicle-drive-identification-results-2026-10-12.md)。本次交付默认关闭实验；完整 ROS Jazzy 门禁仍未运行。

## 1. 验证与配置

按根目录 [CONTRIBUTING.md](../../CONTRIBUTING.md) 准备指定版本的 `dream_interfaces`，运行：

```bash
python3 offline/vehicle_identification/test_vehicle_identification.py
python3 offline/vehicle_identification/vehicle_test_tools.py preview
AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" bash tools/verify_fast.sh
```

当前 YAML：`timer`、必需 `wheel_speed`，预留请求 `0.35`、持续 `3.0 s`、转向 `0.0`、驱动上限 `0.35`、速度中止阈值 `3.0 m/s`。序列前后各有 `2.0 s` 零请求；尚无本工况结果。

补测前在 `config/ai4r_policy.yaml` 将 `id_test.mode` 改为 `drive`、`confirmed` 改为 `true`，其余配置保持上述值，再预览。结束后恢复 `mode: "off"`、`confirmed: false`。转向测试用 `steering` 模式，驱动必须全零；三个阶段数组必须等长。当前范围不测试负驱动。

## 2. 部署与记录

停止旧实验并确认车辆停稳。首次部署或更换车辆，在 Windows PowerShell 的本地仓库执行（替换当前 IP）：

```powershell
$carIp = "10.43.254.20"
scp scripts/policy_node.py "ai4r@${carIp}:~/ai4r_student_workspace/src/ai4r_policy/scripts/"
scp config/ai4r_policy.yaml "ai4r@${carIp}:~/ai4r_student_workspace/src/ai4r_policy/config/"
ssh "ai4r@$carIp" "mkdir -p ~/ai4r_student_workspace/src/ai4r_policy/offline"
scp -r offline/vehicle_identification "ai4r@${carIp}:~/ai4r_student_workspace/src/ai4r_policy/offline/"
ssh "ai4r@$carIp"
```

在小车 SSH 终端执行：

```bash
cd ~/ai4r_student_workspace/src/ai4r_policy
python3 offline/vehicle_identification/test_vehicle_identification.py
python3 offline/vehicle_identification/vehicle_test_tools.py preview
dream build ros student
dream runtime restart ai4r_policy
dream runtime status
ros2 param get /car/ai4r_policy id_test.mode
ros2 param get /car/ai4r_policy id_test.drive_values
ros2 param get /car/ai4r_policy id_test.durations_s
ros2 topic info /car/drive_and_steer_set_point_normalized
ros2 topic hz /car/wheel_speed_m_per_sec
```

确认车辆接口和策略运行、仅一个控制发布者、轮速持续更新且车已静止。重启后暂时找不到节点时，待节点出现再检查；不要把 `Parameter not set` 当作配置成功。仅改 YAML 时，按 DREAM 源配置流程重启对应节点即可。

每批使用新的名称，保存配置并开始记录：

```bash
mkdir -p log/vehicle_identification
run_name="drive035_3s_$(date +%Y%m%d_%H%M%S)"
ros2 param dump /car/ai4r_policy > "log/vehicle_identification/${run_name}-policy.yaml"
ros2 param dump /car/traxxas_vehicle_interface > "log/vehicle_identification/${run_name}-vehicle.yaml"
python3 offline/vehicle_identification/record_vehicle_test.py \
  --namespace car --car-id 10.43.254.20 --notes "0.35 for 3 s; record battery/load/surface" \
  --output "log/vehicle_identification/${run_name}.jsonl"
```

## 3. 启动与停止

出现 `Recording` 后，用 Foxglove 按钮：

1. 点击 **Policy publishing ZERO actions**（State 2）。
2. 点击 **Traxxas Request Enable**，等状态为 `Enabled`。
3. 车辆静止后点击 **Policy publishing actions**（State 3），执行一次。
4. 等实验完成、车辆完全停稳；点击 ZERO actions，再点击 publishing actions 可重复。建议共 3 次。
5. 最后点击 ZERO actions，确认停稳，再使用车辆 Disable / Disarm。记录终端按 `Ctrl+C`，关闭配置并重启策略。

到时、达到速度阈值、时序异常或必需数据过期会回到零请求。零请求仍可能滑行，保留 RC 停止手段。**Ctrl+C 只停止记录；NOT publishing actions（State 1）不负责停车。** 程序不自动重复，也不避障。

## 4. 分析与交付

```bash
python3 offline/vehicle_identification/vehicle_test_tools.py export \
  --log "log/vehicle_identification/${run_name}.jsonl" \
  --output-dir "log/vehicle_identification/${run_name}-export"
cat "log/vehicle_identification/${run_name}-export/summary.json"
```

导出样本、事件 CSV 和 `summary.json`。输出目录已存在时使用新目录名，不覆盖旧结果。保留原始 JSONL；只有摘要不能复算延迟和速度曲线。发布请求不等于执行器输出；轮速无方向且可能重复缓存值。

其他离线工具：

| 子命令 | 输入与用途 |
| --- | --- |
| `geometry` | 尺量尺寸、前后轴荷 → 5 项几何参数 |
| `steering` | CSV 列 `steering_action,delta_left_deg,delta_right_deg,delta_equivalent_deg`；填左右轮角或已确定的等效角 → 转向比例、偏置和已测角度范围 |
| `steering-dynamics` | 已同步 CSV 列 `time_s,steering_action,delta_equivalent_deg` → 观测延迟和变化率摘要 |

用 `python3 offline/vehicle_identification/vehicle_test_tools.py <子命令> --help` 查看必填参数。几何长度用米、轴荷同单位，角度 CSV 用度；静态转向至少三个不同请求且覆盖左右。

工具不会自动改写 `vehicle_params.py` 或连接车辆。原始日志及备份默认排除出 Git，按需单独交付。
