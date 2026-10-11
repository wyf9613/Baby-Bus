# 实车参数测试：运行说明

默认测试关闭；17 项离线测试通过，完整 ROS Jazzy 验证及实车测试尚未完成。以下命令从仓库根目录运行，使用已加载课程 ROS 环境的终端；`car` 替换为实际命名空间。

## 1. 验证与配置

按根目录 [CONTRIBUTING.md](../../CONTRIBUTING.md) 准备指定版本的 `dream_interfaces`，运行：

```bash
python3 offline/vehicle_identification/test_vehicle_identification.py
python3 offline/vehicle_identification/vehicle_test_tools.py preview
AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" bash tools/verify_fast.sh
```

编辑 `config/ai4r_policy.yaml`：

- 设置 `policy_update_mode: timer`、`required_sensors: [wheel_speed]`；需要偏航数据时加入 `imu_angular_velocity`。
- `id_test.mode` 选择 `steering`、`drive`、`brake`；关闭用 `"off"`。
- `durations_s`、`drive_values`、`steering_values` 三个数组等长，用小数填写。
- 现场确认驱动、转向和速度限制后，设置 `confirmed: true`。转向模式驱动必须全零；驱动/制动模式转向必须全零。
- 负驱动另需 `brake` 模式及 `negative_drive_confirmed: true`，仅在已确认 ESC 规则后启用。

先做全零演练：`mode: steering`、`confirmed: true`、`speed_limit_mps: 0.1`，其他实验字段保持默认，共 5 秒零请求。该速度阈值仅用于静止演练。通过后再填现场确认的非零请求；每次修改都重新预览并重启策略。

## 2. 部署与记录

停止旧策略，使车辆处于非运行状态。按课程方法复制修改后的策略、配置和本目录到车上的 `~/ai4r_student_workspace/src/ai4r_policy/`，再运行：

```bash
dream build ros student
dream runtime restart ai4r_policy
ros2 topic info /car/drive_and_steer_set_point_normalized
ros2 topic hz /car/wheel_speed_m_per_sec
```

确认仅一个控制发布者、轮速持续更新。每轮使用新的文件名，保存配置并开始记录：

```bash
mkdir -p log/vehicle_identification
ros2 param dump /car/ai4r_policy > log/vehicle_identification/run01-policy.yaml
ros2 param dump /car/traxxas_vehicle_interface > log/vehicle_identification/run01-vehicle.yaml
python3 offline/vehicle_identification/record_vehicle_test.py \
  --namespace car --car-id YOUR_CAR_ID --notes "电池、载荷、地面" \
  --output log/vehicle_identification/run01.jsonl
```

## 3. 启动与停止

出现 `Recording` 后，在 Foxglove 请求 Vehicle Enable，等到 Enabled，再请求 Policy State 3。人工停止计算用 State 2；实验到时、超速、时序异常或必需数据过期也会回到零请求。

**零请求不等于主动制动或 Disarm。** 现场保留 RC 停止手段，按课程流程确认停驶后，再 Ctrl+C 结束记录。关闭记录程序不会停车。再次显式请求 State 3 才会从头运行新一轮；程序不自动重复，也不避障。

## 4. 分析与交付

```bash
python3 offline/vehicle_identification/vehicle_test_tools.py export \
  --log log/vehicle_identification/run01.jsonl \
  --output-dir log/vehicle_identification/run01-export
```

导出 `samples.csv`（计划请求与观测）、`events.csv`（实际发布请求及传感器/状态事件）和 `summary.json`。发布请求不等于执行器实测输出；轮速无方向，日志与外部视频需同步。

其他离线工具：

| 子命令 | 输入与用途 |
| --- | --- |
| `geometry` | 尺量尺寸、前后轴荷 → 5 项几何参数 |
| `steering` | CSV 列 `steering_action,delta_left_deg,delta_right_deg,delta_equivalent_deg`；填左右轮角或已确定的等效角 → 转向比例、偏置和已测角度范围 |
| `steering-dynamics` | 已同步 CSV 列 `time_s,steering_action,delta_equivalent_deg` → 观测延迟和变化率摘要 |

用 `python3 offline/vehicle_identification/vehicle_test_tools.py <子命令> --help` 查看必填参数。几何长度用米、轴荷同单位，角度 CSV 用度；静态转向至少三个不同请求且覆盖左右。

几何与前轮角需要现场测量。工具结果仅为待复测候选，不自动确认纯延迟、物理速率上限、制动规则或纵向模型，也不改写 `vehicle_params.py`。复测确认后填写参数，未确认项保留 `None`。
