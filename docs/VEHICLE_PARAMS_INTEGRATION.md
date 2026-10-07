# 车辆参数接入 MPC：对照与未解决问题

适用分支：`control-mpc-v0`，提交 `3acd32d` 及以后（v3）。入口文档见 [MPC_README.md](MPC_README.md)。

本文说明三部分如何接到一起：
- 参数辨识组的 `VehicleParams`（[测试计划](vehicle-params-test-plan.md)）；
- 预测模型（`offline/mpc_prediction_model/`）；
- MPC（`scripts/policy_node.py`）。

**当前数值的来源**（车辆 .27，见 `config/ai4r_policy_mpc_newcar27.yaml`）：
- 纵向参数从 2026-10-06 的 MVP 实车日志拟合，脚本是 [`fit_drive_from_logs.py`](../offline/vehicle_identification/fit_drive_from_logs.py)，结果在 `offline/vehicle_identification/results/newcar27_drive_fit.json`；
- 转向符号已实测（正请求向右转，gain 为负）；
- 转向 gain 的大小、零偏、轴距和车身尺寸仍是课程占位值，所以 `valid: false`。

## 数据流

```
测试计划 → offline/vehicle_identification/vehicle_params.py   （离线记录，None = 未确认）
        → config/ai4r_policy.yaml 的 vehicle.*              （运行时，同名字段）
        → VehicleParamsSettings                              （policy_node.py，启动时检查一致性）
            ├─ MPC：mpc.vehicle_params_source = vehicle 时作为预测模型参数和约束
            ├─ valid 时经 records() 覆盖 estimator 输出的 vehicle_params / vehicle_limits → Planning
            └─ records() 同时推导 steering_limit_rad、steering_direction → MVP 的 calibrated 模式（PolicyController）
```

车上只安装 `scripts/policy_node.py`，所以离线文件不能在运行时 import。预测模型以 `course_model_step` 的形式逐行复制进 policy_node；`tests/test_mpc.py` 检查两件事：
- 这份复制和离线 `vehicle_model.step` 的结果一致，误差 ≤ 1e-12；
- `vehicle.*` 的字段和 `VehicleParams` 一一对应。

## 字段对照

| 测试计划 `VehicleParams` / `vehicle.*` | 预测模型 `CourseModelParams` | estimator / Planning 记录（`records()`） | MPC 中的用途 |
|---|---|---|---|
| `wheelbase_m`、`rear_axle_from_cg_m` | `front_axle_m = L − l_r`、`rear_axle_m` | `effective_wheelbase_m`、`rear_axle_x_m = −l_r` | 运动学 |
| `body_front/rear_extent_from_cg_m`、`body_width_m` | — | `footprint_xy_m` 四角点 | 不用（只给 Planning） |
| `steering_gain_rad`、`steering_offset_rad` | 同名 | `steering_map`；另外推导出 `steering_direction = sign(gain)` | `δ_target = gain·u + offset`；增益为负表示正请求向右转 |
| `steering_min_rad`、`steering_max_rad` | 同名 | `steering_limit`，`curvature_limit_1pm`；另外推导出 `steering_limit_rad = min(abs(min), abs(max))` | 转角范围约束（可以不对称） |
| `steering_rate_limit_rad_s` | `steering_rate_lower/upper_rad_s = ∓rate` | 同名 | 转角变化率约束 |
| `drive_min`、`drive_max` | 不限制（[-1, 1]） | `drive_range` | 驱动约束取 `[max(0, drive_min), drive_max]`；车在动时下界提高到 `drive_deadband + 偏置估计` |
| `drive_response_model`（文本） | — | `drive_response.description` | 只作记录 |
| `braking_behavior`（文本） | — | `braking_behavior` | 只作记录 |
| `steering_delay_s`、`drive_delay_s` | — | 同名；`actuation_delay_s = max(两者)` | 用已发出的指令把状态推过延迟 |
| `mass_kg`、`motor_gain_n`、`drag_kg_per_m` | 同名含义 | `drive_response`；`acceleration_max_mps2 = motor_gain_n·(drive_max − drive_deadband)/mass_kg` | 纵向 `m·v̇ = gain·(drive − drive_deadband) − c·v·abs(v)` |
| **v3 新增** `drive_deadband`、`brake_gain_n` | 同名 | `drive_response` | 分段驱动力：drive ≥ 死区时用 `motor_gain_n`，低于死区时用 `brake_gain_n`（电调拖刹）；死区也是车在动时的油门下界 |
| **v3 新增** `drive_breakaway`、`breakaway_wait_s` | —（预测模型里没有静摩擦） | `drive_response` | 控制器的起步阶段：静止时给 `drive_breakaway`（按 `mpc.breakaway_ramp_per_s` 增加），速度达到 `mpc.moving_speed_mps` 后交给 MPC |
| **新增** `speed_max_mps`、`braking_deceleration_mps2`、`safety_margin_m` | — | 同名 | 不用（只给 Planning） |
| （运行时）`valid`、`source` | — | 同名 | 执行门禁 |

v2 新增的字段（质量、电机增益、阻力，以及速度、制动、余量三项限值）原因如下：
- 原计划的 `drive_response_model` 只是文字描述，而 MPC 需要数值系数；
- Planning 原本就需要速度、制动和余量三项限值，但测试计划里没有。

v3 新增的 4 个字段（`drive_deadband`、`brake_gain_n`、`drive_breakaway`、`breakaway_wait_s`）已经加进 `vehicle_params.py`，**还要辨识组确认，并同步写进测试计划**。原因是：.27 的实车日志显示，过原点的纵向模型在 0.2 m/s 只给 0.004 的油门，而实车需要约 0.30。

.27 的拟合值（请求值单位，质量取 3 kg 只作比例换算）：

| 量 | 值 |
| --- | --- |
| 死区 | 0.289 |
| 死区以上加速度斜率 | 10.0 m/s² 每单位油门 |
| 拖刹 | 1.84 m/s² 每单位油门 |
| 阻力 | 1.94 1/m |
| 延迟 | 0.10 s |
| 起步 | 请求 ≥0.30 并保持约 0.2 s |
| 精度 | B10/B11/B12/B14 速度 RMSE 0.024–0.032 m/s |

## valid / source 约定与门禁

- `valid: false`（默认）表示这条记录没有获准上车，其中的 0 是占位值，不代表任何事实。ROS 参数不能为 None，所以用这个标志代替。
- `valid: true` 的条件：
  - 所有值都已实测；
  - `source` 填写能追溯的标签（日期、run id），不能是 `unmeasured`、`course_simulation` 或 `offline_test_assumption`；
  - 通过 `VehicleParamsSettings.problem()` 的一致性检查，例如 `0 < l_r < L`、`min ≤ offset ≤ max`、`drive_min < drive_max`、车身轮廓包住两轴；v3 起还要求 `0 ≤ drive_deadband < drive_max`、`brake_gain_n ≥ 0`，以及（`drive_breakaway` 非零时）`drive_deadband ≤ drive_breakaway ≤ drive_max`、`0 ≤ breakaway_wait_s ≤ 2`。

  任何一项不满足，节点拒绝启动。
- shadow 模式可以使用 `valid: false` 但字段完整的记录（例如 `newcar27_logfit_20261006`）：`gate_problem` 只在执行时检查 `valid`，`problem()` 只检查一致性。
- `mpc.vehicle_params_source`：
  - `course_simulation`：使用 notebook 的数值，只允许 shadow 模式，并且必须和 `planning.vehicle_limits_source: course_simulation` 一起选；
  - `vehicle`：使用 `vehicle.*`，记录无效时日志 branch 为 `vehicle_invalid`。
- 执行（`shadow: false`）需要同时满足：`vehicle_params_source: vehicle`、`vehicle.valid: true`、`v_exec_max_mps > 0`；bypass 模式另需 `bypass_acknowledged`。

## 合不进来、先记录的问题

1. **倒车和负油门没有建模；拖刹已建模（v3）。** 预测模型没有 Dream Gym 的前进转倒车锁存，所以 MPC 只发正向驱动。.27 在低于死区的请求下会主动减速（电调拖刹，约 0.5 m/s²），v3 用 `brake_gain_n` 建模；停车时零油门由框架的状态 2 执行。`braking_behavior` 确认之前，不在模型中加入负驱动。
2. **~~纵向模型只是一个假设形式~~（v3 已解决）。** 已加入 `drive_deadband` 和 `brake_gain_n`，参数从 .27 的日志拟合（见上表）。仍然存在的局限：数据只有一台车、一天、一个电量状态；死区会随电池漂移，由偏置估计 `mpc.drive_bias_gain` 补偿；仿真标定显示起步峰值被低估约 0.07 m/s。测试计划还需要同步加上这 4 个字段。
3. **延迟只做了"已发指令推演"。** 两个通道的延迟不同时，统一推演到较大的那个延迟，较快的通道在差额时间里按保持上一指令处理。轮速滤波和相机时延没有和执行器延迟区分（测试计划已要求测试时区分）。
4. **低速时的转向。** 运动学模型在 v ≈ 0 时转向不影响位置，所以起步阶段转向由变化率代价维持在当前值附近，这是预期行为。
5. **车身轮廓只给 Planning 用。** MPC 内没有车身和锥桶的碰撞约束。bypass 模式下 Planning 的余量检查完全不起作用。
6. **轮角估计。** 没有轮角传感器，`delta_est_rad` 是已应用指令经模型回放得到的，误差来自转向增益、偏置和速率的辨识误差。
7. **~~和 main 的关系~~（v3 已解决）。** main 和 `fix/newcar-control` 已合入（`3acd32d`）。`VehicleCalibrationSettings` 已删除，`vehicle.*` 统一使用本记录，MVP 需要的 `steering_limit_rad` 和 `steering_direction` 由 `records()` 推导。控制器的选择方式是 `control.enabled` 和 `mpc.enabled` 二选一，接口契约 #14 的 `controller_mode` 仍未实现。
8. **Jetson 上的耗时还没测。** v3 改为只建一次 OSQP 之后，笔记本上平均约 2.6 ms、p95 约 2.7 ms（N = 10，每步 1 次 SQP）。G1 需要在车上重新测量。
9. **静摩擦不在预测模型里（v3）。** `drive_breakaway` 和 `breakaway_wait_s` 只由控制器的起步阶段使用；预测模型认为"油门高于死区车就会加速"。所以起步阶段的预测速度会比实际更早上升，这一点由起步阶段直接覆盖油门来规避。

## 验证

```
python -m unittest tests.test_control tests.test_mpc tests.test_planning tests.test_estimation   # 22 + 70 + 17 + 28
cd offline/mpc_prediction_model && python -m unittest discover -p "test_*.py"     # 6
cd offline/vehicle_identification && python -m unittest discover -p "test_*.py"   # 18，含日志拟合回归测试
python offline/vehicle_identification/fit_drive_from_logs.py                      # 重新拟合 .27 的纵向模型
python offline/mpc_prediction_model/validate_vehicle_model.py   # 需要 Dream Gym，对照仿真器开环验证
```

完整命令见 [MPC_README.md §7](MPC_README.md#7-怎么验证)。

Dream Gym 中与 PID 基线的条件一致对比（仿真器参数，不是实车值）见 [offline/mpc_gym/](../offline/mpc_gym/README.md)。

`tests/test_policy_node.py` 需要 ROS Jazzy，要通过 `tools/verify_fast.sh` 在 CI 或 Jetson 上运行。
