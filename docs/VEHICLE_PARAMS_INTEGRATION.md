# 车辆参数接入 MPC：对照与未解决问题

适用分支：`control-mpc-v0`。本文说明三部分如何接到一起：参数辨识组的 `VehicleParams`（[测试计划](vehicle-params-test-plan.md)）、预测模型（`offline/mpc_prediction_model/`）和 MPC（`scripts/policy_node.py`）。文中所有数值都还没有实车测量。

## 数据流

```
测试计划 → offline/vehicle_identification/vehicle_params.py   （离线记录，None = 未确认）
        → config/ai4r_policy.yaml 的 vehicle.*              （运行时，同名字段）
        → VehicleParamsSettings                              （policy_node.py，启动时检查一致性）
            ├─ MPC：mpc.vehicle_params_source = vehicle 时作为预测模型参数和约束
            └─ valid 时经 records() 覆盖 estimator 输出的 vehicle_params / vehicle_limits → Planning
```

车上只安装 `scripts/policy_node.py`，所以离线文件不能在运行时 import。预测模型以 `course_model_step` 的形式逐行复制进 policy_node；`tests/test_mpc.py` 检查两件事：
- 这份复制和离线 `vehicle_model.step` 的结果一致，误差 ≤ 1e-12；
- `vehicle.*` 的字段和 `VehicleParams` 一一对应。

## 字段对照

| 测试计划 `VehicleParams` / `vehicle.*` | 预测模型 `CourseModelParams` | estimator / Planning 记录（`records()`） | MPC 中的用途 |
|---|---|---|---|
| `wheelbase_m`、`rear_axle_from_cg_m` | `front_axle_m = L − l_r`、`rear_axle_m` | `effective_wheelbase_m`、`rear_axle_x_m = −l_r` | 运动学 |
| `body_front/rear_extent_from_cg_m`、`body_width_m` | — | `footprint_xy_m` 四角点 | 不用（只给 Planning） |
| `steering_gain_rad`、`steering_offset_rad` | 同名 | `steering_map` | `δ_target = gain·u + offset`；增益为负表示正请求向右转 |
| `steering_min_rad`、`steering_max_rad` | 同名 | `steering_limit`，`curvature_limit_1pm` | 转角范围约束（可以不对称） |
| `steering_rate_limit_rad_s` | `steering_rate_lower/upper_rad_s = ∓rate` | 同名 | 转角变化率约束 |
| `drive_min`、`drive_max` | 不限制（[-1, 1]） | `drive_range` | 驱动约束取 `[max(0, drive_min), drive_max]` |
| `drive_response_model`（文本） | — | `drive_response.description` | 只作记录 |
| `braking_behavior`（文本） | — | `braking_behavior` | 只作记录 |
| `steering_delay_s`、`drive_delay_s` | — | 同名；`actuation_delay_s = max(两者)` | 用已发出的指令把状态推过延迟 |
| **新增** `mass_kg`、`motor_gain_n`、`drag_kg_per_m` | 同名含义 | `drive_response` | 纵向 `m·v̇ = k·drive − c·v|v|` |
| **新增** `speed_max_mps`、`braking_deceleration_mps2`、`safety_margin_m` | — | 同名 | 不用（只给 Planning） |
| （运行时）`valid`、`source` | — | 同名 | 执行门禁 |

新增字段已经同时加进 `vehicle_params.py` 和测试计划，原因如下：
- 原计划的 `drive_response_model` 只是文字描述，而 MPC 需要数值系数；
- Planning 原本就需要速度、制动和余量三项限值，但测试计划里没有。

## valid / source 约定与门禁

- `valid: false`（默认）表示这条记录没有获准上车，其中的 0 是占位值，不代表任何事实。ROS 参数不能为 None，所以用这个标志代替。
- `valid: true` 的条件：所有值都已实测，`source` 填写能追溯的标签（日期、run id），不能是 `unmeasured`、`course_simulation` 或 `offline_test_assumption`。还要通过 `VehicleParamsSettings.problem()` 的一致性检查，例如 `0 < l_r < L`、`min ≤ offset ≤ max`、`drive_min < drive_max`、车身轮廓包住两轴。任何一项不满足，节点拒绝启动。
- `mpc.vehicle_params_source`：
  - `course_simulation`：使用 notebook 的数值，只允许 shadow 模式，并且必须和 `planning.vehicle_limits_source: course_simulation` 一起选；
  - `vehicle`：使用 `vehicle.*`，记录无效时日志 branch 为 `vehicle_invalid`。
- 执行（`shadow: false`）需要同时满足：`vehicle_params_source: vehicle`、`vehicle.valid: true`、`v_exec_max_mps > 0`；bypass 模式另需 `bypass_acknowledged`。

## 合不进来、先记录的问题

1. **倒车和制动没有建模。** 预测模型没有 Dream Gym 的前进转倒车锁存，在 `brake` 工况的误差达到米级。因此 MPC 只发正向驱动，停车依靠零指令滑行，由框架的状态 2 执行。`braking_behavior` 确认之前，不在模型中加入负驱动。
2. **纵向模型只是一个假设形式。** 线性力加二次阻力描述不了 ESC 死区和静摩擦，静止起步的实际响应可能差很多。如果辨识结果显示死区明显，需要在模型和 `vehicle.*` 中增加例如 `drive_deadband` 的字段，并同步更新测试计划。
3. **延迟只做了"已发指令推演"。** 两个通道的延迟不同时，统一推演到较大的那个延迟，较快的通道在差额时间里按保持上一指令处理。轮速滤波和相机时延没有和执行器延迟区分（测试计划已要求测试时区分）。
4. **低速时的转向。** 运动学模型在 v ≈ 0 时转向不影响位置，所以起步阶段转向由变化率代价维持在当前值附近，这是预期行为。
5. **车身轮廓只给 Planning 用。** MPC 内没有车身和锥桶的碰撞约束。bypass 模式下 Planning 的余量检查完全不起作用。
6. **轮角估计。** 没有轮角传感器，`delta_est_rad` 是已应用指令经模型回放得到的，误差来自转向增益、偏置和速率的辨识误差。
7. **和 main 的关系。** `origin/main` 上 PID 组的 MVP（`e0d226e`）也定义了 `vehicle.*` 前缀（`VehicleCalibrationSettings`），但字段名不同（`front_extent_m`、`steering_limit_rad + steering_direction`，没有 offset），同时还有 `control.*` 的 PID/MVP 控制器。这个分支没有合入 main 的这两个提交。将来合回 main 时，需要和 PID 组商定统一的 `vehicle.*` 字段（建议以测试计划的名字为准），以及控制器选择方式（接口契约 #14 `controller_mode`）。
8. **Jetson 上的耗时还没测。** 笔记本上平均约 5 ms、最大约 9 ms（每步 1 次 SQP）。G1 需要在车上重新测量。

## 验证

```
python -m unittest tests/test_mpc.py tests/test_planning.py tests/test_estimation.py
python offline/mpc_prediction_model/test_vehicle_model.py
python -B offline/vehicle_identification/test_vehicle_identification.py
python offline/mpc_prediction_model/validate_vehicle_model.py   # 需要 Dream Gym，对照仿真器开环验证
```

Dream Gym 中与 PID 基线的条件一致对比（仿真器参数，不是实车值）见 [offline/mpc_gym/](../offline/mpc_gym/README.md)。

`tests/test_policy_node.py` 需要 ROS Jazzy，要通过 `tools/verify_fast.sh` 在 CI 或 Jetson 上运行。
