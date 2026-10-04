# 状态与道路估计首版：实现、接口和验证

本版用于估计组与规划、控制组对接。运行代码在 `scripts/policy_node.py`，调参沿用 `config/ai4r_policy.yaml` 的 `estimation` 节。GitHub 课程框架已有的传感器格式、坐标、时间、超时、状态机和五项动作返回优先；框架未定义的内部组间对象参考 Word 接口草案，并兼容两张对接截图。本文是本版实现说明，不是老师提供的正式报告模板。

## 实现范围

| 输入 | 当前处理 | 交给下游 |
| --- | --- | --- |
| `wheel_speed` | 样本感知的一阶低通 | 无符号轮速，m/s |
| `imu_angular_velocity` | 取车体 z 轴分量，一阶低通 | 偏航角速度，rad/s |
| `cone_detections` | 单帧筛选、左右分组、Huber 鲁棒边界拟合 | 局部道路中心线、边界、宽度和质量信息 |
| `lidar_cartesian` | 复制框架已转换的车体点集及扫描索引 | 障碍物观测，尚不判断是否阻挡行驶 |

当前是各传感器分别处理后统一封装，没有 KF/EKF、加速度积分、全局定位、锥桶跨帧关联、道路跨帧融合或车辆运动补偿。默认按新锥桶批次触发，检测器请求频率为 10 Hz，实际频率需要测量；不把策略 timer 的 50 Hz 当成相机测量频率。

框架和估计器均检查有效性及年龄；数据缺失、过期不能当成零。缓存里的同一测量不会重复更新低通滤波。新一轮策略和中断后的滤波状态重新初始化。

## 状态过滤与时间

一阶低通更新为 `filtered += alpha * (measurement - filtered)`，其中 `alpha = 1 - exp(-sample_dt/tau)`。采用实际样本间隔，而不是假定固定频率。时间常数大则平滑程度高、响应滞后大；时间常数小则响应快、噪声保留更多。

轮速没有原始采样时间戳，用框架的单调接收时间识别新样本和计算间隔；陀螺仪用 ROS 采样时间。两种时间域不相减。状态在当前 ROS 输出时刻保持最近的滤波结果，未做运动预测；各源时间和年龄独立保留。

默认 `base_link` 为名义地面上、车辆重心正下方的参考原点，x 向前、y 向左、z 向上。它不是后轴原点。轮速是无符号值，不能识别前进或倒车。正常车体轴下偏航角速度左转为正。

## 单帧道路重建

输入是老师检测器提供的锥桶位置、颜色和置信度，不直接处理图像。当前按蓝色左侧、黄色右侧的赛道约定分组，筛掉非有限值、低置信度和前向范围外的点，再按 x 排序。

边界拟合采用局部 `y(x)`：少点时用直线；至少 5 点且有足够不同 x 位置时用二次曲线。中值斜率初始化后执行最多 8 轮 Huber 加权拟合，降低离群点影响，再对保留点重新拟合。每侧最多处理 64 个点，输出最多 201 个采样点。覆盖太短、大间隔、异常宽度、过大斜率、交叉或退化几何会被拒绝。

双侧有效时，只使用双方共同的有效范围，取同 x 位置的左右中点作为中心线；宽度使用局部斜率修正后的近似法向距离。单侧有效且已知宽度时，沿边界法向偏移半个宽度推断中心线；缺失边界保持缺失，不伪造为已观测。

每个新批次重新估计当前可见道路，不拼接地图、不沿用旧帧道路补足盲区，也不超出有效点覆盖范围外推。本版适用于能表示为 `y(x)` 的局部适度弯曲道路，急弯、回头弯、分叉和大比例误检需要后续方案及验证。

`road_forward_max_m=3.0` 是本版使用锥桶的前向上限，不是相机量程。检测器的 `detection_depth_range_m=[0.1,5.0]` 是相机深度接受范围，也不是可靠道路探测距离。有效道路以本帧 `x_range_m` 为准。仿真笔记里的 80°/4 m 设置不能当成实车标定。

## 下游读取接口

结果是当前策略节点上的 Python 字典 `self.estimation_output`，不是新增 ROS topic。规划、控制接入后从同一字典读取；如团队另需跨节点消息，要单独约定传输格式。

### `state`

| 字段 | 含义 |
| --- | --- |
| `speed_mps` / `v_mps` | 同值别名，滤波轮速；缺失为 None |
| `yaw_rate_rps` / `yaw_rate_radps` | 同值别名，滤波偏航角速度；缺失为 None |
| `speed_valid`, `yaw_rate_valid` | 两个测量分别是否有效 |
| `valid` | 两个运动字段均有效时为 True |
| `timestamp_s`, `frame_id` | 本轮零阶保持状态的 ROS 时刻和参考坐标系 |
| `source_age_s`, `source_stamp_ns` | 分别保留轮速和陀螺仪年龄/采样时间；轮速采样 stamp 为 None |
| `confidence` | 当前仅是两个运动字段有效性指示，不是准确率或校准概率 |
| `lateral_error_m`, `heading_error_rad` | 尚未实现，均为 None；`road_relative_valid=False` |
| `mode` | `filtered_zero_order_hold`，没有运动预测 |

### `road`

| 字段 | 含义 |
| --- | --- |
| `centerline_xy` | 按前向顺序排列的道路中心线点，m；无效时为空 |
| `left_boundary_xy`, `right_boundary_xy` | 有效的实际可见边界采样；缺失侧为 None |
| `lane_width_m`, `x_range_m` | 道路宽度与中心线可用前向范围，m |
| `local_curvature_1pm` | 双侧中心线在有效范围起点处的局部曲率，1/m；单侧为 None |
| `valid`, `status`, `visibility` | 有效性、无效/推断原因，以及单双侧可见情况 |
| `boundary_source` | 每侧为 `observed` 或 `absent` |
| `confidence` | 保留点比例形成的质量指标；单侧降低权重，不是校准概率 |
| `fit_rms_m` | 有效拟合时的残差指标，不等于真实道路误差 |
| `timestamp_s`, `source_age_s`, `frame_id` | 几何采样时刻、年龄和参考坐标系 |
| `motion_compensated` | False，几何未补偿到当前车体时刻 |

### 其他输出

`obstacles` 含 `points_xyz_m`、`scan_indices`、`available`、`timestamp_s`、`source_age_s`、`frame_id` 和 `motion_compensated=False`。可用但无点与雷达不可用是不同状态；障碍聚类、通道相关性及停车意图由规划负责。

`vehicle_params` 与 `vehicle_limits` 的轴距、转向映射、延迟、车身轮廓、速度及加减速限制目前未测量，保持 None、`valid=False`、`source=unmeasured`。不将仿真参数复制为实车标定。

规划负责从道路生成最终行驶参考，包括截图中的 `path_coeffs`、`x_range_m`、`target_speed_mps`；本模块不输出目标速度。控制负责消费参考并输出归一化动作。当前策略仍返回零驱动、零转向、pan hold 和空 debug，不是已完成的自动驾驶闭环。

## 参数调整

所有参数位于现有 YAML 的 `estimation` 节，启动时读取；修改后重启策略节点。默认值是首版工程设置，尚非实车最优值。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `speed_tau_s` | 0.15 s | 速度滤波平滑程度和响应滞后 |
| `yaw_rate_tau_s` | 0.08 s | 偏航率滤波平滑程度和响应滞后 |
| `max_source_age_s` | 0.5 s | 最大源年龄，也用于中断后滤波重置；不能放宽框架期限 |
| `min_cone_confidence` | 0.5 | 最低可用检测置信度 |
| `road_forward_max_m` | 3.0 m | 使用的前向点范围上限 |
| `road_min_coverage_m` | 0.5 m | 边界及中心线最低有效覆盖 |
| `road_sample_spacing_m` | 0.1 m | 输出点目标间距；减小不增加测量精度 |
| `road_max_gap_m` | 1.5 m | 相邻有效点最大前向间隔 |
| `huber_delta_m` | 0.08 m | 鲁棒残差尺度；也关联离群剔除和 RMS 拒绝门限 |
| `road_width_min_m` / `road_width_max_m` | 0.4 / 2.0 m | 道路宽度合理范围，不是车身宽度 |
| `road_max_abs_slope` | 1.0 | 最大局部 abs(dy/dx) |
| `known_lane_width_m` | 0.0 m | 0 为未知，不允许单侧推断；需要实测或明确仿真宽度 |

先记录原始观测，再比较速度/偏航率抖动与滞后、道路残差、有效覆盖和拒绝原因。控制变量逐项调整，保留失败案例。不要仅通过放宽阈值追求更多 valid 输出。

## 验证与后续对接

在仓库根目录运行，无 ROS 的检查使用 Python 标准库：

```bash
python3 -B tests/test_estimation.py -v
python3 -B tools/study_estimation.py
```

15 项离线检查已通过，覆盖实际策略入口、重复样本、滤波时序、缺失/零值、道路拟合/异常点/单侧/退化、雷达可用性和 YAML 参数。合成比较为 5 个种子各 20 帧，0.03 m 横向噪声及每帧一个 ±0.8 m 异常点：100/100 帧有效，平均逐帧中心线 RMSE 约 0.0115 m，普通二次最小二乘约 0.0668 m。它仅验证该合成单帧场景，不代表实车或完整闭环表现；脚本打印包含源文件哈希的 JSON，可保存至 PR/CI 附件。

Ubuntu 24.04 / ROS 2 Jazzy 的完整 gate 按 `CONTRIBUTING.md` 和固定接口依赖版本执行。本次 Windows 环境缺少 ROS/colcon/rclpy 和已安装 WSL，完整 gate、修改过的 ROS 框架测试、Gym 闭环及实车实验均未运行；历史上游 gate 结果不算本版通过。

接入前需要规划、控制组核对字段、单侧/无效数据处理、几何采样时刻与车辆参考点。后续工作包括车辆参数辨识、实测探测范围、运动补偿、跨帧融合调查及联合验证。本分支提交首版模块供审阅，不包含这些尚未验证的功能。
