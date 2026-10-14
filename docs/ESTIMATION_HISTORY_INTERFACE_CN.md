# 历史覆盖保留与 Planning 接口

基线：`mvp2.0@f7cf825`。本地修复，尚未部署。

## 改动

连续道路模式（当前 YAML 已开启 `continuous_road_output`）下：

- 合并拟合失败保留此前接受的历史缓存，返回 `history_fit_failed`，按原有预测时间和距离限制尝试延续；不会把当前帧变成新的可信道路。
- 成功拟合的内点只决定本次道路输出，未入选的历史点不因此永久删除；过期、位置窗口、关联替换和真实运动失效仍管理缓存。
- 新旧观测冲突仅隔离对应颜色、重叠观测范围中的冲突点。其他区域保留，但冲突帧仍为无效，结束预测信任；重复同一相机帧不能恢复有效状态。
- 单条全局曲线无法保留真实支持时，使用重叠局部拟合，沿用已有最小覆盖、Huber 误差、斜率、宽度检查；不外推未观测区域。局部结果平滑加权后重新检查道路几何。
- 相机帧与历史无法运动对齐时明确返回原因，不再把对齐失败静默解释为“没有历史”。真实运动断档/重置仍丢弃无法定位的缓存。
- 旧 camera/time 模式保留原行为；本次不调整运行门限、速度或车辆参数。

## 保持的道路接口

| 字段 | 含义 |
|---|---|
| `valid` / `status` | 是否可用及失败原因；缓存存在不等于道路有效 |
| `frame_id` | `base_link`；坐标为米，+x 向前、+y 向左 |
| `timestamp_s` | 道路几何对齐后的自车状态参考时刻 |
| `measurement_timestamp_s` | 真实相机观测时刻；预测不伪造新采集时间 |
| `source_age_s` | 原始观测年龄，不是缓存更新年龄 |
| `centerline_xy` / `left_boundary_xy` / `right_boundary_xy` | 原有离散点格式，按 x 顺序输出；V2 仍做全车身与路径检查 |
| `lane_width_m` / `local_curvature_1pm` | 原单位和定义；分段曲率在刚体运动补偿后对应同一实际点 |
| `near_field.coverage_x_m` | 左右边界与中心线的共同覆盖；不能用单侧缓存范围替代 |
| `history_used` / `boundary_source` | 本次输出实际使用历史的情况，逐侧标明 observed/history/observed_with_history |
| `degraded` / `prediction_remaining_s` / `recommended_speed_scale` | 原有预测限制和减速建议；预测、重复帧不续期 |
| `forward_geometry` | 近端不足时保存的真实前方几何，仍不能被标成完整有效道路 |

内部拟合参数和逐点曲率不进入规划公共点数组，未改变 ROS 消息定义。

## 新增诊断

`road.history_diagnostics`：

- `frame_id`、`timestamp_s`：诊断的当前车身坐标与时刻。
- `cache.left/right.count`、`x_range_m`、`oldest_age_s`：已保存历史的点数、范围和原始年龄。
- `fitted_boundaries.left/right.x_range_m`：本次实际重建边界范围，使用对齐后的坐标；也适用于 forward_geometry。
- `common_coverage_x_m`：最终共同覆盖范围。
- `current_camera_boundaries`：仅当前相机帧的原始拟合诊断，不能当成融合范围。
- `alignment_problem`：诊断时无法定位历史的原因。
- `events`：本周期删除/拒绝原因与数量，包括 motion gap/reset、age expired、behind local window、overlap conflict、merged fit failed。

`reconstruction_method` 为 `global_observed_fit` 或 `segmented_observed_support`。

现有 Planning 拒绝日志增加 `road_history_diagnostics` 和 `road_reconstruction_method`；
异步 worker 的 `worker_road` 同步保留这两个字段。日志可区分“缓存已经没了”与
“缓存还在但拟合/双侧共同范围不包含车旁”。本次未添加相机图像录像或锥桶绘图话题。

## 拒绝与控制对齐

合并拟合失败可以使用已有的有界预测；明确的观测冲突、错误宽度/顺序、异常
分段斜率、乱序帧不能继续使用旧巡航参考。控制同时检查当前估计器输出和所选
worker 道路，防止较早的有效 worker 结果掩盖当前已知冲突。

## 验证

226 项离线检查通过：估计 102、规划 18、lattice 60、控制 46。包含真实规划
子进程接口检查；完整安装 ROS gate 因本机没有 Bash/WSL Linux/ROS 环境未运行。

新增回归覆盖：拟合失败后缓存与观测时间保留；冲突只隔离局部且重复帧无效；
曲线分段的覆盖与误差；多帧移动后后方支持；单侧历史不足不制造双侧覆盖；
历史对齐和运动断档拒绝；分段道路进入真实 V2 规划器及实际 worker JSON 接口；
当前硬冲突不能被旧控制缓存或较早 worker 结果遮蔽。

完整安装 ROS gate 和真实传感器/车辆运行仍未验证。保留缓存并不建立运动估计
漂移保证，也不保证任何场景始终覆盖车后固定距离。合并前需按仓库要求人工审阅。
