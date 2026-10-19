# 控制参考与影子规划轨迹

2026-10-20 新增 **Local grid** 标签页：在现有 `layout.json` 基础上增加俯视地图，
保留原有控制按钮、道路图和其他标签。重新导入此文件后切换到 Local grid。
地图为车体坐标前方 `x=[0,3)`、`y=[-1.5,1.5)`，5 cm 一格，即 60×60 格。
固定参考系为 `base_link`；同时显示原始雷达点及绿色/蓝色 Path，路径和原始扫描
仍各自保留自己的观测时刻，叠加视图不代表严格同步的定位测量。

| 话题 | 类型 / 含义 |
| --- | --- |
| `/car/local_occupancy_grid` | `nav_msgs/msg/OccupancyGrid`；-1 未知、0 已观测空闲、100 命中 |
| `/car/local_inflated_grid` | 同类型；增加 90 表示膨胀余量，原命中仍为 100 |
| `/car/local_grid_status` | `std_msgs/msg/String` JSON；状态、观测年龄、时间、耗时及占据格数 |

图层使用 Foxglove 内置 Costmap 配色。原始图默认可见，膨胀图默认关闭；
在 3D 面板话题设置中切换两者，避免重合图层遮住颜色。膨胀半径默认 0.10 m，
**只包含余量，不包含车辆长宽**，规划端应检查完整车身并明确处理未知/过期数据。
无论策略是否处于 Run，地图都独立以 10 Hz 更新；启动雷达和策略即可停车看单帧。
要看随车运动对齐，需启动已有轮速及 IMU gyro；无需启动车辆自主动作。

`local_grid` 只有 enabled、resolution_m、history_s、inflation_radius_m 四个设置。
有限且在原扫描距离范围内的点参与射线清除和命中标记，没有聚类数量或置信度门限。
每个扫描时间戳只融合一次，同帧命中优先于清除，后帧空闲可清除旧障碍；
命中后方、无效/无返回射线不凭空清空。射线从雷达扫描原点出发，与端点一起运动对齐。
默认重建最近 0.3 s 的扫描历史；未重观测的旧证据退出历史后恢复未知。

状态 `ALIGNED` 表示对齐到地图消息的当前车体时刻；`SCAN_ONLY_NO_MOTION` 表示
只有最新单帧，消息时间是原采样时间，`time_aligned=false`。
`SCAN_OR_TF_UNAVAILABLE` / `MAP_ERROR` 时两个图全部变为未知。
雷达过期复用原有 `sensor_timeout_s.lidar_scan/lidar_cartesian`，不增加停车门限。
控制状态重置或 ROS 时钟回退会丢弃跨坐标代次的扫描历史。
二维扫描空闲只描述雷达扫描截面；本版没有逐束去畸变、目标速度估计或自动车身回波屏蔽。

新增 17 项离线几何/适配检查，另有 2 项真实 ROS 消息检查接入原有 ROS gate。
本机只有 Windows，尚未运行 ROS gate、实际 Foxglove 导入或车端测试；离线结果见
[验收记录](../../docs/acceptance.md)。

---

导入同目录 `layout.json`，在原来的 `Cone locations` 图里显示一条绿色轨迹。
话题为 `/car/policy_trajectory`，类型是标准 `nav_msgs/msg/Path`。
布局直接读取 `poses[:].pose.position.x/y`，无需 Foxglove 消息脚本。

2026-10-15：同一个图已加入蓝色 `Shadow planner trajectory`，读取
`/car/shadow_trajectory`（`nav_msgs/msg/Path`），颜色为 `#3b82f6`。
在 Foxglove 的 Layouts 菜单选择 Import from file，重新导入本目录 `layout.json`。
车端需要部署当前 mvp2.1 影子实现，并以 `shadow.enabled: true` 启动；规划频率
由 `shadow.rate_hz` 决定。蓝线显示有效的影子选中轨迹，停止或无有效选中轨迹时清线。
绿色表示控制器实际采用的参考；启用 planner_control 后也可为当前对齐裁剪的 planner
参考，降级时回到基线。实际来源见 /car/planner_execution，详见
[接入与录制说明](../../docs/PLANNER_CONTROL_CN.md)。原布局添加时按用户要求未进行本机测试
或车端部署；以下记录属于原基线绿色参考的实现与历史验证。

车端只记录控制器已经采用的参考路径，以最多 10 Hz 发布；中心线使用原始点，
其他路径调用该版本已有的路径求值函数。每条线最多 201 点，保留参考路径的
车体坐标系和原始时间戳。已有停止/参考过期处理触发时发布空 Path 清线。
停车时没有预览，不增加估计规划计算，不创建线程，不发布额外控制动作。
启动只读参数 `visualization.enabled` 默认为 true，设为 false 后不发布轨迹。

部署需要把可视化增量加到车上当前 `scripts/policy_node.py`，并同步
`package.xml` 中的 `nav_msgs` / `geometry_msgs` 运行依赖声明。
确认车端已有这两个标准 ROS 消息包，然后按原流程构建、重启策略；电脑导入布局。
补丁已接入 `fix/newcar-control` 并部署到 15 号车，用户已确认运行时绿线可见。

完整 ROS gate 通过：241 pytest cases / 246 colcon checks，零错误、失败或跳过。
最小运行时代码补丁为新增 61 行（含注释与空行），原算法代码没有删除。

2026-10-14 追加检查：与 15 号车已部署文件比较，确认为 61 行新增、0 行删除；
估计设置/估计器/规划器/控制器的 AST 均相同。补充缺黄与缺蓝两个真实 ROS
订阅测试，收到的 Path 点序与控制器选用的单侧偏移轨迹完全一致，停止时收到空 Path。
最终完整 gate：248 项，零失败/错误/跳过，日志
`.verification/car15-trajectory-review-fast-gate.log`；对应文件哈希见
`.verification/car15-trajectory-review-manifest.json`。
当时通过只读 SSH 确认 `10.43.254.28` 上两个消息类型可导入，尚未部署。
之后用户授权部署和试跑；`visualization2` 收到 63 条非空 Path，停车清线通过，用户确认可见。
该轮因双侧共同范围不足停止；已补近侧回退，最新完整 gate 为 250 项。
下一轮用户报告早段碰蓝桶，诊断发现实时 steering trim=0.5；已恢复为 0.0。
零偏置复测运行 19.746 s / 3.817 m，用户反馈效果满意；收到 147 条非空 Path，最多 34 点。
本轮未确认停车空线接收，保留该显示验证缺口；驱动归零/Disabled 已确认。
源代码及实车结果见 `docs/acceptance.md`。

布局的各曲线 X-value 独立设置符合 [Foxglove Plot 文档](https://docs.foxglove.dev/docs/visualization/panels/plot)。
绿线表示控制器当前采用的参考路径；需启用策略并有有效参考才显示。
代码在停车时发送空 Path，合成 ROS 测试和 visualization2 已验证；后两轮实车记录未确认清线。
初次本地审查未导入 GUI；后续用户已在 Foxglove 确认绿线。锥桶位置来自相机采集时刻，参考线对齐
控制时刻；运动中两个最新消息的坐标时刻不同，因此点/线叠加适合观察参考形状，
不能直接作为外部定位精度测量。现有图的 Y 范围为 ±1 m，较远急弯可能超出视窗，
可在 Plot 的 Y-axis 中调整显示范围。
