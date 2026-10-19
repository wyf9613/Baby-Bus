# 参考轨迹与雷达障碍显示

## 当前雷达分支

重新导入同目录 `layout.json`，原 `Cone locations` 图变为 `Cones, reference and lidar obstacles`。
不需要 Foxglove 自定义消息脚本；控制按钮和其他面板保持不变。

| 图形 | 含义 | ROS 话题与类型 |
| --- | --- | --- |
| 紫点 | 相机锥桶 | 原 `/car/cone_detections` |
| 绿线 | 控制器当前参考路径 | `/car/policy_trajectory`，Path |
| 红点 | 检测区内的障碍簇回波 | `/car/lidar_obstacle_points`，PoseArray |
| 橙点 | 未确认的零散回波，同样触发停车 | `/car/lidar_obstacle_candidates`，PoseArray |
| 灰框 | 配置的停车检测区域，不代表安全 | `/car/lidar_stop_roi`，Path |

新增话题是标准 `geometry_msgs/msg/PoseArray` 和 `nav_msgs/msg/Path`，已有依赖无需增加。
PoseArray 的 XY 字段是 `poses[:].position.x/y`，Path 则是 `poses[:].pose.position.x/y`。
每条曲线分别设置自己的 X-value；点不连线，检测框闭合连线。

新扫描和检测状态变化时更新；停车时仍显示当前障碍。无障碍或雷达过期时发送空点集清旧点，
灰框保持不变。完整状态和原因仍查看 `/car/lidar_obstacles` 或策略状态字符串。
没有红点不一定安全：橙点、雷达未知、断连都不能当作畅通。若节点/网络完全断开，
界面可能保留最后图像；必须结合连接状态和时间戳判断，图形不作为车辆安全联锁。

`visualization.enabled: false` 关闭路径及障碍图形，但**不关闭检测和停车**；参数修改后重启。
先同步当前分支的 `scripts/policy_node.py` 与 `config/ai4r_policy.yaml`，按原流程重建、重启策略，
再导入布局。没有自动传输或部署到实车。验证范围见 `docs/acceptance.md`。

原路径与相机、雷达点保留各自测量/参考时刻，不是同步定位真值；运动时叠加只供调试。
当前布局沿用已有 Foxglove 消息路径格式，实际版本的 GUI 导入与清点效果尚未验证。

## main 原有轨迹模块与历史验证

以下是原轨迹模块的历史记录，不是本次雷达显示的上车验证。

导入同目录 `layout.json`，在原来的 `Cone locations` 图里显示一条绿色轨迹。
话题为 `/car/policy_trajectory`，类型是标准 `nav_msgs/msg/Path`。
布局直接读取 `poses[:].pose.position.x/y`，无需 Foxglove 消息脚本。

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
