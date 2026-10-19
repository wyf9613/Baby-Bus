# 雷达检测与遇障停车 v0.1

基线：main `0c8593a`。交付分支 `feature/lidar-obstacle-stop`，不合并或覆盖 main。
保留 main 的道路估计、20 Hz 循迹控制、速度目标 0.2 m/s 和车辆配置。
本模块只检测与请求停车，不绕行、不启用车辆、不发送倒车/主动制动。

## 检测范围

在 `base_link` 下：x 向前、y 向左，原点在重心下方。

- 宽：车宽 0.25 m，左右各加 0.05 m，共 0.35 m。
- 长：从车头向前 1.0 m，即 x=[0.297, 1.297] m。
- 几何沿用之前辨识车的记录，**当前车需重新核对**；1.0 m 是暂定检测距离，不是已验证停车距离。
- 使用雷达安装 TF 转换后的点，不把原始零度当车头方向。当前标准安装 yaw=180°。

配置在 `config/ai4r_policy.yaml` 的 `lidar_obstacle`；修改后重建并重启 policy。
`required_sensors` 新增 `lidar_scan`、`lidar_cartesian`。两者超时仍为 0.5 s。
首版要求水平全周扫描、雷达位于车头后方；其他安装/部分扫描输出 UNKNOWN。

## 处理与停车

1. 复用框架的距离有效性过滤和坐标转换；不修改原始 `/car/scan`。
2. 裁剪前方矩形；车身及车后点不作为前方障碍，但遮挡回波不能证明前方畅通。
3. 按原始索引相邻且空间接近的点聚类；不跨无效射线连接，不做平滑抹掉障碍边缘。
4. 至少 2 点形成障碍；不足 2 点仍保留为可疑点并请求停车，不默默删除。
5. 前方有效射线比例不足 70% 输出 UNKNOWN。NaN、无穷远、量程不足、被近物遮挡不算有效穿入检测区。

| 状态 | 含义 | 动作 |
| --- | --- | --- |
| CLEAR | 本次检测区未检出点，且满足最低回波要求 | 允许原控制器继续工作，不代表安全认证 |
| OBSTACLE | 检测区存在障碍簇 | 状态 2，持续零驱动、零转向 |
| UNKNOWN | 可疑单点、缺数据、超时、TF 失败或不支持的扫描 | 同样锁存状态 2 |

每个新扫描只检测一次；独立监督定时器持续检查新鲜度，停车不依赖下一次控制计算。
障碍移开不会自动恢复。重新请求策略状态 3 时，仍需通过雷达和其他传感器检查。
策略状态 1、车辆 Enable/Disable 和遥控权限仍沿用框架语义。

## 交付接口

- 输入：`/car/scan` (`sensor_msgs/msg/LaserScan`) 与原有 TF。
- 纯函数：`detect_lidar_obstacles(scan, cartesian, settings, stamp_ns)`，在 `scripts/policy_node.py`，无 ROS 依赖，可供其他组复用。
- 内部结果：`self.lidar_obstacle_output`。不覆盖估计模块已有的 `estimation_output['obstacles']`。
- 输出：`/car/lidar_obstacles`，`std_msgs/msg/String` 内含 JSON；按新扫描/状态变化发布，不是心跳。
- 字段：`status`、`reason`、`available`、`frame_id`、`measurement_timestamp_s`、`source_age_s`、`roi_xy_m`、`points_xyz_m`、`scan_indices`、`clusters`、`candidate_clusters`、`known_fraction`。
- 每簇包含点数、点集、索引、中心和 `bbox_xy_m=[x_min,x_max,y_min,y_max]`。
- `nearest_bumper_distance_m` 是最近点距车头的 **x 方向间隙**，不是欧氏距离。

所有距离为米、时间为秒。时间戳是原扫描第一条射线的采集时间，不因复用而刷新。
接收方必须检查原时间戳的新鲜度；消息停止时不能一直沿用最后一次 CLEAR。
原始估计器障碍接口仍含全量点；下游若要用裁剪点，须显式选择本模块结果。

## 核对与试验

Foxglove：导入 `tools/foxglove_road_visualization/layout.json`，同图显示紫色锥桶、绿色参考路径、
红色障碍点、橙色可疑点和灰色检测框。路径只在有效控制时显示，障碍在停车时仍更新。
新增显示话题为 `/car/lidar_obstacle_points`、`/car/lidar_obstacle_candidates`（PoseArray）及
`/car/lidar_stop_roi`（Path）。`visualization.enabled` 仅控制图形，不关闭雷达停车。
详细显示规则见该布局目录的 README；暂无 GUI/实车显示验证。

先保持车辆 Disabled，仅检查结果：

```bash
ros2 topic echo /car/lidar_obstacles --once
ros2 topic echo /car/policy_fsm_state_string --once
```

依次在车前、侧方放障碍，核对状态、点位和距离；再检查缺失 TF/断流为 UNKNOWN。
核对车身尺寸、雷达朝向/高度、回波有效率后，再由操作者按原 Foxglove 流程低速启动。
在空旷场地实测触发距离、实际滑行距离、移开障碍不自启及手动恢复，准备遥控停用。

限制：二维扫描可能漏掉低矮/悬空/不反光物体；不识别障碍类别，雪糕筒也可能触发。
不进行逐射线运动补偿，不覆盖弯道扫掠区、侧碰或动态障碍预测。
零驱动可能滑行；固定 1 m 区域及 0.5 s 超时**不保证所有速度下能及时停住**。

离线：`python3 -B tests/test_lidar_obstacles.py`。ROS 构建/消息/回调验证用 CONTRIBUTING 的固定依赖 fast gate。
当前验证证据与未完成项见 `docs/acceptance.md`；未进行上车测试。
