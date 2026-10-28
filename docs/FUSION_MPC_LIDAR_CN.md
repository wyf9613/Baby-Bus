# 最新 MPC + 雷达检测

分支 `feature/fusion-mpc-lidar-detection`，基于队友 `fusion-prior-reset` 的 `d1b4b83`。
运行标识：`fusion-mpc-lidar-detect-only-20261028`。该功能分支保存检测接入和配套部署配置；
助手未上传车辆，实车验证尚未确认。

## 行为

- 检测车头前1.4m、左右各0.4m：`base_link` 下 x=[0.297,1.697]m，y=[-0.4,0.4]m。
- 沿用原雷达过滤与聚类：无效点处理、扫描索引、相邻点聚类及安装TF转换。
- 默认 `lidar_obstacle.stop_enabled=false`。命中、稀疏或缺失雷达均只报告，不拒绝启动、不停车。
- 不增加雷达路径碰撞检查、评分、执行中复查、恢复门槛或传感器必需项。
- 最新分支使用原生规划+MPC，没有旧RNN评分器；本次未移植RNN，也未实现主动绕行。
- 队友的估计、规划、MPC纯算法保留，包括短暂几何异常处理与保留已学路宽功能。
  这两项按上游要求在相应部署配置中启用；已有非雷达停车原因仍可能触发。

原始扫描与笛卡尔数据匹配且有效时处理一次，保留真实采集时间和车体坐标。
无新消息、无效扫描或转换失败后报告UNKNOWN、清除显示点，不使用旧点伪装新观测。
`allow_no_return_clear=true` 沿用此前约定：+inf在量程允许时是假定无回波，不是测得自由空间。
红点为ROI内雷达回波，可能包括雪糕筒/路边，不是语义分类后的独立障碍物。

## Foxglove

导入本地已重命名的 `tools/foxglove_road_visualization/layout_v2.0.json`。同一图保留锥桶、绿色控制参考、
橙色MPC预测，新增红色雷达点和灰色ROI框，无需Foxglove用户脚本。

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/car/lidar_obstacles` | std_msgs/String JSON | 状态、ROI、聚类、点、采集时间、stop_enabled/mode/stop_requested |
| `/car/lidar_roi_points` | sensor_msgs/PointCloud | 原始扫描采集时刻的ROI回波；无效/过期时为空 |
| `/car/lidar_roi` | nav_msgs/Path | 闭合检测框；不是控制路径 |

默认应显示 `mode=detect_only`、`stop_requested=false`。传感器和显示在策略未Run时也工作。
锥桶、雷达与控制参考的采集/计算时间不同；运动中的同图显示不等于外部定位验证。
如未来显式开启 `stop_enabled=true`，恢复原雷达停车并要求手动Run；零驱动不等于主动制动。

## 部署文件

上游根 `config/ai4r_policy.yaml` 是基础层，不能直接当作已启用MPC的部署配置。
仓库程序为 `scripts/policy_node.py`，配套完整配置保存于
`checkpoints/fusion-mpc-lidar-car14-20261028/ai4r_policy.yaml`，哈希清单位于同目录。
`.verification/fusion-mpc-lidar-deploy/` 是本地部署缓存，不纳入Git；队友使用仓库中的两份文件。
配置基于队友保存的 `v050-5m-noprior` 包：0.5m/s、黄左1/蓝右2、joint纵向、
关闭融合旧参考；叠加最新几何异常/保留路宽开关及本次雷达设置。
按操作者要求，部署包的 `control.max_distance_m=0.0`、`control.max_run_time_s=0.0`，
解除5m/60s预算。0为上游已支持的“不限”，未删除代码或改变其他停车原因。
本次没有沿用旧分支的2m候选或0.55rad接入门槛，也不修改车辆接口/安装配置。
该包沿用队友的车辆参数，不表示车14已重新标定。

上车上传仓库程序和checkpoint配置，目标仍分别为
`scripts/policy_node.py` 和 `config/ai4r_policy.yaml`，重建后只重启policy。
电脑导入的Foxglove布局使用本分支的新文件；本次未执行SSH或实车操作。

小车启动后核对：

```bash
ros2 param get /car/ai4r_policy planning.cruise_speed_mps
ros2 param get /car/ai4r_policy mpc.geometry_glitch_tolerable
ros2 param get /car/ai4r_policy estimation.keep_width_on_geometry_glitch
ros2 param get /car/ai4r_policy lidar_obstacle.stop_enabled
ros2 param get /car/ai4r_policy control.max_distance_m
ros2 param get /car/ai4r_policy control.max_run_time_s
timeout 5 ros2 topic echo /car/lidar_obstacles --once --full-length
```

配套部署包预期依次为0.5、true、true、false、0.0、0.0，以及 `mode=detect_only`。

## 验证

254项估计/规划/控制/原生lattice/执行/颜色/雷达回归，加1项MPC短帧几何异常测试通过。
11项新增雷达检查包含区域、左右符号、旋转TF、清空旧点、静止显示、丢流不停车、
控制动作等价、可选停车恢复、部署配置继承，已包含在254项中。
AST比较确认上游101个纯定义未变；基础配置仅增加雷达组，原Foxglove控制按钮和曲线保留。
完整ROS gate因本机缺少Bash/ROS Jazzy无法执行；未实际连接Foxglove GUI或进行实车验证。
