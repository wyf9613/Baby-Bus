# MVP 任务式雷达绕行 20261030 r5

版本`mvp21-lidar-maneuver-20261030-r5`从旧r4（171e488）独立修复，分支`ying/mvp21-lidar-maneuver-r5`。
新native版本（f39bf6d）和旧r4部署包均保留。本版继续使用MVP/PID与SHIFT/PASSING/RETURN独立绕行任务，未接入MPC或RNN。

## 本轮调整

仅放宽额外几何余量，不缩小车身，不删除完整车身扫掠、道路包含、曲率与规划失败检查。

| 参数 | r4 | r5 |
| --- | --- | --- |
| 车宽 | 30cm | 30cm |
| 障碍余量 | 3cm | 2cm |
| 跟踪余量 | 2cm | 1cm |
| 边界余量 | 2cm | 1cm |
| 额外绕行间隙 | 2cm | 2cm |
| 未观测横向包络 | 2cm | 2cm |

有路边LiDAR表面回波时，直道初始通道预算从约44cm降至约40cm；只是准入预算，旋转车身、实际路径和跟踪误差仍可能让一个更宽通道不可行。
无路边回波时约需39cm。候选通过实际扫掠检查后才交给PID，不能把预算当作实车标定净空。
保留2cm额外绕行间隙与2cm未观测横向包络：首次雷达可能只扫到障碍的一部分，全部缩小会令横向目标贴近障碍、后续新增回波反而把路径截停。
诊断新增`raw_measured_channel_m`、`required_raw_channel_m`、`extra_over_body_width_m`和`channel_deficit_m`，直接说明实测点间隙和缺多少空间。

## 两份配置

默认包保留原15cm车头前向停车。诊断包只将`obstacle_stop.proximity_stop_enabled`设为false，关闭这段提前停车区域，
`obstacle_stop.enabled`始终true：雷达源0.30s/接收0.20s时效、坐标/数据有效性、点进入当前车身范围、绕行路径碰撞检查仍会要求停车。
点进入车身的诊断原因是`lidar_body_overlap`。无路可绕或道路无效仍可能停车，关闭15cm门禁不会令失败路径变成可执行路径。
只读参数通过完整YAML及重启切换。启动仍为零动作，不自动Enable或Run。

两份包Python相同，只有YAML的`obstacle_stop.proximity_stop_enabled`不同。日志和`/car/planner_execution.obstacle_stop`同时显示开关状态及停车来源。
先以默认包对比native策略；若现场明确出现`Control: lidar_obstacle`并要排查15cm门禁，再使用诊断包。
已锁停需要人工Run，不因清空扫描自行恢复。禁止用`obstacle_stop.enabled=false`替代该诊断开关，否则源保护也会丢失且节点拒绝启用。

## 其余行为

左蓝2/右黄1、目标0.20m/s、固定左绕，车身前0.275m/后0.198m/宽0.30m保持。
普通巡线1Hz/1m native lattice与原MVP控制；任务激活后2Hz/2.5m专用参考。
沿用r4自适应横移长度：首选2m、最短0.8m、额外纵向余量10cm、80%曲率预算。
不重做道路估计；现有道路缺失、宽度未知和弯道限制仍存在。支持目标仍是简单偏侧静态障碍直道绕行。
两侧实体锥桶也占用通道；不得用虚构道路宽度或车宽隐藏几何失败。

打包默认档：`python3 tools/package_lidar_avoidance.py --output 新目录`。
打包15cm关闭诊断档：`python3 tools/package_lidar_avoidance.py --output 另一新目录 --no-proximity-stop`。
需ROS Jazzy、匹配dream_interfaces及python3-numpy；Python与完整YAML一起上传。

## 已有证据及范围

保留的r3现场日志有18次insufficient_common_range、11次single_side_width_unknown、7次too_close_to_start、2次no_side_clearance、1次lidar_obstacle。
这些是旧日志的次数，未收到r4现场记录，不能将它们当成r4的新诊断。
关闭15cm门禁只能区分最后一类原因，不能修复前三类。

本轮80cm直道、25×25cm障碍靠右2cm、两侧16cm锥桶占位、2.28125m雷达量程的观察式闭环，默认与诊断配置都完成绕行/回中，未记录接触。
仿真驱动映射和负驱动制动只用于模拟器，不写入部署YAML。实车刹停距离和真实转向能力尚未测量确认。
完整检查结果随部署包旁的验证记录提供；没有进行车辆连接、上传或实车运行。
