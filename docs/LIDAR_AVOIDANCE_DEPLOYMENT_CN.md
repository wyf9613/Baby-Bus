# MVP 雷达绕行任务版 20261028 r1

版本标识 `mvp21-lidar-maneuver-20261028-r1`。本包是独立的部署候选，未上传或启用车辆。
旧20261026-r1/r2包保留；本版实现的是新的绕行任务管理，不是仅调整旧A2门限。

## 本版行为

- 正常巡线继续由现有native lattice产生参考，MVP控制；没有RNN，也没有MPC。
- 有挡路目标时进入SHIFT→PASSING→RETURN→NORMAL。固定局部道路坐标、任务ID和横向目标；专用轨迹独立于普通巡线候选。
- 第一版固定左绕，`lidar_avoidance.maneuver_side=1`；-1为固定右绕。空间不足停车。
- 环境回波只参与碰撞检查；绕行完成由目标障碍与旋转后车身最后端决定，不等待所有雷达点消失。
- 单个有效回波也可建立保守目标包络。`target_depth_allowance_m=0.30`是未观测深度假设，`target_lateral_allowance_m=0.10`是横向额外包络，不是测量出的障碍尺寸。目标明显更大时需重新设置；本版针对单个小型静态障碍。
- 横移和回正长度各至少2m，目标速度0.20m/s；初次任务开始得太晚时停车，不强行生成急转轨迹。
- 原近距离雷达停车优先，15cm车头间隙、源0.30s／接收0.20s保持。源失效、无路、路径过期、控制回退不安全时停车，显式Run恢复。
- 20Hz控制、2Hz规划、2.5m局部路径；每个控制周期检查实际参考，当前障碍及有时限的道路边界继续约束执行。原计划年龄1.5s，不续期测量来源。
- 道路侧边界可暂用已经观测过的缓存，原测量年龄上限0.65s，过期拒绝；首次没有有效道路支持不会凭空补路。
- 引入Zaiyang的独立开关，部署配置`shadow.fusion_use_prior=false`；保留其他历史帧融合。

## 使用场景与配置

首次使用建议宽直道，约1.8–2m宽，留足横移／通过／回正距离，障碍距起步位置约3.5–4m。道路边界需要可观测。该范围是本版开发场景，不是实测可通行保证。

配置固定左蓝2／右黄1。车身沿用原停车配置前0.275m、后0.198m、宽0.30m，仍是未绑定具体车辆的暂定值。没有更改硬件TF、车辆驱动或转向标定。

包内两份文件必须一起安装：
- `policy_node.py` → `~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py`
- `ai4r_policy.yaml` → `~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml`

保持车辆Disabled、停止旧policy，保留原文件后按既有DREAM流程构建启动：

```bash
chmod +x ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py
dream build ros student && dream runtime start ai4r_policy
```

启动保持state2／零动作，不自动Enable或Run。ROS Jazzy、匹配的dream_interfaces和python3-numpy仍是运行依赖。

核对实际配置：
```bash
ros2 param get /car/ai4r_policy lidar_avoidance.maneuver_enabled
ros2 param get /car/ai4r_policy lidar_avoidance.maneuver_side
ros2 param get /car/ai4r_policy estimation.left_colour
ros2 param get /car/ai4r_policy estimation.right_colour
```
预期true、1、2、1。`planner_execution.lidar_avoidance.maneuver`包含任务、阶段和目标；`first_failure`保留本次运行第一次拒绝原因。

## 证据边界

本轮做了任务行为、实际worker和ROS节点检查，并在观测传感器驱动的闭环仿真中完成左绕、通过和回中。验证记录另附，不能据此宣称实车绕行成功。仿真使用显式纵向模型适配，现场包保留原实车drive配置，不发送仿真的负drive制动。

尚未证明动态障碍、弯道、多个连续障碍、未知尺寸障碍、车端最坏耗时及实际停车距离。原始失败试验仍保留，未覆盖为成功。

重新打包：`python3 tools/package_lidar_avoidance.py --output 新目录`。
