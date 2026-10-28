# MVP 雷达窄道绕行任务版 20261028 r4

版本标识 `mvp21-lidar-maneuver-20261028-r4`。本包是独立的部署候选，未上传或启用车辆。
旧20261026-r1/r2包保留；本版实现的是新的绕行任务管理，不是仅调整旧A2门限。

## 本版行为

- 正常巡线继续由现有native lattice产生参考，MVP控制；没有RNN，也没有MPC。
- r4修复固定2m横移要求导致的提前拒绝。`adaptive_shift_enabled=true`后，2m是首选长度；根据当前横移需求和障碍前剩余距离选择长度。保留车头、3cm障碍余量之外的额外10cm纵向余量，采用80%曲率预算，最短长度0.8m；实际曲率、车身碰撞和道路包含检查仍由worker执行。距离真正不足仍停车。
- r3面向用户描述的80cm直道、约25cm障碍靠右布置。绕行净空按障碍通过区间的局部边界计算，不再取整条可见道路的全局最窄矩形；存在路边锥桶回波时，同时使用其表面位置缩小可用通道。
- 在左侧可行中心位置区间内选择较小的横移目标，并保持该目标；不强制移到宽道路的中点。完整车身扫掠、局部边界内包含检查与跟踪余量继续约束专用路径。看不到的后续道路不外推，后续规划逐段重查。
- 修复r1实车中约2.2秒运行后因`avoidance_planner_stale`锁停：NORMAL不再对仍通过原执行检查的路径施加1.5秒原始输入期限。当前状态、道路、预瞄覆盖、雷达时效和原停车检查继续生效。
- NORMAL恢复1Hz、1m、prefix碰撞／评分范围，不再用绕行的额外8cm车身余量圈截停普通巡线。实车记录的左后方点不会单独触发新增guard；原停车区域内的点仍由原模块处理。该修改不删除后方点，进入绕行后完整车身扫掠仍会检查它们。
- 有挡路目标时进入SHIFT→PASSING→RETURN→NORMAL。固定局部道路坐标、任务ID和横向目标；专用轨迹独立于普通巡线候选。
- 第一版固定左绕，`lidar_avoidance.maneuver_side=1`；-1为固定右绕。空间不足停车。
- 环境回波只参与碰撞检查；绕行完成由目标障碍与旋转后车身最后端决定，不等待所有雷达点消失。
- 单个有效回波也可建立目标包络。`target_depth_allowance_m=0.30`是未观测深度假设，r3的`target_lateral_allowance_m=0.02`是横向额外包络，不是已知完整尺寸。新增回波扩大目标后仍会重新检查碰撞。
- 窄道实验配置：障碍余量3cm、跟踪余量2cm、边界余量2cm、额外绕行间隙2cm（r3为1cm）、目标未观测横向包络2cm。均为低速静态实验参数，未经过车辆测量标定；不保证80cm道路上任意摆放的25cm障碍都可绕行。旧A2行为与原停车参数未改。
- 横移首选2m，允许在曲率预算内缩短；回正仍为2m，目标速度0.20m/s。无法在剩余距离内生成符合曲率与碰撞要求的路径时停车。
- 原近距离雷达停车优先，15cm车头间隙、源0.30s／接收0.20s保持。源失效、无路、绕行任务路径过期、控制回退不安全时停车，显式Run恢复。
- 20Hz控制；仅在任务激活后切到2Hz规划、2.5m局部路径，配置为`lidar_avoidance.maneuver_plan_rate_hz`和`maneuver_plan_length_m`。绕行任务每周期检查实际参考、当前障碍与道路边界，并保留1.5s原计划期限。NORMAL不使用该任务期限。
- 道路侧边界可暂用已经观测过的缓存，原测量年龄上限0.65s，过期拒绝；首次没有有效道路支持不会凭空补路。
- 引入Zaiyang的独立开关，部署配置`shadow.fusion_use_prior=false`；保留其他历史帧融合。

## 使用场景与配置

使用80cm左右的可观测直道，障碍靠右且左侧保留实际通道。闭环开发场景为25cm宽、25cm长矩形障碍，距右边界约2cm、起步前方4m；路边加入8cm半宽的方形锥桶替身供雷达与接触检查，并限制实验雷达量程来覆盖较晚检测。该场景不是现场尺寸测量或实车可通行保证。r4不再使用统一2.305m启动阈值；是否能开始由实际偏移、曲率、剩余纵向距离和净空共同决定。未增加已知道路宽度先验。

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
预期true、1、2、1。`planner_execution.lidar_avoidance.maneuver`包含任务、阶段和目标；`admission`保留目标包络、局部边界、锥桶表面限界、可用/所需净空及选定偏移。r4新增`shift_geometry`，记录横移的可用长度、最低需求、选定长度、10cm纵向余量和曲率预算。`first_failure`保留第一次拒绝时的这些信息。

## 证据边界

r4的ROS gate、较晚检测窄道闭环和实际单文件检查结果见`docs/baselines/lidar_maneuver_20261028_r4.json`。包含日志中约2.28m起始距离的几何回归、真正不足的距离拒绝，以及普通巡线/原停车回归。不能据此宣称实车绕行成功。仿真使用显式纵向模型适配，现场包保留原实车drive配置，不发送仿真的负drive制动。

本次没有修改MVP控制器、普通巡线规划或单侧道路推断；现场转弯跟踪问题尚未修复。尚未证明动态障碍、弯道绕行、多个连续障碍、未知尺寸障碍、车端最坏耗时及实际停车距离。原始失败试验仍保留。

重新打包：`python3 tools/package_lidar_avoidance.py --output 新目录`。
