# 最新 MPC + 雷达绕行与窄缝排除

分支 `feature/fusion-mpc-lidar-detection`，基于队友 `fusion-prior-reset` 的 `d1b4b83`。
运行标识：`fusion-mpc-lidar-path-continuity-20261028`。该功能分支保存检测、绕行和配套部署配置；
助手未上传车辆，实车验证尚未确认。

## 行为

- 检测车头前1.4m、左右各0.6m：`base_link` 下 x=[0.297,1.697]m，y=[-0.6,0.6]m。
- 沿用原雷达过滤与聚类：无效点处理、扫描索引、相邻点聚类及安装TF转换。
- `lidar_obstacle.stop_enabled=false`：检测器的“区域内有点即停”仍关闭；规划无可行候选时停车。
- 完整部署配置启用 `shadow.obstacle_avoidance_enabled=true` 和 `shadow.obstacle_check_enabled=true`。
  自动比较左右候选；保留48条采样、1.4m路径长度、1Hz规划和MPC。
- 当前部署设置 `shadow.min_turn_radius_m=0.9`，在候选生成后做曲率受限的前向追踪，
  将几何最小转弯半径设为0.9m（曲率上限约1.11/m）。原有道路/雷达检查及评分使用处理后的路径，不在检查后改形状。
  参数默认0关闭；这是候选几何约束，不是实车转弯半径标定，也不保证纵向驱动平滑。
  暂不缩短1.4m路径，以保留平缓转向和绕障空间；MPC预测时域仍0.6s、目标仍0.5m/s。
- 新增 `shadow.weight_consistency=4.0`、`continuity_window_m=0.6` 和 `switch_cost_margin=0.05`。
  将正在执行的路径运动补偿到本轮坐标，比较前0.6m的位置/航向变化；不修改候选几何。
  先做本轮道路/雷达检查，再在合格候选内评分。最优候选相比“形状最接近旧路径的合格候选”
  改善不足0.05分时优先后者。0.05为综合评分差，不是距离；不锁住旧轨迹，不恢复fusion旧引导。
  无历史、运动epoch不匹配、历史过旧或剩余覆盖不足时不加此偏好，仍常规选路。
- `mpc.r_ddelta` 从2.0改为4.0，增大转向变化惩罚；MPC算法、纵向权重与速度目标不改。
- 障碍项使用估计器运动补偿到当前状态时间的ROI点，覆盖整条生成路径。
  使用现有shadow车身矩形，空间索引减少远点计算；点可能包含锥桶/路边。
  沿路径计算矩形有符号距离，取接近惩罚的峰值加平均暴露量；半车宽作为软衰减尺度。
  重复回波不叠加惩罚。`shadow.weight_obstacle=30.0`，增大会更偏好远离障碍；0关闭该代价。
- 最小直行净宽按规划检查总宽0.30m，`shadow.obstacle_margin_m=0.0`，不另加宽度余量。
  规划时用完整旋转车身扫掠观测点，重叠候选不可行；转弯可能需要超过0.30m的空间。
  原道路边界算法保留，`shadow.collision_coverage=generated` 改为检查整条候选路径。
- 有对齐障碍点时，未得到通过通行性检查且能被MPC接收的路径，保持零输出并继续规划，
  `/car/planner_execution` 显示mode=waiting；不会退回未检查障碍的中心线。
  确认所有候选不可行时进入状态2持续零输出，手动Run才能恢复。零输出不是主动制动保证。
- 已有检查通过的活动路径时，短暂的道路排序/几何/覆盖缺失可限时续用。
  沿用 `mpc.geometry_glitch_tolerable=true` 和 `mpc.reference_dropout_tolerance_s=0.5`；
  缓存道路边界按轮速/偏航运动补偿，保留原观测时间，仍检查运动有效性、剩余长度和车身边界。
  mode仍为planner，reason=`planner_tracking_road_dropout`，不插入零输出、不重置MPC。
  NO_ROAD结果保留原路径ID/锚点/时间，不刷新期限；超过0.5s进入waiting，reason=`road_dropout_timeout`。
  道路恢复有效后恢复常规检查。未检查路径、运动断档、坐标epoch改变或明确碰撞不走此容忍分支。
- 空扫描时生成的路径没有规划期障碍检查标记。此类路径使用当前对齐扫描与剩余路径的车身扫掠复查，
  不挡路的回波不再触发waiting、回正或清空MPC；标记不提升为规划期已检查，后续扫描仍复查。
  扫掠重叠时等待重规划，确认所有候选不可行仍锁存停车。
- 无雷达、过期或无法运动对齐时，仍沿用原无障碍处理，不新增传感器必需项。
  MPC算法、其他原有停车逻辑保留；没有旧RNN，实车净空与停车距离仍待验证。

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

检测话题仍显示 `mode=detect_only`、`stop_requested=false`，表示雷达专用停车关闭；
主动绕行是否参与评分看 `/car/shadow_planning` 的 `obstacle_avoidance`。
`hard_rejection=true` 表示启用车身通行性检查；顶层 `stop_required=true`、
`reason=no_passable_obstacle_path` 表示所有候选不可行。检测器的stop_requested=false不代表规划不会停车。
`status=ACTIVE` 表示有对齐的ROI点；CLEAR为空、UNAVAILABLE为未取得可用对齐数据。
选中路径的原始/加权障碍代价为 `selected_cost` / `selected_weighted_cost`；
`selected_nearby_clearance_m` 是整段采样车身相对邻域点的最小有符号净空，负数为点落入矩形，
null表示邻域无点，不代表测得自由空间。所有候选的代价位于 `candidates[].costs.obstacle`。
绿色线显示实际接受的控制参考；mode=planner表示已接入，waiting表示已知障碍下等待检查通过的路径。
`/car/shadow_planning` 的 `path_continuity` 显示旧路径偏好是否可用、原最优/近似旧形状候选ID、
`incumbent_cost_gap`、`hysteresis_applied` 和一致性代价。无需改Foxglove布局。
传感器和显示在策略未Run时也工作。
锥桶、雷达与控制参考的采集/计算时间不同；运动中的同图显示不等于外部定位验证。
如未来显式开启 `stop_enabled=true`，恢复原雷达停车并要求手动Run；零驱动不等于主动制动。

## 部署文件

上游根 `config/ai4r_policy.yaml` 是基础层，不能直接当作已启用MPC的部署配置。
仓库程序为 `scripts/policy_node.py`，配套完整配置保存于
`checkpoints/fusion-mpc-lidar-car14-20261028/ai4r_policy.yaml`，哈希清单位于同目录。
`.verification/fusion-mpc-lidar-deploy/` 是本地部署缓存，不纳入Git；队友使用仓库中的两份文件。
配置基于队友保存的 `v050-5m-noprior` 包：0.5m/s、蓝左2/黄右1、joint纵向、
关闭融合旧参考；叠加最新几何异常/保留路宽开关及本次雷达通行性设置。
按操作者要求，部署包的 `control.max_distance_m=0.0`、`control.max_run_time_s=0.0`，
解除5m/60s预算。0为上游已支持的“不限”，未删除代码或改变其他停车原因。
本次没有沿用旧分支的2m候选或0.55rad接入门槛，也不修改车辆接口/安装配置。
该包沿用队友的车辆参数，不表示车14已重新标定。

上车上传仓库程序和checkpoint配置，目标仍分别为
`scripts/policy_node.py` 和 `config/ai4r_policy.yaml`，重建后只重启policy。
电脑导入的Foxglove布局使用本分支的新文件；本次未执行SSH或实车操作。

电脑PowerShell：

```powershell
cd "C:\Users\17892\OneDrive\文档\2026 Semester 2\Ai4R\final\mpc-fusion-latest-workspace"
scp .\scripts\policy_node.py "ai4r@10.43.254.14:~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py"
scp .\checkpoints\fusion-mpc-lidar-car14-20261028\ai4r_policy.yaml "ai4r@10.43.254.14:~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml"
ssh ai4r@10.43.254.14
```

两份上传成功后，在小车SSH中执行：

```bash
cd ~/ai4r_student_workspace/src/ai4r_policy
dream runtime stop ai4r_policy
dream build ros student
```

构建成功后启动，再核对：

```bash
dream runtime start ai4r_policy
sleep 3
dream runtime logs ai4r_policy | grep "Policy revision" | tail -n 1
ros2 param get /car/ai4r_policy planning.cruise_speed_mps
ros2 param get /car/ai4r_policy mpc.geometry_glitch_tolerable
ros2 param get /car/ai4r_policy mpc.reference_dropout_tolerance_s
ros2 param get /car/ai4r_policy estimation.keep_width_on_geometry_glitch
ros2 param get /car/ai4r_policy lidar_obstacle.stop_enabled
ros2 param get /car/ai4r_policy shadow.obstacle_avoidance_enabled
ros2 param get /car/ai4r_policy shadow.obstacle_check_enabled
ros2 param get /car/ai4r_policy shadow.obstacle_margin_m
ros2 param get /car/ai4r_policy shadow.collision_coverage
ros2 param get /car/ai4r_policy shadow.weight_obstacle
ros2 param get /car/ai4r_policy shadow.weight_consistency
ros2 param get /car/ai4r_policy shadow.switch_cost_margin
ros2 param get /car/ai4r_policy mpc.r_ddelta
ros2 param get /car/ai4r_policy control.max_distance_m
ros2 param get /car/ai4r_policy control.max_run_time_s
timeout 5 ros2 topic echo /car/lidar_obstacles --once --full-length
timeout 5 ros2 topic echo /car/shadow_planning --once --full-length
timeout 5 ros2 topic echo /car/planner_execution --once --full-length
```

配套部署包参数预期依次为0.5、true、0.5、true、false、true、true、0.0、generated、30.0、4.0、0.05、4.0、0.0、0.0。
保持原Foxglove按钮请求Run，查看障碍代价和绿色路径；本次未增加自动启动/车辆Enable。

## 验证

2026-10-28操作者实车反馈：当前版本仍存在驶出道路的问题，尚未解决。
本次保存调试进度，不代表道路约束或实车绕行验收通过。

新增离线检查覆盖中央/左右障碍、原候选及道路检查不变、远处障碍全路径评分、缺失雷达、
重复回波、不可通行时停车、真实worker传输、部署配置和实际节点到MPC接入。
实际节点检查证明选中路径进入MPC并改变转向输出；不等同于实车闭环或停车验证。
318项离线回归通过，包含26项绕行/通行性、8项短时失边和7项路径连续性检查。
新增检查覆盖过窄通道、0.30m零额外余量、转弯车身、完整路径、worker传输、
等待期零输出、宽路绕行接入MPC、无路停车锁存及旧代结果隔离。
短时失边检查覆盖持续输出、原路径/控制器保留、NO_ROAD不续期、超时等待及恢复，
并确认无已检查路径、运动epoch改变和已确认边界冲突不会获得续用权限。
检查基于当前观测点与配置车身，不建立完整障碍实体，也不保证未观测区域可通行。
完整ROS gate因本机缺少Bash/ROS Jazzy无法执行；未实际连接Foxglove GUI或进行实车验证。

## 最小连续性修补

在已推送6efe99d上，补充共同覆盖不足、单边路宽暂缺两类短时道路失效。
已有检查通过且仍可执行的路径继续运动对齐并交给原MPC跟踪，避免无必要的回正/零驱动脉冲。
期限仍为0.5s；超时、运动/边界冲突、无路可绕仍停车，NO_ROAD不续期。
诊断`road_dropout_trigger`记录触发原因。配置和MPC不改，不含后续simple实验。
新日志还显示：同一条有效路径在planner/waiting间切换，原因是空扫描生成路径的障碍检查标记缺失。
现在复用原车身扫掠检查当前扫描；执行期检查通过的路径也可参与原0.5s失边续用，NO_ROAD不续期。
`/car/planner_execution`新增`obstacle_check_source`、`current_scan_path_clear`、
`current_obstacle_collision`和`waiting_cause`，区分实际碰撞、缺少检查和参考路径接入失败。
新版本运行标识`fusion-mpc-lidar-scan-continuity-20261028`；先留本地，实车待验证。

## 平缓转弯试调

初版运行标识`fusion-mpc-lidar-radius1m-20261028`，当时开启1.0m候选最小转弯半径。
新增离线检查覆盖直线/关闭时保持、曲率界限与左右对称、处理后道路/雷达检查、
完全阻挡停车、独立worker和实际节点接入原MPC。中央障碍示例选中路径峰值曲率约
4.596/m→1.000/m，仍有合格路径；此结果不等于实车通过率或驱动平滑度提升。
本次需要同时上传程序和完整checkpoint YAML；回退试调可将该参数设0后重建。

## 路径切换与转向试调

当前运行标识`fusion-mpc-lidar-path-continuity-20261028`。保留半径试调（当前0.9m），增加合格候选的
新旧形状评分/切换迟滞，转向变化权重改4。明显道路变化仍可立即换路，旧路线出现新障碍时
不获得豁免，完全阻挡仍停车。仅留本地，当前修改未上车验证。
单独关闭路径偏好：`weight_consistency=0.0`、`switch_cost_margin=0.0`；恢复原转向权重：
`mpc.r_ddelta=2.0`。参数需改完整部署YAML并重建/重启policy。

## U弯半径微调

`shadow.min_turn_radius_m: 1.0→0.9`，允许曲率约1.11/m；其余配置保持。
检查总宽按最新要求回退为0.30m，完整YAML保留0.9m半径及其他调优。
此前合成1.2m宽、中心半径0.65m的U弯，在0.38m检查宽度下，1.0m半径无合格候选，
0.9m半径有19条合格候选。此为合成几何回放，不等于实车U弯验证。
