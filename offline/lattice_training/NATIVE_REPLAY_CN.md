# 实车锥桶道路：native planner 理想输入诊断

以下保留改动前的诊断证据。后续已按用户要求将净空检查改为连续边界与完整车身
几何，修复、回归与重放结果另见 [连续车身检查记录](CONTINUOUS_BODY_CN.md)。

2026-10-15。当前结果：**理想道路输入仍未走完全路，纯参数诊断尚未取得整路成功**。
不能据此转入“planner 已正常、仅仿真环境与实际环境不一致”的结论。
本阶段没有改动运行中的 policy、车辆参数、native 源码、训练数据或评分模型。

## 输入与协议

从 `origin/fix/newcar-control` 的 `5e8d1b8e4bec54d1306f769315e57246f9fff496`
按 Git blob 原样提取三份 2026-10-14 Car 15 录制。主要诊断采用 `end-straight1/run.jsonl`：
270 帧锥桶、241 个轮速和237 个 raw gyro 样本，包含启动与结束阶段。
完整录制不等于完整的物理道路测绘。

日志参数读回为 **20 Hz policy timer**；锥桶 acquisition 间隔中位数
0.099986 s，发布延迟中位数0.192415 s。现有 Dream Gym v0.4.0 的0.05 s
时间步对应20 Hz，虚拟相机使用10 Hz/0.20 s延迟。使用原训练环境的虚拟解释器
`../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe`。

输出采用现有训练线的直接 Frenet 导出：原 `frenet_path`、`longitudinal_profile`
和物理 `speed_profile`。`path=None`、`output_errors=None`，关闭 Cartesian 拟合及其
阈值检测。边界/车身、转角、转角速率、运动、范围与来源时效检查继续由 native 执行。
未使用 learned scorer、policy checkpoint 或重新训练。

## 重建与理想输入的边界

轮速/raw gyro 平面里程计对齐锥桶 acquisition 时刻；不重复减去已经体现在 stamp 中的延迟。
raw gyro 缺少 mounting TF，负号通过记录中的顺时针弯道及双侧地图一致性推断，
不是独立测量的安装标定。置信度≥0.75、前向0.25–3 m 的点按0.15 m分箱取中位数，
再用0.6 m半窗口拟合边界。全部原始点、分箱、里程计、配置及SHA256保留。

双侧共同覆盖的道路长度为 **4.234 m**，宽度0.809–1.024 m。
Dream Gym 的分段直线/圆弧对中心线的最大拟合偏差0.007945 m。
另外显式加入1 m起步缓冲，故 plant 长度5.234 m；缓冲没有锥桶观测，不计入4.234 m。
原录制起点投影约0.408 m，理想基线从该站点居中、静止起步。
末端不补造道路；最终基线终点沿用 `pipeline.road_goal_progress()`，为4.737 m。

`road_input=map_oracle` 使用重建地图的当前双侧几何，不经过相机视野与估计器拒绝。
`state_input=plant_truth` 使用当前 plant 的精确速度与偏航率。
它用来隔离 planner，不构成感知闭环通过，也不证明重建地图等同真实道路。

## 已观察结果

| 对照 | 有效路径帧 | 最后站点 | 结果 |
|---|---:|---:|---|
| native 原类默认采样，理想地图/精确状态，零速起步，隔离 wall budget | 107 | 0.739 m | `no_feasible_lattice_trajectory` |
| 同类基线，实际 wall budget 35 ms | 35 | 0.447 m | `lattice_time_budget_exceeded` |
| 较短范围/过渡及1 s纵向窗口，理想地图/精确状态 | 182 | 1.334 m | 转角、转角速率与过渡范围拒绝 |
| 将理想前视范围从0.9 m扩至1.1 m，允许0.3–0.6 m过渡 | 203 | 1.512 m | 全部过渡长度大于可用支持范围 |

这里的“原类默认”指 native dataclass 的采样、硬阈值与打分默认值；实验显式选择
`course_simulation`、受控起步及无障碍检查。完整解析后参数在最终 summary 中。
前两项隔离和保留 wall clock，分别回答几何可行性与运行预算问题。
后两项是参数对照，初速0.1 m/s；没有把它们当成零速基线的重复实验。
有效输出的 Frenet 查询/时效/字段合同检查均为零错误；**没有任何一项达到道路终点**。

每0.05 s调用一次 native planner。控制复用现有 `CStarSolver`，求解完整参考，
每0.5 s更新解并执行其前缀；不是训练标签的“完整候选执行”采集模式。
C* 的求解耗时不推进仿真时钟，沿用离线实验假设；它是受约束数值近似，
没有全局最优证明。不能把这些结果记为整个系统已满足实车20 Hz实时执行。

## 失败定位

**原基线0.739 m：边界余量的全局导数界造成保守拒绝。**
同一候选几何采样中，plant 按相同0.05 m硬余量计算的最小车身道路余量为0.213 m；
native 的全局边界导数界计算出现−0.00454 m，共22个采样点被该项拒绝。
该失败帧的实际车身道路余量为0.294 m。仅说明这项检查过于保守，不单独证明候选
可以完成动态执行。

实验对照将导数界限制到车身覆盖的局部区间，保留其它 native 检查与阈值，
同一失败输入可生成一条通过 native 检查的 Frenet 参考：1.0 m过渡、0偏移、
3 s窗口、0.1 m/s末速度。此对照仅存在于离线 runner 的显式
`local_footprint_experiment` 分支，尚未变更生产检查或认定其通用安全性。

**1.334 m：范围与运动采样不匹配。**
可用范围只有0.231 m，0.3/0.4 m过渡被排除；0.2 m候选受转角、不可达CG曲率或
转角速率拒绝。该位置实际车身道路余量仍有0.238 m。
局部导数界对照仍在同处失败，因而这一失败不能归因于全局余量项。
同一状态下改变理想前视支持：0.9 m拒绝，1.0–1.3 m可生成路径，进一步加长又可能
引入保守边界拒绝。不能通过无条件放宽转角/余量来解决。

**1.512 m：固定过渡网格缺少能装入当前支持域的长度。**
该帧所有候选均为 `transition_exceeds_common_horizon`。
后续 native 调整应先处理局部支持域、过渡采样与边界检查的配合，并单独验证
CG运动方向/运动初值和转角速率；感知参数调试暂不继续。

## 能否只调整 planner 参数

**局部失败可以改善，但目前没有纯参数完成整路的证据。** 为排除前视范围变更的影响，
本轮固定理想道路输入算法和2 m前视、精确plant状态、0.05 s仿真步长、原C*配置，
保留车身尺寸、0.05 m硬余量及全部物理限制。仍使用直接Frenet输出；几何测试隔离
wall budget，因此不构成20 Hz实时通过。

在3个既有失败状态和8个沿路居中静止状态上扫描180套采样配置，共1980次查询。
扫描空间为检查网格0.025/0.015/0.01 m、规划范围0.3/0.5/0.8/1.2/2 m、
三组过渡长度、1/3 s纵向及评分窗口、0或±0.1 m末端偏移；每组都提供
0.05/0.1/0.2 m/s末速度。打分权重保持原值，没有把这次有限扫描当成全部参数空间。

| 固定输入状态 | 可生成非STOP有效路径的配置数 |
|---|---:|
| 原基线失败状态，站点0.739 m | 120 / 180 |
| 既有1.334 m失败姿态，重新提供固定2 m前视输入 | 0 / 180 |
| 既有1.512 m失败姿态，重新提供固定2 m前视输入 | 80 / 180 |
| 居中静止于1.0、1.5、2.0、2.5 m | 每个状态均为0 / 180 |

按“恢复既有失败帧数、总通过状态数”排序，首组恢复2/3个既有失败状态、通过6/11个
固定状态。将这组参数从0.408 m静止起步做完整C*闭环，产生83帧有效路径后，
仍于**0.742 m**失去候选，未到4.737 m终点。该组检查网格0.015 m、规划范围0.8 m、
过渡长度0.15–0.8 m、1 s窗口、0偏移；配置和精简证据保存在
[pure-parameter-summary.json](artifacts/2026-10-15-native-diagnosis/pure-parameter-summary.json)。

硬检查提供了独立于打分的解释：在重新构建2 m输入的1.334 m姿态，以及1.0、2.0、
2.5 m居中静止状态，连将非负的网格扫掠项降至零，初始车身净空上界仍为负。
其中1.334 m姿态为−0.00453 m。当前检查使用整段输入边界的最大斜率/二阶导数，
缩短planner规划范围不会重新限制这些全局导数界；过渡、末速度和权重也不改变同一
输入状态的初始车身位置，因而无法使这些状态通过这项硬检查。

这不证明所有其它参数组合或提前改变路线都必然失败，也不证明重建道路与实车一致。
下一步应先核对输入边界插值与局部导数界，再处理支持域与过渡采样的配合；不能把
缩小车身、取消安全余量或放宽物理限制当成参数调优成功。目前未修改生产planner。

## 复现

从 lattice-training 根目录，在 PowerShell 中执行以下命令。输出中的失败是诊断结果，
`native_replay` 在未到达终点时返回非零退出状态；不能仅以进程启动成功判为通过。

```powershell
$py = '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe'
& $py -B -m offline.lattice_training.log_audit --ref 5e8d1b8e4bec54d1306f769315e57246f9fff496 --output .verification/native-log-road-20261014/source
& $py -B -m offline.lattice_training.log_road --log .verification/native-log-road-20261014/source/end-straight1/run.jsonl --output .verification/native-log-road-20261014/reconstruction-end1
& $py -B -m offline.lattice_training.native_replay --road .verification/native-log-road-20261014/reconstruction-end1/road.json --config offline/lattice_training/native_recorded_road.json --output .verification/native-log-road-20261014/reproduce-native-wall
& $py -B -m offline.lattice_training.native_replay --road .verification/native-log-road-20261014/reconstruction-end1/road.json --config offline/lattice_training/artifacts/2026-10-15-native-diagnosis/pure-parameter-candidate-00.json --output .verification/native-log-road-20261014/reproduce-pure-parameters
```

几何隔离对照的配置为 `.verification/native-log-road-20261014/ideal-geometry-baseline.json`，
仅将 `enforce_planner_wall_clock` 设为false；仿真调用间隔仍为0.05 s，wall耗时仍记录。
完整 trace、逐次求解、失败帧、30套理想范围扫描及失败输入对照在
`.verification/native-log-road-20261014/`。精简证据和图在
[artifacts/2026-10-15-native-diagnosis](artifacts/2026-10-15-native-diagnosis/verification.json)。

已运行原有 `test_lattice.py` 34项、`test_lattice_training.py` 53项，全部通过。
ROS fast gate 实际尝试后因缺失 pinned `dream_interfaces` checkout 而未运行；
没有修改ROS依赖或在车上执行测试。
