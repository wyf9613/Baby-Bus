# MVP2.1 ROS 雷达停车：本地接入与验收

代码仍基于 `feature/mvp21-sim-integration-20261015@465e581`，功能分支 `ying/lidar-stop-mvp21`。
2026-10-21 操作者报告使用 0.30/0.20 s 时效配置实车行驶并成功遇障停车；这是用户报告的实车结果，
不再仅有离线证据。尚无实测车速、制动距离或重复成功率。当前整合版另修复整帧坏扫描误判为空场景的问题，
需要在重新部署后复验。部署内容和时间解释见 [整合版说明](LIDAR_STOP_DEPLOYMENT_CN.md)。

## 数据流与优先级

```text
合成发布者：LaserScan / TF / 轮速 / IMU / 锥桶
  → 实际 ROS 回调与坐标转换
  → 实际 EstimationPipeline（按同一快照时间做运动对齐）
  → 共用 ObstacleStopMonitor
  → 停车优先于 MVP2.1 规划、启动预热、终端直行及控制回退
  → 原状态机进入 publishing-zero，独立监督定时器持续发布零动作
```

门控必须使用感知记录的快照时间。估计完成后重新读一次墙上时钟，会让已对齐的点集与校验时刻产生微小偏差；因此本实现捕获一次时间，把传感器年龄、感知和停车判定对应到同一快照。发布驾驶指令之前再次按实际时间检查数据是否已经过期。

显式开始时会重置运动历史。如果当前 scan 的运动对齐还没准备好，只在原有 startup grace 内发布零动作，期间不执行规划或提交 native 任务；对齐可用后才允许驾驶。缺 scan、TF 失败和数据超时由独立监督检查处理，不会借预热窗口放行。雷达回调停止或控制定时器取消，也不会取消监督定时器的停车和零动作发布。

触发后的第一个原因锁存。障碍移走、数据恢复和 disable/zero 状态切换都不会清除它；只有传感器重新可用且收到显式 state-3 请求，才会开始新运行并复位门控。

## 参数与启动边界

参数均为 startup-only。裸节点默认 `obstacle_stop.enabled=false`；本功能分支的公共
`config/ai4r_policy.yaml` 现已显式启用，并固定用户报告成功的实际实验配置。
启用需要 timer 更新和已启用的 MVP control。下表区分裸节点默认与公共部署值。

| 参数 | 裸节点默认 | 公共部署值 | 含义 |
|---|---:|---:|---|
| `obstacle_stop.enabled` | false | true | 显式启用门控 |
| `obstacle_stop.trigger_distance_m` | 0.15 | 0.15 | 车头前沿之外的触发间隙 |
| `obstacle_stop.front_extent_m` | 0.297 | 0.275 | base_link 原点到车头，部署值含暂估偏移 |
| `obstacle_stop.rear_extent_m` | 0.198 | 0.198 | base_link 原点到车尾 |
| `obstacle_stop.width_m` | 0.25 | 0.30 | 车宽 |
| `obstacle_stop.lateral_margin_m` | 0.0 | 0.0 | 车身走廊额外横向余量 |
| `obstacle_stop.max_source_age_s` | 0.15 | 0.30 | 雷达源数据年龄上限 |
| `obstacle_stop.max_receipt_age_s` | 0.15 | 0.20 | 距上次接纳扫描/点集的单调时钟年龄上限 |

frame 自动跟随 `policy_frame_id`；不会用修改 frame 字符串代替实际 TF。上述车身尺寸不构成真实车辆标定，实车启用前需要测量。当前只检查直行车身走廊，不处理弯道扫掠车身、绕行、高度分类或自车回波。

共享 monitor 只请求停止，ROS 只发布归一化零 drive/steer，pan 保持。离线 Dream Gym 的负制动输入没有接入 ROS。本次没有验证零油门在真实车上能以多短距离停稳。

## 运行与证据

在 Mac 功能 worktree 中执行：

```bash
bash tools/ros2_jazzy/run.sh check       # 编译与完整 ROS 回归
bash tools/ros2_jazzy/run.sh lidar-stop  # 七场景 DDS 演示，写入新的记录目录
```

也可双击课程目录的 `运行ROS雷达停车测试.command`。结果位于 `.verification/ros-lidar-stop/runs/<时间>/`。每组 JSONL 记录合成输入和真正订阅到的 drive/steer、FSM、门控原因、实际 planner/baseline 来源。`summary.json` 保存实际模块指纹、公共配置 hash、测试参数和验收结果。演示拒绝使用与当前源码不一致的已安装模块；源码变更后先运行 check 构建。

合成轮速固定为零；这里验证消息处理与指令，不模拟小车运动，也不宣称实测速度达到 0.2 m/s。配置的规划目标仍为 0.2 m/s。颜色按当前配置的左右约定发布，前方点通过实际 TF 的 0.05 m 测试偏移后进入 base_link；没有给控制器注入障碍真值。

| DDS 场景 | 必须观察到的结果 |
|---|---|
| empty | 有效空点集，正常驾驶指令 |
| central | 确实使用过 native planner；16 cm 不触发，14 cm 输出零指令；移走后保持零，显式恢复后继续 |
| outside | 前向接近但车宽走廊外，不触发 |
| dropout | 持续其他传感器、停止 scan，保护停车并保持零 |
| missing_tf | 新 scan 无可用 TF，旧点集失效，保护停车 |
| watchdog | 暂停控制定时器并停止 scan，监督定时器仍能停车 |
| startup_obstacle | 启动时已有 14 cm 障碍，不能先发布一次正 drive |

最终完整 ROS gate 报告 380 tests，0 errors/failures/skipped（369 个 pytest 用例及 11 个 CTest 测试组）。其中新增 ROS 门控用例 17 个、共用纯 monitor 用例 8 个，七组 DDS 场景包含在端到端用例中。Mac 侧相关 portable 回归 250 项通过。四组离线车辆回归共 697 帧与接入前状态和动作逐帧一致；coast 的碰撞失败仍被保留。此前 25 场景的记录保留为 ROS 接入前的历史证据，不改写其源码指纹。

主日志为 `.verification/ros-lidar-stop/gate-final.log`、`portable-regression-final.log`；最终 DDS 运行目录与其他证据索引见 `.verification/ros-lidar-stop/acceptance.json`。

## 建议 review 顺序

1. `mvp21_policy/node.py` 的参数声明、`run_policy_step()` 与 `_check_obstacle_stop()`：驾驶前如何进入门控。
2. `_update_policy_estimation()`：复用原感知并统一快照时间。
3. `_obstacle_stop_source_fault()` / `_obstacle_stop_watchdog()`：无回调时如何处理数据故障。
4. `fsm_transition_request_callback()`：只有明确恢复才复位，`_change_state()` 不清除门控触发原因。
5. `tests/test_ros_obstacle_stop.py` 和 `tools/ros2_jazzy/lidar_stop_demo.py`：边界测试、实际 DDS 发布和动作记录。

上面的 380 项验收与“公共配置未修改”等描述属于 10月20日历史版本。后续阶段已获得真实 TF、
实车日志和操作者停车反馈；真实制动距离仍需要单独测量。下面按时间保留各候选版本的历史证据。

## 2026-10-21 静止实车检查与超时诊断候选

车辆 `.19` 在硬件 Disabled 下使用真实 scan 请求 policy 计算，状态明确进入
`Control: lidar_obstacle`；移走挡板后仍锁存零动作。显式恢复后另一次运行因
`lidar_stale_or_bad_timestamp` 停止，根因尚未定位。这不是行驶/刹停验收。
操作员将本车实验源年龄设为 0.25 s；源码默认值仍为 0.15 s，15 cm 距离阈值不变。

独立订阅探针的一个窗口测得 10 Hz、约 100 ms stamp 延迟，最大缓存年龄
0.179 s。这不能代表 policy 内部缓存，也不能排除其他时间窗口的抖动。
16:19 的源码、有效参数和同 run-id 日志表明，车辆 16:14:22 已启动另一版学习模型
policy，速度配置 0.5 m/s，且无 obstacle_stop 参数。该次日志不能用作此前超时的根因证据；
日志不确定部署操作者身份。

本地诊断改动保持原有时效和停车语义，首次源故障附近用现有有界 warning 输出
`LiDAR stop source:` JSON：原始/Cartesian 源 stamp、receipt/stamp/effective age、
故障源、年龄上限、上次计算耗时及检查阶段（start_admission、before_calculation、
after_calculation、supervision）。日志失败不会阻止原调用者执行停车。
这些字段解释源 watchdog，不能单凭它们推断雷达硬件或调度根因。

新增三项 ROS 回归分别检查：10 Hz/100 ms 延迟在 0.25 s 上限下持续运行；
额外计算延迟使输入真正过期时禁止非零输出并记录阶段；日志故障仍执行保护停车。
完整 fast gate 为 372 个 pytest 用例和 11 个 CTest 组（工具汇总 383），无失败/错误/跳过。
诊断候选是下一次硬件 Disabled 采证用版本，尚不宣称已修复原实车超时。

### 16:38 故障快照与时效预算修正候选

快照 run `6136bf3c-edda-43b6-8ab8-ae2280d5ace3` 的源码 hash 与我们上传的诊断包一致，
日志和服务状态 run-id 一致。故障发生在 supervision：扫描及点集 stamp age 均为
0.253753087 s，receipt age 为 0.113141551 s；因此到达 policy 时的数据年龄为
0.140611536 s。上一轮 policy 计算为 0.025266238 s。0.25 s 采集期限实际被超过，
不能称为时间戳倒序，也不能单凭此记录定位上游延迟/排队的具体来源。

修正候选将两个条件分别检查：采集源年龄仍由 max_source_age_s 限制，新增
max_receipt_age_s 按单调时钟限制多久未接纳新帧，两者任意超时均停车。两项
源码默认值均为 0.15 s，公共 YAML 不变。静止实验包显式设为 source=0.30 s、
receipt=0.15 s；其余 15 cm 阈值、锁存、显式恢复和零动作语义不变。
这些实验期限不是物理制动或硬实时保证；Python executor 调度仍影响实际检测时刻。
对齐后的纯 monitor 继续验证采集时间；接收期限由持有原始 receipt 的 ROS watchdog 执行。

新增回归用日志中的两个年龄重放：source=0.25 s 仍拒绝，source=0.30 s 接纳；
source 放宽到 0.30/0.50 s 时，超过 0.15 s 的接收中断仍触发零动作；刚收到但
stamp 已老于 0.30 s 的扫描仍被拒绝。这里重放日志时序，不是原始真实 LaserScan bag。
必需 gate 通过 378 个 pytest 用例和 11 个 CTest 组（汇总 389），无失败/错误/跳过。
实车尚未验证新候选。同一旧 run 的另一轮曾因 insufficient_common_range 拒绝规划，
该道路条件问题不能归为已由雷达时效修正解决。

### 后续实车反馈与本地整合修复

操作者随后报告：仍使用 source=0.30 s、receipt=0.20 s，实车行驶并成功遇障停车。
此前候选的“未实车验证”描述是当时状态，不能用于否定这次反馈。没有新增制动距离或重复性测量。

本轮把同一实验配置写入公共 YAML，并修复整帧无效距离被转换成“可用空点集”的漏洞：
全部为 NaN、负无穷或越界距离时保留原始数据，但 Cartesian 不可用，触发已有故障保护；
正常 +Inf 无回波、夹杂少量坏点仍按原路径处理。恢复有效数据后仍锁存零动作，只有显式恢复才重启。
这不检测局部视野缺失，也不改变规划、控制、线程模型或15 cm阈值。

必需 fast gate 本轮通过 383 个 pytest 用例与 11 个 CTest 组（汇总394），零错误/失败/跳过，
包括新增5项坏扫描用例和原有七场景 DDS 验收。公共配置安装加载也核对了0.30/0.20秒。
Mac侧 control/monitor 共42项通过。日志为 `.verification/ros2-jazzy/checks/20261021-214922/verify-fast.log`。
这次未连接实车；新的扫描质量修复仍需部署后复验。计算耗时118.5 ms超过50 ms目标周期的记录
作为性能问题保留，未以扩大阈值或多线程重构掩盖；计算前后检查仍会拒绝过期输入。
