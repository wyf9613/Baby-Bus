# MVP2.1 ROS 雷达停车：本地接入与验收

本次完成 ROS 接线与消息级测试。真实车辆数据采集、安装标定、动力学制动和部署未进行。代码仍基于 `feature/mvp21-sim-integration-20261015@465e581`，功能分支 `ying/lidar-stop-mvp21`。

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

新增参数均为 startup-only，公共 `config/ai4r_policy.yaml` 未修改。默认 `obstacle_stop.enabled=false`，现有配置继续按原路径运行。启用需要 timer 更新和已启用的 MVP control。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `obstacle_stop.enabled` | false | 显式启用门控 |
| `obstacle_stop.trigger_distance_m` | 0.15 | 车头前沿之外的触发间隙 |
| `obstacle_stop.front_extent_m` | 0.297 | base_link 原点到车头，当前是课程仿真测试尺寸 |
| `obstacle_stop.rear_extent_m` | 0.198 | base_link 原点到车尾，测试尺寸 |
| `obstacle_stop.width_m` | 0.25 | 车宽，测试尺寸 |
| `obstacle_stop.lateral_margin_m` | 0.0 | 车身走廊额外横向余量 |
| `obstacle_stop.max_source_age_s` | 0.15 | 雷达源数据年龄上限 |

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

下一阶段使用停稳车辆采集的真实数据检查安装方向、坐标偏移、源时间戳与消息年龄；数据采集后可在本地重复回放。真实制动距离仍需要单独的受控验证。
