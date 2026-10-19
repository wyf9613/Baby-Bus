# MVP2.1 LiDAR 15 cm 遇障停车

本地仿真实现，基于 MVP2.1 模块与原课程 DREAM Gym。旧 V2 未参与调用。第一版范围为低速直道、静态矩形障碍、前方车身宽度内的点，不声称支持弯道完整扫掠或真实车辆制动。

## 运行

在当前 worktree 根目录运行，输出目录必须尚不存在：

```bash
MPLBACKEND=Agg MPLCONFIGDIR="$PWD/.verification/environment/matplotlib" \
  .venv/bin/python -B -m offline.lidar_stop --output .verification/lidar-stop-new
```

`--scenario central` 可只跑正前方障碍；默认依次运行 empty、central、outside、coast。输出 `REPORT.md`、`suite.json`、`viewer.html`、结果图，以及各场景完整观测/动作/模型指纹和控制记录。

## 已实现的规则

- 前方车身走廊内，`障碍 x − 车头前伸长度 ≤ 0.15 m` 时锁存停车；不是把整个 LiDAR 量程裁成 15 cm。
- 车身尺寸来自本次仿真配置：车头前伸 0.297 m、车宽 0.25 m。因此中线上的点达到 base_link 前方约 0.447 m 时进入触发区。
- 数据流为仿真测距 → `lidar_cartesian` → 实际 EstimationPipeline 的 `obstacles` → `ObstacleStopMonitor`。障碍真值不参与决策。
- 检查数据是否可用、有限、当前车体坐标、已对齐且未过期。有效空点集与缺数据分开。关键输入异常同样锁存停止。
- 检查每个 20 Hz 控制周期运行；停车优先于 planner 控制和中心线回退。障碍消失不能恢复，只有显式新跑次/reset 清除锁存。

## 仿真速度与制动的明确区别

原公共配置的目标速度是 0.2 m/s，但它使用实车油门前馈 0.30。直接作用在课程无死区模型上会超调。本实验仅在 runtime 配置副本中覆盖：feedforward=0.004、speed_kp=0.35、speed_ki=0、drive_max=0.08；仍使用同一个 MVP 控制器。

0.004 对应课程模型在 0.2 m/s 的阻力平衡：`drag*v² / motor_gain = 1*0.2²/10`。没有修改车辆模型、真实速度或传感器缩放，也没有改公共 YAML。

共享 monitor 只请求停止/零动作。课程模型零油门主要靠二次阻力，不能在 15 cm 内停住。离线 runtime 因而显式使用该模型已支持的 `drive=-0.1` 制动，在轮速 ≤0.01 m/s 后切零，继续观察至少 1 秒。此负输入仅在本实验中启用，ROS 节点没有接入它，不能直接用于实车。

`coast` 场景保留同一 15 cm 判定、仅改为零油门。它应接触障碍并被标为失败；套件通过表示这个失败被正确暴露，绝不表示零油门也能安全停车。

## 验收

- empty：到达直道目标，不误停。
- central：触发间隙在 15 cm 内，实际速度降到 ≤0.01 m/s 并保持至少 1 秒，无接触且最终间隙大于零。
- outside：同一前向位置、走廊外的障碍不导致停车，到达任务目标。
- coast：明确保留零油门接触失败。
- 所有场景检查实际峰值 ≤0.22 m/s，目标保持 0.2 m/s。此容差属于本仿真验收，不是实车限速变更。

以实际记录为准，典型 central 结果为：峰值约 0.19997 m/s，车头间隙约 14.52 cm 时触发，最终速度为零、间隙约 8.63 cm，无接触。15 cm 是触发间隙，不是要求最终间隙等于 15 cm。

任务终点完成 `completed` 和停车验收 `obstacle_stop.passed` 分开记录。central 提前停下时 `completed=false` 是正常的；传感器故障停下也不会被当作成功检测到障碍。

## 阅读和测试入口

1. `mvp21_policy/obstacle_stop.py`：几何、数据有效性、锁存。
2. `mvp21_policy/execution.py::calculate_selected_control`：停车如何压过回退。
3. `offline/mvp21_sim/runtime.py`：复用原闭环，接入 monitor 和显式仿真制动/停止观察。
4. 本目录 `__main__.py`：场景、仅限仿真的控制参数、独立评价与图表。
5. `tests/test_obstacle_stop.py`：阈值、左右范围、故障、锁存和回退优先级。

使用 `.venv/bin/python -B -m unittest tests.test_obstacle_stop -v` 跑针对性检查。完整 ROS gate 和实车验证仍需各自环境与测试；此处没有部署、Enable、传感器服务变更或实车试验。

## 2026-10-19 直道鲁棒性优化

开发基点仍为 `origin/feature/mvp21-sim-integration-20261015@465e581`，功能工作分支为 `ying/lidar-stop-mvp21`。当日刷新远端后，基点与远端相同。源码使用 MVP2.1；没有回到旧 V2。改动仍在本地，未提交、推送或合并。

这次优化让锁存停车在道路规划、baseline 检查和启动运动历史预热之前进入共用执行门控，避免预热延迟停车；同时把仿真负输入制动和真正零动作分开计数，峰值速度统计包含停车尾段。

新增的 `sensors.py` 在原传感器采集队列处注入降频、固定延迟和断流，在原测距转 Cartesian 之前注入带种子的 Gaussian 距离噪声。断流只丢新采集，已在队列中的帧仍按时交付；无返回射线不会被噪声变成障碍。默认注入配置与原环境逐帧一致。车辆模型、射线生成器和实际 MVP2.1 estimator 均复用。

扩展测试命令（输出目录必须是新的）：

```bash
MPLBACKEND=Agg MPLCONFIGDIR="$PWD/.verification/environment/matplotlib" \
  .venv/bin/python -B -m offline.lidar_stop --matrix \
  --output .verification/lidar-stop-matrix-my-review
```

`robustness.py` 定义固定 25 个场景：原四组、宽/窄障碍、左右部分重叠、仅侵入车宽 1 mm、车宽外 5 mm、10 Hz、10 Hz 加 50 ms 延迟、5 Hz、150 ms 延迟、丢一帧、300 ms 断流、启动缺数据、三种 5 mm 噪声种子、左右边缘加低频延迟、启动即有近障碍。

必须区分三类验收：正常障碍停车、数据失效后的保护停车、明确保留的零油门碰撞失败。套件“全部符合预期”包括正确暴露 `coast` 碰撞，不代表所有运行都无碰撞。150 ms 是数据年龄上限；5 Hz / 150 ms 延迟实验会触发保护停车，不是证明可以继续低频驾驶。

边缘场景在离散射线下可能迟至约 8.52 cm 才有点落入车身走廊，最终约留 2.63 cm；普通中间障碍约 14.52 cm 触发、8.63 cm 停稳。固定的 15 cm 规则没有扩大，也不保证所有物体都在进入 15 cm 的瞬间被射线捕获。三种噪声种子与 25 个固定场景只构成本次覆盖范围。

回归测试还复现了原仿真 `restore()` 的对象生命周期问题：恢复后强制内存回收会清空车辆 backend 的状态字典。修复仅为复制临时对象的这一层 `__dict__`，不改动力学。`preserved-files.json` 保留原 hash、修复后 hash、基点和测试入口，继续校验其他历史文件；没有重写旧实验结果。新增传感器队列也支持快照复制和噪声 RNG 恢复。

本次 review 优先顺序：

1. `offline/mvp21_sim/runtime.py`：停车优先级、制动/零动作计数与停车观察。
2. `offline/lidar_stop/robustness.py`：25 场景的输入和各自预期。
3. `offline/lidar_stop/sensors.py`：噪声、采集丢帧和交付时钟。
4. `tests/test_lidar_stop_sim.py`：默认行为一致、排队帧交付、启动停车、快照回收回归。
5. `offline/planner_sim/environment.py::restore` 与 `offline/mvp21_sim/preserved-files.json`：仅涉及状态复制的修复和原始 hash 记录。

最终本地结果在 `.verification/lidar-stop-robustness-20261019-final/`：`REPORT.md`、`suite.json`、25 组原始记录、`viewer.html`、`robustness-results.png` 和原四组速度/间隙图。72 项 portable tests 通过，日志为 `.verification/environment/lidar-stop-optimization-regression-final.log`。ROS fast gate 仍在缺少固定 `dream_interfaces` checkout 的预检处停止，不能据此宣称 ROS 集成完成。下一阶段应使用录制的真实 scan 做离线回放，核对安装方向、时间戳和点集，再讨论实车接入。


## 依赖下载后的状态更新（2026-10-19）

已将官方 `dream_interfaces` 的 `v0.2.0` 下载到工作目录 `.verification/dependencies/dream_interfaces`，并核对提交为 `5f50902ccee44e8370d6e2be85607b3054ffbf98`，源码无修改。依赖缺失已经解决。重新运行 ROS gate 后，当前阻塞是本机不存在 `/opt/ros/jazzy/setup.bash`，即尚未准备 ROS 2 Jazzy 开发环境。日志为 worktree 的 `.verification/environment/lidar-stop-ros-gate-after-download.log`。此前关于缺接口依赖的描述属于下载前状态。


## ROS 环境就绪更新（2026-10-20）

本机已配置独立 Colima / Ubuntu 24.04 ARM64 / ROS 2 Jazzy 环境，`dream_interfaces` 与 policy 均已构建。完整 fast gate 通过：353 tests、0 failures（344 pytest 用例及 9 CTest 测试组），launch 参数检查通过。8 项停车门控单元检查、两帧合成 LaserScan 的 MCAP 读写和停止/重启验证也通过。此前“缺接口依赖”和“缺 ROS 安装”的描述属于配置前状态。

当前环境已停止以释放内存，双击课程工作目录的“启动ROS2开发环境.command”即可自动启动。使用说明见 `tools/ros2_jazzy/README_CN.md`，实际版本和结果见 `.verification/ros2-jazzy/environment-manifest.json` 与 `final-ros-check.log`。真实 ROS 检查没有接通新的 LiDAR 停车门控到实车节点，也没有进行实车验证。


## ROS 门控接入更新（2026-10-20）

现在 ROS 节点已复用共用门控，并通过七场景合成消息验收；默认关闭，未部署实车。完整 ROS gate 为380 tests、0 failures，Mac portable 回归250项通过；四组离线车辆回归697帧与接入前状态和动作一致。ROS 测试只证明零指令和状态机行为，不证明真实制动距离。最新说明见 `docs/ROS_LIDAR_STOP_CN.md`，证据索引见 `.verification/ros-lidar-stop/acceptance.json`。旧25场景及环境初次配置结果保留为当时版本记录。
