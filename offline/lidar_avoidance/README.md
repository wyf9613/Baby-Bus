# MVP2.1 静态雷达绕行：第一版离线闭环

本入口复用原 `offline.mvp21_sim.runtime`、实际道路/障碍估计、MVP 控制器和
课程车辆模型。没有另写一套简化车辆闭环，也没有接入 MPC 或重新训练模型。
本版仅用于离线实验，ROS 节点未构造绕行组件，公共 YAML 保持原样。

## 运行

在 `ying/mvp21-lidar-avoidance` 工作区运行，输出目录必须尚不存在：

```bash
/Users/xiaotonghe/Documents/Baby-Bus/.verification/worktrees/mvp21-lidar-sim/.venv/bin/python -B \
  -m offline.lidar_avoidance --output .verification/lidar-avoidance-new
```

可用 `--scenario central` 单独运行中心障碍。默认七组：clear、central、left、
right、blocked、dropout、stop_only。课程目录另有
`运行MVP21雷达绕行离线验证.command`，每次使用新目录。

结果包含 `REPORT.md`、`suite.json`、`source.json`、`avoidance-results.png`、
每场景完整 trace/submissions/shadow JSONL，以及沿用原框架的 `viewer.html`。
回放数据单独存于 `viewer-data/`，需通过本地 HTTP 服务打开，例如在结果目录执行：

```bash
python3 -m http.server 8765 --bind 127.0.0.1
```

然后打开 `http://127.0.0.1:8765/viewer.html`。回放只读取已有结果，不运行车辆或算法。

## 数据与决策链

1. 原仿真射线传感器采集 LiDAR，按 10 Hz、50 ms 延迟交付；原估计器转成
   当前 `base_link` 障碍点。锥桶和道路也经过原观测与估计链。
2. `LidarAvoidanceGuard` 使用原轮速/偏航运动对齐维护有界静态障碍记忆。
   当前扫描必须持续新鲜；记忆点保留原测量时间，空扫描不能刷新旧点的时间。
3. 原 native lattice pool 保留，另在当前感知道路的 Frenet chart 上加入平滑
   横移保持候选。横移族保持偏移，不假设未观测到的障碍后缘在哪里。
4. 全生成路径检查车身与 LiDAR 点、实际观测边界的相交，并检查曲率。
   各雷达点独立检查，不把相邻回波连成虚构的实墙。逐候选记录拒绝原因。
5. 绕行期间优先可行的平滑横移保持族，族内按确定性几何代价选择；保留
   已选绕行侧。车尾通过所有记忆障碍点后，才释放绕行侧并返回道路跟踪。
6. 共用 `calculate_selected_control` 在执行路径和同周期 baseline fallback
   之前都复查当前障碍。控制器拒绝绕行路径后，不能直接开进受阻中线。
7. 原 `ObstacleStopMonitor` 优先级最高，仍先检查 15 cm 紧急停车和源故障。
   新增保护停车也锁存到跑次结束，没有自动重启。

`offline/lidar_avoidance/service.py` 只给现有因果交付服务加入不可变的作业快照。
障碍真值、完整道路、车辆真值姿态均只用于物理推进、评价和画图，不进入选路。

## 明确的实验条件

- 5 m 直道、宽 1.4 m，静态障碍位于 2.8 m。按原 runtime 的任务终点规则完成，
  结束进度约 4.5 m；不代表实现了终端停车。
- 实际 MVP 目标仍为 0.2 m/s，使用原雷达停车实验的仿真纵向 override。
  课程无死区模型需要 feedforward=0.004，不能把它当作实车参数。
- 本实验显式使用 2.5 m 生成长度、全长筛选/评分、5 Hz 生成、100 ms 固定
  交付模型和 20 Hz control；没有改公共 YAML 的 1 Hz / 1 m 默认规划配置。
- 规划保留 0.08 m 点云/感知余量和额外 0.08 m 跟踪余量；执行检查用 0.08 m。
  曲率上限为 2.5 1/m。这些值尚无实车标定依据。
- 已知两侧边界必须来自 observed/observed_with_history；新增横移族需要当前
  感知中线。缺少支持时本轮不承诺绕行，原 native/default 的部分道路语义不变。
- blocked、dropout、stop_only 使用原课程模型支持的负 drive 制动并观察至少
  1 秒停稳；共享保护请求依然是零动作，负 drive 没有接入 ROS。

## 验收与限制

clear 应正常完成；central/left/right 应绕过障碍并回到中线附近；blocked 和
dropout 应保护停车；stop_only 是未启用绕行的原遇障停车对照。
成功绕行要求没有障碍接触、车身不出道路、峰值不超过 0.22 m/s，任务结束时
横向偏移不超过 0.12 m、航向误差不超过 0.20 rad。启动 baseline 周期照常计数。

完整通过情况以本次 `suite.json` 为准，不能把保守停车算成完成绕行。
2026-10-24 首轮七场景全部满足各自验收；中心障碍最终横向偏移约 6.8 cm，
峰值速度约 0.204 m/s，无障碍接触。三组停车场景按停车验收，未计为绕行完成。
开发中较窄的 1.118 m 道路、1.7 m 近障场景仍出现保护停车；原始失败记录保留在
`.verification/avoidance-dev-smoke-*`，没有改写成成功案例。

当前规划计算仍较慢。5 Hz 是离线实验时钟，100 ms 是交付假设；实际 CPU
耗时保存在每个 shadow JSONL 的 `delivery.planner_wall_s`，不能拿仿真时钟
当作实时性能证据。首轮驾驶场景中位规划耗时约 0.49–0.51 秒，超过 0.20 秒
生成周期，见结果目录 `PERFORMANCE.json`。下一阶段应先降低候选/碰撞检查开销，
再评估 ROS 接线。

本版没有动态障碍预测、复杂弯道、倒车、一般遮挡/扫描覆盖证明，也没有实车
尺寸、转向、制动和时序验收。软件 ROS gate 证明既有节点与安装包回归，
不会自动使离线绕行功能成为已验证的 ROS/实车功能。

## 开发与检查入口

- `mvp21_policy/avoidance.py`：候选扩展、硬筛选、静态记忆、绕行侧、执行门控。
- `mvp21_policy/execution.py`：可选门控及 baseline fallback 的同周期复查。
- `offline/mvp21_sim/runtime.py`：原闭环的显式离线开关；默认路径不启用绕行。
- `tests/test_lidar_avoidance.py`：扫掠、断流锁存、车尾净空、旧路径重查、受阻回退等。

本轮 portable regression 为 132 项通过；新工作区独立 ROS gate 报告 407 项
（395 pytest cases 加 12 CTest groups），0 error/failure/skip。
完整日志在 `.verification/ros-avoidance-20261024/verify-fast.log`。
