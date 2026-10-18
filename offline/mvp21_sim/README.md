# MVP2.1 统一模块执行与原虚拟环境

当前统一入口是 `python -m offline.mvp21_sim.validation`，支持 init、add-road、run、
index、serve（默认自动重验/刷新）和 package。详见
[模块验证使用流程](VALIDATION_WORKFLOW_CN.md)。旧实验记录保留，不应手工拼接新的 HTML。

当前分支将实车 MVP2.1 的执行机制与本地结果隔离、Frenet 导出合并。
ROS 与仿真共用 `mvp21_policy/execution.py`；单文件仅为来源快照。
原课程车辆、RK4、重建的 42 个锥桶、10 Hz 相机与 0.20 s 感知延迟保留。

运行链路为：估计、时序融合与 native 规划 → 共同执行器 → 原 PID → 车辆端口。
保留中线作为 MVP2.1 的降级参考；每周期记录实际来源。学习 policy 暂不接入。

详细合并边界、兼容性检查与结果见 [SHARED_EXECUTION_CN.md](SHARED_EXECUTION_CN.md)。

## 共同配置

ROS 与仿真均读取仓库 `config/ai4r_policy.yaml`。默认 planner 1 Hz、控制 20 Hz、
路径长度 1.0 m、前缀碰撞检查/评分、滚动执行检查、原中线降级。
`source.py` 校验上游快照并记录当前配置指纹；快照 YAML 不再控制仿真。

`--path-source perception` 可显式做中线对照。`--collision-coverage`、
`--score-coverage`、`--lattice-length-m`、`--planner-rate-hz` 仅作显式参数实验，
使用相同执行实现。没有 `cached/control/upstream` 三套接线模式。

## 运行

在仓库根目录，使用原虚拟环境，输出目录必须尚不存在：

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.mvp21_sim --output .verification/shared-native-new
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.mvp21_sim --road-input map --output .verification/shared-map-new
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.mvp21_sim --path-source perception --output .verification/shared-baseline-new
```

`trace.jsonl` 保存参考、实际动作、执行来源及原因；`shadow.jsonl` 保存 native 候选和
延迟交付；`summary.json` 分开报告到达目标和严格 native-only 验收。当前默认两组
均到达目标，但存在中线周期，所以 native-only 未通过。

`--require-native-only` 仅改变退出验收：任何中线周期（包括启动等待）均返回失败。
它不会改变回退行为。`--no-shadow` 仅用于中线旁路隔离对照。

## 来源与限制

上游固定 `mvp2.1@3652e9b`，版本化原样快照在 `upstream/3652e9b/`；
最初 `f11428b` 快照继续保留。本地保护清单核对历史模型、数据、车辆和 C* 求解器。

[原接入同步记录](UPSTREAM_SYNC_20261016.md)、[旧 1 Hz 缓存实验](ONE_HZ_CONTROL.md)
为历史证据，其独立执行代码与接线选项已经删除。

仿真延迟是因果服务模型；没有启动 ROS、部署或连接车辆。速度跟踪、物理停车和
完整车身动态避碰仍未验收。ROS fast gate 缺固定 dream_interfaces 依赖。

## 固定道路的 20 Hz 系统回放（2026-10-18）

`validate_roads` 使用同一 runtime 循环、共享 execution 与实际 MVP
`PolicyController`，相机改为每 50 ms 采集一帧（延迟 50 ms），native 每秒
提交一次。规划结果在仿真中延迟 100 ms 交付，期间继续控制；不等候整条候选
执行结束。保留 baseline 回退，不接入 C* 控制或 learned policy，不加场景宽度先验。

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.validate_roads --output <new-directory>
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.visualize_runtime --input <new-directory>
```

八条固化道路分别做入口整路与弯前静止启动。完整原始输入、7 维车身状态、
逐帧 control/参考来源、规划提交与交付、故障零指令滑行保存在每次目录。
`viewer.html` 支持逐帧、播放、按规划交付跳转，以及道路、车辆姿态、感知、
速度和控制时序。原 C* 训练采集/验证入口继续用于其原有用途。

当前入口测试 3/8 到达，均用到 baseline；三次障碍接触、两次单侧输入下执行
检查失败后无可用 baseline。弯前启动 0/8，首个控制失败使规划任务尚未提交。
启动速度峰值约 0.776 m/s，目标仍为 0.2 m/s。见
[报告与问题定位](../lattice_training/artifacts/2026-10-18-mvp-runtime-validation/REPORT_CN.md)。
这份报告验证仿真时钟和模块接线，CPU 实测耗时单独保存，未认定 ROS 实时或实车效果。

## 道路感知三层诊断

对已有 runtime 记录运行只读审计，随后刷新同一回放页面：

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.diagnose_perception --input <runtime-directory>
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.perception_report --input <runtime-directory>
```

完整环境锥桶通过同一固定配置/种子重建，并逐包复核模拟传感器查询；检测点必须
按采集时刻的记录姿态对齐，不能直接使用交付时刻姿态。真值仅用于显示和审计。
页面叠加全部锥桶、实际收到的检测点、原感知导出的边界/中线；native 执行参考
使用独立虚线。可选成链诊断显示当前包的链方向，不替代原感知输出。
原始点表附带采集坐标、真值锥桶 ID、路线段和末端延伸标记，便于发现串支路。

本次连续掉头在 7.25→7.30 s 出现近横向远支路链方向翻转，随后配对失败、
已观测边界未导出、记忆宽度清除。两种颜色同时可见不代表当前支路两侧均可见。
诊断未修改算法、参数、原运行轨迹或控制量；详见
[感知诊断报告](../lattice_training/artifacts/2026-10-18-mvp-runtime-validation/PERCEPTION_DIAGNOSIS_CN.md)。

## 冻结 policy 的当前频率 evaluation

已知限制固定记录在 [SIMULATION_LIMITATIONS_CN.md](../../docs/SIMULATION_LIMITATIONS_CN.md)。
新的显式 `--policy-model` 只在仿真 worker 内替换 native 接纳候选的排序，不启用
ROS 主线 policy，不改模型、输入编码、生成/控制参数，也不旁路共享 execution。
每个 1 Hz 作业使用提交时刻的当前感知快照逐条评分；20 Hz control 持续执行。

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.validate_roads --policy-model offline/lattice_training/artifacts/2026-10-17-expanded-collection/model.json --output <new-directory>
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.diagnose_perception --input <new-directory>
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.policy_evaluation --input <new-directory> --native-baseline offline/lattice_training/artifacts/2026-10-18-mvp-runtime-validation
```

比较页面保留三层感知图，可切换 policy/native，并查看同一候选池的两种选择和
逐候选分数；大型案例按需加载。全部分数、输入指纹和 PID 动作均可独立重放。
固定 100 ms 交付和实测 CPU 用时分别报告，不把逻辑 1 Hz 提交当作实际算力验收。
当前冻结模型在八条入口道路上 0/8 到达，native 为 3/8；八个弯前启动均未进入
评分。生成＋评分有 11 次超过 1 s。见
[评估报告](../lattice_training/artifacts/2026-10-18-current-frequency-policy-evaluation/REPORT_CN.md)。

## 历史随机道路接入同一回放

`random_roads.py` 从已有 collection ZIP 恢复实际保存的 `road_spec`，移除已
单列记录的末端支撑后核对任务长度。首批三条与扩采道路共有 28 个旧 scene ID，
实际是 33 个几何/障碍版本；保留共用 scene ID 的五个障碍改版和空采集序列。
旧数据和模型不变，旧 train/validation/test 分组只作来源说明。

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.evaluate_random_roads --output <new-directory>
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -m offline.mvp21_sim.random_evaluation_report --input <new-directory> --viewer <existing-policy-viewer-directory>
```

33 道路 × 原感知/受控删除中线 × native/冻结 policy，共 132 次入口运行。
继续使用相机/control 20 Hz、规划/评分 1 Hz、实际 MVP PID 和共享回退。
不读取场景宽度先验；删除中线不是相机自然缺失。入口门控可能在首次规划/评分
前停止，必须和模型选路失败分开。原感知输出及实际送入模块的观测分别保留。
原固定掉头回放继续存在；新增道路集和道路输入筛选，分母按当前道路集计算。
