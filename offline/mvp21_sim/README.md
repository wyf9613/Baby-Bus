# MVP2.1 统一模块执行与原虚拟环境

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
