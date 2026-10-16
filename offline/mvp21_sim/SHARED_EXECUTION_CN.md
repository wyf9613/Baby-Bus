# MVP2.1 兼容性边界下的统一执行

用户批准逐项合并，要求道路与边界兼容性不低于实车调试的 `mvp2.1@3652e9b`。
保留拆分模块。两个 script 均调用模块入口，`upstream/` 单文件仅为不可变来源快照。

## 合并结果

| 部分 | 实现 |
|---|---|
| 执行器 | 仅 `mvp21_policy/execution.py` 的 `PlannerExecutionManager`，ROS/仿真共用 |
| 控制交接 | 共用 `calculate_selected_control`，保留 PID 状态及同周期回滚、中线回退 |
| 原本地接入 | 删除 `reference.py`、`one_hz.py`、`planner_control.py` 及过期接线选项 |
| 本地结果隔离 | 先检查运行代次、递增任务号和输入时间，再替换或清空当前路径 |
| 路径对齐/检查 | 保留上游原始运动锚点、单调弧长进度、当前车体/连接段/控制读取区间滚动检查 |
| 生命周期 | 保留真实源时间；不增加源路径固定 TTL，不要求中线规划先成功 |
| 边界输入 | 保留单侧、短段、单点、无中线、垂直/回折与推断来源语义，不增加 guard |
| 规划范围 | 默认 `collision_coverage=prefix`、`score_coverage=prefix`、`length_m=1.0`，与上游一致 |
| 本地全长实验 | 检查与评分分开配置，1.3 m 长度需要显式指定，不能成为默认新增拒绝条件 |
| 配置 | 仿真与 ROS 同读 `config/ai4r_policy.yaml`；快照配置仅校验来源 |
| 环境与学习接口 | 原车辆、传感器、重建道路、C*、历史模型数据和三组输入接口保留，学习评分不接入 |

仿真 `runtime.py` 只连接时钟、观测、因果交付服务、车辆和日志端口；不再实现另一套
轨迹缓存、源时间到期或 PID 回退决策。共同传感器准入函数保留原有新鲜度与空锥桶规则。
本地 YAML 已选择 planner；没有部署、Enable 或 Run。

## 兼容性检查

比较器只在测试中加载不可变的原始 MVP2.1 函数。默认参数下：

- 20 条记录的规划输入逐条比较状态、选中候选、原始轨迹及边界，无新增拒绝或选路差异。
- 单侧且宽度未知、短段、单点、无中线、垂直/回折、无道路输入矩阵与原版本结果一致。
- observed/history/inferred/predicted/absent 边界与两种源年龄的执行结果、拒绝原因一致。
- 另验证无有效中线参考时仍可执行 native 路径，旧结果不能清空新路径，检查/评分范围独立。

记录 fixture 的历史对齐是近似值。这些检查证明已覆盖输入的兼容性，不是对所有实车
环境或 ROS 实时性的普遍证明。ROS fast gate 仍缺固定 `dream_interfaces` checkout。

## 运行结果与验收含义

固定 100 ms 模拟交付，原重建道路：

| 输入/参数 | 周期数 | planner 周期 | 中线周期 | 其中启动/运行后回退 | 到达目标 | native-only |
|---|---:|---:|---:|---|---|---|
| 感知，MVP 默认 | 147 | 141 | 6 | 2 / 4 | 是 | 未通过 |
| 理想地图，MVP 默认 | 148 | 143 | 5 | 2 / 3 | 是 | 未通过 |
| 感知，显式 1.3 m＋全长检查/评分 | 149 | 147 | 2 | 2 / 0 | 是 | 未通过 |
| 感知中线对照 | 152 | 0 | 152 | 对照模式 | 是 | 不适用 |

默认运行后的回退原因为 `preview_coverage_exhausted`。此处没有通过回退掩盖 native
结果；`completed` 和 `native_only_passed` 是不同字段，启动中线也计入严格验收。
保持已有速度跟踪限制，未验证末端停车或实车动态避碰。

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.mvp21_sim --output .verification/shared-native-new
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.mvp21_sim --road-input map --require-native-only --output .verification/shared-strict-new
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.mvp21_sim --lattice-length-m 1.3 --collision-coverage generated --score-coverage generated --output .verification/shared-explicit-new
```

`--require-native-only` 仅改变退出验收条件，不改变执行行为；默认参数下当前应退出 1。
完整检查、版本指纹及结果见 [shared-execution.json](artifacts/2026-10-16/shared-execution.json)。
