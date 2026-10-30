# MVP RNN 任务式雷达绕行 r6

版本 `mvp21-rnn-maneuver-20261030-r6`，开发分支 `ying/mvp21-rnn-maneuver-r6`。
来源与具体切换边界见 [对齐说明](RNN_MANEUVER_ALIGNMENT_CN.md)。

普通巡线使用原生 Lattice + 原冻结 RNN；绕行任务使用原 B 的 SHIFT/PASSING/RETURN。
控制模块与 B r5 相同，前馈0.30、驱动上限0.35，目标速度按用户选择改为0.50m/s。
自动识别左右蓝黄布局，映射在本次 Run 内锁定。严格 IMU、雷达时效、车身侵入和路径检查保持。
原模型速度条件和 SHA-256 校验保留，未接入 MPC。

固定左绕，30cm车宽，障碍/跟踪/边界余量2/1/1cm。
默认档保留车头前15cm门禁；诊断档只关闭 `obstacle_stop.proximity_stop_enabled`，
`obstacle_stop.enabled` 始终 true。道路无效、任务不可行和数据失效仍可能停车。
启动保持零动作；锁停后只通过显式 Run 恢复。

部署时完整上传本包 Python 与 YAML。模型已嵌入单文件，不需要单独上传。
保持车辆 Disabled，先停 policy，备份车端原两份文件，再上传、build/start。
本轮未连接、上传或启动车辆；实际控制和物理刹停距离未验收。
0.50m/s 是参考目标，保留的驱动上限不保证实车达到该速度。

`/car/planner_execution.selection_backend` 显示 learned_policy 或 maneuver；
`learned_policy.state_commits` 只记录已执行普通巡线路径的 RNN 状态提交。
绕行期间该计数不增加，切换时清空记忆并隔离旧规划结果。
