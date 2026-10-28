# 最新 MPC + 雷达部署配置

基线 `fusion-prior-reset` / `d1b4b83`，功能分支 `feature/fusion-mpc-lidar-detection`。

- 上传仓库 `scripts/policy_node.py` 到车上的同名位置。
- 上传本目录 `ai4r_policy.yaml` 到车上的 `config/ai4r_policy.yaml`。
- Foxglove导入 `tools/foxglove_road_visualization/layout_v2.0.json`。
- 完整哈希见本目录 `manifest.json`；本目录不维护另一份程序副本。

当前设置：0.5m/s、蓝左2/黄右1、距离/运行时间均不限；车头前1.4m、左右各0.6m雷达检测。
本次选定版本：通过总宽0.30m、最小转弯半径0.9m、`mpc.r_ddelta=4.0`；
路径一致性权重4.0、比较前0.6m、切换评分余量0.05，保留已检查路径的短时续用。
检测器专用停车关闭；规划启用 `shadow.obstacle_avoidance_enabled` 和 `shadow.obstacle_check_enabled`。
规划检查总宽0.30m、额外余量0.0m；车身与障碍重叠候选排除，道路边界检查覆盖整条路径。
已知障碍下等待合格路径时零输出；无可行候选进入状态2，需手动Run，不退回中心线硬挤。
MPC算法、采样、规划频率和长度保留；沿用上游车辆参数，不代表车14重新标定。
最新MPC的短帧几何异常容忍和保留已学路宽开关已启用，融合旧参考关闭。
已检查路径可在道路几何短暂失效时续用0.5秒，NO_ROAD不清空路径或续期；明确无路可绕仍停车。
其他原有停车原因保留；没有移植旧RNN分支的2m候选或0.55rad接入改动。

运行标识 `fusion-mpc-lidar-path-continuity-20261028`。
2026-10-28实车反馈：仍存在驶出道路的问题，尚未解决；当前版本作为调试进度保存。
新增绕行离线检查覆盖路径评分、worker传输及实际节点到MPC的路径接入。
318项离线回归通过，包含26项绕行/通行性、8项短时失边和7项路径连续性检查；MPC算法保持。
本机缺少Bash/ROS Jazzy，未完成完整ROS gate或实车/GUI验证。未由助手上传车辆。
详细说明见 `docs/FUSION_MPC_LIDAR_CN.md`。

## 最小连续性修补

在已推送6efe99d上，补充共同覆盖不足、单边路宽暂缺两类短时道路失效。
已有检查通过且仍可执行的路径继续运动对齐并交给原MPC跟踪，避免无必要的回正/零驱动脉冲。
期限仍为0.5s；超时、运动/边界冲突、无路可绕仍停车，NO_ROAD不续期。
诊断`road_dropout_trigger`记录触发原因。配置和MPC不改，不含后续simple实验。
空扫描生成路径遇到新回波时，复用原车身扫掠检查当前扫描；不挡路则继续原MPC，
实际重叠仍等待，确认无路可绕仍停车。原规划期标记不改，后续扫描继续检查。
执行期检查通过的路径也可参与原0.5s失边续用；诊断`waiting_cause`记录等待的具体原因。
新版本运行标识`fusion-mpc-lidar-scan-continuity-20261028`；先留本地，实车待验证。

## 平缓转弯试调

当前`shadow.min_turn_radius_m=0.9`，候选曲率上限约1.11/m；0关闭。
处理在道路/雷达检查和评分之前执行，检查对象与交给MPC的候选一致。
保持1.4m候选长度、0.6s MPC时域和0.5m/s目标；未改变车辆接口转向限幅。
初版标识`fusion-mpc-lidar-radius1m-20261028`；当前运行标识见顶部。
这是几何试调，实车驱动平滑度待验证；没有承诺狭窄U弯仍能通过。

## 路径切换与转向试调

新增`shadow.weight_consistency=4.0`、`continuity_window_m=0.6`、`switch_cost_margin=0.05`；
在本轮已通过道路/雷达检查的48条候选中，减少相对正在执行路径的不必要形状变化。
收益不足0.05分时优先形状更接近的合格候选；不原样锁住旧路径，不恢复fusion旧引导。
`mpc.r_ddelta: 2.0→4.0`；其他MPC配置、目标速度和路径长度保持。
`path_continuity`诊断在原shadow话题中显示。当前修补仅本地，需同时上传程序和完整YAML。

U弯微调：最小半径0.9m，检查总宽按最新要求回退0.30m。代码不变，只需上传本目录最新YAML并重建/重启policy。
完整配置保留蓝左黄右、连续性评分和r_ddelta=4；当前配置实车待验证。
