# 最新 MPC + 雷达部署配置

基线 `fusion-prior-reset` / `d1b4b83`，功能分支 `feature/fusion-mpc-lidar-detection`。

- 上传仓库 `scripts/policy_node.py` 到车上的同名位置。
- 上传本目录 `ai4r_policy.yaml` 到车上的 `config/ai4r_policy.yaml`。
- Foxglove导入 `tools/foxglove_road_visualization/layout_v2.0.json`。
- 完整哈希见本目录 `manifest.json`；本目录不维护另一份程序副本。

当前设置：0.5m/s、黄左1/蓝右2、距离/运行时间均不限；车头前1.4m、左右各0.6m雷达检测。
检测器专用停车关闭；规划启用 `shadow.obstacle_avoidance_enabled` 和 `shadow.obstacle_check_enabled`。
车宽0.30m、额外余量0.0m；车身与障碍重叠候选排除，道路边界检查覆盖整条路径。
已知障碍下等待合格路径时零输出；无可行候选进入状态2，需手动Run，不退回中心线硬挤。
MPC算法、采样、规划频率和长度保留；沿用上游车辆参数，不代表车14重新标定。
最新MPC的短帧几何异常容忍和保留已学路宽开关已启用，融合旧参考关闭。
已检查路径可在道路几何短暂失效时续用0.5秒，NO_ROAD不清空路径或续期；明确无路可绕仍停车。
其他原有停车原因保留；没有移植旧RNN分支的2m候选或0.55rad接入改动。

运行标识 `fusion-mpc-lidar-road-hold-20261028`。
2026-10-28实车反馈：仍存在驶出道路的问题，尚未解决；当前版本作为调试进度保存。
新增绕行离线检查覆盖路径评分、worker传输及实际节点到MPC的路径接入。
299项离线回归通过，包含17项绕行/通行性和5项短时失边检查；MPC算法保持。
本机缺少Bash/ROS Jazzy，未完成完整ROS gate或实车/GUI验证。未由助手上传车辆。
详细说明见 `docs/FUSION_MPC_LIDAR_CN.md`。
