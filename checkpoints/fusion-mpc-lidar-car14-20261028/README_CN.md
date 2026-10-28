# 最新 MPC + 雷达部署配置

基线 `fusion-prior-reset` / `d1b4b83`，功能分支 `feature/fusion-mpc-lidar-detection`。

- 上传仓库 `scripts/policy_node.py` 到车上的同名位置。
- 上传本目录 `ai4r_policy.yaml` 到车上的 `config/ai4r_policy.yaml`。
- Foxglove导入 `tools/foxglove_road_visualization/layout_v2.0.json`。
- 完整哈希见本目录 `manifest.json`；本目录不维护另一份程序副本。

当前设置：0.5m/s、黄左1/蓝右2、距离/运行时间均不限；车头前1.4m、左右各0.4m雷达检测。
雷达只检测和显示，专用停车关闭，未接入主动绕行。沿用上游车辆调参，不代表车14重新标定。
最新MPC的短帧几何异常容忍和保留已学路宽开关已启用，融合旧参考关闭。
其他原有停车原因保留；没有移植旧RNN分支的2m候选或0.55rad接入改动。

255项离线集成检查通过；不限预算的3项既有检查通过；ROI更新后11项雷达检查复跑通过。
本机缺少Bash/ROS Jazzy，未完成完整ROS gate或实车/GUI验证。未由助手上传车辆。
详细说明见 `docs/FUSION_MPC_LIDAR_CN.md`。
