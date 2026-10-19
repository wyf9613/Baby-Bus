# MPC 实车 checkpoint 与当前进展（2026-10-19）

## 发布版本

- checkpoint: `checkpoint-user-good-motion-steer050-20261019-073353`
- 启动标识: `local-feedback-lattice21-motion-20261019`
- 原代码与配置按 `checkpoint-20261019.json` 中 SHA256 保存；不是后来尝试的 smooth-single-side 或 fullsteer 版本。
- 转向由 MPC 求解；纵向采用轮速反馈 PI / 前馈和起步处理。
- 归一化转向限幅 ±0.50，转向增益 -0.3142 rad/unit；沿用原车辆模型参数，并非完整重新辨识。
- 锥桶布局：左黄（1）、右蓝（2）。执行目标上限 0.25 m/s；该配置关闭超速锁停、累计里程及时间限制，仍保留传感器、参考有效性和几何拒绝检查。目标速度不等于硬件速度保证。

## 本周进展

1. 改善低速反复起停：通过 wheel-speed、动作输出、状态机和 debug 区分纵向收油和参考拒绝；调整起步、维持油门及轮速反馈，减少达到目标后零油门导致停住的现象。
2. 中心线方案可连续行驶并跟随部分弯道，但用户实测成功率不稳定，急弯不能可靠跟随。
3. 原有 MPC 接入新 lattice，处理投影、裁剪、近车补接及预测支撑范围匹配。用户反馈路径跟随明显改善。
4. 维护 policy/YAML 同步部署、版本标识、日志诊断和可回退 checkpoint。

## 当前未解决事项

- 接近道路末端、急弯或左右锥桶不能完整观测时仍可能停车，未宣称完成连续全程或急弯验收。
- 实车记录出现 `stop:invalid_road_invalid_boundary_pairing`：左右拟合无法形成足够有效配对，且发现冲突宽度；不能仅凭状态认定是误检或安全越界。
- 实车记录出现 `ref_invalid:start_bridge_alignment`：近端补接在运动中也检查 |e_y| <=0.20 m、|e_psi| <=0.35 rad。存在 0.1977 -> 0.2014 m 变化导致允许油门后停车的例子；也有 0.205 m / 0.412 rad 同时超限的例子。
- 另见 `origin_before_path_start`。需记录原始锥桶、拟合道路、裁剪轨迹、参考来源和车辆反馈，区分实际偏离与参考跳变；不应直接删除几何检查。
- 停车样本中存在控制计算约 25.6 ms、轮速/角速度年龄约23/28 ms、参考剩余0.238 s的记录：这些样本未显示求解超时、反馈断流或参考过期；不代表所有运行片段都不存在这些问题。
- 后续尝试过 QP 转向指令变化率约束、有限时效单侧路宽记忆及单侧降速，但尚未确认实车效果；此次 checkpoint 不包含这些尝试。

## 验证边界

历史 checkpoint 便携测试记录为 268 项：266通过，2项因缺少OSQP跳过。Windows 便携测试不能代表 Jetson 实时性或物理车辆安全。ROS fast gate 缺少 pinned dream_interfaces / ROS 环境，未完成；启动和实车表现来自用户提供日志与反馈。

## 部署

将 `scripts/policy_node.py` 与 `config/ai4r_policy_lattice_feedback.yaml` 同时上传；后者在车端保存为 `src/ai4r_policy/config/ai4r_policy.yaml`。车端停止 policy、备份旧文件、处理 LF 和执行权限、执行 `dream build ros student` 后重启。核对版本标识、左右颜色和 mpc.shadow=False。车辆 Enable/Run 由操作者手动执行。

中心线对照使用 `config/ai4r_policy_speed_feedback.yaml`。基础 `config/ai4r_policy.yaml` 保留 shadow；不要将不同用途的配置混用。

## 本次发布复核

在隔离发布目录运行 336 项便携测试：334通过、2项OSQP跳过。测试涵盖估计、规划、控制、MPC、中心线/新lattice接入、短路径和预测修复。测试模块加载后按生产入口方式执行 gc.collect()/gc.freeze()；初次未在加载后冻结的运行触发一项桌面计时超预算，未改 checkpoint 或放宽其运行预算。ROS fast gate 在缺少 pinned dream_interfaces 时停止。代码、三份主配置、package.xml、launch 与保存的 checkpoint 校验一致。

本提交只追加到 feature/mpc-prediction-model，不更新 main 或其他成员分支，不包含后来回退的平滑/单侧道路补丁。
