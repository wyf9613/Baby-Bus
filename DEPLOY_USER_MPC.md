# 新 RNN 规划器 + 用户 MPC：13号车部署入口

只使用本目录下的 scripts/policy_node.py 和 config/ai4r_policy.yaml。
不要从旧 Baby-Bus 目录拿 lattice_feedback.yaml 混用。

版本 user-mpc-trained-rnn-20261026；目标0.50m/s；左蓝右黄；转向限幅±0.50。
当前 mpc.shadow=true，只计算，不输出实际驱动。该模式也不会推进 RNN 执行记忆。
它用于接口检查，不能代表实车跟随验收。部署/Enable/Run由操作者执行。

1. 在小车 SSH 停止 policy 并备份现有脚本、YAML，然后 exit 回到 Windows。
2. PowerShell 进入本目录，上传 scripts/policy_node.py 和 config/ai4r_policy.yaml
   到 ai4r@10.43.254.26 的相同包路径。不要上传本仓库的其他车 chassis/camera/IMU YAML。
3. 小车执行 chmod +x policy_node.py，dream build ros student，成功后重启 ai4r_policy。
4. 日志版本 user-mpc-trained-rnn-20261026；参数 learned_policy.enabled=true、
   planning.cruise_speed_mps=0.5、mpc.v_exec_max_mps=0.5、mpc.shadow=true。
5. planner_execution 记录 RNN 候选、实际 reference_source、MPC后端和记忆提交次数；
   mpc_debug 记录实际求解、接入与拒绝原因。

本地准备物理试验配置时，在 config/ai4r_policy.yaml 改 mpc.shadow=false；
用 docs/USER_MPC_RNN_INTEGRATION_20261026.md 中命令重新生成单文件，然后再次
同步脚本和YAML、编译、重启。启动后确认 mpc.shadow=False。人工 Enable/Run 保持独立。
模型仍沿用旧车参数，未对13号车重新辨识；车端必须核对实际转向映射与轮速单位。

参数映射：原 vehicle.* 模型值位于 mpc_vehicle.*，避免与上游可选标定参数混淆。
未做远程上传、实车测试、Git提交或push；已有checkpoint和原目录未修改。
