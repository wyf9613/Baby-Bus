# main 上周实车配置与当前 MVP2 对比（2026-10-13）

origin/main 已拉取，HEAD 7c19ab1；上周 2026-10-06 的记录已归档于 main。
比较的是 main 中成功实车部署的 deployed-config.yaml，不是仓库根 YAML 默认值。
归档配置 SHA256：80ca460f053b29f8e70a5050b47802f28dd0c3911afe505f2b640d599d058fcd，与本地证据一致。

| 项目 | 上周实车部署 | 当前部署 |
| --- | --- | --- |
| mvp_drive_feedforward | 0.30 | 0.30 |
| drive_max | 0.35 | 0.35 |
| speed_kp / speed_ki | 0.35 / 0.08 | 0.35 / 0.08 |
| cruise_speed_mps | 0.2 | 0.2 |
| 控制频率 | 20 Hz | 20 Hz |
| 实际起步请求 drive | 0.35 | 首次约 0.302，整轮最大 0.303051 |
| 轮速最大值 | 0.312371 m/s | 0 |

上周 V1 直接向控制器传巡航目标 0.2 m/s。静止时前馈 0.30 加比例项 0.35*0.2，触及 drive_max=0.35。
当前 lattice 从测得零速构建 3 秒纵向加速曲线，只取 0.3 秒预瞄速度。对静止、直线且终端速度 0.1/0.2 m/s 的曲线，预瞄速度约 0.0028/0.0056 m/s。每次仍测得零速时重新规划，预瞄目标持续偏小，控制输出靠近前馈 0.30，与当前约 0.302–0.303 的请求吻合。

结论：控制增益和上限没有比上周调低；lattice 纵向起步目标与 MVP 控制器的衔接是当前重点。自动输出低于启动门槛是合理假设，尚未标定门槛。上周归档车辆 IP 为 10.43.254.27，当前为 10.43.254.20，不能仅凭归档确定两车物理启动门槛相同。

当前请求约 0.303 对应记录应用值 0.272、PWM 1568 us；固件源码中的主机死区重映射可以解释这个差值，不能据此认定指令丢失。遥控器可行驶是用户现场确认。

本次仅检查与记录，没有修改控制参数，没有再次驱动车辆。

证据：
- diagnostics/newcar-27-20261006/newcar-test-evidence/deployed-config.yaml
- diagnostics/newcar-27-20261006/newcar-test-evidence/compensated-repeat.jsonl
- diagnostics/newcar-27-20261006/newcar-test-evidence/compensated-repeat-result.json
- diagnostics/mvp2-20261013/budget100/RUN_SUMMARY.json
- .worktrees/rule-lattice-control/scripts/policy_node.py 的 lattice 预瞄目标与 MVP 控制计算
