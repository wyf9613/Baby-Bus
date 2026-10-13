# 10 Hz / 100 ms 配置部署

2026-10-13，按用户要求将当前 mvp2.0 的 planning.lattice.budget_s 从 0.035 改为 0.1 s。规划原已 10 Hz，控制仍 20 Hz，候选上限仍 16。未修改 policy 源码、源年龄/参考期限、车身约束或启动确认规则。

车端先请求 policy=2 和 Disable，再停止 policy；备份源码/配置/git diff 至 /home/ai4r/mvp2-test-20261013/budget100/before-budget100-<时间>/，仅部署 policy YAML，重启 policy。源码哈希 a28fab188e58a86239aeb5a4922b092e06c2015cb95ea2f5ff7c4427d413e2c5 保持；配置源码及安装路径哈希均为 802e53ed4fd9702710139949a9fcca6f1fb1d7276065dbdd1b46d9778262ab4a。

实际节点 GetParameters 回读：planning.lattice.update_rate_hz=10.0、planning.lattice.budget_s=0.1、policy_update_rate_hz=20.0、planning.lattice.max_candidates=16。部署完成。

未发布 Enable 或 policy=3，未进行运动测试或额外自动测试。此配置尚无 3 m 实车效果证据。完整部署输出在车端 /home/ai4r/mvp2-test-20261013/budget100/deploy-console.log；后续有界测试脚本已匹配新配置哈希。

## 100 ms 预算实车测试

drive-budget100-122645：保持已回读的规划 10 Hz / 100 ms、控制 20 Hz、16 候选上限；通过 3 个不同帧的启动确认，进入 TRACKING 并能限时降级。首次非零驱动在请求启动后约 1.656 s；最大请求驱动 0.303051，记录应用驱动最大 0.272000。
轮速和里程始终 0，4.006 s 后外部 no-motion 限制停止；现场用户确认车轮和车身都没有动。该轮未起动的具体电调/电机/驱动力原因未确认。最终 confirmed_stop=true、policy=2、vehicle=Disabled、动作归零。
拒绝计数 {'invalid_estimates_or_boundaries': 3, 'source_deadline_during_planning': 10}；丢弃结果细节 {'reference_expired': 10}；有日志的规划耗时范围 [5.308999971020967e-05, 0.07293025199942349] s（非每次规划的完整性能分布）。原始日志 vehicle/ 下对应 drive-* 的 result.json / run.jsonl，policy 日志在 vehicle/budget100/。没有执行额外离线测试或继续加大驱动。
