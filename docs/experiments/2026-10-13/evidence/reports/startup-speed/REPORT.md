# MVP2 起步目标 0.2 m/s 部署测试，2026-10-13

已部署 MVP lattice 非停车目标下限修复；车端回读巡航 0.2 m/s、油门上限 0.35、10 Hz / 100 ms 规划、20 Hz 控制、16 候选上限。源码与 installed 路径哈希一致，测试结束后再次确认源码和配置哈希。

部署前 policy 仍为上一轮版本，但 YAML 被换成 cone follower 配置。首次部署被哈希保护中止；已备份原文件和差异，按本轮指定的 MVP2 配置恢复后继续。

两轮都未完成起步前 3 个不同观测帧确认：最多到 2/3，过期结果打断并清零计数，最终超过 3 秒启动时限。油门、轮速、里程均为 0。两轮均确认最终状态 2 / Disabled、零动作和静止轮速。

尚未执行到控制阶段，不能据此判断 0.2 m/s 下限的起步效果。直接阻塞是 source_deadline_during_planning / 参考过期与连续确认。没有修改这些门限，没有运行额外离线测试。

| 运行 | 时长 s | 确认帧峰值 | 最大驱动 | 轮速 | 确认停止 |
| --- | ---: | ---: | ---: | ---: | --- |
| drive-startup-speed-floor020-01 | 3.048 | 2/3 | 0.0 | 0.0 | True |
| drive-startup-speed-floor020-02 | 3.041 | 2/3 | 0.0 | 0.0 | True |

原始证据在 vehicle/；汇总为 RUN_SUMMARY.json，部署身份与哈希为 manifest.json、vehicle/startup-speed/final-sha256.txt。
