# 2026-10-13 实车鲁棒性测试记录

部署本地候选 a28fab188e58… / YAML 22f5d6d960b1…，参数未调：启动 3 s、3 个不同观测帧确认、lattice 候选上限 16、硬余量 0.02 m、3 m/30 s 限制。未执行额外离线测试。车端两文件已备份，源码/安装路径哈希相同。

| 测试 | 时长 | 最大请求驱动 | 最大轮速 | 结果 |
| --- | ---: | ---: | ---: | --- |
| drive-robustness-120930 | 4.020 s | 0.3032 | 0.000 m/s | No wheel motion after 4 seconds |
| drive-robustness-repeat-121106 | 3.033 s | 0.0000 | 0.000 m/s | Publishing zero actions: Control startup history deadline exceeded; last planning reason: source_deadline_during_planning; explicit resume required |
| drive-robustness-third-121349 | 3.068 s | 0.0000 | 0.000 m/s | Publishing zero actions: Control startup history deadline exceeded; last planning reason: insufficient_lattice_coverage; explicit resume required |

首轮通过 3 帧确认并进入 TRACKING/DEGRADED；记录实际应用驱动最大约 0.276，底盘 Enabled、主机许可正常，但轮速一直 0。未有同步人工观察，不能确认车轮物理响应或断言具体底盘原因。
第二轮多次规划超时/结果时效耗尽，确认计数反复归零，3 s 期限停车。
第三轮持续 insufficient_lattice_coverage，3.068 s 停车；本轮没有生成驱动力。该原因表示共同支持不符合规划条件，不代表锥桶总数不足。
三轮里程均 0，结束均确认 policy=2、vehicle=Disabled、零动作和零轮速。3 m 运行未完成。
原始资料：vehicle-all/ 下的三个 drive-* 目录（result.json、run.jsonl）及 robustness/ 下的部署、policy 日志；汇总 RUN_SUMMARY.json。

## 第四轮（用户要求原参数重试）

drive-robustness-fourth-121706：3.036 s，持续 insufficient_lattice_coverage；请求驱动、应用驱动、轮速及里程均 0。确认 policy=2、vehicle=Disabled。
末帧原始检测最近蓝锥 0.585 m，最近黄锥 0.960 m；黄侧起点超过当前 0.9 m 近端门限，但该原始末帧不等同于所有 worker 输入的最终拟合支持范围。
原始证据位于 vehicle-fourth/；拒绝计数：{'invalid_estimates_or_boundaries': 3, 'insufficient_lattice_coverage': 21}。未改参数或运行离线测试。

## 收窄道路后重试

drive-robustness-narrow-121953：参数/候选版本保持；多次收到 lattice_selected，但时间预算/源期限失败打断 3 帧启动确认，3.014 s 后以 lattice_time_budget_exceeded 为最后原因达到启动期限停车。请求驱动、应用驱动、轮速及里程均 0。确认 policy=2、vehicle=Disabled。
日志记录确认成功事件 7 次，拒绝计数 {'invalid_estimates_or_boundaries': 2, 'lattice_time_budget_exceeded': 7, 'source_deadline_during_planning': 8}。原始资料 vehicle-narrow/。未执行额外离线测试。
