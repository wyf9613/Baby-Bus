# 15 号车当前控制版本实测

2026-10-14，`fix/newcar-control`，15 号车 `10.43.254.28`。
用户确认已就位并授权起跑；复测反馈“完美完美，直接push这版本”。
保存本轮通过弯道的算法与参数，未在复测后再次调参。

## 当前行为

左黄（1）右蓝（2），20 Hz 定时策略，目标速度 0.2 m/s。
取消距离/时间试跑截止，保留观测、运动数据和参考有效期检查及独立归零。
双侧可用时走估计中线，并从新鲜原始双侧观测学习宽度；单侧缺失时，将可用边界
沿局部法向向内偏移记忆半宽。另一侧返回但共同范围不足时，暂用近侧边界；支持足够后恢复双侧。
半宽在单侧段保持，停止/重新起跑清除记忆；冷启动不猜测道路宽度。

横向 Kp=2.5、航向 Kp=1、方向=-1，策略转向上限 ±1、变化率 2/s，车辆接口区间 ±0.8。
Foxglove 绿线来自控制器已采用参考；本轮保留用户提供的可视化代码和布局。
停车/故障直接归零，绕过普通转向变化率限制。策略不负责 Enable 车辆。

## 执行偏置排查与复测

上一轮早段碰蓝桶时，实际归一化转向约为 `0.8 × 策略请求 + 0.5`。
启动参数仍为 0，实时 trim 话题与诊断却为 0.5。设置来源未被记录，无法归因。
在策略 2 / Disarmed 下用公开接口恢复实时 trim=0，读回话题与诊断均确认。
本轮外部试跑脚本起跑前核对实时 trim，记录 trim set/increment，运行中偏置改变即结束。

| 本轮记录 | 结果 |
| --- | --- |
| 运行时间 / 轮速积分 | 19.746 s / 3.817 m |
| 峰值轮速 | 0.310 m/s |
| 实时 trim | 全程 0.0，未记录到 set/increment 命令 |
| 实际转向 − 0.8×策略转向 | 中位数 0.000036，178 个匹配样本 |
| 参考来源（约 2 Hz Tracking 日志） | 双侧 27、黄侧偏移 8、蓝侧偏移 2 |
| 可视化 | 147 条非空 Path，最多 34 点，base_link |
| 最终停车原因 | `Planning: invalid_road_insufficient_geometry` |
| 最终状态 | 策略 2、车辆 Disabled、驱动/轮速归零已确认 |

停车前记录帧中，黄/蓝侧前向覆盖约 0.209/0.203 m，均低于 0.30 m 拟合覆盖要求。
该记录帧不保证与策略使用的故障帧完全相同；故障日志确认运动状态有效、道路几何不足。
停车空 Path 在这轮记录中未确认接收，不能标记为清线通过。
轮速积分和内部误差不是外部定位真值；本轮现场反馈良好，尚未完成重复圈次验收。

## 可复查证据

- [原始记录](evidence/car15/visualization4/run.jsonl)、[结果](evidence/car15/visualization4/result.json)、[汇总](evidence/car15/visualization4-review.json)。
- [策略日志](evidence/car15/visualization4-policy.log)、[试跑控制台](evidence/car15/visualization4-console.log)、[实际使用的外部试跑脚本](evidence/car15/run_visualization.py)。脚本是本轮历史记录，固定目录不得原样重复执行。
- [实时 trim 恢复记录](evidence/car15/steering-trim-restore-final.log)、[软件检查](evidence/car15/car15-disjoint-return-fast-gate.log)、[运行文件哈希](evidence/car15/car15-disjoint-return-manifest.json)。

最终相关运行代码已完成 pinned ROS Jazzy gate：250 项，零失败/错误/跳过；portable 164。
本次试跑及提交前比对本地、部署与检查清单，运行文件哈希一致；复测后只追加证据/文档。
