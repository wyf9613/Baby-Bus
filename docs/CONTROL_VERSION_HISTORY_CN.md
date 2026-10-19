# 控制版本记录与回退

2026-10-14：用户选择 main 默认采用“弯道激进版”，保留温和版本历史；
状态机及未上车修复另存实验分支，不合入 main。

| 版本 | 源提交 | 保留方式 | 实测范围 / 已知问题 |
| --- | --- | --- | --- |
| 温和转弯版 | [271bfb9](https://github.com/wyf9613/Baby-Bus/commit/271bfb9) | 完整历史合并；温和版本合并节点 55d50ff | 缓弯与末段直行获得正面反馈；未证明急弯/U 弯能力 |
| 弯道激进版（main 默认） | [5e8d1b8](https://github.com/wyf9613/Baby-Bus/commit/5e8d1b8) | 完整历史合并；激进版本合并节点 f86561d | 用户反馈急弯改进可用；U 弯仍有问题，未完成外部定位精度验收 |
| 状态机实验版 | [3aa06e8](https://github.com/wyf9613/Baby-Bus/commit/3aa06e8) | [feature/newcar-turn-fsm-experiment](https://github.com/wyf9613/Baby-Bus/tree/feature/newcar-turn-fsm-experiment)，不合入 main | 初版直角弯两轮停车；PREPARE/共同支持等修复尚未上车，仅通过软件检查 |

温和版和激进版按先后分别 merge，未 squash，旧策略及配置可直接按源提交取回。
main 只有一个默认运行配置；记录旧提交不会在当前策略中同时启用另一套算法。

## 参数差异

| 设置 | 温和版 271bfb9 | 激进版 5e8d1b8 |
| --- | --- | --- |
| 道路编码 | y(x) | 二维有序 XY / 弧长 |
| 单侧边界向内偏移 | 完整宽度的 0.5 倍 | 完整宽度的 0.6 倍 |
| 横向 Kp / Ki | 2.5 / 0 | 2.5 / 0 |
| 航向 Kp | 1.0 | 1.5 |
| 普通转向变化率 | 2/s | 4/s |
| 策略 / 车辆接口转向范围 | ±1.0 / ±0.8 | ±1.0 / ±1.0 |
| 目标速度 | 0.2 m/s | 0.2 m/s |
| 油门前馈 / 上限 | 0.30 / 0.35 | 0.30 / 0.35 |
| 几何不足后的末段直行 | 0.8 m | 关闭 |
| 入弯状态机 | 无 | 无；实验分支另存 |

车辆接口区间是归一化输出范围，不是实测前轮角；不同车辆仍需各自校准。
温和版的末段直行按几何不足触发，不会识别真正的赛道终点，弯道缺桶也可能触发。

## 取回旧版而不改 main

在本地仓库运行：

```bash
git worktree add --detach ../ai4r-policy-gentle 271bfb9
```

该目录保存温和版完整源码和配置，main 不变。仅查看配置可以运行：

```bash
git show 271bfb9:config/ai4r_policy.yaml
git show 5e8d1b8:config/ai4r_policy.yaml
```

上车回退应复制对应版本源码/配置并 build、restart；车辆接口参数变化还需
重启对应接口服务。修改 main/Git 历史不会自动更新车端。

实测与原始软件验证范围见[弯道激进版记录](experiments/2026-10-14/AGGRESSIVE_BEND_BASELINE_CN.md)
及[验收记录](acceptance.md)。
