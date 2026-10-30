# B 方案 RNN 对齐 r6

开发分支 `ying/mvp21-rnn-maneuver-r6`，从 B r5 `f7ec808` 独立创建。
对齐来源为 `origin/feature/mvp21-sim-integration-20261015@2c41240`。
旧 B r5 与 A native r1 的分支和部署包保留。

## 实际数据流

普通巡线：道路/状态/当前障碍 → native Lattice 候选及道路硬检查 → 原冻结 RNN 评分 →
执行准入及运动补偿 → 原 B 的 MVP/PID 控制。
目标障碍任务激活：SHIFT/PASSING/RETURN 专用五次曲线路径 → 原 B 的完整车身、道路、
曲率和雷达碰撞检查 → 执行准入 → 同一个原 B 控制器。任务路径不参与 RNN 排名。
独立 ObstacleStopMonitor 及 B 的最终参考检查保留在规划选路之外。

进入/退出任务时清空 RNN 隐状态，递增规划 generation，清空已接收路径及旧参考引导，
拒绝切换前仍在计算的结果。只在普通巡线路径真正发布非过期控制后提交相应 RNN 状态，
基线回退、绕行任务、停车和失败计算均不提交。Run/Stop 边界重置状态，不自动恢复。

## 保留与对齐

- `control.py`、`maneuver.py`、`avoidance.py`、`obstacle_stop.py`、车辆接口 YAML 与 r5 相同。
- 原 MVP/PID 全部控制参数保持：前馈 0.30、drive_max 0.35；没有移植上游 0.50 力度设置。
- 用户明确选择原 RNN 的 0.50 m/s 目标速度，替代 B 原 0.20 m/s。它是目标值，不是实测车速。
- 原 checkpoint 字节及 SHA-256 保持，不训练、不重写其 provenance、不绕过速度校验。
- 移植自动蓝黄边界关联，映射在一次 Run 内锁定；不确定时只允许原有有界零动作启动等待。
- 移植二分弧长查询及规划任务内 knot-frame 缓存，保留 B 的 fusion_use_prior=false。
- 严格 IMU admission 保留；没有接入上游可选的 camera-only 控制降级。
- 固定左绕、30cm 车宽、r5 余量及15cm默认门禁保持。诊断档只关闭 proximity_stop_enabled。

当前 RNN 仍受原模型的观测和训练分布限制。本次集成没有改变道路估计的几何支持要求，
也没有把上游当时的巡线实车反馈当作 r6 绕行实车证据。

## 交付和检查

`python3 tools/package_lidar_avoidance.py --output <新目录>` 生成单文件、完整 YAML、
manifest、来源记录和压缩包；模型及 inference 编码代码嵌入单文件，不需要额外复制模型。
`--no-proximity-stop` 生成独立诊断档，仍保留源/车身/路径保护。

验证记录见 `docs/baselines/rnn_maneuver_20261030_r6.json` 和部署包旁的检查结果目录。
合成 ROS 检查不等于物理车辆停止/绕行验收；r6 0.50m/s 闭环实车尚未运行。

## 本轮观察到的限制

窄道 DDS 夹具的首次尝试出现雷达接收超过0.20s而锁停；记录原样保留。
缓存静态光线求交后发送周期约0.10s，但3°扫描仍漏掉障碍靠中线的边缘，任务未触发。
改用0.5°扫描后静态任务接管通过。这不代表任意稀疏回波都能识别障碍完整轮廓。
本轮没有使用旧离线 runtime 的 feedforward selector 冒充新的 recurrent ROS 路径，
也没有据旧 r5 的0.20m/s闭环结果宣称 r6 的0.50m/s闭环通过。
