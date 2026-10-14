# MVP2.1 全新影子 lattice：场地观察说明

2026-10-15。基线为 main 的二维道路 / PID 版本。影子规划全部重新实现，
没有查阅、移植或加载旧 lattice 的代码、参数、起步状态或参考寿命。
实车动作仍由稳定基线产生；影子结果只用于 Path、JSON 和日志。

## 启用和频率

当前 `config/ai4r_policy.yaml` 中 `shadow.enabled: true`、`shadow.rate_hz: 1.0`。
可以改成 2.0、5.0、10.0；这些参数启动时读取，修改后重启 policy。
提交快照依托原 20 Hz 控制周期，频率不会超过控制频率。一个独立的低优先级
Python 进程负责计算，后台线程负责通信；待处理任务和结果各只保留最新一份。
工作线程从同一个 policy 源文件提取纯算法定义，不启动 ROS 或硬件节点。

源码 / 配置上车构建、重启命令仍为：

```bash
dream build ros student && dream runtime restart ai4r_policy
```

这里没有执行部署、Enable 或 Run。影子是否成功不参与基线控制或停车判断。
`shadow.enabled: false` 可以关闭整个影子侧通道。

## 算法及评估范围

二维弧长参考和法向构成局部 Frenet 坐标。采样中间横向偏移，在笛卡尔空间用
分段 Hermite 曲线连接，起点为当前车辆原点、初始切向沿车头。候选包括参考
跟踪、保持偏移、快速回正、弯外准备、左右准备和成对 Halton 低差异采样。
共 48 个候选；1 m 是基础输出长度，执行 / 预瞄所需范围更长时自动扩展。

不使用道路 valid、短覆盖、单侧缺失、斜率、曲率或大转角作为道路准入门槛。
一个有限中心点也能建立推断方向；没有中心线时可从唯一边界偏移建立参考。
没有路宽时搜索宽度是配置假设，绝不宣称已观测到另一侧。完全没有可用几何才
返回 NO_ROAD。尾部和近场按切向延伸，明确保留实际观测范围与推断来源。

近期检查长度 = `|实际轮速| × (1 / rate_hz + 交接时间估计)`。
交接时间估计初始为 `handoff_allowance_s=0.10`，后续至少覆盖本跑次已观察到的
提交到接收/发布延迟。当前周期超出估计时，日志标记 `handoff_exceeded_estimate`；
这是影子检查范围的估计，不是有保证的实际执行期限。

硬碰撞只检查这段运动中的完整无额外净空矩形和 `observed` / `observed_with_history`
边界的现有线段，不延伸硬边界。width_inferred、predicted_history、absent 都不是硬墙。
矩形前端自然可能覆盖 CG 检查段之外。位移和朝向同时自适应细分；这是数值姿态
采样，尚不是连续扫掠认证或真实物理接触保证。观测点之间的输入线段来自感知，
未另外宣称感知拟合是无误差的物理边界。

评分范围至少为 `score_min_length_m=0.2`，执行范围更长时随之扩展。
近处权重更大，比较横向目标、朝向、初期偏离与轻量平滑项。
从有支持中心线的方向变化连续计算外侧准备目标，左右急弯对称；准备与直接回正
候选同时存在。候选远端不做硬碰撞淘汰或全段累积评分。

2026-10-15 更新：执行预测代价、逐候选 PID 副本 / 自行车推演及其模型参数全部删除。
控制器限幅、增益和削峰不参与候选打分；不通过软代价压制激进回正或入弯准备。
`timing_s` 分别记录输入整理、Frenet 建立、采样、候选生成、碰撞初筛、评分、选择、
结果组装、未归类开销和总耗时，单位秒。worker 的 `wire_timing_s` 另记 JSON 解码 / 编码。

## 必须区分的配置事实

`body_width_m=0.30` 来自操作员此前报告；前后 CG 伸出各 0.25 m 是暂用假设。
已删除全部动力学和车轮转角映射假设。稳定基线的预瞄距离仅用于确保生成轨迹
能够提供足够的控制参考范围；新影子规划没有采用现有参考寿命，也没有改变稳定链路。

## Foxglove / 录制

以下为相对话题名，现场命名空间以 `ros2 topic list` 为准。通常在 `/car/` 下：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| policy_trajectory | nav_msgs/Path | 原稳定控制器实际采用参考 |
| shadow_trajectory | nav_msgs/Path | 影子选中的完整候选 |
| shadow_checked_prefix | nav_msgs/Path | 实际做硬碰撞检查的 CG 前段 |
| shadow_reference | nav_msgs/Path | 本次局部参考，包括标记在 JSON 中的推断部分 |
| shadow_planning | std_msgs/String | 本轮完整 JSON：候选、分项代价、碰撞、来源与时刻 |

当前 `tools/foxglove_road_visualization/layout.json` 在同一个 Cone locations 图中显示
稳定参考绿线和影子选中轨迹蓝线（`#3b82f6`）；重新导入该布局即可添加蓝线显示。
此次布局修改按用户要求未进行本机测试。检查段和影子参考仍可另行配置显示。
影子 Path 保留输入坐标时刻，不用完成时间伪装成当前坐标；低频模式中线可能明显
滞后，应同时看 `input_stamp_s` 与 `Shadow planning` 日志的 `result_age_s`。
停止 / 重新起跑清空所有影子线，代次隔离防止迟到结果重新出线。

JSON 中 VALID 只表示“在配置尺寸、采样精度和已知边界下，指定近段未检测到碰撞”。
ALL_COLLIDE 仍保留排名最好的碰撞候选与接触详情用于分析，但不显示为有效选中线；
NO_ROAD、WORKER_ERROR 或 TRANSPORT_ERROR 清空选中线。它们都不改变实车动作。

现场每轮保存源码哈希、YAML、实际频率和完整区间日志。除了所有影子话题，还应录制
原始锥桶、轮速、gyro、实际动作和稳定参考；不要只保存一张参考线截图。

```bash
dream runtime logs ai4r_policy --tail 1000
```

关注 `candidate_count`、`feasible_count`、`check_length_m`、`selected_family`、
`preparation_offset_m`、`elapsed_s`、`late_for_requested_period`、`round_trip_s`。
配置提交频率不等于完成频率；计算跟不上时丢弃待处理旧任务，不阻塞基线。
如提高频率后持续来不及完成，可减少 sample_count 或回到低频观察，不能宣称已达到 10 Hz。

## 验证

2026-10-15，本机无 ROS。按用户要求不运行 ROS gate。
纯 Python 估计 / 规划 / 控制 / 中心线 / 影子检查合计 210 项通过，含真实工作进程
协议、异步运输与停止代次隔离；影子提交故障与基线动作完全一致的对照通过。
ROS 测试源码已补安装包 worker / Path / Stop 场景，但本次未运行。

本次重新检查启动、感知、参考选择、控制、动作发布、监督停车和退出流程。
与稳定基线 `0c8593a` 比较，9 个核心类、7 个控制 / 状态方法、启动文件及除
shadow 外的 YAML 值完全一致。按当前 YAML 运行 40 个连续控制周期，影子成功、
全部碰撞、无路、算法异常结果均不改变基线动作或内部控制状态；运输崩溃只重启
影子 worker。手动停止和监督停车现在先完成基线零动作发布，再清空影子显示；
退出也先发零动作再关闭 worker。旧代次结果不会恢复停止后的控制。

影子在道路快照副本上运行，结果只写影子字段和话题。它仍消耗 CPU、通信线程和
同一 ROS executor 的显示 / 日志发布时间，因此这次检查确认的是控制决策隔离，
没有证明实车调度开销为零，也没有读取当前车上部署的源码、参数或话题连接。
细项见 [控制隔离审计](experiments/2026-10-15/evidence/shadow/control-isolation-audit.json)。

15 号车 visualization4 的 254 条真实锥桶记录用于静态几何重放，检查抽取的第
0/65/130/195/253 条记录：前四条有道路时各生成 48 候选，末条无道路为 NO_ROAD。
这不是完整传感器时钟重放，也不是影子规划的闭环或实车验收。
计算耗时记录在本次本地证据；目标车辆频率和控制调度影响待现场观察。

证据：[检查清单](experiments/2026-10-15/evidence/shadow/manifest.json)、
[纯 Python 日志](experiments/2026-10-15/evidence/shadow/portable-checks.log)、
[真实锥桶几何复查](experiments/2026-10-15/evidence/shadow/real-cone-geometry.json)。

## 重测脚本与单侧场景

在有 PyYAML 的本机 Python 环境中运行（不需要 ROS）：

```powershell
python -B offline/shadow_lattice/evaluate.py --start 60 --frames 24 --repeats 2
python -B offline/shadow_lattice/evaluate.py --start 150 --frames 24 --repeats 2 --output docs/experiments/2026-10-15/evidence/shadow/segment150
```

默认按当前 YAML 的 48 候选测试 1 / 10 Hz，实际轮速参与检查长度。分别保留原始颜色、
仅黄左侧、仅蓝右侧，以及把单侧边界和参考支持截短至 0.15 m 的场景。
在遮挡前，先用此前一帧真实双侧观测学习路宽；随后在原始检测点上删去另一颜色，
直接调用当前稳定感知，禁止把被遮挡侧历史送入影子碰撞检查。这是连续真实几何
序列的人工遮挡试验，不是完整传感器时钟重放或闭环行驶。

第 60–83 帧中，右侧原本就稀疏 / 缺失：仅右侧重建有 3 帧可用，21 帧不输出道路。
可用 3 帧均规划成功；NO_ROAD 不计入有道路时的性能平均值（参见 planned_timing_ms）。
第 150–173 帧左右单侧分别 24/24 帧成功，短至 0.15 m 支持时也各 24/24 成功。
每轮两次重复，重复计数与独立帧数分别记录，不能把 48 次调用说成 48 个不同帧。

[耗时分解和首段单侧结果](experiments/2026-10-15/evidence/shadow/timing-single-side.json)、
[第二段结果](experiments/2026-10-15/evidence/shadow/segment150/timing-single-side.json)。
新结果含均值、中位数、p95、最大值、阶段分解和稳态子进程往返耗时；首次往返单列
进程启动。geometry_plus_planning 仅包含静态道路重建与规划，不代表完整 ROS 节点耗时。
报告中的 before 对照仅是本次全新影子原型删除执行预测之前的版本，不是旧 mvp2.0 lattice。
