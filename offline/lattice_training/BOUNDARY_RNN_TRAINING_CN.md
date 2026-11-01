# 独立边界道路的分阶段 RNN 重训

本轮固定使用 `boundary-first-reviewed-20261031/selection.json` 中全部 39 条
`qualified` 道路，保留原始独立左右边界、固定桶位和缺桶布局。
这是中心参考线上感知/planner 筛选合格的道路集合，不代表自主闭环已经通过。

采集使用每路 4 个分层随机起点、2 次独立重复，共 312 个独立 episode。
每个 episode 的传感器噪声重新采样；每路原有 0.08–0.12 m 的径向高斯标准差不变，
保留 80° 视场角、无近场裁剪、12% 重复检测、20 Hz 采样及 1 帧延迟。雷达为 dummy。
不重新摆桶、不额外删桶、不增加宽度先验。非法初始状态和无候选案例保留在报告中，
不按 planner/C* 成败重新抽起点。

共享采集管线逐条拟合 native 候选，以 0.5 m/s 参考速度、道路进度增量项生成 C* 标签，
再按每次决策 50% 随机合法选择 / 50% native 首选完整执行一条候选。
沿用 8 条候选上限、64 次优化评估、5 轮、12 个体和每候选 30 s 预算。
保留失败候选及原始失败状态。native 首选失败不会偷偷改选随机分支。

独立边界的真实多边形及真实桶圆用于 C* 车身约束和实际回放碰撞检查。
仿真载体的派生参考线仅用于 native 动力学宿主和原有带局部提示的有符号进度项，
不作为真实道路净空。policy 外部输入保持当前带噪状态、道路/障碍、单条无时序候选。
每个 episode 从零记忆开始；后续训练按固定执行分支、用当前权重重放隐状态。

在观测 C* 标签前冻结新的组划分：训练 25 路、验证 7 路、测试 7 路。
验证构造种子为 4108/4109，测试为 4102/4107，其余为训练。
同构造种子的镜像、所有起点和重复采样均不跨集合。
39 路中只有一条右 compound，保留在训练集；验证/测试不能支持该族的独立泛化结论。

阶段分别为：

1. 并行采集候选和 C* 标签，输出各 worker 的原始 `decisions.jsonl`、报告及状态。
2. 用户反馈后合并、校验固定执行序列和标签、按冻结组划分导入数据，并报告覆盖/失败。
3. 用户反馈后训练新的独立 RNN 检查点，仅使用 Validation 选择，最后一次评估 Test。

每个阶段启动后助手结束当轮，不持续轮询，不自动启动下一阶段。
worker 内的日志和源文件指纹检查只用于保存结果和拒绝混合版本标签。
本轮不更换主线模型，不覆盖历史模型，也不推送验证记录。

准备及启动入口（准备命令不会启动采集或训练）：

```powershell
$py = '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe'
& $py -X utf8 -m offline.lattice_training.collect_boundary_policy prepare --pool .verification/boundary-first-reviewed-20261031 --output offline/lattice_training/artifacts/2026-10-31-boundary-rnn
& $py -X utf8 -m offline.lattice_training.collect_boundary_policy worker --output offline/lattice_training/artifacts/2026-10-31-boundary-rnn --worker 0
```

另两个 worker 使用 `--worker 1` / `--worker 2`，各使用独立分片。
后台启动须使用隐藏窗口并分别重定向日志。汇总前以各分片 `status.json` 的
`COMPLETE` 和完整报告为准，不能把父级 `RUNNING` 标记当作完成。

## 中断后续采

首轮在 71 个 episode 结束后进程消失，保留了 125 个决策 / 904 个候选。
原始分片和设置保持不可变，续采另写 `resumes/resume-001`；原有 71 个结束记录跳过。
两个中断序列的 5 个决策从相同初始种子重放已执行控制，核对车辆状态、观测特征、
候选几何和随机选择顺序后继续。旧标签不重算、不复制到新决策文件。
续采完成后，正式导入应合并原始分片与续采分片，按 group_id 查重，并按 episode.id
连接跨分片序列；不能只读取新目录而丢掉两个序列的旧前缀。

续采运行指纹单独记录：pipeline.py 只增加可选跳过/重放入口，默认采集算法不变；
原始 pipeline 字节及前后指纹存入续采目录。道路、噪声、划分、C* 参数和行为概率不变。
隐藏进程由 WMI 独立创建并使用 breakaway 标志，启动入口检查根进程的 job 归属，
保存 stdout/stderr、退出记录及原生崩溃信息。每个结束 episode 另存原子检查点。
这些是采集进程自身的输出，不会启动监控、自动重启或下一阶段训练。

## 合并及训练入口

用户在 2026-11-01 授权本轮完成合并后立即启动训练。使用
`python -m offline.lattice_training.finalize_boundary_policy merge --batch BATCH --output NEW_CONSOLIDATED`
按已冻结划分合并六个原始/续采分片，校验跨分片车辆状态和已执行控制，按 episode/segment
排序，调用共享 importer 从执行轨迹重新计算全部标签，并验证 RNN 序列。
聚合报告引用原始完整地图/规划失败诊断，保留执行轨迹；原始分片完全保留。

`prepare-training --dataset DATASET --output NEW_TRAINING --warm-start CURRENT_MODEL`
冻结训练数据、源码、原模型和主线配置指纹，并复制微调初始化权重。
沿用当前 16 维隐状态、1841 参数的 RNN，学习率 0.0002，最多 200 轮，seed 42，
Validation 可避免失败数/回报遗憾值/RMSE 依次选优，前两项 60 轮不改善则早停。
保留已有仅由 Train 得到的归一化统计；固定采集时执行分支，用当前权重重放记忆并 BPTT。
模型输入、0.5 m/s C* 标签条件、损失定义和网络结构不变。
Test 只在训练和检查点选择结束后计算报告，不参与参数更新或选优。

训练结果单独写 `artifacts/2026-11-01-boundary-policy`，默认策略及旧模型不改写。
隐藏独立进程使用与续采相同的 WMI 启动方式，输出逐轮学习记录及退出诊断。
启动后结束当轮，不持续检查，也不自动进行车辆闭环或部署。

## 从零训练对照（2026-11-01）

用户在当前频率闭环与理想控制诊断之后授权用同一新数据从 scratch 训练。
独立输出 `artifacts/2026-11-01-boundary-policy-scratch`；不加载旧权重或旧归一化，
后者只由这批 Train 输入重新计算。仍为 hidden=16、1841 参数的同一 RNN、
相同结构化特征、C* 标签和固定执行分支 BPTT，保持 25/7/7 道路划分。
不加入理想控制实验轨迹，不改写原数据或之前的模型。

新 Train 实际为 2707 候选/374 决策，旧已部署模型实际训练使用 3175/417；
同一量级但新 Train 分别少约 14.7%/10.3%。新完整批次 4354 候选含 755
Validation、892 Test，不应将总量全视作训练量。

从头优化采用 Adam lr=0.001、seed=42、最多 1000 轮、Validation 失败数/回报
连续 150 轮无改善则停；Validation 选优，Test 在最终检查点后计算。这也改变了
优化预算，因此是完整 fresh-training 对照，不能仅把差异归因于初始化权重。
原局部 C* 标签缺少后续感知/规划可继续性的问题仍然存在；本轮不修改该目标。
独立启动器保存源码/数据/保护文件指纹、学习轨迹和退出诊断；启动后结束本轮，
不自动运行闭环或替换主线。
