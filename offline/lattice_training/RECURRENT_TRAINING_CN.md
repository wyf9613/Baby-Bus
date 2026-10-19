# 循环 policy 训练

本轮使用现有感知、native planner、native plant 与 C*；ROS/common 默认配置不变。
外部输入仍为当前状态、道路/障碍和单条候选，无速度曲线、planner 评分项或人工历史向量。

## 记忆和微调

保留原有 16 维线段编码、场景注意力、融合及评分层，在评分层加入
`h_next = tanh(x W_head + h_previous W_recurrent + b_head)`。
`x` 是候选/场景汇聚与当前状态拼接后的 24 维向量，隐状态为 16 维。
新增 16×16 参数，当前模型共 1841 个可训练参数。

每次决策的所有候选使用同一个 `h_previous`，分别产生分数和 `h_next`。
只有被执行的候选产生的 `h_next` 会传给下一次决策；候选遍历顺序不改变记忆。
新道路从零状态开始。离线模块验证中，共享执行模块实际采用某个 job 时才提交一次
对应状态；被拒绝、被覆盖的规划结果以及 baseline 回退不提交候选状态。

数据记录 `episode.id`、`episode.segment`、`selected_candidate_id`，导入后保存为
`episode_ids`、`sequence_steps`、`behavior_selected` 数组。
训练固定实际执行序列，每轮用当前权重从零状态完整重放，并通过已执行分支进行 BPTT。
各未执行候选仍有独立 C* 监督损失；其评分依赖的前序状态也参与梯度传播。
不把采集时某一套权重产生的隐状态数值固定为训练输入。

微调复制旧模型的编码器、评分层和仅由旧 Train 数据得到的归一化统计，
循环矩阵初始化为 0.1 倍单位阵。新目标和标签条件来自本轮数据；不沿用旧 0.2 m/s 标签。
默认检查点按 Validation 评分 RMSE 选取，Test 不参与选取。
多起点重训可显式选择 `--checkpoint validation_feasible_then_return`：依次比较
同组存在成功候选时的失败误选数、平均原始回报遗憾值、评分 RMSE。
名义 C* 成败只作为 Validation 检查点依据，不加入 policy 输入，回归损失保持不变。
`--early-stopping-patience` 限制前两项选路指标连续不改善的轮数；RMSE 单独改善不会延长训练。
此模式也评估第 0 轮初始化，避免强制接受比微调前更差的权重。

## 道路、行为和标签

- 默认 96 个道路案例：6 个场景族，10 个 Train、3 个 Validation、3 个 Test 种子。
  包含直道、左右弯、左右特殊 U-turn 及组合道路。相同种子的左右场景属于同一数据划分。
- 宽度范围为 0.75–1.6 m。特殊 U-turn 保持外围圆弧接直线、内侧单一直线的几何关系，
  宽度改变时参考半径随之改变。
- 每段原本存在双侧锥桶的道路，以 0.65 概率删除一侧某个 0.4–1.4 m 区间的锥桶。
  这是实际道路锥桶布置的变化，物理道路边界仍然存在。已有单侧的 U-turn 圆弧不再删至双侧皆空。
- 每次决策以 50% 概率随机选一条可完整合法执行的候选，另外 50% 采用 native 第一选择。
  planner 分支若 C* 失败，本路段停止并保留失败，不偷偷切换随机选择。
  50/50 是每次决策的概率，有限批次不强行凑出精确相等计数。
- C* 参考巡航速度统一为 0.5 m/s，仍保持全路径从当前车速到巡航速度的显式时间规律。
  默认使用一次名义执行生成标签；硬约束和速度上限没有为提高成功率而放宽。
- 在原跟踪损失上增加 `-1.0 × Δs_road / 1 m`，所以 policy 回报增加相同的道路进度收益。
  `s_road` 是世界道路参考线的有符号弧长站位，使用 native 带上一站位提示的局部投影，
  禁用全局分支回退，并限制在原任务道路范围内；不奖励末端感知延伸。
  优化和实际回放标签都使用同一项，失败仍返回声明的失败损失。

## 使用

```powershell
$py = '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe'
& $py -m offline.lattice_training.recurrent_training --output offline/lattice_training/artifacts/new-recurrent-batch --epochs 300 --planner-sample-count 8
```

默认输出包含实际 C* 轨迹、固定执行序列、三组划分的数据、学习曲线、模型及
`recurrent-summary.json`。输出目录必须不存在，避免覆盖旧批次。
采集仍是候选生成→逐条 C* 标签→完整执行一个被选候选→下一决策；
native job 默认 1 Hz 参数不变，但采集过程不模拟主线每秒重规划。
本轮实际使用 8 条候选上限；主线 native 默认是 48 条。`--planner-sample-count` 可显式修改，
两种候选分布的结果必须分开报告。通过 `PipelineSettings` 调用完整道路评估时，
候选数、planner 频率及末段长度规则随采集设置传递；不会悄悄回到默认 48 条。

已有数据单独微调可用：

```powershell
& $py -m offline.lattice_training train --dataset offline/lattice_training/artifacts/2026-10-19-recurrent-policy/dataset --output offline/lattice_training/artifacts/recurrent-refit --recurrent --hidden 16 --seed 42 --epochs 300 --warm-start offline/lattice_training/artifacts/2026-10-17-expanded-collection/model.json --learning-rate 0.001
```

序列评分使用 `predict_dataset()` 或 `score` CLI；`step(input, previous_hidden)` 是单候选纯函数。
直接 `score(input)`/`predict(inputs)` 使用零状态，可用于无记忆对照，不能当成有记忆评估。
旧模型仍按原架构读取，观察编码语义不因加入循环层而改变。

## 2026-10-19 实际结果

96 个道路案例产生 377 次决策、2919 条候选和 2260 次成功的名义 C* 执行。
Train/Validation/Test 候选为 1861/440/618。先运行 120 epoch，再因 Validation
仍在下降而以相同初始化重跑至 300；两者按 Validation RMSE 选择，最终取第 300 轮。
Test 从未用于检查点选择。

Test 有可行候选且存在多选项的 71 次决策中，C* 最优候选命中从 native 的 18 次
提高到 35 次；可避免的 C* 失败误选仍为 7 次。清零记忆并未改变这些 Test 决策的选路，
因此不能将改善归因于 RNN 记忆。与采集一致的 6 类闭环中，native 与新模型均通过
直道及左右弯，共 3/6；左右 U-turn 和该随机道路仍失败。

本地 [完整报告](artifacts/2026-10-19-recurrent-policy/REPORT_CN.md) 包含失败候选复核、
学习曲线、道路实际轨迹和车身姿态。新模型为
`artifacts/2026-10-19-recurrent-policy/model.json`，旧模型和默认 native 运行设置保留。
原始采集、验证记录及图片只保存在本地，不纳入 Git。

显式验证新模型与当前 MVP 控制器的接口及模块时钟：

```powershell
& $py -m offline.mvp21_sim.validation run --road quick-straight --config offline/mvp21_sim/workspace/examples/recurrent-model.json --max-steps 100
```

这项短直路检查使用 20 Hz 感知/控制、1 Hz planner/policy，包含实际 baseline 回退，
不能代替整路 C* 闭环或宣称 U-turn 已通过。

## 多起点扩采

`stratified_random` 在最终生成的物理道路上划分等长起点区间，每区间随机抽一个起点，
适用于套件道路、U-turn 和随机组合道路。道路几何、宽度、障碍及缺桶布局在同一道路的
所有起点间保持一致。车辆位置、航向误差和初始速度由起点种子分别生成。

起点范围预留完整车身和至少 1.5 m 到任务终点的距离，按物理障碍排除不可用区间。
不根据感知、planner 或 C* 成败反复抽起点；非法初始状态和无候选案例照常记录。
每个起点是独立 episode，RNN 从零状态开始，不能把不同起点拼成一条虚假的历史。
从后段起点到达终点只表示子序列完成，不能算成整条道路从入口通过。

```powershell
& $py -m offline.lattice_training.recurrent_training --output offline/lattice_training/artifacts/new-multistart-batch --collect-only --train-seeds 1800,1801,1802,1803 --validation-seeds 1900 --test-seeds 2000 --starts-per-road 4 --initial-start-mode stratified_random --max-evaluations 64 --iterations 5 --population 12
```

此命令生成 36 条道路、144 个起点序列；Train/Validation/Test 按种子整路分组。
0.5 m/s、道路进度项、50/50 随机/native 行为和原生特征保持一致。
C* 搜索量与前一批不同，应保留完整预算及原始失败状态，单独存放和审计；
求解失败仍只表示该预算内未找到解。`--collect-only` 不训练或更换模型。

## 多起点批次重训

2026-10-19 多起点批次包含 644 次决策、4982 条候选，Train/Validation/Test 为
3175/938/869 条。旧低预算标签未混入；道路和所有起点保持整路分组。
后续训练使用已移除 Test 行的 development-dataset，命令如下（输出应另选未使用路径）：

```powershell
& $py -m offline.lattice_training train --dataset offline/lattice_training/artifacts/2026-10-19-multistart-policy/development-dataset --output offline/lattice_training/artifacts/new-multistart-refit/model.json --recurrent --hidden 16 --seed 42 --epochs 200 --warm-start offline/lattice_training/artifacts/2026-10-19-recurrent-policy/model.json --learning-rate 0.0002 --checkpoint validation_feasible_then_return --early-stopping-patience 60
```

比较学习率 0.001、0.0002 的微调和 0.003 的重新初始化后，只按 Validation 选择
0.0002 微调。实际训练 60 轮，保留第 2 轮；评分误差继续下降却未改善失败误选。
Test 最优命中 66/108 → 67/108，可避免失败仍为 3/108。配对 C* 起点子序列新旧均完成
5/6，右掉头仍失败，不能宣称通过能力已改善。默认运行配置不替换。
见本地 [重训报告、学习曲线与实际姿态](artifacts/2026-10-19-multistart-policy/REPORT_CN.md)。
