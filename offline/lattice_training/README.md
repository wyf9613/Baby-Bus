# Lattice policy 的完整训练管线

2026-10-15：[实车锥桶道路的 native 理想输入诊断](NATIVE_REPLAY_CN.md) 已记录。
直接 Frenet 输出、20 Hz查询及现有C*执行下，原生基线仍不能完整通过重建道路；
该实验用于定位planner问题，未修改训练模型或实际部署。
后续已将净空检查替换为[连续边界与完整车身几何](CONTINUOUS_BODY_CN.md)，保留5 cm
硬余量；原失败帧恢复，整路闭环仍未通过。[入弯与计时复核](REMAINING_DIAGNOSIS_CN.md)
进一步区分低速硬阈值、主机计时中止，以及隔离计时后仍存在的入弯候选/运动限制失败。

实际 policy 接线见[模型接入 native planner 指南](INTEGRATION_CN.md)：模型加载、
实时 observation、`candidate_scorer` 回调、节点插入位置和当前 ROS 打包条件。

可随仓库获取的[当前模型、训练曲线和结果记录](artifacts/2026-10-11-augmented-sigmoid/README.md)
已保留，包含扩充前的对照模型。大型冻结数据及原始仿真 trace 仍位于本地 `.verification/`。

## 2026-10-11 扩充 Train 后重新训练

已使用合并数据从头训练：Train 3295 候选/145 状态/11 路，Validation 942/37/2、
Test 699/35/2 保持。与上一轮相同的 140→32 tanh→1 sigmoid、Adam/.005/seed 0，
150 epochs；按 Validation score RMSE 保存第 150 轮。目标仍为 sigmoid(mean(raw C* returns))。
本轮通过有效候选门槛正常启动，没有使用覆盖例外；历史 metadata 不改写。

相同 Validation 上旧/新模型的选路平均原始回报为 .229130/.299922、两两排序准确率
64.99%/68.49%，评分 RMSE=.103350/.097947。相同 Test 上回报 .445127/.424509、
排序 79.72%/76.84%，评分 RMSE=.074287/.059393。Validation 改善，Test 的评分误差
改善但选路表现略降；两个模型均保留，不宣称全面提升。Test 不参与训练或模型选择。
最佳 Validation 位于本轮最后一轮，不能据此宣称已收敛。未运行新 policy 整路闭环。

模型、曲线与完整对照在 `.verification/augmented-sigmoid-training-20261011/`。
加载、评分输出、保存模型指标、Train-only归一化和原数据/模型/holdout 保护已核对。

```powershell
python -B -u -m offline.lattice_training train --dataset .verification/train-augmentation-20261011/dataset --output .verification/augmented-sigmoid-training-20261011 --epochs 150 --hidden 32 --seed 0 --learning-rate 0.005
```

## 当前训练条件：候选标签有效，整路覆盖单独报告

2026-10-11 修正：此前将“未到道路终点/某条请求道路无候选”作为整批训练拒绝条件，
与当前逐候选监督目标不一致，已取消。完整路径指候选自身（通常约 2m），不是整个随机道路。
成功 C* trial 的候选完成检查、约束及回报重算仍保留；失败 trial 继续使用已定义的惩罚标签。
后续随机序列中断不会让前面已标注候选失效；无候选道路只贡献覆盖缺口，不生成训练行。

训练入口先执行 source/label/objective/hash/schema/分区检查，再按已有有效样本计算
`training_readiness`：Train 至少两行且 Validation 非空。新导出数据的 `training_eligible`
依据样本判定，`road_coverage_complete` 单独记录整路覆盖。既有冻结 metadata 不改写；
旧 `training_eligible=false` 不再作为拒绝条件，来源中同时记录旧标记和新的判定结果。
`--allow-incomplete-coverage` 仅为旧命令兼容保留，已无需该例外。

当前扩充数据可直接用于候选监督训练：Train 3295/145、Validation 942/37、Test 699/35
（候选/决策点）。seed 8 无标签、0 road goal 仍记入报告，不阻止其余有效样本训练。
本次只修正门槛并检查真实数据的可用性，没有重新训练或改写标签/模型。

## 2026-10-11 仅向 Train 追加新道路

`augment` 复用现有 `collect_dataset` / C* 打标器，采集新随机道路并合并到新冻结数据目录。
不会为 Validation/Test 生成道路；旧 Train 行保留，新行全部追加到 Train。
合并前要求不同道路及决策 ID 不重叠、C* objective/solver/plant/采样合同相同；
道路 seeds 和是否只采集 Train 的范围信息允许按 batch 不同，完整来源分别保留。
合并后逐项比较所有 Validation/Test 数组，原数据及旧模型文件保持。
道路覆盖不足、非法初态或无标签道路继续记录，训练门槛保留。此命令只增加数据。
编码按决策流式进行，合并复用已冻结的 NPZ，不重新计算旧标签。

本批计划 8 条新道路 seed 4–11，2 初态 × standard/wide，共 32 连续随机序列；
配置与原批相同。输出 `.verification/train-augmentation-20261011/`，完成后合并数据在
`dataset/`，新增批在 `addition/`，`run.json` / `collection.log` 记录实际状态和进度。

```powershell
python -B -u -m offline.lattice_training augment --dataset .verification/eight-road-collection-20261011/dataset --output .verification/train-augmentation-20261011 --train-seeds 4,5,6,7,8,9,10,11 --starts-per-road 2 --planner-profiles standard,wide --until-road-end --max-segments 100 --trials 2 --population 4 --iterations 1 --max-evaluations 8 --solver-wall-s 30 --max-near-gap-m 1.2 --max-extension-m 1.6 --allow-unobserved-origin
```

启动记录不代表采集已经完成。结果及实际新增数量以 augmentation.json / REPORT.md 为准。
Validation 保持 942 行/37 决策点/2 道路，Test 保持 699 行/35 决策点/2 道路。

本批已于 2026-10-11 结束并自动合并，总耗时 35.21min；新增 67 状态/1392 候选，
2784 trials 中 2776 成功。8 新路中 7 路有标签，seed 8 的四个序列均在
`insufficient_lattice_coverage` 处无候选。合并 Train 为 3295 候选/145 状态/11 有标签道路。
Validation/Test 全部数组及原数据、旧模型、职责外文件和 refs 核对保持。
本次仍是 25 个规划失败、7 个完整计划 STOP、0 road goal；冻结 metadata 的旧标记
`training_eligible=false` 保留。上方的门槛修正已使现有有效候选可训练，尚未重新训练。

## 2026-10-11 Sigmoid 目标重新训练

默认训练目标改为 `z = sigmoid(R / T)`，`T=1`，其中 `R` 是候选所有原始 C* trial
回报的算术平均。顺序为先平均、再 sigmoid；C* objective、失败惩罚和冻结标签保留。
同一个 `train_model` 用评分空间的决策加权 MSE 和候选差值 MSE，取消 raw y 缩放；
MLP 输出层采用 sigmoid，0–1 分数用于 argmax 选路，不代表原始回报或成功概率。
新 v3 checkpoint 记录变换参数/顺序/输出空间；旧 v2 checkpoint 仍按原始回报读取。
评分输出及 RMSE 显式标记所在空间，选路回报/regret/排序评价使用原始 C* 标签。

复用同一冻结数据及 Adam/.005/seed 0/140→32/150 epochs，按 validation score RMSE
保存 epoch 49。测试集原始平均选路回报由旧模型 .295319 提高至 .445127（规则 .279209、
Oracle .484249）；两两排序准确率由 52.46% 提高至 79.72%，评分 RMSE=.074287。
新旧 RMSE 单位不同，不能直接比较。原采集 `training_eligible=false`、0 road goal 和
实验覆盖标记保留；本次评价是已有 35 个 test 决策点，没有运行新 policy 的整路闭环。
数据及旧模型未改写，结果位于 `.verification/eight-road-sigmoid-training-20261011/`。

```powershell
python -B -u -m offline.lattice_training train --dataset .verification/eight-road-collection-20261011/dataset --output .verification/eight-road-sigmoid-training-20261011 --epochs 150 --hidden 32 --seed 0 --learning-rate 0.005 --allow-incomplete-coverage
```

详见该目录 REPORT.md、training.png 和 comparison.png。已有输出仍拒绝覆盖。
极端失败回报变换后数值饱和为 0，失去这些极端负数之间的差异；失败标签没有删除。

## 2026-10-11 第一轮实验训练

已使用冻结数据完成 150 epochs：1903 train、942 validation、699 test 候选，
对应 78/37/35 个决策状态。MLP 为 140→32 tanh→1，Adam/.005/seed 0；
归一化仅使用 train，按 validation RMSE 保留 epoch 14，test 不参与模型选择。
模型及曲线位于 `.verification/eight-road-training-20261011/`。

原数据的 `training_eligible=false` 和整路完成 0 保留。显式实验选项仅允许
缺少 road goal 的覆盖例外，source/标签/目标/hash/特征/分区检查，以及所有道路有标签、
合法初态、三个分区非空的要求保留。模型标记 `partial_road_coverage_experiment`。
这一轮不重新打标、不修改 reward，也没有完成道路闭环验收。

```powershell
python -B -u -m offline.lattice_training train --dataset .verification/eight-road-collection-20261011/dataset --output .verification/eight-road-training-20261011 --epochs 150 --hidden 32 --seed 0 --learning-rate 0.005 --allow-incomplete-coverage
```

`train` 调用同一个 train_model，不构成另一条训练实现；已有模型输出会拒绝覆写。
默认省略 `--allow-incomplete-coverage` 仍执行原覆盖门槛。测试标签上的学习/规则平均
选路回报为 .2953/.2792，但学习两两排序准确率仅 52.46%，目前只是弱基线。
训练集中 5 条、测试集中 1 条候选带约 -50 万的失败惩罚，回归输出误差较大；
没有删除这些失败标签或改变惩罚。详见该目录 REPORT.md。

## 2026-10-11 新一轮采集

按用户要求清理旧训练/采集输出、旧模型及接口检查数据，共 9 个目录、100 个文件、
448.25MiB；独立诊断报告与图保留，历史输出数据路径已不再可用。
新输出为 `.verification/eight-road-collection-20261011/`，8 条固定随机道路：
train 0/1/2/3、validation 100/101、test 200/201；每路两个初态、standard/wide
两套 lattice 配置，共 32 条连续随机序列，直到 road goal、完整 STOP 或显式失败。
采用原始 Frenet 导出、`--allow-unobserved-origin`、near/extension=1.2/1.6m，
其余道路、车辆和传感器设置保持。每条候选完整 C* 求解与实际执行、两个 trial，
后继仍均匀随机选择，仅完整执行终点空间非法才改抽。采集过程与耗时见 collection.log；
cleanup.json 记录清理范围，run.json 记录命令。最终道路覆盖按 summary.json 判定，
未达到 road goal 的序列保持未完成，不通过整路覆盖门槛时不会进入训练。
本轮最终处理 32 序列、150 决策点、3544 候选、7088 trials（7082 成功），
采集 136.58min、编码 63.03s。所有 8 道路有数据但 0 条达到 goal；随后仅以上述
显式实验覆盖例外训练这批数据，原采集 metadata 没有改写。

## 原始 Frenet 候选作为唯一训练输入

离线工厂默认直接导出 native planner 的 `frenet_path`、`longitudinal_profile` 和
`speed_profile`。原始 Frenet 车身、障碍、运动和范围硬筛选保留；导出阶段不再要求
单段 Cartesian y(x) 拟合。C*、特征和可视化直接使用原曲线，没有重新拟合或截断。
完整已支持路径执行结束后，仍继承完整环境快照，在结束状态重新 rollout。

参考标记 `candidate_representation=frenet_native`，`path=None`、`output_errors=None`
明确表示没有生成 Cartesian 近似，不伪造零误差。该参考仅用于本离线训练管线；
native/live 的输出协议和控制代码保持。`--require-cartesian-compatibility` 可在同一
工厂/管线恢复旧输出筛选作对照，不构成另一套训练流程。

训练导出语义版本为 `native_frenet_export_v1`，参与 source signature；旧数据/模型
的 source signature 不会自动混入新管线，文件保留。C* 目标、标签版本与特征维度不变。

实际失败案例：20.344m 处旧模式为 0 候选，新模式为 9 候选。原图中那条候选经
完整 2m / 10.25s 的 C* 执行后终点误差 .00969m，下一次 rollout 可生成 39 候选。
公开规则最高分评价从起点连续执行 17 段、3418 步，达到 34.362/35.081m；
末端不足双侧几何，仍未达到 34.584m goal。不能记为完成整路。
证据：`.verification/frenet-export-20261010/`。以下 Cartesian 审计数字为改动前的历史结果。

## 暂时允许原点支持缺失的离线实验

`collect`、`run` 和 `visualize` 可追加 `--allow-unobserved-origin`。默认关闭。
开启后，仅当原生 planner 已经记录过原点支持、当前状态为
`near_field_unobserved`，且估计器保留已对齐、几何有效的双侧 `forward_geometry`
时，离线接入层允许 native planner 再使用有界后向延拓。常用实验参数：

```text
--allow-unobserved-origin --max-near-gap-m 1.2 --max-extension-m 1.6
```

原始 observation、估计器历史和 `observed_origin_seen` 保持；单侧、未对齐、过期、
距离/延拓超限等情况仍拒绝，native 车身/障碍/动力学检查及 C* 实际安全检查保持。
结果保存 `origin_support_assumption` 诊断、`origin_support_assumed` 和
`origin_support_override_applied` 标记；采集/评价报告记录实验选项。
这是训练工厂的临时输入假设，不改 native planner、原 env 或 live 配置。
证据和连续对照位于 `.verification/origin-support-20261010/`。
同一个失败终态开启前/后分别返回 0/27 条候选。相同 8 条随机序列中，完整 C* 段数
从 24 增至 53，总前进从 46.496m 增至 104.660m，但整路完成仍为 0；后续无可行
候选、Cartesian 复核和终点支持问题仍存在。最高规则分的公开可视化入口在 seed 0
执行 10 段、2009 步后报告 no_valid_cartesian_output，完整已执行 trace 保存在报告中。

## 2026-10-10 环境设置对照

已 fetch native planner 分支，远端最新仍为 `5068e9b`。车身保持 Notebook 的
0.25m × 0.495m，车辆动力学、80° 相机与 24 个检测容量保持。随机道路新增
`cone_spacing_m` 设置（默认 .5m）；采集、评价和可视化支持
`--max-near-gap-m` / `--max-extension-m`（默认 .9 / 1.3m），只配置离线 planner。

同一 8 道路、16 初态、两套 profile 的 32 次查询：原配置有效 12 次；锥桶 .25m
有效 22 次；道宽 .9–1.0m 有效 12 次；near/extension=1.2/1.6m 有效 29 次；
三项组合有效 28 次。所有配置的初态和车辆模型一致，障碍通行空间检查通过。
每套配置再随机执行 8 条 C* 连续序列；near=1.2m 的 8 条都能执行首段，7 条
进入第二次有效决策，共 24 个完整段，但 0 条完成道路。六套配置总计 48 条序列、
84 个完整段，仍有后续 near_field 丢失和终点支持不足，不能冻结为完整道路训练集。

本轮只生成配置/诊断，默认值保留；native planner、原 env 和 live YAML 未修改。
报告和对照图在 `.verification/environment-settings-20261010/REPORT.md`。
复现 near 实验可在唯一 `collect`/`run` 入口追加：

```text
--max-near-gap-m 1.2 --max-extension-m 1.6
```

密集锥桶/较窄道路通过 `--road-settings` 指定 JSON。可视化的同名 near/extension
参数也贯通完整 evaluation 和点位 candidate probe。原点观察 guard 保持，较大的
near 值并不允许已失去近场支持的序列重新使用起步假设。下方审计与旧训练数值为历史。

## 2026-10-10 覆盖问题审计后的状态

旧的 8-road 采集未完成整路覆盖，且发现初态空间合法性与 C* 有限道路约束遗漏。
旧 internal_cstar_native_env v5 数据/模型现被训练和加载接口拒绝；保留原文件作审计证据，
不可继续作为当前模型的验收数据。当前标签生产版本为 v6，目标损失仍是同一个完整路径目标。

修复包括：完整车身的有限道路起终点约束（覆盖初态及每个仿真步）；从物理自由空间
作一次初态抽样（车尾留边距、保守避开障碍横截面）；候选查询不提前提交规则选路；
warmup 已结束/非法初态显式失败；道路评价先查初态安全再判终点；覆盖图逐序列绘制实际 trace；
当时 `collect`/`run` 将缺少请求道路的标签或 `--until-road-end` 未达终点与训练拒绝绑定，
部分 metadata 因此标记 `training_eligible=false`。这项过严门槛已被上方的样本有效性判定替代；
覆盖诊断和历史标记保留。使用新输出目录，避免覆盖历史数据。

原 9 个 train 决策点是 seed 0 上四个序列的 3+3+2+1 之和，154 条候选对应 9 个决策状态；
实际前进进度区间合并后只有 9.369m，占这条 35.081m 道路的 26.7%，其余三个 train 道路为 0。
3 次 STOP 都来自当时只有停车候选的集合，并非从巡航与停车的混合集随机选中。

剩余覆盖限制已通过受控回放定位：原 Notebook 相机 80°、0.5m 锥桶间距与 native
max_near_gap_m=.9m 的组合，使 18/32 序列在启动时无候选；这些状态近端公共支持为
.939–1.190m。只在诊断实验将相机视野改为 120°，保持车辆动作/位置相同，启动可规划数
从 14/32 增加到 30/32，车辆回放误差为 0；这不证明能完整跑完道路。途中还存在 5s
近场历史失效、丢失双侧支持和终点前几何不足。运行配置仍保持 80°，不重置 observed_origin_seen、
不放宽原 planner/perception 安全检查、不按候选接受情况改抽初态，也不改用最高分推进训练。
修复后的原 16 个随机起点均空间合法，但同一 8-road 设置仍不满足完整采集条件。

审计报告与修正的覆盖图位于 `.verification/eight-road-bug-audit-20261010/`。
以下 v5 随机采集和训练数值为历史证据，受上述审计限制；当前仅受控场景的管线运行通过验证，
尚未生成满足整路覆盖要求的 8-road 新训练集。

本 feature 内包含训练用 C* 求解器，并直接调用已有 planner_sim。只有一套共享管线；
`run` 执行全部阶段，`collect` 只执行同一套采集、打标和数据集冻结阶段：

~~~text
env.reset/step 产生 observation
  → native lattice 硬筛选并导出候选
  → C* 用实际车辆模型优化每条候选的控制序列
  → env.fork/step 实际执行并返回感知/状态
  → 共享损失生成标签
  → policy 监督训练
  → 排序及规则/学习选路的闭环对比
  → 模型、数据、报告和图表导出
~~~

模型为候选评分器：输入当前 perception/estimation 和候选轨迹，输出预测负损失，
评分最高的候选仅用于模型评价/选路推理。训练数据采集始终随机选路，再由完整 C*
执行终态接续下一段；没有训练控制网络，也没有切回旧 PI 目标。

## 运行（完整路径目标，2026-10-10）

需要 Python 3.12 与 requirements.txt 中的依赖。当前机器已存在可复用的已验证环境：

~~~powershell
& ../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.lattice_training run --output .verification/policy-training
~~~

其他环境先安装依赖再执行同一入口：

~~~powershell
python -m pip install -r offline/lattice_training/requirements.txt
python -B -m offline.lattice_training run --output .verification/policy-training
~~~

随机道路的小规模完整运行（用于检查管线，不代表充分训练）：

~~~powershell
python -B -u -m offline.lattice_training run --output .verification/full-path-training-run --kinds randomized --train-seeds 0,1 --validation-seeds 100 --test-seeds 200 --starts-per-road 2 --planner-profiles standard,wide --decisions 1 --trials 2 --population 4 --iterations 1 --max-evaluations 8 --solver-wall-s 30 --epochs 60 --evaluation-steps 1
~~~

### 固定 8 条随机道路的连续采集

~~~powershell
python -B -u -m offline.lattice_training collect --output .verification/eight-road-collection-20261010 --kinds randomized --train-seeds 0,1,2,3 --validation-seeds 100,101 --test-seeds 200,201 --starts-per-road 2 --planner-profiles standard,wide --until-road-end --max-segments 100 --trials 2 --population 4 --iterations 1 --max-evaluations 8 --solver-wall-s 30
~~~

总计 8 条道路，4/2/2 分配到 train/validation/test，每路两个分层随机起点、两套 lattice，
共 32 个候选序列。`--until-road-end` 覆盖 `--decisions` 的固定段数终止条件；每段仍随机
选路并继承完整 C* 实际终态。终点与完整道路评价共用 `road_length−car_front_extent−.2m`，
检查通过后记录 `road_completed`。无候选、求解/执行失败、完整 STOP 和达到 100 段上限
分别记录真实结束原因；不会重置起点、筛除低分或将这些中断标为道路完成。

`collect` 不训练模型、不运行 policy 闭环。输出 `decisions.jsonl`、每个 episode 更新的
`decisions.collection.json`、`dataset/data.npz`、`dataset/metadata.json`、`summary.json`
和道路覆盖图。报告区分完整候选打标次数、成功次数、道路完成次数，并计时 planning、
C* solve、实际 trial replay、随机 successor replay 和序列化。选择已有输出目录会拒绝覆盖。
冻结后的 NPZ 可直接传给 `model.train_model(dataset_path, model_path, epochs=...)`；增加 epoch
不再运行采集器或 C*。8 次评估预算与上次小规模运行相同，标签是约束数值近似解，
不代表全局最优或更大求解预算下的标注质量。

2026-10-10 实测上述 8-road 命令：采集/打标 909.28s（15.15 分钟），编码 9.39s；
30 个决策点、537 条候选、1074 次试验，其中 1072 次完整成功，2 次保留执行约束违反标签。
8 条生成道路中仅 5 条产生标签；train seeds 1/2/3 的两个随机起点均未通过 native
lattice 可用几何检查。最终 train/validation/test 行数 154/190/193，训练行目前只来自
seed 0。32 个序列中 29 个因 native planning 失败结束，3 个因计划 STOP 结束，0 个
到达道路终点；最长连续 4 段约 8m，未触及段数上限。因此这些耗时是本次真实采样的
耗时，不是 8 条道路完整通行的耗时，也不能把该结果当作充分覆盖 8 条道路的训练集。
实际仿真 trial replay 533.95s（58.7%），C* 优化 345.71s（38.0%），平均每候选 1.69s。
已核验 16 对相邻决策点的状态/动作/时间接续一致，标称预测与完整 env 回放最大状态差为 0。
详细证据保存在 `.verification/eight-road-collection-20261010/REPORT.md`、`analysis.json`
和 `collection_timing.png`。

不需要从外部准备执行记录。默认生成随机长道路，train seeds 0–7、validation 100/101、
test 200/201；每路 2 个初态、standard/conservative 两套 lattice 参数，每个组合最多
3 个完整 C* 段（直到候选空间轨迹终点，.05s 采样，时长由路径/速度决定）。每候选 1 次名义及 1 次
性能偏差试验。闭环步数是整段预算参考，执行已经开始的段不会被预算截断；额外保留 blocked 失败场景。默认规模的 C* 数据生成
明显慢于上面的冒烟运行，尤其是扩大决策数、求解预算和随机试验数时。

原直线/左右弯/障碍场景仍可通过 `--kinds straight,left_curve,right_curve,obstacle` 在同一
管线中运行；其初态为包含车尾及安全边距的 progress≈.233m、lateral±.1m、heading±.01rad、speed=.08–.16m/s。
单障碍受控场景的位置从 1.4m 改为 1.6m，以在合法初态下给 native 的最小 .6m STOP transition 留出空间。
可将它们与 randomized 组合。随机长道路包含双侧锥桶和随机矩形障碍，并保留一条连续通行空间。
车辆参数保持 Notebook 原值。相机为 10Hz/50ms 延迟，wheel/gyro 每步；所有感知和估计
仍由原 env 返回。提交 STOP 后结束该采集 episode；候选标签已完整回放其整个计划时域。

## 随机长道路、初态与反馈采集

`roads.py` 复制并适配 upskill Notebook 的 `clean-random-road-distribution` 配置和
Dream Gym 0.2.2 `random_generation.py` 的范围采样、直线/圆弧采样和路段生成循环。
保留来源文件、cell、版本和 SHA256；运行时不读取或导入 upskill 工程。这里使用既有
Dream Gym 0.4 的嵌套 lane/cone schema，尺寸缩放到 Baby-Bus；车辆动力学不作缩放。

默认分布在 `road_randomization.json`，可用 `--road-settings <json>` 替换：

| 参数 | 默认范围 |
|---|---|
| 路段数 / 直线概率 | 8–10 / .55 |
| 首段直线 / 后续直线长度 | 6–8m / 2–4m |
| 左右圆弧半径 / 转角 | 4.5–8m / 20–35° |
| 参考车道宽度 | 1–1.2m |
| 路段限速元数据 | .72–1.08km/h |
| 累计航向限制 | ±120° |

限速是道路元数据，当前 perception 特征没有道路限速输入，C* 运行速度上限仍由
`--speed-max-mps` 设置。航向限制减少绕圈，未证明道路无自交；分段直线/圆弧保持
位置和切向连续，曲率在连接处可跳变。

**2m 是 native lattice 的局部规划范围，旧直道路本来就是 10m。** 新道路通常长几十米，
提升覆盖的是整条道路上的初始位置和状态，并未将候选长度放大到感知范围以外。
每路把 [车尾/边距对应的安全起点, road_length−3m] 等分为 `--starts-per-road` 个区间，
先扣除保守的障碍占用进度区间，再按剩余长度作一次均匀抽样，
再随机 lateral±.12m、heading±.035rad、speed=.08–.25m/s。各 lattice 参数组合从同一
初态和传感器种子出发。每个初始位置只抽取一次；相机交付需要的零请求预热为 1 步。
感知覆盖不足、native 无候选时记录失败/空 episode，不改换初始位置。

### 每段的状态接续规则

~~~text
当前 observation → native lattice 候选集合
  → 每个候选各自经 C* 优化、实际 env 回放和标注（共用原始快照）
  → 均匀随机选择一个候选（不读取 policy 分数或标签 reward）
  → 在 fork 中完整执行其 C* 控制序列（覆盖完整候选路径）
  → 终点车身处于合法空间：继承整个 env 快照，开始下一段
  → 完整执行后的终点非法：丢弃这个 fork，从同一候选集合重新随机抽取
~~~

下一段保留实际速度、姿态、转向角、方向锁存、最后应用动作、感知/估计历史、
传感器队列和随机数状态。不会只拿终点坐标重新 reset，也不会取候选曲线的几何终点
直接当作实际终态。巡航先保留原纵向时间曲线，再以末端物理速度沿同一 Frenet 路径延伸
至空间终点，时长向上对齐 .05s；不把 3s 时间曲线结束当作 2m 路径结束。停车只执行
完整停车计划及 .5s 稳定段，不延长到障碍物另一侧。超过 max_horizon_s=60s 会明确拒绝，
不会生成截断的高分。

终点合法性检查完整车身及 `--safety-margin-m`，包括双侧参考车道边界、有限道路起终点
和真实障碍物；不因跟踪误差大、reward 低或 policy 排名低而重新选择。非法终点重抽
不放回，最多 `--segment-resampling-attempts` 次，全部失败则显式结束该 episode。

C* 数值求解失败、模拟未完成或 env 提前终止时没有可继承的完整终态：记录失败，
结束当前 episode；这些情况不触发换一条高分路径、重抽初态或重新开始道路。
STOP 完整执行后按停车行为结束。C* 自身仍保留原来的全时域动力学/安全约束，因此
名义执行通常直接得到合法终点；性能偏差试验仍用于标签，不用于选择下一段。

`--planner-profiles standard,conservative,wide` 使用同一 native lattice 的不同采样参数，
局部 horizon 均为 2m。conservative 使用较短过渡距离、±.15m 偏移和 .08/.16m/s 终端
速度；wide 使用五个 [-.25,.25] 偏移。standard/wide 终速包含 .1/.2/.3m/s；离线候选上限 256，
避免较大采样组合被实车默认候选数量上限提前截断；native 其他硬筛选保持。真实随机选中项、每次重抽原因、完整段起终态
和首末动作写入 JSONL。规则基线 ID 独立保存，用于评价模型排序。

### 图里的起点名称

旧图 `accepted starts` 的含义是“抽出的初态经相机预热后，standard lattice 能生成
有效候选”。它只表示当时感知/规划可用，不表示 C* 已经执行、物理最优或道路任务完成。
该筛选已取消。新图黑色叉号 `Initial draw` 表示未按感知/规划可用性筛选的初始抽样，
绿色点 `Legal C* endpoint` 表示完整执行后通过终点空间检查、实际用于下一段的状态。

道路几何 SHA256 是 `scene_id`，初态/参数组合使用独立 episode/group ID。同一路的
所有组合只能进入同一 train/validation/test 分区；不同种子若生成相同几何也拒绝跨区。
每次运行仍是有限数据集，epoch 不自动生成新道路。增加道路 seed、初态或连续段数
可以扩大覆盖；验证/测试道路保持独立。采集过程没有 `--behavior-model` 或规则/学习
选择模式；监督训练和部署式闭环评价仍在同一 run 管线中。

### 随机障碍物的通道约束

默认每路 3–6 个矩形障碍，长度 .20–.35m、宽度 .16–.28m，纵向间隔至少 3.5m，
起终各 3m 留空。位置和尺寸随机；每路随机选择一个障碍物侧，另一侧保持同一条
连续通道，避免交替障碍留下彼此不连通的缝隙。最窄侧向通道至少 .55m；生成后以
原生几何检查 0.25m 宽车身加两侧 .05m 余量，沿通道每 .1m 检查完整不对称矩形。
检查失败直接拒绝生成该布局。它是离散几何可通行检查，不是动态/连续扫掠安全证明，
也不保证当前 native planner 能找到该通道。

同一几何道路的不同障碍布局和限速注释共享 road ID，不能跨数据分区。
单一 run 的 report 记录通道宽度、完整车身余量和采样检查结果。

## C* 的实现与目标

~~~text
C*(s,tau) = argmin_u J_full, subject to native vehicle/input/safety/completion constraints

J_full = mean_k discount^k * (
    w_position * position_error² / position_scale²
  + w_heading * heading_error² / heading_scale²
  + w_speed * speed_error² / speed_scale²
  + w_drive_rate * drive_rate² / drive_rate_scale²
  + w_steering_rate * steering_rate² / steering_rate_scale²
  - w_speed_reward * positive_speed_along_reference / speed_reward_scale
) + w_terminal_position * endpoint_error² / position_scale²
  + w_terminal_speed * final_speed_error² / speed_scale²

policy_target(s,tau) = -mean_trial J_full(actual_env_execution_of_C*, tau)
~~~

共享目标升级为 cstar_full_path_loss_v2 / negative_expected_full_path_cost。
原来 3s 的模型/执行记录不能混用，需要重新生成完整路径标签和训练。
horizon_s=3 现在只表示最小采样时长，不是评分截断；完整 2m 巡航通常需要 7–25s。
同一 objective.py/tracking_loss 用于优化、实际标签和评价，按完整时域平均，使不同
速度造成的不同时长可比。dt=.05s、discount=1，位置/朝向/速度尺度仍为 .1m/.1rad/.1m/s。
新增 speed_reward_weight=.5、speed_reward_scale=.2m/s、terminal_position_weight=2、
terminal_speed_weight=.2；速度跟踪权重仍为 .2。配置尚需根据训练分布整定。

速度奖励使用真实速度沿参考运动方向的正向投影；目标停车阶段不奖励继续移动。
完成约束要求终点误差≤.15m，停车末速≤.02m/s。原地不动不能完成巡航路径，也得不到
速度收益；有足够速度但没到终点的试验按失败编码。道路进度仍另作报告指标。

cstar.py 搜索全部 2N 个控制量，N 来自完整参考。多起点包含原生模型逆运动方向的
瞬态初始化、曲率前馈、缩放、零请求和制动；这些仅是搜索种子，全部经过相同约束、
数值优化和真实 env 回放。默认 population=12、iterations=4、max_evaluations=64、
wall budget=10s。它是数值近似 C*，不证明一般非凸问题的全局最优。没找到解记录
failed 并报告预算/停止原因，不伪称已证不可行。

预测调用原生 BicycleModelDynamic.set_action_request/integrate，克隆完整实例与隐藏
方向锁存，不使用另外一套车辆方程。保持 3kg、轴距 .33m、10N 驱动增益、
1kg/m 二次阻力、RK4/.05s、±45° 转向请求和 ±90°/s 物理转向速率。
零 drive 是零电机请求；制动/反向锁存完全交给原模型。

原始 Frenet d(s)、纵向 s(t) 和 speed_profile 生成统一时域参考，在决策瞬间 CG
坐标系固定。短停车轨迹保持终点，巡航不外推；不用道路中心线或 Cartesian
兼容拟合替换目标。状态包含 CG 位置、运动方向和速度模。零速时运动方向采用
车身朝向加运动学转向方向；与位置不一致的任意车身 yaw 不替代有侧滑时的运动方向。

硬约束使用完整不对称矩形车身、真实道路 reference-lane 边界及原生闭矩形障碍检查。
默认额外运行域为 speed≤.5m/s、纵向/侧向加速度≤1m/s²、仅前进、安全余量 .02m；
这些是显式模拟设置，不是实车标定。检查原模型接受的积分状态，未证明连续 swept
安全性。参数通过 run 的对应选项配置，并写入 label_source。

每个候选从同一完整 env 状态开始，模型输入不使用车辆真值。优化只推进克隆的车辆
模型；然后完整 env.fork/step 回放名义解并验证状态一致。性能试验使用固定种子的
驱动/转向 gain [.9,1.1]、转向 bias±.005、0/1 步延迟，候选共用试验条件。
迟延保存历史请求队列，不每步错误地重置为前一实际动作。

标签来自实际 env 状态及实际应用动作。采样的 N+1 个状态、N 个动作和损失分量
写入内部 JSONL。失败按共享 failure_loss 编码，不把短失败轨迹当成低损失解。
名义回放必须与预测在 1e-7 容差内一致；数据保留原生模型、Notebook、求解预算及种子。
参考跟踪不精确时允许安全的最近执行；安全约束无可行结果时显式失败。

## policy 与闭环评价

输入为 140 维：当前速度/偏航率、道路与边界采样、传感器年龄/可用性、雷达扇区，
以及候选参数、净空、规则代价、Frenet 几何与速度采样。不含 PI 内部状态、仿真
真值或未来结果。模型为 140→32 tanh→1，Adam 优化决策均衡回报 MSE、同决策分差
MSE 与 L2。仅 train 拟合归一化，仅 validation 选 checkpoint，test 不参与优化。

排序报告包含 RMSE、pairwise accuracy、实际选中标签回报及相对候选集合最优标签
的 regret。闭环对比用相同场景初态、噪声和 C* 设置分别执行规则/学习选择。
该闭环评价严格取最高模型/规则分数候选，完整执行这一段，再从实际终态重新规划。
native 的近似平分排序不替代这里的数值 argmax。STOP 也完整执行，再按停车行为结束。
这一阶段用于检验评分器推理，不决定采集数据的下一段起点。
stop_completed 按当前 stop_speed_tolerance_mps=.02m/s 判定；执行完参考不自动代表停车成功。

完全阻塞、无 native 候选或无安全 C* 解均保留为失败，并施加失败编码。报告中的
零碰撞不能单独代表成功，必须结合 fault/completion/time-limit 字段。这是离线
训练/研究评估，未通过实车 35ms 时限、ROS gate 或物理安全验收。

## 整路 evaluation 与可视化脚本

在项目根目录运行；--road 接受原始 road_spec JSON 或包含 road_spec 的配置。
未传 --model 时用 native 规则分数，传入新目标训练的 model.json 时用 policy 分数。
每段都是硬筛选后的最高数值分数候选，经同一个 C* 完整执行；不会换成较低分候选
来掩盖失败。到达车身安全终点、停车、无候选或求解失败后导出完整实际 episode trace。
road_completed 只有到达道路终点前的车身安全位置才为真；停止/中断不会标成成功。

~~~powershell
python -B -m offline.lattice_training.visualize --road road.json --output .verification/road-evaluation --serve
python -B -m offline.lattice_training.visualize --road-seed 0 --output .verification/random-road-evaluation --inspect-progress 1,5,10 --serve
python -B -m offline.lattice_training.visualize --road road.json --model .verification/full-path-training-run/model.json --cruise-speed .2 --output .verification/model-evaluation --serve
~~~

--profile 默认为 wide；--cruise-speed 默认 .3m/s 是 native 规则的期望速度，仍经过硬筛选。
训练默认期望速度为 .2m/s，比较模型时可显式保持一致。--inspect-progress 给出预计算
位置，--start-progress 默认 .25m，--max-segments 默认 100；求解预算选项与 run 相同。
--serve 启动仅监听 127.0.0.1 的页面（默认 8765），结束服务用 Ctrl+C。

页面显示完整实际轨迹、速度曲线、障碍物和候选分数。点击道路或输入进度添加位置；
可以选择沿用真实执行状态（保留感知历史），或关闭此选项设置速度/横向偏移/朝向。
未经过的位置采用该点初始化后的真实感知，不填造道路估计；感知不足会说明无候选。
蓝线为通过硬筛选的候选，红虚线为从 native 参数和同一参考曲线重建的拒绝几何；
后者展示拒绝原因、没有可选分数。没有初始化几何的输入错误也不会造出路径。
点击候选列表可以高亮每条路径；不同速度对应的同形路径仍独立列出。

导出 evaluation.json、evaluation.html、evaluation.png、candidates.png 和 road.json。
直接打开 HTML 可查看已保存位置；任意新点需要正在运行的 --serve 页面。

**开销：** 单候选完整 2m 的变量数量比原 3s 明显增加。实测 7.75–7.8s 的候选，在
8 次评估预算下求解约 .3–.4s，在 64 次预算（约 40 次实际评估）下约 1.5–2.1s；
完整 env 回放还需另计。156 条候选的完整打标已需要数分钟；默认道路/初态/多段/
候选规模可能需要数十分钟到数小时。优先用小规模检查，保持完整路径，不缩回 3s。

## 产物和模块

| 产物 | 内容 |
|---|---|
| decisions.jsonl / decisions.collection.json | 实际采样、求解/回放诊断、场景配置与采样摘要 |
| dataset/ | 数值数据、重算标签、分区/来源/代码指纹 |
| model.json / model.training.json | checkpoint、归一化、目标/来源、训练曲线与排序指标 |
| evaluation.json | 提供的 test 标签上的预测/排序评价 |
| closed_loop.json | 实际 env 动作/轨迹、损失、失败、停车与性能偏差 |
| summary.json | 数据、模型与闭环摘要 |
| training.png / closed_loop.png / road_coverage.png | 学习曲线、实际局部轨迹、完整道路、初始抽样与合法 C* 终点 |

| 模块 | 职责 |
|---|---|
| __main__.py | 唯一 run 入口及 schema/config/score 工具 |
| pipeline.py | env 采样、native 候选、训练和闭环编排 |
| roads.py / road_randomization.json | upskill 随机道路片段适配与尺寸分布 |
| cstar.py | 原模型预测、受约束优化与实际回放 |
| objective.py / cstar_objective.json | 共享目标实现与配置 |
| dataset.py | 内部执行记录、标签校验、分区与持久化 |
| features.py / model.py | 编码、监督训练、排序和权重导出 |
| source.py | 只读使用 native 规划及几何函数 |
| plots.py | 训练/闭环/道路覆盖图 |
| evaluation.py / visualize.py / viewer.html | 共享完整执行、整路评价、位置候选与交互页面 |

schema、cstar-objective、score 仍使用同一目标；只有 run 生成并训练数据：

~~~powershell
python -B -m offline.lattice_training schema
python -B -m offline.lattice_training cstar-objective --output data/objective.json --position-weight 2
python -B -m offline.lattice_training score --records .verification/full-path-training-run/decisions.jsonl --model .verification/full-path-training-run/model.json --output .verification/full-path-training-run/scores.json
~~~

model 保存 feature schema v3、model 格式 v2、C* 目标、数据与 native 规划指纹；
载入拒绝不一致 schema/来源/目标和非法参数。控制组算法/增益不改变规划源指纹。
此 feature 的 C* 服务训练和模拟评估，原 planner_sim、实车 PolicyController、配置和
其他 branch 的文件保持不变。

## 检查

~~~powershell
python -B tests/test_lattice_training.py -v
~~~

单一测试文件同时包含目标算式检查、原生制动/转向限制、预测与完整 env 回放一致性、
父状态/方向锁存隔离、重复性、性能延迟、约束/预算失败、随机道路/初态/分区隔离、
多参数候选差异、完整 C* 终态接续、只因非法终点重抽、分数不影响采样和完整 CLI 训练。
