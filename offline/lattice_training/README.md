# 当前状态、道路与障碍、单条空间路径的 policy

模型每次只评价一条候选空间路径，输入协议为 `spatial_path_policy_v1`。
不输入速度曲线、planner 评分、手工净空/平滑性代价或历史路径。
[输入说明](POLICY_INPUT.md)定义各组数据和标签边界，
[接线说明](INTEGRATION_CN.md)记录当前离线入口。

道路与候选保留原生有序几何，不重采样成固定点表；模型内部使用可训练线段编码
和候选对场景的注意力，交互后汇聚评分。障碍按前方 8 片区保留最近点 XY 和可用性。
道路各曲线独立可空，无中线不阻止有效 native 候选评分。

原 Dream Gym 车辆、RK4、虚拟传感器、C* 求解与监督目标保留。C* 仍读取训练适配器
附加的时间/速度参考；policy 不读取这些字段。标签包含速度收益及跟踪成本，
不能把本次输入替换说成已实现路径最高速度求解。默认训练执行条件仍为巡航 0.2 m/s。

```powershell
python -B -m offline.lattice_training schema
python -B -m offline.lattice_training run --output .verification/spatial-policy-new --kinds straight --train-seeds 0 --validation-seeds 100 --test-seeds 200 --starts-per-road 1 --planner-profiles standard --decisions 1 --trials 1 --population 4 --iterations 1 --max-evaluations 8 --evaluation-steps 2 --epochs 15
python -B -m offline.lattice_training train --dataset .verification/spatial-policy-new/dataset --output .verification/spatial-policy-trained-new --epochs 150
python -B -m offline.lattice_training score --model .verification/spatial-policy-new/model.json --records .verification/spatial-policy-new/decisions.jsonl --output .verification/spatial-policy-new/scores.json
python -B -m unittest tests.test_policy_frenet tests.test_lattice_training -v
```

训练/验证/测试道路仍隔离，Train-only augment 不改变 Holdout。
`collect`、`train`、`score`、`augment` 和可视化共用唯一新模型/输入入口。
历史模型与数据保留，当前入口拒绝旧 schema，不自动补列、重算或转换。
[旧数据审计](artifacts/2026-10-16-spatial-policy/data-audit.json)保留存在性、完整性和协议检查结果。
主线仍未接入学习评分。

## 随机合法续接的独立采集

当前 `collect` 在冻结的仿真决策状态生成 native 候选，为每条候选求解 C*，执行
标称与扰动试验并保留所有标签。续接时对 C* 可行候选随机排序，完整执行后检查
全过程约束和终点；失败则尝试下一条，不按 native 分数、C* 回报或 policy 排序。
成功后恢复真实执行的完整终态及传感器历史，携带上一控制量继续生成候选。
无合法续接或达到预算时显式失败，不跳过失败状态重置车辆。

`--until-road-end` 运行到任务终点附近；终点为原道路长度减车头长度及 0.2 m，
收集判定复用 C* 的 0.15 m 完成容差。末段按剩余任务距离调整 native 生成长度，
不截断或重拟合候选。这个判定不要求停车。
`--terminal-support-m` 可在任务道路后方明确延伸物理道路和锥桶，原始采样范围与任务终点不变。
`--planner-rate-hz` 是 native 内部预测保持时间的参数，本批取 20；实际每走完一整段才再调用
planner，求解期间仿真不前进。这不是主线频率或实时性能验收。

首批数据、模型、审计、完整归档和可复现命令见
[2026-10-16 流程记录](artifacts/2026-10-16-new-protocol-batch/REPORT_CN.md)。

## 扩展场景与重新划分

`scenarios.py` 定义左右弯、五类障碍布局和左右掉头。掉头外侧是半圆接两条直线，
内侧只有一排直线锥桶；不生成内侧圆弧。`expanded.py` 复用上述完整随机合法续接，
`--modes no_centerline` 同时从 native 和 policy 的感知输入中移除中线及 forward 中线，
保留实际边界/障碍。它是明确记录的受控缺失，不是相机自然缺线的证明。

单侧宽度未知时仍保留已观测边界供 native 生成候选，不补道路宽度或对侧。
`--start-mode road_start` 用于检查
连续感知历史；`--scenario-width-prior` 则通过既有 `known_lane_width_m` 参数显式提供
该场景宽度，仍使用相机重建曲线，不注入地图几何。两类实验独立存档，主线 YAML 不变。

本次按用户要求重新划分，而非 Train-only augment：`repartition.repartition_records`
仅改写 split 并保存来源。道路、同路缺中线视图及镜像/同构造种子场景保持在同一集合。
可以合并不同道路/采集数量，C* 求解设置、目标、车辆及采样语义仍须一致；旧协议拒绝加载。
先前 holdout 此次明确退役，不再用于声称独立测试。

[扩展实验目录](artifacts/2026-10-17-expanded-collection/)包含复现脚本、数据审计和训练脚本。
训练脚本先导出无 Test 行的开发数据，三个初始化只比较验证集失败选路次数及分数 RMSE，
冻结模型选择后再评价完整测试集。`failure_audit.py` 区分可避免失败、全候选失败和扰动失败；
`recheck_failures.py` 从原始 episode 完整回放到同一状态，以更大 C* 预算诊断失败，不覆盖标签。
这些检查均不把 C* 可行性或 planner 分数送入模型，也不接入主线。

扩采结果及剩余失败见 [实验报告](artifacts/2026-10-17-expanded-collection/REPORT_CN.md)。
`--obstacle-layout near_center_alternating --candidate-count 16` 明确补充车头可能接触障碍的难例，
不改变物理道路分组。所有标签及无可行续接状态保留；测试结果不保证学习策略闭环成功。

## 固定掉头验证道路

[validation_roads/](validation_roads/README.md) 固化原版、通行宽度减半、弯曲掉头及
半宽连续双掉头，每类左右方向各一份，均有版本和文件指纹。
半宽按用户最终定义，同时减半道路全宽与掉头参考半径，直线段保持原长。
`python -m offline.lattice_training.validate_uturns --output <new-dir> --start-mode road_start`
通过现有感知、native 与 C* 验证；可显式传入冻结模型增加 policy 对照，不自动训练或接入主线。
当前单边无中线且宽度未知的 native 内部参考偏移为 0.60 m，搜索半宽仍为 0.45 m；
已知宽度、双边和已提供中线的规则保持。
