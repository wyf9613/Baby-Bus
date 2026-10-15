# 当前状态、道路与障碍、候选轨迹的 policy

模型只接收三组数据：`当前状态，当前道路与障碍，候选轨迹`。
[输入协议](POLICY_INPUT.md)定义字段、坐标、采样和版本。

当前只有 `mvp21_policy.lattice.ShadowLatticePlanner` 生成候选。训练评分器不读取
planner 的代价、规则评分、候选类型、净空统计、感知侧降速建议或上一条计划路径。
没有旧 planner / 140 维 / 148 维模型的兼容加载或旧多项式候选解码。

`features.py`只做原始数据的固定采样、点排序、填充和掩码编码；MLP 的输入归一化仅
使用 Train 数据。原始道路形状和障碍点保留，不转成道路偏移统计或雷达扇区最小距离。
碰撞准入、源时效、真实车辆动力学和控制执行属于模型外的验证与执行框架。

原 Dream Gym 车辆、RK4、虚拟传感器和 C* 执行管线继续使用。C* 使用完整候选路径
执行产生标签，失败如实记录；目标仍是各 trial 原始回报的均值再经过 sigmoid。
native 规则分数只用于独立基线比较，不进入模型张量。训练采样仍均匀随机选路，
不按 policy 或标签选择训练接续。道路分区隔离、真实执行终态接续和原输出防覆盖保留。

```powershell
# 使用新目录；python 应为项目虚拟环境解释器
python -B -m offline.lattice_training schema
python -B -m offline.lattice_training run --output .verification/state-road-trajectory-new --kinds straight --train-seeds 0 --validation-seeds 100 --test-seeds 200 --starts-per-road 1 --planner-profiles standard --decisions 1 --trials 1 --population 4 --iterations 1 --max-evaluations 8 --evaluation-steps 2 --epochs 15
python -B -m offline.lattice_training train --dataset .verification/state-road-trajectory-new/dataset --output .verification/state-road-trajectory-trained-new --epochs 150
python -B -m offline.lattice_training score --model .verification/state-road-trajectory-new/model.json --records .verification/state-road-trajectory-new/decisions.jsonl --output .verification/state-road-trajectory-new/scores.json
python -B -m offline.lattice_training.visualize --model .verification/state-road-trajectory-new/model.json --output .verification/state-road-trajectory-view-new --max-segments 1
python -B -m unittest tests.test_policy_frenet tests.test_lattice_training -v
```

`collect`复用同一采集器而不训练；`augment`只追加新 Train 道路并核对 Holdout 不变。
它们仅接受当前格式数据。旧权重、冻结数据及实验材料属于历史档案，当前入口不加载。
小预算 C* 未找到可行解不等于证明候选不可行；随机候选链可能提前中止，报告保留原因。
ROS 节点仍未接入学习评分，实际接线范围见[集成说明](INTEGRATION_CN.md)。
