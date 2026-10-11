# 2026-10-11 候选评分模型与训练记录

此目录保存可随 Git 分享的小型训练产物。模型来源于唯一 `offline.lattice_training`
管线的真实冻结标签训练；不包含大型原始仿真 trace 或数据集。

- [model.json](model.json)：扩充 Train 后的模型，140→32 tanh→1 sigmoid，Adam/.005/seed0，150 epochs，保存 epoch150。
- [previous-model.json](previous-model.json)：扩充前同目标模型，保存 epoch49，保留用于对照。
- [训练报告](REPORT.md)、[训练曲线](training.png)、[固定验证/测试集对照](comparison.png)。
- [训练历史](model.training.json)、[标签评价](evaluation.json)、[保存模型与保护核对](verification.json)。
- [测试排序退步诊断](ranking_diagnosis.md)、[逐状态诊断](ranking_diagnosis.json)。
- `manifest.json`：本目录各文件的 SHA256 与源/模型合同摘要。

## 模型含义

目标为 `sigmoid(mean(raw C* trial returns))`，T=1。模型输出 0–1 评分，用最大分数
选择通过 native lattice 硬筛选的候选；分数不是成功概率或原始回报。
新模型使用 Train3295候选/145状态/11路，Validation942/37/2和Test699/35/2保持。
Validation 选路改善；Test 的评分误差改善，但选路平均回报和排序略降，详见报告。
没有整路闭环验证或全局最优保证。模型加载会检查当前 native planner/特征/目标合同。

## 使用现有候选记录评分

先按 [管线依赖](../../requirements.txt) 准备环境，再运行唯一评分入口：

```powershell
python -B -m offline.lattice_training score --model offline/lattice_training/artifacts/2026-10-11-augmented-sigmoid/model.json --records your_decisions.jsonl --output scores.json
```

候选记录遵循管线的 source/feature/schema 合同；模型只使用 observation 和 candidate 特征。
本目录不在 ROS 节点中自动启用学习选择器。

报告中的 `.verification/` 路径和训练时保护记录为原始实验记录。完整冻结数据仍在本地：
`.verification/train-augmentation-20261011/dataset/`。完整复现实验需要这批原始数据，
或通过 `collect`/`augment` 重新采集；模型推理不需要训练数据。
