# 2026-10-15 native 道路实验快照

保存分支：`feature/native-road-checkpoint-20261015`，基于
`feature/lattice-training-pipeline` 的 `f1a6c20`，保存本轮连续边界改动、测试、
记录道路重建/回放工具和诊断证据。原训练/评分模型和训练数据未重新训练或改写。
后续MVP2.1接入使用另一个分支，不能据其结果改写此快照的验收结论。

当前结论见[入弯诊断](REMAINING_DIAGNOSIS_CN.md)。连续车身几何修改后187项相关
检查通过；实测道路的20 Hz闭环未通过，隔离主机计时的最新原生对照在
中心线进度1.946846 m失败，目标4.736800 m。ROS gate缺少固定的dream_interfaces
checkout，未完成；没有部署或实车动作。

可移植的最小重建道路fixture为
`artifacts/2026-10-15-native-diagnosis/recorded-road.json`，包含原重建的road_spec、
42个融合锥桶、颜色映射及来源/限制。保留原日志SHA256，不将入口延拓称为实测道路。
完整原始日志、仿真trace、训练数据、依赖源码和Python虚拟环境仍保存在本机
`.verification/`中，按现有.gitignore排除；这些运行产物不靠Git复制。

当前继续复用的解释器，从本工作树根目录调用：

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe
```

环境为Python3.12.14、Dream Gym0.4.0、NumPy2.3.5、Gymnasium1.4.0、Matplotlib3.11.2。
重建环境的版本要求继续使用`offline/lattice_training/requirements.txt`与
`offline/planner_sim/requirements.txt`，不提交虚拟环境本身。

复现当前基线：

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.lattice_training.native_replay --road offline/lattice_training/artifacts/2026-10-15-native-diagnosis/recorded-road.json --config offline/lattice_training/native_recorded_road.json --output .verification/native-road-checkpoint-replay
```

该基线保留真实35 ms预算，未完成道路时命令返回非零；这不是预期通过的验收用例。
