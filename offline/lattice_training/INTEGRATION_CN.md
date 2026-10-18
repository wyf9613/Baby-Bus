# 单条空间路径评分接线

当前唯一 native 生成器为 `mvp21_policy.lattice.ShadowLatticePlanner`。
离线 policy 使用[结构输入契约](POLICY_INPUT.md)，不依赖 native 评分项。

1. `pipeline.make_planner()` 创建最新生成器适配器。中线参考仅作诊断，不再先以其成功
   作为 native 候选生成的准入条件；保留无中线和单侧道路能力。
2. `candidates_at(planner, observation, model)` 每次将一条已过滤候选传入
   `encode_candidate` 和 `SpatialPathPolicy.score`。不传候选列表、速度曲线或历史路径。
3. 原始候选几何不变。C* 使用适配器附加的显式时间/速度规律执行并生成标签；
   此参考不进入 policy 编码。
4. 数据集保存逐条结构 JSON；变长曲线由模型内的线段编码和场景注意力处理。
   不保留旧 140/1285 维加载分支或旧模型兼容入口。

两个 ROS script 名称进入同一模块化 node。ROS/仿真当前均由 native 路径加固定
巡航目标的 PID 执行，暂未加载学习模型；CMake 不安装离线训练包或权重。
这些离线接线不改变主线控制、停止状态机或传感器准入。

扩展采集见 `scenarios.py`、`expanded.py` 和 `repartition.py`。缺中线是同时施加到 native
与学习输入的显式观测遮除。掉头的场景宽度先验通过现有感知配置注入，仅用于标记的
离线实验；相机边界、C* 约束及主线 YAML 保持原实现。模型选择使用无 Test 行的开发数据。
