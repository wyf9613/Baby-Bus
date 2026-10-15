# 接入候选评分模型

旧 V2 planner 已移除。当前生成器是 `mvp21_policy.lattice.ShadowLatticePlanner`，
policy 通过[三组输入契约](POLICY_INPUT.md)与生成器评分解耦。

离线接线已经实现：

1. `pipeline.make_planner()` 创建最新生成器的 `PolicyCandidatePlanner` 适配器。
2. `candidates_at(planner, observation, model)` 生成并筛选候选，再用相同的
   `encode_candidate` 编码，以 `ReturnMLP.predict` 回调评分。
3. 评分最高的已准入候选保持原始空间几何，按明确的时间曲线交给原 C* 求解器。
4. C* 真实执行日志用于训练标签；模型按共享接口/encoder 校验，生成器指纹仅追踪来源。

输入只包含当前状态、当前道路与障碍、候选轨迹；不读取 planner costs、净空代价或
前一条路径。旧模型/数据只作历史档案，旧加载器与多项式候选解码已删除。

两个 ROS script 名称均使用 `mvp21_policy.node`。节点中的 lattice 当前仍为 shadow，
尚未装入模型或评分回调，CMake 也不安装离线训练包和权重。本次迁移没有启用机器人
上的学习规划。后续接入需显式安装 scorer、核对同一接口/encoder，并保持状态机、输入
有效期与停止行为。1 Hz 执行接线目前仅在[虚拟环境](../mvp21_sim/ONE_HZ_CONTROL.md)验证。
