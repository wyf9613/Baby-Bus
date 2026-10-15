# 在实际 policy 中接入候选评分模型

## 1. 当前已经具备什么

| 项目 | 当前状态 |
|---|---|
| 模型和特征编码 | `ReturnMLP.load/predict`、`scene_features/encode_candidate` 已实现 |
| native planner 接口 | `FrenetLatticePlanner.plan(..., candidate_scorer=callback)` 已实现 |
| 可用 checkpoint | [最新模型](artifacts/2026-10-11-augmented-sigmoid/model.json)、[扩充前模型](artifacts/2026-10-11-augmented-sigmoid/previous-model.json) |
| 实际节点接线 | 尚未启用；`PolicyNode.calculate_policy_actions()` 当前没有传入评分回调 |
| ROS 安装 | 当前 CMake 仅安装 policy 脚本、launch/config，没有安装此 Python 包或模型 |

下面提供接线示例，需要实施对应节点修改才能生效。`planning.scorer.*` 是本指南建议新增的
启动参数，当前节点尚未声明它们。已有 `score` 命令只对候选记录评分，不会启用 ROS 节点。

在线流程：

```mermaid
flowchart LR
    O[当前感知与状态估计] --> P[native lattice 生成并检查候选]
    P --> M[模型逐候选评分]
    M --> S[选择当前合格集合中的最高分]
    S --> R[原 planning_reference_v2 输出]
    R --> C[现有控制器]
```

推理只需要 observation、native 候选和模型参数。C*、仿真环境、trial 和 reward 打标用于训练，
不在在线回调中运行。评分不直接产生油门/转向动作。

## 2. 准备模型与运行路径

完整 checkout 中的模型路径为：

```text
offline/lattice_training/artifacts/2026-10-11-augmented-sigmoid/model.json
```

模型输入为 `lattice_scene_candidate_v3` 的 140 维特征，输出 0–1 分数。
目标是 `sigmoid(mean(raw C* trial returns))`，T=1；分数不是成功概率或原始回报。
`predict()` 已执行输入归一化和输出 sigmoid，调用方直接返回该数值。

以下示例复用仓库中的推理实现。运行 policy 的 Python 需要 NumPy，并能导入完整 checkout
内的 `offline.lattice_training`。当前推理导入链不需要运行 dreamgym、C* 或 Matplotlib。
在 Ubuntu/ROS 的启动终端中，可显式提供 checkout 路径：

```bash
export BABY_BUS_REPO=/absolute/path/to/Baby-Bus
export PYTHONPATH="$BABY_BUS_REPO${PYTHONPATH:+:$PYTHONPATH}"
```

`model_path` 使用绝对路径，避免 ROS launch 的工作目录改变后找不到文件。
仅复制 `model.json` 不足以提供特征编码和推理代码；仅构建当前 CMake 包也不会安装这些文件。
最终单文件 policy 交付需要把相同版本的特征/推理实现纳入该交付，保持特征顺序及模型合同。
本指南的源码接线示例不代表这项打包工作已完成。

`ReturnMLP.load()` 默认检查特征 schema、张量/归一化、C* 目标以及 native 源码指纹。
当前指纹读取位置是 `offline.lattice_training.source.SOURCE`，默认指向该 checkout 的
`scripts/policy_node.py`；特征几何也使用这份源码。因此必须确保 ROS 安装的实际 planner 与
该 checkout 的 native 实现一致。这个默认检查不能替你验证另一份安装脚本。
若在新位置重新打包，需要显式保持或适配这项源码定位和版本校验；关闭 `check_source`
不能解决版本不一致。

在已经 source 正确 ROS overlay 的终端，可对照当前安装入口和 checkout：

```bash
cmp "$BABY_BUS_REPO/scripts/policy_node.py" "$(ros2 pkg prefix ai4r_policy)/lib/ai4r_policy/policy_node.py"
```

没有输出且退出码为 0 表示这两份脚本逐字节相同；不一致时先确认实际启动包/overlay，
并按现有构建流程安装同一份修改后的节点。

## 3. 在节点启动时加载一次

位置：[scripts/policy_node.py](../../scripts/policy_node.py) 的 `PolicyNode.__init__()`，
在 `self.planning_settings` / `self.lattice_settings` 初始化之后加入。下面使用已有
`ParameterDescriptor`；三个推理函数由启用分支导入并保存到节点。

```python
self.declare_parameter(
    "planning.scorer.enabled", False,
    ParameterDescriptor(read_only=True),
)
self.declare_parameter(
    "planning.scorer.model_path", "",
    ParameterDescriptor(read_only=True),
)
self.candidate_model = None

if self.get_parameter("planning.scorer.enabled").value:
    if self.planning_settings.algorithm != "lattice_v2":
        raise ValueError("Candidate scoring requires lattice_v2")

    from pathlib import Path
    from offline.lattice_training.model import ReturnMLP
    from offline.lattice_training.features import scene_features, encode_candidate
    from offline.lattice_training.source import candidate_api

    path = Path(self.get_parameter("planning.scorer.model_path").value)
    if not path.is_absolute():
        raise ValueError("planning.scorer.model_path must be absolute")

    self.candidate_model = ReturnMLP.load(path)
    self.scorer_scene_features = scene_features
    self.scorer_encode_candidate = encode_candidate
    candidate_api()  # 提前完成几何定义加载，避免首次评分时做 AST 编译。
```

模型文件在启动时加载，policy step 中不重新加载模型或训练。
这里的几何预热消除首次 AST 编译，但当前 `candidate_api()` 仍会在候选编码时读取源码以检查
缓存。该示例是直接复用现有代码的接线版本；正式实时适配还需要把固定版本的几何 API
缓存到推理路径，并保持相同特征计算，避免这项源码文件 I/O。当前尚未实现这项适配。
模型加载错误在启动阶段报告；`enabled=False` 继续使用现有规则选择。

## 4. 每次规划用同一帧 observation 构造评分回调

所需信息已经存在于当前 policy 内部，下面的 `scoring_observation` 只是把现有字段组装到
训练时相同的字典结构。无需额外感知输入或单独生成一套仿真 observation。
函数入参 `observations` 是原始传感器快照；评分器的 observation 还组合了本帧估计输出。

| 模型 observation 字段 | 实际 policy 中已有的来源 |
|---|---|
| `estimation.state` | `self.estimation_output["state"]`：速度、偏航率和有效性 |
| `estimation.road` | `self.estimation_output["road"]`：边界/中心线、宽度、置信度、范围、来源年龄等 |
| `estimation.obstacles` | `self.estimation_output["obstacles"]`：已处理的障碍点和可用性 |
| `sensor_age_s` | `calculate_policy_actions()` 的同名入参 |
| `sensor_available` | 由同一调用的 `observations[name] is not None` 派生；节点已将过期可选测量置为 `None` |
| `timestamp_s` / `frame_id` / `dt_s` | 当前估计时间、`self.policy_frame_id` 和函数入参 `dt` |

当前 `scene_features()` 实际读取的顶层键是 `estimation`、`sensor_available`、`sensor_age_s`；
时间/坐标系/dt 是同时保留的元信息，不是额外模型测量输入。候选特征由 native 回调的
`proposal` 提供。某帧测量缺失时保留原来的缺失/有效性状态，不能为了凑输入填成有效测量。

位置：`calculate_policy_actions()` 中，已有的
`estimates = self.estimation_output`、`source_timeouts` 构造之后，替换当前的
`self.planner.plan(...)` 调用。保留原有 planner 的初始化/恢复逻辑。

```python
plan_options = {
    "mvp": getattr(self, "control_settings", ControlSettings()).mode == "mvp",
}

if isinstance(self.planner, FrenetLatticePlanner) and self.candidate_model is not None:
    feature_sensors = (
        "cone_detections", "wheel_speed", "imu_angular_velocity", "lidar_cartesian",
    )
    scoring_observation = {
        "timestamp_s": estimates["state"]["timestamp_s"],
        "dt_s": dt,
        "frame_id": self.policy_frame_id,
        "estimation": estimates,
        "sensor_available": {
            name: observations.get(name) is not None
            for name in feature_sensors
        },
        "sensor_age_s": {
            name: sensor_age_s.get(name)
            for name in feature_sensors
        },
    }
    context = self.scorer_scene_features(scoring_observation)

    def score_candidate(proposal):
        features = self.scorer_encode_candidate(
            scoring_observation, proposal, context=context,
        )
        return float(self.candidate_model.predict(features[None, :])[0])

    plan_options.update(
        candidate_scorer=score_candidate,
        collect_candidates=False,
    )

self.planning_output, self.planning_diagnostics = self.planner.plan(
    estimates["road"], estimates["state"], estimates["obstacles"],
    estimates["vehicle_limits"], estimates["state"]["timestamp_s"],
    source_timeouts, **plan_options,
)
```

这个调用对应实际节点已有的局部变量。评分使用同一次 policy step 的估计、传感器可用性与
原始年龄；缺失传感器的年龄保留 `None`，由既有特征编码处理。不要用未来状态或仿真真值。
传给 planner 的估计记录同时用于该帧的模型特征。

`context` 只在本次 `plan()` 内复用；下一帧重新计算。不能跨帧仅按 `candidate_id` 缓存分数，
因为相同采样参数在不同道路/状态下可能得到相同 ID，但几何和 observation 已变化。
MLP 无循环隐藏状态；native planner 的历史参考、近场历史和 STOP latch 继续由同一
`self.planner` 管理，不能每帧重新创建 planner。

## 5. callback 收到什么、planner 返回什么

`candidate_scorer` 的输入是一个 native `proposal`，不是只有空间 waypoint 的数组：

| 字段 | 特征用途 |
|---|---|
| `candidate_id` | 本帧候选标识和诊断 |
| `descriptor` | 横向过渡长度/偏移、名义时长、末端速度、规则代价分量 |
| `reference.frenet_path` | 原始参考曲线和横向多项式，用于几何采样 |
| `reference.longitudinal_profile` / `speed_profile` | 纵向运动及速度采样 |
| `reference.planned_stop` / `valid_for_s` | 停车与寿命特征 |
| `minimum_clearance_m` | native 车身/障碍检查得到的净空 |

回调返回一个有限 Python `float`。使用现成 `encode_candidate()`，保持 140 维的特征顺序；
不要自行改成仅输入 `reference.path`，也不要手工重新归一化。

native 先执行当前行为下的车身、障碍、运动、来源时效和 Cartesian 输出复核，再调用评分。
成功返回时，选择本次实际评估且输出兼容的候选集合中最高分的一条；这不是全空间最优保证。
巡航/停车行为优先仍由 native 决定，模型只在当前允许的集合内排序。

输出仍是 `planning_reference_v2`，实际节点继续把 `self.planning_output` 交给已有控制器。
诊断中可读取 `selected_candidate_id`、`selected.model_score` 和 `selection_scope`。
`collect_candidates=True` 可额外导出候选作诊断；实时评分不需要开启它。

回调异常/非有限分数会使规划返回无效及停止请求；预算或来源期限超出也会拒绝输出。
沿用现有 `planning_output["valid"]`、`PolicyStopRequest` 和控制侧检查。
不要在回调里运行 C*、仿真 rollout、JSON 文件写入或其它阻塞工作。

## 6. 启动设置和当前适用范围

实施上述节点修改并解决 Python/单文件交付路径后，现有 ROS 参数配置可增加：

```yaml
planning.algorithm: lattice_v2
planning.scorer.enabled: true
planning.scorer.model_path: /absolute/path/to/Baby-Bus/offline/lattice_training/artifacts/2026-10-11-augmented-sigmoid/model.json
```

这些是启动参数，换模型后重启节点。执行周期沿用当前 policy 的触发频率，不另起评分定时器。
这三个设置不会替代原有传感器、车辆参数、控制和显式启动配置。

当前模型的训练和实际 native 推理存在一个已知候选集合差异：训练默认直接使用通过
Frenet 检查的候选，绕过 Cartesian 拟合阶段；实际 native 仍只给兼容输出候选评分。
特征编码相同，但实际可选集合可能更小。离线的 `allow_unobserved_origin` 补足假设也
不随模型自动进入 ROS 节点。模型不会修复 native 的无候选、近场支持或 Cartesian 拒绝。

此指南说明接口与源码接线，不是已完成的 ROS 部署。当前已核对模型加载和记录候选评分，
尚未核对目标车上的完整 native 搜索加评分时延，也未完成 ROS/实车闭环。
评分回调需要放入现有规划预算内；不能按离线预算直接修改实时节点参数。
