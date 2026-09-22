# Baby Bus：三个子组如何对应代码

团队：9 人，Perception / Planning / Control 各 3 人。本文是团队工作划分建议，不是已经实现的算法或已通过实车验证的接口。

## 当前代码是什么

本仓库起点为老师的 `ai4r_policy`，上游提交 `ff6049d80838f817350547708a112d2389793c21`。实际策略入口在 [scripts/policy_node.py](../scripts/policy_node.py) 的 `calculate_policy_actions()`。

目前 INSERT POLICY CODE 区域只有注释示例，默认驱动和转向为零。已有的是 ROS 输入、传感器健康检查、动作验证、发布与策略状态机。团队要实现的是中间的决策算法。

三个子组是同一个策略内部的逻辑分工，不是三个独立 ROS 节点。遵循老师的单文件要求，实车策略仍放在 `scripts/policy_node.py`，保留插入标记与框架契约。

```text
老师提供的传感器节点
        ↓
框架订阅、校验、缓存观测
        ↓
calculate_policy_actions()
  ① Perception：观测 → 道路、障碍物与车辆状态
  ② Planning：环境与状态 → 参考路径、目标速度、行为
  ③ Control：参考值与状态 → drive_action / steering_action
        ↓
框架检查有限值、裁剪、发布
        ↓
车辆接口（独立 Enable / Disable / Disarm）
```

## 各组具体编辑什么

| 子组 | 已有输入 | 开发工作 | 下游输出（建议） |
| --- | --- | --- | --- |
| Perception | `x_coords`、`y_coords`、`cone_colour`、`cone_confidence`、`lidar`、轮速和 IMU | 过滤异常锥桶、拟合边界、识别路径附近障碍、按需估计状态 | `scene`：边界、障碍物、估计质量；`state`：速度、姿态等有效状态 |
| Planning | `scene`、`state` | 中心线/局部路径、单侧边界处理、跟随/减速/停车决策 | `plan`：路径、目标速度、行为及有效性 |
| Control | `plan`、`state`、`dt` | 横向跟踪、纵向速度控制、标定映射、限幅与积分处理 | `drive_action`、`steering_action`，可选调试量 |

Perception 接收的是已检测出的锥桶，而不是重新实现 YOLO。相机、LiDAR、IMU 和车辆驱动节点由老师维护。

Planning 决定“在哪里走、需要多快、是否停车”；Control 决定“如何把这些目标变成实际电机与转向请求”。目标速度单位 m/s，驱动动作是归一化电机请求，不能直接互相赋值。

## 先约定接口，再实现

以下名字是团队待实现的内部接口，不是框架现有变量：

| 字段 | 建议约定 |
| --- | --- |
| `scene` 的边界/障碍物坐标 | 统一车体平面，x 前、y 左，米；明确 LiDAR 安装变换和原点 |
| `scene` 的有效性 | 区分双侧、单侧、无可靠边界；未知障碍信息不能等同于道路畅通 |
| `state.speed_mps` | 明确来源和有效性；实际轮速无符号，缺失值不当作 0 |
| `plan.path_xy_m` | 有序的局部路径点，车体坐标；首版可只使用一个前视目标点 |
| `plan.target_speed_mps` | 非负前进目标速度；首版不包含倒车 |
| `plan.behavior` | 例如 FOLLOW / STOP；需明确停止是否锁存以及恢复条件 |
| `plan.valid` | False 时执行双方约定的停止处理，不能沿用不明期限的旧动作 |

同一次策略调用按 Perception → Planning → Control 顺序执行。需要跨步保留的状态使用明确的 `self` 属性，在 `is_first_policy_step` 时初始化；第一步 `dt == 0`，禁止直接除以 dt。

停车是跨组接口：Perception 提供障碍与可信程度，Planning 决定停止，Control 实现并验证减速响应。目标速度 0、零驱动指令和物理停稳并不等价。框架还独立处理必需传感器过期，但不能替代障碍停车逻辑。

## 与配置、仿真的关系

[config/ai4r_policy.yaml](../config/ai4r_policy.yaml) 由各组共同确认，指定唯一触发源和真正依赖的传感器。例如速度闭环依赖有效轮速，障碍停车依赖雷达，则需要对应配置与 runtime unit。不要仅启用传感器却忘记策略依赖声明。

本地课程 Notebook `../notebooks/ad_gym_system_project_v2026_09_18.ipynb` 中的 `ConeFollowingPolicy.compute_action()` 是另一套仿真接口：边界拟合对应 Perception；边界推算前视中心对应 Planning；转向增益和固定 drive 对应初始 Control。

Notebook 的固定 drive 不是速度控制器。仿真策略可以作为基线，但观测名、颜色常量、雷达角度和制动行为需要适配后才能进入实车策略。本次代码推送不包含工作区外层的 Notebook、PDF 或个人 Obsidian 设置。

## 团队开发流程

1. 共同跑通简单完整基线：锥桶 → 前视目标 → 低速跟踪，先仿真再验证实车。
2. 为每个子组建立工作分支，例如 `feature/perception-boundaries`、`feature/planning-stop`、`feature/control-tracking`。
3. 各组约定同一文件内的编辑区域和变量，保持短提交；使用 PR 讨论后集成，避免三组同时重写整个策略方法。
4. 每组指定一名集成联系人，共同审核接口；每次合入后验证完整流程，不只看单个模块。
5. 记录代码版本、参数、场景和指标：边界/障碍误差、路径偏差、跟踪误差、停车距离、误停车和接触次数。

部署仍是：本机修改 → scp → 车上 build → restart policy → 明确 Enable / Run。GitHub push 不会自动改变车上代码。

## 本次整理的验证边界

仅添加团队文档和 GitHub 协作入口，执行代码和运行参数保持老师版本。尚未实现团队算法，也未在本机运行 ROS fast gate、仿真或实车测试。上游 `docs/acceptance.md` 中的测试结果属于老师的历史证据，不是本团队本次验证结果。
