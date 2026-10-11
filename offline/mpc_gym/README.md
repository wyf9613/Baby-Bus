# MPC 的 Dream Gym 仿真实验

本目录有两类实验，用的都是 `scripts/policy_node.py` 中的 MPC（`MPCController` / `VehicleModelMPC`），和车上运行的是同一份代码。全部是仿真结果，不是实车证据。入口文档见 [docs/MPC_README.md](../../docs/MPC_README.md)。

| 实验 | 被控对象 | 参考来源 | 用途 | 结果 |
| --- | --- | --- | --- | --- |
| **M6 对比（v2）**：`run_gym.py`、`harness.py` | 课程 Gym 车辆，0.5–1.2 m/s | 道路真值多项式 | 在 PID 实验完全相同的条件下比较 MPC 和 PID | `results/comparison.md` |
| **实车条件（v3）**：`pipeline_sim.py`、`newcar27_experiment.py` | 拟合出的 .27 纵向响应（`NewcarPlant`）叠加在 Gym 车辆上，0.2 m/s | Gym 锥桶 → 估计 → Planning，即完整链路 | 在 .27 的时序和动力特性下比较 MPC 和 MVP | `results/newcar27_comparison.md` |

下面先讲 M6 对比，再讲实车条件仿真。

## M6 对比：MPC 与 PID 基线

把 MPC 放进 PID 组的 Dream Gym 实验（`offline/control_pid/tune.py`，未作改动），在**同一条件**下与 PID 基线比较。

**v3 复核（2026-10-07）**：v3 的新功能在课程车辆上默认关闭（死区为 0、偏置增益为 0、起步油门为 0、失效容忍为 0），所以行为不变。用 v3 代码重跑 `--validate`，62 次运行的跟踪指标与 `results/` 中 v2 的结果最大相差约 1e-4（转向变化率），其余在 1e-6 以下。只有耗时下降：OSQP 改为复用后，`notebook_03` 的单步 p95 从 13.2 ms 降到 3.4 ms。所以 `results/` 里的 v2 结果保留不变，其中的耗时数字偏保守。

### 运行

使用装有 Dream Gym **0.4.0** 和 numpy、scipy、osqp 的 Python 环境，在仓库根目录执行：

```powershell
python offline/mpc_gym/test_mpc_gym.py          # 框架一致性和冒烟测试
python offline/mpc_gym/run_gym.py --tune        # 整定 MPC 权重，然后自动验证（约 15–20 分钟）
python offline/mpc_gym/run_gym.py --validate    # 只用已保存的权重重跑验证
```

### 条件怎么对齐

| 项目 | 做法 |
| --- | --- |
| 被控对象 | `tune.vehicle(scenario)`：Gym `BicycleModelDynamic`，含驱动/阻力缩放和转向偏置 |
| 参考 | `tune.reference_for`：道路真值 → 车体坐标三次多项式，噪声和 dropout 相同，随机数调用顺序相同 |
| 仿真循环、延迟、包络检查、完成判定、指标 | `harness.run` 逐行对应 `tune.simulate`。`test_mpc_gym.py` 检查 PID 走这个循环得到的指标和逐步轨迹与 `tune.simulate` **完全相等** |
| 执行器范围 | 两者都是 drive `[-0.3, 0.35]`（MPC 只用 `[0, 0.35]`），转向 ±π/4 |
| 整定 | MPC 在 PID 的训练场景（`speed_nominal` / `speed_mismatch`、`TRAINING`）上，用 PID 的评分公式，网格规模与 PID 相同（27 + 27） |
| 验证 | PID 的 9 个验证场景 × 种子 101/202/303。PID 使用其保存的 `selected_gains.json`，在同一次运行中重跑 |

额外场景：低速 `notebook_03`（0.3 m/s，接近实车计划速度），两个控制器都跑。MPC 专项研究（已知延迟、时域 N = 5 / 20）单独列表，不计入对比。

### 与 PID 实验不同的地方

- MPC 使用 v0.1 点列：`poly_to_trajectory` 在多项式上每 5 cm 采样一次（yaw = atan y'，曲率 y''/(1+y'²)^1.5），并在末端沿切线补一段直线，长度覆盖预测时域。否则在道路尽头会因 `reference_too_short` 被拒绝。这一点只影响 MPC。
- MPC 以 `shadow=False` 执行，`v_exec_max_mps` 取场景速度（速度阶跃测试取 1.0）。车辆记录使用 notebook 的仿真器参数，标注为 `sim:dreamgym-0.4.0-notebook`，不是实车测量值。
- MPC 拒绝某一步时，在车上会锁定停车，需要显式请求 state 3。这里在下一步重建控制器，并计入 `MPC stops`，对应 PID 离线时"无效参考清零后继续"的做法。
- 停车：MPC 只发正向驱动，所以只能滑行停车；PID 用仿真器的负驱动锁存刹车。速度阶跃测试中的停车时间不能直接比较。

### 输出（`results/`）

- `comparison.md`：逐场景并排对比表、速度阶跃测试、MPC 专项研究、局限说明
- `comparison.png`：横向误差、转向、速度、驱动的对比曲线
- `validation_metrics.csv`、`mpc_studies.csv`：每次运行的完整指标
- `*_trace.csv`：种子 101 的逐步记录
- `tuning_candidates.csv`、`selected_weights.json`：整定过程和选出的权重
- `experiment.json`：各版本、源码与 notebook 的哈希、场景、种子、权重和增益

`selected_weights.json` 是针对 0.5–1.2 m/s 仿真选出的权重，**没有**回写到 `config/ai4r_policy.yaml`。实车的计划速度是 0.2–0.3 m/s，是否采用由团队决定。

### 在 notebook 中查看

```python
from pathlib import Path
import pandas as pd
from IPython.display import Image, Markdown, display

results = Path("offline/mpc_gym/results")          # 从仓库根目录运行
display(Markdown((results / "comparison.md").read_text(encoding="utf-8")))
display(Image(results / "comparison.png"))
metrics = pd.read_csv(results / "validation_metrics.csv")
metrics.groupby(["scenario", "controller"])[["lateral_rmse_m", "lateral_max_m"]].mean()
```

### 结论的边界

参考来自道路真值，没有经过锥桶检测、估计和规划。MPC 用的是仿真器自身的参数，所以名义场景对 MPC 有利，判断鲁棒性要看失配、延迟、噪声和 dropout 场景。耗时是笔记本电脑上的数据，不是 Jetson 上的。

## 实车条件仿真（v3）：.27 的时序和动力特性

### 做法

`pipeline_sim.py` 用 Dream Gym 的相机锥桶驱动节点里真实的 `calculate_policy_actions`（估计 → 规划 → MPC 或 MVP）和 `_store`，构成闭环。`run()` 的关键参数如下，默认值保持原来的流程测试：

| 参数 | 含义 | `NEWCAR27_CONDITIONS` 的取值 |
| --- | --- | --- |
| `trigger` | `cone`：每到一批锥桶算一次；`timer`：每 50 ms 算一次（实车 20 Hz timer） | `timer` |
| `cone_latency`、`latency_jitter` | 相机从采集到发布的延迟和抖动（s） | 0.19 + 0–0.03（实车约 0.19–0.22） |
| `gap_probability` | 每批锥桶被丢掉的概率，丢掉时连续丢两批，形成约 0.3 s 的空档 | 0.05（实车出现过 331 ms 的间隔） |
| `lane_width_m`、`cone_spacing_m` | 赛道宽度和锥桶间距 | 0.72 m、0.2 m（实车锥桶在 y ≈ +0.33 / −0.38 m） |
| `plant` | 请求和 Gym 车辆之间的执行器模型；`None` 表示课程车辆 | `NewcarPlant.from_fit()` |
| `distance_m` | 轮速积分达到这个距离就算完成 | 3.0 |

`NewcarPlant` 的做法：
- 把拟合出的 .27 纵向模型强加到 Gym 车辆上，模型参数来自 `offline/vehicle_identification/results/newcar27_drive_fit.json`：
  - 死区以上 `a·(u − d0)`，以下 `e·(u − d0)`（拖刹），再减去 `c·v·abs(v)`；
  - 0.1 s 延迟；
  - 静摩擦：请求 ≥ `u_break` 并保持 `t_break` 后车轮才转。
- 每一步先算出目标加速度，再**反解 Gym 自己的力学方程**得到要给 Gym 的油门。所以锥桶、几何和横向动力学仍是 Gym 的，纵向响应则等于拟合模型。
- 速度接近 0 时不给 Gym 负油门，避免进入它的倒车锁存。
- 转向：Gym 收到的是取反后的请求（.27 正请求向右转），可以再加一个前轮角零偏 `steering_offset_rad`。

### 运行

在仓库根目录执行：

```powershell
# 完整矩阵：先用 MVP 做标定，再在 25 个场景上比较 MVP 与 MPC（N = 10/8/5），约 1.5 分钟
python offline/mpc_gym/newcar27_experiment.py --horizons 10 8 5
python offline/mpc_gym/newcar27_experiment.py --quick          # 小矩阵

# 单次运行：用合并好的 YAML，套用 .27 条件，3 m 判定完成
python offline/mpc_gym/pipeline_sim.py <yaml> --car newcar27 --distance 3.0
```

`newcar27_experiment.py` 按顺序合并 `config/ai4r_policy.yaml` → `ai4r_policy_newcar27.yaml`（MVP）→ `ai4r_policy_mpc_newcar27.yaml`（MPC）。MPC 运行时**仅在仿真里**把 `vehicle.valid` 设为 true、`shadow` 设为 false。

场景：
- 起始位姿：0、+0.15 m、−0.15 m 加 0.08 rad；
- 锥桶噪声 2 cm / 5 cm，3 个随机种子；
- 死区 0.27 / 0.289 / 0.305（模拟电量变化）；
- 转向零偏 0 / ±0.03 rad；
- 一个 20% 丢帧的极端场景。

### 当前结果（`results/newcar27_comparison.md`、`.json`）

- **标定**（MVP 用实车参数，对照实车 B12）：起步延迟 0.50 s（实车 0.53 s），稳态 0.202 m/s（实车 0.20），**峰值 0.261 m/s（实车 0.333）**。拟合低估了脱离静摩擦时的加速度。
- **跑完 3 m 的数量**：MVP 0/25，MPC 24/25（N = 10/8/5 相同）。MPC 唯一失败的是 20% 丢帧场景。这个差距**主要来自失效处理方式**：MVP 遇到任意一帧规划失败就锁停，MPC 会沿用上一步计划。它不代表跟踪精度差这么多。
- MPC 横向 RMSE 平均 0.034 m（N = 10），峰值速度 ≤0.293 m/s；笔记本上单步 p95 为 3.0 ms（N = 10）和 1.8 ms（N = 5）。
- 死区在 0.27–0.305 之间都能跑完（靠起步爬升和偏置估计）；±0.03 rad 的转向零偏造成约 0.02 m 的稳态偏移。

### 局限

- 仿真丢帧比实车多，两种控制器的绝对距离都偏保守。
- 起步峰值偏低约 0.07 m/s，评估超速时要考虑这一点。
- 拟合只来自一台车、一天的数据；Gym 的横向动力学和锥桶检测不等于实车。

## 测试

`python -m unittest discover -p "test_*.py"`（在本目录执行）共 9 项：
- 原有 6 项：框架一致性、冒烟测试，以及两项流程闭环测试；
- `Newcar27Plant` 类 3 项：
  - MVP 复现 B12 的起步延迟和稳态速度；
  - MPC 在 .27 条件下跑完 3 m 且不锁停；
  - 用过原点的课程纵向模型时，车在 .27 上起步不了。
