# MPC 与 PID 基线的 Dream Gym 对比（M6 证据）

本目录把 `scripts/policy_node.py` 中的 MPC（`MPCController` / `VehicleModelMPC`，与车上运行的是同一份代码）放进 PID 组的 Dream Gym 实验（`offline/control_pid/tune.py`，未作改动），在**同一条件**下与 PID 基线比较。全部是仿真结果，不是实车证据。

## 运行

使用装有 Dream Gym **0.4.0** 和 numpy、scipy、osqp 的 Python 环境，在仓库根目录执行：

```powershell
python offline/mpc_gym/test_mpc_gym.py          # 框架一致性和冒烟测试
python offline/mpc_gym/run_gym.py --tune        # 整定 MPC 权重，然后自动验证（约 15–20 分钟）
python offline/mpc_gym/run_gym.py --validate    # 只用已保存的权重重跑验证
```

## 条件怎么对齐

| 项目 | 做法 |
| --- | --- |
| 被控对象 | `tune.vehicle(scenario)`：Gym `BicycleModelDynamic`，含驱动/阻力缩放和转向偏置 |
| 参考 | `tune.reference_for`：道路真值 → 车体坐标三次多项式，噪声和 dropout 相同，随机数调用顺序相同 |
| 仿真循环、延迟、包络检查、完成判定、指标 | `harness.run` 逐行对应 `tune.simulate`。`test_mpc_gym.py` 检查 PID 走这个循环得到的指标和逐步轨迹与 `tune.simulate` **完全相等** |
| 执行器范围 | 两者都是 drive `[-0.3, 0.35]`（MPC 只用 `[0, 0.35]`），转向 ±π/4 |
| 整定 | MPC 在 PID 的训练场景（`speed_nominal` / `speed_mismatch`、`TRAINING`）上，用 PID 的评分公式，网格规模与 PID 相同（27 + 27） |
| 验证 | PID 的 9 个验证场景 × 种子 101/202/303。PID 使用其保存的 `selected_gains.json`，在同一次运行中重跑 |

额外场景：低速 `notebook_03`（0.3 m/s，接近实车计划速度），两个控制器都跑。MPC 专项研究（已知延迟、时域 N = 5 / 20）单独列表，不计入对比。

## 与 PID 实验不同的地方

- MPC 使用 v0.1 点列：`poly_to_trajectory` 在多项式上每 5 cm 采样一次（yaw = atan y'，曲率 y''/(1+y'²)^1.5），并在末端沿切线补一段直线，长度覆盖预测时域。否则在道路尽头会因 `reference_too_short` 被拒绝。这一点只影响 MPC。
- MPC 以 `shadow=False` 执行，`v_exec_max_mps` 取场景速度（速度阶跃测试取 1.0）。车辆记录使用 notebook 的仿真器参数，标注为 `sim:dreamgym-0.4.0-notebook`，不是实车测量值。
- MPC 拒绝某一步时，在车上会锁定停车，需要显式请求 state 3。这里在下一步重建控制器，并计入 `MPC stops`，对应 PID 离线时"无效参考清零后继续"的做法。
- 停车：MPC 只发正向驱动，所以只能滑行停车；PID 用仿真器的负驱动锁存刹车。速度阶跃测试中的停车时间不能直接比较。

## 输出（`results/`）

- `comparison.md`：逐场景并排对比表、速度阶跃测试、MPC 专项研究、局限说明
- `comparison.png`：横向误差、转向、速度、驱动的对比曲线
- `validation_metrics.csv`、`mpc_studies.csv`：每次运行的完整指标
- `*_trace.csv`：种子 101 的逐步记录
- `tuning_candidates.csv`、`selected_weights.json`：整定过程和选出的权重
- `experiment.json`：各版本、源码与 notebook 的哈希、场景、种子、权重和增益

`selected_weights.json` 是针对 0.5–1.2 m/s 仿真选出的权重，**没有**回写到 `config/ai4r_policy.yaml`。实车的计划速度是 0.2–0.3 m/s，是否采用由团队决定。

## 在 notebook 中查看

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

## 结论的边界

参考来自道路真值，没有经过锥桶检测、估计和规划。MPC 用的是仿真器自身的参数，所以名义场景对 MPC 有利，判断鲁棒性要看失配、延迟、噪声和 dropout 场景。耗时是笔记本电脑上的数据，不是 Jetson 上的。
