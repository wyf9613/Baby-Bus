# MVP2.1 lattice → MPC：15 号车 run.jsonl 开环回放（2026-10-16）

生成方式：`tools/replay_run_jsonl.py <run.jsonl> --planner-control`。同一个 run 回放两遍：

- 第一遍只用中心线（ordered_xy + road_centerline），和以前的回放一样；
- 第二遍叠加 `config/ai4r_policy_mpc_lattice21.yaml`，lattice 选出的路径经执行适配器后交给 MPC。

两遍都是 MPC shadow，逐拍对比。lattice 在本进程里同步计算，但结果要等"本机实测规划耗时 + 0.05 s"之后才可见，模拟 1 Hz 子进程的交付延迟。

**这是开环回放**：车实际是按 PID 走的，MPC 的输出不会改变后面的车位置。所以下表只说明"参考选了谁、和中心线差多少"，不代表闭环跟踪结果，也不是车载耗时。规划耗时是本机墙钟时间，每次运行略有不同，`planner_ticks` 可能差 1。

## 参考选择

| run | 拍数 | 用 lattice | 碰到已知边界退回 | 中心线无效时不用 lattice | 其他退回 | 切换次数 | lattice 结果 | 本机规划耗时 均值 / 最大 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| visualization4 | 388 | 319 | 66 | 0 | 等首个结果 3 | 11 | 19/19 VALID | 50 / 77 ms |
| end-straight1 | 398 | 347 | 27 | 13 | 分支不可用 9，等首个结果 2 | 8 | 20/20 VALID | 61 / 92 ms |
| end-straight2-1m | 394 | 320 | 60 | 12 | 等首个结果 2 | 10 | 19/19 VALID | 55 / 70 ms |

用 lattice 的拍上，MPC 全部是 `shadow`（求解成功），没有因为 lattice 参考被 MPC 拒绝。

## 和中心线 MPC 的差别

| run | 用 lattice 的拍 | MPC 转向和中心线 MPC 反号 | 和车上 PID 反号（lattice） | 和车上 PID 反号（中心线 MPC，对照） |
|---|---:|---:|---:|---:|
| visualization4 | 319 | 61 | 81 | 43 |
| end-straight1 | 346 | 145 | 203 | 46 |
| end-straight2-1m | 319 | 94 | 118 | 33 |

"反号"只统计两边转向绝对值都大于 0.05 的拍。

**这是本次最需要注意的结果。** lattice 参考下，MPC 转向和 PID 反号的拍数是中心线 MPC 的 2–4 倍。从选中的候选看，原因主要有两个：

1. **Halton 随机候选占多数**：19 次里 10 次、20 次里 12 次。评分只看前 `score_min_length_m = 0.2` m，0.2 m 以外的形状不参与比较。
2. **"弯外准备"**：end-straight1 在大转角（`turn_angle_rad` −1.7 到 −2.9）期间，`preparation_offset_m` 一直约 +0.14 m，路径先往弯外走，MPC 于是往弯外打方向。

mvp2.1 文档自己也写了"大转向 / 参考方向冲突"还没解决。这次 MPC 接入没有改评分，也没有改候选，所以这个问题原样带了过来。

另外，中心线的近端是只用前方几何（forward-only）补出来的；在这些拍上，中心线 MPC 的横向误差有 −0.35 到 −0.45 m，lattice 路径从车身起步，误差接近 0。所以单看 `e_y` 的差别（中位数 5–10 cm，最大 0.57 m），并不能说明哪边对。

## 结论

- 接线本身在三段真实数据上都正常：结果按 1 Hz 到达，经对齐、裁剪后被 MPC 采用，退回原因都有记录。
- 不能据此进入执行模式。下周的 shadow 要重点看：
  - `/car/mpc_debug` 的 `reference_selection` 和 `candidate_steer_action`，与车上实际转向是否反号；
  - `/car/shadow_planning` 选中的候选类型（`selected_family`）。
- 要真正改善，需要 Planning 组去调评分范围或候选（例如 `score_min_length_m` 不小于 MPC 预测距离）。这些不在本次接入范围内。

## 复现

```powershell
python -B tools/replay_run_jsonl.py docs/experiments/2026-10-14/evidence/car15/visualization4/run.jsonl --planner-control --json-out lattice21_vis4.json
```

`--lattice-delivery-s` 用来改交付延迟。逐拍数据在 `--json-out` 的 `lattice_steps` 里，字段包括 `reference_selection`、`reference_selection_reason`、`mpc_e_y_m`、`mpc_steer`。
