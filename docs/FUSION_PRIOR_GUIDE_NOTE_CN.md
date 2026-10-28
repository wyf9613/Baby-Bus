# lattice 时间融合在急弯里越融越短、越融越偏（给 Zhenyu / astune 组的说明，2026-10-28）

适用：mvp2.1、sim 分支、`checkpoint/rnn-mpc-car13-20261026`（`45d492f`）、本仓库 `mpc-car-run`，`_shadow_fuse_road` 逻辑相同（`45d492f` 在 `mvp21_policy/lattice.py` 第 337 行起）。详细数据见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 10-28。

## 现象

10-28 用 1 号车跑一个左 U 弯，0.2 和 0.5 m/s 下一共 3 次，每次都是转到大约 70–90° 时，车突然往右（往弯外）打满舵，然后出界。MPC 只是照着 lattice 选的路径在走；lattice 每次规划用时 0.13–0.34 s，没有超时。

## 原因：`_shadow_fuse_road` 用上一次的参考线当骨架

1. **骨架**：每次融合都以 `previous_reference` 为骨架（guide），只要它的年龄不超过 `fusion_window_s + 2/rate_hz`（3 s），先变换到车当前位姿下。
2. **往前延伸只有一条路**：当前观测中心线上的点，必须投影在骨架末端之后（`s > base.end + fusion_spacing_m`），并且横向距离小于 `search_half_width_m`（0.45），才会接到骨架后面。
3. **拟合窗口不超过骨架末端**：上限是 `min(base.end, s0 + 1.5 × length)`，车身后面的部分被裁掉（只留 `fusion_rear_support_m`）。

急弯里，当前观测的中心线朝弯内拐，点落在骨架侧面，不在末端之后，一个都接不上。于是：

- 车每往前走一段，骨架尾部被裁掉一段，前面补不上；
- 骨架一次比一次短，方向停在旧的朝向；
- 车已经左转了，这截旧骨架在车头坐标系里指向右边；
- lattice 的候选围绕这条参考线生成，选出往右的路径。

只看得到单侧锥桶时（U 弯里只看得到外圈）没有两侧观测纠正，更严重。直道和缓弯不受影响，新观测的点就在骨架前方，接得上。

## 证据

- 把车上 `/car/shadow_planning` 录下的 `temporal_input` 和 `raw_planning_input` 原样喂给 `_shadow_fuse_road` 和 `ShadowLatticePlanner.plan`，**逐位重现**车上选出的路径。
- 骨架长度：2.0 → 1.68 → 1.24 → 0.55 m，0.69 → 0.60 → 0.39 m；每次接上的新点数都是 0。

| 规划时刻 | 当前观测中心线 | 融合后（车上实际用的） | 去掉 previous_reference 重算 |
| --- | --- | --- | --- |
| 0.5 m/s 21.58 s | 左 50° | 右 22°，长 1.24 m | 左 46° |
| 0.5 m/s 22.59 s | 左 38° | 右 32°，长 0.55 m | 左 55° |
| 0.2 m/s 36.44 s | 左 10° | 右 64°，长 0.60 m | 左 32° |
| 0.2 m/s 37.48 s | 右 12° | 右 101°，长 0.39 m | 左 9° |

- lattice 最终选中的路径：坏的四次是右 43° / 53° / 157° / 54°，去掉骨架后左 17° / 43° / 31° / 55°。
- 开环回放（`replay_policy_bag --lattice`）复现不出：回放出图时刻和车上不同，骨架链也不同。要直接用录下的融合输入复现。

## 改法：`shadow.fusion_use_prior`

新增布尔参数，默认 `true`（原行为）；`false` 时融合不再拿 `previous_reference` 当骨架。历史帧照样参与融合；没有上一次参考线时本来就走"最新的两侧可见帧，否则最新帧"，现在每次都走这条。改动 3 处：

```python
fusion_use_prior: bool = True          # ShadowSettings
...
if (cfg.fusion_use_prior and prior.get("odometry_epoch") == epoch and prior.get("pose")
        and 0 <= prior_age <= cfg.fusion_window_s+2/cfg.rate_hz):
    prior_points = ...
```

另外在 `__post_init__` 里加了布尔检查，yaml 里声明默认 `true`。

## 结果

- **实车**（1 号车，同一个 U 弯，同一个起点）：0.2 m/s 下 5 次都没有再往右拐，不限距离的包转完了整个 U 弯（+155°）；0.5 m/s 下 2 次在 5 m 内转到 +142° 和 +137°，没有越界、没有碰锥。
- **单元测试**：四个规划时刻的录制输入做成 fixture，默认设置重现往右，`false` 时往左。
- **Gym**（0.5 m/s，60 次）：原来 46/60，去掉骨架 48/60，整个关掉融合 42/60。**按场景配对这些差异都不显著**（46 对 48：p = 0.77；关融合：p = 0.15），翻转几乎都在直角弯。Gym 的 U 弯两侧都看得见，测不到这个问题；Gym 只说明"没有看到去掉骨架把别的路段弄坏"，改法的依据是实车和录制输入。

## 你们那边可能受的影响

RNN 是在融合后的参考线周围的候选里挑路径，参考线本身歪了，换评分器也纠正不过来。急弯、单侧可见时很可能同样出界。可以先在车上的 `/car/shadow_planning` 里看：

- `fusion.reference_crop.output_length_m` 有没有一次比一次短；
- `fusion.prior_used` 是不是 `true`。

代码在分支 `fusion-prior-reset`（提交 `a21f8cb`，已过 WSL 门禁），checkpoint：tag `checkpoint-car01-uturn-noprior-20261028`。
