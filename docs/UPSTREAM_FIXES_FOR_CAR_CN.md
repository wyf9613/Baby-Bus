# 估计与 Planning V2 的实车问题和修复（mpc-car-run）

读者：估计组、Planning 组。分支 `mpc-car-run`（基于 `control-mpc-v0`，内含 `mvp2.0` 的 V2 和多帧估计）。

**所有改动都加了开关，代码默认值不变**：`config/ai4r_policy.yaml` 和你们的测试行为完全不变，只有 MPC 组的车上配置 `config/ai4r_policy_mpc_newcar27.yaml` 打开了这些开关。改法供你们参考，也可以直接在你们的配置里打开。

## 一页结论

在 car .27 的条件下（0.19–0.22 s 相机延迟、5% 锥桶批次丢失、2 cm 锥桶噪声、20 Hz 定时器、拟合的驱动模型），用 Dream Gym 跑真实的估计 → V2 → 控制链，每格 10 个随机种子：

| 场景 | MPC 改前 | MPC 改后 | MVP 改前 | MVP 改后 |
| --- | --- | --- | --- | --- |
| 直道 1.0 m，居中 / 偏 5 cm / 车头偏 5° | 5 / 5 / 5 | **10 / 10 / 10** | 2 / 6 / 4 | **10 / 10 / 10** |
| 直道 0.72 m，居中 / 偏 5 cm / 车头偏 5° | 2 / 0 / 1 | **10 / 8 / 10** | 0 / 0 / 0 | **10 / 9 / 10** |
| 缓弯 R 5 m、45°，两侧锥桶，1.0 m（跑完整个弯，5 m） | 0 | **10** | 0 | **10** |
| 同上，0.72 m | 0 | **9** | 0 | **9** |
| 课程赛道弯道（只有左侧锥桶）/ S 弯 | 0 | 0 | 0 | 0 |

（数字为 10 次里跑完的次数。完整表：`offline/mpc_gym/results/car_readiness.md`。）

**MVP 的改善和 MPC 一样**，说明修的是上游，不是控制器。只剩单侧锥桶的弯道没解决，原因见第 7 节。

## 问题与修复

### 1. 相机断档就清空锥桶历史 → `estimation.history_propagation: continuous`

- **现象**：跑到一半，一次约 0.3 s 的相机断档之后，近处道路一直是 `near_field_unobserved`，直到锁停。
- **根因**：断档期间最后一帧对齐到当前时刻的跨度 = 断档 + 相机延迟 ≈ 0.5 s，大于 `motion_max_interval_s`，报 `motion_interval_too_long`。这不在 `QUALITY_FAILURES` 里，于是 `update()` 走到 `_clear_road_history()`。之后新帧只能看到 0.9 m 以外，而 V2 见过真实近处道路后（`observed_origin_seen`）不再用起步假设，所以再也拿不到有效道路。可是这段时间轮速和 IMU 一直是连续的，历史本来可以推过去。
- **改法**（`EstimationPipeline`）：
  - `_advance_history(now_s)`：每个控制步都把历史推到当前时刻，每次只积分约 0.05 s，不会再跨长时间段；
  - `_history_at(source_s)`：把历史放到相机帧的采集时刻，向前用 `transform_xy`，向后用新增的 `EstimationMotionHistory.inverse_transform_xy`；
  - 断档时（`CAMERA_GAP_FAILURES = ("unavailable", "motion_interval_too_long")`）这一拍不输出道路，但**不清空历史**。
- **不变的规则**：运动数据缺失或中断、重置、时钟回退、乱序帧、与新观测冲突时，照样清空。
- **改后**：断档之后的第一帧就恢复 `observed_with_history`（测试 `CarRunHistoryChecks.test_continuous_history_survives_a_camera_gap`，以及使用原逻辑的对照测试）。

### 2. 历史只能活 5 s，车慢时不够 → `estimation.road_history_mode: distance`

- **现象**：没有断档也会周期性地丢失近处道路。Gym 里能看到，最老的锥桶一到 5.0 s 被删掉，近端覆盖就从 −0.15 m 跳到 +0.45 m。
- **根因**：相机最近只能看到约 0.9 m。车速 0.2 m/s 时，一个锥桶从被看到到走到车身原点要 4.5 s；要让原点前后都有覆盖，至少需要 5.5 s。代码把 `road_history_s` 限制在 ≤ 5 s。
- **改法**：`distance` 模式下，锥桶一直保留到落在车后 `road_rear_support_m` 以外；时间上限改用新参数 `road_history_max_s`（默认 20 s、最大 30 s），给静止时兜底。由于锥桶只在 `[-road_rear_support_m, road_forward_max_m]` 范围内保留，单个锥桶被推算的距离最多约 3.5 m。

### 3. 去重半径把相邻锥桶删了 → `estimation.history_merge_radius_m: 0.08`

- **现象**：开了上面两项之后，车身旁边的历史里仍然出现 0.6–0.8 m 的空洞。每当一个锥桶走到大约 0.4–0.5 m（正好离开相机视野），它的前一个锥桶就从历史里消失。
- **根因**：新检测会删除历史里 `2*huber_delta_m = 0.16 m` 范围内的旧点（同一个半径也用在冲突检查上）。锥桶间距是 0.2 m，只要推算有几厘米偏差，第 k 个锥桶的新检测就会把第 k−1 个锥桶当作重复删掉。
- **改法**：去重半径单独设为 `history_merge_radius_m`（0 表示沿用原来的 0.16 m），**冲突检查仍然用 0.16 m**。取值要小于锥桶间距的一半。
- **效果**：1 + 2 + 3 一起开之后，直道 1.0 m 从 5/10 变成 10/10。只开 1 和 2 时 MPC 没有改善，所以这一项是关键。

### 4. 单侧帧不使用历史 → `estimation.history_merge_single_side: true`

- **现象**：入弯或车头偏时，当前帧是 `single_side_width_unknown` 或 `insufficient_common_range`，只能靠 0.2 s 的预测桥接，之后就失效。G2 实车 E 场景（车头右偏）有 76% 是单侧。
- **改法**：这类质量失败的帧，在通过原有冲突检查后，把**看得到的那一侧**新锥桶和两侧历史合并重拟合。两侧都有效且覆盖原点时输出 `observed_with_history`，`boundary_source` 标明哪一侧只来自历史，并带 `merged_partial_frame: true`。合并失败就原样返回这一帧，**不改历史**。
- **效果**：两侧都有锥桶的缓弯从 0/10 到 10/10，这一项和上面几项一起起作用。

### 5. 盲区外推把噪声放大 → `planning.lattice.blind_extension_mode: parallel`

- **现象**：0.72 m 路上车偏 3–4 cm 就报 `recovery_body_boundary_conflict`，属于硬停车。
- **根因**：`LatticeCurve` 向后外推最多约 1.3 m 时，沿用了**每条边界自己**近端的斜率和曲率。近端斜率是三点二次拟合出来的，锥桶噪声被一路外推到车身旁边，走廊在盲区里变窄。
- **改法**：左右边界都沿**中心线**在近端的切线方向直线外推，曲率为 0，保持近端观测到的路宽（`LatticeCurve(..., extension_slope=)`，由新的 `_lattice_corridor` 统一构造；`FrenetLatticePlanner` 和 `MvpRecoveryPlanner` 共用）。观测范围内的曲线完全不变。
- **效果**：0.72 m 缓弯从 2/10 到 9/10，0.72 m 直道从 9/10 到 10/10。
- **适用范围**：盲区内假设道路是直的。起步时车停在直道上没有问题；在弯道里起步需要重新评估。

### 6. 起步近处空缺上限没有余量 → `max_near_gap_m: 1.0`、`max_extension_m: 1.4`（只改配置）

- **证据**：G2 实车回放中，车左偏 10 cm 的 B 场景，75% 的帧因为最近能看到的锥桶超过 0.9 m 被拒（`insufficient_lattice_coverage`）。Gym 里车偏位加航向时最近锥桶在 0.949 m。
- **改法**：上限放宽到 1.0 m，`max_extension_m` 相应改为 1.4（补足长度 = near + 0.198 + 0.15）。依赖第 5 项，避免外推更长时放大噪声。
- **Gym 无差异**：0.2 m 锥桶间距下最近能看到约 0.6 m。效果以 G2 回放为准。

### 7. 没有解决：弯道上只有一侧锥桶

课程赛道的弯道（R 5 m、45°）只摆了左侧锥桶，S 弯也一样。第 4 项能多撑约 0.35 m；之后右侧历史只延伸到车前 0.76 m，扣掉车头长度后规划距离不够，V2 报 `no_feasible_lattice_trajectory`，硬停车。要继续，就得用**测得的路宽**把看不到的一侧推出来（估计器已有 `known_lane_width_m`，但 V2 要求双侧观测）。这改变了“看不到就不规划”的安全语义，需要你们决定。我们没有做。

## 还发现的两点

- **回正规划推演的是 MVP 的 P 控制器**（`MvpRecoveryPlanner`，`mvp_lateral_kp`、`mvp_heading_kp`）。我们试过在 MPC 模式下关掉回正规划（`recovery_enabled: false`），跑完的次数完全相同，失败原因只是从 `recovery_body_boundary_conflict` 变成 `no_feasible_lattice_trajectory`。所以那几次失败是物理上真的挤，不是控制器模型对不上。我们保留了默认设置。
- **垃圾回收停顿**：测试进程里一次完整回收会停顿 80–150 ms，超过 50 ms 控制步，也接近规划的源截止时间（ROS 门禁曾因此稳定失败）。`main()` 和规划子进程启动后各调用一次 `gc.collect(); gc.freeze()`；`test_lattice` 和 `test_mpc` 的计时检查也做了同样的隔离。

## 怎么在你们的配置里打开

```yaml
estimation:
  history_propagation: continuous
  road_history_mode: distance
  history_merge_radius_m: 0.08      # < 半个锥桶间距
  history_merge_single_side: true
planning:
  lattice:
    blind_extension_mode: parallel
    max_near_gap_m: 1.0
    max_extension_m: 1.4
```

## 怎么复现

```bash
# 完整矩阵（约 3 分钟，6 个进程）：改前和改后，MPC 和 MVP
python offline/mpc_gym/car_readiness.py --workers 6
# 只看某个开关的作用，例如关掉单侧融合
python offline/mpc_gym/car_readiness.py --variants fixes --roads gentle_arc --extra estimation.history_merge_single_side=false
```

单元测试：
- `tests/test_estimation.py` 的 `CarRunHistoryChecks`：断档、对照的原逻辑、运动中断仍清空、逆变换、按位置保留、去重半径、单侧融合，以及默认关闭和冲突清空；
- `tests/test_lattice.py`：平行外推几何、窄路偏位起步。

## 需要你们决定的
1. 第 1–5 项是否并入你们的分支、默认是否打开；
2. 单侧锥桶弯道是否允许用实测路宽推断另一侧（第 7 节）；
3. 实车上 `history_merge_radius_m` 按实际锥桶间距取值（G2 场地 0.3 m，10-06 场地 0.2 m）。
