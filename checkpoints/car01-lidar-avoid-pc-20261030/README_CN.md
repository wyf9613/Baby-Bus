# Checkpoint：1 号车雷达避障，执行中复查（2026-10-30）

用来验证 Zhengxi `2c4fbff` 的修复：规划时检测区域为空的路径，执行中用当前扫描复查，不再"等待、方向回正"。

- 代码：`eadf40b`（`mpc-car-run` 合并 `2c4fbff`）。WSL 门禁 offline PASS、ros gate PASS（`~/ai4r-evidence/ros-20261030-140457`）。
- **yaml 和 10-28 的 `-lidar-avoid` 包逐字节相同**，只换了 `policy_node.py`。新参数都用代码默认值：`shadow.min_turn_radius_m` 0、`weight_consistency` 0、`switch_cost_margin` 0；`mpc.r_ddelta` 2.0。
- 半径为什么是 0：雷达回放（周三 `175647`）里，半径 0.5–0.9，或只在有点时用 0.9，等待帧数都是半径 0 的 1.8–2.4 倍；半径 0 时绕行路径曲率中位数 5 /m，车开不出来。**所以这个包只跑无障碍环线，不做放锥桶绕行。**

## 文件

| 文件 | SHA256 前 8 位 | 说明 |
| --- | --- | --- |
| `policy_node.py` | `036b4051` | 两个包共用，上传为车上的 `scripts/policy_node.py` |
| `config/traxxas_vehicle_interface.yaml`（仓库） | `6e025315` | 没改 |
| `v050-nolimit-kcost-lidar-avoid-pc/ai4r_policy.yaml` | `ff8fa158` | 黄左 |
| `v050-nolimit-blueleft-kcost-lidar-avoid-pc/ai4r_policy.yaml` | `5ce4cb5a` | 蓝左 |

## 上车要看的三项

1. 控制节拍间隔 p95 和最大值，和周三比（66–77 ms，最长 2.2 s）；
2. `/car/planner_execution` 的 `waiting_cause` 和等待次数，无障碍环线上不应出现 `path_has_no_obstacle_check`；
3. `/car/shadow_planning` 的 `elapsed_s`，平均值不应明显超过 0.42 s。
