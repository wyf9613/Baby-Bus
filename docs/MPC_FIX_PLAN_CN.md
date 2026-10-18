# MPC 修复计划（2026-10-15 晚，同日更新）

> 更新：18:32/18:37 V2 在弯前不起步之后，先做中心线版本：新发现的 H 已修（部分），C 做成参数但维持 0.03，5 号车中心线包已生成（[CAR05_CENTERLINE_RUN_CN.md](CAR05_CENTERLINE_RUN_CN.md)）。V2 不改。

分支 `mpc-car-run`。依据：今天 5 号车的三次实车运行（`car-evidence/car05-v2-startfix-exec-20261015-*`）和中心线 Gym
矩阵（[`offline/mpc_gym/results/car_readiness_centerline.md`](../offline/mpc_gym/results/car_readiness_centerline.md)）。
每天的结果记在 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md)，5 号车的现场步骤见
[CAR05_V2_START_FIX_RUN_CN.md](CAR05_V2_START_FIX_RUN_CN.md) 和 [CAR05_MPC_RUN_CN.md](CAR05_MPC_RUN_CN.md)。

## 现在能跑什么

| 道路 | V2 + MPC | 中心线 + MPC | 实车 |
| --- | --- | --- | --- |
| 直道 | Gym 10/10 | Gym 3–10/10（H 修后） | V2 在 5 号车上跑完 0.5 m（17:53）；18:18 两次超速停车；中心线待测 |
| 缓弯 R5 m | Gym 10/10 | Gym 1–5/10 | 没跑 |
| 直角弯 / U 弯 | 不支持；弯前 2 m 内起步就报 `recovery_body_boundary_conflict`（18:32、18:37） | Gym 0/10（D、E）；弯前起步能出参考线（重放） | 不跑 |
| 小环线 | 没测 | 没测 | 不跑 |

## 问题清单

| # | 问题 | 根因 | 修法 | 状态 |
| --- | --- | --- | --- | --- |
| A | 5 号车起步失败（17:06） | 规划短暂等待时，起步油门被旧规划序列拉低，1.5 s 起步超时 | 固定 0.35 起步、等待期间保持油门；等待时也检查超速和起步超时 | ✅ `0dcd075`，17:53 实车验证 |
| B | 交接后车速 0.18 → 0.06 m/s（17:53）；0.32 下两次超速（18:18） | 死区在 0.289 和 0.32 之间；车动时 QP 的油门下限是死区 + 偏置，MPC 没法松油门 | `drive_deadband` 0.30；10-18 加软超速（超过 0.20 可以把油门降到死区以下，超过 0.23 置 0），只在持续 1 s 或超过 0.40 时锁停 | 🟡 `fc6e18e` + 10-18 软超速，待实车验证 |
| C | 中心线参考线只到车前 0.1 m（`reference_too_short`） | 估计层 `_ordered_road` 配对两侧边界时，对侧弧长回退容差只有 3 cm，锥桶噪声下偶尔一个点配不上，中心线就断开，代码选离车最近的一小段 | 做成参数 `estimation.ordered_pairing_backtrack_m`，**维持 0.03**：0.10 会在 5 号车真实帧上把误识别的锥配成 0.41 m 的短路；实车重放里断链不到 1% | ✅ `d155a80`（参数化）；Gym 的 2 cm 噪声下仍然会断，Gym 偏悲观 |
| D | 急弯超速（`overspeed`） | QP 里速度只是代价项，转向打满时 MPC 靠提速消除横向误差 | 治本：QP 里加软的速度上限约束；不改 `q_v`（实测全局加大会让直道变差）。10-18 的软超速是兜底：Gym 里直角弯和 U 弯的 `overspeed` 基本消失，只有 r1.5 m 0.72 车道还有 8/10 | 🟡 兜底已做，QP 约束 ⏳ |
| E | C、D 修完后急弯仍 0/10 | 估计层在弯里报 `invalid_ordered_geometry`、`insufficient_geometry` | 先诊断 | ⏳ |
| F | 5 号车横向一直偏 6 cm、不收敛 | 转向是借 .20 的（满舵 18°），5 号车区间 ±1.0，增益可能不对；也可能只是车速太低 | 现场短弧标定（`CAR05_MPC_RUN_CN.md` 第 1 步） | ⏳ 现场做 |
| H | 中心线方案下，车居中却算出横向 12 cm、航向 0.19 rad 的偏差（18:18 重放） | 中心线从车前 0.58 m 开始，适配器把第一段 5 cm 往回延长到车身；最近一个锥放偏造成的倾斜被放大 | 沿二次拟合在起点处的切线往回加引入段（`mpc.ordered_lead_in_fit_m` 1.0） | 🟡 `baac940`：直道、各种弯都准；那一帧降到 8 cm、0.135 rad，剩下的是估计层的问题 |
| I | MVP 能跑的地方 MPC 被自己的门禁停下 | `reference_too_short`（MVP 只要 0.35 m 预瞄点）、`solver_fail` 立即锁停、dropout 0.3 s、起步超时 1.5 s、距离 1 m | 10-18：末端外推 0.3 m、求解失败可容忍、dropout 0.5 s、起步超时 2.5 s、重启 5 次（tracking 10 s 清零）、距离和时间可以设 0。Gym 直道 3–10 → 9–10/10，缓弯 5/1 → 9–10/7 | 🟡 待实车（[CAR05_RELAXED_EXEC_RUN_CN.md](CAR05_RELAXED_EXEC_RUN_CN.md)）；跑完的直道里有部分横向误差变大（最多 12 cm），原因待查 |
| G | V2 规划时效紧 | 异步 V2 频繁 `planning_pending`，偶有 `lattice_time_budget_exceeded`、`source_deadline_during_planning` | 目前都被 `plan_hold` 桥接住，没有锁停；先监控，跑到 3 m 如果出现锁停再处理 | 👀 |

## 步骤

**当前顺序（10-18 更新）**：
1. 中心线放宽版实车，只跑 exec：1 m → 5 m → 不限，按 [CAR05_RELAXED_EXEC_RUN_CN.md](CAR05_RELAXED_EXEC_RUN_CN.md) 做。这一步顺带验证 B 和 I。
   （10-15 的旧顺序是先 shadow 再 exec，见 [CAR05_CENTERLINE_RUN_CN.md](CAR05_CENTERLINE_RUN_CN.md)。）
2. F 转向标定。
3. D 速度约束。
4. E 急弯估计诊断，同时诊断 H 剩下的部分：最近锥放偏的情况，在估计层处理。

下面是原计划，留作参考：


每一步单独提交，跑完离线测试和 WSL ROS 门禁（`BRANCH=mpc-car-run bash .verification/run_ros_check.sh`）再往下走。

1. **B 实车验证**（现场，约 10 分钟）：V2 跑 1 m，用 `.verification/car05-v2-db032-exec`。
   - 通过：交接后车速稳在 0.12 m/s 附近，不超过 0.20。
   - 车速冲过 0.20：0.32 给高了，退回 0.30 再试。
   - 还是掉到 0.06 以下：说明死区更高，用这次的包重新估，不靠猜。
2. **F 转向标定**（现场，约 30 分钟）：在 1 m 稳定之后做，结果写进 `config/ai4r_policy_mpc_car05.yaml` 的转向参数。
   然后 V2 跑 3 m，看横向偏差能不能收敛。
3. **C 配对容差**（约半小时）：
   - 代码：`scripts/policy_node.py` `_ordered_road` 的 `other_s >= previous_other_s-0.03`，0.03 改为新参数
     `estimation.ordered_pairing_backtrack_m`；`config/ai4r_policy_mpc_centerline.yaml` 设 0.10。
   - 测试：单点回退 7 cm 的合成锥桶链，默认值下会断、0.10 下不断；main 的 U 弯 hairpin 场景在 0.10 下仍不会配到出口。
   - 目标：中心线 Gym 直道 ≥ 9/10、缓弯 0.72 m ≥ 9/10；V2 的 `car_readiness.py` 基线逐字节不变。
   - 通知估计组：main 的 PID 预瞄只看 0.35 m，可能也被截短了，只是没表现出来。
4. **中心线实车 shadow**（现场）：C 修完后重新生成 `car05-cl-*` 包，和 V2 在同一段直道上各推一遍，比较 `shadow` 比例。
5. **D 速度约束**（1–2 小时）：
   - 代码：`VehicleModelMPC.solve` 的压缩 QP 里，每一步加 `v_k = base_k[3] + G_k[3]·U ≤ v_cap + 余量`，带松弛变量，
     OSQP 和 dense 都要支持。
   - 测试：转向饱和时预测速度不超过上限；现有 102 个 MPC 测试保持全过。
   - 目标：中心线 Gym 急弯不再出现 `overspeed`，直道和缓弯不变差。
6. **E 急弯估计诊断**（约 1 小时）：在直角弯失败帧逐点看锥桶链和配对（方法同 C 的诊断），用 `tests/fixtures/car05_*`
   交叉检查，先确认是 Gym 噪声模型的问题还是实车也会出现。之后再定修法。

## 不在这次计划里

- U 弯：main 的 PID 在实车上也过不了，需要入弯策略或状态机。
- 小环线：Gym 里没有环线道路，需要另外造一条；连续行驶超过 5 m 的情况也还没测过。
- 5 号车的完整纵向辨识（电机增益、阻力、刹车）：现在只有起步油门和死区是 5 号车自己的，其余仍借 .27。

## 已知限制（写报告时要说明）

- 5 号车的车辆参数只有起步油门（0.35）和死区（0.32，一次运行估计）是实测的，转向和其余纵向参数都借自 .20、.27。
  `vehicle.source` 里写明了这一点。
- Gym 对中心线路线可能偏悲观：main 的 PID 用同一套估计和规划在实车上过了直角弯，但 Gym 里同样的估计在 2 cm 噪声下
  会断链（C）。
