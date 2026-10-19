# 下一步计划：消除"一段一段"的走法（5 号车 MPC，`mpc-car-run`）

写于 2026-10-19 晚，基于 `4c7a5de` 之后的状态。读者：接手实施的新 session（可以在云端运行）以及本人。
**只实施改动、跑单元测试和 bag 统计，不跑 Gym**（云端拿不到 `dream-gym`；Gym 和 WSL ROS 门禁由本人在本机跑）。

## 1. 背景

10-19 一整天在 5 号车上跑 lattice 2.1 + MPC（限速 0.20 m/s，`make_deploy_yaml --mode mpc-exec-lattice21 --car 05`）。最后一版（`ea0304c`）已经没有控制层停车，剩下的停车都在估计层。但车走起来是**一段一段的**：在弯道里慢下来、停住，再用起步油门冲起来，冲过软超速线被切油门，滑行，又在下一个弯停住。

全部数据见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 10-19 各条。和本计划直接相关的结论：

| 现象 | 数据（`tools/car05_run_stats.py`，下同） |
| --- | --- |
| 停车/重启**不是**参考切换造成的 | 4 个 bag 共 62 次重启，前 1 s 内有参考切换的只有 5 次 |
| 转弯要更多油门，模型里没有 | 车速 0.12–0.28 m/s、加速度 \|a\| < 0.15 时的油门中位数：直行 0.28–0.30，\|舵\| 0.35–0.6 约 0.31，满舵 0.33–0.35 且多数仍在减速 |
| 起步后冲过头 | 起步 → 跟踪的交接 65 次，其中 48 次（74%）在 2.5 s 内进入软超速/切油门 |
| 车速在 0.05 附近来回切换状态 | 重启中约一半是"车速低于 0.05 回到起步"，而不是卡住判定 |
| 切回 lattice 时转向突变 | 中心线 → lattice：每次转向变化中位数 0.27–0.39（即 QP 允许的单步最大变化），横向误差最多跳 0.44 m。lattice → 中心线已有跳变保护（`switch_hold_s`），这个方向没有 |

运行 `python tools/car05_run_stats.py` 可以复现上表（读 `tests/fixtures/car05_20261019_runs/*.json.gz`，是从 4 个 bag 抽出的逐拍数据）。

## 2. 要做的三项改动

约定（今天所有改动都遵守，继续沿用）：
- 新参数加在 `MPCSettings`（[scripts/policy_node.py](../scripts/policy_node.py)），**默认值等于关闭**，在 `__post_init__` 里加范围检查；
- 基础 [config/ai4r_policy.yaml](../config/ai4r_policy.yaml) 的 `mpc:` 段补上带注释的默认值（`tests/test_mpc.py::ConfigChecks` 要求两者一致）；
- 只在 [config/ai4r_policy_mpc_car05.yaml](../config/ai4r_policy_mpc_car05.yaml) 里打开，注释写清数据来源；
- 新的 `mpc_debug` 字段直接加进 `d`（`MPCController.step` 里的调试字典）；
- `MPCController.step` 新增的关键字参数要有默认值，不能破坏现有调用方和测试替身（有几个替身的 `solve` 不接受 `**kwargs`，所以给 `solve` 的新参数只在功能打开时才传）。

### 2.1 模型加入转弯阻力（解决弯里减速卡住）

- 新参数 `mpc.turn_drive_per_steer`（默认 0 = 关）。含义：舵量为 \|s\| 时，维持车速需要的油门比直行多 `k*|s|`。
- 实现：把它当作随舵量变化的死区偏移，和 `drive_bias` 叠加。
  - `MPCController.step` 里求解前算 `turn = k*abs(self.applied_steer)`，传给求解器时用 `drive_bias=self.bias + turn`（`VehicleModelMPC.effective` 已经负责平移死区、截断到 `drive_max-1e-3`、抬高起步阈值）。
  - `MPCController.predicted_speed`（偏置估计用的单步预测）里，死区也要按当时的指令舵量加上 `k*|steer|`。`command_at(t)` 返回 `(drive, steer)`，用它的 steer。否则偏置会把转弯阻力当成误差再学一遍。
  - `mpc_debug` 新增 `turn_drive`（这一拍加上的偏移量）。
- 车 05 取值：**0.06**（数据：满舵比直行多约 0.05–0.07）。
- 测试（`tests/test_mpc.py`，参照 `HeldWheelReading` 那一组的写法和辅助函数）：
  - 默认 0 时求解器收到的 `drive_bias` 不变；
  - 打开后，舵量 0.8 时求解器收到的 `drive_bias` 比舵量 0 时多 `0.8*k`；
  - 真实求解器（`Solver`）在同一参考下，打开后给出的油门更大；
  - 偏置估计：车按"死区 + k\|s\|"的车模型跑时，`bias` 不会漂到 `k|s|` 那么大（构造一段固定舵量、按含转弯阻力的模型生成的车速序列，喂给 `update_bias`）。

### 2.2 起步后平顺交接 + 状态回差（打断"冲过头 → 切油门 → 又卡住"）

- 新参数：
  - `mpc.rest_speed_mps`（默认 0 = 等于 `moving_speed_mps`，即现状）：跟踪中车速低于它才回到起步。车 05 取 **0.03**，`moving_speed_mps` 保持 0.05。
  - `mpc.handover_s`（默认 0 = 关）和 `mpc.handover_drive_margin`：从起步切到跟踪后的 `handover_s` 内，油门上限 = 有效死区（`deadband + bias + turn`）+ margin。车 05 取 **0.5 s、0.04**。
- 实现位置：`MPCController.step` 里 `if speed_mps >= cfg.moving_speed_mps: ... elif self.phase == "tracking" and breakaway:` 那一段（搜 `too_many_restarts`）。
  - 回差：`elif` 的条件改为 `self.phase == "tracking" and breakaway and speed_mps < rest`，其中 `rest = cfg.rest_speed_mps or cfg.moving_speed_mps`。车速在 `[rest, moving)` 之间时保持原来的状态（跟踪就继续跟踪）。注意第一个 `if` 现在会把 starting 切成 tracking，要保证 `[rest, moving)` 区间里 starting 仍是 starting。
  - 交接限幅：记录 `self.handover_since`（starting → tracking 那一拍的 `clock_s`），求解后、`accept` 之前，若在 `handover_s` 内，`drive = min(drive, d_eff + margin)`，`d_eff` 与 2.1 一致。`reset()` 里清零。`mpc_debug` 新增 `handover`（布尔）。
  - 不能影响：超速 `cut`（仍强制 0）、`plan_hold` 分支、起步阶段的 breakaway 油门。
- 测试：
  - 车速 0.04（介于 0.03 和 0.05）时，跟踪不会回到起步，`restarts` 不增加；默认参数下行为不变（现有 `test_restart_count_resets_after_sustained_tracking` 等测试必须继续通过）；
  - 交接后 0.5 s 内求解器给 0.35 时，实际输出被压到 `d_eff + 0.04`；超过 0.5 s 不再限；`cut` 时仍为 0。

### 2.3 参考来源切换时限制转向变化率（消除切回 lattice 时的那一下）

- 原因：QP 每个预测步允许的转向变化是 `steering_rate_limit_rad_s*dt_pred_s/|gain|` ≈ 1.21×0.1/0.37 ≈ 0.33，切换参考时这一步一下用满。
- 新参数 `mpc.switch_blend_s`（默认 0 = 关）和 `mpc.switch_steer_rate_per_s`：参考来源（lattice/中心线）变化后的 `switch_blend_s` 内，每拍转向变化不超过 `rate*dt`。车 05 取 **0.4 s、1.5 /s**（每拍 0.075）。两个方向都生效；已有的 `switch_hold_s` 跳变保护保持不变。
- 实现：
  - `MPCController.step` 新增关键字参数 `steer_slew=None`（这一拍允许的最大转向变化）。在 `accept()` 里先把 `steer_action` 限制在 `applied_steer ± steer_slew` 以内，再 `record`。这样 MPC 记录的施加历史和实际一致，初始状态回放（`initial_state`）也正确。`plan_hold` 分支同样限幅。`mpc_debug` 新增 `steer_slew`。
  - policy 侧（`calculate_policy_actions` 里调用 `self.mpc_controller.step` 之前，跳变保护旁边）：根据 `planner_traj is not None` 判断这一拍的来源，和上一拍比较。变了就记下时间，在 `switch_blend_s` 内传 `steer_slew=cfg.switch_steer_rate_per_s*dt`。状态在 MPC 控制器新建时（`is_first_policy_step`，和 `self.wheel_hold` 同一处）清零；替身节点没有这些属性时用 `getattr` 兜底（参照 `wheel_hold` 的写法）。
- 测试：
  - `MPCController` 层：给 `steer_slew=0.075`，求解器给出比上一拍大 0.3 的转向时，输出只变 0.075，`applied_steer` 记录的是限幅后的值；
  - 节点层（`tests/test_mpc_planner_control.py`，用 `make_node(...)` 替身，参照 `CarFiveGuards`）：同一 run 里先让参考来源从中心线切到 lattice（`planner=True` 并用 `offset_result` 注入一个偏移的 lattice 结果），切换后 0.4 s 内每拍转向变化 ≤ 0.075 + 1e-9。

## 3. 必须同步修改的地方（今天踩过的坑）

- 测试按名字从 `policy_node.py` 抽取定义：新增的**顶层函数或类**要加进 `tests/test_mpc.py`（`NAMES`）、`tests/test_mpc_planner_control.py`（`EXTRA`）、`tests/test_mpc_real_frames.py`（`NAMES`）；`PolicyNode` 新增的**方法**要加进 `tests/test_mpc_planner_control.py` 的 `METHODS`，以及 `offline/mpc_gym/pipeline_sim.py`、`tools/replay_run_jsonl.py`、`tools/replay_policy_bag.py` 的 `METHODS`。本计划的设计尽量不新增顶层名字和方法。
- `MPCSettings` 的校验：不同部署模式叠加的参数不同（V2 模式 `reference_dropout_tolerance_s` 是 0.3，中心线是 0.5）。新的跨参数约束要对所有模式都成立。改完用下面的命令检查每个模式都能构造 `MPCSettings`：
  ```bash
  python - <<'EOF'
  import runpy, sys; sys.path.insert(0, "tools")
  from policy_offline import load_policy_namespace
  d = runpy.run_path("tools/make_deploy_yaml.py"); ns = load_policy_namespace()
  for car in ("05", "27"):
      for mode in ("mpc-shadow","mpc-exec","mpc-shadow-centerline","mpc-exec-centerline","mpc-shadow-lattice21","mpc-exec-lattice21","mvp"):
          try: p = d["build"](mode, 1.0, cones="yellow-left", car=car)[0]["/**/ai4r_policy"]["ros__parameters"]["mpc"]
          except AssertionError: print(car, mode, "refused by build (exec on 27 is expected)"); continue
          ns["MPCSettings"](**p); print(car, mode, "ok")
  EOF
  ```
- `tests/test_mpc_planner_control.py::test_car05_profiles_turn_on_the_wheel_hold_and_jump_guard` 断言车 05 打开了哪些参数，新参数加进去（车 27 保持关闭）。
- `vehicle.*` 的约束：`drive_deadband <= drive_breakaway <= drive_max`。

## 4. 验证（云端能做的）

```bash
cd Baby-Bus
for t in tests/test_mpc.py tests/test_mpc_planner_control.py tests/test_mpc_real_frames.py tests/test_shadow_lattice.py \
         tests/test_planner_execution.py tests/test_control.py tests/test_estimation.py tests/test_planning.py \
         tests/test_release_contract.py tests/test_lattice.py; do
  echo "$t: $(python -m unittest -q $t 2>&1 | grep -E '^(Ran|OK|FAILED)' | tr '\n' ' ')"
done
python tools/car05_run_stats.py      # 改动前的基线数字，写进日志作对照
```
依赖：Python 3.12，numpy、scipy、osqp、pyyaml（`pytest` 不需要，用 `unittest`）。`test_policy_node.py` 需要 ROS，云端跳过（门禁里会跑）。`offline/mpc_gym/*` 需要 `dream-gym`，云端不跑。

## 5. 交付

1. 代码 + 测试 + 两个配置文件，按 2.1 / 2.2 / 2.3 分成三个提交，或者合成一个提交并在提交说明里分段写。提交说明末尾加 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`。
2. 重新生成部署包（`.verification/` 被 gitignore，只在本地；云端生成了也带不回来，可以跳过，留给本人做）：
   ```bash
   for a in "1.0 60 1m" "5.0 60 5m" "0 0 nolimit"; do set -- $a
     python tools/make_deploy_yaml.py --mode mpc-exec-lattice21 --car 05 --cones yellow-left --distance $1 \
       --run-time-s $2 --v-exec-max 0.2 --out .verification/car05-lattice21-exec-v020-$3; done
   ```
3. [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 顶部加一条：做了什么、新参数和车 05 取值、测试结果、`car05_run_stats.py` 的基线数字，以及"未在 Gym 和实车上验证"。
4. [CAR05_LATTICE21_EXEC_RUN_CN.md](CAR05_LATTICE21_EXEC_RUN_CN.md) 里加一段：新参数的核对命令和期望值、新的 `mpc_debug` 字段（`turn_drive`、`handover`、`steer_slew`）怎么看。部署哈希留给本人本地生成后填。
5. push 到 `origin/mpc-car-run`。

## 6. 本人在本机做（不属于云端任务）

1. `git pull`，按第 5 节第 2 步重新生成部署包，把哈希填进现场清单。
2. Gym 对比（可选）：`python offline/mpc_gym/car05_variants.py`（满电仿真车），`--plant-d0 0.31 --plant-ubreak 0.37`（电量低）；和改动前的提交各跑一次比较。
3. WSL 门禁：`BRANCH=mpc-car-run bash /mnt/e/26b/AI4R/Baby-Bus/.verification/run_ros_check.sh`。
4. 上车，录包，用 `tools/car05_run_stats.py` 的同一套指标和 10-19 对比（先用 `tools/export_car_run.py` 把新 bag 抽成 `.json.gz`，它只需要 `pip install mcap mcap-ros2-support`）。期望：
   - 重启次数和"起步后冲过软超速线"的比例明显下降；
   - 满舵弯道里不再减速到停；
   - 中心线 → lattice 切换时转向每拍变化 ≤ 0.075。

## 7. 不在本计划内

- 弯道里降低目标车速：要先有 2.1，否则低速时更容易卡住。
- 估计层的锥桶连续性检查（别组锥桶误导 `boundary_pairing`）：交估计组。
- 转向零位和变化率的实测：下次上车时在架子上用量角器量。
