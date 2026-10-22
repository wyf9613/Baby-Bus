# MPC 参考曲率限制（方案 B，只抹钩子）：实现规格

2026-10-22。背景和数据见 [MPC_PROGRESS_LOG.md](MPC_PROGRESS_LOG.md) 的 10-22（再续二）。本文是给实现者（包括云端 agent）的规格，**实现时默认关闭，不改任何部署包**。

## 1. 要解决的问题

1 号车在急右弯里多次转过头、从弯内侧越界。逐帧回放显示：

- 那几秒 MPC 拿到的参考路径在 0.3–0.6 m 内方向变化 110°–300°（"钩子"或折返）。`e_y` 接近 0，`e_psi` 跳到约 ±150°，MPC 一直压着满舵去追，车就转过了头。
- 钩子主要来自 lattice 2.1 选出的路径：它没有车辆模型，`travel` 下打分只看前约 0.25 m，0.4–0.8 m 处的急弯没人管。另一部分来自估计层在弯心处拼出的折返，中心线参考也出现过。
- MPC 自己的转向模型最多 0.37 rad，对应最大曲率 tan(0.37)/0.33 ≈ **1.18 1/m**。参考里比这更急的部分，MPC 本来就跟不上。

**目标**：只在参考里出现明显不可能的急弯时，把它抹平成 MPC 能跟的曲率；其余参考一律不动。

## 2. 已有证据（实现前就知道的）

| 证据 | 结果 |
| --- | --- |
| 形状变化（1,180 条实车参考，从参考自己的起点抹平到 1.18 1/m） | 直道 p95 0.6 cm，普通弯 3.4 cm（前 0.2 m）；急弯前 0.4 m p95 22 cm |
| 安全（前 0.5 m 内离锥桶 < 车半宽 + 3 cm） | 抹平后新增约 1%，都在急弯、偏向外侧 → **必须做碰撞复查**（第 4.4 节） |
| Gym，每组 60 次，"曲率 > 4 1/m 才抹平" | 17° 车：47 → 45；24° 车：38 → 39。**直道和缓弯一次都不触发**（只占 1.4–1.5% 的参考，全在 U 弯和直角弯） |
| Gym，"一律抹平" | 17° 车：47 → 42（R1.5 9 → 6）→ **不要一律抹平** |
| 开环 MPC 回放（实车包） | 钩子拍的满舵占比：rd2 右弯（`r_ddelta` 2）**64% → 36%**；rd5 右弯（`r_ddelta` 5）53% → 52%；左弯 64% → 63% |
| 触发率（实车参考，0.15 m 窗口曲率 > 4 1/m） | 11–35% 的参考，右弯里所有带钩子的参考都被抓到，但约一半触发的参考并不是钩子 |

原型只在 Gym 包装脚本和离线脚本里跑过，仓库代码没动。

## 3. 改哪里

`scripts/policy_node.py`，`PolicyNode.calculate_policy_actions` 的 MPC 分支里，选出 `traj` 之后、`speed = state["speed_mps"] …` 之前：

```python
            if planner_traj is not None:
                traj = planner_traj
            elif mpc_settings.reference_source == "estimation_centerline":
                traj = reference_trajectory_from_road(estimates["road"], state, mpc_settings)
            else:
                traj = reference_trajectory_from_planning(planning_ref, ...)
            # ← 在这里调用新函数：traj, curvature_debug = limit_reference_curvature(traj, estimates["road"], ...)
            speed = state["speed_mps"] if state.get("speed_valid") else None
```

- 新函数写成模块级纯函数（不依赖 ROS、不依赖 `self`），方便单元测试和离线工具复用。
- lattice 路径和中心线参考都要经过它。
- 不改 lattice 2.1、估计层和 MPC 求解器。

参考的数据格式（`reference_trajectory_v0.1`）：`traj["points"]` 是一串 dict，每个点有 `s_m`、`x_m`、`y_m`（base_link）、`yaw_rad`、`curvature_1pm`（现在被截在 ±5）、`target_speed_mps`。函数只改 `points`，其余字段原样保留。

## 4. 行为规格

### 4.1 参数（`MPCSettings`，全部要在 `config/ai4r_policy.yaml` 的 `mpc` 段声明默认值）

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `reference_curvature_limit_mode` | `"off"` | `"off"`：不动（行为逐位不变）。`"hooks"`：只在触发时抹平 |
| `reference_hook_curvature_1pm` | `4.0` | 触发阈值：0.15 m 窗口曲率的最大值超过它才抹平。正常早转约 1.6–2.75 1/m，钩子约 5–11 1/m |
| `reference_hook_window_m` | `0.15` | 算曲率用的弧长窗口。**不要用相邻点的 `yaw_rad`**：点间距 5 cm，太抖，实测会误触发 |
| `reference_hook_trigger_length_m` | `0.8` | 只看参考前这么长的一段来判断触发 |
| `reference_curvature_max_1pm` | `0.0` | 抹平后的曲率上限。0 表示用车辆参数推出：tan(min(\|steering_min_rad\|, \|steering_max_rad\|)) / wheelbase_m（5 号车配置约 1.18） |
| `reference_curvature_clearance_m` | `0.03` | 碰撞复查的余量：抹平后的路径离已知边界不得小于车半宽 + 这个值 |

校验：mode 只能是 `off` / `hooks`；阈值 > 0；窗口在 (0.05, 0.5]；触发长度在 (0, 2]；上限 ≥ 0；余量 ≥ 0。

### 4.2 触发（"稳定判断"）

1. 用参考点的 `x_m, y_m` 做几何计算：对每个起点，找弧长刚超过 `window` 的终点，曲率 = 两端切线方向差 / 弧长（方向差用 `atan2(sin, cos)` 包到 ±π）。
2. 只看前 `trigger_length` 的部分，取最大绝对值。
3. 大于 `reference_hook_curvature_1pm` → 触发；否则原样返回。
4. 点数少于 3，或者参考 `valid` 不为 True、`stop_required` 为 True → 原样返回。

### 4.3 抹平算法（受曲率限制的追踪）

从参考**自己的第一个点**出发，起始方向取参考在起点处的几何切线方向（不是车的方向，否则车本来就有的横向误差会被算成形状变化）。按 `ds = 0.01 m` 前进：

1. 在参考的稠密化点列（间距 ≤ 1 cm）里，从上一步的索引往前找最近点（只往前、窗口有限，保证 O(n)）；
2. 取最近点再往前 `lookahead = 0.25 m` 的点作为目标；
3. 纯追踪曲率 κ = 2·ly / (lx² + ly²)（目标点在当前位置、方向坐标系下的坐标），截到 ±`curvature_max`；
4. 方向 += κ·ds，位置沿新方向前进 ds；
5. 走完参考的总弧长，或者最近点已经到参考末端，就停止。

然后按原参考的点间距重采样：`s_m` 从原第一个点的 `s_m` 起累加；`yaw_rad` 用新路径的切线；`curvature_1pm` 用新路径的曲率；`target_speed_mps` 取原参考在最近 `s_m` 处的值。结果少于 3 个点 → 原样返回。

参考实现见附录 A。它是在 Gym 和离线脚本里跑过的原型，最近点搜索是暴力法，正式实现请改成只往前推进。

### 4.4 碰撞复查（必须有）

- 边界：`estimates["road"]` 里的 `left_boundary_xy` 和 `right_boundary_xy`（base_link，已经对齐到当前时刻）。只有一侧时就只查那一侧。
- 检查范围：抹平后路径的前 `check + preview` 长度。没有现成值时用 `planning.preview_m + 0.15`（约 0.5 m），和 lattice 执行检查覆盖的长度一致。
- 判定：路径点到边界折线的最小距离 < 车半宽（`vehicle.body_width_m / 2`；为 0 时用 `shadow.body_width_m / 2`）+ `clearance` → 复查失败。
- **复查失败：返回原参考**（不抹平），并在诊断里记下 `rejected_clearance`。这样 MPC 跟的永远是一条"至少不比现在更少被检查过"的路径。是否进一步减速，留到实车验证之后再决定，这一版不做。

### 4.5 诊断（只在 mode ≠ off 时加进 `mpc_debug`，off 时 `mpc_debug` 必须和现在逐字段相同）

`reference_curvature`：`{"triggered": bool, "peak_1pm": float, "limit_1pm": float, "applied": bool, "reason": null | "rejected_clearance" | "too_short", "max_shift_m": float}`。其中 `max_shift_m` 是抹平前后，前 0.4 m 内两条路径的最大距离。

## 5. 测试（新文件 `tests/test_reference_curvature_limit.py`，不需要 ROS）

1. 默认 `off`：函数原样返回同一个 `traj`，`mpc_debug` 不含新字段；所有部署配置（`tools/make_deploy_yaml.py` 的各个模式，car 27 / 05 / 01）都是 `off`。
2. 直线参考、R = 2 m 圆弧参考：`hooks` 下都不触发，`points` 逐位不变。
3. 钩子参考（直线 0.3 m 后，0.25 m 内转 150°）：触发；抹平后 0.15 m 窗口曲率 ≤ 上限 + 5%；第一个点的位置不变；起始方向和原参考起点切线的差 < 1°；`s_m` 单调递增。
4. 上限推导：车辆参数 steering ±0.37、轴距 0.33 → 1.18；不对称时取较小的一侧。
5. 碰撞复查：构造一条抹平后会压到外侧边界的钩子（外侧边界离路径 0.12 m）→ 返回原参考，`reason == "rejected_clearance"`。
6. 参数校验：非法的 mode、阈值、窗口都会抛 `ValueError`。
7. 接入 `calculate_policy_actions` 的集成测试：参照 `tests/test_mpc_planner_control.py` 里的 `make_node` / `run` 写法；`hooks` 模式下，lattice 给一条带钩子的路径，`mpc_debug["reference_curvature"]["applied"]` 为 True，MPC 照常求解。
8. 全量：`python -m unittest discover -s tests` 只允许 `test_policy_node`（需要 ROS）报错。

## 6. 验证

### 6.1 云端能做的：Gym（和 10-22 的基线同设置）

```bash
python offline/mpc_gym/car05_variants.py --plant-turn-drag 0.06 --seeds 6 --workers 6 \
    --lattice-planning-s 0.3 --set mpc.solver_time_limit_s=1.0 --plant-steer-rad 0.30 \
    --set mpc.reference_curvature_limit_mode=hooks --json-out hooks_030.json
# 再跑一次 --plant-steer-rad 0.42；基线是同样命令去掉 --set mpc.reference_curvature_limit_mode=hooks
```

通过标准：
- 直道、缓弯完成数和基线相同（基线 17° 车：12 / 12；24° 车：11 / 11）；
- 总完成数和基线相差不超过 ±2（基线：17° 车 47/60，24° 车 38/60）；
- 失败原因仍全部在估计层；
- 报告触发比例（预期约 1–2%，只在 U 弯和直角弯）。

### 6.2 只能在本地做的（车包不在仓库里）

用 `tools/replay_policy_bag.py` 回放 1 号车 10-22 的右弯包（`car-evidence\car01-lattice21-20261022-152447\newpolicy-nolimit-yellowleft-rightbend-*`、`car-evidence\car01-rddelta-20261022\rd5-run2-*`），打开 `--set mpc.reference_curvature_limit_mode=hooks`，统计触发比例、`rejected_clearance` 的比例和 `max_shift_m`。

### 6.3 上车前

1. WSL ROS 门禁：`BRANCH=<分支> bash .verification/run_ros_check.sh`；
2. 出一个打开 `hooks` 的包（只多一行配置），在 `r_ddelta` 2 下的急右弯和原包交替各跑 3–5 次，用 `tools/car_run_metrics.py` 比较越界次数、最多转过的角度和满舵占比。

## 7. 约束

- 从 `mpc-car-run` 拉新分支，比如 `mpc-reference-curvature-limit`，不要直接改 `mpc-car-run`。
- 默认 `off`，`.verification` 里的包、`config/ai4r_policy_mpc_car05.yaml`、`config/ai4r_policy_mpc_car01.yaml` 都不改。
- 文件保持 LF 换行（车上直接用 `policy_node.py`）。
- 车上没有 osqp，MPC 用 `dense` 求解器，这个改动不涉及求解器。
- 写法和周围代码保持一致：注释密度、命名、日期标注（`# 2026-10-22: …`）。

## 附录 A：原型（离线脚本里用过的版本，仅作参考）

```python
def smooth_points(points, kmax, look=0.25, ds=0.01):
    if len(points) < 3:
        return points
    pts = [(p["x_m"], p["y_m"]) for p in points]
    dense = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0]-a[0], b[1]-a[1]); n = max(1, int(L/ds))
        dense += [(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(1, n+1)]
    total = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pts, pts[1:]))
    x, y = dense[0]; th = points[0]["yaw_rad"]   # 正式实现：用起点处的几何切线（和触发判断一致；车包里录下的 /car/policy_trajectory 没有方向角）
    out = [(x, y, th, 0.0)]; j = 0
    for _ in range(int(total/ds)):
        j = min(range(j, min(len(dense), j+60)), key=lambda i: (dense[i][0]-x)**2+(dense[i][1]-y)**2)
        gx, gy = dense[min(len(dense)-1, j+int(look/ds))]
        dx, dy = gx-x, gy-y
        lx = math.cos(th)*dx+math.sin(th)*dy; ly = -math.sin(th)*dx+math.cos(th)*dy
        d2 = lx*lx+ly*ly; k = 0.0 if d2 < 1e-9 else max(-kmax, min(kmax, 2*ly/d2))
        th += k*ds; x += math.cos(th)*ds; y += math.sin(th)*ds; out.append((x, y, th, k))
        if j >= len(dense)-2:
            break
    s0 = points[0]["s_m"]; spacing = (points[-1]["s_m"]-s0)/max(1, len(points)-1) or 0.05
    step = max(1, int(round(spacing/ds))); res = []
    for i in range(0, len(out), step):
        x, y, th, k = out[i]; s = s0+i*ds
        src = min(points, key=lambda p: abs(p["s_m"]-s))
        res.append({**src, "s_m": s, "x_m": x, "y_m": y, "yaw_rad": th, "curvature_1pm": k})
    return res if len(res) >= 3 else points
```

触发用的窗口曲率（离线脚本版）：

```python
def peak_window_curvature(xy, window=0.15, length=0.8):
    segs = [(math.dist(a, b), math.atan2(b[1]-a[1], b[0]-a[0])) for a, b in zip(xy, xy[1:]) if math.dist(a, b) > 1e-6]
    s = [0.0]
    for d, _ in segs:
        s.append(s[-1]+d)
    peak = 0.0
    for i in range(len(segs)):
        if s[i] > length:
            break
        j = i
        while j < len(segs)-1 and s[j+1]-s[i] < window:
            j += 1
        if s[j+1]-s[i] < 0.9*window:
            break
        turn = math.atan2(math.sin(segs[j][1]-segs[i][1]), math.cos(segs[j][1]-segs[i][1]))
        peak = max(peak, abs(turn)/(s[j+1]-s[i]))
    return peak
```
