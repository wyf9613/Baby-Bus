# PID 基线离线整定

这里保存周二实车接入之前的控制器候选和离线证据。所有脚本均在 Baby Bus 中运行，使用 AI4R Conda 环境安装的 Dream Gym **v0.4.0**，不引用 workshop。此次没有修改 `scripts/policy_node.py` 或执行任何实车操作。

## 运行

从 Baby Bus 根目录执行：

```powershell
conda activate AI4R
python offline/control_pid/test_controller.py
python offline/control_pid/tune.py --tune
# 仅复核已保存的参数，不重新搜索：
python offline/control_pid/tune.py --validate
```

`--tune` 完成搜索后自动验证。脚本直接从课程 Notebook 提取车辆、道路和积分配置，不执行整个 Notebook，也不修改其输出。仿真使用 Dream Gym 本身的 `BicycleModelDynamic`；测试对比其与 Notebook Gym 环境的完整状态演化，允许接口 Float32 转换产生的微小差别。

文件用途：

| 文件 | 用途 |
| --- | --- |
| `controller.py` | 可独立调用的候选控制器，无 ROS 依赖 |
| `notebook_model.py` | 读取 Notebook 中明确列出的配置赋值 |
| `tune.py` | 分阶段搜索、验证、生成指标和图 |
| `test_controller.py` | 转向方向、坐标、抗饱和、坏数据、停车和模型一致性检查 |
| `results/selected_gains.json` | 实际选出的参数，不依赖类的初始默认值 |
| `results/tuning_candidates.csv` | 全部 54 个参数候选及训练分数 |
| `results/validation_metrics.csv` | 验证场景和速度对照实验的指标 |
| `results/*_trace.csv` | 第一组验证 seed 和速度实验的逐步记录 |
| `results/experiment.json` | 模型、版本、输入文件哈希、场景、seed 和参数 |
| `results/summary.md`、`results/validation.png` | 自动汇总与曲线 |

## Planning 输入

### 2026-10-05：直接适配上游 path 接口

控制器现在直接消费 `CenterlinePlanner.plan()` 返回的 reference，不要求 Planning
修改字段、补到 x=0 或将直线强制升阶。支持 1–6 个系数，即常数到五次多项式。
远端 `43c3e96` 当前实际生成一次多项式；六系数五次输入已用合成接口测试验证，
还没有收到规划组的五次实现提交。本次没有修改上游规划算法或 ROS 动作输出。

```python
reference = {
    "timestamp_s": now_s,
    "frame_id": "base_link",
    "vehicle_reference_point": "cg_ground_projection",
    "valid": True,
    "valid_for_s": 0.1,
    "path": {
        "type": "CARTESIAN_Y_OF_X",
        "independent_variable": "x_m",
        "origin": 0.0,
        "scale": 1.0,
        "coeffs_low_to_high": [0.10, 0.03, 0.02, -0.01, 0.002, 0.005],
        "range": [0.5, 2.0],
    },
    "target_speed_mps": 0.3,
    "stop_requested": False,
}
drive, steer, diagnostics = controller.calculate(reference, speed_mps, now_s, dt)
```

定义为 `u=(x-origin)/scale`、`y(x)=Σ a_i*u^i`。`range` 是物理 x 范围，单位米，
不是 u 的范围。导数按实际 x 换算，曲率单位 1/m。系数低次在前，保留所有项。
Frenet d(s)、时间多项式或其他车体参考点需另行适配，不能按 y(x) 偷换解释。

最近点仅在原始范围内搜索，检查距离平方导数的所有实根和两个端点，解决五次曲线
有多个局部最近点时单一 Newton 初值选错的问题。搜索区间先映射至 [-1,1]，求根
阶数最多 9。前方路径允许下界大于 0；若最近点落在端点，误差是该端点切线的有符号
法向分量，属于近场控制近似，不表示车旁盲区已被观测。诊断包含 `closest_x_m`、
`closest_y_m`、`closest_at_range_end` 和 `path_degree`，便于现场检查。

到期判断尊重 `valid_for_s` 和消费者最大年龄，边界时刻即拒绝。新 path 接口要求
同轮生成、同轮消费（几何 timestamp 与 now_s 差不超过 1 μs），不在车辆运动后
把旧固定车体帧当当前帧重用。当前节点本来就是同轮估计/规划；若后续控制改为独立
timer，须另外实现参考坐标随运动的对齐。`stop_requested=True` 优先于正目标速度，
进入已有的仿真停车分支；invalid/过期/错误坐标或坏数据清空积分、返回零动作并标记
停止请求。零动作仍不构成实车制动，ROS 状态机接入及显式恢复仍须在 policy 中实现。

### 兼容的旧离线 mock

```python
reference = {
    "timestamp_s": 0.0,
    "valid": True,
    "path_coeffs": [0.10, 0.05, 0.02, -0.005],
    "x_range_m": [-0.5, 2.0],
    "target_speed_mps": 0.5,
}
```

旧示例的 `y(x) = a0 + a1*x + a2*x² + a3*x³`，也可提供至 a5 的六项系数。坐标原点是车辆重心在地面的投影，x 前、y 左、米；时间戳属于该车体坐标系的时刻，和 `now_s` 使用同一时钟。接口不要求离散点、航向、曲率或控制指令。速度为非负标量，0 表达停止目标。系数绝不能强制过车辆原点。

控制器接受有前向支持的有限区间，不再要求包含 x=0。旧离线 mock 仍给车辆附近约 0.6 m 后向、2 m 前向的道路点拟合路径。兼容接口保留原来的最大年龄检查（0.15 s），新 path 接口额外检查上游期限和当前参考帧。参考每轮重新生成、同轮消费；0.15 s 是旧候选配置，未宣称符合实车频率。

## 控制结构与增益定义

纵向采用速度 PI：`e_v = target_speed - measured_speed`，输出归一化驱动。搜索也比较了带滤波微分的 PID，但 PI 的训练分数在最优 PID 的 2% 范围内，因此选择更简单的 PI。微分作用于测量以避免目标阶跃冲击；积分使用条件积分抗饱和。

横向采用路径最近点误差 PI + 航向 P + 曲率前馈。先在有效区间内求多项式最近点，再求该点切线和曲率。

```text
e_y = 从车到参考路径的有符号法向距离，路径在左为正
e_heading = 路径切线相对车头的角度 - beta_ff
delta_request = delta_ff + Kp_y*e_y + Ki_y*integral(e_y) + Kd_y*d(e_y)/dt
                + Kp_heading*e_heading
steering_action = delta_request / steering_limit_rad
```

曲率 κ 描述**重心路径**：`beta_ff = asin(l_r*κ)`，`delta_ff = atan(L*κ/cos(beta_ff))`。这与直接把重心路径当作后轴路径的公式不同。当前选中的横向微分为 0；航向反馈仍存在，不能把它误解为仅对世界坐标 y 做 PI。

增益对应的误差和单位：

| 参数 | 作用 | 单位 |
| --- | --- | --- |
| `speed_kp` | 速度误差 | normalized / (m/s) |
| `speed_ki` | 速度误差积分 | normalized / m |
| `speed_kd` | 速度测量变化率 | normalized / (m/s²) |
| `lateral_kp` | 路径法向误差 | rad/m |
| `lateral_ki` | 路径法向误差积分 | rad/(m·s) |
| `lateral_kd` | 路径法向误差变化率 | rad/(m/s) |
| `heading_kp` | 航向误差 | rad/rad |

驱动请求限制为 `[-0.3, 0.35]`，转角请求限制为老师模型的 ±45°；模型内部另有 ±90°/s 的转向变化率限制。这些数值不是实车标定。

`target_speed=0` 清除纵向积分并进入负驱动制动/保持。停止状态利用老师模型的前进转倒车锁存，不能直接移植成实车制动承诺。输入无效则清除控制器状态并返回 `(0,0)`；零驱動只代表零请求。离线仿真恢复新参考后继续计算，实车接入仍须保留框架要求的显式恢复行为。

## 搜索与验证边界

纵向 24 组候选在名义模型和驱动增益 0.75 倍、阻力 1.4 倍的模型上测试启动、升速、降速、停止保持和再次启动。评价包括速度 RMSE、各阶跃末段稳态误差、动作变化和停止时间。

横向 27 组候选在课程道路、半径 3 m 的连续左右 S 弯、带 2° 转向偏置的模型上测试；随后比较 3 种积分增益。评价包括是否完成、横向 RMSE、最大偏移和转向变化。搜索范围是粗网格，不保证全局最优。

验证使用独立 seed 101、202、303，覆盖 0.5/1.0/1.2 m/s、正负初始偏移与航向误差、1.5 cm 路径偏移噪声、0.015 rad 航向噪声、0.02 m/s 速度噪声、驱动/阻力/转向偏差、100 ms 命令延迟、100 ms 控制步长及短暂无效参考。无噪声的重复是确定性的，不能当作随机独立试验。

mock Planning 使用真实道路生成三次多项式，控制器只接收 reference 和速度。结果验证的是**给定参考条件下的控制行为**，没有验证锥桶估计、实际规划、障碍感知或完整 ROS 系统。碰撞判据是基于局部切线和车身矩形的车道余量近似，没有做障碍物碰撞实验。

## 后续实车接入

同日接入更新：运行控制已进入 `scripts/policy_node.py`，不导入本目录，采用标准库
有界求根（无需 NumPy/Gym）；新增实际 policy timer 方法的课程模型测试，控制 suite
现在共 20 项。运行版本是前进控制，动作失败或 stop_requested 进入框架状态 2，
不使用下面离线模型的负驱动制动/保持。当前 YAML 选择 MVP：直接计算归一化转向，
目标速度 0.2 m/s，原始轮速累计到 3 m 自动停车并需显式重新启动；30 s 兜底截止。
无需完整 `vehicle.*` 标定。可选 calibrated 模式仍保留测量参数入口，无自动仿真回退。
实际 policy 模型测试还覆盖无标定 MVP、10 Hz/50 ms 延迟相机、正负偏移，运行至 3 m
自动零动作。以下 18 项记录及“后续放进 policy”保留为前一次适配记录。

本次已完成真实 Planning reference 的离线消费，以及估计→规划→控制→课程模型的双向初始偏移直线闭环测试。五次输入另覆盖归一化、端点求值、多个局部极小点、期限、停止和坏数据。执行上述 `test_controller.py` 共 18 项通过。闭环使用每轮当前时刻的合成 cones/轮速/偏航率和 course_simulation 限制，两种初始偏移 ±0.1 m、航向 ±0.04 rad，每种 100 步、dt=0.1 s，末段横向偏移小于 0.03 m、末速在 0.3±0.05 m/s；不包含真实检测器、延迟链路或实车标定。

后续把适配后的控制逻辑放进 `policy_node.py` 的学生代码区域，保留插入标记、传感器健康检查、单一触发源和显式恢复。核对轮速、实际转向映射与极限、制动含义、控制频率及 `required_sensors`。本文参数是模拟条件下的候选起点，实车首次实验应重新确认动作限幅与响应。原 `results/` 保持原样，代表此前 mock 三次路径和旧控制器版本的证据，不是本次适配后的再验证结果；没有重新整定。
