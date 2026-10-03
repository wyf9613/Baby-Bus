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

```python
reference = {
    "timestamp_s": 0.0,
    "valid": True,
    "path_coeffs": [0.10, 0.05, 0.02, -0.005],
    "x_range_m": [-0.5, 2.0],
    "target_speed_mps": 0.5,
}
```

`y(x) = a0 + a1*x + a2*x² + a3*x³`，系数按常数项到三次项排列。坐标原点是车辆重心在地面的投影，x 前、y 左、米；时间戳属于该车体坐标系的时刻，和 `now_s` 使用同一时钟。接口不要求离散点、航向、曲率或控制指令。速度为非负标量，0 表达停止目标。系数绝不能强制过车辆原点。

控制器接受包含 x=0 的有效区间。离线 mock 给车辆附近约 0.6 m 后向、2 m 前向的道路点拟合路径，支持最近点误差计算；实际 Planning 不必凭空补后方数据，但需要明确近车范围的有效性。如果只能提供 `[0, 2]`，最近点可能被约束在端点，真实接入前应验证这一接口差异。参考每轮重新生成、同轮消费，默认超过 0.15 s 或来自未来会被拒绝；0.15 s 是候选配置，未宣称符合实车频率。

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

先让真实 Planning 的五字段数据接入同样的离线测试，再把控制逻辑放进 `policy_node.py` 的学生代码区域，保留插入标记、传感器健康检查、单一触发源和显式恢复。核对轮速、实际转向映射与极限、制动含义、控制频率及 `required_sensors`。本文参数是模拟条件下的候选起点，实车首次实验应重新确认动作限幅与响应。
