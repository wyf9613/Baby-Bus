# Planner 离线模拟环境

从 `docs/ad_gym_system_project_v2026_09_18.ipynb` 提取车辆、道路、虚拟传感器、
初始状态、RK4 和终止配置，使用 **Dream Gym v0.4.0** 的原始动力学与传感器后端。
外层将观测转换为 Baby-Bus 输入，并调用 `scripts/policy_node.py` 中的实际估计器。
本目录提供环境及验证用的命令序列；lattice planner、评分模型、policy 和路径跟踪选择
留给后续接入。本环境不参与 ROS 安装，也不启动实车组件。

## 安装与运行

Python 3.11–3.13，推荐 3.12。在独立 Python 环境中，从 Baby-Bus 根目录执行：

```powershell
python -m pip install -r offline/planner_sim/requirements.txt
python -B tests/test_planner_sim.py -v
python -B -m offline.planner_sim --steps 20 --drive 0.1 --steering 0.0 --output .verification/planner-sim-demo.json
```

requirements 固定 notebook 对应的源码提交
`9e9d031fabf5c97b32428925e56bb6a13de713f3`，不安装 RL、MPC 或图形窗口 extras。
构造环境时检查 Dream Gym 版本；旧版 0.2.2 无法提供本 notebook 的接口。

本次创建的项目内验证解释器为 `.verification/planner-sim-venv/Scripts/python.exe`，
可将上述命令中的 `python` 替换为此路径。源码缓存和验证解释器都被 git 忽略。

```python
from offline.planner_sim import PlannerSimulationEnv, SensorTiming, load_notebook

config = load_notebook()  # 默认含 notebook 的两个矩形障碍
# 可深复制/修改 config 的道路、车辆、噪声或初始状态，用于后续场景调查。
env = PlannerSimulationEnv(config, max_steps=1000)
observation, info = env.reset(seed=0)

# 动作由调用方提供；顺序为归一化 [drive, steering]。
observation, reward, terminated, truncated, info = env.step([0.1, 0.0])
# 同样接受 calculate_policy_actions 的五项返回，pan 只支持 None/hold。
observation, reward, terminated, truncated, info = env.step((0.1, 0.0, None, None, None))
env.close()
```

drive 是电机作用请求，steering 按 notebook 的 ±45° 请求上限转换；动作必须有限且
位于 [-1,1]。路径参考需要由调用方的跟踪控制器转换成动作后再送入 `step()`。
零 drive 是滑行，不能解释为即时停稳。保留原模型的 forward-to-reverse direction latch。

## 观测接口

`reset()` 返回 `(observation, info)`；`step()` 返回
`(observation, reward, terminated, truncated, info)`。observation 是丰富的 Python
字典，不是固定大小的 Gym observation space；后续模型可自行编码/填充。

| observation 字段 | 内容 |
|---|---|
| `timestamp_s`, `elapsed_time_s`, `dt_s`, `frame_id` | 参考时刻、已模拟时长、步长和 `base_link` |
| `sensors.cone_detections` | `detections=[(x,y,z,color,confidence), ...]` 与采集到发布延迟 |
| `sensors.wheel_speed` | 无符号速度，m/s；当前为 COM 前向速度的虚拟遥测代理 |
| `sensors.imu_angular_velocity` | `(0,0,yaw_rate)`，rad/s |
| `sensors.lidar_scan` | 原扫描及角度/量程元数据 |
| `sensors.lidar_cartesian` | 车体点集及对应 `scan_indices` |
| `sensor_age_s`, `sensor_stamp_ns`, `sensor_available` | 每个输入的年龄、采样 stamp 和可用性 |
| `estimation` | 现有估计器的 `state/road/obstacles/alignment/vehicle_params/vehicle_limits` |

速度、偏航率与锥桶坐标必须使用未缩放的 SI 单位；gyro 每步只接受一个末端采样，
多速率通过 SensorTiming 表达。配置日志记录实际安装库版本/源码哈希，并分别记录
notebook 所固定的预期依赖提交，便于辨认同版本本地改动。

锥桶去掉固定容量 padding；Gym yellow=0/blue=1 转换为 DREAM yellow=1/blue=2。
原虚拟检测器没有 confidence，本适配器的默认 1.0 是合成常量，可用
`synthetic_cone_confidence` 调整，不代表实车检测概率。z=0 是平面模拟点。
仿真 COM 的平面原点映射到 CG 地面投影；安装误差、车辆俯仰和物理相机模型未模拟。

LiDAR 只检测 notebook 的矩形障碍，不检测锥桶、道路边缘或车身。61 束在
[-90°,90°] 上，index 30 向前。原仿真最大量程 4 m 也是无返回 sentinel；适配器将其
转为原 scan 的 `inf`，排除出 Cartesian 点集，防止制造 4 m 假障碍墙。
恰在量程上限或噪声饱和到上限的 hit 与无返回无法区分，也会被排除。

状态/道路来自实际估计代码，不由道路真值直接填充；仍有该代码的局部 y(x)、
时间补偿和未标定参数限制。`vehicle_params/vehicle_limits` 不被伪造为实车标定。
配置日志提供真实仿真模型参数，未来 planner 可显式选择其模拟车辆限制。
ArUco、IMU orientation/specific force 未模拟，字段为 None。

## 多速率与延迟

默认所有传感器每一步采样、即时交付，沿用 notebook。可显式加入异步采样：

```python
timing = SensorTiming(
    period_steps={"cone_detections": 2, "wheel_speed": 2},
    delay_steps={"cone_detections": 1},
    timeout_s=0.5,
)
env = PlannerSimulationEnv(sensor_timing=timing)
```

dt=0.05 s 时，这是 10 Hz cones/轮速、50 ms cone 延迟。其余流每步采样。
流名为 `cone_detections/wheel_speed/imu_angular_velocity/lidar`；LiDAR raw/Cartesian
同步交付。保持原测量 stamp 和年龄；达到 timeout 的数据置 None。
轮速遵循实车无 header 的接收时间代理。估计器重复消费缓存时不重复更新滤波，
道路/雷达用运动历史补偿至当前参考时刻；延迟启动时可能暂时 invalid。
虚拟扫描瞬时完成，没有逐束运动畸变。时钟从 1 s 开始，以满足现有代码的非零 stamp。

## 评估、reward 与同状态分支

`info["evaluation"]` 包含车辆真值、道路进度、步内/累计障碍接触和 `progress_delta_m`；
这些是评估/监督标签信息，不应作为评分模型输入。`reset()` 的 info 另外记录全部解析后
配置、notebook/估计源码 SHA-256 和依赖版本/提交。

notebook 原始环境 reward 每步为 **0.0**，本环境默认保持这一行为。后续确定标签目标
后可以传入无内部状态的 `reward_function(evaluation)`，例如：

```python
def experiment_reward(evaluation):
    return evaluation["progress_delta_m"] - 100.0 * evaluation["collision"]

env = PlannerSimulationEnv(reward_function=experiment_reward)
```

这只是接口示例，不是已选定的训练 reward。`terminate_on_collision=True` 在接触时
结束，`max_steps` 到达时 truncated；原 notebook 的宽松速度/横向误差限制仍保留。
结束后须 reset，环境不会继续执行动作；终止标记不代表车辆已物理停稳。

```python
env.reset(seed=0)
checkpoint = env.snapshot()
branch = env.fork()
# 后续在分支上执行某候选对应的控制动作序列。
branch.step([0.1, 0.2])
branch.close()
env.restore(checkpoint)
```

快照复制动力学隐藏 direction latch、RNG、障碍状态、传感器队列及估计历史，保留
Dream Gym v0.4.0 的内部 sentinel 身份并重建只读映射。仅复制 `car.state` 不够。
快照是内存对象；外部控制器的积分状态需由调用方另行复制。reward callable 应保持
无内部状态。固定噪声种子和 reset seed 具有不同作用；若需要共享 reset 随机流，
设 `config.observation_config["fixed_noise_seed"] = None`。

仿真观测是几何传感器模型，不执行真实图像神经网络。参数及模型一致性验证只建立
课程模拟条件下的证据；实车探测、标定与运行效果由实车实验确认。
