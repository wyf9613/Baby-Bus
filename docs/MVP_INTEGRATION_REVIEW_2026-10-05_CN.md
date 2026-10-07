# 10 月 6 日最小实车验证：分支审阅与接入缺口

## 当前 MVP 与老师要求

已按本地老师文档重新简化：运行代码全部在 `scripts/policy_node.py`，配置仍在
`config/ai4r_policy.yaml`，保留插入标记与老师框架的动作发布、状态机和传感器超时。
`control.mode: mvp` 无需完整车辆/制动/转角标定，速度 PI + 路径误差及航向误差直接
生成归一化动作。目标速度 **0.2 m/s**，drive 上限 **0.15**，转向绝对值上限 **0.5**。
当前上游 V1 仍生成直线，控制接口可接至五次 y(x)，未擅自替换上游规划算法。

每次显式请求 policy 状态 3 时，将本次累计里程置零；原始轮速按单调时间梯形积分。
达到 **3.0 m** 后立即请求框架状态 2，持续发零 drive/steer，保持 pan，不自动恢复。
策略执行前、发布前和独立监督 timer 均检查距离；重复状态 3 请求不能重置正在运行
的里程。停止后重新显式请求状态 3 才是新一轮。另有 **30 s** 运行截止，防止新鲜轮速
持续错误报零时无限运行。debug1 为路径误差（m），debug2 为本次累计里程（m）。
3 m 是轮速估计距离；编码器比例、打滑、采样误差和滑行会影响实际最终停车位置。

单一 20 Hz timer；需要 cones、wheel_speed、imu_angular_velocity。规划继续检查
双侧边界、近直线、小偏移、近端/远端覆盖、时间与参考有效期；不做避障。
传感器缺失/过期、道路无效、参考过期或超出运行限制会锁存停止。可选 `calibrated`
模式的测量入口保留，当前 MVP 不把该入口作为运行先决条件。

老师文档依据（PDF 页码从 1 开始）：

- 《(2) Clone and edit your policy》p1–3：本地修改单文件 policy/YAML，不要求学生
  电脑安装完整 ROS；框架负责发布和状态控制。p5：drive 是电机 effort，不是目标速度，
  五项动作返回、归一化有限值、首轮 dt=0、pan=None 保持。p7：必要数据缺失/过期
  输出零，恢复需要明确请求。
- 《DREAM Car Setup & Development Reference》p3：将代码复制/同步到车上、构建 student
  overlay、重启 policy，再用 Foxglove 检查。
- 《Technical Requirements》p10：先实现简单 baseline，再完成集成；完整车辆辨识
  不是老师要求的首轮 MVP 前置条件。我们原先把 calibrated profile 的限制当成了
  MVP 必须条件，现已改为上述显式模式。

明天最小流程：把这两个更新文件同步到车上的 student checkout，运行
`dream build ros student`，再 `dream runtime restart ai4r_policy`。在 Foxglove 确认
状态 2 与零动作，确认双侧直线锥桶、轮速和 gyro 数据正常；车端显式 Enable 后再
请求 policy 状态 3。观察 debug2 达到 3 m 后状态 2/零动作，以及实际停车位置。
按老师要求 RC 就近可接管，不无人值守。小幅转向方向若相反，修改
`control.mvp_steering_direction` 为 -1.0 并重启 policy；驱动是否克服死区需现场确认。

离线与模型验证已通过，当前完整 ROS gate 因 WSL 缺少固定接口依赖/ROS Jazzy 而
未运行，真实车端构建和动作验证仍待现场完成。保留的 offline PID 增益和结果文件未改。

## 初次审阅历史（不代表当前部署要求）

以下记录保留最初的 calibrated 接入分析；当前以本文上方 MVP 和 acceptance.md 为准。

后续更新（同日）：控制侧离线适配已经完成，下文接口拒绝探针记录的是修改前状态。
`offline/control_pid/controller.py` 现在直接接受上游嵌套 path，支持常数至五次多项式、
origin/scale、前方有效范围、参考期限和 stop_requested；没有改变上游规划算法。
新增模型内联合闭环和接口回归后，控制器测试 18/18 通过。原增益、模型及结果文件未变。
当前远端仍是一次直线规划，五次输入用合成契约验证，待规划组提供实际版本再联测。
实车限制注入、控制进入 ROS policy、真实动作映射和完整 ROS gate 仍待完成。
详细新契约与测试边界见 [offline PID 说明](../offline/control_pid/README.md)。

审阅日期：2026-10-05，Australia/Sydney。
远端基线为 `origin/feature/state-road-estimation` 的 `43c3e96`；本地在该分支上
cherry-pick offline PID 提交 `3ce7b34`，得到 `3d04eb2`。原 `control-pid-baseline`
分支保留。`git diff control-pid-baseline HEAD -- offline/control_pid` 无差异，
代码、选定增益和已有结果均原样保留；没有重新整定或生成结果。

## 初次审阅时的链路

`DREAM 检测器/轮速/IMU → EstimationPipeline → CenterlinePlanner → [待接控制] → 动作`

- 上游检测器提供锥桶位置、颜色、置信度；本分支做状态过滤、鲁棒道路估计和时间补偿，
  不包含新的图像检测算法。约定蓝左黄右，现场需要确认颜色、坐标与安装参数。
- 状态和道路保存于 `self.estimation_output`；规划参考和拒绝原因保存于
  `self.planning_output`、`self.planning_diagnostics`，目前没有专门的 ROS 输出 topic。
- V1 规划仅支持双侧边界、近直线、小偏移和小航向误差；不处理障碍物。
- `calculate_policy_actions()` 仍返回零 drive、零 steering、pan hold。
  offline PID 尚未参与 ROS 动作计算，当前分支没有形成驾驶闭环。

## 初次审阅提出的接入缺口

| 缺口 | 代码中的实际行为 | 下一步 |
| --- | --- | --- |
| 实车限制未注入 | 估计器每轮创建 `source=unmeasured / valid=False` 的车辆限制；默认规划拒绝 | 增加有来源的实车参数接入，提供 CG 坐标车身轮廓、速度上限、制动能力、执行延迟和余量；单改现有 YAML 不能提供这些字段 |
| 参考结构不同 | 规划为 `path.coeffs_low_to_high=[a0,a1]`、`path.range`；PID 要顶层四系数及 `x_range_m` | 设计适配，保留系数真实偏移、坐标、参考时刻、源年龄、期限和停止语义 |
| 有效范围不同 | 规划为车身边界支持裁剪后，近端通常大于 0；PID 要求范围包含 0 | 明确控制误差定义并验证前方有限路径；不能把范围改成包含 0、凭空外推未观测道路 |
| 参考到期不同 | 规划期限至多 0.1 s，可能被源剩余期限进一步缩短；PID 仅按默认 0.15 s 检查年龄 | 消费者必须尊重 `valid_for_s`，跨周期参考还需要坐标时间对齐 |
| 停止与恢复 | PID 的负驱动停车/保持依赖仿真模型，输入无效时仅返回零请求；框架要求显式恢复 | 接入实车停止语义，规划/控制失效进入停止状态，恢复不能自动重用旧积分或旧参考 |
| 反馈健康要求 | 当前 YAML 的 `required_sensors` 只有 cones，估计内部另有速度/偏航率检查 | 闭环模式至少要求 cones、wheel_speed、imu_angular_velocity；不要用缺失反馈的零值代替 |
| 转向与驱动映射 | PID 按仿真 ±45° 和归一化驱动整定；实车接口还缩放转向并限幅 PWM | 核对方向、死区、极限、动作至转角/轮速的响应，再选现场低速限幅 |
| 现场诊断 | 内部规划字典不能直接用 `ros2 topic echo` 查看，debug 默认为空 | 提供有限频率的 valid/reason、源年龄、范围、目标速度和控制误差记录；需要能区分“感知失败”和“参数未提供” |

接口探针使用实际 planner 和保留的 Controller：有效直线路径 `[0.15, 2.8]`，
直接消费和仅扁平化/补零系数后消费均得到 `(0, 0, valid=False)`。
同一几何换成未测量限制，规划返回 `vehicle_limits_unavailable_or_reference_mismatch`。
这说明只搬入 PID 文件并不能建立闭环。

## 初次审阅拟定的验证顺序（已被上方 MVP 流程替代）

1. **先验证静止数据链路。** 使用匹配的 DREAM underlay / student overlay，检查
   固定消息版本、传感器频率、轮速静止时仍发布、偏航率符号、蓝左黄右、TF 和锥桶距离。
   观察启动后的运动历史覆盖及估计拒绝原因；等待新数据不会替代显式 policy 启动。
2. **验证零动作下的估计与规划。** 先分清道路 invalid 和车辆限制缺失；提供实车限制后，
   在双侧直线走廊中确认有效范围、偏移、目标速度、源年龄与参考期限。
   未标定时可以验证估计，并记录规划的预期拒绝；不要切换 course_simulation 冒充实车参数。
3. **完成模型内联合验证与 ROS gate。** 用实际规划输出驱动适配后的控制器，覆盖
   正负偏移、前方范围、参考失效/到期、传感器断流、停止和显式恢复。所有运行策略仍放在
   `scripts/policy_node.py`，保留插入标记、单一触发源、watchdog 和 pan hold。
4. **验证实车动作响应，再做短距直线闭环。** 现场确认转向方向和小幅驱动/停车响应；
   随后使用双方边界可见、无障碍的直线低速场景，记录路径误差、轮速、请求动作、
   valid/reason 和停止恢复过程。当前 V1 不适合作为弯道或避障演示。

## 本次验证证据与限制

- `python -B tests/test_estimation.py -v`：28/28 通过。
- `python -B tests/test_planning.py -v`：16/16 通过。
- `conda run -n AI4R python -B offline/control_pid/test_controller.py`：9/9 通过，
  包含与 Notebook Gym 的完整模型状态对照；Gym 发出 Box 上下界相等的 warning，测试通过。
- `tools/study_planning.py`：100/100 合成帧有效，平均逐帧路径 RMSE
  0.000922 m，最大 0.002202 m。显式采用 course_simulation 限制，不含控制闭环或真实传感器。
- 接口探针位于忽略目录 `.verification/review_pid_handoff.py`，没有修改运行算法。
- 当前 WSL Ubuntu 已存在，但未找到 `/opt/ros/jazzy/setup.bash` 或 colcon，且本地缺少
  固定 `dream_interfaces` checkout。尝试 CONTRIBUTING fast gate 时在依赖路径检查处失败；
  未运行完整 ROS gate。所需接口 commit：`5f50902ccee44e8370d6e2be85607b3054ffbf98`。
- 没有连接/启动车辆、修改实时配置、执行实车测试或推送远端。

初次审阅完成拉取、PID 保留与审阅。当时控制接入、参数辨识和联合验证仍是后续工作，
不能把离线算法测试通过解释为明天可以直接开始自动驾驶。
