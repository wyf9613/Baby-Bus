# V1 直线中心线规划器

同日控制接入更新：`scripts/policy_node.py` 已消费本模块的参考并计算限幅后的
归一化动作，支持一次至五次 y(x)，没有改变本 V1 的直线规划算法。YAML 选择
20 Hz timer，由估计器每轮把缓存道路对齐到当前状态。当前 `control.mode=mvp`
使 planner 使用 `mvp=True`：保留道路、反馈、坐标、时间、覆盖和直线域检查，
跳过车辆轮廓/制动模型检查，目标速度 0.2 m/s。不宣称具备车辆模型验证。
原始轮速累计达到 3 m、30 s 运行截止、无效或过期参考均通过状态 2 锁存停止。
显式恢复才可重置里程。下文车辆参数要求仅针对可选 calibrated 模式。
以下原始 V1 说明中的“动作继续为零/控制未接入”描述规划提交时的状态；当前
控制实现和测试边界见 policy 注释、YAML 和 acceptance.md。

实现依据：外层 Materials/PLANNING_MODULE_TECHNICAL_DESIGN.md 的 V1 与通用参考接口。
所有运行算法保留在 scripts/policy_node.py；配置在现有 YAML 的 planning 节。
2026-10-05 完善后已通过 16 项规划与 28 项估计离线测试及 100 帧合成回放。
ROS gate 因当前 Windows 缺少 Bash/ROS Jazzy/colcon 未能启动，未做控制闭环或实车实验。

## 输入、算法与输出

`CenterlinePlanner.plan(road, state, obstacles, vehicle_limits, now_s, source_timeout_s=None, mvp=False)`
直接读取当前估计字典，返回 `(reference, diagnostics)`。不做候选搜索、障碍响应、
Frenet 回中曲线或旧路径复用。仅支持双侧边界、受限偏移/航向的近直线低速场景。
单侧输入明确失败，后续版本再扩展已知宽度推断。

算法依次检查有效性、各字段年龄、坐标/时间、曲率、有限有序几何与采样间隔。
在共同覆盖范围内用均值中心化最小二乘拟合 `y=a0+a1*x`，检查最大拟合残差，
保留 a0 的真实偏移；不强制生成三次多项式。当前上游只有采样点，V1 消费这些点。

利用以 CG 地面投影为原点的车身轮廓包围矩形，按路径切向放置车身，缩短有效
路径范围以保持四角均有边界支持，并在分段线性边界的相关断点检查车身余量。
当前车身检查使用最近有观测支持的道路横截面，受 `max_near_x_m` 限定；这是直线
近场的显式工程近似，不代表车辆原点附近未观测区域已被确认，也不是回中扫掠验证。

速度取巡航设置、车辆上限和可靠长度停车包络的最小值；以当前实际速度检查
`v*delay + v²/(2*braking)`，不以降低目标速度掩盖当前不可停车情况。
停车距离计算采用保守有效路径长度、前悬投影与余量，模型依赖提供的车辆限制。
本版没有速度时域轨迹，控制器需周期更新并处理参考到期、制动和停稳。

结果保存为 `self.planning_output`，诊断为 `self.planning_diagnostics`；没有新增 topic。
参考包含版本/ID、CG 参考点、几何时刻、生成时刻、期限、路径、目标速度、
停止语义与源年龄。路径编码为 `CARTESIAN_Y_OF_X`，`x_m` 为自变量，
`origin=0`、`scale=1`、低次在前系数 `[a0,a1]` 和实际支持范围。
`evaluate_planning_path(reference, x_m, now_s)` 是控制/绘图可共用的求值入口，
返回位置、航向、零曲率；无效、到期或超范围返回 None，不外推。
期限内跨周期消费仍需把车辆状态对齐至参考固定帧，求值函数不替消费者做运动补偿。

正常为 TRACK / valid=True。失败为 INVALID_INPUT / valid=False、path=None、
target_speed_mps=0、stop_requested=True，reason 与 stop_reason 记录原因。
stop 请求本身不会制动车辆，恢复也不能绕过框架显式状态 3 请求。
框架状态变更清空输出，首步重新建立规划器；现有动作继续为零驱动、零转向与 pan hold。

## 时间与参数前提

2026-10-05 已整合估计组提交 144ab8a：估计器使用轮速/陀螺仪历史将有效道路
对齐至 state.timestamp_s，并保留 measurement_timestamp_s 与 source_age_s。
规划器消费已对齐几何时不会重复补偿；默认关闭的是规划器自身的备用补偿。
几何仍不在当前时刻时返回 motion_alignment_required。
可显式设置 `planning.compensate_constant_twist=true`，在有限年龄内利用当前滤波
速度/偏航率做短时 SE(2) 匀速转弯补偿。它假定区间内运动恒定且侧滑可忽略，
不是 EKF/实测里程计。道路补偿后几何 timestamp 改为当前时刻，但源年龄不刷新；
期限取配置期限及道路、轮速、陀螺仪剩余期限中的最小值，同时尊重框架/估计期限。

默认 `planning.vehicle_limits_source=upstream`。vehicle_limits 必须包含 valid=True、明确 source、
vehicle_reference_point='cg_ground_projection'、footprint_xy_m、speed_max_mps、
braking_deceleration_mps2、actuation_delay_s、safety_margin_m。
当前估计器仍输出未标定限制，因此 calibrated 模式的规划输出会保持 INVALID_INPUT；
该模式需要标定组提供限制后才能得到可消费参考，不能把测试轮廓当实车数据。
当前 YAML 的 MVP 跳过这些模型检查，给出 0.2 m/s 目标，不声明已标定车辆能力。

离线可显式选择 `planning.vehicle_limits_source=course_simulation`，无需覆盖或修改
估计器的未标定限制。几何引用课程笔记：轴距 0.33 m、前伸 0.297 m、后伸 0.198 m、
宽 0.25 m。最高速度 0.5 m/s、制动减速度 0.5 m/s²、延迟 0.1 s、余量 0.05 m
是独立的 `simulation_*` 工程假设，不是笔记的实车测量值。参考与诊断标注
simulation_only、参数来源；不得将此模式用于实车资格声明。动作依然为零。

修正了近端观测与车身支持范围裁剪混淆的问题：先验证原始近端观测覆盖，再裁剪
参考范围以容纳车身。边界求值使用二分查找，减少长点集的重复扫描。
输出期限同时受估计器 motion_max_gap_s、源年龄和框架超时约束。

## 测试方法与结果

1. 离线执行 `python -B tests/test_planning.py -v`。
   用例覆盖左右偏移与系数、有限范围、失效/过期/坐标/单侧/车辆限制、
   弯曲/采样间隔/覆盖/余量、补偿开关、剩余有效期、实际速度停车不可行、
   输入不被修改、轨迹 ID，以及求值器的尺度、范围和到期规则。
   车辆限制均标注为 offline_test_assumption，测试加载实际单文件算法。
2. 执行 `python -B tests/test_estimation.py -v` 检查估计回归和实际学生入口；
   该入口继续返回零动作，同时计算规划参考。可添加真实记录回放及边界极限场景。
3. 在 Ubuntu 24.04 / ROS Jazzy 按 CONTRIBUTING.md 使用固定接口依赖执行
   `AI4R_INTERFACES_SOURCE=... bash tools/verify_fast.sh`。
   CMake 已注册规划用例；ROS suite 应验证配置加载、安装源码、状态重置与传感器期限。
4. 再接控制器做限定直线场景模型闭环，记录路径、偏移/航向、速度、年龄、到期与停车原因，
   最后验证真实标定与受控实车。离线结果不构成实车通过证据。

本地 Python 结果：16/16 规划测试、28/28 估计测试通过。
另执行 `python -B tools/study_planning.py`：5 个种子各 20 帧，直线中心偏移变化，
3 mm 横向噪声，50 ms 延迟，100/100 帧有效。平均逐帧路径 RMSE 约 0.000922 m，
最大约 0.002202 m；最终源码本机规划耗时中位数/p95/最大约 0.496/0.658/1.367 ms。
该回放不包含控制器、DDS、真实硬件，也不包含道路估计计算的耗时。
脚本输出完整参考、配置、诊断和源码哈希，便于复现。
本地日志与报告在忽略的 `.verification/planning-tests.log`、
`.verification/estimation-tests.log`、`.verification/planning-study.json`。
