# Lattice / 基线控制来源开关

代码默认保持基线；按用户要求，当前分支 config/ai4r_policy.yaml 已切换为 planner：

```yaml
planner_control:
  enabled: true    # false 基线 + 同步影子规划；true PID 跟踪 lattice，异常退回基线
shadow:
  enabled: true
  rate_hz: 1.0
```

修改同一 YAML 中已有字段，不要创建第二份同名配置。参数启动时读取；修改后构建、
重启 policy。ros2 param set 不适用。启动依旧是零动作，需要原流程显式 Enable / Run。
开关不改变规划采样、评分、控制增益、转向限制、巡航速度或感知准入条件。

## 执行流程

20 Hz 的估计和基线规划一直运行。基线模式直接走原控制分支；独立 worker 继续规划。
planner 模式拿到 VALID 结果前使用基线。每个任务携带原始车体时刻、累计运动位姿和
坐标代次；结果返回后不会把到达时刻冒充源时刻。

planner 模式在控制前以轮速 / 车体系 gyro 的短区间增量更新位姿，将选中轨迹变换到
当前车体系，沿有序弧长裁掉已驶过部分，保留未来负 x 回折段。生成当前时刻的
ORDERED_CENTERLINE reference，交给同一个 PolicyController 做原有预瞄、误差滤波、
限幅和转向变化率处理。正常切换不清空 PID 状态。

源轨迹没有固定 0.2 s 或其他固定秒数寿命；运动对齐、剩余覆盖和滚动检查可用时，
可以跨多个规划周期使用，source_age_s 始终保留。每次重建的执行 reference 有效期为
control.max_dt_s + 一个控制周期，当前为 0.25 s。这是执行调度看门狗，不是 1 Hz 源
轨迹寿命；原有控制 dt / 反馈看门狗仍独立生效。

剩余轨迹需覆盖预瞄距离及一个控制周期预计前进距离。每周期重新检查当前车体、到
最近轨迹点的几何连接、轨迹近段，覆盖实际预瞄位置及已有 PID 切向平滑窗口（预瞄
后最多 0.15 m，按剩余轨迹截断）。不检查整个 1 m 尾段。只使用当前同坐标时刻的
observed / observed_with_history 边界，无额外净空；推断单侧边界不作为硬墙。

这是选中轨迹的几何姿态采样，不是逐候选控制器预测，不加入新评分项。车体尺寸和
数值扫掠精度沿用 shadow 配置，仍是近似。没有标定转向动力学或验证实际削峰后的
车体扫掠，几何通过不等于物理避碰证明。

下列情况同周期回到当前基线参考，不新增零动作状态、降速或锁存：

- 等待首条结果、NO_ROAD / ALL_COLLIDE / worker 或传输失败。
- 运动坐标代次变化、对齐不可用或执行参考异常。
- 预瞄覆盖耗尽、当前确定边界滚动检查碰撞。
- planner 参考被控制器拒绝：恢复调用前 PID 状态，改用基线再计算。

新结果有效、执行条件恢复时自动回到 planner。显式 Stop 清缓存，旧跑次结果不能
进入新跑次。基线也无效、传感器过期、动作断流或操作者 Stop 时保留原有停车机制；
不能保证道路与反馈全部丢失时仍不停驶。基线降级沿用原流程，不声称基线另有 lattice
碰撞检查。

## 显示与记录

蓝线 /car/shadow_trajectory 保留规划输入时刻。绿线 /car/policy_trajectory 表示实际
采用的参考：基线或已对齐裁剪的 planner 轨迹。两条线坐标时刻不同，运动时不应
期待完全重叠。

新增 /car/planner_execution（std_msgs/msg/String JSON），跟随实际动作发布，通常
20 Hz。mode 是 planner / baseline，requested_source 是开关选择；fallback、reason
记录降级；停止时 mode=idle。还有 job_id、原始年龄、运动位姿 / 代次、弧长进度、剩余覆盖、检查长度、
当前确定边界、剩余路径、控制预瞄及转向。模式或原因变化时打印 Reference selection。
诊断传输故障不影响动作。shadow_planning 中 shadow_only=false 只表示请求接管，实际
是否使用 planner 始终以 planner_execution 为准。历史快照新增每轮速度 / yaw 状态及
frame_id，便于后续精确回放。

本次没有改变候选评分或修复此前发现的大转向 / 参考方向冲突；这些仍需闭环实验评估。

## 更新与小直道录制

本机修改开关后，沿用已有流程让 policy 处于零动作，再传输 scripts 和 config。
确认目标车地址；下例沿用 10.43.254.20，此处没有执行部署。

```powershell
scp -r "C:\Users\23733\Desktop\AI4R\project\workspace\Baby-Bus\scripts" "C:\Users\23733\Desktop\AI4R\project\workspace\Baby-Bus\config" ai4r@10.43.254.20:~/ai4r_student_workspace/src/ai4r_policy/
```

车端原 ROS 环境：

```bash
dream build ros student && dream runtime restart ai4r_policy
ros2 param get /car/ai4r_policy planner_control.enabled
ros2 param get /car/ai4r_policy shadow.rate_hz
```

先开始录制，再在另一个终端按已验证流程 Enable / Run：

```bash
RUN=~/planner_straight_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN"
ros2 param dump /car/ai4r_policy > "$RUN/policy_parameters.yaml"
cp ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py "$RUN/policy_node.py"
cp ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml "$RUN/ai4r_policy.yaml"
ros2 bag record -s mcap -o "$RUN/bag" \
  /car/cone_detections /car/imu/data /car/wheel_speed_m_per_sec \
  /car/drive_and_steer_set_point_normalized /car/drive_and_steer_applied_normalized \
  /car/policy_trajectory /car/shadow_trajectory /car/shadow_checked_prefix \
  /car/shadow_reference /car/shadow_planning /car/planner_execution \
  /car/policy_fsm_state_value /car/policy_fsm_state_string \
  /car/traxxas_state /car/request /car/policy_fsm_transition_request /tf /tf_static
```

结束时先停止 policy，再在录制终端 Ctrl+C，等待 bag 写完。仍在同一终端执行：

```bash
dream runtime logs ai4r_policy --tail 500 > "$RUN/policy_runtime.log"
printf '%s\n' "$RUN"
ros2 bag info "$RUN/bag"
```

保留整个 RUN 目录，包括源码、参数、日志和 bag。复制使用打印出的真实目录名，不要
使用“实际时间”占位文字。先录 enabled=false，再改 true、重启后录另一组，可比较
同一小直路的转向、实际控制来源、时延、覆盖和降级。无需 Python 录制脚本。
