# 最近两轮 MVP2 完整拒绝和时序证据

范围：drive-startup-speed-floor020-01 / -02；仅分析已有日志，没有改代码或再次驱动车辆。

## 字段含义与记录缺口

- result_timestamp_s 是 worker 原始 reference.timestamp_s，即本次规划输入状态的参考时刻，不是计算完成时刻。
- 原始 reference.valid_for_s 未直接打印。对已成功选路但被 reference_expired 丢弃的结果，可按部署代码与该条日志的源年龄还原：min(0.2, 0.4-road_age, 0.25-wheel_age, 0.25-gyro_age)。这两轮没有障碍检查，所列记录均非 degraded，没有额外障碍或预测期限。
- worker 自己返回无效结果时，原始 ref.valid_for_s 按代码为 0。表中另列输入允许时间，不能把它误称为失败结果的实际有效期。
- 主进程准确接收/控制 poll 时刻、worker 开始/完成时刻均未记录。JSON 中这些字段为 null。
- test_recorder_error_log_receipt_unix_s 是测试记录器收到 ROS 错误日志的墙钟时间；不是规划结果被主进程接收的时刻，也不能把它减 timestamp_s 后全部归因于 IPC。
- 外层 road_source_age_s 属于处理结果时的当前道路；分析该任务应使用 planning_diagnostics.source_ages_s.road / worker_road.source_age_s。
- source_deadline_during_planning 被复用：既表示 worker 内输入有效期耗尽，也表示主进程 transport_control_reference 抛 reference_expired。需结合 discarded_result 区分。

## 全部观测到的拒绝类型

- `invalid_estimates_or_boundaries`：6 条。
- `source_deadline_during_planning`：27 条。
- 上述 source deadline 中，`reference_expired` 丢弃：23 条；worker 内输入期限耗尽：4 条。
- 初始道路状态 `missing_motion_history` 对应 6 条 invalid_estimates_or_boundaries。
- `blind_correction_limit` 是部分候选的拒绝原因，共 18 次；同次任务仍存在可行候选，不能视为整次规划无解。
- `near_field_unobserved` 是道路状态，本轮 controlled_near_extension=true 允许规划，并非最终停止原因。
- `planning_pending` 是等待任务状态，不计为完成任务错误。
- 两轮最终完整停止文本均为：`Publishing zero actions: Control startup history deadline exceeded; last planning reason: source_deadline_during_planning; explicit resume required`。

## 示例与全部 worker 内超时记录

### drive-startup-speed-floor020-01，trajectory_id=4

```json
{
  "reason": "source_deadline_during_planning",
  "discarded_result": "reference_expired",
  "worker_reference_timestamp_s_recorded": 1791856106.1659575,
  "worker_reference_valid_for_s_reconstructed": 0.09000125200000003,
  "worker_reference_expires_at_s_reconstructed": 1791856106.2559588,
  "parent_receive_time_s_recorded": null,
  "control_poll_time_s_recorded": null,
  "test_recorder_error_log_receipt_unix_s": 1791856106.28628,
  "worker_input_road_source_age_s": 0.309998748,
  "worker_input_source_ages_s": {
    "road": 0.309998748,
    "wheel_speed": 0.051560118999987026,
    "imu_angular_velocity": 0.104261536
  },
  "worker_input_remaining_s_reconstructed": {
    "road": 0.09000125200000003,
    "wheel_speed": 0.19843988100001297,
    "imu_angular_velocity": 0.145738464
  },
  "input_lifetime_s_reconstructed": 0.09000125200000003,
  "limiting_source_reconstructed": "road",
  "planner_elapsed_s_recorded": 0.04456322700025339,
  "speed_target": {
    "preview_mps": 0.005596266324708258,
    "floor_mps": 0.2,
    "command_mps": 0.2,
    "measured_mps": 0.0
  }
}
```

### drive-startup-speed-floor020-02，trajectory_id=5

```json
{
  "reason": "source_deadline_during_planning",
  "discarded_result": "reference_expired",
  "worker_reference_timestamp_s_recorded": 1791856142.1201916,
  "worker_reference_valid_for_s_reconstructed": 0.14073558800000002,
  "worker_reference_expires_at_s_reconstructed": 1791856142.2609272,
  "parent_receive_time_s_recorded": null,
  "control_poll_time_s_recorded": null,
  "test_recorder_error_log_receipt_unix_s": 1791856142.3315287,
  "worker_input_road_source_age_s": 0.259264412,
  "worker_input_source_ages_s": {
    "road": 0.259264412,
    "wheel_speed": 0.01913411100031226,
    "imu_angular_velocity": 0.037647383
  },
  "worker_input_remaining_s_reconstructed": {
    "road": 0.14073558800000002,
    "wheel_speed": 0.23086588899968774,
    "imu_angular_velocity": 0.212352617
  },
  "input_lifetime_s_reconstructed": 0.14073558800000002,
  "limiting_source_reconstructed": "road",
  "planner_elapsed_s_recorded": 0.04535353400024178,
  "speed_target": {
    "preview_mps": 0.005597414745926933,
    "floor_mps": 0.2,
    "command_mps": 0.2,
    "measured_mps": 0.0
  }
}
```

### drive-startup-speed-floor020-01，trajectory_id=10

```json
{
  "reason": "source_deadline_during_planning",
  "discarded_result": null,
  "worker_reference_timestamp_s_recorded": 1791856106.9184058,
  "worker_reference_valid_for_s_reconstructed": 0.0,
  "worker_reference_expires_at_s_reconstructed": null,
  "parent_receive_time_s_recorded": null,
  "control_poll_time_s_recorded": null,
  "test_recorder_error_log_receipt_unix_s": 1791856107.0694923,
  "worker_input_road_source_age_s": 0.256724287,
  "worker_input_source_ages_s": {
    "road": 0.256724287,
    "wheel_speed": 0.007779188999847975,
    "imu_angular_velocity": 0.233780349
  },
  "worker_input_remaining_s_reconstructed": {
    "road": 0.14327571300000003,
    "wheel_speed": 0.24222081100015203,
    "imu_angular_velocity": 0.016219651
  },
  "input_lifetime_s_reconstructed": 0.016219651,
  "limiting_source_reconstructed": "imu_angular_velocity",
  "planner_elapsed_s_recorded": 0.050152810999861686,
  "speed_target": null
}
```

### drive-startup-speed-floor020-01，trajectory_id=13

```json
{
  "reason": "source_deadline_during_planning",
  "discarded_result": null,
  "worker_reference_timestamp_s_recorded": 1791856107.292018,
  "worker_reference_valid_for_s_reconstructed": 0.0,
  "worker_reference_expires_at_s_reconstructed": null,
  "parent_receive_time_s_recorded": null,
  "control_poll_time_s_recorded": null,
  "test_recorder_error_log_receipt_unix_s": 1791856107.396583,
  "worker_input_road_source_age_s": 0.328602891,
  "worker_input_source_ages_s": {
    "road": 0.328602891,
    "wheel_speed": 0.00742103399898042,
    "imu_angular_velocity": 0.205648446
  },
  "worker_input_remaining_s_reconstructed": {
    "road": 0.07139710900000001,
    "wheel_speed": 0.24257896600101958,
    "imu_angular_velocity": 0.04435155399999999
  },
  "input_lifetime_s_reconstructed": 0.04435155399999999,
  "limiting_source_reconstructed": "imu_angular_velocity",
  "planner_elapsed_s_recorded": 0.04590365399963048,
  "speed_target": null
}
```

### drive-startup-speed-floor020-02，trajectory_id=16

```json
{
  "reason": "source_deadline_during_planning",
  "discarded_result": null,
  "worker_reference_timestamp_s_recorded": 1791856143.6165473,
  "worker_reference_valid_for_s_reconstructed": 0.0,
  "worker_reference_expires_at_s_reconstructed": null,
  "parent_receive_time_s_recorded": null,
  "control_poll_time_s_recorded": null,
  "test_recorder_error_log_receipt_unix_s": 1791856143.741329,
  "worker_input_road_source_age_s": 0.357479107,
  "worker_input_source_ages_s": {
    "road": 0.357479107,
    "wheel_speed": 0.0074337389996799175,
    "imu_angular_velocity": 0.097227663
  },
  "worker_input_remaining_s_reconstructed": {
    "road": 0.042520893000000004,
    "wheel_speed": 0.24256626100032008,
    "imu_angular_velocity": 0.15277233699999998
  },
  "input_lifetime_s_reconstructed": 0.042520893000000004,
  "limiting_source_reconstructed": "road",
  "planner_elapsed_s_recorded": 0.0634645090012782,
  "speed_target": null
}
```

### drive-startup-speed-floor020-02，trajectory_id=22

```json
{
  "reason": "source_deadline_during_planning",
  "discarded_result": null,
  "worker_reference_timestamp_s_recorded": 1791856144.4035766,
  "worker_reference_valid_for_s_reconstructed": 0.0,
  "worker_reference_expires_at_s_reconstructed": null,
  "parent_receive_time_s_recorded": null,
  "control_poll_time_s_recorded": null,
  "test_recorder_error_log_receipt_unix_s": 1791856144.5699291,
  "worker_input_road_source_age_s": 0.338984962,
  "worker_input_source_ages_s": {
    "road": 0.338984962,
    "wheel_speed": 0.0658577680005692,
    "imu_angular_velocity": 0.100511813
  },
  "worker_input_remaining_s_reconstructed": {
    "road": 0.06101503800000002,
    "wheel_speed": 0.1841422319994308,
    "imu_angular_velocity": 0.149488187
  },
  "input_lifetime_s_reconstructed": 0.06101503800000002,
  "limiting_source_reconstructed": "road",
  "planner_elapsed_s_recorded": 0.06210177799948724,
  "speed_target": null
}
```

## 可支持的结论

输入剩余时间不足和结果取用时已过期都实际发生。例：第一轮 #10 的 IMU 年龄为 233.780349 ms，250 ms 门限只留 16.219651 ms，规划花 50.152811 ms，worker 内即失败。第二轮 #16 的道路年龄 357.479107 ms，400 ms 门限只留 42.520893 ms，规划花 63.464509 ms，同样在 worker 内失败。

而第一轮 #4 已成功选路，目标 0.2 m/s，输入允许 90.001252 ms，规划耗时 44.563227 ms；主进程明确以 reference_expired 丢弃。第二轮 #5 也已成功选路，输入允许 140.735588 ms、规划耗时 45.353534 ms，仍被 reference_expired 丢弃。这证明不能把所有错误解释为 planner 算法运行超过 100 ms。

时间戳起点到记录器收到错误日志分别约 120.322466 / 211.337090 ms；其中还包含输入估计、任务提交、进程通信、排队、主线程调度及 ROS 日志交付，缺少准确分段时间，尚不能量化各段或断言控制刷新是唯一瓶颈。

下一轮若要分段定位，需要只增日志字段：reference.timestamp_s/valid_for_s、t_submit、t_worker_start/done、t_pipe_receive、t_control_poll、t_control_use，以及时钟种类；当前未实施、未重新测试。

完整逐条数据：TIMING_DETAILS.json（含原始拒绝对象和各源年龄）；TIMING_DETAILS.csv（便于对比）。计数仅覆盖已打印拒绝记录，不是所有成功/失败搜索的总体统计。
