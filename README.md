# Baby Bus · ai4r_policy

2026-10-14 起步接入进度改为软偏好：保留按运动递减的接入距离，优先搜索早接入，
但不再把它作为候选最大距离。短接入触发曲率数学域或碰撞拒绝时仍可尝试更长连接；
16 候选上限内保留远端兜底，前段回中排序仍优先，延后接入增加软代价。
`startup_join_space.preferred_join_x_m` 与 `selected_beyond_preference` 显示选择情况。

减少主线程重复计算：没有旧点需要融合时直接复用已通过验证的当前帧道路；有效
整段拟合仅丢弃内部异常点时不重复启动分段拟合，失败或端部覆盖不足仍可分段恢复。
测距错误记录实际 dt、前后时刻和门限，全部受控停车都输出 `Control timing`，含
`distance_check`、`execution_check` 和上轮 `policy_duration_s`。确认控制间隔同时
超限时明确报告 `Control execution gap exceeded`。0.2 s 控制期限和停车规则保留；
本日志不能证明实际阻塞发生在哪一步，因此没有宣称调度超时已在实车消失。
8 项定向检查通过，并用日志中两条成功轨迹的道路输入做了搜索复查；尚未实车验证。
以下硬接入上限描述保留为历史记录，以本段为准。

2026-10-14 已修复冷启动拟合入口：连续道路模式的首帧也允许分段拟合，不再因
整段左侧拟合失败且没有历史缓存而提前退出。历史融合及观测冲突检查统一按实际
边界表示求值，兼容分段结果，消除 `KeyError: coeffs`。远端某窗口失败时可保留
连续且覆盖合格的近端前缀，不跨越失败窗口或增加未观测道路。受控起步条件保留。

本次新日志已能规划并执行轨迹 8–12，最终原因为 `Warmstart motion feedback
unavailable`，没有碰撞记录；原日志无法确定是哪路反馈或具体失效条件。当前把
内部运动最大间隙由 0.25 s 对齐到既有轮速/陀螺仪源期限 0.5 s，仍逐周期检查，
真实源超时、缺失反馈和无效历史仍停车。拟合 `fit_details` 记录具体拒绝条件、
数值和失败时最多 64 个输入点；`Control timing` 增加当前状态、停车时两路实际
运动年龄及原始观测年龄。轨迹日志补齐 `near_centering` 与两个试验开关字段。
10 项定向测试及一组冷启动到实际配置规划器串联检查通过；完整 ROS gate 受本机
Git Bash 启动错误阻断，实车未验证。以下旧门限描述保留为历史记录。

2026-10-14 当前启用 `immediate_centering_enabled: true`，关闭普通回中原先的
25% / 2 cm 步长限制，保留完整回中与保持偏移候选。过渡长度改为 0.25 / 0.6 m，
优先搜索短过渡；受控起步仍从真实原点/朝向/当前曲率连接道路，不跳过静止起步和
三帧完整双侧支持交接。规划模型 `steering_limits_enabled: false` 关闭转角及
转向变化率拒绝（含输出转换和恢复规划），实际控制输出限幅保持。

已通过几何/运动检查的候选，优先比较 0.05 / 0.1 / 0.2 / 0.3 m 处实际道路中心
误差，权重 4 / 3 / 2 / 1；相对当前朝向直行继续偏离的部分额外惩罚。超过 1 cm
需要纠偏时，近段代价先于总分和旧轨迹一致性排序，转向/一致性软权重降至原值的
10%；接近中心后恢复原有软评分。摄像头只看到更远道路时改比最近有支持的前缀，
不会伪造盲区中心线。日志 `planning_details.near_centering` 给出检查点误差、
相对直行的回中进展和是否有近场支持；这是规划预测，不是车辆实测回中量。
碰撞、数学有效性、侧向加速度及盲区检查、warmstart 缓存续用保持。
按用户要求仅做 7 项定向离线检查，全部通过；完整 ROS 与实车验证未完成。
以下记录保留为之前各次修改的历史说明。

2026-10-14 当前启用 `planning.lattice.startup_progress_enabled: true`：受控起步
在实际道路中心有支持的范围内计算回中代价，不再把沿起步连接行驶统一记为零。
优先搜索较早的可行接入点；控制接受的连接距离随连续运动历史中的行驶距离递减，
后续规划不能重新推到 0.9 m。连接仍受真实道路起点和采样分辨率限制，无法提前接入
时继续跟踪已有剩余缓存，不放宽转向/碰撞约束。三帧完整双侧车身支持交接保持。

同时修复起步运动同步死锁：主线程持续补偿固定起步走廊，较旧 worker 结果不能
回滚已验证运动；每个任务均可接收当前时刻、同一初始走廊的历史补偿状态，而非
只在 worker 初始化时恢复。真实运动缺失仍拒绝，单次规划间隔 >250 ms 不再必然
造成永久 `startup_motion_history_gap`。新增日志 `startup_motion_sync`、
`center_reference`、`road_center_cost`、`remaining_join_distance_m` 和
`remaining_startup_join_m`。本地 298 项回归通过；尚未实车验证。

2026-10-14 修复 warmstart 已走过姿态误停：负 x 缓存点继续用于控制插值，但不再
作为待执行 CG 姿态做整车碰撞检查。每周期先独立检查真实当前车身 `(0,0,0)`，
再检查插值后的轨迹原点与前方剩余路径；障碍检查使用相同姿态范围。即使相机源帧
没变，也重新检查当前车身。日志 `check_pose_type` 区分 `current_body`、
`path_origin`、`future_path`，并记录忽略的已消耗点数。
已有剩余缓存时，规划失败/无解/pending/超时/worker 故障保留缓存并继续重规划；
原点起步、实际当前/前方碰撞、反馈失效和路径耗尽的处理保留。本地 293 项回归通过，
其中包含按实车日志数值重现的约 7.4 mm 旧车尾误判。尚未实车验证。

2026-10-14 已补齐 `Warmstart collision:` 日志：`stage=candidate/selected` 区分
新候选拒绝和最终选用轨迹冲突；记录轨迹编号、路径距离、预测车身位姿、左右侧、
带符号净空/越界量、车身接触点、当前帧边界坐标及附近采样、边界来源与测量时间。
最终 `Control timing:` 同步保存本周期碰撞记录和后台日志状态，零输出先于停车日志。
后台不同日志级别使用独立调用位置，异常计数/最后异常可见，首次错误写入 stderr，
避免静默丢失轨迹日志。这次只增加诊断，没有放宽碰撞检查或改变起步/续行逻辑。

2026-10-14 当前启用 `control.warmstart_enabled: true`：首次有效观测轨迹通过
受控起步与控制检查后，保存剩余路径。新规划失败、pending、worker 故障或道路过期时，
继续用新鲜轮速/陀螺仪逐周期补偿并跟踪旧路径，同时后台重规划；原 1 s / 0.25 m
缓存上限在此模式下不生效。不会缓存执行器指令，也不延长几何终点。剩余弧长按模型
限制速度，保留 0.08 m 末端余量，路径耗尽、运动反馈失效或有效新观测确认碰撞仍停车。
新轨迹先通过时间/坐标、接入连续性、剩余停车距离与当前边界检查，再替换缓存。
日志提供 `remaining_path_length_m`、`cached_trajectory_id`、`replan_accepted`。

规划前瞻现为 1.0 m（允许实际支持更短的起步轨迹），新规划道路年龄放宽至 1.5 s；
首次控制成功前仍保留 0.6 s 道路年龄、原点起步连接和 3 s 零输出等待门限。
起步状态随本次运行保存，worker 重启保留固定起步走廊和交接状态；显式停车/重启
清除缓存。20 Hz 控制、0.25 s 运动反馈门限、0.2 s 控制期限及 30 s 运行上限不变。
轨迹/拒绝/控制状态大日志改为有界后台输出，积压时丢弃旧诊断；worker 回收也在后台。
本地估计/规划/控制共 287 项回归通过，包含真实 worker 子进程的失败及重启恢复；
完整 ROS 检查和实车验证尚未完成。以下按日期记录的旧配置说明保留为历史证据。

2026-10-14 已完成急弯单侧重建修改 1–3：新增 `reconstruction_failure`，区分
真实边界顺序/宽度异常、中心线法向偏移奇点/折返、推断边界奇点/折返和共同覆盖不足。
两侧各自拟合成功但共同范围不足或组合宽度不合格时，使用未过期的学习路宽尝试
单侧回退，按覆盖、拟合质量、与上一帧中心线的连续性选择，并对原侧保留 10% 容差。
`single_side_fallback` 记录两侧候选和选择；回退不学习新路宽，两侧真实拟合都保留
为独立碰撞约束。中心线和推断边界独立诊断；远端推断失败时可保留有效近端前缀，
规划只使用中心线与两侧边界的共同支持范围。仅中心线有效仍不能行驶。
没有改为弧长 XY 表示；急弯超出当前 y(x) 表示或共同范围不足时仍会拒绝。
本地估计/规划/控制回归共 275 项通过；完整 ROS 检查和实车验证尚未完成。

2026-10-14 已修复异步时间戳与过期交接：实际提交时配对 ROS/单调时钟，
worker 用提交时刻计算完成时间，保留原始输入坐标帧时间，避免估计耗时把有效期算短。
控制入口和监督回调先非阻塞接收结果；旧控制参考过期时，仅对当前代次、未过期的
有效结果且控制间隔未超限的情况执行一次正常校验/控制刷新，失败仍按原逻辑停车。
不会延长旧动作或取消反馈/保持期限；停车和显式重启清除暂存结果。
主线程记录轨迹前先运动补偿并提交下一次任务。`Planned trajectory` 的 `timing`
记录提交、worker 排队/计算、结果传输及接收后等待；`Control timing:` 在相关停车时
记录最后控制接受时间、实际控制截止时间、暂存轨迹 ID 和待完成任务年龄。
尚未上车验证。

2026-10-14 当前启用小幅 MVP 回中：`incremental_centering_enabled: true`，
`centering_step_max_m: 0.02`。正常 lattice 跟踪把原来的直接回中目标替换为
缩小当前横向偏差 25%（最多 2 cm）的末端目标，并保留当前偏移候选。
所有几何/运动检查通过后优先选择减小偏差的候选；原有评分在同组内排序，
回中候选不通过或输出失败仍可用保持偏移轨迹。候选数量不增加，起步连接不受影响。
速度、0.6 m 规划距离、转向变化率限制和评分权重保持原值。
日志 `planning_details.centering` 记录初始偏差、请求目标和选中目标；其中
`selected_progress_m` 是目标偏差缩小量，不是实车已经移动的距离。尚未实车验证。

2026-10-14 起步完成与当前支持状态已解耦：成功完成三帧完整车身支持交接后，
`startup_complete` 在本次运行中保持。有效单侧重建、当前覆盖变化或规划失败不会
把正常跟踪改回原点起步连接。缺失/无效输入仍走原有拒绝、有限恢复和停车流程；
显式重启产生的新规划器重新确认起步。日志 `planning_details.startup_complete`
与 `body_support.confirmed` 分别显示运行阶段和当前支持确认，允许前者 true、后者 false。

2026-10-14 起步交接改为完整车身支持确认：左右真实/历史边界需覆盖
当前车身前 0.297 m、后 0.198 m，并连续通过 3 个不同相机源帧才退出起步连接。
只覆盖参考点或使用路宽推算的边界不触发初次交接；重复帧不累计，覆盖丢失或规划失败
仅重置当前支持确认，不清除已完成的起步阶段。
未覆盖部分保留原始起步区域及其轮速/偏航运动补偿，矩形边缘检查同样只在观测支持内
检查道路边界，不用车尾外推曲线判碰撞。区域不会随车辆重新居中或重新建立。
原 `startup_corridor_check_enabled: false` 仍按用户测试配置保留：未观测区域依赖
起步清空假设；改为 true 时按固定区域检查其完整边缘。运动历史失效仍停车。
轨迹日志 `planning_details.body_support` 显示实际覆盖、所需范围及确认帧数。

2026-10-14 当前启用 `planning.lattice.boundary_collision_only_enabled: true`：
车身边界拒绝只看拟合道路与矩形车身是否越界，不再扣除硬净空、插值、扫掠和外推余量。
检查每个预测姿态的全部矩形边缘与分段三次边界的端点/极值，避免十个离散点之间漏检。
姿态仍按 0.05 m 轨迹网格检查，不代表已验证网格间连续扫掠或真实车辆不会碰撞。
日志 `collision_only: true`、各边界余量为 0；真实越界仍拒绝。改回 false 恢复余量模式。
该设置同时作用于 lattice 和恢复规划的共享车身检查；传感器、运动、运行时间、
覆盖范围及可信轨迹保持期限仍可触发停车。当前 lidar 障碍检查仍按原配置关闭。

2026-10-14 已扩展 lattice 起步连接搜索：除原有连接点外，增加允许的最远连接点
（当前上限 0.9 m）及它与原有最远连接点之间的中点（当前通常为 0.75 m）。
优先检查较长连接；按实际横向/纵向候选组合裁剪连接点数量，保留 16 候选上限、
计算预算和原有运动/边界约束。日志 `startup_join_space.sampled_join_x_m` 显示
实际搜索点，`candidate_cap_limited` 表示是否受候选上限裁剪。尚未上车验证。

当前启用单双侧中心线平滑交接：`estimation.centerline_transition_enabled: true`。
上一条中心线先通过轮速/IMU补偿到当前车身坐标系；双侧、左单侧、右单侧间切换
使用 0.5 s 五次时间权重过渡，中途反复切换从当前输出继续衔接。
横向参考变化率上限为 0.3 m/s；目标仍变化时交接可能超过 0.5 s。
路宽仍可随双侧观测更新。真实观测边界不做混合，规划器仍检查车身和运动约束。
日志同时输出 `raw_centerline_xy`、平滑后的 `centerline_xy` 及 `centerline_transition`。
无共同支持或运动补偿失败时不会凭空连接不相交的道路。轮速数值错误仍会影响补偿。

当前启用 `estimation.single_side_width_memory_enabled: true`：双侧相机拟合通过时，
从共同观测起点前方 1 m 内估计垂直道路宽度，取最近 5 个不同相机帧的中位数。
单侧缺失后，用当前可见边界沿局部法向偏移半宽重建中心线、偏移全宽重建缺失边界；
可见侧已经观测过的近端支持仍保留。推算侧标记 `width_inferred`，可见性仍为
`left_only/right_only`，不会把推算点写入真实锥桶缓存。双侧恢复后更新宽度。
宽度记忆有效期 30 s，运动历史重置时清除；尚未学到宽度时不凭空推算。
轨迹日志的 `width_memory` 记录宽度、左右半宽及双侧学习时间，
`single_side_reconstructed` 标记单侧重建。当前仍使用 lattice 受控起步。

成功的 `Planned trajectory:` JSON 现包含同一规划输入帧的 `planning_input`（中心线、
左右边界和源时间）、`planning_details.startup_connection.selected_join`（实际选中
连接点的位置/航向/曲率）及 `planning_state`（轮速/偏航角速度）。与轨迹 ID 同行，
在异步结果运动补偿前记录；不代表该轨迹已执行。道路点每侧最多记录 128 个，
超出时标明原始数量和抽样索引。建议测试后保存 `dream runtime logs ai4r_policy --tail 1000`。

当前临时关闭低速轮速/偏航一致性检查：
`planning.lattice.low_speed_yaw_check_enabled: false`，覆盖 lattice 和恢复规划。
改回 true 恢复；轮速为零时仍采用原有低速曲率近似，不代表轮速问题已解决。

当前恢复上一版 lattice 受控起步：`control.straight_startup_enabled: false`，
保留 `planning.lattice.startup_connection_enabled: true` 和 `clear_start_assumed: true`。
运动历史不足或尚无可行轨迹时保持零输出；规划器通过从车身原点连接前方观测道路的
轨迹起步，控制器跟随该轨迹转向。近端空白仍依赖操作员确认可通行的起步假设。
前瞻仍为 0.6 m、巡航目标 0.2 m/s；保留起步等待期限、30 s 总运行上限。
可选直线模式及其 3 s / 0.5 m 限制留在代码中，当前配置不启用。

当前临时测试关闭人工起步走廊净空检查：
`planning.lattice.startup_corridor_check_enabled: false`。
真实观测道路的车身边界、转向、运动与超时检查保留；改回 true 即恢复。

当前本地测试：过渡候选为 0.3/0.6 m，近似规划模型转角上限为 90°。
实车归一化转向输出上限仍为 0.5；这不代表测得车轮可以转到 90°。

2026-10-14 当前配置已取消累计 3 m 停车上限：`control.max_distance_m: 0.0`。
里程仍计算并发布；30 s 时间上限、传感器/运动和路径停车检查保留。尚未上车验证。

2026-10-14 用户要求的当前测试配置：`estimation.history_conflict_check_enabled: false`，
关闭新旧观测重叠差异的拒绝/隔离；其他道路几何、车身净空、运动和时间检查保留。
该修改尚未上车，230 项离线检查通过。

2026-10-14 当前本地 YAML 已按用户要求改为 0.6 m 路径弧长前瞻；
最短输出 0.2 m、控制预瞄 0.2 m，纵向候选/评分时长同步为 1 s，规划频率仍为 10 Hz。车身边界检查保留，
前端车身仍可能延伸到路径参考点之外。配置尚未上车，实车效果待验证。

2026-10-14 历史覆盖修复：连续道路模式下，合并拟合失败不再将后方历史替换为
当前前方观测；拟合筛点与历史保存分开。冲突仅隔离对应观测区域，该帧仍失效。
单条曲线无法保留真实支持时，尝试受同一覆盖、误差、斜率和宽度检查的局部分段
重建。Planning 的边界/中心线字段、坐标和时间含义保持；新增缓存、拟合和当前
相机范围诊断。当前明确道路冲突不会被旧控制参考遮蔽。
接口及验证说明见 [历史道路与 Planning 接口](docs/ESTIMATION_HISTORY_INTERFACE_CN.md)。

2026-10-14 修复及用户提供的实车日志：道路过期/拒绝不再跳过已建立起步走廊的运动更新。
只使用新鲜、有效且时空对齐的轮速/偏航率；真正运动断档仍停车，未放宽超时。
213 项离线检查通过，包含真实规划子进程的连续过期后恢复；完整 ROS 检查未运行。
用户反馈直道行驶约 2 m 后停止；日志显示近端覆盖丢失，未完成 3 m 验收。
后续弯道布局的一轮触发固定起步走廊前端约束。感知/估计组接手重点见验收记录。

本地起步重构（未部署）：受控居中、对齐起步时，从车辆原点生成平滑的 lattice
接入候选，接入距离上限仍为 0.9 m；不再用后向延拓边界判断初始车后角碰撞。
观测区保留真实边界检查，未观测区使用固定的人工确认起步走廊。轨迹寿命为
生成完成后 200 ms。另增加受检查的短路径候选，起步接入距离与横向过渡长度
分开；0.4 m 最低输出和 16 候选上限保留。首次起步改为第一条有效观测巡航
参考即可驱动，不再要求三帧确认；降级恢复确认和 3 s 初始等待仍保留。
211 项离线检查通过；尚未完成 ROS
门禁或实车验证。

2026-10-13 实车结论：尚未完成自主 3 m；两轮曾输出驱动但轮速为 0，最后的
0.2 m/s 起步下限修复被参考过期/启动确认阻塞，物理效果未验收。
完整尝试、失败原因、证据缺口和原始日志见
[当天 MVP2 复盘](docs/experiments/2026-10-13/MVP2_REVIEW_CN.md)。

`mvp2.0` 下周测试版本优先连续运行：20 Hz 控制与独立进程的 10 Hz 规划分开，
2026-10-13 当前配置在直线、弯道和起步均使用 Frenet lattice，关闭中心线恢复，
保留原采样集合及 16 条候选上限。当前规划频率 10 Hz、单次计算预算 100 ms，控制频率保持 20 Hz。
道路短暂失效或规划超时进入限时降级；默认降级目标 0.1 m/s，连续
3 个新的可信锥桶帧后平滑恢复。保持同一辆车周二实测的转向方向、驱动力前馈
与速度 PI。启动前在状态 2 静止至少 1 s，以学习 gyro 零偏；不要求先对齐中心线。
首次起步在原 3 s 预算内允许静止等待新的 lattice 结果，期间持续零输出；
当前本地改为第一条真实观测的有效巡航路径即可起步。候选淘汰原因、位置和数值
在参考选择/停车清理前记录；已上车尝试，尚未完成 3 m 运行。
起步修复：正常 MVP lattice 巡航目标下限为 0.2 m/s，防止零轮速重规划
反复给出极小目标；实测速度、停车和降级减速语义保留。已部署并尝试两轮，
均因过期结果打断 3 帧起步确认而在输出油门前停止，起步效果仍未验证。
具体实现、模型闭环和实际 ROS 进程证据见 [验收记录](docs/acceptance.md)。
3 m 运行仍未完成；达到 3 m / 30 s、运动反馈失效、明确道路冲突仍会停车。

已在 `scripts/policy_node.py` 接入估计→V2 Frenet lattice 规划→PID 控制→归一化动作。
V2 包含近端盲区补足、横纵向多项式采样、车身/障碍/运动初筛、固定打分与停车行为，
`mvp2.0` 输出原始 Frenet 记录及已检查的 Cartesian 采样点，控制侧插值预瞄位置、
航向和曲率，不再二次拟合五次曲线；当前配置不再选择二次 y(x) 中心线主路径。
实现和边界见 [V2 规划说明](docs/PLANNING_V2_CN.md)。供给的 YAML 选择单一
20 Hz timer，要求 cones、轮速和偏航率。近期无障碍锥桶道路测试配置关闭雷达障碍检查，
雷达继续采集但不是必需传感器；道路边界和车身净空检查仍开启。
当前 `control.mode: mvp` 使用路径误差与
航向误差直接计算归一化转向，目标速度 0.2 m/s；不要求先完成完整车辆标定。
每次显式启动累计原始轮速里程，达到 3.0 m 自动进入状态 2、持续输出零动作；
30 s 为兜底时间限制。无效/过期路径或反馈断流也会锁存停止，重新请求状态 3 才
重置里程并恢复。3 m 是编码器估计距离，实际停车位置受轮速尺度、打滑和滑行影响。
可选 `calibrated` 模式仍支持有来源的 `vehicle.*` 参数。上车步骤及老师要求见
[MVP 接入说明](docs/MVP_INTEGRATION_REVIEW_2026-10-05_CN.md)。
以下段落描述原 V2 接入；`mvp2.0` 的执行策略与参数以上方说明和验收记录为准。
离线测试与实际 policy 方法的模型闭环通过；实车验证仍未完成。
详细当前证据见 [验收记录](docs/acceptance.md)。

团队策略仓库，基于 DREAM 课程框架。当前分工见 [四组项目计划](docs/TEAM_PROJECT_PLAN.md)。接口优先沿用 GitHub 中的课程框架代码和配置；框架未定义的组间数据结构再按 Word 接口会议草案补充。状态、道路和规划分别在 `self.estimation_output`、`self.planning_output`；控制诊断在 `self.control_diagnostics`，debug1/debug2 为路径误差（m）和本次累计里程（m），停止原因沿用 policy 状态字符串。算法、字段、调参和联调说明见 [估计模块首版说明](docs/ESTIMATION_V1_CN.md) 与 [V1 规划说明](docs/PLANNING_V1_CN.md)。离线检查命令为 `python3 -B tests/test_estimation.py -v`、`python3 -B tests/test_planning.py -v`、`python3 -B tests/test_control.py -v`；模型测试为 `python offline/control_pid/test_controller.py`（AI4R 环境）。下文为上游使用说明。

2026-10-05 更新：有效道路/雷达点通过轮速与偏航率历史补偿到自车状态的参考时刻，并保留原测量时间；历史不足或断档时拒绝使用。该补偿采用前进、平面无侧滑近似，轮速时间为接收时刻代理，未完成实车精度验证。详情见上方模块说明。

2026-10-08 更新：道路估计增加短时历史锥桶缓存（最多 128 点），逐帧运动补偿后补足已经观测过的近端盲区。旧点保留原始年龄，5 秒内过期；首次启动的未观测近处道路仍拒绝使用。有效道路还要求双侧边界与中心线覆盖车体原点，增加 `geometry_valid` / `near_field` 诊断。该检查不替代规划组的完整车身和起步轨迹检查；当前规划分支已接入 V2 与控制器。离线回归、参数和下游接入要求见模块说明。

同日补充：新鲜相机帧偶发空帧、单侧拟合失败或共同覆盖不足时，可短暂运动补偿最近一条完整有效道路，标记 `degraded=True` / `predicted_history`；默认失败延续上限 0.2 秒，原始源年龄、运动跨度和近端历史期限可能使其更早结束。预测不续期，当前失败原因与每侧点数/拟合范围另行保留。过期、运动缺失、异常宽度或观测冲突仍拒绝使用。当前规划分支 V1/V2 已消费降速建议和剩余有效时间，V2 支持车后道路及明确受控起步时的近端补足；PID 与车辆动作映射保持。详细规则见 V2 说明。

Student ROS 2 policy package for the DREAM robot, targeting Ubuntu 24.04 and
ROS 2 Jazzy. Version 0.2.0 adds ArUco observations to the first source release;
a published annotated tag and successful release CI establish release identity.
It is not a physically qualified driving controller.
[Acceptance](docs/acceptance.md) records observed software checks and remaining
robot checks, and [the v0.2.0 release preparation](docs/release-v0.2.0.md)
defines promotion, tag, artifact, and publication evidence.

**Start with [scripts/policy_node.py](scripts/policy_node.py)** and its
`INSERT POLICY CODE` section. It contains the observation/action explanations
and a zero-action starter. The [policy YAML](config/ai4r_policy.yaml) explains
every setting and includes lidar-only and wheel-speed-controller examples.

| Update mode | Executes your policy when |
| --- | --- |
| `cone_detection` | A fresh cone-detection batch arrives |
| `fiducial_detection` | A fresh fiducial-detection batch arrives |
| `lidar` | A fresh lidar scan arrives |
| `timer` | The configured policy timer runs |

Other callbacks cache observations. Declare `required_sensors` for your exercise;
missing/stale required data stops the policy. Required cones also have an
independent empty-detection timeout. Recovery needs an explicit start request.
Cone batches carry `acquisition_to_publish_latency_s`, measured from OAK-D's
end-of-exposure timestamp until immediately before the detector's `publish()`
call. Exposure duration is excluded; this is not exposure-start-to-publication
latency. The student starter exposes
`cone_acquisition_to_publish_latency_s` and `cone_measurement_age_s` alongside
the existing coordinate/colour/confidence lists. The latter is the current age
from the same end-of-exposure acquisition reference, already including latency;
never add the two together.
Internally, `observations["cone_detections"]` is now a batch dictionary with
`detections` (the existing five-value tuples) and
`acquisition_to_publish_latency_s`, or `None` when unavailable/stale. This
replaces the previous raw-list value; the student function signature is unchanged.
Rebuild the OAK-D producer and policy against the same updated cone message.
Fiducial batches include dictionary/source-frame metadata and marker records;
valid poses are transformed into `policy_frame_id` at image acquisition time.
A fresh empty marker list is valid data, with no automatic visibility timeout.
The exact student Python record fields are documented beside
`calculate_policy_actions` in the policy script. A valid pose is a successful
geometric solve, not proof that the marker is real or safe to drive toward.
Your policy selects expected IDs and evaluates distance, reprojection error
and consistency across observations; background false detections are possible.

## Build and run

The robot must first have a compatible DREAM underlay and a built student
overlay at `~/ai4r_student_workspace`. In particular, `dream_interfaces` v0.2.0
must provide `FiducialDetections` and the units-bearing `DriveAndSteer`;
older definitions are incompatible. The compatible dependency is pinned in
[ci/dependencies.repos](ci/dependencies.repos). Staff/developer standalone build
and test instructions are in [CONTRIBUTING.md](CONTRIBUTING.md).

For managed DREAM runs, keep this package in the external student workspace and
set `~/.config/dream/ai4r.yaml` to:

```yaml
student_workspace: ~/ai4r_student_workspace
```

DREAM uses that workspace's installed overlay only for `ai4r_policy`. Its
[AI4R runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/feature/initial-autonomous-car-stack/docs/ai4r-runtime.md)
describes the explicit build, source-YAML validation and restart commands.

With the compatible underlay and student overlay sourced:

```bash
ros2 launch ai4r_policy ai4r_policy.launch.py namespace:=car
```

The launch supports optional `namespace` and `params_file` arguments. Shipped
policy settings load first, then the explicit overlay. It starts only the
`ai4r_policy` node and does not respawn it. DREAM owns hardware services and
mounting transforms separately. Its camera transform describes the fixed
zero-pan pose; physically panning the camera invalidates that transform until a
measured dynamic TF is supplied.

The policy starts in state **2: publishing zero drive and steering**. It holds
the pan target by sending no pan command. Vehicle Enable/Disable/Disarm is
separate: wait for the vehicle's Enabled state after its explicit Enable request,
then request policy state 3. Ongoing zero commands satisfy the policy side of
the vehicle's fresh-zero pre-enable handshake. The policy never enables it.

```bash
# Start policy calculations after required sensors and the vehicle are ready:
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
# Stop policy driving actions and publish zeros continuously:
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
```

State **1** ceases all action publication; it is neither Disarm nor an immediate
vehicle stop. A blocked Python calculation also blocks this node's watchdog;
the independent vehicle command watchdog remains necessary. Sending a zero
command does not prove physical stopping or take control away from manual RC.

## Interfaces and configuration

All names below are relative to the selected namespace. The fixed node name is
`ai4r_policy`. Sensor subscriptions keep the latest observations; IMU fields have
independent validity/freshness. Cone positions are metres in `policy_frame_id`
(normally `base_link`). DREAM places the normal `base_link` origin on nominal
ground directly below the agreed vehicle centre of gravity: +x forwards, +y
left, +z up. A cone position's `z` is the detected point's nominal height above
that ground plane, not the cone's total height. The fixed-pose transform does
not compensate for terrain or vehicle pitch. Raw lidar remains in its message
frame with its existing origin; Cartesian points use mounting TF to reach the
body frame. Body IMU vectors use mounting TF, and relative
heading resets only when entering policy state 3.

Every accepted scan prepares `lidar_scan` and attempts `lidar_cartesian` in the
subscriber callback, before a lidar-triggered policy step. There is no conversion
switch. Students receive `lidar_ranges`, `lidar_intensities`, `lidar_points_xyz`
and `lidar_scan_indices`, with separate `lidar_scan_available` and
`lidar_cartesian_available` flags. Cartesian points are finite, in-range `(x,y,z)`
tuples in metres; index `j` corresponds to raw ray `lidar_scan_indices[j]`.
The Cartesian dictionary contains `points_xyz`, `scan_indices` and `frame_id`.
Missing TF or failed conversion leaves the raw scan usable and immediately
discards the previous Cartesian result. A fresh empty Cartesian list is valid.
Snapshots retain matching acquisition stamps and cannot expose Cartesian data
beyond either its own timeout or the raw scan timeout. One nonblocking transform
lookup uses the first ray's timestamp for the entire scan; there is no per-ray
motion correction or IMU levelling.

**Configuration migration:** rename the old observation/required-sensor name
`lidar` to `lidar_scan`, and `sensor_timeout_s.lidar` to
`sensor_timeout_s.lidar_scan`. The update mode remains `lidar`. That mode must
require `lidar_scan`, `lidar_cartesian`, or both. Cartesian driving policies
should require `lidar_cartesian`; its timeout defaults to 0.5 s. Timer and other
sensor modes reuse the same cached representations and freshness handling.

| Topic | Type | Direction / QoS |
| --- | --- | --- |
| `cone_detections` | `dream_interfaces/ConeDetections` | Input; reliable, depth 1 |
| `fiducial_detections` | `dream_interfaces/FiducialDetections` | Input; reliable, depth 1 |
| `scan` | `sensor_msgs/LaserScan` | Input; best effort, depth 1 |
| `wheel_speed_m_per_sec` | `std_msgs/Float32` | Input; reliable, depth 1; unsigned m/s |
| `imu/data` | `sensor_msgs/Imu` | Input; best effort, depth 5 |
| `policy_fsm_transition_request` | `std_msgs/UInt16` | Input; reliable, depth 10; states 1/2/3 |
| `drive_and_steer_set_point_normalized` | `dream_interfaces/DriveAndSteer` | Output; reliable, depth 1; explicit normalized units |
| `pan_set_point_normalized` | `std_msgs/Float32` | Output; reliable, depth 1; optional normalized target |
| `policy_fsm_state_value`, `policy_fsm_state_string` | `std_msgs/Int8`, `std_msgs/String` | Output; reliable, depth 10; current state and reason |
| `imu_heading_angle` | `std_msgs/Float32` | Output; reliable, depth 10; valid tared heading in degrees |
| `debug1`, `debug2` | `std_msgs/Float32` | Output; reliable, depth 10; optional student scalars |

All QoS is volatile. Standard ROS parameter/introspection services and TF
subscriptions also exist. Finite actions are clipped to `[-1,1]`; invalid
outputs or student exceptions stop the policy. A pan target of `None` holds;
zero recentres. Drive is motor effort, not a speed setpoint. Full meanings and
examples are beside the variables in the Python file.

Five commented ROS parameter files are installed:

- [ai4r_policy.yaml](config/ai4r_policy.yaml): triggers, required sensors and timing.
- [traxxas_vehicle_interface.yaml](config/traxxas_vehicle_interface.yaml): editable steering limits/trim, timeouts, slew rates, RC calibration and wheel/encoder settings; commented identity and pan-joint references.
- [oakd_cone_detector.yaml](config/oakd_cone_detector.yaml): perception and debug settings.
- [bno08x_imu_interface.yaml](config/bno08x_imu_interface.yaml): selected IMU products and accuracy.
- [aruco_detector.yaml](config/aruco_detector.yaml): dictionary, marker sizes and detection filtering.

ArUco filtering runs in `aruco_detector`, before observations reach the policy.
Set `allowed_marker_ids: [4, 5]` for an exercise using only those IDs; the shipped
empty list allows every dictionary ID. `min_consecutive_frames: 3` requires an
ID in three consecutive processed images before publishing the current result,
adding about 0.2 s at 10 Hz. Set it to `1` to disable confirmation. A missed
detection removes the ID immediately; reacquisition must confirm again. A gap
over 0.5 s between image acquisitions resets confirmation, so rates below 2 Hz
require confirmation to be disabled. This filters IDs, not individual copies
of the same ID, and cannot exclude a persistent false detection of an allowed
ID. Repeated-ID observations remain separate and poses are never replayed.
An image with no accepted markers still supplies a fresh empty batch; the
student continues to decide the driving response to marker absence.

The optional [camera mount file](config/camera_mount.yaml) is also installed,
but it is DREAM source configuration, not ROS parameters. The shipped `{}` uses
the defaults: camera optical centre `0.30 m` above nominal ground, `0.10 m`
behind the CG (`forward_from_cg_m: -0.10`), centred at `y = 0`, and tilted
`20 degrees` down. It accepts only `height_above_ground_m` (finite and
nonnegative), `forward_from_cg_m` (finite and signed), and
`downward_tilt_deg` (finite, strictly between `-90` and `90`, positive down).
Fields may be omitted to inherit their defaults; a missing file also uses all
defaults.

The optional [lidar mount file](config/lidar_mount.yaml) likewise ships `{}` to
inherit DREAM's baseline: scan origin 0.20 m forward of the CG, centred laterally,
0.12 m above nominal ground, level with 180-degree yaw. It admits
`forward_from_cg_m`, `left_from_cg_m`, `height_above_ground_m`, `roll_deg`,
`pitch_deg` and `yaw_deg`. Distances are finite metres, height is nonnegative,
and angles are finite degrees in [-180, 180]. Rotations are absolute,
right-handed `Rz(yaw) * Ry(pitch) * Rx(roll)` from the lidar frame into
`base_link`, not offsets added to the default. The file documents signs and
measurement references. Apply edits using `dream runtime restart rplidar_c1`;
policy restart alone does not reload the pose. Invalid settings leave the
current lidar run intact. Cartesian points use the updated TF; raw scan ranges
are unchanged. Install the matching DREAM lidar-mount support first; older
DREAM versions ignore this optional file. No dynamic/IMU compensation is added.

Policy settings are startup-only. DREAM reads the current source YAML under
`~/ai4r_student_workspace/src/ai4r_policy/config/` on each new start or restart,
so editing these files needs no student-workspace rebuild. Restart only the
affected unit: `dream runtime restart ai4r_policy` for policy settings, or
`dream runtime restart traxxas_vehicle_interface`, `oakd_cone_detector`,
`bno08x_imu_interface`, or `aruco_detector` for that component's YAML.
Camera-mount changes take effect through `dream runtime restart oakd_cone_detector`. DREAM validates and
snapshots the effective camera configuration before stopping a running detector;
malformed YAML, unknown keys, or invalid values therefore leave it running with
its previous snapshot. An idempotent `dream runtime start oakd_cone_detector`
does not restart or reload an already-running unit. Restarting policy alone does
not reload a hardware unit. The standalone policy launch loads only its own ROS
parameter file and never consumes `camera_mount.yaml` or `lidar_mount.yaml`; DREAM admits component
requests before passing them to independently launched units. The active vehicle
settings override DREAM's baselines; use measured calibration and geometry for
your car. Omitted settings inherit the baseline. This requires DREAM's expanded
Traxxas delegation; older versions reject the newly exposed settings. Commented
reference values remain system-owned. See the [runtime guide](https://gitlab.unimelb.edu.au/dream/dream_system/-/blob/feature/classroom-ros-environment/docs/ai4r-runtime.md)
for the complete source-YAML workflow.

Direct-node equivalent for integrations that need one (Python ROS launch):

```python
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path
policy = Node(
    package="ai4r_policy", executable="policy_node.py", name="ai4r_policy",
    namespace="car", output="screen", respawn=False,
    parameters=[str(Path(get_package_share_directory("ai4r_policy")) /
                    "config" / "ai4r_policy.yaml")],
)
# Append one explicit policy overlay after the shipped file if needed.
```

## Integration status and development

This is the standalone source repository. DREAM's AI4R integration defines the
managed policy and component runtime units and admits source-YAML overrides.
Its `base_link` origin is nominal ground directly below the agreed CG. The
default zero-pan camera optical pose is `[-0.10, 0.00, 0.30]` m relative to that
origin, with a 20-degree downward tilt; the optional camera mount file overrides
height, forward offset and tilt. The nominal IMU translation now has
`z = 0.10 m` in this ground-level frame, with its existing rotation unchanged.
Sensor observations retain their contracts except that transformed
cone `z` is rebased to the ground-level origin; raw lidar data is unchanged.
Physical pan motion requires a measured dynamic transform before cone positions
can be trusted. The fixed flat-ground pose does not compensate for terrain or
vehicle pitch, and physical frame validation remains separate. The policy does
not own those services, perform hardware discovery, or install student
dependencies.

`dev` is the active development branch; `main` is the release-ready line.
Annotated immutable tags identify candidates and releases. Use feature branches
and reviewed MRs; safety, public-interface and CI-policy changes require recorded
human review.
See [contribution checks](CONTRIBUTING.md), [change history](CHANGELOG.md),
[acceptance](docs/acceptance.md), and the [MIT license](LICENSE).

Traxxas output mapping and final PWM bounds are documented beside the eight
integer microsecond settings in `config/traxxas_vehicle_interface.yaml`. DREAM
admits and validates them before vehicle-interface restart. They affect manual
RC fallback too; restarting only the policy does not reload them. Matching
firmware/interface 0.5.0 is required.
