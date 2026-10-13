# MVP2 车载测试记录（2026-10-13）

车辆：`ai4r@10.43.254.20`。候选：`mvp2.0`，提交
`ae74be454bccec472c677092afb35de899cdc130`。现场已确认场地清空并有人持遥控器看护。

## 部署

车端原学生 checkout 为 `dev` 且有已有修改。完整源文件备份位于
`/home/ai4r/mvp2-test-20261013/student-source-before.tar.gz`，差异与状态同目录保存。
仅同步候选的 `scripts/policy_node.py` 和 `config/ai4r_policy.yaml`，保留硬件配置。
`dream build ros student` 成功，安装脚本及源 YAML 的 SHA-256 与本地候选一致。
候选完整源码独立解包在 `/home/ai4r/mvp2-test-20261013/candidate`，用于完整测试。

## 软件检查

使用 exact-clean `dream_interfaces`：`5f50902ccee44e8370d6e2be85607b3054ffbf98`。
执行 `AI4R_INTERFACES_SOURCE=~/dream_system/ros2_ws/src/dream_interfaces bash tools/verify_fast.sh`，
使用隔离 localhost domain 218。

231 个实际用例中 230 通过、1 失败：ROS 75/75、估计 53/53、规划 18/18、
lattice 42/43、控制 42/42。失败用例
`LatticeChecks.test_actual_node_can_follow_road_without_optional_lidar` 返回
`Planning: source_deadline_during_planning`，单独重跑同样失败。
完整 gate **未通过**。该用例使用默认大搜索配置与实际墙钟，而非活动 YAML 的
中心线优先、16 候选、直接采样和异步规划配置；尚未修复该用例或修改运行截止条件。

## 首次真实传感器联测（禁止驱动）

车辆保持 Disabled，策略请求 state 3。3.022 s 后因
`Control startup history deadline exceeded; explicit resume required` 进入 state 2。
最大请求 drive、最大轮速与累计里程均为 0。清理确认 state 2、Disabled、零请求和静止。
见 `evidence/dry/result.json` 与 `evidence/dry/run.jsonl`。

使用原始候选另起诊断节点，其全部策略输出和请求入口均重映射到
`/car/mvp2_shadow/`；未向车辆发送动作或 Enable。25 次 worker 结果全部为
`invalid_estimates_or_boundaries`，道路全部为 `invalid_width_or_order`，
速度与偏航率有效，worker 正常就绪。

消息合同为 `COLOR_YELLOW=1`、`COLOR_BLUE=2`，body +y 向左。
首次联测末帧黄色 y 中位数 +0.435 m，蓝色 -0.229 m；即黄左蓝右。
候选约定蓝左黄右，因此有符号道路宽度为负，被估计器拒绝。
这是本次无法建立初始路径的直接原因，与软件用例的耗时失败不同。

## 调整场景后的检查

实时连续五帧确认蓝左黄右：一次读回黄色 y 中位数 -0.287 m，蓝色 +0.332 m。
再次联测前发现另一会话已修改脚本和 YAML，运行参数为 `control.enabled=false`、
`planning.cruise_speed_mps=0.1`。测试脚本在参数检查阶段中止，没有请求 Enable。
新修改保存至车端 `concurrent-edit-backup/`；用户明确选择备份后恢复原版继续。
再次部署前还保存了 `/home/ai4r/mvp2-test-20261013/redeploy-before.egcXMI/`。

恢复原版、重建和启动成功，源码/YAML 再次与候选哈希一致。
测试脚本增加启动前及运行期间每 0.25 s 的候选文件哈希检查。

原版在纠正颜色的场景中仍未通过禁止驱动联测：0.3647 s 后因
`Control reference: Planning: recovery_body_boundary_conflict` 停止。
最大请求 drive、最大轮速、累计里程仍均为 0，清理确认 state 2、Disabled、零请求、静止。
见 `evidence/dry-restored/result.json` 和对应 `run.jsonl`。
该原因表示当前道路拟合下的近端车身/预测轨迹净空检查未通过，尚未证明真实车身碰撞。
没有放宽或绕过该检查。后续影子诊断还遇到新鲜轮速不可用，未成功进入计算状态。

本轮落地 3 m 测试 **未放行**，没有发出车辆 Enable 请求，也没有观察到车辆运动。
需要先解决当前场景的车身边界拒绝以及软件用例的墙钟截止失败，再完成运动验收。

首次独立停止读回确认车辆 Disarmed、policy state 2、请求 drive/steer 为 0、轮速 0。
末次清理的独立读回保存在 `evidence/final-*.txt`。
原始策略分支未修改、未提交或推送。

## 加宽后的重试，11:01

用户再次请求直接测试；已核对车端源码、配置、安装脚本仍与原版候选一致。
禁止驱动联测在 3.0287 s 后触发启动历史截止；请求 drive、轮速和里程仍均为 0，
清理确认 state 2、Disabled。静止检查未通过，因此后续一次 3 m 运行没有执行。
结果与日志保存在 `retry-1101/`，未覆盖先前证据。

本轮影子节点取得 24 次 worker 原始结果：最初 3 次为启动时运动历史不足，随后
21 次为 `insufficient_lattice_coverage`，候选列表全部为空。
双侧道路拟合及时间对齐有效，一份代表性结果的道路宽度约 1.032 m。
两侧共同有效道路的起点为车前 0.942–0.984 m，超过活动配置
`planning.lattice.max_near_gap_m=0.9`。右侧黄色边界的最近有效观测决定了这个起点。
中心线方案与 lattice 均在该输入覆盖检查处拒绝；这轮尚未进入候选轨迹搜索。
持续没有初始可用参考，最终达到 3 s 启动期限。

建议现场在车前约 0.5–0.7 m 补充清晰可见、与右侧道路平顺衔接的黄色锥桶，使双侧
共同支持从更近处开始，再验证后续净空与跟踪检查；本次结果不证明后续检查一定通过。
末次独立读回为 Disarmed、policy state 2、drive/steer 均为 0。
