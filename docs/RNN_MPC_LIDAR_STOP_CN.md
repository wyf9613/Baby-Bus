# RNN＋MPC 雷达停车

基线：`checkpoint/rnn-mpc-car13-20261026`，提交 `45d492f`。
本地分支：`feature/rnn-mpc-lidar-stop`。
运行版本：`user-mpc-rnn-lidar-stop-20261026`。

## 行为与参数

- 沿用 RNN 选路、MPC/PI、0.5 m/s 目标速度及原有求解/运动数据时限。
- 水平全周雷达，经采集时刻 TF 转到 `base_link`；过滤无效距离、提取前方点、相邻点聚类。
- 车头前方 **0.5 m**、车宽 **0.25 m**、两侧余量各 **0.02 m**。
  以当前几何计算，车体坐标 ROI 为 x=[0.297, 0.797]、y=[-0.145, 0.145] m。
- 量程下限 0.2 m；传感器允许的更近有效障碍点仍保留。裸节点默认关闭，使用本分支 YAML 开启。
- 至少 2 个相邻点为障碍；孤立点、可观测区域数据不足为 UNKNOWN，也停车。
- 延续之前的显式设置：`allow_no_return_clear=true`，仅量程覆盖检测区时将 +inf 视为假定无回波；NaN、-inf 不视为空闲。这不能排除不反光物体或持续无回波故障。
- 原始扫描及转换结果必须同时间戳且新鲜，沿用本基线的 0.5 s 时限；TF 失败或无效扫描使本次检测不可用。

启动前、控制计算前后、独立监督定时器均检查雷达。遇障或数据不可用时持续请求
零驱动、零转向并清除当前运行参考；障碍消失后不自动恢复，由 Foxglove 手动 Run。
被拦截的动作不提交 RNN 执行记忆。云台保持原目标。

零驱动不等于物理停车。`STOPPED` 仅在收到停车请求后多个新鲜轮速零值覆盖 0.2 s 后给出；
非零为 STOPPING，轮速失效为 UNKNOWN。没有增加主动制动或倒车请求。
该矩形不覆盖转弯扫掠区域，0.5 m 是检测距离，实际停车距离尚待实车验证。

## 上传与运行

只上传以下两份文件，不覆盖车辆驱动、传感器或相机安装配置。RNN 权重和模块已嵌入单文件。

本机 PowerShell：

```powershell
cd "C:\Users\17892\OneDrive\文档\2026 Semester 2\Ai4R\final\mpc-latest-workspace"
scp scripts/policy_node.py "ai4r@10.43.254.26:/home/ai4r/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py"
if ($LASTEXITCODE -ne 0) { throw "Policy upload failed" }
scp config/ai4r_policy.yaml "ai4r@10.43.254.26:/home/ai4r/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml"
if ($LASTEXITCODE -ne 0) { throw "Configuration upload failed" }
ssh ai4r@10.43.254.26
```

车辆 SSH，在车辆 Disabled 时执行：

```bash
cd ~/ai4r_student_workspace/src/ai4r_policy
dream build ros student && dream runtime restart ai4r_policy
```

等待节点出现后核对：

```bash
ros2 param get /car/ai4r_policy lidar_obstacle.enabled
ros2 param get /car/ai4r_policy lidar_obstacle.stop_distance_m
ros2 param get /car/ai4r_policy mpc.shadow
timeout 5 ros2 topic echo /car/lidar_obstacles --once --full-length
```

预期为 true、0.5、false。日志版本须为本文所列版本。雷达状态 CLEAR 后按原 Foxglove
流程手动 Enable/Run；停车后查看原因、移除障碍，再手动 Run。

| 主题 | 类型 | 内容 |
| --- | --- | --- |
| `/car/lidar_obstacles` | String JSON | CLEAR / OBSTACLE / UNKNOWN、ROI、聚类、距离、采集时间 |
| `/car/stop_status` | String JSON | STOPPING / STOPPED / UNKNOWN、轮速与确认情况 |
| `/car/stop_state` | Int8 | 0 未请求停车、1 停车中、2 零轮速确认、3 未知、4 不发布动作 |

这次提供检测数据主题；未新增 Foxglove 布局，现有路径显示保留。

## 验证

本地检查和限制记录在 `docs/acceptance.md`。操作者反馈本版已能在实车运行；
遇障停车距离及故障恢复尚未提供专项实测结果。代码按用户要求推送至独立功能分支。
源模块为 `mvp21_policy/lidar_obstacles.py` 和 `mvp21_policy/node.py`；修改后重新生成：

```bash
python3 -B tools/bundle_python.py scripts/mvp21_policy_node.py --root . --output scripts/policy_node.py --no-auto-resources --resource config/ai4r_policy.yaml --resource mvp21_policy/models/path_policy.json
```

原 checkpoint 的哈希仍属于原版；本版文件哈希见 `docs/rnn-mpc-lidar-manifest-20261026.json`。
