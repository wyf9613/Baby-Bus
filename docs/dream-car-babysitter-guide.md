# DREAM 小车 Babysitter Guide：从连接到 MPC Shadow 验证

整理日期：2026-10-05（Australia/Sydney）。适用本机：Windows PowerShell、WSL Ubuntu 24.04 / ROS Jazzy；目标设备：课程 DREAM 小车的 Jetson Orin Nano。

本指南依据用户提供的课程页面全文，结合当前 Baby-Bus 源码和 [MPC prototype 文档](MPC_PROTOTYPE.md) 整理。课程页面是本次提供的快照，没有重新验证 Canvas 或车上软件的最新状态。课程第 3 页本身注明了 draft；实际命令可用性要由 `dream runtime catalog`、`--help` 和 demonstrator 确认。

**当前推荐走到 G2：车辆保持未 Enable，MPC 只计算候选动作并输出零动作。G3 标定和 G4 动力执行另行开展。**

## 0. 先分清自己在哪台电脑

### 0.1 三个终端，三种用途

| 标记 | 位置 | 做什么 | 常见提示符 |
|---|---|---|---|
| **本机 PowerShell** | 你自己的 Windows | SSH、SCP、本地版本和文件管理 | `PS E:\26b\AI4R\Baby-Bus>` |
| **本机 WSL** | 你自己的 Ubuntu | ROS 软件验证、MPC Python 环境、本地生成部署 YAML | `lucien_lap_a@DESKTOP-IUA0CF1:...$` |
| **车上 SSH** | 登录后的小车 Jetson | 查询、接收文件、DREAM build/runtime、ROS 检查与录包 | 用户应为 `ai4r`，主机名以现场为准 |

WSL Ubuntu 不等于 Jetson。电脑上的 Ubuntu 检查通过后，还需在车载计算机确认依赖、运行耗时和真实传感器输入。

不要复制终端提示符里的用户名、`$`、`>`。Bash 的反斜杠 `\` 续行命令在 Ubuntu / SSH Bash 中执行；PowerShell 命令单独标注。

### 0.2 课程规则

- 在自己的电脑编辑和保存代码，以本机版本为准。
- 用普通终端和 SSH 登录小车，用 SCP 传文件。
- 不在小车上安装软件、使用编辑器、运行 VS Code Remote-SSH Server 或 AI 助手。
- 本机运行的 AI 助手可以通过 SSH 查询、传文件和执行已授权命令；AI 工具本体留在本机。
- 只操作 demonstrator 分配给自己团队的车，不尝试相邻地址。
- 车辆共享；协调本次操作时段、操作者和 RC 接管人员。

因此，本指南不要求在车上 clone Git 仓库，也不给出车上 `pip install` / `apt install` 的安装步骤。依赖缺失交给 demonstrator / 系统负责人处理。

### 0.3 软件运行、车辆状态和策略状态是三件事

```text
runtime unit 启动成功
    ↓
策略进程启动，默认 state 2：持续发布零动作
    ↓
shadow 验证：车辆保持未 Enable → 请求策略 state 3 → 算法计算，应用动作仍为零

日后动力执行：完成标定和执行条件 → 发布新配置 → build / restart
    ↓
确认策略在 state 2 发布新鲜零动作
    ↓
Vehicle Enable → 等待 Enabled → 策略 state 3
```

策略 state 3 不会替你 Enable 车辆；启动服务也不代表车辆已 Enable。

## 1. 课程页面顺序与每页的实际作用

| 顺序 | 课程页面 | 完成后应得到什么 | 本项目特别注意 |
|---|---|---|---|
| 1 | SSH and VPN Setup | 确认自己的车号，SSH 登录 `ai4r` | Jetson 型号已明确；保持 VPN，不在车上开发/安装 |
| 2 | Clone and Edit Your Policy | 本机策略和参数准备好 | 已有 Baby-Bus，不需要重做上游 clone |
| 2a | Pass Your Notebook Policy to the Car | 算法适配真实输入、动作和时间 | 不能照抄颜色 ID、LiDAR 索引、刹车和转角转换 |
| 3 | Transfer and Build | 本次文件传到 student workspace，构建成功 | Git push 不部署车；只传本次需要的文件 |
| 4 | Run the Car | 所需 runtime units 启动，状态控制正确 | 当前 MPC 走 shadow，跳过动力 Enable 步骤 |
| 5 | Foxglove UI | 本机连上 bridge，看到传感器、状态和动作 | Foxglove 不是策略执行器，关闭它不停止策略 |

本指南在这些课程步骤之间加入 MPC G1/G2 检查，并把 G3/G4 放在最后。

## 2. 第 1 页：VPN、车号与 SSH

### 2.1 连接前

在校园现场：自己的电脑连接 UniWireless，开启 GlobalProtect 的大学 VPN；小车上电并连接 UniWireless。密码由 workshop 提供，不写进代码、文档或日志。

用户提供的 VPN 链接含 `YOUR_VPN_LINK_HERE`，是占位链接。通过学校正式 VPN 入口获取设置说明，不能把该占位链接当成有效地址。

### 2.2 确定自己的地址

车上 Ethernet 端口标签为 `01`～`20`：

```text
最后一段地址 = 标签数字 + 13
IP = 10.43.254.<最后一段地址>
```

例如 `07 → 10.43.254.20`，`20 → 10.43.254.33`。标签与团队分配由 demonstrator 确认，不通过扫描邻近地址寻找车辆。

**位置：本机 PowerShell。** 输入已确认的标签数字，下面的变量供本次 PowerShell 会话后续使用：

```powershell
$taskCarPort = [int](Read-Host '输入 demonstrator 已确认的 Ethernet 标签数字（1～20）')
if ($taskCarPort -lt 1 -or $taskCarPort -gt 20) { throw '车号必须在 1～20 范围内' }
$taskCarIp = '10.43.254.' + ($taskCarPort + 13)
Write-Host "本次目标：ai4r@$taskCarIp"
ssh "ai4r@$taskCarIp"
```

首次连接会显示 host key fingerprint。核对目标车身份后接受，输入 workshop 密码。密码输入不显示字符是正常现象。

**位置：车上 SSH。**

```bash
whoami
hostname
command -v dream ros2 python3
dream runtime catalog
dream runtime status
```

预期用户为 `ai4r`，DREAM 命令可用，catalog 列出支持的 units。主机名不在资料中，不能用本指南猜测。`ros2` 不可用时，让 demonstrator 确认如何加载现场 ROS 环境。

本轮 MPC 源码基于当前 Baby-Bus 消息接口。部署前做只读检查：

```bash
ros2 interface show dream_interfaces/msg/DriveAndSteer
ros2 interface show dream_interfaces/msg/ConeDetections
ros2 interface show dream_interfaces/msg/FiducialDetections
```

对照本次仓库 README / 消息要求：动作消息包含归一化单位约定，锥桶批次包含延迟元数据，源码导入的 fiducial 消息可用。这些检查不能证明 underlay 来自精确的接口 commit；构建来源仍由系统负责人确认。接口不兼容时暂停部署，由负责人更新课程环境，不在车上复制自制消息库或安装 ROS 包来凑齐。

退出 SSH 返回本机：

```bash
exit
```

### 2.3 SSH 故障

| 现象 | 首先检查 | 下一步 |
|---|---|---|
| Timeout / no route to host | 车是否上电、UniWireless、GlobalProtect | 记录自己的车号和完整报错，找 demonstrator |
| Permission denied | 用户是否 `ai4r`、自己的车 IP、密码 | 不把密码发给他人或放进 remote URL |
| Host key changed | 是否有换机/重装，目标 IP 是否正确 | 先确认原因，再在本机清除旧 entry |

确认车确实更换了 host key 后，**本机 PowerShell**：

```powershell
ssh-keygen -R $taskCarIp
ssh "ai4r@$taskCarIp"
```

课程允许每次输密码；SSH key 是可选设置，不是本次流程的前置条件。

## 3. 第 2 页：本地代码与版本

### 3.1 你已经有仓库

本机 Windows 仓库为 `E:\26b\AI4R\Baby-Bus`，WSL 验证 clone 为 `~/ai4r-dev/Baby-Bus`。不用为了课程 clone 示例再创建一份上游仓库。

**本机 PowerShell：**

```powershell
cd E:\26b\AI4R\Baby-Bus
git branch --show-current
git log -1 --oneline
git status --short
```

**本机 WSL：**

```bash
cd ~/ai4r-dev/Baby-Bus
git branch --show-current
git rev-parse HEAD
git status --short
```

核对部署源与通过验证的源码一致。当前已知 MPC 分支为 `control-mpc-v0`，MPC 实现提交为 `54bc3a9`；以后有新提交时，以本次验证的完整 SHA 为准。

同一分支名不代表同一版本，未提交改动也不能由 SHA 唯一标识。不要为了让 status 干净覆盖 Notebook 或他人的改动。

### 3.2 编辑边界

课程主要允许学生编辑 `scripts/policy_node.py` 和自己的 `config/ai4r_policy.yaml`。当前仓库已包含估计、规划、MPC 和 shadow 参数，继续保留插入标记、状态机、超时和零动作发布行为。

不改车上驱动代码，不通过复制整个 `config/` 到活动 workspace 覆盖其他团队/车辆的硬件标定。车载源参数由 DREAM 在启动时读取；改 Python 后需 build，加载新策略需 restart。

课程要求每次复制后执行 `dream build ros student`；本指南沿用该完整流程。仓库 README 对纯 YAML 的说明更细：DREAM 重启会读取源 YAML，单纯参数变更通常不要求编译，但依然需要重启相应 unit。

### 3.3 本机验证记录

用户在本会话报告已完成离线测试、ROS gate 和 WSL shadow 参数加载。完整日志仍应保留，不能把参数查询结果当成全部测试通过的唯一证据。

本机复跑与解释器排查见 [ROS verification babysitter](ros-verification-babysitter.md)。当前 CMake 没有注册 `tests/test_mpc.py`，需要单独运行 MPC 离线套件。

此前遇到的错误：能导入 `ament_index_python`，但当前终端只加载了 `/opt/ros/jazzy`，测试因此去找安装版 `ai4r_policy` 并失败。工作区离线检查可临时清除单条命令的 `PYTHONPATH`；实际 ROS gate 则应构建并加载正确 overlay。

## 4. 第 2a 页：Notebook 移植核对

这是检查当前实现的清单，不要求把课程示例覆盖到已有 MPC 代码。

### 4.1 输入、输出与单位

| 内容 | Notebook / 仿真 | 当前真实策略应使用 |
|---|---|---|
| 锥桶位置 | padded observations 数组 | `x_coords`、`y_coords`、`z_coords`，同索引代表同一锥桶 |
| 颜色 | 黄 `0`、蓝 `1` | `ConeDetection.COLOR_YELLOW` / `COLOR_BLUE`，当前值 `1` / `2` |
| 空锥桶帧 | padded count / arrays | `num_cones == 0` 可同时意味着新鲜空帧；另看 availability |
| 地面真值 | `info` / simulator state | 车上没有对应的真实位置 `info` |
| 轮速 | 仿真状态 | `wheel_speed_in_meters_per_second`，无符号；`None` 不是测得 0 |
| 原始 LiDAR | 仿真固定扫描配置 | `lidar_scan`、`lidar_ranges`、`angle_min` / `angle_increment` |
| 车体 LiDAR 点 | 取决于仿真接口 | 当前源码有 `lidar_points_xyz` / `lidar_cartesian_available` |
| 计时 | 仿真调用方式 | `dt`、`is_first_policy_step`；首次 `dt == 0.0` |
| 动作出口 | notebook 返回动作数组 | 在插入区赋值 `drive_action`、`steering_action`，框架负责返回/发布 |
| 相机 pan | 取决于模型 | `camera_pan_action=None` 保持当前目标 |

车体坐标为 `base_link`：+x 向前、+y 向左、+z 向上；距离 m、角度 rad，正向转向约定向左。

### 4.2 六个必须核对的移植陷阱

1. **颜色。** 不复制 notebook 的裸数字筛选条件，否则蓝/黄边界可能反转。当前接收逻辑遇到不支持颜色会拒绝整批锥桶。
2. **动作。** `drive_action` 是归一化电机努力量，不是 m/s；`steering_action` 不是轮角 rad。赋值后交给框架，别把 notebook 的 `return np.array(...)` 塞进插入区。
3. **转角映射。** 课程教学示例用“rad 除以自选转角限值”演示归一化。当前 MPC 有 gain / offset / sign 映射，必须沿用并实测，不能额外再归一化一次。零 action 映射到 offset；软件零命令不证明车轮物理零角。
4. **LiDAR。** 原始第 i 束角度是 `angle_min + i * angle_increment`，在雷达原始 frame；中间索引不保证向前。当前源码已增加车体 Cartesian 点，若用它做决策，应检查 `lidar_cartesian_available` 并声明对应 required sensor。
5. **时间和状态。** 首步不能除以 0；积分器和控制器要在显式启动/恢复时重置。不能在一次策略 step 内 sleep、等待或运行无界循环。普通有限次数数值循环与阻塞等待不同。
6. **物理响应。** 仿真 `drive=-1` 的刹车/方向锁行为和无 dead zone 假设不能直接移植。负 drive、死区、滑行距离和轮速延迟要实测；当前 MPC shadow 不进行这些动力试验。

### 4.3 当前源码与教学摘要的差异

- 第 2a 页使用旧的 `lidar["ranges"]` 写法；当前仓库名称为 `lidar_scan`，同时提供 Cartesian 表示。
- required sensor 的原始雷达名称为 `lidar_scan`，车体点为 `lidar_cartesian`；触发模式仍可叫 `lidar`。
- 当前 MPC 需要 `[cone_detections, wheel_speed, imu_angular_velocity]`。课程的“IMU 可选”是泛指策略；本 MPC 的 IMU 是必需。
- 课程插入区示例不是现有 MPC 的替代实现，不使用其中示例 steering limit / drive command 作为实车已标定数值。
- 最准确的接口以本次部署的 `policy_node.py` 注释、YAML 和兼容消息定义为准。

## 5. 上车前准备：本机部署副本、证据和停车状态

### 5.1 建立本次本机目录

以下继续使用第 2 节已设置的 `$taskCarIp`；若换了 PowerShell 窗口，要重新设置自己的车 IP。

**本机 PowerShell：**

```powershell
cd E:\26b\AI4R\Baby-Bus
$taskSourceSha = (git rev-parse HEAD).Trim()
$taskStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$taskDeployDir = Join-Path (Get-Location).Path '.verification\car-deploy-shadow'
$taskEvidenceDir = Join-Path 'E:\26b\AI4R\car-evidence' $taskStamp
New-Item -ItemType Directory -Force -Path $taskDeployDir, $taskEvidenceDir | Out-Null
Copy-Item -LiteralPath 'scripts\policy_node.py' -Destination (Join-Path $taskDeployDir 'policy_node.py')
$taskSourceSha | Set-Content -Encoding utf8 -LiteralPath (Join-Path $taskEvidenceDir 'source-sha.txt')
git status --short | Set-Content -Encoding utf8 -LiteralPath (Join-Path $taskEvidenceDir 'source-status.txt')
$taskCarIp | Set-Content -Encoding utf8 -LiteralPath (Join-Path $taskEvidenceDir 'car-ip.txt')
```

`.verification/` 已在仓库 ignore 中。证据目录在仓库外；不把密码放进去。

### 5.2 在本机生成 DREAM 读取的完整 shadow YAML

DREAM 主线读取车上的 `config/ai4r_policy.yaml`。只 SCP 一个 MPC overlay 文件，不证明 DREAM 会使用它。本指南将基础 YAML 与 prototype overlay 在本机合并，生成完整部署副本，保留仓库默认 `mpc.enabled=false`。

**位置：本机 WSL，使用已有 MPC Python 环境。** 这里读取 Windows 仓库，保证部署副本来自同一份 Windows 源码：

```bash
source ~/ai4r-python/mpc-ros/bin/activate
cd /mnt/e/26b/AI4R/Baby-Bus

python - <<'PY'
from copy import deepcopy
from pathlib import Path
import yaml

root = Path.cwd()
key = '/**/ai4r_policy'
base = yaml.safe_load((root / 'config/ai4r_policy.yaml').read_text(encoding='utf-8'))
overlay = yaml.safe_load((root / 'config/ai4r_policy_mpc_prototype.yaml').read_text(encoding='utf-8'))

def merge(target, changes):
    for name, value in changes.items():
        if isinstance(value, dict) and isinstance(target.get(name), dict):
            merge(target[name], value)
        else:
            target[name] = deepcopy(value)

merge(base, overlay)
p = base[key]['ros__parameters']
p['mpc'].update(enabled=True, shadow=True, reference_source='planning',
                vehicle_params_source='course_simulation',
                v_exec_max_mps=0.0, bypass_acknowledged=False)
assert p['vehicle']['valid'] is False
assert p['required_sensors'] == ['cone_detections', 'wheel_speed', 'imu_angular_velocity']
assert p['planning']['vehicle_limits_source'] == 'course_simulation'
assert p['mpc']['shadow'] is True

out = root / '.verification/car-deploy-shadow/ai4r_policy.yaml'
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(yaml.safe_dump(base, sort_keys=False), encoding='utf-8')
print('Generated:', out)
print('MPC:', {n: p['mpc'][n] for n in ('enabled', 'shadow', 'reference_source',
      'vehicle_params_source', 'v_exec_max_mps')}, 'vehicle.valid:', p['vehicle']['valid'])
PY
```

这是本机生成文件，不在车上编辑。不改 `traxxas_vehicle_interface.yaml` 或其他硬件配置。YAML dump 会移除副本注释；带注释的仓库基础 YAML 保持原样。

**配置冲突需要明确：**基础 YAML 把 `course_simulation` 标为离线假设并警告不要作为实车限制；MPC prototype G2 又使用它生成 shadow 参考。本指南仅按 prototype 的“未 Enable、零应用动作”实验范围准备该配置，并要求现场负责人确认；`simulation_only` 不能作为动力执行许可或实测车辆能力证明。

### 5.3 本次部署前确认车辆停用

从 Foxglove 或现场控制确认车辆未 Enable。若现场已有运行中的 ROS 策略和车辆接口，**车上 SSH**可用：

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
ros2 topic pub --once /car/request std_msgs/msg/UInt8 '{data: 0}'
ros2 topic echo --once /car/traxxas_state
```

第一条让策略持续发布零动作，第二条请求车辆 Disable；读取一次状态不等于等到 Disabled，必须实际看清状态。若相关 units 未运行，先通过现场人员确认车辆状态，不靠没有回应的命令推断停车。

课程特别指出策略 restart 不会自动 Disable 车辆。未确认停用时不进行本次部署和手推测试。

## 6. MPC G1：车载依赖和离线计时

### 6.1 先做只读依赖检查

**车上 SSH：**

```bash
python3 -B -c "import sys, numpy, scipy, osqp, yaml; print('Python:', sys.executable); print('NumPy:', numpy.__version__); print('SciPy:', scipy.__version__); print('OSQP:', osqp.__version__); print('PyYAML:', yaml.__version__)"
```

确认该 Python 与 DREAM 策略实际使用的解释器一致；不能把本机 venv 的结果代替车上结果。已运行的节点可先检查进程命令行，配合 demonstrator 确认启动环境；仅看到入口 shebang 不能证明实际包版本。

缺 `osqp` / `scipy` 或版本不兼容时：保存报错，交给 demonstrator / 系统负责人。**按课程规则不在车上自行安装软件。**此前 MPC 文档提到的 `pip3 install osqp` 不适用于本课程提供的安装禁令。

### 6.2 使用独立的源码快照跑测试

测试放在独立目录，避免把活动 workspace 的完整 shadow YAML 当成仓库默认配置测试。`test_mpc.py` 的配置检查需要原始基础 YAML；如果先把它改成 enabled=true，再测，会产生配置契约失败。

**本机 PowerShell，仍在 Baby-Bus 根目录：**

```powershell
$taskRemoteCheck = "ai4r_mpc_check/$taskStamp"
ssh "ai4r@$taskCarIp" "mkdir -p ~/$taskRemoteCheck"
scp -r scripts config tests "ai4r@${taskCarIp}:~/$taskRemoteCheck/"
$taskSourceSha | Set-Content -Encoding utf8 -LiteralPath (Join-Path $taskEvidenceDir 'snapshot-sha.txt')
scp (Join-Path $taskEvidenceDir 'snapshot-sha.txt') "ai4r@${taskCarIp}:~/$taskRemoteCheck/source-sha.txt"
Write-Host "车上测试目录：~/$taskRemoteCheck"
```

这里的 `config/` 只进入测试快照，不进入活动 student workspace，不改变驱动配置。不传 `.git`、Windows venv、`.verification/ws`、build/install/log。

**车上 SSH：**把下一行的 `<本次时间戳>` 替换为 PowerShell 刚输出的目录名：

```bash
cd ~/ai4r_mpc_check/<本次时间戳>
cat source-sha.txt
uname -a
```

先验证离线源码选择和求解器依赖：

```bash
env -u PYTHONPATH python3 -B -c "import importlib.util, numpy, scipy, osqp, yaml; s=importlib.util.find_spec('ament_index_python'); print('ament_index_python:', s); assert s is None, '停下：当前环境仍选择安装版源码，需核对测试环境'"
```

通过后：

```bash
env -u PYTHONPATH python3 -B -m unittest \
  tests/test_mpc.py tests/test_planning.py tests/test_estimation.py \
  2>&1 | tee offline-tests.log
task_mpc_test_rc=${PIPESTATUS[0]}
printf '%s\n' "$task_mpc_test_rc" > offline-exit-code.txt
printf 'exit code: %s\n' "$task_mpc_test_rc"
```

`env -u PYTHONPATH` 只影响该命令，让这组数值测试读取测试快照。若它也隐藏了负责人部署的求解器依赖，或仍能找到 ament，先核对正式解释器/依赖方案，不人为 mock 导入来得到 PASS。

### 6.3 G1 通过条件

| 看什么 | 条件 |
|---|---|
| 导入 | 节点实际 Python 能导入 NumPy / SciPy / OSQP / PyYAML |
| 套件 | exit code 0，最后 `OK` |
| 完整 MPC step | P95 明显低于 50 ms，超预算 0/200 |
| 闭环数值 | 与本机相同场景结果接近，无明显数值异常 |
| OSQP 超时 | 确认目标版本接受 `time_limit`，并记录目标平台限制生效证据 |

计时测试的打印标题写死为 `laptop, not the target platform`；日志必须另外注明此次实际是 Jetson。该测试包含 step 计时，但正常场景通过不等于已经测出强制超时效果，也不覆盖全部 ROS callback/调度延迟。

当测试失败或超预算：记录第一处失败、机器和版本。调整 horizon / 迭代等属于新配置试验，应先本机验证再部署，不能删测试或改门槛消除失败。

离线测试日志稍后复制回本机，车上不是长期证据库。

## 7. 第 3 页：SCP → Build → Restart

### 7.1 检查车上目标目录

**车上 SSH：**

```bash
ls -ld ~/ai4r_student_workspace/src/ai4r_policy
ls -l ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py
ls -l ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml
```

若不存在，交给 demonstrator 确认 workspace 初始化，不猜目录、不把本机整套 ROS workspace 复制进去。

### 7.2 保留旧学生文件到本机

**本机 PowerShell：**

```powershell
scp "ai4r@${taskCarIp}:~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py" (Join-Path $taskEvidenceDir 'previous-policy_node.py')
scp "ai4r@${taskCarIp}:~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml" (Join-Path $taskEvidenceDir 'previous-ai4r_policy.yaml')
```

备份失败就先核对路径；已有文件可能来自别的团队，部署前按共享车约定协调。恢复也应使用已确认的旧文件、build 和 restart，不能只关掉 Foxglove认为恢复完成。

### 7.3 传本次的两个活动文件

**本机 PowerShell：**

```powershell
scp (Join-Path $taskDeployDir 'policy_node.py') "ai4r@${taskCarIp}:~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py"
scp (Join-Path $taskDeployDir 'ai4r_policy.yaml') "ai4r@${taskCarIp}:~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml"

Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $taskDeployDir 'policy_node.py')
Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $taskDeployDir 'ai4r_policy.yaml')
```

**车上 SSH：**

```bash
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py
sha256sum ~/ai4r_student_workspace/src/ai4r_policy/config/ai4r_policy.yaml
```

比较本机与车上两项 hash，字母大小写不影响比较。相同说明传输内容匹配；运行时是否加载它仍需 restart 和参数检查。

课程给出的 `scp -r scripts config ...` 适用于整套配置都应更新的情况。本次只传策略和策略参数，避免覆盖车上硬件 YAML。单独复制 `.py` 也不能代替新版本所需 launch / package 文件；本次若这些文件与车上版本不兼容，由集成负责人完成包级同步。

### 7.4 构建

**车上 SSH，任意工作目录：**

```bash
dream build ros student
```

命令成功后才继续。build 失败不等于旧运行策略已停止，也不意味着新代码可用。

`dream build ros student --clean` 是课程的特殊恢复入口，不作为每次操作默认步骤；只有确认构建问题确需 clean 并协调当前 workspace 后使用。

### 7.5 稍后启动或重启

如果策略已运行：

```bash
dream runtime restart ai4r_policy
```

如果尚未运行，先按下一节启动所需 units，最后 `dream runtime start ai4r_policy`。

重复 `start` 不能保证加载新源码/配置；本次更新用 `restart`。新进程回到策略 state 2；车辆本身可能仍保留先前的 Enabled 状态，所以部署前已要求显式停用。

## 8. 第 4 页：启动所需 units，检查 shadow

### 8.1 检查 catalog 和状态

**车上 SSH：**

```bash
dream runtime catalog
dream runtime status
```

本 MPC 的 required sensors 是锥桶、轮速、IMU angular velocity。因此需要：

| Unit | 作用 | 本轮 |
|---|---|---|
| `foxglove_bridge` | 本机查看和控制入口 | 需要 |
| `traxxas_vehicle_interface` | 状态、轮速、动作接口 | 需要，但不 Enable |
| `oakd_cone_detector` | 锥桶批次 | 需要 |
| `bno08x_imu_interface` | angular velocity | 需要 |
| `ai4r_policy` | 估计、规划和 MPC | 需要 |
| `rplidar_c1` | 雷达 | 本配置未列为必需；若本次策略实际依赖则另外启动并声明 |

### 8.2 启动

**车上 SSH：**

```bash
dream runtime start foxglove_bridge
dream runtime start traxxas_vehicle_interface
dream runtime start oakd_cone_detector
dream runtime start bno08x_imu_interface
dream runtime start ai4r_policy
dream runtime status
```

已运行的策略若刚更新过文件，必须使用上节 `restart` 加载更新。启动其他 units 不需要无理由全车重启。

### 8.3 运行时参数必须实查

```bash
ros2 topic info /car/drive_and_steer_set_point_normalized
ros2 param get /car/ai4r_policy required_sensors
ros2 param get /car/ai4r_policy mpc.enabled
ros2 param get /car/ai4r_policy mpc.shadow
ros2 param get /car/ai4r_policy mpc.reference_source
ros2 param get /car/ai4r_policy planning.vehicle_limits_source
ros2 param get /car/ai4r_policy mpc.vehicle_params_source
ros2 param get /car/ai4r_policy vehicle.valid
ros2 param get /car/ai4r_policy mpc.v_exec_max_mps
```

预期：恰好一个动作发布者；三项 required sensors；MPC enabled=true、shadow=true、source=planning、limits source=course_simulation；两项 verified=false，执行速度 cap=0.0。

两个发布者时先定位并消除重复策略进程。课程使用 DREAM 管理，本次不同时再开手动 `ros2 launch`。

### 8.4 传感器与车辆状态

```bash
ros2 topic hz /car/wheel_speed_m_per_sec
```

观察后 `Ctrl+C`。轮速 topic 静止时也应持续发消息，手推时读数应变化。

分别检查（每条看完后 `Ctrl+C`）：

```bash
ros2 topic hz /car/cone_detections
ros2 topic hz /car/imu/data
ros2 topic echo /car/policy_fsm_state_string
ros2 topic echo /car/traxxas_state
```

IMU topic 有消息不等于 angular velocity 字段有效；实际状态拒绝原因和字段检查仍需看策略日志。车辆必须未 Enable。

## 9. 第 5 页：Foxglove 连接与界面检查

### 9.1 从自己的电脑连接

在自己的电脑打开课程链接的 Foxglove Web App，登录后：

1. 选择 **Open connection**。
2. 数据源选择 **Foxglove WebSocket**。
3. 地址填 `ws://<自己的车IP>:1234`；把占位符替换为第 2 节的实际 IP。
4. 保持 GlobalProtect 连接。
5. **Layouts → Import from file…**，导入本机文件：

```text
E:\26b\AI4R\Baby-Bus\docs\AI4R_Foxglove_UI_v2026-09-22.json
```

课程快照说明账号会保存布局。不同 UI 版本菜单可能变化，仍以实际按钮功能和发送的状态请求确认。

### 9.2 两套按钮的对照

| 按钮 | ROS 请求 | 意义 |
|---|---|---|
| Policy publishing actions | `policy_fsm_transition_request` UInt16 `3` | 执行策略算法 |
| Policy publishing ZERO actions | 同 topic UInt16 `2` | 持续发布 drive/steer 零动作，正常停止策略动作 |
| Policy NOT publishing actions | 同 topic UInt16 `1` | 完全停止动作发布；不是正常停车按钮 |
| Vehicle Request Enable | `/car/request` UInt8 `1` | 请求车辆接受 Host 命令 |
| Vehicle Request Disable | `/car/request` UInt8 `0` | 车辆输出 neutral，不管策略正在请求什么 |
| Vehicle Request Disarm | 课程 UI 的 Disarm 请求 | 返回手动控制；原文未给出数值，不猜命令编码 |

**当前 shadow 手推：不按 Vehicle Request Enable。只在检查完参数和传感器后按 Policy publishing actions。**

### 9.3 面板检查顺序

| 面板 | 本轮看什么 |
|---|---|
| Vehicle state | 确认未 Enabled；实际状态可能为 Disarmed/Disabled |
| Policy state | 初始 Publishing zeros；运行 shadow 时 Publishing policy actions |
| RC drive / RC steer | RC 是否可用；BAD 时检查遥控器和链路 |
| Control source | Neutral / RC / Host；结合车辆状态理解，不靠策略状态猜 |
| Cone locations | 真实蓝/黄边界、+x 前/+y 左、中心位置 |
| Drive / steer commands | 策略请求和车辆应用输出；shadow 应用动作必须零 |
| Wheel speed | 手推读数；无符号、低速更新慢、停车衰减延迟 |
| Heading angle | 进入策略 state 3 时 tare 的相对 heading |
| IMU | angular velocity 的有效性与变化 |
| `debug1` | 当前 MPC 横向误差 `e_y`，m |
| `debug2` | MPC 候选轮角 `candidate_delta`，rad，不是已应用转向 action |

课程通用布局未保证包含 `mpc_debug`。添加 Raw Messages 面板订阅 `/car/mpc_debug`；它是 String，内容为 JSON。现成 Plot 可直接画数值 `debug1` / `debug2`，JSON 内字段需要解析后才能按数值绘制，不能当作原生消息字段。

### 9.4 图像只在需要排查时开启

**车上 SSH：**

```bash
dream runtime restart oakd_cone_detector --debug-images annotated
```

课程列出的可选值为 `off`、`annotated`、`depth`、`both`。图像占 WiFi 带宽，并影响时序；记录本轮是否开启。

排查结束后：

```bash
dream runtime restart oakd_cone_detector
```

课程说明普通 restart 恢复图像 off。检测器 restart 会暂时中断 required cones，可能让策略进入停止状态；等输入恢复后再显式请求 state 3。

关闭 Foxglove 或 SSH 断线不能作为停止策略的方法。DREAM 管理的进程可能继续运行。

## 10. MPC G2：先录包，再手推

### 10.1 开两个以上 SSH 终端

每个都从本机运行 `ssh ai4r@<自己的车IP>` 并使用车辆 ROS 环境：一个录包，一个看 debug，一个发状态请求/看状态。不要把 WSL 测试 domain 218/219 设置复制到车上。

先确认车辆未 Enable、应用动作全零、RC 在手、测试是短直线锥桶道路。

### 10.2 录包

**车上 SSH，录包终端：**

```bash
mkdir -p ~/ai4r-car-bags
cd ~/ai4r-car-bags
ros2 bag record \
  /car/mpc_debug \
  /car/debug1 \
  /car/debug2 \
  /car/policy_fsm_state_string \
  /car/drive_and_steer_set_point_normalized \
  /car/wheel_speed_m_per_sec \
  /car/cone_detections \
  /car/imu/data \
  /car/traxxas_state
```

等录包初始化成功，再开始试验。记下实际生成的 bag 目录名。没有 `ros2 bag` 子命令时找负责人，不在车上自行安装。

### 10.3 请求策略 state 3

**车上 SSH，控制终端：**

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 3}'
```

**车上 SSH，观察终端：**

```bash
ros2 topic echo /car/mpc_debug
```

如果没有进入 state 3：看 `/car/policy_fsm_state_string`、required sensors、runtime logs。不要为了通过检查移除必需轮速/IMU。恢复需要显式请求 state 3，不会自动恢复。

### 10.4 固定场景和实测姿态

先用卷尺/角度参照记录位置，再轻推向前产生连续轮速历史。当前估计器/规划器依赖运动对齐，静止摆放后不保证马上有有效参考。

航向测试先选小角度，例如左右约 5°（约 0.087 rad），并保持道路可见。当前 Planning 的 heading-error 上限为 0.15 rad、lateral-offset 上限为 0.2 m；超过范围可能正常拒绝参考，不能把这种拒绝直接判为 MPC 符号错误，也不为得到样本临时放宽门槛。

| 场景 | 实测条件 | 预期 `e_y_m` / `e_psi_rad` | 预期候选轮角 |
|---|---|---|---|
| A 居中平行 | 左右距离接近一致、车头沿路 | 都接近 0 | 接近 0 |
| B 左偏平行 | 车比中心线左约 0.10 m | `e_y ≈ +0.10`，heading error 接近 0 | 通常负，向右修正 |
| C 右偏平行 | 车比中心线右约 0.10 m | `e_y ≈ -0.10`，heading error 接近 0 | 通常正，向左修正 |
| D 居中左偏航 | 有记录的向左 heading error | `e_psi > 0` | 通常向右修正 |
| E 居中右偏航 | 有记录的向右 heading error | `e_psi < 0` | 通常向左修正 |

方向判断针对单独横向/航向误差的场景；两项误差同时存在时不能要求候选角分别与两项都反号。比较同一场景的连续样本，考虑转向速率限制和上一候选状态，不以单点瞬时值代替判断。

### 10.5 日志字段怎么读

| 字段 | 期望 | 不符合时 |
|---|---|---|
| `branch` | 有效参考时主要 `shadow` | 收集 `ref_invalid` 的理由；`solver_fail` / `over_budget` 必须零 |
| `reject_reason` | 正常样本无拒绝 | 分别统计 stale、alignment、forward coverage 等原因 |
| `applied_drive` / `applied_steer_action` | 始终为 0 | 立即 state 2，保持车辆停用并查原因 |
| `candidate_delta_rad` | 单独误差场景应有修正趋势 | 检查估计器、坐标和模型符号 |
| `candidate_steer_action` / `candidate_drive` | 符合 course_simulation 映射；驱动 ≥ 0 | 未标定之前不能证明物理轮角或速度响应正确 |
| `step_s` / `solve_s` | 完整 step P95 < 0.05 s | 保存耗时和版本，回本机调整与验证 |
| `dt` | 约 cone batch 周期，通常约 0.1 s | 大间隔/抖动尤其 >0.2 s 需解释 |
| `ref_age_s` | 小于有效期 | 排查原始时戳、延迟、运动对齐 |
| `delta_est_rad` / `vehicle_source` | shadow 下为偏置角（应用动作恒为 0）；`course_simulation` | 由已应用指令按模型回放得到，不是测得的轮角 |
| `simulation_only` | 当前 Planning shadow 为 true | 确认未把模型假设当实测能力 |
| `planning_bypassed` / `reference_source` | 不 bypass、`planning` | 核对是否加载错 overlay |

shadow 内 MPC 拒绝通常只记录，不锁停策略；required sensor 失效仍会由外层框架停止。不要因为它没有锁停就忽略大比例无效参考。

### 10.6 G2 通过条件

- 横向/航向误差符号正确，量级与卷尺/角度记录相符。
- 分别给出有效参考比例，以及各类 `ref_invalid` 比例和原因。
- solver_fail、over_budget 为零；耗时满足预算。
- 手推轮速有读数；算法运行期间策略状态稳定，required sensors 健康。
- 所有应用动作零；车辆未 Enable。
- bag、配置、源码版本、场景姿态和失败记录均保留。

课程/prototype 没给出一个可直接套用的 `ref_invalid` 百分比阈值。不能自行写成“少于 5% 就通过”；应根据场景和团队约定预先确定并解释。

## 11. 结束、回收数据与下一轮修改

### 11.1 正常停止顺序

**车上 SSH：**

```bash
ros2 topic pub --once /car/policy_fsm_transition_request std_msgs/msg/UInt16 '{data: 2}'
ros2 topic pub --once /car/request std_msgs/msg/UInt8 '{data: 0}'
ros2 topic echo --once /car/traxxas_state
```

确认状态，再在录包终端按 `Ctrl+C` 完成写盘。需要时使用课程 Foxglove Disarm 按钮交回手动 RC。软件零动作/neutral 不证明车已物理停稳，现场确认。

### 11.2 查询日志和 bag

```bash
dream runtime status ai4r_policy
dream runtime logs ai4r_policy --tail 100
ros2 bag info ~/ai4r-car-bags/<本次实际bag目录>
```

课程说明 runtime logs 没有 live follow；需要重新查询，不照搬 `--follow`。

**本机 PowerShell：**把 `<本次实际bag目录>` 替换为录包的真实目录名：

```powershell
scp -r "ai4r@${taskCarIp}:~/ai4r-car-bags/<本次实际bag目录>" $taskEvidenceDir
scp "ai4r@${taskCarIp}:~/$taskRemoteCheck/offline-tests.log" (Join-Path $taskEvidenceDir 'jetson-offline-tests.log')
scp "ai4r@${taskCarIp}:~/$taskRemoteCheck/offline-exit-code.txt" (Join-Path $taskEvidenceDir 'jetson-offline-exit-code.txt')
ssh "ai4r@$taskCarIp" 'dream runtime logs ai4r_policy --tail 100' | Set-Content -Encoding utf8 -LiteralPath (Join-Path $taskEvidenceDir 'runtime-policy.log')
Copy-Item -LiteralPath (Join-Path $taskDeployDir 'ai4r_policy.yaml') -Destination (Join-Path $taskEvidenceDir 'deployed-ai4r_policy.yaml')
Copy-Item -LiteralPath (Join-Path $taskDeployDir 'policy_node.py') -Destination (Join-Path $taskEvidenceDir 'deployed-policy_node.py')
```

导出是否成功、bag 是否完整要实查，不以运行过 scp 命令代替。数据删留按共享车约定处理，本指南不提供批量删除车上目录的命令。

### 11.3 下一轮修改循环

```text
本机编辑 → 本机相关测试/最终 ROS gate → 固定版本和部署副本
→ 确认车辆停用 → SCP → DREAM build → restart ai4r_policy
→ 参数/状态复查 → 新一轮 shadow 或已批准执行 → 保存全部证据
```

关闭 Foxglove不是停止，SCP不是构建，build不是加载，restart不是 Enable，也不是运行策略 state 3。

## 12. G3 / G4：何时才进入动力执行

本节用于计划，不在当前 shadow 流程自动执行 Enable。

### 12.1 G3 要补的实际测量

| 项目 | 结果写入/记录 |
|---|---|
| 小范围左右 steering action 与真实轮角，含方向和零点 | gain、offset、sign、steering limit |
| 转向步进耗时/速率 | steering rate limit，并与 Traxxas slew 约束兼容 |
| 测试速度对应 drive、dead zone、速度环响应 | drive_max、执行速度 cap、PI/PID 参数 |
| state 2 后滑行距离和停稳时间 | 保守制动/停车能力、轮速衰减延迟 |
| 约 0.5 s command timeout 的实际效果 | 单独的现场记录，不当作正常停车 |

prototype 提到 vehicle-identification 分支的 `id_test`。该分支及 README 不在本次用户提供的课程附件里；不能凭本指南补造其参数和执行步骤。向团队获取经过确认的识别流程；识别策略替代当前策略，不并行运行。

原型明确尚无“转向保持零的速度 PI 单独测试”入口。该验证缺口要记录和解决/获得现场认可，不能把未实施项目写成通过。

### 12.2 执行门槛

依据 prototype，执行需要 shadow=false、`mpc.vehicle_params_source: vehicle`、`vehicle.valid: true`（全部字段实测并填写 source 标签，见 [VEHICLE_PARAMS_INTEGRATION.md](VEHICLE_PARAMS_INTEGRATION.md)）和正的 v_exec_max_mps。Planning-bypass 还需 bypass_acknowledged，并明确缺失 clearance / stopping / speed cap 检查。本次主线使用 Planning 直线，不自动切换 bypass。

满足软件 flags 仅代表程序 gate 接受配置，不代替课程现场安排、实测参数或物理验收。`course_simulation` 的使用限制和仍未测量的参数由负责人明确解决。

### 12.3 动力执行的顺序（仅在 G3 和现场条件满足后）

1. 预先记录短直路线、起始姿态、目标速度、停止距离、接管人和通过门槛。
2. 本机准备执行配置，传输、build、restart；复查实际参数。
3. 确认策略 state 2 持续发送新鲜零动作。
4. Foxglove **Vehicle Request Enable**，看清 **Enabled** 后再继续。
5. **Policy publishing actions** / state 3，先做一次方向和速度环检查。
6. 若检查正常，做文档要求的三次连续运行；干预、碰锥或离路记失败，保留全部尝试。
7. 正常停止：**Policy publishing ZERO actions → Vehicle Request Disable**；必要时 RC 接管。
8. 执行模式拒绝会锁回 state 2。查明原因后显式重启 state 3，控制器重新建立，不自动恢复。

课程 CLI 的 Enable 请求为：

```bash
ros2 topic pub --once /car/request std_msgs/msg/UInt8 '{data: 1}'
ros2 topic echo /car/traxxas_state
```

`echo --once` 只能取一条消息，不保证那条已为 Enabled；必须读到目标状态。只有此时才发送策略 state 3。完整 G4 记录见 [MPC_PROTOTYPE.md](MPC_PROTOTYPE.md)。

## 13. 按实际报错排查

| 现象 | 先查 | 处理方向 |
|---|---|---|
| SSH / Foxglove 超时 | 自己的车 IP、VPN、UniWireless、上电 | 网络/bridge，找 demonstrator；不尝试邻车 |
| SCP No such file or directory | 本机当前目录、源文件、远端 student workspace | 核实路径，不反复覆盖猜测目录 |
| 缺 SciPy / OSQP | 节点解释器与依赖路径 | 负责人供给依赖，不在车上 pip/apt |
| `ai4r_policy` package not found | ROS underlay vs student overlay、测试源码选择 | 离线快照用上述探针；ROS运行需正确 build/load |
| 表现像旧代码 | 本机与车上 hash、build、restart、参数 | 按证据定位哪一步没有完成 |
| MPC enabled 仍为 false | DREAM 读取的完整源 YAML | 单独 overlay 没被使用；重新核对部署副本 |
| required IMU 缺失 | IMU unit、angular velocity 字段、freshness | 本 MPC 不能跳过 IMU 或用无效 0 冒充 |
| state 3 立刻停 | policy state string、required sensors、异常日志 | 修复后显式请求 state 3，不自动恢复 |
| 大量 ref_invalid | reject_reason、源时间、alignment、coverage、车速 | 保存比例与姿态/输入，不能仅靠加长 freshness 掩盖 |
| 方向反了 | 颜色常量、+y 左、e_y/e_psi定义、steering映射 | 保持停用，回本机查代码与数据 |
| Foxglove空面板 | bridge之外的 units、正确车IP/布局 | runtime status，核对数据 topics |
| 图像黑 | debug images默认关闭 | 需要时 annotated；影响时序需记录 |
| Enable无反应（未来执行） | 新鲜零动作、RC模式、车辆状态 | 先 state 2；不是反复发非零动作 |
| 重启后仍 Enabled | 车辆与策略是两个状态机 | 显式 Disable；restart不代替停用 |
| 小 drive没动（未来标定） | 死区、RC控制源、Enabled与实际输出 | 按识别方案测量；不随意大幅加油 |

向 demonstrator/Ed反馈时提供：自己的车号、确切命令、完整错误、源码/配置版本和本次日志。不要提供密码。

## 14. 本次验证记录模板

```text
日期/时区：
团队、车号、IP、操作者、RC接管人：
本机部署来源分支/完整SHA：
相关工作区修改（若dirty）：
部署.py/YAML本机与车上SHA256：
Jetson/OS/ROS/Python：
NumPy/SciPy/OSQP/PyYAML版本、实际节点解释器：
本机离线/ROS gate日志：
Jetson离线测试exit code：
Jetson step mean/P95/max、over-budget：
OSQP time_limit确认方法/结果：
DREAM build/restart结果、参数实查：
车辆状态、策略状态、required sensors：
图像debug是否开启：
场景A～E的实测偏移/角度、误差和候选动作：
有效参考比例、各reject_reason比例：
solver_fail/over_budget次数：
应用动作是否始终零：
bag/运行日志/部署文件的本机归档位置：
通过/失败/未运行项目：
G3/G4动力测试：not run（或对应单独记录）
```

## 15. 来源、差异与修正索引

### 15.1 用户提供的课程页面

- 用户粘贴的《System Project — DREAM Car Setup & Development Reference》总指南：本机编辑 → 传输 → build → 运行 → 观察；正文未给出总页自身 URL，因此不补造链接。
- [1. SSH and VPN Setup](https://canvas.lms.unimelb.edu.au/courses/238681/pages/system-project-docs-1-ssh-and-vpn-setup)：用户直接粘贴全文；Jetson型号、网络、IP、用户、操作限制。
- [2. Clone and Edit Your Policy](https://canvas.lms.unimelb.edu.au/courses/238681/pages/system-project-docs-2-clone-and-edit-your-policy)：附件 `c032ffb4-2233-4e2d-920e-d6b36c14e5cf` 的粘贴文本；开发接口和本地编辑。
- [2a. Pass Your Notebook Policy to the Car](https://canvas.lms.unimelb.edu.au/courses/238681/pages/system-project-docs-2a-pass-your-notebook-policy-to-the-car)：附件 `effe56b0-299d-4e7b-b691-36c505c10a98`；颜色、动作、LiDAR、时间、制动差异。
- [3. Transfer and Build](https://canvas.lms.unimelb.edu.au/courses/238681/pages/system-project-docs-3-transfer-and-build)：附件 `f5d619fa-fc78-4666-92cf-882772ce5af8`；共享workspace、scp、DREAM build；原文标为draft。
- [4. Run the Car](https://canvas.lms.unimelb.edu.au/courses/238681/pages/system-project-docs-4-run-the-car)：附件 `1134db7b-5a84-419b-ba2f-69cae645c61c`；units、Enable/策略顺序、停止和restart。
- [5. Foxglove UI](https://canvas.lms.unimelb.edu.au/courses/238681/pages/system-project-docs-5-foxglove-ui)：附件 `15f8713d-1e9d-4b1a-9350-648760972a49`；WebSocket、布局、按钮、面板、debug images。

### 15.2 项目依据

- [policy_node.py](../scripts/policy_node.py)：实际输入、源码选择相关测试所用算法、MPC和动作框架。
- [基础参数](../config/ai4r_policy.yaml)与[prototype overlay](../config/ai4r_policy_mpc_prototype.yaml)：默认关闭、shadow、required sensors和simulation参数。
- [MPC_PROTOTYPE.md](MPC_PROTOTYPE.md)：G0～G4、日志、方向约定、限制。
- [ROS verification babysitter](ros-verification-babysitter.md)：本机WSL、依赖、gate和解释器排查。
- [CONTRIBUTING.md](../CONTRIBUTING.md)、[CMakeLists.txt](../CMakeLists.txt)、[接口pin](../ci/dependencies.repos)：软件验证入口和接口兼容性。

### 15.3 对此前建议的具体修正

| 此前不够准确/缺少前提 | 本指南采用 |
|---|---|
| Jetson型号和登录信息未确认 | 课程明确 Jetson Orin Nano、`ai4r`、车号IP规则 |
| 建议在车上 Git pull/clone | 本机保留Git，课程主线SCP到已有student workspace |
| 缺OSQP时可能pip安装 | 课程明确不自行安装，转交系统负责人 |
| 直接使用通用Enable→state3 | 当前G2保持未Enable，只运行shadow计算 |
| 以为restart会停用车辆 | restart策略回零，但车辆可保持Enabled；显式Disable |
| overlay复制即生效 | DREAM使用完整源YAML，本机合并、部署、参数实查 |
| 静止或混合误差时逐点要求反号 | 分别测试横向/航向误差，考虑运动对齐与速率状态 |
| 动力执行可直接套simulation limits | 明确未测量/配置冲突和负责人确认；G3/G4单独记录 |
| 安装包/参数加载代表真实MPC已验证 | 分别记录导入、模块测试、ROS gate、实际shadow和实车证据 |
