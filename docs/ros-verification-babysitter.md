# Baby-Bus ROS 检查保姆级操作手册

日期：2026-10-04。适用：Windows VS Code 开发、GitHub 功能分支协作、WSL2 Ubuntu 24.04 / ROS 2 Jazzy 验证，以及 MPC V0＋M0 计划。

本文是操作说明，不是通过报告。根据本次聊天，WSL 已能执行 gate，但最后展示的结果是 `Wrong dream_interfaces revision`；尚未收到修正后完整 gate 的结果，不能写成已通过。本文未在你的 WSL 或真实小车上执行安装、构建或验收。

## 当前采用的工作方式：固定一台电脑做 ROS 验证

本次已选择先固定 `DESKTOP-IUA0CF1` 上的 WSL Ubuntu 作为 ROS 验证环境。另一台电脑暂不要求安装或迁移 ROS 环境。

- 两台电脑都可以在 VS Code 编辑源码，向约定的 GitHub 功能分支 commit／push。
- 换电脑开始编辑前，先检查本地修改并同步同一分支，避免覆盖另一台已推送的工作。
- ROS 验证时，在 `DESKTOP-IUA0CF1` 的 `~/ai4r-dev/Baby-Bus` 拉取分支，核对 commit，按第 9 节运行相关检查和完整 gate。
- 记录验证机器、分支、commit、工作区状态与结果；之后又推送的相关源码改动需要重新验证。
- 验证依赖这台电脑可用。在另一台 push 代码不会自动触发本机 gate；当前尚未设置远程验证或 CI 自动运行。

## 0. 先知道自己现在该做哪一步

你已经有 `~/ai4r-dev/Baby-Bus`，不需要重新 clone。先按第 11.1 节完成精确依赖 checkout，再按第 7 节运行 gate。若已经成功，日常使用第 9 节。

从头准备另一台电脑时，按第 6 节安装，再按第 7 节验证。Windows 的 Dream Gym 环境按第 8.1 节准备。

本文代码块的 `powershell` 在 Windows PowerShell 中执行；`bash` 在 Ubuntu 终端中执行。不要把命令提示符里的 `$`、`>` 或用户名一起复制进去。命令报错时停在该步骤，先处理错误，不继续复制整段流程。

## 1. ROS 检查到底在检查什么

Baby-Bus 的 ROS 软件检查入口是仓库提供的 `tools/verify_fast.sh`，下文简称 fast gate。

它使用真正的 ROS 消息类型、节点、DDS 通信和定时器，配合合成传感器消息、受控时钟和测试节点，检查**安装后的策略软件是否遵守框架约定**。其中某些测试会实际 launch 策略节点，但不会启动物理设备驱动或车辆执行器。

可以理解为：给程序发送可控的假观测和状态请求，检查它什么时候计算、什么时候发动作、什么时候停止发布非零动作，以及安装与启动流程是否正确。

这不是“在电脑上开一辆完整虚拟小车”。测试节点没有真实车辆动力学、轮胎、相机、雷达或电机。

```text
合成 ROS 观测与状态请求
            ↓
安装后的 Baby-Bus 策略节点
            ↓
记录发布消息、状态、参数与启动结果
            ↓
断言实际软件行为符合预期
```

## 2. 必须分开的测试与证据

| 层次 | 使用环境 | 可以证明 | 不能由此推断 |
| --- | --- | --- | --- |
| 包能导入／小型 QP smoke | 对应 Python 环境 | 包可加载；指定小问题能求解 | MPC 跟踪正确、ROS 已接好、满足实时预算 |
| MPC／Planner 模块测试 | 开发 Python 环境 | 模型、参考、矩阵、约束、状态处理符合测试 | 完整仿真闭环成功、真实车辆表现 |
| Dream Gym／plant 闭环 | 仿真环境 | 所测道路、速度、扰动下的算法表现 | 实车坐标、动作映射、物理停车正确 |
| Baby-Bus fast gate | Ubuntu 24.04＋ROS Jazzy＋精确接口源码 | 安装后的策略框架、合成 ROS 行为与 launch 检查 | 学生 MPC／Planner 全部正确、物理停车或车架坐标准确 |
| MPC／Planner ROS 接入测试 | 同一 ROS 验证环境 | 真正学生算法与 ROS 输入／输出适配在所测情况下正确 | 小车硬件与真实道路表现 |
| 上车 build／load | 课程小车、兼容 DREAM underlay、学生 workspace | 本轮代码能在这台车构建／加载 | 算法达到跟踪目标、停车距离合格 |
| 日志回放／无动作试运行 | 对应运行平台 | 输入兼容、数值、数据年龄和候选结果处理 | 闭环成功率；回放输入不会响应控制动作 |
| 目标平台计时 T8 | 实际目标车载计算平台 | 测得的 setup、完整 step、抖动及超预算情况 | 其他电脑同样快；真实路线已完成 |
| M0 实车验收 | 匹配整车集成、真实路线、现场操作和记录 | 所测配置下的真实跟踪、停止与路线结果 | 更高速度、急弯、主动绕障均已合格 |
| 发布契约检查 | Git／Python 环境 | 发布与源码打包工具的所测契约 | 替代 ROS gate 或实车验收 |

WSL 可以作为软件验证环境候选，实际合格以该机器的完整 gate 结果为准。Windows、WSL 或云端耗时不能代替计划里的目标车载平台 T8 计时。安装依赖属于 provisioning；gate 本身不负责下载和安装。

## 3. 现有 fast gate 实际执行的步骤

依据当前 [verify_fast.sh](../tools/verify_fast.sh)：

1. 找到 Baby-Bus 根目录，以及 `AI4R_INTERFACES_SOURCE` 指定的接口仓库。
2. 从 [ci/dependencies.repos](../ci/dependencies.repos) 读取要求的 Git commit。
3. 比较依赖仓库 HEAD；不一致就报 `Wrong dream_interfaces revision` 并退出。
4. 用 `git diff --quiet HEAD --` 检查依赖的已跟踪内容是否改动；有改动则退出。
5. 清除继承的 ROS overlay、Python 路径和发现设置，加载 `/opt/ros/jazzy/setup.bash`。
6. 设置测试 domain `218`、localhost discovery，并取得本机文件锁，避免两次 gate 同时占用该测试域。
7. 在 `.verification/ws/src` 为 `dream_interfaces` 和 `ai4r_policy` 建立源码链接；检查链接和包数量。
8. `colcon build` 两个包，开启测试并使用 `--symlink-install`。
9. 加载刚构建的 overlay，运行 `ai4r_policy` 的测试，输出详细测试结果。
10. 检查安装后的 `ros2 launch ... --show-args`。
11. 全部成功后输出：

```text
verify-fast: PASS (synthetic ROS only; no physical hardware)
```

检查依赖时，本文另外要求查看 `git status --short`。脚本的 `git diff` 不会发现所有未跟踪文件，因此“脚本没有报修改”与“目录完全干净”不是完全一样的检查。

`--symlink-install` 有利于开发，但不能因此跳过最终 gate。构建、安装资源、消息定义和测试仍需针对最终版本核对。

## 4. 现有测试覆盖哪些行为

依据当前 [tests/test_policy_node.py](../tests/test_policy_node.py)，主要覆盖如下；测试数量会随版本和参数化变化，不把某个固定数字当成永久标准。

| 主题 | 现有测试内容举例 | 边界 |
| --- | --- | --- |
| 状态机与起始动作 | 启动零动作；不发布状态的含义；显式启动与恢复；状态切换时重置 | 不是车辆 Enable／Disarm 或电机响应测试 |
| 策略触发源 | cone、fiducial、lidar、timer 中选定的触发源才执行；重复时间戳处理 | 不是完整控制频率性能测试 |
| 观测有效性和新鲜度 | 必需传感器丢失／过期；空锥桶与缺流的不同期限；过期可选字段不可用；快照副本 | 不能证明真实传感器准确度 |
| 锥桶批次元数据 | 延迟是否合法；延迟已包含在观测年龄中；非法内容不刷新数据 | 不测 OAK-D 真正的检测质量与延迟 |
| ArUco／fiducial | 字段检查；有效／空批次；变换、缺失 TF、异常变换 | 不证明真实 marker 的识别与定位正确 |
| LiDAR | 无效射线过滤、索引保留、三轴变换、TF 失败、原始与 Cartesian 数据的区别及过期 | 不证明真实安装角度／位移已标定 |
| IMU | 安装旋转、四元数合法性、各字段有效性／新鲜度、heading 和 tare 行为 | 不证明真实航向精度 |
| 时间异常 | 旧时间戳、时钟回退、状态重置 | 不代替目标平台的真实抖动统计 |
| 学生函数返回处理 | NaN／Inf／错误类型；幅度裁剪；pan hold；异常与返回后超预算处理 | 不证明任意阻塞求解都能及时停止 |
| 参数与配置安装 | 非法启动参数、参数变更限制、安装 YAML、namespace 和加载 | 不运行对应真实硬件节点；有些断言依赖当前配置内容 |
| installed launch | 可执行文件存在，真实 launch 启动、就绪与正常退出 | 不证明车辆能驾驶 |
| 合成 ROS graph | DDS、QoS、定时器、连续零动作、启动、传感器 watchdog、显式恢复 | 握手只验证策略侧消息，不验证 MCU 接受／物理停止 |

一个关键限制：很多现有测试临时替换 `calculate_policy_actions()`，用预设的假策略返回值检查框架。因此即便全部通过，也不能认定你们新写的 MPC／Planner 已被执行和验证。

当前还存在 `test_shipped_student_calculation_is_zero_and_handles_missing_observations`，专门验证老师的零动作起始策略。真正接入算法后，要根据新契约更新这个起始策略测试，保留“未允许接管／无效输入时的正确输出”等要求，并补充有效输入时真正学生算法的检查。不能为得到绿灯删掉停止、有效性等框架测试。

## 5. 三套环境、两份代码：不要弄混

| 位置 | 示例 | 用途 |
| --- | --- | --- |
| Windows Baby-Bus | `E:\26b\AI4R\Baby-Bus` | VS Code 编辑、commit、push 功能分支 |
| Windows Dream Gym | `E:\26b\AI4R\dream-gym\.venv` | 仿真、MPC 原型、图表与算法测试 |
| WSL Baby-Bus | `~/ai4r-dev/Baby-Bus` | 拉取同一功能分支，构建与 ROS 验证 |

Windows 与 WSL 的独立 clone 不自动同步。commit 但没有 push 的代码、没有保存的 VS Code 内容、未提交修改，都不会通过 WSL 的 `git pull` 自动出现。

WSL 建议把构建源码放在 Linux 的 home 目录。用 Git 同步明确版本，比在 `/mnt/e` 上直接构建更容易管理本次验证的源码与构建产物。[微软文件存放说明](https://learn.microsoft.com/en-us/windows/wsl/filesystems)

GitHub 保存代码与协作记录；Python 环境、ROS 安装、编译结果仍分别位于各机器。连接一个云端任务也不会自动继承 WSL 的这些东西。

## 6. 首次初始化：从 Windows 到 ROS 验证环境

### 6.1 检查并安装 WSL2 Ubuntu 24.04

**位置：Windows 管理员 PowerShell。**

```powershell
wsl --list --verbose
wsl --list --online
```

已有 `Ubuntu-24.04` 就使用它，不重复安装。没有时执行：

```powershell
wsl --install -d Ubuntu-24.04
```

按提示重启、启动 Ubuntu、创建用户与密码。Linux 输入密码时不显示字符或星号，直接输入后按 Enter，再输入一次确认。`sudo` 也是同样的隐藏显示。

在 PowerShell 确认 VERSION 为 `2`；若现有发行版为 `1`，可以转换：

```powershell
wsl --set-version Ubuntu-24.04 2
wsl -d Ubuntu-24.04
```

这些命令依据 [微软 WSL 安装说明](https://learn.microsoft.com/en-us/windows/wsl/install)。发行版名称以你本机列表为准。

### 6.2 确认 Ubuntu 版本，准备 UTF-8 与安装工具

**位置：Ubuntu 终端。后续第 6、7 节均在这里执行。**

```bash
cat /etc/os-release
```

确认是 Ubuntu `24.04`、代号 `noble`。下文下载命令明确针对 noble；若不是这个版本，先准备匹配环境。

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl ca-certificates python3
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
locale
```

`locale` 应显示 UTF-8 配置。终端输入 sudo 密码后按 Enter，不显示输入属于正常行为。

### 6.3 启用 Universe 与 ROS 软件源

```bash
sudo add-apt-repository -y universe
sudo apt update
```

第一条变量赋值命令是一整行：

```bash
task_ros_apt_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c "import json,sys; print(json.load(sys.stdin)['tag_name'])")
echo "$task_ros_apt_version"
```

必须得到非空版本号，且前一条没有下载／JSON 错误，才继续：

```bash
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${task_ros_apt_version}/ros2-apt-source_${task_ros_apt_version}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

第 6.2–6.4 节基于 [ROS Jazzy 官方 Ubuntu 安装文档](https://repo.test.ros2.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。变量取得的是软件源配置包版本，不是让你更换 ROS 发行版；下面仍安装 Jazzy。

### 6.4 安装 ROS 和构建工具

```bash
sudo apt upgrade
sudo apt install -y ros-jazzy-ros-base ros-dev-tools
sudo apt install -y \
  git cmake build-essential util-linux \
  python3-colcon-common-extensions \
  python3-pytest python3-yaml \
  ros-jazzy-ament-cmake-pytest \
  ros-jazzy-tf2-ros-py
```

`ros-base` 为本 gate 提供基础 ROS 工具；`util-linux` 提供脚本所用的 `flock`；C++ 工具链用于接口消息构建。算法是 Python 不代表消息构建只需 Python。

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
command -v python3
python3 --version
python3 -c "import rclpy, yaml; print('ROS/PyYAML import OK')"
colcon --help
rosdep --help
```

期望 ROS_DISTRO 为 `jazzy`，系统 Python 为 Ubuntu 24.04 的 Python 3.12 系列，导入成功。基础 gate 阶段不激活 Windows venv，也不使用 conda 的 Python 替代 Ubuntu 系统解释器。

每次新开 Ubuntu 终端都需要加载 ROS。gate 自己也会重新加载 ROS，但下面的 rosdep 和其他手动检查需要正确环境。

### 6.5 初始化 rosdep

首次执行：

```bash
sudo rosdep init
rosdep update
```

若 `sudo rosdep init` 提示 default sources list 已存在，说明初始化过，跳过 init、执行 update。其他网络／权限错误要按实际报错处理，不能都当成“已经初始化”。

### 6.6 取得 Baby-Bus：已有目录就检查，首次才 clone

你的 WSL 目录已经存在，直接执行：

```bash
cd ~/ai4r-dev/Baby-Bus
git remote -v
git status --short
git branch --show-current
git rev-parse HEAD
```

另一台新电脑还没有目录时，首次执行：

```bash
mkdir -p ~/ai4r-dev
cd ~/ai4r-dev
git clone https://github.com/wyf9613/Baby-Bus.git
cd Baby-Bus
```

私有仓库使用已授权的 GitHub 认证。HTTPS 若要求凭据，按 GitHub 提示使用受支持的认证方式；不要把 token 拼进 remote URL、写进文档或提交到仓库。

不要在已有目录上反复 clone。仓库已经存在但代码不一致，按第 9 节同步分支。

### 6.7 取得精确的 dream_interfaces 提交

`dream_interfaces` 是 ROS 消息／接口定义包，不是 MPC 求解器。策略和其他节点必须使用兼容的消息契约。

当前本地清单记录：`5f50902ccee44e8370d6e2be85607b3054ffbf98`。这只是本次记录；以后始终读取当前分支上的清单。

**位置：Ubuntu 的 Baby-Bus 根目录。**

```bash
mkdir -p .verification/dependencies
```

若 `.verification/dependencies/dream_interfaces` 尚不存在，首次 clone：

```bash
git clone --no-checkout \
  https://gitlab.unimelb.edu.au/dream/dream_interfaces.git \
  .verification/dependencies/dream_interfaces
```

`--no-checkout` 只取得 Git 数据，后面还必须 checkout。若已有目录，先检查 remote、状态和 HEAD；不要重建或覆盖已有修改。

```bash
task_interfaces_revision=$(python3 -c 'import yaml; print(yaml.safe_load(open("ci/dependencies.repos"))["repositories"]["dream_interfaces"]["version"])')
echo "$task_interfaces_revision"
git -C .verification/dependencies/dream_interfaces checkout --detach "$task_interfaces_revision"
```

确认版本和状态：

```bash
git -C .verification/dependencies/dream_interfaces rev-parse HEAD
git -C .verification/dependencies/dream_interfaces status --short
```

过关条件：HEAD 与清单一致，状态没有输出。出现你主动做过的修改时，先保留并检查，不使用强制 checkout 或 reset 覆盖。

学校 GitLab 网络不通时检查大学 VPN；网络可达但拒绝访问时检查仓库授权。VPN 不授予仓库权限。目标 commit 不存在时参见第 11.2 节。

### 6.8 安装 package.xml 声明的依赖

```bash
source /opt/ros/jazzy/setup.bash
rosdep install \
  --from-paths . .verification/dependencies/dream_interfaces \
  --ignore-src -r -y --rosdistro jazzy
```

该命令依据 [CONTRIBUTING.md](../CONTRIBUTING.md)，安装 [package.xml](../package.xml) 等清单声明的依赖。`--ignore-src` 表示已有源码包由本地构建，不用系统包替代它们。

如果 rosdep 总结有 unresolved／failed 项，环境仍未准备完成。下载和安装结束后，才运行 gate。

## 7. 第一次 gate：执行、判定、记录

### 7.1 最简单的执行命令

**位置：Ubuntu `~/ai4r-dev/Baby-Bus` 根目录。**

```bash
cd ~/ai4r-dev/Baby-Bus
AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" \
  bash tools/verify_fast.sh
```

第二行前的 `>` 若由终端自动显示，表示反斜杠续行的提示符，不是要输入的命令。反斜杠必须是该行最后一个字符。

判断通过：命令退出码为 0、构建成功、测试结果无失败／错误、launch 参数检查成功，并出现末尾 PASS。未运行、被中断、只开始构建、只看到节点就绪，都不能记作通过。若出现 skip，要记录并说明它是否影响本轮验收，不能隐藏跳过项。

### 7.2 保存日志并避免 tee 掩盖失败

下面是带日志的版本，**代替**上面的简单命令使用，不必无理由连续跑两次。日志保存在仓库外，避免误提交。

```bash
cd ~/ai4r-dev/Baby-Bus
task_source_sha=$(git rev-parse HEAD)
task_run_stamp=$(date -u +%Y%m%dT%H%M%SZ)
task_evidence_dir="$HOME/ai4r-verification-logs/${task_run_stamp}-${task_source_sha:0:12}"
mkdir -p "$task_evidence_dir"
git rev-parse HEAD > "$task_evidence_dir/policy-sha.txt"
git status --short > "$task_evidence_dir/policy-status.txt"
git -C .verification/dependencies/dream_interfaces rev-parse HEAD > "$task_evidence_dir/interfaces-sha.txt"
cat /etc/os-release > "$task_evidence_dir/os-release.txt"
python3 --version > "$task_evidence_dir/python-version.txt" 2>&1

AI4R_INTERFACES_SOURCE="$PWD/.verification/dependencies/dream_interfaces" \
  bash tools/verify_fast.sh 2>&1 | tee "$task_evidence_dir/verify-fast.log"
task_gate_rc=${PIPESTATUS[0]}
printf '%s\n' "$task_gate_rc" > "$task_evidence_dir/exit-code.txt"
printf 'gate exit code: %s\nlogs: %s\n' "$task_gate_rc" "$task_evidence_dir"
```

`PIPESTATUS[0]` 必须紧跟测试 pipeline 读取。单看 `tee` 成功或最后一条 shell 命令成功，不能认定 gate 成功。这个代码块适用于交互式 Bash；如果放进另一个脚本，还要正确传播 gate 的退出码。

正式证据最好对应干净的已提交源码。若 `policy-status.txt` 非空，SHA 不能完整标识本次源码，必须注明 dirty 状态和差异；最终提交后再对实际交付版本做完整 gate。

主要构建／测试产物在 `.verification/ws/`。根目录 `.gitignore` 已忽略 `.verification/`，不将接口 checkout、build、install、log 提交到功能分支。

## 8. MPC 的求解器依赖：仿真和 ROS 要分别处理

### 8.1 Windows Dream Gym：现在就能准备的仿真依赖

NumPy 做数组／矩阵运算，SciPy 构造稀疏矩阵，OSQP 求解 QP。`dream_interfaces` 定义 ROS 消息，与这三者用途不同。

当前 Dream Gym 的 [setup.py](../../dream-gym/setup.py) 声明 NumPy `>=2.0,<3.0`，SciPy `>=1.13,<2.0`，OSQP `>=1.1.3,<2.0`。这属于 Dream Gym 当前版本的要求，不自动成为所有 ROS 部署方式的已验证版本。

**位置：Windows PowerShell。已有 `.venv` 时：**

```powershell
cd E:\26b\AI4R\dream-gym
.\.venv\Scripts\python.exe -m pip install -e ".[mpc]"
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -c "import numpy, scipy, osqp; print(numpy.__version__, scipy.__version__, osqp.__version__)"
```

新电脑没有 venv 时先创建，Python 3.12 须已安装：

```powershell
cd E:\26b\AI4R\dream-gym
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip setuptools
.\.venv\Scripts\python.exe -m pip install -e ".[mpc]"
```

OSQP 的 Python 接口使用 NumPy 数组、SciPy 稀疏矩阵；首版可使用默认 CPU 后端。[OSQP 接口说明](https://osqp.org/docs/interfaces/python.html)、[安装说明](https://osqp.org/docs/get_started/python.html)

### 8.2 Ubuntu 中准备 ROS 兼容的 MPC 开发探针

这一节是独立 QP／导入探针，不表示当前 fast gate 与实车部署已自动接好。ROS 预编译二进制要求兼容的系统 Python；基于系统解释器创建 venv 是官方支持的路线。[ROS Python 包说明](https://repo.test.ros2.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)

**位置：Ubuntu；环境放在仓库外，首次创建。**

```bash
sudo apt install -y python3-venv
mkdir -p ~/ai4r-python
/usr/bin/python3 -m venv --system-site-packages ~/ai4r-python/mpc-ros
source ~/ai4r-python/mpc-ros/bin/activate
source /opt/ros/jazzy/setup.bash
python -m pip install --upgrade pip
python -m pip install "numpy>=2.0,<3.0" "scipy>=1.13,<2.0" "osqp>=1.1.3,<2.0"
python -m pip check
python -c "import sys, rclpy, numpy, scipy, osqp; print(sys.executable); print(numpy.__version__, scipy.__version__, osqp.__version__)"
```

`--system-site-packages` 让 venv 能访问系统 Python 包，同时第三方包仍安装到这个 venv。[Python venv 说明](https://docs.python.org/3.12/library/venv.html)

小型求解探针：下面应求得接近 `0.5` 的解，并报告 `solved`，不依赖车辆或 ROS topic。

```bash
python - <<'PY'
import numpy as np
from scipy import sparse
import osqp

solver = osqp.OSQP()
solver.setup(
    P=sparse.csc_matrix([[2.0]]),
    q=np.array([-1.0]),
    A=sparse.csc_matrix([[1.0]]),
    l=np.array([0.0]),
    u=np.array([1.0]),
    verbose=False,
)
result = solver.solve()
print('status:', result.info.status)
print('solution:', result.x)
assert result.info.status == 'solved'
assert abs(float(result.x[0]) - 0.5) < 1e-3
PY
```

通过后记录实际安装版本。项目依赖清单以后应锁定经过验证的具体版本／后端／平台；当前给出的范围不是已经完成的团队版本冻结。

返回基础 ROS 验证终端时，退出探针环境：

```bash
deactivate
source /opt/ros/jazzy/setup.bash
```

### 8.3 MPC 接入后必须确认“哪个 Python 实际在跑”

安装 OSQP 到一个 venv，不代表 ROS 测试 runner、installed launch 和真实部署节点会使用它。

当前 gate 会清除继承的 `PYTHONPATH` 等变量，并经 CMake／ament 生成测试。构建缓存、测试 runner 和节点启动使用的解释器可能不同；仅激活 venv 或临时设置 PYTHONPATH 不是已验证的完整接入方案。

基础环境检查可执行：

```bash
source /opt/ros/jazzy/setup.bash
command -v python3
python3 -c "import sys; print(sys.executable)"
```

基础 gate 构建后，可查看 CMake 选择的 Python 和实际生成的测试命令：

```bash
sed -n '/Python.*EXECUTABLE/p' .verification/ws/build/ai4r_policy/CMakeCache.txt
ctest --test-dir .verification/ws/build/ai4r_policy -N -V
```

这些只帮助定位；最终要在**实际测试进程与 installed 节点进程**记录解释器／包版本，并让其执行真实 MPC。普通 shell 的 import 测试不能替代这一步。

集成负责人应选择并实现一致的依赖交付方式：可用且版本合适的 rosdep／系统包，或者基于系统 Python、明确配置构建／测试／运行入口的 venv。若改变 gate／CI 或公开接口，按仓库 review 流程交付。不要为修一个 import 错误直接在共享车上安装包或覆盖系统环境。

本次没有修改 gate、CI、package.xml 或 MPC 入口；所以“ROS＋OSQP 正式接入环境”仍须随实现确定。第 6 节完整准备的是当前仓库的基础 ROS gate 环境。

## 9. 日常工作：Windows push 分支，WSL 拉取并测试

### 9.1 Windows 编辑与提交

主要在 Windows VS Code 编辑，WSL 主要验证，减少两边同时改代码。功能分支例子为 `feature/mpc-v0`，请替换为实际分支名。

**位置：Windows PowerShell，Baby-Bus 根目录。**

```powershell
cd E:\26b\AI4R\Baby-Bus
git branch --show-current
git status --short
git diff
```

按实际改动选择文件，避免 `git add .` 把无关 Notebook 或个人文件一起提交。下面是假设本轮改动就是这两个文件的例子，不是每次必须提交这两个文件：

```powershell
git add scripts/policy_node.py tests/test_policy_node.py
git diff --cached
git commit -m "Add MPC policy integration checks"
git push -u origin feature/mpc-v0
git rev-parse HEAD
```

已有 tracking 后可用 `git push`。不要把示例 commit 文案用于并未实现的工作。普通正式贡献按仓库 runbook 从 `dev` 建功能分支，经 PR／MR review；不直接或强制 push 受保护的 `dev`／`main`。如果团队 GitHub 的实际分支布局不同，先由负责人明确协作分支，不擅自创建或重写 release 历史。

### 9.2 WSL 同步相同分支

**位置：Ubuntu。**

```bash
cd ~/ai4r-dev/Baby-Bus
git status --short
```

有本地改动时先停下来检查和保留；工作区干净后：

```bash
git fetch origin
git switch feature/mpc-v0
git pull --ff-only origin feature/mpc-v0
git branch --show-current
git rev-parse HEAD
git status --short
```

若本地没有分支，Git 通常会根据同名 `origin` 分支自动创建跟踪分支；若自动推断失败且确实没有该本地分支，可明确执行：

```bash
git switch --track -c feature/mpc-v0 origin/feature/mpc-v0
```

确认 WSL SHA 与 Windows 刚推送的 SHA 相同。没有先合并进 main 的要求，feature 分支可以完整验证。

若 Windows 又推送了新提交，WSL 在同一分支上 `git pull --ff-only` 后重新检查。若分支分叉／存在冲突，先分析差异并保留工作，不能用 reset／force push 代替处理。

### 9.3 检查依赖 pin 是否变化

每次拉取后读取当前清单、比较接口 HEAD：

```bash
task_interfaces_revision=$(python3 -c 'import yaml; print(yaml.safe_load(open("ci/dependencies.repos"))["repositories"]["dream_interfaces"]["version"])')
echo "$task_interfaces_revision"
git -C .verification/dependencies/dream_interfaces rev-parse HEAD
git -C .verification/dependencies/dream_interfaces status --short
```

相同且干净：不用更新接口。不同且已有普通 checkout 干净：先 fetch，再 checkout 清单指定的 commit：

```bash
git -C .verification/dependencies/dream_interfaces fetch origin
git -C .verification/dependencies/dream_interfaces checkout --detach "$task_interfaces_revision"
git -C .verification/dependencies/dream_interfaces rev-parse HEAD
git -C .verification/dependencies/dream_interfaces status --short
```

不要对 detached 的接口仓库直接 `git pull` 最新 dev。策略要求的是固定 commit，更新太新也会不匹配。若 package.xml 或依赖清单有变化，重新执行第 6.8 节的 rosdep provisioning；不要指望 gate 安装缺包。

### 9.4 根据改动选测试，再做最终 gate

| 本轮改动 | 至少检查什么 |
| --- | --- |
| 纯文档 | 内容、链接、路径、命令适用终端和版本；通常不用为措辞变化跑整套 ROS |
| MPC 模型／QP／参考／权重／约束 | 对应 U 测试＋受影响 S 场景＋相关 F 场景；接入 Baby-Bus 的版本再完成 gate |
| Planner 路径／stop／goal | 路径退化、坐标和时间、停止与路径结束；与控制器组合的闭环；实际 ROS 接口测试＋gate |
| PI/PID／物理动作映射 | 单位、符号、限幅、上一实际输入、停止响应；声明的纵向 plant 闭环；gate 与必要现场复测 |
| 策略状态机／触发／观测新鲜度 | 相关 ROS 框架测试＋完整 gate；不以仿真代替 |
| YAML／launch／安装内容 | 参数加载与非法输入、资源安装、相关节点配置契约＋gate；配置断言与真实契约一起更新 |
| 新依赖／消息定义 | 重新 provisioning、导入和小问题检查、实际 installed 节点执行、相关接口测试＋gate |
| 多模块合并 | 固定最小模块／闭环／故障回归＋真实学生策略 ROS 接入＋完整 gate |

改 MPC 原型但未进入 Baby-Bus 时，先做仿真算法检查。发布 Baby-Bus 可运行改动时，不能把只做了原型测试写成 ROS 交付通过。

在最终相关源码确定后运行第 7 节完整 gate，并记录提交。失败时先定位有关测试，修复后再跑受影响测试和最终 gate；成功后无需没有原因地反复扩大测试范围。

若 WSL 中修复并 push 了代码，Windows 也需要在干净工作区拉取该分支。不要让后续 Windows 提交重新覆盖修复。

## 10. MPC V0＋M0 计划要求补充的测试

### 10.1 U／S／F 的对应关系

| 计划项 | 要验证的关键内容 |
| --- | --- |
| U1–U2 | 坐标／符号／单位、模型预测、已知偏差的正确纠偏方向 |
| U3 | 单次 QP 的初始状态、动力学、约束残差、首动作提取 |
| U4 | 角度跨 ±π、重复点、短路径、参考无效状态 |
| U5 | 非零上一实际输入、切换／重启、变化率约束 |
| S1–S4 | 直线、左右横向／航向偏差、左右缓弯；整个车身边界、误差和动作指标 |
| S5 | stop／goal／路径结束后不继续使用旧跟踪动作 |
| F1–F2 | 无效／过期输入、不可行／超时／失败解不被下发 |
| F3 | 所用 solver 版本的真实限时／迭代行为，以及独立监督不等待阻塞求解结束 |
| F4 | plant 参数偏差／转向延迟下的退化和预测误差 |
| F5 | 停止／重启后重置状态，显式恢复 |
| T8 | 目标平台完整 step 均值／P95／最大值／超预算比例，启动 setup、周期抖动和数据年龄 |
| T9–T10 | 无动作试运行、逐级接管、固定配置三次正式路线和独立停止记录 |

门槛采用你们事先冻结的路线／车宽／余量／速度／时间预算，不能看完结果后改变通过标准。零电机请求不等于瞬时物理停车；solver 返回后才发现耗时过长，不证明阻塞期间独立停止有效。

### 10.2 现有 Dream Gym 测试能做什么

以下是当前上游组件的示例检查，不是你们团队全部 U／S／F 场景的实现：

**位置：Windows PowerShell、Dream Gym 根目录。**

```powershell
.\.venv\Scripts\python.exe -B -m unittest unit_tests.test_registration -v
.\.venv\Scripts\python.exe -B -m unittest unit_tests.test_linear_quadratic_mpc_solver_api -v
.\.venv\Scripts\python.exe -B -m unittest unit_tests.test_mpc_closed_loop -v
```

接下来需给团队自己的 controller／reference／planner／plant 添加对应回归入口，并在运行记录中写真实命令。本文不虚构当前尚不存在的团队测试文件或“已经通过”的结果。

### 10.3 ROS 中至少有一条测试真正执行学生算法

从 installed 策略入口开始，用合成状态／观测和符合团队契约的路径，实际调用 MPC／Planner，不替换成恒定输出假函数。至少覆盖：正常合法输入、路径结束／stop、无效数据、失败结果、超预算／过期结果、重启与实际上一输入。

同时保留框架层的假策略测试：它们用于独立检查裁剪、状态机和异常处理，两类测试职责不同。

当前 CMake 只注册 `tests/test_policy_node.py`。新写另一个 ROS pytest 文件并不会自动进入 gate；需在测试注册／现有入口中明确接入，并验证结果中确实执行到它。改测试基础设施时遵循仓库 review 要求。

仓库要求实车策略代码保持在 `scripts/policy_node.py`、保留插入标记。计划中按职责拆模块的仿真原型，与实车交付组织需协调；若希望改变单文件约定，先由课程／仓库负责人明确许可范围。

### 10.4 Planner 若是独立节点

fast gate 只构建接口包和策略包，不会自动安装或测试另一个 Planner 包。独立 Planner 要有自己的软件测试，再验证它与 MPC 的消息、frame、时间戳、path_id、短路径和 stop／goal 行为。

只 launch 两个节点并看到没有报错，不等于接口集成通过。若 Planner 在同一学生策略内部，仍需内部接口与组合闭环测试。

## 11. 常见报错：按顺序检查

### 11.1 本次遇到的 Wrong dream_interfaces revision

你实际展示的是：

```text
要求：5f50902ccee44e8370d6e2be85607b3054ffbf98
实际：63ae907f61a9acc0e7afab7916c23562aeeabf41
status：所有已跟踪文件显示 D
```

结合首次 `git clone --no-checkout`，这与尚未初始化工作区 checkout 的状态相符：下载了仓库，但没有完成指定版本的文件检出。不是 ROS 构建阶段的报错。

**位置：Ubuntu Baby-Bus 根目录。**

```bash
task_interfaces_revision=$(python3 -c 'import yaml; print(yaml.safe_load(open("ci/dependencies.repos"))["repositories"]["dream_interfaces"]["version"])')
git -C .verification/dependencies/dream_interfaces checkout --detach "$task_interfaces_revision"
git -C .verification/dependencies/dream_interfaces rev-parse HEAD
git -C .verification/dependencies/dream_interfaces status --short
```

HEAD 匹配、status 无输出后重跑 gate。这里只解释你展示的首次 clone 情况；以后看到 D，不能一律认定是 no-checkout，也可能是你实际删除的文件。

### 11.2 reference is not a tree／无法找到 commit

检查接口 remote，再 fetch：

```bash
git -C .verification/dependencies/dream_interfaces remote -v
git -C .verification/dependencies/dream_interfaces fetch origin
git -C .verification/dependencies/dream_interfaces cat-file -t "$task_interfaces_revision"
```

最后应输出 `commit`。注意变量在新终端要重新赋值。若仍没有目标提交，检查授权／clone 是否浅克隆，以及上游是否提供该历史。请依赖提供者给包含指定 commit 的 Git 仓库或 bundle；只复制无 Git 历史的源码不满足 gate。

CONTRIBUTING 的 feature pin 未发布说明和清单的 v0.2.0 注释存在发布描述差异；本文未确认远端发布状态。无论是否有 tag，精确 commit 和源码状态仍是检查依据。

### 11.3 Modified dream_interfaces source／checkout 拒绝覆盖修改

```bash
git -C .verification/dependencies/dream_interfaces status --short
git -C .verification/dependencies/dream_interfaces diff
git -C .verification/dependencies/dream_interfaces diff --cached
```

先看是主动改动、误操作还是 checkout 状态；保留需要的内容后处理。不要直接照抄强制 reset、checkout -f 或删除依赖目录。

### 11.4 ModuleNotFoundError

| 缺少的包 | 首先检查 |
| --- | --- |
| `yaml` | Ubuntu 安装 `python3-yaml`；当前 Python 是否系统解释器 |
| `rclpy` | ROS Jazzy 安装／source；Python 是否兼容；是否进入了不兼容环境 |
| `dream_interfaces` | 指定源码是否存在；消息包是否构建；正确 overlay 是否加载 |
| `osqp`／`scipy`／`numpy` | 依赖装在哪个 Python；报错来自 shell、测试 runner 还是 installed 节点；按第 8 节处理 |

不要为了让 gate 跳过版本检查而修改清单 SHA。需要改变接口契约时，属于正式的跨仓库集成工作。

### 11.5 另一个 gate 占用 domain 218

`Another policy test gate is using domain 218` 表示文件锁正在被使用。检查是否另一个终端仍在运行同一 gate，等待其结束或正常停止。不要通过删锁文件让两个 gate 同时运行；锁随持有进程退出释放。

### 11.6 出现测试失败，而不是环境缺包

记录第一个实际失败的断言、测试名和日志，先判断是行为回归、配置契约更新、版本混用，还是零动作 starter 测试需要随正式算法演进。不能把全部失败都归类为“WSL 不行”。

如果 gate 已构建成功，要单独调试当前注册的 ROS pytest，可以在**专用的新 Ubuntu 终端、没有其他 gate 运行时**准备同样的隔离环境：

```bash
cd ~/ai4r-dev/Baby-Bus
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
source /opt/ros/jazzy/setup.bash
source .verification/ws/install/setup.bash
export ROS_DOMAIN_ID=218
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

下面示例聚焦“裁剪／pan／异常／超预算”那个已有测试；使用与正式测试相同且已核对的解释器，基础环境通常为系统 python3。将 `-k` 表达式换为实际失败的测试名：

```bash
flock -n /tmp/ai4r-policy-domain-218.lock \
  python3 -B -m pytest tests/test_policy_node.py \
  -k test_clipping_pan_hold_exception_and_overrun -v
```

这是调试入口，不会重新构建所有包，不替代最终完整 gate。MPC 接入后若已确定使用其他解释器，这里也必须同步，不能混用。

### 11.7 Unexpected source path／Unexpected verification package

脚本发现 `.verification/ws/src` 的链接或包数量不符合预期。先查看 `ls -l .verification/ws/src` 和链接实际指向，不随意覆盖现有目录。需要重建验证产物时，仅处理确认属于本 gate 的生成目录，并保留日志与源码；不是重新删除整个项目。

### 11.8 Windows／WSL 版本没有同步

分别执行 `git branch --show-current`、`git rev-parse HEAD`、`git status --short`。确认文件保存、commit、push、WSL pull 每一步都做过。相同 branch 名称不代表相同 commit；相同 commit 在 dirty 工作区也不代表相同源码。

若脚本出现 `$'\r'`／`bash\r` 等报错，检查文件换行是否为 CRLF、仓库 `.gitattributes` 与 checkout 设置。先定位，不把所有源码全局转换换行。

## 12. 上车前与 V0／M0 的边界

软件验证机与小车 runtime 用途不同。不要为搭建开发验证环境在课程共享车上额外安装工具／AI 助手或覆盖 underlay。

部署前由 demonstrator／集成负责人确认：车上 DREAM、接口定义、传感器与配置委派支持这轮策略。可按课程允许的只读流程查看：

```bash
ros2 pkg prefix dream_interfaces
ros2 interface show dream_interfaces/msg/DriveAndSteer
ros2 interface show dream_interfaces/msg/FiducialDetections
```

这些检查只说明找到包／消息，不单独证明安装源码对应哪个 commit。精确版本仍需构建来源与集成记录。新的学生 YAML 也可能要求匹配的 DREAM 参数委派支持。

按课程既有入口部署学生 workspace、build、restart；GitHub push 不会自动改变车上程序。控制启用、停止和人工接管遵循课程流程，算法策略不能自行扩大硬件控制权限。

T8 在目标平台用样例／回放进行无执行器输出的计时，记录完整 step 而不是只记录 solver.solve。目标平台访问和运行方式由车辆负责人协调；WSL 测得的时间另外标注为开发机证据。

T9／T10 的实时状态／路径、动作映射、共享纵向 PI/PID、停止和现场路线验收单独记录。M0 的三次成功与停止测试是待执行目标，历史软件 PASS 不抵扣这些任务。

## 13. 最小验证记录模板

将当前结果摘要记录在 [docs/acceptance.md](acceptance.md) 或团队对应 PR／MR；详细日志存放 CI artifact／约定证据位置。正式源码记录与证据文件的归档方式由团队确认。

```text
日期／操作者：
验证类型：模块 / 仿真 / ROS gate / ROS 算法接入 / 回放 / 目标平台计时 / 实车
Baby-Bus 分支／commit：
工作区状态：clean / dirty（列出差异）
Dream Gym 或团队算法 commit（如适用）：
dream_interfaces 清单要求 commit：
dream_interfaces 实际 HEAD 与源码状态：
机器／架构／Ubuntu／ROS／Python：
实际测试与节点 Python、NumPy／SciPy／OSQP 版本（如适用）：
配置／场景／参数来源与预先规定的门槛：
执行命令与退出码：
通过／失败／跳过项目：
日志／图表／视频位置：
实际观察结论：
限制：
硬件结果：not run / 有现场记录的具体结果
```

发布契约相关改动另按 runbook 执行 `python3 -B tests/test_release_contract.py -v`。它是发布工具检查，不能代替 ROS gate；普通 MPC 调参不要求无理由重复发布流程。安全、公开接口、CI 策略和发布等 review 要求以仓库规定为准。

## 14. 每次交付前的速查清单

- [ ] Windows 改动已保存、提交并推送到约定功能分支。
- [ ] WSL 拉到了同一 commit；正式记录注明工作区状态。
- [ ] 当前清单要求的接口 commit 匹配，接口源码状态已核对。
- [ ] 新增依赖在实际测试和运行环境可用；不是只在另一个 venv 可用。
- [ ] 跑了本轮算法所需 U／S／F 测试，并记录真实命令和结果。
- [ ] 学生算法实际 ROS 接入有测试，未仅用假策略代表 MPC／Planner。
- [ ] 最终相关源码完整 gate 退出 0；结果、PASS 和日志已核对。
- [ ] 新测试已进入注册入口；没有通过删除框架检查得到绿灯。
- [ ] 目标平台计时、回放、上车 build、实车验收分别报告。
- [ ] 没有亲自观察的硬件结果写 `not run`。
- [ ] 通过 PR／MR 协作与必要人类 review，不擅自 merge／release。

## 15. 来源与维护

仓库依据：[CONTRIBUTING.md](../CONTRIBUTING.md)、[AGENTS.md](../AGENTS.md)、[verify_fast.sh](../tools/verify_fast.sh)、[ROS 测试](../tests/test_policy_node.py)、[CMakeLists.txt](../CMakeLists.txt)、[package.xml](../package.xml)、[接口 pin](../ci/dependencies.repos)、[acceptance.md](acceptance.md)。团队 V0／M0 验收内容来自本次提供的 2026-10-03 开发计划。

外部依据：上文链接的微软 WSL 文档、ROS Jazzy 安装／Python 包文档、Python venv 文档和 OSQP 文档。仓库与依赖更新后重新核对脚本、消息 pin、测试注册和安装方式。本文的示例 branch 名、版本记录、目录和测试入口不应替代实际 checkout 的清单与结果。
