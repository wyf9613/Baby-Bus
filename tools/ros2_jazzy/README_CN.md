# Mac 上的 ROS 2 Jazzy 开发环境

这套环境在 Apple Silicon Mac 的独立 Colima 虚拟机中运行官方 Ubuntu 24.04 ARM64 / ROS Jazzy 容器。Mac 的 Python 3.13 / Dream Gym 仿真继续使用原 `.venv`；ROS 使用容器里的 Python 3.12。两套 Python 包互不混装，读取同一功能工作目录的代码。

2026-10-20 本机完整 gate 已通过：353 tests、0 errors/failures/skipped（344 个 pytest 用例及 9 个 CTest 测试组）。另外，8 项停车门控单元检查、两帧合成 LaserScan 的 MCAP 往返验证、60 项 Mac 侧受影响 fixture 回归及环境停止/重启验证均通过。验证结束后停止环境以释放内存，`shell` / `check` 会自动启动。

环境只提供终端、ROS 构建与测试、rosbag 读写，不包含 Gazebo / RViz 图形桌面。当前 ROS 节点已接入新 LiDAR 停车门控，默认关闭，本地测试显式开启；没有部署实车。最新功能验收见 [ROS 雷达停车说明](../../docs/ROS_LIDAR_STOP_CN.md)，380 tests、0 failures。

## 平时使用

从仓库根目录执行：

```bash
bash tools/ros2_jazzy/run.sh shell   # 自动启动环境并进入 ROS 终端
bash tools/ros2_jazzy/run.sh check   # 完整构建、测试、检查 launch 参数
bash tools/ros2_jazzy/run.sh lidar-stop # 七场景合成雷达停车演示与记录
bash tools/ros2_jazzy/run.sh status  # 查看环境状态
bash tools/ros2_jazzy/run.sh stop    # 停止本项目环境，释放内存
```

也可使用课程工作目录里的 `启动ROS2开发环境.command`、`运行ROS2检查.command`、`停止ROS2环境.command`。开发终端的提示符以 `(Baby-Bus ROS Jazzy)` 开头。输入 `exit` 退出终端；要释放虚拟机内存，再使用停止入口。

终端自动加载 ROS Jazzy，以及已经编译好的 DREAM 接口和 policy overlay。它保持在本功能 worktree；源码编辑直接共享，不复制一份容易过时的 policy。源码 Git 元数据只读挂载，提交和切分支在 Mac 侧进行；ROS 编译结果位于独立 Docker volume。

`check` 原样运行 `tools/verify_fast.sh`：检查指定版本的接口源码，构建两个包，运行当前 CMake 注册的测试组，检查安装后的 launch 参数。每次日志、测试 XML 保存在 `.verification/ros2-jazzy/checks/<时间>/`。它运行真实 ROS 类型、受控时钟和合成 ROS peers，没有启动传感器驱动或车辆服务。

## 本机配置

- Colima profile：`babybus-jazzy`，2 CPU、4 GiB 内存、20 GiB 容器数据磁盘上限（稀疏分配）。虚拟机另有默认 20 GiB 根盘，按实际写入占空间。
- Docker context：`colima-babybus-jazzy`；所有命令显式选择它，不切换用户默认 context。
- 容器：`babybus-jazzy`；本地镜像：`babybus-ros-jazzy:local`。
- 构建 volume：`babybus-jazzy-verification`，挂在 worktree 的 `.verification/ws`。因此 Mac 侧这个目录不展示容器里的 build/install；检查日志会复制到 Mac 侧。
- 挂载：本功能 worktree 可写，原仓库 Git 元数据只读。没有将整个 Mac 用户目录交给容器。
- ROS domain：218；发现范围：`LOCALHOST`；未发布容器网络端口。用于隔离的本地测试；连接实车需要单独安排网络配置。
- `dream_interfaces`：`v0.2.0` / `5f50902ccee44e8370d6e2be85607b3054ffbf98`。
- 基础镜像的不可变摘要记录在 `.verification/ros2-jazzy/base-image-ref.txt`，编译时按该摘要取镜像。

虚拟机状态和镜像位于 `~/.colima/` 及 Colima 缓存目录；工具由 Homebrew 安装。不会开机自动运行此 profile。停止环境会保留安装、编译缓存和源码。

## 从干净 Mac 重建的顺序

先装 Homebrew 的 `colima` 与 `docker` **formula**，创建上述 profile，使用 `aarch64` / `vz` / `virtiofs`，共享本地 Baby-Bus 仓库目录，并设置 `--activate=false --ssh-config=false --binfmt=false`。首次创建要明确指定 2 CPU、4 GiB 内存、20 GiB 数据盘。不要把 Docker Desktop cask 与这里使用的命令行工具混为一谈。

随后在功能 worktree 内执行：

```bash
mkdir -p .verification/ros2-jazzy
docker --context colima-babybus-jazzy pull --platform linux/arm64 ros:jazzy-ros-base-noble
docker --context colima-babybus-jazzy image inspect ros:jazzy-ros-base-noble \
  --format '{{index .RepoDigests 0}}' > .verification/ros2-jazzy/base-image-ref.txt
bash tools/ros2_jazzy/run.sh build
bash tools/ros2_jazzy/run.sh check
```

接口源码需按 `ci/dependencies.repos` 预先下载到 `.verification/dependencies/dream_interfaces`。当前 Mac 已完成下载，不必再次克隆。恢复已有 profile 时，`shell` / `check` 会按保存的配置自动启动。

## 验证与修复记录

环境建立后发现，`tests/test_lattice.py` 的共用 fixture 仍从 executable 旁边寻找配置，安装后的模块目录没有该资源。已让 ROS 测试通过 ament index 使用安装包的 share/config；无 ROS 的 portable 测试继续使用源码 config。改变的是测试资源路径，没有改变规划、控制参数或算法。

`probe.py` 使用真实消息序列化与 MCAP writer/reader，写入并读回两帧**合成** LaserScan，检查距离、frame_id 和时间戳，且构造已编译的 DREAM 消息类型。运行方式（在 ROS 终端）：

```bash
python3 tools/ros2_jazzy/probe.py --report .verification/ros2-jazzy/ros-probe.json
```

当前检查结果、版本、镜像摘要见 `.verification/ros2-jazzy/environment-manifest.json`，合成 bag 探测见 `ros-probe.json`，最终 gate 输出见 `final-ros-check.log`。旧的下载前依赖缺失、安装前 ROS 缺失日志只作为历史保留。

官方来源：[Colima 安装说明](https://github.com/abiosoft/colima/blob/main/docs/INSTALL.md)、[ROS 官方镜像清单（含 Jazzy/Noble/ARM64）](https://github.com/docker-library/official-images/blob/master/library/ros)。
