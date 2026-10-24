# MVP2.1 主动雷达绕行：本地开发入口

整理日期：2026-10-22，Australia/Melbourne。

本轮唯一开发分支是 `ying/mvp21-lidar-avoidance`，本地目录：

```text
/Users/xiaotonghe/Documents/Baby-Bus/.verification/worktrees/mvp21-lidar-avoidance
```

2026-10-22 保存了 MVP2.1 雷达停车基线。2026-10-24 已加入第一版静态雷达
绕行离线闭环，运行与证据边界见 `offline/lidar_avoidance/README.md`。
公共配置和 ROS 默认执行未启用绕行，没有部署或连接车辆。

## 基线身份

- 上游整合基点：Steve 的 `feature/mvp21-sim-integration-20261015@465e581`。
- 原停车分支：`ying/lidar-stop-mvp21`，原 HEAD 为 `eb94979`。
- `eb94979` 后的本地配置、时效诊断、无效扫描修复、测试和打包工具都已纳入
  新的源码快照 `8c75519ea9df38a7ebf97f53a9cbc3cf6cc77974`。
- 本轮从这个完整快照开始，而不是仅从 `eb94979` 开始。
- 基线快照的 23 个部署来源文件 SHA256 与
  `mvp21-lidar-stop-integrated-20261021/MANIFEST.json` 全部相符。
- 使用 MVP2.1 模块、native lattice planner 和原 MVP 控制/回退链；未引入 MPC。
  学习评分模型尚未接入 ROS 默认执行路径。

精确来源与文件指纹见 `docs/baselines/mvp21_lidar_stop_20261022.json`。

## 原工作区与备份

原 MVP、MPC、Planning V2–MPC 和 mpc-car-run 工作区均保留；原分支、HEAD、
暂存状态与未提交修改未改动。两个 dirty 功能工作区已通过独立 Git index
写入本地快照，不依赖原工作区的未提交文件才能恢复。

备份目录：

```text
/Users/xiaotonghe/学习/ai for robotics/group project/工作区版本整理/20261022-1940
```

其中包括四个源码归档、完整 Git 历史 bundle、源码与部署文件指纹、部署包、
已保存的车辆记录、关键验证回执和训练历史产物。所有训练历史产物也已保留在
本工作区的原有相对路径；原本被 Git 忽略的产物仍保持忽略状态。

大型仿真逐帧记录和运行环境仍保留在原工作区，没有全部复制到备份目录。
这是一份本机恢复副本，不是异地备份，也不能证明车辆端所有资料都已取回。

已验证：bundle 可以在临时目录独立 clone，恢复的 452 个 MVP 源文件与快照
逐个匹配；四份源码 tar 也已逐文件验证内容。

## 2026-10-22 基线检查

新工作区运行现有 112 个 portable checks：全部通过，0 failure/error/skip。
范围：control、obstacle_stop、planner_execution、lattice、MVP2.1 compatibility
和模块/worker 协议。确认实际导入的是本工作区模块。

证据：`.verification/baseline-20261022/RESULTS.json` 与 `portable-tests.log`。
相同回执另存于上述备份目录的 `new-workspace-baseline/`。

本轮复用了原 MVP 工作区的 Python 3.13.5 虚拟环境，没有安装或修改依赖。
可在本工作区重跑：

```bash
/Users/xiaotonghe/Documents/Baby-Bus/.verification/worktrees/mvp21-lidar-sim/.venv/bin/python -B -m unittest \
  tests.test_control tests.test_obstacle_stop tests.test_planner_execution \
  tests.test_lattice tests.test_mvp21_compatibility tests.test_mvp21_modules -q
```

保存基线时未重跑 ROS gate。匹配源码的历史整合版记录为 383 pytest cases 加 11 CTest
groups，共 394 reported tests，0 failure/error/skip；这是 2026-10-21 的记录，
不是 10 月 22 日重新执行的结果。
原 `tools/ros2_jazzy/run.sh` 的固定容器绑定原工作区；未来配置新 ROS 环境时
应单独处理，不能把原容器中的安装结果当作本工作区的新构建。

2026-10-24 新工作区已有独立合成 ROS build；407 reported tests 全部通过。
独立容器为 `babybus-avoidance-jazzy-20261024`，源码、依赖、镜像和网络设置
见 `.verification/ros-avoidance-20261024/environment.json`。旧容器绑定没有改变。

## 功能与证据边界

- 公共 YAML 继承整合版：停车保护开启、目标 0.2 m/s、触发间隙 0.15 m、
  source/receipt 年龄上限分别 0.30/0.20 s；保留显式恢复。
- 车身尺寸仍是暂定值；停车走廊不代表绕弯时的车身扫掠覆盖。
- 操作者曾报告前一时效配置在 car19 行驶和停车成功；整合版新增的扫描质量
  修复仍需实车重验，没有测得制动距离或重复成功率。
- 单线程 executor 超过标称 50 ms 周期的问题仍保留，不能宣称硬实时。
- 本工作区的离线实验已实现 LiDAR 候选硬筛选、绕行侧保持、静态障碍记忆和
  执行滚动复查；受阻 baseline 回退也经过同周期门控。紧急停车保留最高优先级。
- 实车/ROS 主循环的主动绕行接线、实际计算预算和一般场景验收尚未完成。

后续开发只在这个分支进行；旧工作区和部署包作为可追溯的对照版本保留。
