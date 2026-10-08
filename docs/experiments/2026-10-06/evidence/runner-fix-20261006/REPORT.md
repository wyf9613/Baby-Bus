# 2026-10-06 runner 性能诊断与实车单轮测试

已在车端部署进程组扫描优化。车端 DREAM 基线为 91f6ed4；保留完整身份校验、UID/session 校验、进程组存在检查及停止前检查。使用 os.getpgid 筛选候选，只为候选进程读取 stat/status/stat。没有修改车辆安全超时。未提交或推送。

- 单次扫描：75.96 ms → 1.70 ms，约 44.6 倍加速；见 scan-benchmark.json。
- runner 定向测试：53 项通过；完整 bin/dream verify fast：1264 项通过，253.831 秒。日志为 runner-tests.log、verify-fast.log。
- cProfile 记录了 runner 的完整启动/监督/停止过程，原始 before/after-runner.pstats 和对应 txt 都在此目录。生产代码没有保留临时 profiler，避免持续开销。
- 整体 profile 还显示运行记录重新校验、参数 YAML 重复序列化开销；按用户时间约束，本轮未继续优化。
- before/after-unprofiled-cpu.json 的短采样包含启动阶段，且采样时完整检查还在后台运行，不能把它当成严格 CPU A/B。final-services-cpu.json 在完整检查结束后采集，四个 runner 平均每个约 72.2% CPU，策略节点约 32.2%，车辆接口节点约 11.3%。2026-10-06 综合复盘重新按进程统计，修正了此前误写的 36.1%；统计和限制见项目 docs/experiments/2026-10-06/。
- 两轮 10 秒共存诊断中车辆 connected/ready 始终为 1，session/连接次数不再变化。启动过程曾重连，reason 仍保留 worker receive-service gap，因此不宣称连接问题已彻底消除。

## 实车测试结果

用户明确授权远程启动车辆。启动车前发现 student 策略和 YAML 再次被替换为简化代码；已备份到 student-overwrite-backup/，恢复 main 的 MVP，并保留 planning.max_source_age_s=0.3、max_near_x_m=0.7。student overlay 构建成功。未更改驱动上限、速度、里程或运行时间限制。

- 确认 control.enabled=true、mode=mvp、距离限制 3 m、时间限制 30 s、目标速度 0.2 m/s、drive 上限 0.15、steer 上限 0.5。
- 有双侧锥桶、轮速、有效角速度和 RC 信号，车辆连接就绪；显式 Enable 后观察到 Enabled，再请求一次策略状态 3。
- 策略 active 约 0.441 秒后因 Control reference expired 停止。
- 最大请求 drive 约 0.15；车辆 applied drive 最大约 0.116。
- 本轮轮速最高 0.0 m/s，debug2 里程 0.0 m，未完成 3 m 行驶。遥测不等于对车体运动的视觉确认。
- 最终策略状态 2，车辆 Disabled。没有重复请求运行或绕过参考过期检查。

详细记录：mvp-run-result.json、mvp-run.jsonl、policy-after-run.log。run_mvp_once.py 是有边界检查、停止清理和日志记录的单轮测试脚本，不会自行再次运行。

## 交付与回滚

runner-fix.patch 为部署补丁；runtime_runner.fixed.py 为固定版本；deployed-sha256.txt 记录部署哈希。profile_scan.py、sample_cpu.py 与临时 profiling 版本保留以复现诊断。

车端备份位于 /home/ai4r/runner-fix-evidence/。如需恢复 runner 原始源码，在车端执行 bash ~/runner-fix-evidence/rollback.sh，然后在现场准备好时重启相关服务。脚本仅恢复源码，不自动操作车辆。

## 仓库归档说明

本阶段报告的原路径以实验时工作目录为准。本次仅归档选定证据；完整结论、CPU 更正和本周计划见本目录上层的综合实验文档，完整遥测/备份保留在原工作区。
