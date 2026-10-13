# 2026-10-13 逐条参考失效分类

从全部 15 个归档 run.jsonl 解析 Planning rejection details。只分类顶层 source_deadline_during_planning；其他原因另计。没有运行规划器或连接车辆。

有效期按部署规则还原为 min(0.2, 各输入年龄上限−worker 输入年龄)，而不是取外层当前道路年龄。输入限制来自每条诊断；轨迹上限 0.2 s 来自当天部署配置。

分类表示决定合并截止时刻的最早限制。原代码只有一个 reference_expired 判断，没有独立报告哪类源过期。准确主进程取用时刻未记录，因此不能判断该时刻还有哪些非最早限制也同时过期。worker 行表示其 elapsed_s 达到还原的输入寿命；交付前排队时间不包含在这个耗时比较中。

## 汇总

| 拒绝阶段 | 最早限制 | 条数 |
| --- | --- | ---: |
| 主进程取用 reference_expired | 旧 IMU | 4 |
| 主进程取用 reference_expired | 道路 | 46 |
| worker 内期限检查 | 旧 IMU | 2 |
| worker 内期限检查 | 道路 | 2 |

## 逐条记录

剩余时间和规划耗时单位 ms。任务编号仅在同一轮内有意义；链接定位原始日志。

| 轮次 | 任务 | 阶段 | 道路剩余 | 旧轮速剩余 | 旧 IMU 剩余 | 轨迹窗口 | 最早限制 | 规划耗时 | 原始行 |
| --- | ---: | --- | ---: | ---: | ---: | ---: | --- | ---: | --- |
| budget100-122645 | 4 | 取用过期 | 131.845 | 225.686 | 227.088 | 200.000 | 道路 | 64.627 | [177](evidence/runs/drive-budget100-122645/run.jsonl#L177) |
| budget100-122645 | 6 | 取用过期 | 133.579 | 235.555 | 209.615 | 200.000 | 道路 | 72.930 | [203](evidence/runs/drive-budget100-122645/run.jsonl#L203) |
| budget100-122645 | 9 | 取用过期 | 128.067 | 246.759 | 122.780 | 200.000 | 旧 IMU | 52.961 | [242](evidence/runs/drive-budget100-122645/run.jsonl#L242) |
| budget100-122645 | 16 | 取用过期 | 80.993 | 242.931 | 154.371 | 200.000 | 道路 | 53.522 | [326](evidence/runs/drive-budget100-122645/run.jsonl#L326) |
| budget100-122645 | 18 | 取用过期 | 128.735 | 243.757 | 163.332 | 200.000 | 道路 | 63.645 | [352](evidence/runs/drive-budget100-122645/run.jsonl#L352) |
| budget100-122645 | 20 | 取用过期 | 133.469 | 242.891 | 187.044 | 200.000 | 道路 | 63.801 | [380](evidence/runs/drive-budget100-122645/run.jsonl#L380) |
| budget100-122645 | 22 | 取用过期 | 82.235 | 235.806 | 96.229 | 200.000 | 道路 | 45.125 | [399](evidence/runs/drive-budget100-122645/run.jsonl#L399) |
| budget100-122645 | 24 | 取用过期 | 83.778 | 239.396 | 194.808 | 200.000 | 道路 | 45.786 | [425](evidence/runs/drive-budget100-122645/run.jsonl#L425) |
| budget100-122645 | 26 | 取用过期 | 77.400 | 235.769 | 150.257 | 200.000 | 道路 | 57.557 | [447](evidence/runs/drive-budget100-122645/run.jsonl#L447) |
| budget100-122645 | 27 | 取用过期 | 134.044 | 234.522 | 206.884 | 200.000 | 道路 | 59.422 | [461](evidence/runs/drive-budget100-122645/run.jsonl#L461) |
| robustness-120930 | 17 | 取用过期 | 133.114 | 244.232 | 82.755 | 200.000 | 旧 IMU | 32.166 | [365](evidence/runs/drive-robustness-120930/run.jsonl#L365) |
| robustness-120930 | 30 | 取用过期 | 85.241 | 240.281 | 214.244 | 200.000 | 道路 | 26.666 | [514](evidence/runs/drive-robustness-120930/run.jsonl#L514) |
| robustness-narrow-121953 | 5 | 取用过期 | 81.633 | 247.712 | 120.466 | 200.000 | 道路 | 31.811 | [196](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L196) |
| robustness-narrow-121953 | 10 | 取用过期 | 124.019 | 197.854 | 82.397 | 200.000 | 旧 IMU | 33.102 | [259](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L259) |
| robustness-narrow-121953 | 12 | 取用过期 | 113.349 | 243.161 | 114.034 | 200.000 | 道路 | 24.611 | [280](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L280) |
| robustness-narrow-121953 | 14 | 取用过期 | 65.835 | 181.029 | 146.186 | 200.000 | 道路 | 27.850 | [300](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L300) |
| robustness-narrow-121953 | 15 | 取用过期 | 50.184 | 247.116 | 88.641 | 200.000 | 道路 | 26.543 | [311](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L311) |
| robustness-narrow-121953 | 20 | 取用过期 | 84.973 | 243.280 | 145.019 | 200.000 | 道路 | 28.728 | [364](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L364) |
| robustness-narrow-121953 | 21 | 取用过期 | 70.428 | 223.564 | 148.645 | 200.000 | 道路 | 30.758 | [372](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L372) |
| robustness-narrow-121953 | 23 | 取用过期 | 90.348 | 234.635 | 190.000 | 200.000 | 道路 | 29.618 | [397](evidence/runs/drive-robustness-narrow-121953/run.jsonl#L397) |
| robustness-repeat-121106 | 7 | 取用过期 | 90.953 | 239.140 | 191.311 | 200.000 | 道路 | 33.993 | [349](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L349) |
| robustness-repeat-121106 | 14 | 取用过期 | 91.905 | 185.028 | 113.409 | 200.000 | 道路 | 34.071 | [420](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L420) |
| robustness-repeat-121106 | 15 | 取用过期 | 89.262 | 242.925 | 190.508 | 200.000 | 道路 | 31.398 | [430](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L430) |
| robustness-repeat-121106 | 18 | 取用过期 | 90.840 | 244.264 | 211.830 | 200.000 | 道路 | 27.877 | [464](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L464) |
| robustness-repeat-121106 | 19 | 取用过期 | 83.281 | 238.675 | 164.783 | 200.000 | 道路 | 34.771 | [473](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L473) |
| robustness-repeat-121106 | 22 | 取用过期 | 140.513 | 232.937 | 139.050 | 200.000 | 旧 IMU | 24.652 | [503](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L503) |
| robustness-repeat-121106 | 23 | 取用过期 | 87.620 | 246.918 | 189.096 | 200.000 | 道路 | 34.834 | [513](evidence/runs/drive-robustness-repeat-121106/run.jsonl#L513) |
| startup-speed-floor020-01 | 4 | 取用过期 | 90.001 | 198.440 | 145.738 | 200.000 | 道路 | 44.563 | [159](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L159) |
| startup-speed-floor020-01 | 5 | 取用过期 | 93.270 | 230.447 | 208.592 | 200.000 | 道路 | 49.301 | [173](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L173) |
| startup-speed-floor020-01 | 7 | 取用过期 | 91.481 | 238.118 | 204.818 | 200.000 | 道路 | 37.519 | [193](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L193) |
| startup-speed-floor020-01 | 8 | 取用过期 | 90.030 | 229.703 | 184.750 | 200.000 | 道路 | 61.616 | [204](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L204) |
| startup-speed-floor020-01 | 9 | 取用过期 | 91.422 | 241.463 | 164.356 | 200.000 | 道路 | 53.553 | [209](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L209) |
| startup-speed-floor020-01 | 10 | worker 到期 | 143.276 | 242.221 | 16.220 | 200.000 | 旧 IMU | 50.153 | [222](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L222) |
| startup-speed-floor020-01 | 12 | 取用过期 | 77.403 | 246.802 | 150.324 | 200.000 | 道路 | 45.815 | [241](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L241) |
| startup-speed-floor020-01 | 13 | worker 到期 | 71.397 | 242.579 | 44.352 | 200.000 | 旧 IMU | 45.904 | [250](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L250) |
| startup-speed-floor020-01 | 14 | 取用过期 | 142.957 | 241.077 | 157.711 | 200.000 | 道路 | 70.850 | [262](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L262) |
| startup-speed-floor020-01 | 15 | 取用过期 | 86.044 | 243.721 | 157.848 | 200.000 | 道路 | 62.800 | [271](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L271) |
| startup-speed-floor020-01 | 18 | 取用过期 | 82.609 | 244.256 | 134.322 | 200.000 | 道路 | 46.096 | [303](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L303) |
| startup-speed-floor020-01 | 19 | 取用过期 | 138.965 | 216.937 | 211.897 | 200.000 | 道路 | 50.744 | [318](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L318) |
| startup-speed-floor020-01 | 20 | 取用过期 | 90.300 | 225.540 | 181.775 | 200.000 | 道路 | 64.192 | [327](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L327) |
| startup-speed-floor020-01 | 22 | 取用过期 | 93.889 | 240.903 | 208.531 | 200.000 | 道路 | 70.330 | [352](evidence/runs/drive-startup-speed-floor020-01/run.jsonl#L352) |
| startup-speed-floor020-02 | 5 | 取用过期 | 140.736 | 230.866 | 212.353 | 200.000 | 道路 | 45.354 | [270](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L270) |
| startup-speed-floor020-02 | 6 | 取用过期 | 78.833 | 229.534 | 168.647 | 200.000 | 道路 | 63.587 | [280](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L280) |
| startup-speed-floor020-02 | 8 | 取用过期 | 136.078 | 232.521 | 187.084 | 200.000 | 道路 | 61.248 | [301](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L301) |
| startup-speed-floor020-02 | 9 | 取用过期 | 89.025 | 231.907 | 179.173 | 200.000 | 道路 | 49.490 | [310](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L310) |
| startup-speed-floor020-02 | 11 | 取用过期 | 134.658 | 246.520 | 164.776 | 200.000 | 道路 | 73.288 | [336](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L336) |
| startup-speed-floor020-02 | 12 | 取用过期 | 81.778 | 206.235 | 89.490 | 200.000 | 道路 | 55.185 | [347](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L347) |
| startup-speed-floor020-02 | 13 | 取用过期 | 140.589 | 225.476 | 149.901 | 200.000 | 道路 | 69.676 | [358](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L358) |
| startup-speed-floor020-02 | 14 | 取用过期 | 91.300 | 247.967 | 160.754 | 200.000 | 道路 | 52.305 | [365](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L365) |
| startup-speed-floor020-02 | 15 | 取用过期 | 84.570 | 197.626 | 112.016 | 200.000 | 道路 | 35.134 | [370](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L370) |
| startup-speed-floor020-02 | 16 | worker 到期 | 42.521 | 242.566 | 152.772 | 200.000 | 道路 | 63.465 | [382](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L382) |
| startup-speed-floor020-02 | 19 | 取用过期 | 89.541 | 242.594 | 178.964 | 200.000 | 道路 | 70.751 | [418](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L418) |
| startup-speed-floor020-02 | 20 | 取用过期 | 131.143 | 247.484 | 178.387 | 200.000 | 道路 | 77.960 | [430](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L430) |
| startup-speed-floor020-02 | 22 | worker 到期 | 61.015 | 184.142 | 149.488 | 200.000 | 道路 | 62.102 | [454](evidence/runs/drive-startup-speed-floor020-02/run.jsonl#L454) |

## 未归为参考到期的拒绝

- insufficient_lattice_coverage: 40 条
- invalid_estimates_or_boundaries: 23 条
- lattice_time_budget_exceeded: 26 条

这些计数只覆盖打印的拒绝日志，不是全部规划任务的失败率。无明细的早期几何/监测中止未纳入期限分类。

## 代码位置

- scripts/policy_node.py:2066：逐源计算剩余寿命。
- scripts/policy_node.py:2341：lifetime=min(reference_lifetime_s,*deadlines)，worker 耗时达到 lifetime 时拒绝。
- scripts/policy_node.py:2866：transport_control_reference 计算 timestamp_s+valid_for_s-now_s；小于等于零抛 reference_expired。
- scripts/policy_node.py:4689：主进程把上述异常统一标记为 source_deadline_during_planning。
