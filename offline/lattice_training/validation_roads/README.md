# 固定掉头验证道路 v2

此目录的 JSON 是具体道路定义，运行时不从种子重新生成。`manifest.json` 保存版本、
语义和每份道路文件的 SHA-256；`validation_roads.validation_case()` 校验后加载。
这 8 条道路用于重复验证，本次没有将其加入训练数据或重新划分数据集。

| 变体 | 道路全宽 | 掉头参考半径 | 构造 |
|---|---:|---:|---|
| original | 2.253501 m | 1.126751 m | 原始半圆掉头；入口内侧直线锥桶、外侧圆弧接直线 |
| half_width | 1.126751 m | 0.563375 m | 通行全宽与掉头参考半径同时减半，直线长度保持 |
| curved | 2.253501 m | 主掉头约 1.296–1.634 m | S 形进出段，45° + 90° + 45° 不同半径圆弧构成 180° 掉头 |
| half_width_double | 1.126751 m | 0.563375 m | 两次半宽半径掉头：左 180°、连接段、右 180°；另有完整镜像 |

每个变体均有左右方向文件，镜像包括障碍横向位置。按用户最终定义，半宽同时减半
道路全宽和掉头参考半径，直线段保持原长；仍满足全宽等于两倍参考半径，内侧在
弯心退化为一点，保留单一内侧直线分隔的构造。掉头段只放外侧锥桶，不用不可见
真值边界替代感知输入。此前只减宽度的 v1 定义和实验保留在历史构型对照目录，
当前运行只加载本目录 v2。
弯曲变体的进出 S 段分别使用半径约 3.944 m、正负 18° 圆弧；主掉头保持总转角 180°。
连续双掉头的连接段提供双侧锥桶，第二次转弯只提供新的外侧锥桶。

道路、锥桶间距、障碍几何和位置均已固定。`base_seed: 713` 只记录创作来源。
道路全宽元数据及真值只用于仿真和 C* 约束，不直接注入感知或 policy；
运行仍允许从真实双边观测估宽。传感器延迟、车辆和共同 C* 目标沿用现有管线。

从仓库根目录运行，输出目录必须不存在：

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.lattice_training.validate_uturns --output .verification/fixed-uturn-native --start-mode road_start
```

默认使用 native 自选、两种中线输入模式。`--start-mode default` 在第一主掉头前 0.9 m
冷启动；`road_start` 从真实入口开始，不跳过弯曲进出段。每段从剩余任务距离确定完整
候选长度，C* 完整执行后以实际终态继续；失败即停止，没有换候选、回退或重新抽状态。

增加冻结 policy 对照时显式传入模型：

```powershell
../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe -B -m offline.lattice_training.validate_uturns --output .verification/fixed-uturn-policy-compare --selectors native,policy --start-mode road_start --model offline/lattice_training/artifacts/2026-10-17-expanded-collection/model.json
```

`--kinds` 可选择 `validation_uturn_original_left`、`validation_uturn_half_width_left`、
`validation_uturn_curved_left`、`validation_uturn_half_width_double_left` 及对应 `_right`。
`--modes perception` 保留实际感知输出；`no_centerline` 明确移除两处中线、保留边界和障碍。
本验证不设置场景宽度先验，要求共同配置 `estimation.known_lane_width_m=0`。

当前 native 的 `unknown_width_reference_scale=4/3`，只将单边、无中线、宽度未知时的
CG 内部参考偏移从 0.45 m 调到 0.60 m，搜索半宽仍为 0.45 m。它不是道路全宽或
车身安全间隙。`--reference-scale 1` 可复现原参考偏移作同路消融，保持其他条件不变。
固定道路变更应明确升级 suite 版本及指纹，保留既有结果，不覆盖历史验证证据。
