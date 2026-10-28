# Checkpoint：1 号车 U 弯 `-noprior`（2026-10-28）

**目前最好的一套设置**。急 U 弯（只看得到外圈锥桶）在 0.2 和 0.5 m/s 下都不再中途往外拐。

- 代码：分支 `fusion-prior-reset`，提交 `a21f8cb`（之后只改文档）。tag `checkpoint-car01-uturn-noprior-20261028`。
- WSL 门禁：offline PASS、ros gate PASS（`~/ai4r-evidence/ros-20261028-112040`）。
- 和 `mpc-car-run` `9a5d342`（v050 包，policy_node `bd5ce140…`）相比只多一项：`shadow.fusion_use_prior`，这些包里都是 `false`。原因和数据见 [MPC_PROGRESS_LOG.md](../../docs/MPC_PROGRESS_LOG.md) 10-28，给其他组的说明见 [FUSION_PRIOR_GUIDE_NOTE_CN.md](../../docs/FUSION_PRIOR_GUIDE_NOTE_CN.md)。
- **还没合进 `mpc-car-run`**。

## 文件

| 文件 | SHA256 前 8 位 | 说明 |
| --- | --- | --- |
| `scripts/policy_node.py`（本提交） | `25f9e3dd` | 所有包共用，部署时上传它 |
| `config/traxxas_vehicle_interface.yaml`（本提交） | `6e025315` | 转向区间 ±1.0、trim 0；车辆接口读它 |
| `<包名>/ai4r_policy.yaml` | 见下表 | 部署时上传为车上的 `config/ai4r_policy.yaml` |
| `manifest.json` | — | 全部完整哈希 |

| 包 | yaml | 车速 | 距离 | 纵向 | 颜色 |
| --- | --- | --- | --- | --- | --- |
| **`v050-5m-noprior`**（目前最好） | `45ab0e51` | 0.5 | 5 m / 60 s | joint | 黄左 |
| `v050-nolimit-noprior` | `cbc5533b` | 0.5 | 不限 | joint | 黄左 |
| `v050-5m-blueleft-noprior` | `08fbdacb` | 0.5 | 5 m / 60 s | joint | 蓝左 |
| `v050-nolimit-blueleft-noprior` | `87eb1fd4` | 0.5 | 不限 | joint | 蓝左 |
| `v020-5m-noprior` | `79da2379` | 0.2 | 5 m / 60 s | joint | 黄左 |
| `v020-nolimit-noprior` | `d475398c` | 0.2 | 不限 | joint | 黄左 |
| `v020-5m-blueleft-noprior` | `42268988` | 0.2 | 5 m / 60 s | joint | 蓝左 |
| `v020-nolimit-blueleft-noprior` | `26f95cb4` | 0.2 | 不限 | joint | 蓝左 |
| `v020-nolimit-fb-noprior` | `60a7db80` | 0.2 | 不限 | feedback | 黄左 |
| `v020-nolimit-blueleft-fb-noprior` | `f21674c3` | 0.2 | 不限 | feedback | 蓝左 |
| `v050-nolimit-fb-noprior` | `d216087d` | 0.5 | 不限 | feedback | 黄左 |
| `v050-nolimit-blueleft-fb-noprior` | `38834fb7` | 0.5 | 不限 | feedback | 蓝左 |

feedback 的 4 个包还没上过车。

## 实车结果（1 号车，同一个左 U 弯，黄左）

| 包 | 次数 | 结果 |
| --- | --- | --- |
| 改前 `v050-5m` | 1 | +91° 后往右 -66°，出界 |
| 改前 `v020-5m` | 2 | +84° 后往右越界；+73° 后往右 |
| `v020-5m-noprior` | 3 | 两次跑满 5 m（+85°、+83°）；一次切内侧撞锥 |
| `v020-nolimit-noprior` | 2 | 一次起步估计失败；一次转完整个 U 弯（+155°），但一走一停 |
| **`v050-5m-noprior`** | 2 | 两次跑满 5 m，+142°、+137°，无越界、无碰锥、无重启 |

已知未解决：0.2 下 joint 一走一停（轮速读数冻住 + 切油门循环），0.5 下没出现；0.5 完整出弯（不限包）还没跑；估计层排序失败（Gym 主要停车原因）。

## 恢复 / 部署

1. 本机：`git checkout checkpoint-car01-uturn-noprior-20261028`。
2. 车上 workspace 要有本提交的包框架（`package.xml`、`CMakeLists.txt`、`dream_interfaces.required.json`、`launch/`、`config/`、`tests/`）。10-28 1 号车 workspace 被清空过，当时就是从这里上传的；`config/traxxas_vehicle_interface.yaml` 必须是本提交的版本（±1.0）。
3. 上传 `scripts/policy_node.py` 和所选包的 `ai4r_policy.yaml`（`Deploy-StartFix` 用的是 `.verification\` 里的同名包，内容和这里相同；没有 `.verification` 时把这里的目录当包用，`policy_node.py` 从 `scripts/` 拷）。
4. 车上：`sha256sum` 核对（`25f9e3dd…` + 上表 yaml），`chmod +x` policy_node（新建文件时），`dream build ros student && dream runtime restart ai4r_policy`。
5. 参数核对：`shadow.fusion_use_prior` 必须是 `False`；其余按 [CAR05_LATTICE21_EXEC_RUN_CN.md](../../docs/CAR05_LATTICE21_EXEC_RUN_CN.md) 10-28 的清单。

## 重新生成

```powershell
python tools/make_deploy_yaml.py --mode mpc-exec-lattice21 --car 05 --cones yellow-left --distance 5.0 --run-time-s 60 --speed-profile v05 --out .verification\car05-lattice21-exec-v050-5m-noprior
# 然后把 yaml 里的 fusion_use_prior: true 改成 false（只这一行）
```

0.2 用 `--v-exec-max 0.2` 代替 `--speed-profile v05`；不限用 `--distance 0 --run-time-s 0`；蓝左 `--cones blue-left`；feedback 加 `--longitudinal feedback`。生成后和 `manifest.json` 比对哈希。
