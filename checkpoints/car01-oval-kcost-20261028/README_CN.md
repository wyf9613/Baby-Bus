# Checkpoint：1 号车椭圆环线 kcost（2026-10-28）

10-28 收工时 1 号车现场用的包：lattice 2.1 曲率代价（kcost，`shadow.weight_curvature: 10.0`），0.5 m/s，不限距离。同目录还有同一份代码的 k0 对照包（权重 0）。

- 代码：提交 `5ddba23`。WSL 门禁 offline PASS、ros gate PASS（`~/ai4r-evidence/ros-20261028-154814`）。tag `checkpoint-car01-oval-kcost-20261028`。
- 本目录的 `policy_node.py` 就是 `5ddba23` 的 `scripts/policy_node.py`（`3270f8d2`）。`mpc-car-run` 后来又加了反向重配（`03adfb7`），所以当前分支上的 `scripts/policy_node.py` 和它不同，部署时**用这里的副本**。
- 每个包都包含：noprior（`fusion_use_prior: false`）、A（`mpc.geometry_glitch_tolerable: true`）、B'-1（`estimation.keep_width_on_geometry_glitch: true`）。kcost 和 k0 的 yaml 只差 `weight_curvature` 这一行。
- 原因、数据和 Gym 结果见 [MPC_PROGRESS_LOG.md](../../docs/MPC_PROGRESS_LOG.md) 10-28 第 10、11、13 节。

## 文件

| 文件 | SHA256 前 8 位 | 说明 |
| --- | --- | --- |
| `policy_node.py` | `3270f8d2` | 所有包共用；部署时上传为车上的 `scripts/policy_node.py` |
| `config/traxxas_vehicle_interface.yaml`（仓库） | `6e025315` | 转向区间 ±1.0、trim 0；没改 |
| `<包名>/ai4r_policy.yaml` | 见下表 | 部署时上传为车上的 `config/ai4r_policy.yaml` |
| `manifest.json` | — | 完整哈希和重新生成方法 |

| 包 | yaml | 曲率代价 | 颜色 |
| --- | --- | --- | --- |
| **`v050-nolimit-kcost`** | `f138f4c1` | 10.0 | 黄左 |
| **`v050-nolimit-blueleft-kcost`** | `0e291ac7` | 10.0 | 蓝左 |
| `v050-nolimit-k0` | `12f05a0b` | 0.0 | 黄左 |
| `v050-nolimit-blueleft-k0` | `89cb25b9` | 0.0 | 蓝左 |

参数统一为：0.5 m/s、joint、硬超速线 0.7、N 6、lattice 长度 1.4 m、check travel、距离和时间都不限。

## 实车结果（1 号车，10-28）

| 场地 | 包 | 结果 |
| --- | --- | --- |
| 椭圆环线，逆时针 | k0 / kcost | 最长一次 17.6 m / 19.8 m（约 1.7 / 1.9 圈） |
| 椭圆环线，顺时针 | k0 / kcost | 最长一次 18.8 m / **22.8 m（约 2.6 圈）** |
| 每米跟踪（kcost 比 k0） | — | 转向突变 4.61 → 3.88（逆时针）、7.28 → 3.29–5.08（顺时针）；打满方向 46% → 42%、32% → 21–28%；横向误差 p90 0.33 → 0.26 m、0.27 → 0.14–0.28 m |
| 窄路，逆时针 | kcost | 5.9 m（+192°）、13.7 m（+425°），两次都越出外侧 |

已知问题：
- 所有自动停车都是判路造成的，包括急弯里配对失败超过 0.5 s，以及几乎看不到锥桶；
- 逆时针左弯会冲出外侧，因为 1 号车打满左转只有 20–21°，正好顶在模型上限；
- 窄路上路宽可能被单帧错误配对带偏。

## 恢复 / 部署

1. 车上 workspace 要有完整的包框架：`package.xml`、`CMakeLists.txt`、`dream_interfaces.required.json`、`launch/`、`config/`、`tests/`。
2. 上传本目录的 `policy_node.py`，以及所选包的 `ai4r_policy.yaml`。可以把本目录当作 `Deploy-StartFix` 的包目录：先把 `policy_node.py` 复制进所选包的子目录，再运行 `Deploy-StartFix checkpoints\car01-oval-kcost-20261028\<包名>`。
3. 在车上用 `sha256sum` 核对哈希：`3270f8d2…` 加上表里的 yaml 哈希。新上传的文件要先 `chmod +x` policy_node，然后 `dream build ros student && dream runtime restart ai4r_policy`。
4. 参数核对时，`shadow.weight_curvature` 应该是 10.0（kcost）或 0.0（k0）。`estimation.reorient_reversed_pairing` 应该报 `Parameter not set`，因为这份代码里还没有这个参数。
