# 模块验证与自动回放

验证器调用当前模块并记录实际结果；回放只读取 JSON 记录，不执行算法、不生成
替代轨迹。默认使用观测感知、当前 native planner、native 选择和实际 MVP PID。
policy 质量不作为本轮目标；模型、输入编码和历史数据不重训、不转换。

## 使用

在仓库根目录使用已有虚拟环境。新工作区可用 `--workspace <path>` 指定：

```powershell
$py = '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe'
& $py -m offline.mvp21_sim.validation init
& $py -m offline.mvp21_sim.validation add-road my-road.json
& $py -m offline.mvp21_sim.validation run --road my_road_id
& $py -m offline.mvp21_sim.validation serve
```

默认工作区是 `offline/mvp21_sim/workspace`。初始化提供 8 条固定掉头、33 个固化
随机道路版本。道路定义独立于训练/验证归档；源码仓库只保存运行输入和冻结模型，
验证轨迹、日志及截图保留本地，不随本次提交推送。若本地存在原回放清单，初始化
还会索引原有 164 个案例。页面为 http://127.0.0.1:8792/ 。
`run --all` 验证所有道路；`run --road <文件名去后缀/fixture_id/kind>` 选择道路。
`run --max-steps 100` 可快速检查链路，该结果会明确记录步数上限，不能冒充跑完全路。
`index` 只更新数据索引；`serve --no-watch` 只读回放，不启动新的验证。

可直接放入 `roads/` 的 JSON 支持：原始 road_spec 字典、`{"road_spec": ...}`、
原固定道路 fixture、现有 runtime fixture。原始 spec 必须含 `elements` 和
`cone_paths`；`obstacles` 可空。每条道路的任务长度从几何计算，末端感知延伸
单列配置。fixture 的 `mission_road_length_m` 必须匹配未延伸的道路长度。
修改道路同名文件即可；几何变更记录独立源码/输入指纹。错误 JSON 会在索引
`errors` 中报告，不产生伪造的成功轨迹。

## 模块与频率

修改工作区 `validation.json`，不覆盖 ROS/common 配置：

```json
{
  "schema": "module_validation_v1",
  "modules": {
    "perception": "observed",
    "planner": "native",
    "policy": "native",
    "control": "mvp"
  },
  "frequencies_hz": {
    "physics": 20, "perception": 20, "planner": 1, "policy": 1, "control": 20
  },
  "camera_latency_s": 0.05,
  "planner_latency_s": 0.1,
  "terminal_support_m": 2,
  "max_steps": 2400,
  "stop_tail_s": 1,
  "model_path": "offline/lattice_training/artifacts/2026-10-17-expanded-collection/model.json",
  "cstar_solver": {},
  "cstar_objective": {}
}
```

|模块|可选值|含义|
|---|---|---|
|perception|`observed`|虚拟环境锥桶/传感器经过当前感知模块|
||`perfect`|显式地图道路、边界与障碍真值，状态仍来自测量端口|
||`no_centerline`|受控删除规划/选路输入中线，原感知另存显示|
|planner|`native`|当前唯一 native lattice 实现|
||`mvp1_straight`|单条固定车体前向直线，y=0，长度取当前 shadow.length_m；无搜索、无中线拟合|
|policy|`native`|采用 planner 的 native 选择|
||`model`|当前冻结模型逐条评分 native 接纳的候选|
|control|`mvp`|共享 execution 后的实际 MVP PID 和同 tick 回退|
||`cstar`|原 CStarSolver 求解每个新参考，按 control 频率执行其控制序列；新规划到达时更新|

感知采集、规划提交、候选选择、control 更新独立计时。policy tick 没有新候选池时
空闲；规划快于选路时保留最新候选池，旧池仍留在生成记录里。评分始终使用候选
提交时刻的同坐标观测，不能把延迟候选与当前坐标直接混合。控制低于物理频率时
保持上一个实际动作，并保留参考原坐标；回放按记录姿态正确对齐。

各频率必须能表示为整数物理步；需要 3 Hz 等组合时可把 physics 设为 60 Hz。
物理步长变化也会改变数值积分条件，应视为独立实验。输入延迟、超时和启动门控
仍受当前实际模块约束，配置低频不意味着自动延长实车看门狗或取消已知限制。

C* 使用原 native plant/道路/障碍真值做预测约束；这是显式仿真对照条件，并不
证明其可直接部署。C* 未找到可行解时记录失败，不偷偷切成 PID。它仍可能在
首次 baseline 参考无法拟合时停在 planner 提交之前。单直线只保证一条候选，
不保证弯道通行；碰撞和共享执行检查仍生效。未改动任一算法以抬高完成率。

## 自动更新

`serve` 默认监听源码、common YAML、模型、工作区配置及道路，变化稳定 1 秒后
用新 Python 进程重新导入当前模块。新增/修改道路优先验证对应文件；模块或配置
变化重新执行上一次 `run` 的道路范围。希望持续验证全套时先执行 `run --all`。
运行中继续变更则完成当前记录后再验证最新状态；源码在运行中改变的记录不
晋升为最新结果。仅改 HTML/CSS 时页面自动重新加载模板。

页面每 2 秒读取 `/api/index`，最新结果完成后自动更新。结果 immutable，重新
验证写新目录；加载当前案例的新数据时沿用同一案例身份。每次运行保存：

- `inputs.json`：道路、模块选择、频率、源码 SHA-256。
- `raw/trace.jsonl`：实际状态、动作、控制更新/保持、实际输入、执行来源。
- `raw/submissions.jsonl`、`generation-records.json`、`shadow.jsonl`：提交、生成、选路及因果交付。
- `raw/module-events.json`：独立 policy tick；候选池没有新数据时标为空闲。
- `raw/perception-*`：环境锥桶、真实检测包、输出曲线与采集姿态复核。
- `case.json`：统一回放数据；`record.json`：配置、源码、数据指纹与验收结果。

旧源码运行记录不会被假装成当前模块效果；记录中保留原始指纹。到达终点允许
实际 baseline 回退，不能叫 native-only 通过。所有频率均按仿真时间调度，CPU
时间另记，固定交付延迟不构成真实系统实时性验收。

## 打包

```powershell
& $py -m offline.mvp21_sim.validation package --output build/module-validation-20261018.zip
```

包包含当前工作区源文件、配置、道路、模型和已保存的正式历史证据，包含未提交
修改；附逐文件哈希、Git HEAD/分支、工作树 diff。不改 Git、不发布。
`.git`、`.verification`、虚拟环境、缓存和 build 目录不在包内。依赖安装参考
`offline/planner_sim/requirements.txt` 与 `offline/lattice_training/requirements.txt`；
Dream Gym 保持现有固定版本。解压后在对应根目录运行同一 CLI；绝对历史路径只作
来源记录，新验证使用包内相对道路/模型和当前模块。
