# 短道路生成与感知/planner 筛查

## 当前入口：独立边界候选池（2026-10-31）

```powershell
$py = '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe'
& $py -m offline.lattice_training.screen_boundaries --output .verification/new-boundary-pool
& $py -m offline.lattice_training.screen_roads_review --input .verification/new-boundary-pool
& $py -m offline.lattice_training.screen_roads_failures --input .verification/new-boundary-pool
& $py -m http.server 8799 --bind 127.0.0.1 --directory .verification/new-boundary-pool
```

打开 `pool.html` 查看推荐、全部运行通过、全部生成/拒绝案例，按拓扑和宽度组合筛选，
再跳入 `viewer.html` 逐帧核对真实桶、检测和当前感知/规划。默认 12 个固定种子 × 6 类，
共 72 条候选；同种子镜像及所有观测版本保持同一划分。推荐最多 24 条，先覆盖
拓扑 × 宽度组合，并在 compound 中优先较强但符合车辆限位的弯形。没有合格项的
组合明确记录在 `selection.json`，不把不合格项补成推荐。

左右边界先分别随机生成控制点、转折位置和局部鼓包，再拟合其间的中间参考路线。
U-turn 包含前宽后窄、前窄后宽、两端近似等宽和弯中收窄；compound 包含不等长反向弯、
偏移的弯顶及短弯段。固定外移桶位、帧高斯噪声和相机条件沿用下述设置。

先检查原始边界是否自交、局部法向净宽是否在 0.5–1.5 m，再检查整车几何、转角和速率。
几何不合格的道路仍存 `road.json`、`geometry.json` 和候选池图片，不进入传感器巡检。
**边界缺失不作为运行筛选条件。每个实际 1 Hz 规划作业都必须返回 VALID，并至少有一条
有限、无碰撞的候选；有 NO_ROAD、无有效候选或异常的道路均不入选。** 三种观测均通过
才进入可用集合。候选池全体与筛选后的推荐集分别报告，不能把推荐集当作无偏评测结果。

巡检先静止采集 8 帧（0.4 s），这段只预热感知，不调用 planner；从行进阶段开始提交
1 Hz 规划，所有实际提交作业均纳入筛选。没有忽略已提交但失败的预热作业。

Dream Gym 0.4.0 的原生 Road 只支持直线/圆弧，因而使用由拟合路线导出的原生道路承载
观测基础设施；它的对称车道不是物理真值。真实桶直接来自独立边界，净宽、车身边缘与
边界的距离均对原始物理边界计算。原生地图/预览不进入感知或 planner。没有新增依赖、
宽度先验或第二套感知/planner。巡检仍为规定姿态运动，尚未执行 policy 或控制闭环。
承载道路的自动锥桶路径显式清空，避免在急弯上编译一套不存在的偏移桶；相机只读取
随后安装的独立边界实体桶。

`pool.json` 保留全部候选及拒绝原因，`selection.json` 保存完整可用集合和推荐子集；
`runtime-source/` 固定本次生成与运行源码。边界缺失事件仍可跳转，但新批次的
`failure-index.json` 不将其计入筛选失败；所有观察事件另存 `event-index.json`。

## 历史入口与共同约束

先生成固定道路，再规定车辆沿中心线移动，检查当前感知与 native planner 的输出。
这一步不训练 policy，不用 PID 或 C* 跟踪，也不证明车辆已经能够自主通过道路。
现有感知、planner、共同 YAML、模型权重不改动。

## 道路与输入

- 当前版净宽 0.5–1.5 m 指**锥桶内沿之间**的通行宽度。半径 0.04 m 的桶对应
  0.58–1.58 m 的边界中心线间距；道路规范仍用后者表达。历史 v1 的 0.3 m
  下限及其记录保留，不用新版生成器重解读旧轨迹。
- 左右 S、U 和三个交替弯向的复合道路由短直线/圆弧片段组成，曲率逐段渐变，
  每条道路的宽度控制点经平滑插值连接。范围边缘、普通宽度和窄路层次在每个划分内覆盖。
- 读取 notebook 的车辆模型：轴距、车身尺寸、转向角与转向速率均使用原值。
  从几何中线姿态的左右车身余量计算平滑横移，再反解调整路线的质心运动学。
  路线按等行驶弧长采样，转角/速率超限的调整不采用；只按车身几何余量选参考路线，
  不使用感知或 planner 的成败选路线。独立检查车身/锥桶接触，不通过仍保留原因。
- regular 沿各侧边界以 0.18 m 弧长布桶；irregular 初始边界弧长桶距 0.12–0.25 m，随机缺桶后
  修复过长间隙，同时限制沿名义边界弧长及实际相邻桶中心距离不超过 0.5 m。
  每个不规则桶沿该站位向路外的法线固定偏移 0–8 cm；不向内侵占名义净宽。
  固定桶位独立于帧噪声；若外移使实体桶相互穿透，则重采该桶的外移量，必要时省略并
  重新检查最大间隙。桶半径 4 cm，实体桶中心距离至少 8 cm。视角重叠和重复检测仍保留。
  noisy 复用 irregular 的真实桶，叠加每帧独立的零均值径向高斯误差和重复检测。
  标准差每条路固定采自 0.08–0.12 m，不再使用旧版的相关累积漂移。
  其他道路的局部干扰桶仍由 `--no-extra-cones` 关闭；当前复查批次始终关闭。
  “重叠”在这里包含角度投影重叠、干扰链及检测重复，不生成相互穿透的物理桶。
- 先由原生相机按原视场/量程/检测容量生成可见桶，再增加测距误差；不补出视场外的桶，
  不新增近距离截断。径向误差仅沿虚拟相机射线改变距离。误差幅度未按实车标定。
- 雷达为 dummy。传感器 20 Hz、相机延迟 50 ms、native 规划 1 Hz、48 条候选。
  启动静止预热后，规定沿调整参考路线以 0.5 m/s 移动；感知/规划失败不会终止巡检。
- 末端 2 m 只作显式感知支撑，不改变任务终点。图中的中心线及真实桶仅作环境与被动评估，
  不作为感知或 planner 输入。查看器分开画道路几何中线和实际巡检参考路线。

## 运行

输出目录必须不存在。以下生成 24 个道路案例和 72 次观测巡检；Train/Validation/Test 为
12/6/6，同种子镜像与所有观测版本保持同一划分。所有种子在筛查前固定，不反复抽到成功。

```powershell
$py = '../Baby-Bus/.verification/planner-sim-venv/Scripts/python.exe'
& $py -m offline.lattice_training.screen_roads --output .verification/new-road-screen --no-extra-cones --paired-noise
& $py -m offline.lattice_training.screen_roads_review --input .verification/new-road-screen
& $py -m http.server 8795 --bind 127.0.0.1 --directory .verification/new-road-screen
```

打开 `http://127.0.0.1:8795/viewer.html`。可以按划分、道路和观测条件逐帧对照：真实桶、
采集时刻相机视场及检测、当前感知输出、真实 1 Hz 规划帧的候选。
另有 `REPORT_CN.md`、三组道路总览和机器可读汇总。逐帧记录只保存在本地。

## 解释筛查结果

中心线几何、至少单侧边界输出、中线输出、planner VALID 和完整覆盖率分别记录。
输出可用不代表支路关联一定正确；中心线几何失败也不证明不存在其他合法行驶轨迹。
规定姿态巡检不执行动力学控制，不能混称实际闭环或最高可行速度测试。

正式训练前应人工查看道路是否合理，排除不符合目标场景的物理布局；其余感知/规划失败
作为固定难例保留。不能通过过滤噪声版本、拆分同一道路或改感知门槛来制造通过率。

## 额外道路桶移除对照

以下对照结果属于历史 v1。当前版的独立高斯噪声及固定外移桶位不能与它逐值混比。

`--no-extra-cones` 仅删除 `adjacent_road` 物理桶，保留原道路与全部真实边界桶（包括缺桶）、
原有不均匀桶距、测距漂移与重复检测。噪声对照两组均使用 `--paired-noise`：按原始桶身份
和采集 tick 固定噪声过程，避免删除额外桶改变共同桶的随机数序列。
传感器容量造成的可见边界桶数量变化仍是干扰桶本身的影响，不人工补齐。

```powershell
& $py -m offline.lattice_training.screen_roads --output .verification/extra-on --paired-noise
& $py -m offline.lattice_training.screen_roads --output .verification/extra-off --paired-noise --no-extra-cones
& $py -m offline.lattice_training.screen_roads_review --input .verification/extra-off
```

旧默认顺序噪声与该配对噪声的数值序列不同，不能把两次差异都归因于移除桶；应比较
本节同一噪声模式下的新旧两组。无噪声视图仍可与此前同道路记录核对。

2026-10-30 的 24 路配对试验中，移除其他道路桶后，带测距漂移条件下的全程边界输出
从 1/24 提高到 13/24，全部 planner 作业有效从 6/24 提高到 17/24。
同时要求同一道路在三种观测条件中均通过中心线几何、边界输出和候选生成，结果为
0/24 → 10/24。它支持额外道路桶是该批次的重要干扰源，不代表剩余缺桶和几何问题已解决。
这次只增加可切换的离线试验选项，未改感知、planner 或实车默认配置。

## 失败位置

在已有筛查目录生成故障标注，不重新运行车辆、感知或 planner：

```powershell
& $py -m offline.lattice_training.screen_roads_failures --input .verification/extra-off
& $py -m offline.lattice_training.screen_roads_review --input .verification/extra-off --viewer-only
```

`FAILURE_LOCATIONS_CN.md` 汇总失败站位和各道路三种条件的区间图；`failure-index.json` 和
各视图的 `failures.json` 保存跳转帧、时间、原因和逐帧几何余量。后者由原始规定姿态与
已冻结的真实道路/锥桶重新计算，并要求与原筛查的最小余量一致。
感知中断仅指没有导出任何一侧边界；没有中线但仍有可用单侧边界，不归入此项。
planner 只标真实 1 Hz 作业的失败，不能把两次作业间的帧当作新规划失败。

查看器提供“只列失败道路”、失败区间下拉框和首个/上一处/下一处失败跳转。
链接可带 `?case=道路ID&view=noisy&frame=帧号&failures=1` 定位同一份原始证据。
图中车身几何失败是规定中心线走法的被动检查，不是动态驾驶碰撞；不据此断言其他路线也不可通行。
