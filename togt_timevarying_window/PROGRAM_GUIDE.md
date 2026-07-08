# DynaTOGT 程序结构与调用关系

本文档说明 `togt_timevarying_window/` 子项目中各个程序的职责、核心数据结构和它们之间的调用关系。该子项目是独立的 DynaTOGT 动态时变窗口穿越实验，不依赖 `gap_step/` 主线 PPO。

## 总体数据流

```text
environment.py
  构造 WindowTrack / DynamicWindow / MotionProfile
        |
        v
optimizer.py
  DynaTOGTOptimizer.solve()
  选择顺序 -> 离散 warm start -> L-BFGS-B 优化 -> 生成 DynaTOGTPlan
        |
        +--> trajectory.py
        |      build_trajectory() 生成 Hermite 连续轨迹
        |
        v
baselines.py
  包装 DynaTOGT、静态 TOGT、离散动态、中心点 waypoint 等对照方法
        |
        +--> demo.py
        |      单次终端演示
        |
        +--> export_demo.py
        |      单次演示导出 CSV/PNG/GIF
        |
        +--> experiments.py
               批量实验，生成 summary.csv 和所有结果文件

visualize.py
  被 export_demo.py / experiments.py 调用，用于导出 CSV、PNG、GIF
```

## 核心概念

`WindowTrack` 表示一个完整任务，包含起点、终点、窗口列表和穿越顺序。穿越顺序使用 0-based 窗口索引保存，对外展示时转换成 `G1 -> G6 -> ...`。

`DynamicWindow` 表示一个三维动态窗口。它由局部二维形状、初始三维中心、初始姿态和 `MotionProfile` 组成。优化器选择的是穿越时间 `t_i` 和局部穿越点，`DynamicWindow` 负责把局部点映射到时刻 `t_i` 的世界坐标窗口 `G_i(t_i)`。

`DynaTOGTPlan` 是一次求解的完整结果，包含穿越时间、穿越点、局部点、关键点、连续轨迹、总代价、成功标记和优化耗时。后续评估、导出和可视化都围绕它展开。

## 各程序职责

### `geometry.py`

几何基础工具层，不关心具体场景和优化算法。

主要内容：

- `Shape2D`：定义窗口在局部平面内的二维形状，包括矩形、圆形近似、多边形和斜四边形。
- `rotation_matrix()`：把 yaw/pitch/roll 转成三维旋转矩阵。
- `local_from_unconstrained()`：把优化变量映射到凸窗口内部，是“无约束变量 -> 合法穿越点”的关键。
- `point_in_convex_polygon()` / `convex_margin()`：判断点是否在窗口内，并计算到边界的安全裕度。
- `sample_polygon()`：为离散 warm start 生成窗口内部候选点。
- `path_length()`：计算轨迹采样点折线长度。

关系：

- 被 `environment.py` 用于窗口几何、局部/世界坐标映射和裕度计算。
- 被 `optimizer.py` / `trajectory.py` 用于路径长度指标。

### `environment.py`

场景和动态窗口定义层。

主要内容：

- `MotionProfile`：描述窗口平移、旋转、缩放随时间变化的周期函数。
- `DynamicWindow`：提供 `center_at()`、`basis_at()`、`point_from_local()`、`contains()` 等方法，负责动态窗口 `G_i(t)` 的几何计算。
- `WindowTrack`：保存完整任务。
- `canonical_track()`：构造默认六窗口场景。
- `random_track()`：构造随机场景。
- `make_scenario()`：把 CLI 中的场景名称转换为 `WindowTrack`。

关系：

- 是 `optimizer.py`、`baselines.py`、`demo.py`、`export_demo.py`、`experiments.py` 的场景来源。
- `visualize.py` 也会调用窗口在不同时刻的姿态来绘图。

### `trajectory.py`

连续轨迹生成和采样层。

主要内容：

- `PolynomialTrajectory`：保存 Hermite 轨迹采样结果和轨迹指标。
- `build_trajectory()`：根据起点、穿越点、终点生成连续轨迹。
- `hermite_position()`：查询任意时刻的分段三次 Hermite 插值位置。

关系：

- 被 `optimizer.py` 调用，用于把优化出的关键点变成连续轨迹。
- 轨迹的 `max_speed`、`max_acceleration`、`mean_jerk` 会进入优化目标和实验汇总。
- `visualize.py` 导出轨迹采样点到 CSV，并用采样点绘制 PNG/GIF。

### `optimizer.py`

DynaTOGT 算法核心。

主要内容：

- `DynaTOGTConfig`：速度、加速度、jerk、离散搜索、L-BFGS-B 和代价权重配置。
- `DynaTOGTOptimizer.solve()`：求解入口。
- `_warm_start()`：离散搜索初值，枚举未来时间和窗口内部候选点。
- `_objective()`：连续优化目标，包括时间、路径长度、可飞性惩罚和窗口裕度惩罚。
- `_decode()`：把优化变量解码为 `DynaTOGTPlan`。
- `_validate_plan()`：验证每个穿越点是否位于对应动态窗口内。
- `plan_metrics()`：把计划整理成实验统计指标。

变量向量格式：

```text
[duration_0, duration_1, ..., duration_N,
 z_0_u, z_0_v, z_1_u, z_1_v, ..., z_(N-1)_u, z_(N-1)_v]
```

其中 `duration` 决定穿越时间，`z` 通过 `DynamicWindow.point_from_unconstrained()` 映射为窗口内部局部点。

关系：

- 向上被 `baselines.py` 包装成不同对照方法。
- 向下调用 `environment.py` 的窗口几何和 `trajectory.py` 的轨迹生成。

### `baselines.py`

实验对照方法封装层。

主要内容：

- `DynaTOGT`：动态窗口 + 连续优化。
- `DiscreteDynamic`：动态窗口 + 离散 warm start，不做连续优化。
- `StaticTOGT`：冻结窗口规划，再按真实动态窗口评估。
- `WaypointCenter`：只穿越窗口中心。
- `ShuffledDynaTOGT`：搜索一次性 permutation 顺序的动态对照。
- `baseline_metrics()`：统一计算不同基线的输出指标。

关系：

- 被 `demo.py`、`export_demo.py`、`experiments.py` 直接调用。
- 对外隐藏 optimizer 内部细节，让 CLI 只需要指定 baseline 名称。

### `visualize.py`

结果导出和展示层。

主要内容：

- `export_plan_csv()`：导出 crossing 行和 sample 行。
- `draw_plan_png()`：导出静态中文展示图。
- `draw_plan_gif()`：导出动态穿越 GIF。
- `_project()`：把三维轨迹和窗口投影到二维示意图。

CSV 中关键字段：

- `contains`：穿越点是否位于当前动态窗口内。
- `plane_error`：穿越点到窗口平面的法向距离。
- `gate_margin`：穿越点到窗口边界的安全裕度。
- `local_u/local_v`：穿越点在窗口局部平面内的坐标。

关系：

- 被 `export_demo.py` 和 `experiments.py` 调用。
- 不参与优化，只负责把 `DynaTOGTPlan` 变成可检查的文件。

### `demo.py`

单次终端演示入口。

典型命令：

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
```

职责：

- 解析场景、模式、顺序、基线和迭代次数。
- 调用 `solve_baseline()` 求解。
- 在终端打印 success、order、duration、path_length、cost、margin 和动力学指标。

适用场景：

- 快速确认某个场景/顺序是否能求解。
- 调试优化参数，不需要生成图片和 CSV。

### `export_demo.py`

单次演示导出入口。

典型命令：

```bash
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
```

职责：

- 解析单个场景和穿越顺序。
- 求解对应计划。
- 导出轨迹 CSV、静态 PNG 和动态 GIF。
- 打印文件路径和核心指标。

适用场景：

- 生成组会展示材料。
- 验证重复穿越顺序，例如 `G1,G6,G1,G3,G2,G5,G4,G2`。

### `experiments.py`

批量实验入口。

典型命令：

```bash
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
python -m togt_timevarying_window.experiments --suite default --outdir togt_timevarying_window/results
```

职责：

- `smoke`：只跑 canonical 场景和少量基线，用于快速回归。
- `default`：跑 canonical、运动类型消融、快慢动态和随机场景。
- 为每个场景/基线导出 CSV。
- 为 DynaTOGT 或 smoke 套件导出 PNG/GIF。
- 写入 `summary.csv`。

适用场景：

- 回归测试实验产物。
- 对比不同基线在动态窗口任务中的成功率和轨迹质量。

### `tests/test_dynatogt.py`

自动测试。

覆盖内容：

- 动态窗口确实会移动、旋转、缩放。
- 无约束变量映射不会跑出窗口。
- DynaTOGT 保持默认指定顺序。
- DynaTOGT 支持重复窗口序列。
- Hermite 轨迹在穿越时刻精确命中穿越点。
- `demo.py`、`export_demo.py`、`experiments.py` 三个 CLI 入口可运行并产出文件。

## 典型调用链

### 运行一次终端 demo

```text
demo.py main()
  -> make_scenario()
  -> solve_baseline()
  -> DynaTOGTOptimizer.solve()
  -> build_trajectory()
  -> baseline_metrics()
  -> print()
```

### 导出一次演示

```text
export_demo.py main()
  -> make_scenario()
  -> solve_baseline()
  -> export_plan_csv()
  -> draw_plan_png()
  -> draw_plan_gif()
```

### 跑批量实验

```text
experiments.py main()
  -> suite_tracks()
  -> run_suite()
       -> solve_baseline()
       -> baseline_metrics()
       -> export_plan_csv()
       -> draw_plan_png() / draw_plan_gif()
  -> write_summary()
```

## 三种规划模式

`static`：冻结窗口运动，只使用初始窗口几何规划。它用于静态 TOGT 对照，导出评估时仍可按真实动态窗口检查是否失败。

`ordered_dynamic`：主算法模式，按用户指定的顺序穿越动态窗口。该顺序可以重复，例如 `G1 -> G6 -> G1`。

`shuffled_dynamic`：对照模式，用 beam search 搜索一次性 permutation。它不表达重复穿越任务。

## 输出目录

默认输出位于：

```text
togt_timevarying_window/results/<suite>/
  summary.csv
  trajectories/*.csv
  figures/*.png
  gifs/*.gif
```

其中 `summary.csv` 用于横向比较，`trajectories/*.csv` 用于数值验证，`figures/*.png` 和 `gifs/*.gif` 用于展示。
