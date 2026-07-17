# Architecture

## Active Runtime Line

```text
window_maze_env.py -> GraphObs -> model.py -> ppo.py
train_window.py    -> curriculum training
evaluate_window.py -> ID/OOD evaluation
visualize_window.py -> GIF rollouts
```

## Environment

- Procedural maze generator.
- `WallRect` for static walls.
- `ApertureWindow` for straight/polyline/curve dynamic window bodies.
- Swept-circle continuous collision.
- Compact graph construction around the route, local cells, and window nodes.

## Observation

`GraphObs` keeps the project-wide graph contract:

```text
global_features
node_features
node_type
edge_index
edge_features
```

Window nodes include current gap state plus future gap width/center features.

## Training

- Pure PPO teacher.
- Explicit `model_old` sampling and `model <- model_old` sync.
- Internal bridge curriculum up to `C5`.
- No planner, BC, expert demonstrations, or fallback controller.

## Offline DynaTOGT Extension

```text
复现/TOGT-Planner-reproduction/   source-level reproduction package
togt_timevarying_window/          standalone DynaTOGT dynamic time-varying window experiment
```

`togt_timevarying_window/` 是独立子项目，不使用旧迷宫环境，也不接入 `gap_step/` PPO 主线。它基于 TOGT 论文的 gate 几何约束思想，把静态 `p(t_i) in G_i` 扩展为动态窗口 `p(t_i) in G_i(t_i)`。

当前 DynaTOGT 架构：

```text
geometry.py      2D/3D 窗口几何、局部点映射、裕度计算
environment.py   DynamicWindow / WindowTrack / MotionProfile / 场景生成
optimizer.py     DynaTOGT warm start + L-BFGS-B 连续优化
trajectory.py    Hermite 连续轨迹和速度/加速度/jerk 采样
baselines.py     WaypointCenter / StaticTOGT / DiscreteDynamic / DynaTOGT
experiments.py   smoke/default 实验套件与 summary.csv
visualize.py     中文组会展示风格 PNG/GIF
```

关键语义：

- `ordered_dynamic` / `static` 的 `--order` 是穿越任务序列，不是 permutation；同一窗口可以出现多次。
- `shuffled_dynamic` 保留为一次性 permutation 搜索对照。
- 导出 CSV 中 `contains=True`、`plane_error≈0`、`gate_margin>0` 是穿越成功的数值证据。

## Non-Convex Time-Varying Window Research

`nonconvex_timevarying_window/` 是支持多种解法并列研究的总目录。问题定义与算法实现分离：

```text
nonconvex_timevarying_window/
  PROBLEM_DEFINITION.md         非凸时变窗口的通用问题定义
  README.md                     方法索引
  atlas_dynatogt/
    ALGORITHM.md                AtlasDynaTOGT 原理与图解
    geometry.py                 非凸区域、ear clipping、TriangleChart / ChartAtlas
    environment.py              动态窗口、轨道和实验场景
    optimizer.py                chart 选择、多初值 L-BFGS-B 和轨迹指标
    visualize.py                CSV / PNG / GIF 导出
    experiments.py              smoke/default 批量实验入口
    tests/                      方法单元测试和 CLI 测试
    algorithm_figures/          方法图解
    results/                    方法实验结果
  sc_dynatogt/
    boundary.py                 Chang 边界均匀采样和角点保留
    offset.py                   Clipper2 安全偏置
    sc_mapping.py               Schwarz--Christoffel 圆盘映射
    preprocessing.py            离线预处理和持久化
    environment.py              动态窗口及空间/时间梯度
    minco.py, dynamics.py       原 TOGT 轨迹与动力学代价
    optimizer.py                [K,D] 联合 L-BFGS-B
    experiments.py              E0--E5 实验入口
    diverse_demo.py             任意边界列表与六形状多窗口 PNG/GIF 演示
    validation.py               映射合法性和数值梯度检查
    visualization.py            PNG / CSV / GIF
    simulation_render.py        可选 EGL/OpenGL 实体场景、追踪相机和 MP4
    explain_figures.py          中文算法、组件和实验结果图解
    tests/                      方法测试
```

依赖关系为：

```text
geometry -> environment -> optimizer -> experiments
                         -> visualize

boundary -> offset -> sc_mapping -> preprocessing
                              \-> environment -> optimizer -> experiments
                                  minco/dynamics -/
```

SC-DynaTOGT 的可视化是独立于求解器的三层输出：

```text
PreprocessedGate -> preprocessing.png              边界/安全区/SC 诊断
SCWindowTrack + MINCO -> visualization.py           真实门框 PNG/GIF
saved summary + same track/MINCO -> simulation_render.py  EGL/OpenGL PNG/MP4
```

OpenGL 层不重新求解轨迹，也不提供新的动力学或传感器模型。它在每帧的窗口节点变换中使用 `s(t)R(t)`；追踪相机下的缩放不明显是视觉参照问题，不是几何状态缺失。

根目录不保存任何具体算法实现。后续算法应建立与 `atlas_dynatogt/` 并列的子目录，并各自管理源码、测试、文档和试验产物。
