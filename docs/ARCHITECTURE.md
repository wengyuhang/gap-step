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

## Closed-Loop Continuously Deformable Window Research

`closed_loop_deformable_window/` 按共同问题定义组织两种并列方法：

```text
closed_loop_deformable_window/
  PROBLEM_DEFINITION.md         闭环、连续形变、开放机会与不可行性定义
  README.md                     方法索引
  fapp_ppo/
    geometry.py                 真实非凸安全区与可通行状态
    scenario.py                 独立外生开放日程、位姿和边界形变
    environment.py              闭环四旋翼环境与按序穿越判定
    model.py, ppo.py            特权预览 actor-critic 与 PPO
    configs/, tests/            训练配置和方法测试
  mdg/
    src/mdg/dynamic_gate.py     PCHIP 连续形变窗口统一接口
    src/mdg/safe_disks.py       真实非凸安全区的网格多圆盘内部近似
    src/mdg/disk_tracking.py    Hungarian 匹配、PCHIP 轨迹和安全收缩
    src/mdg/time_graph.py       开放时机上的粗细时空分层图
    src/mdg/backend_adapter.py  移动圆盘自由点到 MINCO/TOGT 的适配
    src/mdg/planner.py          DP、后端和 Lazy Repair 总编排
    scripts/                    单实例、E1--E6、汇总和视频入口
    tests/                      几何、图、梯度、后端和端到端测试
```

物理开口始终拓扑有效，但安全内缩区允许暂时为空。FAPP-PPO 通过预览特征和控制
策略抢占开放机会；MDG 仅在安全区非空的时刻建立图节点，并联合选择动力学可达的
开放时机。两种方法都不得把闭合时刻、凸包或仅非空的物理开口视为合法穿越。

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
    results_manager.py          无损迁移、run manifest、结果索引和中文主页
    explain_figures.py          中文算法、组件和实验结果图解
    tests/                      方法测试
  sip_dynatogt/
    model.py, constraints.py    真实边界、整机与有限 witness 约束
    intervals.py                Arb 边界/窗口/MINCO/平坦性区间扩展
    certificate.py              连续时间×边界参数分支定界
    solver.py                   [K,D] SLSQP—分离—witness 回填循环
    io.py, verify.py, tests/     可重放证书和反例/端到端测试
  avs_ppo/
    geometry.py                  动态简单非凸多边形、精确恒加速度门平面交点
    environment.py               三维平移动力学、顺序穿越和可恢复动作盾牌
    model.py, ppo.py              masked categorical actor-critic 与显式旧策略 PPO
    train.py, evaluate.py         正式训练、独立 ID/OOD 安全审计
    configs/, tests/              可复现配置和几何/盾牌/PPO 回归测试
  废案/cwb_sc_dynatogt/        旧整机数值验证和 Exact-Area 反例
```

`SIP-DynaTOGT` 保持原 `x=[K,D]` 和 constant yaw，但将整机边框距离及全部动力学限制作为连续域硬约束。SLSQP 只产生局部候选；最终成功状态由 Arb 区间有限覆盖决定。旧 `CWB/WBSC/Exact-Area` 代码保留在 `废案/`，其数值验证状态不得升级为严格证书。

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
run manifests + summaries -> results_manager.py           catalog/Markdown/HTML
```

OpenGL 层不重新求解轨迹，也不提供新的动力学或传感器模型。它在每帧的窗口节点变换中使用 `s(t)R(t)`；追踪相机下的缩放不明显是视觉参照问题，不是几何状态缺失。固定世界尺度的六窗口局部图和缩放曲线由 Matplotlib 层单独提供。

SC-DynaTOGT 结果根目录只平铺 `experiments/`、`demos/`、`diagnostics/`、`work/` 和索引文件。新运行带时间戳与 `run_manifest.json`；`current_demo.json` 指向最近一次成功演示。历史结果的移动由 `migration_manifest.json` 逐文件记录大小和 SHA-256，不删除或覆盖原始字节。

根目录不保存任何具体算法实现。后续算法应建立与 `atlas_dynatogt/` 并列的子目录，并各自管理源码、测试、文档和试验产物。
