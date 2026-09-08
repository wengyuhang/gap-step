# 代码架构与模型边界

更新：2026-09-06，依据 `40692e8` 与本地目录。现状/结果见 [PROJECT_CONTEXT](PROJECT_CONTEXT.md)，具体命令见 [RUNBOOK](RUNBOOK.md)。

## 目录组织

```text
时变窗口/
  AGENTS.md / CLAUDE.md          现行约定与镜像
  docs/                         状态、架构、命令、决策、计划、历史
  nonconvex_timevarying_window/
    PROBLEM_DEFINITION.md       基础非凸按序穿越问题
    atlas_dynatogt/             三角 chart atlas + Hermite
    sc_dynatogt/                SC 参数化 + degree-7 MINCO 基础实现
    msr_dynatogt/               SC 多初值 + 采样动力学修复
    sip_dynatogt/               witness 优化 + Arb 连续域认证
    planar_rs_dynatogt/         固定平面旋转/缩放的认证加速
    rot_sync_sc_togt/           自旋窗口的解析同步段 + MINCO
    interpolated_rot_sync_sc_togt/  双 SC 输入插值同步段 + MINCO
    avs_ppo/                   动作掩码/可恢复盾牌 PPO
    phaseguard_rl/             点/时间 PPO + 认证准入
    comparisons/               SC/SIP 压力场景与速率基准
    废案/                       WBSC、CWB、Exact-Area 历史方法
    实验方案/                   本地 ICRA 方案，审计时未跟踪
  closed_loop_deformable_window/
    PROBLEM_DEFINITION.md       安全区可为空、机会选择、完整初态返回
    fapp_ppo/                  未来预览残差 CTBR 控制
    mdg/src/mdg/               移动圆盘图规划
  togt_timevarying_window/       独立 DynaTOGT/Hermite 动态窗口实验
  gap_step/                     二维生成迷宫与历史训练入口
  复现/                         本地 TOGT 外部复现，Git 忽略
  非凸窗口/                     早期问题/概念材料
```

方法独立组织，但允许显式复用基础能力，不能据目录并列就宣称源码完全互不依赖。新方法不把源码散放在研究总目录根部。

## 非凸规划与认证

| 实现 | 主要入口与数据流 |
|---|---|
| Atlas | `geometry.py` 的 ear clipping/chart → `environment.py` → `optimizer.py` → `experiments.py` / `visualize.py` |
| SC | `boundary.py` → `offset.py` → `sc_mapping.py` → `preprocessing.py`；`environment.py` + `minco.py/dynamics.py/time_mapping.py` → `optimizer.py`；`experiments.py` 与 `diverse_demo.py` 分开 |
| MSR | `initializations.py/candidate_pool.py` → 复用 SC 求解 → `feasibility_repair.py` → `solver.py`；`comparison.py/experiments.py` 记录匹配预算比较 |
| SIP | `model.py/constraints.py` 定义真实边界和整机 → `solver.py` 的 SLSQP witness 循环 → `intervals.py/certificate.py` 连续域细分；`io.py/verify.py` 序列化与重放 |
| Planar-RS | `model.py/scenario.py` 限定运动模型 → `certificate.py` 的平面严格排除与 SIP 原始曲线检查 → `solver.py`；`verify.py` 独立重放 |
| RotSync | `geometry.py/scenarios.py` 建立自旋门 → `trajectory.py` 的固定点 Sync、PVAJ 与 MINCO 拼接 → `optimizer.py`；`collision.py` 密集整机截面审计；`single_window_comparison.py` 单独维护 |
| Interpolated-RotSync | 独立目录的 `trajectory.py` 在双 SC 输入间插值并解析到 snap → `optimizer.py` 以入口/出口 PVAJ 连接共享 MINCO/L-BFGS → `experiments.py` 单独导出结果；几何、场景、碰撞和可视化通过明确接口复用 RotSync |

SC 的 Chang 方法只用于边界均匀重采样和角点保留。内部点来自 Schwarz–Christoffel 圆盘映射 `q(d)=Psi(B(d))`，不能写成 Chang harmonic measure 或 Atlas 三角 chart。SC/SIP 基础变量为 `[K,D]`；RotSync 使用 `[K_free,K_sync,d]`，独立的 Interpolated-RotSync 使用 `[K_free,K_sync,d_entry,d_exit]`；旧 WBSC 的 `[K,D,Y]` 属于废案。

SIP 支持真实 Line/CircularArc/Bézier/非有理 B-spline 边界原语；用于映射的采样多边形和认证的原始曲线是两种数据，不得混用。SLSQP 给候选，Arb 完整有限覆盖给认证状态，优化器成功字段不能替代认证。

Planar-RS 要求固定中心/平面，允许面内旋转和统一缩放；RotSync 与 Interpolated-RotSync 都仅绕法向匀速自旋。RotSync 保持窗口局部点不变；Interpolated-RotSync 在无约束 SC 输入空间线性插值，再经 `B` 和 `Psi` 得到随时间变化的局部点，不是连接两个实际位置。两者都以 PVAJ 接入相邻七阶 MINCO，不是 MINCO 拟合 Sync。

## 强化学习路径

AVS-PPO 的 `geometry.py/environment.py` 提供动态非凸门交点和动作可恢复性检查，`model.py/ppo.py` 在状态相关掩码支持集上训练 categorical 策略。`train.py/evaluate.py` 的三窗平移模型与 `train_hardest.py/hardest_evaluate.py` 的六窗长方体压力试验分开，不能共享一条无条件“100% 安全”的摘要。

PhaseGuard-RL 的实际核心链为：

```text
model.py 的相位/状态观测与连续动作
  -> planner.py：点/时间转为固定 MINCO 轨迹
  -> shield.py：复用 SIP 原始曲线检查，按绝对起始时间检查
  -> SafePlanManager：仅接受认证候选；拒绝时保留已有认证计划
  -> environment.py / train.py / ppo.py：一步完整规划训练
```

它复用 SC 的 MINCO 和 SIP 的模型/认证内部接口，但不调用 SIP 优化器；没有既有认证计划时禁止起飞。当前不是完整物理飞控跟踪仿真。修改 SIP 认证内部接口时须考虑 PhaseGuard 的依赖。

## 连续形变问题族

FAPP-PPO 的 `scenario.py/geometry.py` 生成独立外生的开放日程、位姿与局部边界形变；自然三次样条作用于位姿/有序边界点。`environment.py/dynamics.py` 执行按序穿越、残差 CTBR 和刚体/推力分配。模型细节由 [WINDOW_MODEL](../closed_loop_deformable_window/fapp_ppo/WINDOW_MODEL.md)维护，不再复制到全局记忆。

MDG 的 `src/mdg/dynamic_gate.py` 提供 PCHIP 窗口接口；`safe_disks.py/disk_tracking.py` 构造和追踪内含安全圆盘；`time_graph.py/dynamic_programming.py` 选择机会序列；`backend_adapter.py` 复用 SC/MINCO；`planner.py` 管理 Lazy Repair。它是同一问题族的离线规划方法，不是 FAPP-PPO 子模块。

两方法均需区分物理开口与安全内缩区，后者可为空；“闭环任务返回初态”“反馈控制闭环”和“名义轨迹连续域认证”是不同属性。

## 二维与早期 TOGT 入口

GAP-Step 生成窗口迷宫路径为 `window_maze_env.py -> GraphObs -> model.py/ppo.py`，配套 `train_window.py/evaluate_window.py/visualize_window.py`。静态墙体、带动态开口的线/折线/曲线窗口及 swept-circle 碰撞保持连续几何语义。图接口只属于相关图观测代码，不是所有算法统一观测契约。

旧 `env.py/train.py/evaluate.py/visualize.py` 与 passage 入口仍在。`train_window.py` 保留可选 planner auxiliary 路径和 BC 相关文件；默认 teacher/smoke 配置不启用辅助。PPO 正确同步方向是 **`model_old <- model`**。相对配置和结果路径通过 `gap_step/utils.py` 解析到 `gap_step/`。

DynaTOGT 由 `geometry.py/environment.py` 描述动态窗口，`optimizer.py` 热启动并连续优化，`trajectory.py` 使用 Hermite，`baselines.py` 提供对照，`visualize.py` 输出中文图像。`ordered_dynamic/static` 的 `--order` 是可重复任务序列，`shuffled_dynamic` 是一次性排列对照。

## 结果、显示与测试

SC 的 `visualization.py` 生成物理门框图和轨迹图，`simulation_render.py` 用 EGL/OpenGL 回放已保存轨迹，`results_manager.py` 管理分类结果、manifest 与主页。显示层不改变求解几何或提供飞控/传感器认证。历史结果目录保留 `experiments/demos/diagnostics/work` 分类与迁移校验。

SC/SIP 宽域比较中的本地 `gazebo/` 是单独适配层，目前平移/RPY 与均匀缩放支持不同；其 README 中明确缩放和飞控接入的缺口。方法渲染、Gazebo 世界与实际动力学执行各自记录。

根 `pytest.ini` 只自动收集 `gap_step/tests`。研究方法及比较目录都有自己的测试目录；影响共享代码时按依赖选择回归，不能用裸 `pytest -q` 代替。
