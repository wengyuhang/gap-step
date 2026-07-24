# Project Context

Last updated: 2026-07-24 (Asia/Shanghai).

## Current Focus

The repository mainline remains a generated family of continuous 2D time-varying window mazes trained with a pure privileged PPO teacher. The active independent research extensions are the multi-method non-convex time-varying window project under `nonconvex_timevarying_window/` and the closed-loop continuously deformable-window reinforcement-learning project under `closed_loop_deformable_window/`.

```text
gap_step/window_maze_env.py
gap_step/train_window.py
gap_step/evaluate_window.py
gap_step/visualize_window.py

nonconvex_timevarying_window/PROBLEM_DEFINITION.md
nonconvex_timevarying_window/atlas_dynatogt/
nonconvex_timevarying_window/sc_dynatogt/

closed_loop_deformable_window/PROBLEM_DEFINITION.md
closed_loop_deformable_window/fapp_ppo/
```

## Environment Contract

- Static black walls are hard obstacles.
- Each aperture window is a wall-to-wall line/polyline/curve with one dynamic gap.
- The agent moves with continuous 2D actions.
- Collision is swept-circle and terminal for walls, window bodies, boundary contact, or post-phase overlap.
- Blue overlays visualize current openings only.

## Current Results

```text
id_test         200 episodes, 71.5% success
ood_window_test 200 episodes, 54.0% success
ood_maze_test   200 episodes, 74.5% success
```

The ID target is met. Unseen window timing is the current generalization weakness.

## Key Outputs

```text
gap_step/checkpoints/window_generated/C5/teacher_final.pt
gap_step/results/window_generated/eval_c5.csv
gap_step/results/window_generated/gifs/
gap_step/preview/high_difficulty_window_maze.gif
gap_step/preview/high_difficulty_window_maze_phases.png
```

## TOGT Reproduction Context

`复现/TOGT-Planner-reproduction/` contains the source-level reproduction package and notes for arXiv:2309.06837v3.

`togt_timevarying_window/` has been rebuilt as **DynaTOGT**, an independent dynamic time-varying window traversal experiment. It keeps the TOGT paper idea of choosing traversal points inside gate geometry, but changes the constraint from static `p(t_i) in G_i` to dynamic/deformable `p(t_i) in G_i(t_i)`.

Current DynaTOGT facts:

- independent from `gap_step/` PPO and the old maze environment;
- supports moving, rotating, scaling/deforming 3D windows;
- supports arbitrary ordered traversal task sequences, including repeated visits to the same window;
- default canonical order remains `G1 -> G6 -> G3 -> G2 -> G5 -> G4`;
- repeated demo example uses `G1 -> G6 -> G1 -> G3 -> G2 -> G5 -> G4 -> G2`;
- exports Chinese presentation-style PNG/GIF plus trajectory CSV under `togt_timevarying_window/results/`;
- traversal evidence is recorded per crossing with `contains`, `plane_error`, and `gate_margin`.

## Closed-Loop Deformable Window RL Context

`closed_loop_deformable_window/` is an independent simulation-only research project. Its first algorithm is `fapp_ppo/`: Future-Aware Privileged-Preview PPO with a schedule-aware nominal CTBR controller and a bounded learned residual.

The task is to traverse four windows exactly once in the specified order and then recover the complete initial state \((p,v,R,\omega)\). The current time-critical ID setting uses a 26 s episode, 1.40 s mean fully-open opportunities, 3.80 s mean recurrence, motion multiplier 1.8, and deformation multiplier 2.0.

### Window process and independence

Every episode fixes the complete future at reset. Window \(i\) uses three independent component streams derived from `(scenario_seed, window_index, component_id)`:

```text
component 0: opening schedule
component 1: center translation and rotation
component 2: overall size and local boundary shape
```

Different windows also use disjoint streams. The generator never reads route length, cruise speed, UAV position/velocity/action, or estimated arrival time. Openings therefore cannot be triggered by UAV arrival.

The opening schedule is an independent non-periodic renewal process. Each window samples its own initial phase, opportunity width, and recurrence interval. A 1,000-seed audit measured pairwise first-opening correlations of only `-0.034..0.014`; each window had 6--8 opportunities and first openings covered `0.321..4.118 s`.

### Pose and boundary deformation model

Time-critical windows use keyframes spaced at approximately 0.30 s. Center \(c_i(t)\), rotation vector \(\rho_i(t)\), and every ordered local boundary point \(b_{i,k}(t)\) are queried through natural cubic splines. The world-plane pose is

\[
x^{world}_{i,k}(t)=c_i(t)+R(\rho_i(t))
\begin{bmatrix}b_{i,k}(t)\\0\end{bmatrix}.
\]

At each keyframe, the 64-point local boundary is a positive radial graph. With
\(\theta_k=2\pi k/64\),

\[
\begin{aligned}
q_i(\theta,t)=1
&+\alpha_{i,1}(t)\cos 2\theta
+\alpha_{i,2}(t)\sin 3\theta\\
&+\alpha_{i,3}(t)\cos 5\theta
+\alpha_{i,4}(t)\sin \theta
+\alpha_{i,5}(t)\sin 4\theta ,
\end{aligned}
\]

\[
b_i(\theta,t)=
\begin{bmatrix}
r_{i,x}(t)q_i(\theta,t)\cos\theta\\
r_{i,y}(t)q_i(\theta,t)\sin\theta
\end{bmatrix}.
\]

The five \(\alpha\) coefficients are smooth random walks clipped to `[-0.22,0.22]`. They independently change lobe depth, concavity location, asymmetry, and boundary curvature; they are not per-frame independent vertex noise. The keyframe generator additionally enforces \(q_i>0.28\), preserving a star-shaped, connected, hole-free physical opening at keyframes.

The two axis radii combine three effects:

\[
r_{i,x}(t)=1.05\,[1+\delta_{i,x}(t)]\,\sigma_i(t),\qquad
r_{i,y}(t)=0.88\,[1+\delta_{i,y}(t)]\,\sigma_i(t).
\]

- \(\delta_{i,x},\delta_{i,y}\) are independent smooth size walks clipped to `[-0.20,0.20]`, so aspect ratio and overall size change continuously;
- \(\sigma_i(t)\) is the opening/closing envelope, ranging from `0.16` to `1.05`;
- every rise and fall of \(\sigma_i(t)\) uses smoothstep over 0.32 s, so there is no instantaneous geometry switch.

The resulting ordered boundary points are spline-interpolated between keyframes. Interpolation is validated on dense times: the physical polygon must remain valid, simple, connected, hole-free, and have positive area. Thus the implemented intermediate shape is the spline of boundary points, not a claim that the five harmonic coefficients themselves have a closed-form continuous trajectory at every instant.

The physical opening is \(\Omega_i(t)\). The safe traversal region is the true non-convex inward offset

\[
\Omega_i^{safe}(t)=\Omega_i(t)\ominus B(0,0.16\ {\rm m}).
\]

When \(\sigma_i(t)\) approaches 0.16, the physical polygon still has nonzero area but the inward offset becomes empty. The window is then physically present yet impossible for the UAV to traverse safely. In the current ID audit, safe passability occupied `45.86%..56.35%` of time and complete non-passability occupied `43.65%..54.14%`.

Translation, rotation, opening scale, axis-scale drift, and local harmonic deformation occur simultaneously but come from isolated random streams. Their amplitudes are limited and every scenario also passes a sampled inter-window envelope separation check.

### Current FAPP-PPO status

The validation run used 100 PPO updates and 102,400 environment steps:

```text
closed_loop_deformable_window/fapp_ppo/runs/independent_validation_v3/
```

The final checkpoint achieved `0/10` on the paired ID pilot, versus `1/10` for each nominal baseline. Update 25 achieved `3/10` on an independent development slice, while updates 50/75/100 achieved `0/10`, demonstrating late policy collapse. The Chinese MP4 at
`fapp_ppo_early_checkpoint_independent_seed53017_zh.mp4` is an explicitly labeled early-checkpoint mechanism demonstration, not the main performance result.

The primary reward defect is now diagnosed: after a legal crossing, the potential target immediately switches to the next distant window. This creates a `-6.5..-9.8` shaping jump that nearly cancels the `+10` gate reward. Persistent action standard deviation near 0.30 and insufficient residual-prior strength then let the learned residual drift away from the nominal controller. Maximum PPO approximate KL remained 0.0108 below the 0.02 target, so this is not evidence of a KL update explosion.

Current validation:

```text
pytest -q closed_loop_deformable_window/fapp_ppo/tests  # 12 passed
pytest -q                                                # 46 passed
```

Detailed model, figures, experiment protocol, and negative-result record:

```text
closed_loop_deformable_window/fapp_ppo/ALGORITHM.md
closed_loop_deformable_window/fapp_ppo/FIGURE_GUIDE.md
closed_loop_deformable_window/fapp_ppo/ACADEMIC_EXPERIMENTS.md
closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md
```

## Non-Convex Time-Varying Window Research Context

`nonconvex_timevarying_window/` 是一个独立的非凸时变窗口研究总目录，不属于 `gap_step/` PPO 主线，也不替代已有的凸窗口 `togt_timevarying_window/` 子项目。

研究目标是在论文 *Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing*（`arXiv:2309.06837v3`）的 TOGT 问题上，将原有静态凸窗口扩展为非凸且随时间平移、旋转和缩放的窗口。

当前通用问题范围：

- 窗口是无洞、无自交的简单闭合非凸区域；
- 折线、光滑曲线和混合边界都可以通过有序边界点表示；
- 无人机按给定顺序穿越窗口，当前任务不要求重复穿越同一窗口；
- 穿越点必须位于穿越时刻的真实非凸区域内，不能用凸包代替真实窗口验证；
- 目标是在窗口几何、时变运动、指定顺序和轨迹动力学约束下尽量减小总飞行时间。

总目录和方法目录的边界为：

```text
nonconvex_timevarying_window/
  README.md                 总任务与方法索引
  PROBLEM_DEFINITION.md     与具体算法无关的问题定义
  atlas_dynatogt/           已实现的 AtlasDynaTOGT 方法
  sc_dynatogt/              已实现的 SC-DynaTOGT 方法
  <algorithm_name>/        后续方法的并列目录
```

当前有两个相互独立的方法：

- `AtlasDynaTOGT`：将非凸区域用 ear clipping 剖分成三角 chart atlas，使用 softmax 重心坐标生成 chart 内穿越点；
- `SC-DynaTOGT`：Chang 等人的工作只用于边界均匀重采样和角点保留，内部取点严格使用圆盘 Schwarz--Christoffel 映射，并接入原 TOGT 的时间变量、degree-7 MINCO 和动力学代价。

两种方法不共享内部参数化代码，各自从本目录的 `experiments.py` 进入。

SC-DynaTOGT 当前必须保持的技术语义：

- *Real-Time Conformal Maps and Parameterizations* 中 Chang 等人的方法只用于边界均匀重采样和角点保留；
- 非凸区域内部穿越点使用圆盘 Schwarz--Christoffel 映射 `q(d)=Psi(B(d))`，不使用 Chang 的 harmonic measure / Poisson kernel，也不复用 AtlasDynaTOGT 的三角 chart；
- 边界支持非凸多边形、光滑闭曲线、直线–曲线混合边界和 CSV 稠密边界，但仍限于无洞、无自交的简单闭区域；
- 窗口位姿由三维中心、RPY 旋转和均匀缩放定义，空间梯度与窗口时间梯度均使用解析链式法则；
- 场景可视化只画统一橙黑实体门框表示原始物理窗口，不显示内缩安全区；安全区仍用于优化、预处理诊断图和真实非凸区域合法性验证；
- E0--E5 正式统计场景和 `diverse_demo` 功能演示互相独立，不得用演示结果改写正式实验定义。

SC-DynaTOGT 完整 default 实验已于 2026-07-14 完成：

```text
E0  SC/原凸 TOGT 总时间相对差 0.4942% < 1%
E1  6 类边界 x 5 个顶点数，30/30 通过，最大边界误差 1.9699 mm < 5 mm
E2  SC 30/30 收敛且真非凸区域合法；凸包映射仅 1/30 在真安全区内
E3  151/155 收敛，155/155 穿越合法
E4  153/155 收敛，155/155 穿越合法
E5  完整/去时间梯度均 153/155 收敛，155/155 穿越合法
SC mapping 1,000,000/1,000,000 legal, no NaN/Inf/degenerate Jacobian
```

`diverse_demo.py` 是独立的六形状六窗口全动态演示，顺序为 `L -> U -> star -> limacon -> wavy -> line_bezier`。当前默认配置是：

- `layout=paper_irregular`：参考原论文配套 `race_uzh_7g_multiprisma.yaml` 的 Gate1--Gate7 位置、RPY 和穿越顺序，六种形状对应 Gate1、2、3、4、6、7；
- 起点与终点同为 `[-16,4,3.2] m`；中心覆盖 `x=[-9.9,20.24] m`、`y=[-13.2,14.96] m`、`z=[1.8,6.48] m`，七段闭环航程为不等长的 `14.77--30.74 m`；
- `motion_scale=3.5`：平移、旋转、均匀缩放振幅均为正式 E3--E5 场景的 3.5 倍，缩放系数范围 `[0.58,1.42]`；
- 实跑结果为 6/6 指定顺序穿越合法，六个映射各 `1000/1000` 点合法，总时间 `14.90481 s`，385 次迭代；
- 该强运动长距离演示的 `sampled_dynamic_limits_satisfied=false`：窗口合法性已通过，但不应把 TOGT 软惩罚收敛表述为全部动力学硬上限可行；
- 结果已无损整理为 `results/experiments/`、`results/demos/`、`results/diagnostics/` 和 `results/work/`；543 个原文件共 60,079,505 字节均在 `migration_manifest.json` 中记录原/新路径、大小和 SHA-256，旧结果没有删除或覆盖。
- 当前精选运行是 `results/demos/runs/20260717_paper_irregular_closed/`。全局图只画一个代表性四旋翼，另有六窗口固定尺度局部图、缩放曲线和统一时间轴 GIF；旧图及旧 OpenGL 成片保存在该运行的 `legacy/`。
- 可选 `simulation_render.py` 通过 EGL/OpenGL 生成带实体门框、四旋翼网格、低干扰建筑/植被、阴影、大气雾、HUD 和追踪相机的离线画面；它不替代 AirSim 动力学/传感器仿真。
- OpenGL 门框在每帧的三维变换中实际应用 `s(t)R(t)`，当前缩放范围为 `[0.58,1.42]`。追踪相机下仍不易直接比较，因此 `figures/crossings_grid.png` 和 `figures/scale_profile.png` 提供固定尺度与数值证据；OpenGL `GATE CAM` 视频和实时 `SCALE ×` 仍未实现。

当前入口和产物：

```text
python -m nonconvex_timevarying_window.sc_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.sc_dynatogt.experiments --suite default
python -m nonconvex_timevarying_window.sc_dynatogt.diverse_demo --mode full --quality smoke --layout paper_irregular --motion-scale 3.5 --validation-samples 1000
PYOPENGL_PLATFORM=egl python -m nonconvex_timevarying_window.sc_dynatogt.simulation_render
python -m nonconvex_timevarying_window.sc_dynatogt.results_manager verify
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests

results homepage: nonconvex_timevarying_window/sc_dynatogt/results/index.html
formal results: nonconvex_timevarying_window/sc_dynatogt/results/experiments/formal/20260714_default/E0..E5/
featured demo: nonconvex_timevarying_window/sc_dynatogt/results/demos/runs/20260717_paper_irregular_closed/
legacy demos: nonconvex_timevarying_window/sc_dynatogt/results/demos/archive/ and featured-run legacy/
detailed record: nonconvex_timevarying_window/sc_dynatogt/TEST_RESULTS.md
```

当前验证状态：

```text
default suite: 14 scenarios, 14 successes
pytest -q nonconvex_timevarying_window/atlas_dynatogt/tests  # 7 passed
SC-DynaTOGT smoke: E0--E5 all passed
SC-DynaTOGT default: E0--E5 complete; all traversal legality rates 100%
SC-DynaTOGT mapping: 1,000,000 / 1,000,000 legal, no NaN/Inf/degenerate Jacobian
SC-DynaTOGT diverse demo: paper-inspired irregular closed-loop 3D layout, start=goal, 6/6 legal crossings
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests  # 111 passed (2026-07-17)
```
