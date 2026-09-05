# RotSync-SC-TOGT

本目录独立实现“固定中心、固定平面、仅绕自身法向匀速旋转”的非凸窗口安全穿越。原有 `sc_dynatogt/` 不做修改；这里直接复用其非凸安全内缩、圆盘 Schwarz–Christoffel 映射、degree-7 MINCO、四旋翼微分平坦性、动力学软惩罚、时间映射和 L-BFGS 框架。

## 轨迹与优化变量

每个窗口先按无人机包络半径 `rho` 内缩，并以

```text
q_i = Psi_i(B(d_i))
```

选择安全开口内的二维穿越点。窗口中心、平面基和法向固定，角度为 `theta_i(t) = theta_i0 + omega_i t`。同步段严格采用解析轨迹

```text
c_i + E_i R(theta_i(t)) q_i + n_i z_i(t),
z_i(t) = -D_i + 2 D_i (t-t_i-) / Delta_i,
D_i = h_i/2 + rho.
```

`RotationSyncSegment` 解析返回 position、velocity、acceleration 和 jerk。完整轨迹始终按

```text
MINCO -> Sync -> MINCO -> ... -> Sync -> MINCO
```

拼接；Sync 的入口/出口 PVAJ 直接成为相邻 degree-7 MINCO 的边界，接口至少为 C3，MINCO 不近似同步旋转曲线。

联合变量为 `[K_free, K_sync, d]`。窗口绝对角度由所有前序自由段和同步段时间累积得到，不作为独立变量。目标为

```text
T_total + lambda_s * integral(||snap||^2) + lambda_d * P_dyn.
```

## 正式赛道

`formal` 是默认实验套件，共四条起点 PVAJ 与终点 PVAJ 完全相同的闭合赛道：

- `D1_compact_planar`：L、limacon、U，三门近似平面紧凑环；
- `D2_spatial_slalom`：wavy、star、U、line–Bézier，四门三维回转；
- `D3_uzh_irregular`：六种窗口全部出现，采用 TOGT UZH 赛道的非规则顺序模式；
- `D4_split_s_endurance`：六种窗口重新排序，连续大高差 Split-S 式闭环。

难度配置同时改变窗口数量、边界类型、名义航程、最大转角、高差和窗口角速度。完整设计依据、参数和判据见 [FORMAL_EXPERIMENTS.md](FORMAL_EXPERIMENTS.md)。旧 `smoke` 入口只保留为快速代码回归，不属于正式实验，也不会由默认命令运行。

## 长方体整机与碰撞率

无人机采用扁平方柱，默认全尺寸为 `0.53008 × 0.53008 × 0.11780 m`，底面为正方形，高度明显较小。`rho` 取长方体外接球半径，使 SC 内缩包含完整机体。

PNG/GIF 按微分平坦性姿态绘制真实比例长方体；碰撞帧显示为红色。整条轨迹上密集采样，对每个时刻求长方体与窗口厚度带的相交截面，再在旋转窗口坐标系中检查该截面是否接触真实非凸边界。`result.json` 和汇总表保存碰撞样本数、碰撞数、碰撞率、首次碰撞时刻和最小门框距离。

## 运行与产物

```bash
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments \
  --suite formal \
  --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/formal

pytest -q nonconvex_timevarying_window/rot_sync_sc_togt/tests
```

正式默认值为 256 个边界顶点、64 阶 SC 求积、统一的 `lambda_d=0.1`、120 次 L-BFGS 迭代上限、每段 11 个动力学目标采样、5001 个整轨迹碰撞样本和 140 帧动画。每条赛道保存 `config.json`、`result.json`、SC 预处理产物、`trajectory.csv`、`trajectory_3d.png` 和 `rotation_sync.gif`；根目录保存 `summary.csv/json`。

本实现不包含 SIP、MPC、强化学习、混合整数规划或额外规划器。

2026-09-02 的完整正式结果已保存到 `results/formal/`：D1/D2 完整通过；D3/D4 的几何、C3、闭合和长方体零碰撞验证通过，但最大速度分别为 7.1894/7.1040 m/s，略高于 7 m/s 正式上限，因此保留失败标记。详见 [正式实验结果](FORMAL_EXPERIMENTS.md#2026-09-02-正式结果)。

## 现实尺度极限复跑

`realistic_extreme` 只重跑最复杂的六窗口闭合赛道。机体碰撞盒为 `0.60 × 0.60 × 0.18 m`，对应外接球半径 `rho=0.433705 m`；尺寸依据 DJI F450 的 450 mm 对角轴距和 10 英寸推荐桨叶，并为桨叶外廓保留约 0.60 m 的保守正方形包络。动力学采用 1.2 kg、0.159 m 力臂的 F450 级设置。

六种物理窗口缩放为原边界的 0.60–0.72 倍，物理跨度约 2.52–3.25 m；安全内缩后最窄二维跨度约 1.09 m。相邻窗口中心至少间隔 10.07 m，闭合名义航程为 92.84 m。动画为每个短 Sync 段保留帧，并用橙色高亮当前解析同步轨迹，同时显示真实姿态的 X 型机臂、桨盘、机身和半透明碰撞盒。

```bash
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments \
  --suite realistic_extreme \
  --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/realistic_extreme
```

## 单窗口固定点竞速对比

`single_window_comparison.py` 比较传统固定 waypoint 思路与完整 RotSync。
固定点基线在优化前选定真实安全区的最大内接圆中心，固定其窗口局部坐标；
它随窗口旋转，在优化得到的到达时刻形成世界坐标 waypoint。
基线只优化穿越前后两段七阶 MINCO 的时间，没有 Sync，也不优化穿越点。
这不是将时变窗口冻结成静态窗口，亦不是给基线加入同步段。

默认任务为 L、U、star 三种窗口乘以 0、0.75、1.5 rad/s 三档角速度，共九对；
初相位为 0.3 rad，机体为原始 0.53008 × 0.53008 × 0.11780 m 长方体。
两组使用相同初终 PVAJ、窗口、动力学约束、目标权重与优化器配置。
统一验收包括 1 ms 整机碰撞/动力学采样、分段接口双侧检查、指定穿越点、
按序一次穿越和初终状态误差；单窗口任务为门前到门后，不要求返回起点。
优化器收敛与轨迹验收分别记录。图中净距针对原有物理边界沿厚度挤出的门框模型。

```bash
python -m nonconvex_timevarying_window.rot_sync_sc_togt.single_window_comparison
pytest -q nonconvex_timevarying_window/rot_sync_sc_togt/tests/test_single_window_comparison.py
```

默认生成带时间戳的新目录，也可用 `--outdir` 指定一个尚不存在的目录。
产物包括协议、逐案例配置/结果、轨迹 CSV、各案例三面板图和汇总图表。
首次九对实测见 [单窗口报告](results/single_window_fixed_wp_20260905/REPORT.md)。
这组结果中两种方法均为 9/9 合格，固定点基线在全部案例中飞行更快；
它没有证明同步结构在当前较宽窗口和转速范围内具有性能优势。
这是两种完整方法的比较，点优化和同步结构同时不同，不能将差异单独归因于某一个组件。
