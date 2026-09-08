# Interpolated-RotSync-SC-TOGT

本目录是从 `rot_sync_sc_togt/` 拆分出的独立方法。原 RotSync 在同步段固定一个窗口局部点；本方法联合优化入口和出口两组无约束 SC 输入：

```text
d(s) = (1-s) d_entry + s d_exit
q(t) = Psi(B(d(s)))
p(t) = c + E R(theta0 + omega t) q(t) + n z(t)
```

插值在 SC 输入空间中进行，不是连接两个实际位置。`trajectory.py` 用截断 Taylor jet 计算位置到 snap 的时间导数，入口和出口 PVAJ 作为两侧 degree-7 MINCO 边界。`optimizer.py` 优化 `[K_free,K_sync,d_entry,d_exit]`，明确复用原 RotSync 的目标、动力学限制、时间变换和 L-BFGS 驱动。

## 运行

```bash
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.experiments \
  --suite oblique_smoke --no-animation --audit-dt 0.001 \
  --collision-samples 5001 \
  --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/oblique_20260907

pytest -q nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/tests
```

原方法对照仍从以下入口运行：

```bash
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments \
  --suite smoke --no-animation \
  --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/smoke_control
```

## 2026-09-07 斜向 L 单窗结果

- 两组优化输入距离 `0.121204`，映射后端点相距 `0.113786 m`；
- 总时间 `3.559735 s`，最大 C3 接口跳变 `2.737e-14`；
- 动力学审计最大步长 `0.999468 ms`，最大速度 `4.791764 m/s`，限制全部通过；
- 实姿长方体碰撞审计 `0/5001`，网格最大步长 `0.711947 ms`；
- 两输入相同时 0–4 阶对原 Sync 的最大绝对误差 `4.441e-16`；不同时 1–4 阶导数差分误差均不超过 `7.108e-10`。

新方法完整产物位于 [`results/oblique_20260907/single_L_oblique/`](results/oblique_20260907/single_L_oblique/)，原固定输入对照位于 [`../rot_sync_sc_togt/results/oblique_control_20260907/`](../rot_sync_sc_togt/results/oblique_control_20260907/)。对照总时间 `3.827154 s`，C3 最大跳变 `4.519e-14`，动力学和 `0/5001` 碰撞采样验收通过，但 L-BFGS 状态为 `ABNORMAL`；这不会被记为优化器收敛。

上述碰撞与动力学结果是独立于优化目标判值的整轨迹稠密采样验证，不是 SIP 或连续域安全证书。优化器收敛状态和轨迹验收状态在 `result.json` 中分开保存。

## 零厚度窗口的固定/双 SC 输入对比

`compare_fixed_wp_counterexample.py` 复用实验三的均衡 U 几何（尺寸比 `1.9`、角速度 `4.5 rad/s`、初相位 `1.1 rad`），但将窗口厚度设为 `0`。规划球包络半径为 `rho=0.3944227368 m`，所以 Sync 入口/出口球心的法向坐标严格为 `-rho/+rho`。对照改为原始的固定一组 SC 输入 RotSync，不再用两段普通 MINCO 的 Fixed-WP，因此新方法的可行域确实包含对照。目标按 TOGT 配套 C++ 代码使用：

```text
J = T_total + trapezoidal_integral(smoothedL1(h))
n_i = clamp(int(T_i / 0.05), 8, 32)
```

snap、速度和碰撞都不进入目标；机体角速率和旋翼推力权重为 1。`smoothedL1` 在 `[0,0.01]` 内三次平滑，大超限时转为线性；同时复刻 `|z_B,z+1|≤0.001` 的 robust singularity 分支。L-BFGS 使用源码的 memory 256、past 32、最大 64 次线搜索，`maxIterations=0`，不设墙钟预算。由于发布的 C++ 不包含双 SC 输入段，这里逐项复刻其目标函数取值，该新参数化对决策变量的梯度仍用中心有限差分。

```bash
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_counterexample \
  --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_new_run
```

2026-09-08 的最终无预算复跑见 [`results/zero_thickness_nested_warmstart_togt_code_unlimited_20260908/REPORT.md`](results/zero_thickness_nested_warmstart_togt_code_unlimited_20260908/REPORT.md)：

- 运行前的包含关系检查中，两轨迹 0–4 阶导数的最大误差为 `7.088e-13`，目标误差为 `7.283e-14`。
- 先求固定输入 RotSync，再将该解以“入口=出口”精确嵌入新方法作为热启动；嵌入目标误差 `3.109e-14`。
- 两者最终均为 `T=5.811201508 s`、`J=5.829428767`；之前新方法独立初始化得到的 `6.602938 s` 是线搜索停滞点，不是新可行域的更优性结论。
- 两者均为 `0/5999` 碰撞样本、一次有效穿窗，但 TOGT-code 1 ms 审计都有 192 个动力学违规样本，所以轨迹验收仍失败。新方法 C3 最大接口跳变为 `3.803e-13`。

两套 1 ms 动力学审计均保留：TOGT-code 审计使用 C++ tilt-yaw/robust-singularity 残差，原审计使用项目既有姿态恢复。优化器停止、离散目标惩罚较小和采样无碰撞都不等于轨迹验收成功；采样也不是连续域证明。旧 Fixed-WP、论文字面式、带碰撞惩罚和中间试跑产物均保留在原目录，不覆盖。

### Fixed-WP 轨迹反推初值

`compare_fixed_wp_seeded.py` 保留普通两段 MINCO `Fixed-WP` 作为对照。它先求 Fixed-WP，再在该轨迹上求规划球心法向距离等于 `-rho/+rho` 的时刻，将两处世界位置转回各自时刻的窗口局部坐标，再经 SC 逆映射生成双输入初值。

```bash
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded \
  --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_fixed_wp_seeded_new_run
```

2026-09-08 无预算实际运行见 [`results/zero_thickness_fixed_wp_seeded_togt_code_unlimited_20260908/REPORT.md`](results/zero_thickness_fixed_wp_seeded_togt_code_unlimited_20260908/REPORT.md)：

- Fixed-WP 得到 `T=2.245913723 s`、`J=2.283023207`。反求相切时刻为 `0.986851567/1.074603514 s`，三段时间初值为 `[0.986851567, 0.087751947, 1.171310209] s`。
- 反推入口/出口局部点的安全边界余量为 `0.139567/0.045006 m`，SC 逆映射回代误差不超过 `2.220e-16 m`。
- 反推轨迹初值虽然同样是 `T=2.245913723 s`，但由于 `0.087752 s` 内强制完成局部移动与旋转同步，初始动力学惩罚为 `504891.2374`。
- 无预算 L-BFGS 正常停止后，双输入方法为 `T=2.674461462 s`、`J=2530.901485`，动力学惩罚 `2528.227023`；独立 1 ms 审计有 `2704` 个动力学违规样本，轨迹验收失败。

新方法的空间输入更自由，但它同时额外强制从入口相切到出口相切的整段遵循 SC 插值 Sync。普通 Fixed-WP 在这段仍是无此结构限制的 MINCO 多项式，所以 Fixed-WP 轨迹不是新参数化的可行子集，不存在“新方法最优时间必然不大于 Fixed-WP”的数学保证。
