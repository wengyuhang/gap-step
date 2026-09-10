# Conditional Dual-Constraint CEM for Convex Windows

该方法用于 `convex_timevarying_window/` 的七个周期平移、三维旋转凸窗口。它保留 TOGT 的
`[K,D]` 参数化、MINCO 七阶轨迹、发布版 standard 动力学目标和 L-BFGS 配置，只在名义轨迹
存在安全软积分时激活安全恢复。

## 算法流程

1. 用不含安全项的原 TOGT 目标做一次 L-BFGS，得到名义轨迹。
2. 在整条轨迹上计算安全软积分。若结果严格为零，CEM 只优化 TOGT 动力学软积分；若不为零，
   先以 `J_TOGT + 100000 J_safe` 做带手写梯度的 L-BFGS，再启动双约束 CEM。
3. CEM 直接搜索完整 `[K,D]`。多边形 D 块归一化到单位球面，圆形 D 保持 TOGT Ball 映射；
   不使用按窗口编写的结构化前端。候选同时计算动力学软积分和安全软积分，以 Pareto 层及
   固定 `epsilon_dyn=epsilon_safe=1e-6` 排序。epsilon 只用于排序分辨率，合格仍要求两个原始
   积分严格等于零。
4. 合格与失败候选都参与精英记忆、均值和完整协方差更新。第一次得到严格双零后继续搜索 5 轮。
5. 将所有严格双零候选按飞行时间排序，逐条做最大 1 ms 步长的完整动力学与真实外接球安全
   检查。第一条双通过即返回，后续候选不再参与“最好轨迹”的选择。

动力学积分完全使用 TOGT 发布实现：`smoothedL1(mu=0.01)`、每段 8–32 个区间和梯形积分。
最终动力学验收也直接调用同一发布版 C++ `QuadManifold::toStateWithTiltYaw`，不再用 Python
重新推导四旋翼状态。发布 `standard_planning.yaml` 配置为速度 `60 m/s`、倾角 `6.28 rad`、
XY/Z 机体角速度 `10 rad/s`、单旋翼推力 `0.25–5 N`，并由四个旋翼界限派生总推力
`1–20 N`；权重为 `weightVel=0, weightOmg=1, weightRot=1, weightThr=1`。
安全积分独立加密为按 20 ms 目标步长、每段 16–64 个区间的梯形积分，不先寻找接触区间。
本次规划半径为真实外接球半径再加 30 mm；最终安全验收只用真实外接球半径。

## 安全场与手写梯度

世界点先按动态窗口位姿变到局部坐标 `y=R(t)^T(p-c(t))=(q,z)`。凸多边形预先转换为单位
外法向半空间 `a_j^T q-b_j<=0`，并使用

```text
h(q) = tau log sum_j exp((a_j^T q-b_j)/tau),  tau=0.01 m
```

圆窗使用等零集的平方隐式场并归一化到距离量纲：

```text
h(q) = (||q||^2-a^2)/(2a)
```

有限零厚度门框的球体接触残差为

```text
g = r_plan^2 - z^2 - h(q)^2
```

对 `g` 施加与 TOGT 相同形式的 `smoothedL1` 后沿时间积分。代码手写了 `dh/dq`、局部到世界
位置梯度，以及 `dR/dt`、`dc/dt` 引起的显式时间梯度；随后调用发布 C++ `MincoSnap::propagateGrad`
回传到中间点与分段时长，再经动态窗口 D Jacobian 和 K 时间映射回传到 `[K,D]`。三组有限差分
测试中，MINCO 安全梯度方向相对误差约 `2.64e-9`，完整 `[K,D]` 目标约 `2.35e-8`。

方法组合参考 TOGT 的 MINCO/动力学解析梯度、经典 Cross-Entropy Method 的分布更新，以及
[CHOMP](https://www.cs.cmu.edu/~mzucker/icra09-chomp.pdf)和
[GPMP2](https://arxiv.org/abs/1707.07383)用可微距离场构造碰撞代价的思路。当前凸半空间
smooth-max 与圆形隐式场是本问题的具体实现；它们是优化辅助量，不是连续域安全证书。

## 2026-09-10 正式结果

正式产物见 [实验报告](results/formal_cpp_aligned_margin30mm_20260910/REPORT.md)，成功轨迹图见
[selected_route.png](results/formal_cpp_aligned_margin30mm_20260910/figures/selected_route.png)，完整硬验收见
[selected_audits.json](results/formal_cpp_aligned_margin30mm_20260910/selected_audits.json)。运行期间源码起止
SHA-256 清单一致。

原 TOGT 名义解为 `18.724959468 s`，动力学软积分 `0.00966076679109`、安全软积分
`0.0659155760885`，完整动力学与安全检查都失败。条件式方法的首个完整验收成功候选为
`id=324`，飞行时间 `21.872307580 s`，两个软积分均严格为零；动力学全部通过，七窗真实外接球
最小净空为 `50.609726 mm`。这是最大 1 ms 步长的密集采样证据，不是连续域认证。

较早的 `formal_fine_margin30mm_20260910` 使用 Python 平坦性复算做最终动力学验收，把候选324
误判为失败并返回候选201。该结果保留作审计，不再作为当前比较结论。

运行命令：

```bash
conda run --no-capture-output -n wyh python \
  -m convex_timevarying_window.conditional_dual_constraint_cem.experiment \
  --outdir convex_timevarying_window/conditional_dual_constraint_cem/results/new_run \
  --population 64 --maximum-rounds 50 --repair-max-iterations 600
```
