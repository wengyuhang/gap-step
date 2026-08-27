# Planar-RS-DynaTOGT 算法与安全证明

## 1. 问题特化

沿用根目录问题定义、SIP-DynaTOGT 的决策变量 \(x=(K,D)\)、七阶 MINCO 和四旋翼平坦性。第 \(i\) 个窗口仅允许

\[
c_i(t)=c_i,\qquad
R_i(t)=R_{i,0}R_z(\theta_i(t)),\qquad
s_i(t)>0.
\]

因此窗口法向量

\[
n_i=R_{i,0}e_3
\]

与时间无关。原始二维边界曲线 \(q_{ik}(u)\) 的世界坐标为

\[
y_{ik}(t,u)=c_i+R_{i,0}R_z(\theta_i(t))
\begin{bmatrix}s_i(t)q_{ik}(u)\\0\end{bmatrix},
\quad u\in[0,1],
\]

所以对任意 \(t,u,k\)，都有 \(n_i^T(y_{ik}-c_i)=0\)。旋转和缩放再快，也不会改变这一事实。

安全违规函数仍是

\[
g_{ik}(t,u)=\delta^2-\rho^2\bigl(y_{ik}(t,u),\mathcal B(t)\bigr),
\]

其中 \(\mathcal B(t)\) 是由平坦性姿态确定的整机长方体，\(\rho\) 是点到长方体的欧氏距离。要求所有 \(g_{ik}(t,u)\le 0\)。

## 2. 固定平面排除引理

令机体中心为 \(p(t)\)，姿态为 \(R_B(t)\)，三个半尺寸为 \(h_j\)。机体在窗口法向上的支撑半径为

\[
r_i(t)=\sum_{j=1}^3h_j\left|n_i^TR_B(t)e_j\right|,
\]

机体中心到窗口平面的有符号距离为

\[
d_i(t)=n_i^T(p(t)-c_i).
\]

若

\[
|d_i(t)|-r_i(t)-\delta>0,
\]

则整个长方体与窗口平面之间的距离严格大于 \(\delta\)。因为所有边界点都位于该平面，它们到长方体的距离也严格大于 \(\delta\)，于是该时刻对该窗口的所有曲线参数 \(u\) 同时安全。

对一个连续时间格 \(I\)，Arb 计算包含所有真实值的区间扩张 \(D_i(I)\)、\(R_B(I)\) 和

\[
G_i(I)=|D_i(I)|-
\sum_jh_j|n_i^TR_B(I)e_j|-\delta.
\]

若 `lower(G_i(I)) > 0`，则一次证明即可丢弃整个 \(I\times[0,1]\)，与该窗口有多少条原始曲线无关。若不能确定符号，则只二分时间；达到 `plane_prune_max_depth` 或最小时间宽度后仍不能排除的时间格被保留，绝不猜测安全。

这不要求基本函数在区间上单调。Arb 的加、乘、绝对值、三角函数和矩阵运算返回集合包含的外包区间；依赖导致的过宽只会造成“无法排除”，随后通过细分收紧，不会错误删除真实碰撞。

## 3. 剩余连续域认证

平面排除后，对每个保留时间段和每个原始曲线参数段运行 SIP-DynaTOGT 的二维分支定界：

1. 七阶轨迹及其一至四阶导数用 Arb 多项式区间求值；
2. 由加速度、jerk、snap 得到机体姿态、角速度和旋翼推力的包含区间；
3. 直线用仿射区间，圆弧用三角区间，Bézier 用区间 de Casteljau，B-spline 用非负基函数区间；
4. 计算 \(g_{ik}(I,U)\) 的严格外包；上界不大于零则该格安全，下界大于零则该格整体违规，否则在时间或曲线参数的较宽方向二分；
5. 动力学约束不使用平面捷径，仍覆盖每个 MINCO 段的整个 \([0,1]\)。

平面排除格与保留格的二维证明构成完整时间域分割，因此没有漏掉“窗口离无人机很远”的时间，也没有把 SC 稠密点当成边界折线。

浮点采样只用于加速找候选反例。新实现会再用零宽 Arb 区间验证该点的违规函数下界确实大于零，之后才返回 `VIOLATED`；采样未找到反例没有任何证明效力，仍必须完成上述区间覆盖。

## 4. 约束生成与有限问题

有限 SLSQP 初始集合只含：

- 每个 MINCO 段端点/中点的全部动力学约束；
- 每个指定穿越时刻，对应窗口原始曲线段端点/中点的整机安全约束。

不再把“每个轨迹段 × 每个窗口 × 每条曲线”的笛卡尔积预先塞入 SLSQP。候选轨迹经过完整认证后，最严重的严格反例作为新约束加入。穿透时使用点到姿态长方体的有符号距离约束，避免无符号距离在长方体内部梯度为零。若反例重复且有限问题不可行，使用共享 SIP 的 Phase-I 恢复；恢复失败就停止，不能称收敛。

每轮流程为：

```text
有限 SLSQP -> 固定平面 Arb 排除 -> 剩余原始曲线二维 Arb 认证
     ^                                      |
     +------------- 加入严格反例 -----------+
```

终止语义：

- `CERTIFIED_FEASIBLE`：所有安全和动力学连续域格都已证明非正；
- `VIOLATED`：存在 Arb 已确认的严格违规点；
- `UNRESOLVED`：精度、深度或单元预算耗尽，仍不能确定某些格；
- `NUMERICAL_FAILURE`：映射、平坦性或数值程序失败并关闭。

`max_cells`、`max_depth` 和交换轮数是“这次计算愿意花多少资源”的预算，不是安全阈值。提高预算可能把 `UNRESOLVED` 变成明确状态，但不会把已证明的 `VIOLATED` 自动变安全。

## 5. 时间最优性的准确表述

每个有限问题的目标是总飞行时间，SLSQP 求局部解；约束生成只在最终候选通过后停止。因此输出是“通过严格连续域安全认证的局部时间优化解”。非凸 SC/MINCO/SIP 中不能从一次局部 SLSQP 推出全局最优，也不能对任意窗口数、曲线复杂度和临界接触统一保证一分钟或十分钟运行时间。

## 6. 方法来源

本方法不是逐字复现某一篇论文，而是三部分组合：经典 Blankenship–Falk 型反例约束生成、碰撞 SIP 的区间细分，以及本文针对固定平面的支撑函数排除引理。相邻研究包括：

- Zhang 等，*Provably Feasible Semi-Infinite Program Under Collision Constraints via Subdivision*，IEEE T-RO 2024，[DOI](https://doi.org/10.1109/TRO.2024.3391649)；
- Seidel 与 Küfer，*On the convergence of local solutions for the method quADAPT*，Optimization 2024，[DOI](https://doi.org/10.1080/02331934.2024.2372386)；
- Elango 等，*Continuous-time successive convexification for constrained trajectory optimization*，Automatica 2025，[DOI](https://doi.org/10.1016/j.automatica.2025.112464)；
- Turan、Jäschke 与 Kannan，*Bounding-focused discretization methods for the global optimization of nonconvex semi-infinite programs*，COAP 2025，[DOI](https://doi.org/10.1007/s10589-025-00710-y)。

这些工作支持连续约束和自适应离散化的大框架；“固定窗口平面 + 姿态长方体法向支撑半径”的预排除是本目录针对当前问题增加的结构化特化。

