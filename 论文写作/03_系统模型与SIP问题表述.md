# 3　系统模型与问题表述

本章建立面向动态门的全赛道四旋翼轨迹规划模型。给定起终状态、穿越顺序以及各门相对比赛时钟的已知运动规律，目标是联合求解到达时刻、过门位置与全程轨迹，在保持整机与移动门框无碰撞分离的前提下最小化飞行时间。

## 3.1　四旋翼模型

取竖直向上的右手惯性坐标系 $\mathcal{W}$（$\mathbf{e}_3 = [0, 0, 1]^\top$）与原点位于质心的机体坐标系 $\mathcal{B}$。记 $\mathbf{p}, \mathbf{v} \in \mathbb{R}^3$ 为惯性系下的质心位置与速度，$\mathbf{R} \in SO(3)$ 为机体系到惯性系的旋转矩阵，$\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^\top \in \mathbb{R}^3$ 为机体系角速度，$\mathbf{f} = [f_1, f_2, f_3, f_4]^\top \in \mathbb{R}^4$ 为四个旋翼的单桨推力。在忽略气动阻力的名义刚体模型下，四旋翼动力学写为

$$
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v}, &
m\dot{\mathbf{v}} &= f_\Sigma \mathbf{R}\mathbf{e}_3 - m g \mathbf{e}_3, \\
\dot{\mathbf{R}} &= \mathbf{R}[\boldsymbol{\omega}]_\times, &
\mathbf{J}\dot{\boldsymbol{\omega}} &= \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{J}\boldsymbol{\omega}),
\end{aligned}
\tag{1}
$$

其中 $m$ 为质量，$g$ 为重力加速度，$\mathbf{J} \in \mathbb{R}^{3 \times 3}$ 为机体系惯量矩阵，$[\cdot]_\times$ 为反对称叉乘算子；总推力 $f_\Sigma$ 与机体系力矩 $\boldsymbol{\tau}$ 由常值控制分配矩阵 $\mathbf{B} \in \mathbb{R}^{4 \times 4}$ 确定，满足 $[f_\Sigma, \boldsymbol{\tau}^\top]^\top = \mathbf{B}\mathbf{f}$。

利用四旋翼的微分平坦性，系统状态与输入可由三维位置轨迹 $\mathbf{p}(t)$ 及其高阶时间导数代数恢复。当 $\ddot{\mathbf{p}} + g\mathbf{e}_3 \neq \mathbf{0}$ 时，机体推力轴方向与总推力分别为

$$
\mathbf{R}\mathbf{e}_3 = \frac{\ddot{\mathbf{p}} + g\mathbf{e}_3}{\|\ddot{\mathbf{p}} + g\mathbf{e}_3\|_2}, \qquad
f_\Sigma = m\|\ddot{\mathbf{p}} + g\mathbf{e}_3\|_2.
\tag{2}
$$

由于球形机体包络与推力约束均关于 $\mathbf{R}\mathbf{e}_3$ 旋转对称，规划固定零偏航角。在此约定下，对式 (2) 求一阶与二阶导数即可由三阶导数 $\mathbf{p}^{(3)}$ 和四阶导数 $\mathbf{p}^{(4)}$ 解析求得角速度 $\boldsymbol{\omega}$、角加速度 $\dot{\boldsymbol{\omega}}$ 及单桨推力 $\mathbf{f}$。物理可行性要求合成横滚俯仰角速度、偏航角速度与单桨推力分别满足 $\omega_x^2 + \omega_y^2 \le \bar{\omega}_{xy}^2$、$|\omega_z| \le \bar{\omega}_z$ 以及 $f_\ell \in [f_{\min}, f_{\max}]$（$\ell = 1, \ldots, 4$）。

## 3.2　动态门与穿越条件

设赛道按规定顺序包含 $M$ 个中间节点 $\mathcal{I} = \{1, \ldots, M\}$，分为动态实体门集合 $\mathcal{I}_{\mathrm{gate}} \subseteq \mathcal{I}$ 与可选无边框导航航点集合 $\mathcal{I}_{\mathrm{wp}} = \mathcal{I} \setminus \mathcal{I}_{\mathrm{gate}}$。对每扇实体门 $i \in \mathcal{I}_{\mathrm{gate}}$，记其名义中心为 $\bar{\mathbf{c}}_i \in \mathbb{R}^3$，固定门平面的正交单位基为 $\mathbf{E}_i = [\mathbf{e}_{i1}, \mathbf{e}_{i2}] \in \mathbb{R}^{3 \times 2}$，指向正向穿越一侧的单位法向量为 $\mathbf{n}_i = \mathbf{e}_{i1} \times \mathbf{e}_{i2}$。各门在与飞行轨迹共享的比赛时钟 $t \ge 0$ 下沿门平面做已知运动，中心位置表示为 $\mathbf{c}_i(t) = \bar{\mathbf{c}}_i + \mathbf{E}_i \mathbf{A}_i \mathbf{s}_i(t)$，其中 $\mathbf{A}_i \in SO(2)$ 定义面内运动方向。除静止门（$\mathbf{s}_i(t) = \mathbf{0}$）外，正弦平移与绕外部支点的弧线摆动分别由下式给出：

$$
\begin{aligned}
\mathbf{s}_i^{\mathrm{lin}}(t) &= [0,\, -a_i \sin(2\pi \nu_i t + \phi_i)]^\top, \\
\mathbf{s}_i^{\mathrm{arc}}(t) &= [\ell_i(1 - \cos\theta_i(t)),\, -\ell_i \sin\theta_i(t)]^\top, \quad
\theta_i(t) = \arcsin(a_i / \ell_i)\sin(2\pi \nu_i t + \phi_i),
\end{aligned}
\tag{3}
$$

其中 $a_i, \nu_i, \phi_i$ 分别为各门独立配置的幅度、频率与初相，$\ell_i > a_i$ 为摆长。

在门平面局部坐标系中，第 $i$ 扇门的净开口记为包含原点的紧致凸集 $\mathcal{A}_i \subset \mathbb{R}^2$：圆形门由净半径 $R_i^{\mathrm{op}}$ 定义为 $\|\boldsymbol{\xi}\|_2 \le R_i^{\mathrm{op}}$，凸四边形门由单位外法向量 $\mathbf{b}_{ik}$ 与常数 $d_{ik}$ 定义为 $\mathbf{b}_{ik}^\top \boldsymbol{\xi} \le d_{ik}$（$k = 1, \ldots, 4$）。设四旋翼规划包络球半径为 $r_s$，将 $\mathcal{A}_i$ 各边界向内腐蚀 $r_s$ 得到非空凸可行域 $\mathcal{A}_i^-$（即圆门满足 $\|\boldsymbol{\xi}\|_2 \le R_i^{\mathrm{op}} - r_s$，四边形门满足 $\mathbf{b}_{ik}^\top \boldsymbol{\xi} \le d_{ik} - r_s$）。记 $t_i$ 为第 $i$ 个节点的计划到达时刻，$\boldsymbol{\xi}_i \in \mathbb{R}^2$ 为门内穿越坐标，则有序穿越条件为

$$
\mathbf{p}(t_i) = \mathbf{c}_i(t_i) + \mathbf{E}_i \boldsymbol{\xi}_i, \qquad
\boldsymbol{\xi}_i \in \mathcal{A}_i^-, \qquad
0 < t_1 < \cdots < t_M < T,
\tag{4}
$$

其中 $T$ 为飞抵任务终点的全程时间（含末节点后的终段航程）。式 (4) 体现了动态门规划的时空耦合：改变到达时刻 $t_i$ 会移动开口在世界系中的位置 $\mathbf{c}_i(t_i)$，改变 $\boldsymbol{\xi}_i$ 则影响出窗姿态与后续航段转向；强制中心基准对应令 $\boldsymbol{\xi}_i = \mathbf{0}$。为保证单次正向过门，在实体门节点施加最小法向速度约束 $\mathbf{n}_i^\top \dot{\mathbf{p}}(t_i) \ge v_{\min} > 0$，并要求全程内有符号法向距离 $h_i(t) = \mathbf{n}_i^\top[\mathbf{p}(t) - \mathbf{c}_i(t)]$ 仅在 $t_i$ 处穿过净开口 $\mathcal{A}_i$。对导航航点 $i \in \mathcal{I}_{\mathrm{wp}}$，不设门框，仅要求 $\mathbf{p}(t_i) - \mathbf{c}_i(t_i) \in \mathcal{Z}_i$（$\mathcal{Z}_i \subset \mathbb{R}^3$ 为给定邻域）。

> **图 1 位置预留｜动态门的穿越几何与整机净空。** （a）门正面：净开口 $\mathcal{A}_i$、内缩选点域 $\mathcal{A}_i^-$、中心 $\mathbf{c}_i(t_i)$ 与门内坐标 $\boldsymbol{\xi}_i$。（b）法向截面：进门、质心过面与出门三个时刻的机体包络与移动门框，标出质心过面后仍可能发生的边框扫碰及最小净空时刻。

## 3.3　连续时间整机净空

式 (4) 仅约束了质心过面时刻 $t_i$ 的瞬时位置。由于机体具有有限体积且门框持续运动，在 $|h_i(t)|$ 较小的进出门过渡时段内，移动边框仍可能扫过机体（图 1b）。将环绕 $\mathcal{A}_i$ 的实体门框建模为半径 $r_f$ 的圆管，其闭合管中心线在时刻 $t$ 记为 $\mathcal{L}_i(t) \subset \mathbb{R}^3$（圆门对应半径 $R_i^{\mathrm{op}} + r_f$ 的圆周，四边形门对应向外偏置 $r_f$ 的四条边线段并集）。半径为 $r_s$ 的机体球包络与移动门框在全程保持无碰撞分离，等价于质心到 $\mathcal{L}_i(t)$ 的最短欧氏距离在任意时刻均不小于 $r = r_s + r_f$：

$$
d_i(t) = \operatorname{dist}\!\left(\mathbf{p}(t),\, \mathcal{L}_i(t)\right) - r \ge 0, \qquad
\forall i \in \mathcal{I}_{\mathrm{gate}}, \quad \forall t \in [0, T].
\tag{5}
$$

由于门在固定平面内运动，$h_i(t) = \mathbf{n}_i^\top[\mathbf{p}(t) - \bar{\mathbf{c}}_i]$ 与面内相对位移正交，故有 $\operatorname{dist}(\mathbf{p}(t), \mathcal{L}_i(t)) \ge |h_i(t)|$。当 $|h_i(t)| \ge r$ 时式 (5) 自然成立；在 $|h_i(t)| < r$ 的穿越区间内，面内相对运动共同决定真实净空，最小净空时刻一般偏离 $t_i$。

## 3.4　半无限规划表述

设分段光滑轨迹 $\mathbf{p}(t; \boldsymbol{\theta})$ 由有限维决策向量 $\boldsymbol{\theta} \in \Theta \subset \mathbb{R}^{n_\theta}$ 参数化，其中 $\boldsymbol{\theta}$ 联合包含各段时长参数、中间节点选点坐标以及内部节点的一至三阶导数，两端固定为给定起终边界状态 $\mathcal{X}_0$ 与 $\mathcal{X}_f$（具体参数化与积分形式见第 4 节）。令 $J_{\mathrm{dyn}}(\boldsymbol{\theta})$ 表示对角速度与单桨推力越限量的时间积分平滑惩罚，$w_{\mathrm{dyn}} > 0$ 为惩罚权重，全赛道动态门时间最优轨迹规划归结为如下半无限规划（SIP）问题：

$$
\begin{aligned}
\min_{\boldsymbol{\theta} \in \Theta} \quad & J(\boldsymbol{\theta}) = T(\boldsymbol{\theta}) + w_{\mathrm{dyn}} J_{\mathrm{dyn}}(\boldsymbol{\theta}) \\
\mathrm{s.t.} \quad & \mathcal{D}_3 \mathbf{p}(0; \boldsymbol{\theta}) = \mathcal{X}_0, \quad \mathcal{D}_3 \mathbf{p}(T(\boldsymbol{\theta}); \boldsymbol{\theta}) = \mathcal{X}_f, \\
& \mathbf{p}(t_i(\boldsymbol{\theta}); \boldsymbol{\theta}) = \mathbf{c}_i(t_i(\boldsymbol{\theta})) + \mathbf{E}_i \boldsymbol{\xi}_i(\boldsymbol{\theta}), \quad \boldsymbol{\xi}_i(\boldsymbol{\theta}) \in \mathcal{A}_i^-, \quad \forall i \in \mathcal{I}_{\mathrm{gate}}, \\
& \mathbf{p}(t_i(\boldsymbol{\theta}); \boldsymbol{\theta}) - \mathbf{c}_i(t_i(\boldsymbol{\theta})) \in \mathcal{Z}_i, \quad \forall i \in \mathcal{I}_{\mathrm{wp}}, \\
& \mathbf{n}_i^\top \dot{\mathbf{p}}(t_i(\boldsymbol{\theta}); \boldsymbol{\theta}) \ge v_{\min}, \quad \forall i \in \mathcal{I}_{\mathrm{gate}}, \\
& d_i(\boldsymbol{\theta}, t) = \operatorname{dist}\!\left(\mathbf{p}(t; \boldsymbol{\theta}),\, \mathcal{L}_i(t)\right) - r \ge 0, \quad \forall i \in \mathcal{I}_{\mathrm{gate}}, \quad \forall t \in [0, T(\boldsymbol{\theta})],
\end{aligned}
\tag{6}
$$

其中 $\mathcal{D}_3 \mathbf{p} = (\mathbf{p}, \dot{\mathbf{p}}, \ddot{\mathbf{p}}, \mathbf{p}^{(3)})$ 表示位置至三阶导数组成的边界状态算子。

式 (6) 具有有限维决策变量 $\boldsymbol{\theta}$，但净空约束遍及连续时间区间 $[0, T(\boldsymbol{\theta})]$ 内的无穷多个时刻，且时间域上界与各门中心线 $\mathcal{L}_i(t)$ 均随时间分配动态变化。直接在固定离散网格上采样该约束可能遗漏采样间隔内的高速门框扫碰；而采用传统无收紧交换法逐轮添加违反时刻，既缺乏排除安全时段的连续下界，又容易在切触点附近产生无限密集的约束积聚而丧失有限终止性。下一节将给出显式节点导数的稀疏轨迹表示，并通过右端项收紧与连续时间区间分离求解式 (6)。
