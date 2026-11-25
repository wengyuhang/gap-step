# 论文新大纲（IEEE T-RO / T-ON 融合风格版）

> **说明**：本大纲基于截至 2026-11-22 的最新方法与冻结实验结果（`robust_ipopt_full_20261117`、`six_methods_cpc_repaired_20261117`、`minco_position_ablation_20261122`）编写，保留原草案 [PAPER_PLAN.md](file:///home/jack/wyh/时变窗口/论文写作/PAPER_PLAN.md) 不变。整体行文逻辑与章节范式深度仿照以下四篇参考论文：
> 
> 1. **Zhang et al., IEEE T-RO 2024** (*Provably Feasible SIP Under Collision Constraints via Subdivision*)：SIP 问题抽象、核心机制与理论分析（Assumption–Lemma–Theorem–Proof）组织、细分/收敛曲线剖析；
> 2. **Gao et al., IEEE T-RO 2026** (*SIP for Collision Avoidance in Optimal and MPC*)：引言内嵌 Contributions/Notations/Key Symbols 表/Outline、由浅入深的算法展开、显式披露 Approximations & Remarks、分项计算耗时与参数附录表；
> 3. **Shao et al., IEEE T-ON 2026** (*Charging Optimization for Mobile Devices With MARL*)：引言中显式提炼核心挑战（Core Challenges）、相关工作末尾单设 *Differences Between Our Work and Related Work*、算法复杂度分析、系统化消融表以及独立的 *Limitations and Practical Deployment Considerations* 章节；
> 4. **Bianco, Ren, Xing, Scaramuzza, arXiv:2610.37557 (2026)** (*Learning Visuomotor Quadrotor Flight Through Dynamic Narrow Gaps via Differentiable Simulation*)：平实克制的标题与术语规范（不硬造方法缩写，使用 *dynamic gates* / *time-varying gate plane* 等规范表述）。

---

## 论文元数据与定位

- **英文标题**：*Time-Optimal Quadrotor Flight Through Dynamic Gates via Semi-Infinite Programming*
- **中文拟题**：基于半无限规划的动态门四旋翼时间最优轨迹规划
- **方法称呼与图表标签**（仿照 Zhang 2024、Gao 2026 与 Bianco et al. 2026，不硬造缩写）：
  - **正文称呼**：*the proposed SIP-based planner* / *our planner*（本文 SIP 规划器 / 本文方法）；
  - **图表与消融标签**：自由选点主方法记为 **`Ours (Free)`**，固定中心消融版记为 **`Ours (Center)`**，仅位置 $C^6$ MINCO 消融版记为 **`Ours (Pos-Only)`**；对比基线分别记为 **`Dyn-TOGT (Free)`**、**`Dyn-TOGT (Center)`**、**`Wang`**、**`Dyn-CPC`**。
- **目标体量**：IEEE Transactions 双栏 14–16 页（含参考文献与附录）。
- **核心研究问题**：在门框沿已知规律（平面内平移或绕外部支点弧线摆动）持续运动的竞速赛道中，如何联合优化全程段时长、门内穿越位置与高阶节点状态（PVAJ），在最小化全程飞行时间的同时，消除有限机体与移动门框在连续时间域内的“过面扫掠碰撞（frame-sweep collision）”风险？

---

## Abstract & Index Terms（约 250 词，仿照 Gao 2026 & Shao 2026）

**行文逻辑（四步闭环）**：

1. **背景与痛点（2 句）**：面向动态门的自主四旋翼时间最优飞行要求同时决定每扇门的到达时刻、门内穿越位置与过门运动状态。由于门位姿随比赛时钟持续变化，质心在过面瞬间落入开口内并不能保证有限尺寸机体在进门与出门过程中与移动门框保持分离，从而构成一类带有连续时间非凸碰撞约束的半无限规划（SIP）问题。
2. **建模与方法（3 句）**：本文提出一种基于半无限规划的全赛道轨迹规划方法，将段时长、缩放门内二维穿越坐标以及显式节点位置-速度-加速度-加加速度（PVAJ）状态统一纳入分段七次多项式优化。相较于通过 $C^6$ 高阶连续性消去内部导数的仅位置 MINCO，本文释放完整的 $C^3$ 内部节点导数，并结合局部 Bernstein 等式与残差上镜图（residual epigraph）构建具备精确二阶导的块稀疏有限子问题。为处理连续时间分离约束，求解器采用右端项收紧（right-hand-side tightened）交换机制：在每轮有限 NLP 求解后，利用法向多项式求根裁剪活性区间，结合 Bernstein 凸包与门运动解析区间界执行自适应细分，定位真实违反时刻并以随段时长同步更新的时间坐标和原始-对偶热启动迭代补入约束。
3. **理论支撑（1 句）**：在目标子水平集一致有界与完备分离假设下，给出了算法有限终止与输出连续可行性的充分条件。
4. **实验与量化结果（3 句）**：在包含 4 种赛道布局、22 个动态条件的基准集上，`Ours (Free)` 与 `Ours (Center)` 均实现 **22/22** 几何与任务全通过（独立候选通过率分别为 176/176 与 172/176），显著高于 `Dyn-TOGT (Free)`（11/22）、`Dyn-TOGT (Center)`（19/22）、`Wang`（12/22）及 `Dyn-CPC`（13/22），并在 **19/22** 个条件中取得最短有效飞行时间。同框架配对消融表明：开放门内选点在 22/22 条件中均缩短全程时间（节省 **2.99%–10.87%**，12 个纯实体门条件中位节省 **5.18%**）；相比消去内部导数的仅位置 $C^6$ MINCO 变体 `Ours (Pos-Only)`，显式保留节点 PVAJ 自由度避免了中位 **17.36%**（自由点）与 **22.80%**（中心）的飞行时间退化。

**Index Terms**：Quadrotor racing, dynamic gates, semi-infinite programming (SIP), trajectory optimization, continuous-time collision avoidance.

---

## Section I. Introduction（约 2 页，仿照 Shao 2026 + Gao 2026 + Zhang 2024）

### 开篇叙事（3 段，问题驱动）

- **第 1 段（任务背景与物理现象）**：自主四旋翼高速穿门是检验时间最优轨迹规划与几何安全边界的典型基准。静态门竞速已取得显著进展，但当门框沿已知规律平移或摆动时，门框在机体穿越前后持续扫过空间（配合 **Fig. 1(a)** 直观展示进门、过面、出门三时刻的“过面但仍相交”现象）。
- **第 2 段（现有范式的局限）**：
  - 航点邻域类方法（如 CPC）通过离散互补进度约束隐式选择过门位置，缺乏对移动实体门框连续几何净空的显式建模；
  - 静态/动态凸门选点方法（如 TOGT 及其动态适配）采用仅位置 $C^6$ MINCO，虽能在到达时刻优化门内坐标，但一方面仅约束质心过面瞬间，忽略过面前后的有限机体扫掠，在高幅高频运动下易发生碰撞；另一方面由 $C^6$ 连续性消去内部导数，在纯时间最优目标下限制了门节点处的速度、加速度与加加速度调节空间；
  - 滚动局部重规划方法（如 Wang）缺少全赛道时空协同，导致保守且较慢的全程时间。
- **第 3 段（提炼四大核心挑战，仿照 Shao Section I 斜体小标题风格）**：
  1. *Spatiotemporal Coupling of Gate Motion and Arrival Timing*：累积段时长同时决定各门到达相位与世界位姿，前序门的时空间决策通过动力学链式传递影响后续所有航段；
  2. *Continuous-Time Frame-Sweep Safety vs. Finite-Time Crossing*：穿越事件仅发生在有限个节点时刻，而整机与移动门框的分离必须覆盖整个连续时间域 $[0, T(\boldsymbol{\theta})]$，且该时间域本身随决策变量变化；
  3. *High-Order Trajectory Coupling and Numerical Ill-Conditioning*：显式节点高阶导数（PVAJ）与段时长强耦合，叠加姿态奇点附近的非光滑切换，极易导致标准非线性规划求解器陷入恢复失败或迭代耗尽；
  4. *Trade-Off Between Time-Optimality, Dynamic Feasibility, and Clearance*：激进压缩飞行时间会将轨迹推向门框边界与动力学极限，如何在不牺牲时间最优性的前提下可靠满足连续净空约束仍是难点。
- **第 4 段（提出本文 SIP 规划框架总览）**：将全赛道动态门时间最优飞行表述为半无限规划（SIP），介绍本文方法如何通过“释放完整节点 PVAJ 导数的稀疏 Hermite/Bernstein 有限子问题 + 右端项收紧 + 法向求根与 Bernstein 区间包络分离 + 随段时长可微更新的见证索引”系统性解决上述挑战（引出 **Fig. 1(b)–(d)** 方法全景图）。

### A. Contributions（仿照 Gao I-A / Shao I，4 条精准收束）

1. **动态门时间最优飞行的 SIP 建模**：将已知平移/摆动门运动、内缩凸开口二维穿越坐标、显式节点 PVAJ 导数及连续时间整机-门框距离约束统一为全赛道 SIP，刻画过面时刻与最近接近时刻分离的几何本质。
2. **稀疏二阶可微参数化与右端收紧交换求解器**：在 MINCO 框架内释放完整的 $C^3$ 内部节点导数（退化为局部解耦的七次 Hermite 参数化），结合局部 Bernstein 等式中间量、残差上镜图与连续零偏航姿态图构造稀疏精确二阶有限 NLP，并配合法向多项式求根裁剪、Bernstein 区间自适应细分和随段时长同步更新的约束索引完成求解。
3. **有限终止与可行性理论分析**：在右端收紧余量、目标子水平集等度连续性与完备分离假设下，建立求解器的有限终止充分条件与输出轨迹的连续净空可行性保证。
4. **多赛道基准对比与双维度机制消融**：在 4 种赛道布局、22 个动态条件下完成六方法统一协议对比，并通过严格的同框架固定中心消融与仅位置 $C^6$ MINCO 消融，分别量化门内选点自由度与节点高阶导数（VAJ）自由度对飞行时间与求解可靠性的独立贡献。

### B. Notations（仿照 Gao I-B）

- 统一约定向量、矩阵、范数、$SO(3)$、叉乘矩阵 $[\cdot]_\times$、点到集合距离 $\operatorname{dist}(\cdot,\cdot)$ 及索引集合记号。
- **Table I: Key Symbols Used in This Article**（集中列出 $\mathbf{p}, \mathbf{R}, \boldsymbol{\omega}, \mathbf{f}, \mathbf{c}_i(t), \mathbf{E}_i, \mathbf{n}_i, \boldsymbol{\xi}_i, \Delta t_k, \boldsymbol{\theta}, r_s, r_f, r, \delta_r, \mathcal{S}_i^{(k)}$ 等核心符号及首次引入公式号）。

### C. Outline（仿照 Gao I-D）

- 简明交代 Section II–VIII 与 Appendix 的组织脉络。

---

## Section II. Related Work（约 1.2 页，仿照 Zhang II + Shao II）

### A. Time-Optimal Quadrotor Racing Trajectory Planning

- 梳理最小 Snap / 自由导数多项式规划（Richter et al. IJRR 2016）、MINCO 时空参数化（Wang et al. T-RO 2022）、全赛道数值最优控制（CPC，Foehn et al. Science Robotics 2021）以及静态凸门选点规划（TOGT，Qin et al. ICRA 2024）。
- 指出它们在“穿越任务表示”上的演进：固定航点 $\to$ 航点邻域（CPC）$\to$ 静态凸开口（TOGT），但均未显式处理动态门框在过面前后的连续扫掠碰撞，且 TOGT 默认的仅位置 $C^6$ MINCO 消元在纯时间最优目标下过度约束了门节点处的高阶导数。

### B. Dynamic Gate Traversal and Moving-Gap Flight

- 综述面向动态门或移动窄缝的规划与控制方法：包括基于分段多项式的滚动局部重规划（Wang et al.）、以及最新的端到端可微仿真视觉运动策略（Bianco et al., arXiv:2610.37557, 2026，针对单扇动态窄缝 *dynamic narrow gap* 的局部穿越控制）。
- 指出局部滚动规划或单缝学习策略缺乏全赛道多门时钟与相位协同，而直接将静态凸门规划器扩展到动态门时若只检查过面节点，会在高频/大振幅场景中出现严重的有限机体碰撞。

### C. Semi-Infinite Programming for Collision-Constrained Optimization

- 重点综述机器人碰撞约束中的 SIP 方法：
  - **经典交换法（Exchange Method）**（Hauser 2021 等）：按离散/数值检测迭代添加最深穿透点，但在缺乏收紧或连续下界机制时无法保证有限终止或无漏检；
  - **保守细分内点法**（Zhang et al., T-RO 2024）：索引为连续时间，通过 Lipschitz 运动界与自适应细分确保迭代可行与有限终止，但依赖全局保守常数与定制线搜索；
  - **局部约化与外部主动集法**（Gao et al., T-RO 2026）：索引来自机器人多边形形状参数与状态不确定性，通过下层 QP/QCQP 局部约化结合外部主动集实现实时 OCP/MPC，但主要针对离散时间网格上的碰撞避免；
  - **右端项收紧（RHS Restriction）理论**（Mitsos 2011 等）：通过向内收紧有限子问题右端项构造严格可行近似，为交换型 SIP 提供有限终止机制。

### D. Differences Between Our Work and Related Work（仿照 Shao II-D，核心差异段）

- 用一段高度凝练的对比明确本文的独特位置：
  1. **与 TOGT / CPC / Wang 相比**：本文同时保留全赛道时空耦合、凸开口自由选点、显式节点 PVAJ 导数自由度以及连续时间整机-门框距离约束；
  2. **与 Zhang (2024) 相比**：本文处理的是终端时间 $T(\boldsymbol{\theta})$ 与门框位姿均依赖于决策变量 $\boldsymbol{\theta}$ 的时间最优飞行 SIP，采用精确二阶稀疏 IPOPT 与右端收紧分离，而非保守全局 Lipschitz 屏障内点法；
  3. **与 Gao (2026) 相比**：本文的无限索引源于**连续时间域**上的动态门框扫掠而非静态点云与形状参数，因此结合一元法向多项式求根裁剪与 Bernstein 区间包络进行连续时间分离，并将检测到的违反时刻参数化为随段时长自适应变化的约束。

---

## Section III. System Model and SIP Problem Formulation（约 2.5 页，仿照 Gao II + Shao III + Zhang III）

> **写法原则**：每小节严格遵循“定义物理对象 $\to$ 给出数学方程 $\to$ 解释在穿越任务中的几何/动力学作用”的递进顺序。

### A. Quadrotor Nominal Dynamics and Differential Flatness

- 给出惯性系 $\mathcal{W}$ 与机体系 $\mathcal{B}$ 下的刚体动力学方程（位置 $\mathbf{p}$、速度 $\mathbf{v}$、姿态 $\mathbf{R}\in SO(3)$、角速度 $\boldsymbol{\omega}$、四旋翼推力 $\mathbf{f}=[f_1,\dots,f_4]^\top$、质量 $m$、惯量 $\mathbf{J}$、推力分配矩阵 $\mathbf{B}$）。
- 说明微分平坦性如何由位置轨迹二至四阶导数 $(\ddot{\mathbf{p}}, \dddot{\mathbf{p}}, \mathbf{p}^{(4)})$ 代数恢复推力轴 $\mathbf{b}_3$、角速度 $\boldsymbol{\omega}$、角加速度 $\boldsymbol{\alpha}$ 与单旋翼推力 $f_j$。
- **关键实现细节融入建模**：说明采用零偏航连续姿态图（`continuous_attitude_chart`），在 $a_z < 0$ 时用等价无抵消分母 $(a_x^2+a_y^2)/(\|\mathbf{a}\|(\|\mathbf{a}\|-a_z))$ 替代 $1+b_{3z}$ 的分段阈值切换，保证定义域内目标与梯度处处光滑（真实倒置极点与零推力仍为奇点）。

### B. Dynamic Convex Gate Geometry and Kinematics

- 定义赛道中 $M$ 扇按序穿越的动态实体门（以及基础布局中可能包含的导航航点）。每扇门具有固定门平面正交基 $\mathbf{E}_i = [\mathbf{e}_{i1}, \mathbf{e}_{i2}]$、单位正向法向 $\mathbf{n}_i = \mathbf{e}_{i1}\times\mathbf{e}_{i2}$，中心运动统一表示为 $\mathbf{c}_i(t) = \bar{\mathbf{c}}_i + \mathbf{E}_i \mathbf{A}_i \mathbf{s}_i(t)$。
- 给出两类周期性运动规律：平面内正弦平移 $\mathbf{s}_i^{\mathrm{lin}}(t)$ 与绕外部支点（摆长 $\ell_i$）的弧线摆动 $\mathbf{s}_i^{\mathrm{arc}}(t)$，各门独立配置幅度 $a_i$、频率 $\nu_i$ 与初相 $\phi_i$，共享比赛时钟 $t$。
- 定义门框实体几何为半径 $r_f = 0.085\,\text{m}$ 的圆管沿中心线 $\mathcal{L}_i(t)$（圆形为圆环，四边形为四条有限线段并集）扫掠而成；定义规划机体包络球半径 $r_s = 0.20\,\text{m}$ 与独立物理碰撞代理球半径 $r_b = 0.15\,\text{m}$。

### C. Spatiotemporal Traversal Conditions and Continuous-Time Clearance

- **时空耦合穿越等式**：设第 $i$ 扇门的计划穿越时刻为 $t_i$（由累积段时长决定），门内局部穿越坐标为 $\boldsymbol{\xi}_i \in \mathbb{R}^2$，则
  $$\mathbf{p}(t_i) = \mathbf{c}_i(t_i) + \mathbf{E}_i \boldsymbol{\xi}_i, \qquad 0 < t_1 < \cdots < t_M < T.$$
- **内缩凸开口与穿越方向**：为保证质心过面瞬间不碰框，要求 $\boldsymbol{\xi}_i \in \mathcal{A}_i^-$（圆门 $\|\boldsymbol{\xi}_i\|_2 \le R_i^{\mathrm{op}} - r_s$；四边形 $\mathbf{b}_{ik}^\top \boldsymbol{\xi}_i \le d_{ik} - r_s$；固定中心消融 `Ours (Center)` 直接消去 $\boldsymbol{\xi}_i \equiv \mathbf{0}$）；并施加前向法向速度条件 $\mathbf{n}_i^\top \dot{\mathbf{p}}(t_i) \ge v_{\min} > 0$。同时定义基于法向有符号距离 $h_i(t) = \mathbf{n}_i^\top[\mathbf{p}(t)-\mathbf{c}_i(t)]$ 零点的实际正向穿越与无重复穿越判据。
- **连续时间整机净空约束**：定义规划净空函数
  $$d_i(\boldsymbol{\theta}, t) := \operatorname{dist}\!\left(\mathbf{p}(t; \boldsymbol{\theta}),\, \mathcal{L}_i(t)\right) - r \ge 0, \qquad \forall t \in [0, T(\boldsymbol{\theta})],\; \forall i \in \{1,\dots,M\},$$
  其中 $r = r_s + r_f = 0.285\,\text{m}$ 为中心线安全距离阈值。配合 **Fig. 1(a)** 阐明：节点过面合法（$\boldsymbol{\xi}_i \in \mathcal{A}_i^-$）仅保证 $t=t_i$ 一点安全，无法排除 $t \neq t_i$ 时的动态门框扫掠碰撞。

### D. Semi-Infinite Programming Formulation

- 合并轨迹参数 $\boldsymbol{\theta}$、时间与软动力学惩罚目标 $J(\boldsymbol{\theta}) = T(\boldsymbol{\theta}) + w_{\mathrm{dyn}} J_{\mathrm{dyn}}(\boldsymbol{\theta})$、有限边界/开口/方向约束及连续时间安全约束，写出完整的标准编号 **SIP 问题 (P-SIP)**。
- **问题结构剖析（仿照 Zhang III-B & Shao III-D）**：明确指出 (P-SIP) 的两大求解难点——(1) 无限时间约束叠加决策变量依赖的时间域 $[0, T(\boldsymbol{\theta})]$ 与门运动 $\mathcal{L}_i(t)$，使固定网格离散化极易漏检窄时窗擦碰；(2) 朴素交换法在无右端收紧和无连续下界剪枝的情况下，既可能错过违反区间，又可能因无穷聚集点而无法有限终止。自然引出 Section IV。

---

## Section IV. Tightened Exchange-Based SIP Solver（约 3.5 页，仿照 Gao III–IV + Zhang IV）

> **写法原则**：开头用一段话 + **Fig. 2（求解器整体框架与时空见证更新示意图）** 统领全局，随后按“轨迹与子问题参数化 $\to$ 右端收紧有限 NLP $\to$ 连续时间分离 $\to$ 索引更新与主循环”四步展开，伪代码放在所有数学对象定义完毕之后。

### A. Explicit PVAJ Trajectory Parameterization and Soft Dynamics Epigraph

1. **释放完整内部节点导数（$C^6 \to C^3$）与局部 Bernstein 等式**：
   - 将全程划分为 $K$ 段七次多项式（段内归一化时间 $u \in [0,1]$）。标准 MINCO 框架支持指定不同阶数的中间导数约束 $d_i \in \{1,\dots,4\}$；现有竞速规划（如 TOGT）默认取 $d_i=1$（仅位置），借助跨段 $C^6$ 连续性消去内部 $(\mathbf{v}_i, \mathbf{a}_i, \mathbf{j}_i)$。
   - 但在纯时间最优目标下，控制输入（对应 4 阶导数 Snap）在门节点附近天然需要快速切换；强加 $C^4, C^5, C^6$ 连续性会过度限制可行轨迹空间，并使每段系数沿全轴耦合所有节点与段时长。本文取 $d_i=4$，显式保留所有内部节点的完整 0–3 阶导数（PVAJ，$C^3$ 连续），使每段 7 次多项式退化为局部解耦的**七次 Hermite 插值**（单段内部仍为最小 Snap 形式，但允许 Snap 在门节点处切换，且每段系数仅依赖相邻两端节点与本段时长）。
   - 在有限 NLP 中引入每段 Bernstein 控制点作为中间变量，通过局部 Hermite-Bernstein 线性等式保持与端点 PVAJ 一致，赋予二阶优化器严格的块稀疏结构。
2. **正时间映射与缩放门内坐标**：
   - 采用 TOGT 光滑双射正时间映射 $\Delta t_k(\eta_k) > 0$；
   - 实体门穿越点采用按开口尺度归一化的直接二维物理坐标 $\boldsymbol{\xi}_i = \operatorname{diag}(\mathbf{s}_i)\mathbf{z}_i$（`scaled_xy`），圆盘与四边形半空间约束直接进入有限 NLP，避免非线性映射带来的奇异性与冗余自由度。
3. **软动力学惩罚与残差上镜图（Residual Epigraph）**：
   - 给出 6 项动力学越限残差 $q_1,\dots,q_6$（合成 $\omega_{xy}$、$\omega_z$ 及四电机推力）、$C^2$ 光滑正部函数 $\psi_\mu$（$\mu=0.01$）及每段 16 子区间梯形积分 $J_{\mathrm{dyn}}$（权重 $w_{\mathrm{dyn}}=1000$）。
   - 引入非负残差上镜图辅助变量 $s_{k,\ell,j} \ge \max(0, \tilde{q}_{k,\ell,j})$，等价转化复合光滑罚项，使 NLP 具备干净的稀疏结构与精确解析 Hessian。

### B. Right-Hand-Side Tightened Finite NLP Subproblem

- 引入第 $k$ 轮每扇门的有限时间索引集 $\mathcal{S}_i^{(k)}$，将无限约束松弛为有限约束集，同时将距离右端项向内收紧 $\delta_r > 0$（即要求 $d_i(\boldsymbol{\theta}, \tau) \ge \delta_r$，等价于中心线距离 $\ge r + \delta_r$，实现中 $r=0.2850\,\text{m}, \delta_r=10^{-4}\,\text{m}$）。
- 给出圆形中心线（径向-法向平方和）与四边形中心线（四条有限线段截断投影最小值）的距离约束表达式。
- 明确有限 NLP 的严格接纳门槛：仅当 IPOPT 返回 `Solve_Succeeded` 且有限约束最大违反量 $\le \epsilon_{\mathrm{tol}} = 10^{-5} \ll \delta_r$ 时，才将当前候选 $\boldsymbol{\theta}^{(k)}$ 送入连续时间分离模块（保留净正有效余量 $\delta_r - \epsilon_{\mathrm{tol}} > 0$，这是 Section V 有限终止的关键前提）。

### C. Continuous-Time Separation via Normal-Root Pruning and Bernstein Envelopes

- 针对固定候选 $\boldsymbol{\theta}^{(k)}$ 与每对（门 $i$，段 $p$），设计连续时间分离器（**Algorithm 1: Continuous-Time Separation**）：
  1. **重复穿越检测**：利用门平面固定特性，$h_i(u)$ 为一元七次多项式，通过伴随矩阵特征值求根检查非指定节点处是否存在开口内重复过面（`repeat_aperture_crossing`）。
  2. **法向活性区间裁剪**：求解 $h_i(u) = \pm r$ 的实根，将 $[0,1]$ 划分为若干子区间，直接丢弃满足 $|h_i(u)| > r$ 的安全区间，仅对靠近门平面的活性区间构造 Bernstein 控制点。
  3. **时空包围盒与自适应细分**：在每个待检子区间 $[u_a, u_b]$ 上，利用 Bernstein 控制点凸包界定轨迹坐标区间 $[\underline{\mathbf{p}}, \overline{\mathbf{p}}]$，结合门运动 $\mathbf{c}_i(t)$ 的解析区间上下界 $[\underline{\mathbf{c}}_i, \overline{\mathbf{c}}_i]$ 得到相对位置包围盒，进而计算到圆环或四边形框管中心线的距离下界 $\underline{D}_i([u_a, u_b])$。
  4. **优先队列分支定界**：按最小下界弹出区间——若端点/中点采样距离 $< r$ 则立即返回真实违反见证（`violated`）；若下界 $\ge r$ 则排除该区间；否则 de Casteljau 中点二分入队。达到细分上限 $N_{\max}=16{,}384$ 仍未排除时诚实返回 `budget_unresolved`（绝不将其伪装为安全或虚构违反点）。

### D. Time-Varying Index Parameterization and Primal-Dual Warm-Started Exchange

- **随段时长自适应的约束索引**：
  - 对紧邻第 $i$ 扇门穿越节点的相邻前/后段，用距穿越节点的时间偏移 $\Delta \tau$（`before` / `after` offset，当段时长缩短时截断至段内）编码约束，使其在过门附近保持物理时间对齐；
  - 对其余非相邻段，采用段内归一化时间 $u \in [0,1]$ 编码。
  - 在下一轮有限 NLP 中，每个约束均依据当前优化变量 $\boldsymbol{\theta}$ 的新段起始时刻与时长实时重算绝对时刻 $t(\boldsymbol{\theta})$ 与门位姿 $\mathbf{c}_i(t(\boldsymbol{\theta}))$，保持完整梯度传递。
- **单门最严重见证更新与原始-对偶热启动**：
  - 每扇门每轮至多加入一个最严重且与已有索引间隔 $>10^{-5}$ 的新违反见证，保留全部历史索引（单调扩充）；
  - 按约束语义键继承上一轮严格成功解的原始变量与对偶乘子（`warm_start_bound_push/frac = 1e-9`），新加入约束的乘子初始化为零。
- 给出 **Algorithm 2: Tightened Exchange-Based SIP Solver** 完整伪代码。
- **Remarks on Numerical Scope（仿照 Gao IV-D/E 的诚实披露）**：
  - *Remark 1*：动力学约束采用 16 区间软罚积分而非连续硬约束；
  - *Remark 2*：分离模块使用双精度浮点 Bernstein 区间包络，提供高强度数值几何证据，但非基于严格舍入区间算术的形式化连续证明（continuous certificate）。

---

## Section V. Theoretical Analysis and Realization Details（约 2 页，仿照 Zhang V–VI + Shao IV-D + Gao V-D）

### A. Finite Termination and Feasibility Analysis（仿照 Zhang V，严密命题链）

- **Assumption 1（有界候选子水平集）**：外层迭代生成的候选序列 $\{\boldsymbol{\theta}^{(k)}\}$ 位于有界目标子水平集 $J(\boldsymbol{\theta}^{(k)}) \le J_{\max}$ 内，且各段时长满足正下界与上界。
- **Assumption 2（完备分离与有效正余量）**：分离算子在原阈值 $r$ 上完备，且有限子问题满足 $\epsilon_{\mathrm{tol}} < \delta_r$。
- **Lemma 1（子水平集上的轨迹与净空等度连续性）**：由有界总时间 $T$、推力软惩罚界与固定阶七次多项式范数等价性，导出位置速度一致上界，结合光滑周期门运动的有界速度，证明距离函数族 $\{t \mapsto d_i(\boldsymbol{\theta}^{(k)}, t)\}$ 具有一致 Lipschitz 常数 $L_d$。
- **Theorem 1（输出连续可行性）**：若 Algorithm 2 在第 $k$ 轮经分离算子通过并终止，则输出轨迹 $\boldsymbol{\theta}^{(k)}$ 满足所有动态实体门在连续时间域 $[0, T(\boldsymbol{\theta}^{(k)})]$ 上的安全净空约束 $d_i(\boldsymbol{\theta}^{(k)}, t) \ge 0$。
- **Theorem 2（有限轮次终止）**：在 Assumptions 1–2 下，任意新加入的违反时刻 $\tau^{(k)}$ 与此前同一门的历史索引 $\tau^{(j)}$（$j<k$）之间存在一致正间距 $|\tau^{(k)} - \tau^{(j)}| \ge (\delta_r - \epsilon_{\mathrm{tol}}) / L_d > 0$；结合时间域紧致性，Algorithm 2 必在有限轮次内终止。（详细证明放 **Appendix A**，正文给出核心几何直觉与推导骨架）。

### B. Curvature-Driven Independent Multi-Start Initialization

- 详述 `curvature_initials` + `--robust-ipopt-profile` 流程：
  1. 沿赛道名义中心折线估计各节点曲率，经曲率限速与前向-后向加减速传播生成 8 组初始段时长（2 档速度上限 × 4 档加速度比例）；
  2. 按预测到达时刻查询动态门中心，通过位置式 MINCO 拟合得到初始节点 VAJ；
  3. 对初始 17 节点动力学筛查未通过的配置，执行固定 1.25 倍初始时长退让并重算门中心与 MINCO 导数，首次通过后再做 8 步可行括区二分细化。
- **公平性声明**：`Ours (Free)` 与 `Ours (Center)` 严格共享完全相同的 8 组未优化原始曲线并各自独立求解，绝不借用其他方法或另一消融分支的优化解。同时披露短赛道中部分配置可能退化为相同原始曲线（22 条件共 138 条互异原始曲线/方法），避免夸大独立初值数。

### C. Computational Complexity and Software Implementation Details（仿照 Shao IV-D & Gao V-D）

- **复杂度分析**：给出单轮稀疏有限 NLP 的变量规模（随段数 $K$ 线性增长，PVAJ 局部依赖使 KKT 矩阵呈块稀疏）、每轮分离器的复杂度（求根 $\mathcal{O}(M K)$ + Bernstein 最坏细分 $\mathcal{O}(N_{\max})$）以及 16 路进程级并行调度开销。
- **IPOPT 与验收工程配置**：精确 Hessian、adaptive $\mu$ 策略、`theta_max_fact = 1`、关闭 watchdog、最大有限迭代 5,000、最大外层交换 64 轮；有限求解失败立即中止该初值，绝不以未收敛点继续补点。
- **独立终验链**：通过浮点 Bernstein 复查的轨迹进一步接受独立真实门框 STL 网格碰撞检查（物理球半径 $r_b = 0.15\,\text{m}$）、实际过面顺序与无重复穿越检查以及任务终点检查。

---

## Section VI. Numerical Evaluation and Ablation Studies（约 3.5–4 页，仿照 Zhang VII + Gao VI + Shao V）

> **写法原则**：所有数据严格取自 2026-11-17 与 2026-11-22 冻结批次；坚持“成功与失败并列报告、飞行时间与计算墙钟分表报告、同框架消融与跨方法对比边界清晰”。

### A. Experimental Setup and Evaluation Protocol

- **四大赛道布局与 22 个确定性动态条件**（配 **Fig. 3** 四赛道俯视/三维示意图，标注平移直箭头与摆动弯箭头）：
  1. *Base 7-Node Track*（6 实体门 + 1 导航航点，3 平移四边形 + 3 摆动圆门，10 档：静态 + 幅度 $\{0.25, 0.50, 0.75\} \times$ 频率倍率 $\{1, 3, 6\}$）；
  2. *Split-S*（7 实体门，4 动，4 档：`moderate`, `high`, `freq3`, `freq4`）；
  3. *AlphaPilot*（11 实体门，6 动，4 档）；
  4. *AirSim*（21 实体门，10 动，4 档）。
- **统一平台与参数**（**Table II: QuadA Vehicle, Safety, and Solver Parameters**）：QuadA 名义参数、$r_s=0.20\,\text{m}, r_f=0.085\,\text{m}, r_b=0.15\,\text{m}$、软动力学权重 1000、16 积分区间。
- **六种对比方法及协议差异披露**：`Ours (Free)`、`Ours (Center)`、`Dyn-TOGT (Free)` / `Dyn-TOGT (Center)`（作者源码动态适配，权重 1/单初值）、`Wang`（$N=2, M=4$ 滚动局部规划拼接）、`Dyn-CPC`（修复后等价 $\tau$ 消元、400 区间 16 子步 RK4、7 阶段互补松弛 $10^{-2}\!\to\!10^{-8}$ 收紧、64 子步独立积分复核）。明确声明六方法对比属于系统级基准比较而非同预算消融，结果标签严格区分“通过 / 碰撞 / 未完成 / 积分未通过 / 未找到”。

### B. Single-Gate Traversal Mechanism Analysis

- 选取典型高频横移门或摆动门穿越片段，绘制 **Fig. 4**（门坐标系下穿越点轨迹、相对速度矢量以及随时间变化的中心线净空曲线，明确标注质心过面时刻 $t_i$ 与最小净空时刻 $t^*$）。
- 定量展示 $t^* \neq t_i$ 的时间错位与“顺门运动偏移选点”如何同时降低相对扫掠风险并优化出门转向角。

### C. Full-Course Benchmarking Against State-of-the-Art Planners

- **Table III: Full-Course Planning Times (s) and Task Outcomes Across 22 Dynamic Conditions**（完整列出 22 行 × 6 方法，粗体标注该行有效无碰撞最短时间，括号保留失败候选时间及具体失败类型）：
  - **通过率对比**：`Ours (Free)` **22/22**、`Ours (Center)` **22/22**、`Dyn-TOGT (Center)` 19/22（3 碰撞）、`Dyn-CPC` 13/22（5 碰撞、3 未找到、1 积分未通过）、`Wang` 12/22（8 碰撞、2 未完成）、`Dyn-TOGT (Free)` 11/22（11 碰撞）。
  - **飞行时间对比**：`Ours (Free)` 在 **19/22** 个条件中取得最短有效飞行时间。
  - **深度案例剖析（Split-S 四档权衡）**：Split-S 前三档 `Dyn-CPC` 凭借 0.30 m 航点邻域与硬动力学离散最优控制比 `Ours (Free)` 快 0.090–0.128 s，但在最高频 `freq4` 档发生门框碰撞，而 `Ours (Free)` 以 6.016 s 安全通过；`Dyn-TOGT (Free)` 在高频下因缺乏过面前后连续净空约束导致一半条件发生碰撞，印证了连续时间分离的必要性。

### D. Systematic Ablation Studies（两大已完备配对消融 + 1 项可选待补）

1. **门内选点自由度消融（`Ours (Free)` vs. `Ours (Center)`）**：
   - 同初值、同目标、同验收配对对比：**22/22** 条件自由点全部更快，节省 **2.99%–10.87%**；排除基础赛道第 7 导航航点后，在 **12/12** 个纯实体门经典布局条件中自由点同样全部更快，中位节省约 **5.18%**（配 **Fig. 5** 按赛道分组的配对时间差与相对节省图）。
   - 同时如实报告候选级通过数：`Ours (Free)` 176/176，`Ours (Center)` 172/176（4 个 AirSim 中心初值有限求解失败），说明开放门内自由度在缩短飞行时间的同时还缓解了高频门强制过中心的数值刚性。
2. **节点高阶导数自由度消融（Explicit PVAJ vs. `Ours (Pos-Only)` $C^6$ MINCO）**：
   - 基于 2026-11-22 完整 22 条件（352 候选）配对实验（**Table IV: Ablation on Explicit Knot PVAJ vs. Position-Only $C^6$ MINCO**）：
     - `Ours (Pos-Only)` 自由点与中心各 21/22 通过（候选 154/176 与 150/176；AirSim `freq3` 自由点未找到，`freq4` 中心未找到）；
     - 在双方均有合格解的全部条件下，仅位置轨迹均显著慢于 PVAJ 版本，飞行时间中位增幅分别为 **+17.36%**（自由点）与 **+22.80%**（中心），直接证明显式保留节点 VAJ 导数自由度对动态门时间最优飞行的关键作用。
3. *（待补可选实验）* **约束收紧量 $\delta_r$ 敏感性消融**：固定复查阈值 $r=0.285\,\text{m}$，在典型困难条件上扫描 $\delta_r \in \{0, 10^{-4}, 10^{-3}, 5\times 10^{-3}\}\,\text{m}$，对比外层交换轮数、合格率与飞行时间损失，验证理论与效率之间的权衡。

### E. Computational Performance and Convergence Behavior

- **Table V: Planning Wall-Clock Time (s) and Parallel Budgets Across Six Methods**：完整报告 22 条件计算墙钟（`Ours (Free)` 57.9–1539.5 s，`Ours (Center)` 15.0–2067.8 s，`Dyn-TOGT` 0.11–0.48 s，`Wang` 2.20–9.19 s，`Dyn-CPC` 259.0–17768.3 s），明确区分 8 配置并行墙钟与单初值方法，避免错误归因复杂度。
- **Fig. 6（仿照 Zhang Fig. 4–5 与 Gao Fig. 6–7）**：选取代表性困难赛道（如 `a75_f6` 与 `airsim_freq4`），绘制外层交换轮次上的(a) 累计新增约束数、(b) 连续时间最大几何违反量下降曲线、(c) 目标函数值变化，以及(d) 有限 NLP 求解与连续分离的耗时占比分解，直观呈现求解器仅需少量外层交换轮次即可消除连续违反且计算瓶颈主要集中在高维有限 NLP 求解。

---

## Section VII. Limitations and Practical Deployment Considerations（约 0.8 页，仿照 Shao VI + Zhang VIII）

单设一节坦诚界定本文工作的适用边界与工程落地路径：

1. **离线计算代价**：在 21 门高频 AirSim 赛道上，本文规划器仍为分钟级离线求解（约 15–26 分钟并行墙钟），适用于已知赛道的赛前参考轨迹生成，尚不具备在线毫秒级重规划能力；未来可结合实时局部约化或热启动滚动框架加速。
2. **数值复查与形式化证书的边界**：当前连续分离基于双精度浮点 Bernstein 包络并设细分预算上限，可提供高强度数值几何证据，但在数学意义上不等同于基于严格向外舍入区间算术（如 `python-flint` / arb）的连续安全证书。
3. **名义软动力学与实飞闭环差距**：规划阶段采用忽略气动阻力的名义平坦模型与 16 区间软动力学惩罚；实际真机或高保真仿真执行还需处理气动拖曳、电机响应延迟与跟踪误差（未来可参考 Gao 2026 引入鲁棒管/状态不确定性 backoff 与在线 MPC 协同）。
4. **已知运动假设**：当前假设各动态门运动规律随比赛时钟精确已知；面对未知或含噪的门运动，需结合在线运动估计与事件触发重规划。

---

## Section VIII. Conclusion（约 0.5 页）

总结本文如何通过“时空耦合 SIP 建模 + 显式节点 PVAJ/Bernstein 稀疏有限子问题 + 右端收紧连续时间分离”，在动态门四旋翼时间最优飞行中同时实现 **22/22** 全通过与 **19/22** 最短有效飞行时间，并重申门内自由选点与节点高阶导数自由度的双重量化收益，以面向实时化、鲁棒化与真机闭环验证的未来方向收尾。

---

## Appendices（仿照 Zhang Appendix A–B & Gao Appendix）

- **Appendix A: Proofs of Lemma 1, Theorem 1, and Theorem 2**
  - 给出目标子水平集上多项式导数界、净空 Lipschitz 常数 $L_d$ 构造及右端收紧交换法有限终止的完整数学证明。
- **Appendix B: Track Layouts and Dynamic Gate Motion Parameters**
  - 完整列出 4 种布局的门几何尺寸、位置、法向、运动类型（平移/摆动）、名义幅度、名义频率与初相（仿照 Gao Table II–V，保障完全可复现）。

---

## 图表总清单（规划与数据映射）

| 编号            | 对应章节       | 类型与参照风格                                   | 核心内容                                                                         | 数据/素材来源                                             |
| ------------- | ---------- | ----------------------------------------- | ---------------------------------------------------------------------------- | --------------------------------------------------- |
| **Fig. 1**    | I, III     | 全景与机理图（仿 Gao Fig. 1 / Shao Fig. 1）        | (a) 动态门“过面但仍相交”三时刻截面；(b) 时空耦合与选点自由度；(c) 活性区间与 Bernstein 包围盒；(d) 收紧交换迭代闭环     | 概念手绘 + 典型轨迹渲染                                       |
| **Fig. 2**    | IV         | 算法流水线图（仿 Shao Fig. 2 / Gao Fig. 3）        | 稀疏 PVAJ/Bernstein 有限 NLP $\to$ 法向求根与 Bernstein 分离 $\to$ 随段时长更新的时间索引与原始-对偶热启动 | 方法结构图                                               |
| **Fig. 3**    | VI-A       | 赛道布局图（仿 Gao Fig. 4）                       | Base 7-Node、Split-S、AlphaPilot、AirSim 四赛道三维/俯视布局及动态门运动方向标注                   | `robust_ipopt_full_20261117/source_inputs`          |
| **Fig. 4**    | VI-B       | 单门机理时序图                                   | 单门横移/摆动穿越的门内轨迹、相对速度与净空曲线，标出 $t_i$ 与 $t^*$                                    | 单条冻结轨迹原始记录                                          |
| **Fig. 5**    | VI-D       | 配对收益图                                     | 22 条件 `Ours (Free)` vs. `Ours (Center)` 时间节省百分比与绝对差值（按四赛道分组）                 | `robust_ipopt_full_20261117/FINAL_SUMMARY.json`     |
| **Fig. 6**    | VI-E       | 求解动态与耗时剖面（仿 Zhang Fig. 4 / Gao Fig. 7）    | 代表性困难条件的外层新增约束数、最大几何违反量、目标值演化及耗时占比                                           | `robust_ipopt_full_20261117/results` 逐轮日志           |
| **Table I**   | I-B        | 符号表（仿 Gao Table I）                        | 全文核心数学符号与首次出现位置                                                              | 正文公式核对                                              |
| **Table II**  | VI-A       | 系统与求解器参数表（仿 Shao Table II / Gao Table IV） | QuadA 参数、安全阈值、IPOPT 与分离器预算                                                   | `robust_ipopt_full_20261117/PROTOCOL.md`            |
| **Table III** | VI-C       | 主基准对比表                                    | 22 条件 × 6 方法飞行时间与失败标签（粗体标注有效最短）                                              | `six_methods_cpc_repaired_20261117/PLANNING.md`     |
| **Table IV**  | VI-D       | 轨迹参数化消融表（仿 Shao Table IV）                 | 22 条件 Explicit PVAJ vs. `Ours (Pos-Only)` $C^6$ MINCO 飞行时间与通过率               | `minco_position_ablation_20261122/PLANNING.md`      |
| **Table V**   | VI-E       | 计算墙钟表                                     | 22 条件 × 6 方法墙钟耗时与并行/初值预算说明                                                   | `six_methods_cpc_repaired_20261117/COMPUTE_TIME.md` |
| **Alg. 1–2**  | IV-C, IV-D | 伪代码框（仿 Zhang Alg. 1–3 / Gao Alg. 4–6）     | Alg. 1 连续时间分离器；Alg. 2 右端收紧交换 SIP 主循环                                         | `numpy_passage.hpp`, `sip_planner.cpp`              |
