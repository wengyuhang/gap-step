# SIP-DynaTOGT：完整算法、实现与认证规范

> 本文描述当前代码的实际行为。只有最终状态为 CERTIFIED_FEASIBLE 才可称安全；VIOLATED、UNRESOLVED、NUMERICAL_FAILURE 都不能。

## 1. 范围、问题与术语

目标是在已知名义、连续运动的非凸窗口中，按固定顺序各穿越一次，优化七阶 MINCO 四旋翼平坦输出轨迹。安全不是在采样点成立，而是在

\[
(i,j,b,\tau,u)\in\{\mathrm{MINCO\ 段}\}\times\{\mathrm{窗口}\}\times\{\mathrm{原始边界\ primitive}\}\times[0,1]\times[0,1]
\]

的整个连续集合上成立。这里 \(\tau\) 是一段 MINCO 的归一化时间，\(u\) 是一条原始曲线的参数。

SC 为构造保角映射而用的高密度边界点，只服务 SC 的映射近似；SIP 不把它们视作认证约束，也不将原始曲线替换成由这些点组成的折线。B-spline 的 knot span 分割是原始公式的参数分段，不是折线化。

支持：无洞、无自交闭窗口，指定顺序每窗一次；Line、CircularArc、Bézier、非有理 B-spline；周期平移、RPY 旋转、均匀尺度。范围不包括感知/模型误差、风扰、控制跟踪误差、未知障碍或全局最优性。

- SC-DynaTOGT：已有映射式基线；在有限维内缩域优化，使用 L-BFGS-B 的无显式约束罚函数目标。
- SIP-DynaTOGT：半无限规划方法；有限维决策变量面对无限多个时间/边界约束。
- witness：明确的 (段, tau, 窗口, 原始边界段, u)。违规 witness 是反例；安全证明来自全域区间覆盖，而非有限 witness。
- 原始净距：长方体至真实边界的距离减 \(\delta\)。平方残差写成 \(g=\delta^2-\rho^2\)，安全要求 \(g\le0\)。

## 2. 决策变量、时间和 MINCO 轨迹

给定 \(N\) 个窗口，轨迹有 \(N+1\) 段。沿用 SC 的变量

\[
x=(K_0,\ldots,K_N,D_1,\ldots,D_N),\qquad D_i\in\mathbb R^2.
\]

时间映射为

\[
T(K)=
\begin{cases}
\tfrac12K^2+K+1,&K>0,\\
(\tfrac12K^2-K+1)^{-1},&K\le0 .
\end{cases}
\]

它严格为正；全局时间是各段时长前缀和，第 \(i\) 个窗口穿越位于第 \(i\) 个内部连接点。\(D_i\) 经 SC 的单位圆盘/窗口安全域映射给出穿越点，现有 MINCO 用起点、终点、穿越点和边界状态生成

\[
p_i(s)=\sum_{k=0}^{7}c_{ik}s^k,\quad s\in[0,T_i],\quad \tau=s/T_i\in[0,1].
\]

SIP 不定义第二套轨迹模型：SC 和 SIP 使用同一个 \([K,D]\)、时间映射、七阶系数和平坦性模型。

## 3. 原始窗口、整机安全和动力学

### 3.1 原始曲线与时变窗口

窗口 \(j\) 的局部二维闭边界由 \(q_{jb}(u)\) 构成。实现保留各 primitive 的解析形式：直线用仿射插值；圆弧用中心、半径和 sin/cos；Bézier 用区间 de Casteljau；非有理 B-spline 在每个非零 knot span 内用区间 de Boor。SIPWindow 构造时逐段检查闭合性。

窗口中心、旋转、尺度记为 \(c_j(t),R_j(t),s_j(t)\)，世界边界点是

\[
y_{jb}(t,u)=c_j(t)+R_j(t)[s_j(t)q_{jb,x}(u),s_j(t)q_{jb,y}(u),0]^T.
\]

MotionProfile 对平移、RPY 增量和均匀尺度使用各自幅值/频率/相位的正弦函数；姿态为 \(R_z(\psi)R_y(\theta)R_x(\phi)\)。认证会对正弦、尺度和矩阵乘法重新做区间包含计算。尺度必须持续正；赛道设计还应保证至少有一时段存在可通过的开口。

### 3.2 同一长方体模型和净距

机体半尺寸为

\[
e=(0.26504,0.26504,0.05890)\ {\rm m}.
\]

平坦性给出 \(R_B(t)\)。对边界点 \(y\)，先转至机体系

\[
z=R_B(t)^T[y-p(t)],
\]

再用定向长方体点距公式

\[
\rho^2(z)=\sum_{a\in\{x,y,z\}}\max(|z_a|-e_a,0)^2.
\]

最终物理硬约束是

\[
g_{\rm safe}(t,u)=\delta^2-\rho^2(z(t,u))\le0,\qquad\delta=0.015\ {\rm m}.
\]

\(g_{\rm safe}>0\) 表示净距低于 15 mm；只有距离为 0 才表示物理相交。SC 和 SIP 的最终比较都必须使用这个相同长方体和相同原始曲线复核。

优化阶段可使用 \(\delta_{\rm plan}=\delta+\mathrm{planning\_clearance\_buffer}\)（默认 16 mm）留数值空间；最终认证始终回到原始 \(\delta=15\) mm，规划缓冲绝不等于物理保证。

### 3.3 连续动力学与平坦性

每个 MINCO 段的全部连续时间都要满足下列 residual \(\le0\)：

| 项目 | 检查内容 |
|---|---|
| 平移速度 | \(\|\dot p\|^2-v_{\max}^2\) |
| 比力 | \(\|\ddot p+ge_3\|\) 必须能与零分离 |
| 航向构造 | \(\|b_3\times h\|\) 必须能与零分离 |
| 总推力 | 下/上界 |
| 机体系角速度 | XY 范数和 Z 分量上界 |
| 四个旋翼 | 混控得到的每桨推力上下界 |

默认物理参数：质量 1 kg、\(g=9.8066\)、惯量 diag(0.005,0.005,0.01)、臂长 0.15 m、偏航力矩系数 0.01；默认限制：速度 60 m/s、角速度 10 rad/s、每桨 0.25--5 N、总推力下界 0 N。flatness_floor 为 \(10^{-6}\)，dynamic_guard_fraction 为 \(10^{-3}\)。

由 \(p,\dot p,\ddot p,p^{(3)},p^{(4)}\) 计算比力、\(b_3\)、恒定 yaw 的航向基、姿态和导数、角速度、总推力、力矩，再由混控矩阵得到旋翼推力。若归一化分母区间跨零，认证器只会细分或失败关闭，绝不任选一个姿态。

## 4. SC-DynaTOGT 基线流程

1. 原始边界高密度采样，构造 SC 映射所需多边形近似。
2. 对长方体和固定世界净距构造安全内缩域。修正后的实现按 \(\delta/s(t)\) 换算到窗口局部坐标，而不是仅在 \(t=0\) 内缩再跟随尺度同比放大；前者才保持世界中的固定 \(\delta\)。
3. 将 \(D_i\) 映射到内缩域，MINCO 连接所有穿越点。
4. 最小化总时间与选定碰撞/动力学罚项。
5. L-BFGS-B 停止于连续 past_iterations=32 次相对目标改善小于 function_tolerance=\(10^{-5}\)，或被 max_iterations 预算截断；success 仅代表该数值停止规则。

无显式约束不表示容易收敛：SC 映射、时间映射、MINCO 系数、尖锐罚项和有限迭代会形成强非线性/病态目标。SC 的 L-BFGS-B success 不能证明原始连续安全；只有将输出交给同一 SIP certify 的结果可作为安全结论。

## 5. SIP-DynaTOGT 求解循环

### 5.1 初始化有限约束

solve(problem, config) 使用 SC 初值或调用者的 initial_x。初始 active witness 来自每段的 initial_nodes=(0,0.5,1)；粗分离器用 separator_grid_size（默认 3）的时间、边界节点。它们只用于给有限 NLP 初始约束和加快反例发现，未发现违规没有安全含义。

一个 witness 加入一个有限不等式：安全 witness 为 \(g_{\rm safe}(x;t,u)\le0\)，动力学 witness 为相应 \(g_k(x;t)\le0\)。以类型、段、时间、窗口、边界和参数去重。

### 5.2 内层有限 NLP

SciPy SLSQP 最小化总飞行时间，约束仅为当前 active set。默认 slsqp_max_iterations=250、slsqp_ftol=\(10^{-9}\)。SLSQP 成功只表示有限 witness 问题达到停止准则，既不代表全域安全，也不代表半无限规划已完成。

若 SLSQP 失败，求解器恢复稳定候选而非将异常迭代点当结论；随后仍由认证器决定状态。

### 5.3 外层 exchange

每个候选都执行完整 certify：

1. CERTIFIED_FEASIBLE：立即成功。
2. VIOLATED：按 residual 排序，至多取 max_witnesses_per_iteration=8 个反例加入 active set，再次 SLSQP。
3. UNRESOLVED、NUMERICAL_FAILURE、无新 witness 或达到 max_exchange_iterations：停止且不能声称安全。

当前默认 exchange 预算为 12。数学上可想象无限 exchange 序列；实际程序必须有限预算，预算耗尽绝不转换成成功。当前 solve 没有自动调缓冲、自动提高认证预算或自动多阶段热启动；它们若要加入必须先写成预定义、记录在案的协议，不能针对结果临时手调。

### 5.4 热启动、缓冲与认证预算

- 热启动：下一次 SLSQP 从上一次 \(x\) 开始；这是效率手段，不改物理问题。
- 规划净距缓冲：有限 NLP 用更大的 \(\delta_{\rm plan}\)；最终认证仍用 \(\delta\)。增大它会更保守，也可能使优化不可行。
- 认证预算：precision_bits、max_cells、max_depth、最小宽度；只增加证明计算资源，不会改善轨迹或把碰撞变安全。

## 6. 区间认证器：如何覆盖所有连续点

### 6.1 区间包含而非单调性

区间 \([a,b]\) 代表其中的全部实数。包含扩张 \(F([a,b])=[L,U]\) 满足所有真实函数值都在其中。函数不需要单调：例如跨零的平方按 \([a,b]^2=[0,\max(a^2,b^2)]\) 计算。变量重复会造成依赖性过宽，但只可能引起更多细分或 UNRESOLVED，不能产生错误安全证明。

实现使用 python-flint Arb ball arithmetic。模型中的 binary64 常数先经 float.as_integer_ratio() 转精确有理数，再构造 Arb 区间；所以证明对象是保存模型的确切 binary64 值。加减乘除、开方、三角函数均向外舍入。

### 6.2 cell 的组成和计算

动力学 cell 为 (段 i, tau 区间 [a,b])。安全 cell 为

\[
(i,j,b,[a,b]_\tau,[c,d]_u).
\]

对每个安全 cell，按此顺序计算包含区间：

1. 用 Horner 计算七阶 MINCO 的 0--4 阶导数。
2. 求平坦性区间，得到 \(p,R_B,\omega,f,f_1,\ldots,f_4\)。
3. 由局部时间区间得到全局时间区间和窗口 \(c,R,s\)。
4. 对原始曲线计算 \(q_b([c,d])\)。
5. 算 \(y=c+R[sq,0]^T\)、\(z=R_B^T(y-p)\)。
6. 对 abs、max(.,0)、平方和求区间，得到 \(G=[\underline g,\overline g]\)。

这覆盖的是一个连续“时间小矩形 × 曲线参数小区间”，不是许多离散点。复杂项包括七阶多项式、sin/cos、Euler 旋转、矩阵积、绝对值/截断、范数/归一化、B-spline 和混控，但每个都实现为保守包含扩张。

### 6.3 三分判定、反例和二分

对每个 residual 区间 \(G=[L,U]\)：

- \(U\le0\)：cell 内全部点安全。
- \(L>0\)：cell 内全部点违规，取中心点直接复核为 witness。
- \(L\le0<U\)：不确定。先直接检查端点/中点；若发现 \(g>\mathrm{violation\_tolerance}\)，记录具体反例，否则继续细分。

动力学 cell 只沿时间二分。安全 cell 沿较宽的 \(\tau/u\) 方向二分；平坦性奇异不确定时强制沿时间二分。达到 max_depth、max_cells、min_time_width 或 min_boundary_width 仍不确定即为 UNRESOLVED。

先进行均匀粗查以快速找反例；粗查通过后才运行严格全域覆盖。精度依 precision_bits=(128,256)：128-bit 若能证明/否定即停止；仅 128-bit UNRESOLVED 才以 256-bit 重做。

### 6.4 状态、收敛与正确性

| 状态 | 结论 | 能否报告安全 |
|---|---|---|
| CERTIFIED_FEASIBLE | 所有动力学和所有“段 × 窗口 × 原始边界 span” cell 都有 \(U\le0\) 的有限覆盖 | 可以（仅名义模型） |
| VIOLATED | 有直接反例或 \(L>0\) 的 cell | 不可以 |
| UNRESOLVED | 预算内仍有符号不定 cell | 不可以 |
| NUMERICAL_FAILURE | Arb 缺失、重建/计算失败 | 不可以 |

SLSQP 收敛、exchange 停止、认证成功是三件不同的事。只有最后一项是 SIP 成功。安全正确性来自：初始 cell 覆盖全域；每次二分的闭子 cell 并覆盖父 cell；每个终端 cell 均有 \(U\le0\)，故全域残差均不正。

## 7. 默认配置

| 参数 | 默认值 | 用途 |
|---|---:|---|
| body.half_extents | (0.26504,0.26504,0.05890) m | 长方体半尺寸 |
| clearance | 0.015 m | 最终净距 |
| planning_clearance_buffer | 0.001 m | 仅优化的额外余量 |
| flatness_floor | \(10^{-6}\) | 非奇异下界 |
| dynamic_guard_fraction | \(10^{-3}\) | 优化内部动态保护 |
| initial_speed / minimum_initial_duration | 1.0 m/s / 0.20 s | 初始时长 |
| initial_nodes / separator_grid_size | (0,.5,1) / 3 | 初始约束、粗查 |
| max_exchange_iterations / max_witnesses_per_iteration | 12 / 8 | outer SIP 预算 |
| slsqp_max_iterations / slsqp_ftol | 250 / \(10^{-9}\) | 内层 NLP |
| precision_bits | (128,256) | Arb 精度阶梯 |
| max_cells / max_depth | 200000 / 24 | 认证细分预算 |
| min_time_width / min_boundary_width | \(10^{-7}\) / \(10^{-7}\) | 最小 cell 宽度 |
| violation_tolerance | \(10^{-9}\) | 直接点反例阈值 |

运行前必须冻结这些值并随结果保存。

## 8. 输出、重放、测试与可视化

save_run 保存原始 primitive、窗口运动、物理参数、配置、\([K,D]\)、多项式、时长、active witnesses、exchange 记录及 certificate。verify 从这些模型和系数重新调用 certify，不信任保存的状态字段。

测试覆盖 primitive、运动、长方体距离、平坦性/动力学区间包含、采样漏检碰撞、窄速度/推力峰、姿态奇异和旧中心点安全反例。端到端成功必须能由 verify 得到相同状态。

SC/SIP 比较至少应包含：赛道/轨迹可视化、总飞行时间、墙钟时间、优化迭代、认证 cell 数、最终状态，以及“每时刻对所有原始窗口边界的最短长方体距离”时间图。图必须区分 15 mm 净距阈值和 0 距离物理相交：低于 15 mm 是安全违规，不必然已经相交。

## 9. V5/V7/V8 历史、可复现事实与公平性

复杂闭环对比的冻结物理问题哈希为 efca94b2658a8d1ae598e45907e9f8b65fbbbdb521d0b49d8b7de35b0401ac0e。V5、V7、V8 的物理赛道、原始曲线、窗口运动和飞行器相同；变化只在 SIP 求解设置/初值。

- V5：缓冲 0.001 m、exchange 32、SLSQP 240、128 bit、max_cells=2,000,000、max_depth=26。最后 1,557 个 witness，UNRESOLVED。
- V6：不存在可复现的正式 V6 目录。曾有未编号的中间反例分离运行，不应作为正式实验结果。
- V7：从 V5 热启动，缓冲为 0.005 m，追加中间运行发现的 7 个反例；最后 1,611 个 witness，depth 26 UNRESOLVED；SLSQP 成功、732 次迭代、2349.8196 s。
- V8：从 V7 热启动，缓冲 0.020 m、exchange 16；最后 1,613 个 witness，depth 26 UNRESOLVED；SLSQP 成功、98 次迭代、646.6236 s。
- 最终深认证：没有改 V8 轨迹，只将认证预算提升为 depth 32、4,000,000 cells；实际 depth 27、1,311,838 cells、Arb-128，259.9895 s 后 CERTIFIED_FEASIBLE。这说明先前是证明深度预算不足，而不是 V8 优化本身“解决了”安全。

7 个中间记录都是安全反例，不是安全证明：4 个来自 W5 第 2 条 Bézier/段 4，3 个来自 W0 第 5 条原始边界/段 5。详见 comparisons/sc_sip_fast_closed_loop/results/wide_scrambled_curves_v5/sip_dynatogt/batched_collision_witnesses.json。把它们手工加进 V7 对探索有价值，但不是预先冻结的全自动 benchmark 协议。

SC 的 V5 候选在相同的原始曲线和长方体复核中确有物理相交；将 SC 从 400 次上限再运行 50 次后，L-BFGS-B 达到自身停止条件，仍有相交。这不是通过改赛道/模型强行造成。详细数值和时间线见 comparisons/sc_sip_fast_closed_loop/results/wide_scrambled_certified_final/EXPERIMENT_REPORT.md。

## 10. 未来正式实验的预注册自动策略

以下是建议，尚非当前 solve 自动功能。新比较应先固定场景种子/JSON、初值、两算法预算、允许的热启动来源和 SIP 认证阶梯。推荐对同一候选依次运行：

1. 128 bit，depth 24，200,000 cells；
2. 仅当上一步 UNRESOLVED 时：128 bit，depth 28，2,000,000 cells；
3. 仍 UNRESOLVED 时：256 bit，depth 28，2,000,000 cells。

VIOLATED 时只按既定 batch 大小加入反例；CERTIFIED_FEASIBLE 停止；全部阶梯后仍未解即报告 UNRESOLVED。缓冲应固定，或预先定义列表并完整报告，不能看到某次表现后挑选参数。每次升级和原因均应写入 summary.json。

## 11. 源码索引

| 文件 | 职责 |
|---|---|
| model.py | 类型、默认参数、原始窗口和轨迹数据 |
| solver.py | 初值、SLSQP、outer exchange、记录 |
| certificate.py | 粗查、cell、二分、状态、反例 |
| intervals.py | Arb、曲线/运动/多项式/平坦性/残差包含扩张 |
| constraints.py | 单点 residual 与 witness 直接复核 |
| io.py、verify.py | 保存、加载、独立重放 |
| sc_dynatogt/optimizer.py | SC L-BFGS-B 与停止逻辑 |
| sc_dynatogt/time_mapping.py | 正时长 \(K\mapsto T\) |
| comparisons/sc_sip_fast_closed_loop/ | 冻结场景、协议、结果与图 |

简版导读见 README.md，数学摘要见 ALGORITHM.md，文献关系见 LITERATURE_REVIEW.md。若本文与代码不一致，应以保存的运行配置和代码为准，并同步修订本文。

