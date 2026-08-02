# 在原 TOGT 公式上增加姿态机体约束

本文沿用原论文 *Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing* 的符号和式 (5)--(15)。原文见 [2309.06837v3.pdf](../../复现/论文/2309.06837v3.pdf)。

## 1. 原论文的 TOGT 问题

原论文将四旋翼状态和控制量记为

\[
x=[p^\top,q^\top,v^\top,\omega^\top]^\top,
\qquad
u=[f_1,f_2,f_3,f_4]^\top.
\tag{1}
\]

这里 \(p\) 是位置，\(q\) 是姿态四元数，\(v\) 是速度，\(\omega\) 是机体角速度，\(f_1,\ldots,f_4\) 是四个旋翼推力。

原论文式 (5) 为

\[
\begin{aligned}
\min_{x,u,t_f}\quad &t_f,\\
\mathrm{s.t.}\quad
&x(0)=\bar x_0,
\quad x(t_f)=\bar x_f,\\
&\dot x=f(x,u),
\quad h(x,u)\le0,\\
&\exists\ 0<t_1<t_2<\cdots<t_L<t_f,\\
&h_{\mathcal G^i}(p_x(t_i))\le0,
\quad 1\le i\le L.
\end{aligned}
\tag{2}
\]

这组公式的含义是：在满足动力学、推力和角速度等限制的情况下，用最短时间依次穿过 \(L\) 个门。

式 (2e) 只检查无人机位置 \(p_x(t_i)\) 是否在第 \(i\) 个门内。它没有检查带姿态的完整机体是否碰到门框，这正是需要补充的地方。

## 2. 原论文怎样把轨迹分段

原文按照门的顺序，把整条轨迹分成 \(L+1\) 段。门内穿越点和各段时间分别写成

\[
P=[p_1^\top,p_2^\top,\ldots,p_L^\top]^\top,
\qquad
T=[T_1,T_2,\ldots,T_{L+1}]^\top.
\tag{3}
\]

总时间和第 \(i\) 个门的穿越时刻为

\[
T_\Sigma=\sum_{k=1}^{L+1}T_k,
\qquad
t_i=t_{\Sigma i}=\sum_{k=1}^{i}T_k.
\tag{4}
\]

也就是说，\(P\) 决定从门中的哪里通过，\(T\) 决定什么时候通过。

原文式 (6) 定义了给定穿越点 \(P\) 时的可行时间集合：

\[
\begin{aligned}
\mathcal T(P)=\{T\in\mathbb R_{>0}^{L+1}\mid
&\exists x,u:[0,T_\Sigma]\rightarrow\mathbb R^n,\mathbb R^m,\\
&x(0)=\bar x_0,
\quad x(T_\Sigma)=\bar x_f,\\
&\dot x=f(x,u),
\quad h(x,u)\le0,\\
&p_x(t_{\Sigma i})=p_i,
\quad 1\le i\le L\}.
\end{aligned}
\tag{5}
\]

这条公式看起来复杂，但意思很直接：如果 \(T\in\mathcal T(P)\)，就存在一条动力学可行的轨迹，在指定时刻准确经过所有指定穿越点。

原文随后把问题写成

\[
\begin{aligned}
\min_{P,T}\quad
&T_\Sigma+I_{\mathcal T(P)}(T),\\
\mathrm{s.t.}\quad
&h_{\mathcal G^i}(p_i)\le0,
\quad 1\le i\le L.
\end{aligned}
\tag{6}
\]

其中

\[
I_{\mathcal T(P)}(T)=
\begin{cases}
0,&T\in\mathcal T(P),\\
+\infty,&T\notin\mathcal T(P).
\end{cases}
\tag{7}
\]

式 (7) 相当于一个理想可行性开关：动力学可行时不增加代价，不可行时给无穷大代价。

## 3. 原论文的 MINCO、平坦性和软约束

原论文选择四旋翼平坦输出

\[
y(t)=[p(t)^\top,\psi(t)]^\top,
\tag{8}
\]

其中 \(p\) 是位置，\(\psi\) 是 yaw。

原文式 (9)--(11) 将状态、控制和约束写成平坦输出及其导数的函数：

\[
x=\Psi_x(y,\dot y,\ldots,y^{(s-1)})
\triangleq\Psi_x(y^{[s-1]}),
\tag{9}
\]

\[
u=\Psi_u(y,\dot y,\ldots,y^{(s)})
\triangleq\Psi_u(y^{[s]}),
\tag{10}
\]

\[
h(x,u)=h_\Psi(y^{[s]})\le0.
\tag{11}
\]

这三条式子非常关键。MINCO 产生位置和 yaw 多项式后，不需要再积分四旋翼动力学，就能直接恢复姿态、角速度、总推力和各旋翼推力。

姿态本来就在 \(x=\Psi_x(y^{[s-1]})\) 中。因此，把机体碰撞约束写成 \(x(t_i)\) 的函数，就可以让它在规划中影响位置、时间和 yaw。

原论文没有直接求解式 (11) 的硬约束，而是用式 (12) 的三次正部软罚近似理想指示函数：

\[
\widehat I_{\widehat{\mathcal T}(P)}(T)
=\int_0^{T_\Sigma}
\max\!\left[h_\Psi(y^{[s]}(t)),0\right]^3dt,
\tag{12a}
\]

\[
\widehat I_{\widehat{\mathcal T}(P)}(T)
\approx
\sum_{i=1}^{L+1}\sum_{j=0}^{\kappa_i}
\max\!\left[
h_\Psi\!\left(y^{[s]}(t_{i-1}+j\Delta t_i)\right),0
\right]^3\Delta t_i,
\tag{12b}
\]

其中 \(\Delta t_i=T_i/\kappa_i\)。

式 (12) 就是原论文中的动力学软约束。速度、机体角速度、总推力和单旋翼推力一旦越界，就会增加目标函数。

新算法必须保留式 (12)。新增机体碰撞约束不能替代这些动力学软约束。

## 4. 原论文怎样消除门和时间约束

原论文为门构造从无约束变量到门内点的光滑满射。球形门使用原文式 (13)：

\[
g_{\mathcal B}(d)
=p_w+
\left[
\frac{2\delta d}{d^\top d+1}
\right]_3,
\qquad d\in\mathbb R^4.
\tag{13}
\]

无论 \(d\) 取什么值，\(g_{\mathcal B}(d)\) 都位于半径为 \(\delta\) 的球形门内。

凸多边形或多面体门使用原文式 (14)：

\[
g_{\mathcal P}(d)
=o+V
\left[
\frac{[d]^2}{d^\top d}
\right]_v.
\tag{14}
\]

这里 \([d]^2\) 表示逐元素平方，\(o,V\) 是门的重心坐标参数。它的作用同样是让无约束变量自动生成门内点。

本项目的非凸窗口用“圆盘映射 \(B\) + SC 映射 \(\Psi_i\)”完成相同工作：

\[
p_i
=c_i(t_i)+E_i(t_i)s_i(t_i)\Psi_i(B(d_i)).
\tag{15}
\]

式 (15) 同时保留了非凸窗口形状以及 \(c_i(t),E_i(t),s_i(t)\) 的时变规律。

时间正性也通过 \(T=T(K)>0\) 消除。于是原论文最终式 (15) 为

\[
\boxed{
\min_{D,K}\quad
T_\Sigma(K)
+\widehat I_{\widehat{\mathcal T}(P(D))}(T(K)).}
\tag{16}
\]

这就是原 TOGT 的无约束 L-BFGS 形式：门位置和正时间通过变量映射自动满足，动力学限制通过式 (12) 进入软罚目标。

## 5. 为了规划姿态，增加 yaw 变量

原论文的平坦输出式 (8) 已经包含 yaw，但原文最终式 (15) 只明确写出 \(D,K\)，发布代码也使用外部给定 yaw。

要让姿态主动适应窄缝，需要把 yaw 节点显式交给优化器：

\[
\psi_i=2\arctan(Y_i),
\qquad
Y_i\in\mathbb R.
\tag{17}
\]

因此新的决策向量为

\[
\boxed{z=[K,D,Y].}
\tag{18}
\]

\(Y\) 直接改变 yaw。\(D,K\) 改变位置轨迹及其加速度，因此通过原文的 \(x=\Psi_x(y^{[s-1]})\) 改变 roll/pitch。

这里没有把 roll/pitch 设置成脱离动力学的自由角度；它们始终与当前 MINCO 轨迹一致。

## 6. 只增加一个完整机体约束

无人机在机体坐标系中近似为长方体

\[
\mathcal B_q=
\left\{b\in\mathbb R^3:
|b_x|\le l/2,
|b_y|\le w/2,
|b_z|\le h_b/2
\right\}.
\tag{19}
\]

利用原文式 (9) 恢复第 \(i\) 个穿越时刻的状态：

\[
x_i(z)
=\Psi_x\!\left(y^{[s-1]}(t_i;z)\right),
\tag{20}
\]

从 \(x_i\) 中取出位置 \(p_x(t_i;z)\) 和姿态旋转矩阵 \(R(q(t_i;z))\)。世界坐标中的姿态机体为

\[
\mathcal B_i(z)
=p_x(t_i;z)+R(q(t_i;z))\mathcal B_q.
\tag{21}
\]

式 (20)--(21) 表示：每当优化器修改 \(K,D,Y\)，MINCO 轨迹、穿越姿态和长方体位置都会重新计算。

设第 \(i\) 个窗口平面为 \(\Pi_i(t_i)\)，保留安全裕度后的开口为 \(\Omega_i^\varepsilon(t_i)\)。新增约束为

\[
\boxed{
\mathcal B_i(z)\cap\Pi_i(t_i)
\subseteq\Omega_i^\varepsilon(t_i),
\qquad 1\le i\le L.}
\tag{22}
\]

这条式子就是完整的算法修改：当前姿态下与窗口平面相交的机体截面，必须完全位于开口内。

当前实现采用更保守的长方体平面投影，并在长方体 12 条边上取边界点。第 \(m\) 个点在窗口局部坐标中的位置为

\[
q_{im}(z)
=\frac{1}{s_i(t_i)}E_i(t_i)^\top
\left[
p_x(t_i;z)+R(q(t_i;z))b_m-c_i(t_i)
\right].
\tag{23}
\]

记 \(h_{\Omega_i^\varepsilon}(q)\le0\) 表示点 \(q\) 位于带裕度开口内，则数值约束为

\[
\boxed{
h_{\mathcal B}^i(z)
=\max_m h_{\Omega_i^\varepsilon}(q_{im}(z))
\le0.}
\tag{24}
\]

式 (24) 与原论文的门约束 \(h_{\mathcal G^i}(p_i)\le0\) 写法一致，只是检查对象从“无人机中心”换成了“当前姿态下的整个机体边界”。

## 7. 修改后的最终规划公式

在原论文式 (15) 上增加式 (24)，得到

\[
\boxed{
\begin{aligned}
\min_{D,K,Y}\quad
&T_\Sigma(K)
+\widehat I_{\widehat{\mathcal T}(P(D),\psi(Y))}(T(K)),\\
\mathrm{s.t.}\quad
&h_{\mathcal B}^i(D,K,Y)\le0,
\quad 1\le i\le L.
\end{aligned}}
\tag{25}
\]

式 (25) 是推荐的主问题：

- 第一项仍是原论文的总时间；
- 第二项仍是原论文式 (12) 的动力学软约束；
- 唯一新增的几何约束是姿态长方体穿窗条件；
- yaw 由 \(Y\) 直接优化，roll/pitch 通过位置轨迹和时间分配优化。

当前实现使用 SLSQP 求解式 (25)。优化结束后的几何检查只是高精度复核，不是姿态第一次进入规划的位置。

## 8. 为什么这不是“先规划，再恢复姿态”

在求解式 (25) 的每一次迭代中，计算链为

\[
(D,K,Y)
\rightarrow(P,T,\psi)
\rightarrow\text{MINCO系数}
\rightarrow y^{[s]}
\rightarrow\Psi_x
\rightarrow R(q)
\rightarrow h_{\mathcal B}^i.
\tag{26}
\]

如果当前姿态会碰窗，\(h_{\mathcal B}^i>0\)。求解器随后修改 \(D,K,Y\)，下一次迭代会产生新的位置轨迹、加速度、roll/pitch 和 yaw。

例如，机体约束对穿越点变量的导数包含

\[
\frac{\partial h_{\mathcal B}^i}{\partial D}
=\frac{\partial h_{\mathcal B}^i}{\partial p}
\frac{\partial p}{\partial D}
+\frac{\partial h_{\mathcal B}^i}{\partial R}
\left(
\frac{\partial R}{\partial\ddot p}
\frac{\partial\ddot p}{\partial D}
+\frac{\partial R}{\partial p^{(3)}}
\frac{\partial p^{(3)}}{\partial D}
+\frac{\partial R}{\partial p^{(4)}}
\frac{\partial p^{(4)}}{\partial D}
\right).
\tag{27}
\]

式 (27) 的直观含义是：如果碰撞由 roll/pitch 不合适引起，约束仍能通过姿态对加速度的依赖，反向推动优化器修改穿越点。\(K\) 和 \(Y\) 也有对应的梯度链。

## 9. 能否像原论文一样改成精确无约束变量

原论文式 (13)--(14) 能消除门约束，是因为门区域是预先固定的。完整机体的安全中心集合却依赖当前姿态和时间：

\[
\mathcal F_i(R_i,t_i)
=\Omega_i^\varepsilon(t_i)
\ominus\operatorname{proj}_{\Pi_i(t_i)}(R_i\mathcal B_q),
\tag{28}
\]

而

\[
R_i=R_i(D,K,Y),
\qquad t_i=t_i(K).
\tag{29}
\]

式 (28)--(29) 表明安全区域会随着优化变量变化。对非凸窗口，它还可能变空或断成多个区域。因此一般不能预先构造一个固定光滑满射，使任意 \(D,K,Y\) 都自动满足机体约束。

所以，式 (24) 一般不能像原门约束那样被精确变量替换消除。

## 10. 与原论文一致的无显式约束近似

如果希望继续使用原来的无约束 L-BFGS，可以仿照原论文式 (12)，将机体约束也写成三次正部软罚：

\[
\widehat I_{\mathcal B}(D,K,Y)
=\sum_{i=1}^{L}
\max\!\left[h_{\mathcal B}^i(D,K,Y),0\right]^3.
\tag{30}
\]

最终得到

\[
\boxed{
\min_{D,K,Y}\quad
T_\Sigma(K)
+\widehat I_{\widehat{\mathcal T}(P(D),\psi(Y))}(T(K))
+\rho_{\mathcal B}\widehat I_{\mathcal B}(D,K,Y).}
\tag{31}
\]

式 (31) 没有显式约束，并且姿态仍然在每次迭代中参与规划，不是事后检查。

但式 (31) 是软约束：有限 \(\rho_{\mathcal B}\) 下仍可能保留少量碰撞。若要求机体条件必须满足，应使用式 (25)；若优先保留原 L-BFGS 结构，可以使用式 (31)，但最终碰撞检查失败时必须返回失败。

## 11. 是否需要独立优化 roll/pitch

当前式 (25) 已经让 roll/pitch 参与规划，但它们通过原论文的平坦性映射 \(x=\Psi_x(y^{[s-1]})\) 由轨迹加速度产生，而不是独立变量。

若把 \(\phi_i,\theta_i\) 也设为独立变量，必须额外满足推力方向一致性：

\[
\ddot p(t_i)+ge_3
=\lambda_iR(\phi_i,\theta_i,\psi_i)e_3,
\qquad \lambda_i>0.
\tag{32}
\]

式 (32) 保证人为选择的 roll/pitch 与四旋翼实际所需推力方向相同。这样做需要新增姿态、推力大小和动力学一致性等式，并改写一部分 MINCO 参数化。

因此当前先采用式 (25)：yaw 直接优化，roll/pitch 通过 \(D,K\) 间接优化，完整姿态进入机体硬约束。只有当实验表明这组变量仍不足以找到可行姿态时，才需要升级到式 (32) 的显式 roll/pitch 版本。

## 12. 最终计算流程

每次求解器迭代都执行：

1. 由 \(K\) 得到正的段时间和穿越时刻；
2. 由 \(D\) 和 SC 映射得到动态窗口内穿越点；
3. 由 \(Y\) 得到 yaw 节点；
4. MINCO 生成当前的位置和 yaw 轨迹；
5. 用原论文式 (9)--(10) 恢复姿态和动力学量；
6. 用式 (12) 计算原动力学软罚；
7. 用式 (24) 计算姿态长方体硬约束；
8. 求解器同时更新 \(K,D,Y\)。

因此，姿态是规划变量和轨迹变量共同决定的，并在整个优化过程中持续参与约束。
