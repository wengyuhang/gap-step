# WBSC-DynaTOGT：姿态参与规划的单约束扩展

## 保留的原规划

原 TOGT/SC-DynaTOGT 的 MINCO、时间映射、SC 穿越点映射和动力学软约束保持不变。决策变量扩展为 \(x=[K,D,Y]\)，其中 \(Y\) 生成连续 yaw 轨迹。记原目标为

\[
J_{\mathrm{TOGT}}(K,D,Y)
=T_\Sigma(K)+\widehat I_{\mathcal T(P(D),\psi(Y))}(T(K)),
\]

其中 \(\widehat I\) 保留速度、推力、机体角速度和单旋翼推力等软罚。源码级复现中的 snap-energy 和 `smoothedL1` 也不修改。

## 唯一新增约束

无人机用机体系长方体 \(\mathcal B\) 表示。穿越第 \(i\) 个窗口时，机体姿态由当前优化迭代中的位置/yaw轨迹经微分平坦性恢复：

\[
\mathcal B_i=p(t_i;K,D)+R(t_i;K,D,Y)\mathcal B.
\]

令 \(\Pi_i(t_i)\) 为窗口平面，\(\Omega_i^\varepsilon(t_i)\) 为保留安全裕度后的物理开口。新增条件只有

\[
\boxed{
\bigl(\mathcal B_i\cap\Pi_i(t_i)\bigr)
\subseteq\Omega_i^\varepsilon(t_i).}
\]

它同时包含 roll、pitch 和 yaw 的影响。roll/pitch 随 \(D,K\) 改变，yaw 随 \(Y\) 改变。该约束在每次迭代中计算并反馈给 \(K,D,Y\)，不是规划完成后的检查。窗口变化仍沿用原来的 \(c(t),E(t),s(t)\)，不新增时变 SDF。

## 无约束形式

姿态固定时，可以先对窗口做姿态相关几何腐蚀，再对安全中心集合建立 SC 映射。但联合优化时姿态由轨迹决定，安全集合反过来又决定穿越点；对非凸窗口它还可能变空或断开。因此一般不存在与原论文门映射同样简单、固定且全局精确的变量替换。

若使用无显式约束的近似形式，可定义机体约束违反量 \(v_i(K,D,Y)\ge0\)，并求解

\[
\boxed{
\min_{K,D,Y}\quad
J_{\mathrm{TOGT}}(K,D,Y)+\rho_b\sum_i v_i(K,D,Y)^3.}
\]

源码级实现可继续使用已有 `smoothedL1`。该形式保留无约束 L-BFGS，但软罚不提供硬安全保证。当前默认采用直接非线性硬约束；原动力学限制仍保持软约束。

详细推导、原论文公式对应关系以及为何不能一般性精确消除该约束，见 [POSE_CONSTRAINT_FORMULATION.md](POSE_CONSTRAINT_FORMULATION.md)。
