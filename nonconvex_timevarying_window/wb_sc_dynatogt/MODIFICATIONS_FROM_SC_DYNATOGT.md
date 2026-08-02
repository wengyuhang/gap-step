# 相对 SC-DynaTOGT 的修改

## 保留

- 原时间与穿越点变量 \((D,K)\)；
- Chang 边界重采样和非凸窗口 SC 参数化；
- \(c(t),E(t),s(t)\) 时变窗口规律；
- 正时间映射和 degree-7 MINCO；
- 原时间、snap-energy 目标；
- 原速度、总推力、机体角速度和单旋翼推力软罚。

## 唯一新增几何约束

原算法用固定球形内缩保证中心点安全。新算法直接使用物理开口，并增加

\[
\bigl(p(t_i)+R(t_i)\mathcal B\bigr)\cap\Pi_i(t_i)
\subseteq\Omega_i^\varepsilon(t_i),
\]

即由原轨迹恢复出的姿态长方体在穿越时不得碰门框。

为使完整姿态能够响应这个约束，规划器把原论文平坦输出中的 yaw 显式暴露为变量 \(Y\)；roll/pitch 不独立指定，而是由当前 MINCO 轨迹的加速度恢复。姿态长方体约束在每次优化迭代中计算，直接反馈到 \(K,D,Y\)。默认使用 SLSQP 施加该硬约束，原动力学限制仍保留为软罚。

本版不加入独立 roll/pitch 变量、约束延拓或碰撞节点生成。完整公式见 [POSE_CONSTRAINT_FORMULATION.md](POSE_CONSTRAINT_FORMULATION.md)。

## 参考范围

whole-body/SE(3) 狭窄通道建模的动机参考 ICRA 2025 [Whole-Body Control Through Narrow Gaps from Pixels to Action](https://doi.org/10.1109/ICRA55743.2025.11128088)。这里只借鉴“姿态改变机体可通过集合”的观点，没有使用其视觉、强化学习、蒸馏或控制器。

算法主干和软约束处理依据原 TOGT 论文 [Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing](../../复现/论文/2309.06837v3.pdf) 的式 (5)--(15)。
