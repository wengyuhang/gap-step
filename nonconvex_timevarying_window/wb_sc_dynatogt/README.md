# WBSC-DynaTOGT：姿态机体约束版 SC-DynaTOGT

问题定义仍使用 [PROBLEM_DEFINITION.md](../PROBLEM_DEFINITION.md)。

## 核心修改

本算法保留原 SC 参数化、正时间映射、MINCO 和动力学软罚，并将决策变量扩展为 \([K,D,Y]\)：时间和穿越点会改变加速度及 roll/pitch，\(Y\) 直接改变 yaw。相对原规划只增加一个几何条件：

\[
\bigl(p(t_i)+R(t_i)\mathcal B\bigr)\cap\Pi_i(t_i)
\subseteq\Omega_i^\varepsilon(t_i).
\]

其中 \(\mathcal B\) 是无人机长方体，\(R(t_i)\) 由当前迭代的 MINCO 轨迹和 yaw 经微分平坦性恢复。该条件在求解器的每次迭代中计算并反向影响 \(K,D,Y\)，不是轨迹规划完成后的事后检查。

窗口仍使用 SC-DynaTOGT 的 \(c(t),E(t),s(t)\) 变化规律，不新增时变 SDF。

## 无约束处理

姿态相关安全中心集合随 \(D,K\) 改变，对非凸窗口还可能变空或不连通，所以一般不能像原论文的固定门映射那样，通过一个全局光滑满射精确消除。

姿态相关安全中心集合随 \(K,D,Y\) 改变，对非凸窗口还可能变空或不连通，所以一般不能通过一个固定 SC 映射精确消除。默认实现保留原动力学软罚，但使用 SLSQP 在规划中直接施加姿态长方体硬约束。可选软罚模式可以继续使用 L-BFGS，但不提供硬安全保证。

详细推导见 [POSE_CONSTRAINT_FORMULATION.md](POSE_CONSTRAINT_FORMULATION.md)，简版见 [ALGORITHM.md](ALGORITHM.md)，修改范围与论文归因见 [MODIFICATIONS_FROM_SC_DYNATOGT.md](MODIFICATIONS_FROM_SC_DYNATOGT.md)。

## 长方体参数

默认长宽高为 `0.53008 × 0.25522 × 0.11780 m`，最大外包半径为 `0.300 m`；另保留 `0.005 m` 几何误差和 `0.010 m` 数值裕度。参数均写入结果 manifest，可替换为真实 CAD 包围盒。

## 运行

```bash
python -m nonconvex_timevarying_window.wb_sc_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.wb_sc_dynatogt.experiments --suite formal
pytest -q nonconvex_timevarying_window/wb_sc_dynatogt/tests
```

默认对比原 `0.315 m` 球形内缩、中心点乐观模型和姿态联合规划模型。原算法与新算法使用相同场景和种子，原算法保留自己的安全裕度。

## 安全声明

默认求解器在规划中施加离散机体边界点硬约束，收敛后再检查完整投影凸包。只有优化收敛且几何检查通过，结果才记为安全成功；否则明确返回失败。当前模型检查规定穿越时刻，不声称给出连续时间门框碰撞证书。
