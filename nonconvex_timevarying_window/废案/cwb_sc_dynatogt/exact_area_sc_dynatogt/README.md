# Exact-Area Whole-Body SC-DynaTOGT

这是 `nonconvex_timevarying_window/` 下的独立并列方法。它按《基于精确交集面积的分段可微整机穿窗安全惩罚——专家审阅修订版》实现，不修改基础 `sc_dynatogt/`。

方法在窗口的标准正交平面基中构造姿态长方体的完整凸截面 (B_i(t))，并与无洞、无自交、直边非凸窗口 (G_i(t)) 做精确布尔交。交集允许包含多个连通分量：

\[
A_i=\operatorname{Area}(B_i),\qquad
C_i=\sum_\ell\operatorname{Area}(P_{i\ell}),\qquad
p_i=A_i[r_i(1-r_i)]^3,\quad r_i=C_i/A_i.
\]

`geometry.py` 保留所有交集连通分量；`penalty.py` 实现理论链式公式；`solver.py` 真实调用基础 `[K,D]` MINCO/L-BFGS 求解器。当前实验 B 的安全附加项使用中心有限差分回传，基础 TOGT 梯度仍为解析实现；因此这是真实优化运行，但尚不是文档规定的全解析安全梯度终版。

当前只完成实验 B 的一个动态压力反例，使用 canonical E4 五角星边界及平移/旋转/缩放函数。机体为 `0.530 × 0.530 × 0.118 m` 的扁平正方形旋翼包络长方体。世界中心距离要求始终不小于 `0.315 m`，由随接触区间更新的约束与 15,001 点独立验证器检查。Old/Ours 使用同一窗口、机体、闭合起终点以及配对初值集。

\[
d_{\mathrm{world}}(t_i)\ge 0.315\ \mathrm m,
\qquad E(t_i)=0,
\]

正式 CSV、NPZ 和静态图只从 MINCO/L-BFGS 输出的多项式系数重建，不再使用 `stress_case.py` 的手工正弦轨迹作为正式数据。Old 在首次整机碰撞时应立即停止；静态时间序列中的后续 Old 面板只回放求解器原轨迹。

运行：

```bash
pytest -q nonconvex_timevarying_window/exact_area_sc_dynatogt/tests
python -m nonconvex_timevarying_window.exact_area_sc_dynatogt.experiments \
  --suite B \
  --outdir nonconvex_timevarying_window/exact_area_sc_dynatogt/results/experiment_b
```

输出包括 `summary.json`、两条轨迹的 `experiment_b.csv`、可直接复核多项式系数的 `optimized_solutions.npz` 和静态图 `full_planned_timeline.png`。静态图将 3D 全景与完整机体侧视同时排列，并标注门框平移、缩放、旋转和 Old 的精确越界面积。按用户要求不再生成新 GIF。

完整 A–F 协议与当前状态见 [EXPERIMENT_PROTOCOL.md](EXPERIMENT_PROTOCOL.md)。
