# 算法说明

本方法的优化变量仍为基础 TOGT/MINCO 的 (K,D)。新增安全量只依赖这些变量生成的质心轨迹和微分平坦性姿态，不引入新的规划变量。

每个时刻先将姿态长方体 12 条棱与动态窗口平面求交，得到按逆时针排列的 3–6 边凸截面。窗口平面基满足 (E^TE=I_2)，因此二维坐标直接保持真实长度和面积。随后对截面与无洞非凸直边窗口做精确布尔交，并对所有 Polygon/MultiPolygon 分量分别求面积后相加。

瞬时惩罚为：

\[
p=A[r(1-r)]^3,\qquad r=C/A.
\]

它只惩罚正面积的部分重叠：无交和完全包含均为零，(0<C<A) 时严格为正。`instantaneous_penalty_gradient` 实现稳定拓扑内

\[
\nabla r=\frac{A\nabla C-C\nabla A}{A^2},\qquad
\nabla p=\psi(r)\nabla A+A\psi'(r)\nabla r.
\]

`integrated_penalty_gradient` 对分段积分组合梯度，并显式加入

\[
p(T_\Sigma)\nabla T_\Sigma
\]

终端项。内部正规切换项只在左右函数值连续时抵消。当前实验 B 只使用精确几何值和统一验证器；完整 (K,D) 活动顶点导数接入属于后续 A/F 里程碑，代码和结果均不声称已经完成全局解析优化器。
