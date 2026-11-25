# TOGT C++ 解析梯度后端

`native/togt_analytic.cpp` 直接使用发布 TOGT 的 `QuadManifold::computeRobustPenalityCost`
和 `MincoSnap::propagateGrad`。前者在每个积分节点返回目标对位置、速度、加速度、jerk 和
snap 的手写导数；后者解 MINCO 转置伴随系统，把系数偏导回传为航点和段时长梯度。

动态窗口额外在 Python 的 `native_objective.py` 中解析回传两项：航点梯度乘开口映射
Jacobian 得到 D 梯度，航点梯度与窗口点速度的内积得到穿越时刻梯度，再按前缀关系回传
到各段时间及 K。该层只做小规模数组运算，不调用 PyTorch。

发布代码在 `z_B.z()+1` 接近零时进入奇异姿态分支，并保留力矩表达式等历史 `TODO`。
本后端不改写这些分支。完整 `[K,D]` 梯度以中心方向差分验收，当前误差约 `1.4e-10`；
单次七窗目标与梯度评价约 `0.0005 s`。
