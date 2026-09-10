# Convex Time-Varying Window TOGT

该目录与 `nonconvex_timevarying_window/` 并列，保存凸多边形/圆形动态窗口实验。窗口支持周期性的三维平移与 roll/pitch/yaw 三维旋转。

求解直接调用发布版 C++ `QuadManifold` 手写解析梯度和 MINCO 伴随回传，不建立
PyTorch 自动微分图。奇异姿态分支和发布源码中的历史 `TODO` 语义原样保留。目标函数严格
采用发布 `standard_planning.yaml`：时间权重 1、snap/速度/总推力权重 0、机体角速度与
单旋翼推力权重 1、`smoothedL1 mu=0.01`、每段 8–32 个动力学积分区间。安全不进入目标函数。

运行：

```bash
conda run -n wyh python -c \
  'from convex_timevarying_window.native_backend import build_native; build_native(force=True)'
conda run -n wyh python -m convex_timevarying_window.experiment \
  --outdir convex_timevarying_window/results/seven_convex_togt_margin_1p1_body_diameter_20260910
```

输出记录建图时间、TOGT L-BFGS 求解时间、完整动力学检测和外接球安全检测。

## 2026-09-10 1.1 倍外接球直径 margin 结果

当前运行见 [实验报告](results/seven_convex_togt_margin_1p1_body_diameter_20260910/REPORT.md)，赛道图见
[route_overview.png](results/seven_convex_togt_margin_1p1_body_diameter_20260910/figures/route_overview.png)。
每窗 `margin` 都设为验收用外接球直径的 1.1 倍，即 `0.8347300205 m`，并严格沿用发布版各 shape
的尺度规则：Rectangle 的宽高各减 `margin`，即每侧缩 `margin/2`；Ball 半径减
`margin/2`；Pentagon 外接圆半径和 Hexagon 边长减 `margin`。这些规则并不等价于
所有真实边界都统一向内缩一个外接球半径。

TOGT 正常收敛，飞行时间 `18.724959468 s`；建图/初始化 `0.001796 s`，282 次目标/梯度
评价的 L-BFGS 用时 `0.965737 s`。最大 1 ms 步长复核中，单旋翼推力峰值
`5.047529 N > 5 N`，动力学验收失败。七窗最小外接球净空依次为
`+0.031188/-0.096951/+0.291344/-0.301012/+0.220061/+0.005835/-0.309658 m`，
W1/W3/W5/W6 通过，W2/W4/W7 发生实质碰撞。指定穿越时刻的映射点均有足够净空，碰撞发生在相邻时刻；
原目标不含安全代价，单个穿越点的 margin 不会约束整段轨迹相对运动门框的净空。

恰好一倍外接球直径的前一次结果保留在
[`seven_convex_togt_margin_body_diameter_20260910`](results/seven_convex_togt_margin_body_diameter_20260910/REPORT.md)。
将 `margin` 误设为外接球半径的中间运行保留在
[`seven_convex_togt_margin_body_radius_20260910`](results/seven_convex_togt_margin_body_radius_20260910/REPORT.md)，
不作为当前方法结果。

## 2026-09-10 margin=0 历史对照

无 margin 运行见 [实验报告](results/seven_convex_togt_mapping_no_margin_final_20260910/REPORT.md)，赛道图见
[route_overview.png](results/seven_convex_togt_mapping_no_margin_final_20260910/figures/route_overview.png)。
多边形使用发布版 `Polyhedron::toP` 的单位球平方重心映射，圆窗使用 `Ball::toP` 的二维
同构映射。各窗 D 维数为 `4/2/5/2/6/2/4`，全部 `margin=0`。TOGT 正常收敛，飞行时间为
`18.307572084 s`；建图/初始化 `0.001803 s`，376 次目标/梯度评价的 L-BFGS 总计
`1.168737 s`。最大 1 ms 步长复核时，单旋翼推力峰值达到 `5.053699 N`，超过
`5 N` 上限；七个穿越点都位于开口边界附近，外接球对七个门框均发生碰撞。因此该次
结果的动力学约束和安全约束都不满足。TOGT 的 margin 是场景输入，映射本身只保证质点
属于凸域，不保证距边界大于无人机半径。该负结果符合原 TOGT 目标的软约束语义，不能把
优化器收敛等同于硬约束验收成功。

此前 [Python 自动微分运行](results/seven_convex_periodic_20260910/REPORT.md) 的
`141.081920 s` 只作为后端替换前的历史记录，不再是本方法当前结果。
