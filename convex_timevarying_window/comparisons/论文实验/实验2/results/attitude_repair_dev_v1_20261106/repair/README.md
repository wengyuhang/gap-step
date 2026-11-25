# 姿态重建修复：独立开发验证，不覆盖冻结矩阵

2026-11-06。原a25_f3自由seed0已精确复现160次退出：F≈(0.0195,29.8451,0.1268)N，固定Y叉乘模0.0042993，电机约束缩放梯度最大142541，SLSQP综合方向导数+0.010715。

本修复唯一模型变化：以北图最短倾斜旋转将世界+Z对齐推力，再右乘固定航向pi/2，替代固定世界Y投影。与旧hover姿态一致，但大倾角下的航向约定不同，不能描述为仅浮点优化或完全相同姿态模型。
倾斜分解参考本仓TOGT native/audit_dynamic_frame_collisions.py的北图构造；一般多图方法见Mao等ICRA2023 SecIV-B [作者论文](https://geodesic.mae.cornell.edu/files/Trajectory_Planning_for_the_Bidirectional_Quadrotor_as_a_Differentially_Flat_Hybrid_System.pdf)。本版不复现其双向电机/倒飞切换算法。

不加倾角惩罚、阻尼或新的物理限制；平动阻力、惯量、混控、推力与角速度边界、节点、时间目标、选点映射、四初值、180s/初值、SLSQP与严格收敛准入不变，不使用中心回退。
北图在b3=-世界Z处仍有极点，不能宣称全姿态无奇异。分母保护只用于避免无效试探计算崩溃；节点验收显式检查chart margin>1e-6、推力>1e-4N与SO3误差，失效标为attitude_chart_unresolved，不算动力学可行。
返回参考另外做5ms全程姿态域/节点姿态角速度连续性检查，公开节点间电机/角速度数值；不是连续动力学证明，不改原几何验收与独立球/STL检查。

先复测a25_f3自由与中心，两方同样四初值。完整时间含七窗返航，计算成本单列；未收敛候选仍不冒充成功，失败保留。原40项矩阵继续旧版本，绝不回填新成绩。
源代码暂放主工作树results下，是为了不触发仍运行的旧批次“新增Python文件也算源变化”的严格保护；旧批结束后才提升正式方法入口。这里不是新的论文正式矩阵。

测试：wyh单线程`python -m pytest -q -p no:cacheprovider --import-mode=importlib <本目录>/tests`。
验证入口：`python -u -m convex_timevarying_window.comparisons.论文实验.实验2.results.attitude_repair_dev_v1_20261106.repair.run --method ours --outdir <新目录>`，center同理。不要对已完成条件重跑择优。
