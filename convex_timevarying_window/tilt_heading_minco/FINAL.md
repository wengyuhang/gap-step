# 当前采用的修复流程（2026-11-06）

当前完整修复使用`static_continuation.ContinuationPlanner`，不是只提高迭代上限，也不是新的选点映射或IPOPT分支。

1. 保留原TOGT圆/多边形映射和完整内部PVAJ、八段七窗返航。
2. 用倾斜/航向分解修复固定Y叉乘姿态的水平推力近奇异；图内航向pi/2与旧大倾角航向约定不同，物理参数未改。
3. 直接优化sum(T)，去掉数值上过度压缩的log1p(sum(T))；最优解集合不变。SLSQP上限2000，原容差/硬约束/几何复查不变，必须优化器成功。
4. 静态辅助问题的频率规范为0、有效周期无穷，不允许无效频率改变时间预条件。
5. 静态合格解作为起点，将七窗运动幅度统一按0、.25、.5、.75、1加载；失败时减半幅度步长。只用上一合格解作初值，最终必须在完整目标运动lambda=1收敛且通过原检查。总预算720s、单阶段180s；不是在线滚动。

它修复的是数值求解与初始化，不是降低真实运动幅度、放松安全/动力学、停车或回退中心。原直接四初值版本仍可单独使用，不能与连续求解混成同一版成绩。

验证结果：原40项矩阵中5个自由点未找到条件，当前同一修复流程全部lambda1名义通过，另加最高档也通过。时间含完整七窗返航；6条参考16.406591～17.004319s，总规划40.368766～98.916558s（含所有辅助与建图）。
关键档a25_f3同版自由16.893699s、中心17.553208s，均通过，节省3.757%。另一个高动态中心对照已结束，未找到完整目标合格解，幅度连续分支停在lambda=.875；不能把其低幅度辅助结果算完成，也不构成物理无解证明。

入口：`python -m convex_timevarying_window.comparisons.论文实验.实验2.attitude_repair.run_continuation --level a25_f3 --method ours --outdir <新目录>`。
验证：`python -m convex_timevarying_window.comparisons.论文实验.实验2.attitude_repair.validate --root convex_timevarying_window/comparisons/论文实验/实验2/results/attitude_repair_dev_v1_20261106 --require-complete-free`。
同版原始结果/逐条独立重算：实验2/results/attitude_repair_dev_v1_20261106/REGRESSION.md、regression_validation.json，代码快照validated_source_snapshot。

结果是名义球/STL安全与原节点动力学验收，不是Gazebo/PX4实飞、Arb区间证书或连续动力学严格证明。普通浮点5ms电机诊断最高7.853433N，低于实际8.548580N，但部分超过规划90%节点上限；原节点判据未放宽。
