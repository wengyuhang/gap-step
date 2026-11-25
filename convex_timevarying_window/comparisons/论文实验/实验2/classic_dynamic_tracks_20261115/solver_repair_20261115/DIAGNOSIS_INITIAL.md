# AirSim 21窗高档“未找到”诊断

2026-11-16。当前结论：**已定位失败阶段，尚未得到通过原标准的修复轨迹**。原正式表保持。只做了规划诊断，没有新实飞。

## 已确认的失败位置

原始本文自由点8个初值和中心8个初值均在第一个有限NLP达到5000次迭代，均未进入连续几何复查与补点。这不是因为补点轮次用完，也不是已证明赛道物理不可行。

原独立初值总时长约317.10～351.96秒，由各段距离/1m/s形成；每个初值都有若干窗口的速度违反当前正向穿越条件，常见第2、18、20窗。初值确实不理想，但改成更快的独立初值、修正初始正向速度、重新生成MINCO时间初值后仍未解决，因此不能把初值称为唯一根因。

有限问题有253/211个轨迹变量，另有4356个动力学软惩罚辅助变量，原自由/中心约束行数4431/4377。高阶状态、相位和窄平滑惩罚共同使数值求解困难是待进一步验证的解释；目前并未因果定位到唯一一项。日志有大步长配极小线搜索、近可行处目标反复上升，最后达到迭代上限、步长错误或恢复失败。不能把这些现象改写成严格不可行证明。

## 已核验与已排查

- 原生MincoPV系数和一阶导数、等价归一化边界公式及其二阶导数通过核对。真实目标在中心停滞点的方向差分误差约1.4e-10；自由点停滞点对步长很敏感，最佳误差约1.35e-4，不能宣称所有目标导数都获得同样精度。
- 两个已检查的停滞点最小推力加速度约3.78～4.60m/s²，北图分母最小约0.48～0.60，未支持“这两个点正处于推力/姿态奇异点”的猜测。
- 在新时钟组合诊断中发现局部MX几何函数的组合Jacobian与差分不符；局部函数展开为SX后误差降至约6.31e-12。此问题没有进入原首轮有限NLP，不能解释原16候选全部失败。该修复目前仅在隔离诊断版本中。
- 试过初值速度/正向速度调整、常数目标标度、L-BFGS记忆、软辅助变量精确消元、显式累计时钟、精确二阶导数、状态尺度、等价自然导数坐标、SQP和另一信赖域求解器。没有一项得到原问题合格轨迹。
- 信赖域对照的中心诊断仍在5000次停止，95.934秒，真实目标从约28.516降到约28.514；这只是一条有限求解诊断，尚未进入完整安全复查，不是新的正式竞速成绩。

## 保持的标准

原生完整PVAJ MINCO、TOGT正时间映射、原时间/动力学权重1/1000、原33节点梯形积分、开口约束、正向条件和规划安全球0.30米均未在正式算法中改变。允许比较数值等价表达，但有限问题未收敛时不补点、不计成功。没有把参考曲线、低梯度停止或仅离散约束满足当成安全轨迹。

原冻结源码/二进制及输入共60项SHA-256复核全部一致；正式结果未覆盖。隔离诊断进程已全部结束或明确中断，中断不计作优化器终态。所有原始日志在results/console_logs，未采用的实验代码保留供追溯。

## 本轮诊断原始终态

这些是开发诊断，不是新增赛道统计，不可以把尝试次数当成功率分母。

|诊断|状态|迭代|有限求解秒|
|---|---|---:|---:|
|arrival_speed1_center_0|Maximum_Iterations_Exceeded|5000|308.616|
|arrival_speed1_ours_0|Maximum_Iterations_Exceeded|5000|335.375|
|arrival_speed6p5_center_7|Maximum_Iterations_Exceeded|5000|296.803|
|arrival_speed6p5_ours_7|Maximum_Iterations_Exceeded|5000|336.489|
|clock_center_0|Maximum_Iterations_Exceeded|5000|257.715|
|clock_damped_warm_center_0|Error_In_Step_Computation|1539|689.018|
|clock_ours_0|Maximum_Iterations_Exceeded|5000|357.227|
|clock_reduced_cold_center_0|Error_In_Step_Computation|4017|1210.647|
|clock_reduced_cold_center_7|Maximum_Iterations_Exceeded|5000|1572.784|
|clock_reduced_cold_ours_0|Maximum_Iterations_Exceeded|5000|1864.422|
|clock_reduced_cold_ours_7|Error_In_Step_Computation|3937|1439.287|
|clock_reduced_warm_center_0|Maximum_Iterations_Exceeded|5000|1511.377|
|clock_reduced_warm_ours_0|Maximum_Iterations_Exceeded|5000|1797.642|
|clock_units_cold_center_0|Error_In_Step_Computation|1416|466.712|
|clock_units_warm_center_0|Error_In_Step_Computation|928|311.647|
|continuation_weight1_center_0|Maximum_Iterations_Exceeded|5000|1812.445|
|continuation_weight1_ours_0|Error_In_Step_Computation|1597|687.325|
|exact_speed1_center_0|Restoration_Failed|489|278.453|
|forward_center_0|Maximum_Iterations_Exceeded|5000|296.797|
|forward_ours_0|Maximum_Iterations_Exceeded|5000|303.301|
|history64_center_0|Maximum_Iterations_Exceeded|5000|660.805|
|history64_ours_0|Maximum_Iterations_Exceeded|5000|682.497|
|lifted_metric_center_0|Error_In_Step_Computation|229|14.243|
|lifted_metric_ours_0|Error_In_Step_Computation|2842|126.539|
|lifted_reduced_native_center_0|Error_In_Step_Computation|205|12.418|
|lifted_reduced_native_ours_0|Error_In_Step_Computation|41|2.891|
|lifted_reduced_symbolic_center_0|Error_In_Step_Computation|4939|1745.243|
|lifted_reduced_symbolic_ours_0|Maximum_Iterations_Exceeded|5000|1926.801|
|lifted_time_center_0|Maximum_Iterations_Exceeded|5000|297.402|
|lifted_time_ours_0|Maximum_Iterations_Exceeded|5000|338.993|
|natural_cold_center_0|Error_In_Step_Computation|696|47.659|
|natural_cold_ours_0|Maximum_Iterations_Exceeded|5000|328.857|
|natural_warm_center_0|Maximum_Iterations_Exceeded|5000|323.430|
|observed_limited_center_0|Search_Direction_Becomes_Too_Small|2237|132.606|
|observed_limited_ours_0|Maximum_Iterations_Exceeded|5000|284.445|
|reduced_limited_center_0|Search_Direction_Becomes_Too_Small|2237|131.501|
|reduced_limited_ours_0|Maximum_Iterations_Exceeded|5000|284.722|
|reduced_v2_speed1_center_0|Error_In_Step_Computation|2827|1084.151|
|reduced_v2_speed1_ours_0|Maximum_Iterations_Exceeded|5000|2090.660|
|reduced_v2_speed6p5_center_7|Maximum_Iterations_Exceeded|5000|1892.544|
|reduced_v2_speed6p5_ours_7|Error_In_Step_Computation|3140|1314.609|
|speed6p5_seed_v2_center_7|Maximum_Iterations_Exceeded|5000|283.971|
|speed6p5_seed_v2_ours_7|Maximum_Iterations_Exceeded|5000|328.013|
|sqp_native_center_0|Maximum_Iterations_Exceeded|5000|144.210|
|sqp_native_ours_0|Maximum_Iterations_Exceeded|5000|208.965|
|symbolic_center_0|Maximum_Iterations_Exceeded|5000|1862.268|
|symbolic_ours_0|Maximum_Iterations_Exceeded|5000|2078.374|
|trust_warm_center_v2_0|iteration limit|5000|95.934|
|unscaled_objective_center_0|Maximum_Iterations_Exceeded|5000|375.604|
|unscaled_objective_ours_0|Maximum_Iterations_Exceeded|5000|312.513|

尚未解决项是：获得一个严格收敛且通过原完整几何/顺序复查的目标高档轨迹。当前不能宣称已修好，也没有证明此目标无解。
