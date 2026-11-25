# SIP外层修复后三档C++规划重跑

本轮已结束，其中1个中心初值按用户要求保持未完成。仅名义规划与轨迹复查，未进行Gazebo实飞。

原生PVAJ MINCO、TOGT独立正时间映射，时间/动力学软惩罚权重1/1000。每档双方各8个独立初值、16路单线程并行，随后从本档最佳合格中心开放自由点一次。有限半径0.63094472m，复查0.62094472m。上一轮实际状态与乘子来源均为Solve_Succeeded时才复用乘子，否则全零。

最终接纳沿用本批严格Solve_Succeeded及完整窗口复查；独立STL、顺序和端点再核对，地面另列。非成功但几何复查通过的候选见下文，不能把它们记为碰撞或赛道不可行。

0.75/6有一个中心初值按用户明确要求停止重复计算，记为未完成；其余结果保留。该档不是完整八初值预算统计，计算时间不填作已完成。

时间均含七窗后完整返航。条件是幅度比/频率倍率，不是米/Hz。计算为共同配对启动至本方法所有初值和复查结束的实际墙钟，自由点另包含中心释放，未累加worker时间。表中加入本批公共输入生成成本0.942秒；组间排队和旧调试批不计入单档计算。

|条件|自由点轨迹/s|中心轨迹/s|自由点计算/s|中心计算/s|独立自由合格/已结束|独立中心合格/已结束|状态|
|---|---:|---:|---:|---:|---|---|---|
|a25_f6|16.434|17.019|1058.414|236.342|4/8|7/8|已完成|
|a50_f6|17.360|18.318|2311.567|1194.759|1/8|7/8|已完成|
|a75_f6|22.960|23.498|—|—|2/8|1/7（另1项未完成）|已结束（中心1项未完成）|

## 中心释放与独立初值

|条件|独立自由最好/s|中心释放结果|最终自由点来源|
|---|---:|---|---|
|a25_f6|16.434|16.451|a25_f6_ours_4|
|a50_f6|17.360|17.379|a50_f6_ours_5|
|a75_f6|23.039|22.960|a75_f6_release_8|

## 逐候选结果

|候选|原窗口及独立检查|完整轨迹/s|候选计算/s|有限状态次数|终止原因|
|---|---|---:|---:|---|---|
|a25_f6_center_3|合格|17.347|49.501|Solve_Succeeded:1|verified_solution|
|a25_f6_center_2|合格|17.030|56.136|Solve_Succeeded:1|verified_solution|
|a25_f6_center_4|合格|17.359|75.234|Solve_Succeeded:1|verified_solution|
|a25_f6_center_5|合格|17.019|76.904|Solve_Succeeded:1|verified_solution|
|a25_f6_center_0|合格|17.353|107.630|Solve_Succeeded:1|verified_solution|
|a25_f6_center_6|合格|17.076|111.313|Solve_Succeeded:1|verified_solution|
|a25_f6_center_7|合格|17.112|119.500|Solve_Succeeded:1|verified_solution|
|a25_f6_center_1|未找到合格规划|—|235.209|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a25_f6_ours_7|未找到合格规划|—|300.044|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a25_f6_ours_0|未找到合格规划|—|300.670|Maximum_Iterations_Exceeded:1, Restoration_Failed:1|finite_node_geometry_violation|
|a25_f6_ours_6|未找到合格规划|—|309.506|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a25_f6_ours_1|合格|16.442|404.596|Maximum_Iterations_Exceeded:2, Solve_Succeeded:6|verified_solution|
|a25_f6_ours_3|合格|16.451|472.834|Maximum_Iterations_Exceeded:2, Solve_Succeeded:10|verified_solution|
|a25_f6_ours_2|合格|16.570|502.999|Maximum_Iterations_Exceeded:2, Solve_Succeeded:6|verified_solution|
|a25_f6_ours_4|合格|16.434|745.337|Maximum_Iterations_Exceeded:2, Solve_Succeeded:8|verified_solution|
|a25_f6_ours_5|未找到合格规划|—|982.359|Maximum_Iterations_Exceeded:2|finite_node_geometry_violation|
|a25_f6_release_8|合格|16.451|74.602|Solve_Succeeded:9|verified_solution|
|a50_f6_ours_2|未找到合格规划|—|84.674|Restoration_Failed:1|finite_node_geometry_violation|
|a50_f6_center_2|合格|18.318|139.559|Solve_Succeeded:1|verified_solution|
|a50_f6_center_0|合格|20.292|226.120|Solve_Succeeded:7, Infeasible_Problem_Detected:1, Solved_To_Acceptable_Level:1|verified_solution|
|a50_f6_ours_4|未找到合格规划|—|238.939|Restoration_Failed:1|finite_node_geometry_violation|
|a50_f6_ours_3|未找到合格规划|—|250.868|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a50_f6_center_6|合格|25.093|260.458|Solve_Succeeded:5|verified_solution|
|a50_f6_ours_7|未找到合格规划|—|326.345|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a50_f6_ours_6|未找到合格规划|—|342.052|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a50_f6_ours_1|未找到合格规划|—|401.537|Maximum_Iterations_Exceeded:2|finite_node_geometry_violation|
|a50_f6_center_3|合格|20.211|435.195|Solved_To_Acceptable_Level:2, Solve_Succeeded:4|verified_solution|
|a50_f6_center_5|合格|19.294|493.349|Solve_Succeeded:10, Infeasible_Problem_Detected:1|verified_solution|
|a50_f6_ours_5|合格|17.360|539.987|Solve_Succeeded:9|verified_solution|
|a50_f6_center_1|合格|40.962|735.957|Maximum_Iterations_Exceeded:2, Solve_Succeeded:9, Solved_To_Acceptable_Level:2|verified_solution|
|a50_f6_center_7|合格|22.627|738.547|Maximum_Iterations_Exceeded:1, Solved_To_Acceptable_Level:2, Solve_Succeeded:10|verified_solution|
|a50_f6_center_4|未找到合格规划|—|1193.497|Maximum_Iterations_Exceeded:1, Solved_To_Acceptable_Level:1, Infeasible_Problem_Detected:2, Solve_Succeeded:1, Restoration_Failed:3|finite_node_geometry_violation|
|a50_f6_ours_0|未找到合格规划|—|2212.081|Infeasible_Problem_Detected:2, Maximum_Iterations_Exceeded:2, Solve_Succeeded:1, Solved_To_Acceptable_Level:19|exchange_round_limit|
|a50_f6_release_8|合格|17.379|97.999|Solve_Succeeded:7|verified_solution|
|a75_f6_ours_1|未找到合格规划|—|233.874|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a75_f6_ours_2|未找到合格规划|—|246.079|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a75_f6_center_0|合格|23.498|248.869|Solve_Succeeded:5, Solved_To_Acceptable_Level:2|verified_solution|
|a75_f6_ours_4|未找到合格规划|—|342.726|Maximum_Iterations_Exceeded:1|finite_node_geometry_violation|
|a75_f6_ours_6|未找到合格规划|—|704.711|Maximum_Iterations_Exceeded:2|finite_node_geometry_violation|
|a75_f6_ours_7|未找到合格规划|—|718.257|Maximum_Iterations_Exceeded:2|finite_node_geometry_violation|
|a75_f6_center_3|未找到合格规划|—|862.079|Solve_Succeeded:7, Solved_To_Acceptable_Level:17|exchange_round_limit|
|a75_f6_center_1|未找到合格规划|—|959.689|Solve_Succeeded:4, Maximum_Iterations_Exceeded:4, Solved_To_Acceptable_Level:1|finite_node_geometry_violation|
|a75_f6_center_6|未找到合格规划|—|1151.163|Solve_Succeeded:2, Solved_To_Acceptable_Level:1, Maximum_Iterations_Exceeded:3|finite_node_geometry_violation|
|a75_f6_center_4|未找到合格规划|—|1160.300|Solve_Succeeded:4, Maximum_Iterations_Exceeded:3|finite_node_geometry_violation|
|a75_f6_center_7|未找到合格规划|—|1165.727|Solve_Succeeded:3, Maximum_Iterations_Exceeded:3|finite_node_geometry_violation|
|a75_f6_ours_0|合格|23.039|1166.376|Maximum_Iterations_Exceeded:2, Solve_Succeeded:14, Infeasible_Problem_Detected:2, Solved_To_Acceptable_Level:1|verified_solution|
|a75_f6_ours_3|合格|27.229|1434.821|Maximum_Iterations_Exceeded:3, Infeasible_Problem_Detected:1, Solve_Succeeded:15|verified_solution|
|a75_f6_center_2|未找到合格规划|—|1549.132|Solve_Succeeded:7, Infeasible_Problem_Detected:1, Maximum_Iterations_Exceeded:6, Solved_To_Acceptable_Level:1|finite_node_geometry_violation|
|a75_f6_ours_5|未找到合格规划|—|1635.628|Maximum_Iterations_Exceeded:4, Solve_Succeeded:3, Solved_To_Acceptable_Level:1|finite_node_geometry_violation|
|a75_f6_center_5|未完成（用户停止数值停滞）|—|3596.744||user_stopped_numerical_stagnation|
|a75_f6_release_8|合格|22.960|371.032|Solve_Succeeded:16, Infeasible_Problem_Detected:1|verified_solution|

## 几何已过但求解状态不合格

|候选|轮次|状态|轨迹/s|真实目标|全部有限约束最大违反|
|---|---:|---|---:|---:|---:|
|a50_f6_center_7|11|Solved_To_Acceptable_Level|21.864305|21.864306|0|
|a50_f6_ours_0|7|Solved_To_Acceptable_Level|24.170475|24.170475|7.55e-09|
|a75_f6_center_3|12|Solved_To_Acceptable_Level|23.968826|23.968826|0|

这些项只是原窗口复查通过，不作为本批最终合格解；若动力学软代价很高，也不能据几何安全称其可被x500可靠跟踪。原始逐轮状态全部保留。

政策审计：51条候选记录（含单列的用户中断项），发现0项不变量违反。详见POLICY_AUDIT.json与C++ RESULT_INVARIANTS.log。

v1使用旧半径的多边形中心参数导致中心嵌入不等价，v2按乘子来源而非上一轮状态门控。两批均停止、记录和对应源码保留；最终统计仅使用修正后的v3。重复求解和失败照实计时，没有把有限incumbent当成已验证轨迹。

## 阶段耗时与最终轨迹

0.75/6的中断中心候选实际已耗3596.744秒，按用户要求记为未完成；随后使用已完成的23.498418秒中心解，仅补做一次自由点释放，阶段实际墙钟371.306秒。两阶段之间按用户要求先完成0.25/6；该等待不拼成0.75/6的完整八初值计算时间。

六条最终轨迹均通过原窗口浮点复查、独立STL及顺序/端点复查，地面均无相交记录。这里没有Gazebo闭环执行。

|条件|方法|完整轨迹/s|选中轨迹|
|---|---|---:|---|
|a25_f6|ours|16.434265|[JSON](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime/results/sip_outer_rerun_3pairs_v3_20261114/run/groups/a25_f6/plans/a25_f6_ours/result.json)|
|a25_f6|center|17.019394|[JSON](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime/results/sip_outer_rerun_3pairs_v3_20261114/run/groups/a25_f6/plans/a25_f6_center/result.json)|
|a50_f6|ours|17.360071|[JSON](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime/results/sip_outer_rerun_3pairs_v3_20261114/run/groups/a50_f6/plans/a50_f6_ours/result.json)|
|a50_f6|center|18.317559|[JSON](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime/results/sip_outer_rerun_3pairs_v3_20261114/run/groups/a50_f6/plans/a50_f6_center/result.json)|
|a75_f6|ours|22.960047|[JSON](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime/results/sip_outer_rerun_3pairs_v3_20261114/run/groups/a75_f6/provisional_plans/a75_f6_ours/result.json)|
|a75_f6|center|23.498418|[JSON](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime/results/sip_outer_rerun_3pairs_v3_20261114/run/groups/a75_f6/provisional_plans/a75_f6_center/result.json)|

最终25个合格候选的C++结果不变量检查全部通过；50个已结束候选（25合格、25未找到合格规划）加1个用户停止的未完成项，独立统计审计无违反。CTest3/3通过，主目录门控单元通过；冻结二进制/源/输入、场景和动力学元数据及自包含包校验全部通过。仍有初值未找到合格解，外层修复没有消除有限NLP的收敛问题。
