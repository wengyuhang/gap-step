# 2026-10-13 无规划墙钟限制：单窗继续升档

本轮两批共9组18趟全部完成并核验，未出现设施重试；自由点7次无碰撞完赛/2次未接纳轨迹，中心9次未接纳轨迹。
两批算法、控制器和场景代码相同，仅递增协调器增加了接续起始档与stop-on参数；各批运行期间源码哈希不变。

首次观察到的失败为：中心4m/4.5Hz和4.5m/4Hz；自由点4m/5.5Hz和7m/4Hz。
自由点前一档4m/5Hz为10.092s，6.5m/4Hz为15.784s，均无碰撞完成。
所有18趟任务区间窗框/地面接触为0，失败均未接纳任何参考、仍在起点悬停。因此找到的是当前求解设置的任务失败点，未测到碰撞点。

在线求解与名义离线诊断max_solve_ms均为null，无墙钟截止。MPC、几何、硬约束、初相及40s完整任务截止保持原设置。
不改变原70次迭代/轮、8轮见证交换、24见证槽、4时间种子及首个收敛可行候选返回规则；不是无限迭代或全局求解。
双方同档各一次；时间包含起点等待、执行和终点刹车。未完成不赋竞速时间。任务截止会关闭工作进程，未返回请求可能因此中断。

|振幅 m|频率 Hz|自由点|中心|实际碰撞（自由/中心）|
|---:|---:|---|---|---|
|4|4.5|完成 9.131s|40s内未接纳轨迹|否/否|
|4|5|完成 10.092s|40s内未接纳轨迹|否/否|
|4|5.5|40s内未接纳轨迹|40s内未接纳轨迹|否/否|
|4.5|4|完成 7.674s|40s内未接纳轨迹|否/否|
|5|4|完成 9.364s|40s内未接纳轨迹|否/否|
|5.5|4|完成 8.718s|40s内未接纳轨迹|否/否|
|6|4|完成 9.387s|40s内未接纳轨迹|否/否|
|6.5|4|完成 15.784s|40s内未接纳轨迹|否/否|
|7|4|40s内未接纳轨迹|40s内未接纳轨迹|否/否|

## 求解及失败证据

|振幅/频率|方法|规划请求|规划最大 ms|MPC最大 ms|无时限名义诊断找到解|主要终止状态|
|---|---|---:|---:|---:|---|---|
|4m/4.5Hz|center|57|1193.27|60.24|False|Positive directional derivative for linesearch: 1082, Iteration limit reached: 7, Inequality constraints incompatible: 26|
|4m/4.5Hz|ours|2|793.06|43.52|True|Positive directional derivative for linesearch: 30, Optimization terminated successfully: 3|
|4m/5Hz|center|56|1029.11|60.47|False|Positive directional derivative for linesearch: 1082, Inequality constraints incompatible: 14, Iteration limit reached: 2|
|4m/5Hz|ours|2|439.99|39.75|False|Positive directional derivative for linesearch: 30, Optimization terminated successfully: 3|
|4m/5.5Hz|center|55|1129.58|60.38|False|Positive directional derivative for linesearch: 1053, Iteration limit reached: 6, Inequality constraints incompatible: 20|
|4m/5.5Hz|ours|42|1468.60|61.22|False|Positive directional derivative for linesearch: 806, Iteration limit reached: 16, Inequality constraints incompatible: 11, Optimization terminated successfully: 12|
|4.5m/4Hz|center|49|1173.62|60.57|False|Positive directional derivative for linesearch: 942, Iteration limit reached: 9, Inequality constraints incompatible: 14, Optimization terminated successfully: 2|
|4.5m/4Hz|ours|1|169.65|44.39|True|Positive directional derivative for linesearch: 5, Optimization terminated successfully: 2|
|5m/4Hz|center|52|1786.02|61.87|False|Positive directional derivative for linesearch: 982, Inequality constraints incompatible: 15, Iteration limit reached: 8|
|5m/4Hz|ours|3|1105.60|46.78|True|Positive directional derivative for linesearch: 48, Iteration limit reached: 2, Optimization terminated successfully: 1|
|5.5m/4Hz|center|55|984.20|60.33|False|Positive directional derivative for linesearch: 1018, Inequality constraints incompatible: 14, Iteration limit reached: 6, Singular matrix E in LSQ subproblem: 1|
|5.5m/4Hz|ours|1|178.75|39.47|True|Positive directional derivative for linesearch: 5, Optimization terminated successfully: 3|
|6m/4Hz|center|56|1403.96|61.22|False|Positive directional derivative for linesearch: 1081, Inequality constraints incompatible: 14, Iteration limit reached: 4|
|6m/4Hz|ours|1|253.96|42.00|True|Positive directional derivative for linesearch: 5, Optimization terminated successfully: 2|
|6.5m/4Hz|center|56|1340.33|60.76|False|Positive directional derivative for linesearch: 1084, Iteration limit reached: 5, Inequality constraints incompatible: 13|
|6.5m/4Hz|ours|8|1246.99|43.49|True|Positive directional derivative for linesearch: 130, Inequality constraints incompatible: 3, Iteration limit reached: 3, Optimization terminated successfully: 3|
|7m/4Hz|center|55|1076.65|61.03|False|Positive directional derivative for linesearch: 1081, Inequality constraints incompatible: 13, Iteration limit reached: 4|
|7m/4Hz|ours|44|1597.01|60.74|False|Positive directional derivative for linesearch: 846, Optimization terminated successfully: 8, Iteration limit reached: 16, Inequality constraints incompatible: 11|

## 解释边界

自由点4m/5.5Hz执行42次规划请求、7m/4Hz执行44次请求；多数候选线搜索失败。
两档分别有12/8条候选优化器报告收敛且有限约束接近满足，但全部未通过局部安全复查，因此没有执行。
这不是跟踪飞出去后漏窗，也不是墙钟超时；仍受既定初值、迭代次数、见证交换次数/槽数及40s任务截止约束。

单次规划最大1786.023ms，MPC最大61.871ms。没有再按100ms判规划失败，也不宣称实时规划达标。
5m/4Hz自由点虽完赛，轨迹接纳时已比reference_start晚0.955s；接纳附近误差0.603m，之后最大误差1.552m。
这是取消求解时限后仍用原150ms规划提前量产生的执行时序风险，本轮仅记录，未改变时刻/暂停窗口/重新计时来掩盖。
6.5m/4Hz自由点第8次请求才接纳，接纳在任务5.698s，名义参考起点5.771s、时长10.755s，完整任务15.784s。

未接纳轨迹意味着本求解器在40s任务中未找到满足收敛、有限动力学约束及局部安全复查的候选，不等于物理不可行。
收敛、离散约束通过和局部复查通过分别记录；不能把收敛候选直接称为可执行安全解。
首次失败只针对本初相/初始化/求解设置，不证明失败随频率或振幅单调，也不是碰撞概率估计。
同档中心未完成时，不计算自由点时间优势；两方零碰撞不能声称碰撞率更低。
旧paired_shooting_v1_20261013有90ms预算，与这次无墙钟预算的结果不混成同版本矩阵。
起始批次按任一方法失败停止路线；自由点仍成功后另开补充批次，从5Hz/5m接续寻找自由点失败，两批未重飞已有条件。
原物理步长和接触采样为1ms；高速运动下零接触是仿真观察，不是连续闭环安全证书。
设施异常数量：0，依据见analysis.json。

## 原始结果

- [extension_unlimited_v1_20261013/a4_f4.5_ours](../extension_unlimited_v1_20261013/a4_f4.5_ours/comparison_audit.json)
- [extension_unlimited_v1_20261013/a4_f4.5_center](../extension_unlimited_v1_20261013/a4_f4.5_center/comparison_audit.json)
- [extension_unlimited_v1_20261013/a4.5_f4_center](../extension_unlimited_v1_20261013/a4.5_f4_center/comparison_audit.json)
- [extension_unlimited_v1_20261013/a4.5_f4_ours](../extension_unlimited_v1_20261013/a4.5_f4_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a4_f5_ours](../extension_free_boundary_v1_20261013/a4_f5_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a4_f5_center](../extension_free_boundary_v1_20261013/a4_f5_center/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a4_f5.5_center](../extension_free_boundary_v1_20261013/a4_f5.5_center/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a4_f5.5_ours](../extension_free_boundary_v1_20261013/a4_f5.5_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a5_f4_ours](../extension_free_boundary_v1_20261013/a5_f4_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a5_f4_center](../extension_free_boundary_v1_20261013/a5_f4_center/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a5.5_f4_center](../extension_free_boundary_v1_20261013/a5.5_f4_center/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a5.5_f4_ours](../extension_free_boundary_v1_20261013/a5.5_f4_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a6_f4_ours](../extension_free_boundary_v1_20261013/a6_f4_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a6_f4_center](../extension_free_boundary_v1_20261013/a6_f4_center/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a6.5_f4_center](../extension_free_boundary_v1_20261013/a6.5_f4_center/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a6.5_f4_ours](../extension_free_boundary_v1_20261013/a6.5_f4_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a7_f4_ours](../extension_free_boundary_v1_20261013/a7_f4_ours/comparison_audit.json)
- [extension_free_boundary_v1_20261013/a7_f4_center](../extension_free_boundary_v1_20261013/a7_f4_center/comparison_audit.json)
