# 原始初值2：有界继续优化最终结果

这是原高难八初值研究中最慢分支自身的继续优化。没有借用其他原始初值或中心方法的结果。不能把这份结果填回旧八初值表，冒充统一预算的冷启动结果。

完整轨迹时间包括七窗后返航；六次结果均通过原完整安全复查与独立STL、顺序唯一穿窗和端点检查。没有Gazebo实飞。

|尝试|完整时间/s|该次求解/s|有限调用状态|
|---|---|---|---|
|1|32.608058|144.834|Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded|
|2|31.127332|135.806|Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded|
|3|29.438257|94.665|Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded|
|4|28.415796|279.693|Maximum_Iterations_Exceeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded|
|5|27.107377|338.748|Maximum_Iterations_Exceeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded|
|6|25.582053|280.972|Maximum_Iterations_Exceeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded / Solve_Succeeded|

额外墙钟 1301.299 秒（21.69分钟）。该成本不包括此前找到36.041秒起始分支、重放诊断、34.109秒完整入口恢复的成本，不能写成从零得到最终解的计算时间。

原最慢合格分支36.040703秒，完整入口数值恢复后34.109205秒，本次最后25.582053秒。六次有限调用共34个，其中3次达到5000上限；均在同一2319行问题上冷精确Hessian恢复成功后，才继续补点，最后一次有限调用均为Solve_Succeeded。并非34次都首次收敛。

本研究支持具体方向/初始化/有限NLP恢复的修复有效，不提供全局最优或所有初值优质收敛保证。下一次正式稳定性比较仍须用当前同一版本、同一规则独立重验高难8起点。
