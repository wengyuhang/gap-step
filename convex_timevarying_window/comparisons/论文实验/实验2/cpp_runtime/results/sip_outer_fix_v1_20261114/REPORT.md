# C++ SIP五项外层修复验证（2026-11-14）

源码同步cpp_runtime与IPOPT的c++实现（原生minco版）。当前有限半径0.63094472，复查0.62094472，真实原生MINCO入口已从新配置重建点映射和所有边框核。旧归档数据未改。

- 两份目录的sip_planner、restored_inputs、状态单元及结果检查目标构建通过。
- 自包含包CTest 3/3通过：自由/中心核与梯度、外层策略。
- 主目录sip_outer_unit通过；sip_result_unit对两条真实输出验证成功状态、incumbent最优保持、每窗最多一新点、最终新候选接受和新约束乘子归零。

|真实测试|完整轨迹含返航/s|计算/s|有限状态|
|---|---|---|---|
|static_center|17.092255|9.560|Solve_Succeeded|
|static_ours|16.434977|134.699|Maximum_Iterations_Exceeded / Solve_Succeeded / Solve_Succeeded|

自由点第一轮Maximum_Iterations_Exceeded，第二轮dual_source_status明确为该失败状态，warm_start_used=false，源码与策略单元确保lam_x0/lam_g0均为零。第二轮保存较快有限incumbent，第三轮略慢但完整复查通过后立即接受。结果中的finite_incumbent仍保存第二轮，不冒充可飞解。

几何违反回退覆盖由确定性状态测试验证：有incumbent的前两次允许重建重试，第三次连续违反终止，正常几何轮清零计数。实际静态例未触发此路径。

非成功状态本身的乘子不复用；显式回退若使用乘子，只能来自历史Solve_Succeeded的incumbent。日志区分dual_source_status。每窗最严重指复查器已报告、且尚未添加的witness中距离最小者；不是声称求得连续域全局最坏时刻。

这些是名义规划与原浮点全窗复查，不是新Gazebo实飞。没有在本轮重跑完整动态矩阵。

最终退出逻辑复核：显式incumbent回退后即使本轮未新增witness，也必须重建并再求解，不能被no_new_witness提前终止；仅真正退出时设置失败termination_reason。真实静态运行对应源码保留于validation_source_snapshot及SOURCE_SHA256.json；这两处退出分支修正后的当前源码另记CURRENT_SOURCE_SHA256.json。
