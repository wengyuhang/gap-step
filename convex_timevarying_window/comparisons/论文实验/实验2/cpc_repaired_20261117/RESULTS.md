# 动态CPC逐项记录

仅规划，原始单次求解；时间包含到任务终点。未完成不计为有效竞速成绩。轨迹候选满足有限约束与优化器收敛分别列出。

|条件|判定|候选时间(s)|计算墙钟(s，不含复查)|迭代数|求解器状态|最大有限约束违反|加密积分位置漂移(m)|
|---|---|---:|---:|---:|---|---:|---:|
|static|通过|7.434|258.868|524|Solve_Succeeded|3.421e-10|1.628e-07|
|mixed_base|通过|7.503|321.185|509|Solve_Succeeded|9.065e-11|2.362e-07|
|a25_f3|通过|7.414|459.577|769|Solve_Succeeded|7.105e-15|2.014e-07|
|a25_f6|通过|7.385|569.710|967|Solve_Succeeded|1.772e-13|1.671e-07|
|a50_f1|通过|7.584|368.120|617|Solve_Succeeded|1.439e-11|1.326e-07|
|a50_f3|通过|7.357|637.502|1100|Solve_Succeeded|2.772e-11|1.133e-07|
|a50_f6|通过|7.355|1069.569|1909|Solve_Succeeded|5.279e-11|1.197e-07|
|a75_f1|通过|7.676|442.606|736|Solve_Succeeded|9.372e-11|1.996e-07|
|a75_f3|碰撞|7.222|600.774|1029|Solve_Succeeded|3.173e-10|3.054e-07|
|a75_f6|通过|8.145|4268.279|7461|Solve_Succeeded|2.717e-10|5.220e-08|
|splits_moderate|通过|5.960|415.545|644|Solve_Succeeded|2.489e-11|7.003e-08|
|splits_high|通过|5.936|477.046|828|Solve_Succeeded|2.295e-10|1.530e-07|
|splits_freq3|通过|5.945|430.164|748|Solve_Succeeded|2.494e-10|1.846e-07|
|splits_freq4|碰撞|5.966|977.242|1672|Solve_Succeeded|8.298e-12|5.840e-08|
|alphapilot_moderate|碰撞|24.440|1432.698|2002|Solve_Succeeded|7.450e-10|1.306e-05|
|alphapilot_high|碰撞|12.052|7660.801|12028|Solve_Succeeded|1.094e-10|9.516e-07|
|alphapilot_freq3|未找到|38.327|13626.609|10000|Maximum_Iterations_Exceeded|2.009e+00|—|
|alphapilot_freq4|碰撞|30.906|4934.060|7785|Solve_Succeeded|1.366e-10|2.114e-05|
|airsim_moderate|通过|20.208|9557.358|4034|Solve_Succeeded|5.281e-10|2.512e-05|
|airsim_high|未找到|44.259|17768.308|10000|Maximum_Iterations_Exceeded|1.424e+00|—|
|airsim_freq3|未找到|49.268|6654.656|10000|Maximum_Iterations_Exceeded|2.199e-03|—|
|airsim_freq4|碰撞|60.234|5579.027|7436|Solve_Succeeded|3.597e-13|1.856e-02|

原始assessment.json记录每窗真实网格距离、有限开口过平面事件、导航点最近距离、终点误差及RK4缺陷；≤1 ms采样无碰撞不是连续安全证书。0.01互补放宽可让进度清零但仍未进入0.3 m导航邻域，导航点任务据实际距离判定。正在求解的候选不填终值。
