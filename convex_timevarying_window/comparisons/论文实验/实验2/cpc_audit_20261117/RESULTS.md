# 动态CPC逐项记录

仅规划，原始单次求解；时间包含到任务终点。未完成不计为有效竞速成绩。轨迹候选满足有限约束与优化器收敛分别列出。

|条件|判定|候选时间(s)|计算墙钟(s，不含复查)|迭代数|求解器状态|最大有限约束违反|加密积分位置漂移(m)|
|---|---|---:|---:|---:|---|---:|---:|
|static|未完成|7.386|112.972|606|Solve_Succeeded|7.485e-09|5.014e-05|
|mixed_base|未完成|7.454|179.369|769|Solve_Succeeded|3.977e-09|4.071e-05|
|a25_f3|未完成|7.359|1111.442|4148|Solve_Succeeded|4.695e-09|6.800e-06|
|a25_f6|未找到|7.941|3320.842|10000|Maximum_Iterations_Exceeded|5.401e-05|—|
|a50_f1|未完成|7.534|12217.738|1468|Solve_Succeeded|4.062e-09|1.912e-05|
|a50_f3|未完成|7.288|2619.966|5673|Solve_Succeeded|6.032e-09|3.831e-05|
|a50_f6|碰撞|10.470|3191.059|10000|Maximum_Iterations_Exceeded|5.877e-07|8.186e-05|
|a75_f1|未完成|7.626|671.251|2415|Solve_Succeeded|3.938e-09|1.916e-05|
|a75_f3|碰撞|7.157|1902.334|7326|Solve_Succeeded|6.003e-09|3.613e-05|
|a75_f6|碰撞|19.115|1551.712|5665|Infeasible_Problem_Detected|3.047e-12|6.053e-04|
|splits_moderate|碰撞|5.924|133.759|521|Solve_Succeeded|1.244e-09|1.099e-05|
|splits_high|碰撞|5.894|557.344|2301|Solve_Succeeded|3.867e-09|2.506e-05|
|splits_freq3|碰撞|5.918|976.836|4049|Solve_Succeeded|4.214e-10|6.910e-06|
|splits_freq4|碰撞|5.924|1430.899|5678|Solve_Succeeded|1.605e-09|3.362e-05|
|alphapilot_moderate|未完成|27.184|5977.104|5834|Solve_Succeeded|1.235e-07|7.885e+00|
|alphapilot_high|碰撞|30.751|3342.036|9465|Solve_Succeeded|1.299e-07|1.278e+01|
|alphapilot_freq3|未找到|32.229|18130.394|10000|Maximum_Iterations_Exceeded|9.653e-03|—|
|alphapilot_freq4|碰撞|37.368|10896.785|10000|Maximum_Iterations_Exceeded|1.938e-07|1.879e-01|
|airsim_moderate|碰撞|64.301|2207.017|4449|Solve_Succeeded|6.723e-08|3.095e+02|
|airsim_high|待完成|—|—|—|—|—|—|
|airsim_freq3|待完成|—|—|—|—|—|—|
|airsim_freq4|碰撞|86.020|4743.938|10000|Maximum_Iterations_Exceeded|3.072e-07|5.850e+01|

原始assessment.json记录每窗真实网格距离、有限开口过平面事件、导航点最近距离、终点误差及RK4缺陷；≤1 ms采样无碰撞不是连续安全证书。0.01互补放宽可让进度清零但仍未进入0.3 m导航邻域，导航点任务据实际距离判定。正在求解的候选不填终值。
