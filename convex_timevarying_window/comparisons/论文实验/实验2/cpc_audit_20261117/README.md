# 动态CPC：原作者方程的C++适配

来源：Foehn, Romero, Scaramuzza, *Time-Optimal Planning for Quadrotor Waypoint Flight*, Science Robotics 2021。公开实现https://github.com/uzh-rpg/rpg_time_optimal，当前vendor固定提交f5541b6d9d3dee563e01a64ec1994b8ed6c0076c；GPL-3.0许可证随附。

本次将作者变量排列、初始化、RK4、CPC约束和原IPOPT设置移植到C++ CasADi。此前仓库只有Python动态适配入口，本次不通过Python生成或运行规划器。仅把静态航点替换成已知运动c_j(kT/N)，并使用共同QuadA及任务边界。原0.3m邻域和0.01互补放宽并不能保证实际最近中心距离<=0.3m；原始距离逐项记录。

- planner.cpp：完整单次CPC，400区间、每区间四个RK4小步、原单初值；目标仅总时间。
- model.hpp：原刚体/四元数ODE、物理电机序、动态窗口函数及独立数值积分。
- model_test.cpp、MODEL_TEST.json：作者四元数/电机映射和RK4核对；最大ODE差8.6e-14、积分差4.5e-16、运动位置/导数差1.8e-15。
- audit.cpp：原生参考<=1ms采样、STL距离、开口过平面括区间根、门序及终点，另外记录四倍细积分累积漂移。不是连续安全证书。
- batch.cpp：单协调进程，静态先单次运行，其余21条件最多7并行。
- report.cpp：汇总六方法竞速及计算时间两个表，只有有效结果参与每行最短时间比较。

正式设置及所有场景适配见PROTOCOL.md。静态维度15221变量/22048约束，与原400节点七进度问题对应；只移植方程不声称作者官方动态实现。零阻力、QuadA0.85kg、0.15m实体球；不添加本文SIP、外部优化准入、额外候选或安全过滤。

构建目标在../cpp_runtime：dynamic_cpc_planner、dynamic_cpc_model_test、dynamic_cpc_audit。输入引用../robust_ipopt_full_20261117/source_inputs，不复制修改场景。结果在results；当前完成状态应同时查results/STATUS.json、真实进程、各process.log/result.json。

总表位于../six_methods_planning_20261117/{PLANNING.md,COMPUTE_TIME.md,SUMMARY.json}。未完成项必须保留“待完成”，不写成失败。计算表每列明确不同求解预算和复查口径，不能把墙钟差异单独解释为算法复杂度。
