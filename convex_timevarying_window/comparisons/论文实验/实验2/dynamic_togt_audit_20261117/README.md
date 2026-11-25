# 动态 TOGT 两方法源码复核与 22 条件重跑

2026-11-17。仅规划；44 项已完成。运行入口为 run.cpp，汇总和静态逐值核对为 audit.cpp；全部 C++。旧本文结果及赛道未修改。

## 实际核对结论

当前正式 CMake target dynamic_togt_planner 指向 togt_aligned/dynamic_togt_from_source/main.cpp，并非 cpp_runtime/src/dynamic_togt_planner.cpp。后者只由 legacy_dynamic_togt_planner 编译。旧版本依赖已优化中心解，但当前正式版默认独立单初值。不能把旧版逻辑当作当前版缺陷。

对照作者 RacePlanner::planTOGT：TrajData初始化各段距离/speedGuess，随后 TrajSolver::solve 一次联合优化；当前版本沿用该流程。MincoSnap只优化内部位置及各段时间，内部VAJ由MINCO求出；未替换成本项目独立PVAJ优化。正时间映射、Rectangle顶点顺序及形状映射、动力学代价、16区间积分、L-BFGS均来自作者代码。停止设置past=3、delta=1e-5、梯度阈值0、maxIterations=0保持。

动态扩展仅把每个内点更新为 c_i(sum T)+R_i q_i，并将窗口速度对所有前置段时间的梯度贡献反向累加。窗口运动函数和相位来自冻结scene，窗口平面固定。圆门是作者Ball映射的二维切片；中心版物理门的选点变量为零维，确保真正强制中心。导航航点也沿用中心版固定中心定义。

安全内缩按共同规划球0.20m（含原1e-4数值内缩），物理碰撞复查球0.15m。七节点场景沿用六实体门加第七导航航点；经典赛道沿用原开放终点。这些是本实验场景适配，不宣称作者原论文使用动态窗口、圆形切片或本赛道。

## 本次修改

补存L-BFGS原始返回码与文字原因。作者TrajSolver原本把所有负返回码折叠为LBFGS_FAILED，旧结果无法区分线搜索失败等原因。该修改不改变目标、梯度、初值或停止行为。本次未发现足以支持重写正式优化算法的错误，因此未擅自调参或增加SIP安全机制。

注意作者权重全部1（snap能量权重0），速度上限60m/s、原工作空间代价仍保留；倾角上限6.28rad超过物理倾角范围，正常状态下不激活。本文当前动力学权重1000，八配置初值，另有连续几何补点；因此这张表是方法配置比较，不能称为同目标同预算的单因素消融。TOGT选择作者配置而非偷偷调到本文配置。

## 验证

- 作者原始TrajSolver头文件/实现SHA与既有AUTHOR_SOURCE.json一致。
- 静态直接链接未经修改作者TrajSolver，与动态版零运动逐项对照初始变量、最终变量、时长、系数、目标；结果见AUDIT.json。
- a75_f6完整时间/选点梯度相对差最大6.90e-6；AirSim最高档最大5.59e-5；中心位置误差小于1.1e-14m。两者零运动目标与梯度差为0。有限差分验证，不称精确数学证明。
- 每个成功优化结果独立调用原mesh_evaluate，采用同一球/三角网格/运动范围/门序/终点判据。碰撞与未完成分开。通过是浮点数值证据，不是严格区间认证；未实飞。

## 文件

results/PLANNING.md：44项完整轨迹时间、求解墙钟、结论和迭代次数；碰撞也保留时间。
COMPARISON.md：引用上一批本文两方法与本次TOGT两方法同场景对照。
results/*/{free,center}/result.json、assessment.json、process.log：原始结果。
GRADIENT_AUDIT.json、GRADIENT_AIRSIM.json、AUDIT.json：数值核对。
static_reference：作者静态参考结果。

输入复用 ../robust_ipopt_full_20261117/source_inputs；未复制大量场景网格，INPUT_SHA256.txt冻结22个scene与vehicle。参数使用静态凸窗口/复现/TOGT-Planner-ICRA2024/parameters/cpc/init中的原YAML。
