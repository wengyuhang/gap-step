# 原三赛道扩展四档五方法

2026-11-16完成，12条件、60项名义规划。[完整轨迹时间与计算时间](PLANNING.md)，[逐格数据](PLANNING.json)，[最终审计](FINAL_AUDIT.json)。

Split-S7窗、AlphaPilot11窗、AirSim21窗沿用原固定场景。原moderate/high外新增freq3/freq4，保持high幅度，每窗原频率分别乘1.5/2。每窗自身方向、支点和相位保持。既有终点在末门后5米，本批计时覆盖该完整终点。只规划。

五方法为本文自由/中心、动态TOGT自由/中心与Wang独立复现。本文当前版本含SX局部frame求导修复、16384次连续复查、原目标时间8初值和PVAJ恢复。动力学为软约束，中心不含自由选点变量。原TOGT基线单作者初值，WangN2/M4与4时间初值。方法差异沿原classic协议，特别是Wang角速范数定义与本文/TOGT不完全相同。

C++ prepare.cpp生成场景输入，fixed_target_run执行本文两方法，classic_course_run_baselines执行三个基线，全部实际求解为C++。report.py只读原始JSON整理报告，未参与轨迹生成或优化。

原始结果在results/ours与results/baselines，原始有限求解、失败初值和独立碰撞结果全部保留。FREEZE.json在开始求解前记录统一输入和实际求解器。MOTION_AUDIT.json和WANG_AUDIT.json核对156窗口和156局部图，DATA_AUDIT.json与SIP_RESULT_CHECKS.log核对全部初始化和合格轨迹。
