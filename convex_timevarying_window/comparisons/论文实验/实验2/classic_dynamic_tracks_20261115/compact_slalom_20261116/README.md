# 紧凑交错连弯

2026-11-16新增自建7门受控场景，用于考察中心线横向绕行。场景在任何轨迹成绩产生前冻结，后续完整使用fixed_target_20261116的统一启动和验收规则。

[结果与赛道图](results/REPORT.md)包含正式未找到结果和分开标注的有限解诊断。[PROTOCOL.md](PROTOCOL.md)记录完整场景，生成器为[generate.cpp](generate.cpp)，冻结输入在results/inputs_v1。

## C++入口

CMake目标slalom_inputs生成场景与输入。输入接口为“原六条件输入目录 新输出目录”，新输出目录必须不存在。CasADi路径由CMake变量CASADI_ROOT指定。

正式批次复用上一批C++的fixed_target_run，其参数依次为原规划二进制目录、fixed_target工具目录、本目录results/inputs_v1、本目录config和新批输出目录。入口源码在相邻fixed_target_20261116/run.cpp和prepare_seed.cpp，原求解器在实验2/cpp_runtime/src/sip_planner.cpp。当前结果在results/paired_v1。

独立诊断工具analysis/export_finite.cpp的参数依次为预备输入目录、原候选result.json、scene.json和新诊断输出目录。它从严格有限可行解恢复PVAJ MINCO，不进行新优化，不更改原验收结果。

窗口运动与车辆模型、目标权重、安全内缩、时间映射、PVAJ MINCO、8初值和16路并行设置均与固定协议对应。只规划，全部16候选已结束，正式两方法均0/8通过。
