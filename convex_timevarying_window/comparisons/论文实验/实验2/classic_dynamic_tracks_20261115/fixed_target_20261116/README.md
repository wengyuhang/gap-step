# 固定赛道统一初始化实验

2026-11-16用户要求赛道在确定之后直接求解，并固定初始化和启动规则，用于多条赛道。当前目录采用Split-S 7窗、AlphaPilot 11窗、AirSim 21窗各中/高两档原目标文件。

初始化和SIP每次均读取原目标场景。八组原逐段时间各进行一次当前目标下的动态MINCO中心时间优化，然后将实际PVAJ作为自由/中心对应种子的共同初值。每个方法各解8组，统一数值启动设置。自由点的选点初始为中心，严格中心版没有选点变量。

机器协议位于config/protocol.json。PROTOCOL.md记录初始化、求解上限、数值设置、验收和选优规则。FREEZE.json在整批启动前保存源码、配置、二进制、目标场景和输入的SHA-256。

原目标场景的静止门、平移门和摆动门均有逐窗记录。运动参数直接从原场景导出，见results/motion_audit/MOTION_TABLE.md和MOTION_AUDIT.json。移动门按各自A、f、φ、方向/支点和类型求值。

## 构建和运行

在当前目录运行以下命令。CASADI_ROOT按本机已安装位置设置。

```sh
cmake -S . -B /tmp/fixed_target_build_20261116 -DCMAKE_BUILD_TYPE=Release -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build /tmp/fixed_target_build_20261116 --target fixed_target_run fixed_seed fixed_motion_report mesh_check -j 3
```

规划器和动态MINCO时间初始化器使用本批冻结的原C++二进制，位于/tmp/classic_course_build_20261115。新输入准备和调度入口如下，NEW_RESULTS必须是尚不存在的目录。

```text
fixed_target_run ORIGINAL_BIN FIXED_BIN ORIGINAL_INPUTS CONFIG_DIRECTORY NEW_RESULTS
```

本批ORIGINAL_INPUTS是上级results/inputs_v1_20261115，CONFIG_DIRECTORY是本目录config，结果位于results/six_courses_v1。逐条件SUMMARY.json、逐候选result.json、finite_input.json、CANDIDATE.json与实体复查mesh_assessment.json完整保留。

C++ report.cpp从完成的SUMMARY.json生成REPORT.md和STATISTICS.json，可单独编译。

```sh
g++ -std=c++17 -O2 report.cpp -I ../../cpp_runtime/include -o /tmp/fixed_target_report
/tmp/fixed_target_report NEW_RESULTS
```

## 当前任务中的历史记录

此前solver_repair_20261115中的幅度延续轨迹按用户更正记为历史开发结果。当前启动核对记录位于results/startup_probe，包含原始初值和单个直接目标MINCO种子的诊断。正式六条件统一使用本目录已冻结八初值规则。

主模型使用原生完整PVAJ MINCO、TOGT正时间映射、时间/动力学软约束权重1/1000、原33节点/段积分、规划安全球0.30m和内缩0.3001m。结果分列轨迹时间、实际并行计算墙钟、候选失败及安全/任务复查。当前任务范围是规划。
