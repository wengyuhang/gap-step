# 经典竞速赛道动态适配

用户于2026-11-15要求使用经典非大圈门序、增加门数、丰富窗口平移方向及摆动支点。本目录保存Split-S 7窗、AlphaPilot 11窗、AirSim 21窗的两档动态场景和五方法C++比较。生成规则和适配以PROTOCOL.md为准，原始固定版本数据、许可与SHA在sources/。

全部优化、图生成、安全检查和协调为C++。generate.cpp生成可变门数的原生MINCO PVAJ输入，motion_verify.cpp验证面内旋转后的运动区间与安全节点，togt_motion_verify.cpp核验动态TOGT梯度。核心规划器复用cpp_runtime，TOGT为作者TrajSolver动态扩展，Wang保持现有N2/M4独立复现并推广请求门数。

```sh
BIN=/tmp/classic_course_build_20261115
"$BIN/classic_course_inputs" sources/tracks.json FULL7_TEMPLATE VEHICLE_CONFIG NEW_INPUTS
"$BIN/classic_motion_verify" NEW_INPUTS MOTION_CHECK.json
"$BIN/five_method_wang_inputs" NEW_INPUTS NEW_WANG_GRAPHS
"$BIN/five_method_wang_verify" NEW_WANG_GRAPHS NEW_INPUTS WANG_CHECK.json
"$BIN/classic_course_run_ours" "$BIN" NEW_INPUTS NEW_RESULTS
"$BIN/classic_course_run_baselines" "$BIN" NEW_INPUTS NEW_WANG_GRAPHS AUTHOR_PARAMS NEW_RESULTS
"$BIN/classic_course_report" NEW_RESULTS NEW_INPUTS
```

当前输入results/inputs_v1_20261115，原始结果results/classic3x2_v1_20261115。运行真实状态见RUN_STATE.md及各协调状态/进程。计算与完整轨迹时长分别记录；物理碰撞按0.15米仿真机体，规划安全球0.30米，地面单列。原作者含桨外廓未公开。本轮只做名义规划。

正式批次已完成。完整规划/计算时间、碰撞与初值失败见[PLANNING.md](results/classic3x2_v1_20261115/PLANNING.md)，布局见[TRACKS.png](results/classic3x2_v1_20261115/TRACKS.png)。
