# 随机方形/圆形动态赛道五方法实验

本目录于2026-11-15建立。先冻结PROTOCOL.md，再用generate.cpp生成3个固定种子、每条低中高三档的9个配对条件。每条7个实体窗、4方3圆；闭合周期样条布门，完整竞速时间包含返回起点末段。每条不同难度保持同一静态布局和随机运动倍率及相位。

规划0.30米安全球，物理碰撞为当前QuadA仿真模型0.15米球，沿用已有五方法动力学及优化协议。全部优化、计算图、几何检查与批处理为C++。新增matrix支持有完整场景与输入的自定义level，核心算法沿用主cpp_runtime。

```sh
BIN=/tmp/random_course_build_20261115
"$BIN/random_course_inputs" FULL7_TEMPLATE VEHICLE_CONFIG NEW_INPUTS
"$BIN/five_method_wang_inputs" NEW_INPUTS NEW_WANG_GRAPHS
"$BIN/five_method_wang_verify" NEW_WANG_GRAPHS NEW_INPUTS GRAPH_CHECK.json
"$BIN/random_course_run_ours" "$BIN" NEW_INPUTS NEW_RESULTS
"$BIN/random_course_run_baselines" "$BIN" NEW_INPUTS NEW_WANG_GRAPHS AUTHOR_PARAMETERS NEW_RESULTS
"$BIN/random_course_report" NEW_RESULTS NEW_INPUTS
```

结果目录独立保存原始候选、源/输入SHA及轨迹/计算时间。初始输入和图预生成成本单列。碰撞、求解未找到、任务未完成、数值证据不足分别报告。地面依此前口径另记。Wang为独立复现及QuadA场景适配，角速范数与TOGT分轴形式差异记录在PROTOCOL.md。
