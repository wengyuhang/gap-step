# TOGT七门布局与QuadA：当前仅做规划

2026-11-14按用户要求切换配置。原x500、1.5米小窗和旧高起伏结果保留；本目录的数据独立求解，不与旧时间混表。Gazebo工作已暂停，当前仅验证与运行规划。

## 数据依据与明确适配

对齐论文 *Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing* 的IV-A七方门实验及QuadA。位置、朝向、起终状态直接读取作者2024快照 `9e25117822b1afca1257a77540e0161ee81dcd75` 的 `resources/racetrack/race_uzh_19g.yaml` 前七门；不再用先前按来向重建的第7门134°朝向，采用作者方门文件中的200°。

论文规定方门边长2.4米、总尺寸缩减0.3米，得到可选边长2.1米。原代码 `rectangle.cpp` 使用 `(width-marginW)/2`，因此**每侧内缩0.15米，不是0.3米**。作者公开方门文件中另有marginW/H=0.8这一配置；本实验明确采用论文IV-A的0.3米设置。[论文IV-A与表I](https://arxiv.org/html/2309.06837v3#S4.SS1)；本机原文件及SHA记录在结果的source_inputs/SOURCES中。

将2、4、6门换为直径2.4米的精确圆，保持相同中心和门平面。其余1、3、5、7门保留2.4×2.4米方形。这是本研究的三圆适配，不称为作者原始七方门实例。

|门|中心位置/m|yaw/°|开口|运动|
|---|---|---:|---|---|
|1|(-1.1,-1.6,3.6)|0|2.4米方形|平移|
|2|(9.2,6.6,1.0)|-20|直径2.4米圆|摆动|
|3|(9.2,-4.0,1.2)|-130|2.4米方形|平移|
|4|(-4.5,-6.0,3.5)|180|直径2.4米圆|摆动|
|5|(-4.5,-6.0,0.8)|0|2.4米方形|平移|
|6|(4.75,-0.9,1.2)|70|直径2.4米圆|摆动|
|7|(-2.8,6.8,1.2)|200|2.4米方形|平移|

所有门roll=0、pitch=-90°，平面固定。起点(-5,4.5,1.2)，作者终点(4.75,-0.9,1.2)，两端速度、加速度、jerk均为零。完整时间包含第7门到作者终点的末段；没有另加返回起点任务。

## 车辆与安全参数

|参数|当前值|来源|
|---|---|---|
|质量|0.85 kg|作者cpc_quad.yaml/表I|
|惯量|diag(0.001,0.001,0.0017) kg·m²|同上|
|臂长|0.15 m，beta=45°|作者YAML|
|力矩系数|0.05|作者YAML|
|单电机物理推力|0～6.879 N|作者YAML|
|规划软约束推力|0.1～6.879 N|作者cpc/init/cpc_planning.yaml|
|机体角速度配置|15、15、3 rad/s|作者QuadA|
|规划角速度|XY范数≤15，Z≤2.99 rad/s|作者规划YAML|
|额外动力学折扣|无，fraction=1.0|不沿用x500的95%|
|名义门内缩|每侧0.15 m|论文0.3米总尺寸缩减|

原TOGT没有给出完整含桨碰撞外廓。连续安全检查采用由每侧门余量定义的0.15米球形代理，**不把它写成作者公布的含桨外接球半径**。原门模型为零长度开口，实体框沿用0.085米管半径作为几何适配。

距离参考严格区分：

- 物理开口边界：名义安全球0.15米。
- 数值有限问题额外0.0001米，有限选点内缩0.1501米；方门可选边长2.0998米、圆盘可选半径1.0499米。该0.1毫米是补点数值间隙，不是额外0.15米安全层。
- 到门框中心线：有限距离0.2351米，最终复查距离0.235米。门框管半径只计算一次。
- 独立网格检查使用场景中的0.15米代理，已移除新数据路径上的x500 0.3859米硬编码。

## 动态运动

全部七门都随外部时间运动，不由无人机到达触发；初相统一零。平移在门平面内；圆窗绕上方门外支点摆动，摆长1.6米，大于圆半径1.2米。局部中心位移为：

- 平移：`[0,-A sin(2πft),0]`。
- 摆动：`[L(1-cos θ),-L sin θ,0]`，`θ=asin(A/L) sin(2πft)`。

基频依次为0.30、0.37、0.42、0.33、0.46、0.40、0.35 Hz。实际幅度=`1.2米×幅度比×门因子`，门因子0.90、1.00、0.80、0.90、1.00、0.85、0.95。沿用静态及0.25/0.5/0.75幅度比×1/3/6频率倍率，共十档；最高档幅度不超过0.9米、频率最高2.76 Hz。所有门同时按档位升档，个体幅度/基频不同。这些是研究动态扩展，原TOGT场景本身静止。

## 算法边界与入口

本文仍为原生PVAJ MINCO、显式缩放xy、正时间映射和SIP补点，时间/动力学软权重仍为1/1000；本次对齐场景和车辆，未声称将本文优化器改成原TOGT。八组逐段时间扰动沿用既有独立模式，但基准按新原始几何的逐段距离与作者speedGuess=1 m/s重新生成；VAJ由新轨迹的MINCO重新计算，不能复用x500旧路径的物理状态。无中心精修，双方各8个独立初值。

新配置使用`--soft-mincopv`入口；旧工作空间与逐段0.3～15秒范围已从这个入口移除，只保留独立正时间映射。原0.1 m/s正向穿越条件与原软动力学目标保持。有限问题不成功或残差不合格就停止，不补点、不原样重复失败。

作者终点恰在第6门平面，悬停到终点不是第2次过窗。新协议只去掉这一已知零PVAJ末端的四重平面接触根；其他根、实际穿越和窗框距离检查全部保留。独立任务检查使用同一语义，已做保留内部真实穿越根的回归。

```sh
BIN=/tmp/togt_aligned_build_20261114
# 与cpp_runtime同一个CasADi/TOGT C++构建，目标名如下：
cmake --build "$BIN" --target togt_aligned_inputs togt_aligned_validate sip_planner matrix mesh_check
"$BIN/togt_aligned_inputs" ORIGINAL_PVAJ_TIME_ARCHIVE AUTHOR_2024_SOURCE NEW_INPUT_DIRECTORY
"$BIN/togt_aligned_validate" NEW_INPUT_DIRECTORY AUTHOR_2024_SOURCE VALIDATION.json

env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 SIP_DUMP_NLP=1 \
 "$BIN/matrix" "$BIN" NEW_INPUT_DIRECTORY/inputs NEW_INPUT_DIRECTORY/assets \
 NEW_RESULT_DIRECTORY 16 static --main-xy-eight
```

当前输入：`results/inputs_v3_mesh512_20261114`；`EQUIVALENCE.json`证明与v2的60个规划输入文件逐字节一致，仅将圆窗三角网格从128段加密至512段。`inputs_v1`保留较早辅助变量缩放，`planning_v1`因旧域标志接入错误在求解前停止，不能记成求解失败或正式规划结果。最新四方法十档规划位于`results/planning_v3_mesh512_20261114`，完成状态与每个原始result/progress逐项核对。


## 四方法完整规划

`run.cpp`通过一个C++协调进程逐条件运行本文自由点/中心（各8独立初值、16路）和动态TOGT自由点/中心（原单初值流程）。只做规划，Wang/CPC不在本轮；不启动Gazebo。`PLANNING.md`汇总轨迹结论与计算墙钟，`SUMMARY.json`保留来源，status与实际PID及原始result共同核对。

基础动态组同一条曲线的圆网格收敛复查：128段报约0.038毫米接触，256/512/1024段均通过。128段圆的最大内半径收缩约0.361毫米，大于0.1毫米数值间隙；512段最大收缩约0.0226毫米。轨迹、开口、碰撞球和判定阈值均不变，保留原始检查与resolution_*.json。静态和基础档只重查未重算轨迹，其余八档从头使用512段网格。

## 当前布局适配对结果的影响

作者公开混合七门文件的第7门原为SingleBall，没有门平面；本轮按七个方门布局取第7门yaw=200°，再按用户要求替换为实体方框。作者终点与第6门中心相同，因而第7门是折返段。当前发现静态动态TOGT在第7门通过点之后、转向终点时触碰其边框；部分本文中心候选的距离复查通过，但折返时再次穿过第7门开口，任务检查拒收。这些结果保留，不能统称为高速移动窗口造成的失败，也不等同于作者原场景表现。

完整批次结束后使用`final_report.cpp`汇总；`FINAL_PLANNING.md`为检查原始轨迹/任务证据后的最终表，`FINAL_CANDIDATE_AUDIT.json`列出各初值停止原因。原始协调器表里的“证据不足”若实际已明确违反门序/次数/端点，将在最终表更正为“未完成”，原表不覆盖。

## 从现有冻结输入运行四方法

以下命令在仓库根目录执行。使用新的结果目录，不能和正在运行的目录重复启动。`run.cpp`一次运行十档；每档本文8+8初值并行，随后运行动态TOGT中心和自由点。已有结果目录只用于恢复及重查，不作为一次全新求解计时。

```sh
E='convex_timevarying_window/comparisons/论文实验/实验2'
BIN=/tmp/togt_aligned_build_20261114
cmake -S "$E/cpp_runtime" -B "$BIN" \
 -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build "$BIN" --target sip_planner matrix dynamic_togt_planner \
 togt_aligned_run togt_aligned_report -j4
env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 SIP_DUMP_NLP=1 \
 "$BIN/togt_aligned_run" "$BIN" \
 "$E/togt_aligned/results/inputs_v3_mesh512_20261114" \
 '静态凸窗口/复现/TOGT-Planner-ICRA2024/parameters/cpc' \
 "$E/togt_aligned/results/NEW_FOUR_METHOD_BATCH"
# 仅在协调器正常结束、40行全部生成之后：
"$BIN/togt_aligned_report" "$E/togt_aligned/results/NEW_FOUR_METHOD_BATCH"
```

## 最新静态验证，2026-11-15

用户要求第7处恢复作者原航点。本轮按作者SingleBall半径0.3m三维邻域重做静态四方法，结果在[静态航点报告](results/point7_static_20261115/REPORT.md)。六个实体门的窗框与任务检查四方法均通过。本文两种方案各8/8独立初值通过。本文中心有地面相交，按此前口径单列。原十档实体第7门的数据保留。

C++入口为`point7.cpp`，包含prepare、validate和report。静态四方法入口为`togt_aligned_run BIN INPUTS AUTHOR_PARAMS OUTPUT static`。输入目录为`results/inputs_point7_static_20261115`。原曲线隔离复查和本次重算分别记录在SAME_CURVE_RECHECK.json和FINAL_SUMMARY.json。

## 动态TOGT源码入口更新，2026-11-15

正常`dynamic_togt_planner`已改为直接作者TrajSolver动态扩展，两种模式默认使用作者独立单初值。源码与改动公式见[实现说明](dynamic_togt_from_source/README.md)，最新十档两方法结果见[源码复测报告](results/author_source_rebuild_20261115/REPORT.md)。上文中心后自由设置对应历史批次。静态源代码一致性与整机几何检查分别列出。
