# 基于作者源码的动态TOGT

本实现从本地`静态凸窗口/复现/TOGT-Planner-ICRA2024`复制作者的`TrajSolver`头文件和实现文件。`author_source`保存带动态扩展的副本，其余MINCO、TrajData、正时间映射、Rectangle/Ball选点映射、动力学罚项和L-BFGS直接链接作者文件。`AUTHOR_SOURCE.json`记录原文件SHA。

作者`RacePlanner::planTOGT`调用`TrajData::initData`，用节点间距离除以`speedGuess`初始化各段时长，随后调用一次`TrajSolver::solve`联合优化。当前自由点和中心均按这一路径独立求解。自由点初值采用各门中心，中心版通过零维固定点形状去掉选点变量。

## 动态扩展

每次代价计算先恢复各段时间，再累加得到到达时刻。`MovingShape`在门局部坐标中调用作者形状映射，并用查询得到的门中心及固定旋转矩阵转成世界坐标。

\[
P_i=c_i(t_i)+R_iq_i(d_i),\qquad t_i=\sum_{j=0}^{i}T_j.
\]

作者MINCO回传位置梯度之后，给每个段时间加入后续所有航点运动的链式项。

\[
\frac{dJ}{dT_j}=\left.\frac{dJ}{dT_j}\right|_{P}
+\sum_{i\ge j}\frac{\partial J}{\partial P_i}^{\!\top}\dot c_i(t_i).
\]

当前场景平移及摆动均通过门中心函数描述，门平面保持固定。圆门采用作者Ball映射的二维切片，SingleBall航点采用完整三维映射。四边形采用作者Rectangle构造与顶点顺序。旋翼推力和角速度软代价继续由作者`computeRobustPenalityCost`计算。

## 核对与结果

`verify.cpp`检查零运动下代价与梯度一致，及高频动态场景完整时间/选点梯度。另一个可执行程序`togt_original_static_reference`链接未经修改的作者求解器，和新版本比较静态初值、最终变量及轨迹。

首次验证器的四轮测试错误地连续缩短了上一轮时长，该失败记录保存在结果目录的`DYNAMIC_GRADIENTS_DEBUG.json`。修正后每轮从同一原始时长生成独立扰动，自由/中心最大相对梯度误差分别约3.62e-6和1.81e-6，零运动误差为0。

旧动态版先求中心再开放自由点，采用自定义Polyhedron顶点顺序。原始成功停止只持续了4轮，选点移动约0.1毫米。新入口默认使用作者直接联合求解。显式`--center-result`和`--initial-manifest`仍用于已有实验初值复现，结果会注明来源。

正式入口为`cpp_runtime`构建的`dynamic_togt_planner`，历史入口为`legacy_dynamic_togt_planner`。新十档结果位于上级`results/author_source_rebuild_20261115`。代码均为C++。

```sh
E='convex_timevarying_window/comparisons/论文实验/实验2'
BIN=/tmp/togt_source_build_20261115
cmake -S "$E/cpp_runtime" -B "$BIN" \
 -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build "$BIN" --target dynamic_togt_planner togt_source_verify \
 togt_original_static_reference togt_source_batch -j4
"$BIN/dynamic_togt_planner" SCENE_JSON OUTPUT_DIRECTORY free AUTHOR_PARAMETERS VEHICLE_JSON
```

## 当前TOGT选点内缩配置，2026-11-15

当前十档试验采用每侧0.30米，配置为`config/quada_inset030.json`。每档自由点和中心各独立单初值。完整0.15/0.30/0.40米对照在上级`results/author_source_inset_comparison_20261115/REPORT.md`。机体碰撞球半径0.15米，0.30米包含额外0.15米选点净空。0.30米自由点7/10通过，0.40米5/10通过，两个增大设置均如实保留。

```sh
"$BIN/dynamic_togt_planner" SCENE_JSON OUTPUT_DIRECTORY free AUTHOR_PARAMETERS \
 "$E/togt_aligned/dynamic_togt_from_source/config/quada_inset030.json"
# 十档批处理的末尾参数为每侧选点内缩
"$BIN/togt_source_batch" "$BIN/dynamic_togt_planner" TEN_LEVEL_INPUTS \
 POINT7_INPUTS AUTHOR_PARAMETERS NEW_OUTPUT_DIRECTORY 0.30
```

## 2026-11-17 作者源码复核

最新22条件在实验2/dynamic_togt_audit_20261117；共同规划球0.20m、物理球0.15m。默认仍作者独立单初值和原权重1，未沿用旧中心解初始化。两种静态最终变量、系数及目标与未经改动作者TrajSolver逐值一致；最高档动态梯度检查通过。本次补存optimizer_return_code和optimizer_message，保留具体失败原因，不改变优化行为。自由11/22、中心19/22数值安全完成，其余均检出碰撞。本文当前权重1000且八配置，不应称为同目标同预算消融。详见该目录README/AUDIT/COMPARISON。
