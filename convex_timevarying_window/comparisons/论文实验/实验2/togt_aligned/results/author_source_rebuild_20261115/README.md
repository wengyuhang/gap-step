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
