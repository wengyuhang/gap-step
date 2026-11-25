# 五方法十档，0.30米规划安全球与独立机体碰撞模型

2026-11-15按用户纠正，规划安全球半径保持0.30米；物理碰撞按现有QuadA仿真碰撞体，球半径0.15米。原作者只有臂长等动力学参数，含桨完整外廓未提供。物理尺寸来源及SDF哈希见config/physical_collision_model.json。

六个实体门依次方、圆、方、圆、方、圆，开口2.4米。第7处为作者原SingleBall半径0.3米航点，完整轨迹执行到作者终点。运动、相位、位置与QuadA动力学沿用原批次。

## 当前结果

当前碰撞统计为../results/body_collision_recheck_20261115/PLANNING.md。C++重新检查50条既有轨迹，保留原轨迹系数、段时长、选点及原始求解墙钟。本文自由/中心各10档通过窗框与任务检查，动态TOGT两方各7档，Wang9档。地面分别另记3/10、10/10、0/10、1/10、0/10。原0.30米球包络检查完整保留在five_method_sphere030_20261115，当前表中单列safety_sphere_overlap_detected。

## 求解与碰撞接口

- 本文两方法各8个独立时间初值、16路并行，无中心释放。原生PVAJ MINCO、正时间映射、显式点域与安全补点，时间/软动力学权重1/1000。
- 动态TOGT为作者TrajSolver的动态适配，各独立单初值及原L-BFGS参数。
- Wang为既有N2/M4、s4、四时间初值独立复现的QuadA适配，角速范数15rad/s与本文/TOGT的XY15、Z2.99形式不同，原协议记录保留。
- inputs.cpp分别写planning_safety_sphere_radius_m与physical_collision_radius_m。规划有限距离半径0.3851米、规划复查0.385米，含0.085米框管和0.1毫米数值间隙；scene.json的collision_radius_m从原仿真机体输入读取，用于独立物理碰撞统计。
- recheck_body.cpp通过同一mesh_evaluate检查实体网格与动态运动，保留原始计算墙钟，新增检查耗时独立记录。

全部规划、计算图生成和几何复查为C++。输入生成器的正常入口为five_method_inputs，协调器为five_method_run_ours/five_method_run_baselines，报告器为five_method_report。

```sh
BIN=/tmp/togt_source_build_20261115
"$BIN/five_method_recheck_body" ORIGINAL_RESULTS INPUTS \
  config/physical_collision_model.json NEW_OUTPUT
```

原批次输入为../results/inputs_five_method_sphere030_20261115，Wang图为../results/wang_inputs_sphere030_v3_20261115。这些历史输入保留当时的0.30米判碰字段。新生成输入已采用分开的机体碰撞尺寸，当前重查的十档场景保存在body_collision_recheck_20261115/assets。70张Wang图独立系数与梯度核对、160本文候选外层审计沿用原记录。当前源码修正和轨迹不变核对见RECHECK_AUDIT.json。
