# 仅放宽迭代上限的独立求解

状态 complete，已完成 9/9 条。

每个条件直接使用自身完整运动和原四初值，只将 SLSQP 上限 400 改为 2000。每初值仍为180秒。
原固定Y姿态、log1p总时间、模型、约束、验收和择解保持一致。原9条均未找到合格规划。
时间含七窗后返航至零VAJ，为规划轨迹结果。

|幅度/窗宽|频率倍数（Hz范围）|方法|新结果|合格轨迹时间 s|四初值求解 s|终止记录|
|---|---|---|---|---:|---:|---|
|0.25|3（0.9–1.8）|自由点|名义无碰撞|18.062545|205.976|Positive directional derivative for linesearch ×3 / Optimization terminated successfully ×1|
|0.25|6（1.8–3.6）|自由点|未找到合格规划|—|233.659|Positive directional derivative for linesearch ×3 / Inequality constraints incompatible ×1|
|0.5|3（0.9–1.8）|强制中心|未找到合格规划|—|275.253|Positive directional derivative for linesearch ×3 / wall_budget ×1|
|0.5|6（1.8–3.6）|自由点|未找到合格规划|—|263.185|Optimization terminated successfully ×2 / Positive directional derivative for linesearch ×3 / Inequality constraints incompatible ×1|
|0.5|6（1.8–3.6）|强制中心|未找到合格规划|—|316.234|wall_budget ×1 / Positive directional derivative for linesearch ×3|
|0.75|1（0.3–0.6）|自由点|未找到合格规划|—|477.302|wall_budget ×2 / Inequality constraints incompatible ×1 / Positive directional derivative for linesearch ×1|
|0.75|3（0.9–1.8）|自由点|名义无碰撞|26.124145|245.337|Inequality constraints incompatible ×2 / Optimization terminated successfully ×7 / Positive directional derivative for linesearch ×1|
|0.75|3（0.9–1.8）|强制中心|未找到合格规划|—|267.859|Inequality constraints incompatible ×1 / Positive directional derivative for linesearch ×2 / wall_budget ×1|
|0.75|6（1.8–3.6）|强制中心|未找到合格规划|—|293.955|Positive directional derivative for linesearch ×3 / More than 3*n iterations in LSQ subproblem ×1|

全部原始四候选见 plans，逐条件复算见 assessments。未找到记录不代表物理无解。
合格要求优化器收敛、原节点硬约束及全部几何复查通过，并独立复算整条轨迹与所有实体窗框和地面。
这是名义球/STL浮点数值评估。
