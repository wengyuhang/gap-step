# 实验二：TOGT原始布局的动态小窗适配

状态：formal_planning，11/40条。

官方race_uzh_7g_multiprisma位置/顺序/前六姿态，2.2倍场地尺度与统一高度平移，保留上下叠窗和交错折返。
仍用现有七种实体小窗和x500；原第7个SingleBall改为第7物理窗、终点改返航，故是场景适配而非原场景完整复刻。
W3平移、W4摆动交替升级，其余五窗固定低速运动，初相0。时间含完整七窗返航零VAJ。

|档位|W3平移 Hz/m|W4摆动 Hz/m|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|
|---|---|---|---:|---:|---:|---:|
|static|0.5/0|0.6/0|18.036s，名义无碰撞|18.090s，名义无碰撞|21.475s，名义无碰撞|22.370s，名义无碰撞|
|mixed_base|0.5/1|0.6/1|17.245s，名义无碰撞|18.382s，名义无碰撞|21.457s，名义无碰撞|22.431s，名义无碰撞|
|translate_1|1/1.5|0.6/1|18.363s，名义无碰撞|待完成|待完成|待完成|
|swing_1p5|1/1.5|1.5/1.5|待完成|待完成|待完成|待完成|
|translate_2|2/2|1.5/1.5|待完成|待完成|待完成|待完成|
|swing_2p5|2/2|2.5/2|待完成|待完成|待完成|待完成|
|translate_3|3/3|2.5/2|待完成|待完成|待完成|待完成|
|swing_3p5|3/3|3.5/3|待完成|待完成|待完成|待完成|
|translate_4|4/4|3.5/3|待完成|待完成|待完成|待完成|
|swing_4|4/4|4/4|待完成|待完成|23.355s，名义碰撞|24.332s，名义碰撞|

## 原始证据与计算

|档位|方法|建图 s|四初值求解 s|检查 s|非相邻窗风险|记录|
|---|---|---:|---:|---:|---|---|
|static|本文自由点|2.614|164.458|0.123|无|[记录](assessments/static_ours.json)|
|static|本文中心|2.426|167.719|0.085|无|[记录](assessments/static_center.json)|
|static|动态TOGT自由点|0.001|3.151|0.101|无|[记录](assessments/static_dynamic_togt.json)|
|static|动态TOGT中心|0.001|0.468|0.096|无|[记录](assessments/static_dynamic_togt_center.json)|
|mixed_base|本文自由点|2.678|250.863|0.128|无|[记录](assessments/mixed_base_ours.json)|
|mixed_base|本文中心|2.564|179.158|0.089|无|[记录](assessments/mixed_base_center.json)|
|mixed_base|动态TOGT自由点|0.001|3.525|0.104|无|[记录](assessments/mixed_base_dynamic_togt.json)|
|mixed_base|动态TOGT中心|0.001|0.699|0.099|无|[记录](assessments/mixed_base_dynamic_togt_center.json)|
|translate_1|本文自由点|2.584|237.658|0.128|无|[记录](assessments/translate_1_ours.json)|
|swing_4|动态TOGT自由点|0.001|4.391|0.112|无|[记录](assessments/swing_4_dynamic_togt.json)|
|swing_4|动态TOGT中心|0.001|0.636|0.104|无|[记录](assessments/swing_4_dynamic_togt_center.json)|

![TOGT三维布局](course_layout.png)

先geometry/motion核验、静态四方全通过、低/最高动态TOGT无非相邻风险，preflight_gate通过才完整矩阵。
运动扫掠与1m锚点航段管道分离是布局筛查，不是任意轨迹证明；全部实际参考仍检查所有窗口和地面。
未找到/碰撞/待定/任务未完成分开；共同名义成功才计算时间优势，不把结果当Gazebo接触或跟踪安全。
仅场景适配，不改规划器/代价/约束/四初值/预算/择解；不复用被否决赛道成绩，不启动Gazebo。
