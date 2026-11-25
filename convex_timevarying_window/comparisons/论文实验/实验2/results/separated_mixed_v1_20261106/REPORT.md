# 实验二：独立穿窗区间的混合动态七窗

状态：dynamic_preflight，8/40条。

固定非自交折线+外侧返航；四平移/三摆动。仅W3与W6交替升档，其余五窗维持各自低速运动，初相均零。
完整参考时间含七窗返航至零VAJ，不是实飞时间。名义球/STL碰撞；全部窗口全轨迹检查，非目标窗口风险不忽略。

|档位|W3平移 Hz/m|W6摆动 Hz/m|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|
|---|---|---|---:|---:|---:|---:|
|static|0.5/0|0.6/0|15.165s，名义无碰撞|15.879s，名义无碰撞|17.379s，名义无碰撞|18.072s，名义无碰撞|
|mixed_base|0.5/1|0.6/1|待完成|待完成|16.913s，名义无碰撞|17.954s，名义无碰撞|
|translate_1|1/1.5|0.6/1|待完成|待完成|待完成|待完成|
|swing_1p5|1/1.5|1.5/1.5|待完成|待完成|待完成|待完成|
|translate_2|2/2|1.5/1.5|待完成|待完成|待完成|待完成|
|swing_2p5|2/2|2.5/2|待完成|待完成|待完成|待完成|
|translate_3|3/3|2.5/2|待完成|待完成|待完成|待完成|
|swing_3p5|3/3|3.5/3|待完成|待完成|待完成|待完成|
|translate_4|4/4|3.5/3|待完成|待完成|待完成|待完成|
|swing_4|4/4|4/4|待完成|待完成|18.122s，名义碰撞|18.055s，名义碰撞|

## 计算与轨迹证据

|档位|方法|建图 s|四初值求解 s|独立检查 s|非相邻窗风险|原始结果|
|---|---|---:|---:|---:|---|---|
|static|本文自由点|2.528|51.477|0.135|无|[记录](assessments/static_ours.json)|
|static|本文中心|2.433|43.289|0.091|无|[记录](assessments/static_center.json)|
|static|动态TOGT自由点|0.001|2.922|0.102|无|[记录](assessments/static_dynamic_togt.json)|
|static|动态TOGT中心|0.001|0.454|0.094|无|[记录](assessments/static_dynamic_togt_center.json)|
|mixed_base|动态TOGT自由点|0.001|4.515|0.112|无|[记录](assessments/mixed_base_dynamic_togt.json)|
|mixed_base|动态TOGT中心|0.001|0.898|0.102|无|[记录](assessments/mixed_base_dynamic_togt_center.json)|
|swing_4|动态TOGT自由点|0.001|7.254|0.105|无|[记录](assessments/swing_4_dynamic_togt.json)|
|swing_4|动态TOGT中心|0.001|4.334|0.100|无|[记录](assessments/swing_4_dynamic_togt_center.json)|

几何图/核验：[布局](course_layout.png)、[运动包络](geometry_audit.json)、[正式实验准入](preflight_gate.json)。
几何走廊分离只覆盖声明的航路走廊，不证明任意绕行轨迹安全；所有实际参考仍独立检查。
未找到、检查待定、任务未完成、名义碰撞分别保留；只有双方完整名义通过才比较时间。
静态四方法与最高动态TOGT预检通过“无非相邻窗风险”后才进行其余正式条件；预检规划原样复用，不重新求解择优。
旧紧凑赛道与旧实飞结果不混入本批，规划目标/约束/四时间初值/预算/选解规则未改。
