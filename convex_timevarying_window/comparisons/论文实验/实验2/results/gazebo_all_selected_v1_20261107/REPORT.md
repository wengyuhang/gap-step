# 五方法十赛道Gazebo/PX4执行

状态 complete，已登记 50/50 项。

执行现有选中轨迹，统一x500/PX4和北图航向解析参考MPC。完整时间从统一起跑时刻到七窗后返航停稳，单位秒。
完成条件为七窗整机出面、每窗正向一次、返回距起点0.55m以内且真值速度低于1m/s，并且全部原生接触记录为零。

|幅度/窗宽|频率倍数|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang独立复现|
|---|---:|---:|---:|---:|---:|---:|
|静态|1|16.706|17.452|20.925|21.766|20.831|
|0.25|1|16.668|17.354|20.940|21.786|20.689|
|0.25|3|16.515|17.195|20.844|21.836|21.046|
|0.25|6|21.828|22.791|21.139|22.212|21.690|
|0.5|1|16.661|17.302|20.840|21.754|20.631|
|0.5|3|18.688|18.764|碰撞|21.914|21.874|
|0.5|6|23.985|36.942|碰撞|碰撞|碰撞|
|0.75|1|16.672|17.265|20.726|21.626|20.749|
|0.75|3|26.305|26.431|碰撞|碰撞|碰撞|
|0.75|6|36.843|未找到规划|碰撞|碰撞|碰撞|

七窗基准频率0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz。运动与规划阶段保持相同初相及绝对时钟。

## 各次执行记录

|条件|方法|结果|七窗计数|参考时间 s|闭环时间 s|真值跟踪最大误差 m|原因|
|---|---|---|---:|---:|---:|---:|---|
|static|本文自由点|completed_no_contact|7|17.055|16.706|0.110||
|static|本文中心|completed_no_contact|7|17.810|17.452|0.077||
|static|动态TOGT自由点|completed_no_contact|7|21.475|20.925|0.091||
|static|动态TOGT中心|completed_no_contact|7|22.370|21.766|0.105||
|static|Wang独立复现|completed_no_contact|7|21.284|20.831|0.092||
|mixed_base|本文自由点|completed_no_contact|7|17.012|16.668|0.079||
|mixed_base|本文中心|completed_no_contact|7|17.718|17.354|0.087||
|mixed_base|动态TOGT自由点|completed_no_contact|7|21.469|20.940|0.082||
|mixed_base|动态TOGT中心|completed_no_contact|7|22.404|21.786|0.078||
|mixed_base|Wang独立复现|completed_no_contact|7|21.147|20.689|0.116||
|a25_f3|本文自由点|completed_no_contact|7|16.894|16.515|0.100||
|a25_f3|本文中心|completed_no_contact|7|17.553|17.195|0.081||
|a25_f3|动态TOGT自由点|completed_no_contact|7|21.395|20.844|0.089||
|a25_f3|动态TOGT中心|completed_no_contact|7|22.456|21.836|0.115||
|a25_f3|Wang独立复现|completed_no_contact|7|21.494|21.046|0.054||
|a25_f6|本文自由点|completed_no_contact|7|22.102|21.828|0.103||
|a25_f6|本文中心|completed_no_contact|7|23.102|22.791|0.093||
|a25_f6|动态TOGT自由点|completed_no_contact|7|21.677|21.139|0.125||
|a25_f6|动态TOGT中心|completed_no_contact|7|22.803|22.212|0.068||
|a25_f6|Wang独立复现|completed_no_contact|7|22.143|21.690|0.103||
|a50_f1|本文自由点|completed_no_contact|7|16.993|16.661|0.082||
|a50_f1|本文中心|completed_no_contact|7|17.652|17.302|0.074||
|a50_f1|动态TOGT自由点|completed_no_contact|7|21.369|20.840|0.106||
|a50_f1|动态TOGT中心|completed_no_contact|7|22.375|21.754|0.110||
|a50_f1|Wang独立复现|completed_no_contact|7|21.086|20.631|0.090||
|a50_f3|本文自由点|completed_no_contact|7|18.974|18.688|0.071||
|a50_f3|本文中心|completed_no_contact|7|19.067|18.764|0.097||
|a50_f3|动态TOGT自由点|collision|1|22.268|—|58.197|timeout|
|a50_f3|动态TOGT中心|completed_no_contact|7|22.483|21.914|0.090||
|a50_f3|Wang独立复现|completed_no_contact|7|22.333|21.874|0.089||
|a50_f6|本文自由点|completed_no_contact|7|24.270|23.985|0.088||
|a50_f6|本文中心|completed_no_contact|7|37.231|36.942|0.116||
|a50_f6|动态TOGT自由点|collision|0|24.860|—|50.050|timeout|
|a50_f6|动态TOGT中心|collision|1|23.669|—|67.706|timeout|
|a50_f6|Wang独立复现|collision|2|25.300|—|44.510|timeout|
|a75_f1|本文自由点|completed_no_contact|7|17.004|16.672|0.093||
|a75_f1|本文中心|completed_no_contact|7|17.614|17.265|0.082||
|a75_f1|动态TOGT自由点|completed_no_contact|7|21.235|20.726|0.079||
|a75_f1|动态TOGT中心|completed_no_contact|7|22.259|21.626|0.098||
|a75_f1|Wang独立复现|completed_no_contact|7|21.207|20.749|0.112||
|a75_f3|本文自由点|completed_no_contact|7|26.587|26.305|0.098||
|a75_f3|本文中心|completed_no_contact|7|26.738|26.431|0.083||
|a75_f3|动态TOGT自由点|collision|1|24.234|—|53.661|timeout|
|a75_f3|动态TOGT中心|collision|1|24.401|—|44.945|timeout|
|a75_f3|Wang独立复现|collision|2|23.313|—|15.982|RuntimeError('missed_opening_no_reverse_reentry')|
|a75_f6|本文自由点|completed_no_contact|7|37.152|36.843|0.146||
|a75_f6|本文中心|planning_not_found|—|—|—|—||
|a75_f6|动态TOGT自由点|collision|0|25.334|—|36.361|RuntimeError("RuntimeError('No finite MPC objective')")|
|a75_f6|动态TOGT中心|collision|1|26.632|—|9.114|RuntimeError('missed_opening_no_reverse_reentry')|
|a75_f6|Wang独立复现|collision|1|22.486|—|20.889|timeout|

每条件原初相一次。名义碰撞/超限的49条现有参考均进入执行清单，原未找到的一条登记为未找到。
记录ULog、原生窗框/地面接触、窗口真实位姿、MPC输入/估计状态与参考哈希，原始日志无损压缩并保存原文SHA。
启动和起飞准备时间单独保留。原Wang已生成的局部执行前缀按原时钟拼接为冻结参考，本轮统一评估轨迹的飞控闭环执行。
协议、源码和参数见PROTOCOL.md及protocol.json。
