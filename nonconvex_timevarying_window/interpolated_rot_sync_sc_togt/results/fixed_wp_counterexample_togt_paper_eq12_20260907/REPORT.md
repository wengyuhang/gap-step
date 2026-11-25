# 双 SC 输入方法与 Fixed-WP 反例对比

场景为既有均衡 U 反例：尺寸比 1.9、角速度 4.5 rad/s、初相位 1.1 rad。
两种方法采用 TOGT 论文式 (12)/(15) 目标：J = T + ∫max(h,0)^3dt。
snap 和碰撞都不进入目标；动力学限制、求解器预算与真实姿态长方体审计保持一致。

动力学限制：速度不超过 7 m/s，XY/Z 机体角速率分别不超过 10/10 rad/s，单旋翼推力范围 [0.25, 5] N。

|method|J|T|raw snap (diagnostic only)|paper dynamic penalty|collision objective term|
|---|---:|---:|---:|---:|---:|
|Fixed-WP|2.84906321|2.84622526|14367.2764|0.00283795123|0 (not evaluated)|
|Interpolated-RotSync|6.66376586|6.65576181|91523.049|0.00800404634|0 (not evaluated)|

|方法|轨迹验收|优化器收敛|无碰撞|动力学合格|碰撞点/审计点|最大速度|最大 XY 角速率|最大 Z 角速率|旋翼推力范围|C3 跳变|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|Fixed-WP|False|True|True|False|0/3040|7.02511|2.44261|0.7854|[1.15522, 4.12477]|3.834e-14|
|Interpolated-RotSync|False|False|True|False|0/6853|6.9415|10.0292|100.534|[-116747, 116753]|4.556e-13|

`trajectory_pass` 由独立最大 1 ms 网格及临界时刻细化决定，
与 `optimizer_success` 分开。采样无碰撞不是连续时间安全证明。
