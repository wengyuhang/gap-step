# 双 SC 输入方法与 Fixed-WP 反例对比

场景为既有均衡 U 反例：尺寸比 1.9、角速度 4.5 rad/s、初相位 1.1 rad。
两种方法采用 TOGT 配套 C++ 目标：J = T + 动态 8–32 区间梯形积分的 smoothedL1 动力学惩罚。
snap、速度和碰撞都不进入目标；优化无墙钟和迭代上限，动力学与碰撞仍独立审计。

动力学限制：速度不超过 7 m/s，XY/Z 机体角速率分别不超过 10/10 rad/s，单旋翼推力范围 [0.25, 5] N。

|method|J|T|raw snap (diagnostic only)|TOGT-code dynamic penalty|collision objective term|
|---|---:|---:|---:|---:|---:|
|Fixed-WP|2.28302321|2.24591372|81469.3151|0.0371094836|0 (not evaluated)|
|Interpolated-RotSync|6.55915303|6.55915272|82996.3688|3.16093632e-07|0 (not evaluated)|

|方法|轨迹验收|优化器收敛|无碰撞|动力学合格|碰撞点/审计点|最大速度|最大 XY 角速率|最大 Z 角速率|旋翼推力范围|C3 跳变|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|Fixed-WP|False|True|True|False|0/2427|9.06002|5.08137|1.89494|[-1.06405, 6.11396]|3.887e-13|
|Interpolated-RotSync|False|True|True|False|0/6757|6.95428|10.2918|1334.29|[-3.04684e+06, 3.0468e+06]|4.215e-13|

`trajectory_pass` 由独立最大 1 ms 网格及临界时刻细化决定，
与 `optimizer_success` 分开。采样无碰撞不是连续时间安全证明。
