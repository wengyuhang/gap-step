# 零厚度窗口：双 SC 输入与固定 SC 输入 RotSync 对比

场景为既有均衡 U 几何：尺寸比 1.9、角速度 4.5 rad/s、初相位 1.1 rad，窗口厚度为零。
Sync 入口/出口是规划球包络与窗口平面相切时刻，球心法向坐标分别为 -rho/+rho。
两种方法采用 TOGT 配套 C++ 目标：J = T + 动态 8–32 区间梯形积分的 smoothedL1 动力学惩罚。
snap、速度和碰撞都不进入目标；优化无墙钟和迭代上限，动力学与碰撞仍独立审计。

动力学限制：速度不超过 7 m/s，XY/Z 机体角速率分别不超过 10/10 rad/s，单旋翼推力范围 [0.25, 5] N。

|method|J|T|raw snap (diagnostic only)|TOGT-code dynamic penalty|collision objective term|
|---|---:|---:|---:|---:|---:|
|RotSync|5.82942877|5.81120151|132776.881|0.0182272588|0 (not evaluated)|
|Interpolated-RotSync|6.60293847|6.60293847|85745.0398|0|0 (not evaluated)|

|方法|轨迹验收|优化器收敛|无碰撞|TOGT-code 1ms 动力学|原审计动力学|碰撞点/审计点|最大速度|最大 XY 角速率|最大 Z 角速率|旋翼推力范围|C3 跳变|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|RotSync|False|True|True|False|False|0/5999|7.27075|10.0738|1308.91|[-4.91346e+06, 4.91344e+06]|3.006e-13|
|Interpolated-RotSync|False|True|True|False|False|0/6796|6.88698|10.2202|2328.76|[-1.30662e+06, 1.30652e+06]|3.767e-13|

`trajectory_pass` 由独立最大 1 ms 网格及临界时刻细化决定，
与 `optimizer_success` 分开。采样无碰撞不是连续时间安全证明。
TOGT-code 动力学列在同一 1 ms 网格上使用 C++ tilt-yaw/robust-singularity 残差；
后续角速率和旋翼极值列保留原项目姿态恢复审计，两者不混为同一模型。
