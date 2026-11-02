# MPC 执行模型对齐依据（2026-10-10）

2026-10-12修正：VehicleAngularVelocity先原地40Hz低通，再对该数组求导和20Hz低通。
此前本文件对应代码错将D项用于原始角速度；现已修正，state[26:29]保存上一次滤波角速度。
旧原始数值实验不覆盖；新的直接CTBR acados跟踪/时序对齐见 `../togt_style_mpc/README.md`。

实际容器 PX4 版本：1.18.0，git `83c4f4e580558816f0e398918f66358988004696`。
不改变飞控参数、规划目标、安全余量、赛道、穿窗事件或碰撞判据。

模型按该提交的公开源码和容器内参数建立：

- [rate_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/src/lib/rate_control/rate_control.cpp)：P/I/D、积分限幅和抗饱和。
- [MulticopterRateControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/src/modules/mc_rate_control/MulticopterRateControl.cpp)：偏航2Hz低通。
- [ControlAllocationPseudoInverse.cpp](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp)：归一化伪逆。
- [ControlAllocationSequentialDesaturation.cpp](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp)：MC_AIRMODE=0分配及去饱和，非物理力空间直接混控。
- [VehicleAngularVelocity.cpp](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp)：40Hz二阶角速度滤波、20Hz一阶微分滤波。
- 容器 `/opt/px4-gazebo/share/gz/models/x500{,_base}/model.sdf`：实际桨位±0.174m、高度0.06m，推力系数8.54858e-6，电机上升/下降时间常数0.0125/0.025s，转速150–1000rad/s。
- [Gazebo MulticopterMotorModel](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc)：桨处阻力、阻力力臂及rolling moment；旧预测遗漏了后两项。

注意PX4 CA参数的桨位(±0.13,±0.20/0.22)m和KM=0.05不是Gazebo真实桨位/KM=0.016。
必须分别模拟飞控混控和实际刚体，不能把两份参数混成一份理想混控器。
PID为P=[.15,.15,.2]、I=[.2,.2,.1]、D=[.003,.003,0]，积分限幅.3。
THR_MDL_FAC=0，归一化电机输出线性映射至150–1000rad/s，再平方产生力。

位置接口另依据实际world.sdf的EARTH_WGS84 datum与PX4
[geo.h](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/src/lib/geo/geo.h)
的`CONSTANTS_RADIUS_OF_EARTH=6371000m`，按WGS84子午/卯酉圈曲率半径换算局部GPS地图米数。
这是局部一阶坐标换算，当前百米范围高阶项毫米级；不把它当GPS随机误差的补偿器。

内部状态包含积分和滤波器记忆；由测得角速度、已发指令推进状态观测器，不新增飞行策略。
仍是离散近似模型：不是逐指令复刻线程调度、传感器噪声、旋翼关节和估计器，也不是安全保证。
后续模型日志对照与飞行结果另报，不能以读到参数或单元测试通过宣称跟踪已修复。

分配和PID数学结构依据PX4 BSD-3-Clause源码重新表达。原作者包括PX4 Development Team
(2013–2023)、Julien Lecoeur、Roman Bapst、Beat Küng。
保留许可全文[PX4_LICENSE.txt](PX4_LICENSE.txt)；亦见官方仓库 [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/83c4f4e580558816f0e398918f66358988004696/LICENSE)。
