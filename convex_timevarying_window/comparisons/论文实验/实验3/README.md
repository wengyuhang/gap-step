# 实验3 单窗高速运动安全对比

2026-11-10完整实验已完成。四方法、四档、134条候选和15趟Gazebo/PX4正式执行，档位从静态开始，动态幅度和频率每档同步增加1米及1Hz。本文中心点在3米/3Hz八初值均未找到合格轨迹，按冻结规则停止。

本文自由点四档均无窗框碰撞完成。瞬时安全消融在2米/2Hz及3米/3Hz碰撞，动态TOGT中心在3米/3Hz碰撞。本文中心前三档无碰撞完成，第四档保留未找到及全部有限NLP记录。

- [实验报告](results/single_gate_stress_v1_20261110/REPORT.md)
- [规划结果](results/single_gate_stress_v1_20261110/PLANNING.md)
- [实飞结果](results/single_gate_stress_v1_20261110/FLIGHT.md)
- [逐初值迭代、求解时间及结果](results/single_gate_stress_v1_20261110/CANDIDATES.csv)
- [冻结协议](PROTOCOL.md)

场景沿用W1矩形STL，竖向2.90米、水平2.30米，起点和窗后终点各距平面8米。时间包含完整起点至窗后终点航段。未找到为当前八初值和求解配置的结果，未据此认定物理不可行。

规划入口为`run.py`，串行实飞入口为`flights.py`，独立复核入口为`validate.py`，表图汇总入口为`analyze.py`。原始候选、ULog、两路原生接触、实际窗位姿及无损压缩SHA均保留。原有实验2和控制器源码保持。
