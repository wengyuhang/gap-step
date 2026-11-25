# Random-DK：三窗口开放赛道

按 U1 → U2 → U3 穿越；中心为 (0,0,1.8)、(9,1,2.3)、(18,-0.8,1.4) m，平面法向均为 X。
每窗均为零厚度均衡 U、18 rad/s 自旋，初相位分别为 1.1/0.3/2.0 rad。起点 (-4.5,-0.8,1.8)，终点 (22.5,0.8,1.8)，起终速度/加速度/jerk 为零。
完整轨迹采用原始 SC 的 4 段 MINCO，4 个 K 标量与 3 组二维 D 联合扰动（共 10 个标量）。每个窗口的全部接触区间都检查，后续窗口使用累计绝对到达时刻。

- 结果：NO_FEASIBLE_CANDIDATE_FOUND；SC T=6.180275047 s，优化器停止=True。
- 原始筛选：sphere_violated；随机候选 8000，全部筛选通过 0。
- D 半径比例 [1.0, 2.0, 4.0, 8.0]；K 直接扰动比例 [0.5, 1.0, 2.0, 4.0]；每档 2000 个，种子 20260909。
- 基线来源：/home/jack/wyh/时变窗口/nonconvex_timevarying_window/random_dk_sc_dynatogt/results/three_u_native_seed20260909_4000/baseline.json。
- 通过前 1/2/3 窗球体筛选的候选数：[1351, 149, 57]（短路筛选，未检查者不是通过）。
- 首个失败原因：{'sphere_violated': 7916, 'crossing_order_or_count': 2, 'dynamics_velocity': 57, 'sphere_unresolved': 4, 'boundary_or_interface': 20, 'numerical_failure': 1}；首个失败窗口（0 基索引）：{'1': 1202, '0': 6629, 'None': 78, '2': 91}。
- SC 本次求解 0.000 s；扰动搜索 89.793 s；最终整机检测 0.000 s；总计 92.015 s。
- 最终选择：无，不返回违规最少者。

几何、动力学与整机验收均为名义模型密集采样，不是连续域证书。当前预算没有候选不表示全局不可行。
