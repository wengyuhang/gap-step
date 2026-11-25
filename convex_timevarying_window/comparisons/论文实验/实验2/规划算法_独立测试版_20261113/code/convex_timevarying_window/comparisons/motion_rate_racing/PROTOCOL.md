# 2026-09-18 冻结协议：窗口运动频率与竞速

更新：此处哈希及原结果对应旧`worst_case`约束。新一轮`local_sweep_frozen_20260918`
保持以下场景、相位、预算和成功判据，显式使用`--local-sweep`，源码哈希另存其protocol.json。
新轮完成后提升局部约束为默认；复查旧公式需传`--legacy-crossing`。
新轮运行环境变动及计时证据边界见方法`LOCAL_SWEEP.md`，不将其当作严格硬件效率对照。

场景：当前七凸窗口 PX4/Gazebo x500，布局、平移/旋转幅度、机体、房间和起终点不变。
频率倍率 [1,3,6,10]，相位增量 [0,2π/3,4π/3]，每条件一次，从起点静止开始，七门后返回并减速。
这是每档三个配对相位条件，不是大量独立随机重复；成功比例分母固定为 3。
运动速度随倍率线性增加，加速度随倍率平方增加。不能把倍率当成平移幅度或仿真倍率。

主方法冻结 receding_gate_racing/planner.py 的 c28077294d6293082586f155c852ee5b896e88a8fdad0fb663e282a8fcda0fb4，
保留其默认 3 窗口、12 m/s、8 m/s²、25 m/s³、0.35 m 跟踪余量、65 ms 求解预算。
不按倍率、相位或门编号调参。规划随输入运动参数更新几何及运动界，不改变公式。
全部周期（包括失败）计时；100 ms 是完整周期验收线，不是求解器 CPU 设置。
GUI 在批量测试中关闭，避免图形负载对两方法不一致。保存配置、世界、源码哈希、遥测、接触和 ULog。

物理成功：七门按序、终点误差 <0.55 m 且速度 <1 m/s、零 Gazebo 门接触、真值采样球体门套净空非负、房间检查通过。
实时成功还要求所有完整规划周期 <100 ms。主表分别报告两者。
完赛上限 60 s；不以失败时的截止时间冒充完赛时间。失败圈依然保留已过门数、速度和计算耗时。
基础设施故障（未获取定位/时钟或未解锁）单列，最多重试一次；算法失败不重试。
数据分析提供完赛时间、平均/最大实际速度、最小采样净空、规划周期 P99/最大、接受率、成功数/试验数。
图：倍率—成功率/完赛时间；实际速度—时间（固定选相位0，不挑最快）；各门局部穿越点与真实开口。
安全证据仅为接触日志与离散采样，不称为连续证书。

## 文献基线边界

用户指定 Beyond Reference Trajectories: A Waypoint-Based Model Predictive Path Integral Control for Agile Drone Racing。
作者主页 https://zhaofangguo.github.io/homepage/publications.html 将该标题链接至 arXiv:2509.14726v1，
公开预印本名为 Rethinking Reference Trajectories in Agile Drone Racing: A Unified Reference-Free Model-Based Controller via MPPI。
项目 https://zhaofangguo.github.io/racing_mppi/ 在查询时仍标注 Code (coming soon)，链接回自身。
公开方法采用单桨推力变化率输入、刚体动力学、gate progress、近门惩罚、跨迭代时间一致过门切换。
Table I 给 M=8192,K=20,dt=.03,lambda=.01；近门权重/阈值、采样协方差及全部约束权重未完整给出。
静态论文设置移植到动态异形门与 PX4 必须说明改动。未运行的官方方法不得填 0% 或画成零性能。
如有独立实现，必须另列名称、模型/接口/参数来源和差异，不能宣称作者官方性能。
