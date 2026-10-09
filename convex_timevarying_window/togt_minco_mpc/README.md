# TOGT-MINCO / Checked-Soft / MPC

2026-10-07，新建并列方法。用户要求换掉当前五次 Hermite 曲线，用 TOGT 测试。

最新 v3 已在两个固定 PX4/Gazebo/x500 条件完成：原始 **30.504 s**、10 倍频率 **34.468 s**，
均七窗返航、零接触、真值采样净空为正、在线规划/MPC <100 ms；但比旧优化 MPC 慢
4.868/2.472 s，比 Wang 独立复现慢 4.428/6.920 s。因此不替代旧版本，不宣称提速成功。
详见 [结果与证据边界](RESULTS.md) 和 [原始批次报告](results/px4_v3_lbfgs_20261007/REPORT.md)。

轨迹改为 TOGT 的 MINCO-s4：全局耦合七次多项式、起终点 PVAJ、内部 C6 条件，
门间增加自由三维位置节点，每个飞行区间两段、独立时长；穿越点仍在动态开口内联合优化。
不把高阶 Hermite 或五次参考拟合冒充 MINCO。单位测试与仓库 TOGT MincoSnap 线性系统逐项对比。
CasADi 对归一化 MINCO 线性系统求导；不声称调用了原发布 C++ 优化器。

本版本是明确的组合算法，不是原论文未经改动的复现：保留 Checked-Soft 的时间/jerk/净空目标、
局部安全/动力学限值与 10 ms 整段局部采样准入，以及独立进程、0.2 s 重规划、150 ms 参考接续。
两个未来窗加零 PVAJ 停留终点；头部 jerk 从已接受参考接续，初始悬停取零。
末端零 jerk 和内部导数分配是 MINCO-s4 的变化；不能称除了变量个数之外完全不变。
当前 v3 使用 L-BFGS-B 标量目标/梯度，55 ms 内部墙钟预算，为检查和 IPC 留出余量。
将原离散硬约束的归一化违反量加入三次罚项（固定权重 1e5），执行准入仍要求原残差 <.003，
以及完整局部检查净空 ≥0.35 m。软净空偏好仍为 0.50 m，不用罚项冒充安全保证。
v1/v2 是 IPOPT 65 ms、有限内存 Hessian 的开发版本；源码和负结果保存在各自快照。
jerk/软净空积分在分段网格用梯形权重。
整周期含 IPC 必须小于 100 ms；构图在起飞前单列。两窗通常 6 段、19 个优化变量。
MPC 直接复用优化后的 condensed 17 状态版本，不修改模型、时域、权重或控制界。

预声明两个 PX4/Gazebo/x500 条件：原始 r1/t1/a1/0°，及圈速落后 Wang 的 r10/t1/a1/120°。
旧结果只读：前者旧 MPC/Wang 25.636/26.076 s，后者 31.996/27.548 s。
每版本每条件一次；仅有独立证据的起飞前基础设施失败可重试一次，算法失败保留。
验收沿用七窗返航、无接触、非负真值采样球净空、房间范围、所有在线周期<100 ms。
轻微名义动力学超限单独诊断。两条试飞不是全矩阵、连续安全证明或总体成功率。

开发记录：v1 两条件均起点超时，原因是未执行搜索候选错误扣减已飞时间，已修正并加两项回归；
v2 两条件都七窗返航无接触（31.664/36.512 s），但规划最大 100.550/102.451 ms，实时验收失败。
全约束 Jacobian 本地编译试探产生巨大 C 文件，三次编译试探被主动终止，没有用于飞行；
转为标量 MINCO 梯度后，原始冷启动单次 60.8 ms 找到采样合格候选。该微测不是飞行验收。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/togt_minco_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.togt_minco_mpc.experiment --outdir convex_timevarying_window/togt_minco_mpc/results/new_probe
```
