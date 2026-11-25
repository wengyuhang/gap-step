# 全赛道纯规划计时

## 最新：2026-10-12同一90%轨迹已完整返航

按用户授权只对齐跟踪执行模型/时序，显式 `--tracker execution-aligned`：
直接CTBR输入、33状态PX4内环/电机预测、正确D项顺序、采样/指令时刻对齐与因果电机状态估计。
不改规划/参考、场景、物理界、PX4增益或完成判据；原入口可显式保留作对照。
Gazebo/PX4七窗各正向一次、整机离窗，**完整返航55.928s，七窗框零接触**。
真值全程跟踪误差最大.15117m/RMS .07351m，返航段最大.09722m。
终点真值距离.22410m、估计速度.98475m/s，沿用<.55m/<1m/s判据，非精确零PVAJ；
故完成时刻可早于名义56.682964s。MPC最大25.700ms，平均实际73.84Hz。
一次成功不证明普适可靠/连续安全；地面接触未独立计录，最低机体中心高度.66666m。
50测试通过、运行期间源哈希不变、参考/场景/物理XML/PX4参数一致，experiment暂停。
结果 `results/full_pvaj_thrust90_aligned_px4_v1_20261012`，细节见
[执行链对齐说明](../togt_style_mpc/README.md)。以下保留之前的失败与规划记录。

## 2026-10-12 规划推力预留10%：七窗通过，返航仍失跟

用户要求规划动力学上限略低于平台实际能力。本轮仅将规划单桨推力上限从
8.548580 N 调为 7.693722 N（90%），MPC/PX4的实际界、角速度界、代价、初值规则、
4Hz/4m赛道和动力学方程不变；没有同时补气动力矩，也没有延迟/平滑/重规划补丁。
新增显式参数 `--planning-thrust-fraction .9`；缺省1.0保留历史复现语义。

### 结果

| 项目 | 原100%规划参考 | 本轮90%规划参考 |
|---|---:|---:|
| 名义七窗返航时间 | 55.541224 s | 56.682964 s |
| 名义路程 | 592.132344 m | 588.437118 m |
| 首条通过原检查的求解耗时 | 52.019368 s（历史） | 72.851803 s |
| 同版MPC、4ms PX4数值执行模型 | 46.132 s失跟终止 | 完整跟踪，误差最大0.007516 m |
| Gazebo/PX4全程 | 历史五窗后失跟 | 七窗各正向一次、整机离窗，返航失跟 |
| 有效完整竞速时间 | 无 | 无 |

名义时间增加1.141740s（2.06%），不是闭环完赛时间。数值验证额外跟踪1秒终点悬停，
不能把探针运行长度57.684s当竞速成绩。历史全程Gazebo用的是修正热启动之前的版本，
因此两趟Gazebo不能宣称严格仅推力上限不同；同版数值模型对照用当前签名c1e65dd31eb93963。

本轮Gazebo/PX4：原生真值确认七窗各正向一次，第七窗整机退出49.764s，所有七窗框接触记录为0。
但49.924s位置误差超过1m、51.484s超过3m；返航下坠后54.312s中止，最后报QP失败。
结束时距终点21.979m，不能算完成。出第七窗前最大真值误差0.896m，全程峰值15.687m。
地面实体接触未单独计录，真值高度降至约-5.735m；**仅能说无窗框接触，不能说全程无碰撞安全飞行**。
本轮MPC计算中位8.809ms/P95 13.231ms/最大68.180ms，平均实际更新约49.97Hz，未实现100Hz。

### 证据与解释边界

1ms原规划模型复核：最大单桨7.798440N，仍比新规划界高1.361%，但低于实际物理界；
工作空间节点间越界0.017572m照实保留。17节点/段未变，不能声称连续动力学硬约束严格成立。
独立10ms反算恢复桨速相关气动力矩后，最大单桨8.309577N，未见超过8.54858N；
固定原姿态/角速度、平动残差最大0.027875m/s²，不是完整逆动力学证书。
模型闭环完整通过而Gazebo返航失败，说明留余量改善了参考的可执行性，但尚不足以解决实际执行失配。
不能继续只用旧参考的“第六窗单桨超限”解释这次全部失败，也不把最后QP错误当最早原因。

规划Config唯一差异为physical_thrust_fraction=1→.9；场景manifest、物理physics XML、
所有gate XML、MPC配置和PX4参数覆盖项均相同。规划/实飞期间源码哈希未变。
45项相关测试通过（含新上限只改四个上界残差、不改Jacobian和初始曲线的测试）；
另对共享hard_dynamics所属dynamic_passage_velocity_minco补验11项，全部通过，合计56项。
实验已结束，experiment暂停，replay未操作；没有自动尝试80%或修改控制器。

原始数据：

- 规划：`results/full_pvaj_thrust90_f4_a4_v1_20261012/`，含result、independent_audit、restored_torque_audit。
- 数值闭环：`../togt_style_mpc/results/thrust90_execution_model_v1_20261012/`。
- Gazebo/PX4：`results/full_pvaj_thrust90_px4_f4_a4_v1_20261012/`，含ULog、contacts、
  comparison_audit、reserve_experiment_audit、tracking_error_audit和冻结源码。

复现（新输出目录，不覆盖历史）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
/home/jack/anaconda3/envs/wyh/bin/python -m convex_timevarying_window.global_gate_minco_probe.run \
  --full-state --frequency 4 --amplitude 4 --seconds 180 --planning-thrust-fraction .9 --outdir <新规划目录>
/home/jack/anaconda3/envs/wyh/bin/python -m convex_timevarying_window.global_gate_minco_probe.audit <新规划目录>
```

按togt_style_mpc/README设置acados环境后，用本目录flight入口和`--tracker togt-style --native-camera`执行。

## 历史状态

2026-10-12跟踪入口默认改为 `--tracker togt-style`（独立13状态/单桨输入NMPC→CTBR→PX4）。
规划和历史参考不变，旧跟踪器可用 `--tracker px4-model`。新跟踪短程Gazebo已验证，
同全局PVAJ参考首趟5窗后失跟、46.052s中止，并未完赛；不能升级历史名义合格结论。
详情见[跟踪器验证](../togt_style_mpc/README.md)。下述2026-10-11命令若重现旧控制器需显式加旧tracker参数。

2026-10-11后续完整PVAJ实飞：4Hz/4m、原x500/MPC、同一冻结参考轨迹，前五窗无接触，
第六窗46.428s真实撞框，未完成七窗返航，无完整竞速成绩。碰撞前最大真值跟踪误差0.659m。
保留原始接触/恢复ULog及中断审计；全局适配器修正远离有限框的平面误中止，9测试通过。
详见[实飞与消融报告](PVAJ_ABLATION_20261011.md)。未修改规划参数或MPC来掩盖失败。

2026-10-11新增完整PVAJ消融：`--full-state`释放内部A/J，96变量、C3连续，原四条PV初值精确嵌入。
同代码PV/PVAJ纯时间对比：55.562929/55.541224s、762.171973/592.132344m，
路程减少22.31%，名义时间几乎不变；求解98.226/52.019s。规划阶段不含实飞。
场景/Config/源码冻结一致，7测试通过；节点间轻微超限仍存在。
详见[完整PVAJ消融](PVAJ_ABLATION_20261011.md)。默认仍为PV，位置型开关与完整PVAJ互斥。

2026-10-11位置型消融：`--position-only` 使用33变量C6 MINCO，不独立优化内部V，
保留时间和穿越点优化。四初值约69.928s全部线搜索失败，未找到节点合格解，未启动实飞。
不能证明无解；原默认PV版不变。详见[位置型实验](POSITION_ONLY_20261011.md)。

2026-10-11单因素更新：取消七个到达时刻相对初值的±period/4时间箱，其他求解设置不变。
4Hz/4m纯规划路程761.182→762.172m、名义全程55.562665→55.562929s，未改善；
求解97.068s。未接Gazebo，节点间动力学/工作空间轻微违规仍保留。
详见[时间箱消融](ARRIVAL_ABLATION_20261011.md)。下述2026-10-10结果为历史基线。

2026-10-10，用户要求先不跟踪，测我们的规划一次求全七窗需要多久。
默认同刚才4Hz/4m小窗赛道：水平间距2倍、W1侧移-5m、初相0；不启动Gazebo。

起点PVAJ → 七个窗口内自由P/V节点 → 原起点PVAJ，共八段七次MINCO，54个变量。
内部A/J由全局最小snap消元，C5；与当前单窗/两窗P/V约束系统等价，不加停车点。
TOGT圆盘/多边形映射、总时间目标、17节点/段的x500电机/角速度/工作空间界、机体半径/安全余量保留。
时间原点固定为0，求解过程中不推进场景时钟；这是离线名义耗时测试，不是在线闭环试飞。

数值流程沿用可行性恢复→SLSQP时间最小化→局部浮点包络复查→补入违规见证。
全局扩展核对全部7窗×8段，只在各有限边框相关的法向重叠区间细分，不做整轨迹均匀安全扫描。
额外碰框见证进入同一约束。通过优化节点/局部包络不等于连续动力学认证或闭环安全。
四个相位初值沿用当前规则，逐个尝试；第一个收敛且全部复查通过的解即停止，不声称全局最优。

为避免把七份本地JIT编译误当在线求解时间，按链式法则分块计算解析Jacobian，CasADi VM执行；
没有更改物理/安全条件。分别报告建图预热、首条合格解求解、总准备耗时和名义飞行时间。
单次求解上限180s，未找到合格解仅表示本预算/求解器失败，不判物理无解。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
/home/jack/anaconda3/envs/wyh/bin/python -m convex_timevarying_window.global_gate_minco_probe.run \
  --frequency 4 --amplitude 4 --seconds 180 --outdir <新结果目录>
```

显式测试：`python -m pytest -q convex_timevarying_window/global_gate_minco_probe/tests`。

本次4Hz/4m结果：首条通过当前检查103.824s，建图2.753s，名义全程55.563s。
独立1ms复核节点间电机超限0.895%及工作空间越界0.094m仍保留；
不是严格连续动力学可行或闭环完赛。详见[实测报告](RESULTS_20261010.md)。
