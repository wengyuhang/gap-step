# 代码架构与模型边界

## 2026-10-08：纯软两窗执行与弹性MPC

最新控制入口已切换tracking_mpc.py：继承数值RTI循环但覆盖目标和约束构造，只有跟踪/平滑与
物理输入盒，没有倾角罚或倾角/trust行。elastic_mpc保留历史回放；旧实飞不作为tracking版证据。

后续修正：soft_only自有scheduler.py恢复跟踪误差事件，px4.py误差请求改用测量时间实际估计PVAJ、
不继承旧参考起点/过时穿越时间；recovery_run.py仅复测七条件自由点。旧simple_two_gate_mpc不改。

`convex_timevarying_window/simple_two_gate_soft_only_mpc/`独立复用MINCO目标，移除名义残差/事件/时限准入。
`execution.py`负责实际计窗与数值有效性；`reference.py`延续原零VA终点供预测使用，不伪造过窗。
`elastic_mpc.py`保留17状态/电机模型，真实输入盒＋倾角弹性罚，替换旧硬倾角/trust QP。
`mpc_replay.py`重放旧失败并保存不可行QP，`mpc_recovery_probe.py`验证失败状态上新控制，不冒充实飞。
`run.py`全部重飞0.5Hz/0.5m三路线，时间含返航，以真实碰撞/完成为主；当前half_step_elastic_v2批次。

2026-10-08最新：simple_two_gate_mpc是完整闭环指标的简化配对入口，复用第三窗终点MINCO几何和缓冲求解器。
Config强制jerk_weight=0、exclude_postrace_time=False、horizon=2；统一初值。Scheduler仅首次/整机出窗触发。
两版只差free_points；run保存full_race_time_s（严格全任务通过才赋值），旧race_time_s仅作W7诊断。
继承的安全违反罚/穿窗加密和事件检查仍存在，不能称只有时间最优化或纯软约束安全证明。

2026-10-08初始化修正：`convex_timevarying_window/third_gate_racing_mpc/`复用third_gate_stop全部数学问题，
仅改变跨窗滚动时新加入段的时间初值，以配置速度上限替代固定6m/s，保留共享窗绝对时刻/PVA。
不增加候选、远端窗口、期限约束或控制模式；第三窗仍精确移动中心/世界零VA。
`initialization.py`无实验数据依赖，`diagnose.py`单独读历史头状态做机制对照。
新Config保留gcc -O1（无fast-math）；曾试-O2未消除超时，现改CasADi FunctionBuffer减少求值包装开销。
父类仅新增可选编译标志读取，旧Config默认仍-O1；缓冲版保留SciPy设置/最终结果，并复制输出梯度避免别名。
在线函数的目标、梯度、容差不变，编译耗时单列。新worker与flight保持原MPC/PX4接口。
后续最终版：冷起交点速度初值为.75倍上限；失败重试从已接纳事件重建，失败曲线仍用于几何细化。
exclude_postrace_time仅在末视界从目标减去真实赛后返航时长，曲线与安全约束不变；早期第三窗仍计时。
上句“目标/梯度不变”特指缓冲求值实现，不适用于这一明确的末视界目标修正。

2026-10-08后续：`third_gate_stop_mpc/`把前两自由窗后的尾段终点精确替换为第三窗未来中心、世界V=A=0。
复用MINCO-s3/加密残差与正常收敛求解，消除9个尾态变量，不含tau估值；局部28变量、3段/6片。
第三窗是到达终点而非已完成穿越，实际出第一窗后继续滚动；剩最后两窗时用真实返航终点。
独立worker/flight注入到原MPC执行器，run只新飞一趟与历史中心比较，不新增中心消融。

2026-10-08：`convex_timevarying_window/third_gate_time_mpc/`为独立两窗＋第三窗到达时间估计。
复用event_free_terminal的MINCO/穿窗加密，但移除共同窗期限和旧best-iterate求解；
第二窗后短尾精确投影到带原跟踪裕量的出口平面，tau仅查询第三窗未来中心，不生成第三段航路。
ConvergedSolver只返回L-BFGS-B最终解，成功停止与原残差/事件/100ms准入分别核验。
`flight/worker/px4`隔离接入原MPC；`run`只冻结1Hz/2m/0相的自由＋估计、中心＋估计、自由无估计。
不把该方案称为全程安全或优于中心的理论保证；旧完整赛程版本保持。

更新：2026-09-10，依据 `40692e8` 与本地目录。现状/结果见 [PROJECT_CONTEXT](PROJECT_CONTEXT.md)，具体命令见 [RUNBOOK](RUNBOOK.md)。

2026-10-07补充：`复现/polynomial_dynamic_racing_2023/paper_core/`是独立论文数学实现，
`minco.py`构造归一化轨迹，`model.py`给非线性阻力/窄缝/椭球，`planner.py`构造公开目标和动态中心等式，
`solver.py/native.cpp`连接固定版本LBFGS-Lite。核心没有我们的竞速规划或安全过滤依赖。
与旧`paper_aligned`的x500匹配基线分开；核心基准不是七窗飞行，见其README/RESULTS。

同日后续新增并列`复现/polynomial_dynamic_racing_2023/px4_fixed_plane/`：`scenario`固定原七窗平面并
定义面内中心平移；`adapter`包裹纯paper_core滚动接口和明确末态/初值规则；`reference`保留按时刻
可查询的旧参考；`flight`只复用公共PX4运输/MPC；`probe/audit`独立核对实际门姿态/飞行真值；
`matrix`冻结六条件。公共运动插件新增可选分量相位，不指定则保持旧世界默认运动。
该适配已实际执行，但六次第一窗前MPC中止；不能把存在PX4入口等同全程飞行验证通过。

## 目录组织

`comparisons/comprehensive_wang/`冻结单次条件90＋45飞行：registry定义唯一清单，flight/adapter
只包装当前方法和3个单因素消融，audit按七窗竞速前缀与完整返航分开验收，run做串行调度/
设施重试/哈希校验，report生成经验成功比例/配对速度/延迟与穿窗点图。
`复现/polynomial_dynamic_racing_2023/px4_matched_x500/`独立包装paper_core，不改其公开方程；
明确投影固定尾态、边界一致冷初值、rho选择和LBFGS错误码处理；qualify/readiness只验证接口。
数学核心和旧基线全部保留，不能将该x500适配称作者完整实现。

`convex_timevarying_window/crossing_resolved_mpc/`为独立穿窗邻域几何加密方法：
复用Wang MINCO构造/求解与deadline封装，在原代价/约束上追加每窗事件两侧≤20ms节点，
新残差同步进入求解器和接纳，validate保持明确未检查。
`comparisons/original_fixed_plane_crossing_resolved/`提供隔离worker、固定平面场景、三次重复
与真值审计入口；测试保存实际碰撞候选，不读取窗号或场景标签来调参。

`convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/`仅配置消融：
`config.py`覆盖软余量/权重，`flight.py`进程内注入到既有no-postcheck控制入口，
规划worker/优化/事件/MPC均复用；`run.py`冻结双条件并断言只改变两个配置字段。
不修改任何旧算法默认。采样软惩罚不是安全证书。

`convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/`是独立仿真消融。
`planner`仅覆盖validate返回未检查None；`px4`是原接口的声明差异副本，只移除该准入并标记日志，
原优化/补点/事件/MPC不变；`postcheck`仅在飞行后用原检查核对已接纳多项式，不参与控制。
旧默认入口保持，不把该消融作为安全部署版。

`convex_timevarying_window/comparisons/original_fixed_plane_deadline/`是旧`wang_progress_mpc`在当前原布局
的隔离场景接口；覆盖符号几何、继承的密集净空/补点及事件坐标，原规划/参考/MPC保持。
保存共同窗时刻契约日志并复用独立真值审计，不把原Checked与其当只差deadline的消融。

`convex_timevarying_window/comparisons/original_fixed_plane_checked/`只适配旧Checked/PVA与新原布局场景，
`adapter`替换符号/向量查询，`flight`调用原`--checked`入口，`audit`保留4ms严格真值但不伪造MPC日志，
`run`冻结两条件。原Checked在线中心事件逻辑保留，事后独立核验整机退出/反穿/空间和安全球余量。

`convex_timevarying_window/wang_free_terminal_mpc/`是用户明确指定的自由局部末端PVAJ扩展。
`planner`独立保留原目标/检查的构图，继承原求解/准入接口；`trajectory`以末端J替换最后内部点，
不改变五次MINCO族。`worker/flight`只注入新规划器和共同固定平面查询；`run/audit`冻结两条件并
核验实际多项式末端。旧`wang_paper_aligned_mpc`与Wang基线源码未改，不把旧固定末态规则称用户要求。

`convex_timevarying_window/comparisons/original_fixed_plane_ours/`是我们既有free_warm对上一轮
原布局固定平面输入的场景接口，不是新规划算法。`adapter`在隔离工作进程中替换运动查询，
`flight`调用原PX4入口，`run`冻结六条件并对齐世界/参数、复用独立真值审计；
旧规划器/控制器源码保持。当前六次均停在W1后，固定末端可行性规则尚未修复。

```text
时变窗口/
  AGENTS.md / CLAUDE.md          现行约定与镜像
  docs/                         状态、架构、命令、决策、计划、历史
  nonconvex_timevarying_window/
    PROBLEM_DEFINITION.md       基础非凸按序穿越问题
    atlas_dynatogt/             三角 chart atlas + Hermite
    sc_dynatogt/                SC 参数化 + degree-7 MINCO 基础实现
    random_dk_sc_dynatogt/      独立均匀 D/K 扰动基线
    feasibility_guided_cem_sc_dynatogt/  安全相位前端 + 完整协方差 CEM
    msr_dynatogt/               SC 多初值 + 采样动力学修复
    sip_dynatogt/               witness 优化 + Arb 连续域认证
    planar_rs_dynatogt/         固定平面旋转/缩放的认证加速
    rot_sync_sc_togt/           自旋窗口的解析同步段 + MINCO
    interpolated_rot_sync_sc_togt/  双 SC 输入插值同步段 + MINCO
    avs_ppo/                   动作掩码/可恢复盾牌 PPO
    phaseguard_rl/             点/时间 PPO + 认证准入
    comparisons/               SC/SIP 压力场景与速率基准
      seven_unique_dual_constraint_cem/gazebo/  七异形闭环 Gazebo Harmonic 世界与高保真网格
    废案/                       WBSC、CWB、Exact-Area 历史方法
    实验方案/                   本地 ICRA 方案，审计时未跟踪
  closed_loop_deformable_window/
    PROBLEM_DEFINITION.md       安全区可为空、机会选择、完整初态返回
    fapp_ppo/                  未来预览残差 CTBR 控制
    mdg/src/mdg/               移动圆盘图规划
  convex_timevarying_window/    凸多边形/精确圆 + 周期三维位姿 + TOGT/MINCO
  togt_timevarying_window/       独立 DynaTOGT/Hermite 动态窗口实验
  gap_step/                     二维生成迷宫与历史训练入口
  复现/                         本地 TOGT 外部复现，Git 忽略
  非凸窗口/                     早期问题/概念材料
```

方法独立组织，但允许显式复用基础能力，不能据目录并列就宣称源码完全互不依赖。新方法不把源码散放在研究总目录根部。

## 2026-10-07：Wang MINCO 自由穿窗状态

`convex_timevarying_window/wang_center_seed_mpc/` 是后续无中途停车的独立三窗版本。
`trajectory`直接Wang自由内部位置坐标；`planner`中心初值、自由窗P/V/A、无窗间零速终端，
仅实际返航goal有停止终态；`reference`保留时间线/阶段一致，移除所有过窗时间不退步约束。
`events`区分最后预测窗到达平面与前面窗完整离面；末端未延伸时不能作为悬停参考。
`worker/px4/flight/experiment`独立入口复用原MPC/场景/审计，源码冻结于新结果，不改旧基线。

`convex_timevarying_window/wang_progress_mpc/` 独立扩展事件v4：`contracts/reference`按共同窗口身份
比较到达时间，`planner.DeadlineSolver`把公共截止时间加入原有界搜索；`worker`仅在新子进程注入
新规划器，`flight`复用原PX4/MPC和固定平面适配，`experiment`独立冻结两预声明开发条件。
原`wang_free_state_mpc`与Wang源码不改，旧矩阵仍可按原版本追溯，不混入新结果。

`convex_timevarying_window/comparisons/fixed_plane_wang/` 独立实现固定平面、仅中心水平平移的新七窗赛道。
`scenario/flight`显式注入同一解析运动与结果目录世界，不改当前事件v4或Wang独立复现源码；
`audit`也绑定新场景，不能直接拿旧构造器审计新日志。`run`编排12条件×2方法，
`probe`只读核对Gazebo实际门框位姿，`report`保留失败并生成配对对比与穿越位置图。
这是当前MPC整套方案对Wang中心点/PVA整套方案，不是只选点自由度的消融。

`convex_timevarying_window/comparisons/wang_free_state_mpc/` 独立编排当前事件v4的38条件矩阵。
冻结63项既有飞行源码，另存编排/协议哈希；报告器只读复核历史ULog全事件，并保留旧验收标签。
不把实验包装器当作新的飞行算法，也不将旧基线改成当前事件约束后重跑。

`convex_timevarying_window/wang_free_state_mpc/` 直接使用 Wang 独立复现的
`normalized_inverse(2)`：每飞行区间两个等时长五次 MINCO-s3，内部 C4、过窗 C2。
`trajectory.from_arrival_jerk` 把原自由中间点等价变换为到达侧 jerk；位置、速度、加速度
仍独立，窗口外生运动不变。无额外 C6 或零末端 jerk；不声称过窗 jerk 双侧连续。
规划器复用旧 TOGT 组合的几何/安全检查与有界标量优化器，但完全覆盖七次轨迹构造/初值/求解。
同一 condensed MPC；`--center-only` 只把窗内二维坐标上下界设为零，作为匹配控制器/检查的对照。
历史 Wang PVA/短前缀结果只列历史参考，不视作单因素消融。
后续事件修复将 `Reference` 独立到 `reference.py`，统一实际参考/未来头部时间线；
`events.py` 管理中心穿越→整机脱离、近窗不重置、局部单次穿窗检查；
`passage_audit.py` 独立审计全部真值有限窗口事件，防止顺序首次计数漏检重复穿窗。
`Progress.observe` 使用新鲜位置样本的时间查询动态窗口；遥测新增 `state_time`，
与用于主循环调度的 `t` 分列，避免不同步时间产生假反向穿越。
MPC/原复现源码不改，历史 v1 与修复各版本结果分别冻结。

## 2026-10-07：在线 TOGT-MINCO 组合方法

`convex_timevarying_window/togt_minco_mpc/` 为并列试测：`trajectory.py` 实现归一化七次
MINCO-s4 全局线性系统及完整八系数采样；`planner.py` 联合自由门点/门间点/段时长，
以 CasADi 标量梯度和 L-BFGS-B 搜索。三次罚项用于求解，独立采样准入仍保留，不能当安全证明。
`worker.py` 使用持久 spawn 进程与有类型 Plan IPC；`px4.py` 接入同一
`checked_soft_mpc_isolated.fast.mpc`，不修改 MPC 模型/权重/控制界。
`experiment.py` 冻结两个条件与源码，调用现有接触/真值净空审计。
头尾 PVAJ、内部 C6 和额外自由节点不同于旧单段五次 Hermite；也不等于发布版 C++ TOGT
优化器直接复现。结果见方法 `RESULTS.md`，旧方法/矩阵保留不替换。

## 非凸规划与认证

| 实现 | 主要入口与数据流 |
|---|---|
| Atlas | `geometry.py` 的 ear clipping/chart → `environment.py` → `optimizer.py` → `experiments.py` / `visualize.py` |
| SC | `boundary.py` → `offset.py` → `sc_mapping.py` → `preprocessing.py`；`environment.py` + `minco.py/dynamics.py/time_mapping.py` → `optimizer.py`；`experiments.py` 与 `diverse_demo.py` 分开 |
| Random-DK | 复用 SC/MINCO 前向轨迹 → `search.py` 一次性独立 D/K 候选 → `safety.py` 球体接触区间与动力学短路筛选 → 单窗或多窗最终整机审计 |
| Feasibility-Guided CEM | 单窗已验证 `(相位,D)` → `search.py` 周期别名前端 → K 时间尺度归一化、D 极坐标与完整协方差 CEM → 复用 Random-DK 硬筛选和多窗最终整机审计 |
| MSR | `initializations.py/candidate_pool.py` → 复用 SC 求解 → `feasibility_repair.py` → `solver.py`；`comparison.py/experiments.py` 记录匹配预算比较 |
| SIP | `model.py/constraints.py` 定义真实边界和整机 → `solver.py` 的 SLSQP witness 循环 → `intervals.py/certificate.py` 连续域细分；`io.py/verify.py` 序列化与重放 |
| Planar-RS | `model.py/scenario.py` 限定运动模型 → `certificate.py` 的平面严格排除与 SIP 原始曲线检查 → `solver.py`；`verify.py` 独立重放 |
| RotSync | `geometry.py/scenarios.py` 建立自旋门 → `trajectory.py` 的固定点 Sync、PVAJ 与 MINCO 拼接 → `optimizer.py`；`collision.py` 密集整机截面审计；`single_window_comparison.py` 单独维护 |
| Interpolated-RotSync | 独立目录的 `trajectory.py` 在双 SC 输入间插值并解析到 snap → `optimizer.py` 以入口/出口 PVAJ 连接共享 MINCO/L-BFGS → `experiments.py` 单独导出结果；几何、场景、碰撞和可视化通过明确接口复用 RotSync |

SC 的 Chang 方法只用于边界均匀重采样和角点保留。内部点来自 Schwarz–Christoffel 圆盘映射 `q(d)=Psi(B(d))`，不能写成 Chang harmonic measure 或 Atlas 三角 chart。SC/SIP 基础变量为 `[K,D]`；RotSync 使用 `[K_free,K_sync,d]`，独立的 Interpolated-RotSync 使用 `[K_free,K_sync,d_entry,d_exit]`；旧 WBSC 的 `[K,D,Y]` 属于废案。

SIP 支持真实 Line/CircularArc/Bézier/非有理 B-spline 边界原语；用于映射的采样多边形和认证的原始曲线是两种数据，不得混用。SLSQP 给候选，Arb 完整有限覆盖给认证状态，优化器成功字段不能替代认证。

Planar-RS 要求固定中心/平面，允许面内旋转和统一缩放；RotSync 与 Interpolated-RotSync 都仅绕法向匀速自旋。RotSync 保持窗口局部点不变；Interpolated-RotSync 在无约束 SC 输入空间线性插值，再经 `B` 和 `Psi` 得到随时间变化的局部点，不是连接两个实际位置。两者都以 PVAJ 接入相邻七阶 MINCO，不是 MINCO 拟合 Sync。

`convex_timevarying_window/` 是并列问题目录。`geometry.py` 对多边形使用发布版 TOGT
单位球归一化、平方重心权重映射，对平面圆使用 `Ball::toP` 的二维同构映射。`togt/` 保存
名义基线：`togt/native/togt_analytic.cpp` 直接调用发布版 C++ 四旋翼手写梯度和 MINCO 伴随
回传，`togt/native_objective.py` 补齐动态窗口和 `[K,D]` 链式梯度；安全不进入名义目标。
`conditional_dual_constraint_cem/` 在名义安全积分非零时增加手写凸安全梯度，再以双软约束
完整协方差 CEM 搜索 `[K,D]`。动力学最终验收经同一原生库调用发布版C++ `QuadManifold`，
安全最终验收使用真实外接球到物理门框的距离。两种方法都以最大1 ms步长密集采样，不经过非凸SC
预处理，也不在当前求解路径建立 PyTorch 自动微分图。

2026-10-06新增 `convex_timevarying_window/checked_soft_racing/` 是在线 checked 的并列
目标函数变体，而非以上离线原生TOGT。依赖链为 `receding_gate_racing.Planner`
→ `continuation_racing.Planner` → `continuation_racing.guard.CheckedPlanner`
→ `checked_soft_racing.Planner`；前三层保持原约束/接续/检查，新层仅覆盖默认零
`extra_stage_cost`。五次Hermite、两窗加停留终点、CasADi/IPOPT与PX4 PVA接口不变。
共享PX4入口通过 `--checked-soft` 选择；跨方法冻结运行/只读报告在
`convex_timevarying_window/comparisons/checked_soft_racing/`。此依赖链的共享修改须显式回归父方法。

同日新增 `checked_soft_mpc_racing/` 是控制器消融：规划器直接导入Checked-Soft，
独立PX4入口复用Worker/场景与CTBR Session；`mpc.py`不访问任何窗口信息。
17状态预测、CasADi离散模型/雅可比、OSQP逐次线性化与非线性回溯，输出总推力/角速度。
平台坐标与遥测沿用CTBR接口，原规划和PVA入口不改；模式交接在既有准备期完成。
不包含CBF或新的安全证明，真实门框接触、真值采样及计算时限独立审计。

同日优化版 `checked_soft_mpc_isolated/` 保留上述模型和规划，使用持久spawn进程独立
运行Checked-Soft，避免IPOPT进程CPU限制计入MPC线程；显式序列化最后实际采用的计划，
100 ms准入计入IPC和接收延迟。可选`fast/mpc.py`解析消去线性化状态，将QP由542维
缩减到100维，保留目标、动力学、输入/倾角/信赖域约束和非线性回溯。
正式38条件运行/对比在`comparisons/checked_soft_mpc_optimized/`，旧方法和结果不覆盖。
轻微名义动力学超限按用户要求只作诊断，接触/净空/任务完成与实时验收保持不变。

## 强化学习路径

AVS-PPO 的 `geometry.py/environment.py` 提供动态非凸门交点和动作可恢复性检查，`model.py/ppo.py` 在状态相关掩码支持集上训练 categorical 策略。`train.py/evaluate.py` 的三窗平移模型与 `train_hardest.py/hardest_evaluate.py` 的六窗长方体压力试验分开，不能共享一条无条件“100% 安全”的摘要。

PhaseGuard-RL 的实际核心链为：

```text
model.py 的相位/状态观测与连续动作
  -> planner.py：点/时间转为固定 MINCO 轨迹
  -> shield.py：复用 SIP 原始曲线检查，按绝对起始时间检查
  -> SafePlanManager：仅接受认证候选；拒绝时保留已有认证计划
  -> environment.py / train.py / ppo.py：一步完整规划训练
```

它复用 SC 的 MINCO 和 SIP 的模型/认证内部接口，但不调用 SIP 优化器；没有既有认证计划时禁止起飞。当前不是完整物理飞控跟踪仿真。修改 SIP 认证内部接口时须考虑 PhaseGuard 的依赖。

## 连续形变问题族

FAPP-PPO 的 `scenario.py/geometry.py` 生成独立外生的开放日程、位姿与局部边界形变；自然三次样条作用于位姿/有序边界点。`environment.py/dynamics.py` 执行按序穿越、残差 CTBR 和刚体/推力分配。模型细节由 [WINDOW_MODEL](../closed_loop_deformable_window/fapp_ppo/WINDOW_MODEL.md)维护，不再复制到全局记忆。

MDG 的 `src/mdg/dynamic_gate.py` 提供 PCHIP 窗口接口；`safe_disks.py/disk_tracking.py` 构造和追踪内含安全圆盘；`time_graph.py/dynamic_programming.py` 选择机会序列；`backend_adapter.py` 复用 SC/MINCO；`planner.py` 管理 Lazy Repair。它是同一问题族的离线规划方法，不是 FAPP-PPO 子模块。

两方法均需区分物理开口与安全内缩区，后者可为空；“闭环任务返回初态”“反馈控制闭环”和“名义轨迹连续域认证”是不同属性。

## 二维与早期 TOGT 入口

GAP-Step 生成窗口迷宫路径为 `window_maze_env.py -> GraphObs -> model.py/ppo.py`，配套 `train_window.py/evaluate_window.py/visualize_window.py`。静态墙体、带动态开口的线/折线/曲线窗口及 swept-circle 碰撞保持连续几何语义。图接口只属于相关图观测代码，不是所有算法统一观测契约。

旧 `env.py/train.py/evaluate.py/visualize.py` 与 passage 入口仍在。`train_window.py` 保留可选 planner auxiliary 路径和 BC 相关文件；默认 teacher/smoke 配置不启用辅助。PPO 正确同步方向是 **`model_old <- model`**。相对配置和结果路径通过 `gap_step/utils.py` 解析到 `gap_step/`。

DynaTOGT 由 `geometry.py/environment.py` 描述动态窗口，`optimizer.py` 热启动并连续优化，`trajectory.py` 使用 Hermite，`baselines.py` 提供对照，`visualize.py` 输出中文图像。`ordered_dynamic/static` 的 `--order` 是可重复任务序列，`shuffled_dynamic` 是一次性排列对照。

## 结果、显示与测试

SC 的 `visualization.py` 生成物理门框图和轨迹图，`simulation_render.py` 用 EGL/OpenGL 回放已保存轨迹，`results_manager.py` 管理分类结果、manifest 与主页。显示层不改变求解几何或提供飞控/传感器认证。历史结果目录保留 `experiments/demos/diagnostics/work` 分类与迁移校验。

SC/SIP 宽域比较中的本地 `gazebo/` 是单独适配层，目前平移/RPY 与均匀缩放支持不同；其 README 中明确缩放和飞控接入的缺口。七异形闭环比较另有独立 Gazebo Harmonic 场景，直接从冻结物理边界生成碰撞网格，以原生转动关节执行固定平面法向自旋。方法渲染、Gazebo 世界与实际动力学执行各自记录。

根 `pytest.ini` 只自动收集 `gap_step/tests`。研究方法及比较目录都有自己的测试目录；影响共享代码时按依赖选择回归，不能用裸 `pytest -q` 代替。
