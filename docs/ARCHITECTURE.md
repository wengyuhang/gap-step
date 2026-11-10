# 代码架构与模型边界

## 2026-11-09结果内三项补飞执行与来源合并

higher_swing_incomplete_completion_v1_20261109/run_frozen.py复用原matrix.collect_flight与高摆动执行器。子进程通过taskset固定控制CPU、容器通过docker update固定独立CPU组，ObservedSession仅旁路记录read年龄/有限值，原返回值和保护保持。三条参考从原矩阵字节复制，不重新规划。新50格汇总为原47格加三条单独补飞来源，原50趟全文件保护；统计同时报告条件覆盖和累计53次真实执行。三项已完成全部实飞与额外实际开口/档案审计，生产算法源码保持。

## 2026-11-09高起伏完整五方法流水

higher_swing.matrix复用matched_full_matrix的候选复查/择优/实飞归档函数，独立协调50项。matrix_worker调用原higher_swing自由/中心、原matched TOGT或原Wang rollout，只注入新场景并保存传输元数据，原Wang config另存wang_config。matrix_flight接同一个完整参考执行器，统一新场景和窗运动审计，五方都按全参考时钟结束。matrix_report生成全新五方法同口径两表，不引用历史Wang时长。具体设置见higher_swing/MATRIX_PROTOCOL.md。 本批340候选、50新规划/50正式飞行已结束，37实飞完成、10实际窗框碰撞、3执行中止均保留。原生与源一致性及37条实际开口独立重算通过，结果单列。

## 2026-11-09独立高起伏单摆场景

higher_swing.scenario通过replace生成仅swing length2.2m的赛道副本。原解析Motion/pose/bounds直接复用参数，world writer只把每个plugin的length同步为各窗motion.length；launch临时注入writer后恢复。run.py复用已验证best_center_release协调流程并冻结新模块源码，worker和校验都构造新场景。flight.py将原执行器和原生窗运动审计切到同场景，要求参考携带higher_swing_length_2p20标记，复用原MPC/PX4与完整时钟。规划与GUI执行各一协调，后者等待最终规划，旧结果保持。 本轮17候选与一次GUI实飞已结束，19.172615秒参考实际19.186秒，7/7整机退出、零接触，原生运动/参考/控制参数审计通过。

## 2026-11-09匹配参数的全矩阵流水

matched_full_matrix/togt.py复用原Objective和ConfiguredMINCOCore(.95)，私有safe域采用当前point_apertures，MatchedObjective在center模式用零局部点逆参数替代均匀权重。worker派发本文原best_center_release.worker或单初值动态TOGT。run.py按条件/初值调度16单线程进程，各中心/TOGT完成8条后汇总；本文8独立自由和8中心结束后仅启动一次release_best，再按原目标择优。所有本文候选完整复查，TOGT按原收敛目标选解再统一STL分类，找到的参考均正式飞行一次。report.py分别导出五方法规划/实飞表，Wang原数据和旧计时†保持。


该批已完成330候选、40规划与40正式实飞。动态TOGT中心零点逆参数、全部四方初始曲线等价和动态目标/梯度回归通过，完整轨迹分类与物理执行分开。五方法表、全部选择和原生日志复核通过，当前无任务worker。
## 2026-11-09最佳独立0.25/6参考闭环执行

independent_best_center_a25_f6_gazebo_v1_20261109/run_frozen.py复用上一95%内缩参考飞行流程，只更换选中来源为新批best_result/ours_3及条件a25_f6。冻结原系数和时长，调用no_tilt_comparison.flight与原radius_ablation.execution，完整参考时钟终止、真实七窗整机退出、原生八接触/ULog/窗位姿恢复重审。一次正式执行实际16.418秒、零接触，原世界/场景/MPC/PX4一致。此次生产规划器和控制器保持原实现。

## 2026-11-09独立自由点加最佳中心释放入口

soft_dynamics/best_center_release.py的run动作冻结新批次，16路worker分别完成同8中心初始曲线的自由/中心优化。协调完整复查原56几何/STL/顺序端点/内缩归属后，选择目标最低的中心轨迹，worker将8时长与63VAJ之间插入25个中心点逆映射变量，系数与目标等价核对后只进行一次release_best。原始独立阶段从不读中心优化解，最终从独立自由和释放合格候选按原目标选择，所有失败保留。

## 2026-11-09空间初值面积采样入口

spatial_initialization.Planner继承逐窗时间初始化，仅覆盖initial_points/seeds及来源标记。圆采用R√u和2πv，多边形按内部分块三角形面积选择后采用平方根重心坐标，生成内缩区域点，原TOGT逆映射将点转为优化参数。相同到达时间下重新查询位姿、构造世界点和MINCO PVAJ，正式模型和约束保持。8独立worker的完整原始初值/局部点/曲线记录在spatial_phase_minco_eightstarts_v1_20261109，批尾另组装保留样本7中心释放的策略汇总。

## 2026-11-09中心优先按样本分支策略

center_first_phase_eightstarts_v1_20261109/run_frozen.py根据已完成八中心的有效性逐组分支。有效中心由worker转成自由96变量初值，全部时刻/PVAJ与物理曲线保持；其余组读取同源原始自由记录并重新复查。释放失败可采用同组原始独立结果。唯一新release_7成功19.169840秒，最终8组4有效，源和原记录保持。各新/复用轨迹audit在本目录，来源和历史成本明确分列，生产源本轮未改。

## 2026-11-09逐窗时间相位初始化入口与验收完成

phase_initialization.Planner继承原内缩Planner，只覆盖initial_times/seeds与来源元数据。原几何旅行估计和6.5/12m/s生成分段区间，Sobol8样本分别决定每段旅行时间与逐窗相位，最近可用周期和原正时长界处理到达顺序。每组重新查询窗口中心并解析MINCO VAJ。原正式优化、目标和几何验收复用原实现，15回归通过。独立16路批次已complete，best_ours_result为21.030547秒，best_center_result为19.231067秒。first_finite_inclusion_check仅做中心曲线在自由有限模型中的只读求值，未新增求解或输入已有优化解。

## 2026-11-09最新八初值自由点参考执行完成

margin95_inset_free_best_gazebo_v1_20261109/run_frozen.py导出eightstarts的best_ours_result，逐值核对等于ours_4原记录并保留原config的95%软界/.15m安全参数，使用原no_tilt_comparison.flight与radius_ablation.execution，同一MPC/PX4和世界。保持原参数之外的执行接口，原生日志归档恢复重审、全部SHA通过。原规划和源本轮未改，实际20.109秒结果在summary/REPORT，首次正式执行完成。

## 2026-11-09八初值扩展与中心初始化对照完成

新增八初值worker沿用95%与内缩Planner，父进程保留旧八结果后在16候选中择优。center_release仅将两个中心结果的8时长、63内部VAJ与25个中心点逆映射拼接为96维自由初值，系数和目标逐值等价。两条在补点后失败。failed_finite_reference_evaluation只读重建见证，确认各输入中心仍满足失败有限集。正式来源标记修正为center/external=True，原输出及SHA保留。两批均结束，生产源本轮保持。

## 2026-11-09更强安全距离与内缩选点参数化

soft_dynamics/inset_geometry.py为可选Planner子类，构造前沿用现有Config factory注入passage_margin增量，确保原几何与factorized几何采用同一距离。point_apertures仅为决策映射副本，原physical track不修改。构造point_function和解析初值时临时使用对应内缩TOGT映射/逆映射并恢复原模块函数。kernels的平面节点直接约束读取planner.point_apertures，缺省沿用物理开口，避免新映射与旧节点表达不一致。两条有限几何的值及Jacobian、固定中心、全参数内缩归属和原窗口保持均有回归。独立8路worker/协调与冻结协议在margin95_clearance5cm_inset_paired_v1_20261109。

该批已complete，best_ours_result保存23.427409秒参考，best_center_result保存22.977863秒参考。validation记录8初值独立复现、同初始时长、源与旧记录保持。五个合格候选各保存原始结果、assessment和validation，三个未找到保留。

## 2026-11-09独立24初值最佳轨迹实际飞行完成

free_24_parallel_best_gazebo_v1_20261109/run_frozen.py只包装normal_free_24_parallel_a75_f6_v1_20261109/best_result，核对其等于所选05_center原result并保持原数组。执行调用no_tilt_comparison.flight与radius_ablation.execution，同一33状态MPC/PX4和原场景。主进程持原Gazebo互斥锁，原生日志无损压缩后恢复重审，逐项核对历史世界/场景/MPC/PX4及源/旧记录SHA。当前已complete，summary与REPORT记录实际24.433秒，生产方法文件本轮保持。

## 2026-11-09仅自由点24初值同时独立求解

normal_free_24_parallel_a75_f6_v1_20261109/worker.py固定free_points=True，以单个temporal_scale实例化Planner，解析生成当前冷曲线；center_initial只修改未优化空间初值并重新解MINCO。run_frozen.py一次Popen24个进程，不维护中心方法。每个进程独立持有NLP/乘子/见证集；父进程验证全部安全和任务后择优，生产方法源码不变。

批次已complete，summary包含24条、best_result保存05_center原始曲线。父进程复算每个合格候选目标/56几何/STL/任务，批尾逐一复现24条解析初值并核对各异，validation保存原源/旧结果SHA与并行计时。REPORT逐初值列出失败及合格解，当前参考24.421905秒。

## 2026-11-09正常独立求解与并行对照完成

normal_independent_diagnosis_a75_f6_v1_20261109的worker/run_frozen只依赖生产Planner及解析MINCO，12进程同时启动，分别独立求解三初始化组。ANALYSIS.md、finite_history.csv、first_finite_inclusion_control.json及center_cross_evaluation.json记录早期较差局部解和后续快分支丢弃。生产方法文件本轮保持，所有物理约束与半径原口径保留。

## 2026-11-09正常独立求解并行原因对照

normal_independent_diagnosis_a75_f6_v1_20261109/worker.py独立构造Production Planner，三组只在free_points及解析初始点不同。free_center_initial用几何中心与MINCO直接构造未优化初值，点变量25保持开放。所有NLP通过同一factory去掉三tiny warm push覆盖，原模型/目标/约束不变。run_frozen同时启动12进程，BLAS各1，按原安全/STL核对并记录物理初值一致性。生产方法文件保持。

## 2026-11-09更短自由点轨迹已实际飞通

free_refinement_a75_f6_v1_20261109的worker/manage复用原Planner/fast_ipopt/全域复查，三个数值对照及14周期邻居共17个候选，当前均结束。最佳gate4_earlier原目标最小，额外polish目录不替换它。free_refined_a75_f6_gazebo_v1_20261109只导出所选原始数组并调用原no_tilt_comparison.flight，参考无重定时或重规划，原CTBR MPC/PX4。生产方法文件本轮未编辑。

## 2026-11-09可飞自由点解的进一步优化

新优化仅在results/free_refinement_a75_f6_v1_20261109的worker.py/manage.py隔离运行。worker复用生产Planner与fast_ipopt，每个有限问题使用正常内点初始化；canonical模式在数值起点选择同一物理圆点的内支预像，exact模式使用同NLP精确二阶导。生产文件保持。manage以/proc实际参数识别既有worker，最多四进程，接收初始三对照及14周期初值，避免重复执行。

## 2026-11-09新续解轨迹实际飞行完成

continued_free_a75_f6_gazebo_v1_20261109/run_frozen.py仅包装导出诊断selected参考并调用原no_tilt_comparison.flight，系数/时长/PVAJ逐值保持。原radius_ablation.execution及同一MPC/PX4，未重规划、未重定时。一次执行与归档恢复重审完成，正式记录在flights/attempt_1。

## 2026-11-09有限NLP失败机制诊断完成

诊断目录free_center_solver_diagnosis_v1_20261109包含原39调用见证重建、原始曲线数值推入控制、重构乘子辅助控制及12轮续解。cross_evaluate_final_nodes.py逐曲线交叉不同见证集。生产源保持。已定位fast_ipopt.py第85至88行热启动三个1e-8与global_gate_minco_probe/planner.py第315至317行break。kernels.geometric_constraints的radius包含witness_slack，实验2.Planner.verify的radius不含该5mm。完整结果与口径差异写入REPORT。

## 2026-11-09有限NLP失败机制隔离诊断

本次诊断全部放在实验2/results/free_center_solver_diagnosis_v1_20261109，analyze.py重建逐轮见证与固定失败NLP，passage_analysis.py计算相对运动，restart_finite/original_primal_probe/single_push_probe只在独立进程覆盖数值启动选项。continue_exchange.py在隔离诊断中使用原目标及原56几何继续补点。生产soft_dynamics及global_gate_minco_probe本次未编辑。现有break位置为global_gate_minco_probe/planner.py第315至317行，热启动推入量为fast_ipopt.py第85至88行。

## 2026-11-09独立时间初值扩展验证

实验2/planner.py的seeds从可选temporal_scales读取比例，缺省原四比例。soft_dynamics.Planner接受可选正有限时间尺度，worker新增--time-scales，并记录number_initial_starts与all_starts_attempted。图、动力学软项、安全交换、free96/center71维及IPOPT保持。圆窗压缩映射/参数界、精确Hessian选项及解析中心初值原型已撤回生产入口，源快照在各开发results。

## 2026-11-08圆窗参数化修正开发

soft_dynamics/fast_ipopt.py在数值z与原轨迹x之间增加圆窗一一坐标，solver_point负责保持原物理曲线的逆变换。原TOGT共享point_mapping不变。圆窗分量界从safety.frame_aperture及原平面安全半径推出，中心无选点变量。图维数、原目标与完整PVAJ不变。

## 2026-11-08明确中心方法定义，完成140个名义穿越节点核对

用户强调中心是严格固定穿越点，必须从决策向量消除选点变量，任意优化时刻节点位置均为窗口运动查询c_i(t_i)。该定义适用于本轮本文中心与动态TOGT中心，不能解释为仅初值在中心。

源码和十条件原始结果已只读核对。本文中心71个轨迹变量=8时长+63内部V/A/J，选点变量0，point_mapping.dimension=0、symbolic=二维零向量，所有点对V/A/J的位置Jacobian为0。动态TOGT中心仅8个时长参数，选点变量0。两方法共140个名义穿越节点，导出点与对应中心误差0，多项式端点误差最大2.4932413305e-12米。冻结源码SHA一致。centre_definition审计位于results/free_center_gap_analysis_v1_20261108/center_definition_audit.json。

同目录REPORT、audit和crossings.csv分析时间差。0.25/6自由与中心路程167.109/176.331米、平均速度9.833/9.843米每秒，路程缩短5.23%、时间缩短5.13%；自由点平均偏移0.795米，最大1.131米。两版共享时刻及完整V/A/J优化能力。0.75/6自由26.793秒、中心24.472秒，中心曲线直接用自由变量表示后目标24.471949826且原56几何通过，表明本次自由求解遗漏更好可行曲线。此为只读诊断代入，原实验解未替换，优化和飞行次数0。


## 2026-11-08最新本文软约束与动态TOGT四方法完整矩阵全部完成

结果论文实验/实验2/results/latest_soft_margin90_full_matrix_v1_20261108已complete。唯一协调PID2965844/session27059和全部worker退出，3178.263秒完成。按用户明确的最新本文软约束自由/中心、最新动态TOGT自由/中心，原十条件全部40次新规划、40趟正式x500/PX4执行。Wang十组规划与实飞原数据沿用，源码/脚本冻结、331原记录SHA、40趟原生恢复重审、同世界/MPC/PX4均通过。42相关回归记录保留。

规划名义通过数为本文自由10/10、中心10/10、动态TOGT自由6/10、中心7/10、Wang5/10，全部40个新问题均找到收敛候选，无“未找到”。新实飞无窗框碰撞完成数为本文自由10/10、中心10/10、动态TOGT自由6/10、中心7/10，Wang旧数据7/10。7次新碰撞均在TOGT高档，collision_events列首窗和时刻。本文自由点9/10实际更快，最高0.75/6中心24.479秒，自由26.805秒，保留该负结果。

PLANNING/FLIGHT分别五方法50格，MD/CSV/JSON全部生成，碰撞直接标注，最快同口径项加粗。Wang旧实飞0.55m/1m/s结束口径原值加†，新口径缺测未以规划T补填。0.25/6动态TOGT自由首次参考未激活初始化失败，同配置重启后正式21.565秒，另1次启动尝试完整保留。此前三组和单组成绩只保护，不复用为新矩阵成绩。

FINAL_REPORT、final_validation、posthoc_validation完整。posthoc逐份核对原生接触与碰撞分类、成功实际七窗和完整参考时钟，以及实测用时来源。当前无运行中的规划/飞行，不再续跑本批；experiment由执行器暂停，replay未操作。当前本文仍为0.9原动力学软界、固定32梯形积分、IPOPT与完整PVAJ原安全交换。


## 2026-11-08最新软约束四方法十条件完整重跑已启动

用户明确本文自由点和中心均为此前17.008/17.243/26.803秒三组实飞通过的最新IPOPT软约束版本。动态TOGT两方法为刚完成0.25/6同90%软界、固定32梯形积分的版本。原十条件（静态加0.25/0.50/0.75×1/3/6）、原七窗混合运动、初相零，每个方法四独立冷初值。四方法全部40次重新计算，已有成绩只保护不复用。Wang十组规划和实飞沿用原始数据。

唯一协调results/latest_soft_margin90_full_matrix_v1_20261108/run_frozen.py，PID2965844/session27059。4个独立规划worker与一个串行物理执行worker流水运行，原方法顺序逐条件正反交替，每条找到的参考正式一次。已激活参考按实际成绩保留，未激活设施故障允许原配置重启一次。原生八路接触/ULog/窗口姿态恢复重审。只操作experiment容器，replay保持。生产源和协调脚本SHA冻结，当前不得改算法/代价/场景/判据或重复启动协调器。

本文两方法显式solver=ipopt、dynamics=togt、dynamic-limit-fraction=.9，全局完整PVAJ及原安全交换。TOGT两方法同软参数和固定32积分，原MINCO消元和内缩映射、原L-BFGS，四初值墙钟均不限。原相关42项回归已通过。PROTOCOL在结果目录。source_snapshot、protocol、各原始plan、planning_summary和flight_summary持续保存。

PLANNING与FLIGHT分别汇总五方法各50格，碰撞/未找到直接标注，未完成/证据不足区分。Wang原实飞使用0.55m/1m/s历史终点阈值，原实测时间加†展示，缺少完整参考时钟的测量保持缺测，不以规划T替代。加粗仅比较同口径有效时间。仅跟踪当前批次到完成，不建立定时任务。核对status、两个summary、REPORT、原始新增result/接触以及实际进程，不凭状态文件认定运行。


## 2026-11-08动态TOGT自由点0.25／6同参数规划与实飞完成

结果dynamic_togt_margin90_a25_f6_v1_20261108已complete，PID2917010/session35623及所有worker退出。原动态TOGT MINCO、内缩映射、点时刻完整链式梯度及原L-BFGS，软上界0.9和固定32梯形积分，与当前本文参数同步。四独立时长初值原目标22.410838591/21.996375728/22.835482848/21.557779587，选四号初值（索引3），参考21.557776003秒；原球/STL和任务通过。原停止规则保留，结果不宣称全局最优。

同x500/PX4/MPC各一次实际执行，动态TOGT七窗按序、零窗框/地面接触，完整含返航21.565秒，末端误差0.068197m、速度0.028301m/s，跟踪RMS0.072766m、最大0.121186m。本文同组参考16.994420803秒、实际17.008秒，本组实际时间缩短21.131%。原生日志恢复独立重审和八路接触/窗姿态核对均通过，MPC/PX4参数相同，原三组文件SHA保持。42相关测试通过。FINAL_REPORT、PLANNING、FLIGHT、protocol、validation和原四候选齐全，当前无规划或飞行进程。

新增soft_dynamics/minco_penalty.py，复用原生MINCO反传与参数化软积分，仅供同参数TOGT。static_calibration Objective/solve和no_tilt worker接受dynamic_limit_fraction默认1.0，默认原调用保持。没有修改原C++库，没有把本文SIP加入基线求解。


## 2026-11-08动态TOGT自由点0.25／6同软参数复算启动

用户接受三组预留余量结果后，要求仅跑0.25/6动态TOGT自由点，软约束用当前90%上界，积分沿用同步32区间梯形。新入口results/dynamic_togt_margin90_a25_f6_v1_20261108/run_frozen.py，session35623，实际PID见status。沿用static_calibration的真实动态TOGT、原内缩映射、MINCO消元状态、原L-BFGS与四独立时长初值，墙钟不限。新增ConfiguredMINCOCore复用现有参数化软积分和原生MINCO反传，完整点与时间链式梯度测试通过。原C++库与默认比例1.0行为保持，软参数及积分与当前本文方法一致，原安全评估仅报告、不参与TOGT优化或替换候选。

42项相关回归通过。待四初值结束后，选原完整目标最低收敛轨迹，原球/STL/任务结果另列，再各一次真实x500/PX4与原MPC执行。规划与实飞分别和已冻结的本文0.25/6（16.994421参考、17.008实际）对照。原三组及所有旧记录SHA保护，只操作experiment容器。


## 2026-11-08三组10%动力学余量实验全部完成

original_constraints_x500_margin90_three_v1_20261108已complete，PID2780696/session17788与全部worker退出。三组规划参考16.994420803/17.238424434/26.792866621秒，四初值求解369.256/676.747/967.614秒，合格4/4、4/4、1/4。三组各一次真实x500/PX4飞行，3/3七窗顺序与零窗框接触，完整含返航17.008/17.243/26.803秒，末端误差0.066356/0.052029/0.077501m、速度0.146857/0.184433/0.125382m/s，全程跟踪最大0.098214/0.090164/0.101003m。原生记录恢复独立重审通过，MPC与PX4参数一致，2532源/16原记录SHA通过，31相关测试通过。FINAL_REPORT、PLANNING、FLIGHT和final_validation齐全，旧结果保持。

原名义模型5ms电机峰值7.703615/7.701475/7.702090N，相对软上界7.693722N有小超限，保留为诊断。实际参考映射峰值8.116363/8.116711/8.733530N，最高档短时需求超出8.54858物理上界；实际三组仍零接触。软余量与实际闭环证据分列。用户已接受本轮，后续任务仅0.25/6动态TOGT自由点，同当前90%软上界和每段32区间梯形积分，规划和实际执行分别记录。


## 2026-11-08按用户要求给原动力学软约束预留10%余量

当前唯一协调为论文实验/实验2/results/original_constraints_x500_margin90_three_v1_20261108/run_frozen.py，PID2780696/session17788，三worker2781217/2781218/2781219。使用原TOGT重力模型与零tilt-yaw，原两类动力学软约束，dynamic_limit_fraction=0.9。单桨上界7.693722N，XY角速度198deg/s，Z180deg/s，原下限0、质量/惯量/桨臂/矩系数、权重1000、mu0.01、32积分、原几何与安全补点保持。MPC与实际PX4可用物理上限保持。10%为本轮自主选择，已向用户说明。没有增加新的约束种类或拒收条件。四原解析初值独立，5000有限迭代、24交换、无限墙钟。

每组四初值与原几何/STL复核完成后即各一次实飞，协调器串行取得原Gazebo锁，其他规划worker继续计算。实飞逐条件在gazebo/<level>，每组含七窗及返航，接触优先分类、末端误差/速度单列。只操作experiment，replay不操作。未预留批次original_constraints_x500_three_v1已按用户要求停止，a25_f6/a75_f3各4/4名义通过，a75_f6未结束；其中a25先行实飞在10.555秒第5窗碰撞，原始接触已恢复重审并保留。

代码仅新增软界比例参数，原1.0默认行为保持。原C++积分器未改，fraction1使用原生计算；预留版本使用同代数软罚的现有局部CasADi核和梯度。SLSQP、legacy IPOPT与factorized IPOPT均传比例，重构历史记录默认1.0。新增三项回归核对原动力学映射不变、几何与初值不变、两比例初始有限维数同为1680/2033、代数目标与梯度一致。31项直接/执行回归已通过，最终重跑在进行；生产源冻结后不得修改或重复启动。


## 2026-11-08按用户新指示恢复原约束与x500数值，三组复算启动

用户要求原约束种类、仅x500参数，重新实验。正在运行的唯一协调为论文实验/实验2/results/original_constraints_x500_three_v1_20261108/run_frozen.py，PID2698945/session28834。三worker显式dynamics=togt，各原四独立冷初值，原重力映射、零tilt-yaw、原角速度及单桨推力软罚、原积分权重/安全交换与MPC。已有原版质量2.064307692kg、惯量[0.02383948,0.02394241,0.04399995]、桨臂0.174m、矩系数0.016、推力0至8.54858N及220/220/200deg/s软界均为既有x500配置，故本轮恢复原参数，不引入新气动规划模型。原推力下限0沿用TOGT名义约定，执行模型最低转速150rad/s另列。容器px4-param直接读取三个角速度界均核对，原SDF复核及记录保存在x500_parameters。

三组通过后各一次真实Gazebo/PX4执行，完整七窗及返航，源/旧记录SHA保护。前一气动模型分支和所有结果保持。10000上限旧重算按用户上一条要求已停止，此处三组新任务由用户最新重做指示授权。当前不得重复协调或修改冻结生产源。


## 2026-11-08用户停止重算，0.75／6只读原因分析完成

用户明确先不用重算。10000上限复算PID2606878/session84038已终止，最后记录为初值0有限NLP6749步，没有新正式result。当前无优化、无飞行。不要自行续算。诊断在论文实验/实验2/results/x500_failure_diagnosis_v1_20261108，audit/model_ablation/gradient_step_sweep/REPORT齐全，全部只读求值，无优化。2531项生产源与8项原记录SHA保持，原px4.py保持。

实际x500改动包含气动映射、north航向pi/2和软界。质量、惯量及8.54858N上限此前已经是x500。旧四条曲线在新版原56几何下仍4/4通过，但新动力学软罚变为17010.797/29672.573/12202.132/11041.323，旧约2e-7。仅新界、零航向无气动罚19.822/115.996/29.943/20.471，改pi/2无气动后2508.005/3471.939/1763.104/2514.314。新模型显著改变目标，动力学保持软罚，不支持几何无解解释。

新版四次共同失败在round2第三个有限NLP5000上限，原硬节点残差负0.318056/0.215369/0.315464/0.005528。round1尚有12/12/10/7个窗段不合格，下一轮补点求解失败。多尺度目标差分在1e-9至1e-10接近解析导数，1e-6跨狭窄软罚过渡，未发现直接一阶导实现错误。具体曲率、尺度与热启动贡献仍未做优化对照分解。已完成两组实际飞行0.25／6和0.75／3均零窗框接触且七窗顺序，完整含返航15.913/16.057秒，末端误差0.034482/0.053747m。最高档本轮未规划成功、未实飞。


## 2026-11-08x500对齐后两组实飞通过，最高档独立放宽求解中

x500_matched_three_v1_20261108的5000上限规划已全部结束。a25_f6/a75_f3为4/4合格，所选15.893673557/16.043927440秒；a75_f6为0/4，四初值均有安全补点后有限NLP迭代耗尽，初批协调已退出，不能写成整批运行中。两组可用轨迹已在其gazebo子目录各一次实飞，2/2七窗顺序通过、零窗框接触，完整参考执行15.913/16.057秒，含返航，末端误差0.034482/0.053747米，速度0.158291/0.232503米每秒，全程跟踪RMS0.130246/0.204916米、最大0.358687/0.672260米。两份原生日志无损恢复重审通过，原MPC/参数一致，飞行协调PID2606249已退出。

最高档单独重算在x500_matched_a75_f6_10k_v1_20261108，PID2606878/session84038。只把每有限问题上限提高到10000，保留同四独立解析初值、当前x500目标、场景与安全交换，不读旧优化解。input_audit核对2528余生产源及原结果，确切数量见原始文件。当前最高档尚无新的合格规划、未实飞。28项直接回归及39项扩展回归均通过，二者有重叠。当前无Gazebo飞行在运行，experiment已由执行器暂停，replay未操作。


## 2026-11-08按真实x500参数重算并实飞三组

当前唯一协调为论文实验/实验2/results/x500_matched_three_v1_20261108/run_frozen.py，PID2507556、session13213。三worker各四独立冷初值，显式IPOPT与dynamics=x500。规划与MPC前馈共用togt_style_mpc/flatness.py，纳入SDF核对的2.064307692kg质量、整机惯量、旋翼阻力与滚动力矩，物理单桨界KF×150²至8.54858N，角速度界3.8/3.8/3.49rad/s。保留完整八段PVAJ、原时间加软动力学目标形式、权重1000、32积分、零倾角罚及原SIP安全交换。原三条件、原球与场景保持，各四独立解析初值，无其他方法解输入。

这是三次拖曳定点迭代和气动力矩补偿的共有参考映射。执行MPC的33状态、PX4控制器和电机响应保持，规划仍为软动力学，不能据此宣称完整执行可行或零碰撞。SDF与模型参数审计落在x500_parameters；原规划及实飞原始结果SHA保护。已通过28项软动力学、执行接口及跟踪回归，扩展相关测试在运行。规划通过后协调器顺序自动实际飞行三组，只操作experiment容器，replay不操作。不得重复启动或在冻结阶段修改生产源码，后续核对新原始候选和实际进程。


## 2026-11-08三组实飞诊断模型边界

soft_dynamics原生TOGT目标使用重力模型，gazebo_matrix.reference.MPC.references使用north-heading阻力修正与旋翼气动力矩补偿，shooting_controller预测含33状态PX4控制链。ipopt_three_flight_diagnosis_v1_20261108按相同PVAJS分别计算需求并与ULog分配目标对照。convergence.py通过实际测量/历史输入重建隐藏内环状态，对固定NLP比较30/300迭代，生产实现保持。shooting_controller日志solver_status固定0，真实收敛应读converged与optimization_status。

## 2026-11-08最新IPOPT参考的实际执行入口

results/ipopt_factorized_three_gazebo_v1_20261108/run_frozen.py串行调用soft_dynamics.flight，直接复用radius_ablation.execution.execute和原RecordedSession/33状态CTBR MPC。三组原系数与时长来自ipopt_factorized_validation_v3_cached_20261108，执行不重规划。ULog真值/八路实体接触/实际窗口位姿独立重审，完整参考时钟与真实七窗事件决定任务完成，窗框碰撞优先，地面单列。三次实际结果均碰撞；该批的证据验证通过不表示飞行成功。

## 2026-11-08有限IPOPT函数分块与安全交换

最终实现进一步将相位时间原点、尺度和目标归一化变成NLP参数，保存一个冷图与一个当前补点图。各初值重新传入原始状态，冷图使用warm_start_init_point默认关闭的独立IPOPT初始化，缓存复用与解热启动在日志中分列。窗框距离的编译原语只有三维局部位置，26维轨迹与窗口运动链式关系留在外层。当前选用6对历史的limited-memory Hessian，原目标与约束梯度解析计算。最终四条件四初值30测试与全几何回归见ipopt_factorized_validation_v3_cached_20261108。

论文实验/实验2/soft_dynamics/fast_ipopt.py实现当前显式ipopt入口，kernels.py提供单节点动力学/罚函数、单段PVAJ系数、单窗几何及21阶MINCO状态消元。小函数在/tmp/convex_ipopt_local_jit局部编译，33积分点和8段共享函数，避开完整NLP导数的巨型代码生成。MINCO仅作坐标基准，63个VAJ偏差均保留，目标仍为原完整时间加角速度和电机软积分。

每个独立初值持有一个有限问题缓存、原始变量与按约束语义键索引的乘子。补点后新约束乘子置零，已有乘子按键转移，同有限集可重用求解对象。初始只放每窗平面点，之后使用原全部56几何oracle。global_gate_minco_probe/Planner.solve仅对显式verification_ready的有限中间点开放复查/补点，最终仍需成功与复查通过。旧调用者保持原分支。ipopt_legacy保留旧精确Hessian与20个双侧初始点，slsqp保留原直接目标与TOGT正时长坐标。

## 2026-11-08直接SLSQP默认入口

soft_dynamics.Planner默认solver=slsqp，worker同默认。interior_point共享MINCO偏差和原几何构图，SLSQP分支直接调用symbolic_penalty.objective，仅96/71变量。slsqp_solver适配解析梯度/Jacobian及等式、上下界约束，返回自身success，原core继续安全交换。IPOPT显式分支保留原1584辅助量。

共享sc_dynatogt.time_mapping新增symbolic_durations_from_k，与原TOGT forwardT和导数逐值匹配。SLSQP优化每段k，原时长界通过k_from_durations转换，初值相同物理时间经逆映射输入。SLSQP中不再构造绝对时刻差或截断时长。test_slsqp_solver覆盖仅轨迹变量、原软目标、正逆时间映射及导数。


## 2026-11-08最终独立软版数值入口

soft_dynamics.Planner.seeds为四时间尺度、运动转折查询、开口内射线投影和代数MINCO PVAJ，point_mapping.inverse增加圆代数/凸多边形均值坐标逆映射。interior_point保持完整轨迹自由度，用绝对时刻、MINCO偏差坐标和多边形冗余尺度规范，软超限量辅助变量等价表达原固定32积分。当前没有子空间预优化，所有变量首轮自由。

solver成功仅按Solve_Succeeded返回，core继续原节点/56几何，原C++完整J择优。新增辅助差和残差仅诊断，不做拒收。v17实际运行版本与当前验收代码的精确差异及维护在结果内保存，独立重审通过且最优选择保持。未来运行使用同步后的当前PROTOCOL。

## 2026-11-08软目标等价辅助变量求解修复

soft_dynamics.Planner.seeds由当前时间与中心点解MINCO PVAJ。interior_point构造原轨迹变量加1584个超限辅助量s>=r、s>=0，保持原smoothedL1软罚，IPOPT解析Hessian求解，原C++目标重算选择。symbolic_penalty使用与原C++一致的紧凑归一化推力/Hopf角速及角加速。固定32积分统一x500共享C++与符号目标。

geometric_constraints仅配对工作空间上下界、移除重复见证，原任务/安全判据保持。global_gate_minco_probe新增curve_values，避免无用Jacobian输出参与高阶构图。regression负责16条新冷启动代表回归、冻结、原结果SHA及目标/几何/初值/残差复核，完整实飞分开。

## 2026-11-08当前无倾角批次已按用户要求暂停

results/no_tilt_five_methods_v1_20261108在27/40规划后停止，3项中断，0实飞，协调和worker均退出。USER_STOPPED.json留证，后续先分析数值求解，不自动续跑。只读证据在no_tilt_solver_diagnosis_20261108/DIAGNOSIS.md。

## 2026-11-08无倾角惩罚对比协调层

实验2/no_tilt_comparison复用soft_dynamics.Planner、static_calibration.Objective及radius_ablation全参考执行。worker统一元数据，四十规划独立冷启动；run负责冻结、旧数据SHA、40参考串行实飞及原生恢复重审；report生成硬/软两套五方法规划与实飞四表。build使用同一固定TOGT源补齐的八cpp和原vendor头，原子替换两本地库。唯一目标改变为x500 weightRot=0。

## 2026-11-07本文完整PVAJ的TOGT软动力学版本

实验2/soft_dynamics/planner继承原独立自由/中心几何模型，仅构造工作空间硬节点，原安全见证、56窗框复查与正向约束保持。objective通过原CasADi曲线Jacobian将原生任意多项式软积分梯度回传至96/71变量。native/polynomial_penalty.cpp包含旧togt_x500.cpp，调用原integrate/QuadManifold，显式处理归一化系数与时长链式梯度。链接原libtogt_x500.so，原源码/库保持，固定TOGT头文件和许可证在vendor。

worker每条件方法四独立冷初值，按完整软目标择优。run负责20规划隔离并发、原数据SHA保护、规划独立验证、实飞串行及原生日志无损恢复复核。flight复用已有完整参考执行器和MPC/Session，方法标记来自当前源result。soft动态重力/零tilt-yaw原生名义模型与north图拖曳执行接口分别记录，不能将该变体说成仅将旧17节点残差换成罚函数。

## 2026-11-07冻结轨迹物理执行层完成

实验2/gazebo_matrix协调50项、49参考。reference复用Plan与33状态单重射击MPC，将参考姿态/角速/角加速度接至规划north图与解析PVAJS。flight复用PX4执行/准备/原生传感器链及实际出窗计数，audit在ULog/实际窗姿态/八路完整sleeve和地面接触上重验，并核对真实终速。archive保存逐文件gzip、原文/压缩SHA及按趟恢复上下文。catalogue/references保持原选中系数/时长与来源SHA。
gazebo_all_selected_v1_20261107已50/50 complete，49趟均attempt_1，原始记录全部恢复并独立复核通过。20相关测试、2460冻结项核对通过，各条件五方world/PX4/MPC/执行配置一致。结果39无碰撞/10碰撞/1原规划未找到。Wang拼接参考同样接受统一跟踪，原规划计算墙钟另计。
结果export_results.py从已有数据生成CSV/配对时间差/首次碰撞及文件SHA，原始基类兼容method/checked_soft/continuation字段在SUMMARY说明，方法身份以experiment_variant/reference_manifest为准，所有实际solves为0。原生日志12.077GiB无损压缩1.363GiB，原始x500模型与9碰撞体另行归档。协调器结束且世界暂停。

## 2026-11-07恢复并完善Wang数学核

复现/wang2023_verified来自历史paper_core快照，保留原泛化s3/4 MINCO及Eq18–32软罚、动态中心等式与原LBFGS-Lite。native桥收敛返回accepted iterate，异常/预算才返回最佳试探点，避免converged标签与曲线不一致。flatness_function作为显式平台物理接口，独立x500.py逐值匹配当前名义模型，数学核不导入我们的规划器。
实验2/wang2023以N2/M4/s4实例化，配合当前PVAJ起终/曲线拼接。Reference一次解析生成单段全局MINCO作初值，固定sf用相邻预测中心中点向当前出侧投影后的巡航状态，V6.5m/s、A/J0。整机离面事件求根后截取已执行前缀并按实际PVAJ换下窗。最后窗执行返回真实零VAJ终点。这样避免直接固定插值造成的异常终点，开发版本及源保留。
此层为论文未公开sf/重规划触发细节的明确场景适配，理想跟踪与计算墙钟分离，原软残差只诊断。FG本地C编译通过等价测试，启动建图与局部请求成本分别记录。正式wang2023_full_v1_20261107十条完成，全部70局部请求/280候选解析初值、目标、系数、动态中心等式、事件和执行前缀重建，公共STL/5ms重评通过一致性核对，联合59测试通过。
source_snapshot保留2452项求解时冻结源码。末轮只修复验证输出目录与单文件版本核对，HARNESS_REPAIR显式记录run.py旧/新SHA及完整差异，其他全部2451项内容一致。验证器仅允许该精确记录的入口差异，未来新批次仍使用完整当前冻结。原始轨迹和原先停止错误保留，最终validation通过。

## 2026-11-07完整独立求解实验协调层

新增实验2/independent_full_matrix，仅协调/继承核验/报告，复用当前independent_free_solve.Planner。本文自由96维与中心71维分别直接求本条件四解析初值，300秒/初值。25历史同版/固定基线结果核验后原样复制，15新算，全部40重新STL/地面/任务和5ms诊断。
matrix_reuse.json保存来源与原始文件SHA，避开历史已有reuse.json。几何evidence与额外dynamics_evidence分别保存，相关字段冲突已由实际静态继承评估测试覆盖。2433冻结项、31相关测试通过。
passed表示原名义几何验收，time_comparison_eligible额外要求姿态、5ms角速和电机实际模型界通过，90%规划余量超限独立列。最终验证重建本文20模型/80初值及全部找到曲线、原节点/56几何，未加入动力学补点算法。原各冻结算法不改。
v2已40/40 complete且validation通过，2433冻结项不变、31测试最终通过，全部进程退出。几何通过数10/9/6/5、诊断可比9/8/6/5，两个角速超限单列。39已找到轨迹均超统一90%参考界，全部电机实际模型界诊断通过；原节点合格与额外诊断标签分别保存。

## 2026-11-07独立自由选点入口

用户否决中心初始化。实验2/independent_free_solve继承smooth_augmented，只改数值求解顺序为直接归一化SQP→必要时原同问题恢复。四个原解析初值不变，窗内点、时间和全PVAJ从第一次优化起全部开放。补点续优化也先直接SQP。
run仅调用free_points=True图，保留center_solution_used=false及center_subproblem_solved=false元数据。没有center_recovery依赖，不读取任何历史/其他赛道轨迹。56几何/STL/地面/节点/姿态与返航验收复用原逻辑，中心初始化入口保留为历史方法。
正式5/5完成且validation通过，20解析初值逐值重生成，2429源冻结、22测试通过。额外dense_dynamics_diagnostic为独立结果层，保留原评估记录，a50_f6节点间角速6.575507>规划3.8，五条最大电机7.798052>90%余量但<物理模型界；姿态域passed仅表示图域/连续性，不能等同角速约束全域通过。

## 2026-11-07当前算法验收完结

same_course_center_recovery同版五条件完成。5条均selected_stage=free_refinement，free_refinement_converged=true，中心全部本次独立四初值，无历史轨迹输入；中心图71维嵌入自由图96维保持曲线与V/A/J/T。
runtime返回完整选择范围标签和两阶段成本；22测试、20初值逐值重生成、双方完整原几何/STL/地面/任务/姿态/节点证据通过。2426冻结项不变。当前没有运行协调器。
名义动力学是原节点硬约束，5ms电机可略超90%规划界但低于实际物理模型上限；结果仍属名义离线规划证据。原单窗center_initialized方法与当前全局适配边界保持区分。

## 2026-11-07当前同赛道中心初始化的全局适配

same_course_center_recovery.Planner复用independent_repair的完整图与数值方法，先以同一track构建中心图，四冷初值求中心可行解；embed_center在8个logT后插入TOGT中心映射变量，原V/A/J全保留，所以得到同一条C3/PVAJ曲线。
refine_center从嵌入点直接SQP并必要时原增广恢复，最终全部原节点/56几何/姿态通过的自由候选若更快才替换中心基准。明确返回阶段和优化范围，中心保留不是自由求解器成功。中心未找到时才自由四冷初值。
全局变量维数71→96，无法直接调用旧单窗7变量CenterInitialized混入类，机制沿用但保留独立适配。run对原5自由失败重新独立求全部阶段，冻结2426项并完整计入两阶段成本，独立STL/地面/姿态及模型/嵌入复算。

## 2026-11-07独立恢复v2终止精度

正式算法仍smooth_augmented→augmented→原实验2全局图。修正归一化SQP额外收紧的ftol1e-9，当前1e-5并写入结果sqp_normalized_ftol字段；原最终节点验收1e-5与56几何/STL判据未调整。
v1三条后中止，v2从每个真实目标赛道的原四初值重新独立计算，不能回填或继承旧单种子结果。原Y→north航向变化照录，物理参数/TOGT映射/PVAJ/返航与节点模型结构继续复用。

## 2026-11-07当前独立恢复与四初值回归

independent_repair/smooth_augmented继承augmented的同条件增广拉格朗日恢复/SLSQP，用tilt_heading_minco/dynamics的平滑姿态构图和域检查。物理参数/原映射/全PVAJ/八段含返航/节点硬约束及56几何验收复用原实验2。航向定义与原固定Y不同。
augmented只在一次目标请求内部保存严格节点可行点作为SQP初值，SQP未收敛不放行。恢复内点数值余量1e-4，最终仍按原约束优化；正的常数时间目标归一化不改变最优解集合。四次冷启动互不继承状态，不读其他条件轨迹。
run冻结2421项源/资源，最多3隔离worker，协调原9未找到条件，各4初值300s，独立STL/地面和5ms姿态复算；最终重建原模型检查候选/曲线/节点/56几何和初值。
coordinates、symbolic、ipopt_planner、scaled、原planner信赖域实现均是保留的开发试验，正式run只调用smooth_augmented。

## 2026-11-07：独立初值的迭代上限对照

实验2/iteration_limit_only继承原实验2/Planner，仅在隔离子进程包装core.minimize将maxiter400改2000，finally恢复入口。没有更换姿态、目标或安全逻辑。
run仅针对原9条未找到，各赛道生成自身原四初值；旧数据仅核验场景/配置/初值等价，不参与优化。保留所有候选、终止变量、源快照，完成后重新评估整条曲线所有STL/地面。用户已否决的tilt_heading_minco连续法模块保留追溯，不属于本轮。

## 2026-11-06：tilt_heading_minco独立数值修复与连续初始化

dynamics.py复用原阻力/惯量/MIXER，姿态改北图倾斜旋转×固定图内航向；航向约定变更明确，不声称全姿态无奇异，极点/零推力验收为unresolved。
planner.py继承实验2完整PVAJ/几何模型，作用域内注入新物理映射并finally恢复，sumT目标等价替换log1p，SLSQP2000，不改原TOGT选点与约束。必须隔离进程，不支持多线程同时切换共享构造接口。
continuation.py只负责幅度辅助问题与暖启动，目标lambda1才返回；static_continuation.py把零运动频率规范为0/周期∞，解决无效频率影响原时间缩放。原phase/2D映射/IPOPT分支仅开发证据，不是最终算法。
实验2/attitude_repair/run.py为直接4初值版本，run_continuation.py为最终连续版本；validate.py逐条复算最终目标/源/原节点/56几何/独立STL/姿态并归档快照。最终代码路径见tilt_heading_minco/FINAL.md。
不改共享core默认、历史结果或MPC/PX4；原40项已完成保留，修复后只有失败回归，不能当完整新矩阵。

## 2026-11-06：whole_course_difficulty全窗幅度×频率适配

实验2/basic_group/whole_course_difficulty/scenario保持togt_course全部静态输入，仅替换七窗Motion的频率与幅度，独立Case/profile/manifest公开归一化幅度、频率因子和中心峰速；不再使用两挑战窗等级。
layout在串行、隔离的实验进程中临时注入新scenario复用同一TOGT几何/运动筛查，finally恢复，不改旧场景或碰撞算法。run复用separated_course的单锁预检/协调/最终复核，覆盖新场景/计划/评估/报告/源冻结接口。
静态继承严格核对旧全部源与原场景，复制未修改的result/候选，另存reuse证据并重评；原成本与新求解计时区分。动态全部新算。规划器和目标/约束/初值/择解不改。
完整3×3动态矩阵加静态、四方法共40项；同幅度按频率递增报告，非相邻风险保留并停批。协议不提供随机成功率或实际飞控安全结论。

## 2026-11-06：TOGT原始布局适配，不再使用外围环线

basic_group/togt_course/upstream.json为官方固定版本YAML的事实提取与来源哈希；scenario还原7位置/顺序/6姿态，保留叠窗。仅统一尺度/高度平移、沿用现有物理窗并披露G7/终点/长廊任务适配。
layout用实体STL定向顶点范围与解析运动范围构造3D扫掠AABB，机体球膨胀后对线段1m管道做分段二次精确最短距离（普通浮点）；并非原任意大球走廊。所有实际参考继续原STL检查。
run在单独进程复用separated_course的固定预检/协调流程，但覆盖scene/plan/assess/report与源元数据，不继承被否决的坐标、法向或成绩；底层四规划器未改。
W3/W4交替升档、五背景固定、初相0；静态四方和动态非相邻预检没全过不启动余下正式矩阵。旧外围与紧凑记录全保留。

## 2026-11-06：separated_course场景与预检门槛

实验2/basic_group/separated_course并列于旧trajectory_only：scenario只替换锚点/固定法向与每窗Motion频幅，旧physical aperture/shape inset深复制不改；沿用相同符号pose和数值Motion。
layout用最大运动距离+STL顶点半径+机体半径构造膨胀包络，对所有非相邻航路走廊做分离核验；figure绘制布局和最大扫掠图，不操作世界。
run只注入新Case/scene接口到既有PVAJ和native x500 TOGT，保留全部优化设置。先静态4方通过、低/最高动态TOGT无非相邻风险，准入后完整40条；预检参考原样复用，不重新择优。
评估仍复用trajectory_only.collision完整全窗/全段/STL与地面检查；foreign_diagnostics将非相邻接触/待定另列，并使批次停止。走廊证明不能替代实际参考核验。

## 2026-11-06：实验二纯轨迹适配

basic_group/trajectory_only独立于flight：run只注入基础场景到旧plan，四时间初值/原择解未改；先复用经旧源和scene核对的12份规划，再计算16新规划。
collision用真实sleeve STL点/三角面距离、KDTree保守候选、1-Lipschitz盒下界、多项式Bernstein和解析运动范围，在所有窗×所有段做浮点自适应分离；不使用凸包、附加安全余量或零厚度边界冒充实体网格。地面用标量多项式极值。
完整时长为八段之和，含返航零VAJ；按序正向一次、端态单列验证。碰撞见证/通过下界/待定三分，最终全量复算并用全三角面核对所有碰撞见证。
模型仍为外接球，而不是姿态机体；结果不涉及飞控/跟踪、Gazebo实体接触、连续动力学或严格区间证书。旧实飞代码和数据原样保留。

## 2026-11-06：基础组四方法共同动态场景与完整结果

实验2/basic_group为独立评估适配：复用compact original静态世界，Motion只将局部(up,horizontal)坐标中的运动改写成水平平移/向上圆弧；物理开口R不变。pose/local/local_bounds及Gazebo插件axis0/1完全同义。
plan只注入场景，复用原PVAJ全局求解与历史x500 TOGT原目标/求解器；flight沿用冻结参考/MPC/PX4和8路完整实体接触；run只协调12条件并继承已有静态TOGT两条。
validate独立复算ULog/接触，自动读gzip且缺日志报错；2359冻结项/同世界MPC/PX4核对通过。完整结果basic_group/RESULTS.md；5完成/4碰撞/3未找到，协调器退出、世界暂停，未操作replay。
结果目录plot_precontact.py仅处理实际轨迹绘图，碰撞曲线截至首次接触并显式标叉，全时段原曲线/ULog仍保留；不是修改执行或掩盖失败。

## 2026-11-06：紧凑七窗与x500执行内缩

实验2/compact_calibration显式复用原TOGT/飞控接口，水平锚点1/4、W1侧移−4m，实际物理窗口不缩小；safe map按x500半径+统一0.20m设计执行裕量设置，不是误差界证明。
高度组HEIGHT_NORMALS是离线固定三维窗朝向，非运行时优化/追踪逻辑；前两布局manifest与v3完全一致。未采用高度起伏缩半方案。
v3已飞5条、4成功1碰撞；新高度v4只有两份名义参考。28测试通过，原始独立验收完成；用户要求的日志无损归档/快照去重已释放13.55GiB，见实验2/STORAGE.md；当前不运行协调器。

## 2026-11-06：用户纠正的交错七窗场景适配

实验2/interlaced_calibration/scenario.py保留single_gate_stop_racing历史七位置集合并交换4/5，原形状顺序不变，窗竖直、朝向开发校准后固定；只支持已验证静态，不声称动态适配完成。
planner/flight/run通过显式接口复用static_calibration原目标、内缩、MINCO与MPC/PX4，不修改共享算法。
probe_v5是实际交错参考；probe_v4错误绕过入口产生旧椭圆，协议检查中止未飞，不能混入成绩。
px4_v4/results内resume_unflown仅保留有效中心并对0任务时间设施失败重试一次；finalize独立核对真值/接触、配对和冻结源；archive_to_main逐文件哈希复制验证产物，不重写原始路径。
当前6/6无碰撞七窗返航，主结果interlaced_calibration/RESULTS.md；旧椭圆static_calibration仅历史开发与复用基础。

## 2026-11-06：历史七窗三布局静态校准适配

`实验2/static_calibration/scenario.py`复用旧apertures与sleeve资源，三布局几何生成；仍支持共同4平移/3摆动接口，本轮仅A=0。
`planner.py`继承历史X500Objective，中心只消除空间变量；原MINCO/C++目标/伴随/shape-specific映射不改，使用历史SciPy TOGT包装。
`flight.py`复用冻结参考和统一MPC/PX4，完整mesh8接触通道/原生位姿/起跑与被动磁力计记录；不修改共享执行器。
`run.py`冻结源、复制网格、校验六离线参考、顺序执行与配对；结果目录`finalize.py`只读独立复算并作图，旧2314项源未变。
完整六趟结果见static_calibration/RESULTS.md。此适配与旧实验2小圆窗/C++桥区分，不能混用成绩或称新动态矩阵已完成。

## 2026-11-06：实验二四全局方法与混合运动适配

`convex_timevarying_window/comparisons/论文实验/实验2/`是独立实验适配层，不更改历史全局算法默认。
`scenario.py`统一4平移矩形/3外支点圆弧摆动圆窗、三布局三运动档；`safety.py`对解析摆动提供局部浮点范围。
`planner.py/plan.py`复用global_gate_minco_probe完整PVAJ，精确中心消元，八段含返航，四初值取最短合格；并非在线滚动。
`togt_dynamic.py`复用现有原生位置型C6 MINCO/目标伴随及发布L-BFGS，动态时刻与映射点联合，完整链式梯度，无安全滤波。
`global_flight.py`四方法统一冻结参考+MPC/PX4，禁用调度器；20原生接触、真实门位姿、起跑就绪和被动磁力计记录，不注入控制或传感器值。
`run_four/report_four/continue_four`处理冻结矩阵、继承证据、设施未起飞重试和完整结果；旧Wang/滚动草稿不是当前入口。
本批结果目录`finalize_results.py`只读复算原始ULog/接触和源哈希，补充protobuf省略nsec=0支持，写validation/图表而不覆盖历史audit。
36条件完整结果见实验2/RESULTS.md，后续不能把无规划等价于物理无解或把节点动力学升级为连续认证。

## 2026-11-05：双方对称的时间多初值

`multistart_minco/planner.py`复用原单窗规划图，每次独立调用原求解器并固定使用四个原时间种子之一，空间初值始终中心。
自由/中心仅选点变量界不同，各自按全部合格候选的最小T1+T2返回；无预先中心优化或候选间热启动。
记录temporal_candidates全部初值/结果/耗时/失败、selected_seed_index及总周期；显式有限预算时共享，而当前无墙钟限。
worker/flight复用原隔离进程与物理验收；实验入口/同请求诊断/六趟对比在实验1/multistart，旧两阶段方法保留。

## 2026-11-05：中心初始化的单窗精修

`center_initialized_minco/planner.py`的CenterInitialized混入复用原P/V-MINCO同一图，中心阶段仅将2:4坐标界锁零，
精修阶段恢复原界并将中心全部七变量作为唯一初值，保持同一PVAJ和绝对时刻。失败/无改进返回已验收原中心曲线。
日志保存两阶段统计、完整内部中心参考与总耗时，有限预算跨阶段共享；未知异常仍报告。原父类求解和安全/飞控不改。
worker/flight复用原隔离进程和Gazebo验收；旧single_gate_swing仍可复现历史。新开发协议/固定请求/六趟配对在实验1/center_initialized。

## 2026-11-05：凸窗口论文实验1与独立摆动适配

`convex_timevarying_window/comparisons/论文实验/实验1`保存独立入口、协议、分析与原始记录。
平移复用原single_gate_swing数学问题/单重射击MPC；pendulum独立适配圆盘TOGT坐标、作者阻尼单摆ODE查询和圆外集合局部包络。
同一三次样条运动表传给规划与独立Gazebo插件；原生pose/info核对实际门位置/姿态，不只核对指令。
High-MPC保留作者缓存目标/动力学和原权重，显式转坐标/状态/CTBR单位并适配共同物理输入箱，未重训。
公共soft_only_mpc/px4新增终点容差常量，默认仍0.55m/1m/s；仅新摆动入口将在线位置阈值收紧0.50m，真值仍0.55m。
无新的控制奖励、滤波、降速或重规划机制；平移旧结果/默认行为不变。

## 2026-10-09：名义动力学硬约束分支

`dynamic_passage_velocity_minco/hard_dynamics.py` 构造阻力平坦映射、角速度/角加速度和混控单电机推力；
`hard_planner.py` 保持七变量两段C5，时间目标、节点硬执行量界、出口位置半空间和局部安全交换。
`hard_worker/hard_flight` 提供隔离入口，复用上轮allocation_mpc，旧入口不覆盖。
`hard_probe` 是固定旧头测试；`comparisons/single_gate_translation/hard_validation_report.py` 生成真值/接触与离线1ms诊断。
硬界仅优化节点强制，最高5.095%节点间推力超限仍存在；不可描述为连续域动力学证书。

## 2026-10-09：单窗执行修复分支

`dynamic_passage_velocity_minco/repaired_flight.py` 是新增入口，旧 `flight.py` 保留。
`allocation_mpc.py` 继承原跟踪RTI，仅替换为耦合电机力/力矩分配预测；不是新的避障控制器。
`repaired_planner.py` 通过旧planner的默认空扩展接口补推力软残差，并支持gate=1后的单段终点恢复。
`repaired_worker.py` 为隔离进程提供对应规划器；共享px4的末窗后事件监听/漏窗中止仅新Config开关启用。
结果汇总/真值审计入口为 `comparisons/single_gate_translation/repair_validation_report.py`。
六档复测四成功两漏窗；下面“不改MPC”的描述仅适用于旧flight，不适用于修复入口。

## 2026-10-09：单窗自由速度与前向尾段 MINCO

`convex_timevarying_window/dynamic_passage_velocity_minco/`独立于旧四变量版本。
`trajectory.py`构造内部位置/速度约束的两段minimum-snap线性系统（C5，不称原版TOGT的C6）；
`planner.py`7变量、原局部安全见证、尾段速度Bernstein形状条件；`flight/worker`仅替换规划器，
不改MPC/PX4或固定窗后零VAJ终点。`probe/analyze`分别负责固定输入诊断与真实接触/ULog审计。
最终flight未启用开发用A自由和提前相位配置。后续升档/修复仍仅单窗，不能描述为七窗实现。

## 2026-10-09：点式 TOGT MINCO 单窗变体

`convex_timevarying_window/togt_point_only_mpc/` 与旧软安全版并列。
`planner.py` 直接用全局 MINCO-s4 将起点PVAJ、窗口内二维点和终点PVAJ
映射为每航段一个七次多项式，窗口状态不再是自由变量；`trajectory.py`
修正每航段一片对应的穿窗事件时刻。`worker.py` 复用隔离进程接口，
`flight.py` 复用原单窗场景、MPC、PX4执行与实体接触记录。
同目标采用通用多初值、原物理航段密度的软安全采样，不属于纯换曲线阶次消融。

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
