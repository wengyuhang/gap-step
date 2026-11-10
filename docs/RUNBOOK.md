# 运行与验证手册

## 2026-11-09三项执行中止的补飞已完成

higher_swing_incomplete_completion_v1_20261109已complete，协调3218870/session80108及worker退出。三条原参考补飞16.416、16.423、21.955秒，均完整七窗返航、零接触。生产源码、参考数组和世界/MPC/PX4与原矩阵一致，100ms保护保持；CPU0至3用于MPC/传输，4至23用于容器，原规划重任务已结束。最大状态年龄均小于51ms，延迟根因尚不能唯一确定。

看新目录FLIGHT.md/CSV/JSON得到补齐50格，其中3格来源为补飞，其余47格原记录；PLANNING为原规划表副本。原完整矩阵仍保存首次50趟与3中止，累计真实记录53趟。新validation和posthoc_validation通过，3趟实际开口重算、33原生档案与原矩阵全文件SHA保持。补飞代码在该结果目录run_frozen.py，附原授权和CPU/观测元数据。当前无待运行补飞。

## 2026-11-09高起伏十难度五方法批次已完成

higher_swing_10x5_full_v1_20261109已complete，340候选任务、50项新规划和50趟正式实飞全部结束，唯一协调3027864/session43715及所有worker已退出。总墙钟5104.289865秒。27相关测试、final_validation和posthoc_validation通过，37条无碰撞完赛额外重算有限开口，550原生档案SHA保持。

规划和实飞分别看PLANNING.md/FLIGHT.md及同名CSV、JSON，概要与真实执行异常看REPORT.md、execution_incidents.json和RUN_INCIDENTS.md。本文自由8/10完成、0碰撞、2未完成，本文中心10/10，TOGT自由5/10、TOGT中心7/10、Wang7/10。三条未完成保留原状态保护分支异常，根源未定。原矩阵和展示保持，参数/代码冻结，当前批次无待执行步骤。


## 2026-11-09高起伏0.75/6规划与实飞完成

higher_swing_a75_f6_v1_20261109的17候选全部结束，总墙钟1940.017354秒，最终选独立ours_5的19.172615343秒。higher_swing_a75_f6_view_v1_20261109正式一次GUI执行完整七窗返航19.186秒、7/7整机退出、窗框/地面零接触，原生恢复重审与场景/参考/控制参数一致通过。原协调2724584/session68818和2737466/session98967及worker均退出，当前无待运行步骤。两目录validation.json均passed。

新场景绳长2.20m，必须经higher_swing.flight执行携带higher_swing_length_2p20标记的参考，使Gazebo插件length和规划相符。原95%软界/.15m安全、MPC/PX4、完整参考计时保持。4项相关回归通过。不得将原矩阵场景与新摆幅混用，失败和原矩阵记录完整保留。

## 2026-11-09匹配95%软界和内缩的完整矩阵已完成

matched95_inset8_full_matrix_v1_20261109已complete，330/330候选、40/40新规划和40/40正式实飞，协调1617319/session63213及全部worker退出。PLANNING和FLIGHT各50格MD/CSV/JSON，REPORT汇总结果、额外初始化预算、成本和计时口径。final_validation/posthoc_validation均通过，八原生接触/ULog/实际门位姿已恢复重审，所有源/旧结果SHA保持。每条flights/<condition_method>/attempt_1有无损归档，可用archive.restored恢复核对，全部首次激活，无重试。禁止重复协调或重飞已完成条件。

## 2026-11-09最新0.25/6自由点实飞完成

independent_best_center_a25_f6_gazebo_v1_20261109已complete，协调1539857/session6411、worker1540502退出，attempt_1正式一次完成。实际完整七窗返航16.418秒，7/7整机退出、八路零接触，原始16.413536秒参考逐值保持。summary/validation/posthoc_audit/REPORT保存结果，flights/attempt_1原生日志已无损压缩，可用archive.restored恢复重审，旧数据和源SHA通过。禁止重飞已执行条件，只操作experiment的原流程已完成。

## 2026-11-09独立加最佳中心释放已完成

independent_best_center_a25_f6_v1_20261109已complete，17/17收齐，协调1464305/session20297及全部worker已退出。独立自由7/8、16.413536秒；中心8/8、16.986935秒；最佳中心开放点1/1、16.444846秒。最终选独立自由，原56几何/STL/任务/内缩与来源/初值/源及旧记录SHA通过。REPORT/summary/validation/posthoc_audit保存证据，0次Gazebo。入口soft_dynamics.best_center_release run --level <condition> --outdir <new-output>支持后续独立新批，禁止重启当前冻结结果。4项相关回归通过。

## 2026-11-09分散选点八初值完成

当前批次spatial_phase_minco_eightstarts_v1_20261109已complete，8/8收齐且协调与worker全部退出。独立1/8合格、21.991795秒，中心优先汇总另列2/8、最佳为旧释放19.169840秒。REPORT/summary/validation/posthoc_audit保存证据，原8时间、初值重构和源/旧记录SHA通过。17相关测试通过，未Gazebo。禁止重启冻结协调或重跑已结束样本。

## 2026-11-09中心优先八组策略完成

center_first_phase_eightstarts_v1_20261109已complete，协调1236256/session18577与worker1236867退出。summary8条、release_summary1条、best/validation/REPORT齐全，最佳19.169840秒。新求解284.365秒、墙钟294.377秒，中心与原始独立记录复用成本另列。输入系数/目标保持、源/旧记录SHA及完整几何/STL/任务/内缩复查通过。新参考尚未实飞，不重复启动本批。

## 2026-11-09中心相位MINCO八初值对比完成

center_phase_minco_eightstarts_v1_20261109已complete，1126322/session30459及16worker退出，禁止重复启动。summary16条、best/validation/REPORT齐全。自由3/8合格21.030547s、中心1/8合格19.231067s。并行735.247s，全部初值复现与配对初始曲线、源/旧记录SHA通过。相关test_phase_initialization、test_inset_geometry、test_fast_ipopt、test_dynamics_margin共15通过。本批Gazebo零次，完整参考时间与实际执行分开。

## 2026-11-09最新自由点实际20.109秒完成

margin95_inset_free_best_gazebo_v1_20261109已complete，协调1018391/session7278及worker1019026退出，attempt_1正式一次完成，禁止重复启动本批。summary/validation/REPORT齐全，七窗按序且整机退出、八路零接触。flights/attempt_1保存原参考、真值、telemetry和原生ULog/接触/窗姿态压缩文件，archive.restored恢复重审通过。原世界/场景/MPC/PX4、源/旧记录SHA通过，只操作experiment，replay保持。

## 2026-11-09八初值扩展与中心初始化对照完成

margin95_clearance5cm_inset_eightstarts_v1_20261109已complete，919451/session42575与8worker退出，累计16结果齐全，自由20.103570秒/中心22.498943秒。margin95_clearance5cm_inset_center_release_v1_20261109也已complete，947038/session96250与两个worker退出，0/2新合格，333.040秒。初始曲线等价与失败有限集可行参照核对保存在结果目录。provenance_corrections保留来源标记修正，原输出在result_raw_solver。不得重复启动，实飞另列。

## 2026-11-09原四初值，95%动力学软界与内缩选点对比完成

results/margin95_clearance5cm_inset_paired_v1_20261109已complete，PID856966/session60444及8worker全部退出，禁止重复启动已完成批次。summary含8原结果、best分别为自由23.427409s与中心22.977863s，validation的初值/同初始时长/源/旧数据保护均通过。总墙钟476.862s，候选求解合计1914.192s。本批Gazebo零次。相关回归显式运行soft_dynamics/tests/test_inset_geometry.py、test_fast_ipopt.py与test_dynamics_margin.py，共12通过。

## 2026-11-09独立24初值最佳轨迹实际飞行完成

free_24_parallel_best_gazebo_v1_20261109已complete，协调428193/session74205与worker428820退出，首次启动正式一次通过，禁止重启已执行批次。summary/validation/REPORT齐全，实际完整参考时钟24.433秒，7/7整机退出，八路接触为零。flights/attempt_1保存原参考、真值与telemetry及ULog/接触/门姿态压缩日志，可用gazebo_matrix.archive.restored恢复重审。原世界/场景/MPC/PX4和源/旧记录SHA核对通过，只操作experiment。

## 2026-11-09仅自由点24初值同时独立求解

normal_free_24_parallel_a75_f6_v1_20261109已complete，协调299075/session95232及24worker全部退出，禁止重启已完成批次。24条result齐全，12通过、12未找到，最佳05_center完整参考24.421905秒，validation全部true。24初值独立重构与唯一性、源和旧结果SHA均通过。并行1378.255s、最长进程1372.300s、记录求解合计21273.016s。逐候选结果见REPORT/summary，最佳曲线见best_result；本批Gazebo零次。后续实飞须单列实际执行时间及碰撞证据。

## 2026-11-09正常独立求解与并行对照完成

normal_independent_diagnosis_a75_f6_v1_20261109已complete，协调152826/session7880及12worker全部退出。summary/best/validation齐全，12项已收集、四对初始曲线一致、旧源与结果SHA通过。并行878.777秒，最长875.248秒。原因看ANALYSIS.md，原始有限调用看finite_history.csv。不得把未找到解算碰撞，也不将精修结果混入正常冷启动表。

## 2026-11-09正常独立求解并行原因对照

当前唯一协调normal_independent_diagnosis_a75_f6_v1_20261109/run_frozen.py，PID152826/session7880。12个worker同时运行，禁止重复启动。核对status、summary、REPORT、真实进程及各原始result。case固定原0.75/6，三组各四原时间初值，无历史优化轨迹初始化，未实飞。所有初始化与候选结果独立保存。

## 2026-11-09更短自由点轨迹已实际飞通

free_refinement_a75_f6_v1_20261109/manage.py已complete，17个result齐全、validation通过；free_refinement_polish_a75_f6_v1_20261109已complete且未改进。free_refined_a75_f6_gazebo_v1_20261109/run_frozen.py已complete，单次正式24.434秒、7/7零接触，validation和COMPARISON齐全。全部协调/worker退出，禁止重复启动已执行条件。原生日志无损压缩可用archive.restored重审；仅后续用户新任务才继续。

## 2026-11-09可飞自由点解的进一步优化

free_refinement_a75_f6_v1_20261109/manage.py为唯一协调，PID4041727/session75018，禁止重复启动。status、summary、best、REPORT与实际worker及原始result共同核对。原input JSON、生产源和worker脚本SHA冻结，文档更新可继续。参考含七窗及返航，未找到/碰撞/任务失效按原规则保留。

## 2026-11-09新续解轨迹实际飞行完成

continued_free_a75_f6_gazebo_v1_20261109已complete，不再重启。71.232秒完成一次正式飞行，validation的same_world_scene_mpc_and_px4及native_logs_restored_and_reaudited均true。原birthday接触/ULog已无损压缩，本地archive.restored可复核。当前无飞行或规划进程，下一步为用户新授权求更好规划。

## 2026-11-09新续解轨迹实际飞行验证启动

当前单次飞行唯一入口实验2/results/continued_free_a75_f6_gazebo_v1_20261109/run_frozen.py，session91688，禁止重复启动。同原acados与单线程BLAS，Gazebo矩阵锁已取得。常规预热后执行固定参考24.637927秒，参考激活后仅一次。出现设施故障先核对reference_activation，再按冻结同配置规则处理。

## 2026-11-09有限NLP失败机制诊断完成

free_center_solver_diagnosis_v1_20261109/status.json已complete，当前无进程，禁止将该批当运行中。REPORT.md给出复现入口、原始起点一致性与全部负结果；validation.json、continuation_stl_and_task.json及safety_radius_consistency.json复核通过。单分支24.637927秒是诊断结果，未飞；原四初值正式矩阵和Gazebo记录保持。无需恢复定时任务或启动新的协调器。

## 2026-11-09有限NLP失败机制隔离诊断

当前诊断目录为实验2/results/free_center_solver_diagnosis_v1_20261109。只读复盘入口analyze.py与passage_analysis.py。固定问题启动控制脚本独立进程运行，最多四CPU隔离进程、单线程BLAS，禁止重复启动已运行对照。continuation_result只作为诊断结果，完整复核前不计新规划成绩。所有旧result与生产文件SHA在protocol和fixed_problem_input中记录，最终核对。

## 2026-11-09独立时间初值扩展验证

本轮唯一运行脚本为实验2/results/expanded_time_initials_a75_f6_v6_20261109/run_frozen.py，session81847，禁止重复启动。新增独立初值索引4至7分别对应0.5/0.6/0.7/0.9，ours/center两方同配，最多四worker。默认worker仍四尺度，扩展可显式--time-scales 1 .8 1.2 1.5 .5 .6 .7 .9。前四原候选沿用，新增与累计计时分别记录。

## 2026-11-08圆窗参数化修正开发

复算入口为wyh python -u convex_timevarying_window/comparisons/论文实验/实验2/results/circle_coordinates_a75_f6_v2_20261108/run_frozen.py，该批已启动，禁止重复启动。单线程BLAS、原NUMBA_CACHE_DIR。原物理四初值与旧批逐值比对，源码和旧原始result的SHA核对。43项回归日志见该目录tests.log。

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


## 2026-11-08三趟失败只读复算

目录论文实验/实验2/results/ipopt_three_flight_diagnosis_v1_20261108，analyze.py读取原三飞行并以archive.restored临时恢复ULog，核对模型需求、实际目标饱和、估计误差与时序。convergence.py重建实际观察器后做九个固定状态30/300迭代对照，使用相同前馈初值，与实飞热启动不同。wyh单线程、原acados环境，均不启动Gazebo或修改生产控制器。当前全部诊断进程退出，REPORT.md、audit.json、convergence.json与输入SHA为入口，不能当新增闭环成绩。

## 2026-11-08最新三组IPOPT参考试飞已完成

结果目录论文实验/实验2/results/ipopt_factorized_three_gazebo_v1_20261108，REPORT.md/PLANNING.md、summary/validation/status、failure_diagnostics与flights原记录齐全。三组均碰撞，0/3性能通过。协调session6762/PID2308843和worker已退出，experiment世界已pause true，replay未操作。新任务不得重启该complete协调器。

执行环境为wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba、GZ_SHOW_GUI=0、ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4及该目录lib。入口soft_dynamics.flight --source <冻结result.json> --method ours --outdir <全新目录>。只读原生日志恢复使用gazebo_matrix.archive.restored，按radius_ablation.execution.audit重审，原压缩SHA保持。4接口测试通过，使用全参考执行时钟和真值事件，碰撞后无有效圈时。

## 2026-11-08当前IPOPT加速入口

最终v3_cached批次已complete，当前无运行协调器。结果入口为论文实验/实验2/results/ipopt_factorized_validation_v3_cached_20261108/REPORT.md；status/validation、逐条件原始result/assessment/validation及finite_calls/finite_problems/cold_starts.csv齐全。正常单条件显式ipopt使用当前参数化冷图缓存与6对历史limited-memory Hessian。30相关测试通过。

v3的report.py可从原记录重生成统计，无需重跑规划或实飞。新实验必须使用全新目录，不能续跑已complete目录。v1和v2也已complete，旧PID已退出，下文v1协调记录仅为追溯。当前四条件所选全部名义几何通过，实飞记录为零。

wyh、PYTHONPATH为主仓库、单线程BLAS、PYTHONDONTWRITEBYTECODE=1及NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。正常单条件入口为python -m convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.worker --level a25_f6 --method ours --solver ipopt --outdir <新目录>。四原解析冷初值、全局七窗后返航，日志分别记录每轮建图编译与纯IPOPT时间。旧实现使用--solver ipopt_legacy，原默认SLSQP仍可显式--solver slsqp。

当前冻结回归为results/ipopt_factorized_validation_v1_20261108/run_frozen.py，session84203，PID1863073，四worker运行期间不能重复启动。核对status.json、summary.json、四progress.jsonl及真实PID，全部结束后核对validation.json的源与旧结果SHA和各条件几何复核。相关测试显式指定soft_dynamics/tests和global_gate_minco_probe/tests。

## 2026-11-08用户指定三组IPOPT全局规划全部完成

结果results/ipopt_requested_three_v1_20261108已complete，三组全部独立新算、各四解析冷初值，17相关测试通过。按a25_f6/a75_f3/a75_f6，完整七窗后返航参考时间15.565861920/19.112578264/29.896435302秒；四初值求解4247.456/2755.796/1608.905秒，另建图分别记录；合格初值2/4、3/4、2/4。选中轨迹原C++完整目标择优，原节点、全部56几何、STL窗框、顺序和端点复核均通过。

采用此前IPOPT辅助变量版，96轨迹加1584软超限量、绝对到达时刻及MINCO偏差，原固定32积分/零倾角/5000迭代/24安全交换/无限墙钟。没有改算法、代价、场景、准入或复用旧解。2555冻结源码和协调脚本SHA核对一致，1477历史原始记录SHA保持。repeatability_check单独记录新算a25_f6/a75_f6与v17选中曲线的一致性，旧结果仅用于事后核对。

三个名义规划窗框结果均clear，实际新Gazebo飞行0。执行模型5ms诊断最大单电机分别9.010197/8.711875/8.882232N，高于名义8.54858N界；软残差单列，未新增拒收。不能将名义几何通过当作已验证实飞。REPORT.md、summary.json、protocol.json、各原始result/progress/assessment/validation完整保存。

协调PID1120974/session42515正常退出，全部worker退出，当前无新协调运行。用户指定这批规划已完成，旧五方法完整矩阵与对应实飞仍未恢复。


## 2026-11-08用户指定三组IPOPT全局规划正在运行

当前唯一协调为results/ipopt_requested_three_v1_20261108/run_frozen.py，PID1120974/session42515。三个独立worker分别a25_f6 PID1121216、a75_f3 PID1121217、a75_f6 PID1121218，均已观察到实际内点迭代。三条件全部独立新算，各四解析冷初值，不复用旧解，最大5000迭代、原24安全交换、墙钟不限，固定32积分/零倾角罚。

显式--solver ipopt选择此前辅助变量分支，96轨迹加1584软超限量，原绝对到达时刻、MINCO偏差、原安全余量和验收。没有改算法、代价、场景或准入。17相关测试通过，2555源码及三组初值/场景冻结，1477历史原始记录SHA保护。仅规划，无本批Gazebo飞行。幅度0.25/0.75按各窗水平宽度比例，3/6为各窗基频倍数。

继续时核对status.json、summary.json、REPORT.md、三份progress.jsonl/日志及主机实际父子进程，不重复启动协调器或改冻结源码。四初值原C++完整J择优，独立复核节点/56几何/STL/任务与执行模型软动力学诊断，全部结束后汇总。原五方法完整矩阵仍未恢复。


## 2026-11-08当前直接SLSQP验证

当前默认入口为python -m convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.worker --level static --method ours --solver slsqp --outdir <全新目录>。wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。SLSQP直接96变量、TOGT正时长映射、ftol1e-8、最多5000、四解析冷初值、原安全交换。

已完成results/slsqp_direct_static_v4_20261108，FINAL_DIRECT_SLSQP.md及validation.json为结果入口。四初值2/4合格16.415551569s，全部进程退出，未实飞。旧IPOPT通过--solver ipopt显式运行，已有v17结果保持。新矩阵尚未恢复，不能续跑已完成/中止开发目录。


## 2026-11-08独立求解修复验证已结束

v17结果independent_soft_repair_dev_v17_20261108，四自由条件四初值全结束，validation_original_acceptance passed，FINAL_REPAIR.md为入口。audit_final.py读取当前原验收规则，核对记录的ACCEPTANCE_CORRECTION和POST_RUN_MAINTENANCE精确差异后重审既有轨迹，不调用新求解或仿真。使用PYTHONPATH仓库根及wyh单线程环境执行，19测试通过。

当前无运行协调/worker。完整五方法十条件矩阵和实飞仍待继续，不能自动把四代表回归当完整实验。未来新批次按当前soft_dynamics/PROTOCOL，固定32积分、倾角0、独立几何/运动/MINCO初值，额外数值诊断不作准入。

## 2026-11-08独立求解修复代表回归

入口convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.regression --outdir <新目录>，四条件四方法16条新规划。使用wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1及NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。当前independent_soft_repair_regression_v1_20261108/session87484运行，不启动第二协调器或修改冻结源。有效算法为解析MINCO初始PVAJ加等价软超限辅助变量内点法，双方固定32积分、倾角0。完成validation后才整理结论。

## 2026-11-08当前无倾角批次已按用户要求暂停

results/no_tilt_five_methods_v1_20261108在27/40规划后停止，3项中断，0实飞，协调和worker均退出。USER_STOPPED.json留证，后续先分析数值求解，不自动续跑。只读证据在no_tilt_solver_diagnosis_20261108/DIAGNOSIS.md。

## 2026-11-08无倾角惩罚四方法新矩阵

模块前缀convex_timevarying_window.comparisons.论文实验.实验2.no_tilt_comparison。build原子重建两原生库，run --outdir <新目录>执行40规划及串行对应飞行。使用wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba，飞行环境ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4、LD_LIBRARY_PATH的lib及GZ_SHOW_GUI=0。当前批次no_tilt_five_methods_v1_20261108已启动，不启动第二协调器或修改冻结源。

## 2026-11-07本文TOGT软动力学完整矩阵

入口convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.run，使用wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。plan动作使用新结果目录，四隔离worker、20条件各四独立解析初值，无墙钟限制。validate-plan复算目标、原初值和几何。

flight动作在规划validation通过后串行执行参考，环境增加GZ_SHOW_GUI=0、ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4、LD_LIBRARY_PATH包含该目录lib。指标为完整参考执行用时，真实终态另列，旧提前结束阈值取消。

正式soft_dynamics_paired_v1_20261107已20规划/20正式实飞结束，全部原生记录重审通过。14趟后起飞前预检失败保留，结果目录resume_flights.py恢复剩余六条，须从仓库根用python -m加载，原十四趟逐文件SHA保持。该脚本仅适用本次冻结停止点，后续新实验使用主入口新目录。PLANNING/FLIGHT、REPORT及各自CSV齐全。当前无协调/worker，世界暂停。

## 2026-11-07全部轨迹实飞完成与只读复核

入口实验2/gazebo_matrix/run，结果gazebo_all_selected_v1_20261107已50/50 complete、49条原生独立复核通过。PID4031204/session98308正常退出，世界原生stats暂停，2460项冻结未变，49趟均attempt_1。结果39无碰撞/10碰撞/1原未找到，当前不重启协调或重飞条件。
使用wyh、单线程BLAS、GZ_SHOW_GUI=0、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。完整跟踪回归设置ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4及LD_LIBRARY_PATH的lib，20测试通过。
只读复核使用同模块validate --outdir <本结果目录>，根据compression_manifest逐趟临时恢复pbtxt/ULog、校验SHA、重验真值/真实窗运动/接触，然后保留压缩源，最多恢复一趟。结果export_results.py仅从已有记录导出CSV/SUMMARY/配对差与SHA，不启动飞行。12.077GiB原生日志压缩1.363GiB。SUMMARY/REPORT/results.csv可直接读取，experiment已暂停，replay未操作。

## 2026-11-07当前Wang复现开发

数学核复现/wang2023_verified，从历史paper_core快照恢复并修复收敛点返回，LBFGS-Lite固定a96c50e3...，Eigen自动取CasADi/include/eigen3。30公式/平台/编译等价/执行测试通过，三开发条件结束并保留源码快照。
正式输出results/wang2023_full_v1_20261107已10/10 complete、validation通过、联合59测试通过，全部进程退出。原2452冻结源与原始十轨迹保持，验证目录单文件修复见HARNESS_REPAIR.json及repaired_run.py.txt。使用wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。
入口python -m convex_timevarying_window.comparisons.论文实验.实验2.wang2023.run，run启动新批次，solve --level <level> --outdir <新目录>作单赛道，validate --outdir <已完成目录>独立复算。N2/M4/s4，4初值各2000ms/5000迭代。当前任务已完成。只读复算使用validate，它会重建全部候选/初值/目标/事件/执行前缀和公共STL/5ms，精确允许HARNESS_REPAIR所列的单文件验证入口差异。

## 2026-11-07完整独立求解矩阵已完成

入口实验2/independent_full_matrix，协议/README在该目录。输出results/independent_full_matrix_v2_20261107，40/40 complete、validation通过。PID3384945/session6435、全部worker及monitor73844均退出，当前无求解进程。
25核验继承+15新算，全部40重新评估；本文20问题/80解析初值及全部找到曲线/节点/56几何已重建。2433源冻结未变，31测试最终53.80s通过，日志tests.log。SUMMARY/REPORT/analysis完整，角速与90%推力预留超限单列。
只读复核使用wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba，同模块validate --outdir <上述目录>。该操作只复算已有40条，不能解释为全域动力学或实飞证书。
v1于0/40、新优化启动前因evidence字段冲突停止，源/错误/STOPPED保留；v2更名dynamics_evidence并增加实际继承评估测试后完成。当前不续跑或自动启动新批次/仿真/容器。

## 2026-11-07独立自由选点回归已完成

正式结果results/independent_free_regression_v1_20261107，5/5 complete、validation通过。协调PID2897374/session57711、开发PID2879703及所有worker已退出，无当前优化进程。2429源冻结、旧2421项逐键内容不变，22测试最终31.14s通过。
当前入口实验2/independent_free_solve/run，从头开放全部自由变量，不求中心子问题。使用wyh、单线程BLAS、PYTHONDONTWRITEBYTECODE=1和/tmp/convex_trajectory_only_numba。
只读复算使用该模块validate与上述outdir，不启动新规划。报告SUMMARY.md与dense_dynamics_diagnostic.json单列a50_f6角速6.575507>3.8及节点间推力余量超限；原验收为节点硬约束与完整名义几何，不能把validation passed解释为全域动力学证明。
开发结果与旧same_course_center_regression均保留追溯，正式五条件全部独立新算。当前任务已完成，不自动新开批次或仿真。

## 2026-11-07最终同赛道中心初始化回归已完成

当前结果same_course_center_regression_v1_20261107，5/5 complete、validation与initializer_validation通过。PID2664087/session79662正常退出，实际无求解进程；原五自由失败均在同版独立新算后得到自由精修收敛解。
算法入口same_course_center_recovery.Planner，运行说明见其README。只读复核使用该模块run validate，outdir为上述目录，wyh单线程BLAS。原始20中心初值/中心与自由全部独立评估的额外证据见initializer_validation.json。
完整结果SUMMARY.md已列所有飞行参考时间、分钟级规划成本、模型/容差变化、节点间规划余量超限与相对旧版变慢的条件。根git diff检查通过。当前阶段不启动新规划、仿真或定时任务。

## 2026-11-07当前同赛道中心初始化五条件回归

当前入口same_course_center_recovery.run，输出results/same_course_center_regression_v1_20261107，唯一PID2664087/session79662，0/5。2426项冻结，每个条件自己的中心4初值和自由精修均本次重算，300s/初值；禁止历史/跨赛道初始化和运行改源。
查status/summary/REPORT/progress与主机实际父子进程。完成后同入口 `validate --outdir convex_timevarying_window/comparisons/论文实验/实验2/results/same_course_center_regression_v1_20261107` 仅独立复算，不启动新规划。
V2 complete9/9/validation通过、PID2427396已退出。新方法文件加入后旧全量source-set断言会变化，旧2421项内容逐键全部未改、旧validation.json保留；不能为跑旧校验删新文件或回滚源。

## 2026-11-07最新固定独立回归v2

当前输出results/independent_cold_regression_v2_20261107，唯一协调PID2427396/session40735，0/9。原四冷初值300s，smooth_augmented，归一化SLSQP ftol1e-5、最终节点验收1e-5及完整几何/STL原判据。源2421项冻结，运行中不改源码或启动新开发分支。
v1的ftol1e-9额外收紧已确认有问题，3条后主动停止，PID2316728/session74892退出130，STOPPED.md保留，不能续跑。并发开发均已清空，当前仅3正式worker。
联合读summary/status/REPORT/progress和主机进程；结束后用independent_repair.run validate，outdir指向上述v2，仅独立复算。无Gazebo/PX4/容器/定时任务。

## 2026-11-07当前独立赛道正式回归

实验2/independent_repair/PROTOCOL.md和README.md，输出results/independent_cold_regression_v1_20261107。唯一协调PID2316728/session74892，0/9，实际子进程已核对，不重复启动。
四原初值各300秒，smooth_augmented固定版本，2421项冻结。运行中不能编辑该目录或tilt_heading_minco/attitude_repair等冻结依赖；根docs状态可更新。只读核对summary/status/REPORT与progress及主机进程。
完成后 `python -m convex_timevarying_window.comparisons.论文实验.实验2.independent_repair.run validate --outdir convex_timevarying_window/comparisons/论文实验/实验2/results/independent_cold_regression_v1_20261107`，wyh和单线程BLAS。会复算全部参考/节点/56几何/姿态/初值与哈希，不开启新规划。
下方只加上限试验已结束，连续法被用户否决，其他开发模块不能当本轮正式算法。

## 2026-11-07当前：独立赛道，仅提高迭代上限

当前协议实验2/iteration_limit_only/PROTOCOL.md，输出results/iteration_limit_only_v1_20261107，9/9完成、2通过7未找到，PID1994383/session12168退出，不重跑。各自原四初值，SLSQP2000/180s。
用status/summary/REPORT、各plans/progress.jsonl及主机真实父子进程联合核对。只读复算入口 `python -m convex_timevarying_window.comparisons.论文实验.实验2.iteration_limit_only.run validate --outdir convex_timevarying_window/comparisons/论文实验/实验2/results/iteration_limit_only_v1_20261107`，使用wyh单线程BLAS。
运行期间原2386项及新冻结2402项不改，不调预算/模型/代价/验收、不启动仿真或容器。

用户继续要求独立修复。新independent_repair仅开发诊断，结果independent_repair_dev_v1_20261107；当前source可能演进，单次代码快照保存在各试探目录。单种子试探不是完整四初值回归。不要使用run_continuation或读其他赛道轨迹作初值。

## 2026-11-06当前：求解失败修复已5/5自由回归通过

入口实验2/attitude_repair；当前算法tilt_heading_minco/FINAL.md，最终调用`attitude_repair.run_continuation`。旧`attitude_repair.run`为仅数值直接4初值，不混用成绩。
结果root为实验2/results/attitude_repair_dev_v1_20261106；同版static_scaled_continuation_*_ours六条已complete，5原失败+最高补充；关键中心a25_f3也complete。不要重跑择优。
只读复算并刷新报告（wyh单线程）：`python -m convex_timevarying_window.comparisons.论文实验.实验2.attitude_repair.validate --root convex_timevarying_window/comparisons/论文实验/实验2/results/attitude_repair_dev_v1_20261106 --require-complete-free`。会保存源快照与重算报告，不启动规划或仿真。
最终高动态中心对照PID1909345已退出，目录static_scaled_continuation_a50_f6_center complete/found=false，lambda=.875不是目标完成；旧中止诊断status已标明，不能凭旧solving字样认为仍运行。当前无修复求解进程。原40矩阵已完成，原协调1510546退出。
显式测试`python -m pytest -q convex_timevarying_window/tilt_heading_minco/tests`，另复核global_gate_minco_probe/tests/test_full_state.py与trajectory_only/tests；裸根pytest不够。无Gazebo/PX4/容器/定时任务。

## 2026-11-06当前：全七窗双因素矩阵已运行，不重复启动

新入口实验2/basic_group/whole_course_difficulty/README与PROTOCOL；输出results/togt_whole_course_factorial_v1_20261106，唯一PID1510546/session97252，当前8/40、准入通过。查真实协调/子进程、progress.jsonl与summary/status/REPORT，不只看状态文件。
全部七窗参与幅度.25/.50/.75窗宽 × 各自频率1/3/6倍，4方法40记录。静态4条继承/重评，成本原测量；动态全新。不改规划器或判据，不在运行中编辑冻结2386项。
33相关测试通过。完成后wyh单线程环境用`python -m convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty.run validate --outdir <上述输出>`复核40条，不启动新规划。
旧togt_layout_mixed_v1已11条停止，PID1367275/1452634退出143，status/STOPPED为准，不续跑。所有旧外围/紧凑批次同样不续跑；无Gazebo/容器/定时任务。

## 2026-11-06当前：官方TOGT布局适配预检

入口实验2/basic_group/togt_course/README，输出togt_layout_mixed_v1_20261106，唯一PID1367275/session42981，不重复启动。先静态4方法及低/最高动态TOGT非相邻风险核验，preflight_gate通过才正式。
分清源布局与适配：upstream.json记录官方commit/blob/原YAML哈希，场地×2.2和z+2、现有小窗、第7窗平面/闭环终点/G3长廊差异见协议。
新的driver仅在隔离进程注入场景/报告/评估接口；复用separated_course预检协调功能，不复用其坐标/法向或成绩，不改规划算法。
旧separated_mixed_v1（8条）与basic_trajectory...（24条）均已停止，不能续跑或拿其进行中REPORT误判；以各status/STOPPED为准。当前无Gazebo/容器操作。

## 2026-11-06当前：新混合赛道先预检再完整40项

入口实验2/basic_group/separated_course/README.md，输出results/separated_mixed_v1_20261106；唯一PID1283797/session97848，勿重复启动。
status先geometry/static/dynamic_preflight，preflight_gate.json通过才formal_planning。geometry通过不等于四条静态参考已通过，更不等于正式实验完成。
旧basic_trajectory_f0p5_to3_a2_v1_20261106已24/28停止，status/STOPPED.md说明，f3_a2自由为中断不判失败；旧PID1110072/1212780已终止，不续跑。
新10档仅W3/W6交替提升，五背景不变，初相0。命令/环境见新README；全部不接Gazebo/控制器。查真实进程及progress，不只看status。非相邻窗风险则停止保留，不能忽略继续。

## 2026-11-06 当前：实验二纯轨迹28项

入口实验2/basic_group/trajectory_only/README.md及PROTOCOL.md。输出basic_trajectory_f0p5_to3_a2_v1_20261106；协调PID1110072/session39628，运行中不要重复启动。
读status/summary/REPORT及当前计划progress.jsonl，并用主机ps核对协调/子进程；默认沙箱ps只能看到自身，不能据此判断主机进程退出。
阶段为静态+2m幅度0.5～3Hz、四方法；独立网格评估，无Gazebo/PX4。完成后`python -m convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.run validate --outdir <输出>`只复核名义结果。
环境wyh，OPENBLAS/OMP/MKL线程1，PYTHONDONTWRITEBYTECODE=1，NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。不能把历史实际碰撞/提前完成时间搬入新名义表。

## 2026-11-06：基础组中间档2Hz2m补充已完成

结果实验2/results/basic_group_f2_a2_v1_20261106/RESULTS.md：四方法均找到参考，本文自由15.888s无碰撞七窗返航，其他三条实际碰撞。协调PID813968/session40149已退出、实验世界暂停，不再启动run。
独立入口仅在隔离进程添加f2_a2=(2,2)，2359既有源与旧12条件一致；不能直接用旧basic_group.validate处理该新KEY。只读原始复核（自动读取.gz、缺失报错、不启动仿真）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 /home/jack/anaconda3/envs/wyh/bin/python -m \
 convex_timevarying_window.comparisons.论文实验.实验2.results.basic_group_f2_a2_v1_20261106.run_supplement validate
```

报告完整时间包含七窗返航；碰撞时刻不算完成时间。旧12条不重跑，未修改共享算法或混入求解流程改进，不操作replay。

## 2026-11-06：基础组完整12条件已完成

主输出实验2/results/basic_group_full_v1_20261106，协调PID689304/session84068已正常退出，原生stats确认暂停；不重启已完成协调器。主结果实验2/basic_group/RESULTS.md；12条件全部结束，独立验收通过。
范围紧凑original三档四全局方法，保留既有静态TOGT两条，余10条件仅各一次。实验协议实验2/basic_group/PROTOCOL.md；34相关测试通过，高动态真实位姿预检通过。
basic_group/validate.py已独立复算全部原始ULog和接触（自动读.gz，缺失报错），结果validation.json；压缩归档后再次验收通过。报告主表时间含七窗返航，未找到/碰撞分开。后续只读复核可用`python -c 'from pathlib import Path; from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.validate import validate; validate(Path("convex_timevarying_window/comparisons/论文实验/实验2/results/basic_group_full_v1_20261106").resolve())'`，不启动仿真。

## 2026-11-06：实验二日志已无损归档，旧审计前先恢复

存储说明`convex_timevarying_window/comparisons/论文实验/实验2/STORAGE.md`；清单及工具在其`results/storage_cleanup_20261106/`。
174份大pbtxt保留为.gz；旧审计不自动读取压缩包，不能以原文本路径缺失判零接触。先按批次恢复（工具检查空间并验证原始哈希），再使用原审计且仍遵守源码冻结校验：

```bash
/home/jack/anaconda3/envs/wyh/bin/python \
 convex_timevarying_window/comparisons/论文实验/实验2/results/storage_cleanup_20261106/maintain.py \
 restore --select compact_layout_px4_v3_20261106/
```

仅查看原文可用`gzip -cd <文件.pbtxt.gz>`，不恢复全批。只读核验命令将`restore --select ...`替换为`verify`。不要重复运行`clean`，不要原地编辑硬链接归档快照。

## 2026-11-06：紧凑七窗等待高度组实飞

当前说明实验2/compact_calibration/RESULTS.md。用户已授权清理并完成无损压缩，空间阻塞已解除；本次没有新实飞，原始失败证据保留。
前两布局4条有效实飞在compact_layout_px4_v3_20261106，当前manifest完全相同，不重飞。新高度参考在compact_layout_probe_v4_20261106，只有名义证据。
不要直接启动compact_calibration.run重跑全部六条；新续跑应核对仅场景/测试源变化、继承4条原始证据和旧参考哈希，再只执行高度两条。
v3/finalize.py是旧朝向冻结批次的审计脚本，v3已完成独立验收；当前高度源已变，不绕过其源哈希保护强行重验旧批。
世界已暂停、协调器已退出，replay未动。当前28相关测试最终通过，diff与主结果链接检查通过。

## 2026-11-06：交错布局六趟完成，只读入口

当前结果为实验2/interlaced_calibration/RESULTS.md，参考probe_v5、实飞px4_v4均已哈希归档到主工作树实验2/results。
不要重启已退出的PID161082，不重飞按结果择优；原生stats已确认实验世界暂停，不操作replay。
只读复算使用wyh运行results/interlaced_layout_px4_v4_20261106/finalize.py；原始证据路径仍指向保留的8cc4结果，主工作树是逐字节一致归档。
规划必须调用interlaced_calibration.planner.solve或该模块CLI，不能绕过到shared.solve；后者使用旧场景，会被协议一致性检查拦住。
目前只验证静态，未自动启动动态矩阵/本文方法。下述椭圆入口仅历史，不是当前选定赛道。

## 2026-11-06：历史七窗三静态布局六趟已完成

范围和结果见`convex_timevarying_window/comparisons/论文实验/实验2/static_calibration/RESULTS.md`；新六趟在`实验2/results/static_layout_px4_v2_20261106`。
不再次启动已结束协调器，不续跑挂载失败v1。需要只读复算/重画布局图，在主仓根用wyh：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
/home/jack/anaconda3/envs/wyh/bin/python \
convex_timevarying_window/comparisons/论文实验/实验2/results/static_layout_px4_v2_20261106/finalize.py
```

新适配显式测试：`python -m pytest -q convex_timevarying_window/comparisons/论文实验/实验2/static_calibration/tests`（9项）。
该校准使用历史七窗shape-specific安全区、sleeve网格和历史SciPy TOGT后端；不要把旧小圆窗/C++桥结果并入。

## 2026-11-06：实验二四全局方法已完成

入口与最终范围见`convex_timevarying_window/comparisons/论文实验/实验2/FOUR_METHOD_PROTOCOL.md`；完整36条见`实验2/RESULTS.md`。
当前无运行协调器，不再继续旧v1/v2/v3，也不重复启动已完成v4；仅experiment容器用于本实验，不操作replay。
在主工作树根用wyh，只读复算原始记录并重建补充图表：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
/home/jack/anaconda3/envs/wyh/bin/python \
convex_timevarying_window/comparisons/论文实验/实验2/results/mixed_global_four_v4_20261105/finalize_results.py
```

显式相关测试：`python -m pytest -q convex_timevarying_window/comparisons/论文实验/实验2/tests convex_timevarying_window/global_gate_minco_probe/tests/test_full_state.py`。
全局规划赛前完成、在线请求0；四初值求解与建图成本独立于七窗返航竞速。不要用旧三方法run.py重建本轮四方法结果。
补充接触复算支持protobuf省略零纳秒字段；原记录不覆盖，以validation.json核对全部结果分类未变。

## 2026-11-05：当前双方四时间初值直接优化

完整17对已完成：`实验1/results/multistart_full_v1_20261105/REPORT.md`，28条新飞+6条源等价保留。
新批次运行（新目录须不存在，且仅新协调/分析文件添加时才允许保留旧六条）：
`python -m convex_timevarying_window.comparisons.论文实验.实验1.multistart.full_run --outdir <new-directory> --reuse <matching-six-flight-directory>`。
原始真值/接触复算：仓库根`PYTHONPATH=. python <result-directory>/validate_raw.py`，不启动仿真、不改原始记录。
完整结果规划中位自由207.05/中心182.96ms，均未100ms达标；不要把下列三组开发验证误作完整矩阵。

`python -m convex_timevarying_window.multistart_minco.flight --method ours --frequency 3 --amplitude 2 --outdir <new-directory>`；
`--method center`使用相同四时间初值，只固定选点。当前两方均不先求中心，旧center_initialized_minco仍为历史两阶段入口。
测试：`python -m pytest -q convex_timevarying_window/multistart_minco/tests convex_timevarying_window/comparisons/single_gate_swing/tests/test_protocol.py`。
同请求诊断和三组六趟配对入口/协议见`convex_timevarying_window/comparisons/论文实验/实验1/multistart/README.md`。
使用wyh/原动态库变量/单线程BLAS；当前无墙钟限，四候选全计时，本轮总周期130～207ms不称100ms在线。

## 2026-11-05：中心可行解初始化的单窗自由精修

当前新入口：`python -m convex_timevarying_window.center_initialized_minco.flight --method ours --frequency 3 --amplitude 2 --outdir <new-directory>`。
原single_gate_swing入口保留旧初始化，不能把旧结果当新两阶段方法。运行环境仍为wyh及原acados动态库变量，单线程BLAS。
测试：`python -m pytest -q convex_timevarying_window/center_initialized_minco/tests convex_timevarying_window/comparisons/single_gate_swing/tests/test_protocol.py`。
开发协议及固定请求/六趟配对报告见`convex_timevarying_window/comparisons/论文实验/实验1/center_initialized/README.md`。
总求解时间包含中心和自由两阶段，同请求名义不劣不代表独立闭环竞速时间保证；目前仅矩形平移单窗实飞。

## 2026-10-13：当前单窗单重射击MPC

单窗配对入口single_gate_swing.flight默认已切换新MPC；旧paired_final_20261012结果保留。
同参考重飞：python -m convex_timevarying_window.togt_style_mpc.frozen_single_gate --baseline-dir <原单窗结果目录> --outdir <新目录>。
使用togt_style_mpc/README中的acados运行环境变量，当前新MPC自身求解为原生CasADi+L-BFGS-B。
七窗可显式--tracker single-shooting；尚未用此新跟踪实飞七窗，不以单窗结果代替。
测试：python -m pytest -q convex_timevarying_window/togt_style_mpc/tests convex_timevarying_window/comparisons/single_gate_swing/tests。
报告与五条验证：convex_timevarying_window/togt_style_mpc/results/tracking_repair_20261013/REPORT.md。

## 点式 TOGT MINCO 单窗重测（2026-10-09）

独立方法说明、命令及原 W1 2.5Hz/3.5m 已运行结果见
[`togt_point_only_mpc/README.md`](../convex_timevarying_window/togt_point_only_mpc/README.md)
和其 `results/f2p5_a3p5_point_only_v1_20261009/REPORT.md`。运行会控制
`convex_seven_togt_experiment` 容器，不触碰旧回放容器；新结果目录必须不存在。
同结构自由／中心配对结果另见
`convex_timevarying_window/togt_point_only_mpc/results/f2p5_a3p5_point_only_pair_20261009.md`。

## 自由纯跟踪版继续升档（2026-10-08）

`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.raise_difficulty_run --baseline convex_timevarying_window/simple_two_gate_soft_only_mpc/results/pure_tracking_v1_20261008 --outdir <new-directory>`。
固定算法、原初相，频率/幅度/同步0.5步长四轮，每路线首次物理窗框接触停止；未碰撞超时不算边界。
看status.json、boundaries.json与REPORT.md；本批测试上限不等于物理临界值。

## 纯跟踪MPC匹配中心对比（2026-10-08）

`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.center_run --free-results convex_timevarying_window/simple_two_gate_soft_only_mpc/results/pure_tracking_v1_20261008 --outdir <new-directory>`。
仅跑已有七条件的中心版、每组一次；冻结源须匹配自由版，配置只允许free_points不同，世界文件须相同。
新目录REPORT.md按幅度/频率排序，status.json显示进行中/结束；失败及证据缺口保留。

## 无名义准入两窗＋弹性MPC（2026-10-08）

误差恢复自由单方复测：`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.recovery_run --outdir <new-directory>`。
固定v3七条件各一次、只ours；源码快照/summary/status保存于新目录，勿覆盖旧v3配对实验。

完整说明：`convex_timevarying_window/simple_two_gate_soft_only_mpc/README.md`。
conda `wyh`：`python -m pytest -q convex_timevarying_window/simple_two_gate_soft_only_mpc/tests`。
串行新结果目录：`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.run --outdir convex_timevarying_window/simple_two_gate_soft_only_mpc/results/<new-name>`。
基线1Hz/2m重跑，随后频率/幅度/同步0.5递增；中心→自由，每条件一次；仅experiment容器。
主指标完整无碰撞时间与碰撞/未完成；原严格审计保留诊断。`status.json`/`REPORT.md`看当前结果；
被中途停止的旧批次另读`COORDINATOR_STATE.json`，不能把原running状态当仍在执行。
旧硬MPC精确重放与新控制快照验证分别为`mpc_replay.py`、`mpc_recovery_probe.py`，不是实飞。

## 简化两窗：同结构中心完整闭环配对（2026-10-08）

主指标包含返航，不能使用W7退出时间。冻结两次串行飞行；只操作experiment、不操作replay。
协议见simple_two_gate_mpc/README.md。输出目录必须不存在；运行中不编辑方法/依赖源码。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/simple_two_gate_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.simple_two_gate_mpc.run --outdir convex_timevarying_window/simple_two_gate_mpc/results/new_pair
```

## 第三窗零VA＋竞速初始化（2026-10-08）

最难原赛道1Hz/2m/初相，一版本一次，与冻结历史中心24.428s比较。编译发生在飞行前，
不是离线预求航路；不要更改正在运行批次的源码。仅操作experiment容器，不操作replay。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/third_gate_racing_mpc/tests convex_timevarying_window/third_gate_stop_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.third_gate_racing_mpc.run --outdir convex_timevarying_window/third_gate_racing_mpc/results/new_run
```

## 两自由窗＋第三窗中心零VA（2026-10-08）

仅最难1Hz/2m/初相，新版本一次；与历史中心24.428s比较，不额外跑中心或上轮估值版本。
协议见`convex_timevarying_window/third_gate_stop_mpc/README.md`；独占experiment，不操作replay。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/third_gate_stop_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.third_gate_stop_mpc.run --outdir convex_timevarying_window/third_gate_stop_mpc/results/new_run
```

## 两窗＋第三窗时间估计（2026-10-08）

新方案位于`convex_timevarying_window/third_gate_time_mpc/`，与完整赛程续接版本分开。
仅原布局1Hz/2m/初相，各变体一次；独占experiment容器，不操作replay。
构图/编译不求轨迹，耗时与在线正常收敛求解分开记录。输出目录必须不存在。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/third_gate_time_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.third_gate_time_mpc.run --outdir convex_timevarying_window/third_gate_time_mpc/results/new_hardest_run
```

正常收敛不等于可行，残差合格不等于实时，七窗通过不等于严格返航通过；各项分别报告。

## 自由末态事件版（2026-10-07）

`event_free_terminal_mpc`才是当前两窗/M2/加密/时限/事件触发且非最终局部末态PVAJ自由的入口。
与旧`event_triggered_mpc`（固定停车末端）和`wang_free_terminal_mpc`（另一N2/M4分支）区分。
使用全新结果目录，独占experiment容器；旧结果不可覆盖，replay容器不动。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.event_free_terminal_mpc.run --outdir convex_timevarying_window/event_free_terminal_mpc/results/new_run
```

## 事件触发单场景 MINCO/MPC（2026-10-07）

协议：`convex_timevarying_window/event_triggered_mpc/README.md`。1Hz/2m/初始相位，单次PX4/Gazebo，
只替换experiment专用容器，不操作replay，不并发飞行。旧f1_a2_v1_20261007不可覆盖。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.event_triggered_mpc.run --outdir convex_timevarying_window/event_triggered_mpc/results/new_run
```

## 当前算法与Wang初始相位27次比较（2026-10-07）

协议在`convex_timevarying_window/comparisons/comprehensive_wang/PROTOCOL.md`。
每条件只飞一次、全局初相位0并保留各窗相对相位；18主比较＋9消融。
必须先验证readiness，正式飞行无GUI，不能同时运行其他飞行。
只操作专用experiment容器；未完成既有目录不能用新路径偷偷覆盖失败。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.comprehensive_wang.run --readiness 复现/polynomial_dynamic_racing_2023/px4_matched_x500/results/readiness_v3_20261007.json --outdir convex_timevarying_window/comparisons/comprehensive_wang/results/new_run
```

结束自动生成图/报告。`--resume`仅用于完整记录边界的中断且源/配置不变；存在未裁决目录时拒绝覆盖。

## 穿窗邻域加密修复（2026-10-07）

保留旧no-postcheck/软目标入口，使用新目录；重复试验首个失败即停止该版本，
协议在`convex_timevarying_window/comparisons/original_fixed_plane_crossing_resolved/PROTOCOL.md`。
新几何测试会安装进程内场景查询，建议和旧测试分开进程运行。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/crossing_resolved_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_crossing_resolved.run --repeats 3 --gui-last --outdir convex_timevarying_window/comparisons/original_fixed_plane_crossing_resolved/results/new_run
```

只替换专用experiment容器，不动replay；--gui-last需DISPLAY可用，原v1_20261007不得覆盖。

## 更保守软目标配对消融（2026-10-07）

仅在专用Gazebo实验容器执行；不会改replay容器。协议在
`convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/PROTOCOL.md`。
使用全新输出目录，不覆盖已完成的soft_v1_20261007：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/tests convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/tests convex_timevarying_window/wang_progress_mpc/tests convex_timevarying_window/comparisons/original_fixed_plane_deadline/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline_soft_conservative.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/new_run
```

更新：2026-09-10。命令入口和参数按源码/方法说明核对；默认从仓库根目录运行，MDG 单独注明。

## Wang 2023 独立论文核心（非Gazebo入口）

原文方程与所有未公开参数的选择见[paper_core说明](../复现/polynomial_dynamic_racing_2023/paper_core/README.md)。
单独安装的是公开LBFGS-Lite优化器，不是作者竞速完整源码；旧匹配版本/成绩保留。

```bash
conda run -n wyh python -m 复现.polynomial_dynamic_racing_2023.paper_core.setup_vendor
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q 复现/polynomial_dynamic_racing_2023/paper_core/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 conda run --no-capture-output -n wyh python -m 复现.polynomial_dynamic_racing_2023.paper_core.benchmark --outdir 复现/polynomial_dynamic_racing_2023/paper_core/results/new_core --repeats 5
```

是短局部数值核验，不产生七窗圈时；输出目录需全新，保留所有超预算、未收敛和约束残差。
不得用本轮实时通过数量代替安全飞行成功率。

## 新论文核心：原位置固定平面PX4/Gazebo测试

[协议](../复现/polynomial_dynamic_racing_2023/px4_fixed_plane/PROTOCOL.md)与
[六条件结果](../复现/polynomial_dynamic_racing_2023/px4_fixed_plane/results/original_layout_v2_20261007/REPORT.md)。
这不是下节矩形环线布局，也不调用旧匹配版规划器；当前第一窗前MPC中止，尚无竞速成绩。
会替换专用`convex_seven_togt_experiment`容器，保留旧回放容器。

```bash
./convex_dynamic_seven_window_gazebo/build_plugin.sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q 复现/polynomial_dynamic_racing_2023/px4_fixed_plane/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh python -m 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.matrix --outdir 复现/polynomial_dynamic_racing_2023/px4_fixed_plane/results/new_run
```

## 固定平面七窗：只对比当前方法与Wang

2026-10-07删除事后几何/动力学复查的独立仿真消融（事件检查仍保留，不是安全部署入口）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/tests convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/results/new_run
```

两档结果：低档24.748s通过，高档过5窗后第6窗真实碰撞；旧算法默认未修改。详见新目录报告。

2026-10-07既有共同过窗时刻限制版（原MINCO＋MPC，不是Checked/PVA）同场景入口：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline/tests convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline/results/new_run
```

原布局0相两档：低档24.876s通过，高档3窗后60s超时；并非完整矩阵。原方法/历史结果不改。
完整证据见该目录`results/deadline_v1_20261007/REPORT.md`。

2026-10-07原始Checked/PVA回归入口（不是MPC，也不是Checked-Soft）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_checked/tests convex_timevarying_window/continuation_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_checked.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_checked/results/new_run
```

同原布局0相两档28.284/57.360s均返航，但高频−5.3mm球净空违反，不能称2/2安全通过。
保留旧可停车机制/PVA，仅场景查询适配。详见该目录`results/original_checked_v1_20261007/REPORT.md`。

2026-10-07后续自由局部末端PVAJ版（独立目录，旧版保留）两条件回归：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_free_terminal_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_free_terminal_mpc.run --outdir convex_timevarying_window/wang_free_terminal_mpc/results/new_run
```

非终局末端PVAJ可变，最终任务返航PVA固定；目前两档退出2窗/1窗后失败，无全程圈时。
报告在该目录`results/free_terminal_v1_20261007/REPORT.md`；运行会替换专用实验容器，不操作旧回放。

最新**原不规则位置**、与新paper_core相同六条件的我们现有free_warm入口：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_ours.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_ours/results/new_run
```

这六次目前均通过W1后参考耗尽，固定末端自相矛盾，见该目录报告；与下述较早矩形环线结果分开。

最新无中途停车、中心初值三窗版本为独立`wang_center_seed_mpc`；两条件已完成16.432/16.816s，
不宣称全面快于旧版。重跑必须新目录，不覆盖已保存结果或恢复旧矩阵混版：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_center_seed_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_center_seed_mpc.experiment --outdir convex_timevarying_window/wang_center_seed_mpc/results/new_development
```

后续公共过窗时刻修复在独立`wang_progress_mpc`，不向旧暂停矩阵混入新版：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_progress_mpc.experiment --outdir convex_timevarying_window/wang_progress_mpc/results/new_development
```

此入口只运行预声明的慢联调与快速回归两条件，不是完整24趟矩阵。

2026-10-07新增独立场景适配与24趟配对协议，详见
[入口说明](../convex_timevarying_window/comparisons/fixed_plane_wang/README.md)。
需要Docker/PX4/Gazebo；只重建专用实验容器，不改旧场景或原方法源码。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/fixed_plane_wang/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.fixed_plane_wang.run --outdir convex_timevarying_window/comparisons/fixed_plane_wang/results/new_formal
OPENBLAS_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.fixed_plane_wang.report convex_timevarying_window/comparisons/fixed_plane_wang/results/new_formal
```

可先以`--pilot`和单独新目录联调；正式输入是Hz和米，不再使用旧运动倍数。
事后单趟用`fixed_plane_wang.audit RUN --method ours`（或`wang`），禁止默认旧赛道审计。

## 环境与输出

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
```

根 `environment.yml` 来自早期 GAP-Step。SC、SIP、RotSync、FAPP、PhaseGuard 等依赖以各自 README 和 requirements 为准；SIP/Planar-RS/PhaseGuard 的严格检查需要 `python-flint`，缺少时不能当成采样安全通过。

已有结果、checkpoint 和外部复现资料可能被 Git 忽略，检出源码不代表它们存在。新实验使用独立结果目录，保留原运行。GAP-Step 的部分训练配置含 `clean_outputs: true`，会清空所配置输出目录；复用结果前应在新配置中指定新路径或关闭该开关。

## 测试范围

### 2026-10-07：当前Wang自由状态事件v4完整矩阵

仅新增编排，飞行源码须与 `event_v4_20261007/protocol.json` 的63项哈希完全一致。
38条件约需40余分钟；不要同时运行其他飞行或重型历史审计。运行中不修改冻结源码。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.wang_free_state_mpc.run --outdir convex_timevarying_window/comparisons/wang_free_state_mpc/results/new_frozen
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.wang_free_state_mpc.report convex_timevarying_window/comparisons/wang_free_state_mpc/results/new_frozen
```

仅进程中断恢复使用 `--resume`；不会覆盖已有趟或重飞算法失败。报告器要求status=complete，
历史只读ULog事件复核存入新批次，不改历史原标签。协议在对应comparison目录。

### 2026-10-07：Wang MINCO 自由状态与中心对照

两批必须串行运行（共用 PX4/Gazebo 实例），自由点/中心的区别仅为局部坐标界，均使用同一 MPC。
下列两个运行命令不能并行执行；各自指定全新输出目录，历史批次不覆盖。
当前入口包含事件修复；每趟除原检查外生成 `passage_audit.json`，
主验收必须再通过“七窗正向各一次/整机离面”，不能只读 `gates=7`。
`results/free_v1_20261007` 是修复前冻结结果；`event_v2_20261007` 保留开发结果，
`event_v3_20261007` 是纠正远处下一窗平面误判后的复测，保留高频假反向中止；
`event_v4_20261007` 再将进度检测统一到新鲜测量时间。不混用版本或挑选最快成绩。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_free_state_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.wang_free_state_mpc.experiment --outdir convex_timevarying_window/wang_free_state_mpc/results/new_free
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.wang_free_state_mpc.experiment --center-only --outdir convex_timevarying_window/wang_free_state_mpc/results/new_center
```

### 2026-10-07：TOGT-MINCO 两条件试测

独立入口，七次 MINCO-s4 + 原优化 MPC，默认跑原始及 r10/t1/a1/120° 两个条件，不是全矩阵。
v1/v2 开发失败和 v3 完成记录分目录保留；使用新目录，不能覆盖已有批次。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/togt_minco_mpc/tests convex_timevarying_window/checked_soft_mpc_isolated/tests convex_timevarying_window/checked_soft_mpc_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.togt_minco_mpc.experiment --outdir convex_timevarying_window/togt_minco_mpc/results/new_probe
```

`protocol.json` 冻结配置/源码；在线 100 ms 含检查与 IPC，构图单列。方法 `RESULTS.md`
记录本次结果与历史同条件来源，不能把两趟通过当总体安全保证或速度优势。

### 2026-10-06：优化MPC完整矩阵

使用独立规划进程和等价QP消元；当前只跑优化版，不继续旧未优化MPC批次。
三个开发条件和正式38条件分开保存；正式批次中不改源码/配置。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_mpc_isolated/tests convex_timevarying_window/comparisons/checked_soft_mpc_optimized/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_optimized.run --outdir convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/new_frozen
# 仅在全部计时飞行结束后生成对比图和共同动力学诊断。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_optimized.report convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/new_frozen --historical-dynamics
```

中断续跑加`--resume`并使用原目录；哈希一致才继续，已有原始飞行不重跑。
用户允许轻微名义超限，但碰撞、负净空、未完成七窗返航和100 ms超限仍失败。

### 2026-10-06：MPC替换跟踪的碰撞条件复测

独立入口保留Checked-Soft规划与原赛道设置；每次指定新目录，旧PVA矩阵当前暂停。
只有控制器变更，不自动恢复旧38条件批次。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_mpc_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.checked_soft_mpc_racing.px4 --motion-rate 10 --translation-scale 4 --rotation-scale 1 --phase 2.0943951023931953 --outdir convex_timevarying_window/checked_soft_mpc_racing/results/new_hard_case
conda run -n wyh python -m convex_timevarying_window.checked_soft_mpc_racing.report convex_timevarying_window/checked_soft_mpc_racing/results/new_hard_case --outdir convex_timevarying_window/checked_soft_mpc_racing/results/new_comparison
```

MPC为1 s滚动预测、直接总推力/角速度；构图/准备与在线耗时分列，真实净空和接触独立审计。
不能把无接触但未七窗返航的试验计为成功；报告保留所有开发失败和准备期异常。

```bash
# 根 pytest.ini 只默认收集该目录
pytest -q gap_step/tests

# 根据修改范围显式选择方法
pytest -q togt_timevarying_window/tests
pytest -q nonconvex_timevarying_window/atlas_dynatogt/tests
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
pytest -q nonconvex_timevarying_window/msr_dynatogt/tests
pytest -q nonconvex_timevarying_window/sip_dynatogt/tests
pytest -q nonconvex_timevarying_window/planar_rs_dynatogt/tests
pytest -q nonconvex_timevarying_window/rot_sync_sc_togt/tests
pytest -q nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/tests
pytest -q nonconvex_timevarying_window/random_dk_sc_dynatogt/tests
pytest -q nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/tests
pytest -q nonconvex_timevarying_window/avs_ppo/tests
pytest -q nonconvex_timevarying_window/phaseguard_rl/tests
pytest -q nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/tests
pytest -q nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/tests
pytest -q closed_loop_deformable_window/fapp_ppo/tests
pytest -q closed_loop_deformable_window/mdg/tests
```

以上是入口清单，不是要求每次全量运行。修改 SC 的 MINCO/动力学或 SIP 认证接口时，还需按 [ARCHITECTURE](ARCHITECTURE.md) 选择受影响调用方。旧废案测试仅在维护相应方法/反例时运行。纯文档改动核对链接、路径、镜像与 `git diff --check` 即可。

## 近期方法

Random-DK 与 Feasibility-Guided CEM 的三窗入口分别是：

```bash
python -m nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window --outdir nonconvex_timevarying_window/random_dk_sc_dynatogt/results/new_three_u_run
python -m nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.multi_window --outdir nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/results/new_three_u_run
```

后者默认读取方法 README 中列出的冻结 SC 基线和两条单窗安全模板；若迁移或清理本地忽略结果，必须用 `--baseline-json` 与两个 `--template-result` 显式提供来源。结果目录必须不存在，避免覆盖历史运行。

七异形闭环正式轨迹的 Gazebo Harmonic 场景位于 `comparisons/seven_unique_dual_constraint_cem/gazebo/`：

```bash
cd nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/gazebo
conda run -n wyh python export_world.py
conda run -n wyh python validate_course.py
conda run -n wyh python gazebo_smoke.py
./run_track.sh           # 有 X11 桌面；只加载实体旋转门赛道
./run_track.sh gui       # 流畅展示版
./run_track.sh headless  # 无显示服务器
./setup_manual_control.sh # 首次下载 QGroundControl 与隔离键盘遥控环境
./run_px4_manual.sh      # PX4 SITL + x500 + QGroundControl 鼠标手动飞行
./run_keyboard_teleop.sh # 可选键盘手动飞行
./stop_px4_manual.sh     # 停止 PX4/Gazebo
```

`gazebo_smoke.py` 需要 Docker daemon 权限，使用 Gazebo Harmonic 容器实际加载世界并核对七个关节角速度。赛道资产版仍只定义场地和门；PX4 手动版由 PX4 Gazebo 桥生成官方 x500，使用传感器、飞控、MAVLink 与 QGroundControl，不运行路径规划或轨迹跟踪。离开该目录后返回仓库根目录再执行其他模块命令。

RotSync 的默认 suite 是 `formal`，不是 smoke；快速回归应显式选择：

```bash
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments --suite smoke --no-animation --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/smoke_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.experiments --suite oblique_smoke --no-animation --audit-dt 0.001 --collision-samples 5001 --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/oblique_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_counterexample --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_fixed_wp_seeded_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_sc_dynatogt_fixed_wp --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_sc_dynatogt_vs_fixed_wp_new_run
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments --suite formal --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/formal_new_run
python -m nonconvex_timevarying_window.rot_sync_sc_togt.single_window_comparison
```

最后一个入口默认生成新时间戳目录。正式判据见 [FORMAL_EXPERIMENTS](../nonconvex_timevarying_window/rot_sync_sc_togt/FORMAL_EXPERIMENTS.md)。

PhaseGuard-RL 当前使用 `train.py` 中的 Python `train(environment, model, ...)` 函数，需要先构造 `PlanningEnvironment` 与 `PhaseActorCritic`；没有可直接套用的完整场景训练 CLI。先读[核心说明](../nonconvex_timevarying_window/phaseguard_rl/README.md)和[最小实验协议](../nonconvex_timevarying_window/phaseguard_rl/EXPERIMENT_PROTOCOL.md)，不要把 `python -m ...train` 写成已可执行的正式训练流程。

SIP 生成与重放示例（第二条依赖第一条生成的具体 run）：

```bash
python -m nonconvex_timevarying_window.sip_dynatogt.experiments --suite smoke --outdir nonconvex_timevarying_window/sip_dynatogt/results/smoke_new_run
python -m nonconvex_timevarying_window.sip_dynatogt.verify --run nonconvex_timevarying_window/sip_dynatogt/results/smoke_new_run/static_1gate
```

Planar-RS 用位置参数传入 run，和 SIP 的 `--run` 不同：

```bash
python -m nonconvex_timevarying_window.planar_rs_dynatogt.experiments --case ordinary --baseline-certificate --outdir nonconvex_timevarying_window/planar_rs_dynatogt/results/ordinary_new_run
python -m nonconvex_timevarying_window.planar_rs_dynatogt.verify nonconvex_timevarying_window/planar_rs_dynatogt/results/ordinary_new_run
```

AVS-PPO 的普通训练/ID-OOD 与极难长方体实验分别按[方法 README](../nonconvex_timevarying_window/avs_ppo/README.md)和[极难报告](../nonconvex_timevarying_window/avs_ppo/HARDEST_COMPARISON_REPORT.md)运行，不混用模型与 checkpoint。

## 其他研究入口

```bash
python -m nonconvex_timevarying_window.atlas_dynatogt.experiments --suite smoke --outdir nonconvex_timevarying_window/atlas_dynatogt/results/smoke_new_run
python -m nonconvex_timevarying_window.sc_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite smoke
python -m closed_loop_deformable_window.fapp_ppo.experiments --suite smoke
```

MDG 使用自己的 scripts/config 路径：

```bash
cd closed_loop_deformable_window/mdg
python scripts/run_benchmark.py --config configs/smoke.yaml --suite smoke
```

执行完需返回仓库根目录再使用其他命令。跨方法比较与本地 Gazebo 的命令分别在[宽域比较](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/README.md)、[速率基准](../nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/README.md)、[Gazebo README](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/gazebo/README.md)中；Gazebo 为本地未跟踪内容，不能假定所有检出都存在。

## 保留的 GAP-Step / DynaTOGT

较新的生成窗口迷宫使用以下入口，`configs/...` 与 `checkpoints/...` 会由 `gap_step/utils.py` 解析到 **`gap_step/` 下**：

```bash
python -m gap_step.train_window --config configs/train_window_smoke.yaml
python -m gap_step.train_window --config configs/train_window_teacher.yaml
python -m gap_step.evaluate_window --checkpoint checkpoints/window_generated/C5/teacher_final.pt --episodes 200 --stage C5
python -m gap_step.visualize_window --checkpoint checkpoints/window_generated/C5/teacher_final.pt
```

例如该 checkpoint 的实际位置是 `gap_step/checkpoints/window_generated/C5/teacher_final.pt`。旧 `gap_step.train --config configs/train_teacher_smoke.yaml` 仍是旧环境入口，不能与生成窗口迷宫的 checkpoint/结果混用。纯教师实验确认 planner auxiliary 默认关闭；课程表与超参数读本次选用 YAML，不从旧根 README 抄固定值。

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic --order G1,G6,G1,G3,G2,G5,G4,G2 --outdir togt_timevarying_window/results/repeated_new_run
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results/smoke_new_run
```

外部复现先确认本地 `复现/TOGT-Planner-reproduction/` 存在，再按其说明运行；历史构建证据见 [TOGT_REPRODUCTION_AUDIT](TOGT_REPRODUCTION_AUDIT.md)。
## 凸多边形/圆形周期三维窗口

### 在线 Checked-Soft 目标函数变体（2026-10-06）

以下命令从仓库根目录运行；新输出目录必须不存在。批量实验会重启专用PX4实验容器，
不能与其他占用该容器的飞行并行。完成计时飞行后再运行共同ULog动力学审计。

```bash
conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_racing/tests convex_timevarying_window/continuation_racing/tests convex_timevarying_window/receding_gate_racing/tests convex_timevarying_window/comparisons/checked_soft_racing/tests
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.checked_soft_racing.run --outdir convex_timevarying_window/comparisons/checked_soft_racing/results/new_frozen_run
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.checked_soft_racing.report convex_timevarying_window/comparisons/checked_soft_racing/results/new_frozen_run --dynamics
```

中断后同一未改源码的批次可给运行器加 `--resume`。协议见
[Checked-Soft](../convex_timevarying_window/comparisons/checked_soft_racing/PROTOCOL.md)。
下面命令属于较早的离线凸TOGT/CEM，不是新在线算法入口。

```bash
pytest -q convex_timevarying_window/tests
conda run -n wyh python -c \
  'from convex_timevarying_window.native_backend import build_native; build_native(force=True)'
conda run --no-capture-output -n wyh python -m convex_timevarying_window.experiment \
  --outdir convex_timevarying_window/togt/results/seven_convex_togt_margin_1p1_body_diameter_20260910
conda run --no-capture-output -n wyh python \
  -m convex_timevarying_window.conditional_dual_constraint_cem.experiment \
  --outdir convex_timevarying_window/conditional_dual_constraint_cem/results/new_run \
  --population 64 --maximum-rounds 50 --repair-max-iterations 600
```
## 2026-10-13 无规划时间上限的单窗升档

环境仍wyh、ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4及对应lib、单线程BLAS、GZ_SHOW_GUI=0。
入口`python -u -m convex_timevarying_window.comparisons.single_gate_swing.extension --outdir <新目录>`，
默认固定4m从4.5Hz起、固定4Hz从4.5m起，步长.5；任一方法失败后完成配对并停止该路线。
`--start-frequency 5 --start-amplitude 5 --stop-on ours`可接续寻找自由点首次失败，已飞条件不重跑。
本场景默认max_solve_ms=None，在线/诊断无墙钟截止，但保留40s完整任务与数值迭代/见证上限。
新修改验证：`python -m pytest -q convex_timevarying_window/comparisons/single_gate_swing/tests convex_timevarying_window/dynamic_passage_velocity_minco/tests convex_timevarying_window/togt_style_mpc/tests`，32通过。
# 2026-10-13离线横移靠近实验

`comparisons/single_gate_linear_approach/offline_plan.py:compute`先在新目录保存各plan.json，
随后wyh环境、ACADOS_SOURCE_DIR与LD_LIBRARY_PATH按togt_style_mpc配置，线程数1：
`python -u -m convex_timevarying_window.comparisons.single_gate_linear_approach.offline_run --outdir <已有8条plan且尚未实飞的新批目录>`。
不重复运行已有protocol.json目录；只操作convex_seven_togt_experiment。
协议见该方法OFFLINE_PROTOCOL.md。旧在线run入口不再用于当前用户任务。

