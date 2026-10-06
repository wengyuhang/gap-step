# 当前项目状态

更新：2026-09-26。本页来自当前源码、方法报告、本地正式运行和 Git 历史；实验数字均标明来源与证据等级。

## 当前重心

2026-09-26最新局部接续开发仍未达到“快于旧方法”：原始r1/t1/a1/0°新增六个PX4试验版本，
三次完整采样/实时通过35.588/37.076/34.464 s，慢于此前默认v7的31.376 s，更慢于
checked25.528/Wang独立复现26.076 s。另有4门净空中止、4门超时、7门后返航姿态超限，均保留。
新私有TOGT软动力学目标、移动平面穿越方向、选择性初始短段、CTBR jerk角速度前馈
仅`--coupled-racing`启用，默认仍v7；当前实验v6实际倾角42.483°>42°，不可推荐部署。
相关65项测试通过，不代替飞行验收；本批MPPI/TOGT单次<100 ms，不代表50 Hz或递归安全。
参考动力学偏好不是硬可行，门外错误侧停滞及延迟后姿态失效尚无普适解决证据。
全部结果/当前机制见[局部接续试验](../convex_timevarying_window/gs_mppi/direct_ctbr/COUPLED_RACING.md)及
[新旧对账](../convex_timevarying_window/gs_mppi/direct_ctbr/results/coupled_targets_20260926/REPORT.md)。
未重跑最难条件/全矩阵/默认PX4；下文此前“最新”为历史批次，不作当前实验验收。

2026-09-26此前按旧方法圈速目标复核：原始r1/t1/a1/0° checked/Wang独立复现历史有效
25.528/26.076 s，直接CTBR v7本轮31.376 s，仍更慢。新增竞速引导最终32.204 s七窗返航、
采样/实时通过，但没提速；其强末端价值停滞、几何对齐版加速度超限、激进种子姿态/加速度
超限与解锁失败全部保留。默认仍v7，新引导显式opt-in，57项相关测试通过。
同PX4参数并不代表同动力学约束：旧checked真值9.179 m/s²/43.849°超出当前8/42，
旧有效标签不改写，不能称完全同约束对比；当前限制未放宽。
共享TOGT动力学目标与执行限值未完全对齐，改点范围未解决该差异。
详见[最新实验记录](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)和
[历史对账](../convex_timevarying_window/gs_mppi/direct_ctbr/results/old_targets_final_20260926/REPORT.md)。
“快于以前方法”的目标尚未达成；以下最难条件相对自身旧版的改善不能替代这一目标。

2026-09-26后续直接CTBR已实现通用连贯穿越/减晃：对准后保留法向速度、候选按穿窗事件切换、
过门前后共同下一门价值、100 ms节点线性噪声、角速度变化代价和提前姿态余量，52项回归通过。
原最难10×/4×/120° GUI PX4：中间v2为34.076 s通过，但同版重复第7门中止；
最新v7同版两趟33.912/37.644 s七窗返航、零接触、10 ms采样与实时验收均通过，相对37.800 s基准快10.3%/0.4%。
实测roll/pitch高频RMS下降25.7%/26.2%，但最大命令跳变2.774→3.795/4.695 rad/s，
不能说完全解决晃动；两固定试飞非总体成功率证明，全部正负结果详见[结果](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)。
额外未来会合候选因PX4停滞撤出，名义加速不当PX4成绩；有限门口判据通过显式选项启用，
父方法默认不变。旧物理限值/真实门框检查不放宽，未获连续安全证书或稳定成功率证据。

2026-09-26 此前用户澄清直接CTBR的yaw目标为规划轨迹切线，而非实测速度方向。
当前普通段取跟踪参考切线、自主接管取MPPI候选预测路径切线，加入角度连续展开、低速保持及曲率前馈；
33项回归通过；随后同一最难10×/4×/120° GUI PX4复飞7/7返航、零接触、37.800 s，
10 ms真值采样净空0.351024 m、全部周期<100 ms，平均优化约22 Hz。估计均速5.475 m/s，
相比旧40.992 s运行主要是路程226.96→206.43 m减少；不单独归因为yaw修改。
速度诊断定位准备引导的无条件近门减速、预测阶段未推进和缺少出口接续价值；后续已实现修订，见上段，仍无独立消融。
见[本次结果](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)。
以下40.992 s及航向统计属于2026-09-24旧航向机制，不当新版验收。

2026-09-24 最难10×频率/4×平移/120°相位的直接CTBR新增平顺CTBR首拍、
机头朝实际速度方向的软引导及穿窗/倾角余量代价；第三人称Gazebo GUI跟随展示。
最终PX4/x500一次7/7返航、零接触、40.992 s，10 ms真值采样审计和全部在线周期<100 ms通过；
实际指令>1 rad/s跳变较上次GUI的98次降为3次，速度>2 m/s时机头误差中位角99.94°降为21.62°，
代价是这两次GUI圈时相差+2.132 s。多次算法中止和两次气压计过期解锁失败均保留；
这不是总体成功率或连续安全保证，见[方法记录](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)。

2026-09-24 直接CTBR已按昨日中心运动24条件矩阵、同一PX4/x500/Gazebo参数完整实验：
严格22/24通过，昨日checked为23/24、Wang 2023独立复现为17/24。
与checked共同通过21组，新方法0组更快，飞行时间差中位数+9.496 s；与Wang共同通过16组也0组更快，差值中位数+8.820 s。
两次零接触但未完赛的失败分别为3×/2×/120°第4门后和10×/4×/120°首门前，均在最新状态下无合格2 s CTBR候选；不能称物理不可行。
全部有效运行源码快照/PX4参数一致、MPPI/TOGT最大75.929/79.884 ms；一次解锁基础设施失败按协议重试并保留。
这是24个预设条件的有限采样PX4证据，不是随机成功率、连续安全证书或组件单独归因。
旧PVA方法有50 ms跟踪预览，新方法直接CTBR，接口差异见[完整对比报告](../convex_timevarying_window/comparisons/direct_ctbr_center_motion/results/direct_ctbr_20260924/COMPARISON_REPORT.md)。
以下原速/10倍固定相位的两次开发试飞是匹配矩阵之前的历史结果，不能代替本次全矩阵结论。

2026-09-24 后续`gs_mppi/direct_ctbr/`针对10倍PX4零候选，加入未来0.8 s已知开口偏移代价、
有限开口中心距离接管守门，以及普通/准备段多引导CTBR候选。最终相同控制器/核函数源码SHA-256的
原速0°与10倍0°两次PX4/Gazebo均七窗返航，分别34.032/34.524 s；零门框接触，10 ms真值采样
最小机体净空0.34569/0.41496 m，MPPI最大70.205/50.319 ms，TOGT最大76.635/76.885 ms。
33项父/子目录测试通过；中间10倍60 s仅3门徘徊、原速W2出口和10倍W1出口零候选中止均保留。
两固定单次结果不能推断成功率，原速最小实测净空略小于规划0.35 m误差余量，
不能声称连续鲁棒安全。完整代价、源码快照和原始记录见[直接CTBR报告](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)。
本节下面先前称“最新”的直接CTBR段落是本轮前的历史状态。

2026-09-24 此前`gs_mppi/direct_ctbr/`把准备触发改为预测动态入口时间及开口对准，把准备和实际入窗后的
候选生成/代价分开。相同源码快照在原速七窗PX4/Gazebo相位0°与120°分别40.492/38.312 s返航，
两次10 ms真值采样审计均通过，零门框接触；MPPI最大64.475/53.212 ms，TOGT最大78.112/77.795 ms。
31项相关测试通过。首次分阶段版本同两相位另有35.396/41.288 s完整飞行；版本不能混排成功率。
10倍频率同模型新版本37.46 s七窗返航，但PX4只完整通过1门后6.264 s候选失效；
阶段逻辑在原速基准成功，尚未解决高频下完整2 s候选可保持性，也未证明连续鲁棒安全。
详见[当前方法和完整正负结果](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)。
下面直接CTBR的4/7门、28项测试等内容为此轮修订前的历史状态。

2026-09-24 后续按用户“直接输出控制量”新增`gs_mppi/direct_ctbr/`：MPPI采样/输出总推力与机体系角速度，
17状态预测包含姿态、实际角速度与电机响应；竞速发送端没有PVA/加速度跟踪器，保留PX4角速度内环。
冻结同模型单窗8/8、七窗4/4返航，37.38–41.34 s；8499周期MPPI最大41.437 ms，116次TOGT完整调用最大65.272 ms。
BLAS单线程消除了已复现的小矩阵线程开销；此前149.55 ms超时结果保留。6873周期>20 ms，不能称严格50 Hz。
这是求解时钟暂停的名义结果，不是Gazebo成绩；输入有界离散非线性投影不继承GS原定理，递归可行性未证明。
新PX4第6次开发完整过4/7门，20.276 s在W5准备阶段零合格候选中止，零接触；尚未全窗完成。
该次MPPI最大29.796 ms，45次TOGT最大76.845 ms，真值采样最小机体净空0.38676 m。
穿窗阶段/状态时间不一致、一次投影不足、稀疏观测粗下界的问题已固定初态复现并修复，相关28项测试通过。
最新正负结果以[直接CTBR报告](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)为准，不混用旧加速度原型圈速。
另更正旧GS批次相位单位：旧`phase=120`实际为120 rad，非120°；新直接版本明确转换为2π/3 rad。

2026-09-24 新增独立`convex_timevarying_window/gs_mppi/`：时变复合障碍函数、输入有界安全滤波、
GPU16384直接加速度控制序列；按最新澄清取消旧动作100 ms安全前置条件，三阶段与整机退出重规划保留。
13单测通过；冻结双积分器单窗8/8、七窗4/4并返航，33.12–37.32 s；7821个MPPI周期最大61.10 ms。
这是求解时钟暂停的六状态名义模型，不能当PX4结果；142个七窗周期超过20 ms。
17状态姿态/电机执行诊断2.92 s、0门后无合格动作，未碰框；加速度瞬时可实现假设不成立，尚未完成目标。
报告`gs_mppi/RESULTS.md`与`DESIGN.md`区分实现错误、证据不足和模型假设缺口；旧方法与基线未改。
以下100 ms旧策略准入等描述属于早期版本，不约束本次新方法。

2026-09-23 按用户三阶段机制新增独立`convex_timevarying_window/three_phase_mppi/`：
普通区间跟踪三窗TOGT，临时避让/误差触发重规划，整机完全出窗触发下一次滚动规划，准备/穿越阶段自主GPU MPPI。
新TOGT不作路径安全验收，计算准入检查当前动作未来100 ms。22项测试通过，但最终同版本名义与PX4各四组均未完成任务。
PX4频率1/10倍×相位0/120度完成4/2/4/3门，全部在PREPARE失去合格后续动作；三组最后100 ms前缀仍通过，
主要暴露短跟踪到完整穿越候选的交接缺口，不是已证明赛道不可行。2677个MPPI周期最大49.448 ms，107次TOGT最大78.495 ms。
最终四组零接触、真值采样净空最小0.411 m、最大定位误差0.133 m，仍不能称安全全窗竞速成功。
已匹配旧论文实验GPS延迟参数，并修正本方法事后审计的四元数反号插值伪影；旧控制器/baseline和所有负结果未改。
结果见[独立方法报告](../convex_timevarying_window/three_phase_mppi/RESULTS.md)，不可用开发旧版本名义成功代替最终验收。

2026-09-23 2023 Wang多项式动态竞速独立复现已按当前checked方法的窗口中心运动矩阵完整运行：
同一七窗/PX4/x500、频率1/3/6/10×、中心平移幅度1/2/4×、旋转幅度1×、相位0/120°，各24个有效条件。
当前方法严格23/24，Wang复现严格17/24；共同严格通过16组中当前12组更快、Wang 4组更快，
Wang减当前飞行时间差中位数+0.710 s。Wang有4次Gazebo门套接触、3次零接触但10 ms真值采样净空为负，
另有1次解锁基础设施失败按协议重试；最大完整规划周期68.124 ms，未出现>=100 ms周期。
最强10×/4×两相位，当前32.004/52.668 s均严格通过；Wang相位0撞第4门且60秒超时，
相位120°七门返航32.760 s但采样净空−0.108 m，均无合格圈速。
这仅比较有披露差异的独立复现，非作者官方成绩；两个固定相位不估计总体成功率，采样安全不等于连续认证。
原始证据和逐条件表见[匹配实验报告](../convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/COMPARISON_REPORT.md)。

2026-09-23 V3 CTBR按用户要求修订前三项算法逻辑：入口恢复进度、持久穿窗方案、远期预警/近端接管分离。
统一实际/预测反馈限幅，原生复核与法向工作区区间界；92项方法测试通过。原七窗开发5次有效飞行仍全部未完赛，
另1次解锁失败；当前`ctbr_contract_px4_20260923_06`为1门、15.712秒短段预测离开反馈域，非安全竞速成功。
完整控制周期中位45.96/最大83.84 ms，335周期均>20 ms、0次>=100 ms；TOGT153/153接入。
真值采样净空0.71816 m、零接触，但最大绝对定位误差1.40 m，原0.35 m假设仍未成立。
前三项为已实现/已单测，不能写成闭环已全部解决；见方法CONTRACT_REVISION.md，全部负结果和旧版本保留。

2026-09-23 当前`--checked --tracking-preview 0.05`两窗可停车方法新增窗口中心运动矩阵：
固定七窗/PX4/x500，中心平移幅度1/2/4倍 × 频率1/3/6/10倍 × 两个固定相位，旋转幅度1倍；
24个有效飞行严格通过23个，另有一次飞控解锁基础设施失败按协议重试并保留。
未通过者为6倍频率/1倍平移/相位0，七门返航、零Gazebo接触但W7真值10 ms采样净空−0.00648 m。
全批次完整规划周期最大74.343 ms。10倍频率/4倍平移两相位32.004/52.668 s，慢例直到18.876 s才接收首条计划，
说明大幅高速平移可能显著拖慢竞速；不以23/24估计总体成功率。
Gazebo模型实测W1三轴采样行程1.406/1.020/0.780 m，与规划器位置查询一致；
原视频10倍频率只缩短周期，原始单轴±0.10–0.20 m位移不变。
实验协议及结果见[中心运动扫描](../convex_timevarying_window/comparisons/amplitude_racing/README.md)。

2026-09-20 V3新增CTBR反馈分支：GPU16384、完整状态反馈机动、实际穿窗阶段、100 ms短段复核，
修正遥测时钟/年龄、高频实际几何检查与位置—速度不一致。77项方法测试通过。
单门PX4完整退出3.684秒；历史_08为60秒6门超时，当前默认_18为60秒3门超时，均无门接触但未完赛。
_18 MPPI最大89.61 ms，TOGT466/466接入，独立反馈短段复核43次最大13.20 ms；执行间隔故障改善不等于竞速解决。
真值采样净空0.42839 m，但绝对定位误差最高1.46 m超过0.35 m假设；不宣称鲁棒安全。
名义CTBR32.62秒不是PX4成绩。原代码和基线保留；方法COMMITMENT_RESULTS.md记录所有失败和证据边界。
TOGT高频接收已验证改善；两段式候选/末端速度代价未稳定改善，保留为非默认消融项。不可将经验内环/反馈域写成鲁棒安全证明。

2026-09-20 V3穿越控制后续：完整出门/出口速度/后续转向的GPU反馈提案，保留移位安全方案，
条件式穿越期限防止滚动推迟；模型一致阻力补偿与姿态阻尼，在线局部视界2.5秒。
同模型开发已七门+返航成功（42.70/42.86/43.70秒），但冻结v2仅1/3完整任务成功，三次均过七门。
v3三种子冻结3/3完整完成45.22/55.22/48.12秒；但5次新PX4诊断全部未完赛，最好60秒5门，均无接触。
修复发送线程按时激活、请求前更新当前PVAJ、正常/危险模式以及60 ms电机接入；当前版本另做v4回归，
v4独立回归3/3名义完成47.20/44.02/38.66秒；MPPI最大56.85 ms、0次超100 ms，但TOGT worker34次超时丢弃。
不能把名义完成当PX4成功。54项测试通过；反馈加速度采样试验5门超时，已撤回并保留快照。
详见dynamic_gate_mppi_v3/PASSAGE_CONTROL.md；未改两论文baseline或赛道。
下方45项/仅1门是此前历史阶段，不是当前版本验收。

2026-09-20 最新V3：实际门序与参考时间表分离，修复正常跟踪项被门序掩码关闭的问题；
实际P/V与模型估计A/J启动三窗规划，新增跟踪误差触发且不直接切避险。
按用户澄清改为当前执行方案未来100 ms无危险即可启动，取消额外200 ms/0.15 m/三周期条件。
45项回归通过；GPU16384原七窗两轮60秒均仅1门超时，第二轮删除自主门前停留目标后仍绕行未合法通过。
MPPI最大分别21.312/20.368 ms，不能称七门成功；详见dynamic_gate_mppi_v3/TRACKING_REPLAN_REPORT.md。
本修订未运行新Gazebo/PX4，以下200 ms/未来PVAJ等段落是历史状态。

2026-09-20 最新澄清：V3每次完整过门产生三窗滚动更新事件，重规划pending/running不再控制避险模式。
正常继续跟踪、仅当前风险触发自主MPPI；增加generation防止旧门序参考覆盖。40项回归通过。
GPU16384的60秒开发仍1门超时，但完整出门后gate_exit请求成功返回并接受；风险与滚动调度链已分开，不代表整圈成功。
GPU16384验证gpu_gate_rolling_20260920_01；此前GPU PX4 20秒试飞0门、零接触、每轮至少1555合格样本，
906个完整MPPI周期中位14.579/最大24.632 ms，不是竞速成功，也不代表新事件逻辑结果。

2026-09-20 GPU后续：V3批量采样、预测、筛选、打分和加权已迁移CUDA，默认16384候选，CPU仅共享查询/最终FP64准入等。
33项回归通过（1.35秒）。RTX A4000起点16384候选完整select中位7.416 ms、最大8.798 ms（12次，不是最坏保证）。
16384候选单门18/18完整退出；8192候选原七窗60秒仅1门超时，但无缓存耗尽，合格候选每轮68–3303。
GPU_REPORT.md记录证据；PX4接入目录gpu_px4_20260920_01，不冒充正式矩阵。以下“未接GPU”为较早开发记录。

2026-09-20：V3当前源码已加入零参考避险、前缘入窗FSM、局部不绕窗/完整退出准入、几何反馈提案；28项回归通过。
原七窗同模型开发仍仅1门，最新7.16秒缓存耗尽，非PX4。末段候选合格数0而MPPI最大34.069 ms，非内存耗尽。
源码仍为CPU OpenMP，未接GPU；不能写GPU性能。记录见`dynamic_gate_mppi_v3/PASSAGE_REVISION.md`。
以下9月19日16矩阵属于旧冻结快照，不能代表当前开发源码。

2026-09-19 V3后续：ROS 2/Agent通过临时代理转发完成准备，PX4四电机直控输出已验证，
模式切换约0.5秒并移到原起飞准备期；原失效代理不再阻塞本实验，全局配置未改。
开发60秒PX4试飞0门、零接触、超时；不是成功。修复执行队列与“3周期”计数后20项测试通过。
完整16条件冻结集成评估已结束，目录`paper_baselines_2026/results/v3_px4_frozen_20260919_02`，status为finished。
严格0/16，全部0门、60秒超时、零接触；真值10 ms最小净空0.406621 m，不是安全竞速成功。
41483个MPPI周期中位18.952、P99 26.782、最大41.242 ms；13213次超过20 ms，0次超过100 ms。
所有记录求解周期含初始TOGT最大76.200 ms，实际启动间隔中位24 ms，不能声称严格50 Hz。
25文件冻结哈希及全部条件与旧矩阵一致；20项回归通过。报告和图已生成，不具备三方法成功圈速排名条件。
`_01`首项为调度计数修复前诊断，保留但不计入最终矩阵。
以下“接口未接通/未运行”是此前阶段记录；本轮不重跑两论文，也不把RL未验证迁移写成0/16。

2026-09-19 新增用户PDF算法 [dynamic_gate_mppi_v3](../convex_timevarying_window/dynamic_gate_mppi_v3/README.md)：
动态三门TOGT参考、四电机推力MPPI、区间碰撞筛选、完整出门计数与延后异步重规划核心已实现；
18项测试通过。当前只有同模型开发诊断，未完成PX4动作通道验证或正式16条件实验，不能与两论文排圈速名次。
当前代码60秒开发测试仅过1门，任务超时；开发中缓存耗尽等失败均保留，20 ms周期也并非每拍满足。
现有MAVLink不是四电机直控接口，ROS 2/Agent缺失且镜像下载受失效代理阻断；未改全局代理或旧飞控。
结果详情与证据边界见方法README和results中的每次summary，不将数值模型时间写成Gazebo圈速。

两论文队列现已结束，状态为`finished_with_rl_transfer_not_validated`：2023矩阵16次中8次严格通过、
7次完赛但安全验收失败、1次解锁失败；2026完成约5亿步训练后仍未通过静态PX4迁移。
以下“执行中”段落为此前启动记录，不再是当前队列状态。原结果未在V3工作中重跑或覆盖。

2026-09-19 两篇baseline执行进展：2023多项式独立实现通过直线7门理想跟踪（8.86 s）及
原动态七窗开发相位0.4 PX4（25.116 s，零接触，10 ms真值净空0.179022 m，最大周期68.028 ms）；
同布局静态开发测试过4门后碰撞超时，保留失败。2026状态RL独立核心完成2500万步pilot，
最终开发样条集2/12、开发最优模型4/12；静态七窗PX4迁移0门、1.980 s越界，未验证成功。
43项相关测试通过（9.64 s）。已启动[串行实验队列](../convex_timevarying_window/comparisons/paper_baselines_2026/README.md)：
2023基线16条件 → 2026基线继续至5亿步 → 静态验收 → 动态诊断/矩阵。
当前运行状态以`comparisons/paper_baselines_2026/results/two_papers_20260919/status.json`为准；
不能将排队任务写成已完成结果。学习方法的动态信息和返航终态尚不对齐，失败不能宣称作者论文性能。

2026-09-19 baseline选型更新：用户排除视觉论文；学习方法改选2026预印本
Bridging Performance and Generalization in Reinforcement Learning for Agile Flight
（arXiv:2606.27348）的原生状态版本，目录`复现/agile_generalist_rl_2026/`。
多项式规划基线保留arXiv:2306.14461。选型时仅复现准备；后续执行状态见上段，没有完成三方正式比较。
新学习论文原文未验证动态门，后续动态扩展须单列，CRL下载记录保留但不参与比较。

2026-09-19 后续：[可停车延续方法](../convex_timevarying_window/continuation_racing/README.md)采用两窗与停车末段联合优化、整个局部计划采样准入、少量自适应检查点、连续参考及50 ms平台预览。
冻结原幅度16条件全部七门返航、零接触；15次严格物理/实时通过，1次真值日志缺失单列，不能写首轮16/16严格通过。
旧3个失败条件均通过；2446个完整规划周期最大90.614499 ms，最小真值采样净空0.022683 m。
12个共同成功条件平均快2.482 s，但强双幅度条件35.108/35.604 s并未全面提速。
源码、配置及PX4参数冻结，未逐门特调；新布局与日志补测单独保存于方法results/holdout_checked_20260919。
两个新布局均严格通过（44.920/36.752 s）；缺失真值条件独立补测25.372 s通过，不替换首轮缺失记录。
这不是连续域或递归硬安全证明，经验跟踪管仍无已证明误差上界；开发失败完整保留。

2026-09-19 [幅度冻结实验](../convex_timevarying_window/comparisons/amplitude_racing/README.md)完成16次PX4/Gazebo试飞，13/16物理及实时通过。
同一算法/余量下，1倍频率的1/2/4倍双幅度均2/2；10倍频率对应2/2、2/2、1/2；仅平移4倍和仅摆角4倍均1/2。
最大完整周期83.092 ms。三次失败分别为配置工作区超限、实际门框碰撞、计划过期中止后0.104 s碰撞。
因此既有小幅高频12/12不能外推为大幅运动安全；备用动作和跟踪误差界仍未解决。

2026-09-18 后续修复：默认改用实际时刻局部穿越约束，替代全周期最坏加速度的最低法向速度。
同一频率/相位矩阵12/12通过：1/3/6/10倍平均33.344/33.985/33.263/32.536 s，
全部零门接触、严格返航，真值采样最小净空0.354877 m、六侧房间边界通过；最大完整周期88.831 ms。
旧式以`--legacy-crossing`保留；低频圈速没有改善，不称全局最优或连续安全证明。
来源：[新轮结果](../convex_timevarying_window/comparisons/motion_rate_racing/README.md)。
论文独立复现按用户要求新建[复现/waypoint_mppi](../复现/waypoint_mppi/README.md)：
CPU/GPU刚体MPPI核心、测试及场景入口已实现。直线七门数值闭环完成；动态七门长视界过6门后超时；
PX4原视界、长视界和直接CTBR适配均未完赛，不能宣称成功复现论文性能或完成有效文献比较。

2026-09-18 完成[动态频率冻结扫描](../convex_timevarying_window/comparisons/motion_rate_racing/README.md)：
同一七窗、x500、算法和振幅，1/3/6/10倍频率各3个固定相位。1倍3/3、均时32.296 s；
3倍3/3、均时32.556 s；6倍及10倍各0/3，未接受任何规划并悬停超时，不算安全成功。
全部规划周期观测值低于100 ms；成功飞行有零门框接触及真值采样审计，不是连续安全证明。
用户同意独立复现 Beyond Reference Trajectories 的 GP-MPPI；当前PX4适配未验证成功，
失败试飞只能作集成诊断，不能据此宣称优于论文。实际速度、穿越点和轨迹图已生成。

2026-09-18 新增[凸动态窗口三窗口滚动竞速](../convex_timevarying_window/receding_gate_racing/README.md)：
在线联合过窗点/时刻/PVA，直接接 PX4/x500，不读取离线整圈轨迹且没有门编号特例。
含房间边界的 v10 冻结主方法原赛道 `33.640 s`，相位变体 `34.416 s`、布局变体 `32.356 s`；
三次均完成七门零接触、严格返航和真值采样房间验收，最长完整规划周期 `77.415 ms`。
早期 v9 部分轨迹越过仅有 visual 的墙面，不能计入当前成功率。
同配置固定中心消融更快（`30.652 s`），自由选点性能优势未证实；也未低于粗 TOGT 名义
`24.660806 s`。发现并修正仿真 GPS 时间戳的 110 ms 错配，真值只用于事后 ULog 审计。
细节、开发期失败、约束代理及无连续安全证明的边界见[结果报告](../convex_timevarying_window/receding_gate_racing/RESULTS.md)。
同日按用户要求将全部窗口平移/旋转频率提高 3 倍，算法、余量、飞控配置不变；
`px4_20260918_112610` 完成 33.444 s、零门框接触、最大规划周期 76.830 ms，
真值采样最小门套净空 0.356 m、房间检查通过；这是一次小幅快速运动试飞，不是普遍安全结论。

仓库已从连续二维生成迷宫的纯 PPO 教师，发展为多个独立问题族和多种规划/学习方法。最近提交集中在非凸时变窗口：8 月的整机安全约束、SC/SIP 比较与认证加速、AVS 安全强化学习，以及 9 月的 RotSync 和 PhaseGuard-RL。因而“二维 PPO 是唯一主线”不再适合作为全仓定位；GAP-Step 的历史实验仍保留其局部约定。

当前目标包括非凸可行域内选点、穿越时间优化、整机在穿越前后连续时间内的安全、动力学限制，以及在相同物理模型下评估规划与学习的实际贡献。固定平面旋转任务、完整平移/RPY/缩放任务和连续局部形变任务不能互相代替。

## 方法状态与结论边界

2026-09-10 新增并列的 [凸时变窗口 TOGT 实验](../convex_timevarying_window/README.md)：
沿七异形闭合赛道的位置布置 2 个矩形、3 个精确圆、1 个正五边形和 1 个正六边形，所有窗口均作周期三轴平移和
roll/pitch/yaw 三轴旋转。求解严格复用 TOGT 发布 standard 目标及 L-BFGS 数值配置。
当前后端直接封装发布版 C++ 手写四旋翼梯度和 MINCO 伴随回传；凸多边形改用论文的
平方重心映射，平面圆使用 Ball 映射的二维同构，D 维数为 `4/2/5/2/6/2/4`。所有
TOGT margin 已进一步提高到外接球直径的 1.1 倍 `0.8347300205 m`，严格使用发布版
Rectangle/Ball/Pentagon/Hexagon 各自不同的缩量规则。当前冻结运行正常收敛，飞行时间
`18.724959468 s`。最新验收直接使用发布版 C++ `QuadManifold`；最大单旋翼推力
`5.015527 N > 5 N`，安全验收 W1/W3/W5/W6 通过，
W2/W4/W7 仍分别侵入约 `97/301/310 mm`。故动力学和安全验收仍均失败。指定穿越时刻的点净空足够，但 margin 不约束相邻时刻的轨迹与运动门框。结果是最大 1 ms 步长的密集采样证据，
不是连续域认证。

同目录新增[条件式双约束 CEM](../convex_timevarying_window/conditional_dual_constraint_cem/README.md)。
它先求不带安全项的名义 TOGT；名义安全软积分非零时，激活带手写梯度的安全 L-BFGS，再直接
对完整 `[K,D]` 做 Pareto 双约束、完整协方差 CEM。最新正式运行把安全积分加密到每段 16–64
区间、20 ms 目标步长，并在优化外接球上增加 30 mm 余量。385 个候选（含种子）中有 116 个
严格双零。最终动力学已改为直接调用发布版C++状态映射，并补齐倾角 `6.28 rad`、派生总推力
`1–20 N`及原始权重记录；此前Python复算误判候选324失败。重新运行时，按飞行时间排序的第1条
候选 `id=324` 即完整通过，`T=21.872307580 s`，两个软积分均为零，最大单旋翼推力
`4.955388 N`，七窗最小真实球体净空 `50.609726 mm`。总墙钟 `276.740909 s`，其中联合
L-BFGS `213.048734 s`、CEM `47.402488 s`。运行期间源码
哈希一致；结论仍是最大 1 ms 步长的名义模型密集采样证据，不是连续域认证。

2026-09-11 在独立目录新增[联合可行时间精修](../convex_timevarying_window/joint_feasible_time_refinement/README.md)。
该方法从上述已审计候选 324 出发，联合时间、TOGT 动力学软约束和整机安全积分产生
缩时方向，再通过完整 `[K,D]` homotopy 搜索与独立双审计保留可行解。首次正式运行将
`21.872307580 s` 缩短到 `20.123161245 s`（`7.997%`）。1/0.5/0.25 ms 三档加密网格均通过；
0.25 ms 下最大单旋翼推力 `4.999994223 N`，最小净空 `34.941080 mm`。这是单场景的密集采样
数值证据，不是全局最优性或连续域证书。

2026-09-11 完成 [TOGT vs Conditional CEM 冻结 24 赛道 pilot](../convex_timevarying_window/comparisons/conditional_cem_vs_togt/README.md)。
四档难度各 6 条，所有场景引用相同的已知周期窗口模型、`[K,D]`、TOGT C++ 动力学和 1 ms 审计器。
在冻结的 `120` 次安全恢复迭代、`16`种群、最多 `10` 轮 CEM 预算下，TOGT 为 `0/24` 联合通过、`24/24`
碰撞；Conditional CEM 为 `19/24` 联合通过、`5/24` 碰撞。TOGT 没有与新方法共同可行的实例，因此
不对它们的飞行时间作“可行解对比”；图中将其作为失败返回轨迹标示。完整原始表、图和汇总见
[正式报告](../convex_timevarying_window/comparisons/conditional_cem_vs_togt/results/pilot_24_frozen_budget_20260911/REPORT.md)。这是小预算单次冻结批次，不是对一切 CEM 预算的统计结论。

2026-09-10，Dual-Constraint CEM SC-DynaTOGT 的安全软积分已补入该新方法的前置 L-BFGS：目标为原 TOGT 目标加固定权重的全轨迹可微安全积分，梯度经 MINCO、窗口时间导数和 SC Jacobian 回传到原生 `[K,D]`。原始 `sc_dynatogt/`、Fixed-WP 未修改，多中心与停滞重启未加入。七窗口无内缩旧负结果早于此修改，保留作问题定位，不能当成当前流程实验结果。

同日完成无历史轨迹/候选关联的无内缩冷启动正式运行。L-BFGS 用时 `2356.510 s`，CEM 用时 `102.775 s`，预处理、L-BFGS、CEM 合计 `2466.570 s`。512 条 CEM 候选中有 24 条当前离散网格严格双零且完整动力学均通过，但 24 条真实外接球检测全部失败，故没有验收解。失败暴露出每段最多 32 区间的全段安全积分会漏掉 `0.02–0.18 s` 的短接触区间；软积分为零仍不能替代密集安全检测。

三窗最新成功原型：新增 [Feasibility-Guided CEM SC-DynaTOGT](../nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/README.md)。它显式复用两条单窗硬筛选通过轨迹的安全相位，以周期别名生成 2820 个多窗前端候选，再从峰值速度最低的三窗几何通过者启动 7 轮、每轮 256 个样本的完整协方差 CEM。总计 4612 次评估得到 3 条全部中间硬约束通过轨迹；最短者 T=7.390546627 s、最大速度 6.999106268 m/s，三个窗口最终真实姿态长方体审计均为零碰撞采样，最小门框净空 50.215/266.307/58.077 mm。总运行 278.328 s，其中最终审计 156.572 s。失败样本只指导提议分布，最终排序只含全约束通过者。结果是单场景、单种子的名义模型采样证据，不是连续认证或一般成功率结论；详见[三窗结果](../nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/THREE_WINDOW_RESULTS.md)。

曲线边界三方法对比新增一条 11.2 m 开放赛道，依次为利马松、五瓣波浪曲线和直线–三次 Bézier 混合窗口，三窗固定平面并分别以 1.5/-2.0/2.5 rad/s 自旋。原始 Fixed-WP 为 3.878918907 s、原始 SC-DynaTOGT 为 3.497544115 s，两者整机碰撞约束均通过但旋翼推力约束失败；因此不进入合格时间排名。Feasibility-Guided CEM 在 552 个候选中找到 309 个中间硬筛选通过者，最终选择 3.468057675 s，动力学与三窗真实姿态长方体碰撞审计均通过。结果见[曲线赛道报告](../nonconvex_timevarying_window/comparisons/curved_rotating_sc_fixed_wp/results/three_way_20260909/REPORT.md)，证据仍为密集采样而非连续认证。

七窗口混合压力案例依次为均衡 U、利马松、星形、均衡 U、五瓣波浪、Line/Bézier、均衡 U，平面和中心固定且窗口只做面内自旋。四个插入窗口沿此前独立通过验收的三 U 轨迹布置，所以这是有已知可行种子的流程验证案例，不作为未知赛道无偏基准。Fixed-WP/原始 SC-DynaTOGT 分别为 8.666957342/8.666957333 s，二者均超速且在首个 U 窗口发生整机碰撞；Feasibility-Guided CEM 返回 7.390546627 s 的硬约束合格种子，七窗整机审计全部通过。结果见[七窗口报告](../nonconvex_timevarying_window/comparisons/seven_mixed_reference_sc_fixed_cem/results/three_way_trial2_20260909/REPORT.md)。本轮 256 个局部扰动均未新增硬约束合格解，必须保留这一负结果；采样通过不等于连续认证。

新增七种不同形状的分散闭环压力赛道，参考 `20260717_paper_irregular_closed` 的中心和姿态布局，起终点均为 `(-16,4,3.2) m`，窗口固定中心/平面且只做面内自旋。原始 SC-DynaTOGT 正常收敛到 `T=24.127878371 s`。按上传约束的有限零厚度门框模型，仅在全部 `|z|≤r_s` 区间做最大 `0.2 ms`、临界处 `0.05 ms` 球体检查；W5 波浪、W6 Line/Bézier、W7 均衡 U 碰撞，W1–W4 通过。结果与赛道图见[七种形状闭环实验](../nonconvex_timevarying_window/comparisons/seven_unique_sc_sphere/README.md)。若把开口外整张无限平面视作实体，闭合轨迹的回程平面交叉会产生结构性碰撞，因此该口径不用于这条有限门框赛道。

三窗最新续跑：按用户要求进一步扩大独立随机扰动，D 半径比例为 1/2/4/8，K 直接噪声比例为 0.5/1/2/4，共 8000 候选。57 个通过三窗球体和顺序检查，但全部超速，最低最大速度仍为 10.333593 m/s；无最终整机检测候选。搜索 89.793 s，总计 92.015 s（重放原 SC，无本次求解耗时）。未决、边界/C3 数值超限与数值失败均保留。详见[宽范围三窗结果](../nonconvex_timevarying_window/random_dk_sc_dynatogt/MULTI_WINDOW_WIDE_RESULTS.md)。

同日最新多窗试验：新增 Random-DK 三个自旋 U 窗口的开放赛道，固定平面、18 rad/s，使用原生四段 MINCO、4 个 K 标量和 3 组二维 D。4000 个候选中 877/12/4 个依次通过前 1/2/3 窗球体检查；最后 4 个均超速，最低最大速度仍为 10.57885 m/s。没有合格解、没有调用最终整机检测；见[多窗结果](../nonconvex_timevarying_window/random_dk_sc_dynatogt/MULTI_WINDOW_RESULTS.md)。单窗成功不能外推为已完成多窗修复。

2026-09-09 当前新增任务为 [Random-DK SC-DynaTOGT](../nonconvex_timevarying_window/random_dk_sc_dynatogt/README.md)：固定中心/平面、窗口自旋；原始 SC 最终解不合格时直接随机扰动 D/K。用户提供的球体距离约束仅检查全部球平面接触区间，全程动力学仍为硬筛选；只在全部合格候选中选最短时间，真实整机留到最终检测。零厚度均衡 U、18 rad/s、初相位 1.1 rad 的首次两组各 300 候选失败，负结果完整保留。用户随后要求扩大范围；D 半径比例扩大至 0.25/0.5/1/2，K 加噪声比例至 0.1/0.25/0.5/1，重放原解。各 400 候选仍失败；各 4000 候选时分别找到 1 条合格轨迹，T=3.167446933/3.523313547 s，最终最大 0.2 ms 加密整机与动力学检测均通过、零碰撞样本。两者都来自 D/K 联合扰动；详见[扩大范围结果](../nonconvex_timevarying_window/random_dk_sc_dynatogt/EXPANDED_SEARCH_RESULTS.md)。这是本地未提交原型的单窗采样修复结果，不是连续认证或普适成功率证据。

| 方法 | 当前实现/已有证据 | 未完成或不能推出的结论 |
|---|---|---|
| [AtlasDynaTOGT](../nonconvex_timevarying_window/atlas_dynatogt/README.md) | 非凸三角 chart atlas、时空联合优化；历史 default 记录为 14/14 场景成功 | Hermite 轨迹及指标与 SC/MINCO 不同，不能直接当同模型速度基线 |
| [SC-DynaTOGT](../nonconvex_timevarying_window/sc_dynatogt/TEST_RESULTS.md) | Chang 边界重采样、Clipper2 内缩、SC 内部映射、degree-7 MINCO；E0–E5 default 已有记录 | 6/6 名义穿越合法不代表整条轨迹动力学通过；强运动演示明确记录 `sampled_dynamic_limits_satisfied=false` |
| [MSR-DynaTOGT](../nonconvex_timevarying_window/msr_dynatogt/TEST_RESULTS.md) | SC 外层多初值和时间修复；formal 775 个任务、9,300 行；A2/A3 采样动力学可行率均 100% | 是密集采样验收；A3 代价较大，匹配起点数/墙钟预算时 A2/A3 选择相同结果，不能无条件宣称多初值更优 |
| [SIP-DynaTOGT](../nonconvex_timevarying_window/sip_dynatogt/COMPLETE_ALGORITHM_SPECIFICATION.md) | SLSQP witness 循环和 Arb 连续域认证，真实原始曲线、姿态长方体与动力学硬约束；有证书重放入口 | 仅名义模型、局部优化；不保证全局最优、固定求解时限或真实跟踪鲁棒性 |
| [Planar-RS-DynaTOGT](../nonconvex_timevarying_window/planar_rs_dynatogt/TEST_RESULTS.md) | 固定平面排除加原始曲线认证；普通单窗端到端 37.59 s，六窗极难赛道约 30 分 6 秒获认证 | 仅固定中心/平面、面内旋转和统一缩放；不能推广成任意赛道一分钟求解 |
| [RotSync-SC-TOGT](../nonconvex_timevarying_window/rot_sync_sc_togt/README.md) | 固定局部点 Sync 以 PVAJ 连接七阶 MINCO；有正式赛道、现实尺度和单窗对照入口 | 仅固定平面法向匀速自旋；整机碰撞/动力学验收采用密集采样，不是 SIP 连续域证书 |
| [Interpolated-RotSync-SC-TOGT](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/README.md) | 旧双输入线性 Sync 可复现；主入口已扩展为 8 点七阶 SC 输入曲线和可变速单调法向曲线，解析到 snap 并以世界 PVAJ 连接七阶 MINCO | 高转速形状诊断虽显著削弱尖峰，仍未通过动力学或短于 Fixed-WP；安全与动力学为密集采样验收，不是连续域证书 |
| [AVS-PPO](../nonconvex_timevarying_window/avs_ppo/TEST_RESULTS.md) | 动作掩码与可恢复盾牌；三窗平移/球形模型 200 ID + 200 OOD 回合均完赛、零违规 | 这是特定确定性模型的实验统计，不能外推为姿态/电机/不确定性下的硬安全保证 |
| [PhaseGuard-RL](../nonconvex_timevarying_window/phaseguard_rl/README.md) | 精简核心：相位/状态观测、点/时间动作、固定 MINCO 轨迹、认证准入、一步完整规划 PPO 与测试 | 尚未正式场景训练和性能实验；`train.py` 提供 Python 函数，尚不是配套完整场景 CLI |
| [FAPP-PPO](../closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md) | 外生非周期开放日程、连续局部形变、未来预览和残差 CTBR PPO；有验证训练与机制演示 | 最终 ID pilot 为 0/10，后期退化；早期成功视频不能替代最终模型结果 |
| [MDG](../closed_loop_deformable_window/mdg/docs/IMPLEMENTATION_STATUS.md) | 移动安全圆盘、时空图、DP、MINCO 适配与 Lazy Repair；E1–E6 smoke 已记录 | 正式 2,090 次实验矩阵尚未全量运行，完整验收待完成 |

[WBSC/CWB/Exact-Area](../nonconvex_timevarying_window/README.md)已置于 `废案/`，作为旧方案和非凸整机反例保留；不能再列成当前新增主方法。

## 必须保留的负结果

- **RotSync 正式赛道**：2026-09-02 报告中 D1/D2 完整通过，D3/D4 虽几何、C3、闭合与采样零碰撞通过，最大速度仍为 7.1894/7.1040 m/s，超过 7 m/s 上限，必须保留失败标记。见[正式结果](../nonconvex_timevarying_window/rot_sync_sc_togt/FORMAL_EXPERIMENTS.md)。
- **RotSync 单窗口比较**：2026-09-05 的 L/U/star × 三档转速九对案例中，两方法均 9/9 合格，固定点基线全部更快。当前结果未证明 Sync 性能优势；两方法同时改变选点与同步结构，不能单独归因。见[方法 README 的比较记录](../nonconvex_timevarying_window/rot_sync_sc_togt/README.md)。
- **双 SC 输入 Sync 斜向单窗验证**：2026-09-07 本地运行中，新方法的两组优化输入距离为 0.121204，映射端点距离为 0.113786 m；C3 最大跳变 2.737e-14，最大 1 ms 动力学网格限制通过，实姿长方体 0/5001 碰撞采样。原 Sync 对照也通过轨迹采样验收，但优化器异常停止；两种状态分开记录。详见[独立方法 README](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/README.md)。
- **零厚度固定/双 SC 输入对比**：均衡 U、尺寸比 1.9、4.5 rad/s、初相位 1.1 rad 下，窗口厚度改为零，Sync 两端定义为规划球与平面相切（法向坐标 `-rho/+rho`）。对照已纠正为固定一组 SC 输入的原 RotSync，不再用两段普通 MINCO Fixed-WP。运行前 0–4 阶导数嵌入误差不超过 `7.088e-13`；将固定输入最终解精确热启动到双输入法后，两者均为 `T=5.811201508 s`、`J=5.829428767`。两者均为 `0/5999` 碰撞样本，但 TOGT-code 1 ms 审计都有 192 个动力学违规样本，轨迹验收失败。详见[当前对比报告](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_nested_warmstart_togt_code_unlimited_20260908/REPORT.md)。
- **Fixed-WP 反推双输入初值**：保留普通两段 MINCO Fixed-WP 对照，从其零厚度轨迹反求 `-rho/+rho` 相切时刻和两个局部点，再做 SC 逆映射。反推初值 `T=2.245913723 s`，但 `0.087751947 s` 的 Sync 段导致初始动力学惩罚 `504891.2374`；无预算优化后为 `T=2.674461462 s`、`J=2530.901485`，独立 1 ms 审计有 2704 个动力学违规样本。普通 Fixed-WP 在两相切面之间仍是 MINCO 多项式，不是插值 Sync，因此它不是新参数化的可行子集。详见[反推初值报告](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_fixed_wp_seeded_togt_code_unlimited_20260908/REPORT.md)。
- **SC-DynaTOGT 单点自由选点对比**：零厚度均衡 U 窗口上，Fixed-WP 和 SC-DynaTOGT 共享两段 degree-7 MINCO、目标、求解器和起始轨迹，只开放一个二维 SC 穿越点。运行前嵌入的目标及 0–4 阶轨迹误差均为零；自由选点将 `T` 从 `2.486309570 s` 降到 `2.340844067 s`（`5.85066%`）。但两条轨迹的原生 1 ms 动力学审计均失败，自由解的 SC 数值回代点超出安全多边形 `5.122 nm`，故仍保留失败标记。详见[自由选点对比报告](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_sc_dynatogt_vs_fixed_wp_unlimited_20260908/REPORT.md)。
- **ICRA 实验三聚焦转速**：2026-09-07 本地正式运行 L/U 两形状、偏轴可行/轴心过窄几何和 0/1.5/3/4.5/6 rad/s，共 10 场景。Fixed-WP 10/10、Optimized-MINCO 9/10、SC+Sync 6/10 通过；U 形 4.5 rad/s 的 Optimized-MINCO 同时碰撞与动力学超限，L/U 的 SC+Sync 都在 4.5/6 rad/s 动力学实质超限。所有超限都大于 5% 敏感性带；证据为最大 1 ms 加密采样，不是连续域证书。见[实验报告](../nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/focused_results/REPORT.md)。
- **AVS 极难六窗**：严格盾版用 16.15 s 完成闭环，但平均可行动作比例 0.08764、masked entropy 为零。应记录为盾牌几乎接管，不能作为 PPO 学习成功证据。见[极难比较报告](../nonconvex_timevarying_window/avs_ppo/HARDEST_COMPARISON_REPORT.md)。
- **FAPP-PPO**：100 次更新/102,400 步验证训练的最终 ID 为 0/10，两个 nominal 基线各 1/10。旧记录诊断穿越后切换势函数目标带来负奖励跳变；修复和正式复验不能仅凭诊断标为完成。见[测试记录](../closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md)。

## 跨方法比较与本地工作

[SC/SIP 宽域快速六窗比较](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/README.md)保留 SC 整机实体相交和动力学违规，以及 SIP 最终连续域认证。该案例经过续跑、witness 批量补充和规划裕量调整，是历史压力案例，不是冻结协议后一次性无偏基准，也不能推出任意赛道上的普遍优劣。

[运动速率基准](../nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/README.md)有 12 种子 × 3 速率、36 实例的冻结输入和独立审计实现。需分别报告实体相交、15 mm 净距违规和未决状态；本页不因入口已提交就声称全量结果已验收。

审计开始时以下两部分为 **Git 未跟踪的用户本地工作**；2026-09-07 已在 ICRA 方案目录下实现并执行收窄后的实验三：

| 本地内容 | 已有内容 | 尚不能宣称 |
|---|---|---|
| [ICRA_EXPERIMENT_PLAN.md](../nonconvex_timevarying_window/实验方案/ICRA_EXPERIMENT_PLAN.md) | RotSync 的五项实验设计；[实验三收窄版](../nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/README.md)已实现并运行 5 个转速场景 | 其余四项实验已实现或执行；聚焦实验也不能证明 Sync 优势 |
| [Gazebo 适配层](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/gazebo/README.md) | 六窗世界导出、运动桥接和启动脚本；说明周期平移/RPY 接入 | 均匀缩放已完整复现、PX4 闭环接通、实物验证或替代 SIP 证书 |

工作区文件可能不随 Git 克隆存在。后续状态更新应重新查看 `git status`，不要永久把它们标成未提交。

## 保留研究线

GAP-Step 的较新入口是 `window_maze_env.py -> train_window.py / evaluate_window.py / visualize_window.py`。旧 2026-05-15 记录为 ID 71.5%、窗口 OOD 54.0%、迷宫 OOD 74.5%（各 200 回合），是历史验收数字，不是本轮测试。旧 `env.py/train.py` 和 passage、planner/BC 辅助代码仍存在；纯 PPO 配置默认不启用辅助，不应通过抹去代码事实维持“纯 PPO”叙述。路径及配置见 [RUNBOOK](RUNBOOK.md)。

DynaTOGT 在 `togt_timevarying_window/` 使用动态窗口、离散热启动、L-BFGS-B 和 Hermite 轨迹，保留重复穿越任务支持；SC 等目录的 MINCO 后端不代表此目录已替换为 MINCO。TOGT C++ 复现的历史构建结论见[复现审计](TOGT_REPRODUCTION_AUDIT.md)。

下一步见 [ROADMAP](ROADMAP.md)，变更依据见 [DECISIONS](DECISIONS.md)，提交时间线见 [TASK_LOG](TASK_LOG.md)。

## 2026-09-24：动态七窗直接CTBR安全过滤竞速变体

[direct_ctbr](../convex_timevarying_window/gs_mppi/direct_ctbr/RESULTS.md)保留三窗TOGT与三阶段CTBR-MPPI，
把完整2 s预测从硬准入改为竞速/风险评分；当前20 ms动作仍按已知移动门、整机几何和物理限值复核，
局部投影遗漏可行动作时在CTBR空间做小规模候选搜索。最难的10×频率、4×中心平移、120°相位
PX4/Gazebo固定试飞7/7顺序过门并返航，36.912 s，10 ms真值采样最小整机净空0.34089 m，
MPPI/TOGT最大62.558/77.237 ms，零门框接触。相同条件 checked 冻结结果52.668 s；旧直接CTBR
版本无有效圈时。各方法各一次固定试飞，不能推出总体成功率或通用优势；离散安全滤波不是CBF证书。

## 2026-09-09：双软约束 CEM 的七异形闭环验收

新增 [Dual-Constraint CEM SC-DynaTOGT](../nonconvex_timevarying_window/dual_constraint_cem_sc_dynatogt/README.md)：删除按窗口设计的结构化前端，直接在原始 SC-DynaTOGT 最终 `[K,D]` 周围更新完整协方差。每个候选并行计算 TOGT 动力学软积分与含 15 mm 规划余量的可微门框安全软积分；固定 `epsilon=1e-6` 仅作为排序数值平台，合格仍要求两个原始积分严格等于零。首次严格双零后再搜索 5 轮。

在七种不同形状的分散闭环赛道上，共冻结 3072 条随机候选；首次严格双零位于第 42 轮，随后完整运行第 43–47 轮。按飞行时间排序的首条严格双零候选 `id=2701` 即通过独立硬验收，`T=24.709999069 s`，两个软积分均为零；全程 1 ms 动力学检测和七窗口真实外接球检测均通过，最小真实球体余量 `13.878932 mm`。Fixed-WP 与原始 SC-DynaTOGT 分别为 `24.217926422/24.127878371 s`，但二者动力学与球体碰撞检测都失败。正式结果见[三方法报告](../nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/results/formal_final_post5_sphere_only_20260909/REPORT.md)。平面重复穿越只保留为诊断，不参与有限门框球体碰撞成败；所有硬验收仍是密集采样证据，不是连续域证书。

2026-09-10 新增[无内缩 SC 消融](../nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem_no_inset/README.md)。七个原始物理开口 SC 建图全部成功，但改前的原始无内缩 SC 将 W6/W7 点推至物理边界，50×64 个 CEM 候选无严格双零。已知成功轨迹可以等价映射到无内缩参数化并保持双零，说明失败来自时间/动力学基线与局部 CEM 的搜索盆地，而非无内缩参数化排除了可行解。该实验暴露的安全软项未进入前置 L-BFGS 问题已在新方法封装中修复；旧负结果保留，改后完整实验尚未运行。

## 2026-09-10：七异形闭环 Gazebo 场景

同一赛道已导出为[Gazebo Harmonic 场景组](../nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/gazebo/README.md)。当前赛道源是独立的 `course_spec.py`：多边形门由工程尺寸顶点定义，三类曲线门由解析方程或 Bézier 控制点定义，并仅为 Gazebo 网格按 2 mm 弦偏差离散；赛道生成和验证的 `algorithm_inputs=[]`，不读取 SC 预处理、安全内缩或规划结果。赛道资产世界采用室内实验场风格，使用 Gazebo 内置 DART 与 1 ms 步长，85 mm 可见门套就是实际碰撞体。PX4 手动世界保留同一场地与七门几何，按官方 x500 配置使用 4 ms 步长，由 PX4 Gazebo 桥生成 x500 并接入 IMU/GPS/磁场、PX4 SITL、MAVLink 和 QGroundControl；支持鼠标虚拟摇杆与独立键盘遥控，不含路径规划或轨迹跟踪。算法结果仅能通过显式 `--with-replay` 加入另外两个复放世界；计入 10 mm 门框和 2 mm 网格误差后，旧正式轨迹保守净空为 `1.878932 mm`。Gazebo/PX4 手动飞行不替代连续域证书或飞控跟踪实验。

## 2026-09-11：凸动态七窗口 MPC 跟踪状态

[凸动态七窗口 Gazebo/PX4 目录](../convex_dynamic_seven_window_gazebo/README.md)现有实时倍率 1.0 的 100 Hz 全状态 MPC 跟踪入口。它保留 TOGT 引用控制器的 `N=20、dt=50 ms` 预测结构，通过一次实时迭代和独立 MAVLink 发送线程达到约 100 Hz，并按官方 x500 电机转速/推力关系做总推力逆映射。公开 TOGT 包不含 Agilicious 与 300 Hz INDI 内环，因此当前 PX4 体轴速率接口是适配实现，不应描述成论文控制栈逐行复现。

正式结果均为失败：新算法轨迹在 W1 首次接触前位置 RMS `0.2147 m`，参考时间 `3.332 s` 接触；TOGT baseline 为 `0.2807 m`、`3.152 s`。两者都未达到 10 cm，也都未通过 Gazebo 整机碰撞验收。规划阶段的新算法名义安全通过、baseline 名义安全失败的结论保持不变；首次接触后的冲击发散数据不能用作正常跟踪精度。

同目录现有在线 C++ MPC 对照。最终 100 Hz、4 s 运行平均求解 `5.104 ms`，W1 接触前 RMS `0.2075 m`，参考时间 `3.344 s` 接触；因此 Python 调度不是 20 cm 级误差的主要来源。C++ 构建脚本会生成编译数据库，宿主 GCC 构建和 Clang 语法检查均通过。

同日的无控制误差 Gazebo 回放直接使用官方 x500 SDF 动态刚体，在每个 4 ms 物理步覆盖参考位姿/速度/角速度，首次门框接触才释放刚体。TOGT x500 baseline (`24.660806 s`) 和已验收候选 (`28.822504 s`) 都完成七门零接触回放；因此先前 PX4 W2 接触来自闭环跟踪而不是名义参考误差。最终安全口径已统一为 x500 SDF + Gazebo sleeve 网格；规划外接圆只作为保守搜索代理。

## 2026-09-17：凸动态七窗口 VT-GP-MPPI 在线闭环

新增 [online_safe_mppi](../convex_timevarying_window/online_safe_mppi/README.md)，保留 TOGT 联合更新穿越点/时刻的思想，但在线只搜索 7 个候选时刻和中心附近 0.10 m 信赖域。当前门的局部穿越点在初次选定后锁定并随窗口运动；普通事件重规划只更新到达时刻，可行 rollout 枯竭时才允许换点，停滞时退回中心。下层使用 24×50 ms、1024-rollout GPU MPPI、短时可达集门框裁剪、1.2 s 局部屏障、8 m 进窗走廊、2.5 m 离框段和最终一步命令重验证。找不到已验证命令就中止，不执行未检查的“最不坏”动作。

v8 冻结实验中，主方法 `10/10` 完整成功，中心点对照 `9/10`；中心种子 8 在无已验证一步命令时安全中止。共同成功的 9 对中，主方法 6 胜、2 负、1 平，平均快 `0.0778 s` (`0.105%`)；bootstrap 95% 区间 `[-0.667,+0.578] s`、单侧 Wilcoxon `p=0.270`，提速很小且不显著。主方法 5 ms 球体—门框审计的跨种子最小净空 `0.2289 m`，采样动力学全部通过；MPPI/选点规划/完整控制周期观测最大值分别为 `94.372/2.173/99.749 ms`，全部低于 100 ms。普通 Linux/CUDA 下的 0.251 ms 最坏余量不是硬实时证明；5 ms 审计也不是连续域证书或 PX4/Gazebo/真机结果。

2026-09-17 已新增 x500 PX4 Offboard 适配器并完成一次真实 Gazebo 闭环：`convex_dynamic_seven_window_gazebo/results/px4_online_safe_mppi/20260917_115103/` 为 7/7 顺序过门、返回终点、七门实体接触均为 0 的成功运行。它用 50 ms 的在线 MPPI 外环加 PX4 P/V/A 内环，不是参考轨迹回放；MPPI/完整外环最大耗时为 `62.908/69.981 ms`。实际课时 `145.268 s`，显著慢于简化模型的 73.715 s，不能称为竞速性能；该差距是下一步的速度调度、门前制动和内环执行模型研究对象。

2026-09-18 完成当前竞速版本：`convex_dynamic_seven_window_gazebo/trajectories/robust_togt_racing_1p25_100hz.npz` 用 C4 局部同步核对安全 TOGT 路径做 `1.25×` 时间尺度变换，W7 联合保留 `0.29 m` 局部偏移。PX4 只在 W7 前启动实测相位守门，对齐后穿越并恢复返航。最终复现 `results/px4_togt/20260918_021544/` 为 7/7 顺序合法穿越、七门零接触、严格返回终点，实际圈时 `40.216 s`；在线周期均值/P99/最大值 `1.125/1.923/2.719 ms`。`1.20×` 版在 W4 有 1 次旋翼接触，作为速度边界负结果保留。当前结论仅限该冻结七窗赛道的 PX4/Gazebo 采样与实体接触证据，不是连续域或真机认证。
