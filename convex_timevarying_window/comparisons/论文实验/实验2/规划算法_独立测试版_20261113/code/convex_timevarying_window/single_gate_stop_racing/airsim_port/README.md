# AirSim 迁移接入记录

## 当前入口：恢复外包球（2026-10-10用户明确要求）

### 2026-10-11 增加间距、保持算法和参数的试飞

新增 `--spacing-scale`（默认1）。本趟设1.5，将七窗相对起点的水平偏移统一再放大50%，原布局总尺度由2倍变3倍；保持高度、朝向、起终点、4Hz/4m、开口1.886280447倍及arrival9/speed9/tail1.5。规划/求解器/MPC/计窗不变。

`results/native_sphere_spacing15_seed9_f4_a4_v1_20261011`：前5窗整机通过，但71.177491s飞向W6时实际撞到非目标W7框杆，未返航、无完整竞速时间。6条接纳计划/130失败请求；W5等待新计划22.315s（含参考停车后17.196s）。全程最大跟踪误差8.527mm、碰撞前约.206mm，说明碰撞不能归因于大跟踪偏差；当前单窗参考不约束非目标W7。MPC最大32.123ms、接纳规划最大76.841ms。按实际放大场景重新计窗一致，冻结源码核查无变化。详见该结果目录REPORT.md。

### 仅调初值参数的最新负结果

`parameter_probe`保留两轮离线筛选。仅调speed=12、tail=1的原生实飞仍只通过第一窗，15.434546s人工中止，无接触、不完赛。

第二轮将原时间初值常数6.5暴露为arrival_seed_speed（默认保持6.5），筛选32组参数，不改时间初值公式/目标/硬约束/求解器/MPC/重规划机制。选arrival=9、speed=9、tail=1.5，三个历史状态均在最多3次90ms请求中找到候选，不是单次稳定保证。42项相关测试通过。

同组参数实飞`results/native_sphere_seed9_arrival9_tail15_f4_a4_v1_20261010`：首条25.439ms；第一窗4.365166s整机出窗；第二条5.568594s接纳32.775ms，在旧轨迹停车之前接上，7次规划失败请求保留。但7.915144s飞向第二窗途中实际撞到第七窗框杆RaceGate_07_00，原生碰撞记录明确，状态collision。MPC最大30.670ms、记录最大位置误差约2.04mm，源码未变。没有七窗返航、没有有效完整时间，不能称问题全解决。当前单窗规划不对其他窗框增加避障，未擅自通过移动第七窗/放宽碰撞判据掩盖此失败。

`flight.py`只使用新机外包球半径（当前读取并向上取整为.728m），删除--box-body活动分支，恢复原单窗球形距离约束、局部复查与原SLSQP求解流程；原生GenericQuad动力学/PWM跟踪保持。裕量.04m，旧盒子模块及其负结果保留供追溯，不进入当前飞行入口。

放大窗试验参数仍通过`--aperture-scale 1.886280447`显式指定，默认尺度1；窗口运动固定4Hz/4m。18项测试通过，包括当前球形Single确实复用原球形几何/求解器的回归测试。此切换尚未新增实飞成绩；下述--box-body命令仅为历史记录，当前已不支持。

### 外包球原生实飞v1（2026-10-10）

`results/native_sphere_scaled1886_f4_a4_v1_20261010`已实际执行原生PWM闭环：首条接纳规划33.343ms；第一窗中心穿越5.439181s、整机出窗5.493886s，未记录碰撞。出窗触发第二窗规划，但未得到接纳候选；第一条轨迹执行到虚拟停车点后悬停。14.203815s人工中止，累计47次失败请求、只接纳1条轨迹，源码核查无变化。状态development_interrupted，不是竞速完赛；无完整七窗返航时间。最近第二窗失败包含SLSQP线搜索失败/节点约束违反及预算到期，不能认定窗口物理不可行。原始flight、plans、planning_attempts、mpc和contacts保留。

2026-10-10。用户已选择不安装UE，允许现有Blocks网格近似迁移。以下新进展优先于旧准备记录。

## 原生闭环接入进展（2026-10-10）

`scene.py` 用91个原生Cylinder搭建七窗，圆窗为24边外切近似，其余保持多边形边。
开口坐标使用窗口API的(u,v,normal)，不是原始STL的局部轴。原布局、频率4Hz、振幅4m、原初相保留。
移动探针已纠正：simSetObjectPose对不变位姿也会返回false；physics_enabled=false的圆柱也可以移动，
因此窗框不启用自由刚体物理，不会受重力下落。MessagePack管线更新91件约75ms，不能声称实时更新。
采用simPause/simContinueForTime暂停步进：暂停时引擎时钟不变，推进.003秒通常实测.003000064秒。
v1.8.1的continueForTime使用墙钟定时器，并非严格物理tick计数，曾观察到0或6ms推进；必须审计实际时间戳。
近框8m内每步更新，远框24ms更新，明确不是完整连续运动/实时在线验收。
近框采用步中点零阶保持，3ms名义步长下最大正弦位移差上界约0.151m，不能忽略此近似。

`model.py/mpc.py` 独立使用GenericQuad质量、惯量、旋翼混控、二次阻力、5ms电机滤波；
以原生moveByMotorPWMs闭环驱动，不使用位置瞬移或轨迹回放。使用环境API读取空气密度。
`planner.py` 保留单窗两段MINCO、窗内自由点/速度、停车终点及整机出窗重规划，仅替换原生动力学界。
开发测试5项通过。`native_control_probe_v1_20261010`原生起飞到赛道起点并悬停通过，
最大跟踪误差0.007201m、MPC最大25.385ms、实测准备控制步长42.000896ms。
Cube碰撞探针报告CollisionProbe实体接触；它只验证生成网格碰撞通路，不是圆柱窗框或七窗安全认证。
`native_race_v1_f4_a4_20261010`第一窗实际发生边框接触，已保留原记录并增加COLLISION_CORRECTION.json。
官方getMultirotorState不填碰撞字段，现已改为独立simGetCollisionInfo；不能用旧空碰撞列表认定安全。
碰撞前最大跟踪误差约3.8mm，碰撞后才出现大误差。原x500半径不适用原生约0.98m宽的碰撞机体。
`native_race_v2_f4_a4_20261010`使用0.728m包围球，首窗未找到接纳解，开发中断，不算完赛。
原生接触探针支持水平半宽约0.5m；目前测试随姿态盒体与窗框厚度带的局部相交约束，8项单元测试通过。
盒体版本尚未完成七窗，长预算诊断也不算在线飞行成果。虚拟停车点去掉继承的6m法向深度上限，
按半窗间距设置；这一改动尚无成功飞行证据，不应写成已解决问题。
首窗独立诊断v3–v9均保留原始probe.json，没有找到最终局部复查通过的解；
部分v6候选仅优化节点通过，新增危险时刻后失败，不能记为安全轨迹。
v6的2cm余量仅为离线诊断，飞行默认10cm未改变。v9/ v9long的自由加速度是开发消融，
默认仍保持原PV内部点；未据此宣称完成适配。归一化TOGT变量采用[-1,1]的覆盖性规范代表，
不缩小物理开口。求解失败时保留较好已评估迭代仅作下一次初值，绝不直接执行。
`presentation.py`为91件窗框设置原生高对比纹理，不改几何/控制。

运行入口（结果目录必须不存在）：

`python -m convex_timevarying_window.single_gate_stop_racing.airsim_port.flight --outdir <新目录>`

`--prepare-only`只运行原生起飞悬停验证；不计七窗比赛。

## 2026-10-10 用户授权全权适配后的准备

`export_course.py` 已导出当前水平拉开2倍、W1侧移-5m、小窗4Hz/4m/原初相场景。
`results/course_export_f4_a4_20261010/manifest.json` 包含七窗锚点、姿态、正弦平移幅度/频率/相位、起终点及源码参数。
14份原始STL保留SHA-256，同时导出镜像手性修正、米转厘米的Unreal OBJ，未简化三角网格。
导出后独立读取OBJ复核：14份三角形数量不变，顶点最大误差5.000004e-9厘米。
Unreal导入、碰撞烘焙及地面/PlayerStart高度仍待验证；不把OBJ导出称为可运行赛道。
model.py将官方GenericQuad推力/力矩/转动惯量及ENU↔NED坐标变换独立整理，2项测试通过。
这还不是完成的AirSim跟踪MPC或参数辨识，不使用x500控制模型冒充适配。

官方Unreal 4.27 Linux工具链需要Epic/GitHub账号访问。匿名git ls-remote被要求登录，
本机没有编辑器或工程；已向用户请求在本机登录关联账号，不索取密码/令牌。
根分区仅约45GiB可用，完整源码编译还需进一步核对磁盘预算，不擅自删除数据或挂载其他分区。
没有下载来源不明的引擎镜像绕开官方访问要求。AirSim环境本身仍可启动。

导出入口（输出目录必须不存在，防止覆盖）：

`python -m convex_timevarying_window.single_gate_stop_racing.airsim_port.export_course --outdir <新目录>`

`python -m pytest -q convex_timevarying_window/single_gate_stop_racing/airsim_port/test_model.py`

## 已执行

- 从官方 `v1.8.1` 发布下载 Linux Blocks（142533463 bytes），解压后成功启动。
- 使用本目录显式 settings.json，不覆盖用户 Documents/AirSim 全局设置。
- RPC 41451：ping 成功、车辆 RaceDrone、48 个打包资产；Cube、Sphere、Cylinder 等可用，没有原赛道七个窗框 STL。
- 使用原生 SimpleFlight GenericQuad，不是 x500；只进行原生起飞、约5秒状态采样、降落测试，没有飞行位置瞬移。
- 冒烟测试起飞调用返回 false，实际 z 从地面约+0.669到约-0.869（NED米），末次约-0.865；降落返回 true，终态 landed_state=1。悬停采样未报告碰撞。**不将此记为完整起飞验收通过，更不是赛道成功。**

结果在 `results/native_smoke_20261010/`，二进制在 runtime/，均忽略 Git。

## 仍缺少的条件

当前七窗实体边框来自仓库 `convex_dynamic_seven_window_gazebo/meshes/*_sleeve.stl`。
AirSim 的 simSpawnObject 只从已打包 Asset Registry 查找网格，不是运行时 STL 导入器。
严格保持原窗框几何，需要 Unreal 工程将网格、可移动碰撞体及按仿真时钟驱动的窗口运动一起打包；本机未找到 Unreal Editor 或 .uproject。
不能把由 Cube/Cylinder 拼出的近似窗框冒充完全相同的碰撞场景。
高速4Hz/4m场景也不能用低频外部RPC瞬移窗框而不量化时间误差，就宣称与Gazebo同场景。

## 参数和控制适配待办

2026-10-10续测更正：已接入`model.py`原生GenericQuad刚体/二次阻力/5ms PWM滤波和`mpc.py`四电机PWM跟踪，不再沿用PX4控制接口。16项显式单测通过，低速爬升最大误差约7.2mm，尚不能外推为高速跟踪完成。

实际碰撞网格外包盒半尺寸约(.492,.492,.148)m、偏置(0,0,-.033)m，不是下面的核心盒尺寸。修正框杆中心线向外偏移.065m被遗漏导致的重复收缩，实体开口未扩大。当前优化约束使用姿态相关机体OBB与框杆保守OBB分离，局部.5ms复查不是连续认证。开发裕量.04m不构成已证明的闭环误差界。

ClockSpeed=.1下100次近窗步进观测约.3ms，窗口仍为仿真时钟4Hz/4m；实际时间戳计时，不宣称精确步长或实时执行。独立读取simGetCollisionInfo。原生`native_adapt_race_v3_f4_a4_20261010`爬升后因第一窗无接纳规划中止，尚无七窗返航和有效完整竞速时间。`native_adaptation_probe_v10`至`v24`是开发诊断，包括3秒离线求解，不能当作90ms在线实飞成功。尚未证明任务物理不可行。

官方 v1.8.1 SimpleFlightQuadXParams 调用 setupFrameGenericQuad：质量1kg、臂长0.2275m、核心盒0.180×0.110×0.040m、单电机组件0.055kg、旋翼z偏置0.025m。
RotorParams 使用C_T=0.109919、C_P=0.040164、直径0.2286m、max_rpm=6396.667、滤波时间常数0.005s。
这些是官方源码中的名义参数，不等于从发布二进制独立辨识的结果；核心盒尺寸也不是完整碰撞外包络。
现有规划硬动力学、跟踪MPC仍依赖x500/PX4，不可只换接口就声称已对齐AirSim。
坐标需显式进行ENU/FLU与AirSim NED/API控制约定转换；先验证轴向、悬停推力、角速度和电机响应，再跑相同4Hz/4m七窗返航。
计时及碰撞仍要求完整七窗实际正向穿越并返航，保留失败，不用参考轨迹回放代替闭环飞行。

## 入口

### 2026-10-10 同比例放大窗口试验

新增`flight --box-body --aperture-scale 1.886280447`，约为新旧外包球半径比.728/.38594472；仅统一放大开口，不放大框杆截面、不改变布局和4Hz/4m运动。默认仍为小窗，17项测试通过。

原生闭环结果`results/native_scaled1886_f4_a4_v1_20261010/`：正常爬升，竞速时钟6.93373952s后人工中止；35次初始规划请求、零接纳轨迹、零窗穿越、未记录碰撞，源码快照核查无变化。未完成七窗返航，无有效完整竞速时间。此处无接触只是悬停记录，不能称为无碰撞完赛；不能据求解失败证明放大场景不可行。另存长预算离线诊断，不与在线90ms成绩混用。

`results/native_scaled1886_offline_probe_v1_20261010/probe.json`第一请求找到节点收敛、局部.5ms数值复查通过的候选；端到端943.646ms（其中复查19.943ms），名义单窗到虚拟停车点9.392s。不是七窗竞速时间，也没有执行为原生飞行；未满足90ms在线预算。说明当前主要已知瓶颈是求解速度，不能宣称已完成新机高速跟踪验证。

`runtime/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh -settings=<本目录settings.json绝对路径> -windowed -ResX=1600 -ResY=900`

`python -m convex_timevarying_window.single_gate_stop_racing.airsim_port.smoke`

smoke 固定结果目录防覆盖，已运行后不得直接重复此入口覆盖旧结果。

官方依据：

- https://github.com/microsoft/AirSim/releases/tag/v1.8.1
- https://microsoft.github.io/AirSim/settings/
- https://github.com/microsoft/AirSim/blob/v1.8.1/Unreal/Plugins/AirSim/Source/WorldSimApi.cpp
- https://github.com/microsoft/AirSim/blob/v1.8.1/AirLib/include/vehicles/multirotor/MultiRotorParams.hpp
- https://github.com/microsoft/AirSim/blob/v1.8.1/AirLib/include/vehicles/multirotor/RotorParams.hpp
