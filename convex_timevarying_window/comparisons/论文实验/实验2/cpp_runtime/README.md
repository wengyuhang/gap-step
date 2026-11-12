# 七窗轨迹规划 C++实现

本目录负责全局轨迹求解、几何复查、补点、候选调度与规划结果汇总。规划结果交给实验2已有的MPC/PX4执行链。

## 规划入口

- sip_planner实现本文自由点与严格中心的CasADi/IPOPT有限NLP、连续复查和原始—对偶热启动。
- dynamic_togt_planner实现当前动态TOGT自由点与中心，直接复用静态复现作者C++核和L-BFGS，每个方法单初值。
- togt_planner保留旧图导出的历史兼容入口。
- wang_planner实现既有Wang独立复现的N2/M4局部求解及参考拼接。
- matrix运行原多初值候选与择优流程，native_report生成规划表与求解耗时说明。
- frame_check、numpy_frame_check及mesh_check提供原边框数值复查与实体STL复查。

tools/export_runtime.py等工具从原数学定义离线导出共享MX表达式与初值，C++负责运行时组装、优化和复查。SIP输入使用schema 2，包含symbolic.mx和physical_curve.casadi，保持原CasADi map/cumsum/MXDict计算图与NumPy对应的BLAS/LAPACK。

## 构建

```sh
cmake -S convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime \
  -B /tmp/window_cpp_runtime_build \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build /tmp/window_cpp_runtime_build -j2
```

CasADi SDK使用C++ ABI 1。NUMPY_BLAS指向wyh NumPy使用的ILP64 OpenBLAS，保证补点数值流程对齐。

## 与已有飞行代码衔接

已有入口位于../higher_swing/matrix_flight.py。它调用../radius_ablation/execution.py执行完整参考轨迹，跟踪器来自../gazebo_matrix/reference.py，PX4通信沿用togt_style_mpc/aligned_session.py。

输入result.json保留durations、coefficients、variables、config，并包含case、scenario_variant和method等场景标签。全局matrix选中结果已经带这些标签。单次SIP候选可以补齐场景标签后交给同一入口。

修复后的两条自由点与一条中心轨迹已通过原frozen_plan读取器和MPC Reference采样核对，时长/系数逐位相同，PVAJS等参考误差为0。可供旧入口读取的文件保存在results/original_exchange_repair_v2_20261110/execution_ready。该核对仅加载与采样。

此前新增的重复C++MPC、通信与飞行调度源码已从本目录清出，原文件保留在实验2正式记录的runtime_frozen/source_snapshot或execution_scope_cleanup内，清单见execution_scope_cleanup/MANIFEST.json。

## 结果与回归

2026-11-12 新增动力学半无限补点。`tools/export_exchange_dynamics.py GRAPH_DIRECTORY` 导出单点动力学函数，默认采用初值局部峰值及原 TOGT 独立正时间映射。`sip_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY 0 --exchange-hard-reset-togt` 每轮有限问题后逐段搜索峰值，再用普通浮点区间和中心导数界复查整段。新增时刻采用段内归一化时间，每次补点后回到动态 TOGT 中心初值。

`dynamics_check TRAJECTORY_JSON METADATA_JSON OUTPUT_JSON` 独立复查同一六项动力学，metadata 为导出的 exchange_dynamics.json。第二组自由点完成 19 轮，完整参考 16.415054 秒，计算 197.003818 秒，动力学约束 199→457。原始结果与失败诊断在 `../results/dynamics_exchange_mixed_base_v1_20261112`。当前结论为浮点包络数值证据。

results/original_exchange_repair_v2_20261110保存三条冷初值回归，共24次有限NLP逐轮变量和最终曲线精确重现旧版。本文自由点0.75/3×为16.307210415秒，0.75/6×为19.172615343秒，均含七窗后完整返航。

../results/native_cpp_10x5_v1_20261110保留先前50格规划及部分执行记录，修复回归单独保存。使用native_report RESULT_DIRECTORY --planning-only整理规划。

## 第三方依赖

include/json.hpp来自nlohmann/json v3.11.3，保留MIT许可。vendor/scipy包含SciPy v1.15.3原C L-BFGS-B及BSD许可，upstream_lbfgsb.h保留原Python绑定声明作为来源。TOGT MINCO和Wang LBFGS-Lite沿用仓库既有vendor版本。

2026-11-12 最小总推力验证入口为 `sip_planner GRAPH OUTPUT 0 --exchange-hard-min-thrust`，在动力学补点模式加入0.7693722 N总推力硬下界。`minimum_thrust_check TRAJECTORY_JSON OUTPUT_JSON` 使用段内推力平方驻点及端点独立复查，`minimum_thrust_unit` 为对应回归。a25_f6中心版22轮得到含返航17.359316秒参考，C++345.214秒，独立窗框及动力学数值复查通过。自由点首轮5000次未收敛，有限推力节点满足、节点间仍近零。结果和诊断见 `../results/minimum_thrust_a25f6_v1_20261112/REPORT.md`。

## 独立完整 PVAJ MINCO 入口

2026-11-12 新入口直接链接 `静态凸窗口/复现/experiments/free_velocity/minco_pv.cpp`，使用 `togt_pv::MincoPV(head, tail, n, 3)`。每次函数评价将当前世界坐标穿越点、实际 V/A/J、各段时间传给 `fit`，通过 `propagate` 给 CasADi 提供解析 Jacobian。移动窗位置对到达时间的导数由新建的 C++ CasADi 图传递。轨迹和连续复查直接共用本次 MINCO 系数。

```sh
sip_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY 0 --exchange-hard-mincopv SCENE_JSON
native_minco_unit GRAPH_DIRECTORY SCENE_JSON VALIDATION_JSON
```

本入口读取 manifest 中的场景参数与原物理初值，以及仅以单段系数/时间为输入的 frame 和 dynamics 局部函数。原 symbolic.mx、baseline.casadi、trajectory.casadi、physical_curve.casadi 均由其他历史入口使用。新入口的 96 个变量为 8 段独立正时间参数、25 个内缩窗内选点参数和 63 个实际 VAJ 的单位缩放参数。无基线偏移。初始 VAJ 仍采用本组已确定的动态 TOGT 中心参考。

`native_minco_unit` 单独读历史函数做系数/初始几何回归，并以中心差分检查新入口的时间、点与 VAJ 梯度。正式新入口的求解和输出完全在原生 MINCO 路径内。试验记录位于 `../results/native_minco_pvaj_a25f6_v1_20261112`。

原生MINCO回调通过 `has_jac_sparsity/get_jac_sparsity` 声明逐段、逐轴局部依赖，导数回调输出相同稀疏结构。最新结果在 `../results/native_minco_pvaj_a25f6_v3_20261112`。本组约束Jacobian声明非零项12922，5000次68.376秒，规划总69.534秒仍未收敛。v1稠密接入失败和v2部分接口修复中断均保留。

## 当前软动力学入口

2026-11-12按用户要求恢复最近的辅助变量软约束写法，当前使用以下入口。

```sh
sip_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY SEED_INDEX --soft-mincopv SCENE_JSON
```

时间/动力学权重均1，六项动力学采用原95%限值与mu=0.01光滑罚项，每段32区间梯形积分。动力学超限辅助变量1584个，轨迹变量自由96/中心71。安全边框继续使用原硬几何约束和数值补点。新时间映射、MincoPV(order=3)直接完整PVAJ、实际VAJ单位缩放及局部稀疏声明均保留。新软入口使用原始—对偶热启动接续安全补点后的有限问题。自由与中心目标/梯度回归均通过，记录见 `../results/native_soft_restoration_v1_20261112`。本次未运行新的规划实验。

### 动力学软权重显式设置

`--soft-mincopv SCENE_JSON [DYNAMICS_WEIGHT]` 的可选最后一个数值设置动力学权重，省略时为1。时间权重1。当前用户指定的单组1000实验使用以下命令。

```sh
sip_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY SEED_INDEX --soft-mincopv SCENE_JSON 1000
```

权重同时进入NLP目标与重算真实目标，报告的dynamic_soft_integral为未加权积分，dynamic_penalty_contribution记录实际目标贡献。0.25/6同初值6轮均收敛，完整进程67.37秒，合格名义轨迹16.414423604秒含返航，原安全和独立56项STL数值复核通过。记录在 `../results/native_soft_a25f6_weight1000_v1_20261112`。

## 作者C++核的单初值动态TOGT初版记录

```sh
cmake --build /tmp/window_cpp_runtime_build --target dynamic_togt_planner -j2
/tmp/window_cpp_runtime_build/dynamic_togt_planner SCENE_JSON OUTPUT_DIRECTORY free
/tmp/window_cpp_runtime_build/dynamic_togt_planner SCENE_JSON OUTPUT_DIRECTORY center
```

原生算法位于include/dynamic_togt.hpp及src/dynamic_togt_planner.cpp。从静态复现2024快照直接编译MINCO、形状映射、时间映射、动力学积分和原L-BFGS。配置直接读取cpc/init的作者YAML，权重时间1/能量0/其余罚项1，16区间，past3，冷初值distance/speedGuess1。车辆适配在config/x500_togt.json，采用当前95%x500物理界与原内缩。圆形保留2维平面选点，严格中心无空间变量，移动点的时间梯度沿全部前序段累计回传。

matrix两种动态TOGT方法现采用该入口，每方法每条件一次，旧多初值目录应另起结果。历史togt_planner图入口及旧调度脚本保留用于追溯。新算法不依赖Python或CasADi函数导出。静态作者核与动态梯度、旧共享内缩回归通过，结果见../results/upstream_dynamic_togt_native_v1_20261112。静态两版几何通过，0.25/6作者单初值出现115/119秒长候选和名义碰撞，保留原始负结果。未实飞。

## 历史动态TOGT相对梯度设置

2026-11-12排查115/119秒长候选，已修正静态粗设置在动态任务中的使用方式。config/x500_togt.json包含initial_speed_guess_mps=6.5和relative_gradient_tolerance=1e-5。每段时间从自身距离/6.5生成，单次冷启动。L-BFGS past=0，按||g||inf/max(1,||x||inf)<=1e-5停止，作者权重及其他核函数保留。原静态past3配置数值作为original_static_lbfgs_past留档。

正式0.25/6自由/中心20.564818/20.563470秒，计算0.272102/0.079268秒，梯度停止与独立几何复查通过。相同冻结二进制重复变量及系数逐项一致。详见../results/upstream_dynamic_togt_corrected_v1_20261112。原接口命令相同，初版负结果及单因素诊断均保留。

## 历史统一绝对梯度设置

2026-11-12进一步核对自由/中心精度。旧全变量相对梯度受自由点冗余参数增大影响，会把时间梯度容差放大。当前所有方法统一采用config.absolute_gradient_tolerance=1e-4，按max(abs(gradient))结束，原相对梯度和代价停滞均关闭。进度回调主动终止的原L-BFGS状态2保留，gradient_stop_reached与实际梯度一起报告。单初值与目标不变。

0.25/6自由20.553097810秒、中心20.563470435秒，规划3.003150/0.079732秒，均达到统一精度并通过名义几何检查。详细记录在../results/dynamic_togt_absolute_stop_v1_20261112，旧相对梯度报告与更严绝对阈值下的线搜索失败均保留。

## 当前动态TOGT使用作者停止设置与中心解初始化

2026-11-12用户要求恢复作者原优化器配置。当前直接读取cpc/init/cpc_lbfgs.yaml，past=3、delta=1e-5、g_epsilon=0、max_iterations=0，进度回调仅记录迭代数。中心继续以各段距离/6.5生成单初值；自由点使用优化后的中心时间参数和中心位置，MincoSnap生成内部VAJ。

```sh
dynamic_togt_planner SCENE_JSON CENTER_DIRECTORY center
dynamic_togt_planner SCENE_JSON FREE_DIRECTORY free --center-result=CENTER_DIRECTORY/result.json
```

省略--center-result时，free入口先求中心并保存在FREE_DIRECTORY/center_initial/result.json。matrix等待同条件中心候选完成，传入该文件，避免重复生成。自由点stage_compute_s为自由阶段，center_generation_s为中心阶段，total_compute_s包含两者。优化成功按作者非负状态判断，独立几何复查另行记录。

静态中心/自由为20.418764/19.354785秒，0.25/6为20.681774/20.681710秒，四条名义轨迹几何通过。单列结果见../results/dynamic_togt_center_seed_original_lbfgs_v1_20261112/REPORT.md。上述相对与绝对梯度设置均为历史实验。

## 本文软动力学版本接收原生TOGT中心初值

`prepare_togt_seed GRAPH_DIRECTORY TOGT_CENTER_RESULT SCENE_JSON OUTPUT_GRAPH_DIRECTORY`在C++中提取中心轨迹的各段时间和实际VAJ，生成原生MincoPV入口的单初值。通过同一MincoPV(order=3)重建系数并核对中心位置，输出只包含原生入口所需的manifest、动力学局部函数及边框函数。

```sh
prepare_togt_seed SOURCE_GRAPH CENTER_DIRECTORY/result.json SCENE_JSON NEW_GRAPH
sip_planner NEW_GRAPH OUTPUT_DIRECTORY 0 --soft-mincopv SCENE_JSON 1000
```

此生成目录供`--soft-mincopv`及原生MincoPV模式使用。历史图入口读取其原图目录。2026-11-12三组实验在`../results/native_six_frequency_three_v1_20261112`，本文自由/中心均由本组新算的动态TOGT中心初始化，单初值、逐组串行，计算包含初值生成成本。逐轮状态与碰撞复查分开记录。

## 当前本文八初值无时间范围与工作空间约束

2026-11-12用户明确删除每段0.3至15秒边界及工作空间约束。当前原生MincoPV图保留TOGT独立正时间映射，有限问题及时间变量上下界均已移除这两类限制。自由点/中心无补点时的约束行数为1617/1613。时间和动力学权重继续采用1/1000，其他物理与安全参数沿用当前配置。

```sh
matrix BINARY_DIRECTORY GRAPH_DIRECTORY ASSET_DIRECTORY RESULT_DIRECTORY 8 a25_f6 --native-soft-eight
```

新模式只调度本条件本文自由/中心各8个独立初值，共16候选，使用8个worker，关闭中心解释放。图目录命名仍为LEVEL_ours和LEVEL_center。有限优化失败立即结束该初值，安全补点接续上一轮原始变量和乘子，原生模式跳过圆窗参数等价换元。

输入初值直接沿用graphs_v1_20261110存档的8个物理向量，双方对应初值的时间与VAJ逐项相同。空间初值在移动中心，时间及运动相位独立Sobol采样。新批次位于../results/native_no_domain_eightstarts_3pairs_v1_20261112。PAIRED_WALL.json按双方共同启动起点记录各方法全部8候选和复查结束的墙钟，包含8-worker排队；cumulative_worker_s单独记录累计worker时间。

## 直接几何选点坐标

2026-11-12用户要求圆窗采用半径和角度，多边形采用直接二维坐标。入口为`--soft-mincopv-direct SCENE_JSON 1000`，八初值调度为`matrix ... 8 LEVEL --native-direct-eight`。圆窗变量满足0≤rho≤当前内缩半径、0≤theta≤2pi，多边形用当前内缩区域的线性半空间。相应四条参数范数等式移除，自由点轨迹变量96→85，初始有限约束1617→1632行。

原八初值的各段时间与VAJ直接保留，窗内点转换为相同物理位置。中心rho=0时，用原物理目标下降方向初始化角度；该步骤只选择同一中心点的角度表示。三组共24条初始曲线的时间、系数及穿越点逐项一致，方向梯度验证通过。动力学辅助变量、权重、时间正映射、安全补点、0.1m/s节点方向条件和有限优化终止规则沿用上一批。结果位于`../results/native_direct_points_eightstarts_3pairs_v1_20261112`，旧表示仍由`--soft-mincopv`调用。

## 当前初值流程先运行动态TOGT中心八路

2026-11-12按用户最新要求，先将原八个段时间样本交给动态TOGT中心求解。直接入口支持`--initial-manifest=MANIFEST_JSON --initial-index=INDEX`，内部调用作者正时间映射与原L-BFGS配置，VAJ由原MincoSnap消元。

```sh
matrix BIN GRAPHS ASSETS TOGT_STAGE 8 a25_f6 --togt-center-eight
prepare_togt_seed GRAPH TOGT_STAGE/plans/a25_f6_dynamic_togt_center/result.json SCENE PAPER_GRAPH
sip_planner PAPER_GRAPH PAPER_OUTPUT 0 --soft-mincopv-direct SCENE 1000
```

八候选按TOGT原目标选择求解成功的最好结果，几何复查单列记录。该结果给本文自由/中心各一个相同物理初值，两本文方法并行2路，无本文中心释放。计算记录包含八路TOGT墙钟、各自初值转换与本文单初值进程。当前批次为`../results/native_togt8_then_single_paper_3groups_v1_20261112`，被用户叫停的上一八初值批次保留部分结果并标记stopped_by_user。

## PV MINCO对照入口

2026-11-12新增`--soft-mincopv-pv SCENE_JSON 1000`。中间节点只保留P和三维V，调用原`MincoPV(head, tail, n, 1)`消去内部A/J。端点VAJ零。自由点轨迹变量43、中心29，辅助变量1584。圆rho/theta、多边形xy、时间正映射、原安全补点与软动力学设置继续使用当前配置。

PV消元后的系数对所有段时间以及所有同轴节点P/V存在依赖，回调使用对应的全局轴分块Jacobian结构，4224项；PVAJ仍按局部依赖声明。`pv_minco_unit GRAPH SCENE TOGT_SOURCE OUTPUT_JSON`验证初值重建、方向导数及末窗速度对首段系数的非局部影响。

新批为`../results/native_pv_same_togt_initial_3groups_v1_20261112`，与前批使用完全相同的TOGT选中曲线初始化，不增加本文初值或中心释放。


### 动态TOGT动力学1000配置

`config/togt_dynamics1000`提供作者YAML结构，weightVel、weightOmg、weightThr为1000，weightTime为1，L-BFGS停止参数沿用作者设置。向dynamic_togt_planner的AUTHOR_PARAMETER_DIRECTORY参数传入该目录即可用于中心与自由点。2026-11-12的0.5/6同初值来源对照保存在results/native_pv_togt1000_a50_f6_v1_20261112。

## 历史完整软约束流程的原生C++恢复入口

2026-11-12按截图定位到`higher_swing_10x5_full_v1_20261109`。新增`--restored-pv SCENE_JSON 1000`保留现行`MincoPV(derivative_order=1)`、完整全局系数Jacobian及TOGT正时间映射。窗口选点恢复TOGT映射，轨迹变量为自由54、中心29，均21个内部三维V变量。内部A/J由MINCO消元，端点VAJ零。

本入口在C++中构建动力学、框距、工作空间、段时长及辅助变量有限问题，直接生成CasADi表达和导数。运行所需数据为JSON场景、原Sobol段时长样本、物理参数与IPOPT配置。`restored_kernels.hpp`不读取历史Python导出的函数。`restored_inputs`以原时间样本重新查询移动中心并用C++位置式MINCO生成初始速度。

恢复的历史流程包含每段0.3至15秒、17节点/段工作空间`[-100,-120,0.65]`至`[150,120,20]`、0.1m/s节点法向速度、0.62594472米框中心线内缩、0.62094472米最终窗口几何复查、33节点/段软动力学积分及时间/动力学权重1/1000。自由/中心各8个独立初值，随后使用本条件最佳合格中心结果开放全部25个点参数再求一次。各条件独立，最多17候选。

历史补点流程允许有限优化未收敛且节点几何已满足的候选进入复查与后续补点，所有返回状态完整保存。最终接纳要求有限优化收敛、原窗口连续数值复查及独立STL任务复查通过。圆映射热启动使用历史等价反演。原上轮结果及其他入口保留。

```sh
cmake --build /tmp/window_cpp_runtime_build --target sip_planner matrix restored_inputs restored_experiment restored_unit -j2
restored_inputs TIMING_ARCHIVES ASSET_DIRECTORY DYNAMICS_JSON INPUT_DIRECTORY
restored_experiment BINARY_DIRECTORY INPUT_DIRECTORY ASSET_DIRECTORY NEW_RESULT_DIRECTORY 16
```

`restored_unit GRAPH SCENE LEGACY_GRAPH OUTPUT`只在回归测试中读取历史数值函数，对照原动力学/框距定义和完整梯度。正式规划、初始化、并行调度、补点复查、择优和报告生成入口均为C++。本批结果根目录为`../results/restored_soft_cpp_pv_full_v1_20261112`，十组进度及报告位于其中`matrix`。

### 当前使用原生PVAJ恢复入口

用户随后指定使用PVAJ MINCO，先运行0.5/6和0.75/6。`--restored-pvaj SCENE 1000`直接调用`MincoPV(head,tail,n,3)`，完整内部VAJ作为独立实际变量，时间仍用TOGT正映射。自由点96、中心71轨迹变量，1584动力学辅助变量。原PV入口保留。

```sh
restored_inputs TIMING_ARCHIVES ASSET_DIRECTORY DYNAMICS_JSON INPUT_DIRECTORY 3
restored_experiment BINARY_DIRECTORY INPUT_DIRECTORY ASSET_DIRECTORY NEW_MATRIX_DIRECTORY 16 3 a50_f6 a75_f6
restored_audit BATCH_DIRECTORY ASSET_DIRECTORY
```

省略条件列表时逐组运行十条件。当前批为`../results/restored_soft_cpp_pvaj_two_v1_20261112`，RUN.sh串接全部C++运行与最终审计。四项PVAJ核与梯度回归通过。原PV批`restored_soft_cpp_pv_full_v1_20261112`按用户指示停止，状态stopped_by_user，部分结果保留。

两组恢复实验中途按用户要求利用空闲CPU重叠运行。`restored_parallel ORIGINAL_BIN SECOND_BIN INPUTS ASSETS BATCH SECOND_WORKERS`只读取首组协调状态并启动尚未运行的第二组，完成后调用C++最终审计。`WINDOW_MATRIX_CPUS`指定独立CPU池。当前首组原16路，第二组13路，保留首组后续释放任务的CPU0。新调度原始命令为批目录中的RUN_PARALLEL_CONTINUATION.sh。

### 最新正式入口与上限规则

用户指定有限IPOPT达到迭代上限就结束该初值。`sip_planner`记录Maximum_Iterations_Exceeded后立即退出该初值，后续复查与补点跳过。原恢复流程允许上限后继续的描述为历史行为。其他失败状态按原恢复逻辑记录处理，最终仍要求成功/可接受精度和窗口复查。

`restored_two BIN INPUTS ASSETS NEW_BATCH`同时运行a50_f6/a75_f6，两组各8单线程worker，总16路。每组自由/中心各8初值并保留最佳中心释放。正式新批为`../results/restored_soft_cpp_pvaj_stopmax_two_v1_20261112`，RUN.sh为全C++入口。`restored_audit`检查每次上限返回后都没有后续有限问题、安全复查或补点，完成后生成FINAL_REPORT.md和FINAL_SUMMARY.json。

### 当前取消触顶即停，固定中心解开放选点

用户最新取消触顶即停。restored入口恢复节点几何满足时可从未收敛候选继续复查/补点，节点几何最小值低于-1e-5时退出。`termination_reason`明确区分节点违反、没有新见证、交换轮次和合格轨迹。原stopmax批按其冻结二进制和协议保留。

`restored_release_trial BIN INPUTS SCENE CENTER_RESULT NEW_OUTPUT`在C++中执行一次指定中心结果的自由点释放。`center_embedding.json`核对全部初始系数、时间和目标与中心结果一致。当前批`../results/restored_cpp_pvaj_center19742_release_v1_20261112`使用19.742427606896秒中心，仅一次自由点，无中心重算或其他初值。

## 单条件八个独立自由点初值

`matrix BINARY_DIRECTORY GRAPH_DIRECTORY ASSET_DIRECTORY NEW_RESULT_DIRECTORY 8 a75_f6 --restored-pvaj-free-eight`仅运行自由点初值0至7。使用`--restored-pvaj`求解与原安全补点流程，保持取消触顶即停后的设置，每轮热启动上一有限问题的变量和乘子。本入口使用八个独立worker，输出逐候选结果与独立几何检查，以实际并行墙钟记录选优时间。2026-11-12新运行在`../results/restored_cpp_pvaj_free8_a75_f6_v1_20261112`。

## 当前PVAJ实验的完整源码索引

当前八初值试验调用`--restored-pvaj`，使用原生`MincoPV(derivative_order=3)`、TOGT正时间映射、时间/动力学软权重1/1000与原安全交换流程。

| 文件 | 作用 |
|---|---|
| `src/sip_planner.cpp` | 有限IPOPT问题、变量/乘子热启动、几何复查与补点、退出与接纳 |
| `include/native_minco_pvaj.hpp` | 独立PVAJ变量、正时间和窗内选点映射、MINCO回调及稀疏导数接口 |
| `include/restored_kernels.hpp` | C++ CasADi动力学软惩罚与边框距离表达式 |
| `include/geometry.hpp`、`include/numpy_passage.hpp` | 原浮点Bernstein包络与整段窗口复查 |
| `include/trajectory.hpp`、`include/mesh.hpp` | 轨迹读取、独立STL窗框/顺序/端点检查 |
| `src/restored_inputs.cpp` | 从归档的八个时长样本构造运动中心与MINCO初始VAJ，PVAJ模式传入阶数3 |
| `src/matrix.cpp` | 八个独立进程的调度、逐候选记录、择优和实际并行墙钟 |
| `src/restored_release_trial.cpp` | 指定中心轨迹开放自由选点的单次试验 |
| `src/restored_audit.cpp`、`tests/restored_unit.cpp` | 实验结果核对与数值核/梯度回归 |
| `config/x500_togt.json` | x500动力学参数及95%余量 |

底层MINCO代码在仓库根目录下的`静态凸窗口/复现/experiments/free_velocity/minco_pv.cpp`与同名`.hpp`。其使用的TOGT头文件，以及动态TOGT基线所需源码与参数，在`静态凸窗口/复现/TOGT-Planner-ICRA2024/`。CMake中Wang目标所需的原生源文件在`复现/wang2023_verified/`。上述构建实际使用的源文件和许可已显式纳入Git，目录整体的历史产物忽略规则继续适用。

在安装有CasADi/IPOPT、Eigen及原wyh数值库的环境中，可构建当前核心入口。

```sh
cmake -S convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime \
  -B /tmp/window_cpp_runtime_build \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build /tmp/window_cpp_runtime_build --target sip_planner matrix restored_inputs restored_unit -j2
```

当前入口在C++中构造CasADi符号图并生成导数。八组归档时长与实验场景作为输入数据读取，正式试验的输入、协议、源码/二进制SHA和逐候选结果位于`../results/restored_cpp_pvaj_free8_a75_f6_v1_20261112/`。该结果目录按仓库约定作为本地实验记录保存。

## 原生MINCO整合目录

2026-11-12按用户要求，当前PVAJ原生IPOPT运行链及必要依赖、输入与最近原始记录集中整理到同级目录`IPOPT的c++实现（原生minco版）/`。该目录自带CMake、来源与内容校验、十条件压缩输入和完整运行说明，可从自身源码独立构建。入口说明见该目录README.md。
