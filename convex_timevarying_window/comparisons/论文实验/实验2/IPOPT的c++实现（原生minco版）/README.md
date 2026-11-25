# IPOPT的c++实现（原生minco版）

本目录集中保存2026-11-12的原生PVAJ MINCO全局规划实现、构建依赖、配置、十条件输入和最近八独立初值的原始记录。源码整理来源为Git提交`f3ca9736a8a9a2508b8b63302c70876bca2ba653`。`SOURCE_MANIFEST.json`保存原文件路径与SHA-256。

## 目录

| 路径 | 内容 |
|---|---|
| `src/sip_planner.cpp` | CasADi/IPOPT有限问题、变量与乘子热启动、窗口复查和安全补点 |
| `include/native_minco_pvaj.hpp` | PVAJ变量、正时间和窗内选点映射、原生MINCO回调及稀疏导数 |
| `include/restored_kernels.hpp` | C++动力学软惩罚和边框几何表达式 |
| `include/numpy_passage.hpp`、`include/geometry.hpp` | 原浮点Bernstein窗口复查 |
| `include/mesh.hpp`、`src/mesh_check.cpp` | 独立STL窗框、顺序、端点及地面记录 |
| `src/restored_inputs.cpp` | 查询运动中心，以位置式MINCO重建初始VAJ |
| `src/matrix.cpp` | 独立初值并行、逐候选记录、择优与实际墙钟 |
| `src/restored_experiment.cpp` | 十条件自由点/中心配对流程 |
| `src/restored_release_trial.cpp` | 指定中心轨迹开放选点的单次试验 |
| `vendor/minco_pv/` | 原生MincoPV源文件，PVAJ模式采用`derivative_order=3` |
| `vendor/togt/` | MINCO需要的TOGT头文件和原许可 |
| `config/x500_togt.json` | x500参数与95%动力学界 |
| `assets/cases.tar.gz` | 十条件、双方八初值、对应动力学数据与带实体网格的场景 |
| `assets/parity_fixtures.tar.gz` | 数值核与梯度回归使用的冻结参考函数 |
| `records/a75_f6_free8_20261112/` | 最近八初值试验的协议、表格、汇总与原始记录压缩包 |

## 当前算法设置

本批调用`--restored-pvaj`。全局规划七窗与完整返航，每窗开放独立实际PVAJ，自由点采用TOGT内缩窗口映射，严格中心输入没有自由选点变量。各段时间采用TOGT正映射。

目标为飞行时间与x500动力学软惩罚之和，权重分别为1和1000。动力学用每段33节点积分、1584个非负辅助量及原0.01平滑惩罚，使用95%旋翼推力与角速度界。

安全采用有限节点求解、原浮点窗口复查和逐窗最坏witness交换。每轮保留最佳成功有限incumbent；只有上一轮实际状态和乘子来源均为Solve_Succeeded才复用乘子；上轮失败后回退incumbent仅使用变量，乘子清零。有限几何违反允许连续两次回退重建，第三次仍违反终止。单轮最多5000次，最多24轮。最终接纳要求严格Solve_Succeeded及完整窗口复查通过。原恢复版的每段0.3至15秒、工作空间与节点前向速度0.1 m/s条件在冻结输入和实现中保留。

当前有限安全半径为0.63094472米，复查半径为0.62094472米；历史记录保留原参数。窗口与地面结果分别记录。最近八初值记录中的初值0存在地面相交，初值3的地面记录通过。名义规划结果和Gazebo执行记录采用各自原始口径。

## 依赖与构建

使用Linux、C++17编译器、CMake 3.18及以上、CasADi C++ SDK及其IPOPT插件、Eigen和原ILP64 OpenBLAS。当前机器的SDK位于conda环境`wyh`，以下路径与本次验证一致。

```sh
package='/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT的c++实现（原生minco版）'
build='/tmp/ipopt_native_minco_package_build_20261112'
cmake -S "$package" -B "$build" \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build "$build" -j4
ctest --test-dir "$build" --output-on-failure
```

CMake将压缩输入解到构建目录的`cases/inputs`和`cases/assets`，把测试参考函数解到`parity_fixtures`。源目录之外的项目源码依赖已集中到本目录的`vendor`。数值核与梯度测试分别覆盖自由点和严格中心。

## 当前半径下的运行输入

压缩包保存历史半径数据。构建后先重新生成运行输入，沿用归档段时间，按当前内缩重建中心选点参数和MINCO PVAJ：

```sh
"$build/restored_inputs" "$build/cases/inputs" "$build/cases/assets" \
  "$build/cases/inputs/a25_f6_ours/exchange_dynamics.json" "$build/active_inputs" 3
```

## 单个独立初值

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
"$build/sip_planner" "$build/active_inputs/a75_f6_ours" \
  "$package/outputs/a75_f6_seed0" 0 --restored-pvaj \
  "$build/cases/assets/a75_f6/scene.json" 1000
```

将输入目录改为`a75_f6_center`即可运行严格中心。初值编号为0至7。

## 八初值并行

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
"$build/matrix" "$build" "$build/active_inputs" "$build/cases/assets" \
  "$package/outputs/a75_f6_free8" 8 a75_f6 --restored-pvaj-free-eight
```

每个进程使用一个数值库线程。`WINDOW_MATRIX_CPUS`可指定当前机器可用核心，例如`0,2,4,6,8,10,12,14`。该运行选项调度八个独立自由点初值。双方法八初值配对使用`--restored-pvaj`。

## 十条件配对入口

```sh
"$build/restored_experiment" "$build" "$build/active_inputs" \
  "$build/cases/assets" "$package/outputs/ten_conditions" 16 3
```

条件为`static`、`mixed_base`、`a25_f3`、`a25_f6`、`a50_f1`、`a50_f3`、`a50_f6`、`a75_f1`、`a75_f3`、`a75_f6`。此入口逐条件运行双方八初值，再从本条件最佳中心开放选点求解一次。各轮原始状态、补点检查和并行墙钟均写入输出目录。

## 已有记录

`records/a75_f6_free8_20261112/REPORT.md`与`SUMMARY.json`直接查看最近一批结果。`raw_records.tar.gz`保留八个候选的原始result、progress、日志、独立几何检查和选中轨迹。解压后保留原有`run/`结构。

```sh
mkdir -p "$build/archived_records"
tar -xzf "$package/records/a75_f6_free8_20261112/raw_records.tar.gz" \
  -C "$build/archived_records"
```

`ARCHIVES_SHA256.txt`验证压缩包，`SOURCE_MANIFEST.json`对应原文件内容，`PACKAGE_SHA256.txt`对应整个整合目录。记录中的原始绝对路径保存实验来源。



## 2026-11-14 SIP外层修复

本次代码修改叠加于原整理版本；SOURCE_MANIFEST.json继续保存原始整理来源，当前补丁见provenance/sip_outer_fix_20261114/PATCH_MANIFEST.json。历史实验记录和压缩输入不变。

- 每轮只将Solve_Succeeded且几何节点最小值不低于−1e-5的候选纳入最佳有限incumbent，保存求解变量、原物理变量、上下界乘子、按约束键保存的乘子及真实目标。新候选与下一轮初值分开：先复查新候选，通过就直接返回；失败或目标更差时回退成功incumbent。
- 失败和Solved_To_Acceptable_Level返回的乘子不进入热启动。没有成功来源时lam_x0/lam_g0显式为零，只保留primal；显式回退仅在上一轮实际状态也为Solve_Succeeded时使用成功incumbent的乘子，否则清零；previous_solve_status与dual_source_status分别记录两个状态。新增约束键乘子为零。
- 有incumbent时，有限几何违反最多连续回退两次并重新build；第三次仍违反才以finite_node_geometry_violation终止。节点几何恢复后清零连续计数。原24轮总预算保留。
- 每轮每个窗口最多新增一个可新增的最严重几何witness，按复查器返回的最小距离排名，跨所有轨迹段选择；重复点先跳过。一个witness可能产生多条边约束。动力学补点保持原规则。
- config/x500_togt.json的finite_safety_radius_m改为0.63094472m，复查半径保持0.62094472m。原生入口从配置重建窗内映射、knot与frame表达式；restored_inputs及主目录离线导出同步。旧序列化图若仍固化旧半径会明确拒绝，需要重新导出，避免新旧半径混用。

结果中的finite_incumbent只表示有限节点可行快照，未经完整验收不能当可飞轨迹。found仅在严格Solve_Succeeded且原完整几何/动力学复查通过时为true。

测试：sip_outer_unit覆盖incumbent保持、两类非成功乘子清零、约束键重映射、两次几何回退、每窗最坏点及较慢但已验证候选的立即接纳。sip_result_unit可对真实result和SIP_DUMP_NLP输入逐项检查。新增结果单列在主cpp_runtime/results/sip_outer_fix_v1_20261114，无Gazebo实飞。

### 半径变更后的输入准备

压缩包保留历史输入。修改finite_safety_radius_m后，先用当前C++ restored_inputs重建初值，再运行matrix/restored_experiment；多边形中心映射参数随内缩半径变化，不能只更新轨迹图却继续使用旧参数。八组归档段时间和初始MINCO PVAJ保持不变。示例：

```sh
"$build/restored_inputs" "$build/cases/inputs" "$build/cases/assets" \
  "$build/cases/inputs/a25_f6_ours/exchange_dynamics.json" NEW_INPUTS 3
"$build/restored_experiment" "$build" NEW_INPUTS "$build/cases/assets" \
  NEW_RESULTS 16 3 a25_f6 a50_f6 a75_f6
```

2026-11-14修正后真实中心嵌入预检系数相对误差2.32e-13、目标误差3.55e-15，原始时间与VAJ逐值相等。见cpp_runtime/results/sip_outer_rerun_3pairs_v2_20261114。

### 2026-11-14 上一轮状态门控纠正

三组重跑发现旧分支把成功incumbent的来源状态当成上一轮状态，导致上一轮失败后仍用非零乘子；该解释已纠正。当前只有“上一轮实际Solve_Succeeded且乘子来源也Solve_Succeeded”才复用乘子。失败后回退incumbent只取其变量，lam_g0/lam_x0均为零；成功但目标变差时可取incumbent变量及乘子。日志新增previous_solve_status，保留dual_source_status以分别审计。旧v2记录与对应源码哈希快照保留，旧段落中不区分上一轮状态的回退乘子说明由此条取代。


## 2026-11-14 当前主实验入口

当前主实验按[MAIN_EXPERIMENT.md](MAIN_EXPERIMENT.md)使用四个方门、三个圆门、显式缩放xy和有限问题先验收再补点。`main_course_inputs`生成新场景及双方八初值，`matrix ... LEVEL --main-xy-eight`运行双方独立初值；本文主方法不使用TOGT选点映射或gauge。新数据独立保存，旧结果仍按原协议解释。
