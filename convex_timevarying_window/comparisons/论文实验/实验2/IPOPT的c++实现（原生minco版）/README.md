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

安全采用有限节点求解、原浮点窗口复查和补入危险位置的交换流程。每轮使用上一轮变量与乘子热启动，单轮最多5000次，最多24轮。触顶后允许几何有限节点满足条件的候选进入复查，节点几何最小值低于-1e-5时退出。最终接纳要求成功求解状态及完整窗口复查通过。原恢复版的每段0.3至15秒、工作空间与节点前向速度0.1 m/s条件在冻结输入和实现中保留。

有限安全半径为0.62594472米，复查半径为0.62094472米。窗口与地面结果分别记录。最近八初值记录中的初值0存在地面相交，初值3的地面记录通过。名义规划结果和Gazebo执行记录采用各自原始口径。

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

## 单个独立初值

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
"$build/sip_planner" "$build/cases/inputs/a75_f6_ours" \
  "$package/outputs/a75_f6_seed0" 0 --restored-pvaj \
  "$build/cases/assets/a75_f6/scene.json" 1000
```

将输入目录改为`a75_f6_center`即可运行严格中心。初值编号为0至7。

## 八初值并行

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
"$build/matrix" "$build" "$build/cases/inputs" "$build/cases/assets" \
  "$package/outputs/a75_f6_free8" 8 a75_f6 --restored-pvaj-free-eight
```

每个进程使用一个数值库线程。`WINDOW_MATRIX_CPUS`可指定当前机器可用核心，例如`0,2,4,6,8,10,12,14`。该运行选项调度八个独立自由点初值。双方法八初值配对使用`--restored-pvaj`。

## 十条件配对入口

```sh
"$build/restored_experiment" "$build" "$build/cases/inputs" \
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
