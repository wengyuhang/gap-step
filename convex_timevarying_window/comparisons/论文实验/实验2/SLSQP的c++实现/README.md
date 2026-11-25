# SLSQP的c++实现

从同级`IPOPT的c++实现（原生minco版）`复制形成的独立分支。当前入口为`src/sip_planner.cpp`，有限问题使用原生编译的Kraft SLSQP内核，由C++直接调用。CasADi构造目标、约束、目标梯度与完整约束雅可比。

## 本轮改变

- 动力学软惩罚与1584个动力学辅助变量移除，目标只有完整七窗及返航时间。
- 每段采用17个归一化时刻`0,1/16,...,1`，每节点使用原两项角速度残差与四电机推力残差，均要求不大于0。八段共816条动力学标量硬约束。
- 仍用原95%x500参数、原生PVAJ MincoPV3、TOGT正段时间和内缩选点映射。自由点96维，严格中心71维。
- 窗口连续数值复查、安全补点、段时间0.3至15秒、原工作空间、节点前向速度0.1 m/s及端点PVAJ条件沿用父版本。
- 双方各8个原始独立初值，位置在窗口中心，时间与MINCO生成的初始VAJ沿用冻结数组。每个方法同时8路，两种方法顺序运行，完成所有候选后按合格候选最小时间择优。
- 安全补点后以上轮变量热启动。原生SLSQP接口在新有限问题中重新初始化内部QP状态。有限问题最多5000次，最多24轮补点，ftol为1e-8。最终要求求解成功、有限硬约束满足容差和原窗口复查通过。

首次有限问题的总标量约束数为自由点1681、中心1677，含上述816条动力学、原几何/工作空间/时长/前向条件，以及自由点映射规范等式。当前验收覆盖指定动力学采样节点，窗口使用原浮点包络复查和独立STL/顺序/端点检查，地面单列。

## 代码

- `include/hard_problem.hpp`装配有限硬约束问题，所有导数通过CasADi。
- `include/slsqp.hpp`提供C++反向通信接口及求解状态。
- `src/sip_planner.cpp`执行单初值优化、安全复查与补点。
- `src/matrix.cpp`调度8个独立求解进程并择优。
- `src/experiment.cpp`运行十个条件、逐方法记录实际并行墙钟和结果。
- `vendor/minco_pv/`保持原生MINCO源码，`include/native_minco_pvaj.hpp`保持其稀疏导数接入。
- `vendor/slsqp/`保存[SciPy1.14.1官方Kraft SLSQP内核](https://github.com/scipy/scipy/blob/v1.14.1/scipy/optimize/slsqp/slsqp_optmz.f)及原许可。求解库通过gfortran编译，规划、导数计算、调度和报告均由C++执行。
- `assets/cases.tar.gz`与父版本逐字节一致，包含十档场景、双方八初值与动力学参数。
- `provenance/ipopt_parent_*`保存复制时的父版本来源与记录。

## 构建和验证

```sh
package='/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/SLSQP的c++实现'
build='/tmp/slsqp_native_minco_build_20261112'
cmake -S "$package" -B "$build" \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build "$build" -j4
ctest --test-dir "$build" --output-on-failure
```

测试包括SLSQP已知最优解，以及双方目标梯度/全约束雅可比的有限差分检查。

## 单初值

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
"$build/sip_planner" "$build/cases/inputs/a25_f6_ours" \
  "$package/outputs/single" 0 --slsqp-native \
  "$build/cases/assets/a25_f6/scene.json"
```

## 完整十条件

```sh
"$build/experiment" "$build" "$build/cases" "$package/outputs/hard17_ten_new"
```

默认先运行`a25_f6`、`a50_f6`、`a75_f6`，然后运行静态和其余六个条件。每个条件依次执行自由点8路和中心8路。`SUMMARY.json`、`REPORT.md`和逐候选原始结果持续保存。计算耗时为每种方法从启动到全部候选与独立几何检查结束的实际墙钟，冻结初值的历史生成成本单列。

结果为全局规划及名义几何评估。正式输入和代码/二进制校验在具体运行目录中冻结。
