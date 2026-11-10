# 七窗轨迹规划 C++实现

本目录负责全局轨迹求解、几何复查、补点、候选调度与规划结果汇总。规划结果交给实验2已有的MPC/PX4执行链。

## 规划入口

- sip_planner实现本文自由点与严格中心的CasADi/IPOPT有限NLP、连续复查和原始—对偶热启动。
- togt_planner实现动态TOGT自由点与中心，使用原MINCO和SciPy 1.15.3 C L-BFGS-B。
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

results/original_exchange_repair_v2_20261110保存三条冷初值回归，共24次有限NLP逐轮变量和最终曲线精确重现旧版。本文自由点0.75/3×为16.307210415秒，0.75/6×为19.172615343秒，均含七窗后完整返航。

../results/native_cpp_10x5_v1_20261110保留先前50格规划及部分执行记录，修复回归单独保存。使用native_report RESULT_DIRECTORY --planning-only整理规划。

## 第三方依赖

include/json.hpp来自nlohmann/json v3.11.3，保留MIT许可。vendor/scipy包含SciPy v1.15.3原C L-BFGS-B及BSD许可，upstream_lbfgsb.h保留原Python绑定声明作为来源。TOGT MINCO和Wang LBFGS-Lite沿用仓库既有vendor版本。
