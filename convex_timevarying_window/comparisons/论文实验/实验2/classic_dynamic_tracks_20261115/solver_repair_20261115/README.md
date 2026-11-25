# AirSim 21窗高档求解修复

> 2026-11-16用户更正要求固定目标运动直接求解。本文幅度延续结果属于此前开发记录，不纳入当前固定赛道实验。当前统一启动协议及新结果见同级../fixed_target_20261116。

最终运行状态和轨迹结果见 `REPORT.md`；原来的八冷初值对比表保持不变。本目录早期数值诊断均保留，不能把那些变体视为最终算法。

## 当前采用的代码

- `sip_continuation_cached.cpp`：原生完整PVAJ MINCO + 原IPOPT有限问题 + 原安全复查/补点。相对原规划器只加入成功阶段的完整原始变量/乘子热启动、跨阶段安全点集保留和输出记录。目标、动力学与安全标准不变。
- `homotopy_prepare_cached.cpp`：由本题上一成功阶段生成下一阶段输入，将运动幅度设为目标幅度的λ倍；保留已有安全点、原始变量和匹配的成功乘子。已补的约束在新运动下重新计算，不继承旧安全结论。热启动边界内推设为1e-9，防止将几乎为零的软辅助变量抬高。
- `homotopy_adaptive.cpp`：常规幅度增量0.1，失败减半、从上一成功解继续；成功后增量恢复到不超过0.1。每级必须求解并完整复查，不传递失败候选。初始化预算64次阶段尝试，最小分辨率0.001，不是物理时长或工作空间限制。
- `homotopy_static_inputs.cpp`：从当前目标生成零幅度初值输入，不读取其他赛道。
- `homotopy_run.cpp`：保留的固定十级开发对照调度器，正式修复入口采用自适应版本。
- `lift.cpp`：将本题静态MINCO时间初值恢复为原生PVAJ边界状态；只用于初始化，不把静态时间优化器的停止条件当本文有限问题收敛。

轨迹始终使用原生MincoPV、derivative_order=3，全部内部PVAJ独立优化；没有基线加偏移。TOGT独立正时间映射不变。目标仍为时间 + 1000×原动力学软代价，33个节点/段的原梯形积分；安全是原半无限约束的数值复查与补点。规划球0.30米，有限球0.3001米；边框管半径0.085米，对应中心线距离0.385/0.3851米。开口内缩0.3001米、原法向速度至少0.1m/s的条件保留，没有添加工作空间或段时长边界。

## 初始化流程

1. 仅从当前目标赛道生成λ=0的静态副本，不读取其他赛道或其他方法的正式结果。
2. 用本题唯一原始时间初值，在作者TOGT MINCO核上产生静态时间种子；参数位于 `config/time_seed`，只作初始化。自由/中心共享这条中立的中心位置时间种子，然后**分别独立求解各自静态PVAJ问题**。
3. 每个方法沿自己的静态成功解做幅度连续化，完整传递变量、乘子和已补安全点。标准自适应延续，没有对多个最终轨迹挑选最短的一趟。
4. λ=1时必须回到原目标场景，并重新通过有限优化、连续几何数值复查及独立实体碰撞/穿窗顺序/终点检查。

这是新的初始化策略；不能把它记为原八冷初值协议的成功。所有阶段计算成本都应计入，不能只列最后一级的求解时间。当前研究只规划，没有新Gazebo飞行。

## 构建

在本目录运行：

```bash
cmake -S . -B /tmp/classic_repair_build_20261115 -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/classic_repair_build_20261115 --target repair_continuation_cached_sip homotopy_prepare_cached homotopy_adaptive homotopy_static_inputs repair_lift dynamic_togt_planner mesh_check sip_result_unit -j 2
```

运行入口为：

```text
homotopy_adaptive SOLVER PREPARE ORIGINAL_GRAPH TARGET_SCENE OWN_VERIFIED_STATIC_RESULT NEW_OUTPUT 0 100
```

`SOLVER`和`PREPARE`分别是`repair_continuation_cached_sip`和`homotopy_prepare_cached`的绝对路径。`NEW_OUTPUT`必须不存在。严格成功的静态起点、全部中间数据与最终目标数据位于`homotopy_initialization`，原始目标数据位于上级`results/inputs_v1_20261115`。

从零开始重建初始化的C++调用顺序如下，所有OUTPUT须为新目录：

```text
homotopy_static_inputs ORIGINAL_GRAPH TARGET_SCENE STATIC_INPUT
dynamic_togt_planner STATIC_SCENE SEED_OUTPUT center CONFIG_TIME_SEED VEHICLE_JSON --initial-manifest=STATIC_CENTER_MANIFEST --initial-index=0
repair_lift STATIC_INPUT SEED_RESULT STATIC_SCENE LIFTED_INPUT
repair_continuation_cached_sip LIFTED_INPUT STATIC_SIP_OUTPUT 0 --soft-mincopv STATIC_SCENE 1000
homotopy_adaptive SOLVER PREPARE ORIGINAL_GRAPH TARGET_SCENE STATIC_SIP_RESULT NEW_OUTPUT 0 100
```

中立静态中心位置MINCO时间种子只生成一次，自由和中心随后各自完成静态SIP及延续。静态输入生成器已与本次使用输入逐项核对一致，见RUNTIME_INTERFACE_VALIDATION.json。

历史诊断、主动中断、启动错误和未通过有限问题分别记录在`DIAGNOSTIC_SUMMARY.json`、`INTERRUPTED_DIAGNOSTICS*.json`及`results`。其中的时钟提升、辅助变量消元、不同Hessian/SQP等数值试验均未被本修复分支采用。

原始未采用诊断源码已迁到results/diagnostic_source_snapshot_20261116，保留原CMake及SHA映射SOURCE_ARCHIVE.json；需要复原诊断构建时，将归档源码和CMake放回原相对位置。采用的运行文件在本目录顶层。
