# 非凸时变窗口穿越研究

`nonconvex_timevarying_window/` 是整个研究任务的总目录。本项目在 TOGT 论文方法的基础上，研究四旋翼无人机如何按指定顺序穿越非凸且随时间变化的窗口。

详细的研究背景、问题范围和数学定义见 [PROBLEM_DEFINITION.md](PROBLEM_DEFINITION.md)。

## 已有方法

| 方法 | 状态 | 主要思路 |
|---|---|---|
| [AtlasDynaTOGT](atlas_dynatogt/README.md) | 已实现 | 将非凸窗口三角剖分为 chart atlas，联合优化穿越时间和穿越点 |
| [SC-DynaTOGT](sc_dynatogt/README.md) | 已实现，default 实验完成 | Chang 仅做边界均匀采样，SC 圆盘映射负责内部点，接入原 TOGT/MINCO 联合优化；支持真实物理门框、分类结果目录、中文结果主页和可选 EGL/OpenGL 离线渲染，记录见 [TEST_RESULTS.md](sc_dynatogt/TEST_RESULTS.md) |
| [MSR-DynaTOGT](msr_dynatogt/README.md) | 已实现，smoke/formal 完成 | 在 SC-DynaTOGT 外增加可复现多初值候选池、高密度采样动力学检查、uniform/local 时间修复、二分缩放和联合再优化；可行性优先于总时间，记录见 [TEST_RESULTS.md](msr_dynatogt/TEST_RESULTS.md) |
| [WBSC-DynaTOGT](wb_sc_dynatogt/README.md) | 已实现 | 保留原 MINCO 和动力学软罚，联合优化 `[K,D,Y]`；每次迭代由当前轨迹恢复 roll/pitch/yaw 并施加姿态长方体穿窗约束，记录见 [TEST_RESULTS.md](wb_sc_dynatogt/TEST_RESULTS.md) |
| [CWB-SC-DynaTOGT](cwb_sc_dynatogt/README.md) | V1 已实现 | 保持 `[K,D]` 和 constant yaw；在包含规划 `t_i` 的完整相交区间内验证姿态长方体的全部平面截面边，并用活动 witness 温启动修复；当前仅称数值验证，不称严格证书 |
| [Exact-Area Whole-Body SC-DynaTOGT](exact_area_sc_dynatogt/README.md) | 实验 B 已完成 | 使用固定世界 0.315 m 名义裕度，精确计算姿态长方体截面与动态非凸窗口的多连通交集面积；当前展示名义中心穿越安全、但完整动态相交区间提前碰撞的反例，A/C/D/E/F 仅保留协议位置 |

## 目录结构

```text
nonconvex_timevarying_window/
  README.md                 总任务和方法索引
  PROBLEM_DEFINITION.md     与具体算法无关的问题定义
  atlas_dynatogt/           AtlasDynaTOGT 方法的完整实现
  sc_dynatogt/              Schwarz–Christoffel DynaTOGT 方法的完整实现
  msr_dynatogt/             Multi-Start and Repair DynaTOGT 的完整实现
  wb_sc_dynatogt/           姿态感知全机体 SC-DynaTOGT 的完整实现
  cwb_sc_dynatogt/          constant-yaw 连续截面整机安全方法（V1）
  exact_area_sc_dynatogt/   精确交集面积整机安全方法（当前实验 B）
  <algorithm_name>/        后续新增的其他方法
```

每种方法都在以算法名称命名的独立子目录中维护，该目录内放置算法说明、源码、测试、图解和实验结果。

SC-DynaTOGT 的可视化分为三层：预处理诊断图用于检查真实边界、内缩安全区和 SC 网格；Matplotlib 场景图只显示真实门框和四旋翼；可选 OpenGL 渲染器输出实体环境、追踪相机和 MP4。OpenGL 输出是已求解轨迹的离线渲染，不是 AirSim 动力学或传感器仿真。

SC-DynaTOGT 的本地结果按 `experiments/`、`demos/`、`diagnostics/` 和 `work/` 分类；`results/index.html` 是统一浏览入口。历史结果通过带 SHA-256 的迁移清单无损保留，不与新时间戳运行混放。

MSR-DynaTOGT 只在自己的 `results/<timestamp>_<suite>/` 下写入结果，不覆盖历史运行。它与 SC 共用 SC 映射、动态窗口、degree-7 MINCO 和四旋翼模型，但把优化后诊断提升为候选排序和时间修复环节。输出中的“可行”仅指高密度采样可行，不是连续时间严格证明；`optimizer_success` 与动力学可行性分别记录。

2026-07-22 完成的 formal 套件覆盖 5 个场景、每场景 155 个种子（共 775 个任务、9,300 行比较记录）。A2/A3 的高密度采样动力学可行率均为 100%，A0 为 0%，A1 为 0.1%；A3 平均墙钟代价为 A0 的 10.05 倍。逐图解释与完整数值保存在 formal 结果目录和 MSR 的 `TEST_RESULTS.md`。
