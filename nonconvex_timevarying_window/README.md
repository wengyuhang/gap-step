# 非凸时变窗口穿越研究

`nonconvex_timevarying_window/` 是整个研究任务的总目录。本项目在 TOGT 论文方法的基础上，研究四旋翼无人机如何按指定顺序穿越非凸且随时间变化的窗口。

详细的研究背景、问题范围和数学定义见 [PROBLEM_DEFINITION.md](PROBLEM_DEFINITION.md)。

## 已有方法

| 方法 | 状态 | 主要思路 |
|---|---|---|
| [AtlasDynaTOGT](atlas_dynatogt/README.md) | 已实现 | 将非凸窗口三角剖分为 chart atlas，联合优化穿越时间和穿越点 |
| [SC-DynaTOGT](sc_dynatogt/README.md) | 已实现，default 实验完成 | Chang 仅做边界均匀采样，SC 圆盘映射负责内部点，接入原 TOGT/MINCO 联合优化；支持真实物理门框、分类结果目录、中文结果主页和可选 EGL/OpenGL 离线渲染，记录见 [TEST_RESULTS.md](sc_dynatogt/TEST_RESULTS.md) |

## 目录结构

```text
nonconvex_timevarying_window/
  README.md                 总任务和方法索引
  PROBLEM_DEFINITION.md     与具体算法无关的问题定义
  atlas_dynatogt/           AtlasDynaTOGT 方法的完整实现
  sc_dynatogt/              Schwarz–Christoffel DynaTOGT 方法的完整实现
  <algorithm_name>/        后续新增的其他方法
```

每种方法都在以算法名称命名的独立子目录中维护，该目录内放置算法说明、源码、测试、图解和实验结果。

SC-DynaTOGT 的可视化分为三层：预处理诊断图用于检查真实边界、内缩安全区和 SC 网格；Matplotlib 场景图只显示真实门框和四旋翼；可选 OpenGL 渲染器输出实体环境、追踪相机和 MP4。OpenGL 输出是已求解轨迹的离线渲染，不是 AirSim 动力学或传感器仿真。

SC-DynaTOGT 的本地结果按 `experiments/`、`demos/`、`diagnostics/` 和 `work/` 分类；`results/index.html` 是统一浏览入口。历史结果通过带 SHA-256 的迁移清单无损保留，不与新时间戳运行混放。
