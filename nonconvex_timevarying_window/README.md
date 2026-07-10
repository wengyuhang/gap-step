# 非凸时变窗口穿越研究

`nonconvex_timevarying_window/` 是整个研究任务的总目录。本项目在 TOGT 论文方法的基础上，研究四旋翼无人机如何按指定顺序穿越非凸且随时间变化的窗口。

详细的研究背景、问题范围和数学定义见 [PROBLEM_DEFINITION.md](PROBLEM_DEFINITION.md)。

## 已有方法

| 方法 | 状态 | 主要思路 |
|---|---|---|
| [AtlasDynaTOGT](atlas_dynatogt/README.md) | 已实现 | 将非凸窗口三角剖分为 chart atlas，联合优化穿越时间和穿越点 |

## 目录结构

```text
nonconvex_timevarying_window/
  README.md                 总任务和方法索引
  PROBLEM_DEFINITION.md     与具体算法无关的问题定义
  atlas_dynatogt/           AtlasDynaTOGT 方法的完整实现
  <algorithm_name>/        后续新增的其他方法
```

每种方法都在以算法名称命名的独立子目录中维护，该目录内放置算法说明、源码、测试、图解和实验结果。
