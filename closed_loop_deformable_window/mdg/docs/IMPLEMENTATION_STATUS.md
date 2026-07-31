# MDG 实施状态

## M1 仓库审计：完成

- 复用 `sc_dynatogt/minco.py` 的 degree-7 MINCO 和 PVAJ 端点。
- 复用 `sc_dynatogt/time_mapping.py` 的 TOGT 正时间映射与时间反传。
- 复用 `sc_dynatogt/dynamics.py` 的四旋翼平坦性、动力学代价和约束极值。
- 复用 `sc_dynatogt/optimizer.py` 的 TOGT L-BFGS-B 数值协议。
- 参考 MSR 的高密度检查，在 MDG 内实现独立验证；不修改 Atlas、SC 或 MSR。

| 里程碑 | 状态 | 内容 |
|---|---|---|
| M2 | 已实现并通过单元/固定种子验收 | 五类窗口、动态安全区、场景和可视化 |
| M3 | 已实现并通过密集安全测试 | 多圆盘、Hungarian、PCHIP 和安全收缩 |
| M4 | 已实现并通过暴力一致性测试 | 粗图、细图和分层 DP |
| M5 | 已实现并通过梯度/闭环验收 | MINCO、自由点和完整时间梯度 |
| M6 | 已实现并通过 seed=0 修复案例 | Lazy Repair 在第二次阻断轨迹对后成功 |
| M7 | 已实现，E4 smoke 已运行 | 五种方法和 Dense Oracle |
| M8 | 部分完成 | 固定 seed=0 与 E1–E6 全烟测完成；正式 2,090 次尚未全量运行 |
| M9 | 已实现 | 文档和复现命令 |

本文件只在测试或实验真实完成后更新验收状态。
