# 简化版对比实验协议

## 方法

1. `sc_sphere`：原 `0.315 m` 球形内缩；
2. `point_model`：优化时只检查中心，最终仍用完整长方体检查，是不安全乐观上界；
3. `wbsc_dynatogt`：联合优化 `[K,D,Y]`；每次迭代由当前轨迹恢复 roll/pitch/yaw，并施加姿态长方体硬约束。

smoke 运行上述三种方法，用于流程和不安全点模型对照。formal 只运行用户要求的两个主方法 `sc_sphere` 与 `wbsc_dynatogt`。

## 场景

- 六类窗口各 100,000 个中心—姿态组合，用于分析球模型丢失但长方体可行的候选比例；
- 静态窄 L 和静态 U/曲线混合窗口，各 30 个配对种子；
- 平移、旋转、缩放的 L→U→星形场景，155 个配对种子。

## 公平对比口径

- 原 SC 和 WBSC 使用完全相同的场景、种子、窗口运动和初始位置/时间构造。
- 原 SC 保留自身 `0.315 m` 局部球形内缩，按该模型检查安全；WBSC 按带 `0.015 m` 裕度的姿态长方体检查安全。
- formal 两者都恢复原 SC-DynaTOGT 的无固定迭代上限和 past-32 相对代价收敛判据；为使全量配对实验可执行，两者统一每段使用 6 个优化积分节点。最终动力学极值仍以每段至少 33 个节点重算。
- 收敛率用全部尝试作为分母；安全率另行在已收敛结果中统计。
- 总飞行时间、最小间隙和方法间飞行时间差只使用“双方均收敛且各自模型安全”的配对种子。
- 动力学采样可行率、roll/pitch/yaw 峰值和候选恢复率单独报告。

比例使用 Wilson 95% 区间；连续指标使用 10,000 次 bootstrap 区间。球内缩为空仍留在收敛率分母；formal 只报告观察值，不预设新方法必须胜出。

smoke 仍使用 24 次迭代上限做快速端到端检查，不用于正式方法结论。

## 命令

```bash
python -m nonconvex_timevarying_window.wb_sc_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.wb_sc_dynatogt.experiments --suite formal
```
