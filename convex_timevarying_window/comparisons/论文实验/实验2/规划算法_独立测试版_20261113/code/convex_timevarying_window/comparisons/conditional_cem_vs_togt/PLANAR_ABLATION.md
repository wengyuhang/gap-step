# 平面平移--旋转消融协议

本协议替代“同时缩小窗口、增大幅值、提高频率”的混合难度分级。所有窗口都是同一高度的平行
竖直矩形，物理开口固定为 `3.30 m × 2.60 m`；窗口只沿世界 `y` 轴平移，并只绕竖直轴 yaw
旋转。周期固定为 10 s，不引入缩放或三维 RPY 变化。

两组实验独立报告：

- **运动强度扫描**：固定 5 个窗口，扫描 static / low / medium / high / very-high。基础幅度为
  `0.35 m` 平移和 `12 deg` yaw，等级系数为 `0 / 0.5 / 1.0 / 1.5 / 2.0`。
- **窗口数量扫描**：固定 medium 运动（`0.35 m`, `12 deg`），扫描 3 / 5 / 8 / 12 / 16 / 24 / 32 个窗口。起点到
  终点距离固定为 36 m，窗口等距重排，避免把赛道长度效应错误归为窗口数量效应。

每一因素水平使用相同数目的相位随机复现；同一复现的相位由固定种子生成。比较双方共享
轨迹参数化、原生 TOGT 动力学和最大 1 ms 的独立实体安全审计。

正式运行采用放宽预算 `population=64`、`rounds=50`、`repair-iterations=600`，并设置每赛道 CEM
墙钟上限 120 s，即与已有单场景
正式 Conditional CEM 运行相同量级。Conditional CEM 只有存在同时通过动力学和安全硬审计的候选
才记为输出成功。`no_audited_solution_under_budget` 的严格含义是“在该有限预算下没找到”，不是
物理不可行、也不是全局搜索失败的证明。

```bash
conda run --no-capture-output -n wyh python -m \
  convex_timevarying_window.comparisons.conditional_cem_vs_togt.planar_ablation \
  --outdir convex_timevarying_window/comparisons/conditional_cem_vs_togt/results/planar_ablation \
  --repeats 5 --population 64 --rounds 50 --repair-iterations 600 --cem-wall-seconds 120 --workers 4
```
