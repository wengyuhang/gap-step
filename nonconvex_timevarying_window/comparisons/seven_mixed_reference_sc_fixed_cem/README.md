# 七窗口混合旋转赛道三方法比较

本目录冻结一个开放的七窗口赛道，用于比较原始 `Fixed-WP`、原始
`SC-DynaTOGT` 和 `Feasibility-Guided CEM`。所有窗口中心和平面固定，
只做面内自旋；顺序为：

`均衡 U → 利马松 → 星形 → 均衡 U → 五瓣波浪 → Line/Bézier → 均衡 U`。

三个均衡 U 位于既有三 U 赛道的 `x=0/9/18 m` 平面，以 `18 rad/s`
旋转。四个混合窗口插在 `x=3/6/12/15 m`，分别以
`-0.8/0.9/-0.7/0.8 rad/s` 旋转。插入窗口的中心由此前独立验收通过的
三 U 轨迹确定，使该轨迹在这些平面处穿过安全内缩区的 polylabel。
因此这是一个有已知可行种子的压力案例，用于验证硬约束筛选流程，不能当作
未知赛道上的无偏成功率证据。

Fixed-WP 固定使用每个安全内缩区域的 polylabel，只优化八段时间；原始
SC-DynaTOGT 从 Fixed-WP 的精确嵌入启动，联合优化八个 K 和七组二维 D。
新算法把既有三 U 可行轨迹提升为七窗口种子，再在原生 K 和 D 极坐标上运行
两轮完整协方差 CEM。所有中间硬约束失败候选全部删除，只在通过者中按飞行
时间排序，最后才进行真实姿态长方体整机审计。

正式运行：

```bash
conda run --no-capture-output -n wyh python -m \
  nonconvex_timevarying_window.comparisons.seven_mixed_reference_sc_fixed_cem.experiment \
  --outdir nonconvex_timevarying_window/comparisons/seven_mixed_reference_sc_fixed_cem/results/<run>
```

2026-09-09 的正式结果在
[`results/three_way_trial2_20260909/REPORT.md`](results/three_way_trial2_20260909/REPORT.md)：

| 方法 | 飞行时间 (s) | 动力学约束 | 碰撞约束 |
|---|---:|---:|---:|
| Fixed-WP | 8.666957342 | 不满足 | 不满足 |
| SC-DynaTOGT | 8.666957333 | 不满足 | 不满足 |
| Feasibility-Guided CEM | 7.390546627 | 满足 | 满足 |

Fixed-WP 和原始 SC 均在第一个 U 窗口发生整机碰撞；最大速度分别为
`9.415961208/9.415961274 m/s`，超过 `7 m/s` 上限。新算法最大速度为
`6.999106271 m/s`，七窗口整机审计全部通过。CEM 共评价 256 个扰动候选，
它们均未通过全部中间硬约束，最终返回作为候选集合成员保留的已知可行种子；
没有用失败程度替代可行性。

碰撞和动力学结论均为名义模型密集采样验证，不是连续时间认证。

五种唯一窗口的离线产物保存在 `preprocessed_gates/`。默认冻结配置在新进程中
通过 `PreprocessedGate.load()` 加载并交叉校验，在同一进程中再由 LRU 缓存直接
复用；实验入口不会重复求解 SC 参数。
