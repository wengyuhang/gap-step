# 实验三：单窗口 Sync 聚焦转速实验

本目录独立实现固定非凸 L/U 形窗口的转速扫描，比较：

- `Fixed-WP`：已有固定安全点、两段普通 degree-7 MINCO，只优化时间；
- `Optimized-MINCO`：普通 degree-7 MINCO，联合优化入口/穿越/出口三个路径节点和四段时间，没有 Sync；
- `SC+Sync`：已有 SC 选点与解析旋转同步段，两侧由 MINCO 按 PVAJ 连接。

三种方法共享时间、snap、动力学和全轨迹球包络碰撞代价。球包络半径为原始 `0.53008 × 0.53008 × 0.11780 m` 长方体外接球半径再加 `0.015 m`；最终验收按微分平坦性姿态检查真实长方体。门厚固定为 `0.14 m`。

## 固定几何与转速

转轴在缩放前分别固定于规范 L 形窄臂内的 `(0.4, 0.0)` 和 U 形凹槽中的 `(0.0, 1.5)`，之后平移为窗口局部原点。L 轴心局部最大容纳圆半径小于真实机体外接球半径；U 轴在凹槽非开口区，轴心容纳圆记为零。两种偏轴物理开口的最大内切圆半径均是规划包络半径的 `1.15` 倍，所以完整机体仍有几何可行的偏轴穿越区。

固定初相位为 `0.3 rad`，预先固定的角速度为 `0/1.5/3.0/4.5/6.0 rad/s`，共 10 个正式场景、30 个方法-场景求解。转轴、边界、权重和转速网格不根据实验结果调整。安全内缩面积阈值为 `1e-6 m²`。正式动力学限制不放宽；额外报告最大相对超限量，`<=5%` 只标为近边界敏感项，仍属正式失败。

## 权重与运行

权重来自与正式网格不相交的 U/star、尺寸比 `1.4`、角速度 `1.1 rad/s`、初相位 `0.6 rad` 校准任务，并保存为 `focused_results/frozen_weights.json`。正式场景不逐例调参。

从仓库根目录执行：

```bash
bash nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/run.sh
```

脚本可恢复续跑；已有 `results.csv` 行不会覆盖。每方法每场景求解预算默认 `180 s`。调试单场景可用：

```bash
bash nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/run.sh \
  --only L 1.15 3.0 0.3 --budget-seconds 30 --max-iterations 10 --no-plots
```

## 验收与输出

独立验收采用最大 `1 ms` 时间步长，并额外检查所有分段接口两侧、实际穿窗根、厚度带接口附近和最小净距邻域。检查实际姿态长方体碰撞、共享动力学限制、完整初终 PVAJ、C3 接口和有效穿窗次数。这是采样数值验收，不是连续时间证明。

`focused_results/` 保存配置、几何、原始轨迹、CSV/JSON 结果、成功率图、共同成功飞行时间图和共用速度色条/真实机体比例的代表性轨迹图。静止场景、失败和不利结果都保留。

用户收窄方案前已开始但未完成的 81 场景运行保留在 `superseded_81_grid_partial_20260906/`，不纳入本次正式统计。

## 拓扑保持 U 形的 Fixed-WP 反例搜索

`fixed_wp_u_search/` 另存正式网格之外的探索性反例。选中实例使用均衡 U、尺寸比 `1.9`、`4.5 rad/s`、初相位 `1.1 rad`；物理开口和内缩区均保持 `8` 顶点、`2` 凹顶点的 U 形。Fixed-WP 在动力学合格的情况下被真实长方体验收判定碰撞。完整数据和结论边界见 `fixed_wp_u_search/REPORT.md`。

复现选中实例与三方法核对：

```bash
conda run -n wyh python \
  nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/search_fixed_wp_u.py \
  --u-profile balanced --ratios 1.9 --omegas 4.5 --phases 1.1 \
  --methods Fixed-WP Optimized-MINCO SC+Sync \
  --output nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/fixed_wp_u_search/balanced_candidate_comparison

conda run -n wyh python \
  nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/plot_fixed_wp_counterexample.py
```
