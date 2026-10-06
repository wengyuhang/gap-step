# 2023 Wang 独立复现：窗口中心运动匹配实验

日期：2026-09-23。与[当前方法中心运动扫描](../amplitude_racing/CENTER_MOTION_PROTOCOL.md)逐条件对齐。
这不是作者官方代码或原论文硬件成绩，而是仓库中已披露差异的[独立复现](../../../复现/polynomial_dynamic_racing_2023/REPRODUCTION.md)。

- 同一动态七窗口世界、PX4/x500、门运动函数、机体半径、0.35 m 经验跟踪余量、100 ms 完整在线周期门槛、60 s 赛段上限和 0.05 s 跟踪预览。
- 频率倍率 1/3/6/10 同时缩短平移和旋转周期；中心平移幅度 1/2/4 倍；旋转幅度固定 1 倍；相位固定为 0 和 2π/3。共 24 个预设条件，非随机成功率估计。
- 方法仅设 `--paper-polynomial`，动态门中心为硬过门点、局部 MINCO/L-BFGS 优化与短前缀准入。不得加入当前方法的 `--checked`、自由选点或整段准入。差异是所比较算法的一部分。
- Gazebo GUI 关闭。每个结果保存独立世界 SDF、PX4 ULog、估计轨迹、规划日志、门框接触日志、真值审计和源码快照/哈希。仅明确的 PX4 解锁基础设施失败允许一次独立重试；算法失败不重试。先前两次 GUI 诊断不计入本批次。
- 严格通过：顺序穿完七门并返航、零 Gazebo 门框接触、10 ms 真值采样的整机球—门套净空非负、工作区通过、每次完整在线规划周期均小于 100 ms。离散真值采样并非连续时间安全证书。失败或证据缺失不得填成功圈速。

运行命令：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/jack/anaconda3/envs/wyh/bin/python -u \
  -m convex_timevarying_window.comparisons.paper_baselines_2026.run \
  --method polynomial --stage center_motion \
  --outdir convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923
```

结果以 `summary.json`、各 `case_*/result.json`、`audit.json`、`localization_audit.json` 和控制台日志为准。
