# Phase-Governed SC Tracking（单窗负结果）

这个目录保留单窗诊断，不再作为多窗执行层方法。SC-DynaTOGT 离线生成空间轨迹后，调节器通过在起点等待改变单窗相位。多窗中等待会同时改变所有后续窗口的到达相位，且当前位置未必是安全悬停区，因此这个策略不能推广为多窗方法。

该设计不改变 SC 轨迹的空间路径，也不重定时它的速度、加速度、jerk 和 snap；它只选择一个更安全的窗口相位。名义轨迹与调节后轨迹的空间偏差因此为零。若一个旋转周期内没有候选同时满足整段采样无碰撞、默认 `20 mm` 预览净距、真实整机截面在物理开口内、质心在 SC 内缩区内，调节器明确返回失败。`20 mm` 预览门槛用于给最终临界时刻加密审计保留至少 `15 mm` 的目标余量。

运行 U 形反例与新方法：

```bash
conda run -n wyh python -m \
  nonconvex_timevarying_window.phase_governed_sc_tracking.experiment \
  --outdir nonconvex_timevarying_window/phase_governed_sc_tracking/results/u_w18_p1p1
```

运行单元测试：

```bash
conda run -n wyh pytest -q \
  nonconvex_timevarying_window/phase_governed_sc_tracking/tests
```

最终碰撞验收使用最大 `0.2 ms` 网格并加密分段接口、平面交点和最小净距邻域；执行时候选预览仍使用 `1 ms` 网格和更严的净距门槛。这是采样数值验收，不是 SIP 连续域证书。方法也不修复离线轨迹已存在的动力学超限；动力学与碰撞验收分开报告。
