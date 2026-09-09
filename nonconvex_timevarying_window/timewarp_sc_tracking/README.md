# Local Time-Warp SC Tracking

该方法不假设无人机可以在多窗任务中随时等待。对第 `i` 个窗口，执行层在名义穿越时刻周围选择一个有限修正区间 `[t_a,t_r]`，用局部时间映射

```text
tau(t) = t + delta * 256*s^4*(1-s)^4,
s = (t-t_a)/(t_r-t_a)
```

暂时调节 SC 轨迹的进度。该 bump 在两端的 `0..3` 阶导数均为零，所以修正轨迹在 `t_a` 和 `t_r` 与离线轨迹保持 PVAJ 连续。更关键的是 `tau(t_r)=t_r`：恢复时刻之后完全回到原时间轴，后续窗口的到达时刻不被当前修正推迟。多个不重叠修正区间可以串联。

在线部分只需做一维候选搜索：在受限的 `delta` 网格上预览真实窗口的绝对时间、整机碰撞和动力学，选修正量最小的可行项。搜索区间的恢复时刻必须早于下一窗的名义穿越时刻。若无解，必须在进入该区间前触发重新规划，不能默认悬停。

当前实现提供可解析计算到 crackle 的时间变形轨迹、多补丁恢复测试和一个固定参数的 U 形回归实验。自动在线候选选择器尚未实现，因此本目录仍是执行层基础组件，不是完整控制器。

U 形回归的 [报告](results/u_w18_p1p1_local_rejoin_0p2ms_20260908_v3/REPORT.md) 使用最大 `0.2 ms` 网格并加密临界时刻：Fixed-WP 与 SC-DynaTOGT 分别有 `217` 和 `222` 个碰撞样本，局部时间回接为 `0`，且总时间不变。该案例三者均未通过采样动力学限制；结果只支持碰撞消除与时间回接，不支持完整飞控可行。

运行：

```bash
conda run -n wyh python -m \
  nonconvex_timevarying_window.timewarp_sc_tracking.experiment \
  --outdir <new-result-directory>

conda run -n wyh pytest -q \
  nonconvex_timevarying_window/timewarp_sc_tracking/tests
```
