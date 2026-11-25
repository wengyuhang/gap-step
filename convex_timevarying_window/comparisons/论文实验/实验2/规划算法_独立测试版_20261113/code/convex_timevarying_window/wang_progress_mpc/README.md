# 公共过窗时刻接续修复

2026-10-07。用户授权修复固定平面联调中观察到的跨窗口竞速退步。
独立扩展`wang_free_state_mpc`，不修改旧事件v4、Wang独立复现或共享MPC。
只使用已接纳参考的共同未过窗口，不读取赛道条件标签、结果目录或指定窗口编号。

核心：新旧滚动规划虽然起始窗编号不同，仍应按实际窗身份比较共同窗口的过窗时间。
保留原20ms数值容差；不直接比较覆盖不同窗口的新旧终点时间。
同一条件同时进入有界L-BFGS搜索（累计时长违反罚项、首窗剩余时长上界）和独立接纳判定。
不是仅在事后拒绝一个已经很慢的解。没有合格替代时仍执行已接纳参考，不发布停车动作。
旧参考只作为有效前缀和初值；不宣称新曲线逐点冻结旧前缀，也不宣称延伸必定可行。

保留原MINCO-s3曲线、两未来窗、55ms内部预算、0.35m余量、软净空、名义动力学、
整机出窗/单次正向事件检查、MPC和PX4参数。所有新计算计入原100ms在线门槛。
采样准入不是连续安全或递归可行证明；更严的竞速时限可能导致延伸失败，必须保留负结果。

预声明开发验证：固定平面慢联调(.8Hz,1.6m,.4rad)和先前快速条件(.6Hz,1.2m,0rad)，
本版各飞一次，仅日志证明的设施故障允许一次重试。原正式矩阵仍暂停，不向其混入新版结果。
复核全部接纳日志，任何共同窗推迟超过20ms必须判机制失败；同时保留原飞行安全/返航/实时验收。
对照此前已保存的旧版及Wang同条件飞行，明确不是随机重复统计或作者官方性能。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 python -u -m convex_timevarying_window.wang_progress_mpc.experiment --outdir convex_timevarying_window/wang_progress_mpc/results/new_run
```
