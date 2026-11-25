# 删除求解后整段几何/动力学复查：仅仿真消融

2026-10-07，用户要求删除轨迹检查并试飞。本轮针对上条诊断中导致拒绝的`validate(plan)`。
单一改变：不执行候选全局部时域的密集门框/工作区/动力学/末端保持复查，也不依据该复查净空拒绝。
`prefix_clearance`为null表示未检查，不返回伪造的正净空。原算法和历史实验不覆盖。

保留优化中的所有约束/罚项、残差<0.003、原自适应补点、共同窗20ms时刻限制、原MPC/飞控。
保留单次正向穿窗/禁止返穿事件检查、整机退出进度、参考接续和100ms门槛。
因此不是“完全不检查轨迹”：只是移除几何/动力学事后准入这一个因素，事件正确性仍检查。
可停车末段属于被测试历史方法，不在此消融中删除。只限Gazebo，不推断实物安全。

两条件：原七窗固定平面、中心平移0.6Hz/1.2m/0相和1Hz/2m/0相，各首次一次、60s飞行上限。
与紧邻上一轮带检查deadline版核对世界字节、scene、MPC、PX4与配置一致。源码冻结，失败不重试。
不改变场景难度、不放宽事后成功判定、不在飞行中调参。

事后仍用4ms真值、Gazebo接触、窗口事件、房间和返航条件核验，分别报告程序完成和主验收通过。
离线对实际接纳多项式重新调用原validate，统计多少条会被原检查拒绝；这个离线检查不影响控制。
事故、反穿、越界、超时都保留；没有接触不自动等于安全，采样不是连续安全认证。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/tests convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/results/new_run
```
