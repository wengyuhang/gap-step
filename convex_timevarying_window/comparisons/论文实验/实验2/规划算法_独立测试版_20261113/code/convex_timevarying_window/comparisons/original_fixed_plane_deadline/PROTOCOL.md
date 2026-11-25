# 带共同过窗时刻限制版：原位置固定平面回归

2026-10-07，用户要求“带时刻限制版，那用这个呢”。
准确版本：`wang_progress_mpc`，不是在原Checked上临时追加截止时间。

新旧已接纳轨迹共同覆盖的窗，候选穿越时刻不得延迟超过20ms；按窗身份比较，
同时进入优化罚项/首段时长上界和参考接纳判定，不对未知未来窗凭空设deadline。
保留旧两未来窗＋可停留末端、MINCO-s3/M=2片段、软净空/补点、55ms内部预算、
整机退出阶段机、原参考时间线、原condensed MPC和PX4；不是本轮N2/M4自由末端版。
已知旧阶段机的局限也不在该测试中偷偷修复。无合格候选时保持原实现行为。

仅新建场景适配：符号位姿/距离、继承的向量净空检查/自适应补点、局部事件坐标统一为
原不规则七窗固定平面、中心面内已知平移。规划、目标、截止规则、控制器源码不改。

两条件与刚才原Checked完全相同：0.6Hz/单侧1.2m/0相、1Hz/单侧2m/0相；各首次一次、
飞行60s、进程300s，算法失败不重试。不得使用历史矩形环线成绩充当本场景成绩。
逐条件核对world字节、scene、PX4；与旧MINCO/MPC还核对MPC配置。

用同一4ms独立真值审计：七窗正向各一次、全机离面、无反穿、返航、空间、保守球净空与接触。
完整规划/MPC<100ms，名义动力学另列；采样不是连续证书。
另复核每条接纳记录的共同窗延迟≤20ms，不只看圈时。失败不填写有效完成时间。
对原Checked/PVA是整套方案比较，不是只有时间限制一项不同。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline/tests convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline/results/new_run
```
