# Checked-Soft/MPC 38条件冻结复测

复用上一轮38条件并集、x500/PX4和旧主验收，仅冻结后的MPC版本重跑全部条件。
控制器/规划器不在本目录实现；本目录只负责运行、基础设施判定和对照报告。
实际协议见 [PROTOCOL.md](PROTOCOL.md)，控制器机制见
[方法README](../../checked_soft_mpc_racing/README.md)。

换跟踪器后降速的只读逐段核对见[降速诊断](SLOWDOWN_DIAGNOSIS.md)：
区分实际参考停车、跟踪误差和同进程CPU预算串扰；建议修复尚未实施，不修改冻结版本。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_racing.run --outdir convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/new_frozen
# 中断后只有冻结源码哈希一致才能恢复；已有结果不会重新飞。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_racing.run --resume --outdir convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/new_frozen
# 全部计时飞行结束后审计历史动力学/生成图，避免影响控制计算基准。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_racing.report convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/new_frozen --historical-dynamics
```

`summary.json`保留全部尝试；`first_attempt_summary.json`保留首尝试；
`condition_summary.json`仅在预声明准备异常后选用一次同代码重试。
正常起飞后的碰撞、超时、控制失效不重试刷成功。

主验收和严格12/8/42动力学验收分开，MPC/规划/完整发送的计算时限全部计入。
固定相位单次测试不等于随机成功概率；历史PVA的5个未运行条件无成对消融。
