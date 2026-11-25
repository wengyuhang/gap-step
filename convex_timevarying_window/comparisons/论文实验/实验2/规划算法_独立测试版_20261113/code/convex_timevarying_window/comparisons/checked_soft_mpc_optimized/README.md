# 优化MPC全矩阵

2026-10-06正式结果：**38/38首次主验收通过**，零重试/零碰框；圈时中位27.560 s，
最小真值采样净空0.302116 m，规划/MPC最慢83.741/38.608 ms。
完整逐条件、所有历史方法对照及图见[REPORT.md](results/frozen_20261006/REPORT.md)。
中心24中相比旧Checked/Wang圈时差中位仍为+0.236/+0.032 s，不声称全面最快。

只运行独立进程+解析消元的优化版，不继续旧未优化MPC矩阵。
场景仍是已有38条件并集，详见[冻结协议](PROTOCOL.md)。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh \
 python -m convex_timevarying_window.comparisons.checked_soft_mpc_optimized.run \
 --outdir convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/new_frozen
# 如中断，加--resume；已有原始结果复用，不重飞挑成绩。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh \
 python -m convex_timevarying_window.comparisons.checked_soft_mpc_optimized.report \
 convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/new_frozen --historical-dynamics
```

报告只在所有计时飞行结束后运行。主验收保持无碰撞、真值采样净空、七窗返航及100 ms。
用户允许轻微名义动力学超限，因此实测12/8/42诊断单列、不作主失败条件。
首尝试、允许基础设施重试后分别报告，失败不填圈时；所有历史方法只读、按共同条件比较。
