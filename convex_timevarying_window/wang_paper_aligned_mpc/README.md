# N=2、M=4 的 Wang MINCO 对齐比较

详见 [逐项论文对齐与协议](ALIGNMENT.md)。旧代码、结果与暂停矩阵不改。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q convex_timevarying_window/wang_paper_aligned_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 python -u -m convex_timevarying_window.wang_paper_aligned_mpc.experiment --outdir convex_timevarying_window/wang_paper_aligned_mpc/results/new_comparison
```

源码：`planner.py` 优化，`trajectory.py` 四片段MINCO，`initialization.py` 全局/热启动，
`worker.py` 进程隔离，`px4.py` 共享MPC闭环，`flight.py` 固定平面Gazebo场景适配。
Wang 匹配基线入口同时保存在 `复现/polynomial_dynamic_racing_2023/paper_aligned/`。
