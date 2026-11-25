# 当前独立求解版本的完整实验二

run.py协调原10条件×4方法。本文自由与中心统一使用independent_free_solve.Planner。25条同版本或固定基线结果核验后继承，15条新算，全部重新评估。

从主工作树使用wyh和单线程BLAS运行

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.comparisons.论文实验.实验2.independent_full_matrix.run run --outdir <新结果目录>
```

同入口validate复核已有结果。协议见PROTOCOL.md，名义几何与节点间动力学诊断分别报告，时间包含七窗后完整返航。
