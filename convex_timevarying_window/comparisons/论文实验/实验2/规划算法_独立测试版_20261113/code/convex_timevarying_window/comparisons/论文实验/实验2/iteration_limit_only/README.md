# 只提高迭代上限的独立对照

当前用户要求以本目录[协议](PROTOCOL.md)为准。每种真实赛道独立冷启动，仅SLSQP400→2000；不使用前一轮的幅度连续法、姿态更换、目标变换或其他赛道轨迹。

主工作树、wyh、单线程BLAS：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.comparisons.论文实验.实验2.iteration_limit_only.run run --outdir <新结果目录>
```

共9原失败记录，独立四初值。查status、summary、REPORT、progress以及实际父子进程，不能只凭status认为仍在运行。禁止重复启动协调器。
同入口`validate --outdir <目录>`独立复算，不启动新规划。每初值180s不变，超时与迭代失败分开；不自动增加墙钟预算。
