# 求解失败修复验证

算法见[tilt_heading_minco](../../../../tilt_heading_minco/README.md)，判据见[PROTOCOL.md](PROTOCOL.md)。主工作树wyh单线程：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.comparisons.论文实验.实验2.attitude_repair.run --level a25_f3 --method ours --outdir <新的输出目录>
```

中心版用`--method center`。`--seconds-per-seed`默认180、`--max-iterations`默认2000；改变预算需另存协议，不覆盖已跑目录。
关键档已经在开发独立目录original_map_literal_a25_f3_ours/center完成，不为使用新入口再重跑择优；正式模块与开发模型逐值等价测试通过。源代码和历史结果分别保留。
