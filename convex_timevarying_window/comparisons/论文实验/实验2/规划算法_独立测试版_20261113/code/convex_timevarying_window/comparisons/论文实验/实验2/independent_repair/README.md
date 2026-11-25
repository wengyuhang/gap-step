# 独立赛道数值修复

当前回归入口为run，固定使用smooth_augmented.Planner，协议见PROTOCOL.md。其他坐标变换、稀疏二阶和缩放模块保存开发试验。

当前归一化SLSQP终止容差1e-5，原节点验收1e-5与完整几何/STL判据沿用原值。过严1e-9的v1批次已停止，正式新批次使用v2结果目录。各次四初值总成本完整计入。

完整目标赛道的四初值分别求解。原TOGT映射、全PVAJ与安全验收继续使用，数值恢复与时间优化都在该赛道完成。平滑姿态模型改变了航向定义，源码与结果显式标记。

在主仓库、wyh环境使用单线程BLAS执行

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.comparisons.论文实验.实验2.independent_repair.run run --outdir <新结果目录>
```

同入口validate操作只复算已保存结果。检查status、summary、REPORT、原始progress以及真实父子进程。每初值300秒，完整成本另计。
