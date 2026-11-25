# 我们的现有算法 × 原位置固定平面场景

[冻结协议](PROTOCOL.md) · [六次实际PX4/Gazebo结果](results/frozen_v1_20261007/REPORT.md)

使用最新N2/M4自由点＋状态热启动版本，复用新Wang复现刚测试的原位置固定平面六条件。
只适配场景查询和共同时间线；不修改算法、MPC、场景难度或准入。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_ours/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 python -u -m convex_timevarying_window.comparisons.original_fixed_plane_ours.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_ours/results/new_run
```

仓库根目录、conda环境wyh。新输出目录必须不存在。会替换专用Gazebo/PX4实验容器。
2026-10-07六次均通过第一窗后参考耗尽，0/6完赛。原因是下一局部固定末端z=0.390m违反z≥1m准入。
结果不能当有效圈时，也不能说明物理赛道不可行；通用末端规则尚待修改。
