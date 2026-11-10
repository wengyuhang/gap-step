# 基础组：纯轨迹实验（当前入口）

用户2026-11-06要求不再实飞，仅比较规划参考的完整任务时间与碰撞。
本阶段静态+2m振幅、0.5Hz递增至3Hz，四方法共28项；暂不增加其他布局或幅度。
已有静态、1Hz/2m、2Hz/2m共12份相同冻结规划复用并重新评估，新增16份规划。

[协议](PROTOCOL.md)；输出`../../results/basic_trajectory_f0p5_to3_a2_v1_20261106`。
时间为八段时长和，含最后窗后返航至零VAJ；不是历史Gazebo完成时间。
碰撞为包络球与移动实体STL网格相交；不再检查跟踪误差，不把名义结果当实飞安全。
未找到规划、碰撞、任务未完成、检查待定分开，全部保留。

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba \
/home/jack/anaconda3/envs/wyh/bin/python -u -m \
convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.run run \
--outdir convex_timevarying_window/comparisons/论文实验/实验2/results/basic_trajectory_f0p5_to3_a2_v1_20261106
```

单一协调进程与进程锁，源码/协议/网格冻结；旧实飞数据不覆盖，不启动任何Gazebo/PX4/容器或定时任务。
主表`REPORT.md`，逐项`summary.json`，详细`assessments/*.json`，进度`status.json`。
最终`validation.json`重新评估全28项，碰撞见证额外用全三角面枚举核对。
