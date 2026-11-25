# 实验二当前赛道：独立穿窗区间

新赛道把“非目标窗口挡在连接航段上”从研究任务中排除，保留原七种开口/尺寸与x500内缩。
四平移、三摆动；仅W3和W6交替升频升幅，其余五窗维持各自固定低速运动。
[完整协议](PROTOCOL.md)定义10档、四方法40条；全局七窗返航，不接跟踪器或Gazebo。

协调器先做最大运动包络/非相邻走廊分离、运动接口一致性、静态四方法参考核验，以及低/最高动态TOGT非相邻风险核验。
全部通过后才计算其余正式条件。实际轨迹仍检查所有窗，不能仅凭锚点距离认定赛道没有交叉风险。

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba \
/home/jack/anaconda3/envs/wyh/bin/python -u -m \
convex_timevarying_window.comparisons.论文实验.实验2.basic_group.separated_course.run run \
--outdir convex_timevarying_window/comparisons/论文实验/实验2/results/separated_mixed_v1_20261106
```

输出REPORT/summary/status；course_layout PNG/PDF、geometry_audit/motion_audit、preflight_gate、逐条assessments和原规划、最终validation。
发生非相邻窗风险则停止，不擅自忽略、调整算法或改冻结赛道继续跑。
旧紧凑批次`basic_trajectory_f0p5_to3_a2_v1_20261106`已24/28停止，旧结果保留，最后中断项不计失败。
