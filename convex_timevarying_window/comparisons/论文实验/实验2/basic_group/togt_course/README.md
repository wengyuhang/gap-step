# 当前赛道：官方TOGT交错/叠窗布局的动态适配

来源、差异、难度和准入见[协议](PROTOCOL.md)，官方配置提取见[upstream.json](upstream.json)。
保留TOGT位置关系/顺序/前六姿态，场地2.2倍+统一离地平移，现有物理小窗不放大。
W3平移、W4摆动交替升档；其余五窗不同低速运动、初相0，四全局方法共40条纯轨迹。

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba \
/home/jack/anaconda3/envs/wyh/bin/python -u -m \
convex_timevarying_window.comparisons.论文实验.实验2.basic_group.togt_course.run run \
--outdir convex_timevarying_window/comparisons/论文实验/实验2/results/togt_layout_mixed_v1_20261106
```

严格先预检后正式；不重复启动，不操作Gazebo，不改算法或丢弃非目标窗风险。
被否决的外围布局separated_mixed_v1已停止，不当作当前赛道。
