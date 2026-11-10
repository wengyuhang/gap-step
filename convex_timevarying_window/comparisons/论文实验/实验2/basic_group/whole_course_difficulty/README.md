# 全赛道难度矩阵

协议见[PROTOCOL.md](PROTOCOL.md)。保持TOGT交错布局；七窗共同变化，四平移三摆动；幅度/开口宽度0.25/0.50/0.75 × 各自频率基准1/3/6倍，加静态，四全局方法共40记录。
不改规划器、不接Gazebo，保留无解和碰撞。旧两窗升档输出已停止，不续跑。

在主工作树、wyh环境启动一次（新结果目录必须不存在）：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty.run run --outdir convex_timevarying_window/comparisons/论文实验/实验2/results/togt_whole_course_factorial_v1_20261106
```

查status.json、summary.json、REPORT.md、真实协调/子进程和对应progress.jsonl，不能只凭状态文件判断。共享非阻塞文件锁阻止重复协调。
完整完成后同入口将run改validate，复核40条及冻结源，不启动新规划。静态继承来源、原始哈希见每条plans/static_*/reuse.json；原成本不是新计时。

测试显式指定本目录tests及相邻trajectory_only/tests、togt_course/tests，不用根裸pytest代替。
