# 全局同赛道中心初始化

算法入口planner.Planner，默认先求当前完整动态赛道的中心子问题，再开放选点做联合精修。只在同一目标赛道内部传递PVAJ和时长。

实现沿用center_initialized_minco的可行基准保留机制，适配七窗完整PVAJ变量；原单窗混入类依赖固定七变量接口，因此单独保留全局适配。

实验入口run，协议见PROTOCOL.md。保留结果中selected_stage和free_refinement_converged，明确区分自由精修、中心基准与中心未找到后的自由冷启动。

在主工作树使用wyh及单线程BLAS运行完整五条件回归

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.comparisons.论文实验.实验2.same_course_center_recovery.run run --outdir <新结果目录>
```

solve操作加上--level可独立求任意已有目标等级。validate操作只复算保存的结果。

每条赛道从本次自己的中心四初值开始计算，随后只在同一赛道内部传递完整时长和PVAJ进行自由精修。
