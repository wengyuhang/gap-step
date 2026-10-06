# 当前 checked 方法：窗口中心位移与频率扫描

日期：2026-09-23。目标是检验原视频中窗口中心移动不明显的原因，并在 PX4/x500/Gazebo 的同一动态七窗赛道上测量在线方法的实际表现。

- 频率倍率：1、3、6、10。与既有速率扫描一致，同时缩短平移和旋转周期；窗口旋转幅度固定为原始的 1 倍。
- 中心平移幅度倍率：1、2、4。七个窗口的三轴幅度均按同一个倍率缩放。原始单轴幅度为 0.10–0.20 m；4 倍时为 0.40–0.80 m，峰峰行程为 0.80–1.60 m。
- 相位：0 和 2π/3。每个条件各运行一次，共 24 次。这是两个预设场景，不是总体成功概率估计。
- 算法：`--checked --tracking-preview 0.05`，两窗口加可停车末段；算法参数、x500/PX4 参数、机体半径、0.35 m 经验跟踪余量和 100 ms 完整周期准入标准均不按条件调整。
- 每次最多 60 s 赛段，不含 30 s 起飞准备。保留零接触但超时、越界、求解超限、碰撞及真值缺失等不同负结果。
- 物理验收：顺序通过七门并返回终点、七个 Gazebo 接触话题零接触、10 ms 真值采样的机体球对实体门套净空非负、六侧工作空间检查通过。完整规划周期还需全部小于 100 ms。
- Gazebo GUI 关闭以维持计算环境一致。保存每次世界 SDF、飞控 ULog、规划周期、接触话题、PX4 估计状态、真值审计和源码哈希。候选失败仅可因基础设施问题重试一次，算法失败不重试。

运行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/jack/anaconda3/envs/wyh/bin/python -u \
  -m convex_timevarying_window.comparisons.amplitude_racing.run \
  --checked --tracking-preview 0.05 --center-motion-grid \
  --outdir convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923
```

世界物理插件按 `center0 + translation_amplitude × sin(2π(t−30)/translation_period + phase + axis_phase)` 更新实际模型位姿；规划器使用相同公式。扫描前直接读取正在运行的 10 倍、1 倍幅度 Gazebo `pose/info`：W1 在仿真 3124.156/3124.532/3124.912 s 的位置依次为约 `(-2.2536,-3.3962,6.5332)`、`(-2.4903,-3.6359,6.3826)`、`(-2.4822,-3.4758,6.5666)` m，与规划器按对应时刻查询的值在输出精度内一致。这个检查只证明该次场景的实际中心在移动和模型一致，不代替 24 次飞行验收。
