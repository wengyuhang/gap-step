# Fast Global Safe DynaTOGT

该目录保存七凸动态窗口的独立全局安全规划原型，不修改原始 `togt/` 和
`conditional_dual_constraint_cem/`。算法严格从动态 TOGT 的全局 `[K,D]`、degree-7 MINCO、
发布版四旋翼动力学目标和 C++ L-BFGS 出发；不使用 CEM、随机候选或滚动重规划。

第一次实现测试了每扇门的 approach/passage/departure 三点和五点漏斗。结果表明锚点不是走廊：
七阶轨迹可在锚点之间鼓出安全区域，而且回程可能在非指定时刻接近其他门。因此最终快速原型在
MINCO 全轨迹上直接积分移动门框安全场，并由同一 MINCO adjoint 将安全梯度回传至 `[K,D]`。
安全场使用温度 `0.01 m` 的平滑最近边距离、无人机外接球半径 `0.3794227368 m` 和额外
`0.03 m` 优化余量。每段固定 8 个安全积分区间，安全权重 `10000`。

当前实现应准确称为 **全局安全增强 DynaTOGT**，而不是已经完成的凸控制点走廊或连续域认证器。
目录名称保留了本次走廊原型的研究来源。最终验收使用独立最大 1 ms 网格：真实姿态长方体与
保守外接球分别检查，动力学调用发布版 C++ `QuadManifold`。采样通过不能写成连续时间证书。

## 构建与运行

```bash
cd convex_timevarying_window/fast_global_corridor_togt
bash native/build.sh
TOGT_MOTION_SPEED_SCALE=2.5 \
TOGT_INITIAL_SPEED=10 \
TOGT_SAFETY_WEIGHT=10000 \
TOGT_DYNAMICS_WEIGHT=1 \
TOGT_SAFETY_NODES=8 \
TOGT_LBFGS_PAST=8 \
TOGT_OUTPUT_TIME_SCALE=1.02 \
native/build/global_corridor_togt \
  ../../复现/TOGT-Planner-reproduction/source/parameters/cpc \
  cpc_setups.yaml \
  ../togt/native/race_uzh_7g_dynamic_mixed_closed.yaml \
  results/new_run/trajectory.csv
```

`TOGT_OUTPUT_TIME_SCALE=1.02` 在优化后统一延长各段时间，并按新的绝对穿越时刻重新计算动态门
穿越点和MINCO轨迹。它不是静止后重新起飞。该小幅延拓用于给发布版软动力学限制增加密集审计
余量；飞行时间和最终安全均按延拓后的轨迹报告。

重复计时：

```bash
PYTHONPATH=../.. conda run --no-capture-output -n wyh python benchmark.py \
  --repeats 20 --output results/new_run/timing_summary.json
```

正式单场景结果见
[`results/seven_gate_speed2p5_final_smooth/REPORT.md`](results/seven_gate_speed2p5_final_smooth/REPORT.md)。

