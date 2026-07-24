# MSR-DynaTOGT 实验协议

## 场景

所有 A0–A3 共享同一个 `SCWindowTrack`、安全区域、动力学参数和种子：

| 标识 | 场景 |
|---|---|
| `static_single` | 静态 L 形非凸单窗口 |
| `translation_three` | L/U/五角星三个平移窗口 |
| `full_three` | 同三个窗口，同时平移、旋转和均匀缩放 |
| `paper_irregular_six` | 六形状 `paper_irregular` 不规则闭环，起点等于终点 |
| `hard_thrust_full` | 三窗口长距离三维折返、强运动场景，刻意提高推力超限概率 |

`smoke` 使用 1 个种子、4 类初值、每段 65 个高密度节点，并把每次 L-BFGS-B 限为 24 次迭代以控制快速验证时间；达到上限会令 `optimizer_success=false`，但不会取代独立的采样可行性判断。`formal` 默认使用连续 155 个种子、7 个初值和每段 129 个节点，保留 SC-DynaTOGT 的历史代价停止规则而不设置人为迭代上限，尽量对齐其正式动态实验规模。

## 消融与公平比较

- A0：原始 SC；
- A1：SC + 多初值；
- A2：SC + 可行性修复；
- A3：完整 MSR。

每个实验同时输出 `native`、`matched_starts` 和 `matched_time`。`matched_time` 使用本次运行真实记录的墙钟时间，不伪造预算内启动数；由于不可抢占一次 L-BFGS-B，第一次启动总会完整执行。

AtlasDynaTOGT 只输出接口/模型差异说明。它与本实验动力学后端不同，不进入 A0–A3 直接数值排名。

## 指标

`runs.csv` 每行至少含：

- method、scene、seed、comparison_protocol；
- success 与独立的 optimizer_success；
- total_time、wall_clock_seconds、iterations、evaluations；
- 窗口顺序/内部合法性、最小边界裕度；
- 最大角速度、速度、总推力、最小/最大单旋翼推力；
- sampled_dynamic_limits_satisfied；
- 初值类型、候选数量、修复触发/成功、模式与缩放倍数；
- 修复前后总时间、再优化改善量和是否接受；
- 失败原因。

## 运行与输出

```bash
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite formal
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite formal --workers 2
```

`formal` 默认按当前进程的 CPU 亲和性选择并发数（上限 12），并行处理彼此独立的场景—种子任务；这只改变批处理方式，不改变单次求解算法。每个任务完成后会立即写入 `candidates/*.json`、`runs.partial.csv` 和 `status.json`。若运行中断，可以在完全相同的 suite、种子、场景和修复模式下续跑：

```bash
python -m nonconvex_timevarying_window.msr_dynatogt.experiments \
  --suite formal --workers 2 \
  --resume nonconvex_timevarying_window/msr_dynatogt/results/<timestamp>_formal
```

续跑不会重算或覆盖已完整落盘的场景—种子任务。

每次运行自动写入：

```text
results/<YYYYmmdd_HHMMSS_microseconds>_<suite>/
  config.json
  runs.csv
  summary.json
  REPORT.md
  FIGURE_EXPLANATIONS.md
  status.json
  atlas_auxiliary_comparison.json
  candidates/*.json
  figures/trajectory_comparison_*.png
  figures/total_time_comparison.png
  figures/computation_time_comparison.png
  figures/sampled_dynamic_feasibility_rate.png
  figures/repair_thrust_before_after.png
```

时间戳目录通过独占创建，现有目录不会被覆盖。中文报告按保存的 CSV/summary 自动生成，失败种子和原因不会被隐藏。`FIGURE_EXPLANATIONS.md` 逐张解释五张代表轨迹图和四张汇总图，并明确提醒不能脱离动力学可行率单独解读飞行时间。
