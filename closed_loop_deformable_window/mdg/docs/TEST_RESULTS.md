# MDG 测试与验收记录

## 自动测试

```text
python -m pytest -q closed_loop_deformable_window/mdg/tests      18 passed
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests         111 passed
pytest -q nonconvex_timevarying_window/msr_dynatogt/tests        8 passed
pytest -q                                                       46 passed
```

SC 与 MSR 必须分别运行，因为两个测试目录都有顶层 `test_experiments.py`，放在同一
pytest 进程会产生测试模块同名冲突。

MDG 测试包含五类形状各 100 个参数、1,000 时刻圆盘包含、10,000 个自由点和有限
差分雅可比、Hungarian/PCHIP 连续性、DP 与暴力枚举一致性、MINCO 闭环和小型端到端。

## 默认 seed=0 验收

命令：

```bash
python scripts/run_single.py \
  --config configs/default.yaml \
  --seed 0 --method mdg_free --gate-count 8 \
  --difficulty medium --closed-ratio 0.20 \
  --outdir results/raw/acceptance/mdg_free/seed_0 \
  --save-video
```

结果：

- 成功，飞行时间 `57.2505958484 s`；
- 两次 Lazy Repair 后通过；
- 最大平面误差 `1.43e-6 m`，最小余量 `0.04491 m`；
- 最大速度 `23.3368 m/s`，最大 body rate `1.4758 rad/s`；
- 最大单旋翼推力 `5.00095 N`，在配置的 0.1% 数值容差内；
- 闭环位置误差 `1.36e-15 m`，姿态误差 `0 deg`；
- 指定顺序、每门一次、动力学、门框、轨迹区间和连续性全部通过；
- 最大非圆心比例 `rho=0.08119`。

相同命令第二次运行后，`scenario.json`、`disc_tracks.json`、粗细图、`backend.json`
和轨迹 CSV 的 SHA-256 完全一致。MP4 首帧成功解码为 `1080×608`，轨迹 CSV
包含 1,153 个样本。

## E1–E6 smoke

E1–E5 的 31 个方法任务均完成并落盘（E6 复用 E5），其中 28 成功、3 失败且
失败原因完整保留。E4 的六种方法均完成；Center、Uniform-Point、MDG-Center、
MDG-Free 和 Dense Oracle 成功，Largest-Disc 失败。已生成：

- `tables/all_runs.csv` 与 `tables/summary.csv|tex`；
- `tables/e4_oracle_gap.csv`；
- `tables/e6_noncenter_pairs.csv` 与 `e6_noncenter_analysis.json`；
- 飞行时间比较图和 `rho` 直方图。

正式 E1–E6 共 2,090 次方法运行，配置和断点续跑入口已就绪，但本记录不把尚未
实际结束的正式矩阵标记为完成。
