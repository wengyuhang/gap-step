# MDG 实验协议

## 方法

- Center：安全区质心，不在内部时使用 `representative_point`。
- Largest-Disc：每时刻最大圆盘，后端允许圆盘内自由点。
- Uniform-Point Graph：安全区网格的五个确定性最远点。
- MDG-Center：完整多圆盘图，后端固定圆心。
- MDG-Free：完整方法。
- Dense Oracle：四窗口、`0.01 s`、八圆盘的近似参考。

## 正式规模

| 实验 | 共享场景数 | 方法运行数 |
|---|---:|---:|
| E1 形状×难度 | 150 | 750 |
| E2 关闭比例 | 80 | 400 |
| E3 可扩展性 | 80 | 400 |
| E4 近似最优性 | 40 | 240 |
| E5 消融 | 30×10 唯一配置 | 300 |
| E6 非圆心分析 | 复用 E5 | 0 |

总计 2,090 次方法运行。失败实例不参与飞行时间均值，但参与失败率。

```bash
cd closed_loop_deformable_window/mdg
python scripts/run_benchmark.py --config configs/smoke.yaml --suite smoke
python scripts/run_benchmark.py --config configs/default.yaml --suite formal --experiment E1
python scripts/run_benchmark.py --config configs/default.yaml --suite formal --experiment all
python scripts/plot_results.py --results results
```

每个任务先原子写入临时目录，完成后重命名；中断后可直接重跑。汇总输出全部运行
CSV、统计 CSV、LaTeX 表格、飞行时间箱线图和 `rho_i` 直方图。
