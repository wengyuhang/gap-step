# 七种不同窗口的 SC-DynaTOGT 闭环球体碰撞赛道

本实验参考 `sc_dynatogt` 的 `20260717_paper_irregular_closed` 演示布局，
构造七个空间分散、形状互不重复的窗口：

`L → U → star → limacon → wavy → line_bezier → balanced_U`

起点和终点均为 `(-16, 4, 3.2) m`。前六个窗口沿用参考演示的中心和
平面朝向，第七个窗口位于 `(2, 8, 5.5) m`。八段名义中心距离为
`11.23–30.74 m`。窗口中心和平面固定，只绕各自法向自旋。

障碍物采用上传约束中“只有有限宽度门框”的情形：零厚度物理边界曲线是
障碍物，开口之外但远离边界的位置不是无限实体墙。球体半径为整机外接球
半径再加 `0.015 m`。检查只计算球体进入 `|z|≤r_s` 的全部时间区间，密集
步长不超过 `0.2 ms`，并在临界处细化到 `0.05 ms`。这是采样数值证据，
不是连续时间证书。

```bash
conda run --no-capture-output -n wyh python -m \
  nonconvex_timevarying_window.comparisons.seven_unique_sc_sphere.experiment \
  --outdir nonconvex_timevarying_window/comparisons/seven_unique_sc_sphere/results/<run>
```

2026-09-09 的运行结果位于
[`results/irregular_closed_20260909/REPORT.md`](results/irregular_closed_20260909/REPORT.md)，
赛道图为
[`results/irregular_closed_20260909/figures/route_overview.png`](results/irregular_closed_20260909/figures/route_overview.png)。
原始 SC-DynaTOGT 正常收敛，W5、W6、W7 的球体检查失败，W1–W4 通过。

早期沿 X 轴开放赛道试验保存在 `results/superseded_open_axis_trial_20260909/`，
不作为本闭环构造的结果。
