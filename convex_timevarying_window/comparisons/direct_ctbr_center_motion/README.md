# Direct CTBR：动态七窗中心运动匹配实验

2026-09-24 运行了昨日 checked 与 Wang 2023 独立复现使用的同一24条件矩阵。
PX4/x500/Gazebo、窗口运动与严格验收一致；直接CTBR与旧PVA/50 ms预览属于不同控制接口，
因此这不是单组件消融。详见[冻结协议](PROTOCOL.md)。

严格通过：direct CTBR 22/24，checked 23/24，Wang独立复现17/24。
与checked共同通过21组，direct更快0组，圈时差中位数+9.496 s；
与Wang共同通过16组，direct更快0组，差值中位数+8.820 s。
两次失败均为最新状态下无合格CTBR首拍，未碰框但也未完赛。
逐条件与证据见[比较报告](results/direct_ctbr_20260924/COMPARISON_REPORT.md)。

运行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /home/jack/anaconda3/envs/wyh/bin/python -m \
  convex_timevarying_window.comparisons.direct_ctbr_center_motion.run \
  --outdir convex_timevarying_window/comparisons/direct_ctbr_center_motion/results/NEW_RUN
```

比较：

```bash
/home/jack/anaconda3/envs/wyh/bin/python -m \
  convex_timevarying_window.comparisons.direct_ctbr_center_motion.compare \
  convex_timevarying_window/comparisons/direct_ctbr_center_motion/results/NEW_RUN
```

`results/`按仓库约定忽略提交；本地原始世界、ULog、接触记录和真值审计仍保留。
