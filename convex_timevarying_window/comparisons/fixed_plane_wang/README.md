# 固定平面动态七窗：当前方法与 Wang

用户要求的新赛道比较，独立于旧旋转赛道。窗口不旋转、不沿法向平移，
只在自身固定竖直平面内左右往复。保留原七种开口尺寸、x500、PX4与返航要求。
新中心排成顺向闭环，不通过指定某个窗口的规划参数来调整算法。

- `scenario.py`：用物理频率Hz/单侧幅度m定义运动，生成结果目录里的Gazebo世界；
  同一显式场景接口供规划和事后审计使用。
- `flight.py`：仅替换场景，调用未修改的当前事件v4/消元MPC与Wang中心点/PVA独立复现。
- `run.py`：预声明12条件、两方交替先后成对飞行，冻结源码，保存所有尝试。
- `audit.py`：接触、整机球净空、各窗单次正向/整机离面、返航和在线调用验收。
- `probe.py`：只读订阅Gazebo门框实际位姿，检验运动函数；不向控制器提供真值。
- `report.py`：完整表格、配对成功圈时、失败诊断、速度与实际穿越点图。

本轮是整套方法对比，跟踪器也不同，不是仅改变穿越点的消融。
Wang是已有独立复现，不冒称作者官方性能。每条件一次，不视为随机成功概率估计。
采样净空和Gazebo零接触不是连续安全证明。详细设置见 [PROTOCOL.md](PROTOCOL.md)。

## 执行

在仓库根目录、conda `wyh` 下运行；每次使用新目录，不能覆盖历史数据。
启动器会重建专用容器 `convex_seven_togt_experiment`，不触碰独立回放容器。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q convex_timevarying_window/comparisons/fixed_plane_wang/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 python -u -m convex_timevarying_window.comparisons.fixed_plane_wang.run --pilot --outdir convex_timevarying_window/comparisons/fixed_plane_wang/results/new_pilot
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 python -u -m convex_timevarying_window.comparisons.fixed_plane_wang.run --outdir convex_timevarying_window/comparisons/fixed_plane_wang/results/new_formal
OPENBLAS_NUM_THREADS=1 python -m convex_timevarying_window.comparisons.fixed_plane_wang.report convex_timevarying_window/comparisons/fixed_plane_wang/results/new_formal
```

中断后 `run --resume --outdir 原目录`，只读取已有尝试，不重复飞已完成条件。
源码哈希不同则拒绝继续，不能把不同算法版本混入同一矩阵。

## 审计注意

飞行原入口的旧缩放字段被固定为中性值，**实际运动以 `scenario.json` 和 `world.sdf` 为准**。
不能直接调用旧赛道审计器审这批轨迹；单趟重审用：

```bash
python -m convex_timevarying_window.comparisons.fixed_plane_wang.audit RUN_DIRECTORY --method ours
```

Wang使用 `--method wang`。保留原始 `native_result.json`，补充场景元数据的
`result.json` 不改变原完成/碰撞/耗时记录。`source_snapshot/`及逐趟批次引用可核查版本。

留出联调保存于 `results/pilot_20261007/`，不计正式比较；正式记录在
`results/frozen_20261007/`。完整结论以该目录的 `status.json` 和生成后的 `REPORT.md` 为准。

2026-10-07用户中途要求先分析：正式首条件两方均已完成，剩余22趟暂停，未形成完整矩阵结论。
联调降速的只读原因见 [DIAGNOSIS.md](DIAGNOSIS.md)；两种算法仍未修改。
