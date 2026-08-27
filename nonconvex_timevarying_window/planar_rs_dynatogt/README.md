# Planar-RS-DynaTOGT

这是 SIP-DynaTOGT 的固定平面特化版。它只接受窗口中心固定、窗口平面固定，且窗口仅在平面内旋转和统一缩放的问题。SC/MINCO 仍负责穿越点和七阶轨迹参数化；本目录新增的关键步骤是先用“机体到固定平面的严格分离界”排除绝大多数时间，再只对可能接触平面的短时间段运行原始曲线的 Arb 半无限认证。

普通单窗口实测已经达到一分钟内：端到端求解 37.59 s，飞行时间 1.83724 s，最终状态为 `CERTIFIED_FEASIBLE`。同一轨迹的认证由原始 SIP 的 48.53 s / 251,298 单元降至 10.28 s / 26,686 单元。六窗口极难闭环赛道不限时实测在约 30 分 6 秒在线计算后得到 `CERTIFIED_FEASIBLE`；这证明算法能求解该赛道，但也证明目前不能承诺任意复杂赛道一分钟或十分钟内完成。

## 保证范围

- 窗口状态必须是
  \(c_i(t)=c_i\)、\(R_i(t)=R_{i,0}R_z(\theta_i(t))\)、\(s_i(t)>0\)。
- 碰撞模型是与 SIP-DynaTOGT 相同的姿态长方体，默认半尺寸为 `(0.26504, 0.26504, 0.05890) m`，净距为 `0.015 m`。
- 直线、圆弧、Bézier、非有理 B-spline 都以原始曲线原语认证。SC 的高密度点仅用于离线映射，不会展开成 SIP 约束。
- 速度、机体系角速度、总推力、单旋翼推力和平坦性非奇异条件仍在全部连续时间上认证。
- 只有 `CERTIFIED_FEASIBLE` 表示名义模型下连续域安全；`VIOLATED` 表示 Arb 已确认的反例；`UNRESOLVED` 和 `NUMERICAL_FAILURE` 都不得解释为安全。
- 优化是 SLSQP 得到的局部时间优化解，不声明全局时间最优，也没有任意复杂度的固定最坏运行时间。

## 接口

```python
from nonconvex_timevarying_window.planar_rs_dynatogt import (
    PlanarRSConfig, PlanarRSMotion, make_planar_problem, solve, certify,
)

problem = make_planar_problem(sc_track, motions, original_boundaries)
result = solve(problem, PlanarRSConfig())
report = certify(problem, result.trajectory, PlanarRSConfig())
```

`original_boundaries` 必须是一窗一组原始曲线段。省略它会退化为对 SC 稠密物理多边形逐边认证，只适合兼容用途，不应在正式曲线实验中使用。

## 复现实验

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh

python -m nonconvex_timevarying_window.planar_rs_dynatogt.experiments \
  --case ordinary --baseline-certificate \
  --outdir nonconvex_timevarying_window/planar_rs_dynatogt/results/ordinary_20260826

python -m nonconvex_timevarying_window.planar_rs_dynatogt.verify \
  nonconvex_timevarying_window/planar_rs_dynatogt/results/ordinary_20260826

# 六窗口最终高精度重放
python -m nonconvex_timevarying_window.planar_rs_dynatogt.verify \
  nonconvex_timevarying_window/planar_rs_dynatogt/results/hard_unlimited_20260826 \
  --precision-bits 256 --max-cells 3000000 --max-depth 28 \
  --plane-prune-max-depth 14 --plane-prune-min-time-width 1e-5

pytest -q nonconvex_timevarying_window/planar_rs_dynatogt/tests
```

算法证明见 [ALGORITHM.md](ALGORITHM.md)，完整实测数据见 [TEST_RESULTS.md](TEST_RESULTS.md)。
