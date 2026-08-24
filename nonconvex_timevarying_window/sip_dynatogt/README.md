# SIP-DynaTOGT

SIP-DynaTOGT 在不改动 SC-DynaTOGT 的 `[K,D]`、七阶 MINCO 和四旋翼平坦性实现的前提下，求解「真实门框边界 × 连续时间」上的整机半无限安全约束。

本方法只有一个求解循环：

1. SLSQP 求解当前有限 witness 约束问题；
2. Arb 区间分支定界检查完整连续域；
3. 将发现的违规 `(t,xi)` 加回 SLSQP；
4. 只有全域证明完成才返回 `CERTIFIED_FEASIBLE`。

## 安装

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
pip install -r nonconvex_timevarying_window/sip_dynatogt/requirements.txt
```

`python-flint` 是认证必需依赖。缺少它时必须返回 `NUMERICAL_FAILURE`，不会退化为采样检查。

## Python API

```python
from nonconvex_timevarying_window.sip_dynatogt import (
    SIPConfig, SIPProblem, certify, save_run, solve,
)

problem = SIPProblem.from_track(sc_scenario.track)
config = SIPConfig()
result = solve(problem, config)

if result.success:
    save_run("results/my_run", problem, config, result)
```

`SIPProblem.from_track(track)` 把现有 `physical_boundary` 折线作为真实边界。若原始边界是圆弧、Bézier 或 B-spline，应通过 `boundaries=` 显式传入原始 primitive，不要用稠密折线冒充曲线证明。

## 命令行

```bash
python -m nonconvex_timevarying_window.sip_dynatogt.experiments \
  --suite smoke --outdir nonconvex_timevarying_window/sip_dynatogt/results/smoke

python -m nonconvex_timevarying_window.sip_dynatogt.experiments \
  --suite formal --outdir nonconvex_timevarying_window/sip_dynatogt/results/formal

python -m nonconvex_timevarying_window.sip_dynatogt.verify \
  --run nonconvex_timevarying_window/sip_dynatogt/results/smoke/static_1gate
```

`verify` 从序列化后的边界、窗口运动、物理参数和多项式系数重新计算，不相信保存的状态字段。

## 状态语义

- `CERTIFIED_FEASIBLE`：完整连续时间、每个真实边界参数段及全部硬动力学约束都有 Arb 有限覆盖证明。
- `VIOLATED`：发现具体违规 witness，或某个区间的违规下界已严格为正。
- `UNRESOLVED`：轨迹可能安全，但在给定细分预算内无法证明；绝不等于安全。
- `NUMERICAL_FAILURE`：依赖、平坦性或数值重建失败；同样不等于安全。

## 保证边界

- 证明对象是序列化的名义二进制浮点模型和导出的分段多项式轨迹。
- 支持现有 SC 实现的零 constant-yaw、周期平移/RPY 旋转/均匀尺度运动，以及 Line/CircularArc/Bézier/非有理 B-spline。
- 保证包含整机长方体至真实边框的净距离、速度、角速度、总推力、单桨推力和平坦性非奇异性。
- 不包含感知误差、建模误差、风扰或闭环跟踪管。
- SLSQP 只给局部候选；不声称全局最优，也不把「无认证候选」说成全局不可行。

完整的实现、数学、区间证明、状态语义、历史实验参数和公平比较边界见 [COMPLETE_ALGORITHM_SPECIFICATION.md](COMPLETE_ALGORITHM_SPECIFICATION.md)。数学摘要见 [ALGORITHM.md](ALGORITHM.md)，文献关系见 [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md)，测试记录见 [TEST_RESULTS.md](TEST_RESULTS.md)。
