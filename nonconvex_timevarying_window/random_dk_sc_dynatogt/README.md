# Random-DK SC-DynaTOGT

原始 SC-DynaTOGT 求解后的局部随机修复方法。窗口固定中心与平面、仅绕法向自旋；当前实验为零厚度均衡 U 单窗。方法保留原 SC 映射、正时间映射、degree-7 MINCO、原生目标与 L-BFGS，不添加 Sync 段或执行层时间补丁。

用户于 2026-09-09 指定：直接扰动 D/K；任何硬要求不通过即淘汰；只在合格候选中选择最好的；球体约束负责候选安全筛选，真实整机只检测最终轨迹。

## 算法

1. 原始 SC 求得 `x0=[K0,D0]`，保存原始初值、优化器状态和完整轨迹。收敛不代表全局最优或轨迹合格。
2. 用独立筛选器检查原始解。合格时直接进入最终整机检测；不合格时固定以 `x0` 为中心生成全部随机候选。
3. 每个候选直接重新执行原 SC `forward`，更新时间、窗口相位、穿越点和 MINCO。候选不再运行 L-BFGS，搜索中心不移动。
4. 几何、穿越顺序/次数、起终 PVAJ/C3 和全程动力学全部通过后，才按总飞行时间升序排序。不用惩罚值把不合格者混入排名。
5. 按排名依次做最终真实姿态长方体检测；失败就淘汰，直到找到通过者或候选耗尽。无合格项返回 `NO_FEASIBLE_CANDIDATE_FOUND`，没有“最少违规”回退。

原始候选通过快筛、但最终整机检测失败时，本轮直接返回失败；不隐式另开第二轮随机预算。CLI 的最终整机检测目前仅适配单窗，不能当作完整多窗验收入口。

新增的独立 `multi_window.py` 入口支持三窗整轨迹求解、筛选和最终逐窗整机审计；上述原单窗 CLI 保持不变。三窗首次 4000 候选运行未找到合格解，参数维度、场景与结果见 [MULTI_WINDOW_RESULTS.md](MULTI_WINDOW_RESULTS.md)。每窗一组二维 D，三窗即三组 D（六个标量分量），不是六个穿越点。

三窗后续将 D 最大比例扩大到 8、K 最大比例扩大到 4，运行 8000 候选，57 个通过三窗几何/顺序但全部超速，仍无合格解；见 [MULTI_WINDOW_WIDE_RESULTS.md](MULTI_WINDOW_WIDE_RESULTS.md)。

## 固定的第一版实验协议

- 窗口：固定中心/平面，均衡 U，尺寸比 1.9，零厚度，18 rad/s，初相位 1.1 rad；起终状态和动力学上限继承共享反例。
- 默认随机种子 `20260909`。三个尺度各 100 个候选，共 300 个；每尺度 25 个仅 D、25 个仅 K、50 个 D/K 联合。所有候选都检查，不在第一次可行时提前停止。
- 对 K **直接加扰动**：`delta_K_j ~ U[-beta*max(1,abs(K0_j)), +beta*max(1,abs(K0_j))]`，三档 `beta=0.01/0.03/0.05`。这些是 K 参数尺度，绝不是实际时长百分比。
- 每个二维 D 在半径 `alpha*max(1,norm(d0_i))` 的圆盘内均匀采样，`alpha=0.02/0.05/0.10`。圆盘采样使用 `sqrt(U)` 半径，不在物理开口中假装均匀。
- 默认从原始 SC 自身初值求解；`--initialization fixed-wp` 是独立初始化诊断，先求 Fixed-WP，再以其解热启动原始 SC。两组结果分别报告，不能悄悄替换失败的默认运行。

## 候选安全筛选

依据用户提供的 [球形约束](SPHERE_CONSTRAINT.md)：`phi=z²+d_F(q)²-r_s² >= 0`。本实验明确把物理开口外整个平面视作实体，`F` 是开口的闭补集：投影点位于开口内部时取到边界的距离，位于外部/实体中时距离为零。不能用凸包，也不能对开口外点继续取正的边界距离。

`r_s=body.circumscribed_radius+0.015 m`，约 `0.3944227368 m`。外接球是姿态无关的保守包络；相对于真实长方体，它可能淘汰本来可通过的轨迹。

每条候选对所有 MINCO 段在归一化时间内求 `z=±r_s`，用导数根划分单调区间并求数值根，检查全部 `|z|<r_s` 时间区间，包括多次接近平面的情况。分段接口、区间端点和平面根进入网格。根计算异常、整段位于接触平面等退化情况按数值失败处理。

第一层网格不超过 `min(2 ms, 1 degree/abs(omega))`；发现违规立即淘汰。通过后，全接触区间最大步长 `0.2 ms`；小于 5 mm 余量、采样局部极小值和区间端点附近加密到 `0.05 ms`。5 mm 仅为加密阈值。安全余量落在 ±1 nm 数值带内记 `UNRESOLVED`，不作为合格项；这是保守的数值处理，数学公式本身允许相切。SC 多边形成员检查沿用现有 1 nm 数值容差。

动力学保持原始 SC 的姿态/平坦性恢复与速度、总推力、XY/Z 角速率、单旋翼推力上限，整条飞行轨迹最大步长 `1 ms`。候选先做便宜的速度筛选，再调用原生 `flatness_map`；首次失败立即终止。没有换用另一个 tilt-yaw 模型，也不放宽现有动力学上限。

最终真实整机检测复用最大 `0.2 ms` 加临界加密审计，并补充完整机体平面截面在物理开口内的检查，以保持“开口外为实体”的模型一致。最终审计还会复验动力学。所有结果为名义模型下**采样数值验收**，不是连续域证书。

## 运行与产物

使用 conda `wyh`，依赖沿用 [SC README](../sc_dynatogt/README.md)。从仓库根目录：

```bash
conda run -n wyh python -m nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment \
  --outdir nonconvex_timevarying_window/random_dk_sc_dynatogt/results/new_run

conda run -n wyh python -m nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment \
  --initialization fixed-wp \
  --outdir nonconvex_timevarying_window/random_dk_sc_dynatogt/results/new_fixed_warm_run

conda run -n wyh pytest -q nonconvex_timevarying_window/random_dk_sc_dynatogt/tests
```

结果目录必须不存在。保存协议、完整几何与限制、原始解和轨迹系数、逐候选 D/K/扰动/检查状态/耗时、分尺度分类型统计、最终整机审计以及选中轨迹。`--baseline-json <本方法先前的baseline.json>` 可重放同场景同配置解；重放显式记录来源，不能把旧求解耗时当作本轮求解耗时。

筛选顺序采用短路判断，所以淘汰原因统计是**首个发现的失败原因**；它不代表其他条件已通过。球体筛选更便宜是实现动机，未运行逐候选整机检测对照前不宣称加速倍数。

## 扩大扰动范围

2026-09-09 用户要求在首次失败后扩大范围。CLI 支持显式指定匹配长度的 `--d-scales` 和 `--k-scales`，默认仍保留最初小范围协议。扩大版采用 D 半径比例 `0.25/0.5/1/2`，K 直接加噪声比例 `0.1/0.25/0.5/1`；每档仍各含 25% 仅 D、25% 仅 K 和 50% 联合候选。D 半径达到中心模长的 2 倍时已是较宽搜索，不应称为微小扰动。

下面重放上一轮热启动组的同一 SC 解，在扩大范围内每档取 1000 个样本，共 4000 个；不重新求解 SC，不放宽任何筛选阈值：

```bash
conda run -n wyh python -m nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment \
  --initialization fixed-wp \
  --baseline-json nonconvex_timevarying_window/random_dk_sc_dynatogt/results/u_w18_fixed_warm_seed20260909_300/baseline.json \
  --d-scales 0.25 0.5 1 2 --k-scales 0.1 0.25 0.5 1 --per-scale 1000 \
  --outdir nonconvex_timevarying_window/random_dk_sc_dynatogt/results/new_expanded_run
```

重放时几何 SHA-256 与完整求解配置必须匹配。`current_run_solve_seconds=0` 明确表示本轮没有重新求解；基线原始 `solve_seconds` 仍保留为历史来源。400 与 4000 候选的运行分别保存，固定种子不意味着不同预算的数据互相独立。

代码复用边界：`search.py/safety.py` 为独立搜索与检查；`experiment.py` 复用历史单窗场景和 SC 适配接口。原 `sc_dynatogt/`、RotSync、插值 Sync 与 Time-Warp 源码不作修改。此方法只作收敛解邻域的拒绝采样，与 MSR 的多初值优化、时间修复和再优化不同。

实际结果见 [TEST_RESULTS.md](TEST_RESULTS.md)。
