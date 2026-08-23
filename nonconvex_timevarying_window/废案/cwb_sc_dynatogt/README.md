# CWB-SC-DynaTOGT：连续整机安全 SC-DynaTOGT

本目录是 `nonconvex_timevarying_window/` 下的独立方法。它依据
`SC_DynaTOGT_WholeBody_Continuous_Safety_Codex_Plan.md` 实现，不修改
`sc_dynatogt/`、`wb_sc_dynatogt/`、`msr_dynatogt/` 或 `atlas_dynatogt/`。

与已有 `WBSC-DynaTOGT` 的区别是：本方法严格保留原决策变量

\[
x=[K,D],
\]

并保持 constant yaw。roll/pitch 每次都由当前 degree-7 MINCO 轨迹通过微分平坦性恢复。
本方法检查的也不只是规划穿越时刻，而是包含该时刻的完整“机体—窗口平面”相交连通区间。

## 调用链

```text
[K,D] -> SC 穿越点 -> MINCO -> constant-yaw flatness attitude
      -> 8 cuboid vertices -> planned crossing component
      -> complete plane/cuboid section edges
      -> adaptive (time, edge-lambda) SC-radius verification
      -> unsafe witnesses -> active penalties -> warm-start re-optimization
```

固定时刻的平面坐标为

\[
\xi^3_{ij}(t)=n_i(t)^T[p(t)+R_B(t)b_j-c_i(t)].
\]

截面/接触判据严格实现为 `min(xi3) <= eps and max(xi3) >= -eps`，没有使用多输入 XOR。
正式检查区间是包含规划时刻 `t_i` 的相交集合连通分量，其他窗口无限延伸平面造成的相交不会混入。

每条截面边都以 `lambda in [0,1]` 检查，安全量只有

\[
m(q)=r^2-\|\Psi^{-1}(q)\|^2.
\]

质心安全或仅截面顶点安全都不足以判定整机安全。

## 当前安全声明

当前完成的是计划中的 V1：局部运动量和逆映射 Lipschitz 常数来自自适应数值估计。
成功状态只能是 `NUMERICALLY_VERIFIED` / `SAFE_NUMERICAL`，代码不会输出
`CERTIFIED` / `SAFE_CERTIFIED`。严格 Bernstein/区间算术 V2 尚未实现。

窗口外 witness 使用安全圆周上的离散辅助 SC 原像池产生恢复方向；窗口内 witness 使用直接
SC 径向平方铰链。活动安全项仅对有限 witness 求梯度，完整自适应验证器不参与反向传播。
V1 的活动项采用 float64 中心差分；这是相对于原计划 PyTorch 反向模式的已记录实现差异。

## 运行

```bash
pytest -q nonconvex_timevarying_window/cwb_sc_dynatogt/tests
python -m nonconvex_timevarying_window.cwb_sc_dynatogt.experiments \
  --suite smoke \
  --outdir nonconvex_timevarying_window/cwb_sc_dynatogt/results/smoke
```

烟雾实验写出 `summary.json` 和 `crossing_diagnostics.png`。优化器收敛状态与整机安全状态分开记录；
达到优化或外层预算但仍有明确 witness 时，结果必须是 `UNSAFE`，不会包装成成功。

详细公式、模块对应和限制见 [ALGORITHM.md](ALGORITHM.md)，测试记录见
[TEST_RESULTS.md](TEST_RESULTS.md)。

## 文件结构

```text
body_model.py                 长方体顶点与 12 条边
gate_frame.py                 不修改基类的完整窗口坐标系适配器
plane_section.py              xi3、截面、正式穿越区间、拓扑分段
sc_inverse.py                 带回溯且保持单位圆盘内的 Newton 逆映射
whole_body_safety.py          V1 时间×截面边自适应验证和 witness
constraint_generation.py     活动安全目标与温启动外层循环
whole_body_visualization.py   xi3 曲线和最危险截面诊断图
experiments.py                独立 smoke CLI
tests/                        新方法测试
```
