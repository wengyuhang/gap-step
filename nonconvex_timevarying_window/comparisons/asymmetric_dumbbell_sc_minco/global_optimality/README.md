# 单自旋非凸窗：离线最优性实验

2026-10-30。冻结原任务：3 Hz / 2 m 平移、2圈/秒法向自旋，原12边开口、起终点和零PVAJ，原两段七次PV-MINCO。

目标是寻找更短的可行名义轨迹，同时给出有效的全局下界；只有上下界差达到阈值才称 epsilon 全局最优。
不运行Gazebo，不修改原生产规划器，不把多初值最好结果称为全局最优。

上界使用物理窗口坐标替代SC坐标，表示同一内点集合。左右室只是搜索覆盖，窄道宽0.4m，
小于球+框+余量直径，故合法平面穿越点不在窄道。每室多时间/速度初值，SLSQP硬约束，
增加动力学峰值见证和原局部几何复查；安全距离、推力和角速度物理界不放宽。
解析两段Hermite消元与原16阶线性MINCO系统逐项比较。

下界使用SCIP空间分支定界，对原问题的有限约束松弛求界。删掉的约束只放宽可行域；
保留的运动、轨迹及力约束用显式表达式表示。报告具体松弛范围、求解状态和最优性差距。
全局下界不需要松弛问题的候选通过原验收；上界必须满足目标问题的约束。
当前认证范围和未认证部分必须在原始结果和报告中分别记录。

求解依赖PySCIPOpt安装到独立 `/tmp/codex_single_gate_global_deps`，不修改wyh环境。
新结果目录拒绝覆盖历史结果。所有耗时属于离线求解，不宣称100ms在线达标。

## 本次结果

正式结果：[global_spin2_final_20261030/REPORT.md](../results/global_spin2_final_20261030/REPORT.md)。
两批共74个初始化尝试，最好名义2.947577223s，比旧SLSQP参考短2.297%。
已经SC逆映射回原1155行NLP与QR轨迹重建；逆映射误差2.48e-16m、系数差1.60e-11。
原浮点几何包络通过，固定保存轨迹整段名义动力学128位Arb复核通过（22.067s）。
全局求解尚未成功：SCIP消元模型存在数值异常，Bernstein未消元松弛180s超时。
未获得可信非平凡下界，不称全局最优或epsilon最优；未飞，不把名义时间当闭环成绩。

## 运行入口

在主仓库根目录、wyh环境、单线程BLAS下运行（所有outdir必须为新目录）：

```bash
python -m nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.global_optimality.upper --outdir <new-directory> --restore --time-count 23 --max-starts 47
python -m nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.global_optimality.continuous_dynamics --plan <plan.npz> --output <new-audit.json>
python -m nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.global_optimality.bernstein_bound --outdir <new-directory> --upper-plan <plan.npz>
```

`finalize`针对本次已冻结的两批搜索，回原SC/QR验证并生成报告，不是任意轨迹的通用汇总器。
早期模型的原始result存在未剔除的solver标志，须结合追加的`VALIDATION_REJECTED.json`阅读，不能单读raw dual。
源码已对数值LP/未有界辅助量警告采取保守排除。SCIP当前接口仍为研究诊断，不能用于发放安全/最优证书。
