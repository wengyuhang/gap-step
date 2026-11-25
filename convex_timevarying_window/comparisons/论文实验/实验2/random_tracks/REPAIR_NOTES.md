# 当前修复入口与验收范围

这次工作针对冻结的随机赛道求解，正式九赛道五方法矩阵仍暂停。没有改变赛道、碰撞判据、目标权重或安全半径，没有Gazebo实飞，也没有提交Git。

|文件|作用|
|---|---|
|repair.py|由布局固定穿越方向；按每条独立时序做同条件最小snap凸QP，恢复节点VAJ；正常有限问题继续用归档求解器，失败时接入有界恢复|
|finite_recovery.py|在同一有限问题上保留自身返回物理曲线，丢弃失败乘子，以冷精确Hessian恢复一次；仍失败则结束该尝试，避免外层无新约束反复求解|
|finite_direct.py / finite_direct_exact.py|相同目标和约束，以物理节点VAJ与相位缩放到达时刻为数值坐标；区别为L-BFGS或精确Hessian|
|validate_repair.py / repair_worker.py|明确给定冻结赛道后，独立生成8初值并验证自由点、中心；不调用其他方法结果作初值|
|sampled_safe_model.py / safe_model.py / sampled_cycle_worker.py|高难开发入口，以自身初值实际机体/平面重叠区间的有限点启动，再用原完整复查补点；全部结果另作独立STL和穿窗任务检查|
|backtracked_phase_run.py|高难周期赛道的有界开发搜索：保持窗相位、整周期提前，到达点与PVAJ重新优化；它依赖周期运动，不是非周期场景的通用保证|
|replay_finite.py|重建产生重复失败的完全相同有限问题，独立比较两种Hessian求解；用于诊断，不是正式基线|
|summarize_repair.py|仅整理已结束的64条低中难验证；不把高难继续优化混入冷初值结果|

所有脚本由bootstrap.py载入已逐值复现的归档实现。冻结工作副本是`/tmp/first_formal_soft_reproduction_code_20261112`；长期归档来源是实验2/IPOPT软约束_py版第一个正式结果/code，原源文件SHA见comparison_v2批协议。这里没有再复制整个项目，也没有声称完成新版全C++原生MINCO迁移。

## 已有证据

- results/repair_verified_v2_20261113/REPORT.md：第一布局低中难、另两布局中难，自由点和中心各8个独立初值，共64条全部通过完整数值几何复查与独立任务检查，161次有限调用全正常收敛。同一条件/方法的初值间时间差最大3.033%。
- results/repair_phase_eight_v2_20261113：高难8个不同原始初值均有合格结果，但有较差局部解；7搜索结束，最后一条因重复同一失败有限问题停止。历史源快照SHA与原协议吻合，原结果未覆盖。
- results/finite_recovery_development_v1_20261113/REPORT.md：同一2368行有限问题，冷L-BFGS1978次/58.321秒恢复失败；冷精确Hessian75次/3.420秒正常收敛，另有13.908秒建图。正常几何通过不能替代优化收敛。
- results/finite_recovery_full_validation_v1_20261113：原始初值2的自身继续优化，34.109秒，完整求解29.778秒，4轮有限问题正常收敛并通过独立任务检查。
- results/finite_recovery_regression_v1_20261113：正常自由点和中心路径各一次真实回归；两条竞速时间与修复前逐值相同，全部调用是primary，没有触发恢复。
- results/repair_exact_phase_continuation_v1_20261113：原始初值2的后续有界周期继续优化。记录额外成本和失败，不能当作新冷初值或覆盖旧八初值表。

“合格”在这里是指定名义模型下完整窗框浮点数值复查及任务验收通过；动力学仍为原软约束，不代表连续硬动力学证书或实飞结果。

尚未解决通用额外穿窗事件缺少距离witness时的处理，也未证明所有非凸初值能得到同样好的解。恢复五方法比较前须统一固定方向协议（包括Wang尾法向），并按同版本独立重验，不能只修本文的验收器。
