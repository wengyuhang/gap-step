# 更保守软净空：无事后整段检查的配对消融

2026-10-07，用户要求软约束更保守。本轮保持上一轮no-postcheck架构，只统一修改两项：
soft_clearance_buffer由0.15m增为0.25m，soft_clearance_weight由1增为10。
原硬余量0.35m不变，偏好球/框净空从0.50m变为0.60m。
不根据窗口编号、具体碰撞点、频率或幅度调参。原版源码和历史成绩不覆盖。

软罚为 w*[max(0,(p²-d²)/(p²-h²))]^3，其中h=机体半径+框厚+0.35m，p=h+buffer。
扩大buffer会改变归一化，不能只假定buffer大则任何距离罚得更重。
本配置满足 w_new/(p_new²-h²)^3 >= w_old/(p_old²-h²)^3 且p_new>p_old，
故对所有d²>=0，新代价与其对距离减小的梯度强度均不低于原版；测试同时核对解析条件和网格。
这只是代价偏好，不是连续安全保证，固定采样之间和跟踪误差仍可能造成失败。

复用原no-postcheck规划进程和PX4入口，仅注入配置；MPC、55ms求解预算、20ms共同窗时限、
原事件/残差检查、补点、可停留尾段完全不改。求解后几何/动力学复查保持关闭。
4ms真值、接触、整机退出、单次正向/返航/工作区和<100ms仍是事后验收，离线复查不进入控制。

原七窗固定平面、中心面内平移，两档(0.6Hz,1.2m,0相)/(1Hz,2m,0相)，各首次一次，
60s飞行上限，失败不重试；对照no_postcheck_v1_20261007，世界/场景/MPC/PX4一致，
规划配置仅上述两字段不同。飞前冻结源与协议，飞行期间不修改，负结果完整保留。
只做这一组预声明参数，不在看到结果后选择性报告；两次不是统计成功率估计。

运行：
```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline_soft_conservative.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/soft_v1_20261007
```
