# 实验产物清理记录（2026-11-25）

完成：删除 216,803 个文件，磁盘实际释放约 **25.77 GiB**；逐项核对 247,697 个保留文件，内容变化 **0**，删除失败 **0**。

分类：671 个旧 ULog、6,936 个旧接触记录、133 份快照环境中的 205,352 个文件、3,844 个有完全一致保留副本的计算图。1,322 个计算图继续保留。

按用户确认计划，仅清理 comparisons 内旧实飞日志、接触原始记录、源码快照内重复虚拟环境及有字节一致保留副本的旧 CasADi 图。

最新完整 PVAJ、仅位置消融、最新 TOGT/Wang/修正版 CPC 与六方法汇总全部保留。源码、场景、配置、初值、最终轨迹、结果 JSON 和报告均保留。未知恢复来源的唯一计算图未删除。

删除逐文件依据及 SHA-256：DELETE_MANIFEST.json；实际删除日志：DELETED.jsonl；被保留文件的事前 SHA-256：RETAINED_BEFORE.jsonl；版本清单：ENVIRONMENTS.json；核验结果与释放量：SUMMARY.json。

## 历史原始证据可用性

下列旧实验的 ULog 或接触记录已删除，保留的碰撞/失败/成功结论未改，但不能再从完整原始日志复核这些旧实飞。历史日志清单中对这些文件的哈希或路径不再代表文件仍可用。

- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923`：ULog 25，接触记录 175。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/frozen_20260919`：ULog 16，接触记录 112。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006`：ULog 38，接触记录 266。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/frozen_20261006`：ULog 12，接触记录 84。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006`：ULog 33，接触记录 231。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/comprehensive_wang/results/full_v1_20261007`：ULog 27，接触记录 189。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/comprehensive_wang/results/phase_zero_v1_20261007`：ULog 27，接触记录 189。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/direct_ctbr_center_motion/results/direct_ctbr_20260924`：ULog 25，接触记录 175。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/fixed_plane_wang/results/frozen_20261007`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/fixed_plane_wang/results/pilot_20261007`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/large_window_sweep/results/scale3_paired_v2_20261008`：ULog 4，接触记录 28。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/frozen_20260918`：ULog 0，接触记录 84。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/gp_ctbr_calibration_h20`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/gp_ctbr_calibration_h40`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/gp_frozen_20260918`：ULog 0，接触记录 49。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/gp_motor_calibration_noise05`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/gp_motor_mavlink_calibration_noise05`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/gp_motor_radps_calibration_noise3`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/motion_rate_racing/results/local_sweep_frozen_20260918`：ULog 0，接触记录 84。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_checked/results/original_checked_v1_20261007`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_crossing_resolved/results/v1_20261007`：ULog 3，接触记录 21。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline/results/deadline_v1_20261007`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/results/no_postcheck_v1_20261007`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/gui_demo_20261007_01`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/gui_demo_20261007_02`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/gui_demo_20261007_03`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/gui_demo_20261007_04`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/soft_v1_20261007`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/original_fixed_plane_ours/results/frozen_v1_20261007`：ULog 6，接触记录 42。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923`：ULog 25，接触记录 175。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/gui_wang2023_r10_t4_a1_p0_20260923`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/gui_wang2023_r10_t4_a1_p1_20260923`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919`：ULog 19，接触记录 133。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/v3_px4_frozen_20260919_01`：ULog 1，接触记录 7。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/v3_px4_frozen_20260919_02`：ULog 16，接触记录 112。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/simple_difficulty_sweep/results/progressive_v1_20261008`：ULog 8，接触记录 56。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/simple_difficulty_sweep/results/simulation_v2_20261008`：ULog 4，接触记录 28。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_constant_velocity/results/dev_static_20261012`：ULog 1，接触记录 1。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_constant_velocity/results/paired_completion_v1_20261012`：ULog 4，接触记录 4。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_constant_velocity/results/paired_v1_20261012`：ULog 14，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_linear_approach/results/offline_approach80_v2_20261013`：ULog 8，接触记录 16。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_linear_approach/results/offline_approach_v1_20261013`：ULog 4，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_linear_approach/results/paired_v1_20261013`：ULog 10，接触记录 22。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/dev_static_pair_v2_20261012`：ULog 2，接触记录 4。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/extension_free_boundary_v1_20261013`：ULog 14，接触记录 28。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/extension_unlimited_v1_20261013`：ULog 4，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/paired_final_20261012`：ULog 35，接触记录 70。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/paired_shooting_v1_20261013`：ULog 38，接触记录 76。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/paired_v1_20261012`：ULog 7，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/single_gate_swing/results/paired_v2_20261012`：ULog 0，接触记录 2。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/spaced_large_window_sweep/results/spaced3_paired_v1_20261008`：ULog 2，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007`：ULog 38，接触记录 266。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/center_initialized_flight_v1_20261105`：ULog 7，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/multistart_flight_v1_20261105`：ULog 7，接触记录 14。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/multistart_full_v1_20261105`：ULog 30，接触记录 60。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/pendulum_dynamic_interface_v1_20261105`：ULog 1，接触记录 2。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/pendulum_final_v2_20261105`：ULog 3，接触记录 6。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/pendulum_first_contact_resolution_v1_20261105`：ULog 1，接触记录 2。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/pendulum_v1_20261105`：ULog 9，接触记录 18。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/preflight_pendulum_v2_20261105`：ULog 3，接触记录 6。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/preflight_pendulum_v3_20261105`：ULog 3，接触记录 6。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/preflight_translation_v1_20261105`：ULog 4，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/resolution_continuation_v1_20261105`：ULog 2，接触记录 4。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/resolution_v1_20261105`：ULog 3，接触记录 6。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验1/results/translation_v1_20261105`：ULog 34，接触记录 68。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f3_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f3_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f3_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f3_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f3_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f6_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f6_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f6_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f6_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a25_f6_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f1_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f1_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f1_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f1_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f1_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f3_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f3_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f3_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f3_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f3_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f6_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f6_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f6_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f6_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a50_f6_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f1_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f1_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f1_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f1_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f1_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f3_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f3_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f3_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f3_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f3_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f6_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f6_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f6_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f6_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/a75_f6_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/mixed_base_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/mixed_base_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/mixed_base_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/mixed_base_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/mixed_base_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/static_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/static_dynamic_togt/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/static_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/static_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_10x5_full_v1_20261109/flights/static_wang2023/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_incomplete_completion_v1_20261109/flights/a50_f1_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_incomplete_completion_v1_20261109/flights/a50_f6_dynamic_togt_center/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/IPOPT软约束_py版第一个正式结果/records/higher_swing_incomplete_completion_v1_20261109/flights/mixed_base_ours/attempt_1`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/basic_group_f2_a2_v1_20261106`：ULog 4，接触记录 32。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/basic_group_full_v1_20261106`：ULog 7，接触记录 56。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/compact_layout_px4_v1_20261106`：ULog 1，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/compact_layout_px4_v2_20261106`：ULog 3，接触记录 24。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/compact_layout_px4_v3_20261106`：ULog 5，接触记录 40。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/continued_free_a75_f6_gazebo_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/dynamic_togt_margin90_a25_f6_v1_20261108`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/first_formal_soft_full_reproduction_v1_20261112`：ULog 0，接触记录 456。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/free_24_parallel_best_gazebo_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/free_refined_a75_f6_gazebo_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_a50_f6_ours_live_view_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_a75_f6_ours_view_v1_20261107`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107`：ULog 0，接触记录 392。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_10x5_full_v1_20261109`：ULog 0，接触记录 400。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_a75_f6_view_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_a75_f6_view_v2_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_a75_f6_view_v3_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_incomplete_completion_v1_20261109`：ULog 0，接触记录 24。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/independent_best_center_a25_f6_gazebo_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/interlaced_layout_px4_v1_20261106`：ULog 2，接触记录 16。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/interlaced_layout_px4_v2_20261106`：ULog 2，接触记录 16。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/interlaced_layout_px4_v4_20261106`：ULog 7，接触记录 56。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/ipopt_factorized_three_gazebo_v1_20261108`：ULog 0，接触记录 24。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/latest_soft_margin90_full_matrix_v1_20261108`：ULog 0，接触记录 328。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/margin95_inset_free_best_gazebo_v1_20261109`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/matched95_inset8_full_matrix_v1_20261109`：ULog 0，接触记录 320。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/mixed_global_four_v2_20261105`：ULog 1，接触记录 20。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/mixed_global_four_v3_20261105`：ULog 6，接触记录 120。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/mixed_global_four_v4_20261105`：ULog 21，接触记录 420。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/mixed_global_v1_20261105`：ULog 3，接触记录 24。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/native_cpp_10x5_v1_20261110`：ULog 1，接触记录 72。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/original_constraints_x500_margin90_three_v1_20261108`：ULog 0，接触记录 24。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/original_constraints_x500_three_v1_20261108`：ULog 0，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/preflight_dynamic_togt_static_flight_v1_20261105`：ULog 1，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/preflight_global_static_center_flight_v1_20261105`：ULog 1，接触记录 8。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/radius090_a25_f6_v1_20261107`：ULog 0，接触记录 16。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/soft_dynamics_paired_v1_20261107`：ULog 0，接触记录 168。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/static_layout_px4_v2_20261106`：ULog 6，接触记录 48。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/tracking_schedule_repair_v1_20261112`：ULog 0，接触记录 88。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/x500_matched_three_v1_20261108`：ULog 0，接触记录 16。
- `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验3/results/single_gate_stress_v1_20261110`：ULog 0，接触记录 30。

## 计算图恢复

每个被清理图的清单 evidence.retained_copy 指向未删除、SHA-256 完全一致的副本；复制回原路径即可恢复，不需要求解。恢复依赖应遵循各实验原配置。虚拟环境根据 ENVIRONMENTS.json 重建，不承诺二进制逐字节相同。
