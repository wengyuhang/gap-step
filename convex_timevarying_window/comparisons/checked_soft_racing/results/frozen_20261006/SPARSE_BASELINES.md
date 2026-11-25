# 稀疏 CTBR 开发变体：不合并挑选最佳

只列与新 38 条件精确匹配的结果。未知表示缺验收文件，不解释为通过。GUI/开发调参差异见原目录。

| 旧变体 | r/t/a/phase | 旧主验收 | 旧有效圈时 s | 新有效圈时 s | 旧失败 |
|---|---|---|---:|---:|---|
| baseline_target_v7_px4_r1_t1_p0_20260926 | (1.0, 1.0, 1.0, 0.0) | True | 31.376 | 26.600 | None |
| capture_reward_px4_r10_p0_dev02_20260924 | (10.0, 1.0, 1.0, 0.0) | None | — | 31.460 | None |
| capture_reward_px4_r10_p0_dev03_20260924 | (10.0, 1.0, 1.0, 0.0) | None | — | 31.460 | RuntimeError('fresh_state_CTBR_not_admitted') |
| capture_reward_px4_r10_p0_dev04_20260924 | (10.0, 1.0, 1.0, 0.0) | None | — | 31.460 | None |
| capture_reward_px4_r1_p0_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| capture_reward_px4_r1_p0_dev02_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | None |
| capture_reward_px4_r1_p0_dev03_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | None |
| corner_seed_px4_r1_t1_p0_a6p5_20260926 | (1.0, 1.0, 1.0, 0.0) | True | 32.204 | 26.600 | None |
| corner_seed_px4_r1_t1_p0_v1_20260926 | (1.0, 1.0, 1.0, 0.0) | False | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| coupled_px4_r1_t1_p0_v1_20260926 | (1.0, 1.0, 1.0, 0.0) | False | — | 26.600 | RuntimeError('observed_interval_clearance') |
| coupled_px4_r1_t1_p0_v2_20260926 | (1.0, 1.0, 1.0, 0.0) | False | — | 26.600 | timeout |
| coupled_px4_r1_t1_p0_v3_20260926 | (1.0, 1.0, 1.0, 0.0) | True | 35.588 | 26.600 | None |
| coupled_px4_r1_t1_p0_v4_20260926 | (1.0, 1.0, 1.0, 0.0) | True | 37.076 | 26.600 | None |
| coupled_px4_r1_t1_p0_v5_20260926 | (1.0, 1.0, 1.0, 0.0) | True | 34.464 | 26.600 | None |
| coupled_px4_r1_t1_p0_v6_20260926 | (1.0, 1.0, 1.0, 0.0) | False | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| flow_smooth_px4_gui_r10_t4_p120_v2_20260926 | (10.0, 4.0, 1.0, 2.094395102) | True | 34.076 | — | None |
| flow_smooth_px4_gui_r10_t4_p120_v2_repeat_20260926 | (10.0, 4.0, 1.0, 2.094395102) | False | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| flow_smooth_px4_gui_r10_t4_p120_v4_20260926 | (10.0, 4.0, 1.0, 2.094395102) | False | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| flow_smooth_px4_gui_r10_t4_p120_v5_20260926 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('no_bootstrap_CTBR') |
| flow_smooth_px4_gui_r10_t4_p120_v5_retry_20260926 | (10.0, 4.0, 1.0, 2.094395102) | False | — | — | timeout |
| flow_smooth_px4_gui_r10_t4_p120_v6_20260926 | (10.0, 4.0, 1.0, 2.094395102) | False | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| flow_smooth_px4_gui_r10_t4_p120_v7_20260926 | (10.0, 4.0, 1.0, 2.094395102) | True | 33.912 | — | None |
| flow_smooth_px4_gui_r10_t4_p120_v7_repeat_20260926 | (10.0, 4.0, 1.0, 2.094395102) | True | 37.644 | — | None |
| matched_point_px4_r1_t1_p0_v1_20260926 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('arm failed') |
| matched_point_px4_r1_t1_p0_v1_retry_20260926 | (1.0, 1.0, 1.0, 0.0) | False | — | 26.600 | None |
| next_gate_time_px4_r1_t1_p0_v1_20260926 | (1.0, 1.0, 1.0, 0.0) | False | — | 26.600 | timeout |
| prepare_capture_px4_dev01_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| prepare_capture_px4_dev02_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | None |
| prepare_capture_px4_r10_p0_dev01_20260924 | (10.0, 1.0, 1.0, 0.0) | None | — | 31.460 | RuntimeError('no_bootstrap_CTBR') |
| prepare_capture_px4_r10_p0_dev02_20260924 | (10.0, 1.0, 1.0, 0.0) | None | — | 31.460 | RuntimeError('fresh_state_CTBR_not_admitted') |
| prepare_capture_px4_r10_p0_dev03_20260924 | (10.0, 1.0, 1.0, 0.0) | None | — | 31.460 | RuntimeError('fresh_state_CTBR_not_admitted') |
| prepare_capture_px4_r1_p120_dev01_20260924 | (1.0, 1.0, 1.0, 2.094395102) | None | — | 26.292 | None |
| prepare_capture_v2_px4_r1_p0_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | None |
| prepare_capture_v2_px4_r1_p120_20260924 | (1.0, 1.0, 1.0, 2.094395102) | None | — | 26.292 | RuntimeError('arm failed') |
| prepare_capture_v2_px4_r1_p120_retry_20260924 | (1.0, 1.0, 1.0, 2.094395102) | None | — | 26.292 | None |
| px4_dev01_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| px4_dev02_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| px4_dev03_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| px4_dev04_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError("RuntimeError('current_CTBR_filter_rejected_4')") |
| px4_dev05_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('observed_interval_clearance') |
| px4_dev06_20260924 | (1.0, 1.0, 1.0, 0.0) | None | — | 26.600 | RuntimeError('fresh_state_CTBR_not_admitted') |
| reference_handoff_px4_r1_t1_p0_v1_20260926 | (1.0, 1.0, 1.0, 0.0) | True | 32.148 | 26.600 | None |
| safety_filter_r10_t4_p120_gui_demo_20260924 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | None |
| safety_filter_r10_t4_p120_px4_20260924_v1 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| safety_filter_r10_t4_p120_px4_20260924_v2 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| safety_filter_r10_t4_p120_px4_20260924_v3 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError("RuntimeError('current_CTBR_filter_rejected_4')") |
| safety_filter_r10_t4_p120_px4_20260924_v4 | (10.0, 4.0, 1.0, 2.094395102) | True | 36.912 | — | None |
| slew_heading_px4_gui_r10_t4_p120_20260924 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| slew_heading_px4_gui_r10_t4_p120_v2_20260924 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('arm failed') |
| slew_heading_px4_gui_r10_t4_p120_v2_retry_20260924 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('fresh_state_CTBR_not_admitted') |
| slew_heading_px4_gui_r10_t4_p120_v3_20260924 | (10.0, 4.0, 1.0, 2.094395102) | None | — | — | RuntimeError('arm failed') |
| slew_heading_px4_gui_r10_t4_p120_v3_retry_20260924 | (10.0, 4.0, 1.0, 2.094395102) | True | 40.992 | — | None |
| tangent_yaw_px4_gui_r10_t4_p120_20260926 | (10.0, 4.0, 1.0, 2.094395102) | True | 37.800 | — | None |
