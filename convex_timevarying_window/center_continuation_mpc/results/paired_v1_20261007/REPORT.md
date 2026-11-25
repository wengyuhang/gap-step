# Center continuation: paired PX4/Gazebo results

Each condition once, phase zero; safety includes sampled truth and contacts, not continuous certification.
New center and free-prefix share all settings except free-point permission. Free has an extra 25ms search stage.
Historical center used a different two-gate/periodic/fixed-tail planner, so it is not a single-variable control.

|Hz / m|Historical center / s|New center / s|New free prefix / s|
|---|---:|---:|---:|
|1.0 / 2.0|24.428|38.804|29.388|
|0.7 / 2.0|27.640|26.408|21.796|
|1.0 / 1.2|24.432|26.38|24.296|

## All outcomes

- continuation_ours_f1_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=86.069/24.532ms; accepted/free-selected=3/2.
- continuation_center_f1_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=64.859/24.928ms; accepted/free-selected=4/0.
- continuation_center_f0.7_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=58.995/25.991ms; accepted/free-selected=5/0.
- continuation_ours_f0.7_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=83.448/27.899ms; accepted/free-selected=4/3.
- continuation_ours_f1_a1.2_p000: passed=True, exits=7, failure=None; planner/MPC max=86.667/32.705ms; accepted/free-selected=4/4.
- continuation_center_f1_a1.2_p000: passed=True, exits=7, failure=None; planner/MPC max=56.401/27.124ms; accepted/free-selected=4/0.

Exact base-policy value, robust safety and stable strict improvement are not established by this experiment.
