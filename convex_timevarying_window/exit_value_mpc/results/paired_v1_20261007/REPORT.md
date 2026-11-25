# Exit-value proxy: paired PX4/Gazebo results

Each condition once, phase zero; safety includes sampled truth and contacts, not continuous certification.
New center and free-prefix share all settings except free-point permission. Free has an extra 25ms search stage.
Historical center used a different objective/periodic/fixed-tail planner, so it is not a single-variable control.

|Hz / m|Historical center / s|New center / s|New free prefix / s|
|---|---:|---:|---:|
|1.0 / 2.0|24.428|32.828|26.432000000000002|
|0.7 / 2.0|27.640|None|None|
|1.0 / 1.2|24.432|28.352|27.38|

## All outcomes

- exit_value_ours_f1_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=73.962/30.524ms; accepted/free-selected=8/4.
- exit_value_center_f1_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=49.690/29.125ms; accepted/free-selected=8/0.
- exit_value_center_f0.7_a2_p000: passed=False, exits=5, failure=RuntimeError('missed_current_opening'); planner/MPC max=52.150/24.391ms; accepted/free-selected=6/0.
- exit_value_ours_f0.7_a2_p000: passed=False, exits=4, failure=RuntimeError("RuntimeError('MPC QP primal infeasible; primal=0.06799552846909587')"); planner/MPC max=81.611/34.251ms; accepted/free-selected=6/6.
- exit_value_ours_f1_a1.2_p000: passed=True, exits=7, failure=None; planner/MPC max=72.467/29.904ms; accepted/free-selected=8/7.
- exit_value_center_f1_a1.2_p000: passed=True, exits=7, failure=None; planner/MPC max=47.916/31.437ms; accepted/free-selected=8/0.

Exact base-policy value, robust safety and stable strict improvement are not established by this experiment.
