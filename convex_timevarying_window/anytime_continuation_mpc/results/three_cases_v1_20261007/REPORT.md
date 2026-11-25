# Anytime center continuation: three frozen PX4 flights

Each condition once. Historical center and prior continuation are separate algorithms, not same-code ablations.
Unsafe/incomplete flights have no valid race time. Sampled geometry and contacts are not continuous certificates.

|Hz / m|Historical center / s|Prior free-prefix / s|Anytime free-prefix / s|
|---|---:|---:|---:|
|1.0 / 2.0|24.428|29.388|22.76|
|0.7 / 2.0|27.64|21.796|23.264|
|1.0 / 1.2|24.432|24.296|23.196|

## All outcomes

- anytime_ours_f1_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=81.968/28.501ms.
- anytime_ours_f0.7_a2_p000: passed=True, exits=7, failure=None; planner/MPC max=86.235/28.138ms.
- anytime_ours_f1_a1.2_p000: passed=True, exits=7, failure=None; planner/MPC max=85.019/35.669ms.
