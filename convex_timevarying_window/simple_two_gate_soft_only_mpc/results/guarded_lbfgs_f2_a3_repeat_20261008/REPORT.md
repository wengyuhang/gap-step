# L-BFGS-B停滞修复：2Hz/3m自由点实飞

原初相、每组一次；只改数值求解器，不改MPC/规划目标/安全代价，不进行后验安全准入。

```json
{
  "outcome": "complete",
  "complete_time_s": 35.732,
  "collision": false,
  "failure": null,
  "body_exits": 7,
  "audit_error": null,
  "planning_calls": 7,
  "error_replans": 0,
  "planner_max_ms": 171.3150511495769,
  "planner_over_100ms": 1,
  "repaired_calls": 0,
  "first_planning_stats": {
    "iter_count": 144,
    "evaluations": 194,
    "converged": true,
    "return_status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
    "initial_objective": 104522578.74236152,
    "final_objective": 1717.125076966736,
    "projected_gradient_inf": 3751.3671624548733,
    "stagnation_repairs": [],
    "attempts": [
      {
        "status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
        "iterations": 144,
        "objective": 1717.125076966736,
        "last_step": 4.437049999972409e-05,
        "projected_gradient_inf": 3751.3671624548733,
        "stagnant": false
      }
    ],
    "optimization_ms": 23.297778330743313
  },
  "mpc_max_ms": 25.25727590546012,
  "source_unchanged": true
}
```

完整时间包含七窗和返航。超时不等于碰撞；接触记录只涉及窗框。数值停滞修复不等于安全证明。
求解超过100ms如实记录，不能称已满足在线时限；此次实飞初态不保证与旧轨迹重放逐值一致。
