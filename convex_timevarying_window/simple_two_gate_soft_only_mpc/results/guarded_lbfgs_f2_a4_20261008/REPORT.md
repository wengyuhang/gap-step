# L-BFGS-B停滞修复：2Hz/4m自由点实飞

原初相、每组一次；只改数值求解器，不改MPC/规划目标/安全代价，不进行后验安全准入。

```json
{
  "outcome": "collision",
  "complete_time_s": null,
  "collision": true,
  "failure": "RuntimeError(\"ValueError('nonfinite MPC model/reference')\")",
  "body_exits": 4,
  "audit_error": "ValueError('Groundtruth does not cover whole flight')",
  "planning_calls": 19,
  "error_replans": 15,
  "planner_max_ms": 162.1992508880794,
  "planner_over_100ms": 2,
  "repaired_calls": 0,
  "first_planning_stats": {
    "iter_count": 788,
    "evaluations": 1021,
    "converged": true,
    "return_status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
    "initial_objective": 144416667.60790026,
    "final_objective": 18239.39908863983,
    "projected_gradient_inf": 18090.62067218288,
    "stagnation_repairs": [],
    "attempts": [
      {
        "status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
        "iterations": 788,
        "objective": 18239.39908863983,
        "last_step": 0.001312090542724853,
        "projected_gradient_inf": 18090.62067218288,
        "stagnant": false
      }
    ],
    "optimization_ms": 120.94782385975122
  },
  "mpc_max_ms": 31.762578058987856,
  "source_unchanged": true
}
```

完整时间包含七窗和返航。超时不等于碰撞；接触记录只涉及窗框。数值停滞修复不等于安全证明。
求解超过100ms如实记录，不能称已满足在线时限；此次实飞初态不保证与旧轨迹重放逐值一致。
