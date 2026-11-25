# L-BFGS-B停滞修复：2Hz/3.5m自由点实飞

原初相、每组一次；只改数值求解器，不改MPC/规划目标/安全代价，不进行后验安全准入。

```json
{
  "outcome": "collision",
  "complete_time_s": null,
  "collision": true,
  "failure": "timeout",
  "body_exits": 1,
  "audit_error": "ValueError('Groundtruth does not cover whole flight')",
  "planning_calls": 130,
  "error_replans": 128,
  "planner_max_ms": 112.09117528051138,
  "planner_over_100ms": 1,
  "repaired_calls": 32,
  "first_planning_stats": {
    "iter_count": 733,
    "evaluations": 952,
    "converged": true,
    "return_status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
    "initial_objective": 116819793.24880998,
    "final_objective": 8033.957897305527,
    "projected_gradient_inf": 875.4861443894915,
    "stagnation_repairs": [],
    "attempts": [
      {
        "status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
        "iterations": 733,
        "objective": 8033.957897305527,
        "last_step": 0.00025831867522646546,
        "projected_gradient_inf": 875.4861443894915,
        "stagnant": false
      }
    ],
    "optimization_ms": 109.67044671997428
  },
  "mpc_max_ms": 39.14785524830222,
  "source_unchanged": true
}
```

完整时间包含七窗和返航。超时不等于碰撞；接触记录只涉及窗框。数值停滞修复不等于安全证明。
求解超过100ms如实记录，不能称已满足在线时限；此次实飞初态不保证与旧轨迹重放逐值一致。
