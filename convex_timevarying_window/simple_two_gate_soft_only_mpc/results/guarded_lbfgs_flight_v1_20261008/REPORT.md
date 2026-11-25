# L-BFGS-B停滞修复：2.5Hz/3.5m自由点实飞

原初相、每组一次；只改数值求解器，不改MPC/规划目标/安全代价，不进行后验安全准入。

```json
{
  "outcome": "collision",
  "complete_time_s": null,
  "collision": true,
  "failure": "timeout",
  "body_exits": 0,
  "audit_error": null,
  "planning_calls": 118,
  "error_replans": 117,
  "planner_max_ms": 200.95593528822064,
  "planner_over_100ms": 33,
  "repaired_calls": 21,
  "first_planning_stats": {
    "iter_count": 1031,
    "evaluations": 1361,
    "converged": true,
    "return_status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
    "initial_objective": 183719211.220405,
    "final_objective": 6641.184239852082,
    "projected_gradient_inf": 7021.52761008386,
    "stagnation_repairs": [
      {
        "before": 183719211.220405,
        "after": 43942624.795929305,
        "alpha": 0.03125
      }
    ],
    "attempts": [
      {
        "status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
        "iterations": 1,
        "objective": 183719211.220405,
        "last_step": 0.0,
        "projected_gradient_inf": 1969117881.1962633,
        "stagnant": true
      },
      {
        "status": "CONVERGENCE: RELATIVE REDUCTION OF F <= FACTR*EPSMCH",
        "iterations": 1030,
        "objective": 6641.184239852082,
        "last_step": 0.00010111642952959077,
        "projected_gradient_inf": 7021.52761008386,
        "stagnant": false
      }
    ],
    "optimization_ms": 164.63190829381347
  },
  "mpc_max_ms": 56.11116485670209,
  "source_unchanged": true
}
```

完整时间包含七窗和返航。超时不等于碰撞；接触记录只涉及窗框。数值停滞修复不等于安全证明。
求解超过100ms如实记录，不能称已满足在线时限；此次实飞初态不保证与旧轨迹重放逐值一致。
