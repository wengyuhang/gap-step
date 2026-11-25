> 原High-MPC输出适配排错记录，三条作者基线一致排除最终比较。见[排除原因](AUTHOR_V1_EXCLUDED.md)与[最终结果](../pendulum_final_v2_20261105/REPORT.md)。

# 实验1B：按桨径匹配的摆动圆门

门名义直径1.978800727m；作者阻尼单摆模型；x500/PX4物理仿真。
High-MPC采用作者缓存问题与原权重，输入界匹配x500；未重新训练，属于明确标注的零样本适配。
每档每方法一次。时间包括从任务开始到共同终点并减速；碰撞/未完成无有效竞速时间。

|初始角度°|方法|结果|完整时间s|碰框/碰地|最大规划ms|最大控制ms|
|---:|---|---|---:|---|---:|---:|
| 0.0 | ours | 无碰撞完赛 | 1.729 | False/False | 19.17 | 35.56 |
| 0.0 | center | 无碰撞完赛 | 2.081 | False/False | 16.93 | 40.7 |
| 0.0 | high_mpc_author | 无碰撞完赛 | 3.385 | False/False | 0 | 40.87 |
| 60.0 | ours | 无碰撞完赛 | 2.111 | False/False | 32.56 | 33.45 |
| 60.0 | center | 无碰撞完赛 | 2.286 | False/False | 23.89 | 36.1 |
| 60.0 | high_mpc_author | 未完成 | — | False/False | 0 | 43.29 |
| 120.0 | ours | 无碰撞完赛 | 2.352 | False/False | 64.64 | 34.19 |
| 120.0 | center | 无碰撞完赛 | 2.690 | False/False | 34.63 | 39.64 |
| 120.0 | high_mpc_author | 未完成 | — | False/False | 0 | 50.0 |

原始ULog、接触发布端、接触记录、已知运动表、实际穿越审计与源快照分别保留。
圆门和x500/PX4属于场景适配；门/整机外廓比例未核实，不能把按桨径匹配写成完全几何相似。

## 计算耗时与共同完赛配对

```json
{
  "formal_flights": 9,
  "repetitions_per_condition": 1,
  "random_success_probability_estimated": false,
  "methods": {
    "ours": {
      "completed_no_collision": 3,
      "collisions": 0,
      "incomplete": 0,
      "planner": {
        "count": 3,
        "p50_ms": 32.55510702729225,
        "p95_ms": 61.434968444518745,
        "max_ms": 64.64384193532169,
        "over_100ms": 0
      },
      "controller": {
        "count": 202,
        "p50_ms": 28.606010018847883,
        "p95_ms": 32.60630205622874,
        "max_ms": 35.5555210262537,
        "over_100ms": 0
      }
    },
    "center": {
      "completed_no_collision": 3,
      "collisions": 0,
      "incomplete": 0,
      "planner": {
        "count": 3,
        "p50_ms": 23.89467100147158,
        "p95_ms": 33.556930627673864,
        "max_ms": 34.63051503058523,
        "over_100ms": 0
      },
      "controller": {
        "count": 223,
        "p50_ms": 29.228411964140832,
        "p95_ms": 34.57631948404014,
        "max_ms": 40.704801911488175,
        "over_100ms": 0
      }
    },
    "high_mpc_author": {
      "completed_no_collision": 1,
      "collisions": 0,
      "incomplete": 2,
      "planner": null,
      "controller": {
        "count": 146,
        "p50_ms": 15.921104000881314,
        "p95_ms": 23.885321686975658,
        "max_ms": 49.99726603273302,
        "over_100ms": 0
      }
    }
  },
  "paired_free_vs_center": [
    {
      "angle_deg": 0.0,
      "free_s": 1.7289999999999992,
      "center_s": 2.081000000000003,
      "saving_percent": 16.91494473810684
    },
    {
      "angle_deg": 60.0,
      "free_s": 2.110999999999997,
      "center_s": 2.2860000000000014,
      "saving_percent": 7.655293088364137
    },
    {
      "angle_deg": 120.0,
      "free_s": 2.3519999999999968,
      "center_s": 2.6899999999999977,
      "saving_percent": 12.565055762081832
    }
  ]
}
```

High-MPC原高层权重未针对x500重训；此组外部结果属于明确标注的适配，不能称作者原平台性能。
