# 频率/幅度递增：冻结算法仿真实验

幅度为单侧；每条件各一次、原始相位。完整时间包含返回终点。
严格失败包括在线超时；物理/采样完成与在线能力分列，不把失败中止时刻当成绩。

|Hz|幅度 m|方法|严格完整时间 s|不含实时要求完整通过|退出窗数|规划最大 ms|MPC最大 ms|失败分类/原始原因|
|---:|---:|---|---:|---|---:|---:|---:|---|
| 1.0 | 2.0 | ours | 31.148000000000003 | True | 7 | 76.06 | 26.0 |  / None |
| 1.0 | 2.0 | center | 35.988 | True | 7 | 89.86 | 34.51 |  / None |
| 1.25 | 2.5 | ours | None | True | 7 | 127.11 | 28.09 | online_deadline_or_control / None |
| 1.25 | 2.5 | center | None | True | 7 | 118.81 | 28.82 | online_deadline_or_control / None |
| 1.125 | 2.25 | ours | 41.05200000000001 | True | 7 | 46.53 | 26.45 |  / None |
| 1.125 | 2.25 | center | None | False | 1 | 31.58 | 23.86 | physical_contact, sampled_safety, incomplete_gates, terminal_state / RuntimeError("RuntimeError('MPC QP primal infeasible; primal=18.52960850577094')") |
| 1.25 | 2.0 | ours | None | True | 7 | 134.78 | 28.73 | online_deadline_or_control / None |
| 1.25 | 2.0 | center | None | True | 7 | 105.1 | 24.89 | online_deadline_or_control / None |
| 1.125 | 2.0 | ours | 34.352000000000004 | True | 7 | 71.64 | 27.16 |  / None |

## 已观察到的粗档边界

```json
{
  "joint": {
    "ours": {
      "last_pass": [
        1.0,
        2.0
      ],
      "first_fail": [
        1.25,
        2.5
      ]
    },
    "center": {
      "last_pass": [
        1.0,
        2.0
      ],
      "first_fail": [
        1.25,
        2.5
      ]
    }
  },
  "frequency": {
    "ours": {
      "last_pass": [
        1.0,
        2.0
      ],
      "first_fail": [
        1.25,
        2.0
      ]
    },
    "center": {
      "last_pass": [
        1.0,
        2.0
      ],
      "first_fail": [
        1.25,
        2.0
      ]
    }
  }
}
```

中间档另见表；只有单初相单次结果，不假定难度单调，不推断全局不可行或随机成功率。
