# 取消名义轨迹准入检查：0.5步长递增实验

新版本全部重飞，原七窗/初相/同一MPC/PX4，完整时间含返航。每条件每方一次，不推断成功率。
仅物理接触判断碰撞；名义残差、耗时、球包络不否决成绩。未完成不是完成时间。

|频率 Hz|幅度 m|自由完整时间 s|自由碰撞|自由结果|中心完整时间 s|中心碰撞|中心结果|
|---:|---:|---:|---|---|---:|---|---|
| 1.0 | 2.0 | 34.452 | False | complete | 32.824 | False | complete |
| 1.5 | 2.5 | None | True | collision | 32.936 | False | complete |
| 2.0 | 3.0 | None | True | collision | None | True | collision |
| 1.5 | 2.0 | 31.204 | False | complete | None | True | collision |
| 2.0 | 2.0 | None | True | collision | None | True | collision |
| 1.0 | 2.5 | 30.323999999999998 | False | complete | None | True | collision |
| 1.0 | 3.0 | None | True | collision | None | True | collision |

## 已观察到的失败档位

```json
{
  "joint": {
    "center": {
      "frequency": 2.0,
      "amplitude": 3.0,
      "outcome": "collision"
    },
    "ours": {
      "frequency": 1.5,
      "amplitude": 2.5,
      "outcome": "collision"
    }
  },
  "frequency": {
    "center": {
      "frequency": 1.5,
      "amplitude": 2.0,
      "outcome": "collision"
    },
    "ours": {
      "frequency": 2.0,
      "amplitude": 2.0,
      "outcome": "collision"
    }
  },
  "amplitude": {
    "center": {
      "frequency": 1.0,
      "amplitude": 2.5,
      "outcome": "collision"
    },
    "ours": {
      "frequency": 1.0,
      "amplitude": 3.0,
      "outcome": "collision"
    }
  }
}
```

档位顺序不证明物理难度单调；高档成功和失败样本全部保留。
