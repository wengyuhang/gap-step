# 按真实仿真完成与碰撞判定的递增实验

用户更新口径：不以100ms、名义动力学/包络超限否决。算法未改；时间包含七窗后返回终点。
各条件各一次，不能称随机成功率或全局临界值。

|频率 Hz|单侧幅度 m|方法|完整无碰撞时间 s|碰撞|结果|已退出窗数|
|---:|---:|---|---:|---|---|---:|
| 1.0 | 2.0 | ours | 31.148000000000003 | False | complete | 7 |
| 1.0 | 2.0 | center | 35.988 | False | complete | 7 |
| 1.25 | 2.5 | ours | 30.456000000000003 | False | complete | 7 |
| 1.25 | 2.5 | center | 36.596000000000004 | False | complete | 7 |
| 1.125 | 2.25 | ours | 41.05200000000001 | False | complete | 7 |
| 1.125 | 2.25 | center | None | True | collision | 1 |
| 1.25 | 2.0 | ours | 32.972 | False | complete | 7 |
| 1.25 | 2.0 | center | 33.632 | False | complete | 7 |
| 1.125 | 2.0 | ours | 34.352000000000004 | False | complete | 7 |
| 1.125 | 2.0 | center | 32.352 | False | complete | 7 |
| 1.5 | 3.0 | ours | None | False | incomplete | 3 |
| 1.5 | 3.0 | center | None | True | collision | 3 |
| 1.0625 | 2.125 | ours | 29.479999999999997 | False | complete | 7 |

## 当前经验边界（含已测中间档）

```json
{}
```

更高档可能重新成功，不能外推为“超过此档必失败”。超时等原审计仍保留在summary.json。
