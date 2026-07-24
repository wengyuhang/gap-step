# 闭环连续形变非凸时变窗口研究

本目录专门研究：在仿真中，四旋翼按指定顺序各穿越一次未来动态完全已知的连续形变非凸窗口，窗口安全区可暂时完全消失，随后恢复到初始完整状态，并尽量缩短闭环总飞行时间。

与仓库中已有项目的关系：

- 本目录不是 `gap_step/` 二维 PPO 主线的一部分；
- 本目录比 `nonconvex_timevarying_window/` 的既有定义多出闭环全状态返回、局部非刚性连续形变和完整未来动态条件；
- 通用问题定义固定保存在 [PROBLEM_DEFINITION.md](PROBLEM_DEFINITION.md)；
- 每种算法必须放在本目录下独立的同级子目录中。

## 已有方法

| 方法 | 状态 | 核心思路 |
|---|---|---|
| [FAPP-PPO](fapp_ppo/README.md) | 首个可运行实现 | 未来感知特权预览、非对称 actor–critic、残差 CTBR PPO、短暂可通行机会课程 |

FAPP-PPO 现已包含窗口安全区暂时完全消失的时间关键模式，以及可执行的[完整学术实验方案](fapp_ppo/ACADEMIC_EXPERIMENTS.md)。

## 目录结构

```text
closed_loop_deformable_window/
  PROBLEM_DEFINITION.md
  README.md
  fapp_ppo/
    README.md
    ALGORITHM.md
    PAPER_NOTES.md
    configs/
    tests/
    ...
  <next_algorithm>/
```

FAPP-PPO 的实验结果和检查点只写入自己的子目录，不与其他研究线混用。
