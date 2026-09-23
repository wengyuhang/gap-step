# 两篇论文baseline对比：执行入口

基线1：IROS 2023 Polynomial-Based Online Planning for Autonomous Drone Racing in Dynamic Environments。
实现：`复现/polynomial_dynamic_racing_2023/`，MINCO中心硬约束与L-BFGS，非主方法改名。

基线2：2026预印本Bridging Performance and Generalization in Reinforcement Learning for Agile Flight。
实现：`复现/agile_generalist_rl_2026/`，非视觉40维状态PPO、自适应任务切换和样条赛道。

两者均为有披露差异的独立实现，不是作者官方代码。协议见[PROTOCOL.md](PROTOCOL.md)。

## 运行队列

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/jack/anaconda3/envs/wyh/bin/python -u \
  -m convex_timevarying_window.comparisons.paper_baselines_2026.workflow \
  --resume 复现/agile_generalist_rl_2026/runs/static_pilot_25m_20260919/final.pt \
  --outdir convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919
```

顺序：多项式16条件扫描 → 学习方法继续至5亿训练步 → 学习方法静态PX4验收 → 动态测试。
不同时运行正式计时飞行与训练。模型和训练日志保存在复现目录，跨方法汇总保存在本目录results。
`status.json`记录父/子进程PID、当前阶段和进程结果，各阶段独立日志；每次飞行完整保存源码快照/哈希。
运行期间不要修改当前阶段冻结的Python源码；程序会检查源文件变化而中止，不混用版本。

学习基线当前pilot只在自定义近似动力学训练，且原论文任务没有动态门/返航停车，尚未通过本场景迁移。
未通过静态验收时动态测试仅为诊断，不进入宣称论文优劣的正式排行。
主方法引用已有冻结16条件，包含1次缺真值日志，不将后来的补测替换进去。

## 已运行的开发结果（不是正式16条件汇总）

| 实现 | 测试 | 结果 |
|---|---|---|
| 多项式 | 直线7窗理想跟踪 | 8.86 s成功，10 ms采样净空0.513 m |
| 多项式 | 动态七窗，原幅度/频率，相位0.4，PX4 | 25.116 s成功，零接触，真值采样净空0.179 m，最大周期68.028 ms |
| 多项式 | 静态七窗同布局，PX4 | 过4门后碰撞，超时，越界；失败 |
| 状态RL | 2500万步最终模型，开发样条12任务 | 2/12完成；开发最优模型4/12，不是正式测试集 |
| 状态RL | 开发最优模型静态七窗PX4迁移 | 未过门，1.980 s越界，最大完整周期0.883 ms；失败 |

零碰撞但没完赛不算成功；规划软约束收敛、采样通过和连续安全证明分开表述。
