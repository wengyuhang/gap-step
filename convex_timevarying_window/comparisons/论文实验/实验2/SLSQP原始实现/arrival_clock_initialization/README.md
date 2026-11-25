# 到达时刻坐标初值试验

使用七个按窗口周期归一化的绝对到达时刻偏移，加一个返航段log时长，作为8维CMA-ES搜索变量。

```text
tau_i = base_tau_i + period_i * z_i
DeltaT_1 = tau_1
DeltaT_i = tau_i - tau_(i-1)
DeltaT_return = base_return * exp(z_return)
```

改变某一中间窗到达偏移，只改变它前后相邻的两段时长，后续过窗时刻保持。所有时间候选仍通过原时间域和有限约束评价，MINCO根据实际窗口位置重算VAJ。

16次搜索、最多8进程同时运行，80代、每代24个体。前8个沿原始时间分布，后8个各段独立缩短后重算曲线。目标T/Tref+最大归一化违规。档案按窗口运动半周期组合保存，最终8条同时要求不同半周期组合及至少四分之一周期的连续到达差异。

## 实验结果

实验位于`../outputs/cmaes_arrival_clock_v1_20261113`。0.5/6为5/8合格，完整最短轨迹17.033371秒，生成4.062秒。0.75/6为0/8，生成4.063秒，保留全部恢复未收敛及预算终止记录。此坐标试验没有在两档统一改善结果。

初值仍由位置型最小snap生成VAJ。后续同点同时刻诊断表明，已合格轨迹重新使用位置型最小snap状态，会出现明显的采样执行器约束超限。VAJ可行性修复作为下一项待验证设计。

## 构建

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build build --target generate_arrival_clock -j 4
ctest --test-dir build -R arrival_clock_unit --output-on-failure
```

`src/generate_arrival_clock.cpp`为C++搜索器，复用相邻原模型与MINCO表达。`run.py`负责进程、筛选与原Python/SciPy SLSQP后端调用。所有结果为名义规划与数值几何评估。
