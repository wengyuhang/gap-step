# 短总时长分层初值

先规定几个较短的初始总时长，在每个时长档内搜索低约束违规的分段时间分配，再选择不同到达节奏的八条MINCO/PVAJ初值。

## 初值参数化

Tref为原八个解析初值总时长的中位数。四档初始总时长为Tref乘以0.65、0.70、0.75、0.80，每档4次独立CMA-ES搜索。各次搜索使用7个无约束变量，通过稳定softmax分配8段时间。

```text
DeltaT_i = lower_i + (S - sum(lower)) * softmax(theta_1,...,theta_7,0)_i
```

S为本档初始总时长，lower来自原SLSQP段时间下界。分段总和恒为S，时间为正。窗口位置取实际到达时刻的中心，VAJ由原位置型MINCO求得。每轮搜索目标为最小化原有限约束的最大归一化违规。

## 保留八条

每档保存每代优胜者，从该档4次搜索的共享档案选择两条低违规且到达时刻不同的轨迹。四档共8条。每次搜索可以贡献多条，保留不同运动节奏的候选，原始(T,V)全局支配关系不作为删除分支的标准。

固定S仅用于初值生成。正式原SLSQP后端继续联合优化段时间、自由穿越点和完整VAJ，以完整七窗返航时间择优，保留原物理模型及验收。

## 本轮设置

16次搜索，最多8进程同时运行，80代、每代24个体，共30736次候选评价。7维分配的初始sigma为0.25。四档比例与sigma为本轮初值生成参数，由相同规则用于两档场景。原已优化轨迹没有用于生成初值。

新C++生成器调用`pareto_initialization/src/initial_model.hpp`中的原MINCO和冻结CasADi有限约束模型，CMA-ES依赖通过相邻`heuristic_initialization`构建配置复用。Python负责调度、候选筛选和调用原Python/SciPy SLSQP。

## 构建

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build build --target generate_stratified -j 4
ctest --test-dir build -R fixed_total_mapping --output-on-failure
```

`run.py --help`列出实验参数。当前实验位于`../outputs/cmaes_short_time_strata_v1_20261113`，输入和结果按条件保存。
