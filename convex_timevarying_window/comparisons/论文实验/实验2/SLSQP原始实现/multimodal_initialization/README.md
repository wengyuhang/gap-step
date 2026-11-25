# 自由段时间与运动分支初值

C++ CMA-ES搜索八段独立log时间。前8次搜索使用原独立样本，后8次各段独立缩短后重算MINCO。目标为完整时间/Tref加最大归一化违规。按七窗所处运动半周期组合保留各分支候选，以时间—违规的不同折中方向选8条。

16次搜索、最多8并行、80代24个体，最终名额可来自同一次搜索。后端为原完整PVAJ硬约束SLSQP。原物理模型、动力学节点和窗口复查沿用。

实验结果位于`../outputs/cmaes_free_time_branches_v1_20261113`。0.5/6为7/8合格、最短17.158450秒；0.75/6为8/8、最短17.036137秒。生成分别4.268/4.064秒。离散半周期组合不同仍可能落在近似相同的连续时序附近，多条候选最终得到相近轨迹，完整原始记录保留。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build build --target generate_multimodal -j 4
ctest --test-dir build -R multimodal_unit --output-on-failure
```
