# 时间与约束违规双目标初值

面向短竞速时间生成八条MINCO/PVAJ初值。使用C++ CMA-ES搜索八个独立段时间，窗口点初值取实际到达时刻的中心，原TOGT MincoSnap生成实际VAJ。原SLSQP后端及最终检查继续通过冻结入口调用。

## 搜索和选择

定义T为完整七窗返航时间，V为原有限约束的最大归一化违规，Tref取原八个初值总时长的中位数。搜索评分为

```text
J_lambda = T / Tref + lambda * V
```

16次独立搜索使用lambda=1、2、4、8，最多8个进程同时运行，80代、每代24个体，每档共30736次候选评价。时间和违规同时参与每一轮CMA-ES更新。

每次搜索保留已评价候选的时间—违规Pareto前沿。全局共享池再去掉受支配点，使用1至8之间对数均匀的八个权重方向选取八条，按实际到达时刻去重。一场独立搜索可贡献多条最终初值。八条均保留源搜索编号及权重，便于追溯。

## 结果

正式结果位于`../outputs/cmaes_time_violation_pareto_v1_20261113/run`。0.5/6和0.75/6各输出8条初值，原SLSQP八路最终均8/8合格，最短完整轨迹分别为18.799948和17.072015秒。初值搜索/筛选/写出约4.084/4.079秒，首次原模型导出计入约9.031/8.957秒。完整报告、初始违规与各轨迹在结果目录。

初值允许有限节点违规。0.5/6的最大归一化违规范围0至0.08639，0.75/6为0.04594至0.20991。后端以原硬约束及原窗口复查判定最终候选。结果为名义规划与几何评估。

## 源码

- `src/generate_pareto.cpp`实现多权重CMA-ES、非支配候选保存。
- `src/initial_model.hpp`复用前一版初值模型的完整数值表达，包含原MINCO构造及冻结CasADi有限约束评价。
- `run.py`执行16次搜索、全局Pareto选择和最终八路原SLSQP验证。
- 相邻`heuristic_initialization`提供固定版本libcmaes源码、构建配置、模型导出及调度工具，原文件保持。
- 最终后端为相邻`worker.py`调用`code/`内冻结Python/SciPy SLSQP。初值搜索与MINCO构造使用C++，原模型导出和实验编排使用Python。

## 构建

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build build --target generate_pareto -j 4
ctest --test-dir build -R pareto_unit --output-on-failure
```

实验入口参数通过`run.py --help`查询。当前两档复现命令记录在正式批次`RUN.sh`，必须指定新的输出目录。
