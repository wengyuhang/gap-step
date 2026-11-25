# CMA-ES 八初值生成

为当前七窗全局规划生成八条初始轨迹。C++ 使用 libcmaes 搜索八个独立段时间，查询给定时刻的窗口中心，原 TOGT MincoSnap 生成连续曲线及实际 VAJ。窗口点初值取中心，正式自由点优化从第一次迭代起开放窗口内位置。

## 当前共享候选池方案

`run_pool.py`运行16次独立CMA-ES搜索，最多8个进程同时执行，两轮完成。每次80代、每代24个体，实际共30736次候选评价。每个个体包含八段独立log时间，通过exp映射生成正时间，原SLSQP段时域随原模型评价。

每次搜索保留每代优胜者及原始初值，共1296条档案轨迹。所有搜索的档案合并，先按有限检查点可行性排名，可行候选按完整七窗返航时间排名，未满足的按最大归一化违规及均方违规排名。采用实际到达时刻向量去除邻近候选，优先保留至少四分之一局部运动周期差异的方案。数量不足时保留其余不同的到达时刻，最终输出八条。每次搜索可贡献任意数量的最终初值。

`INITIALS.json`保存最终八条轨迹、独立搜索来源、各段时间、七窗到达时刻、实际VAJ、分段系数和有限约束质量。`cases/inputs/*/manifest.json`为原SLSQP自由/中心入口可直接读取的八初值格式。

## 当前验证

优先0.5/6及0.75/6。每档先生成共用八初值，随后运行原自由点SLSQP八路验证。后端代码和原300秒/初值预算、17节点硬动力学、90%推力比例、安全余量及最终几何检查保持。阶段结果、失败和超时完整记录。

本轮正式目录为`../outputs/cmaes_pool_eight_v3_20261113`。早期v1按档案最短筛选集中于同次搜索附近，v2曾采用每次搜索一个固定名额。用户明确只限制最终数量后，v3使用共享候选池并缩短搜索，早期开发记录保留。

初值生成冷启动时间包含原模型导出与搜索/筛选/写出，复用模型的运行时间单列。原模型导出只负责将冻结CasADi约束原样输出为值函数，生成搜索和MINCO计算全部使用C++。后端继续调用用户要求恢复的原Python/SciPy SLSQP。初始有限检查与最终浮点几何复查按各自证据记录。

## 构建和运行

在本目录执行。外部CasADi SDK和Eigen沿用wyh环境，原生MINCO与JSON头通过相邻SLSQP C++包的相对路径复用。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build build --target generate_initials -j 4
ctest --test-dir build --output-on-failure
```

原模型导出与两档实验入口支持`--help`。`run_pool.py`接收当前冻结输入、编译后二进制、导出模型和新结果目录。一次完整运行示例可参考正式批次的RUN.sh。

## 依赖与论文

libcmaes固定提交`6a53cd562c85dc6df3b8df317be3ecb415240c2d`，源码和LGPL许可保存在`vendor/libcmaes.tar.gz`，构建时解压。MINCO、原动力学和几何模型来源通过批次SHA记录。

低维随机搜索与解析轨迹构造借鉴VP-STO的结构，本方案搜索独立段时间并使用当前动态窗口、MINCO和原约束模型。[VP-STO，ICRA 2023](https://arxiv.org/html/2210.04067)、[libcmaes](https://github.com/CMA-ES/libcmaes)

## 本轮结果

每档8初值已生成并完成原SLSQP自由点后端验证。生成运行时间4.324/4.322秒，含首次导出9.271/9.199秒。0.5/6最终5/8、0.75/6最终7/8合格，最短完整轨迹23.374/30.997秒。轨迹时间仍较长。

完整报告与8初值文件见[正式结果](../outputs/cmaes_pool_eight_v3_20261113/run/REPORT.md)。
