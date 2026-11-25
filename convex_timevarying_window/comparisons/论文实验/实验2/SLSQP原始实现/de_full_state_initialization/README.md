# 完整时间与 VAJ 的 DE 初值生成

2026-11-13。独立新入口，历史 CMA-ES 及结果保持。

搜索实际状态 `[logT(8),V(21),A(21),J(21)]` 共71维。位置取相应到达时刻的动态窗口中心，起终 PVAJ 固定。每次评价直接调用原生 `MincoPV(..., derivative_order=3).fit(P, VAJ, T)`，实际 VAJ 不重新覆盖为位置型最小 snap，不使用基线加偏移。

原八初值提供初始时间和 VAJ，其余种群在归一化坐标加扰动。V/A/J 的坐标尺度5/10/20仅影响搜索条件化，不是物理上限。时间与状态初始扰动标准差0.12/0.08。DE/rand/1/bin，F每代在0.5至1采样，CR=0.9，整代并行评价后更新。每线程独立 CasADi/MincoPV 实例。ε初值为种群最大违规的中位数，至80%代数平方下降至原采样容差。两候选在ε内比较时间，否则优先满足ε，均不满足时比较违规。

沿用冻结原始 SLSQP 导出的 CasADi 约束及 exp(logT) 时间映射，包括该历史模型原有段时间范围；本次没有改变后端模型或改接其他时间映射。生成器为C++，约束图来自既有导出文件，后端仍为用户选择的原始 Python/SciPy SLSQP。未实施局部精修。最终从已接受候选档案中按采样可行优先、可行中时间优先、否则违规优先选8条；按完整归一化状态去重。该版本尚未额外实施到达时刻分簇。不得把离散合格称为连续安全。

## 构建和使用

从本目录执行

```sh
cmake -S . -B /tmp/full_state_de_build -DCASADI_ROOT=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
cmake --build /tmp/full_state_de_build -j2
# MODEL为包含model.json及constraints.casadi的既有模型目录
# 下面为使用示例，未执行正式80代实验
/tmp/full_state_de_build/generate_de MODEL result.json 80 64 8 20261113 0
# 仅检查63个实际导数，不运行DE
/tmp/full_state_de_build/generate_de MODEL check.json 0 8 1 20261113 1
# 导出双方八条manifest，保留原位置初值，原样传递时间和实际VAJ
/tmp/full_state_de_build/generate_de --export result.json CASES a75_f6 OUTPUT_CASES
```

程序拒绝覆盖结果。墙钟包含加载各线程模型与63分量回归检查；evaluations字段统计DE种群评价，此外回归检查另有64次模型评价。端点检查没有完整连续安全结论。

## 已做验证

仅0.5/6的导数检查与0.75/6的16个体、2代接口测试。63个独立导数输入验证通过；单线程/8线程候选与档案逐值一致；最终8条分别导入自由点及中心后端，16项系数/约束一致性检查均通过，最大系数差约2.71e-10。见validation/CHECKS.json。未执行正式SLSQP优化、整批实验或Gazebo；尚无性能改进结论。
