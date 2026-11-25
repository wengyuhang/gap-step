# 作者C++核的动态TOGT完整实现

2026-11-12，按用户要求改写动态TOGT自由点与严格中心。当前规划入口为cpp_runtime/src/dynamic_togt_planner.cpp，数学核在include/dynamic_togt.hpp。静态复现作者源码只读链接，未修改。

## 复用与动态适配

直接编译静态凸窗口/复现/TOGT-Planner-ICRA2024中的MincoSnap、TrajSolver::forwardT/backwardT/backPropagateT、addEnergyCost/addPenaltyCost/addTimeCost、QuadManifold、Ball、Polyhedron和原lbfgs_optimize。运行时只读场景JSON、车辆配置JSON和作者YAML，不依赖CasADi或Python导出的函数。ldd仅标准C++/C运行库。

P_i=c_i(t_i)+R_i q_i，t_i为之前各段T累加。本场景固定平面，有水平平移和绕外支点的正弦角单摆。优化中每次重新查询运动。MINCO回传的点梯度与窗口中心速度点积，沿累计到达时刻链式回传至所有前序段时间。R固定，当前模型不包含平面旋转。

自由点保留25个局部形状变量，圆形通过原Ball公式在局部平面嵌入第三分量0，多边形直接调用原Polyhedron。均在当前0.62594472米内缩安全域内。严格中心完全删除空间变量，仅8个时间变量，每次在当前到达时刻查询中心。两者内部VAJ由原位置型MINCO消元。

## 参数和单初值

采用静态复现实际使用的parameters/cpc/init/cpc_planning.yaml及cpc_lbfgs.yaml。时间权重1，能量0，位置/速度/角速度/旋转/推力各1，平滑0.01，固定16区间梯形积分。最大倾角参数沿原6.28 rad，大于姿态夹角最大pi，当前模型中该项不激活。L-BFGS memory256、past3、代价相对容差1e-5、梯度阈值0、最大迭代0、线搜索64，均读原配置。未保留此前手写的全局1000倍率或SciPy L-BFGS-B。

物理参数采用当前x500质量、惯量、混控以及95%单桨/角速度限值，独立保存于cpp_runtime/config/x500_togt.json。速度60 m/s、边界[-10000,10000]及z下界0.3、最小推力/旋转阈值等项目按原init配置，其中桨推力上下界覆盖为当前x500的0至8.121151 N。当前物理适配与作者0.85 kg QuadA实例不同。

每个方法仅一次冷初始化。初始时长为t=0参考门点间距离/speedGuess1，内缩多边形点取原形状顶点均值，圆心初值0，使用原形状反映射。随后按累加初始时刻查询真实动态点。当前初值构造约5至12微秒；初始化与完整优化分别计时。未运行先求另一条中心优化轨迹的准备流程。

## 验证

静态运动置零后，直接与作者TrajSolver::costFunction及系数比较。四个非零罚项测试每版分别通过，最大系数相对误差1.53e-15，目标误差0，梯度相对误差3.69e-14。动态目标对所有时间/点变量中心差分核对，自由最大1.42e-5、中心2.99e-7，覆盖有效点梯度334的激活情况；中心点查询误差1.72e-15，平移/摆动速度差分2.02e-9。原共享内缩函数提取后，本文MincoPV原有系数/梯度与软目标回归仍通过，见shared_inset_regression.json。CTest dynamic_togt_unit通过。

## 单初值试运行

| 场景 | 方法 | 构造及规划时间ms | 候选全轨迹时间s | 作者优化器停止 | 独立几何结果 |
|---|---|---:|---:|---|---|
| static | dynamic_togt | 15.568 | 19.354995 | past相对代价停止 | 无名义碰撞且任务通过 |
| static | dynamic_togt_center | 8.429 | 20.419223 | past相对代价停止 | 无名义碰撞且任务通过 |
| a25_f6 | dynamic_togt | 19.539 | 115.228678 | past相对代价停止 | 名义碰撞及门序未通过 |
| a25_f6 | dynamic_togt_center | 4.985 | 119.338529 | past相对代价停止 | 名义碰撞及门序未通过 |

以上时间均含完整七窗和返航。两条动态候选115至119秒很长，按作者past3停止时梯度仍较大，保留该结果。此处优化器返回原正状态只表示达到作者代价停滞判据。它不等同于安全穿窗或KKT梯度收敛，动态两条存在第1/2/4/5窗框相交和重复/反向穿越。基线仍保留原目标与停止设置，独立几何复核只分类。没有新增本文安全补点或输出过滤。全部原始结果与失败分类存档，没有实飞。

## 调用与集成

```sh
cmake --build /tmp/window_cpp_runtime_build --target dynamic_togt_planner -j2
dynamic_togt_planner SCENE_JSON OUTPUT_DIRECTORY free
dynamic_togt_planner SCENE_JSON OUTPUT_DIRECTORY center
# 可选显式指定作者参数目录和车辆JSON
dynamic_togt_planner SCENE_JSON OUTPUT_DIRECTORY free AUTHOR_PARAMETER_DIRECTORY VEHICLE_JSON
```

当前C++ matrix的两项动态TOGT改调该原生入口，每方法每赛道仅1次，其他方法原候选数保留。已有多初值动态TOGT记录的目录不能作为新协议输出。旧togt_planner与图导出工具保留历史实验兼容用途。结果字段durations、coefficients、variables和config.radius支持既有参考轨迹读取与执行链。此次未启动完整十档矩阵或Gazebo。

PROTOCOL保存来源配置和源码SHA，运行后哈希核验通过。source_snapshot、dynamic_togt_planner_frozen、VALIDATION、SUMMARY和各结果目录支持复查。
