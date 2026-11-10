# 基础组完整实验

用户明确要求TOGT式而非外围环线；当前使用[TOGT官方交错/叠窗布局适配](togt_course/README.md)。separated_mixed_v1已停止8条并保留，不能当用户认可的赛道或继续运行。

当前转为[重新设计的独立穿窗区间](separated_course/README.md)：四平移三摆动，W3/W6交替升档，先几何和参考核验后40条纯轨迹对比。旧紧凑28项已24项停止，全部结果保留，不续跑或混用。

当前用户已改为不启动Gazebo的[纯轨迹评估](trajectory_only/README.md)，同一基础布局先按2m振幅、0.5Hz递增至3Hz。历史实飞记录保留，以下为已结束的实飞协议与结果，不继续飞行。

已完成12/12，见[结果主表](RESULTS.md)：5无碰撞完赛、4碰撞、3未找到。原始ULog/接触与全部冻结源复核通过，协调器已退出，实验世界暂停。

仅当前紧凑original布局，静态、1Hz/2m、3Hz/4m，各一次初相零；本文自由/中心、动态TOGT自由/中心，共12项。全部一次全局规划七窗并返航，无滚动视域。

场景数值查询、符号优化、局部运动包络与Gazebo插件运动轴统一；不旋转旧开口。8个新增测试覆盖静态世界逐字节一致、水平/摆动运动/速度/范围、TOGT梯度、PVAJ点以及缺证据报错。
高动态无解锁位姿预检已通过，七窗各250样本的查询/真实位置最大误差约6.7e-13m。静态TOGT原始16.446/18.076s已核对场景、世界、控制器、旧源差异与证据哈希，不重复飞。

[冻结协议](PROTOCOL.md)。输出目录`../results/basic_group_full_v1_20261106`；主表REPORT.md，逐条summary.json，进度status.json，最终独立复核validation.json。
新协调器使用`python -m convex_timevarying_window.comparisons.论文实验.实验2.basic_group.run --outdir <新目录> --reuse <compact_layout_px4_v3目录> --scene-probe <basic_group_scene_probe_v1目录>`。
仅使用experiment容器；未起跑设施错误最多一次重启，实际执行过不重飞，证据异常停止。所有未找到规划/碰撞/未完成保留。
