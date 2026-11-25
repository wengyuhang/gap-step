> 当前主实验已按用户新指令切换至TOGT七门布局、QuadA和论文安全余量，见[新协议](../togt_aligned/README.md)。下文保留2026-11-14较早的x500小窗协议；其中0.63094472米直接内缩开口的写法已查明重复计算0.085米框管半径，不作为当前配置。

# 当前主实验：四个四边形窗口与三个圆窗

2026-11-14起采用本协议。旧五边形、六边形赛道及原成绩保留为历史/附录材料，不再作为本协议的主实验。新场景和旧场景不能混在同一性能表中。

## 场景与尺寸

沿用高起伏七窗的中心位置、门平面朝向、穿越顺序、平移/摆动运动函数、起终点及十档幅频设置。只替换开口：第1、3、5、7窗为四边形中的方形，第2、4、6窗为圆形。

- 四个方门内边长依次为1.500、1.524、1.500、1.524米。
- 三个圆门内径均为1.524米；该圆门尺寸是与方门跨度匹配的研究设置，并非比赛官方圆门规格。
- 尺寸依据：[AI Grand Prix技术规格§3.7](https://www.theaigrandprix.com/wp-content/uploads/2026/05/260508_Technical_Spec_0002.pdf)内方门1500×1500毫米；[MultiGP官方5×5英尺开口](https://shop.multigp.com/product/standard-multigp-gate-5x5/)为1.524×1.524米。只引用开口尺寸，不宣称采用了两项比赛的完整门框、飞行器或竞赛规则。
- 物理门框沿用0.085米管半径与原解析边框模型；C++重新生成与新尺寸一致的实体三角网格，不复用旧大开口网格。
- 规划有限半径r=0.63094472米，最终原窗口复查半径0.62094472米，机体网格复查球半径0.3859447197米，均保持当前设置。

1.5米方门中，有限选点区域为每轴±0.11905528米；1.524米圆门中为半径0.13105528米的圆盘。这是当前安全半径和新开口尺寸共同决定的范围，不应再把旧大窗的选点自由度当作本轮自由度。

## 自由点与中心变量

每个自由穿越点直接保存门平面物理坐标q_i=(x_i,y_i)，单位米，世界坐标为c_i(t_i)+R_i[q_i,0]。求解器内部只做单位缩放q_i=diag(s_i)z_i，s为开口的两轴半尺寸；此变换是线性缩放，不是TOGT形状映射。

四边形物理开口表示为a_j^T q≤b_j，a_j为单位外法向。加入内缩半空间a_j^T q≤b_j−r。圆窗直接加入x²+y²≤(R−r)²。有限NLP为数值条件对这些表达式做固定正尺度归一化，数学可行域不变；另给z加内缩开口包围盒对应的上下界。圆盘的轴界不能替代圆盘约束，二者均保留。

不含d变量、角度/长度变量或gauge。中心版完全删除选点变量，q_i恒为零。

PVAJ MINCO使用MincoPV derivative_order=3；8段独立时间仍采用现有TOGT正映射。自由点轨迹变量8+14+63=85，中心版8+63=71。当前动力学软约束的1584个epigraph辅助变量保留，它们是软惩罚实现的一部分；本次删除的是冗余选点参数，未改变已确定的软动力学数学模型。

所有八组时间和实际VAJ逐值复制当前归档，不重新生成或借用另一方法的优化解。自由点物理位置初值全部为零，双方从相同中心轨迹开始。`--main-xy-eight`只求双方各八个独立初值，不额外释放中心最优解。

## 有限问题与补点的顺序

1. 用上一轮当前有限问题的解作为本轮原始变量初值；仅严格Solve_Succeeded状态的乘子允许按约束键热启动，新约束乘子为零。
2. 求解有限NLP，重新计算所有有限约束和变量界的最大违反，容差1e-5。该检查包括门内区域、已补窗框点、原有基础约束和全部动力学epigraph行。
3. 要求有限残差合格且优化器成功（Solve_Succeeded或原流程的Solved_To_Acceptable_Level）。否则记录有限问题未完成并停止该初值；不调用连续几何复查、不增加witness、不把同问题同输入再解一遍。Maximum_Iterations_Exceeded不进入补点。
4. 满足上述条件后才做原完整窗口复查。通过则立即接纳；未通过时，每窗最多补一个最严重的新witness。
5. 有新增约束才进入下一轮，继续使用刚得到的有限解。新约束使最优时间或目标值增加是正常现象，不按历轮目标下降要求回退。没有新增witness则停止并如实记录未找到。

`finite_incumbent`只保存所在约束修订集内的成功状态；每次约束修订后可以更新为更高目标的成功状态。它不是连续安全证书，也不用于强制覆盖下一轮初值。原有24轮外层、5000次有限优化上限保持；本次未增加数值恢复机制或新求解器。

除本协议明确的场景、选点表示和外层控制流外，时间/动力学权重1/1000、95%x500软动力学模型、原恢复流程的工作空间与0.3至15秒段时长约束、0.1米/秒法向速度基础约束均未改。它们保存在manifest与结果中，不能称为被本次取消。

## C++入口

```sh
# 原生构建方式见README.md；本例在主仓库执行。
BIN=/tmp/main_xy_build_20261114
SOURCE=convex_timevarying_window/comparisons/论文实验/实验2/cpp_runtime

# ORIGINAL_INPUTS须为当前PVAJ完整物理初值，而非旧的基线偏移坐标。
"$BIN/main_course_inputs" ORIGINAL_INPUTS ORIGINAL_ASSETS NEW_INPUT_DIRECTORY
"$BIN/planar_points_unit" NEW_INPUT_DIRECTORY/inputs/static_ours \
  NEW_INPUT_DIRECTORY/inputs/static_center NEW_INPUT_DIRECTORY/assets/static/scene.json TEST_OUTPUT.json

env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 SIP_DUMP_NLP=1 \
  "$BIN/matrix" "$BIN" NEW_INPUT_DIRECTORY/inputs NEW_INPUT_DIRECTORY/assets \
  NEW_RESULT_DIRECTORY 16 a75_f6 --main-xy-eight
```

场景/输入生成、MINCO、CasADi图、IPOPT、有限残差检查、补点、独立网格与候选调度均为C++。旧TOGT基线自身的作者形状映射和旧历史入口保留；它们不在本文主方法的显式xy分支中使用。十档新几何输入已生成，不表示十档规划或Gazebo已完成。
