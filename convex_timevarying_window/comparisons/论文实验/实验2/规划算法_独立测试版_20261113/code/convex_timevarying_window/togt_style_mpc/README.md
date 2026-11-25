# TOGT 实飞结构的独立 NMPC 跟踪器

## 2026-10-13 当前单窗默认：单重射击 MPC

新增shooting_controller.py，使用同一33状态PX4 PID/分配/电机模型及对齐Session。
1s视界、25×40ms节点，直接优化100个CTBR输入；所有预测状态由非线性积分得到。
L-BFGS-B解析梯度、30次迭代上限、60ms在线预算，执行本次最佳有限输入可行候选并记录是否收敛。
保留位置/速度/姿态/角速度/桨力跟踪项，增加归一化输入偏差平方项（固定权重1）。
仅约束实际CTBR输入箱；不再要求未来实际机体角速度严格低于指令限幅，角速度仍参与跟踪。
没有控制变化率罚、穿窗避障项、参考降速、时间重参数化或恢复路径。
指令接收预测和历史按全局4ms PID节拍量化，适配1ms物理步长。

当前comparisons/single_gate_swing默认使用该MPC；旧aligned_controller.py的RTI保留作历史对照。
七窗global_gate_minco_probe.flight与single_gate_stop_racing.flight可显式--tracker single-shooting；
本轮未用新MPC实飞七窗，也未重新运行自由/中心完整矩阵。

五条原失败参考同系数/时刻/世界/PX4参数验证均无碰撞完赛：
2m/.5Hz自由3.357s、中心3.463s；2m/1.5Hz自由3.575s；2m/2.5Hz自由4.043s；4m/4Hz自由6.039s。
最大真值误差0.084–0.258m，MPC最大41.63ms。20项相关回归通过。
仅修节拍、完整SQP、RTI输入正则的失败结果保留，不能把修复归因于节拍单项。
原规划节点间动力学超限仍存在，本轮支持跟踪改善，不构成连续闭环安全证明。

报告与误差图：[同参考跟踪修复](results/tracking_repair_20261013/REPORT.md)。

```bash
python -m convex_timevarying_window.togt_style_mpc.frozen_single_gate \
 --baseline-dir convex_timevarying_window/comparisons/single_gate_swing/results/paired_final_20261012/a2_f2.5_ours \
 --outdir <new-directory>
```

以下为保留的旧RTI/TOGT式实现和历史证据。

## 2026-10-12 执行链对齐：同一90%参考已完成七窗返航

用户授权修复模型/执行时序，并重飞相同轨迹。新增显式 `--tracker execution-aligned`，
保留原13状态 `togt-style` 入口供对照；不是偷偷替换其历史记录，也不是作者Agilicious实现。

### 改动限于跟踪执行链

- `aligned_controller.py`：仍用1s/25×40ms acados SQP-RTI，但状态扩为物理/电机/PID/滤波记忆33维，
  输入直接是实际发送的 `[总推力, p_cmd, q_cmd, r_cmd]`。预测每个节点走10次真实4ms数字内环，
  包含归一化分配、去饱和和电机升降速；不再由理想四桨解提取40ms未来角速度。
- 位置/速度/姿态/角速度权重2000/30/20/.3不变；.02单桨力参考代价现在作用于预测电机状态产生的桨力，
  而非不存在的直接电机输入。没有新增平滑罚、倾角罚、避障、轨迹降速或重规划。
  控制量和节点角速度保持原物理范围；目标函数结构因执行输入改变有上述明确适配，并非数学上完全同一个OCP。
- 修正共享 `px4_model.py` 的D项：40Hz低通→求导→20Hz低通；历史数值模型结果不覆盖。
- `aligned_session.py` 请求带采样时间的HIGHRES_IMU角速度（估计偏置已扣除的区间平均陀螺，非Gazebo真值），
  对齐位置时刻；电机状态由高频输出遥测/已知响应估计。新电机指令只从它的时间戳之后生效，
  不再将它追溯应用到上一段采样间隔。
- 通过仿真时钟、已发送命令历史和上一拍实测计算耗时，将状态推进至预计生效时刻，参考也查询那个**原始绝对时刻**。
  只预测当前状态，不平移参考/窗口时钟。传输至下一内环tick按4ms建模，其剩余误差由日志明确核对，不称精确同步。
- 消除10ms阈值跳过8ms新测量的问题；只接纳新测量，实际控制频率由计算和通信决定，不宣称实现250Hz/100Hz。
- ULog输出提升至原生4ms控制记录，仅提高观测分辨率，不改飞控增益/物理参数。

### 同参考验证

冻结参考SHA256 `ae54cabca5d7c70eb1a5e65cea82d6e9540bde2942210f059f2de48066da5f30`，
原名义56.682964s/588.437118m，规划、4Hz/4m窗口、world模型/physics、PX4参数覆盖项均未变。
仍按旧终点条件：七窗整机退出后，距目标<.55m且速度<1m/s；不是精确零PVAJ验收。

| 项目 | 修改前 | 执行链对齐后 |
|---|---:|---:|
| Gazebo完整竞速时间（含返航） | 未完成 | **55.928s** |
| 正向穿窗 | 7窗后返航失跟 | 7窗各一次并返航 |
| 窗框接触 | 0（随后失控下坠） | 0 |
| 全程真值跟踪最大误差 | 15.687m | **0.15117m** |
| 返航段真值跟踪最大误差 | 米级失跟 | **0.09722m** |

本次真值RMS .07351m，目标位置误差 .22410m、估计速度 .98475m/s时满足旧判据。
因此55.928s比名义曲线末时刻早，并不是缩短/重规划了参考。实际路程587.961m。
最低机体中心高度 .66666m，七窗均有原始接触日志；没有单独地面接触传感器，不夸大为连续安全证书。

4128次竞速MPC调用，计算中位10.862ms/P95 15.593ms/最大25.700ms，平均实际73.84Hz。
全部4128条命令与接收端匹配；接收首条日志与预计生效时间差的绝对值P95为8ms，
有符号中位-4ms，极值[-20,+12]ms。接收日志4ms粒度，不等于电机物理生效时刻。
前推时长P50/P95/最大为24/32/44ms；尚非严格固定时延或硬实时证明。

数值执行模型独立24/32ms延迟对照均完整跟踪，最大误差1.286/.476mm；这些不计实飞成绩。
50相关测试通过，飞行期间源哈希不变，结束后experiment暂停，replay未操作。
这是一次成功验证，不能由一趟证明普适可靠，也不能把组合改动的改善归于某一个参数。

实飞原始结果：`../global_gate_minco_probe/results/full_pvaj_thrust90_aligned_px4_v1_20261012/`，
含 `physical_audit.json`、`execution_alignment_audit.json`、4ms接收匹配、ULog与冻结源码。
数值结果：`results/aligned_delay24_v1_20261012/` 与 `aligned_delay32_v1_20261012/`。

```bash
# 使用下文的acados环境变量；指定新目录，不能覆盖历史实飞。
python -m convex_timevarying_window.global_gate_minco_probe.flight \
  --plan-dir convex_timevarying_window/global_gate_minco_probe/results/full_pvaj_thrust90_f4_a4_v1_20261012 \
  --tracker execution-aligned --native-camera --outdir <新结果目录>
```

下文保留的是13状态历史实现与失败记录，不作为对齐版当前结果。

最新只读排查见 [第六窗分层诊断](DIAGNOSIS_20261012.md)：七组离线隔离，
同参考理想四桨直驱也失跟，PX4内环使其更早；规划气动力矩缺项已量化，
ULog采样指令运输吻合；未改生产算法、未新增全程Gazebo成绩。

2026-10-12。用户要求把当前跟踪改为 TOGT 实飞结构；不改规划、赛道、窗口时钟、
碰撞/完赛判据。不是作者 Agilicious 源码，未取得其受限许可源码；实现和参数差异必须明示。

## 结构与差异

13 状态 `[p(3),v(3),q_wxyz(4),omega_FLU(3)]`，4 输入为各桨物理推力 N。
模型包含 x500 质量/惯量/真实桨位、旋翼阻力及阻力力矩；不包含 PX4 PID/滤波器、
分配记忆或电机响应状态。原 33 状态预测器保留，不参与新优化；实际 PX4/Gazebo 仍有内环和电机延迟。

代价只有位置、速度、相对四元数、角速度和单桨参考推力跟踪，无倾角罚、控制平滑罚、
窗口避障、安全过滤、时间重参数化。约束为单桨推力及节点机体角速度界；不是连续安全保证。
参考位置时间不变：PVA 按原接口查询，航向沿用原固定 NED yaw=0；姿态来自含阻力的平坦映射，
角速度/角加速度以参考网格差分计算，单桨前馈使用逆分配和气动力矩补偿。不是原版解析全导数接口。

acados **v0.5.4 / d9d0ba2e8f127855119d38874af8a9570df1d8e5**，SQP-RTI、Gauss-Newton、
部分凝聚 HPIPM，多重射击，RK4/每段4小步。1s/25×40ms 预测，调度目标100Hz。
继承位置/速度/姿态权重2000/30/20，角速度.3，单桨输入.02，终端相同状态权重；
这些是 x500 适配值，不称作者实飞配置。每拍一次 RTI，不宣称非线性问题完全收敛。

输出 `sum(f[0])` 和**第一个未来节点（40ms）的预测机体角速度**，交给既有 ENU/FLU→NED/FRD、
推力 N→归一化油门运输，再由 PX4 内环执行。40ms 提取位置是公开原文未给出的明确适配选择。
绝不把固定为测量的 x[0] 角速度当新指令，也不直接向电机发送四路推力。
QP失败明确上报，运输层沿用既有中止/位置保持；不偷偷回退到旧 MPC 或复用失败解。

官方依据：

- [TOGT IV-C](https://arxiv.org/html/2309.06837v2#S4.SS3)：Agilicious/100Hz/PixRacer 总推力+角速度接口。
- [Sun NMPC IV-A](https://arxiv.org/html/2109.01365v6#S4.SS1)：13状态、单桨输入、节点推力和角速度约束。
- [当前 Agilicious 文档](https://agilicious.readthedocs.io/en/latest/modules/controller.html)：acados/HPIPM。
- [acados](https://docs.acados.org/python_interface/index.html)：C代码生成和 SQP-RTI 接口。

Sun 论文写 ACADO/qpOASES；这里使用当前文档所列 acados/HPIPM，不冒充作者当年二进制。
没有引入 INDI，也没有改变 PX4 参数或用真值替代状态估计。

## 依赖与运行

本机外部依赖在 `/home/jack/wyh/dependencies/acados-togt-v0.5.4`，不复制进研究源码。
acados_template 已安装进 conda wyh（包内版本元数据0.5.1，实际源码标签v0.5.4，以commit为准）。
另安装 Deprecated 1.3.1、Cython 3.3.0；现有 CasADi 3.8.1 未降级，acados会提示未测试版本。
生成C/共享库在本目录 `native/build/`，已忽略；构建时间与在线控制时间分开。

```bash
export ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/jack/anaconda3/envs/wyh/bin/python -m pytest -q convex_timevarying_window/togt_style_mpc/tests
```

当前 `single_gate_stop_racing.flight`、`global_gate_minco_probe.flight` 默认 `--tracker togt-style`，
可显式 `--tracker px4-model` 使用原版；旧冻结五档批次 run.py 显式指定旧跟踪器，避免偷换历史协议。
新全局跟踪固定已有轨迹：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 /home/jack/anaconda3/envs/wyh/bin/python -u -m convex_timevarying_window.global_gate_minco_probe.flight \
  --plan-dir convex_timevarying_window/global_gate_minco_probe/results/full_pvaj_f4_a4_v1_20261011 \
  --tracker togt-style --native-camera --outdir <新的结果目录>
```

仅操作 experiment 容器，不操作 replay；实时求解失败/实体接触不得当作成功。

## 验证

最终相关回归43项通过，其中本模块8项：悬停、力矩符号、四元数符号、未来角速度输出、硬输入界、
失败不执行、短程参考解析导数、多重射击热启动。构建缓存用文件锁和已完成库复用，
避免并发测试/飞行同时调用 acados Makefile 清理共享库；曾一次在构建阶段遇到该问题，未起飞。

### 最终版短程 PX4/Gazebo 验证

`results/gazebo_local_v3_20261012`：2s悬停、8s平滑往返、2s悬停，保留PX4估计反馈。
1189拍，无求解失败；估计状态位置误差RMS **0.00890m**、最大 **0.02002m**。
完整控制计算中位 **5.809ms**、P95 **7.649ms**、最大 **14.814ms**。
这支持“PX4接口接通且局部跟踪正常”，不等于七窗竞速、实体碰撞认证或每拍严格100Hz达标。
源模型签名 `c1e65dd31eb93963`，ULog和execution.npz保留；结束后experiment暂停，replay未操作。

### 完整冻结参考：仍然失败，不能宣称优于原跟踪器

`results/global_pvaj_f4_a4_v1_20261012`：同4Hz/4m、同全局完整PVAJ参考、同物理场景和PX4参数。
Gazebo真值确认前五窗正向各一次；**约44s开始明显失跟/下坠，46.052s中止，无完整竞速时间**。
46.040s估计角速度约[-35.8,10.0,-9.3]rad/s，随后QP失败，不能将失跟原因说成“先QP偶发失败”。
七窗框接触记录为0，但最大真值误差约10.05m且明显失控；不能称无碰撞安全飞行，地面接触未独立记录。
完整控制计算中位7.998ms、P95 11.166ms、最大18.621ms；运行期源哈希检查通过。
原版在同一参考第六窗碰框；此次更早失跟，**没有改善闭环竞速的证据**。

随后把RTI热启动改为同时平移上次优化的状态/输入，而非把饱和控制从新测量开环重积分1s。
这是多重射击求解实现修正，不改代价/边界/参考；最终版由上述Gazebo短程验证。
`results/frozen_execution_model_4ms_v3_20261012`：修正后按真实4ms数字内环时钟执行，
同参考仍在46.132s超过3m误差而中止，期间求解未报错，故不能说热启动已解决强激励失跟。
它是数值执行模型，不是第二次全赛道Gazebo完赛；未继续反复飞同一失败参考。

`results/execution_model_4ms_v3_20261012`：悬停恢复4s/平滑转弯12s均完成，
转弯RMS1.27mm、峰值9.91mm；控制中位4.77ms，最大24.58ms（2拍>10ms，0拍>100ms）。
早期`execution_model_v1`和`frozen_execution_model_shifted_v2`使用2ms步推进4ms数字滤波器，
不能当作精确对齐PX4执行模型；保留并由上述4ms结果取代。Gazebo实验未使用该数值探针积分器。

当前不改变规划、不增加避障/输出安全过滤、不放宽推力或角速度界、不调窗口时间。
13状态预测不包含实际PX4内环/电机延迟，CTBR转换也不会原样执行优化四桨力；
已有规划气动力矩缺项、硬件饱和与接口模型差异尚未因果分离，不能承诺换MPC即可完成该参考。
