# FAPP-PPO 算法说明

## 1. 状态、动作与刚体动力学

四旋翼状态采用

\[
x=(p,v,R,\omega),
\]

其中 \(p,v,\omega\in\mathbb{R}^3\)，\(R\in SO(3)\)。策略输出归一化 CTBR 动作

\[
a=(a_T,a_{\omega_x},a_{\omega_y},a_{\omega_z})\in[-1,1]^4.
\]

总推力命令和期望角速度先转换为期望 wrench：

\[
F_d=m\,s(a_T),\qquad
\omega_d=\omega_{\max}a_\omega,
\]

\[
\tau_d=J\frac{\omega_d-\omega}{\tau_\omega}
 \omega\times J\omega .
\]

混控矩阵 \(M\) 把 wrench 分配为四个旋翼推力：

\[
f=M^{-1}[F_d,\tau_d]^\top,\qquad
f_j\leftarrow\operatorname{clip}(f_j,f_{\min},f_{\max}).
\]

再由裁剪后的实际 wrench 积分刚体平移和旋转动力学。因此代码中的单旋翼上限会实际改变下一状态，而不只是事后统计。

## 2. 连续形变窗口

每个实例在 reset 时固定完整的非周期关键帧。常开模式使用七个关键帧；时间关键模式使用约 0.30 s 的稠密关键帧，以表达短暂开放区间。窗口中心、旋转向量和每个有序边界点分别用 natural cubic spline 插值：

\[
c_i(t),\quad \rho_i(t),\quad b_{i,k}(t).
\]

过程在有限规划区间内是连续且不要求首尾相接的，所以不是预设周期运动。程序生成器用正半径极图构造边界：

\[
r(\theta,t)=1+\sum_k \alpha_k(t)\phi_k(\theta)>0,
\]

从而在局部非刚性形变时保持简单、连通和无洞。接口本身接收统一采样的任意边界关键帧，不依赖极图公式。

真实多边形为 \(\Omega_i(t)\)，安全区为

\[
\Omega_i^{\rm safe}(t)
=\Omega_i(t)\ominus B(0,d_{\rm safe}).
\]

时间关键模式允许安全区在收缩过程中变空，但物理多边形始终必须有效、连通、无洞且面积大于零。只有

\[
\operatorname{area}(\Omega_i^{safe}(t))\ge A_{\min}
\]

才算可通行；每个窗口至少存在一个可通行时刻。开放和闭合尺度由 smoothstep 连续连接，不使用瞬时开关。

每个窗口使用三个独立随机流：开放日程、位姿运动和局部形变。不同窗口的随机流也相互
分离。开放日程是带随机初始相位、随机宽度和随机重现间隔的非周期 renewal process，
在 reset 时完整生成。它不读取路线长度、名义到达时间或无人机状态，因此不存在
“无人机快到时由环境临时开门”的因果通路。策略只能从未来预览中获知并适应这个固定
外生过程。

## 3. 穿越、顺序和闭环

对一步轨迹 \((p_k,t_k)\to(p_{k+1},t_{k+1})\)，分别计算窗口移动平面的有符号距离。若符号改变，则在线性插值得到的相对交点处重新查询窗口状态。只有

\[
q_i(t_\times)\in\operatorname{int}
\Omega_i^{\rm safe}(t_\times)
\]

才是合法穿越。

- 穿过当前目标窗口：进度加一；
- 先穿过未来窗口或再次穿过已完成窗口：乱序终止；
- 交点落入物理区域但不在安全区，或接触边框带：碰撞终止；
- 从物理门框外侧绕过窗口平面：既不计穿越，也不判碰撞。

完成所有窗口后，只有同时满足

\[
\|p-p_0\|\le\epsilon_p,\quad
\|v-v_0\|\le\epsilon_v,\quad
d_{SO(3)}(R,R_0)\le\epsilon_R,\quad
\|\omega-\omega_0\|\le\epsilon_\omega
\]

才判为闭环成功。

## 4. 未来预览与非对称 actor–critic

actor 不接收绝对时间。它接收：

- 当前 \(v,R,\omega\)；
- 当前进度和是否进入返回阶段；
- 名义 CTBR；
- 当前目标及后续一个目标在多个相对时域的中心、法向、速度、安全区面积、安全裕度和八方向形状签名。
- 时间关键模式下，每个预览还包含可通行标志、计划机会是否激活、归一化距开放和距关闭时间。

critic 在 actor 输入之外额外接收：

- 归一化绝对时间 \(t/t_{\max}\)；
- 所有剩余窗口在启发式到达时刻的中心、法向、速度、面积和是否已完成。

这使 critic 能在训练时利用完整未来实例，而部署 actor 仍只依赖有限预览。绝对时间只给 critic，避免策略对固定训练时长产生硬编码依赖。

## 5. 残差策略

名义控制器使用预测到达时刻的安全区内部锚点，构造 PD 期望加速度和几何姿态误差，输出 \(a_{\rm nominal}\)。PPO 输出的是有界残差：

\[
a=\operatorname{clip}
\left(a_{\rm nominal}
+\beta\tanh z_\theta,\,-1,1\right),
\]

默认 \(\beta=0.20\)。该结构提供基本可行飞行先验，同时让 RL 学习提前等待、加速、转弯和形变时序补偿。PPO 损失额外加入

\[
\lambda_{\rm prior}
\left\|\tanh\mu_\theta(o)\right\|^2
\]

以抑制样本较少时策略均值快速偏离已验证的名义控制器；默认
\(\lambda_{\rm prior}=0.05\)。

日程感知名义控制器若预计早于窗口开放到达，就先追踪窗口前方的 holding point；窗口开放后再追踪实时安全锚点。PPO 学习等待时机、提前加速、大幅运动/形变补偿和穿越时间裕度。

训练必须将 residual=0 名义控制器作为消融，否则不能把全部性能归因于 PPO。

## 6. 奖励和时间目标

每一步都支付时间代价：

\[
r_{\rm time}=-\Delta t.
\]

到达时刻预览锚点定义势函数

\[
\Phi(x,t)=-
\left(
\|p-p^\star(t+\hat\tau)\|
+\lambda_v\|v-v^\star(t+\hat\tau)\|
+\mathbb{1}_{\rm return}e_{\rm full-state}
\right).
\]

进度塑形采用势差 \(\Phi_{k+1}-\Phi_k\)，另加合法穿越和闭环完成奖励，并减去：

- 碰撞/乱序/超时惩罚；
- 错失当前窗口的一次计划开放机会；
- \(\|a_k-a_{k-1}\|^2\) 动作差平滑项；
- 四旋翼实际单旋翼推力能耗项。

发生碰撞或乱序时，不允许保留正的 progress reward。

## 7. PPO

rollout 只由冻结的 `model_old` 采样，梯度只更新 `model`，每轮结束执行

```text
model_old <- model
```

PPO 使用 clipped surrogate、GAE 和价值损失。KL early stop 使用标准非负近似

\[
\widehat{D}_{KL}
=\mathbb{E}\left[
\exp(\Delta\log\pi)-1-\Delta\log\pi
\right]\ge 0,
\]

不使用可能为负的旧新 log-prob 均值差作为主 KL。

## 8. 课程与评价

课程依次增加：

```text
static       两个静态非凸窗口
moving       三个平移/旋转窗口
deforming    三个局部连续形变窗口
full         四个平移、旋转、尺度与局部形变组合窗口
```

正式结果必须在未参与训练的固定种子上报告成功率。总时间只在成功回合中取平均，失败回合不能以 episode horizon 伪装为飞行时间。
