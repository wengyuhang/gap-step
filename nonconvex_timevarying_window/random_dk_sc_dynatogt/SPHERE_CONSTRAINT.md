# 球形无人机穿越动态窗口的安全约束

无人机建模为半径为 $r$ 的球体，球心轨迹为 $p(t;D,K)$。窗口形状固定、位姿随时间已知，实体门框建模为零厚度平面障碍物。

## 1. 安全半径

$$
r_s=r+r_{\mathrm{margin}},\qquad r_{\mathrm{margin}}\ge0.
$$

## 2. 球心在窗口坐标系中的位置

设第 $i$ 个窗口原点为 $c_i(t)$，$R_i(t)\in SO(3)$ 将窗口坐标变换到世界坐标，则

$$
\begin{bmatrix}
q_i(t;D,K)\\
z_i(t;D,K)
\end{bmatrix}
=R_i(t)^\top\bigl[p(t;D,K)-c_i(t)\bigr].
$$

其中，$q_i\in\mathbb R^2$ 是球心在窗口平面内的投影坐标，$z_i\in\mathbb R$ 是球心到窗口平面的有符号法向距离。

## 3. 投影点到二维门框的距离

设 $\mathcal F_i\subset\mathbb R^2$ 为窗口坐标系中实际障碍物部分的闭集，定义

$$
d_{\mathcal F_i}(q)
=\inf_{y\in\mathcal F_i}\|q-y\|_2.
$$

这是到障碍物集合的无符号距离：投影点位于实体部分时距离为零。若孔外整个平面都是实体，使用开口的补集；若只有有限宽度的门框，使用真实门框区域。

## 4. 球心到三维门框的距离

窗口坐标系中的三维门框为 $\mathcal F_i\times\{0\}$。利用正交坐标分解，距离平方恰好为

$$
\boxed{
d_{i,\mathrm{3D}}^2(t;D,K)
=z_i^2(t;D,K)
+d_{\mathcal F_i}^2\bigl(q_i(t;D,K)\bigr).
}
$$

## 5. 连续时间安全约束

$$
\boxed{
\phi_i(t;D,K)
:=z_i^2(t;D,K)
+d_{\mathcal F_i}^2\bigl(q_i(t;D,K)\bigr)
-r_s^2
\ge0,
\quad\forall t\in[0,T(D,K)],\ \forall i.
}
$$

该条件精确表示球心到实际门框的距离不小于安全半径。等号允许安全球相切；安全余量为正时，真实球仍与门框保持间隔。球心离窗口平面至少一个安全半径时，约束自动满足，无需单独求进入、离开时刻。

##**
