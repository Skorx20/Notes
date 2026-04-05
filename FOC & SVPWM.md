# FOC 与 SVPWM 学习笔记

## 1. 什么是 FOC？
<big>**FOC（Field-Oriented Control，磁场定向控制）**，也称矢量控制（VC），是目前控制无刷直流电机（BLDC）和永磁同步电机（PMSM）的高性能方法之一。  
它通过精确控制磁场的大小与方向，使电机转矩平稳、噪声低、效率高，且具有高速动态响应。

> 可以理解为对无刷电机进行“像素级”控制，实现传统方法难以达到的效果。

---

## 2. FOC 控制流程

1. 采样三相电流 ( $I_a, I_b, I_c$ )
2. Clark 变换：( $I_a, I_b, I_c \rightarrow I_\alpha, I_\beta$ )
3. Park 变换：( $I_\alpha, I_\beta \rightarrow I_q, I_d$ )
4. 计算 ( $I_q, I_d$ ) 与给定值 ( $I_{q\_ref}, I_{d\_ref}$ ) 的误差
5. 误差经 PI 控制器得到输出电压 ( $U_q, U_d$ )
6. 反 Park 变换：( $U_q, U_d \rightarrow U_\alpha, U_\beta$ )
7. 用 ( $U_\alpha, U_\beta$ ) 合成电压空间矢量，经 SVPWM 调制输出三相桥状态
8. 控制逆变器 MOS 管，驱动电机
9. 循环上述步骤

---

## 3. Clark 变换（三相→两相 静止坐标系）

三相坐标系 ( $I_a, I_b, I_c$ ) 是二维平面内的三个非正交基向量，通过投影变换得到正交的 $\alpha$-$\beta$ 坐标系。

<center>
<img src="Picture\三相→两相静止坐标系.jpg" width=400>
<br>三相→两相 静止坐标系
</center>

变换公式：
$$
\begin{cases}
I_\alpha = I_a + \cos\left(\frac{2\pi}{3}\right)I_b + \cos\left(\frac{2\pi}{3}\right)I_c \\[4pt]
I_\beta = \sin\left(\frac{2\pi}{3}\right)I_b - \sin\left(\frac{2\pi}{3}\right)I_c
\end{cases}
$$

代入 $\cos\frac{2\pi}{3} = -\frac{1}{2},\sin\frac{2\pi}{3} = \frac{\sqrt{3}}{2}$ ：
$$
\begin{bmatrix}
I_\alpha \\ I_\beta
\end{bmatrix}
=
\begin{bmatrix}
1 & -\frac{1}{2} & -\frac{1}{2} \\[4pt]
0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2}
\end{bmatrix}
\cdot
\begin{bmatrix}
I_a \\ I_b \\ I_c
\end{bmatrix}
$$

---

## 4. Park 变换（静止→旋转坐标系）

将 $\alpha$-$\beta$ 坐标系旋转转子角度 $\theta$ ，得到随转子旋转的 d-q 坐标系。

<center>
<img src="Picture\静止→旋转坐标系.jpg" width=400>
<br>静止→旋转坐标系
</center>

变换公式：
$$
\begin{cases}
I_d = I_\alpha \cos\theta + I_\beta \sin\theta \\[4pt]
I_q = -I_\alpha \sin\theta + I_\beta \cos\theta
\end{cases}
$$

矩阵形式：
$$
\begin{bmatrix} I_d \\ I_q \end{bmatrix}
=
\begin{bmatrix}
\cos\theta & \sin\theta \\
-\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix} I_\alpha \\ I_\beta \end{bmatrix}
$$

> 注：$\theta$ 由编码器实时获得。经过此变换，旋转向量变为定值，控制变量被线性化。

---

## 5. FOC 的控制目标
前面通过Clark变换和Park变换将转子磁链进行了解耦，分解为了转子旋转的径向和切向这两个方向的变量：

<center>
<img src="Picture\FOC控制目的.jpg" width=400>
<br>FOC控制目标
</center>

- $I_q$ ：期望的力矩输出
- $I_d$ ：希望尽可能为 0

---

## 6. 空间电压矢量

### 6.1 空间电压矢量的定义

空间电压矢量是描述电机合成磁场方向与大小的虚拟矢量。  
以逆变器开关状态 **100**（a 上桥导通，b、c 下桥导通）为例：

<center>
<img src="Picture\电路100状态.jpg" width=400>
<br>电路开关状态为100
</center>
此时电路等效为：

<center>
<img src="Picture\等效电路.jpg" width=400>
<br>等效电路
</center>

等效相电压：
$$
U_a = \frac{2}{3}U_{dc},\quad U_b = -\frac{1}{3}U_{dc},\quad U_c = -\frac{1}{3}U_{dc}
$$
如果我们规定指向中心的方向为正，反之为负，那么此时我们可以画出下图中的三个电压矢量​​$\vec{U_a}$、$\vec{U_b}$、$\vec{U_c}$（左边），以及它们的合成电压矢量$\vec{U}$（右边）：

<center>
<img src="Picture\空间电压矢量合成.jpg" width=400>
<br>空间电压矢量合成
</center>

合成矢量大小：
$$
|\vec{U}| = \frac{2}{3}U_{dc} + \frac{1}{3}U_{dc} = U_{dc}
$$
矢量方向即磁场方向，所以这个矢量​其实就可以表征我们希望转子旋转到的方向，也即所需要生成的磁场方向了。

### 6.2 开关函数

为了研究各相上下桥臂不同开关组合时逆变器输出的空间电压矢量，我们定义开关函数  $S_x \ (x \in a,b,c)$ ：
$$
S_x =
\begin{cases}
1, & \text{上桥臂导通} \\
0, & \text{下桥臂导通}
\end{cases}
$$

共有 8 种状态，其中 $U_0(000)$ 和 $U_7(111)$ 为零矢量（合力矩为零）。

### 6.3 基本电压矢量图

在 $\alpha$-$\beta$ 平面上，6 个非零矢量将平面分为 6 个扇区（I ~ VI），每个矢量幅值为 $\frac{2}{3}U_{dc}$ 。

<center>
<img src="Picture\基本电压矢量图.jpg" width=400>
<br>基本电压矢量图
</center>

---

## 7. SVPWM

### 7.1 SVPWM 合成任意矢量原理

利用这6个空间电压矢量作为基向量就可以合成任意矢量。在每一个扇区，选择相邻两个电压矢量以及零矢量，按照伏秒平衡原则来合成每个扇区内的任意电压矢量，即：
$$
\int_{0}^{T}U_{ref}dt = \int_{0}^{T_x}U_xdt + \int_{T_x}^{T_x+T_y}U_ydt + \int_{T_x+T_y}^{T}U_0^*dt
$$
离散后得：
$$
U_{ref} \times T = U_x \times T_x + U_y \times T_y + U_0^* \times T_0^*
$$
式子中的 $U_{ref}$ 是我们期望得到的电压矢量，T是一个PWM周期。$U_x$ 和 $U_y$ 分别是用于合成 $U_{ref}$ 的两个空间电压矢量，也就是上面说的6个基向量中的两个。$T_x$ 和 $T_y$ 就是在一个周期 T 中 $U_x$ 和 $U_y$ 所占的时间。$U_0^*$ 指的是两个零矢量，通过合理地配置零矢量可以让空间电压矢量的切换更平顺。  
所以上面公式的含义就是：我们可以周期性地在不同空间电压矢量之间切换，只要合理地配置不同基向量在一个周期中的占空比，就可以合成出等效的任意空间电压矢量。  
以第 I 扇区为例，目标矢量 $U_{ref}$ 可由相邻基本矢量 $U_4(100)$ 和 $U_6(110)$ 合成。  

<center>
<img src="Picture\Ⅰ扇区合成目标矢量.jpg" width=400>
<br>Ⅰ扇区合成目标矢量
</center>

设 PWM 周期为 T ， $U_4$ 作用时间 $T_4$ ， $U_6$ 作用时间 $T_6$ ，由正弦定理：
$$
\frac{|U_{ref}|}{\sin\frac{2\pi}{3}} = \frac{\left| \frac{T_6}{T}U_6 \right|}{\sin\theta} = \frac{\left| \frac{T_4}{T}U_4 \right|}{\sin\left(\frac{\pi}{3}-\theta\right)}
$$

已知 $|U_4| = |U_6| = \frac{2}{3}U_{dc}$ ，解得：
$$
\begin{cases}
T_4 = m T \sin\left(\frac{\pi}{3} - \theta\right) \\[4pt]
T_6 = m T \sin\theta
\end{cases}
$$

其中调制比：
$$
m = \sqrt{3}\,\frac{|U_{ref}|}{U_{dc}}
$$

零矢量作用时间：
$$
T_0 = T_7 = \frac{1}{2}\left(T - T_4 - T_6\right)
$$

> 对称分配零矢量，采用中央对齐 PWM 波形，降低谐波。

### 7.2 开关顺序（七段式）

为减少开关损耗，每次只切换一个桥臂。以第 I 扇区为例：
$$
000 \rightarrow 100 \rightarrow 110 \rightarrow 111 \rightarrow 110 \rightarrow 100 \rightarrow 000
$$

<center>
<img src="Picture\Ⅰ扇区目标矢量开关顺序.jpg" width=400>
<br>Ⅰ扇区合成目标矢量开关顺序
</center>

其他扇区顺序如下表：

| 扇区 | 角度范围 | 开关切换顺序 |
|------|----------|----------------|
| I    | 0°~60°   | 0-4-6-7-7-6-4-0 |
| II   | 60°~120° | 0-2-6-7-7-6-2-0 |
| III  | 120°~180°| 0-2-3-7-7-3-2-0 |
| IV   | 180°~240°| 0-1-3-7-7-3-1-0 |
| V    | 240°~300°| 0-1-5-7-7-5-1-0 |
| VI   | 300°~360°| 0-4-5-7-7-5-4-0 |

> 表中数字代表开关状态编号（如 4 对应 100，6 对应 110 等）。

### 7.3 最终输出

根据计算出的时间 $T_4, T_6, T_0, T_7$ 以及切换顺序，配置 MCU 的比较寄存器，生成三路 PWM 信号，控制 MOS 管，产生期望的电压、电流和力矩。