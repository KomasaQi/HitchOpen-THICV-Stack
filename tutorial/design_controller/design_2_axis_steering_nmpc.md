# 考虑执行器时延的双轴自适应转向NMPC轨迹跟踪控制

## 1. 问题描述
对于PIX底盘来说，其前后两个轴都可以进行独立的转向控制，这也引出了更加一般化的轨迹跟踪控制问题。接下来准备对本问题进行分析建模，并建立对应的适用于ROS1和Casadi的C++语言的NMPC轨迹跟踪控制器。

## 2. 模型建立
首先我们采用线性轮胎模型，设前后轴刚度系数分别为$k_1$和$k_2$，则前后轴的侧向力以表示为：
$$
\begin{aligned}
F_{y1} &= k_1 \alpha_1 = k_1 \left(\arctan\left(\frac{v_y}{v_x}+\frac{\omega_r a}{v_x}\right)-\delta_1\right) \tag{1}\\ 
F_{y2} &= k_2 \alpha_2 = k_2 \left(\arctan\left(\frac{v_y}{v_x}-\frac{\omega_r b}{v_x}\right)-\delta_2\right)\\
\end{aligned}
$$
其中$\alpha_1$和$\alpha_2$分别为前后轴的轮胎侧偏角，$\delta_1$和$\delta_2$分别为前后轴的转向角，$\omega_r$为车辆的横摆角速度，$v_y$为车辆质心的侧向速度，$v_x$为车辆质心的纵向速度，$a$和$b$分别为质心到前后轴的距离。

对于车辆质心进行受力分析，假设车辆质量为$m$，则有：
$$
\begin{aligned}
\sum F_{y} &= m a_{y} = F_{y1} \cos{\delta_1} + F_{y2}\cos{\delta_2} \\
\sum M_z &= I_z \dot{\omega_r} = F_{y1}a\cos{\delta_1} - F_{y2}b\cos{\delta_2} \tag{2}
\end{aligned}
$$

其中$I_z$为车辆的横摆转动惯量。根据牛顿欧拉方程，可以近似得到：
$$
\begin{aligned}
\sum a_{x} &= \dot{v}_x - v_y \omega_r \\
\sum M_z &= \dot{v}_y + v_x \omega_r \tag{3}
\end{aligned}
$$

此时我们只需要代入轮胎侧向力的具体表达式就可以得到非线性的二自由度车辆方程：
$$
\begin{aligned}
k_1 \left(\arctan\left(\frac{v_y}{v_x}+\frac{\omega_r a}{v_x}\right)-\delta_1\right) + k_2 \left(\arctan\left(\frac{v_y}{v_x}-\frac{\omega_r b}{v_x}\right)-\delta_2\right) = m v_x(\frac{\dot{v}_y}{v_x} +  \omega_r)\\

a k_1 \left(\arctan\left(\frac{v_y}{v_x}+\frac{\omega_r a}{v_x}\right)-\delta_1\right)\cos{\delta_1} - b k_2 \left(\arctan\left(\frac{v_y}{v_x}-\frac{\omega_r b}{v_x}\right)-\delta_2\right)\cos{\delta_2} = I_z \dot{\omega}_r \tag{4}
\end{aligned}
$$
接下来需要引入执行器时延的描述。我们可以将前后转向的执行器时延分别描述一个一阶惯性环节+纯时延环节：
$$
G(s) = \frac{\delta(t)}{\delta_{des}(t)} = \frac{K}{T_{d}s + 1}e^{-T_{L}s} \tag{5}
$$

其中：
$K$为执行器的增益，$T_{d}$为执行器的时间常数，$T_{L}$为执行器的时延。

我们可以将前后转向的执行器时延分别描述为：
$$
\begin{aligned}
\delta_1(t) &= G(s) \delta_{des,1}(t) \\
\delta_2(t) &= G(s) \delta_{des,2}(t) \tag{6}
\end{aligned}
$$

## 3. 代码描述
我们可以定义一个ODM