#!/usr/bin/env python3
# coding=utf-8

# % 函数：MPC控制器：MPC_Controllor_osqp_Ycons**********************************************
# % 作者：Komasa Qi @ School of Vehicle and Mobility, Tsinghua Univ.
# %****************************使用说明*****************************************************
# % ●输入：
# %   a,b,c:  为离散形式的模型A,B,C矩阵
# %   Q,R:    最优调节的Q，R矩阵，Q为半正定，R为正定
# %   x,u:    状态量x(k)和控制量u(k-1)
# %   Np,Nc:  Np和Nc分别为预测时域和控制时域个数
# %   uconstrain: 控制量及其变化量的限制，形式如下：
# %               [u1min u1max du1min du1max;
# %               u2min u2max du2min du2max];
# %   yconstrain：为观测量，即系统输出的限制，可以设计为硬约束或者软约束，这里使用软约束。
# %               假设观测量数量为3个，即Ny=3，则使用样例如下：
# %               [y1min y1max;
# %                y2min y2max;
# %                y3min;y3max];
# %   rho:    为松弛因子权重，大于0的数字，数值大表示限制松弛因子，即对输出量的约束更硬。
# % ●输出:
# %   du：    控制量的变化量，Nu x 1
# %   dU:     所有预测时域控制量的变化量，最后还有一个元素是松弛因子。用于下次调用函数时输入作为热启动
# %****************************************************************************************
# % 调教MPC时可用以下参数初始化：
# % a=rand(3);b=rand(3,2);c=eye(3);x=rand(3,1);u=[0;0];Q=eye(3);R=0.1*eye(2);rho=5;
# % Np=50;Nc=3;Yr=zeros(Np*size(c,1),1);uconstrain=[-1 1 -0.1 0.1; -2 2 -0.2 0.2];
# % yconstrain=[-0.1,0.1;-1,1;-1,1]*0.01;


import numpy as np
import osqp 
import scipy.sparse as sp


def MPC_Controllor_qpOASES_Ycons(a, b, c, x, u, Q, R, Np, Nc, Yr, uconstrain, yconstrain, rho,
                                     warm_start_x=None, max_iteration=1000, eps_abs=1e-6):
    """
    函数：MPC控制器：MPC_Controllor_osqp_Ycons
    作者：Komasa Qi @ School of Vehicle and Mobility, Tsinghua Univ.

    使用说明:
     ●输入：
       a,b,c: 为离散形式的模型A,B,C矩阵
       Q,R:   最优调节的Q，R矩阵，Q为半正定，R为正定
       x,u:   状态量x(k)和控制量u(k-1)
       Np,Nc: Np和Nc分别为预测时域和控制时域个数
       uconstrain: 控制量及其变化量的限制，形式如下：
                   [u1min u1max du1min du1max;
                   u2min u2max du2min du2max];
       yconstrain：为观测量，即系统输出的限制，可以设计为硬约束或者软约束，这里使用软约束。
                   假设观测量数量为3个，即Ny=3，则使用样例如下：
                   [y1min y1max;
                    y2min y2max;
                    y3min;y3max];
       rho:   为松弛因子权重，大于0的数字，数值大表示限制松弛因子，即对输出量的约束更硬。
     ●输出:
       du:    控制量的变化量，Nu x 1
       dU:    所有预测时域控制量的变化量，最后还有一个元素是松弛因子。用于下次调用函数时输入作为热启动
    """
    # 模型处理 
    # 统计模型状态、控制量和观测量维度
    Nx=np.shape(a)[0] #状态量个数
    Nu=np.shape(b)[1] #控制量个数
    Ny=np.shape(c)[0] #观测量个数
    
    # 构建控制矩阵
    A = np.block([[a, b], [np.zeros((Nu, Nx)), np.eye(Nu)]]) #%(Nx+Nu) x (Nx+Nu)
    B = np.block([[b], [np.eye(Nu)]]);                       #%(Nx+Nu) x Nu
    C = np.block([c, np.zeros((Ny,Nu))]);                    #%   Ny   x (Nx+Nu)

    # 新的控制量为ksai(k)=[x(k),u(k-1)]'
    ksai=np.block([x,u])
    # 新的状态空间表达式为：ksai(k+1)=A*ksai(k)+B*du(k)  
    # 输出方程为： ita(k)=C*ksai(k)   %Ny x 1

    # 预测输出
    # 获取相关预测矩阵
    psai = np.zeros((Ny*Np,Nx+Nu)); #矩阵psai
    for i in range(Np):
        psai[i * Ny:(i + 1) * Ny, :] = C @ np.linalg.matrix_power(A, i + 1)

    theta = np.zeros((Np * Ny, Nc * Nu))  # 矩阵theta
    for i in range(Np):
        for j in range(i + 1):
            if j < Nc:
                theta[i * Ny:(i + 1) * Ny, j * Nu:(j + 1) * Nu] = C @ np.linalg.matrix_power(A, i - j) @ B

    # 输出方程可以写为 Y=psai*ksai(k)+theta*dU  % Ny*Np x 1

    # 控制
    # 变量设置
    E = psai @ ksai
    Qq = np.kron(np.eye(Np),Q)
    Rr = np.kron(np.eye(Nc),R)
    
    # 目标函数设计
    # H=theta'*Qq*theta+Rr;
    H = np.block([[theta.T @ Qq @ theta + Rr,np.zeros((Nu*Nc,1))],
                [np.zeros((1, Nu*Nc)), rho]])
    H = (H + H.T)/2  #保证矩阵对称
    g = np.append((E.T @ Qq @ theta - Yr.T @ Qq @ theta),0)

    # 约束条件相关矩阵
    At_tmp = np.zeros((Nc,Nc)) #下三角方阵
    for i in range(Nc):
        At_tmp[i,0:i+1]=1

    At=np.block([[np.kron(At_tmp,np.eye(Nu)),np.zeros((Nu*Nc,1))],
                        [theta,-np.ones((Ny*Np,1))],
                        [theta, np.ones((Ny*Np,1))],
                        [np.eye(Nu*Nc+1)]])
    
    # 控制量及其变化量的限制
    Umin=np.kron(np.ones((Nc)),uconstrain[:,0])
    Umax=np.kron(np.ones((Nc)),uconstrain[:,1])
    dUmin=np.append(np.kron(np.ones((Nc)),uconstrain[:,2]),-1e5)
    dUmax=np.append(np.kron(np.ones((Nc)),uconstrain[:,3]),1e5)
    
    # 上一时刻的控制量
    Ut=np.kron(np.ones(Nc),u)
    
    # 输出量约束
    Ymin=np.kron(np.ones(Np),yconstrain[:,0])
    Ymax=np.kron(np.ones(Np),yconstrain[:,1])
    
    # 限制量矩阵
    l = np.block([Umin-Ut,
                  np.ones((Ny*Np))*-1e10,
                  Ymin-E,
                  dUmin])
    
    u = np.block([Umax-Ut,
                  Ymax-E,
                  np.ones((Ny*Np))*1e10,
                  dUmax])
    
    # 开始求解过程
    # 在osqp库中，可以通过设置不同的参数来优化求解过程，以满足MPC应用中对更快求解速度的要求。以下是一些常用的参数选项：
    # eps_abs和eps_rel：这两个参数分别控制终止条件的绝对容差和相对容差。通过调整这些参数可以在精度和求解速度之间进行权衡。
    # max_iter：指定最大迭代次数。通过适当调整此参数，可以限制求解器的运行时间。
    # rho：ADMM算法的步长参数。增加rho可以加快收敛速度，但也可能导致数值不稳定。可以尝试不同的值以找到最佳的平衡点。
    # verbose：控制求解器的输出信息级别。将其设置为False可以减少输出信息，从而提高求解速度。
    # warm_start：启用预热功能，即使用上一次求解的结果作为下一次求解的初始点。在MPC应用中，如果问题的结构保持不变，可以使用该选项来提高求解速度。

    prob = osqp.OSQP()
    prob.setup(P=sp.csc_matrix(H), q=g, A=sp.csc_matrix(At), l=l, u=u,
                eps_abs=eps_abs, max_iter=max_iteration, verbose=False)

    if warm_start_x is not None:
        prob.warm_start(x=warm_start_x)
    
    result = prob.solve()
        
    du = result.x[0:Nu]
    dU = result.x
    return du, dU