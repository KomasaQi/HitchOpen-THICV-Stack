import numpy as np
import osqp 
import scipy.sparse as sp
import math
from MPC_Controller_osqp_Ycons import MPC_Controllor_qpOASES_Ycons as MPC

Ts = 0.05
fr = math.pi/6
vr = 15 
deltar = 0.3
l = 2.7

a = np.array([[1,  0, -Ts*vr*np.sin(fr)],
              [0,  1,  Ts*vr*np.cos(fr)],
              [0,  0,                 1]])
b = np.array([[np.cos(fr), 0],
              [np.sin(fr), 0],
              [np.tan(deltar)/l, vr/(l*np.cos(deltar)**2)]])*Ts
c = np.eye(3)

# % 模型处理 
# %统计模型状态、控制量和观测量维度
Nx=np.shape(a)[0] #%状态量个数
Nu=np.shape(b)[1] #%控制量个数
Ny=np.shape(c)[0] #%观测量个数


# %构建控制矩阵
A = np.block([[a, b], [np.zeros((Nu, Nx)), np.eye(Nu)]]) #%(Nx+Nu) x (Nx+Nu)
B = np.block([[b], [np.eye(Nu)]]);                       #%(Nx+Nu) x Nu
C = np.block([c, np.zeros((Ny,Nu))]); 


x=np.array([1,2,0.4])
u = np.array([0,0])


Np = 20
Nc = 3
    
Q = np.diag([1,1,1.2])
R = np.diag([0.1,0.5])

rho = 100
Yr = np.zeros((Np*Ny))

rate_delta=90/180*math.pi  #限定方向盘转动速度
dt = Ts
uconstrain=np.array([[-10,        10,     -0.2*9.806*dt,0.2*9.806*dt],
                     [-30*math.pi/180,30*math.pi/180, -rate_delta,rate_delta]])
yconstrain=np.array([[-1, 1], [-0.05, 0.05], [-0.1, 0.1]])

import time
run_iter = 50
du, dU = MPC(a,b,c,x,u,Q,R,Np,Nc,Yr,uconstrain,yconstrain,rho,eps_abs=1e-3,max_iteration=1000)
start_time = time.time()# 计时开始
for i in range(run_iter):
    du, dU = MPC(a,b,c,x,u,Q,R,Np,Nc,Yr,uconstrain,yconstrain,rho,eps_abs=1e-3,max_iteration=500,warm_start_x=dU)

execution_time  = time.time()-start_time# 计时结束
print("MPC输出控制增量 du = {}".format(du))
print("osqp用时：{:.1f}毫秒".format(execution_time*1000/run_iter))