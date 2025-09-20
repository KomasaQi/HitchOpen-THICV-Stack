
import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer
import math


fr = math.pi/6
vr = 15 
deltar = 0.3
l = 2.7
Ts = 0.05
a = np.array([[1,  0, -Ts*vr*np.sin(fr)],
              [0,  1,  Ts*vr*np.cos(fr)],
              [0,  0,                 1]],dtype=np.float64)
b = np.array([[np.cos(fr), 0],
              [np.sin(fr), 0],
              [np.tan(deltar)/l, vr/(l*np.cos(deltar)**2)]],dtype=np.float64)*Ts
c = np.eye(3,dtype=np.float64)


# % 模型处理 
# %统计模型状态、控制量和观测量维度
Nx=np.shape(a)[0] #%状态量个数
Nu=np.shape(b)[1] #%控制量个数
Ny=np.shape(c)[0] #%观测量个数

x=np.array([1,2,0.4],dtype=np.float64)
u = np.array([0.0, 0.0],dtype=np.float64)

Np = 20
Nc = 3
    
Q = np.array([1,1,1.2]) #这里改动了，Q和R只传入权重即可，不需要完整的对角矩阵
R = np.array([0.1,0.5])


rho = 100.0
Yr = np.zeros((Np*Ny),dtype=np.float64)

rate_delta=90/180*math.pi  #限定方向盘转动速度
dt = Ts
uconstrain=np.array([[-10,        10,     -0.2*9.806*dt,0.2*9.806*dt],
                     [-30*math.pi/180,30*math.pi/180, -rate_delta,rate_delta]])
yconstrain=np.array([[-1, 1], [-0.05, 0.05], [-0.1, 0.1]])






lib = ctypes.CDLL('/home/jetson/catkin_ws/cpp_lib/build/libmy_cpp_code.so')  # 根据实际路径修改
# 声明函数的参数类型和返回类型
MPC_Controller_qpOASES_Ycons = lib.MPC_Controller_qpOASES_Ycons
MPC_Controller_qpOASES_Ycons.argtypes = [
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ndpointer(ctypes.c_double),
    ctypes.c_double,
]
MPC_Controller_qpOASES_Ycons.restype = ndpointer(ctypes.c_double)


# 调用 C++ 函数并传递数组参数
res = MPC_Controller_qpOASES_Ycons(
        a.flatten(order='F'),
        b.flatten(order='F'),
        c.flatten(order='F'),
        x,
        u,
        Q,
        R,
        Nx,
        Nu,
        Ny,
        Np,
        Nc,
        uconstrain.flatten(order='F'),
        yconstrain.flatten(order='F'),
        Yr,
        rho
    )
du = np.ctypeslib.as_array(ctypes.cast(res, ctypes.POINTER(ctypes.c_double)), shape=(Nu,))

print(du[1])
