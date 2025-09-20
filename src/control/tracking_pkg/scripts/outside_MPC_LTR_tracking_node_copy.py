#!/usr/bin/env python3
# coding=utf-8
import ctypes
from numpy.ctypeslib import ndpointer
import rospy
from sensor_msgs.msg import Imu
# from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import math
from PIDController import PIDController as PID
from std_msgs.msg import Float32
from std_msgs.msg import String
import time
import numpy as np
from MatlabFunctions import interp1
# import tf
from tf2_msgs.msg import TFMessage
import importlib.util
import sys
from scipy.linalg import expm
from scipy.interpolate import interp1d
from scipy.linalg import block_diag
import mpc_wrapper
import MB_CSC_get_wrapper

######################################################################################
#####################################变量定义#########################################
######################################################################################
 
#---------------------------------车辆状态量------------------------------------------
heading=0.0 # 航向角初始化 rad
dheading = 0.0  # 撗擺角速度 rad/s
x_pos=0.0   # X位置初始化 m
y_pos=0.0   # Y位置初始化 m
vel_true =0.0 # 轮速计测量速度 m/s 
x_acc = 0.0 # imu测得的牵引车x向加速度   
y_acc = 0.0 # imu测得的牵引车y向加速度
z_acc = 0.0 # imu测得的牵引车z向加速度
droll = 0.0 # imu测得的牵引车侧倾角速度 deg #TODO 證實一下
roll_angle = 0.0 # imu测得牵引车侧倾角 rad/s #TODO 證實一下

x2_acc = 0.0 # imu测得的x挂车向加速度   
y2_acc = 0.0 # imu测得的y挂车向加速度
z2_acc = 0.0 # imu测得的z挂车向加速度
droll2 = 0.0 # imu测得的挂车侧倾角速度 #TODO 證實一下
roll2_angle = 0.0 # imu测得挂车侧倾角 #TODO 證實一下

#---------------------------------参考状态量------------------------------------------
ref_head = 0.0  #rad
ref_vel = 0.80 #m/s
pose1_x = 0.0
pose1_y = 0.0

#---------------------------------车辆几何参数量------------------------------------------
WHEEL_RADIUS = 0.265 #m 轮子周长
l = 0.3  #小车等效轴距大概0.3m



# TODO 壮锐测的参数，嘿嘿

#---------------------------------可调节参数------------------------------------------
acc_limit_global = 1.0 # 全局最大油门指令

#---------------------------------控制量限制------------------------------------------
acc_limit = 1.0 # 实时最大油门指令初始化
acc_cmd = 0.0   #全局油门控制指令初始化
delta_cmd = 0.0 #全局转角控制指令初始化
ddelta_ref =0.0 #全局转角增量初始化
gnss_spd = 0.0  #GNSS定位速度初始化

#---------------------------------控制器相关量------------------------------------------
Np = 50    # 预测时域步数 记得调整！！！！#


#---------------------------------参考轨迹相关量------------------------------------------
path_x = np.zeros(Np)
path_y = np.zeros(Np)
ref_Head = np.zeros(Np)

# #**************************************调用MPC时需要增加的声明**********************************

# 动态库的绝对路径##定义一个接口函数mpc_wraooer.cpp，将mpc_control.cpp在接口函数mpc_wraooer.cpp中调用，使用pybind11调用mpc_wraooer.cpp控制器###
# 对于MB_CSC_get_wrapper.cpp同上
# lib_path = "/home/jetson/catkin_ws/src/tracking_pkg/scripts/mpc_wrapper.so"
# lib_path = "/home/jetson/catkin_ws/src/tracking_pkg/scripts/MB_CSC_get_wrapper.so"

# # 动态加载库
# spec = importlib.util.spec_from_file_location("mpc_wrapper", lib_path)
# mpc_wrapper = importlib.util.module_from_spec(spec)
# sys.modules["mpc_wrapper"] = mpc_wrapper
# spec.loader.exec_module(mpc_wrapper)

# spec = importlib.util.spec_from_file_location("MB_CSC_get_wrapper", lib_path)
# MB_CSC_get_wrapper = importlib.util.module_from_spec(spec)
# sys.modules["MB_CSC_get"] = MB_CSC_get_wrapper
# spec.loader.exec_module(MB_CSC_get_wrapper)



# lib = ctypes.CDLL('/home/jetson/catkin_ws/123/build/lib/libMPC_Controller_qpOASES_Ycons.so')  # 根据实际路径修改
# # 声明函数的参数类型和返回类型
# MPC_Controller_qpOASES_Ycons = lib.MPC_Controller_qpOASES_Ycons
# MPC_Controller_qpOASES_Ycons.argtypes = [
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ctypes.c_int,
#     ctypes.c_int,
#     ctypes.c_int,
#     ctypes.c_int,
#     ctypes.c_int,
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ctypes.c_double,
#     ndpointer(ctypes.c_double),
#     ndpointer(ctypes.c_double),
#     ctypes.c_int,
#     #TODO--
# ]
# MPC_Controller_qpOASES_Ycons.restype = ndpointer(ctypes.c_double)

    #  double* MPC_Controller_qpOASES_Ycons(double* a0, double* b0, double* c0, double* x0, double* u0,
    #                                      double* Q0, double* R0, int Nx, int Nu, int Ny, int Np,
    #                                      int Nc, double* uconstrain0, double* yconstrain0,
    #                                      double* Yr0, double rho, double* Tt0, double* Pipi0,int Nr, 
    #                                       MatrixXd &du, MatrixXd &Y, double* epsilon) 
#***********************************************************************************************


# # axis sequences for Euler angles
# _NEXT_AXIS = [1, 2, 0, 1]

# # map axes strings to/from tuples of inner axis, parity, repetition, frame
# _AXES2TUPLE = {
#     'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
#     'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
#     'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
#     'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
#     'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
#     'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
#     'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
#     'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

# _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
# _EPS = np.finfo(float).eps * 4.0
    
# def quaternion_matrix(quaternion):
#     """Return homogeneous rotation matrix from quaternion.

#     >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
#     >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
#     True

#     """

#     q = np.array(quaternion[:4], dtype=np.float64, copy=True)
#     nq = np.dot(q, q)
#     if nq < _EPS:
#         return np.identity(4)
#     q *= math.sqrt(2.0 / nq)
#     q = np.outer(q, q)
#     return np.array((
#         (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
#         (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
#         (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
#         (                0.0,                 0.0,                 0.0, 1.0)
#         ), dtype=np.float64)

# def euler_from_matrix(matrix, axes='sxyz'):
#     """Return Euler angles from rotation matrix for specified axis sequence.

#     axes : One of 24 axis sequences as string or encoded tuple

#     Note that many Euler angle triplets can describe one matrix.

#     >>> R0 = euler_matrix(1, 2, 3, 'syxz')
#     >>> al, be, ga = euler_from_matrix(R0, 'syxz')
#     >>> R1 = euler_matrix(al, be, ga, 'syxz')
#     >>> numpy.allclose(R0, R1)
#     True
#     >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
#     >>> for axes in _AXES2TUPLE.keys():
#     ...    R0 = euler_matrix(axes=axes, *angles)
#     ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
#     ...    if not numpy.allclose(R0, R1): print axes, "failed"

#     """
#     try:
#         firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
#     except (AttributeError, KeyError):
#         _ = _TUPLE2AXES[axes]
#         firstaxis, parity, repetition, frame = axes

#     i = firstaxis
#     j = _NEXT_AXIS[i+parity]
#     k = _NEXT_AXIS[i-parity+1]

#     M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
#     if repetition:
#         sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
#         if sy > _EPS:
#             ax = math.atan2( M[i, j],  M[i, k])
#             ay = math.atan2( sy,       M[i, i])
#             az = math.atan2( M[j, i], -M[k, i])
#         else:
#             ax = math.atan2(-M[j, k],  M[j, j])
#             ay = math.atan2( sy,       M[i, i])
#             az = 0.0
#     else:
#         cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
#         if cy > _EPS:
#             ax = math.atan2( M[k, j],  M[k, k])
#             ay = math.atan2(-M[k, i],  cy)
#             az = math.atan2( M[j, i],  M[i, i])
#         else:
#             ax = math.atan2(-M[j, k],  M[j, j])
#             ay = math.atan2(-M[k, i],  cy)
#             az = 0.0

#     if parity:
#         ax, ay, az = -ax, -ay, -az
#     if frame:
#         ax, az = az, ax
#     return ax, ay, az


# def euler_from_quaternion(quaternion, axes='sxyz'):
#     """Return Euler angles from quaternion for specified axis sequence.

#     >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
#     >>> numpy.allclose(angles, [0.123, 0, 0])
#     True

#     """
#     return euler_from_matrix(quaternion_matrix(quaternion), axes)
   

#获取车辆的位置信息和速度。
def odom_callback(msg):
    global x_pos,y_pos,gnss_spd
    x_pos = msg.pose.pose.position.x
    y_pos = -msg.pose.pose.position.y
    speed_x = msg.twist.twist.linear.x
    speed_y = msg.twist.twist.linear.y
    gnss_spd = math.sqrt(math.pow(speed_x, 2) + math.pow(speed_y, 2))

#获取当前航向角和横摆角速度
def heading_callback(msg):
    global heading,dheading,publish_rate
    Ts = 1/publish_rate
    heading0 = (2*math.pi-msg.data)+math.pi/2 # 更新航向角
    prev_heading = heading
    if heading0>math.pi:
        heading = heading0-math.pi*2
    else:
        heading = heading0
    # 更新横摆角加速度
    dheading = (heading - prev_heading) / Ts

#获取加速度和滚转角速度
def imu_callback(msg):   
    global x_acc,y_acc,z_acc,droll
    x_acc = msg.linear_acceleration.x
    y_acc = msg.linear_acceleration.y
    z_acc = msg.linear_acceleration.z
    droll = msg.angular_velocity.x
    

#获取2加速度和滚转角速度
def imu2_callback(msg):   
    global x2_acc,y2_acc,z2_acc,droll2
    x2_acc = msg.linear_acceleration.x
    y2_acc = msg.linear_acceleration.y
    z2_acc = msg.linear_acceleration.z
    droll2 = msg.angular_velocity.x

#获取车辆当前的滚转角度
def roll_callback(msg):
    global roll_angle
    roll_angle = msg.data
#获取车辆当前的滚转角度
def roll2_callback(msg):
    global roll2_angle
    roll2_angle = msg.data

#实现二维平面上的点绕原点旋转的坐标变换
def rotation(x0, y0, theta):
    # 计算旋转后的坐标
    x = x0 * math.cos(theta) - y0 * math.sin(theta)
    y = x0 * math.sin(theta) + y0 * math.cos(theta)
    return x, y
#从路径消息中提取路径点，计算参考航向角
def path_callback(path_msg):
    global ref_head,pose1_x,pose1_y,path_x,path_y,Np,ref_Head
    # length=len(path.poses)
    pose1_x=path_msg.poses[0].pose.position.x
    pose1_y=path_msg.poses[0].pose.position.y
    pose2_x=path_msg.poses[3].pose.position.x
    pose2_y=path_msg.poses[3].pose.position.y    
    
    ref_head=np.arctan2(pose2_y-pose1_y,pose2_x-pose1_x)
    
    original_point_num = len(path_msg.poses) - 1
    path_x0 = np.zeros(original_point_num)
    path_y0 = np.zeros(original_point_num)
    ref_Head0 = np.zeros(original_point_num)
    path_x = np.zeros(Np)
    path_y = np.zeros(Np)
    ref_Head = np.zeros(Np)
    
    for i in range(original_point_num):  
        current_x = path_msg.poses[i].pose.position.x
        next_x = path_msg.poses[i+1].pose.position.x
        current_y = path_msg.poses[i].pose.position.y
        next_y = path_msg.poses[i+1].pose.position.y
        path_x0[i] = current_x
        path_y0[i] = current_y
        ref_Head0[i]=np.arctan2(next_y-current_y,next_x-current_x)
        
    path_x = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),path_x0)
    path_y = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),path_y0)
    ref_Head = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),ref_Head0)
 
 
 
 
    
#计算车辆当前航向角和参考航向角之间的误差
def calculate_yaw_error(yaw, yaw_ref):
    (thref_x,thref_y)=rotation(np.cos(yaw_ref),np.sin(yaw_ref),-yaw)
    yaw_error =np.arctan2(-thref_y,thref_x)
    
    return yaw_error

 #####################################################################得到getParam_SP函数#################################################################   
def getParam_SP(x):
# 根据变量生成传输参数
    TotMass = 14462
    m1 = x[0]  # 参数1
    m0 = TotMass - m1
    h0 = x[1]  # 参数2
    hp = x[2]  # 参数3
    lp = x[3]  # 参数4
    g = 9.806
    coeff = x[4]  # 参数5
    exh = 0.77  # 额外的高度是从罐底到简化力的中心：车轴心 的距离
    sim_param = [m0, m1, h0 + exh, hp + exh, lp, g, coeff]
    return sim_param
######################################################################得到getParam_SP函数##################################################################  

#######################################################################离散化A、B函数######################################################################
def c2d_zoh(Ac, Bc, Ts):

    nx = Ac.shape[0]  # 状态维度
    nu = Bc.shape[1]  # 输入维度

    # 构造扩展矩阵 [Ac, Bc; 0, 0]
    M = np.block([
        [Ac, Bc],
        [np.zeros((nu, nx + nu))]
    ])

    # 矩阵指数计算
    M_exp = expm(M * Ts)

    # 提取离散时间的 A 和 B 矩阵
    A = M_exp[:nx, :nx]
    B = M_exp[:nx, nx:nx + nu]

    return A, B
#######################################################################离散化A、B函数######################################################################  
    
  

# def TrailorTruck_5DOF_SP(state0, Ts):
#     global params

#     # 定义全局参数的初始化
#     if not hasattr(params, "initialized"):
#         params = {}
#         # 初始化参数
#         params['g'] = 9.806

#         # 摆模型参数
#         x = [10890, 1.5, 0.13, 0.97, 0.13]
#         sim_param = getParam_SP(x)  # 替代 MATLAB 的 getParam_SP 函数
#         params['m0'], params['mp'], params['h0'], params['hp'], params['lp'], _, params['co'] = sim_param

#         # 车辆模型参数
#         L1 = (5 + 6.27) / 2
#         L2 = (10.15 + 11.5) / 2
#         params['a'] = 1.385
#         params['b'] = L1 - params['a']
#         params['c'] = 5.635 - params['a']
#         params['e'] = 5.5
#         params['d'] = L2 - params['e']
#         params['Tw1'] = (2.03 + 1.863 * 2) / 3
#         params['Tw2'] = 1.863

#         hm1s = 1.02  # 牵引车质心高度
#         hm2s = 1.00  # 半挂车质心高度
#         hhitch = 1.1  # 铰接点高度

#         hroll1 = 0.2314
#         hroll2 = 0.9630
#         params['h1'] = hm1s - hroll1
#         params['h2'] = hm2s - hroll2
#         params['hc1'] = hhitch - hroll1
#         params['hc2'] = hhitch - hroll2

#         params['m1s'] = 6310
#         params['m1'] = params['m1s'] + 570 + 785 * 2
#         params['m2s'] = 20387
#         params['m2'] = params['m2s'] + 665 * 2

#         params['I1xx'], params['I1xz'], params['I1zz'] = 6879, 130, 19665
#         params['I2xx'], params['I2xz'], params['I2zz'] = 9960, 0, 331380

#         # 轮胎模型参数
#         p0 = [-1e4 * val for val in [300, 300, 470]]
#         params['k1'], params['k2'], params['k3'] = p0

#         # 悬架模型参数
#         params['kr1'], params['kr2'], params['k12'] = 18.7e5, 9.14e5, 68.3e5
#         params['c1'], params['c2'] = 64.4e3, 197.7e3

#         params['initialized'] = True

#         # 提取状态变量
#         dF2, f1, vx1, vx2 = state0[7], state0[10], state0[11], state0[11]

#         # 质量矩阵 M
#         M = np.zeros((8, 8))
#         M[0, :] = [params['m1'] * vx1 * params['c'], params['I1zz'], 0, -params['m1s'] * params['h1'] * params['c'] - params['I1xz'], 0, 0, 0, 0]
#         M[1, :] = [params['m1'] * vx1 * params['hc1'] - params['m1s'] * params['h1'] * vx1, -params['I1xz'], 0, params['I1xx'] + 2 * params['m1s'] * params['h1']**2 - params['m1s'] * params['h1'] * params['hc1'], 0, 0, 0, 0]
#         # 其他 M 元素依次填充...

#         # 动力学矩阵 Acm
#         Acm = np.zeros((8, 8))
#         # 填充 Acm 矩阵

#         # 输入矩阵 Bcm
#         Bcm = np.zeros((8, 1))
#         # 填充 Bcm 矩阵

#         # 观测矩阵 C
#         C = np.zeros((6, 14))
#         C[:2, 12:] = np.eye(2)
#         C[2, 10] = 1
#         C[3:5, 8:10] = np.eye(2)
#         C[5, :8] = 2 / (np.mean([params['Tw1'], params['Tw2']]) * (params['m1'] + params['m2']) * params['g']) * np.array([0, 0, -params['kr1'], -params['c1'], 0, 0, -params['kr2'], -params['c2']])

#         # 离散化计算
#         A, B = c2d_zoh(Acm, Bcm, Ts)

#         # 输出结果
#         observe = np.dot(C, state0)
#         return A, B, C, observe

# def MPC_Ctrllor(state, delta, ref_pos, ref_head, Ts, Q, R, Np, Nc, Nr, rho, incre_MB, incre_CSC):

#     global Tt, Pipi, yconstrain, uconstrain  

#     if "Tt" not in globals():
#         rate_delta = 10 / 180 * np.pi * Ts
#         uconstrain = np.array([[-15 * np.pi / 180, 15 * np.pi / 180], [-rate_delta, rate_delta]])
#         yconstrain = np.array([
#             [-1e5, 1e5],  # 约束示例，替换为实际值
#             [-1e5, 1e5],
#             [-1e5, 1e5],
#             [-0.44, 0.44],
#             [-25, 25],
#             [-1, 1]
#         ])

#         nu = B.shape[1]
#         ny = C.shape[0]
#         Tt, Pipi = MB_CSC_get_wrapper.MB_CSC_get(Np, Nc, Nr, Nu, Ny, incre_MB, incre_CSC)

#         # 插值计算参考轨迹
#        lenRef = len(refPos)
        #  refPosx = np.interp(np.linspace(1, lenRef, Np), np.arange(1, lenRef + 1), refPos[:, 0])
        #  refPosy = np.interp(np.linspace(1, lenRef, Np), np.arange(1, lenRef + 1), refPos[:, 1])
        #  refHeads = np.interp(np.linspace(1, lenRef, Np), np.arange(1, lenRef + 1), refHead[:, 0])
        #  Yr = np.hstack([refPosy, refPosx, refHeads, np.zeros(3 * Np)])
#         # 调用MPC优化器
#         du, Y, epsilon = mpc_wrapper.MPC_Controller_qpOASES_Ycons(
#         A, B, C, x, u, Q, R,
#         Nx, Nu, Ny, Np, Nc,
#         uconstrain, yconstrain, Yr,
#         rho, Tt, Pipi, Nr
#     )

#     ddelta = du
#     return ddelta, observe, epsilon, Y
    
 ############################################################输入状态量函数 ##################################################################################     
def RealInput():  
    global vel_true,x_pos,y_pos,heading,dheading
    global droll,droll2,roll_angle,roll2_angle
    vy1 = 0 / 3.6  # 侧向速度
    df1 = dheading  # 撗摆角速度
    F1 = roll_angle / 180 * np.pi  # 侧倾角
    dF1 = droll # 侧倾角速度
    vy2 = 0 / 3.6                                # TODO ??? 啊？？？ 怎麼會是0啊？？？
    df2 = dheading
    F2 = roll2_angle / 180 * np.pi
    dF2 = droll2 # 牵引车
    th = 0 # 单摆模型摆角                         # TODO 后续需要补充！！！！！
    dth = 0 # 单摆模型摆动角速度
    f1 = heading
    vx1 = vel_true  # 纵向速度
    Y1 = x_pos  # 位置Y
    X1 = y_pos  # 位置X

    state = np.array([
        vy1, df1, F1, dF1, vy2, df2, F2, dF2,
        th, dth, f1, vx1, Y1, X1
    ]).reshape(-1, 1)  # 转换为列向量 (14, 1)

    return state
  ############################################################输入状态量函数 ##################################################################################    
 
  ############################################################findTargetIdx函数 ################################################################################## 
def findTargetIdx(pos, path, length, spd, Ts, Np):
    # 计算当前位置到路径点的欧氏距离
    dist = np.sum((path - pos) ** 2, axis=1)
    
    # 找到最近路径点索引
    idx = np.argmin(dist)
    
    # 找到目标路径点索引
    target_distance = length[idx] + spd * Ts * Np
    dist_to_target = np.abs(length - target_distance)
    idxend = np.argmin(dist_to_target)
    
    return idx, idxend
############################################################findTargetIdx函数 ################################################################################## 
def Steering_and_Speed_Ctrl(x_pos,y_pos):
    global publish_rate,delta_cmd,ddelta_ref
    global ref_head,heading,pose1_x,pose1_y,vel_true,ref_vel,acc_cmd,acc_limit
    global path_x,path_y,ref_Head
    lat_err = -(x_pos-pose1_x)*np.sin(ref_head)+(y_pos-pose1_y)*np.cos(ref_head) # 不要对人家动手动脚嘛~嗯~~~
    head_err = calculate_yaw_error(heading, ref_head) # 不要对人家动手动脚嘛~嗯~~~
    global vel_pub;pid_controller_vel
    global Np
    # acc_cmd=0.18 #油门开度
    acc_cmd += pid_controller_vel.update(np.real(ref_vel**2.1),np.real(vel_true**2.1))

    if acc_cmd >= acc_limit:  #油门限制幅度
        acc_cmd = acc_limit
    elif acc_cmd < 0:
        acc_cmd = 0
    
    ############################################################求出A B C ################################################################################## 
    fr = ref_head 
    vr = vel_true
    deltar = 0
    state0 = RealInput()
    
    global params

    # 定义全局参数的初始化
    if not hasattr(params, "initialized"):
        params = {}
        # 初始化参数
        params['g'] = 9.806

        # 摆模型参数
        x = [10890, 1.5, 0.13, 0.97, 0.13]
        sim_param = getParam_SP(x)  
        params['m0'], params['mp'], params['h0'], params['hp'], params['lp'], _, params['co'] = sim_param

        # 车辆模型参数
        L1 = (5 + 6.27) / 2
        L2 = (10.15 + 11.5) / 2
        params['a'] = 1.385
        params['b'] = L1 - params['a']
        params['c'] = 5.635 - params['a']
        params['e'] = 5.5
        params['d'] = L2 - params['e']
        params['Tw1'] = (2.03 + 1.863 * 2) / 3
        params['Tw2'] = 1.863

        hm1s = 1.02  # 牵引车质心高度
        hm2s = 1.00  # 半挂车质心高度
        hhitch = 1.1  # 铰接点高度

        hroll1 = 0.2314
        hroll2 = 0.9630
        params['h1'] = hm1s - hroll1
        params['h2'] = hm2s - hroll2
        params['hc1'] = hhitch - hroll1
        params['hc2'] = hhitch - hroll2

        params['m1s'] = 6310
        params['m1'] = params['m1s'] + 570 + 785 * 2
        params['m2s'] = 20387
        params['m2'] = params['m2s'] + 665 * 2

        params['I1xx'], params['I1xz'], params['I1zz'] = 6879, 130, 19665
        params['I2xx'], params['I2xz'], params['I2zz'] = 9960, 0, 331380

        # 轮胎模型参数
        p0 = [-1e4 * val for val in [300, 300, 470]]
        params['k1'], params['k2'], params['k3'] = p0

        # 悬架模型参数
        params['kr1'], params['kr2'], params['k12'] = 18.7e5, 9.14e5, 68.3e5
        params['c1'], params['c2'] = 64.4e3, 197.7e3

        params['initialized'] = True

        # 提取状态变量
        dF2, f1, vx1, vx2 = state0[7], state0[10], state0[11], state0[11]

        # 质量矩阵 M
        M = np.zeros((8, 8))
        M[0, :] = [params['m1'] * vx1 * params['c'], params['I1zz'], 0, -params['m1s'] * params['h1'] * params['c'] - params['I1xz'], 0, 0, 0, 0]
        M[1, :] = [params['m1'] * vx1 * params['hc1'] - params['m1s'] * params['h1'] * vx1, -params['I1xz'], 0, params['I1xx'] + 2 * params['m1s'] * params['h1']**2 - params['m1s'] * params['h1'] * params['hc1'], 0, 0, 0, 0]
        # 其他 M 元素依次填充...

        # 动力学矩阵 Acm
        Acm = np.zeros((8, 8))
        # 填充 Acm 矩阵

        # 输入矩阵 Bcm
        Bcm = np.zeros((8, 1))
        # 填充 Bcm 矩阵

        # 观测矩阵 C
        C = np.zeros((6, 14))
        C[:2, 12:] = np.eye(2)
        C[2, 10] = 1
        C[3:5, 8:10] = np.eye(2)
        C[5, :8] = 2 / (np.mean([params['Tw1'], params['Tw2']]) * (params['m1'] + params['m2']) * params['g']) * np.array([0, 0, -params['kr1'], -params['c1'], 0, 0, -params['kr2'], -params['c2']])

        # 离散化计算
        A, B = c2d_zoh(Acm, Bcm, Ts)

        # 输出结果
        observe = np.dot(C, state0)
    
    # a = np.array([[1,  0, -Ts*vr*np.sin(fr)],#TODO
    #             [0,  1,  Ts*vr*np.cos(fr)],
    #             [0,  0,                 1]],dtype=np.float64)
    # b = np.array([[np.cos(fr), 0],#TODO
    #             [np.sin(fr), 0],
    #             [np.tan(deltar)/l, vr/(l*np.cos(deltar)**2)]],dtype=np.float64)*Ts
    # c = np.eye(3,dtype=np.float64)#TODO
    ############################################################求出A B C ##################################################################################  

    
    # % 模型处理 
    # %统计模型状态、控制量和观测量维度
    Nx=np.shape(A)[0] #%状态量个数
    Nu=np.shape(B)[1] #%控制量个数
    Ny=np.shape(C)[0] #%观测量个数
    
    x = RealInput()
    
    delta=U
    rate_delta = 10 / 180 * math.pi * Ts  # 转弧度制
    ddelta_lqr = max(min(ddelta_lqr, rate_delta), -rate_delta)
    MPC_coeff = 1
    ddelta = (1 - MPC_coeff) * ddelta_lqr + MPC_coeff * ddelta
    deltacmd = ddelta + delta
    deltacmd = (1 if deltacmd > 0 else -1) * min(abs(deltacmd), 15 / 180 * math.pi)
    U = deltacmd

    u = delta

    # x=np.array([(x_pos-pose1_x),#TODO
    #             (y_pos-pose1_y),
    #             head_err],dtype=np.float64)#这里关注是否加负号
    # u = np.array([0, delta_cmd/180*math.pi],dtype=np.float64)#TODO
    
    
######################################################################################
#####################################调参来啦#########################################
######################################################################################
    ctrlMode = 1 #控制模式，1表示跟踪抑晃，2表示防侧翻
    # Np = 50 # 预测时域 ↑在变量定义的地方调节预测时域
    Nc = 15 # 控制变量个数
    Nr = 23 # 被约束的时刻数

    
    incre_MB = 3
    incre_CSC = 1
    
    Ts = 1/publish_rate     #0.05   
    # Q = np.array([5,5,15],dtype=np.float64) #这里改动了，Q和R只传入权重即可，不需要完整的对角矩阵 #TODO
    # R = np.array([1,10000],dtype=np.float64) #控制量是dv_ref,ddelta_ref #TODO--
    q_th = np.interp(vx1, [0, 50, 120], [11200, 200, 200] * 0.5)
    q_dth = np.interp(vx1, [0, 50, 120], [500, 500, 500] * 0.5)
    #插值计算Q,R
    Q = np.diag([6050, 6050, 15550, q_th, q_dth, 150])
    R = np.interp(vx1, [0, 50, 120], [1e4, 1e5, 5e6] * 1)

    rho = 1e10  
    
    delta_limit=15 #deg   #限制前轮转角
    #rate_delta=90/180*math.pi  #限定前轮转角转速限制
    
    delta = U
    # pos = [x_pos, vel_true]
    # idx, idxend = findTargetIdx(pos, path, len, heading, Ts, Np)###path路径问题###
    
    start_time = time.time()
    if "Tt" not in globals():
        rate_delta = 10 / 180 * np.pi * Ts
        uconstrain = np.array([[-15 * np.pi / 180, 15 * np.pi / 180], [-rate_delta, rate_delta]])
        yconstrain = np.array([
            [-1e5, 1e5],  # 约束示例，替换为实际值
            [-1e5, 1e5],
            [-1e5, 1e5],
            [-0.44, 0.44],
            [-25, 25],
            [-1, 1]
        ])
        Tt, Pipi = MB_CSC_get_wrapper.MB_CSC_get(Np, Nc, Nr, Nu, Ny, incre_MB, incre_CSC)
        
        # 假设 path 是一个 NumPy 数组，形状为 (N, M)
        # refPos = path[idx:idxend, 0:2]
        # refHead 是一个 NumPy 数组或 Python 列表
        # refHead = refHead[idx:idxend]
        
        # lenRef = refPos.shape[0]  # 获取轨迹点数量
    
       # 插值生成参考轨迹
        # refPosx = np.interp(np.linspace(1, lenRef, Np), np.arange(1, lenRef + 1), refPos[:, 0])
        # refPosy = np.interp(np.linspace(1, lenRef, Np), np.arange(1, lenRef + 1), refPos[:, 1])
        # refHeads = np.interp(np.linspace(1, lenRef, Np), np.arange(1, lenRef + 1), refHead[:, 0])
        
        # 构造 Yr
        # Yr = np.reshape(np.vstack([refPosy, refPosx, refHeads, np.zeros((3, Np))]), (Ny * Np, 1))
        Yr = np.reshape(np.vstack([path_y, path_y, ref_Head, np.zeros((3, Np))]), (Ny * Np, 1)) # TODO 确认维度是否能正常拼在一起
       
        # 调用MPC优化器
        du, Y, epsilon = mpc_wrapper.MPC_Controller_qpOASES_Ycons(
        A, B, C, x, u, Q, R,
        Nx, Nu, Ny, Np, Nc,
        uconstrain, yconstrain, Yr,
        rho, Tt, Pipi, Nr
    )

    ddelta = du
    iter_time = time.time() - start_time
    
    rate_delta = 10 / 180 * 3.141592653589793 * Ts  # 转弧度制
    ddelta_lqr = max(min(ddelta_lqr, rate_delta), -rate_delta)
    MPC_coeff = 1
    ddelta = (1 - MPC_coeff) * ddelta_lqr + MPC_coeff * ddelta
    deltacmd = ddelta + delta
    deltacmd = (1 if deltacmd > 0 else -1) * min(abs(deltacmd), 15 / 180 * 3.141592653589793)
    U = deltacmd
    dLTR = (observe[6] - LTR) / Ts
    LTR = observe[6]
    sys = [deltacmd, LTR, ctrlMode, epsilon, iter_time]

   

    # ddelta_ref=float(du[1])
    # delta_cmd += deltar+ddelta_ref*180/math.pi #TODO

        
    # angular_z_cmd=delta_cmd
    
    #******************应急情况判断：刹车处理***********************************************
    if abs(lat_err)>10: #假如横向误差大于10m，也就是对应GNSS失效的情况
        angular_z_cmd = 0.0
        if vel_true > 0.2:
            acc_cmd = -0.8 #紧急制动
            rospy.logerr('GNSS节点失效!正在制动！')
        else:
            acc_cmd = 0
            rospy.logerr('GNSS节点失效!已紧急停止！')
    #*************************************************************************************
    
    vel_cmd=Twist()
    vel_cmd.linear.x=acc_cmd
    vel_cmd.angular.z=round(angular_z_cmd,2)
    vel_pub.publish(vel_cmd)
    
######################################################################################
#####################################打印信息#########################################
######################################################################################
    rospy.loginfo("转向角控制指令= %.2f °, 油门控制指令= %.2f ",vel_cmd.angular.z,vel_cmd.linear.x)
    rospy.logwarn("轨迹跟踪横向误差= %.3f m",lat_err)
    rospy.logwarn("航向角误差= %.3f rad",head_err)
    rospy.loginfo("当前朝向 %.2f,轨迹朝向 %.2f,当前速度 %.2f,预期速度 %.2f,导航速度 %.2f",
                  heading*180/math.pi,ref_head*180/math.pi,vel_true,ref_vel,gnss_spd)
######################################################################################    
    

def rate_callback(msg):
    global vel_true,acc_limit,acc_limit_global
    vel_true = (msg.data/360)*WHEEL_RADIUS*0.04
    acc_limit = np.min([acc_limit_global,vel_true*0.46+0.19])
    



def log_publishing():
    global x_pos,y_pos,heading,vel_true
    global delta_cmd,acc_cmd
    global x_acc,y_acc,z_acc
    global x2_acc,y2_acc,z2_acc
    global roll_angle,roll2_angle
    global droll,droll2
    global log_pub
    time_stamp = rospy.Time.now()
    log_str = (str(time_stamp)+','+str(x_pos)+','+str(y_pos)+','+str(heading)+','
               +str(vel_true)+','+str(delta_cmd)+','+str(acc_cmd)+','
               +str(x_acc)+','+str(y_acc)+','+str(z_acc)+','
               +str(droll)+','
               +str(roll_angle)+','
               +str(x2_acc)+','+str(y2_acc)+','+str(z2_acc)+','
               +str(droll2)+','
               +str(roll2_angle))
    log_msg = String()
    log_msg.data=log_str
    log_pub.publish(log_msg)
####################################################################################################
if __name__ =="__main__":
    rospy.init_node("outside_MPC_LTR_tracking_node")
    odom_sub=rospy.Subscriber("/odom_message",Odometry,odom_callback,queue_size=1)
    heading_sub=rospy.Subscriber("/heading_message",Float32,heading_callback,queue_size=1)
    imu_sub=rospy.Subscriber("/imu_message",Imu,imu_callback,queue_size=1)
    imu2_sub=rospy.Subscriber("/imu_message2",Imu,imu2_callback,queue_size=1)
    path_sub=rospy.Subscriber("/ref_path",Path,path_callback,queue_size=1)
    rate_sub=rospy.Subscriber("/wheel_rate",Float32,rate_callback,queue_size=1)
    # tf_sub=rospy.Subscriber("/tf",TFMessage,tf_callback,queue_size=1)
    vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=3)
    start_time=time.time()
    # Listener = tf.TransformListener()
    roll_sub=rospy.Subscriber("/roll_message",Float32,roll_callback,queue_size=1)
    roll2_sub=rospy.Subscriber("/roll_message2",Float32,roll2_callback,queue_size=1)
    
    log_pub=rospy.Publisher("/log_msg",String,queue_size=1)


    
    publish_rate = 20 # Hz
    
    

    dacc_limit=0.003 #油门开度增量限制
    pid_controller_vel = PID(kp=0.6, ki=0.00, kd=1.9,
                         output_min=-dacc_limit,
                         output_max=dacc_limit,
                         integral_min=-50*dacc_limit,
                         integral_max=50*dacc_limit)

    rate = rospy.Rate(publish_rate)  # 设置发布频率

    try:
        while not rospy.is_shutdown():

            Steering_and_Speed_Ctrl(x_pos,y_pos)
            log_publishing()
            rate.sleep()
    finally:
        vel_cmd=Twist()
        vel_pub.publish(vel_cmd)
        