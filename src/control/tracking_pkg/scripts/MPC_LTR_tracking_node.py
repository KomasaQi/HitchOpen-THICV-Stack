#!/usr/bin/env python3
# coding=utf-8
import ctypes
from numpy.ctypeslib import ndpointer
import pandas as pd
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
from TrailorTruck_5DOF_SP import TrailorTruck_5DOF_SP

######################################################################################
#####################################变量定义#########################################
######################################################################################
# 

#---------------------------------车辆状态量------------------------------------------
heading = 2.6 # 航向角初始化 rad
dheading = 0.0  # 撗擺角速度 rad/s
x_pos=0#-3447226.2   # X位置初始化 m
y_pos=0#-5747449.12   # Y位置初始化 m
vel_true = 0.0 # 轮速计测量速度 m/s 
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
Np = 25    # 预测时域步数 记得调整！！！！#


#---------------------------------参考轨迹相关量------------------------------------------
path_x = np.zeros(Np).reshape(1,-1)
path_y = np.zeros(Np).reshape(1,-1)
ref_Head = np.zeros(Np).reshape(1,-1)
x_dev = 0
y_dev = 0

# #**************************************调用MPC时需要增加的声明**********************************
lib = ctypes.CDLL('/home/jetson/catkin_ws/123/build/libmpc_control.so')  # 根据实际路径修改
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
    ctypes.c_int,
]
# MPC_Controller_qpOASES_Ycons.restype = ndpointer(ctypes.c_double)
MPC_Controller_qpOASES_Ycons.restype = ctypes.POINTER(ctypes.c_double)

# double* MPC_Controller_qpOASES_Ycons(double* a0, double* b0, double* c0, double* x0, double* u0,
#                                          double* Q0, double* R0, int Nx, int Nu, int Ny, int Np,
#                                          int Nc, double* uconstrain0, double* yconstrain0,
#                                          double* Yr0, double rho, int Nr
#                                         ) 


# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
_EPS = np.finfo(float).eps * 4.0
    
def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """

    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)
   

# def odom_callback(msg):
#     global x_pos,y_pos,Listener
#     # 获取 base_footprint 到 map 的变换关系
#     (trans, rot) = Listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
#     # x_pos = msg.pose.pose.position.x
#     # y_pos = msg.pose.pose.position.y
#     x_pos=trans[0]
#     y_pos=trans[1]

def tf_callback(msg):
    global x_pos,y_pos,heading
    # 获取 base_footprint 到 map 的变换关系
    for transform in msg.transforms:
        if transform.header.frame_id =="map":
            if transform.child_frame_id == "odom":
                x_pos = transform.transform.translation.x
                y_pos = transform.transform.translation.y
                quaternion=[
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]
                (roll,pitch,yaw)=euler_from_quaternion(quaternion)
                heading = yaw
                

# def imu_callback(msg):   
#     global heading,Listener
#     (trans, rot) = Listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
#     if msg.orientation_covariance[0]<0:
#         return
#     quaternion=[
#         msg.orientation.x,
#         msg.orientation.y,
#         msg.orientation.z,
#         msg.orientation.w
#     ]
#     (roll,pitch,yaw)=euler_from_quaternion(quaternion)
#     # roll=roll*180/math.pi
#     # pitch=pitch*180/math.pi

#     heading = yaw
#     # yaw=yaw*180/math.pi
#     # rospy.loginfo("滚转= %.3f 俯仰= %.3f 朝向= %.3f",roll,pitch,yaw)


def rotation(x0, y0, theta):
    # 计算旋转后的坐标
    x = x0 * math.cos(theta) - y0 * math.sin(theta)
    y = x0 * math.sin(theta) + y0 * math.cos(theta)
    return x, y

def path_callback(path):
    global ref_head,pose1_x,pose1_y
    # length=len(path.poses)
    pose1_x=path.poses[0].pose.position.x
    pose1_y=path.poses[0].pose.position.y
    pose2_x=path.poses[3].pose.position.x
    pose2_y=path.poses[3].pose.position.y    
    ref_head=np.arctan2(pose2_y-pose1_y,pose2_x-pose1_x)


def calculate_yaw_error(yaw, yaw_ref):
    (thref_x,thref_y)=rotation(np.cos(yaw_ref),np.sin(yaw_ref),-yaw)
    yaw_error =np.arctan2(-thref_y,thref_x)
    
    return yaw_error

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
    vx1 = vel_true + 1e-2 # 纵向速度
    Y1 = x_pos  # 位置Y
    X1 = y_pos  # 位置X

    state = np.array([
        vy1, df1, F1, dF1, vy2, df2, F2, dF2,
        th, dth, f1, vx1, Y1, X1
    ]).reshape(-1, 1)  # 转换为列向量 (14, 1)

    return state

def Steering_and_Speed_Ctrl(x_pos,y_pos):
    global U
    U = 0.0
    global ddelta_lqr
    ddelta_lqr = 0.0
    global ddelta 
    ddelta = 0.0
    global x_dev,y_dev
    global publish_rate,delta_cmd,ddelta_ref
    global ref_head,heading,pose1_x,pose1_y,vel_true,ref_vel,acc_cmd,acc_limit
    global path_x,path_y,ref_Head
    lat_err = -(x_pos-pose1_x)*np.sin(ref_head)+(y_pos-pose1_y)*np.cos(ref_head) # 不要对人家动手动脚嘛~嗯~~~
    head_err = calculate_yaw_error(heading, ref_head) # 不要对人家动手动脚嘛~嗯~~~
    global vel_pub;pid_controller_vel
    global Np
    # print(x_pos,y_pos,pose1_x,pose1_y)

    # acc_cmd=0.18 #油门开度
    acc_cmd += pid_controller_vel.update(np.real(ref_vel**2.1),np.real(vel_true**2.1))

    if acc_cmd >= acc_limit:  #油门限制幅度
        acc_cmd = acc_limit
    elif acc_cmd < 0:
        acc_cmd = 0
    #######求出A B C ########
    state0 = RealInput()
    Ts = 0.05
    A,B,C, observe = TrailorTruck_5DOF_SP(state0, Ts)
    ######求出A B C ##########
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

    u_int = delta_cmd/180*math.pi  #u出问题了
    u = np.array(u_int)
    
    
######################################################################################
#####################################调参来啦#########################################
######################################################################################
    ctrlMode = 1 #控制模式，1表示跟踪抑晃，2表示防侧翻
    # Np = 50 # 预测时域 ↑在变量定义的地方调节预测时域
    Nc = 5 # 控制变量个数
    Nr = 10 # 被约束的时刻数

    vx1 = state0[11]
    # print('vx1',vx1)
    Ts = 1/publish_rate     #0.05   
    q_th = np.interp(vx1, [0, 50, 120], np.array([11200, 200, 200]) * 0.5)
    q_dth = np.interp(vx1, [0, 50, 120], np.array([500, 500, 500]) * 0.5)
    #插值计算Q,R
    Q = np.diag([6050, 6050, 15550, q_th, q_dth, 150]).astype(np.float64)
    R = np.interp(vx1, [0, 1.5, 3], np.array([3e3, 5e3, 1e4]) * 1)

    rho = 1e10
    
    delta_limit = 15 #deg   #限制前轮转角
 
    

    rate_delta = 10 / 180 * np.pi * Ts
    uconstrain = np.array([[-delta_limit * np.pi / 180, delta_limit * np.pi / 180, -rate_delta, rate_delta]])
    yconstrain = np.array([
        [-1e15, 1e15],  # 约束示例，替换为实际值
        [-1e15, 1e15],
        [-1e15, 1e15],
        [-0.44, 0.44],
        [-25.0, 25.0],
        [-1.0, 1.0]
    ])
    
    Yr = np.vstack([path_y, path_x, ref_Head, np.zeros((3, Np))]).flatten(order = 'F').reshape(-1,1) # TODO 确认维度是否能正常拼在一起

    result_ptr = MPC_Controller_qpOASES_Ycons(
        A.flatten(order='F'),
        B.flatten(order='F'), 
        C.flatten(order='F'),
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
        rho, 
        Nr
    )
    
    result_size =Nu + Ny*Np + 1
    result = np.ctypeslib.as_array(result_ptr, shape=(result_size,))
    du = result[0]
    Y_init = result[1:Ny*Np+1]
    Y = Y_init.reshape((Np*Ny, 1))
    epsilon = result[-1]
    
    #Y和epsilon没用到啊
       
    ####################### 调用 MPC_Controller_qpOASES_Ycons 函数取出数组##################################
    
    print('本步求解出的 du = ', np.round(du*180/math.pi,2),' °')
    ddelta=float(du)
    delta_cmd += ddelta*180/math.pi 
    delta_cmd = (1 if delta_cmd > 0 else -1) * min(abs(delta_cmd), delta_limit)
     
    vel_cmd=Twist()
    vel_cmd.linear.x=acc_cmd   
    angular_z_cmd=delta_cmd
    vel_cmd.angular.z=round(angular_z_cmd,2)
    vel_pub.publish(vel_cmd)
    
    
    rospy.loginfo("转向角控制指令= %.2f °, 油门控制指令= %.2f ",vel_cmd.angular.z,vel_cmd.linear.x)
    rospy.logwarn("轨迹跟踪横向误差= %.3f m",lat_err)
    rospy.logwarn("航向角误差= %.3f rad",head_err)
    rospy.loginfo("当前朝向 %.2f,轨迹朝向 %.2f, 当前速度 %.2f,预期速度 %.2f",
                  heading*180/math.pi,ref_head*180/math.pi,vel_true,ref_vel)


def rate_callback(msg):
    global vel_true
    vel_true = (msg.data/360)*WHEEL_RADIUS*0.04



if __name__ =="__main__":
    
    rospy.init_node("MPC_LTR_tracking_node")
    # odom_sub=rospy.Subscriber("/odom",Odometry,odom_callback,queue_size=1)
    # imu_sub=rospy.Subscriber("/imu/data",Imu,imu_callback,queue_size=3)
    path_sub=rospy.Subscriber("/ref_path",Path,path_callback,queue_size=1)
    rate_sub=rospy.Subscriber("/wheel_rate",Float32,rate_callback,queue_size=1)
    tf_sub=rospy.Subscriber("/tf",TFMessage,tf_callback,queue_size=1)
    vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    start_time=time.time()
    # Listener = tf.TransformListener()

    publish_rate = 20 #Hz
    
    dacc_limit=0.005 #油门开度增量限制
    pid_controller_vel = PID(kp=0.6, ki=0.00, kd=1.9,
                         output_min=-dacc_limit,
                         output_max=dacc_limit,
                         integral_min=-50*dacc_limit,
                         integral_max=50*dacc_limit)
    
    rate = rospy.Rate(publish_rate)  # 设置发布频率

    try:
        while not rospy.is_shutdown():

            Steering_and_Speed_Ctrl(x_pos,y_pos)
            rate.sleep()
    finally:
        vel_cmd=Twist()
        vel_pub.publish(vel_cmd)