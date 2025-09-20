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
from std_msgs.msg import Float32
from PIDController import PIDController as PID
import time
import numpy as np
from MatlabFunctions import interp1
# import tf
from tf2_msgs.msg import TFMessage

x_pos = 0.0
y_pos = 0.0
heading = 0.0

#**************************************调用MPC时需要增加的声明**********************************
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
#***********************************************************************************************


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

ref_head = 0.0  #rad
ref_vel = 0.65 #m/s
pose1_x = 0.0
pose1_y = 0.0
vel_true =0.0  #m/s 
WHEEL_RADIUS = 0.265 #m 轮子周长
acc_cmd = 0.0 #全局油门控制指令
delta_cmd = 0.0 #全局转角控制指令
acc_limit = 0.3
ddelta_ref =0.0 #全局转角增量

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

def Steering_and_Speed_Ctrl(x_pos,y_pos):
    global publish_rate,delta_cmd,ddelta_ref
    global ref_head,heading,pose1_x,pose1_y,vel_true,ref_vel,acc_cmd,acc_limit
    lat_err = -(x_pos-pose1_x)*np.sin(ref_head)+(y_pos-pose1_y)*np.cos(ref_head)
    head_err = calculate_yaw_error(heading, ref_head)*0.5
    global vel_pub;pid_controller_vel
    #acc_cmd=0.13 #油门开度
    acc_cmd += pid_controller_vel.update(np.real(ref_vel**2.1),np.real(vel_true**2.1))

    if acc_cmd >= acc_limit:  #油门限制幅度
        acc_cmd = acc_limit
    elif acc_cmd < 0:
        acc_cmd = 0

    vel_cmd=Twist()
    vel_cmd.linear.x=acc_cmd
    #vel_cmd.linear.x=0.16
    #********************************构建MPC问题*******************************************
    fr = ref_head
    vr = vel_true
    deltar = 0
    l = 0.3  #小车等效轴距大概0.3m
    Ts = 1/publish_rate
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

    x=np.array([(x_pos-pose1_x),
                (y_pos-pose1_y),
                head_err],dtype=np.float64)#这里关注是否加负号
    u = np.array([0, delta_cmd/180*math.pi],dtype=np.float64)

    Np = 35
    Nc = 2
        
    Q = np.array([5,5,15],dtype=np.float64) #这里改动了，Q和R只传入权重即可，不需要完整的对角矩阵
    R = np.array([1,10000],dtype=np.float64) #控制量是dv_ref,ddelta_ref


    rho = 100.0
    Yr = np.zeros((Np*Ny),dtype=np.float64)
    delta_limit=9 #deg   #限制前轮转角
    rate_delta=45/180*math.pi  #限定前轮转角转速限制
    uconstrain=np.array([[-10,10,-0.2*9.806*Ts,0.2*9.806*Ts],
                        [-delta_limit*math.pi/180,delta_limit*math.pi/180, -rate_delta*Ts,rate_delta*Ts]])
    yconstrain=np.array([[-1.5, 1.5], [-1.5, 1.5], [-0.5, 0.5]])


    
    #*******************************MPC求解器调用******************************************
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
    #****************************************************************************************
    # dv_ref=du[0]
    ddelta_ref=float(du[1])
    delta_cmd += deltar+ddelta_ref*180/math.pi
    # if delta_cmd >=delta_limit:
    #     delta_cmd = delta_limit
    # elif delta_cmd <=-delta_limit:
    #     delta_cmd = -delta_limit
        
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
    
    rospy.init_node("MPC_tracking_node")
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