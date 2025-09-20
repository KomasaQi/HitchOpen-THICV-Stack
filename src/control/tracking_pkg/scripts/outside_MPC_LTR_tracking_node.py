#!/usr/bin/env python3
# coding=utf-8
import ctypes
from numpy.ctypeslib import ndpointer
import pandas as pd
import rospy
from sensor_msgs.msg import Imu00
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
vel_true = 1.0 # 轮速计测量速度 m/s 
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


#获取车辆的位置信息和速度。
def odom_callback(msg):
    global x_pos,y_pos,gnss_spd,x_dev,y_dev
    x_pos = msg.pose.pose.position.x - x_dev
    y_pos = -msg.pose.pose.position.y - y_dev
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
    global x_dev,y_dev
    # length=len(path.poses)
    pose1_x=path_msg.poses[0].pose.position.x - x_dev
    pose1_y=path_msg.poses[0].pose.position.y - y_dev
    pose2_x=path_msg.poses[3].pose.position.x - x_dev
    pose2_y=path_msg.poses[3].pose.position.y - y_dev   
    
    ref_head=np.arctan2(pose2_y-pose1_y,pose2_x-pose1_x)
    
    original_point_num = len(path_msg.poses) - 1
    # print(original_point_num)
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
        
    path_x = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),path_x0).reshape(1,-1) - x_dev
    path_y = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),path_y0).reshape(1,-1) - y_dev
    ref_Head = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),ref_Head0).reshape(1,-1)
 
 
 
 
    
#计算车辆当前航向角和参考航向角之间的误差
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
    vx1 = vel_true  # 纵向速度
    Y1 = x_pos  # 位置Y
    X1 = y_pos  # 位置X

    state = np.array([
        vy1, df1, F1, dF1, vy2, df2, F2, dF2,
        th, dth, f1, vx1, Y1, X1
    ]).reshape(-1, 1)  # 转换为列向量 (14, 1)

    return state

  
  ############################################################findTargetIdx函数 ################################################################################## 目前用不到
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
############################################################Steering_and_Speed_Ctrl ################################################################################## 
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
    print(x_pos,y_pos,pose1_x,pose1_y)

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
        

    
    Yr = np.vstack([path_y, path_x, ref_Head, np.zeros((3, Np))]).flatten(order = 'F').reshape(-1,1) 
    # pd.DataFrame(Yr).to_csv("_Yr_5DOF.csv", header=False, index=False)

####################### 调用 MPC_Controller_qpOASES_Ycons 函数取出数组##################################

    # uconstrain = read_csv("/home/jetson/catkin_ws/123/uconstrain11.csv")
   # yconstrain = read_csv("/home/jetson/catkin_ws/123/yconstrain11.csv") # TODO 这个地方如果用程序自己构建的,就会报错   ####解决，yconstrain.flatten(order='F')###
                                                                            #  ERROR:  Error number undefined
                                                                            # ->ERROR:  Initial QP could not be solved due to infeasibility!
    
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
    
    #Y和epsilon没用到

    ####################### 调用 MPC_Controller_qpOASES_Ycons 函数取出数组##################################

    print('本步求解出的 du = ', np.round(du*180/math.pi,2),' °')
    ddelta=float(du)
    delta_cmd += ddelta*180/math.pi 
    delta_cmd = (1 if delta_cmd > 0 else -1) * min(abs(delta_cmd), delta_limit)

    angular_z_cmd=delta_cmd

    
    
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
    
def x_dev_callback(msg):
    global x_dev
    x_dev = msg.data
def y_dev_callback(msg):
    global y_dev
    y_dev = msg.data

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
    x_dev_sub=rospy.Subscriber("/x_dev_const",Float32,x_dev_callback,queue_size=1)
    y_dev_sub=rospy.Subscriber("/y_dev_const",Float32,y_dev_callback,queue_size=1)
    rate_sub=rospy.Subscriber("/wheel_rate",Float32,rate_callback,queue_size=1)
    # tf_sub=rospy.Subscriber("/tf",TFMessage,tf_callback,queue_size=1)
    vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=3)
    # Listener = tf.TransformListener()
    roll_sub=rospy.Subscriber("/roll_message",Float32,roll_callback,queue_size=1)
    roll2_sub=rospy.Subscriber("/roll_message2",Float32,roll2_callback,queue_size=1)
    
    log_pub=rospy.Publisher("/log_msg",String,queue_size=1)


    
    publish_rate = 20# Hz
    
    

    dacc_limit=0.003 #油门开度增量限制
    pid_controller_vel = PID(kp=0.6, ki=0.00, kd=1.9,
                         output_min=-dacc_limit,
                         output_max=dacc_limit,
                         integral_min=-50*dacc_limit,
                         integral_max=50*dacc_limit)

    rate = rospy.Rate(publish_rate)  # 设置发布频率

    try:
        while not rospy.is_shutdown():
            start_time = time.time()# 计时开始
            Steering_and_Speed_Ctrl(x_pos,y_pos)
            log_publishing()
            execution_time  = time.time()-start_time# 计时结束
            rospy.loginfo("防侧翻控制MPC用时： %.1f毫秒",execution_time*1000)
            rate.sleep()
    finally:
        vel_cmd=Twist()
        vel_pub.publish(vel_cmd)
        