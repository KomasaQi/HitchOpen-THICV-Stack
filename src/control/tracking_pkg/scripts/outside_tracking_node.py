#!/usr/bin/env python3
# coding=utf-8

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
heading=0.0
x_pos=0.0
y_pos=0.0

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
   
gnss_spd = 0.0 #GNSS定位速度

def odom_callback(msg):
    global x_pos,y_pos,gnss_spd
    x_pos = msg.pose.pose.position.x
    y_pos = -msg.pose.pose.position.y
    speed_x = msg.twist.twist.linear.x
    speed_y = msg.twist.twist.linear.y
    gnss_spd = math.sqrt(math.pow(speed_x, 2) + math.pow(speed_y, 2))
    # # 保存到文本文件
    # timestamp = rospy.Time.now()
    # with open('odom_data231103001.csv', 'w') as file:       
    #     file.write("{}, {}, {}, {}, {}, {} \n".format(timestamp,x_pos,y_pos,speed_x,speed_y,gnss_spd))


def heading_callback(msg):
    global heading
    heading0 = (2*math.pi-msg.data)+math.pi/2
    if heading0>math.pi:
        heading = heading0-math.pi*2
    else:
        heading = heading0
        


# def tf_callback(msg):
#     global x_pos,y_pos,heading
#     # 获取 base_footprint 到 map 的变换关系
#     for transform in msg.transforms:
#         if transform.header.frame_id =="map":
#             if transform.child_frame_id == "odom":
#                 x_pos = transform.transform.translation.x
#                 y_pos = transform.transform.translation.y
#                 quaternion=[
#                     transform.transform.rotation.x,
#                     transform.transform.rotation.y,
#                     transform.transform.rotation.z,
#                     transform.transform.rotation.w]
#                 (roll,pitch,yaw)=euler_from_quaternion(quaternion)
#                 heading = yaw

x_acc = 0.0 # imu测得的x向加速度   
y_acc = 0.0 # imu测得的y向加速度
z_acc = 0.0 # imu测得的z向加速度
                
def imu_callback(msg):   
    global x_acc,y_acc,z_acc
    x_acc = msg.linear_acceleration.x
    y_acc = msg.linear_acceleration.y
    z_acc = msg.linear_acceleration.z

ref_head = 0.0  #rad
ref_vel = 1.50 #m/s
pose1_x = 0.0
pose1_y = 0.0
vel_true =0.0 #m/s 
WHEEL_RADIUS = 0.265 #m 轮子周长
acc_cmd = 0.0 #全局油门控制指令
acc_limit_global = 1.0 
acc_limit = 0.1 
delta_cmd = 0.0 #全局转角控制指令

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
    # ref_head = math.pi/2+0.21

def calculate_yaw_error(yaw, yaw_ref):
    (thref_x,thref_y)=rotation(np.cos(yaw_ref),np.sin(yaw_ref),-yaw)
    yaw_error =np.arctan2(-thref_y,thref_x)
    
    return yaw_error

def Steering_and_Speed_Ctrl(x_pos,y_pos):
    global delta_cmd
    global ref_head,heading,pose1_x,pose1_y,vel_true,ref_vel,acc_cmd,acc_limit
    lat_err = -(x_pos-pose1_x)*np.sin(ref_head)+(y_pos-pose1_y)*np.cos(ref_head)
    head_err = calculate_yaw_error(heading, ref_head)*0.5*2
    global pid_controller;vel_pub;pid_controller_vel
    # acc_cmd=0.17 #油门开度
    acc_cmd += pid_controller_vel.update(np.real(ref_vel**2.1),np.real(vel_true**2.1))

    if acc_cmd >= acc_limit:  #油门限制幅度
        acc_cmd = acc_limit
    elif acc_cmd < 0:
        acc_cmd = 0

    vel_cmd=Twist()
    vel_cmd.linear.x=acc_cmd
    angular_z_cmd=pid_controller.update(0,lat_err+head_err)
    delta_cmd = angular_z_cmd
    vel_cmd.angular.z=round(angular_z_cmd,2)
    vel_pub.publish(vel_cmd)
    rospy.loginfo("转向角控制指令= %.2f °, 油门控制指令= %.2f ",vel_cmd.angular.z,vel_cmd.linear.x)
    rospy.logwarn("轨迹跟踪横向误差= %.3f m",lat_err)
    rospy.logwarn("航向角误差= %.3f m",head_err)
    rospy.loginfo("当前朝向 %.2f,轨迹朝向 %.2f,当前速度 %.2f,预期速度 %.2f,导航速度 %.2f",
                  heading*180/math.pi,ref_head*180/math.pi,vel_true,ref_vel,gnss_spd)

def rate_callback(msg):
    global vel_true,acc_limit,acc_limit_global
    vel_true = (msg.data/360)*WHEEL_RADIUS*0.04
    acc_limit = np.min([acc_limit_global,vel_true*0.56+0.15])
    # # 保存到文本文件
    # timestamp = rospy.Time.now()
    # with open('rate_data231103001.csv', 'w') as file:       
    #     file.write("{}, {} \n".format(timestamp,vel_true))
    

def log_publishing():
    global x_pos,y_pos,heading,vel_true
    global delta_cmd,acc_cmd
    global x_acc,y_acc,z_acc
    global log_pub
    time_stamp = rospy.Time.now()
    log_str = (str(time_stamp)+','+str(x_pos)+','+str(y_pos)+','+str(heading)+','
               +str(vel_true)+','+str(delta_cmd)+','+str(acc_cmd)+','
               +str(x_acc)+','+str(y_acc)+','+str(z_acc))
    log_msg = String()
    log_msg.data=log_str
    log_pub.publish(log_msg)




if __name__ =="__main__":
    rospy.init_node("tracking_node")
    odom_sub=rospy.Subscriber("/odom_message",Odometry,odom_callback,queue_size=1)
    heading_sub=rospy.Subscriber("/heading_message",Float32,heading_callback,queue_size=1)
    imu_sub=rospy.Subscriber("/imu_message",Imu,imu_callback,queue_size=1)
    path_sub=rospy.Subscriber("/ref_path",Path,path_callback,queue_size=1)
    rate_sub=rospy.Subscriber("/wheel_rate",Float32,rate_callback,queue_size=1)
    # tf_sub=rospy.Subscriber("/tf",TFMessage,tf_callback,queue_size=1)
    vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=3)
    start_time=time.time()
    # Listener = tf.TransformListener()
    
    log_pub=rospy.Publisher("/log_msg",String,queue_size=1)
    
    
    delta_limit=9 #deg
    pid_controller = PID(kp=15.5, ki=0.1, kd=40.5,
                         output_min=-delta_limit,
                         output_max=delta_limit,
                         integral_min=-0.1*delta_limit,
                         integral_max=0.1*delta_limit)
    rate = rospy.Rate(20)  # 设置发布频率

    dacc_limit=0.005 #油门开度增量限制
    pid_controller_vel = PID(kp=0.6, ki=0.00, kd=1.9,
                         output_min=-dacc_limit,
                         output_max=dacc_limit,
                         integral_min=-50*dacc_limit,
                         integral_max=50*dacc_limit)

    try:
        while not rospy.is_shutdown():

            Steering_and_Speed_Ctrl(x_pos,y_pos)
            log_publishing()
            rate.sleep()
    finally:
        vel_cmd=Twist()
        vel_pub.publish(vel_cmd)