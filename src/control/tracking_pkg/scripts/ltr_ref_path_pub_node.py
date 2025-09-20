#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# from tf.transformations import euler_from_quaternion
# from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32
import math
# import time
import numpy as np
from PathGenLib import Test_Circle_Gen
# from MatlabFunctions import interp1

# TODO 待验证 
######################################################################################
#####################################变量定义#########################################
######################################################################################
#---------------------------------车辆状态量------------------------------------------
# heading=0.0 # 航向角初始化 rad
# dheading = 0.0  # 撗擺角速度 rad/s
x_pos=0.0   # X位置初始化 m
y_pos=0.0   # Y位置初始化 m
vel_true =0.0 # 轮速计测量速度 m/s 

#---------------------------------轨迹相关量------------------------------------------
######位置标定偏差###相对天安门########
x_dev = -3447246.0 #m
y_dev = -5747461.0 #m
traj_name = 'Test_smooth' #可以是DLC或者Line或者Test_sharp或Test_Smooth
print(traj_name)

#---------------------------------车辆几何参数量------------------------------------------
WHEEL_RADIUS = 0.265 #m 轮子周长

#---------------------------------控制器相关量------------------------------------------
Np = 25    # 预测时域步数 记得调整！！！！#############################################################!!!!!!!!!!!!!!!!!!--Np--!!!!!!!!!!!!!!记得改！！！！！！
Ts = 0.05  # 控制器步长，不是本轨迹发布节点的步长喔

def generate_double_spline_trajectory(point_interval, traj_length, lateral_distance, rotation_angle=0):
    # t = np.linspace(0, 2 * np.pi, discrete_degree)  # 参数化曲线的参数
    # 参考双移线轨迹生成
    # 参数赋值
    shape=15.5  #%整体转向剧烈程度
    dx1=5.0   #%始程平缓程度，越大越平缓
    dx2=5.0   #%回程平缓程度，越大越平缓
    dy1=lateral_distance    #%控制换道开始y向位置
    dy2=lateral_distance    #%控制换道结束y向位置
    Xs1=1.0  #%控制换道开始距离
    Xs2=4.0 #%控制换道结束距离
    x=np.linspace(0,traj_length,round(traj_length/point_interval)+1); #%点的数量根据纵向速度x_dot决定

    # 生成参考轨迹
    z1=shape/dx1*(x-Xs1)-shape/2
    z2=shape/dx2*(x-Xs2)-shape/2
    y=dy1/2.*(1+np.tanh(z1))-dy2/2.*(1+np.tanh(z2))
    # heading=np.atan(dy1*(1./np.cosh(z1))**2*(1.2/dx1)-dy2*(1./np.cosh(z2))**2*(1.2/dx2))

    # 对轨迹进行旋转
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                               [np.sin(rotation_angle), np.cos(rotation_angle)]])
    rotated_points = np.dot(rotation_matrix, np.vstack((x, y)))

    x_rotated = rotated_points[0]
    y_rotated = rotated_points[1]

    return x_rotated, y_rotated


######位置标定偏差###相对天安门########
direction_tan = 0.000 #m
traj_name = 'Line' #可以是DLC或者Line


if traj_name == 'DLC':
    short_path_point_number=100
    point_interval = 0.1  # 散点轨迹离散程度m
    traj_length = 50      # 双移线的长度m
    lateral_distance = -0.5  # 横向移动距离m
    rotation_angle = math.atan(direction_tan)  # 旋转角度deg

    x, y = generate_double_spline_trajectory(point_interval, traj_length, lateral_distance, rotation_angle=rotation_angle)
    refpath_x = x
    refpath_y = y
    

elif traj_name == 'Line':
    point_interval=0.1
    short_path_point_number=100
    t=np.arange(0,2000,point_interval)
    # refpath_x=(np.cos(t)+np.cos(4*t)*0.05)*1.3+0.5
    refpath_x=t*1
    # refpath_y=np.sin(2*math.pi/10*t)*0.1
    refpath_y=t*direction_tan
    # refpath_x=(np.sin(t)*0.5)



def odom_callback(msg):
    global x_pos,y_pos
    x_pos = msg.pose.pose.position.x
    y_pos = -msg.pose.pose.position.y

    
def tf_callback(msg):
    global x_pos,y_pos
    # 获取 base_footprint 到 map 的变换关系
    for transform in msg.transforms:
        if transform.header.frame_id =="map":
            if transform.child_frame_id == "odom":
                x_pos = transform.transform.translation.x
                y_pos = transform.transform.translation.y


def find_next_short_refpath(refpath_x,refpath_y,x_pos,y_pos,point_number):
    datalen=len(refpath_x)
    point_number = int(point_number)
    
    path_x=np.array([float(0)]*point_number)#预先分配短轨迹的空间
    path_y=np.array([float(0)]*point_number)
    dist=np.array([float(0)]*datalen)
    for i in range(0,datalen):
        dist[i]=np.sqrt(np.square(refpath_x[i]-x_pos)
                       +np.square(refpath_y[i]-y_pos))
    idx=np.where(dist==min(dist)) #找到最近点的序号
    idx=idx[0][0]                 #refpath_x[idx]就是最近的点的x
    rospy.logwarn("当前点：(%.2f,%.2f) 最近的点：(%.2f,%.2f)",x_pos,y_pos,refpath_x[idx],refpath_y[idx])
    # 创建Path消息对象
    path = Path()
    # 设置header信息
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'
    for i in range(0,point_number):
        # path_x[i]=refpath_x[(idx+i)%datalen]
        # path_y[i]=refpath_y[(idx+i)%datalen]
        # 创建PoseStamped消息，并添加到poses数组中
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = refpath_x[(idx+i)%datalen]
        pose.pose.position.y = refpath_y[(idx+i)%datalen]
        # pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    # path=path_x.tolist+path_y.tolist
    return path
    
def rate_callback(msg):
    global vel_true,path_point_number,Np,Ts,point_interval
    vel_true = (msg.data/360)*WHEEL_RADIUS*0.04
    path_point_number = np.max([np.round((Np*Ts*vel_true)/point_interval),5]) + 1
    rospy.loginfo("当前发布的参考点的个数为= %d ",path_point_number)
    
    
    
if __name__ =="__main__":
    rospy.init_node("ref_path_pub_node")
    path_pub=rospy.Publisher("/ref_path",Path,queue_size=1)
    x_dev_pub=rospy.Publisher("/x_dev_const",Float32,queue_size=1)
    y_dev_pub=rospy.Publisher("/y_dev_const",Float32,queue_size=1)
    path_pub=rospy.Publisher("/ref_path",Path,queue_size=1)
    tf_sub=rospy.Subscriber("/tf",TFMessage,tf_callback,queue_size=1)
    rate = rospy.Rate(10)  # 设置发布频率
    rate_sub=rospy.Subscriber("/wheel_rate",Float32,rate_callback,queue_size=1)
    odom_sub=rospy.Subscriber("/odom_message",Odometry,odom_callback,queue_size=1)

    while not rospy.is_shutdown():
        path=find_next_short_refpath(refpath_x,refpath_y,
                    x_pos,y_pos,short_path_point_number)
        
        path_pub.publish(path)  # 发布消息
        x_dev_pub.publish(x_dev)  # 发布消息
        y_dev_pub.publish(y_dev)  # 发布消息
        rospy.loginfo("发布了一条轨迹")
        rate.sleep()
