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
import math
# import time
import numpy as np
# from MatlabFunctions import interp1

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
direction_tan = 0.085 #m
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
    path_step=0.1
    short_path_point_number=100
    t=np.arange(0,2000,path_step)
    # refpath_x=(np.cos(t)+np.cos(4*t)*0.05)*1.3+0.5
    refpath_x=t*1
    # refpath_y=np.sin(2*math.pi/10*t)*0.1
    refpath_y=t*direction_tan
    # refpath_x=(np.sin(t)*0.5)

x_pos=0.0
y_pos=0.0

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
  
    

if __name__ =="__main__":
    rospy.init_node("ref_path_pub_node")
    path_pub=rospy.Publisher("/ref_path",Path,queue_size=1)
    tf_sub=rospy.Subscriber("/tf",TFMessage,tf_callback,queue_size=1)
    rate = rospy.Rate(10)  # 设置发布频率
    odom_sub=rospy.Subscriber("/odom_message",Odometry,odom_callback,queue_size=1)

    while not rospy.is_shutdown():
        path=find_next_short_refpath(refpath_x,refpath_y,
                    x_pos,y_pos,short_path_point_number)
        path_pub.publish(path)  # 发布消息
        rospy.loginfo("发布了一条轨迹")
        rate.sleep()
