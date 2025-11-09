#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
import math
import numpy as np

# 双移线函数（保持不变）
def generate_double_spline_trajectory(point_interval, traj_length, lateral_distance, rotation_angle=0):
    # t = np.linspace(0, 2 * np.pi, discrete_degree)  # 参数化曲线的参数
    # 参考双移线轨迹生成
    # 参数赋值
    shape=15.5 #%整体转向剧烈程度
    dx1=5.0   #%始程平缓程度，越大越平缓
    dx2=5.0   #%控制回程平缓程度，越大越平缓
    dy1=lateral_distance    #%控制换道开始y向位置
    dy2=lateral_distance    #%控制换道结束y向位置
    Xs1=1.0  #%控制换道开始距离
    Xs2=4.0 #%控制换道结束距离
    x=np.linspace(0,traj_length,round(traj_length/point_interval)+1) #%点的数量根据纵向速度x_dot决定

    # 生成参考轨迹
    z1=shape/dx1*(x-Xs1)-shape/2
    z2=shape/dx2*(x-Xs2)-shape/2
    y=dy1/2.*(1+np.tanh(z1))-dy2/2.*(1+np.tanh(z2))

    # 对轨迹进行旋转
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                               [np.sin(rotation_angle), np.cos(rotation_angle)]])
    rotated_points = np.dot(rotation_matrix, np.vstack((x, y)))

    x_rotated = rotated_points[0]
    y_rotated = rotated_points[1]

    return x_rotated, y_rotated

# 配置参数
direction_tan = 0.085  # m
traj_name = 'DLC'  # 可以是DLC或者Line

if traj_name == 'DLC':
    short_path_point_number = 100
    point_interval = 0.1  # 散点轨迹离散程度m
    traj_length = 30      # 双移线的长度m
    lateral_distance = -0.5  # 横向移动距离m
    rotation_angle = math.atan(direction_tan)  # 旋转角度rad

    x, y = generate_double_spline_trajectory(point_interval, traj_length, lateral_distance, rotation_angle=rotation_angle)
    refpath_x = x
    refpath_y = y
    
elif traj_name == 'Line':
    path_step = 0.1
    short_path_point_number = 100
    t = np.arange(0, 2000, path_step)
    refpath_x = t * 1
    refpath_y = t * direction_tan

# 存储完整的全局路径（在map坐标系下）
global_refpath_x = refpath_x.copy()
global_refpath_y = refpath_y.copy()

def odom_callback(msg):
    # 这个回调现在只用于日志记录，不用于路径生成
    pass

def tf_callback(msg):
    # 这个回调现在只用于日志记录，不用于路径生成
    pass

def get_fixed_global_path():
    """返回固定的全局路径，不随小车移动"""
    # 创建Path消息对象
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'  # 关键修改：使用map坐标系
    
    # 发布完整的全局路径或固定长度的路径
    point_number = min(short_path_point_number, len(global_refpath_x))
    
    for i in range(point_number):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'  # 关键修改：使用map坐标系
        pose.pose.position.x = global_refpath_x[i]
        pose.pose.position.y = global_refpath_y[i]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    
    return path

if __name__ == "__main__":
    rospy.init_node("ref_path_pub_node")
    path_pub = rospy.Publisher("/ref_path", Path, queue_size=1)
    
    # 由于路径是固定的，我们不需要实时更新位置信息
    # 但为了保持代码结构，可以保留订阅（可选）
    use_tf = False
    
    if use_tf:
        tf_sub = rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=1)
    else:
        odom_sub = rospy.Subscriber("/scanmatch_odom", Odometry, odom_callback, queue_size=1)
    
    rate = rospy.Rate(10)  # 设置发布频率
    
    # 预先计算全局路径
    global_path = get_fixed_global_path()
    
    while not rospy.is_shutdown():
        # 直接发布预先计算好的全局路径
        global_path.header.stamp = rospy.Time.now()  # 更新时间戳
        path_pub.publish(global_path)
        rospy.loginfo_once("发布固定的全局路径，包含 %d 个点", len(global_path.poses))
        rate.sleep()