#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############程序说明：ACC_Target_Detector_node#################
# 节点订阅激光雷达的SCAN消息，并根据点云数据做判断，输出3个信息：
# （3个话题）：话题名称         类型         含义
#            ACC_isexistPV    Bool      前车是否存在（ACC模式的前提）
#            ACC_PV_dv       Float32    相对速度，vp-vf，前车速度减去本车速度
#            ACC_PV_d        Float32     车间距，前车-本车，永远>=0
# 需要用到相关信息的时候直接订阅上述话题即可
# Author: Komasa Qi
# Institute: School of Vehicle and Mobility, Tsinghua Univ.
# Created at 2023/12/01
# Maintained at 2023/12/01

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np

########################变量定义##############################
# 定义话题发布全局变量
isexistPV = False   # 初始假设不存在PV
PV_dv = 0.0         # 初始假设PV的距离变化速度为0.0m/s
PV_d = 10.0         # 初始假设PV的距离为 10.0m

PV_dv0 = 0.0        # 存储上一个时刻有效的相对速度

# 定义目标判断全局变量

target_counter = 0    # 目标初步存在的帧数
loss_counter = 0      # 目标丢失帧数
target_min_exit_frame_num = 6 # 识别为目标至少需要存在的帧数
target_min_vanish_frame_num = 6 # 认为目标丢失至少需要消失的帧数

#######################参数调节################################
# 定义识别目标的算法参数
angle_dev = 5.0    #需要左右偏移25°的激光雷达扫描范围 #修改过啦
PV_d_maxlim = 8.0    #前车最远距离：m
PV_d_minlim = 0.05   #前车最近距离：m
previous_time = 0.0 #上次测量的时间
max_continuous_count = 25  # 最小连续点数量阈值,大于这个阈值才会识别为车辆
continuous_point_max_interval=0.02 # 连续的点应满足的最大距离差m



def scan_callback(msg):
    global isexistPV,PV_d,PV_dv,PV_dv0
    global angle_dev
    global PV_d_maxlim,PV_d_minlim
    global previous_time
    global max_continuous_count,continuous_point_max_interval
    global target_counter,loss_counter
    global target_min_exit_frame_num,target_min_vanish_frame_num
    # 在这里编写处理scan消息的代码
    #######################原始数据获取########################
    # 从scan_msg中提取所需的数据并进行处理
    deviation = int(round(angle_dev/(360/len(msg.ranges))))
    # 提取指定范围内的激光点
    ranges1 = msg.ranges[0:deviation]
    ranges2 = msg.ranges[-deviation:-1]
    selected_ranges = np.concatenate((ranges2,ranges1))

    # 转换激光点到laser坐标系下的x,y表示
    angles_all = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
    angles1 = angles_all[0:deviation]
    angles2 = angles_all[-deviation:-1]
    angles = np.concatenate((angles2,angles1))
    angles = angles[np.isfinite(selected_ranges)] # 获取有效目标点的角度信息
    selected_ranges=selected_ranges[np.isfinite(selected_ranges)] #获取有效目标点的距离信息
    x_values = (selected_ranges * np.cos(angles)) #获取有效目标点的X信息
    y_values = (selected_ranges * np.sin(angles)) #获取有效目标点的Y信息
    
    
    #######################目标判断##############################
    current_time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
    PV_d0 =PV_d  # 记录上时刻的距离
    # PV_d = np.mean(selected_ranges)  # 假设PV的距离为10.0
    # 初始化变量
    continuous_count = 0  # 连续点计数器
    min_distance = float('inf')  # 初始化最小距离为正无穷大
    min_distance_target = float('inf')  # 初始化最小距离为正无穷大

    for i in range(len(selected_ranges)):
        if abs(selected_ranges[i] - selected_ranges[i-1]) <= continuous_point_max_interval:
            continuous_count += 1
            min_distance_target = min(min_distance_target,selected_ranges[i])
        else:
            continuous_count = 1
            min_distance_target = float('inf')
        
        if continuous_count >= max_continuous_count:
            min_distance = min(min_distance_target, min_distance)

    

    if (min_distance < PV_d_maxlim) & (min_distance>= PV_d_minlim):
        # 如果这个测到的最小距离满足距离约束，认为本帧检测到一个疑似目标
        target_counter += 1
        target_counter = min(target_counter,255) #避免这个值过大溢出造成错误
        if target_counter >= target_min_exit_frame_num: 
            #如果连续多帧检测到这个目标，那么认为是真的目标
            isexistPV = True  # 假设存在PV
            loss_counter = 0  # 认为获得目标，清空丢失帧数
            PV_d = min_distance
            PV_dv = (PV_d - PV_d0)/(current_time-previous_time)
            if PV_dv < -5.0: # 设置相对速度阈值，避免刚识别到目标时相对速度超大
                PV_dv = 0.0
        # 否则假定目标还没有，还维持原来的设置
            
    else:
        loss_counter += 1
        loss_counter = min(loss_counter,255) #避免这个值过大溢出造成错误
        if loss_counter >= target_min_vanish_frame_num:
            target_counter = 0 # 认为目标丢失，清空捕获帧数
            isexistPV = False  # 不存在PV
            PV_dv = 0.0         # 初始假设PV的距离变化速度为0.0m/s
            PV_d = 10.0         # 初始假设PV的距离为 10.0m  
        # 否则假定目标还在，还维持原来的设置
            
    if isexistPV:
        PV_dv = (PV_dv0 + PV_dv)/2
        PV_dv0 = PV_dv
    previous_time = current_time

    
    ####################发布话题################################
    # 发布Boolean格式的话题 /ACC_isexistPV
    isexistPV_pub.publish(isexistPV)

    # 发布Float32格式的话题 /ACC_PV_dv
    PV_dv_pub.publish(PV_dv)

    # 发布Float32格式的话题 /ACC_PV_d
    PV_d_pub.publish(PV_d)
    rospy.loginfo("前车存在= %s, 前车距离= %.2f m, 相对速度= %.2f m/s", str(isexistPV), PV_d, PV_dv)





if __name__ == '__main__':
    rospy.init_node('ACC_Target_Detector_node')  # 初始化ROS节点

    # 创建订阅scan消息的订阅者
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    # 创建发布Boolean格式话题的发布者
    isexistPV_pub = rospy.Publisher('/ACC_isexistPV', Bool, queue_size=1)

    # 创建发布Float32格式话题的发布者
    PV_dv_pub = rospy.Publisher('/ACC_PV_dv', Float32, queue_size=1)
    PV_d_pub = rospy.Publisher('/ACC_PV_d', Float32, queue_size=1)

    rospy.spin()  # 进入ROS循环，等待消息
