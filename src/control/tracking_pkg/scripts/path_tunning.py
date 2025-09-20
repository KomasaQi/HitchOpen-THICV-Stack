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

#从路径消息中提取路径点，计算参考航向角
def path_callback(path_msg):
    # length=len(path.poses)
    print(len(path_msg.poses))
    x_dev = -3447246.0 #m
    y_dev = -5747461.0 #m
    Np = 3
    original_point_num = len(path_msg.poses)
    path_x0 = np.zeros(original_point_num)
    path_y0 = np.zeros(original_point_num)
    path_x = np.zeros(Np)
    path_y = np.zeros(Np)
    for i in range(original_point_num):
        # pose1_x=path_msg.poses[0].pose.position.x
        # pose1_y=path_msg.poses[0].pose.position.y
        # pose2_x=path_msg.poses[3].pose.position.x
        # pose2_y=path_msg.poses[3].pose.position.y    
        path_x0[i] = path_msg.poses[i].pose.position.x - x_dev
        path_y0[i] = path_msg.poses[i].pose.position.y - y_dev
        path_x = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),path_x0)
        path_y = np.interp(np.linspace(0,1,Np),np.linspace(0,1,original_point_num),path_y0)
        
    
    # ref_head=np.arctan2(pose2_y-pose1_y,pose2_x-pose1_x)
    # rospy.loginfo("当前点：(%.2f,%.2f) ",pose1_x,pose1_y)
    print("################################")
    print(path_x0,path_y0)
    print(path_x,path_y)
    
    # ref_head = math.pi/2+0.21

if __name__ =="__main__":
    rospy.init_node("path_tunning")
    path_sub=rospy.Subscriber("/ref_path",Path,path_callback,queue_size=1)



    
    publish_rate = 5 # Hz
    
   

    rate = rospy.Rate(publish_rate)  # 设置发布频率

    while not rospy.is_shutdown():
        rate.sleep()

