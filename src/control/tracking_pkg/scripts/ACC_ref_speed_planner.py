#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
from std_msgs.msg import Float32
from PIDController import PIDController as PID
import time
import numpy as np
from std_msgs.msg import Bool

#****************参数设置区************************
ddes = 1.6 # 目标距离 m
w_d = 0.01
w_dv = 0.05
max_ref_vel = 0.85 #m/s 最大参考速度
#*********************ACC消息接收***********************
# 全局变量初始化
ACC_PV_exist=False
ACC_PV_d = 10.0
ACC_PV_dv = 0.0

acc_refspd=0.0

# 笑景修改2023.12.21
#******************************************************
# 全局TTC time to clision 碰撞预计时间
TTC_lim = 2 #s 不允许小于这个值，否则就让参考速度为-0.05；
TTC_true = 10 #s 实际的TTC，会被后面的程序更新，这里是初值
TTC_max = 5 #设置一个TTC上限，超出了会饱和，方便画图展示
ifEmergBrk = False # 是否紧急制动的标志
TTC_counter = 0 # 连续多少次检测到TTC小于阈值的计数器
AEBThrshld = 2 # 连续多少次检测到TTC小于阈值就会制动的阈值
#*****************************************************



def acc_target_exist_callback(msg):
    global ACC_PV_exist
    ACC_PV_exist = msg.data
    

def acc_target_d_callback(msg):
    global ACC_PV_d
    ACC_PV_d = msg.data


def acc_target_dv_callback(msg):
    global ACC_PV_dv
    ACC_PV_dv = msg.data
#*******************************************************

#**************接收到ACC_消息后的参考速度设置*************
def ACC_ref_speed_Calculator():
    global acc_ref_speed_pub
    global ACC_PV_exist,ACC_PV_d,ACC_PV_dv
    global pid_controller_refvel
    global w_d,w_dv
    global max_ref_vel
    global acc_refspd,ddes
    
    # 笑景修改2023.12.21 增加TTC功能*******
    global TTC_lim,TTC_max,TTC_true,TTC_counter,ifEmergBrk,AEBThrshld
    # 计算TTC
    TTC_true = -ACC_PV_d/(ACC_PV_dv+0.0001)
    if TTC_true > TTC_max:
        TTC_true = TTC_max
    elif TTC_true < 0:
        TTC_true = 0
    
    
    if (TTC_true <= TTC_lim) & (TTC_true > 0):
        TTC_counter += 1
        if TTC_counter >= AEBThrshld:
            ifEmergBrk = True
            # rospy.logerr("TTC= %.2f s",TTC_true)
    elif TTC_true > TTC_lim:
        TTC_counter = 0
        ifEmergBrk = False
    #*************************************
    
    if ACC_PV_exist:
        acc_refspd += pid_controller_refvel.update(0,(-w_d*(ACC_PV_d-ddes)-w_dv*ACC_PV_dv)) 
        # 对参考速度进行限幅
        if acc_refspd > max_ref_vel:
            acc_refspd = max_ref_vel
        elif acc_refspd < 0:
            acc_refspd = 0
            
        
    else:
        acc_refspd = 0.0
    
    if ifEmergBrk:  # 笑景修改2023.12.21 加入AEB
        acc_refspd = -0.05
    
    
    acc_ref_speed_pub.publish(acc_refspd)
    rospy.logwarn("ACC_参考速度= %.2f m/s",acc_refspd)



#*******************************************************


if __name__ =="__main__":
    
    rospy.init_node("ACC_ref_speed_planner")

    acc_target_exist_sub=rospy.Subscriber("/ACC_isexistPV",Bool,acc_target_exist_callback,queue_size=1)
    acc_target_dv_sub=rospy.Subscriber("/ACC_PV_dv",Float32,acc_target_dv_callback,queue_size=1)
    acc_target_d_sub=rospy.Subscriber("/ACC_PV_d",Float32,acc_target_d_callback,queue_size=1)
    acc_ref_speed_pub=rospy.Publisher("/ref_speed",Float32,queue_size=1)
    # Listener = tf.TransformListener()

    publish_rate = 10 #Hz
    
    dvref_limit=0.3 #参考速度增量限制
    pid_controller_refvel = PID(kp=0.2, ki=0.4, kd=0.5,
                         output_min=-dvref_limit,
                         output_max=dvref_limit,
                         integral_min=-3*dvref_limit,
                         integral_max=3*dvref_limit)
    
    rate = rospy.Rate(publish_rate)  # 设置发布频率

    try:
        while not rospy.is_shutdown():

            ACC_ref_speed_Calculator()
            rate.sleep()
    finally:
        a=0








