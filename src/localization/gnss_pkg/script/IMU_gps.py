#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import argparse

# import numpy as np
import sys

import serial  # 导入模块
import serial.tools.list_ports
import threading
import struct
import time
import platform
# import navi.msgs

import rospy
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
# from copy import deepcopy
# import sys
# import os
import math
import numpy as np
import tf
import tf2_ros

from geometry_msgs.msg import Quaternion
from serial.serialutil import EIGHTBITS, PARITY_NONE, STOPBITS_ONE




CONSTANTS_RADIUS_OF_EARTH = 6371000. 
# 宏定义参数
PI = 3.1415926
FRAME_HEAD = str('fc')
FRAME_END = str('fd')
TYPE_IMU = str('40')
TYPE_AHRS = str('41')
TYPE_INSGPS = str('42')
TYPE_GEODETIC_POS = str('5c')
TYPE_GROUND = str('f0')
TYPE_SYS_STATE = str('50')
IMU_LEN = str('38')  # //56
AHRS_LEN = str('30')  # //48
INSGPS_LEN = str('48')  # //80
GEODETIC_POS_LEN = str('20')  # //32
SYS_STATE_LEN = str('64')  # // 64
PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295
isrun = True


# 获取命令行输入参数
def parse_opt(known=False):
    parser = argparse.ArgumentParser()
    # parser.add_argument('--debugs', type=bool, default=False, help='if debug info output in terminal ')
    parser.add_argument('--port', type=str, default='/dev/imu_gnss', help='the models serial port receive data; example: '
                                                                 '    Windows: COM3'
                                                                 '    Linux: /dev/ttyUSB0')

    parser.add_argument('--bps', type=int, default=230400, help='the models baud rate set; default: 921600')
    parser.add_argument('--timeout', type=int, default=20, help='set the serial port timeout; default: 20')
    # parser.add_argument('--device_type', type=int, default=0, help='0: origin_data, 1: for single imu or ucar in ROS')

    receive_params = parser.parse_known_args()[0] if known else parser.parse_args()
    return receive_params


def GPStoXY(lat, lon, ref_lat, ref_lon):
        # input GPS and Reference GPS in degrees
        # output XY in meters (m) X:North Y:East
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

        return x, y


# 接收数据线程
def receive_data():
    open_port()
    pub_odom=rospy.Publisher('odom_message',Odometry,queue_size=1)
    pub_imu=rospy.Publisher('imu_message',Imu,queue_size=1)
    pub_roll=rospy.Publisher('roll_message',Float32,queue_size=1)
    pub_pitch=rospy.Publisher('pitch_message',Float32,queue_size=1)
    pub_heading=rospy.Publisher('heading_message',Float32,queue_size=1)
    pub_gps=rospy.Publisher('gps_message', NavSatFix,queue_size=1) 
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) 

    
    
    # 尝试打开串口
    try:
        serial_ = serial.Serial(port=opt.port, baudrate=opt.bps, bytesize=EIGHTBITS, parity=PARITY_NONE,
                                stopbits=STOPBITS_ONE,
                                timeout=opt.timeout)
        print("baud rates = " + str(serial_.baudrate))
    except:
        print("error:  unable to open port .")
        exit(1)
  
    # 循环读取数据
    # while serial_.isOpen() and tr.is_alive() :
    while serial_.isOpen() and not rospy.is_shutdown() :
        # rbdata = ser.readline()
        # rbdata = ser.read_all()
        
        # if len(rbdata) != 0:
        #     rxdata = rbdata.hex()
        #     print(rxdata)
        # if not threading.main_thread().is_alive():
        #     print('done')
        #     break
        check_head = serial_.read().hex()
        # 校验帧头
        if check_head != FRAME_HEAD:
            continue
        head_type = serial_.read().hex()
        # 校验数据类型
        if (head_type != TYPE_IMU and head_type != TYPE_AHRS and head_type != TYPE_INSGPS and
                head_type != TYPE_GEODETIC_POS and head_type != 0x50 and head_type != TYPE_GROUND and
                head_type != TYPE_SYS_STATE):
            continue
        check_len = serial_.read().hex()
        # 校验数据类型的长度
        if head_type == TYPE_IMU and check_len != IMU_LEN:
            continue
        elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
            continue
        elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
            continue
        elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
            continue
        elif head_type == TYPE_SYS_STATE and check_len != SYS_STATE_LEN:
            continue
        elif head_type == TYPE_GROUND or head_type == 0x50:
            continue
        check_sn = serial_.read().hex()
        head_crc8 = serial_.read().hex()
        crc16_H_s = serial_.read().hex()
        crc16_L_s = serial_.read().hex()

        # 读取并解析IMU数据
        if head_type == TYPE_IMU:
            data_s = serial_.read(int(IMU_LEN, 16))
            print("Gyroscope_X(rad/s): " + str(struct.unpack('f', data_s[0:4])[0]))
            print("Gyroscope_Y(rad/s) : " + str(struct.unpack('f', data_s[4:8])[0]))
            print("Gyroscope_Z(rad/s) : " + str(struct.unpack('f', data_s[8:12])[0]))
            print("Accelerometer_X(m/s^2) : " + str(struct.unpack('f', data_s[12:16])[0]))
            print("Accelerometer_Y(m/s^2) : " + str(struct.unpack('f', data_s[16:20])[0]))
            print("Accelerometer_Z(m/s^2) : " + str(struct.unpack('f', data_s[20:24])[0]))
            # print("Magnetometer_X(mG) : " + str(struct.unpack('f', data_s[24:28])[0]))
            # print("Magnetometer_Y(mG) : " + str(struct.unpack('f', data_s[28:32])[0]))
            # print("Magnetometer_Z(mG) : " + str(struct.unpack('f', data_s[32:36])[0]))
            # print("IMU_Temperature : " + str(struct.unpack('f', data_s[36:40])[0]))
            # print("Pressure : " + str(struct.unpack('f', data_s[40:44])[0]))
            # print("Pressure_Temperature : " + str(struct.unpack('f', data_s[44:48])[0]))
            # print("Timestamp(us) : " + str(struct.unpack('ii', data_s[48:56])[0]))
        # 读取并解析AHRS数据
        elif head_type == TYPE_AHRS:
            data_s = serial_.read(int(AHRS_LEN, 16))
            print("RollSpeed(rad/s): " + str(struct.unpack('f', data_s[0:4])[0]))
            print("PitchSpeed(rad/s) : " + str(struct.unpack('f', data_s[4:8])[0]))
            print("HeadingSpeed(rad) : " + str(struct.unpack('f', data_s[8:12])[0]))
            print("Roll(rad) : " + str(struct.unpack('f', data_s[12:16])[0]))
            print("Pitch(rad) : " + str(struct.unpack('f', data_s[16:20])[0]))
            print("Heading(rad) : " + str(struct.unpack('f', data_s[20:24])[0]))
            print("Q1 : " + str(struct.unpack('f', data_s[24:28])[0]))
            print("Q2 : " + str(struct.unpack('f', data_s[28:32])[0]))
            print("Q3 : " + str(struct.unpack('f', data_s[32:36])[0]))
            print("Q4 : " + str(struct.unpack('f', data_s[36:40])[0]))
            # print("Timestamp(us) : " + str(struct.unpack('ii', data_s[40:48])[0]))
        # 读取并解析INSGPS数据
        elif head_type == TYPE_INSGPS:
            data_s = serial_.read(int(INSGPS_LEN, 16))
            print("BodyVelocity_X:(m/s)" + str(struct.unpack('f', data_s[0:4])[0]))
            print("BodyVelocity_Y:(m/s)" + str(struct.unpack('f', data_s[4:8])[0]))
            print("BodyVelocity_Z:(m/s)" + str(struct.unpack('f', data_s[8:12])[0]))
            print("BodyAcceleration_X:(m/s^2)" + str(struct.unpack('f', data_s[12:16])[0]))
            print("BodyAcceleration_Y:(m/s^2)" + str(struct.unpack('f', data_s[16:20])[0]))
            print("BodyAcceleration_Z:(m/s^2)" + str(struct.unpack('f', data_s[20:24])[0]))
            print("Location_North:(m)" + str(struct.unpack('f', data_s[24:28])[0]))
            print("Location_East:(m)" + str(struct.unpack('f', data_s[28:32])[0]))
            print("Location_Down:(m)" + str(struct.unpack('f', data_s[32:36])[0]))
            print("Velocity_North:(m)" + str(struct.unpack('f', data_s[36:40])[0]))
            print("Velocity_East:(m/s)" + str(struct.unpack('f', data_s[40:44])[0]))
            print("Velocity_Down:(m/s)" + str(struct.unpack('f', data_s[44:48])[0]))
            # print("Acceleration_North:(m/s^2)" + str(struct.unpack('f', data_s[48:52])[0]))
            # print("Acceleration_East:(m/s^2)" + str(struct.unpack('f', data_s[52:56])[0]))
            # print("Acceleration_Down:(m/s^2)" + str(struct.unpack('f', data_s[56:60])[0]))
            # print("Pressure_Altitude:(m)" + str(struct.unpack('f', data_s[60:64])[0]))
            # print("Timestamp:(us)" + str(struct.unpack('ii', data_s[64:72])[0]))
        # 读取并解析GPS数据
        elif head_type == TYPE_GEODETIC_POS:
            data_s = serial_.read(int(GEODETIC_POS_LEN, 16))
            print(" Latitude:(rad)" + str(struct.unpack('d', data_s[0:8])[0]))
            print("Longitude:(rad)" + str(struct.unpack('d', data_s[8:16])[0]))
            print("Height:(m)" + str(struct.unpack('d', data_s[16:24])[0]))
        elif head_type == TYPE_SYS_STATE:
            data_s = serial_.read(int(SYS_STATE_LEN, 16))
            print("System_status:" + str(struct.unpack('H', data_s[0:2])[0]))
            print("Filter_status:" + str(struct.unpack('H', data_s[2:4])[0]))
            
            Unix_time=struct.unpack('f', data_s[4:8])[0]
            Microseconds=struct.unpack('f', data_s[8:12])[0]
            lat=struct.unpack('d', data_s[12:20])[0]
            long=struct.unpack('d', data_s[20:28])[0]
            height=struct.unpack('d', data_s[28:36])[0]
            Velocity_north=struct.unpack('f', data_s[36:40])[0]
            Velocity_east=struct.unpack('f', data_s[40:44])[0]
            Velocity_down=struct.unpack('f', data_s[44:48])[0]
            Body_acceleration_X=struct.unpack('f', data_s[48:52])[0]
            Body_acceleration_Y=struct.unpack('f', data_s[52:56])[0]
            Body_acceleration_Z=struct.unpack('f', data_s[56:60])[0]
            Roll=struct.unpack('f', data_s[64:68])[0]
            Pitch=struct.unpack('f', data_s[68:72])[0]
            Heading=struct.unpack('f', data_s[72:76])[0]
            Angular_velocity_X=struct.unpack('f', data_s[76:80])[0]
            Angular_velocity_Y=struct.unpack('f', data_s[80:84])[0]
            Angular_velocity_Z=struct.unpack('f', data_s[84:88])[0]
            lat_degree=math.degrees(lat)
            long_degree=math.degrees(long)
            x,y=GPStoXY(lat_degree,long_degree,116.38,39.90)
            
            
            # 进行heading转换，东为0-PI~PI，北为 PI/2
            new_heading=-Heading+PI/2
            if new_heading<-PI:
                new_heading = new_heading + 2*PI 
            new_pitch=-Pitch
            new_roll=Roll
            
            
            odom_msg=Odometry()
            imu_msg=Imu()

            #odom
            odom_msg.header.stamp=rospy.Time.now()
            odom_msg.header.frame_id='ego_vehicle/odometry'
            odom_msg.child_frame_id='odom'
            odom_msg.pose.pose.position.x=x
            odom_msg.pose.pose.position.y=y
            odom_msg.pose.pose.position.z=height
            odom_msg.twist.twist.linear.x=Velocity_east*math.cos(new_heading)+Velocity_north*math.sin(new_heading) # 随体速度 纵向
            odom_msg.twist.twist.linear.y=Velocity_north*math.cos(new_heading)-Velocity_east*math.sin(new_heading) # 随体速度 横向
            odom_msg.twist.twist.linear.z=-Velocity_down # 我们要向上的速度
            odom_msg.twist.twist.angular.x=Angular_velocity_X
            odom_msg.twist.twist.angular.y=-Angular_velocity_Y # 转换方向
            odom_msg.twist.twist.angular.z=-Angular_velocity_Z # 转换方向
            #imu
            q = tf.transformations.quaternion_from_euler(new_roll, new_pitch, new_heading)
            imu_msg.header.stamp=rospy.Time.now()
            imu_msg.header.frame_id='ego_vehicle/imu'
            imu_msg.orientation.x=q[0]
            imu_msg.orientation.y=q[1]
            imu_msg.orientation.z=q[2]
            imu_msg.orientation.w=q[3]
            imu_msg.linear_acceleration.x=Body_acceleration_X
            imu_msg.linear_acceleration.y=-Body_acceleration_Y # 转换方向
            imu_msg.linear_acceleration.z=-Body_acceleration_Z # 转换方向
            imu_msg.angular_velocity.x=Angular_velocity_X
            imu_msg.angular_velocity.y=-Angular_velocity_Y # 转换方向
            imu_msg.angular_velocity.z=-Angular_velocity_Z # 转换方向
            
            #gps
            gps_msg=NavSatFix()
            gps_msg.header.stamp=rospy.Time.now()
            gps_msg.header.frame_id='ego_vehicle/gps'
            gps_msg.latitude=lat_degree
            gps_msg.longitude=long_degree
            gps_msg.altitude=height

            pub_odom.publish(odom_msg)
            pub_imu.publish(imu_msg)
            pub_roll.publish(new_roll) 
            pub_pitch.publish(new_pitch) # 转换方向 
            
            pub_heading.publish(new_heading) # 转换方向，东为0-PI~PI，北为 PI/2
            pub_gps.publish(gps_msg)


            rate.sleep()
            

            





            #print("Unix_time:" + str(struct.unpack('H', data_s[4:8])[0]))
            # print("Microseconds:" + str(struct.unpack('H', data_s[8:12])[0]))
            # print("Latitude:" + str(struct.unpack('f', data_s[12:20])[0]))
            # print("Longitude:" + str(struct.unpack('f', data_s[20:28])[0]))
            # print("Height:" + str(struct.unpack('f', data_s[28:36])[0]))
            # print("Velocity_north:" + str(struct.unpack('f', data_s[36:40])[0]))
            # print("Velocity_east:" + str(struct.unpack('f', data_s[40:44])[0]))
            # print("Velocity_down:" + str(struct.unpack('f', data_s[44:48])[0]))
            # print("Body_acceleration_X:" + str(struct.unpack('f', data_s[48:52])[0]))
            # print("Body_acceleration_Y:" + str(struct.unpack('f', data_s[52:56])[0]))
            # print("Body_acceleration_Z:" + str(struct.unpack('f', data_s[56:60])[0]))
            # print("G_force:" + str(struct.unpack('f', data_s[60:64])[0]))
            # print("Roll:" + str(struct.unpack('f', data_s[64:68])[0]))
            # print("Pitch:" + str(struct.unpack('f', data_s[68:72])[0]))
            # print("Heading:" + str(struct.unpack('f', data_s[72:76])[0]))
            # print("Angular_velocity_X:" + str(struct.unpack('f', data_s[76:80])[0]))
            # print("Angular_velocity_Y:" + str(struct.unpack('f', data_s[80:84])[0]))
            # print("Angular_velocity_Z:" + str(struct.unpack('f', data_s[84:88])[0]))
            # print("Latitude_standard_deviation:" + str(struct.unpack('f', data_s[88:92])[0]))
            # print("Longitude_standard_deviation:" + str(struct.unpack('f', data_s[92:96])[0]))
            # print("Height_standard_deviation:" + str(struct.unpack('f', data_s[96:100])[0]))
            

# def talker():
#     pub = rospy.Publisher('chatter', Odometry, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# 寻找输入的port串口
def find_serial():
    port_list = list(serial.tools.list_ports.comports())
    for port in port_list:
        if port.device == opt.port:
            return True
    return False


def open_port():
    if find_serial():
        print("find this port : " + opt.port)
    else:
        print("error:  unable to find this port : " + opt.port)
        exit(1)


def UsePlatform():
    sys_str = platform.system()
    if sys_str == "Windows":
        print("Call Windows tasks")
    elif sys_str == "Linux":
        print("Call Linux tasks")
    else:
        print("Other System tasks: %s" % sys_str)
    return sys_str


if __name__ == "__main__":
    print(UsePlatform())
    opt = parse_opt()
    receive_data()
    # tr = threading.Thread(target=receive_data)
    # tr.start()
    # while True:
    #     try:
    #         if tr.is_alive():
    #             time.sleep(1)
    #         else:
    #             break
    #     except(KeyboardInterrupt, SystemExit):
    #         break



    
