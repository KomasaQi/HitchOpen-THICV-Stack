#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import os
import serial
import serial.tools.list_ports
import threading
import struct
import time
import stat
import platform
import rospy
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import math
import numpy as np
import tf
from geometry_msgs.msg import Quaternion
from serial.serialutil import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

# 全局常量
CONSTANTS_RADIUS_OF_EARTH = 6371000.0
PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295
isrun = True

# 帧格式宏定义（保持原有）
FRAME_HEAD = 'fc'
FRAME_END = 'fd'
TYPE_IMU = '40'
TYPE_AHRS = '41'
TYPE_INSGPS = '42'
TYPE_GEODETIC_POS = '5c'
TYPE_GROUND = 'f0'
TYPE_SYS_STATE = '50'
IMU_LEN = '38'
AHRS_LEN = '30'
INSGPS_LEN = '48'
GEODETIC_POS_LEN = '20'
SYS_STATE_LEN = '64'

def GPStoXY(lat, lon, ref_lat, ref_lon):
    """GPS经纬度转平面XY坐标（北东向），参考坐标从ROS参数读取"""
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




def find_serial(target_port):
    """
    兼容修复：用 stat 模块判断字符设备，替代 os.path.ischarfile，避免兼容性问题
    :param target_port: 目标端口路径（如 /dev/imu_gps 或 /dev/ttyUSB0）
    :return: True=有效串口设备，False=无效
    """
    # 步骤1：检查路径是否存在
    if not os.path.exists(target_port):
        rospy.logerr(f"路径不存在: {target_port}")
        return False
    
    # 步骤2：解析软链接（获取实际设备路径）
    real_port = os.path.realpath(target_port)
    rospy.loginfo(f"解析端口路径：{target_port} → 实际设备：{real_port}")
    
    # 步骤3：用 stat 模块判断是否为字符设备（串口本质是字符设备）
    try:
        # 获取文件状态信息
        file_stat = os.stat(real_port)
        # 判断是否为字符设备（stat.S_IFCHR 是字符设备的标志）
        if not stat.S_ISCHR(file_stat.st_mode):
            rospy.logerr(f"实际设备 {real_port} 不是字符设备（非串口）")
            return False
    except Exception as e:
        rospy.logerr(f"获取设备状态失败：{str(e)}（可能是权限不足）")
        return False
    
    # 步骤4：额外校验：实际设备是否是 ttyUSB/ttyACM 类串口（可选，增强兼容性）
    if "ttyUSB" not in real_port and "ttyACM" not in real_port:
        rospy.logwarn(f"实际设备 {real_port} 可能不是串口（非 ttyUSB/ttyACM 类型），仍尝试连接...")
    
    return True

def open_port(port):
    """打开串口前的端口校验（使用修复后的 find_serial 函数）"""
    if find_serial(port):
        rospy.loginfo(f"✅ 找到有效串口设备: {port}（实际设备：{os.path.realpath(port)}）")
    else:
        rospy.logerr(f"❌ 未找到有效串口设备: {port}")
        # 额外输出系统端口列表，帮助用户排查
        port_list = list(serial.tools.list_ports.comports())
        rospy.logerr(f"系统识别的实际串口列表：{[p.device for p in port_list]}")
        sys.exit(1)

def UsePlatform():
    """判断系统类型（保持原有）"""
    sys_str = platform.system()
    if sys_str == "Windows":
        rospy.loginfo("🖥️  运行在Windows系统")
    elif sys_str == "Linux":
        rospy.loginfo("🐧 运行在Linux系统")
    else:
        rospy.loginfo(f"🔧 运行在其他系统: {sys_str}")
    return sys_str

def receive_data():
    """核心数据接收与发布函数（从ROS参数服务器读取配置）"""
    # -------------------------- 1. 从ROS参数服务器获取配置（支持YAML加载）--------------------------
    # 串口参数（默认值沿用原有配置）
    port = rospy.get_param('~port', '/dev/imu_gps')
    bps = rospy.get_param('~bps', 230400)
    timeout = rospy.get_param('~timeout', 20)
    
    # GPS转XY参考坐标（默认值沿用原有，支持YAML修改）
    ref_lat = rospy.get_param('~reference_latitude', 39.90)  # 参考纬度（度）
    ref_lon = rospy.get_param('~reference_longitude', 116.38) # 参考经度（度）
    
    # 话题名称配置（默认沿用原有，支持YAML自定义）
    odom_topic = rospy.get_param('~odom_topic', 'odom_message')
    imu_topic = rospy.get_param('~imu_topic', 'imu_message')
    roll_topic = rospy.get_param('~roll_topic', 'roll_message')
    pitch_topic = rospy.get_param('~pitch_topic', 'pitch_message')
    heading_topic = rospy.get_param('~heading_topic', 'heading_message')
    gps_topic = rospy.get_param('~gps_topic', 'gps_message')
    
    # 坐标系帧ID配置（默认沿用原有，支持YAML自定义）
    odom_header_frame = rospy.get_param('~odom_header_frame', 'ego_vehicle/odometry')
    odom_child_frame = rospy.get_param('~odom_child_frame', 'odom')
    imu_header_frame = rospy.get_param('~imu_header_frame', 'ego_vehicle/imu')
    gps_header_frame = rospy.get_param('~gps_header_frame', 'ego_vehicle/gps')

    # 打印加载的配置（便于调试）
    rospy.loginfo("="*50)
    rospy.loginfo("📌 加载的节点配置：")
    rospy.loginfo(f"  串口路径: {port}")
    rospy.loginfo(f"  波特率: {bps}")
    rospy.loginfo(f"  参考GPS: ({ref_lat}°N, {ref_lon}°E)")
    rospy.loginfo(f"  里程计话题: {odom_topic}")
    rospy.loginfo(f"  IMU话题: {imu_topic}")
    rospy.loginfo("="*50)

    # -------------------------- 2. 初始化串口和发布者 --------------------------
    open_port(port)  # 校验串口是否存在
    rate = rospy.Rate(100)  # 100Hz发布频率（保持原有）

    # 创建ROS发布者（使用配置的话题名称）
    pub_odom = rospy.Publisher(odom_topic, Odometry, queue_size=1)
    pub_imu = rospy.Publisher(imu_topic, Imu, queue_size=1)
    pub_roll = rospy.Publisher(roll_topic, Float32, queue_size=1)
    pub_pitch = rospy.Publisher(pitch_topic, Float32, queue_size=1)
    pub_heading = rospy.Publisher(heading_topic, Float32, queue_size=1)
    pub_gps = rospy.Publisher(gps_topic, NavSatFix, queue_size=1)

    # 打开串口
    try:
        serial_ = serial.Serial(
            port=port,
            baudrate=bps,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=timeout
        )
        rospy.loginfo(f"✅ 串口打开成功！波特率: {bps}")
    except Exception as e:
        rospy.logerr(f"❌ 串口打开失败: {str(e)}")
        sys.exit(1)

    # -------------------------- 3. 数据读取与解析（保持原有逻辑不变）--------------------------
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
            # print(" Latitude:(rad)" + str(struct.unpack('d', data_s[0:8])[0]))
            # print("Longitude:(rad)" + str(struct.unpack('d', data_s[8:16])[0]))
            # print("Height:(m)" + str(struct.unpack('d', data_s[16:24])[0]))
        elif head_type == TYPE_SYS_STATE:
            data_s = serial_.read(int(SYS_STATE_LEN, 16))
            # print("System_status:" + str(struct.unpack('H', data_s[0:2])[0]))
            # print("Filter_status:" + str(struct.unpack('H', data_s[2:4])[0]))
            
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
            rospy.loginfo_throttle(1.0, f"GPS - 纬度: {lat_degree:.6f}° | 经度: {long_degree:.6f}° | 高度: {height:.2f} m")

            rospy.loginfo_throttle(1.0, f"IMU - 陀螺仪: ({Angular_velocity_X:.3f}, {Angular_velocity_Y:.3f}, {Angular_velocity_Z:.3f}) rad/s | 加速度计: ({Body_acceleration_X:.3f}, {Body_acceleration_Y:.3f}, {Body_acceleration_Z:.3f}) m/s²")
            rospy.loginfo_throttle(1.0, f"AHRS - 姿态: ({Roll:.3f}, {Pitch:.3f}, {Heading:.3f}) rad ")
            

            rate.sleep()

if __name__ == "__main__":
    try:
        # 初始化ROS节点（必须在获取参数前初始化）
        rospy.init_node('imu_gps_node', anonymous=True)
        UsePlatform()  # 打印系统类型
        receive_data()  # 启动数据接收与发布
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 IMU/GPS节点被中断，正常退出")
    except Exception as e:
        rospy.logerr(f"💥 节点异常退出: {str(e)}")