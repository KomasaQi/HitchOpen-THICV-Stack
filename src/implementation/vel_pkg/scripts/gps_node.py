#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import math
import can
import time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped

# ================== ROS参数配置 ==================
# 以下参数可以通过ROS参数服务器配置
UKF_POS_ID = rospy.get_param('~ukf_pos_id', 0x308)
UKF_ALTITUDE_ID = rospy.get_param('~ukf_altitude_id', 0x309)
UKF_VEL_NED_ID = rospy.get_param('~ukf_vel_ned_id', 0x311)
FRAME_ID = rospy.get_param('~frame_id', 'gps_link')
# ================================================

# ================== CAN配置 ==================
CAN_CONFIG = {
    UKF_POS_ID: {
        'name': 'UKF_POS',
        'fields': [
            ('latitude', 0, 4, True, 1e-7, 1e-6),
            ('longitude', 4, 4, True, 1e-7, 1e-6)
        ]
    },
    UKF_ALTITUDE_ID: {
        'name': 'UKF_ALTITUDE',
        'fields': [
            ('altitude', 0, 4, True, 0.001, 0.01),
            ('undulation', 4, 2, True, 0.005, 0.01)
        ]
    },
    UKF_VEL_NED_ID: {
        'name': 'UKF_VEL_NED',
        'fields': [
            ('n', 0, 2, True, 0.01, 0.01),
            ('e', 2, 2, True, 0.01, 0.01),
            ('d', 4, 2, True, 0.01, 0.01)
        ]
    }
}

# ================== GPS数据处理类 ==================
class GPSDataHandler:
    def __init__(self):
        # 初始化GPS数据结构
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'velocity_n': 0.0,
            'velocity_e': 0.0,
            'velocity_d': 0.0,
            'last_update': rospy.Time.now(),
            'status': NavSatStatus()
        }
        
        # 设置GPS状态 (假设有固定解)
        self.gps_data['status'].status = NavSatStatus.STATUS_FIX
        self.gps_data['status'].service = NavSatStatus.SERVICE_GPS
        
        # 创建ROS发布器
        self.fix_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        self.vel_pub = rospy.Publisher('gps/vel', TwistStamped, queue_size=10)
        
        # 初始化协方差矩阵
        self.position_covariance = [0.0] * 9
        self.position_covariance[0] = 0.5  # 纬度方差
        self.position_covariance[4] = 0.5  # 经度方差
        self.position_covariance[8] = 1.0  # 高度方差
        self.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    def extract_value(self, b, offset, length, is_signed, factor):
        """从字节数据中提取值"""
        slice_ = b[offset:offset+length]
        fmt = {1:'b',2:'h',4:'i'}[length] if is_signed else {1:'B',2:'H',4:'I'}[length]
        val = struct.unpack('<' + fmt, slice_)[0]
        return val * factor

    def update_data(self, can_id, data_bytes):
        """更新GPS数据"""
        cfg = CAN_CONFIG.get(can_id)
        if not cfg:
            return False
        
        try:
            # 处理位置数据
            if can_id == UKF_POS_ID:
                for field, offset, length, signed, factor, _ in cfg['fields']:
                    value = self.extract_value(data_bytes, offset, length, signed, factor)
                    if field == 'latitude': 
                        self.gps_data['latitude'] = value
                    elif field == 'longitude': 
                        self.gps_data['longitude'] = value
            
            # 处理高度数据
            elif can_id == UKF_ALTITUDE_ID:
                for field, offset, length, signed, factor, _ in cfg['fields']:
                    value = self.extract_value(data_bytes, offset, length, signed, factor)
                    if field == 'altitude': 
                        self.gps_data['altitude'] = value
            
            # 处理速度数据
            elif can_id == UKF_VEL_NED_ID:
                for field, offset, length, signed, factor, _ in cfg['fields']:
                    value = self.extract_value(data_bytes, offset, length, signed, factor)
                    if field == 'n': 
                        self.gps_data['velocity_n'] = value
                    elif field == 'e': 
                        self.gps_data['velocity_e'] = value
                    elif field == 'd': 
                        self.gps_data['velocity_d'] = value
            
            # 更新最后更新时间
            self.gps_data['last_update'] = rospy.Time.now()
            return True
            
        except Exception as e:
            rospy.logerr(f"GPS数据处理错误: {e}")
            return False

    def publish_gps_data(self):
        """发布GPS数据"""
        # 创建NavSatFix消息
        fix_msg = NavSatFix()
        
        # 设置消息头
        fix_msg.header.stamp = self.gps_data['last_update']
        fix_msg.header.frame_id = FRAME_ID
        
        # 设置状态信息
        fix_msg.status = self.gps_data['status']
        
        # 设置位置数据
        fix_msg.latitude = self.gps_data['latitude']
        fix_msg.longitude = self.gps_data['longitude']
        fix_msg.altitude = self.gps_data['altitude']
        
        # 设置协方差
        fix_msg.position_covariance = self.position_covariance
        fix_msg.position_covariance_type = self.position_covariance_type
        
        # 发布位置消息
        self.fix_pub.publish(fix_msg)
        
        # 创建速度消息
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.gps_data['last_update']
        vel_msg.header.frame_id = FRAME_ID
        
        # 设置速度 (NED -> ENU转换)
        # ROS使用ENU坐标系: East, North, Up
        # 原始数据是NED: North, East, Down
        vel_msg.twist.linear.x = self.gps_data['velocity_e']  # East -> X
        vel_msg.twist.linear.y = self.gps_data['velocity_n']  # North -> Y
        vel_msg.twist.linear.z = -self.gps_data['velocity_d']  # Down -> -Up
        
        # 发布速度消息
        self.vel_pub.publish(vel_msg)

# ================== 主程序 ==================
def main():
    # 初始化ROS节点
    rospy.init_node('gps_data_publisher')
    rospy.loginfo("GPS数据发布器启动")
    
    # 初始化GPS处理器
    gps_handler = GPSDataHandler()
    
    # 初始化CAN总线
    can_interface = rospy.get_param('~can_interface', 'can0')
    can_bitrate = rospy.get_param('~can_bitrate', 500000)
    
    try:
        bus = can.interface.Bus(channel=can_interface, bustype='socketcan', bitrate=can_bitrate)
        rospy.loginfo("CAN总线初始化成功")
    except Exception as e:
        rospy.logerr(f"CAN总线初始化失败: {e}")
        return
    
    # 设置循环频率 (10Hz)
    rate = rospy.Rate(10)
    
    rospy.loginfo("开始主循环...")
    
    try:
        while not rospy.is_shutdown():
            # 接收CAN数据
            try:
                message = bus.recv(timeout=0.01)
                if message:
                    # 只处理GPS相关数据
                    if message.arbitration_id in [UKF_POS_ID, UKF_ALTITUDE_ID, UKF_VEL_NED_ID]:
                        gps_handler.update_data(message.arbitration_id, message.data)
            except can.CanError as e:
                rospy.logwarn(f"CAN接收错误: {e}")
            except Exception as e:
                rospy.logwarn(f"其他CAN错误: {e}")
            
            # 发布GPS数据
            gps_handler.publish_gps_data()
            
            # 控制循环频率
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序错误: {e}")
    finally:
        # 关闭CAN总线
        try:
            bus.shutdown()
        except:
            pass
        rospy.loginfo("GPS数据发布器关闭")

if __name__ == "__main__":
    main()