#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import MCP2515
import time
import math
import struct

class RPLidarROSNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('rplidar_can_receiver', anonymous=True)
        self.pub = rospy.Publisher('scan', LaserScan, queue_size=10)
        
        # 初始化 CAN 控制器
        self.can = MCP2515.MCP2515()
        rospy.loginfo("Initializing CAN controller...")
        self.can.Init()
        rospy.loginfo("CAN controller initialized successfully")
        
        # 雷达数据结构
        self.scan_data = {
            'distances': [0.0] * 360,  # 360度距离数据
            'qualities': [0] * 360,     # 360度质量数据
            'last_angle': 0.0,          # 上一个角度
            'new_scan': False           # 新扫描标志
        }
        
        # 激光扫描参数
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = "laser_frame"
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = 2 * math.pi
        self.scan_msg.angle_increment = math.pi / 180.0  # 1度增量
        self.scan_msg.time_increment = 0.00005           # 假设的每点时间
        self.scan_msg.scan_time = 0.1                    # 扫描时间
        self.scan_msg.range_min = 0.15                   # 最小距离 (米)
        self.scan_msg.range_max = 12.0                   # 最大距离 (米)
        
        # 性能计数器
        self.msg_count = 0
        self.scan_count = 0
        self.last_print_time = time.time()
    
    def parse_can_message(self, data):
        """解析 CAN 消息数据"""
        try:
            # 确保数据长度足够 (6字节)
            if len(data) < 6:
                return False
                
            # 将十六进制字符串转换为整数
            int_data = []
            for item in data:
                if isinstance(item, str) and item.startswith('0x'):
                    int_data.append(int(item, 16))
                else:
                    int_data.append(item)
            
            # 解析数据 (小端格式)
            distance_cm = (int_data[1] << 8) | int_data[0]
            angle_centideg = (int_data[3] << 8) | int_data[2]
            quality = int_data[4]
            flags = int_data[5]
            
            # 转换为标准单位
            distance = distance_cm / 100.0  # 厘米转米
            angle = angle_centideg / 100.0  # 0.01度转度
            valid = (flags & 0x01) != 0
            
            # 处理角度回绕 (0-360度)
            if angle < 0:
                angle += 360.0
            elif angle >= 360.0:
                angle -= 360.0
            
            # 检测新扫描开始 (角度从大变小)
            if angle < self.scan_data['last_angle']:
                self.scan_data['new_scan'] = True
            
            # 更新上一个角度
            self.scan_data['last_angle'] = angle
            
            # 只存储有效数据
            if valid:
                # 转换为数组索引 (0-359)
                idx = int(round(angle)) % 360
                self.scan_data['distances'][idx] = distance
                self.scan_data['qualities'][idx] = quality
            
            return True
        except Exception as e:
            rospy.logwarn(f"Error parsing CAN message: {str(e)}")
            return False
    
    def publish_scan(self):
        """发布激光扫描数据"""
        if not self.scan_data['new_scan']:
            return False
        
        # 准备激光扫描消息
        self.scan_msg.header.stamp = rospy.Time.now()
        self.scan_msg.ranges = self.scan_data['distances'][:]
        self.scan_msg.intensities = self.scan_data['qualities'][:]
        
        # 发布消息
        self.pub.publish(self.scan_msg)
        
        # 重置新扫描标志
        self.scan_data['new_scan'] = False
        self.scan_count += 1
        
        # 记录调试信息
        rospy.logdebug(f"Published scan #{self.scan_count}")
        return True
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("Starting RPLIDAR CAN receiver. Waiting for data...")
        
        # 设置ROS循环频率 (50Hz)
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            # 接收 CAN 消息
            readbuf = self.can.Receive()
            
            if readbuf:
                # 提取消息ID和数据
                can_id = readbuf[0]  # 第一个元素是CAN ID
                can_data = readbuf[1]  # 第二个元素是数据列表
                
                # 只处理雷达数据消息 (ID 0x200)
                if can_id == 0x200:
                    if self.parse_can_message(can_data):
                        self.msg_count += 1
            
            # 检查是否需要发布扫描数据
            self.publish_scan()
            
            # 性能统计 (每秒打印一次)
            current_time = time.time()
            if current_time - self.last_print_time > 1.0:
                rospy.loginfo(f"CAN message rate: {self.msg_count} Hz, Scan rate: {self.scan_count} Hz")
                self.msg_count = 0
                self.scan_count = 0
                self.last_print_time = current_time
            
            # 维持ROS循环频率
            rate.sleep()

if __name__ == "__main__":
    try:
        node = RPLidarROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
    finally:
        rospy.loginfo("--------------------------------------------------------")