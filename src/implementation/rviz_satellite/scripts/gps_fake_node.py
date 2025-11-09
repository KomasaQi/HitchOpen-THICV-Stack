#!/usr/bin/env python

import rospy
import random
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
import time

class BeijingGPSPublisher:
    def __init__(self):
        rospy.init_node('beijing_gps_publisher', anonymous=True)
        
        # 发布器
        self.gps_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
        self.velocity_pub = rospy.Publisher('/velocity', TwistStamped, queue_size=10)
        
        # 北京的大致区域范围 (天安门附近区域)
        self.base_lat = 39.9087  # 天安门纬度
        self.base_lon = 116.3975  # 天安门经度
        
        # 当前坐标
        self.current_lat = self.base_lat
        self.current_lon = self.base_lon
        
        # 运动参数
        self.speed = 0.0  # m/s
        self.heading = 0.0  # 方向 (弧度)
        
        # 发布频率
        self.rate = rospy.Rate(1)  # 1Hz
        
        rospy.loginfo("Beijing GPS Publisher Started")
        
    def generate_realistic_movement(self):
        """生成更真实的运动轨迹"""
        # 随机改变速度和方向，但保持连贯性
        speed_change = random.uniform(-1.0, 1.0)
        self.speed = max(0.0, min(15.0, self.speed + speed_change))  # 限制速度在0-15m/s
        
        heading_change = random.uniform(-0.3, 0.3)  # 小角度转向
        self.heading += heading_change
        
        # 确保方向在0-2π范围内
        self.heading %= 2 * math.pi
        
        # 计算新的位置 (近似计算，适用于小范围移动)
        # 1度纬度 ≈ 111km, 1度经度 ≈ 111km * cos(纬度)
        lat_km_per_degree = 111.0
        lon_km_per_degree = 111.0 * math.cos(math.radians(self.current_lat))
        
        # 计算位移 (1秒内的移动)
        distance = self.speed * 1.0  # 米
        delta_lat = (distance * math.cos(self.heading)) / (lat_km_per_degree * 1000)
        delta_lon = (distance * math.sin(self.heading)) / (lon_km_per_degree * 1000)
        
        # 更新位置
        new_lat = self.current_lat + delta_lat
        new_lon = self.current_lon + delta_lon
        
        # 限制在北京中心区域附近 (±约2km范围)
        max_delta = 0.02  # 约2km
        if abs(new_lat - self.base_lat) > max_delta:
            self.heading += math.pi  # 调头
            new_lat = self.current_lat
        if abs(new_lon - self.base_lon) > max_delta:
            self.heading += math.pi  # 调头
            new_lon = self.current_lon
            
        return new_lat, new_lon
    
    def publish_gps_data(self, latitude, longitude):
        """发布GPS数据"""
        # 创建NavSatFix消息
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps"
        
        # 设置状态
        gps_msg.status.status = 0  # 无故障
        gps_msg.status.service = 1  # GPS
        
        # 设置坐标
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = 50.0  # 假设海拔50米
        
        # 设置协方差 (假设精度较好)
        gps_msg.position_covariance[0] = 1.0
        gps_msg.position_covariance[4] = 1.0
        gps_msg.position_covariance[8] = 2.0
        gps_msg.position_covariance_type = 2  # 已知协方差
        
        # 发布消息
        self.gps_pub.publish(gps_msg)
        
        # 发布速度信息 (可选，但有助于rviz_satellite的显示)
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = gps_msg.header.stamp
        velocity_msg.header.frame_id = "gps"
        velocity_msg.twist.linear.x = self.speed * math.cos(self.heading)
        velocity_msg.twist.linear.y = self.speed * math.sin(self.heading)
        
        self.velocity_pub.publish(velocity_msg)
        
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                # 生成新的位置
                new_lat, new_lon = self.generate_realistic_movement()
                self.current_lat = new_lat
                self.current_lon = new_lon
                
                # 发布GPS数据
                self.publish_gps_data(self.current_lat, self.current_lon)
                
                # 打印调试信息
                rospy.loginfo_throttle(10, "Publishing GPS: Lat=%.6f, Lon=%.6f, Speed=%.1f m/s", 
                                      self.current_lat, self.current_lon, self.speed)
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr("Error in GPS publisher: %s", str(e))
                self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = BeijingGPSPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass