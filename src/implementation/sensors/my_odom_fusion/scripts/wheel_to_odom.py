#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32, Int32
import numpy as np

class WheelToOdom:
    def __init__(self):
        rospy.init_node('wheel_to_odom')
        
        # 获取参数
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0425)
        self.wheel_circle = rospy.get_param('~wheel_circle', 360.0)
        
        # 存储最新接收到的数据
        self.current_angle = 0.0
        self.current_rate = 0.0
        self.current_circle = 360.0  # 默认值
        
        # 初始方向（假设初始朝向为0，即x轴正方向）
        self.yaw = 0.0
        
        # 订阅轮速计话题
        rospy.Subscriber('/wheel_angle', Float32, self.angle_callback)
        rospy.Subscriber('/wheel_rate', Float32, self.rate_callback)
        rospy.Subscriber('/wheel_circle', Int32, self.circle_callback)
        
        # 发布者 - 发布到 /wheel_odom 话题
        self.odom_pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=10)
        
        self.last_time = rospy.Time.now()
        self.last_angle = 0.0
        self.x = 0.0
        self.y = 0.0
        
        rospy.loginfo("Pure wheel odometry node started, publishing to /wheel_odom")
        
    def angle_callback(self, msg):
        self.current_angle = msg.data
        self.update_odom()
        
    def rate_callback(self, msg):
        self.current_rate = msg.data
        self.update_odom()
        
    def circle_callback(self, msg):
        self.current_circle = msg.data
        
    def update_odom(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt <= 0:
            return
            
        # 计算线速度 (m/s)
        # wheel_rate 是度每秒，转换为弧度每秒再乘以半径
        angular_velocity_rad = self.current_rate * math.pi / 180.0
        linear_velocity = angular_velocity_rad * self.wheel_radius
        
        # 计算位移
        angle_diff = self.current_angle - self.last_angle
        distance = (angle_diff / self.current_circle) * (2 * math.pi * self.wheel_radius)
        
        # 更新位置
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)
        
        # 创建Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # 位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # 方向 (使用四元数表示)
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.yaw)
        
        # 速度
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0  # 纯轮速计无法提供角速度
        
        # 设置协方差矩阵
        pose_covariance = [0.0] * 36
        twist_covariance = [0.0] * 36
        
        # 设置位置协方差
        pose_covariance[0] = 0.1  # x位置协方差
        pose_covariance[7] = 0.1  # y位置协方差
        pose_covariance[14] = 0.1  # z位置协方差
        
        # 设置线速度协方差
        twist_covariance[0] = 0.05  # x线速度协方差
        twist_covariance[7] = 0.05  # y线速度协方差
        twist_covariance[14] = 0.05  # z线速度协方差
        
        odom_msg.pose.covariance = pose_covariance
        odom_msg.twist.covariance = twist_covariance
        
        # 发布到 /wheel_odom 话题
        self.odom_pub.publish(odom_msg)
        
        self.last_time = current_time
        self.last_angle = self.current_angle
        
    def quaternion_from_yaw(self, yaw):
        # 从偏航角创建四元数
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

if __name__ == '__main__':
    try:
        wto = WheelToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass