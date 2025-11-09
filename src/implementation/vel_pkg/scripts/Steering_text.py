#!/usr/bin/env python3
# coding=utf-8

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

# 自己实现的PID控制器类
class PIDController:
    def __init__(self, kp, ki, kd, max_output=1.0, min_output=-1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
    
    def update(self, setpoint, current_value):
        current_time = rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = current_time
            return 0.0
            
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
        
        error = setpoint - current_value
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 微分项
        derivative = self.kd * (error - self.previous_error) / dt
        
        # 计算输出
        output = proportional + integral + derivative
        
        # 限制输出范围
        output = max(min(output, self.max_output), self.min_output)
        
        # 更新状态
        self.previous_error = error
        self.last_time = current_time
        
        return output

# 全局变量
current_yaw = 0.0
current_x = 0.0
current_y = 0.0
odom_received = False

def odom_callback(msg):
    """订阅odom，提取位置和yaw角"""
    global current_yaw, current_x, current_y, odom_received
    
    # 提取位置
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    # 提取朝向 - 四元数转欧拉角
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    odom_received = True

class SimplePathFollower:
    def __init__(self):
        # 横向控制PID
        self.steering_pid = PIDController(3.0, 0.02, 0.8)
        
        # 控制参数
        self.linear_speed = 0.6
        
        # 路径参数 - 简单的直线路径，带有一点弯曲
        self.path_points = []
        for i in range(100):
            x = i * 0.05  # 5米长的路径
            y = 0.5 * math.sin(x)  # 简单的正弦曲线
            self.path_points.append((x, y))
        
        self.current_target_index = 0
        self.lookahead_distance = 0.5
    
    def find_target_point(self, x, y):
        """找到路径上的目标点"""
        # 找到距离当前位置最近的点
        min_dist = float('inf')
        closest_index = 0
        
        for i, (px, py) in enumerate(self.path_points):
            dist = math.sqrt((x - px)**2 + (y - py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        
        # 选择预瞄点
        target_index = min(closest_index + 5, len(self.path_points) - 1)
        return self.path_points[target_index]
    
    def update(self, x, y, yaw):
        """更新控制命令"""
        # 如果已经到达路径终点，停止
        if self.current_target_index >= len(self.path_points) - 1:
            return 0.0, 0.0, True
        
        # 找到目标点
        target_x, target_y = self.find_target_point(x, y)
        
        # 计算到目标点的角度
        dx = target_x - x
        dy = target_y - y
        target_angle = math.atan2(dy, dx)
        
        # 计算角度误差
        angle_error = target_angle - yaw
        
        # 规范化角度误差到[-π, π]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 使用PID计算转向控制量
        angular_speed = self.steering_pid.update(0, angle_error)
        
        return self.linear_speed, angular_speed, False, target_x, target_y

if __name__ == "__main__":
    rospy.init_node("simple_path_follower")
    
    # 先检查odom话题是否存在
    try:
        rospy.wait_for_message("/odom", Odometry, timeout=5.0)
        print("找到 /odom 话题")
    except rospy.ROSException:
        print("未找到 /odom 话题，请检查odom发布者")
        # 退出或使用默认值继续
    
    rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    # 创建路径跟踪器
    follower = SimplePathFollower()
    
    rate = rospy.Rate(20)  # 20Hz
    
    print("开始简单路径跟踪测试...")
    print(f"固定速度: {follower.linear_speed} m/s")
    
    # 添加计数器，控制打印频率
    print_count = 0
    
    # 记录开始位置
    start_x, start_y = current_x, current_y
    
    while not rospy.is_shutdown():
        # 等待接收到odom数据
        if not odom_received:
            rospy.loginfo_throttle(1.0, "等待odom数据...")
            rate.sleep()
            continue
        
        # 计算相对于起点的位置
        rel_x = current_x - start_x
        rel_y = current_y - start_y
        
        # 更新控制命令
        linear_speed, angular_speed, path_completed, target_x, target_y = follower.update(
            rel_x, rel_y, current_yaw)
        
        if path_completed:
            print("路径跟踪完成!")
            # 停止小车
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            pub.publish(cmd)
            rospy.signal_shutdown("路径完成")
            break
        
        # 发布控制指令
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        pub.publish(cmd)
        
        # 打印调试信息
        print_count += 1
        if print_count >= 20:
            print_count = 0
            rospy.loginfo(
                f"位置: ({rel_x:.2f}, {rel_y:.2f}), "
                f"目标: ({target_x:.2f}, {target_y:.2f}), "
                f"朝向: {math.degrees(current_yaw):.1f}°, "
                f"控制: lin={linear_speed:.2f}, ang={angular_speed:.3f}")
        
        rate.sleep()