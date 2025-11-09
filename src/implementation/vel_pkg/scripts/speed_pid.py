#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
# 此代码用于测试速度PID控制器#
class VelocityPIDController:
    def __init__(self):
        rospy.init_node('velocity_pid_controller', anonymous=True)
        
        # PID参数 
        self.kp = rospy.get_param('~kp', 8.3)  # 比例系数
        self.ki = rospy.get_param('~ki', 0.09)   # 积分系数
        self.kd = rospy.get_param('~kd', 0.1211)   # 微分系数
        
        # 前馈控制参数
        self.feedforward_gain = rospy.get_param('~feedforward_gain', 20.0)  # 前馈增益
        
        # 控制参数
        self.max_throttle = rospy.get_param('~max_throttle', 100.0)  # 最大油门值
        self.min_throttle = rospy.get_param('~min_throttle', 0.0)    # 最小油门值
        self.control_frequency = rospy.get_param('~control_frequency', 10.0)  # 控制频率，Hz
        
        # 车轮半径
        self.wheel_radius = 0.267  # 车轮半径，单位：米
        
        # 状态变量
        self.target_velocity = 0.8  # 固定目标速度为0.8 m/s
        self.last_error = 0.0       # 上一次误差
        self.integral = 0.0         # 积分项
        self.last_time = rospy.get_time()  # 上一次控制时间
        self.vel_true = 0.0         # 实际速度
        self.last_output = 0.0      # 上一次的输出值
        
        # 速度滤波参数
        self.velocity_history = []  # 存储历史速度值
        self.history_length = 5     # 历史数据长度
        self.velocity_threshold = 0.05  # 速度异常阈值，小于此值的负速度视为异常
        
        # 创建发布器和订阅器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/wheel_rate', Float32, self.wheel_rate_callback)
        
        rospy.loginfo("PID速度控制器已启动")
        rospy.loginfo("目标速度固定为: %.2f m/s", self.target_velocity)
        rospy.loginfo("PID参数: Kp=%.2f, Ki=%.2f, Kd=%.2f", self.kp, self.ki, self.kd)
        rospy.loginfo("前馈增益: %.2f", self.feedforward_gain)
        rospy.loginfo("等待/wheel_rate话题数据...")
    
    def filter_velocity(self, raw_velocity):
        """
        过滤速度值，处理异常值
        """
        # 处理异常值：负速度但绝对值很小的情况
        if raw_velocity < 0 and abs(raw_velocity) < self.velocity_threshold:
            filtered_velocity = 0.0
            rospy.logdebug("检测到异常负速度 %.4f，已过滤为0", raw_velocity)
        else:
            filtered_velocity = raw_velocity
        
        # 添加当前速度到历史记录
        self.velocity_history.append(filtered_velocity)
        
        # 保持历史记录长度
        if len(self.velocity_history) > self.history_length:
            self.velocity_history.pop(0)
        
        # 计算移动平均
        if len(self.velocity_history) > 0:
            smoothed_velocity = sum(self.velocity_history) / len(self.velocity_history)
        else:
            smoothed_velocity = filtered_velocity
        
        return smoothed_velocity
    
    def wheel_rate_callback(self, msg):
        """
        处理/wheel_rate话题回调 (Float32)
        计算并过滤实际速度
        """
        # 使用您提供的公式计算原始速度
        raw_velocity = (msg.data / 360) * self.wheel_radius * 0.4
        
        # 过滤速度值
        self.vel_true = self.filter_velocity(raw_velocity)
        
        rospy.logdebug("原始速度: %.4f m/s, 过滤后速度: %.4f m/s", raw_velocity, self.vel_true)
        
        # 在接收到速度数据时执行PID计算并发布控制命令
        self.pid_control_and_publish()
    
    def pid_control_and_publish(self):
        """
        PID控制计算并发布到/cmd_vel话题
        添加前馈控制以确保在达到目标速度时不会输出零油门
        """
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return
        
        # 计算误差
        error = self.target_velocity - self.vel_true
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项 (防止积分饱和)
        self.integral += error * dt
        # 积分限幅，防止积分饱和
        integral_max = 50.0  # 根据系统调整
        self.integral = max(-integral_max, min(integral_max, self.integral))
        integral = self.ki * self.integral
        
        # 微分项
        derivative = self.kd * (error - self.last_error) / dt
        self.last_error = error
        self.last_time = current_time
        
        # 前馈控制 - 根据目标速度提供基础油门
        # 这是防止速度达到目标时输出为零的关键
        feedforward = self.target_velocity * self.feedforward_gain
        
        # 计算PID输出 (包含前馈)
        output = feedforward + proportional + integral + derivative
        
        # 确保输出不会为零（当接近零时使用上一次的输出）
        if abs(output) < 5.0 and abs(self.last_output) > 5.0:
            output = self.last_output * 0.9  # 缓慢减少输出，而不是直接跳到零
        
        # 限制输出范围
        output = max(self.min_throttle, min(self.max_throttle, output))
        
        # 保存当前输出供下一次使用
        self.last_output = output
        
        # 创建控制消息
        cmd_msg = Twist()
        cmd_msg.linear.x = output  # 油门
        cmd_msg.linear.y = 0.0     # 刹车
        cmd_msg.angular.z = 0.0    # 转向
        
        # 发布控制消息
        self.cmd_pub.publish(cmd_msg)
        
        # 记录详细的调试信息
        rospy.loginfo("目标: %.2f m/s, 实际: %.4f m/s, 误差: %.4f, 油门: %.2f (前馈: %.2f, P: %.2f, I: %.2f, D: %.2f)", 
                     self.target_velocity, self.vel_true, error, output,
                     feedforward, proportional, integral, derivative)

if __name__ == '__main__':
    try:
        controller = VelocityPIDController()
        rospy.spin()  # 保持节点运行，等待回调
    except rospy.ROSInterruptException:
        pass