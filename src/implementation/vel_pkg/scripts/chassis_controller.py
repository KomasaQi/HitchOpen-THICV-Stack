#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import can
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

GEAR_HIGH = 1000 # 档位对应channel6
GEAR_LOW = 2000 

class VehicleController:
    def __init__(self):
        # 初始化系统自带CAN接口
        try:
            # 设置CAN接口 (假设使用can0)
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
            rospy.loginfo("系统CAN接口初始化完成 (can0, 500kbps)")
        except Exception as e:
            rospy.logerr("CAN接口初始化失败: %s", str(e))
            raise e
        
        # 安全数据 (ch3和ch4为1500，ch6和ch8为1000)
        self.current_channels = [1500, 1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000]
        
        # 设置关闭时安全回调
        rospy.on_shutdown(self.shutdown_hook)
    
    def send_can_frames(self, ch3_value, ch4_value):
        """发送当前通道值到CAN总线"""
        # 更新通道值
        self.current_channels[2] = ch3_value  # 通道3 (油门/刹车)
        self.current_channels[3] = ch4_value  # 通道4 (转向)
        
        # 确保有9个通道值
        channels = self.current_channels[:9]
        if len(channels) < 9:
            channels = channels + [1500] * (9 - len(channels))
            rospy.logwarn("通道数据不足9个，使用安全值1500填充")
        
        # 提取并处理各个通道值
        ch1 = channels[0] - 1000
        ch2 = channels[1] - 1000
        ch3 = channels[2] - 1000
        ch4 = channels[3] - 1000
        ch6 = 0  # 固定值1000 (1000-1000=0)
        ch7 = channels[6] - 1000
        ch8 = 0  # 固定值1000 (1000-1000=0)
        ch9 = channels[8] - 1000

        # 构建第一帧数据（通道1-4）
        data1 = [
            ch1 & 0xFF, (ch1 >> 8) & 0xFF,
            ch2 & 0xFF, (ch2 >> 8) & 0xFF,
            ch3 & 0xFF, (ch3 >> 8) & 0xFF,
            ch4 & 0xFF, (ch4 >> 8) & 0xFF
        ]
        
        # 创建CAN消息
        msg1 = can.Message(
            arbitration_id=0x100,
            data=data1,
            is_extended_id=False
        )
        
        # 发送第一帧
        try:
            self.bus.send(msg1)
            rospy.logdebug("发送 CAN 0x100: [%s]", ' '.join(f"{b:02X}" for b in data1))
        except can.CanError as e:
            rospy.logerr("CAN 0x100 发送失败: %s", str(e))
            return False
        
        # 构建第二帧数据（通道6-9）
        data2 = [
            ch6 & 0xFF, (ch6 >> 8) & 0xFF,
            ch7 & 0xFF, (ch7 >> 8) & 0xFF,
            ch8 & 0xFF, (ch8 >> 8) & 0xFF,
            ch9 & 0xFF, (ch9 >> 8) & 0xFF
        ]
        
        # 创建CAN消息
        msg2 = can.Message(
            arbitration_id=0x101,
            data=data2,
            is_extended_id=False
        )
        
        # 发送第二帧
        try:
            self.bus.send(msg2)
            rospy.logdebug("发送 CAN 0x101: [%s]", ' '.join(f"{b:02X}" for b in data2))
        except can.CanError as e:
            rospy.logerr("CAN 0x101 发送失败: %s", str(e))
            return False
        
        return True

    def shutdown_hook(self):
        """ROS关闭时发送安全值"""
        rospy.loginfo("发送安全停止指令")
        # 发送安全值
        self.send_can_frames(1500, 1500)
        time.sleep(0.1)  # 确保指令发送
        # 关闭CAN总线
        self.bus.shutdown()


class ChassisNode:
    def __init__(self):
        rospy.init_node('chassis_controller')
        rospy.loginfo("底盘控制器节点启动")
        
        # 创建车辆控制器
        self.controller = VehicleController()
        
        # PID参数 
        self.kp = rospy.get_param('~kp', 8.3)  # 比例系数
        self.ki = rospy.get_param('~ki', 0.09)   # 积分系数
        self.kd = rospy.get_param('~kd', 0.1211)   # 微分系数
        
        # 前馈控制参数#####################
        self.feedforward_gain = rospy.get_param('~feedforward_gain', 20.0)  # 前馈增益
        
        # 车轮半径
        self.wheel_radius = 0.267  # 车轮半径，单位：米
        
        # 转向参数 (根据您的标定结果)
        self.max_steering_angle = 25.0  # 最大转向角度(度)
        self.max_steering_radians = math.radians(self.max_steering_angle)  # 最大转向角度(弧度)
        
        # 状态变量
        self.target_velocity = 0.0  # 目标速度
        self.target_steering = 0.0  # 目标转向(弧度)
        self.last_error = 0.0       # 上一次误差
        self.integral = 0.0         # 积分项
        self.last_time = rospy.get_time()  # 上一次控制时间
        self.vel_true = 0.0         # 实际速度
        self.last_output = 0.0      # 上一次的输出值
        
        # 速度滤波参数
        self.velocity_history = []  # 存储历史速度值
        self.history_length = 5     # 历史数据长度
        self.velocity_threshold = 0.05  # 速度异常阈值，小于此值的负速度视为异常
        
        # 订阅话题
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber("/wheel_rate", Float32, self.wheel_rate_callback)
        
        rospy.loginfo("融合底盘控制器已启动")
        rospy.loginfo("PID参数: Kp=%.2f, Ki=%.2f, Kd=%.2f", self.kp, self.ki, self.kd)
        rospy.loginfo("前馈增益: %.2f", self.feedforward_gain)
        rospy.loginfo("最大转向角度: %.1f度 (%.3f弧度)", self.max_steering_angle, self.max_steering_radians)
        rospy.loginfo("等待/cmd_vel和/wheel_rate话题数据...")
    
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
    
    def cmd_vel_callback(self, msg):
        """
        处理/cmd_vel话题回调 (Twist)
        linear.x: 目标速度 (m/s)
        angular.z: 转向角速度 (rad/s) 
        """
        try:
            # 获取输入
            self.target_velocity = msg.linear.x  # 目标速度
            angular_velocity = msg.angular.z     # 角速度
            
            # 限制速度范围
            self.target_velocity = max(-2.0, min(2.0, self.target_velocity))
            
            # 将角速度转换为转向角度
            # 这是一个简化模型，实际转换可能需要考虑车辆动力学
            # 这里使用一个简单的比例关系
            steering_gain = 0.5  # 角速度到转向角度的增益系数，需要根据实际车辆调整
            self.target_steering = angular_velocity * steering_gain
            
            # 限制转向角度在最大范围内
            self.target_steering = max(-self.max_steering_radians, 
                                      min(self.max_steering_radians, self.target_steering))

            rospy.logdebug("收到目标速度: %.2f m/s, 目标转向: %.3f rad (%.1f°)", 
                          self.target_velocity, self.target_steering, 
                          math.degrees(self.target_steering))

        except Exception as e:
            rospy.logerr("cmd_vel处理错误: %s", str(e))
    
    def pid_control_and_publish(self):
        """
        PID控制计算并发布到CAN总线
        """
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return
        
        # 计算速度误差
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
        
        # 前馈控制 - 根据目标速度提供基础油门######################
        feedforward = self.target_velocity * self.feedforward_gain
        
        # 计算PID输出 (包含前馈)
        output = feedforward + proportional + integral + derivative
        
        # 确保输出不会为零（当接近零时使用上一次的输出）
        if abs(output) < 5.0 and abs(self.last_output) > 5.0:
            output = self.last_output * 0.9  # 缓慢减少输出，而不是直接跳到零
        
        # 限制输出范围
        output = max(-100.0, min(100.0, output))  # 油门/刹车范围
        
        # 保存当前输出供下一次使用
        self.last_output = output
        
        # 将PID输出转换为PWM信号
        # 油门/刹车通道 (ch3)
        pwm_channel3 = int(1500 + output * 5)
        pwm_channel3 = max(1000, min(2000, pwm_channel3))
        
        # 将转向角度(弧度)转换为PWM信号
        # 转向通道 (ch4): 1500 ± (转向角度/最大转向角度) * 500
        steering_ratio = self.target_steering / self.max_steering_radians
        steering_pwm = int(1500 + steering_ratio * 500)
        steering_pwm = max(1000, min(2000, steering_pwm))
        
        # 发送CAN数据
        success = self.controller.send_can_frames(pwm_channel3, steering_pwm)
        
        if not success:
            rospy.logwarn("CAN发送失败，尝试重新初始化CAN控制器")
            try:
                # 重新初始化CAN总线
                self.controller.bus.shutdown()
                self.controller.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
                rospy.loginfo("CAN控制器重新初始化成功")
                # 重试发送
                self.controller.send_can_frames(pwm_channel3, steering_pwm)
            except Exception as e:
                rospy.logerr("CAN控制器重新初始化失败: %s", str(e))
        
        # 记录详细的调试信息
        rospy.loginfo("目标: %.2f m/s, 实际: %.4f m/s, 误差: %.4f, 油门: %.2f, 转向: %.3f rad (%.1f°)", 
                     self.target_velocity, self.vel_true, error, output, 
                     self.target_steering, math.degrees(self.target_steering))

    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ChassisNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点启动失败: %s", str(e))
    finally:
        rospy.loginfo("底盘控制器节点关闭")