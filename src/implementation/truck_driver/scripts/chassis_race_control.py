#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import can
import numpy as np
from geometry_msgs.msg import Twist
from race_msgs.msg import Control
from std_msgs.msg import Float32
import math

###################################################
#                   控制信号参数
# ================================================
# CAN总线波特率
CAN_BUS_BAUD = 500000 # 500kbps

# CH3 油门/刹车控制
DRIVE_STOP = 1500  # 没有油门
DRIVE_FULL_THROTTLE = 1000 # 全油门对应
DRIVE_FULL_REVERSE = 2000  # 全后退对应

# CH4 转向控制
STEER_NULL = 1500 # 0转向对应

# CH5 自动驾驶模式开关（不接收，只能从遥控器改）
AUTO_MODE = 1500

# CH6 档位控制
GEAR_HIGH = 1000 # 高速档对应
GEAR_LOW = 2000  # 低速档对应

# CH8 差速锁 
DIFF_LOCK_OFF = 1000  # 差速锁关闭对应
DIFF_LOCK_ON = 2000  # 差速锁开启对应

# 信号恒定偏置
CONST_BIAS = 1000
SPARE_CHANNEL_VALUE = 1500  # 备用通道安全值
LIGHT_CONTROL_SAPRE = 1000  # 备用灯光控制
VOLUME_CONTROL_SPARE = 1000 # 备用音量控制

VEHICLE_STATE_STOP = 0
VEHICLE_STATE_DRIVE = 1
VEHICLE_STATE_REVERSE = -1

# 静止判定阈值
STATIC_VELOCITY_THRESHOLD = 0.01  # m/s

# 刹车比例系数
BRAKE_VELOCITY_RATIO = 0.5 # 刹车比例系数 刹车= -1* 系数 * 当前速度
MAX_BRAKE_VALUE = 0.5 # 最大刹车值限制(防止爆缸)
#####################################################
#                   车辆自身参数
# ================================================
MAX_STEER_ANGLE = 85.0  # 最大转向角度(度)
#####################################################


velocity = 0.0  # 当前速度 (m/s)
vehicle_state = VEHICLE_STATE_STOP  # 车辆状态

class VehicleController:
    def __init__(self):
        # 初始化系统自带CAN接口
        try:
            # 设置CAN接口 (假设使用can0)
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=CAN_BUS_BAUD)
            rospy.loginfo("系统CAN接口初始化完成 (can0, 500kbps)")
        except Exception as e:
            rospy.logerr("CAN接口初始化失败: %s", str(e))
            raise e
        
        # 安全数据 (ch3和ch4为1500，ch6和ch8为1000)
        self.current_channels = [SPARE_CHANNEL_VALUE, 
                                 SPARE_CHANNEL_VALUE, 
                                 DRIVE_STOP, 
                                 STEER_NULL, 
                                 AUTO_MODE, 
                                 GEAR_LOW, 
                                 LIGHT_CONTROL_SAPRE, 
                                 DIFF_LOCK_OFF, 
                                 VOLUME_CONTROL_SPARE]
        
        # 设置关闭时安全回调
        rospy.on_shutdown(self.shutdown_hook)
    
    def send_can_frames(self, ch3_value_drive=DRIVE_STOP, ch4_value_steer=STEER_NULL, ch6_value_gear=GEAR_LOW, ch8_value_diff_lock=DIFF_LOCK_OFF):
        """发送当前通道值到CAN总线"""
        # 更新通道值
        self.current_channels[2] = ch3_value_drive  # 通道3 (油门/刹车)
        self.current_channels[3] = ch4_value_steer  # 通道4 (转向)
        self.current_channels[5] = ch6_value_gear  # 通道6 (档位)
        self.current_channels[7] = ch8_value_diff_lock  # 通道8 (差速锁)
        
        # 确保有9个通道值
        channels = self.current_channels[:9]
        if len(channels) < 9:
            channels = channels + [1500] * (9 - len(channels))
            rospy.logwarn("通道数据不足9个，使用安全值1500填充")
        
        # 提取并处理各个通道值
        ch1 = channels[0] - CONST_BIAS
        ch2 = channels[1] - CONST_BIAS
        ch3 = channels[2] - CONST_BIAS
        ch4 = channels[3] - CONST_BIAS
        ch6 = channels[5] - CONST_BIAS 
        ch7 = channels[6] - CONST_BIAS
        ch8 = channels[7] - CONST_BIAS 
        ch9 = channels[8] - CONST_BIAS

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
        self.send_can_frames(DRIVE_STOP, STEER_NULL, GEAR_LOW, DIFF_LOCK_OFF)
        time.sleep(0.1)  # 确保指令发送
        # 关闭CAN总线
        self.bus.shutdown()


class ChassisNode:
    def __init__(self):
        rospy.init_node('chassis_controller')
        rospy.loginfo("底盘控制器节点启动")
        
        # 创建车辆控制器
        self.controller = VehicleController()
        

        # 前馈控制参数#####################
        # self.feedforward_gain = rospy.get_param('~feedforward_gain', 20.0)  # 前馈增益
        
        # 订阅话题
        rospy.Subscriber("/race/control", Control, self.control_callback, queue_size=1)
        rospy.Subscriber("/race/speedometer", Float32, self.speedometer_callback, queue_size=1)
        
        rospy.loginfo("模型小车底盘RACE平台控制器已启动")
        rospy.loginfo("最大转向角度: %.1f度 (%.3f弧度)", MAX_STEER_ANGLE, math.radians(MAX_STEER_ANGLE))
    

    
    def speedometer_callback(self, msg):
        """
        处理/race/speedometer话题回调 (Float32)
        """
        global velocity
        velocity = msg.data

    
    def control_callback(self, msg):
        """
        处理/race/control话题回调 (Control)
        """
        global velocity
        global vehicle_state
        # 将PID输出转换为PWM信号
        throttle = msg.throttle # 期望油门值 (0 ~ 1)
        brake = msg.brake       # 期望刹车值 (0 ~ 1)
        current_gear = msg.gear  # 当前档位
        
        # 油门/刹车通道 (ch3)
        drive_cmd = 0.0
        if current_gear > 0: # 档位在前进挡上
            if velocity > STATIC_VELOCITY_THRESHOLD:
                drive_cmd = throttle - brake  # 油门减去刹车
            else:
                drive_cmd = throttle -BRAKE_VELOCITY_RATIO * velocity * abs(brake) 
                rospy.logwarn("车辆静止或低速，刹车按比例控制: %.3f", drive_cmd)
            
        elif current_gear == Control.GEAR_REVERSE:
            if velocity < -STATIC_VELOCITY_THRESHOLD:
                drive_cmd = -throttle + brake  # 倒车时，油门为负，刹车为正
            else:
                drive_cmd = -throttle -BRAKE_VELOCITY_RATIO * velocity * abs(brake)  
     
        elif current_gear == Control.GEAR_NEUTRAL:
            drive_cmd = 0 # 空挡不驱动，但是刹车信号起作用
            if abs(brake) > 0.01:
                drive_cmd = -BRAKE_VELOCITY_RATIO * velocity * abs(brake)  # 比例控制刹车   
        else:
            drive_cmd = 0.0  # 其他情况不驱动
            rospy.logwarn("Invalid gear: %d", current_gear)
            
    
        drive_pwm = max(1000, min(2000, int(1500 + drive_cmd * 500)))  # 转换为PWM信号范围1000-2000
        rospy.loginfo("油门命令: %.3f, 刹车命令: %.3f, 驱动PWM: %d", throttle, brake, drive_pwm)
        # 限制转向角度在最大范围内
        target_steering = max(-math.radians(MAX_STEER_ANGLE), 
                                    min(math.radians(MAX_STEER_ANGLE), msg.lateral.steering_angle))
        
        # 将转向角度(弧度)转换为PWM信号
        # 转向通道 (ch4): 1500 ± (转向角度/最大转向角度) * 500
        steering_ratio =  target_steering / math.radians(MAX_STEER_ANGLE)
        steering_pwm = int(1500 + steering_ratio * 500)
        steering_pwm = max(1000, min(2000, steering_pwm))
        
        # 将档位转换为PWM信号
        gear_pwm = GEAR_LOW
        if current_gear == Control.GEAR_1:
            gear_pwm = GEAR_LOW
        elif current_gear == Control.GEAR_2:
            gear_pwm = GEAR_HIGH
        elif current_gear == Control.GEAR_3:
            gear_pwm = GEAR_HIGH
        elif current_gear == Control.GEAR_4:
            gear_pwm = GEAR_HIGH
        elif current_gear == Control.GEAR_5:
            gear_pwm = GEAR_HIGH
        elif current_gear == Control.GEAR_6:
            gear_pwm = GEAR_HIGH      
        elif current_gear == Control.GEAR_PARK:
            gear_pwm = GEAR_LOW
        elif current_gear == Control.GEAR_REVERSE:
            gear_pwm = GEAR_LOW         
        elif current_gear == Control.GEAR_NEUTRAL:
            gear_pwm = GEAR_LOW              

        # 将差速锁转换为PWM信号
        if msg.hand_brake == True:
            diff_lock_pwm = DIFF_LOCK_ON
        else:
            diff_lock_pwm = DIFF_LOCK_OFF
            
        # 发送CAN数据
        success = self.controller.send_can_frames(drive_pwm, steering_pwm, gear_pwm, diff_lock_pwm)
        
        if not success:
            rospy.logwarn("CAN发送失败，尝试重新初始化CAN控制器")
            try:
                # 重新初始化CAN总线
                self.controller.bus.shutdown()
                self.controller.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
                rospy.loginfo("CAN控制器重新初始化成功")
                # 重试发送
                self.controller.send_can_frames(drive_pwm, steering_pwm, gear_pwm, diff_lock_pwm)
            except Exception as e:
                rospy.logerr("CAN控制器重新初始化失败: %s", str(e))
        

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