#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import threading
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import MCP2515

# 导入服务定义
from vel_pkg.srv import EngineControl, EngineControlResponse
from vel_pkg.srv import GearControl, GearControlResponse
from vel_pkg.srv import ThrottleControl, ThrottleControlResponse
from vel_pkg.srv import BrakeControl, BrakeControlResponse
from vel_pkg.srv import SteeringControl, SteeringControlResponse
from vel_pkg.srv import LightControl, LightControlResponse
from vel_pkg.srv import SpeedGearControl, SpeedGearControlResponse
from vel_pkg.srv import DiffLockControl, DiffLockControlResponse
from vel_pkg.srv import VolumeControl, VolumeControlResponse
from vel_pkg.srv import EmergencyStop, EmergencyStopResponse

class VehicleController:
    def __init__(self):
        # 初始化CAN控制器
        self.can = MCP2515.MCP2515()
        self.can.Init()
        rospy.loginfo("MCP2515 CAN控制器初始化完成")
        
        # 安全数据 (1-4为1500，5-9为1000)
        self.SAFE_DATA = [1500, 1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000]
        
        # 当前通道值 (初始化为安全值)
        self.current_channels = self.SAFE_DATA.copy()
        
        # 油门状态
        self.active_pulse = None
        # 初始化通道值为1500（中位）
        self.channel_value = 1500

        # 引擎状态
        self.engine_operation_in_progress = False
        self.horn_operation_in_progress = False  # 喇叭操作标志
        self.engine_running = False
        
        # 灯光序列状态
        self.light_sequence_active = False
        self.light_sequence_step = 0
        self.light_sequence_last_time = 0
        
        # 创建互斥锁
        self.lock = threading.Lock()
        
        # 设置关闭时安全回调
        rospy.on_shutdown(self.shutdown_hook)
    
    def send_can_frames(self):
        """发送当前通道值到CAN总线"""
        with self.lock:
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
            ch6 = channels[5] - 1000
            ch7 = channels[6] - 1000
            ch8 = channels[7] - 1000
            ch9 = channels[8] - 1000

            # 构建第一帧数据（通道1-4）
            data1 = [
                ch1 & 0xFF, (ch1 >> 8) & 0xFF,
                ch2 & 0xFF, (ch2 >> 8) & 0xFF,
                ch3 & 0xFF, (ch3 >> 8) & 0xFF,
                ch4 & 0xFF, (ch4 >> 8) & 0xFF
            ]
            
            # 发送第一帧
            try:
                self.can.Send(0x100, data1, 8)
                rospy.loginfo("发送 CAN 0x100: [%s]", ' '.join(f"{b:02X}" for b in data1))
            except Exception as e:
                rospy.logerr("CAN 0x100 发送失败: %s", str(e))
            
            # 构建第二帧数据（通道6-9）
            data2 = [
                ch6 & 0xFF, (ch6 >> 8) & 0xFF,
                ch7 & 0xFF, (ch7 >> 8) & 0xFF,
                ch8 & 0xFF, (ch8 >> 8) & 0xFF,
                ch9 & 0xFF, (ch9 >> 8) & 0xFF
            ]
            
            # 发送第二帧
            try:
                self.can.Send(0x101, data2, 8)
                rospy.loginfo("发送 CAN 0x101: [%s]", ' '.join(f"{b:02X}" for b in data2))
            except Exception as e:
                rospy.logerr("CAN 0x101 发送失败: %s", str(e))

    def set_channel(self, channel_idx, value, duration=0):
        """
        设置通道值
        :param channel_idx: 通道索引 (1-9)
        :param value: PWM值 (1000-2000)
        :param duration: 保持时间 (秒), 0表示永久
        """
        with self.lock:
            # 创建定时器恢复原始值
            original_value = self.current_channels[channel_idx - 1]

            # 确保值在范围内
            safe_value = max(1000, min(2000, value))
            
            # 更新通道值
            self.current_channels[channel_idx - 1] = safe_value
            
            # 如果需要临时设置
            if duration > 0:
                def reset_value():
                    time.sleep(duration)
                    with self.lock:
                        self.current_channels[channel_idx - 1] = original_value
                threading.Thread(target=reset_value).start()

    # ===================== 引擎控制方法 =====================
    def engine_control(self, command):
        """统一处理引擎控制命令"""
        rospy.loginfo(f"收到引擎控制命令: {command}")
        
        if command == 0:  # 启动引擎
            return self.start_engine()
        elif command == 1:  # 停止引擎
            return self.stop_engine()
        elif command == 2:  # 按喇叭
            return self.sound_horn()
        else:
            return False, "无效的引擎命令"
        
    def start_engine(self):
        """启动引擎"""
        if self.engine_operation_in_progress:
            return False, "引擎操作正在进行中"
            
        if not self.engine_running:
            rospy.logwarn("启动引擎")
            self.engine_operation_in_progress = True
            self.set_channel(2, 1000, 0.2)  # 通道2保持1000 0.2秒
            # 添加延时重置标志
            rospy.Timer(rospy.Duration(0.2), self._reset_engine_operation_flag, oneshot=True)

            self.engine_running = True
            return True, "引擎启动中..."
        else:
            rospy.logwarn("引擎已在运行状态")
            return False, "引擎已在运行状态"

    def stop_engine(self):
        """停止引擎"""
        if self.engine_operation_in_progress:
            return False, "引擎操作正在进行中"
            
        if self.engine_running:
            rospy.logwarn("停止引擎")
            self.engine_operation_in_progress = True
            self.set_channel(2, 1000, 0.2)  # 通道2保持1000 0.2秒
            # 添加延时重置标志
            rospy.Timer(rospy.Duration(0.2), self._reset_engine_operation_flag, oneshot=True)

            self.engine_running = False
            return True, "引擎停止中..."
        else:
            rospy.logwarn("引擎已在停止状态")
            return False, "引擎已在停止状态"
            
    def _reset_engine_operation_flag(self, event):
        self.engine_operation_in_progress = False
        rospy.logdebug("引擎操作标志已重置")

    def sound_horn(self):
        """按喇叭"""
        if self.horn_operation_in_progress:
            return False, "喇叭操作正在进行中"
            
        rospy.logwarn("按喇叭")
        self.horn_operation_in_progress = True
        self.set_channel(2, 2000, 0.2)  # 通道1保持2000 0.1秒
        # 添加延时重置标志
        rospy.Timer(rospy.Duration(0.3), self._reset_horn_flag, oneshot=True)
        return True, "喇叭已响"
        
    def _reset_horn_flag(self, event):
        self.horn_operation_in_progress = False
        rospy.logdebug("喇叭操作标志已重置")

    # ===================== 挡位控制方法 =====================
    def set_gear_forward(self):
        """挂前进挡"""
        rospy.logwarn("挂前进挡")
        # 脉冲操作: 2000 → 1500
        self.set_channel(1, 2000, 0.2)  # 保持2000 200ms
        return True, "前进挡已挂"

    def set_gear_reverse(self):
        """挂倒挡"""
        rospy.logwarn("挂倒挡")
        # 脉冲操作: 1000 → 1500
        self.set_channel(1, 1000, 0.2)  # 保持1000 200ms
        return True, "倒挡已挂"

    def set_gear_neutral(self):
        """设置空挡"""
        rospy.logwarn("设置空挡")
        self.set_channel(1, 1500)
        return True, "空挡已设置"

    # ===================== 油门/刹车控制方法 =====================
    def set_throttle(self, value):
        """
        设置油门深度（瞬时操作，0.5秒后回中）
        :param value: 油门深度 [0.0-1.0], 0=无油门, 1=全油门
        """
        # 映射到PWM值: 1500(无油门) → 1000(全油门)
        pwm_value = 1500 + int(value * 500)
        pwm_value = max(1500, min(2000, pwm_value))
        
        # 发送瞬时油门脉冲
        rospy.logwarn(f"[油门] 设置: {pwm_value} (原始值: {value:.2f})")
        self._send_pulse(3, pwm_value)
        return True

    def set_brake(self, value):
        """
        设置刹车深度（瞬时操作，0.5秒后回中）
        :param value: 刹车深度 [0.0-1.0], 0=无刹车, 1=全刹车
        """
        # 映射到PWM值: 1500(无刹车) → 2000(全刹车)
        pwm_value = 1500 - int(value * 500)
        pwm_value = max(1000, min(1500, pwm_value))
        
        # 发送瞬时刹车脉冲
        rospy.logwarn(f"[刹车] 设置: {pwm_value} (原始值: {value:.2f})")
        self._send_pulse(3, pwm_value)
        return True
    def _send_pulse(self, channel, value):
        """发送脉冲并管理定时器"""
        # 取消任何现有的脉冲定时器
        if self.active_pulse:
            self.active_pulse.shutdown()
        
        # 设置新值
        self.set_channel(channel, value)
        
        # 创建1秒后回中的定时器
        self.active_pulse = rospy.Timer(
            rospy.Duration(0.5), 
            lambda event: self._reset_channel(channel),
            oneshot=True
        )

    def _reset_channel(self, channel):
        """重置通道到中位"""
        rospy.logwarn(f"[通道复位] 通道: {channel} → 1500")
        self.set_channel(channel, 1500)

    # ===================== 转向控制方法 =====================
    def set_steering(self, value):
        """
        设置转向
        :param value: 转向角度 [-1.0-1.0], -1=右满舵, 0=居中, 1=左满舵
        """
        # 映射到PWM值: 1000(右满舵) → 1500(居中) → 2000(左满舵)
        pwm_value = 1500 + int(value * 500)
        self.set_channel(4, pwm_value)
        return True

    # ===================== 速度挡位控制方法 =====================
    def set_speed_gear(self, gear):
        """
        设置速度挡位
        :param gear: 0=默认挡位, 1=二挡
        """
        if gear == 0:
            rospy.logwarn("设置速度挡位: 默认挡")
            self.set_channel(6, 1000)
        elif gear == 1:
            rospy.logwarn("设置速度挡位: 二挡")
            self.set_channel(6, 2000)
        return True

    # ===================== 差速锁控制方法 =====================
    def set_diff_lock(self, state):
        """
        设置差速锁
        :param state: True=打开, False=关闭
        """
        if state:
            rospy.logwarn("打开差速锁")
            self.set_channel(8, 2000)
        else:
            rospy.logwarn("关闭差速锁")
            self.set_channel(8, 1000)
        return True

    # ===================== 音量控制方法 =====================
    def set_volume(self, level):
        """
        设置音量
        :param level: 0=最小, 1=中等, 2=最大
        """
        if level == 0:
            self.set_channel(9, 1000)
        elif level == 1:
            self.set_channel(9, 1500)
        elif level == 2:
            self.set_channel(9, 2000)
        return True

    # ===================== 灯光控制方法 =====================
    def set_hazard_lights(self, on):
        """
        设置双闪灯
        :param on: True=开启, False=关闭
        """
        if on:
            rospy.logwarn("开启双闪灯")
            self.set_channel(7, 1000)
        else:
            rospy.logwarn("关闭双闪灯")
            self.set_channel(7, 2000)
        return True
    
    def activate_turn_signal(self):
        """激活转向灯序列"""
        if not self.light_sequence_active:
            rospy.logwarn("启动转向灯序列")
            self.light_sequence_active = True
            self.light_sequence_step = 0
            self.light_sequence_last_time = time.time()
        return True
    
    def update_light_sequence(self):
        """更新转向灯序列状态"""
        if self.light_sequence_active:
            now = time.time()
            
            # 序列步骤定义: (目标值, 持续时间)
            sequence = [
                (1000, 0.1),  # 给1000，保持0.1s
                (1500, 0.1),  # 给1500，保持0.1s → 开灯1
                (1000, 0.1),  # 给1000，保持0.1s
                # (1500, 0.1),  # 给1500，保持0.1s → 开灯2
                # (1500, 0.1),  # 给1500，保持0.1s
                # (1500, 0.1),  # 给1500，保持0.1s → 开灯3
                # (1000, 0.1),  # 给1000，保持0.1s
                # (1500, 0.1)   # 给1500，保持0.1s → 关所有灯
            ]
            
            if self.light_sequence_step < len(sequence):
                target_value, duration = sequence[self.light_sequence_step]
                
                # 设置当前值
                self.set_channel(7, target_value)
                
                # 检查是否完成当前步骤
                if now - self.light_sequence_last_time > duration:
                    self.light_sequence_step += 1
                    self.light_sequence_last_time = now
            else:
                # 序列完成
                self.light_sequence_active = False
                rospy.logwarn("转向灯序列完成")

    # ===================== 安全方法 =====================
    def emergency_stop(self):
        """紧急停止所有操作"""
        rospy.logwarn("执行紧急停止!")
        self.current_channels = self.SAFE_DATA.copy()
        self.engine_running = False
        self.send_can_frames()
        return True

    def shutdown_hook(self):
        """ROS关闭时发送安全值"""
        rospy.loginfo("发送安全停止指令")
        self.emergency_stop()
        time.sleep(0.1)  # 确保指令发送


class ChassisNode:
    def __init__(self):
        rospy.init_node('chassis_controller')
        rospy.loginfo("底盘控制器节点启动")
        
        # 创建车辆控制器
        self.controller = VehicleController()
        
        # 订阅veh_cmd话题（只用于ch3油门/刹车和ch4转向）
        rospy.Subscriber("/veh_cmd", Twist, self.veh_cmd_callback, queue_size=1)
        
        # 设置高级控制服务
        self.setup_services()
        
        # 设置更新定时器 (50Hz)
        self.update_timer = rospy.Timer(rospy.Duration(0.02), self.update_callback)
        
        rospy.loginfo("节点初始化完成，等待指令...")

    def setup_services(self):
        """设置所有高级控制服务"""
        # 引擎控制服务
        self.engine_service = rospy.Service('/vehicle/engine', EngineControl, self.handle_engine)
        
        # 挡位控制服务
        self.gear_service = rospy.Service('/vehicle/gear', GearControl, self.handle_gear)
        
        # 油门控制服务
        self.throttle_service = rospy.Service('/vehicle/throttle', ThrottleControl, self.handle_throttle)
        
        # 刹车控制服务
        self.brake_service = rospy.Service('/vehicle/brake', BrakeControl, self.handle_brake)
        
        # 转向控制服务
        self.steering_service = rospy.Service('/vehicle/steering', SteeringControl, self.handle_steering)
        
        # 灯光控制服务
        self.light_service = rospy.Service('/vehicle/lights', LightControl, self.handle_lights)
        
        # 速度挡位服务
        self.speed_gear_service = rospy.Service('/vehicle/speed_gear', SpeedGearControl, self.handle_speed_gear)
        
        # 差速锁服务
        self.diff_lock_service = rospy.Service('/vehicle/diff_lock', DiffLockControl, self.handle_diff_lock)
        
        # 音量控制服务
        self.volume_service = rospy.Service('/vehicle/volume', VolumeControl, self.handle_volume)
        
        # 紧急停止服务
        self.emergency_service = rospy.Service('/vehicle/emergency_stop', EmergencyStop, self.handle_emergency)
        
        rospy.loginfo("高级控制服务已启动")

    # ===================== 服务处理函数 =====================
    def handle_engine(self, req):
        """处理引擎控制请求"""
        success, message = self.controller.engine_control(req.command)
        return EngineControlResponse(success, message)
    
    def handle_gear(self, req):
        """处理挡位控制请求"""
        if req.gear == 0:  # 空挡
            success, message = self.controller.set_gear_neutral()
        elif req.gear == 1:  # 前进挡
            success, message = self.controller.set_gear_forward()
        elif req.gear == 2:  # 倒挡
            success, message = self.controller.set_gear_reverse()
        else:
            success = False
            message = "无效的挡位命令"
        return GearControlResponse(success, message)
    
    def handle_throttle(self, req):
        """处理油门控制请求"""
        if 0.0 <= req.value <= 1.0:
            success = self.controller.set_throttle(req.value)
            return ThrottleControlResponse(success)
        return ThrottleControlResponse(False)
    
    def handle_brake(self, req):
        """处理刹车控制请求"""
        if 0.0 <= req.value <= 1.0:
            success = self.controller.set_brake(req.value)
            return BrakeControlResponse(success)
        return BrakeControlResponse(False)
    
    def handle_steering(self, req):
        """处理转向控制请求"""
        if -1.0 <= req.angle <= 1.0:
            success = self.controller.set_steering(req.angle)
            return SteeringControlResponse(success)
        return SteeringControlResponse(False)
    
    def handle_lights(self, req):
        """处理灯光控制请求"""
        if req.command == 0:  # 关双闪
            success = self.controller.set_hazard_lights(False)
        elif req.command == 1:  # 开双闪
            success = self.controller.set_hazard_lights(True)
        elif req.command == 2:  # 转向灯序列
            success = self.controller.activate_turn_signal()
        else:
            success = False
        return LightControlResponse(success)
    
    def handle_speed_gear(self, req):
        """处理速度挡位请求"""
        if req.gear in [0, 1]:
            success = self.controller.set_speed_gear(req.gear)
            return SpeedGearControlResponse(success)
        return SpeedGearControlResponse(False)
    
    def handle_diff_lock(self, req):
        """处理差速锁请求"""
        success = self.controller.set_diff_lock(req.lock)
        return DiffLockControlResponse(success)
    
    def handle_volume(self, req):
        """处理音量控制请求"""
        if req.level in [0, 1, 2]:
            success = self.controller.set_volume(req.level)
            return VolumeControlResponse(success)
        return VolumeControlResponse(False)
    
    def handle_emergency(self, req):
        """处理紧急停止请求"""
        success = self.controller.emergency_stop()
        return EmergencyStopResponse(success)

    def veh_cmd_callback(self, msg):
        """
        处理/veh_cmd话题回调 (Twist)
        linear.x: 油门 0-100
        linear.y: 刹车 0-100
        angular.z: 转向 -15~15
        """
        try:
            # 获取输入
            throttle = msg.linear.x
            brake = msg.linear.y
            steering = msg.angular.z

            # Clamp限制范围
            throttle = max(-100, min(100, throttle))
            brake = max(0.0, min(100.0, brake))
            steering = max(-15.0, min(15.0, steering))

            # 默认PWM
            pwm_channel2 = 1500  # 油门/刹车
            steering_pwm = int(1500 + steering / 15.0 * 500)

            # 油门/刹车互斥
            if brake > 0:
                # 刹车
                pwm_channel2 = int(1500 - brake * 5)
                pwm_channel2 = max(1000, min(1500, pwm_channel2))
            else:
                # 油门
                pwm_channel2 = int(1500 + throttle * 5)
                pwm_channel2 = max(1000, min(2000, pwm_channel2))


            # 限幅转向
            steering_pwm = max(1000, min(2000, steering_pwm))

            with self.controller.lock:
                self.controller.current_channels[2] = pwm_channel2
                self.controller.current_channels[3] = steering_pwm

            rospy.logdebug(
                "veh_cmd: throttle=%.1f brake=%.1f steering=%.1f -> channel2_pwm=%d steering_pwm=%d",
                throttle, brake, steering,
                pwm_channel2, steering_pwm
            )

        except Exception as e:
            rospy.logerr("veh_cmd处理错误: %s", str(e))


    def update_callback(self, event):
        """定时更新回调"""
        try:
            # 更新灯光序列
            self.controller.update_light_sequence()
            
            # 发送CAN数据
            self.controller.send_can_frames()
            
        except Exception as e:
            rospy.logerr("更新过程中出错: %s", str(e))
            self.controller.emergency_stop()

    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ChassisNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("底盘控制器节点关闭")