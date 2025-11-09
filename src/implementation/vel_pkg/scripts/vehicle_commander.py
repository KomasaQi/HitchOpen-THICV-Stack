#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from vel_pkg.srv import (
    EngineControl, EngineControlRequest,
    GearControl, GearControlRequest,
    ThrottleControl, ThrottleControlRequest,
    BrakeControl, BrakeControlRequest,
    SteeringControl, SteeringControlRequest,
    LightControl, LightControlRequest,
    SpeedGearControl, SpeedGearControlRequest,
    DiffLockControl, DiffLockControlRequest,
    VolumeControl, VolumeControlRequest,
    EmergencyStop, EmergencyStopRequest
)

class VehicleCommander:
    def __init__(self):
        rospy.init_node('vehicle_commander')
        rospy.loginfo("车辆控制节点启动")
        
        # 等待服务可用
        rospy.loginfo("等待服务...")
        rospy.wait_for_service('/vehicle/engine')
        rospy.wait_for_service('/vehicle/gear')
        rospy.wait_for_service('/vehicle/throttle')
        rospy.wait_for_service('/vehicle/brake')
        rospy.wait_for_service('/vehicle/steering')
        rospy.wait_for_service('/vehicle/lights')
        rospy.wait_for_service('/vehicle/speed_gear')
        rospy.wait_for_service('/vehicle/diff_lock')
        rospy.wait_for_service('/vehicle/volume')
        rospy.wait_for_service('/vehicle/emergency_stop')
        
        # 创建服务代理
        self.engine_proxy = rospy.ServiceProxy('/vehicle/engine', EngineControl)
        self.gear_proxy = rospy.ServiceProxy('/vehicle/gear', GearControl)
        self.throttle_proxy = rospy.ServiceProxy('/vehicle/throttle', ThrottleControl)
        self.brake_proxy = rospy.ServiceProxy('/vehicle/brake', BrakeControl)
        self.steering_proxy = rospy.ServiceProxy('/vehicle/steering', SteeringControl)
        self.lights_proxy = rospy.ServiceProxy('/vehicle/lights', LightControl)
        self.speed_gear_proxy = rospy.ServiceProxy('/vehicle/speed_gear', SpeedGearControl)
        self.diff_lock_proxy = rospy.ServiceProxy('/vehicle/diff_lock', DiffLockControl)
        self.volume_proxy = rospy.ServiceProxy('/vehicle/volume', VolumeControl)
        self.emergency_proxy = rospy.ServiceProxy('/vehicle/emergency_stop', EmergencyStop)
        
        rospy.loginfo("所有服务已连接")

    def control_engine(self, command):
        """发送引擎控制命令"""
        rospy.loginfo(f"发送引擎控制命令: {command}")
        try:
            resp = self.engine_proxy(command=command)
            rospy.loginfo(f"引擎控制结果: {resp.success}, {resp.message}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"引擎控制服务调用失败: {e}")
            return False

    def set_gear(self, gear):
        """
        设置挡位
        :param gear: 0=空挡, 1=前进挡, 2=倒挡
        """
        rospy.loginfo(f"设置挡位: {gear}")
        try:
            resp = self.gear_proxy(gear=gear)
            rospy.loginfo(f"挡位设置结果: {resp.success}, {resp.message}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"挡位设置服务调用失败: {e}")
            return False

    def set_throttle(self, value):
        """
        设置油门
        :param value: 油门值 [0.0-1.0]
        """
        rospy.loginfo(f"设置油门: {value}")
        try:
            resp = self.throttle_proxy(value=value)
            rospy.loginfo(f"油门设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"油门设置服务调用失败: {e}")
            return False

    def set_brake(self, value):
        """
        设置刹车
        :param value: 刹车值 [0.0-1.0]
        """
        rospy.loginfo(f"设置刹车: {value}")
        try:
            resp = self.brake_proxy(value=value)
            rospy.loginfo(f"刹车设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"刹车设置服务调用失败: {e}")
            return False

    def set_steering(self, angle):
        """
        设置转向
        :param angle: 转向角度 [-1.0-1.0]
        """
        rospy.loginfo(f"设置转向角度: {angle}")
        try:
            resp = self.steering_proxy(angle=angle)
            rospy.loginfo(f"转向设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"转向设置服务调用失败: {e}")
            return False

    def set_lights(self, command):
        """
        设置灯光
        :param command: 0=关双闪, 1=开双闪, 2=转向灯序列
        """
        rospy.loginfo(f"设置灯光命令: {command}")
        try:
            resp = self.lights_proxy(command=command)
            rospy.loginfo(f"灯光设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"灯光设置服务调用失败: {e}")
            return False

    def set_speed_gear(self, gear):
        """
        设置速度挡位
        :param gear: 0=默认挡, 1=二挡
        """
        rospy.loginfo(f"设置速度挡位: {gear}")
        try:
            resp = self.speed_gear_proxy(gear=gear)
            rospy.loginfo(f"速度挡位设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"速度挡位设置服务调用失败: {e}")
            return False

    def set_diff_lock(self, lock):
        """
        设置差速锁
        :param lock: True=开, False=关
        """
        rospy.loginfo(f"设置差速锁: {'开' if lock else '关'}")
        try:
            resp = self.diff_lock_proxy(lock=lock)
            rospy.loginfo(f"差速锁设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"差速锁设置服务调用失败: {e}")
            return False

    def set_volume(self, level):
        """
        设置音量
        :param level: 0=最小, 1=中等, 2=最大
        """
        rospy.loginfo(f"设置音量: {level}")
        try:
            resp = self.volume_proxy(level=level)
            rospy.loginfo(f"音量设置结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"音量设置服务调用失败: {e}")
            return False

    def emergency_stop(self):
        """紧急停止"""
        rospy.loginfo("发送紧急停止请求...")
        try:
            resp = self.emergency_proxy()
            rospy.loginfo(f"紧急停止结果: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"紧急停止服务调用失败: {e}")
            return False

    def demo_scenario(self):
        """演示车辆控制场景"""
        rospy.loginfo("开始车辆控制演示场景...")
        
        # 1. 启动引擎
        self.control_engine(0)  # 0=启动引擎
        rospy.sleep(3)  # 等待启动完成

        # 2. 按喇叭
        self.control_engine(2)  # 2=按喇叭
        rospy.sleep(1) 
        
        # 3. 挂前进挡
        if not self.set_gear(1):  # 1=前进挡
            rospy.logerr("无法挂前进挡")
            return
            
        rospy.sleep(0.5)
        
        # 4. 设置速度挡位为二挡
        self.set_speed_gear(1)  # 1=二挡
        rospy.sleep(0.5)
        
        # 5. 开启双闪灯
        self.set_lights(1)  # 1=开双闪
        rospy.sleep(1)
        
        # 6. 前进并转向
        rospy.loginfo("前进并转向...")
        self.set_throttle(0.4)  # 40%油门
        self.set_steering(0.3)  # 左转30%
        rospy.sleep(3)
        
        # 7. 刹车停止
        rospy.loginfo("刹车停止...")
        self.set_brake(0.8)  # 80%刹车
        rospy.sleep(1)
        
        # 8. 播放转向灯序列
        rospy.loginfo("播放转向灯序列...")
        self.set_lights(2)  # 2=转向灯序列
        rospy.sleep(2)  # 等待序列完成
        
        # 9. 倒车
        rospy.loginfo("倒车...")
        self.set_gear(2)  # 2=倒挡
        self.set_throttle(0.3)  # 30%油门
        self.set_steering(-0.2)  # 右转20%
        rospy.sleep(2)
        
        # 10. 停车
        rospy.loginfo("停车...")
        self.set_brake(1.0)  # 100%刹车
        rospy.sleep(0.5)
        
        # 11. 设置空挡
        self.set_gear(0)  # 0=空挡
        rospy.sleep(0.5)
        
        # 12. 关闭双闪灯
        self.set_lights(0)  # 0=关双闪
        
        # 13. 停止引擎
        self.control_engine(1)  # 1=停止引擎
        rospy.sleep(3)  # 等待停止完成

        rospy.loginfo("演示场景完成!")

    def run(self):
        """运行节点"""
        # 等待1秒确保所有服务就绪
        rospy.sleep(1)
        
        # 执行演示场景
        self.demo_scenario()
        
        # 保持节点运行
        rospy.spin()

if __name__ == '__main__':
    try:
        commander = VehicleCommander()
        commander.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("车辆控制节点已关闭")