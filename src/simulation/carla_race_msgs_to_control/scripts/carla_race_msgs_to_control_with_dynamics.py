#!/usr/bin/env python3
#
# Copyright (c) 2025 Komasa Qi THICV
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive race_msgs::Control and publish carla_msgs::CarlaEgoVehicleControl
use max wheel steer angle
"""

import sys
import rospy
from rospy.exceptions import ROSException
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from race_msgs.msg import Control  # 确保race_msgs在ROS1中存在
import numpy as np




class ControlToVehicleControlWithDynamics: 
    """
    接收Control消息并转换为Carla车辆控制指令
    使用最大车轮转向角
    """

    MAX_LON_ACCELERATION = 10

    def __init__(self):
        """初始化节点和回调函数"""
        # ROS1中不需要继承Node类，直接初始化节点名称
        rospy.init_node("race_msgs_to_control_with_dynamics")
        # 打印节点命名空间
        rospy.logwarn("race_msgs_to_control_with_dynamics 节点命名空间为: %s", rospy.get_namespace())
     

        
        # 获取参数
        self.role_name = rospy.get_param("~role_name", "ego_vehicle")
        self.steer_Lag = rospy.get_param("~steer_Lag", 0.05)
        self.acc_Lag = rospy.get_param("~acc_Lag", 0.05)
        self.brake_Lag = rospy.get_param("~brake_Lag", 0.05)
        self.steer_tau = rospy.get_param("~steer_tau", 0.2)
        self.acc_tau = rospy.get_param("~acc_tau", 0.05)
        self.brake_tau = rospy.get_param("~brake_tau", 0.05)
        self.steer_max_vel = rospy.get_param("~steer_max_vel", 0.7854) # 45度/s
        
        self.store_periods = rospy.get_param("~store_periods", 5) # 存储的指令周期数量
        
        self.actual_steer = 0.0
        self.actual_acc = 0.0
        self.actual_brake = 0.0
        
        self.steer_cmd_store = np.array([0.0] * self.store_periods)
        self.acc_cmd_store = np.array([0.0] * self.store_periods)
        self.brake_cmd_store = np.array([0.0] * self.store_periods)
        self.cmd_time_store = None
        self.max_steering_angle = None

        # 订阅车辆信息
        self.vehicle_info_sub = rospy.Subscriber(
            f"/carla/{self.role_name}/vehicle_info",
            CarlaEgoVehicleInfo,
            self.update_vehicle_info,
            queue_size=1
        )

        # 订阅Control消息
        self.race_control_sub = rospy.Subscriber(
            f"/race/control",
            Control,
            self.race_control_received,
            queue_size=10
        )

        # 创建控制指令发布者
        self.control_pub = rospy.Publisher(
            f"/carla/{self.role_name}/vehicle_control_cmd",
            CarlaEgoVehicleControl,
            queue_size=10
        )

    def update_vehicle_info(self, vehicle_info):
        """更新车辆最大转向角信息"""
        if not vehicle_info.wheels:
            rospy.logerr("无法获取车辆车轮信息，无法确定最大转向角")
            sys.exit(1)

        self.max_steering_angle = vehicle_info.wheels[0].max_steer_angle
        if not self.max_steering_angle:
            rospy.logerr(f"最大转向角获取失败，值为: {self.max_steering_angle}")
            sys.exit(1)
        rospy.loginfo(f"车辆信息已接收，最大转向角: {self.max_steering_angle}")

    def race_control_received(self, race_control):
        """将race_msgs::Control消息转换为车辆控制指令并发布"""
        if self.max_steering_angle is None:
            rospy.logwarn("尚未接收车辆信息，暂不处理控制指令")
            return
        if self.cmd_time_store is None:
            # ========== 关键修改：初始化时间数组为连续的过去时间 ==========
            # 获取当前时间（节点启动后的时间）
            current_time = rospy.get_time()
            print(f"当前时间: {current_time}")
            # 假设控制周期为0.05秒（20Hz），可根据实际频率调整
            # 生成 [current_time - (N-1)*dt, ..., current_time - dt]
            dt = 0.05  # 控制周期（与实际接收频率一致）
            self.cmd_time_store = np.array([
                current_time - i * dt 
                for i in range(self.store_periods-1, -1, -1)  # 从N-1到0倒序
            ])
            # 验证初始时间数组（应接近current_time，且间隔均匀）
            rospy.loginfo("初始时间数组: %s", self.cmd_time_store)
        control = CarlaEgoVehicleControl()
        
        # 存储指令
        self.steer_cmd_store = np.roll(self.steer_cmd_store, -1)
        self.acc_cmd_store = np.roll(self.acc_cmd_store, -1)
        self.brake_cmd_store = np.roll(self.brake_cmd_store, -1)
        self.cmd_time_store = np.roll(self.cmd_time_store, -1)
        
        self.steer_cmd_store[-1] = -race_control.lateral.steering_angle
        # rospy.logwarn(f"steer_cmd stored: {race_control.lateral.steering_angle}")
        # rospy.logwarn(f"steer_cmd stored: {self.steer_cmd_store}")
        
        self.acc_cmd_store[-1] = race_control.throttle
        self.brake_cmd_store[-1] = race_control.brake
        self.cmd_time_store[-1] = rospy.get_time()
        
        ts = self.cmd_time_store[-1] - self.cmd_time_store[-2]
        
        # numpy插值计算去掉延迟的控制指令
        steer_cmd = np.interp(self.cmd_time_store[-1]-self.steer_Lag, self.cmd_time_store, self.steer_cmd_store)
        acc_cmd = np.interp(self.cmd_time_store[-1]-self.acc_Lag, self.cmd_time_store, self.acc_cmd_store)
        brake_cmd = np.interp(self.cmd_time_store[-1]-self.brake_Lag, self.cmd_time_store, self.brake_cmd_store)
        
        # rospy.loginfo(f"current time: {self.cmd_time_store[-1]}, stored times: {self.cmd_time_store}, lagged steering time:{self.cmd_time_store[-1]-self.steer_Lag}, stored steer_cmd:{self.steer_cmd_store}, steer_cmd: {steer_cmd}, acc_cmd: {acc_cmd}, brake_cmd: {brake_cmd}")
        # 计算带有一节动力学的实际动态
        self.actual_steer = self.actual_steer + float(np.clip((steer_cmd - self.actual_steer)  / self.steer_tau, -self.steer_max_vel, self.steer_max_vel) * ts)
        self.actual_acc = self.actual_acc + float(acc_cmd - self.actual_acc) * ts / self.acc_tau
        self.actual_brake = self.actual_brake + float(brake_cmd - self.actual_brake) * ts / self.brake_tau
        rospy.loginfo(f"actual_steer: {self.actual_steer}, actual_acc: {self.actual_acc}, actual_brake: {self.actual_brake}")
        # 处理油门和倒车
        control.throttle = float(self.actual_acc)
        control.brake = float(self.actual_brake)
        if race_control.gear == Control.GEAR_REVERSE:
            control.reverse = True
        else:
            control.reverse = False
            
        control.steer = float(self.actual_steer)
        
        if race_control.emergency:
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0

        control.hand_brake = race_control.hand_brake
        
        
        # 发布控制指令
        try:
            self.control_pub.publish(control)
        except ROSException as e:
            if not rospy.is_shutdown():
                rospy.logwarn(f"发布控制指令失败: {e}")


def main(args=None):
    """主函数"""
    try:
        # 直接创建类实例（类内部已初始化节点）
        control_to_control = ControlToVehicleControlWithDynamics()
        rospy.spin()  # 保持节点运行
    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")
    finally:
        rospy.loginfo("程序退出")


if __name__ == "__main__":
    main()
