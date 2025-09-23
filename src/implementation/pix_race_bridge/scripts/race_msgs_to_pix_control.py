#!/usr/bin/env python3
#
# Copyright (c) 2025 Komasa Qi THICV
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive race_msgs::Control and publish can_msgs::ecu
"""

import sys
import rospy
from rospy.exceptions import ROSException

from race_msgs.msg import Control  # 确保race_msgs在ROS1中存在
from can_msgs.msg import  ecu
from race_msgs.msg import VehicleStatus
from race_msgs.msg import Control

PI = 3.1415926
DRIVE = 2
TO_DRIVE = 1
STOPPED = 0
TO_REVERSE = -1
REVERSE = -2

state_change_counter = 0
stopping_counter = 0
STATE_CHANGE_THRESHOLD = 20
STOPPING_THRESHOLD = 80
drive_state = STOPPED
vehicle_gear = 0
race_vehicle_state_ = VehicleStatus()

class ControlToVehicleControl: 
    """
    接收Control消息并转换为can_msgs::Ecu
    """


    def __init__(self):
        """初始化节点和回调函数"""
        # ROS1中不需要继承Node类，直接初始化节点名称
        rospy.init_node("race_msgs_to_pix_control")
        

        # 获取参数
        self.role_name = rospy.get_param("~role_name", "ego_vehicle")
        self.max_steering_angle = None

        # 订阅车辆信息
        self.vehicle_state_sub = rospy.Subscriber(
            f"/race/vehicle_state",
            VehicleStatus,
            self.update_vehicle_state,
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
            f"/ecu",
            ecu,
            queue_size=10
        )

    def update_vehicle_state(self, vehicle_state):
        """更新车辆最大转向角信息"""
        global race_vehicle_state_
        race_vehicle_state_ = vehicle_state
        return

    def race_control_received(self, race_control):
        """将race_msgs::Control消息转换为车辆控制指令并发布"""
        global drive_state
        global state_change_counter, stopping_counter

        ecu_cmd = ecu()

        # 处理油门和倒车
        
        throttle_cmd = race_control.throttle
        speed_cmd = race_control.longitudinal.velocity
        brake_cmd = race_control.brake

        ecu_cmd.steer = -race_control.lateral.steering_angle/PI*180
        ecu_cmd.steer_rear = -race_control.lateral.rear_wheel_angle/PI*180
        if race_control.control_mode == ecu.THROTTLE_CONTROL:
            ecu_cmd.driver_mode = 0
        else:
            ecu_cmd.driver_mode = 1
        
        if race_control.steering_mode == Control.DUAL_STEERING_MODE:
            ecu_cmd.steer_mode = ecu.STEER_FREE
        else:
            ecu_cmd.steer_mode = ecu.STEER_FRONT
        
        
        if race_control.emergency or race_control.hand_brake:
            ecu_cmd.throttle = 0.0
            ecu_cmd.motor = 0.0
            if race_vehicle_state_.vel.linear.x > 0.1:
                ecu_cmd.brake = 1.0 
                ecu_cmd.shift = ecu.SHIFT_D
                drive_state = DRIVE
            elif race_vehicle_state_.vel.linear.x < -0.1:
                ecu_cmd.brake = 1.0
                ecu_cmd.shift = ecu.SHIFT_R
                drive_state = REVERSE
            else:
                ecu_cmd.brake = 0.0
                ecu_cmd.shift = ecu.SHIFT_N
                drive_state = STOPPED
            ecu_cmd.steer = 0.0
        else:
            ecu_cmd.brake = brake_cmd
            if drive_state == STOPPED:
                print("当前为停止状态")
                ecu_cmd.shift = ecu.SHIFT_N
                if race_vehicle_state_.vel.linear.x > 0.1 and race_control.gear < 1:
                    ecu_cmd.brake = 1.0
                    ecu_cmd.throttle = 0.0
                    ecu_cmd.motor = 0.0
                elif race_vehicle_state_.vel.linear.x < -0.1 and race_control.gear > 0:
                    ecu_cmd.brake = 1.0
                    ecu_cmd.throttle = 0.0
                    ecu_cmd.motor = 0.0

                if race_control.control_mode == Control.THROTTLE_BRAKE_ONLY:
                    print("油门刹车控制模式")
                    if throttle_cmd > 0.01:
                        if race_control.gear==Control.GEAR_1:
                            drive_state = TO_DRIVE
                        elif race_control.gear==Control.GEAR_REVERSE:
                            drive_state = TO_REVERSE
                if race_control.control_mode == Control.DES_SPEED_ONLY:
                    print("期望速度控制模式")
                    if speed_cmd > 0.01 and race_control.gear==Control.GEAR_1:
                        drive_state = TO_DRIVE
                    elif speed_cmd < -0.01 and race_control.gear==Control.GEAR_REVERSE:
                        drive_state = TO_REVERSE
            elif drive_state == TO_DRIVE:
                ecu_cmd.shift = ecu.SHIFT_D
                state_change_counter += 1
                ecu_cmd.motor = 0
                ecu_cmd.throttle = 0
                if state_change_counter >= STATE_CHANGE_THRESHOLD:
                    state_change_counter = 0
                    drive_state = DRIVE
            elif drive_state == DRIVE:
                ecu_cmd.shift = ecu.SHIFT_D
                ecu_cmd.motor = speed_cmd
                ecu_cmd.throttle = throttle_cmd
                if race_control.control_mode == Control.THROTTLE_BRAKE_ONLY:
                    if race_control.gear < 1:
                        drive_state = STOPPED
                        ecu_cmd.brake = 1.0
                        ecu_cmd.throttle = 0.0
                        ecu_cmd.motor = 0.0
                        ecu_cmd.shift = ecu.SHIFT_N
                else:
                    if abs(speed_cmd) < 0.01:
                        stopping_counter += 1
                        if stopping_counter >= STOPPING_THRESHOLD:
                            stopping_counter = 0
                            drive_state = STOPPED
                            ecu_cmd.brake = 1.0
                            ecu_cmd.throttle = 0.0
                            ecu_cmd.motor = 0.0
                            ecu_cmd.shift = ecu.SHIFT_N
                    if race_control.gear < 1:
                        drive_state = STOPPED
                        ecu_cmd.brake = 1.0
                        ecu_cmd.throttle = 0.0
                        ecu_cmd.motor = 0.0
                        ecu_cmd.shift = ecu.SHIFT_N
            elif drive_state == REVERSE:
                ecu_cmd.shift = ecu.SHIFT_R
                ecu_cmd.motor = -speed_cmd
                ecu_cmd.throttle = throttle_cmd
                if race_control.control_mode == Control.THROTTLE_BRAKE_ONLY:
                    if race_control.gear > 0:
                        drive_state = STOPPED
                        ecu_cmd.brake = 1.0
                        ecu_cmd.throttle = 0.0
                        ecu_cmd.motor = 0.0
                        ecu_cmd.shift = ecu.SHIFT_N
                else:
                    if abs(speed_cmd) < 0.01:
                        stopping_counter += 1
                        if stopping_counter >= STOPPING_THRESHOLD:
                            stopping_counter = 0
                            drive_state = STOPPED
                            ecu_cmd.brake = 1.0
                            ecu_cmd.throttle = 0.0
                            ecu_cmd.motor = 0.0
                            ecu_cmd.shift = ecu.SHIFT_N
                    if race_control.gear > 0:
                        drive_state = STOPPED
                        ecu_cmd.brake = 1.0
                        ecu_cmd.throttle = 0.0
                        ecu_cmd.motor = 0.0
                        ecu_cmd.shift = ecu.SHIFT_N
            elif drive_state == TO_REVERSE:
                ecu_cmd.shift = ecu.SHIFT_R
                state_change_counter += 1
                ecu_cmd.motor = 0.0
                ecu_cmd.throttle = 0.0
                if state_change_counter >= STATE_CHANGE_THRESHOLD:
                    state_change_counter = 0
                    drive_state = REVERSE
        print(f"drive_state: {drive_state}")
        # 发布控制指令
        try:
            self.control_pub.publish(ecu_cmd)
        except ROSException as e:
            if not rospy.is_shutdown():
                rospy.logwarn(f"发布控制指令失败: {e}")


def main(args=None):
    """主函数"""
    try:
        # 直接创建类实例（类内部已初始化节点）
        control_to_control = ControlToVehicleControl()
        rospy.spin()  # 保持节点运行
    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")
    finally:
        rospy.loginfo("程序退出")


if __name__ == "__main__":
    main()
