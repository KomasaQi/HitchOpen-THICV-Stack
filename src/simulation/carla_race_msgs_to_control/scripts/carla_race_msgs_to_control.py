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

class ControlToVehicleControl: 
    """
    接收Control消息并转换为Carla车辆控制指令
    使用最大车轮转向角
    """

    MAX_LON_ACCELERATION = 10

    def __init__(self):
        """初始化节点和回调函数"""
        # ROS1中不需要继承Node类，直接初始化节点名称
        rospy.init_node("race_msgs_to_control")

        # 获取参数
        self.role_name = rospy.get_param("role_name", "ego_vehicle")
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

        control = CarlaEgoVehicleControl()

        # 处理油门和倒车
        control.throttle = race_control.throttle
        control.brake = race_control.brake
        if race_control.gear == Control.GEAR_REVERSE:
            control.reverse = True
        else:
            control.reverse = False
            
        control.steer = -race_control.lateral.steering_angle
        
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
        control_to_control = ControlToVehicleControl()
        rospy.spin()  # 保持节点运行
    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")
    finally:
        rospy.loginfo("程序退出")


if __name__ == "__main__":
    main()
