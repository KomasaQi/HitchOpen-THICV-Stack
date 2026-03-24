#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
vehicle_state_recorder.py

功能：
1. 订阅 /race/vehicle_state 话题；
2. 将 race_msgs/VehicleStatus 的全部状态量展开并写入 CSV；
3. 支持通过参数输入实验名称；
4. 自动在文件名中追加开始记录时间，避免覆盖历史实验；
5. 自动创建输出目录。
"""

from __future__ import print_function

import csv
import os
import re
import threading
from datetime import datetime

import rospy
from race_msgs.msg import VehicleStatus


class VehicleStateRecorder(object):
    """车辆状态记录节点。"""

    def __init__(self):
        # =========================
        # 1. 读取参数
        # =========================
        self.topic_name = rospy.get_param('~topic_name', '/race/vehicle_state')
        self.experiment_name = rospy.get_param('~experiment_name', 'default_experiment')
        self.output_dir = rospy.get_param('~output_dir', '~/.ros/exp_utils/vehicle_state_logs')
        self.flush_every_n = int(rospy.get_param('~flush_every_n', 1))
        self.use_wall_time_in_filename = bool(rospy.get_param('~use_wall_time_in_filename', True))

        # 展开并规范化输出目录
        self.output_dir = os.path.expanduser(self.output_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # 对实验名称做简单清洗，避免文件名中出现不安全字符
        self.experiment_name = self._sanitize_filename(self.experiment_name)

        # =========================
        # 2. 构造输出文件路径
        # =========================
        self.start_wall_time = datetime.now()
        self.start_wall_time_str = self.start_wall_time.strftime('%Y%m%d_%H%M%S')

        # 文件名中默认使用系统时间，更适合实验记录管理
        if self.use_wall_time_in_filename:
            time_tag = self.start_wall_time_str
        else:
            # 若使用 ROS 时间，则在仿真未启用 /clock 时也可正常工作
            ros_now = rospy.Time.now()
            time_tag = str(ros_now.secs)

        self.csv_filename = '{}_{}.csv'.format(self.experiment_name, time_tag)
        self.csv_path = os.path.join(self.output_dir, self.csv_filename)

        # 若极端情况下重名，则自动追加序号，确保绝不覆盖
        self.csv_path = self._deduplicate_path(self.csv_path)

        # =========================
        # 3. 初始化 CSV
        # =========================
        self.fieldnames = self._build_fieldnames()
        self.file_lock = threading.Lock()
        self.msg_count = 0

        # 使用 line buffering，尽量降低实验中断时的数据丢失风险
        self.csv_file = open(self.csv_path, 'w')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()
        self.csv_file.flush()

        # =========================
        # 4. 订阅话题
        # =========================
        self.sub = rospy.Subscriber(self.topic_name, VehicleStatus, self._callback, queue_size=100)

        rospy.loginfo('================ Vehicle State Recorder ================')
        rospy.loginfo('订阅话题: %s', self.topic_name)
        rospy.loginfo('实验名称: %s', self.experiment_name)
        rospy.loginfo('输出目录: %s', self.output_dir)
        rospy.loginfo('输出文件: %s', self.csv_path)
        rospy.loginfo('CSV字段数: %d', len(self.fieldnames))
        rospy.loginfo('=======================================================')

        # 注册退出时关闭文件，保证 CSV 完整写入
        rospy.on_shutdown(self._on_shutdown)

    @staticmethod
    def _sanitize_filename(name):
        """清理文件名中的非法字符，保留中文、英文、数字、下划线、横杠。"""
        if not isinstance(name, str):
            name = str(name)
        name = name.strip()
        if not name:
            name = 'default_experiment'
        # 将空白字符替换为下划线
        name = re.sub(r'\s+', '_', name)
        # 去除不安全字符
        name = re.sub(r'[^\w\-\u4e00-\u9fff]', '_', name)
        # 避免连续下划线太多
        name = re.sub(r'_+', '_', name)
        return name

    @staticmethod
    def _deduplicate_path(path):
        """若文件已存在，则自动在末尾追加序号，避免覆盖。"""
        if not os.path.exists(path):
            return path

        base, ext = os.path.splitext(path)
        idx = 1
        while True:
            new_path = '{}_{:03d}{}'.format(base, idx, ext)
            if not os.path.exists(new_path):
                return new_path
            idx += 1

    @staticmethod
    def _build_fieldnames():
        """构造 CSV 表头，覆盖 VehicleStatus 中全部状态量。"""
        return [
            # 记录辅助信息
            'record_seq',
            'record_wall_time_iso',
            'record_wall_time_float',
            'record_ros_time',

            # Header
            'header_seq',
            'header_stamp',
            'header_frame_id',
            'child_frame_id',

            # pose.position
            'pose_position_x',
            'pose_position_y',
            'pose_position_z',

            # pose.orientation
            'pose_orientation_x',
            'pose_orientation_y',
            'pose_orientation_z',
            'pose_orientation_w',

            # euler
            'euler_roll',
            'euler_pitch',
            'euler_yaw',

            # vel.linear / angular
            'vel_linear_x',
            'vel_linear_y',
            'vel_linear_z',
            'vel_angular_x',
            'vel_angular_y',
            'vel_angular_z',

            # acc.linear / angular
            'acc_linear_x',
            'acc_linear_y',
            'acc_linear_z',
            'acc_angular_x',
            'acc_angular_y',
            'acc_angular_z',

            # lateral
            'lateral_steering_angle',
            'lateral_steering_angle_velocity',
            'lateral_rear_wheel_angle',
            'lateral_rear_wheel_angle_velocity',

            # wheel speed
            'wheel_speed_left_front',
            'wheel_speed_left_rear',
            'wheel_speed_right_front',
            'wheel_speed_right_rear',

            # driveline / control state
            'gear',
            'control_mode',
            'hand_brake',
            'emergency',
            'clutch',
            'steering_mode',
            'throttle_fb',
            'brake_fb',

            # tracking
            'tracking_lateral_tracking_error',
            'tracking_heading_angle_error',
            'tracking_velocity_error',

            # trailer.pose.position
            'trailer_pose_position_x',
            'trailer_pose_position_y',
            'trailer_pose_position_z',

            # trailer.pose.orientation
            'trailer_pose_orientation_x',
            'trailer_pose_orientation_y',
            'trailer_pose_orientation_z',
            'trailer_pose_orientation_w',

            # trailer.euler
            'trailer_euler_roll',
            'trailer_euler_pitch',
            'trailer_euler_yaw',

            # trailer.vel.linear / angular
            'trailer_vel_linear_x',
            'trailer_vel_linear_y',
            'trailer_vel_linear_z',
            'trailer_vel_angular_x',
            'trailer_vel_angular_y',
            'trailer_vel_angular_z',

            # trailer.acc.linear / angular
            'trailer_acc_linear_x',
            'trailer_acc_linear_y',
            'trailer_acc_linear_z',
            'trailer_acc_angular_x',
            'trailer_acc_angular_y',
            'trailer_acc_angular_z',

            # ltr state
            'ltr_state_ltr',
            'ltr_state_ltr_rate',
        ]

    def _msg_to_row(self, msg):
        """将 VehicleStatus 消息展开成一行字典。"""
        now_wall = datetime.now()
        now_wall_float = rospy.get_time() if not rospy.is_shutdown() else 0.0
        now_ros_time = rospy.Time.now().to_sec()

        row = {
            # 记录辅助信息
            'record_seq': self.msg_count,
            'record_wall_time_iso': now_wall.strftime('%Y-%m-%d %H:%M:%S.%f'),
            'record_wall_time_float': now_wall_float,
            'record_ros_time': now_ros_time,

            # Header
            'header_seq': msg.header.seq,
            'header_stamp': msg.header.stamp.to_sec(),
            'header_frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,

            # pose.position
            'pose_position_x': msg.pose.position.x,
            'pose_position_y': msg.pose.position.y,
            'pose_position_z': msg.pose.position.z,

            # pose.orientation
            'pose_orientation_x': msg.pose.orientation.x,
            'pose_orientation_y': msg.pose.orientation.y,
            'pose_orientation_z': msg.pose.orientation.z,
            'pose_orientation_w': msg.pose.orientation.w,

            # euler
            'euler_roll': msg.euler.roll,
            'euler_pitch': msg.euler.pitch,
            'euler_yaw': msg.euler.yaw,

            # vel.linear / angular
            'vel_linear_x': msg.vel.linear.x,
            'vel_linear_y': msg.vel.linear.y,
            'vel_linear_z': msg.vel.linear.z,
            'vel_angular_x': msg.vel.angular.x,
            'vel_angular_y': msg.vel.angular.y,
            'vel_angular_z': msg.vel.angular.z,

            # acc.linear / angular
            'acc_linear_x': msg.acc.linear.x,
            'acc_linear_y': msg.acc.linear.y,
            'acc_linear_z': msg.acc.linear.z,
            'acc_angular_x': msg.acc.angular.x,
            'acc_angular_y': msg.acc.angular.y,
            'acc_angular_z': msg.acc.angular.z,

            # lateral
            'lateral_steering_angle': msg.lateral.steering_angle,
            'lateral_steering_angle_velocity': msg.lateral.steering_angle_velocity,
            'lateral_rear_wheel_angle': msg.lateral.rear_wheel_angle,
            'lateral_rear_wheel_angle_velocity': msg.lateral.rear_wheel_angle_velocity,

            # wheel speed
            'wheel_speed_left_front': msg.wheel_speed.left_front,
            'wheel_speed_left_rear': msg.wheel_speed.left_rear,
            'wheel_speed_right_front': msg.wheel_speed.right_front,
            'wheel_speed_right_rear': msg.wheel_speed.right_rear,

            # driveline / control state
            'gear': msg.gear,
            'control_mode': msg.control_mode,
            'hand_brake': msg.hand_brake,
            'emergency': msg.emergency,
            'clutch': msg.clutch,
            'steering_mode': msg.steering_mode,
            'throttle_fb': msg.throttle_fb,
            'brake_fb': msg.brake_fb,

            # tracking
            'tracking_lateral_tracking_error': msg.tracking.lateral_tracking_error,
            'tracking_heading_angle_error': msg.tracking.heading_angle_error,
            'tracking_velocity_error': msg.tracking.velocity_error,

            # trailer.pose.position
            'trailer_pose_position_x': msg.trailer.pose.position.x,
            'trailer_pose_position_y': msg.trailer.pose.position.y,
            'trailer_pose_position_z': msg.trailer.pose.position.z,

            # trailer.pose.orientation
            'trailer_pose_orientation_x': msg.trailer.pose.orientation.x,
            'trailer_pose_orientation_y': msg.trailer.pose.orientation.y,
            'trailer_pose_orientation_z': msg.trailer.pose.orientation.z,
            'trailer_pose_orientation_w': msg.trailer.pose.orientation.w,

            # trailer.euler
            'trailer_euler_roll': msg.trailer.euler.roll,
            'trailer_euler_pitch': msg.trailer.euler.pitch,
            'trailer_euler_yaw': msg.trailer.euler.yaw,

            # trailer.vel.linear / angular
            'trailer_vel_linear_x': msg.trailer.vel.linear.x,
            'trailer_vel_linear_y': msg.trailer.vel.linear.y,
            'trailer_vel_linear_z': msg.trailer.vel.linear.z,
            'trailer_vel_angular_x': msg.trailer.vel.angular.x,
            'trailer_vel_angular_y': msg.trailer.vel.angular.y,
            'trailer_vel_angular_z': msg.trailer.vel.angular.z,

            # trailer.acc.linear / angular
            'trailer_acc_linear_x': msg.trailer.acc.linear.x,
            'trailer_acc_linear_y': msg.trailer.acc.linear.y,
            'trailer_acc_linear_z': msg.trailer.acc.linear.z,
            'trailer_acc_angular_x': msg.trailer.acc.angular.x,
            'trailer_acc_angular_y': msg.trailer.acc.angular.y,
            'trailer_acc_angular_z': msg.trailer.acc.angular.z,

            # ltr state
            'ltr_state_ltr': msg.ltr_state.ltr,
            'ltr_state_ltr_rate': msg.ltr_state.ltr_rate,
        }
        return row

    def _callback(self, msg):
        """订阅回调：收到一帧车辆状态就写入一行 CSV。"""
        with self.file_lock:
            row = self._msg_to_row(msg)
            self.csv_writer.writerow(row)
            self.msg_count += 1

            # 按设定频率 flush，兼顾性能和可靠性
            if self.flush_every_n <= 1 or (self.msg_count % self.flush_every_n == 0):
                self.csv_file.flush()

    def _on_shutdown(self):
        """节点退出时关闭文件。"""
        try:
            with self.file_lock:
                if hasattr(self, 'csv_file') and self.csv_file:
                    self.csv_file.flush()
                    self.csv_file.close()
            rospy.loginfo('vehicle_state_recorder 已停止，CSV 已保存: %s', self.csv_path)
        except Exception as exc:
            rospy.logwarn('关闭 CSV 文件时出现异常: %s', str(exc))


if __name__ == '__main__':
    rospy.init_node('vehicle_state_recorder', anonymous=False)
    recorder = VehicleStateRecorder()
    rospy.spin()
