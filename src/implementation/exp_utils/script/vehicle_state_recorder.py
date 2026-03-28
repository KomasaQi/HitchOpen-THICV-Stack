#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
vehicle_state_recorder.py

功能：
1. 同时订阅 /race/vehicle_state 和 /race/control；
2. 不再按消息到达就立即写入，而是采用“缓存 + 定时采样写入”机制；
3. 以固定时间间隔（默认 0.05 s）向 CSV 写入一行；
4. 若某个话题暂时未收到消息，则使用该话题缓存中的当前值；
   若启动后尚未收到任何消息，则使用 0 初值；
5. 自动在文件名中加入实验开始时间，避免覆盖历史实验；
6. 自动创建输出目录；
7. 所有字段展开为扁平列，便于 MATLAB / Python / Excel 后处理。
"""

from __future__ import print_function

import csv
import os
import re
import threading
import time
from copy import deepcopy
from datetime import datetime

import rospy
from race_msgs.msg import VehicleStatus, Control


class VehicleStateRecorder(object):
    """车辆状态与控制记录节点。"""

    def __init__(self):
        # =========================
        # 1. 读取参数
        # =========================
        self.state_topic_name = rospy.get_param('~state_topic_name', '/race/vehicle_state')
        self.control_topic_name = rospy.get_param('~control_topic_name', '/race/control')
        self.experiment_name = rospy.get_param('~experiment_name', 'default_experiment')
        self.output_dir = rospy.get_param(
            '~output_dir',
            '~/komasa/HitchOpen-THICV-Stack/src/implementation/exp_utils/data/vehicle_state_logs'
        )
        self.flush_every_n = int(rospy.get_param('~flush_every_n', 5))
        self.use_wall_time_in_filename = bool(rospy.get_param('~use_wall_time_in_filename', True))
        self.record_period = float(rospy.get_param('~record_period', 0.05))
        self.subscriber_queue_size = int(rospy.get_param('~subscriber_queue_size', 100))
        self.write_header_on_start = bool(rospy.get_param('~write_header_on_start', True))

        if self.record_period <= 0.0:
            rospy.logwarn('参数 ~record_period <= 0，不合法，已强制改为 0.05 s')
            self.record_period = 0.05

        if self.flush_every_n <= 0:
            rospy.logwarn('参数 ~flush_every_n <= 0，不合法，已强制改为 1')
            self.flush_every_n = 1

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

        if self.use_wall_time_in_filename:
            time_tag = self.start_wall_time_str
        else:
            ros_now = rospy.Time.now()
            time_tag = str(ros_now.secs)

        self.csv_filename = '{}_{}.csv'.format(self.experiment_name, time_tag)
        self.csv_path = os.path.join(self.output_dir, self.csv_filename)
        self.csv_path = self._deduplicate_path(self.csv_path)

        # =========================
        # 3. 初始化缓存
        # =========================
        self.data_lock = threading.Lock()

        # 记录器启动时刻（墙钟）
        self.start_wall_time_epoch = time.time()

        # 最新 VehicleStatus 缓存
        self.latest_state_row = self._build_zero_state_dict()
        self.latest_state_msg_ros_time = 0.0
        self.latest_state_wall_time = 0.0
        self.state_received = False

        # 最新 Control 缓存
        self.latest_control_row = self._build_zero_control_dict()
        self.latest_control_msg_ros_time = 0.0
        self.latest_control_wall_time = 0.0
        self.control_received = False

        # 记录计数
        self.record_count = 0

        # =========================
        # 4. 初始化 CSV
        # =========================
        self.fieldnames = self._build_fieldnames()
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)

        if self.write_header_on_start:
            self.csv_writer.writeheader()
            self.csv_file.flush()

        # =========================
        # 5. 订阅话题
        # =========================
        self.state_sub = rospy.Subscriber(
            self.state_topic_name,
            VehicleStatus,
            self._state_callback,
            queue_size=self.subscriber_queue_size
        )

        self.control_sub = rospy.Subscriber(
            self.control_topic_name,
            Control,
            self._control_callback,
            queue_size=self.subscriber_queue_size
        )

        # =========================
        # 6. 启动定时写入器
        # =========================
        self.timer = rospy.Timer(rospy.Duration(self.record_period), self._timer_callback)

        rospy.loginfo('================ Vehicle State Recorder ================')
        rospy.loginfo('状态话题: %s', self.state_topic_name)
        rospy.loginfo('控制话题: %s', self.control_topic_name)
        rospy.loginfo('实验名称: %s', self.experiment_name)
        rospy.loginfo('输出目录: %s', self.output_dir)
        rospy.loginfo('输出文件: %s', self.csv_path)
        rospy.loginfo('固定记录周期: %.6f s (%.2f Hz)', self.record_period, 1.0 / self.record_period)
        rospy.loginfo('flush_every_n: %d', self.flush_every_n)
        rospy.loginfo('CSV字段数: %d', len(self.fieldnames))
        rospy.loginfo('=======================================================')

        rospy.on_shutdown(self._on_shutdown)

    # =========================================================
    # 基础辅助函数
    # =========================================================
    @staticmethod
    def _sanitize_filename(name):
        """清理文件名中的非法字符，保留中文、英文、数字、下划线、横杠。"""
        if not isinstance(name, str):
            name = str(name)
        name = name.strip()
        if not name:
            name = 'default_experiment'
        name = re.sub(r'\s+', '_', name)
        name = re.sub(r'[^\w\-\u4e00-\u9fff]', '_', name)
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
    def _safe_to_sec(stamp):
        """安全地将 ROS Time 转为秒。"""
        try:
            return stamp.to_sec()
        except Exception:
            return 0.0

    # =========================================================
    # 字段定义
    # =========================================================
    @staticmethod
    def _build_fieldnames():
        """构造 CSV 表头，覆盖 VehicleStatus + Control 的全部字段。"""
        return [
            # ========= 记录辅助信息 =========
            'record_seq',
            'record_wall_time_iso',
            'record_wall_time_float',
            'record_ros_time',
            'record_dt_since_start',
            'record_period',

            # 当前缓存是否收到过真实消息
            'state_received',
            'control_received',

            # 两个缓存最新消息的时间信息
            'state_msg_ros_time',
            'state_msg_wall_time',
            'state_msg_age_ros',
            'state_msg_age_wall',
            'control_msg_ros_time',
            'control_msg_wall_time',
            'control_msg_age_ros',
            'control_msg_age_wall',

            # ========= VehicleStatus.header =========
            'header_seq',
            'header_stamp',
            'header_frame_id',
            'child_frame_id',

            # ========= VehicleStatus.pose =========
            'pose_position_x',
            'pose_position_y',
            'pose_position_z',
            'pose_orientation_x',
            'pose_orientation_y',
            'pose_orientation_z',
            'pose_orientation_w',

            # ========= VehicleStatus.euler =========
            'euler_roll',
            'euler_pitch',
            'euler_yaw',

            # ========= VehicleStatus.vel =========
            'vel_linear_x',
            'vel_linear_y',
            'vel_linear_z',
            'vel_angular_x',
            'vel_angular_y',
            'vel_angular_z',

            # ========= VehicleStatus.acc =========
            'acc_linear_x',
            'acc_linear_y',
            'acc_linear_z',
            'acc_angular_x',
            'acc_angular_y',
            'acc_angular_z',

            # ========= VehicleStatus.lateral =========
            'lateral_steering_angle',
            'lateral_steering_angle_velocity',
            'lateral_rear_wheel_angle',
            'lateral_rear_wheel_angle_velocity',

            # ========= VehicleStatus.wheel_speed =========
            'wheel_speed_left_front',
            'wheel_speed_left_rear',
            'wheel_speed_right_front',
            'wheel_speed_right_rear',

            # ========= VehicleStatus driveline/control state =========
            'gear',
            'control_mode',
            'hand_brake',
            'emergency',
            'clutch',
            'steering_mode',
            'throttle_fb',
            'brake_fb',

            # ========= VehicleStatus.tracking =========
            'tracking_lateral_tracking_error',
            'tracking_heading_angle_error',
            'tracking_velocity_error',

            # ========= VehicleStatus.trailer.pose =========
            'trailer_pose_position_x',
            'trailer_pose_position_y',
            'trailer_pose_position_z',
            'trailer_pose_orientation_x',
            'trailer_pose_orientation_y',
            'trailer_pose_orientation_z',
            'trailer_pose_orientation_w',

            # ========= VehicleStatus.trailer.euler =========
            'trailer_euler_roll',
            'trailer_euler_pitch',
            'trailer_euler_yaw',

            # ========= VehicleStatus.trailer.vel =========
            'trailer_vel_linear_x',
            'trailer_vel_linear_y',
            'trailer_vel_linear_z',
            'trailer_vel_angular_x',
            'trailer_vel_angular_y',
            'trailer_vel_angular_z',

            # ========= VehicleStatus.trailer.acc =========
            'trailer_acc_linear_x',
            'trailer_acc_linear_y',
            'trailer_acc_linear_z',
            'trailer_acc_angular_x',
            'trailer_acc_angular_y',
            'trailer_acc_angular_z',

            # ========= VehicleStatus.ltr_state =========
            'ltr_state_ltr',
            'ltr_state_ltr_rate',

            # ========= Control.header =========
            'control_header_seq',
            'control_header_stamp',
            'control_header_frame_id',

            # ========= Control.longitudinal =========
            'control_longitudinal_velocity',
            'control_longitudinal_acceleration',
            'control_longitudinal_jerk',

            # ========= Control.lateral =========
            'control_lateral_steering_angle',
            'control_lateral_steering_angle_velocity',
            'control_lateral_rear_wheel_angle',
            'control_lateral_rear_wheel_angle_velocity',

            # ========= Control basic =========
            'control_throttle',
            'control_brake',
            'control_gear',
            'control_control_mode',
            'control_emergency',
            'control_hand_brake',
            'control_clutch',
            'control_steering_mode',
        ]

    @staticmethod
    def _build_zero_state_dict():
        """构造 VehicleStatus 对应的全 0 初值字典。"""
        return {
            'header_seq': 0,
            'header_stamp': 0.0,
            'header_frame_id': '',
            'child_frame_id': '',

            'pose_position_x': 0.0,
            'pose_position_y': 0.0,
            'pose_position_z': 0.0,
            'pose_orientation_x': 0.0,
            'pose_orientation_y': 0.0,
            'pose_orientation_z': 0.0,
            'pose_orientation_w': 0.0,

            'euler_roll': 0.0,
            'euler_pitch': 0.0,
            'euler_yaw': 0.0,

            'vel_linear_x': 0.0,
            'vel_linear_y': 0.0,
            'vel_linear_z': 0.0,
            'vel_angular_x': 0.0,
            'vel_angular_y': 0.0,
            'vel_angular_z': 0.0,

            'acc_linear_x': 0.0,
            'acc_linear_y': 0.0,
            'acc_linear_z': 0.0,
            'acc_angular_x': 0.0,
            'acc_angular_y': 0.0,
            'acc_angular_z': 0.0,

            'lateral_steering_angle': 0.0,
            'lateral_steering_angle_velocity': 0.0,
            'lateral_rear_wheel_angle': 0.0,
            'lateral_rear_wheel_angle_velocity': 0.0,

            'wheel_speed_left_front': 0.0,
            'wheel_speed_left_rear': 0.0,
            'wheel_speed_right_front': 0.0,
            'wheel_speed_right_rear': 0.0,

            'gear': 0,
            'control_mode': 0,
            'hand_brake': 0,
            'emergency': 0,
            'clutch': 0,
            'steering_mode': 0,
            'throttle_fb': 0.0,
            'brake_fb': 0.0,

            'tracking_lateral_tracking_error': 0.0,
            'tracking_heading_angle_error': 0.0,
            'tracking_velocity_error': 0.0,

            'trailer_pose_position_x': 0.0,
            'trailer_pose_position_y': 0.0,
            'trailer_pose_position_z': 0.0,
            'trailer_pose_orientation_x': 0.0,
            'trailer_pose_orientation_y': 0.0,
            'trailer_pose_orientation_z': 0.0,
            'trailer_pose_orientation_w': 0.0,

            'trailer_euler_roll': 0.0,
            'trailer_euler_pitch': 0.0,
            'trailer_euler_yaw': 0.0,

            'trailer_vel_linear_x': 0.0,
            'trailer_vel_linear_y': 0.0,
            'trailer_vel_linear_z': 0.0,
            'trailer_vel_angular_x': 0.0,
            'trailer_vel_angular_y': 0.0,
            'trailer_vel_angular_z': 0.0,

            'trailer_acc_linear_x': 0.0,
            'trailer_acc_linear_y': 0.0,
            'trailer_acc_linear_z': 0.0,
            'trailer_acc_angular_x': 0.0,
            'trailer_acc_angular_y': 0.0,
            'trailer_acc_angular_z': 0.0,

            'ltr_state_ltr': 0.0,
            'ltr_state_ltr_rate': 0.0,
        }

    @staticmethod
    def _build_zero_control_dict():
        """构造 Control 对应的全 0 初值字典。"""
        return {
            'control_header_seq': 0,
            'control_header_stamp': 0.0,
            'control_header_frame_id': '',

            'control_longitudinal_velocity': 0.0,
            'control_longitudinal_acceleration': 0.0,
            'control_longitudinal_jerk': 0.0,

            'control_lateral_steering_angle': 0.0,
            'control_lateral_steering_angle_velocity': 0.0,
            'control_lateral_rear_wheel_angle': 0.0,
            'control_lateral_rear_wheel_angle_velocity': 0.0,

            'control_throttle': 0.0,
            'control_brake': 0.0,
            'control_gear': 0,
            'control_control_mode': 0,
            'control_emergency': 0,
            'control_hand_brake': 0,
            'control_clutch': 0,
            'control_steering_mode': 0,
        }

    # =========================================================
    # 消息展开
    # =========================================================
    def _state_msg_to_dict(self, msg):
        """将 VehicleStatus 消息展开成字典。"""
        return {
            'header_seq': msg.header.seq,
            'header_stamp': self._safe_to_sec(msg.header.stamp),
            'header_frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,

            'pose_position_x': msg.pose.position.x,
            'pose_position_y': msg.pose.position.y,
            'pose_position_z': msg.pose.position.z,
            'pose_orientation_x': msg.pose.orientation.x,
            'pose_orientation_y': msg.pose.orientation.y,
            'pose_orientation_z': msg.pose.orientation.z,
            'pose_orientation_w': msg.pose.orientation.w,

            'euler_roll': msg.euler.roll,
            'euler_pitch': msg.euler.pitch,
            'euler_yaw': msg.euler.yaw,

            'vel_linear_x': msg.vel.linear.x,
            'vel_linear_y': msg.vel.linear.y,
            'vel_linear_z': msg.vel.linear.z,
            'vel_angular_x': msg.vel.angular.x,
            'vel_angular_y': msg.vel.angular.y,
            'vel_angular_z': msg.vel.angular.z,

            'acc_linear_x': msg.acc.linear.x,
            'acc_linear_y': msg.acc.linear.y,
            'acc_linear_z': msg.acc.linear.z,
            'acc_angular_x': msg.acc.angular.x,
            'acc_angular_y': msg.acc.angular.y,
            'acc_angular_z': msg.acc.angular.z,

            'lateral_steering_angle': msg.lateral.steering_angle,
            'lateral_steering_angle_velocity': msg.lateral.steering_angle_velocity,
            'lateral_rear_wheel_angle': msg.lateral.rear_wheel_angle,
            'lateral_rear_wheel_angle_velocity': msg.lateral.rear_wheel_angle_velocity,

            'wheel_speed_left_front': msg.wheel_speed.left_front,
            'wheel_speed_left_rear': msg.wheel_speed.left_rear,
            'wheel_speed_right_front': msg.wheel_speed.right_front,
            'wheel_speed_right_rear': msg.wheel_speed.right_rear,

            'gear': msg.gear,
            'control_mode': msg.control_mode,
            'hand_brake': int(msg.hand_brake),
            'emergency': int(msg.emergency),
            'clutch': int(msg.clutch),
            'steering_mode': msg.steering_mode,
            'throttle_fb': msg.throttle_fb,
            'brake_fb': msg.brake_fb,

            'tracking_lateral_tracking_error': msg.tracking.lateral_tracking_error,
            'tracking_heading_angle_error': msg.tracking.heading_angle_error,
            'tracking_velocity_error': msg.tracking.velocity_error,

            'trailer_pose_position_x': msg.trailer.pose.position.x,
            'trailer_pose_position_y': msg.trailer.pose.position.y,
            'trailer_pose_position_z': msg.trailer.pose.position.z,
            'trailer_pose_orientation_x': msg.trailer.pose.orientation.x,
            'trailer_pose_orientation_y': msg.trailer.pose.orientation.y,
            'trailer_pose_orientation_z': msg.trailer.pose.orientation.z,
            'trailer_pose_orientation_w': msg.trailer.pose.orientation.w,

            'trailer_euler_roll': msg.trailer.euler.roll,
            'trailer_euler_pitch': msg.trailer.euler.pitch,
            'trailer_euler_yaw': msg.trailer.euler.yaw,

            'trailer_vel_linear_x': msg.trailer.vel.linear.x,
            'trailer_vel_linear_y': msg.trailer.vel.linear.y,
            'trailer_vel_linear_z': msg.trailer.vel.linear.z,
            'trailer_vel_angular_x': msg.trailer.vel.angular.x,
            'trailer_vel_angular_y': msg.trailer.vel.angular.y,
            'trailer_vel_angular_z': msg.trailer.vel.angular.z,

            'trailer_acc_linear_x': msg.trailer.acc.linear.x,
            'trailer_acc_linear_y': msg.trailer.acc.linear.y,
            'trailer_acc_linear_z': msg.trailer.acc.linear.z,
            'trailer_acc_angular_x': msg.trailer.acc.angular.x,
            'trailer_acc_angular_y': msg.trailer.acc.angular.y,
            'trailer_acc_angular_z': msg.trailer.acc.angular.z,

            'ltr_state_ltr': msg.ltr_state.ltr,
            'ltr_state_ltr_rate': msg.ltr_state.ltr_rate,
        }

    def _control_msg_to_dict(self, msg):
        """将 Control 消息展开成字典。"""
        return {
            'control_header_seq': msg.header.seq,
            'control_header_stamp': self._safe_to_sec(msg.header.stamp),
            'control_header_frame_id': msg.header.frame_id,

            'control_longitudinal_velocity': msg.longitudinal.velocity,
            'control_longitudinal_acceleration': msg.longitudinal.acceleration,
            'control_longitudinal_jerk': msg.longitudinal.jerk,

            'control_lateral_steering_angle': msg.lateral.steering_angle,
            'control_lateral_steering_angle_velocity': msg.lateral.steering_angle_velocity,
            'control_lateral_rear_wheel_angle': msg.lateral.rear_wheel_angle,
            'control_lateral_rear_wheel_angle_velocity': msg.lateral.rear_wheel_angle_velocity,

            'control_throttle': msg.throttle,
            'control_brake': msg.brake,
            'control_gear': msg.gear,
            'control_control_mode': msg.control_mode,
            'control_emergency': int(msg.emergency),
            'control_hand_brake': int(msg.hand_brake),
            'control_clutch': int(msg.clutch),
            'control_steering_mode': msg.steering_mode,
        }

    # =========================================================
    # 回调
    # =========================================================
    def _state_callback(self, msg):
        """状态消息回调：只更新缓存，不直接写 CSV。"""
        with self.data_lock:
            self.latest_state_row = self._state_msg_to_dict(msg)
            self.latest_state_msg_ros_time = self._safe_to_sec(msg.header.stamp)
            self.latest_state_wall_time = time.time()
            self.state_received = True

    def _control_callback(self, msg):
        """控制消息回调：只更新缓存，不直接写 CSV。"""
        with self.data_lock:
            self.latest_control_row = self._control_msg_to_dict(msg)
            self.latest_control_msg_ros_time = self._safe_to_sec(msg.header.stamp)
            self.latest_control_wall_time = time.time()
            self.control_received = True

    def _timer_callback(self, event):
        """定时器回调：按固定间隔采样当前缓存并写一行 CSV。"""
        now_wall_dt = datetime.now()
        now_wall_time = time.time()
        now_ros_time = rospy.Time.now().to_sec()

        with self.data_lock:
            state_row = deepcopy(self.latest_state_row)
            control_row = deepcopy(self.latest_control_row)

            state_received = int(self.state_received)
            control_received = int(self.control_received)

            state_msg_ros_time = self.latest_state_msg_ros_time
            state_msg_wall_time = self.latest_state_wall_time
            control_msg_ros_time = self.latest_control_msg_ros_time
            control_msg_wall_time = self.latest_control_wall_time

        # 计算消息“新鲜度”
        state_msg_age_ros = (now_ros_time - state_msg_ros_time) if state_msg_ros_time > 0.0 else -1.0
        control_msg_age_ros = (now_ros_time - control_msg_ros_time) if control_msg_ros_time > 0.0 else -1.0

        state_msg_age_wall = (now_wall_time - state_msg_wall_time) if state_msg_wall_time > 0.0 else -1.0
        control_msg_age_wall = (now_wall_time - control_msg_wall_time) if control_msg_wall_time > 0.0 else -1.0

        row = {
            # ========= 记录辅助信息 =========
            'record_seq': self.record_count,
            'record_wall_time_iso': now_wall_dt.strftime('%Y-%m-%d %H:%M:%S.%f'),
            'record_wall_time_float': now_wall_time,
            'record_ros_time': now_ros_time,
            'record_dt_since_start': now_wall_time - self.start_wall_time_epoch,
            'record_period': self.record_period,

            # 缓存状态
            'state_received': state_received,
            'control_received': control_received,

            'state_msg_ros_time': state_msg_ros_time,
            'state_msg_wall_time': state_msg_wall_time,
            'state_msg_age_ros': state_msg_age_ros,
            'state_msg_age_wall': state_msg_age_wall,

            'control_msg_ros_time': control_msg_ros_time,
            'control_msg_wall_time': control_msg_wall_time,
            'control_msg_age_ros': control_msg_age_ros,
            'control_msg_age_wall': control_msg_age_wall,
        }

        row.update(state_row)
        row.update(control_row)

        self.csv_writer.writerow(row)
        self.record_count += 1

        if self.flush_every_n == 1 or (self.record_count % self.flush_every_n == 0):
            self.csv_file.flush()

    def _on_shutdown(self):
        """节点退出时关闭文件。"""
        try:
            with self.data_lock:
                # 主动 final flush
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
