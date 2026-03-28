可以，下面我直接给你一套**完整可替换**的版本，文件名保持不变：

* `vehicle_state_recorder.py`
* `vehicle_state_recorder.launch`
* `vehicle_state_recorder.yaml`
* `README.md`

这版相对你之前的版本，核心改进有四点：

1. **不再是收到 `/race/vehicle_state` 就立刻写一行**，而是：

   * `/race/vehicle_state` 和 `/race/control` 各自更新缓存
   * 由一个**固定周期定时器**按 `record_period=0.05` s 写一行

2. **支持同时记录 `/race/control`** 的全部控制量

3. **即使某个话题暂时没收到消息，也会按你的要求先用 0 初值占位继续记录**

   * 这很适合后续和视频、动画按固定时间间隔对齐

4. 增加了一些辅助字段，方便你后处理：

   * 当前这一行写入时两个缓存是否收到过真实消息
   * 两个话题最新消息距离当前记录时刻的“新鲜度”
   * 当前记录周期 `record_period`

你给的旧版脚本内容我已经作为本次修改的基础来处理了。

---

# 1）vehicle_state_recorder.py

请完整替换为下面内容：

```python
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
```

---

# 2）vehicle_state_recorder.launch

完整替换为：

```xml
<launch>
    <!-- ============================= -->
    <!-- 可在命令行覆盖的参数 -->
    <!-- ============================= -->

    <!-- 车辆状态话题 -->
    <arg name="state_topic_name" default="/race/vehicle_state" />

    <!-- 控制话题 -->
    <arg name="control_topic_name" default="/race/control" />

    <!-- 实验名称 -->
    <arg name="experiment_name" default="default_experiment" />

    <!-- 输出目录 -->
    <arg name="output_dir" default="~/komasa/HitchOpen-THICV-Stack/src/implementation/exp_utils/data/vehicle_state_logs" />

    <!-- 每隔多少行 flush 一次 -->
    <arg name="flush_every_n" default="5" />

    <!-- 文件名是否使用系统时间 -->
    <arg name="use_wall_time_in_filename" default="true" />

    <!-- 固定记录周期，单位：秒。0.05 s = 20 Hz -->
    <arg name="record_period" default="0.05" />

    <!-- 订阅队列长度 -->
    <arg name="subscriber_queue_size" default="100" />

    <!-- 是否启动时写入表头 -->
    <arg name="write_header_on_start" default="true" />

    <!-- ============================= -->
    <!-- 先加载 yaml 默认参数 -->
    <!-- ============================= -->
    <rosparam command="load" file="$(find exp_utils)/config/vehicle_state_recorder.yaml" />

    <!-- ============================= -->
    <!-- launch 参数覆盖 yaml 同名参数 -->
    <!-- ============================= -->
    <node pkg="exp_utils"
          type="vehicle_state_recorder.py"
          name="vehicle_state_recorder"
          output="screen">

        <param name="state_topic_name" value="$(arg state_topic_name)" />
        <param name="control_topic_name" value="$(arg control_topic_name)" />
        <param name="experiment_name" value="$(arg experiment_name)" />
        <param name="output_dir" value="$(arg output_dir)" />
        <param name="flush_every_n" value="$(arg flush_every_n)" />
        <param name="use_wall_time_in_filename" value="$(arg use_wall_time_in_filename)" />
        <param name="record_period" value="$(arg record_period)" />
        <param name="subscriber_queue_size" value="$(arg subscriber_queue_size)" />
        <param name="write_header_on_start" value="$(arg write_header_on_start)" />
    </node>
</launch>
```

---

# 3）vehicle_state_recorder.yaml

完整替换为：

```yaml
# vehicle_state_recorder 默认参数配置

# =============================
# 订阅话题
# =============================

# 车辆状态话题
state_topic_name: "/race/vehicle_state"

# 控制话题
control_topic_name: "/race/control"

# =============================
# 文件输出
# =============================

# 实验名称，会自动拼接开始记录时间形成最终文件名
# 例如：double_lane_change_20260324_153000.csv
experiment_name: "default_experiment"

# CSV 输出目录
output_dir: "~/komasa/HitchOpen-THICV-Stack/src/implementation/exp_utils/data/vehicle_state_logs"

# 文件名中是否使用系统时间（推荐 true）
# false 时将改用 ROS 时间戳秒数
use_wall_time_in_filename: true

# 是否启动后立即写入表头
write_header_on_start: true

# =============================
# 记录策略
# =============================

# 固定记录周期（秒）
# 0.05 s = 20 Hz
record_period: 0.05

# 每记录多少行主动 flush 一次到磁盘
# 1 表示每行都 flush，最安全但稍慢
flush_every_n: 5

# ROS 订阅队列长度
subscriber_queue_size: 100
```

---

# 4）README.md

下面给你更新后的完整 `README.md`。
这版已经把“固定周期记录 + 缓存 + 同时记录 `/race/control`”写进去了。

````markdown
# exp_utils

`exp_utils` 是一个用于实验辅助的小工具包，当前已提供 `vehicle_state_recorder.py`，用于以**固定时间间隔**将 `/race/vehicle_state` 与 `/race/control` 两个话题中的数据同步展开后记录到 CSV 文件中，便于后续离线分析、绘图、回放、视频对齐与实验归档。后续你还可以继续在这个包里扩展其他实验工具。

---

## 1. 当前功能

目前本包已包含以下功能：

### 1.1 vehicle_state_recorder

用于同时订阅：

- `race_msgs/VehicleStatus` 类型的 `/race/vehicle_state`
- `race_msgs/Control` 类型的 `/race/control`

并将两个话题中的全部字段展开后，以**固定记录周期**写入同一个 CSV 文件。

它具备以下特点：

- 支持输入实验名称；
- 自动在 CSV 文件名后附加**开始记录时间**；
- 自动避免覆盖历史实验文件；
- 自动创建输出目录；
- 将 `VehicleStatus` 与 `Control` 中的嵌套字段全部展开为独立列；
- 采用“**缓存 + 定时采样写入**”机制，而不是“消息一到就立即写一行”；
- 即使某一时刻某个话题没有新消息，也会使用当前缓存值继续按固定周期写入；
- 若节点启动后某个话题尚未收到过消息，则该话题相关字段先以 0 初值记录；
- 提供默认 `yaml` 配置文件与 `launch` 文件，调用方便。

这种设计特别适合：

- 与实验视频做时间同步；
- 后续制作固定采样率的数据动图；
- 多个话题频率不同但希望统一导出为一张表；
- 真实实验中存在消息频率波动时仍保持稳定记录节奏。

---

## 2. 目录结构建议

建议你的 `exp_utils` 包目录至少包含如下内容：

```bash
exp_utils/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── vehicle_state_recorder.yaml
├── launch/
│   └── vehicle_state_recorder.launch
└── scripts/
    └── vehicle_state_recorder.py
````

---

## 3. 使用前准备

### 3.1 依赖

本节点依赖：

* `rospy`
* `race_msgs`
* ROS1 环境

其中 `race_msgs` 至少需要包含：

* `VehicleStatus.msg`
* `Control.msg`

你已经说明当前已经创建了引用 `race_msgs` 的 `exp_utils` 包，因此通常只需保证工作空间已经正确编译并 source。

例如：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果你使用的是 `catkin build`，则改为：

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

---

## 4. 功能说明

### 4.1 输出文件命名规则

输出的 CSV 文件名格式如下：

```bash
实验名称_开始记录时间.csv
```

例如：

```bash
double_lane_change_20260324_153015.csv
```

若同名文件已存在，程序会自动继续追加序号，例如：

```bash
double_lane_change_20260324_153015_001.csv
```

这样可以避免覆盖其他实验记录，也能清楚区分每次实验的开始时间。

---

### 4.2 当前记录机制

当前节点不是“某个消息到来就写一行”，而是如下流程：

1. `/race/vehicle_state` 到来时，仅更新**车辆状态缓存**；
2. `/race/control` 到来时，仅更新**控制命令缓存**；
3. 由一个固定周期定时器（默认 `0.05 s`）统一写一行 CSV；
4. 每一行都会取“当前时刻的最新状态缓存 + 最新控制缓存”。

因此：

* 即使两个话题频率不同，也能最终导出到同一个统一时间轴；
* 即使某一时刻某个话题没更新，也不会中断记录；
* 很适合做视频同步和数据动画。

---

### 4.3 默认记录内容

当前节点会完整记录以下内容：

#### A. `race_msgs/VehicleStatus` 中的全部状态量

包括但不限于：

* `header`
* `child_frame_id`
* `pose`
* `euler`
* `vel`
* `acc`
* `lateral`
* `wheel_speed`
* `gear`
* `control_mode`
* `hand_brake`
* `emergency`
* `clutch`
* `steering_mode`
* `tracking`
* `throttle_fb`
* `brake_fb`
* `trailer`
* `ltr_state`

#### B. `race_msgs/Control` 中的全部控制量

包括：

* `header`
* `longitudinal`
* `lateral`
* `throttle`
* `brake`
* `gear`
* `control_mode`
* `emergency`
* `hand_brake`
* `clutch`
* `steering_mode`

#### C. 额外辅助字段

同时还会额外记录：

* 写入序号 `record_seq`
* 当前系统时间字符串 `record_wall_time_iso`
* 当前系统时间秒 `record_wall_time_float`
* 当前记录时刻 ROS 时间 `record_ros_time`
* 相对记录开始的累计时间 `record_dt_since_start`
* 当前记录周期 `record_period`

以及两个缓存状态：

* `state_received`
* `control_received`

以及两个缓存消息的时间与新鲜度：

* `state_msg_ros_time`
* `state_msg_wall_time`
* `state_msg_age_ros`
* `state_msg_age_wall`
* `control_msg_ros_time`
* `control_msg_wall_time`
* `control_msg_age_ros`
* `control_msg_age_wall`

这些字段对于后续判断：

* 某一段时间控制是否掉线；
* 某一段时间状态更新是否过旧；
* 视频同步时状态是否“新鲜”；

都很有帮助。

---

## 5. 参数说明

默认参数文件位于：

```bash
config/vehicle_state_recorder.yaml
```

主要参数如下：

| 参数名                         | 含义               | 默认值                                                                                   |
| --------------------------- | ---------------- | ------------------------------------------------------------------------------------- |
| `state_topic_name`          | 车辆状态话题名称         | `/race/vehicle_state`                                                                 |
| `control_topic_name`        | 控制话题名称           | `/race/control`                                                                       |
| `experiment_name`           | 实验名称             | `default_experiment`                                                                  |
| `output_dir`                | CSV 输出目录         | `~/komasa/HitchOpen-THICV-Stack/src/implementation/exp_utils/data/vehicle_state_logs` |
| `use_wall_time_in_filename` | 文件名是否使用系统时间      | `true`                                                                                |
| `write_header_on_start`     | 启动时是否写入表头        | `true`                                                                                |
| `record_period`             | 固定记录周期（秒）        | `0.05`                                                                                |
| `flush_every_n`             | 每记录多少行后 flush 一次 | `5`                                                                                   |
| `subscriber_queue_size`     | 订阅队列长度           | `100`                                                                                 |

说明：

* `record_period=0.05` 表示以 20 Hz 固定频率记录；
* 如果你以后想改为 50 Hz，则设置 `record_period=0.02`；
* 若担心实验过程中掉电或异常退出，建议把 `flush_every_n` 调小，例如 `1` 或 `2`；
* 若更关注性能，可适当调大 `flush_every_n`。

---

## 6. 启动方式

### 6.1 使用 launch 文件启动

最推荐的启动方式：

```bash
roslaunch exp_utils vehicle_state_recorder.launch
```

### 6.2 指定实验名称

例如：

```bash
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=slalom_test
```

### 6.3 同时修改记录周期

例如改为 100 Hz：

```bash
roslaunch exp_utils vehicle_state_recorder.launch \
    experiment_name:=high_rate_test \
    record_period:=0.01
```

### 6.4 同时修改输出目录

```bash
roslaunch exp_utils vehicle_state_recorder.launch \
    experiment_name:=brake_test \
    output_dir:=/home/user/exp_logs
```

### 6.5 修改话题名称

如果你后续状态或控制话题名称变化，也可以直接覆盖：

```bash
roslaunch exp_utils vehicle_state_recorder.launch \
    state_topic_name:=/my_vehicle/state \
    control_topic_name:=/my_vehicle/control
```

---

## 7. 直接运行脚本方式

除了 launch 方式，也可以直接运行节点：

```bash
rosrun exp_utils vehicle_state_recorder.py _experiment_name:=test1
```

例如：

```bash
rosrun exp_utils vehicle_state_recorder.py \
    _state_topic_name:=/race/vehicle_state \
    _control_topic_name:=/race/control \
    _experiment_name:=fishhook_test \
    _record_period:=0.05 \
    _output_dir:=/home/user/exp_logs
```

注意在 `rosrun` 下，私有参数要写成 `_参数名:=参数值` 的形式。

---

## 8. CSV 字段设计说明

由于 `VehicleStatus` 与 `Control` 中存在大量嵌套消息，节点内部会自动将其展开为扁平化列名。例如：

### 8.1 VehicleStatus 示例

* `pose.position.x` → `pose_position_x`
* `vel.angular.z` → `vel_angular_z`
* `trailer.euler.yaw` → `trailer_euler_yaw`
* `ltr_state.ltr_rate` → `ltr_state_ltr_rate`

### 8.2 Control 示例

* `longitudinal.velocity` → `control_longitudinal_velocity`
* `lateral.steering_angle` → `control_lateral_steering_angle`
* `throttle` → `control_throttle`
* `control_mode` → `control_control_mode`

这样设计的好处是：

1. 不需要后处理时再解析嵌套结构；
2. MATLAB / pandas / Excel 读取更直接；
3. 更适合后续画图、滤波、对比实验与自动分析脚本；
4. 状态与控制在同一行中，便于分析控制输入与车辆响应的对应关系。

---

## 9. 典型使用流程

下面给出一个常见实验流程示例：

### 第一步：启动 ROS 环境

```bash
source ~/catkin_ws/devel/setup.bash
```

### 第二步：启动你的车辆系统

确保以下两个话题正常发布：

* `/race/vehicle_state`
* `/race/control`

可以先检查：

```bash
rostopic echo /race/vehicle_state
rostopic echo /race/control
```

### 第三步：启动记录器

```bash
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=dlc_01
```

### 第四步：完成实验后结束节点

按 `Ctrl+C` 结束后，CSV 会自动关闭并保存。

---

## 10. 建议

### 10.1 实验命名建议

建议实验名称尽量简洁且有辨识度，例如：

* `dlc_01`
* `fishhook_speed15`
* `brake_test_lowmu`
* `ramp_entry_case3`

这样后续做批量分析时会更方便。

### 10.2 关于记录周期

如果你后续要和视频做同步、做动图，固定记录周期非常有帮助。

例如：

* `0.05 s` → 20 Hz，常用于常规实验记录；
* `0.02 s` → 50 Hz，更适合更快的动态；
* `0.01 s` → 100 Hz，数据更细，但文件更大。

一般模型车实验中，20 Hz 或 50 Hz 往往已经够用。

### 10.3 关于缓存机制

当前记录器采用“缓存 + 定时采样”的方式，因此某一行数据并不意味着那一时刻两个话题都刚好同时发布了新消息，而是表示：

> 在该记录时刻，系统把当时最新的状态缓存和最新的控制缓存一起写入了表格。

这是为了解决：

* 多话题频率不同；
* 真实系统通信存在抖动；
* 后续需要统一时间轴；

这些问题。

如果你后续想做更严格的同步，也可以后续再扩展成“近似时间同步器”版本。

---

## 11. 可能的后续扩展方向

既然 `exp_utils` 以后还会继续扩展，下面是一些很适合继续加入的工具方向：

* 多话题时间同步记录器；
* 自动实验编号管理工具；
* 实验结束后自动生成统计摘要；
* CSV 转 MAT / NPZ 工具；
* 记录与视频时间戳对齐工具；
* 一键绘图分析脚本；
* 多圈实验自动分圈与轨迹平滑工具；
* 自动生成实验报告摘要。

这样这个包后面会逐渐变成一个比较完整的实验辅助工具箱。

---

## 12. 注意事项

1. 请确保 `vehicle_state_recorder.py` 具有可执行权限：

```bash
chmod +x scripts/vehicle_state_recorder.py
```

2. 请确保 `package.xml` 和 `CMakeLists.txt` 中已经声明了对 `rospy` 与 `race_msgs` 的依赖。

3. 若使用 `roslaunch` 时提示找不到脚本，一般是以下几种原因：

* 没有给 `scripts/vehicle_state_recorder.py` 可执行权限；
* 工作空间没有重新编译；
* 没有重新 `source devel/setup.bash`。

---

## 13. 最小自检方法

你可以用下面的方法快速检查是否工作正常：

```bash
rostopic list | grep race
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=debug_test
```

然后检查输出目录下是否生成类似文件：

```bash
~/komasa/HitchOpen-THICV-Stack/src/implementation/exp_utils/data/vehicle_state_logs/debug_test_20260324_153015.csv
```

打开前几行确认：

* 表头是否包含 `pose_position_x`、`euler_yaw`、`ltr_state_ltr`
* 是否包含 `control_throttle`、`control_lateral_steering_angle`
* `record_period` 是否为期望值
* `state_received` 与 `control_received` 是否正常变为 `1`

即可。

---

## 14. 当前版本说明

当前版本提供：

* `vehicle_state_recorder.py`
* `vehicle_state_recorder.yaml`
* `vehicle_state_recorder.launch`
* 本中文 README

当前版本的 `vehicle_state_recorder.py` 已支持：

* 同时记录状态与控制；
* 固定周期写入；
* 启动初期 0 值占位；
* 输出统一时间轴 CSV。

后续你可以继续在 `exp_utils` 中逐步加入新的实验工具，而无需重新组织包结构。

````

---

# 5）几点我顺手帮你检查出来的技术问题

你原来的想法是对的，而且这次这样改是合理的，但有几个实现细节要特别注意：

## 第一，为什么不能继续沿用“收到状态就写一行”
因为现在你还要加 `/race/control`，两个话题频率不一定一样。  
如果继续采用“某个话题一来就写一行”，会出现：

- 有些行只有状态新，控制是旧的
- 有些行只有控制新，状态是旧的
- 时间轴不均匀，和视频不好同步

所以固定周期写入是更合适的。

---

## 第二，为什么要保留 `state_msg_age_*` 和 `control_msg_age_*`
这几个字段以后非常有用。  
你后处理时可以直接检查某一段是不是：

- 控制命令长时间没更新
- 状态估计掉线
- 消息延迟过大

这比只看数值本身强很多。

---

## 第三，0 初值策略的副作用
你要求“开始先给 0 初值”，这没问题。  
但要知道启动最开始几行可能会是：

- 状态全 0
- 控制全 0
- 或其中一个真实、另一个仍是 0

所以后处理时建议用：

- `state_received`
- `control_received`

来过滤启动阶段的无效记录。

例如 MATLAB 里：

```matlab
valid = (T.state_received == 1) & (T.control_received == 1);
T2 = T(valid, :);
````

---

## 第四，`topic_name` 这个旧参数我改成了两个

旧版只有：

* `topic_name`

现在需要两个话题，所以改成：

* `state_topic_name`
* `control_topic_name`

这属于必要修改，否则语义不清楚。

---

# 6）你现在怎么启动

最常用：

```bash
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=dlc_test
```

改记录周期，比如 0.02 s：

```bash
roslaunch exp_utils vehicle_state_recorder.launch \
    experiment_name:=dlc_test \
    record_period:=0.02
```

---

# 7）我再给你一个很实用的建议

你现在这个记录器已经很适合后处理了。
下一步如果你准备做“多圈平均 + 与视频同步”，我建议你后处理时优先使用这几列作为基础：

* `record_ros_time`
* `pose_position_x`
* `pose_position_y`
* `euler_yaw`
* `vel_linear_x`
* `vel_linear_y`
* `vel_angular_z`
* `ltr_state_ltr`
* `control_throttle`
* `control_brake`
* `control_lateral_steering_angle`

这几列最关键。

如果你愿意，我下一条可以继续直接给你配套写一个 **MATLAB 读这个新 CSV 表头的预处理脚本**，把：

* 分圈
* 固定时间重采样
* 多圈按参考轨迹对齐
* 平滑后导出动画用表

一起接上。
