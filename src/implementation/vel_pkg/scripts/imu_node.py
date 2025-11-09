#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import math
import tf
import can
import threading
from queue import Queue
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, Vector3

# ================== ROS参数配置 ==================
IMU_ACCEL_ID = rospy.get_param('~imu_accel_id', 0x289)
IMU_GYRO_ID = rospy.get_param('~imu_gyro_id', 0x290)
IMU_INFO_ID = rospy.get_param('~imu_info_id', 0x288)
UKF_EULER_ID = rospy.get_param('~ukf_euler_id', 0x306)
FRAME_ID = rospy.get_param('~frame_id', 'imu_link')

# ================== CAN配置 ==================
CAN_CONFIG = {
    IMU_ACCEL_ID: {
        'fields': [
            ('x', 0, 2, True, 0.01),
            ('y', 2, 2, True, 0.01),
            ('z', 4, 2, True, 0.01)
        ]
    },
    IMU_GYRO_ID: {
        'fields': [
            ('x', 0, 2, True, 0.001),
            ('y', 2, 2, True, 0.001),
            ('z', 4, 2, True, 0.001)
        ]
    },
    IMU_INFO_ID: {
        'fields': [
            ('timestamp', 0, 4, False, 0.01),
            ('temperature', 6, 2, True, 0.01)
        ]
    },
    UKF_EULER_ID: {
        'fields': [
            ('roll', 0, 2, True, 0.0001),
            ('pitch', 2, 2, True, 0.0001),
            ('yaw', 4, 2, True, 0.0001)
        ]
    }
}

# ================== IMU数据处理类 ==================
class IMUDataHandler:
    def __init__(self):
        self.imu_data = {
            'accel': Vector3(0, 0, 0),
            'gyro': Vector3(0, 0, 0),
            'orientation': Quaternion(0, 0, 0, 1),
            'temperature': 0.0,
            'last_update': rospy.Time.now()
        }

        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=5)
        self.temp_pub = rospy.Publisher('imu/temperature', Float32, queue_size=5)

        # 用 float64 数组，避免 rostopic echo 卡死
        self.orientation_covariance = [0.01, 0, 0,
                                       0, 0.01, 0,
                                       0, 0, 0.01]
        self.angular_velocity_covariance = [0.01, 0, 0,
                                            0, 0.01, 0,
                                            0, 0, 0.01]
        self.linear_acceleration_covariance = [0.01, 0, 0,
                                               0, 0.01, 0,
                                               0, 0, 0.01]

    def extract_value(self, b, offset, length, is_signed, factor):
        slice_ = b[offset:offset+length]
        fmt = {1: 'b', 2: 'h', 4: 'i'}[length] if is_signed else {1: 'B', 2: 'H', 4: 'I'}[length]
        val = struct.unpack('<' + fmt, slice_)[0]
        return val * factor

    def update_data(self, can_id, data_bytes):
        cfg = CAN_CONFIG.get(can_id)
        if not cfg:
            return False

        if can_id == IMU_ACCEL_ID:
            for field, offset, length, signed, factor in cfg['fields']:
                val = self.extract_value(data_bytes, offset, length, signed, factor)
                setattr(self.imu_data['accel'], field, val)

        elif can_id == IMU_GYRO_ID:
            for field, offset, length, signed, factor in cfg['fields']:
                val = self.extract_value(data_bytes, offset, length, signed, factor)
                setattr(self.imu_data['gyro'], field, val)

        elif can_id == IMU_INFO_ID:
            for field, offset, length, signed, factor in cfg['fields']:
                if field == 'temperature':
                    self.imu_data['temperature'] = self.extract_value(data_bytes, offset, length, signed, factor)

        elif can_id == UKF_EULER_ID:
            roll = pitch = yaw = 0
            for field, offset, length, signed, factor in cfg['fields']:
                val = self.extract_value(data_bytes, offset, length, signed, factor)
                if field == 'roll': roll = val
                elif field == 'pitch': pitch = val
                elif field == 'yaw': yaw = val
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            self.imu_data['orientation'] = Quaternion(*q)

        self.imu_data['last_update'] = rospy.Time.now()
        return True

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.imu_data['last_update']
        imu_msg.header.frame_id = FRAME_ID
        imu_msg.orientation = self.imu_data['orientation']
        imu_msg.orientation_covariance = self.orientation_covariance
        imu_msg.angular_velocity = self.imu_data['gyro']
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        imu_msg.linear_acceleration = self.imu_data['accel']
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.imu_pub.publish(imu_msg)
        self.temp_pub.publish(Float32(self.imu_data['temperature']))

# ================== CAN接收线程 ==================
def can_listener(bus, queue):
    while not rospy.is_shutdown():
        try:
            msg = bus.recv(timeout=0.05)
            if msg:
                queue.put(msg)
        except can.CanError:
            continue

# ================== 主程序 ==================
def main():
    rospy.init_node('imu_node')
    rospy.loginfo("IMU数据发布器启动")

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except Exception as e:
        rospy.logerr(f"CAN总线初始化失败: {e}")
        return

    imu_handler = IMUDataHandler()
    msg_queue = Queue()
    threading.Thread(target=can_listener, args=(bus, msg_queue), daemon=True).start()

    rate = rospy.Rate(50)  # 50Hz
    rospy.loginfo("开始主循环...")

    try:
        while not rospy.is_shutdown():
            while not msg_queue.empty():
                msg = msg_queue.get()
                if msg.arbitration_id in [IMU_ACCEL_ID, IMU_GYRO_ID, IMU_INFO_ID, UKF_EULER_ID]:
                    imu_handler.update_data(msg.arbitration_id, msg.data)

            imu_handler.publish_imu_data()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        bus.shutdown()
        rospy.loginfo("IMU数据发布器关闭")

if __name__ == "__main__":
    main()
