#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import can  # 替换MCP2515为python-can
import time
import math
import threading

# ===================== 全局配置 =====================
WHEEL_SPEED_ID = 0x050  # CAN ID
WHEEL_RADIUS = 0.0425    # 轮子半径 (米)
DEFAULT_SAMPLING_TIME = 0.01  # 默认角速度采样时间 (秒)
DEBUG_MODE = False       # 调试输出开关
# ===================================================

wheel_speed_latest = None
lock = threading.Lock()


class WheelSpeedDecoder:
    def __init__(self, can_id):
        self.can_id = can_id
        self.angle = 0.0
        self.angular_velocity = 0.0  # 角速度 (度/秒)
        self.revolution = 0
        self.temperature = 0.0
        self.sampling_time = DEFAULT_SAMPLING_TIME

    def decode(self, data_bytes):
        """根据手册解析轮速计数据"""
        try:
            # 将字节数据转换为十六进制字符串列表，与原代码兼容
            data = [format(x, '02x') for x in data_bytes]
            
            int_data = [int(x, 16) for x in data]

            if int_data[0] == 0x55 and int_data[1] == 0x55:
                # 角度
                angle_reg = (int_data[3] << 8) | int_data[2]
                self.angle = angle_reg * 360.0 / 32768.0

                # 角速度
                angular_vel_reg = (int_data[5] << 8) | int_data[4]
                if angular_vel_reg > 32767:
                    angular_vel_reg -= 65536
                self.angular_velocity = angular_vel_reg * 360.0 / 32768.0 / self.sampling_time

                # 转数
                self.revolution = (int_data[7] << 8) | int_data[6]
                return "speed"

            elif int_data[0] == 0x55 and int_data[1] == 0x56:
                # 温度
                temp_reg = (int_data[3] << 8) | int_data[2]
                self.temperature = temp_reg / 100.0
                return "temp"

        except Exception as e:
            rospy.logerr(f"解码错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

        return None


def can_reader(bus):
    """高频读取 CAN 总线，防止缓冲区溢出"""
    global wheel_speed_latest
    decoder = WheelSpeedDecoder(WHEEL_SPEED_ID)
    while not rospy.is_shutdown():
        try:
            message = bus.recv(timeout=0.001)  # 1ms超时
            if message and message.arbitration_id == WHEEL_SPEED_ID:
                data_type = decoder.decode(message.data)
                if data_type == "speed":
                    angular_velocity_rad = decoder.angular_velocity * (math.pi / 180.0)
                    linear_speed = angular_velocity_rad * WHEEL_RADIUS
                    with lock:
                        wheel_speed_latest = linear_speed
        except can.CanError as e:
            rospy.logwarn_throttle(1.0, f"CAN接收错误: {e}")
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"其他CAN错误: {e}")


def main():
    rospy.init_node('wheel_speed_publisher', anonymous=True)
    speed_pub = rospy.Publisher('/wheel_speed', Float32, queue_size=10)

    rospy.loginfo("启动轮速计ROS节点...")
    rospy.loginfo(f"监听轮速计ID: 0x{WHEEL_SPEED_ID:03X}")
    rospy.loginfo(f"轮子半径: {WHEEL_RADIUS} 米")
    rospy.loginfo(f"角速度采样时间: {DEFAULT_SAMPLING_TIME} 秒")

    # 初始化 CAN 总线
    bus = None
    while not rospy.is_shutdown() and bus is None:
        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
            rospy.loginfo("CAN总线初始化成功")
        except Exception as e:
            rospy.logerr("CAN总线初始化失败: %s, 5秒后重试...", e)
            rospy.sleep(5)

    try:
        # 启动高频接收线程
        threading.Thread(target=can_reader, args=(bus,), daemon=True).start()

        rate = rospy.Rate(50)  # 50Hz 发布
        while not rospy.is_shutdown():
            with lock:
                if wheel_speed_latest is not None:
                    speed_msg = Float32()
                    speed_msg.data = wheel_speed_latest
                    speed_pub.publish(speed_msg)
                    if DEBUG_MODE:
                        rospy.loginfo(f"当前线速度: {wheel_speed_latest:.4f} m/s")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
    finally:
        # 关闭CAN总线
        if bus:
            try:
                bus.shutdown()
            except:
                pass
        rospy.loginfo("节点已关闭")


if __name__ == "__main__":
    main()