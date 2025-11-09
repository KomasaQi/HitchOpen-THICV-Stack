#!/usr/bin/env python3
#coding=utf_8

import rospy
import serial
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time


#####################################################
#                   车辆自身参数
# ================================================
WHEEL_PERIMETER = 0.265 # 车轮周长(m)
#####################################################

def receive_serial_data():
    pub_angle = rospy.Publisher('/race/wheel_angle', Float32, queue_size=1)
    pub_circle = rospy.Publisher('/race/wheel_circle', Int32, queue_size=1)
    pub_rate = rospy.Publisher('/race/wheel_rate', Float32, queue_size=1)
    pub_speed = rospy.Publisher('/race/speedometer', Float32, queue_size=1)
    rospy.init_node('wheel_speed_sensor_node', anonymous=True)
    rate = rospy.Rate(200)  # 指定发布话题的频率，这里是100Hz

    ser = None
    
    # 串口连接函数
    def connect_serial():
        nonlocal ser
        try:
            if ser and ser.is_open:
                ser.close()
            ser = serial.Serial('/dev/anglessr', 2000000)  # 打开串口，根据实际情况修改串口号
            rospy.loginfo("Serial port opened successfully")
            return True
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port: %s" % str(e))
            return False
        except Exception as e:
            rospy.logerr("Unexpected error opening serial port: %s" % str(e))
            return False
    
    # 重连函数
    def reconnect_serial():
        rospy.logwarn("尝试重连串口...")
        while not rospy.is_shutdown():
            if connect_serial():
                return True
            rospy.logwarn("重连失败，2秒后重试...")
            time.sleep(2)
        return False
    
    # 初始连接
    if not connect_serial():
        if not reconnect_serial():
            return

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:  # 如果串口有数据可读取
                data = ser.read() # 读取一个数据
                if hex(data[0])=='0x55':
                    data = ser.read() # 再读取一个数据
                    if hex(data[0])=='0x55':
                        data = ser.read() # 再读取一个数据
                        aa = int(data[0])
                        data = ser.read() # 再读取一个数据
                        bb = int(data[0])
                        data = ser.read() # 再读取一个数据
                        cc = int(data[0])
                        data = ser.read() # 再读取一个数据
                        dd = int(data[0])
                        data = ser.read() # 再读取一个数据
                        ee = int(data[0])
                        data = ser.read() # 再读取一个数据
                        ff = int(data[0])
                        
                        ssr_angle = float((bb<<8)|aa)*360/32768
                        ssr_circle = int((ff<<8)|ee)
                        if ssr_circle >=32768 : ssr_circle -= 65536
                        result = (cc|(dd<<8))
                        if result >=32768 : result -= 65536
                        ssr_rate = float(result)*360/32768/0.02
                        speed = ssr_rate / 360 * WHEEL_PERIMETER
                        rospy.loginfo("angle:{:.2f}°  circle:{}  rate:{:.2f}°/s  speed:{:.2f}m/s".format(ssr_angle, ssr_circle, ssr_rate, speed))
                        
                        try:
                            pub_angle.publish(ssr_angle)  # 发布接收到的值到话题
                            pub_circle.publish(ssr_circle)  # 发布接收到的值到话题
                            pub_rate.publish(ssr_rate)  # 发布接收到的值到话题
                            pub_speed.publish(speed)  # 发布接收到的值到话题
                            # rospy.loginfo("%d" % value)
                            
                        except ValueError:
                            rospy.logwarn("Invalid data received: %s" % data)
                            
        except serial.SerialException as e:
            rospy.logerr("Serial communication error: %s" % str(e))
            # 串口异常，尝试重连
            if not reconnect_serial():
                break
        except OSError as e:
            rospy.logerr("Serial device error (device may be unplugged): %s" % str(e))
            # 设备可能被拔掉，尝试重连
            if not reconnect_serial():
                break
        except Exception as e:
            rospy.logerr("Unexpected error in main loop: %s" % str(e))
            time.sleep(0.1)  # 短暂延时避免错误循环过快
            
        rate.sleep()
    
    # 程序退出时关闭串口
    try:
        if ser and ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed")
    except Exception as e:
        rospy.logerr("Error closing serial port: %s" % str(e))

if __name__ == '__main__':
    try:
        receive_serial_data()
    except rospy.ROSInterruptException:
        pass
