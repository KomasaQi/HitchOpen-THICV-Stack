#!/usr/bin/env python3
# coding: utf-8

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import time

# ======================================
#   参数配置
# ======================================
MAX_LINEAR = 2.0       # 最大线速度 (m/s)
MAX_ANGULAR = 0.872    # 最大角速度 (rad/s)
ACC_LINEAR = 1.0       # 线速度加速度 (m/s^2)
ACC_ANGULAR = 3.0      # 角速度加速度 (rad/s^2)
RATE_HZ = 25           # 发布频率 (Hz)
EXIT_KEYS = {'q'}      # 退出键集合

# ======================================
#   状态变量
# ======================================
keys_held = set()
curr_linear = 0.0
curr_angular = 0.0
running = True

# ======================================
#   键盘事件回调函数
# ======================================
def on_press(key):
    global running
    try:
        c = key.char
        if c in EXIT_KEYS:
            running = False
        else:
            keys_held.add(c)
    except AttributeError:
        pass

def on_release(key):
    try:
        keys_held.discard(key.char)
    except AttributeError:
        pass

# ======================================
#   主函数
# ======================================
if __name__ == "__main__":
    rospy.init_node('key_contor', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(RATE_HZ)
    last_time = time.time()

    # 键盘监听线程
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    def publish_stop():
        t = Twist()
        pub.publish(t)

    rospy.on_shutdown(publish_stop)

    print("""
==============================
  ROS 键盘控制小车节点启动
------------------------------
  w/s : 前进 / 后退
  a/d : 左转 / 右转
  q   : 退出控制
------------------------------
  支持组合键：
    w+a : 左弧线前进
    w+d : 右弧线前进
==============================
""")

    try:
        while not rospy.is_shutdown() and running:
            now = time.time()
            dt = now - last_time
            last_time = now

            # 判断各方向键状态
            w_held = 'w' in keys_held
            s_held = 's' in keys_held
            a_held = 'a' in keys_held
            d_held = 'd' in keys_held

            # 确定方向
            if w_held and not s_held:
                linear_dir = 1
            elif s_held and not w_held:
                linear_dir = -1
            else:
                linear_dir = 0

            if a_held and not d_held:
                angular_dir = 1
            elif d_held and not a_held:
                angular_dir = -1
            else:
                angular_dir = 0

            # 更新线速度（平滑加速）
            if linear_dir == 0:
                curr_linear = 0.0
            else:
                target_linear = linear_dir * MAX_LINEAR
                diff = target_linear - curr_linear
                curr_linear += max(min(diff, ACC_LINEAR * dt), -ACC_LINEAR * dt)

            # 更新角速度（平滑加速）
            if angular_dir == 0:
                curr_angular = 0.0
            else:
                target_angular = angular_dir * MAX_ANGULAR
                diff = target_angular - curr_angular
                curr_angular += max(min(diff, ACC_ANGULAR * dt), -ACC_ANGULAR * dt)

            # 发布消息
            twist = Twist()
            twist.linear.x = curr_linear
            twist.angular.z = curr_angular
            pub.publish(twist)

            rate.sleep()

    except Exception as e:
        print("⚠️ 异常:", e)

    finally:
        publish_stop()
        listener.stop()
        print("✅ 已退出控制节点，机器人已停止。")

