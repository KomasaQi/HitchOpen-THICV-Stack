#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("veh_cmd_publisher")
    pub = rospy.Publisher("/veh_cmd", Twist, queue_size=1)
    
    rospy.loginfo("✅ veh_cmd 发布节点已启动")
    rospy.loginfo("📝 输入说明：油门(0~100)，刹车(0~100)，转向(-15~15)，按 Ctrl+C 可退出")

    while not rospy.is_shutdown():
        try:
            throttle = float(input("请输入油门 (0~100)："))
            brake = float(input("请输入刹车 (0~100)："))
            steering = float(input("请输入转向 (-15~15)："))
        except ValueError:
            rospy.logwarn("❌ 输入格式错误，请输入有效数字")
            continue
        except KeyboardInterrupt:
            print("\n👋 接收到退出指令，节点关闭")
            break

        # 限制范围
        throttle = max(0.0, min(100.0, throttle))
        brake = max(0.0, min(100.0, brake))
        steering = max(-15.0, min(15.0, steering))

        # 构建消息
        msg = Twist()
        msg.linear.x = throttle
        msg.linear.y = brake
        msg.angular.z = steering

        # 发布消息
        pub.publish(msg)
        rospy.loginfo(
            "🚗 发布 veh_cmd: 油门=%.1f 刹车=%.1f 转向=%.1f",
            msg.linear.x, msg.linear.y, msg.angular.z
        )

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("📴 veh_cmd 发布节点退出")
