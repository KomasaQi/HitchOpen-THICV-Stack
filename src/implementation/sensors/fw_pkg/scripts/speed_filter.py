#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import time

class SpeedFilter:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('speed_filter_node', anonymous=True)
        
        # 获取配置参数：时间窗口大小（秒），默认5秒
        self.window_seconds = rospy.get_param('~window_seconds', 5.0)
        
        # 初始化数据缓存：存储(时间戳, 速度值)的列表
        self.speed_data = []
        
        # 订阅速度话题
        self.sub = rospy.Subscriber(
            '/race/speedometer',  # 订阅的话题名
            Float32,              # 消息类型
            self.speed_callback,  # 回调函数
            queue_size=100        # 队列大小，防止数据丢失
        )
        
        rospy.loginfo("Speed Filter Node started")
        rospy.loginfo("Window size: {} seconds".format(self.window_seconds))

    def speed_callback(self, msg):
        """速度话题回调函数"""
        current_time = time.time()  # 获取当前时间戳
        speed_value = msg.data      # 获取速度值
        
        # 将新数据添加到缓存
        self.speed_data.append((current_time, speed_value))
        
        # 清理超出时间窗口的旧数据
        self._clean_old_data(current_time)
        
        # 计算并打印平均值
        self._calculate_and_print_average()

    def _clean_old_data(self, current_time):
        """清理超出时间窗口的旧数据"""
        # 计算时间窗口的起始时间
        window_start = current_time - self.window_seconds
        
        # 保留窗口内的数据
        self.speed_data = [data for data in self.speed_data if data[0] >= window_start]

    def _calculate_and_print_average(self):
        """计算并打印平均速度"""
        if not self.speed_data:
            # 无数据时的提示
            rospy.loginfo("No speed data received yet")
            return
        
        # 提取所有速度值并计算平均值
        speed_values = [data[1] for data in self.speed_data]
        average_speed = sum(speed_values) / len(speed_values)
        
        # 打印结果（保留2位小数）使用Throttle控制输出频率，避免过于频繁的打印
        rospy.loginfo_throttle(1.0, f"Average speed in last {self.window_seconds}s: {average_speed:.2f} m/s (total samples: {len(speed_values)})")
    def run(self):
        """节点主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        # 创建过滤器实例并运行
        speed_filter = SpeedFilter()
        speed_filter.run()
    except rospy.ROSInterruptException:
        # 捕获ROS中断异常，优雅退出
        rospy.loginfo("Speed Filter Node stopped")
    finally:
        # 换行，保证终端输出格式整洁
        print()