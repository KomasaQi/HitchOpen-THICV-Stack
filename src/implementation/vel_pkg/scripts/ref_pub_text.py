#!/usr/bin/env python3
# coding=utf-8

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import numpy as np

def publish_ref_path():
    rospy.init_node('ref_path_publisher')
    path_pub = rospy.Publisher('/ref_path', Path, queue_size=1)
    
    rate = rospy.Rate(1)  # 1Hz，可以根据需要调整发布频率
    
    while not rospy.is_shutdown():
        # 创建路径消息
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"  # 确保与跟踪节点中的坐标系一致
        
        # 创建路径点 - 这里创建一个简单的直线路径
        num_points = 10
        start_x = 0.0
        start_y = 0.0
        end_x = 10.0
        end_y = 0.0
        
        for i in range(num_points):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            
            # 线性插值生成路径点
            ratio = i / (num_points - 1)
            pose.pose.position.x = start_x + (end_x - start_x) * ratio
            pose.pose.position.y = start_y + (end_y - start_y) * ratio
            pose.pose.position.z = 0.0
            
            # 设置朝向（这里简单设置为朝向下一路径点）
            if i < num_points - 1:
                next_ratio = (i + 1) / (num_points - 1)
                next_x = start_x + (end_x - start_x) * next_ratio
                next_y = start_y + (end_y - start_y) * next_ratio
                
                # 计算朝向角度
                yaw = np.arctan2(next_y - pose.pose.position.y, 
                                next_x - pose.pose.position.x)
                
                # 转换为四元数
                from tf.transformations import quaternion_from_euler
                quat = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
            else:
                # 最后一个点保持与前一个点相同的朝向
                pose.pose.orientation = path_msg.poses[-1].pose.orientation
            
            path_msg.poses.append(pose)
        
        # 发布路径
        path_pub.publish(path_msg)
        rospy.loginfo("Published reference path with %d points", num_points)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_ref_path()
    except rospy.ROSInterruptException:
        pass