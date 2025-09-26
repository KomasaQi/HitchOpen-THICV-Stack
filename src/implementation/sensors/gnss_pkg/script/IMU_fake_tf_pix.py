#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

class ImuTfBroadcaster:
    def __init__(self):
        # 初始化节点
        rospy.init_node('imu_tf_broadcaster', anonymous=True)
        
        # 订阅IMU话题（需根据实际话题名修改）
        self.imu_topic = rospy.get_param('~imu_topic', '/imu_raw')  # 默认话题
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 坐标系命名（可根据实际需求修改）
        self.parent_frame = rospy.get_param('~parent_frame', 'ego_vehicle')  # 父坐标系
        self.child_frame = rospy.get_param('~child_frame', 'imu_link')     # IMU对应的子坐标系
        
        rospy.loginfo("IMU TF广播器已启动，等待IMU数据...")

    def imu_callback(self, imu_msg):
        """处理IMU消息，提取四元数并发布TF变换"""
        # 创建TF变换消息
        transform = TransformStamped()
        
        # 设置时间戳（使用IMU消息的时间戳）
        transform.header.stamp = imu_msg.header.stamp
        # 设置父坐标系
        transform.header.frame_id = self.parent_frame
        # 设置子坐标系（IMU对应的坐标系）
        transform.child_frame_id = self.child_frame
        
        # 平移部分（假设IMU在父坐标系原点，可根据实际安装位置修改）
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 1.0
        
        # 旋转部分（直接使用IMU消息中的四元数）
        transform.transform.rotation = imu_msg.orientation
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        ImuTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
