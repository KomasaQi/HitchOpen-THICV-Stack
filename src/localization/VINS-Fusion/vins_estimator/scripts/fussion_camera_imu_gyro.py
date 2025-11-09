#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

class ImuMerger:
    def __init__(self):
        # 初始化节点
        rospy.init_node('imu_merger', anonymous=True)
        
        # 创建发布者，发布合并后的IMU数据
        self.merged_imu_pub = rospy.Publisher('/camera/imu_raw', Imu, queue_size=10)
        
        # 缓存最新的陀螺仪数据（gyro频率高，持续更新）
        self.latest_gyro = None
        
        # 订阅加速度计（触发发布）和陀螺仪（仅缓存）
        rospy.Subscriber('/camera/accel/sample', Imu, self.accel_callback)  # 100Hz，触发合并
        rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_callback)    # 400Hz，仅更新缓存
        
        rospy.loginfo("IMU merger node started. Merging /camera/accel/sample (100Hz) and /camera/gyro/sample (400Hz) into /camera/imu_raw (100Hz)")

    def gyro_callback(self, gyro_msg):
        """仅缓存最新的陀螺仪数据（不触发发布）"""
        self.latest_gyro = gyro_msg

    def accel_callback(self, accel_msg):
        """收到加速度计数据时，用最新的陀螺仪数据合并并发布"""
        # 若还未收到陀螺仪数据，等待（避免发布无效数据）
        if self.latest_gyro is None:
            rospy.logwarn_throttle(1, "Waiting for first gyro data...")  # 每秒最多警告1次
            return
        
        # 创建新的IMU消息
        merged_imu = Imu()
        
        # 基本信息：使用加速度计的时间戳（因以它为触发基准），frame_id保持一致
        merged_imu.header.stamp = accel_msg.header.stamp
        merged_imu.header.frame_id = accel_msg.header.frame_id  # 假设与gyro的frame_id相同
        
        # 从加速度计获取线加速度及协方差
        merged_imu.linear_acceleration.x = accel_msg.linear_acceleration.z
        merged_imu.linear_acceleration.y = -accel_msg.linear_acceleration.x
        merged_imu.linear_acceleration.z = -accel_msg.linear_acceleration.y
        # merged_imu.linear_acceleration = accel_msg.linear_acceleration
        merged_imu.linear_acceleration_covariance = accel_msg.linear_acceleration_covariance
        
        # 从最新的陀螺仪获取角速度及协方差
        merged_imu.angular_velocity.x = self.latest_gyro.angular_velocity.z
        merged_imu.angular_velocity.y = -self.latest_gyro.angular_velocity.x
        merged_imu.angular_velocity.z = -self.latest_gyro.angular_velocity.y
        # merged_imu.angular_velocity = self.latest_gyro.angular_velocity
        merged_imu.angular_velocity_covariance = self.latest_gyro.angular_velocity_covariance
        
        # 姿态数据：同前，标记为无效（Realsense的原始accel/gyro不提供姿态）
        merged_imu.orientation.x = 0.0
        merged_imu.orientation.y = 0.0
        merged_imu.orientation.z = 0.0
        merged_imu.orientation.w = 1.0
        merged_imu.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 发布合并后的消息（频率与accel一致，100Hz）
        self.merged_imu_pub.publish(merged_imu)

if __name__ == '__main__':
    try:
        imu_merger = ImuMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU merger node stopped.")
