#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

def imu_callback(imu_msg):
    # 复制原始IMU消息（避免修改原始数据）
    rectified_imu = Imu()
    rectified_imu = imu_msg  # 复制所有字段（包括header、orientation、angular_velocity等）

    # 对加速度的x、y分量取负号，z分量保持不变
    rectified_imu.linear_acceleration.x = -imu_msg.linear_acceleration.x
    rectified_imu.linear_acceleration.y = -imu_msg.linear_acceleration.y
    # z分量不变，无需处理：rectified_imu.linear_acceleration.z = imu_msg.linear_acceleration.z

    # 发布修正后的IMU消息
    pub.publish(rectified_imu)

def main():
    # 初始化节点，名称为"ouster_imu_rectifier"
    rospy.init_node('ouster_imu_rectifier', anonymous=True)
    
    # 定义发布者，发布到/ouster/imu_rect话题，消息类型为Imu，队列大小10
    global pub
    pub = rospy.Publisher('/ouster/imu_rect', Imu, queue_size=10)
    
    # 定义订阅者，订阅/ouster/imu话题，回调函数为imu_callback，队列大小10
    rospy.Subscriber('/ouster/imu', Imu, imu_callback, queue_size=10)
    
    rospy.loginfo("IMU加速度修正节点已启动：将/ouster/imu的x、y加速度取反，发布到/ouster/imu_rect")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass