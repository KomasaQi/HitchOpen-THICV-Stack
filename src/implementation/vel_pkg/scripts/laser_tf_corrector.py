#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

class LaserTFCorrector:
    def __init__(self):
        rospy.init_node('laser_tf_corrector')
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 发布校正后的激光数据
        self.corrected_scan_pub = rospy.Publisher('/scan_corrected', LaserScan, queue_size=10)
        
        # 订阅原始激光数据
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 定时发布校正后的TF变换
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tf_timer_callback)
        
        rospy.loginfo("激光TF校正节点已启动")
    
    def scan_callback(self, msg):
        # 创建校正后的激光数据
        corrected_scan = LaserScan()
        corrected_scan.header = msg.header
        corrected_scan.header.frame_id = 'laser_corrected'  # 使用校正后的坐标系
        
        # 反转激光数据方向（如果需要）
        # 这里可以根据需要调整激光数据的处理
        
        corrected_scan.angle_min = msg.angle_min
        corrected_scan.angle_max = msg.angle_max
        corrected_scan.angle_increment = msg.angle_increment
        corrected_scan.time_increment = msg.time_increment
        corrected_scan.scan_time = msg.scan_time
        corrected_scan.range_min = msg.range_min
        corrected_scan.range_max = msg.range_max
        
        # 如果需要反转激光数据，可以在这里处理
        corrected_scan.ranges = msg.ranges
        corrected_scan.intensities = msg.intensities
        
        # 发布校正后的激光数据
        self.corrected_scan_pub.publish(corrected_scan)
    
    def tf_timer_callback(self, event):
        # 发布校正后的TF变换（绕Y轴旋转180度）
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'base_link'  # 父坐标系
        transform.child_frame_id = 'laser_corrected'  # 子坐标系（校正后的激光坐标系）
        
        # 设置变换（绕Y轴旋转180度）
        transform.transform.translation.x = 0.15  # 激光雷达在X方向上的偏移
        transform.transform.translation.y = 0.0   # 激光雷达在Y方向上的偏移
        transform.transform.translation.z = 0.20  # 激光雷达在Z方向上的偏移
        
        # 绕Y轴旋转180度（pitch=π）
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 1.0  # 绕Y轴旋转180度
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.0
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        corrector = LaserTFCorrector()
        corrector.run()
    except rospy.ROSInterruptException:
        pass