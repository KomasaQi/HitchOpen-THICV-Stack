#!/usr/bin/env python3
import rospy
from nmea_navsat_driver.msg import NavSatGHFPD
from sensor_msgs.msg import NavSatFix, NavSatStatus

class FusedFixToGnss:
    def __init__(self):
        # 初始化节点
        rospy.init_node('fused_fix_to_gnss', anonymous=True)
        
        # 创建发布者，发布NavSatFix消息到/gnss话题
        self.gnss_pub = rospy.Publisher('/gnss', NavSatFix, queue_size=10)
        
        # 创建订阅者，订阅/fused_fix话题
        self.fused_sub = rospy.Subscriber('/fused_fix', NavSatGHFPD, self.fused_callback)
        
        # 创建订阅者，订阅/fix话题以获取最新的协方差
        self.fix_sub = rospy.Subscriber('/fix', NavSatFix, self.fix_callback)
        
        # 初始化协方差（使用合理的默认值）
        self.position_covariance = [2.89, 0.0, 0.0, 
                                    0.0, 2.89, 0.0, 
                                    0.0, 0.0, 5.78]
        self.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # 初始化GPS状态为NO_FIX
        self.gps_status = NavSatStatus.STATUS_NO_FIX
        
        # 记录最后一次更新协方差的时间
        self.last_covariance_update = rospy.Time.now()
        
        rospy.loginfo("FusedFix to NavSatFix converter initialized with dynamic covariance")

    def fix_callback(self, fix_data):
        """处理/fix话题消息，更新协方差数据"""
        # 更新协方差信息
        self.position_covariance = fix_data.position_covariance
        self.position_covariance_type = fix_data.position_covariance_type
        # 更新GPS状态
        self.gps_status = fix_data.status.status
        
        # 更新最后一次更新时间
        self.last_covariance_update = rospy.Time.now()

    def fused_callback(self, fused_data):
        """处理/fused_fix话题消息，转换为NavSatFix并发布"""
        # 检查协方差是否长时间未更新
        if (rospy.Time.now() - self.last_covariance_update).to_sec() > 5.0:
            rospy.logwarn_throttle(5, "No update from /fix topic for more than 5 seconds, using last known covariance")
        
        # 创建NavSatFix消息
        gnss_msg = NavSatFix()
        
        # 复制header信息
        gnss_msg.header = fused_data.header
        
        # 设置状态信息
        gnss_msg.status.status = NavSatStatus.STATUS_FIX if self.gps_status == 0 else NavSatStatus.STATUS_NO_FIX
        gnss_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # 复制位置信息
        gnss_msg.latitude = fused_data.latitude
        gnss_msg.longitude = fused_data.longitude
        gnss_msg.altitude = fused_data.altitude

        
        # 使用从/fix获取的最新协方差信息
        gnss_msg.position_covariance = self.position_covariance
        gnss_msg.position_covariance_type = self.position_covariance_type
        
        # 发布消息
        self.gnss_pub.publish(gnss_msg)

    def run(self):
        # 保持节点运行
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = FusedFixToGnss()
        converter.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
    
