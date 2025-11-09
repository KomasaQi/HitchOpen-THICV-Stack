#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
import tf2_ros
import tf2_geometry_msgs

from threading import Lock

class GPSBiaserNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('gps_bias_node', anonymous=True)
        self.lock = Lock()  # 线程安全锁，避免数据竞争

        # -------------------------- 1. 可配置参数（通过ROS参数服务器设置） --------------------------
        self.total_lidar_frames = rospy.get_param('~total_lidar_frames', 50)  # 需收集的雷达帧总数
        self.avg_lidar_frames = rospy.get_param('~avg_lidar_frames', 15)      # 取平均的最后帧数
        self.radar_odom_topic = rospy.get_param('~radar_odom_topic', '/liorf_localization/mapping/odometry')  # 雷达定位话题
        self.gps_odom_topic = rospy.get_param('~gps_odom_topic', '/odometry/gps')  # 原始GPS话题
        self.biased_gps_topic = rospy.get_param('~biased_gps_topic', '/odometry/gps_biased')  # 偏移后GPS话题

        # 参数合法性检查
        if self.avg_lidar_frames > self.total_lidar_frames:
            rospy.logerr("【错误】平均帧数（avg_lidar_frames=%d）不能大于总收集帧数（total_lidar_frames=%d）！", 
                         self.avg_lidar_frames, self.total_lidar_frames)
            rospy.signal_shutdown("参数不合法")
            return

        # -------------------------- 2. 核心变量初始化 --------------------------
        self.lidar_odom_list = []  # 存储收集的雷达定位帧
        self.bias_pose = Pose()    # 雷达定位的平均偏移姿态（x,y,z + quaternion）
        self.is_offset_ready = False  # 偏移量是否计算完成的标志

        # -------------------------- 3. ROS话题订阅与发布 --------------------------
        # 订阅雷达定位话题（用于计算偏移）
        self.sub_lidar = rospy.Subscriber(
            self.radar_odom_topic, Odometry, self.lidar_odom_callback, queue_size=10
        )
        # 订阅原始GPS话题（用于叠加偏移）
        self.sub_gps = rospy.Subscriber(
            self.gps_odom_topic, Odometry, self.gps_odom_callback, queue_size=10
        )
        # 发布偏移后的GPS话题
        self.pub_biased_gps = rospy.Publisher(
            self.biased_gps_topic, Odometry, queue_size=10
        )

        rospy.loginfo("【GPS偏移节点启动成功】")
        rospy.loginfo("参数配置：总收集雷达帧=%d，取平均帧数=%d", 
                      self.total_lidar_frames, self.avg_lidar_frames)
        rospy.loginfo("等待收集雷达定位帧...")

    def lidar_odom_callback(self, msg):
        """
        雷达定位话题回调：收集指定帧数的雷达帧，达到总数后计算平均偏移
        """
        with self.lock:  # 加锁保护共享变量
            # 1. 收集雷达帧（未达到总帧数时）
            if len(self.lidar_odom_list) < self.total_lidar_frames:
                self.lidar_odom_list.append(msg)
                rospy.loginfo_throttle(1, "已收集雷达帧：%d/%d", 
                                       len(self.lidar_odom_list), self.total_lidar_frames)
                return

            # 2. 达到总帧数后，计算最后N帧的平均偏移（仅执行一次）
            if not self.is_offset_ready:
                rospy.loginfo("开始计算雷达定位平均偏移（取最后%d帧）...", self.avg_lidar_frames)
                # 取最后N帧雷达数据
                last_n_lidar_odoms = self.lidar_odom_list[-self.avg_lidar_frames:]
                # 计算平均姿态（位置+四元数）
                self.bias_pose = self.calculate_average_pose(last_n_lidar_odoms)
                # 标记偏移量已就绪
                self.is_offset_ready = True
                # 打印偏移结果
                rospy.loginfo("="*50)
                rospy.loginfo("雷达定位平均偏移计算完成！")
                rospy.loginfo("偏移位置：x=%.3f, y=%.3f, z=%.3f",
                              self.bias_pose.position.x,
                              self.bias_pose.position.y,
                              self.bias_pose.position.z)

                rospy.loginfo("="*50)
                # 停止订阅雷达话题（后续无需再收集）
                self.sub_lidar.unregister()
                rospy.loginfo("已停止收集雷达帧，开始处理GPS数据...")

    def calculate_average_pose(self, odom_list):
        """
        计算多个Odometry消息的平均姿态（位置算术平均，四元数球面线性插值平均）
        :param odom_list: Odometry消息列表
        :return: 平均后的Pose对象
        """
        avg_pose = Pose()
        frame_count = len(odom_list)
        if frame_count == 0:
            rospy.logwarn("计算平均姿态时输入帧列表为空，返回默认姿态（0,0,0 + 单位四元数）")
            avg_pose.orientation.w = 1.0  # 单位四元数（无旋转）
            return avg_pose

        # -------------------------- 1. 位置平均（算术平均） --------------------------
        sum_x = sum_y = sum_z = 0.0
        for odom in odom_list:
            sum_x += odom.pose.pose.position.x
            sum_y += odom.pose.pose.position.y
            sum_z += odom.pose.pose.position.z
        avg_pose.position.x = sum_x / frame_count
        avg_pose.position.y = sum_y / frame_count
        avg_pose.position.z = sum_z / frame_count


        return avg_pose

    def gps_odom_callback(self, msg):
        """
        GPS话题回调：将GPS姿态叠加雷达偏移量，发布偏移后的GPS话题
        """
        # 偏移量未就绪时，跳过处理
        if not self.is_offset_ready:
            rospy.logwarn_throttle(2, "偏移量尚未计算完成，暂不处理GPS数据...")
            return

        with self.lock:
            # -------------------------- 1. 复制原始GPS消息结构 --------------------------
            biased_gps = Odometry()
            biased_gps.header = msg.header  # 保持时间戳和坐标系（与原始GPS一致）
            biased_gps.child_frame_id = msg.child_frame_id
            biased_gps.twist = msg.twist    #  twist（速度）不叠加偏移（仅位置和姿态偏移）
            biased_gps.pose.covariance = msg.pose.covariance  # 保留GPS原始协方差（精度信息）
            # -------------------------- 2. 叠加位置偏移 --------------------------
            biased_gps.pose.pose.position.x = self.bias_pose.position.x + msg.pose.pose.position.x
            biased_gps.pose.pose.position.y = self.bias_pose.position.y + msg.pose.pose.position.y
            biased_gps.pose.pose.position.z = self.bias_pose.position.z + msg.pose.pose.position.z



            # -------------------------- 4. 发布偏移后的GPS话题 --------------------------
            self.pub_biased_gps.publish(biased_gps)
            rospy.logdebug_throttle(1, "GPS偏移后位置：x=%.3f, y=%.3f",
                                    biased_gps.pose.pose.position.x,
                                    biased_gps.pose.pose.position.y)

if __name__ == '__main__':
    try:
        node = GPSBiaserNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS偏移节点正常退出")
    except Exception as e:
        rospy.logerr("GPS偏移节点异常退出：%s", str(e))