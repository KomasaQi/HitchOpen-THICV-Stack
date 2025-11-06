#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from collections import deque

class TrajectoryAnalyzer:
    def __init__(self):
        # 初始化节点
        rospy.init_node('trajectory_analyzer', anonymous=True)
        
        # 参数设置（可在launch文件或参数服务器中配置）
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.1)  # 距离阈值(m)
        self.max_points = rospy.get_param('~max_points', 100)                  # 最大轨迹点数
        self.wheelbase = rospy.get_param('~wheelbase', 0.3)                    # 车辆轴距(m)
        
        # 存储轨迹点的队列（存储PoseStamped类型，包含XYZ和时间戳）
        self.trajectory = deque(maxlen=self.max_points)
        self.last_recorded_point = None  # 上一个记录的点（仅用于XY距离计算）
        
        # 订阅odometry话题
        rospy.Subscriber('/liorf/mapping/odometry', Odometry, self.odometry_callback)
        
        # 发布轨迹可视化话题（nav_msgs/Path类型，带时间戳）
        self.traj_pub = rospy.Publisher('/traj_vis', Path, queue_size=10)
        
        rospy.loginfo("轨迹分析器已启动，参数:")
        rospy.loginfo(f"  距离阈值: {self.distance_threshold}m")
        rospy.loginfo(f"  最大轨迹点数: {self.max_points}")
        rospy.loginfo(f"  车辆轴距: {self.wheelbase}m")

    def odometry_callback(self, msg):
        """处理里程计数据回调函数"""
        # 获取当前位置(XYZ)和时间戳
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z
        current_stamp = msg.header.stamp
        current_frame = msg.header.frame_id  # 坐标系ID（保持与里程计一致）
        
        # 构建PoseStamped消息（用于存储和发布）
        current_pose = PoseStamped()
        current_pose.header.stamp = current_stamp
        current_pose.header.frame_id = current_frame
        current_pose.pose.position.x = current_x
        current_pose.pose.position.y = current_y
        current_pose.pose.position.z = current_z
        # 姿态信息不使用，保持默认（全零）
        current_pose.pose.orientation.w = 1.0  # 确保四元数合法
        
        # 仅用XY判断是否记录点（Z不参与距离计算）
        current_xy = (current_x, current_y)
        if self.should_record_point(current_xy):
            self.trajectory.append(current_pose)
            self.last_recorded_point = current_xy  # 仅记录XY用于下次距离计算
            
            # 发布更新后的轨迹
            self.publish_trajectory(current_frame)
            
            # 当轨迹点足够时，进行圆拟合和转角估计
            if len(self.trajectory) >= 3:
                try:
                    radius = self.estimate_circle_radius()
                    steering_angle = self.estimate_steering_angle(radius)
                    self.print_info(radius, steering_angle)
                except Exception as e:
                    rospy.logwarn(f"计算出错: {str(e)}")
            else:
                rospy.loginfo(f"轨迹点不足，无法进行拟合,当前轨迹点数: {len(self.trajectory)}")

    def should_record_point(self, current_xy):
        """判断是否应该记录当前点（仅用XY坐标）"""
        # 如果是第一个点，直接记录
        if self.last_recorded_point is None:
            return True
            
        # 计算与上一个记录点的XY距离
        dx = current_xy[0] - self.last_recorded_point[0]
        dy = current_xy[1] - self.last_recorded_point[1]
        distance = math.hypot(dx, dy)
        
        # 距离超过阈值则记录
        return distance >= self.distance_threshold

    def estimate_circle_radius(self):
        """使用最小二乘法估计圆半径（仅用XY坐标）"""
        # 从轨迹点中提取XY坐标
        x = np.array([p.pose.position.x for p in self.trajectory])
        y = np.array([p.pose.position.y for p in self.trajectory])
        n = len(x)
        
        # 构造线性方程组：x² + y² = A*x + B*y + C
        X = np.column_stack((x, y, np.ones(n)))  # 设计矩阵
        Y = x**2 + y**2                          # 目标向量
        
        # 线性最小二乘求解A, B, C
        params, residuals, rank, s = np.linalg.lstsq(X, Y, rcond=None)
        A, B, C = params
        
        # 计算圆心(a,b)和半径r
        a = A / 2  # 圆心x坐标
        b = B / 2  # 圆心y坐标
        radius = math.sqrt(a**2 + b**2 + C)  # 半径计算公式
        
        return radius

    def estimate_steering_angle(self, radius):
        """根据运动学模型估计前轮转角"""
        # 防止除以零（直线行驶时半径无穷大）
        if radius < 0.001:
            return 0.0
            
        # 运动学模型：tan(δ) = L/R，其中δ为前轮转角，L为轴距，R为转弯半径
        steering_rad = math.atan(self.wheelbase / radius)
        # 转换为角度（可选，便于阅读）
        steering_deg = math.degrees(steering_rad)
        
        return steering_deg

    def publish_trajectory(self, frame_id):
        """发布轨迹到/traj_vis话题（nav_msgs/Path类型）"""
        traj_msg = Path()
        traj_msg.header.stamp = rospy.Time.now()  # 轨迹消息的时间戳（当前时间）
        traj_msg.header.frame_id = frame_id       # 与里程计坐标系保持一致
        traj_msg.poses = list(self.trajectory)    # 将队列转换为列表存入
        self.traj_pub.publish(traj_msg)

    def print_info(self, radius, steering_angle):
        """在终端输出信息"""
        # 清除当前行并打印新信息（保持在同一行更新）
        print(f"\r轨迹点数: {len(self.trajectory)}, 估计圆半径: {radius:.2f}m, 前轮转角: {steering_angle:.2f}°", end='')

if __name__ == '__main__':
    try:
        analyzer = TrajectoryAnalyzer()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("\n程序被中断")
    finally:
        # 确保最后换行
        print()

# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# import rospy
# import math
# import numpy as np
# from nav_msgs.msg import Odometry
# from collections import deque

# class TrajectoryAnalyzer:
#     def __init__(self):
#         # 初始化节点
#         rospy.init_node('trajectory_analyzer', anonymous=True)
        
#         # 参数设置（可在launch文件或参数服务器中配置）
#         self.distance_threshold = rospy.get_param('~distance_threshold', 0.1)  # 距离阈值(m)
#         self.max_points = rospy.get_param('~max_points', 100)                  # 最大轨迹点数
#         self.wheelbase = rospy.get_param('~wheelbase', 0.5)                    # 车辆轴距(m)
        
#         # 存储轨迹点的队列（先进先出，自动维护最大长度）
#         self.trajectory = deque(maxlen=self.max_points)
#         self.last_recorded_point = None  # 上一个记录的点
        
#         # 订阅odometry话题
#         rospy.Subscriber('/odometry/imu', Odometry, self.odometry_callback)
        
#         rospy.loginfo("轨迹分析器已启动，参数:")
#         rospy.loginfo(f"  距离阈值: {self.distance_threshold}m")
#         rospy.loginfo(f"  最大轨迹点数: {self.max_points}")
#         rospy.loginfo(f"  车辆轴距: {self.wheelbase}m")

#     def odometry_callback(self, msg):
#         """处理里程计数据回调函数"""
#         # 获取当前位置(x, y)
#         current_x = msg.pose.pose.position.x
#         current_y = msg.pose.pose.position.y
#         current_point = (current_x, current_y)
        
#         # 判断是否需要记录当前点
#         if self.should_record_point(current_point):
#             self.trajectory.append(current_point)
#             self.last_recorded_point = current_point
            
#             # 当轨迹点足够时，进行圆拟合和转角估计
#             if len(self.trajectory) >= 3:
#                 try:
#                     radius = self.estimate_circle_radius()
#                     steering_angle = self.estimate_steering_angle(radius)
#                     self.print_info(radius, steering_angle)
#                 except Exception as e:
#                     rospy.logwarn(f"计算出错: {str(e)}")
#             else:
#                 rospy.loginfo("轨迹点不足，无法进行拟合,当前轨迹点数: {}".format(len(self.trajectory)))

#     def should_record_point(self, current_point):
#         """判断是否应该记录当前点"""
#         # 如果是第一个点，直接记录
#         if self.last_recorded_point is None:
#             return True
            
#         # 计算与上一个记录点的距离
#         dx = current_point[0] - self.last_recorded_point[0]
#         dy = current_point[1] - self.last_recorded_point[1]
#         distance = math.hypot(dx, dy)
        
#         # 距离超过阈值则记录
#         return distance >= self.distance_threshold

#     def estimate_circle_radius(self):
#         """使用最小二乘法估计圆半径，基于提供的MATLAB代码转换"""
#         # 将轨迹点转换为numpy数组
#         path = np.array(self.trajectory)
#         n = len(path)
        
#         # 提取x和y坐标
#         x = path[:, 0]
#         y = path[:, 1]
        
#         # 构造线性方程组：x² + y² = A*x + B*y + C
#         X = np.column_stack((x, y, np.ones(n)))  # 设计矩阵
#         Y = x**2 + y**2                          # 目标向量
        
#         # 线性最小二乘求解A, B, C
#         params, residuals, rank, s = np.linalg.lstsq(X, Y, rcond=None)
#         A, B, C = params
        
#         # 计算圆心(a,b)和半径r
#         a = A / 2  # 圆心x坐标
#         b = B / 2  # 圆心y坐标
#         radius = math.sqrt(a**2 + b**2 + C)  # 半径计算公式
        
#         return radius

#     def estimate_steering_angle(self, radius):
#         """根据运动学模型估计前轮转角"""
#         # 防止除以零（直线行驶时半径无穷大）
#         if radius < 0.001:
#             return 0.0
            
#         # 运动学模型：tan(δ) = L/R，其中δ为前轮转角，L为轴距，R为转弯半径
#         steering_rad = math.atan(self.wheelbase / radius)
#         # 转换为角度（可选，便于阅读）
#         steering_deg = math.degrees(steering_rad)
        
#         return steering_deg

#     def print_info(self, radius, steering_angle):
#         """在终端输出信息"""
#         # 清除当前行并打印新信息（保持在同一行更新）
#         print(f"\r轨迹点数: {len(self.trajectory)}, 估计圆半径: {radius:.2f}m, 前轮转角: {steering_angle:.2f}°", end='')

# if __name__ == '__main__':
#     try:
#         analyzer = TrajectoryAnalyzer()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         print("\n程序被中断")
#     finally:
#         # 确保最后换行
#         print()