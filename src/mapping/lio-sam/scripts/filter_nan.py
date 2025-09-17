#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

class NaNPointFilter:
    """过滤Carla激光雷达点云中的NaN点并添加ring字段的ROS节点"""
    def __init__(self):
        # 通过ROS参数配置激光雷达和过滤阈值
        self.lidar_channels = rospy.get_param("~lidar_channels", 64)
        self.upper_fov = rospy.get_param("~upper_fov", 15.0)
        self.lower_fov = rospy.get_param("~lower_fov", -25.0)
        self.min_distance = rospy.get_param("~min_distance", 1.0)
        self.z_min = rospy.get_param("~z_min", -10.0)
        self.z_max = rospy.get_param("~z_max", 100.0)

        # 预计算FOV和角度步长（弧度）
        self.lower_fov_rad = math.radians(self.lower_fov)
        self.fov_range = math.radians(self.upper_fov - self.lower_fov)
        self.angle_step = self.fov_range / (self.lidar_channels - 1)

        # 订阅和发布话题
        self.sub = rospy.Subscriber(
            "/carla/ego_vehicle/lidar",
            PointCloud2,
            self.pointcloud_callback,
            queue_size=10
        )
        self.pub = rospy.Publisher(
            "/carla/ego_vehicle/lidar_filtered",
            PointCloud2,
            queue_size=10
        )

        
        
        rospy.loginfo(f"✅ 点云过滤节点启动：{self.lidar_channels}线，FOV[{self.lower_fov}, {self.upper_fov}]度")

    def calculate_ring(self, x, y, z):
        """根据3D点坐标计算ring值"""
        distance = math.sqrt(x**2 + y**2 + z**2)
        if distance < self.min_distance:
            rospy.logwarn_once("点距离传感器过近，分配ring=0")
            return 0

        # 计算垂直角度
        pitch_angle = math.asin(z / distance)
        normalized_angle = pitch_angle - self.lower_fov_rad
        if normalized_angle < 0.0 or normalized_angle > self.fov_range:
            rospy.logwarn_once(f"垂直角度{pitch_angle:.2f}弧度超出FOV，执行夹紧")
            normalized_angle = max(0.0, min(self.fov_range, normalized_angle))

        ring = int(round(normalized_angle / self.angle_step))
        return max(0, min(self.lidar_channels - 1, ring))

    def pointcloud_callback(self, raw_cloud_msg):
        """处理点云，过滤NaN点，添加ring字段并发布"""
        filtered_points = []
        nan_count = 0
        distance_count = 0
        z_count = 0

        # 遍历点云
        for point in point_cloud2.read_points(
            raw_cloud_msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=False
        ):
            x, y, z, intensity = point

            # 过滤NaN点（包括intensity）
            if not (x == x and y == y and z == z and intensity == intensity):
                nan_count += 1
                continue
            # 过滤距离和高度
            if math.sqrt(x**2 + y**2 + z**2) < self.min_distance:
                distance_count += 1
                continue
            if not (self.z_min <= z <= self.z_max):
                z_count += 1
                continue

            # 计算ring值
            ring = self.calculate_ring(x, y, z)
            filtered_points.append([x, y, z, intensity, ring])

        # 记录过滤统计信息
        total_points = len(filtered_points) + nan_count + distance_count + z_count
        if total_points > 0:
            rospy.loginfo(f"处理{total_points}个点：保留{len(filtered_points)}个，"
                          f"NaN {nan_count}个，距离过近{distance_count}个，高度异常{z_count}个")

        # 发布过滤后的点云
        if filtered_points:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1)
            ]
            filtered_cloud_msg = point_cloud2.create_cloud(
                header=raw_cloud_msg.header,
                fields=fields,
                points=filtered_points
            )
            filtered_cloud_msg.is_dense = True
            self.pub.publish(filtered_cloud_msg)
            

if __name__ == "__main__":
    rospy.init_node("nan_filter_with_ring", anonymous=True)
    try:
        NaNPointFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("❌ 节点已中断！")




