#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def cloud_callback(cloud_msg):
    # 解析点云数据，提取 x, y, z 字段
    # 注意：根据实际点云结构调整字段名（通常是 'x', 'y', 'z'）
    points = list(pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False))
    
    # 转换为 numpy 数组，方便检查 NaN
    points_np = np.array(points, dtype=np.float32)
    
    # 检查是否存在 NaN（任意维度）
    has_nan = np.isnan(points_np).any()
    # 统计 NaN 的数量
    nan_count = np.isnan(points_np).sum()
    total_points = points_np.shape[0]
    
    # 输出结果
    rospy.loginfo(f"点云总点数: {total_points}")
    rospy.loginfo(f"包含 NaN 的点数: {nan_count}")
    rospy.loginfo(f"是否存在 NaN: {'是' if has_nan else '否'}")
    
    # 若存在 NaN，可打印前10个含 NaN 的点（可选）
    if has_nan:
        nan_indices = np.where(np.isnan(points_np).any(axis=1))[0]
        rospy.loginfo(f"前10个含 NaN 的点索引: {nan_indices[:10]}")
    
    # 检查一次后退出（如需持续检查，注释掉此行）
    # rospy.signal_shutdown("检查完成")

def main():
    rospy.init_node('check_nan_in_cloud', anonymous=True)
    rospy.loginfo("开始检查 /ouster/points 中的 NaN...")
    # 订阅点云话题
    rospy.Subscriber("/ouster/points", PointCloud2, cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()