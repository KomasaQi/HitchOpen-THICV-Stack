#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def cloud_callback(cloud_msg):
    # 解析点云，提取 x, y, z（保留所有字段）
    fields = [f.name for f in cloud_msg.fields]
    points = list(pc2.read_points(cloud_msg, field_names=fields, skip_nans=False))
    
    # 过滤掉 x/y/z 含 NaN 的点
    valid_points = []
    for p in points:
        x, y, z = p[fields.index('x')], p[fields.index('y')], p[fields.index('z')]
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            valid_points.append(p)
    
    # 构造新的点云消息
    if valid_points:
        dense_msg = pc2.create_cloud(
            header=cloud_msg.header,
            fields=cloud_msg.fields,
            points=valid_points
        )
        dense_msg.is_dense = True  # 此时无 NaN，可设为 true
        pub.publish(dense_msg)

if __name__ == '__main__':
    rospy.init_node('ouster_remove_nan')
    pub = rospy.Publisher('/ouster/points_dense', PointCloud2, queue_size=10)
    rospy.Subscriber('/ouster/points', PointCloud2, cloud_callback)
    rospy.loginfo("已启动 NaN 过滤节点，发布话题: /ouster/points_dense")
    rospy.spin()