#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def cloud_callback(cloud_msg):
    try:
        # 提取x, y, z字段的偏移量和类型
        x_idx = y_idx = z_idx = -1
        x_type = y_type = z_type = np.float32
        
        for i, field in enumerate(cloud_msg.fields):
            if field.name == 'x':
                x_idx = i
                if field.datatype == PointField.FLOAT64:
                    x_type = np.float64
            elif field.name == 'y':
                y_idx = i
                if field.datatype == PointField.FLOAT64:
                    y_type = np.float64
            elif field.name == 'z':
                z_idx = i
                if field.datatype == PointField.FLOAT64:
                    z_type = np.float64
        
        if x_idx == -1 or y_idx == -1 or z_idx == -1:
            rospy.logwarn("点云不包含x, y, z字段")
            return

        # 计算点数量和步长
        point_step = cloud_msg.point_step
        num_points = len(cloud_msg.data) // point_step
        if num_points == 0:
            return

        # 获取x, y, z的偏移量
        x_offset = cloud_msg.fields[x_idx].offset
        y_offset = cloud_msg.fields[y_idx].offset
        z_offset = cloud_msg.fields[z_idx].offset

        # 读取原始字节数据为uint8数组（兼容所有NumPy版本）
        data = np.frombuffer(cloud_msg.data, dtype=np.uint8)

        # 计算每个坐标的字节大小
        x_size = 8 if x_type == np.float64 else 4
        y_size = 8 if y_type == np.float64 else 4
        z_size = 8 if z_type == np.float64 else 4

        # 构造x, y, z数组（不使用stride参数，兼容旧版NumPy）
        x = np.ndarray(
            shape=(num_points,),
            dtype=x_type,
            buffer=data,
            offset=x_offset,
            strides=(point_step,)
        )
        y = np.ndarray(
            shape=(num_points,),
            dtype=y_type,
            buffer=data,
            offset=y_offset,
            strides=(point_step,)
        )
        z = np.ndarray(
            shape=(num_points,),
            dtype=z_type,
            buffer=data,
            offset=z_offset,
            strides=(point_step,)
        )

        # 过滤NaN点
        valid_mask = ~(np.isnan(x) | np.isnan(y) | np.isnan(z))
        valid_indices = np.where(valid_mask)[0]
        if len(valid_indices) == 0:
            return

        # 提取有效点的原始字节数据
        valid_data = bytearray()
        for idx in valid_indices:
            start = idx * point_step
            end = start + point_step
            valid_data.extend(cloud_msg.data[start:end])

        # 构造新点云消息
        dense_msg = PointCloud2()
        dense_msg.header = cloud_msg.header
        dense_msg.height = 1
        dense_msg.width = len(valid_indices)
        dense_msg.fields = cloud_msg.fields
        dense_msg.is_bigendian = cloud_msg.is_bigendian
        dense_msg.point_step = point_step
        dense_msg.row_step = point_step * len(valid_indices)
        dense_msg.data = valid_data
        dense_msg.is_dense = True

        pub.publish(dense_msg)

    except Exception as e:
        rospy.logerr(f"处理点云时出错: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('ouster_remove_nan')
    pub = rospy.Publisher('/ouster/points_dense', PointCloud2, queue_size=30)
    rospy.Subscriber(
        '/ouster/points', 
        PointCloud2, 
        cloud_callback,
        buff_size=2**24
    )
    rospy.loginfo("已启动高效NaN过滤节点，发布话题: /ouster/points_dense")
    rospy.spin()





# #!/usr/bin/env python3
# import rospy
# import numpy as np
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2

# def cloud_callback(cloud_msg):
#     # 解析点云，提取 x, y, z（保留所有字段）
#     fields = [f.name for f in cloud_msg.fields]
#     points = list(pc2.read_points(cloud_msg, field_names=fields, skip_nans=False))
    
#     # 过滤掉 x/y/z 含 NaN 的点
#     valid_points = []
#     counter = 0
#     for p in points:
#         x, y, z = p[fields.index('x')], p[fields.index('y')], p[fields.index('z')]
#         if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
#             valid_points.append(p)
    
#     # 构造新的点云消息
#     if valid_points:
#         dense_msg = pc2.create_cloud(
#             header=cloud_msg.header,
#             fields=cloud_msg.fields,
#             points=valid_points
#         )
#         dense_msg.is_dense = True  # 此时无 NaN，可设为 true
#         pub.publish(dense_msg)

# if __name__ == '__main__':
#     rospy.init_node('ouster_remove_nan')
#     pub = rospy.Publisher('/ouster/points_dense', PointCloud2, queue_size=1)
#     rospy.Subscriber('/ouster/points', PointCloud2, cloud_callback)
#     rospy.loginfo("已启动 NaN 过滤节点，发布话题: /ouster/points_dense")
#     rospy.spin()