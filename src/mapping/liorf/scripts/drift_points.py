import os
import open3d as o3d
import numpy as np
import argparse

def load_point_cloud(file_path):
    """加载点云文件"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件不存在: {file_path}")
    
    pcd = o3d.io.read_point_cloud(file_path)
    if len(pcd.points) == 0:
        raise ValueError(f"点云文件为空: {file_path}")
    
    print(f"成功加载点云，共 {len(pcd.points)} 个点")
    return pcd

def find_core_region(pcd, percentile=90):
    """找到包含指定百分比点的核心区域"""
    points = np.asarray(pcd.points)
    
    # 计算每个维度的百分位数，找到包含大部分点的范围
    lower_percentile = (100 - percentile) / 2
    upper_percentile = 100 - lower_percentile
    
    # 计算x, y, z三个维度的上下分位数
    x_lower, x_upper = np.percentile(points[:, 0], [lower_percentile, upper_percentile])
    y_lower, y_upper = np.percentile(points[:, 1], [lower_percentile, upper_percentile])
    z_lower, z_upper = np.percentile(points[:, 2], [lower_percentile, upper_percentile])
    
    # 计算核心区域的中心
    center = np.array([
        (x_lower + x_upper) / 2,
        (y_lower + y_upper) / 2,
        (z_lower + z_upper) / 2
    ])
    
    print(f"核心区域中心坐标: {center}")
    print(f"核心区域范围:")
    print(f"  X: [{x_lower:.2f}, {x_upper:.2f}]")
    print(f"  Y: [{y_lower:.2f}, {y_upper:.2f}]")
    print(f"  Z: [{z_lower:.2f}, {z_upper:.2f}]")
    
    return center

def filter_points_by_radius(pcd, center, radius):
    """根据中心和半径过滤点云"""
    points = np.asarray(pcd.points)
    
    # 计算每个点到中心的距离
    distances = np.linalg.norm(points - center, axis=1)
    
    # 筛选出距离小于等于半径的点
    within_radius = distances <= radius
    filtered_points = points[within_radius]
    
    # 创建新的点云
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    
    # 保留颜色信息（如果存在）
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_colors = colors[within_radius]
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
    
    print(f"过滤后保留 {len(filtered_pcd.points)} 个点")
    print(f"过滤掉 {len(pcd.points) - len(filtered_pcd.points)} 个漂移点")
    
    return filtered_pcd

def visualize_results(original_pcd, filtered_pcd, center, radius):
    """可视化原始点云和过滤后的点云"""
    # 创建中心点标记
    center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius*0.05)
    center_sphere.translate(center)
    center_sphere.paint_uniform_color([0, 1, 0])  # 绿色
    
    # 创建半径范围可视化
    radius_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    radius_sphere.translate(center)
    radius_sphere.paint_uniform_color([0, 0, 1])  # 蓝色
    radius_sphere.compute_vertex_normals()
    # radius_sphere.rendering_option.alpha = 0.1  # 半透明
    
    # 设置原始点云为灰色（半透明）
    original_pcd_copy = original_pcd
    original_pcd_copy.paint_uniform_color([0.5, 0.5, 0.5])
    # original_pcd_copy.rendering_option.alpha = 0.3
    
    # 设置过滤后的点云为红色
    filtered_pcd_copy = filtered_pcd
    filtered_pcd_copy.paint_uniform_color([1, 0, 0])
    
    # 可视化
    o3d.visualization.draw_geometries(
        [original_pcd_copy, filtered_pcd_copy, center_sphere, radius_sphere],
        window_name="点云过滤结果",
        width=1280,
        height=720
    )

def main():
    parser = argparse.ArgumentParser(description='过滤点云中的严重漂移点')
    parser.add_argument('input_file', help='输入的PCD文件路径')
    parser.add_argument('--output', help='过滤后的PCD文件输出路径', 
                      default='filtered_pcd.pcd')
    parser.add_argument('--radius', type=float, default=500.0, 
                      help='保留点的半径范围，默认500米')
    parser.add_argument('--percentile', type=float, default=90.0, 
                      help='用于确定核心区域的点百分比，默认90%%')
    args = parser.parse_args()
    
    try:
        # 加载点云
        pcd = load_point_cloud(args.input_file)
        
        # 找到核心区域中心
        center = find_core_region(pcd, args.percentile)
        
        # 过滤点云
        filtered_pcd = filter_points_by_radius(pcd, center, args.radius)
        
        # 保存过滤后的点云
        o3d.io.write_point_cloud(args.output, filtered_pcd)
        print(f"过滤后的点云已保存至: {args.output}")
        
        # 可视化结果
        visualize_results(pcd, filtered_pcd, center, args.radius)
        
    except Exception as e:
        print(f"处理出错: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()
    