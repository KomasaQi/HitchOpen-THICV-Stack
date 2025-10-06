import os
import open3d as o3d
import numpy as np
import argparse

def read_point_cloud(file_path):
    """读取点云文件"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"点云文件不存在: {file_path}")
    
    # 读取点云
    pcd = o3d.io.read_point_cloud(file_path)
    
    if len(pcd.points) == 0:
        raise ValueError(f"点云文件为空: {file_path}")
    
    return pcd

def calculate_centroid(pcd):
    """计算点云的中心点（质心）"""
    points = np.asarray(pcd.points)
    centroid = np.mean(points, axis=0)
    return centroid

def read_radius_from_file(range_file_path):
    """从范围文件中读取半径值"""
    if not os.path.exists(range_file_path):
        raise FileNotFoundError(f"范围文件不存在: {range_file_path}")
    
    with open(range_file_path, 'r') as f:
        # 假设文件中只包含一个数值，表示半径
        radius_str = f.read().strip()
        try:
            radius = float(radius_str)
            if radius <= 0:
                raise ValueError("半径必须是正数")
            return radius
        except ValueError:
            raise ValueError(f"范围文件内容无效，应为正数: {radius_str}")

def filter_point_cloud_by_radius(pcd, center, radius):
    """根据中心点和半径过滤点云，只保留范围内的点"""
    points = np.asarray(pcd.points)
    
    # 计算每个点到中心点的距离
    distances = np.linalg.norm(points - center, axis=1)
    
    # 筛选出距离小于等于半径的点
    within_radius = distances > radius
    filtered_points = points[within_radius]
    
    # 创建新的点云
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    
    # 如果有颜色信息，也保留相应的颜色
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_colors = colors[within_radius]
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
    
    return filtered_pcd

def visualize_results(reference_pcd, filtered_pcd, center, radius):
    """可视化结果：参考点云（灰色）、过滤后的点云（红色）和中心点（绿色球体）"""
    # 创建中心点的可视化球体
    center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius*0.1)
    center_sphere.translate(center)
    center_sphere.paint_uniform_color([0, 1, 0])  # 绿色
    
    # 创建半径范围的可视化球体（半透明）
    radius_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    radius_sphere.translate(center)
    radius_sphere.paint_uniform_color([0, 0, 1])  # 蓝色
    radius_sphere.compute_vertex_normals()
    # radius_sphere.rendering_option.mesh_show_back_face = True
    # radius_sphere.rendering_option.alpha = 0.1  # 半透明
    
    # 设置参考点云颜色为灰色
    reference_pcd_copy = reference_pcd
    reference_pcd_copy.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色
    
    # 设置过滤后的点云颜色为红色
    filtered_pcd_copy = filtered_pcd
    filtered_pcd_copy.paint_uniform_color([1, 0, 0])  # 红色
    
    # 可视化
    o3d.visualization.draw_geometries(
        [reference_pcd_copy, filtered_pcd_copy, center_sphere, radius_sphere],
        window_name="点云过滤结果",
        width=1280,
        height=720
    )

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='根据参考点云的中心点和半径范围过滤第二个点云')
    parser.add_argument('reference_pcd', help='参考点云文件路径（第一个点云）')
    parser.add_argument('target_pcd', help='目标点云文件路径（第二个点云）')
    parser.add_argument('range_file', help='包含半径值的范围文件路径')
    parser.add_argument('--output', help='过滤后的点云输出文件路径', 
                      default='filtered_point_cloud.pcd')
    args = parser.parse_args()
    
    try:
        # 读取参考点云并计算中心点
        print(f"读取参考点云: {args.reference_pcd}")
        reference_pcd = read_point_cloud(args.reference_pcd)
        centroid = calculate_centroid(reference_pcd)
        print(f"参考点云中心点坐标: {centroid}")
        
        # 读取半径范围
        print(f"读取范围文件: {args.range_file}")
        radius = read_radius_from_file(args.range_file)
        print(f"使用的过滤半径: {radius}")
        
        # 读取目标点云并过滤
        print(f"读取目标点云: {args.target_pcd}")
        target_pcd = read_point_cloud(args.target_pcd)
        print(f"原始目标点云点数: {len(target_pcd.points)}")
        
        filtered_pcd = filter_point_cloud_by_radius(target_pcd, centroid, radius)
        print(f"过滤后的目标点云点数: {len(filtered_pcd.points)}")
        
        # 保存过滤后的点云
        o3d.io.write_point_cloud(args.output, filtered_pcd)
        print(f"过滤后的点云已保存至: {args.output}")
        
        # 可视化结果
        print("正在显示可视化结果...")
        visualize_results(reference_pcd, filtered_pcd, centroid, radius)
        
    except Exception as e:
        print(f"处理过程中出错: {str(e)}")
        exit(1)

if __name__ == "__main__":
    main()
    