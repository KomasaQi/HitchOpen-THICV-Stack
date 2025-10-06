import os
import laspy
import open3d as o3d
import numpy as np
import argparse

def convert_las_to_pcd(las_path, pcd_path):
    """将单个LAS文件转换为PCD文件"""
    try:
        # 读取LAS文件
        with laspy.open(las_path) as f:
            las = f.read()
            
            # 提取点坐标
            points = np.vstack((las.x, las.y, las.z)).transpose()
            
            # 创建Open3D点云对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # 尝试提取颜色信息（如果存在）
            if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
                # LAS颜色通常是16位整数，需要归一化到[0,1]范围
                colors = np.vstack((las.red, las.green, las.blue)).transpose()
                colors = colors / 65535.0  # 归一化
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # 保存为PCD文件
            o3d.io.write_point_cloud(pcd_path, pcd)
            return True
            
    except Exception as e:
        print(f"转换 {las_path} 时出错: {str(e)}")
        return False

def batch_convert(folder_path):
    """批量转换文件夹中的所有LAS文件"""
    # 检查文件夹是否存在
    if not os.path.isdir(folder_path):
        print(f"错误: 文件夹 {folder_path} 不存在")
        return
    
    # 遍历文件夹中的所有文件
    total_files = 0
    converted_files = 0
    
    for filename in os.listdir(folder_path):
        # 检查文件扩展名是否为LAS或LAZ
        if filename.lower().endswith(('.las', '.laz')):
            total_files += 1
            las_path = os.path.join(folder_path, filename)
            
            # 生成PCD文件名（替换扩展名为.pcd）
            pcd_filename = os.path.splitext(filename)[0] + '.pcd'
            pcd_path = os.path.join(folder_path, pcd_filename)
            
            # 检查PCD文件是否已存在
            if os.path.exists(pcd_path):
                print(f"跳过 {filename} - PCD文件已存在")
                continue
            
            # 转换文件
            print(f"转换 {filename}...")
            if convert_las_to_pcd(las_path, pcd_path):
                converted_files += 1
                print(f"已保存为 {pcd_filename}")
    
    # 输出转换统计信息
    print("\n转换完成!")
    print(f"总文件数: {total_files}")
    print(f"成功转换: {converted_files}")
    print(f"转换失败: {total_files - converted_files}")

if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='批量将LAS/LAZ点云文件转换为PCD格式')
    parser.add_argument('folder', help='包含LAS/LAZ文件的文件夹路径')
    args = parser.parse_args()
    
    # 执行批量转换
    batch_convert(args.folder)
    