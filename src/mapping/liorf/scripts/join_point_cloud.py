import os
import open3d as o3d
import argparse

def process_point_clouds(folder_path, voxel_size=0.05):
    """
    处理文件夹中的点云文件：降采样、拼接并可视化
    
    参数:
        folder_path: 包含点云文件的文件夹路径
        voxel_size: 体素降采样的体素大小，值越大降采样越明显
    """
    # 支持的点云文件格式
    supported_formats = ['.pcd', '.ply', '.xyz', '.xyzn', '.xyzrgb']
    
    # 读取并处理所有点云文件
    pcd_list = []
    for filename in os.listdir(folder_path):
        # 检查文件格式是否支持
        file_ext = os.path.splitext(filename)[1].lower()
        if file_ext in supported_formats:
            file_path = os.path.join(folder_path, filename)
            print(f"正在处理: {file_path}")
            
            # 读取点云
            pcd = o3d.io.read_point_cloud(file_path)
            
            # 检查点云是否为空
            if len(pcd.points) == 0:
                print(f"警告: {filename} 是空的，已跳过")
                continue
            
            # 降采样
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            print(f"降采样完成: {len(pcd.points)} -> {len(downsampled_pcd.points)} 个点")
            
            pcd_list.append(downsampled_pcd)
    
    if not pcd_list:
        print("错误: 没有找到有效的点云文件")
        return
    
    # 拼接所有点云
    print("正在拼接点云...")
    combined_pcd = pcd_list[0]
    for pcd in pcd_list[1:]:
        combined_pcd += pcd
    
    print(f"拼接完成: 总点数 = {len(combined_pcd.points)}")
    
    # 保存拼接后的点云
    output_path = os.path.join(folder_path, "combined_point_cloud.pcd")
    o3d.io.write_point_cloud(output_path, combined_pcd)
    print(f"拼接后的点云已保存至: {output_path}")
    
    # 可视化
    print("正在显示点云...")
    o3d.visualization.draw_geometries([combined_pcd], window_name="拼接后的点云")

if __name__ == "__main__":
    # 解析命令行参数
    # parser = argparse.ArgumentParser(description='点云处理：降采样、拼接与可视化')
    # parser.add_argument('folder', help='包含点云文件的文件夹路径')
    # parser.add_argument('--voxel_size', type=float, default=0.2, 
    #                   help='体素降采样的体素大小，默认0.5')
    # args = parser.parse_args()
    folder = "/home/user/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Tianmen_Map"
    # 处理点云
    # process_point_clouds(args.folder, args.voxel_size)
    process_point_clouds(folder, voxel_size=0.2)
