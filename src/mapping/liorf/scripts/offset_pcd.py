import open3d as o3d
import argparse
import os

def main():
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='PCD点云坐标偏移工具')
    parser.add_argument('input_file', type=str, help='输入PCD文件路径')
    parser.add_argument('--dx', type=float, default=0.0, help='X轴偏移量')
    parser.add_argument('--dy', type=float, default=0.0, help='Y轴偏移量')
    parser.add_argument('--dz', type=float, default=0.0, help='Z轴偏移量')
    parser.add_argument('--output', type=str, default=None, help='输出文件名（默认在原文件名后加_offset）')
    
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not os.path.exists(args.input_file):
        print(f"错误：输入文件不存在 - {args.input_file}")
        return
    
    # 读取PCD文件
    print(f"正在读取文件: {args.input_file}")
    pcd = o3d.io.read_point_cloud(args.input_file)
    
    if not pcd.has_points():
        print("错误：文件中没有点云数据")
        return
    
    # 获取点云数据
    points = pcd.points
    
    # 应用坐标偏移
    for i in range(len(points)):
        points[i] = [
            points[i][0] + args.dx,
            points[i][1] + args.dy,
            points[i][2] + args.dz
        ]
    
    # 设置输出文件名
    if args.output:
        output_path = os.path.join(os.path.dirname(args.input_file), args.output)
    else:
        base_name = os.path.basename(args.input_file)
        name, ext = os.path.splitext(base_name)
        output_path = os.path.join(os.path.dirname(args.input_file), f"{name}_offset.pcd")
    
    # 保存新的PCD文件
    print(f"正在保存文件: {output_path}")
    o3d.io.write_point_cloud(output_path, pcd)
    
    print("处理完成！")

if __name__ == "__main__":
    main()
    
# python3 offset_pcd.py /media/komasa/Data/Ubuntu文件备份/data/pcd_test/GlobalMap.pcd --dx -449389.0 --dy -3215400.0 --dz -1028.0 --output GlobalMap_offset.pcd

# python3 offset_pcd.py /home/user/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Tianmen_Map/GlobalMap.pcd --dx 0.0 --dy 0.0 --dz -800.0 --output GlobalMap_offsetted.pcd

# % X、Y、Z坐标偏移量
# offsetX = -449389.0;   % X方向偏移
# offsetY = -3215400.0;    % Y方向偏移
# offsetZ = -1028.0;    % Z方向偏移
