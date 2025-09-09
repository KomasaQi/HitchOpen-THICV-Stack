import open3d as o3d
import numpy as np
from PIL import Image
import os

# -------------------------- 1. 配置参数（根据需求调整） --------------------------
PCD_INPUT_PATH = "/home/komasa/HitchOpen-THICV-Stack/src/planning/race_global_static_planner/config/carla_town10_map/carla_town10_map.pcd"       # 输入PCD文件路径
PGM_OUTPUT_PATH = "/home/komasa/HitchOpen-THICV-Stack/src/planning/race_global_static_planner/config/carla_town10_map/carla_town10_map.pgm"  # 输出PGM文件路径（与YAML的image字段一致）
RESOLUTION = 0.2                          # 地图分辨率：0.2m/像素（与YAML一致）
HEIGHT_MIN = 0.05                           # 保留的最小高度（0.1m）
HEIGHT_MAX = 2.0                           # 保留的最大高度（2.0m）


# -------------------------- 1. 读取并预处理PCD点云 --------------------------
def preprocess_pcd(pcd_path, z_min, z_max):
    # 读取PCD文件
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        raise ValueError("PCD文件为空，请检查路径或文件完整性！")
    
    # 转换为numpy数组（X, Y, Z）
    points = np.asarray(pcd.points)
    
    # 过滤高度在[0.1, 2.0]范围内的点（视为障碍物）
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    obstacle_points = points[mask]
    
    # 投影到XY平面（保留X, Y坐标）
    obstacle_points_2d = obstacle_points[:, :2]
    
    return obstacle_points_2d


# -------------------------- 2. 计算地图参数（核心：保持原点(0,0)不变） --------------------------
def calculate_map_params(points_2d, resolution):
    # 计算点云在X、Y方向的最大范围（确保覆盖所有点）
    x_all = points_2d[:, 0]
    y_all = points_2d[:, 1]
    
    # 确定地图需要覆盖的范围（包含原点(0,0)和所有障碍物点）
    x_min = min(np.min(x_all), 0)  # 确保原点X=0在范围内
    x_max = max(np.max(x_all), 0)  # 确保原点X=0在范围内
    y_min = min(np.min(y_all), 0)  # 确保原点Y=0在范围内
    y_max = max(np.max(y_all), 0)  # 确保原点Y=0在范围内
    
    # 计算地图原点（PGM左下角坐标），确保原点(0,0)在地图内
    # 地图原点 = 最小坐标（向下取整到分辨率的整数倍，确保对齐栅格）
    map_origin_x = np.floor(x_min / resolution) * resolution
    map_origin_y = np.floor(y_min / resolution) * resolution
    
    # 计算地图尺寸（像素数）
    map_width = int(np.ceil((x_max - map_origin_x) / resolution))
    map_height = int(np.ceil((y_max - map_origin_y) / resolution))
    
    # 验证原点(0,0)是否在地图范围内
    origin_col = int((0 - map_origin_x) / resolution)
    origin_row = map_height - int((0 - map_origin_y) / resolution)
    if not (0 <= origin_col < map_width and 0 <= origin_row < map_height):
        raise ValueError("计算错误：原点(0,0)不在地图范围内，请检查点云数据！")
    
    print(f"地图参数计算完成：")
    print(f"- 地图原点（PGM左下角）：({map_origin_x:.2f}, {map_origin_y:.2f})")
    print(f"- 地图尺寸：{map_width}×{map_height}像素（宽×高）")
    print(f"- 原点(0,0)在地图中的像素位置：行={origin_row}, 列={origin_col}")
    
    return map_origin_x, map_origin_y, map_width, map_height


# -------------------------- 3. 栅格化：将2D点云转为PGM地图 --------------------------
def rasterize_to_pgm(points_2d, map_origin_x, map_origin_y, map_width, map_height, resolution):
    # 初始化PGM：255（白色，空闲区域）
    pgm_data = np.ones((map_height, map_width), dtype=np.uint8) * 255
    
    # 遍历所有障碍物点，标记对应栅格为占据（0，黑色）
    for (x, y) in points_2d:
        # 计算点在PGM中的列号（X方向）
        col = int((x - map_origin_x) / resolution)
        # 计算点在PGM中的行号（Y方向，注意PGM行从上到下递增）
        row = map_height - int((y - map_origin_y) / resolution)
        
        # 确保栅格坐标在地图范围内
        if 0 <= col < map_width and 0 <= row < map_height:
            pgm_data[row, col] = 0  # 标记为障碍物
    
    return pgm_data


# -------------------------- 4. 自动生成YAML配置文件 --------------------------
def generate_yaml(pgm_filename, yaml_filename, resolution, origin):
    yaml_content = f"""image: {pgm_filename}
resolution: {resolution:.5f}
origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    # 写入YAML文件
    with open(yaml_filename, 'w') as f:
        f.write(yaml_content)
    print(f"YAML配置文件已生成：{yaml_filename}")


# -------------------------- 5. 保存PGM文件 --------------------------
def save_pgm(pgm_data, output_path):
    img = Image.fromarray(pgm_data, mode='L')
    img.save(output_path)
    print(f"PGM地图已保存：{output_path}")


# -------------------------- 主函数 --------------------------
if __name__ == "__main__":
    # 1. 预处理点云（滤波+投影）
    obstacle_2d = preprocess_pcd(PCD_INPUT_PATH, HEIGHT_MIN, HEIGHT_MAX)
    print(f"过滤后障碍物点数量：{len(obstacle_2d)}")
    
    # 2. 计算地图参数（确保原点(0,0)不变）
    map_origin_x, map_origin_y, map_w, map_h = calculate_map_params(obstacle_2d, RESOLUTION)
    
    # 3. 栅格化生成PGM数据
    pgm_data = rasterize_to_pgm(obstacle_2d, map_origin_x, map_origin_y, map_w, map_h, RESOLUTION)
    
    # 4. 保存PGM文件
    save_pgm(pgm_data, PGM_OUTPUT_PATH)
    
    # 5. 自动生成YAML文件
    generate_yaml(
        pgm_filename=os.path.basename(PGM_OUTPUT_PATH),
        yaml_filename="/home/komasa/HitchOpen-THICV-Stack/src/planning/race_global_static_planner/config/carla_town10_map/carla_town10_map.yaml",
        resolution=RESOLUTION,
        origin=(map_origin_x, map_origin_y)
    )