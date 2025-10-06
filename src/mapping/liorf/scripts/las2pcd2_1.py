import laspy
import numpy as np
import os
from tqdm import tqdm

folder_path = r'/media/komasa/Data/Ubuntu文件备份/data/las'
output_path = r'/media/komasa/Data/Ubuntu文件备份/data/pcd'

os.makedirs(output_path, exist_ok=True)

def write_pcd_binary(path, fields, dtypes, points_struct):
    def type_code(dt):
        if np.issubdtype(dt, np.floating):
            return 'F'
        elif np.issubdtype(dt, np.signedinteger):
            return 'I'
        elif np.issubdtype(dt, np.unsignedinteger):
            return 'U'
        else:
            raise ValueError(f'Unsupported dtype: {dt}')

    sizes = [np.dtype(dt).itemsize for dt in dtypes]
    types = [type_code(np.dtype(dt)) for dt in dtypes]
    counts = ['1'] * len(fields)

    header = []
    header.append('# .PCD v0.7 - Point Cloud Data file format')
    header.append('VERSION 0.7')
    header.append('FIELDS ' + ' '.join(fields))
    header.append('SIZE ' + ' '.join(str(s) for s in sizes))
    header.append('TYPE ' + ' '.join(types))
    header.append('COUNT ' + ' '.join(counts))
    header.append(f'WIDTH {points_struct.shape[0]}')
    header.append('HEIGHT 1')
    header.append('VIEWPOINT 0 0 0 1 0 0 0')
    header.append(f'POINTS {points_struct.shape[0]}')
    header.append('DATA binary\n')

    with open(path, 'wb') as f:
        f.write('\n'.join(header).encode('utf-8'))
        f.write(points_struct.tobytes())

# 需要导出的基础字段
base_fields = ['x','y','z','intensity']

las_files = [f for f in os.listdir(folder_path) if f.lower().endswith('.las')]
print(f'找到 {len(las_files)} 个 LAS 文件待转换…')

for file in tqdm(las_files):
    las_path = os.path.join(folder_path, file)
    try:
        las = laspy.read(las_path)
        dims = set(las.point_format.dimension_names)

        # 字段与dtype定义
        fields = base_fields.copy()
        dtypes = [np.float32, np.float32, np.float32, np.uint16] #float32的精度提供约6-7位十进制有效数字
        if 'intensity' not in dims:
            fields.remove('intensity')
            dtypes = [np.float32, np.float32, np.float32]


        structured_dtype = np.dtype([(name, dt) for name, dt in zip(fields, dtypes)])
        npts = las.header.point_count
        out = np.empty(npts, dtype=structured_dtype)

        out['x'] = np.asarray(las.x, dtype=np.float32)
        out['y'] = np.asarray(las.y, dtype=np.float32)
        out['z'] = np.asarray(las.z, dtype=np.float32)

        if 'intensity' in fields:
            # intensity整型
            out['intensity'] = np.asarray(las.intensity, dtype=np.uint16)

        # 写 PCD
        out_name = os.path.splitext(file)[0] + '.pcd'
        out_path = os.path.join(output_path, out_name)
        write_pcd_binary(out_path, fields, dtypes, out)

    except Exception as e:
        print(f'\n处理文件 {file} 出错：{e}')

print('\n全部转换完成。')
