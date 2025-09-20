import numpy as np

import math

# # 给定曲线段的数据
# x1 = np.linspace(0, 2*np.pi, 100)
# y1 = np.sin(x1)

# x2 = np.linspace(0, 2*np.pi, 100)
# y2 = np.cos(x2)


def Concatenate2Curve(x1,y1,x2,y2,rotation_deg=0):
    rotation_angle = rotation_deg*math.pi/180
    # 构建旋转矩阵
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                                [np.sin(rotation_angle), np.cos(rotation_angle)]])

    # 将第二段曲线进行旋转
    curve2_rotated = np.dot(np.column_stack((x2-x2[0], y2-y2[0])), rotation_matrix.T)

    # 提取旋转后的曲线数据
    x2_rotated = curve2_rotated[:, 0]
    y2_rotated = curve2_rotated[:, 1]

    # 将曲线段的数据连接成完整曲线
    x = np.concatenate((x1, x2_rotated[1:]+x1[-1]))
    y = np.concatenate((y1, y2_rotated[1:]+y1[-1]))
    return x,y

# # 绘制完整曲线
# plt.plot(x, y)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Concatenated Curve')
# plt.show()


def arc_Gen(radius, start_deg, end_deg, point_interval):
    # 计算圆弧长度
    length = (abs(end_deg - start_deg)/180*math.pi)*radius
    num_points = round(length/point_interval)+1
    if num_points < 20:
        num_points = 20
    # 生成等间距的角度值
    angles = np.linspace(start_deg/180*math.pi, end_deg/180*math.pi, num_points)

    # 根据极坐标转换为直角坐标
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)

    return x, y


def ConcatenateNCurves(curves, rotations):
    # 基本情况：只有一条曲线时直接返回
    if len(curves) == 1:
        return curves[0]

    # 取出第一条曲线作为初始曲线
    x = curves[0][0]
    y = curves[0][1]

    # 递归地连接剩余的曲线
    for i in range(1, len(curves)):
        # 取出当前曲线和对应的旋转角度
        curve = curves[i]
        rotation_deg = rotations[i-1]

        # 调用Concatenate2Curve函数进行连接和旋转
        x, y = Concatenate2Curve(x, y, curve[0], curve[1], rotation_deg)

    return x, y

def DLC_Gen(point_interval=0.1, traj_length=100, lateral_distance=0.5, plot=False, rotation_angle=0,dlc_start=3,dlc_end=6,sharpness=5.5):
    # t = np.linspace(0, 2 * np.pi, discrete_degree)  # 参数化曲线的参数
    # 参考双移线轨迹生成
    # 参数赋值
    shape=sharpness  #%整体转向剧烈程度
    dx1=5.0   #%始程平缓程度，越大越平缓
    dx2=5.0   #%回程平缓程度，越大越平缓
    dy1=lateral_distance    #%控制换道开始y向位置
    dy2=lateral_distance    #%控制换道结束y向位置
    Xs1=dlc_start  #%控制换道开始距离
    Xs2=dlc_end #%控制换道结束距离
    x=np.linspace(0,traj_length,round(traj_length/point_interval)+1); #%点的数量根据纵向速度x_dot决定

    # 生成参考轨迹
    z1=shape/dx1*(x-Xs1)-shape/2
    z2=shape/dx2*(x-Xs2)-shape/2
    y=dy1/2.*(1+np.tanh(z1))-dy2/2.*(1+np.tanh(z2))
    # heading=np.atan(dy1*(1./np.cosh(z1))**2*(1.2/dx1)-dy2*(1./np.cosh(z2))**2*(1.2/dx2))

    # 对轨迹进行旋转
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                               [np.sin(rotation_angle), np.cos(rotation_angle)]])
    rotated_points = np.dot(rotation_matrix, np.vstack((x, y)))

    x_rotated = rotated_points[0]
    y_rotated = rotated_points[1]

    # if plot:
    #     plt.plot(x_rotated, y_rotated)
    #     plt.axis('equal')
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('Double Lane Change Trajectory')
    #     plt.show()

    return x_rotated, y_rotated


def line_Gen(point_interval,length):
    num_points = round(length/point_interval)+1
    x = np.linspace(0, length, num_points)
    y = np.zeros(num_points)
    return x,y



def Test_Circle_Gen(dlc_len,dlc_start,dlc_end,dlc_with,left_corner_rad,side_width,rot_angle,line_len,sharpness,point_interval=0.1):
    #***************生成DLC轨迹***********************************
    # point_interval = 0.1     # 散点轨迹离散程度m
    traj_length = dlc_len         # 双移线的长度m
    lateral_distance = dlc_with  # 横向移动距离m
    rotation_angle = 0       # 旋转角度deg
    
    x1, y1 = DLC_Gen(point_interval, traj_length, lateral_distance, plot=False, 
                     rotation_angle=rotation_angle/180*math.pi,dlc_start=dlc_start,dlc_end=dlc_end,sharpness=sharpness)



    #**************生成左转轨迹************************************
    # 指定圆弧参数
    radius = left_corner_rad  # 半径
    start_angle = 0.0  # 起始角度deg
    end_angle =   90  # 结束角度deg

    # 生成圆弧轨迹
    x2, y2 = arc_Gen(radius, start_angle, end_angle, point_interval)


    #**************生成直线轨迹*************************************
    x3, y3 = line_Gen(point_interval,line_len)

    #**************生成圆弧轨迹************************************
    # 指定圆弧参数
    radius = side_width-line_len-left_corner_rad  # 半径
    if radius < 2:
        radius = 2  # 避免不小心给宽度太小了结果这个转弯特别小
    start_angle = 0.0  # 起始角度deg
    end_angle =   90  # 结束角度deg
    # 生成圆弧轨迹
    x4, y4 = arc_Gen(radius, start_angle, end_angle, point_interval)

    #**************生成直线轨迹*************************************
    x5, y5 = line_Gen(point_interval,dlc_len+left_corner_rad-radius)


    #**************生成圆弧轨迹************************************
    # 指定圆弧参数
    radius = side_width/2  # 半径
    start_angle = 0.0  # 起始角度deg
    end_angle =   180  # 结束角度deg
    # 生成圆弧轨迹
    x6, y6 = arc_Gen(radius, start_angle, end_angle, point_interval)


    # print("X:", x)
    # print("Y:", y)

    # 定义两条曲线和对应的旋转角度
    curve1 = (x1, y1)
    curve2 = (x2, y2)
    curve3 = (x3, y3)
    curve4 = (x4, y4)
    curve5 = (x5, y5)
    curve6 = (x6, y6)
    rotations = np.array([-90, 90, 0,180,90])

    # 调用ConcatenateNCurves函数连接曲线
    x0, y0 = ConcatenateNCurves([curve1, curve2, curve3,curve4,curve5,curve6], rotations)
    
    rotation_angle = rot_angle/180*math.pi
    # 对轨迹进行旋转
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                               [np.sin(rotation_angle), np.cos(rotation_angle)]])
    rotated_points = np.dot(rotation_matrix, np.vstack((x0, y0)))

    x = rotated_points[0]
    y = rotated_points[1]
    return x,y


# dlc_len = 15        # m 双移线轨迹总长
# dlc_start = 3       # m 双移线起始的距离
# dlc_end = 6         # m 双移线结束的距离
# dlc_with = 0.5      # m 双移线换道的宽度
# left_corner_rad = 2 # m 左转弯半径
# line_len = 2        # m 左转弯后接的直线长度
# side_width = 9      # m 轨迹去和回的间距
# rot_angle = 15      # deg 测试轨迹的旋转角度


# x, y = Test_Circle_Gen(dlc_len,dlc_start,dlc_end,dlc_with,left_corner_rad,side_width,rot_angle,line_len)
# # 绘制合并后的曲线
# plt.plot(x, y)
# # 设置坐标轴比例尺相等
# plt.axis('equal')
# plt.show()