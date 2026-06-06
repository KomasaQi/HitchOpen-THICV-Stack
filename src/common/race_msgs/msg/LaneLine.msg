# 单条车道线/中心线消息（自车坐标系，FLU：x前 y左 z上）
# 几何以横向偏移 y 关于纵向距离 x 的多项式表示：
#   y(x) = c[0] + c[1]*x + c[2]*x^2 + ... + c[order]*x^order
# 多项式在 [x_start, x_end] 区间内有效，单位均为米/弧度。

# 车道线类别
uint8 type
uint8 TYPE_LANE_LINE = 0    # 车道边界线（实线/虚线，分隔相邻车道）
uint8 TYPE_CENTER_LINE = 1  # 车道中心线（可行驶轨迹）

# 车道线在横向上的整数索引（相对自车所在车道）
# 边界线 lane_id 约定：0=自车道左边线，1=自车道右边线，
#                       负数向左递减(-1为左侧相邻车道左边线)，正数向右递增。
# 中心线 lane_id 约定：0=自车道中心线，负数向左(-1为左侧相邻车道)，正数向右。
int32 lane_id

# 多项式系数，索引由低次到高次：coeffs[0]为常数项。
# 长度 = order + 1。默认保留到5阶(6个系数)，每个系数保留6位小数。
float64[] coeffs

# 实际拟合阶数（默认3，最多5）
uint8 order

# 多项式有效的纵向范围（自车坐标系前向x，单位：米）
float64 x_start
float64 x_end

# 拟合质量与可信度
float64 fit_rmse      # 拟合残差均方根 (m)，越小越好
float64 confidence    # 综合置信度 0~1
int32 num_points      # 参与拟合的有效采样点数

# 该车道线对应的离散采样点（自车坐标系，可选，供调试/可视化使用）
# 与多项式表示冗余，下游可任选其一。
geometry_msgs/Point[] points
