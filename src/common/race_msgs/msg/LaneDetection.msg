# 车道线检测综合结果消息
# 所有几何量均已从图像逆透视变换(IPM)到自车坐标系(FLU：x前 y左 z上)。
std_msgs/Header header        # 时间戳与坐标系(child_frame_id所指自车坐标系)
string child_frame_id         # 自车坐标系名称，如 "ego_vehicle"

# 所有检测到的车道边界线（含相邻车道），按从左到右排序
race_msgs/LaneLine[] lane_lines

# 所有车道中心线（含相邻车道），按从左到右排序
# 自车道中心线即下游可直接跟踪的局部参考轨迹
race_msgs/LaneLine[] center_lines

# 自车所在车道的中心线在 center_lines 数组中的索引（-1表示未知）
int32 ego_center_index

# 换道可行性快速判据（由几何与检测置信度综合给出，供上层决策参考）
bool left_lane_available      # 左侧是否存在可换入的相邻车道
bool right_lane_available     # 右侧是否存在可换入的相邻车道

# 自车相对自车道中心线的横向偏移(m，左正)与航向偏差(rad，左正)
float64 ego_lateral_offset
float64 ego_heading_error

# 检测元信息
float64 detect_latency_ms     # 单帧检测耗时(ms)
uint8 source                  # 几何来源
uint8 SOURCE_LEARNING = 0     # 学习模型(UFLDv2等)
uint8 SOURCE_CLASSIC = 1      # 传统规则(Canny+IPM+滑窗)
uint8 SOURCE_FUSED = 2        # 融合结果
