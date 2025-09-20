int16 err_code
string err_msg

int16 ERROR_UNDEFINED = 0           # 未定义的错误
int16 ERROR_SUBWAY_UNFOUND = 1      # 指定的地铁数据未找到
int16 ERROR_MOTOR_UNFOUND = 2       # 指定的车厢数据未找到
int16 ERROR_MOTOR_MAP_UNFOUND = 3   # 车厢地图数据未找到
int16 ERROR_MOTOR_PATH_UNFOUND = 4  # 车厢行驶轨迹数据未找到
int16 ERROR_TARGET_POSE_INVALID = 5 # 目标位置无效
int16 ERROR_VERSION_UNMATCH = 6     # 协议版本不匹配
int16 ERROR_VIN_UNMATCH = 7         # 车辆VIN不匹配
int16 ERROR_TYPE_INVALID = 8        # 指令类型非法, 未定义该指令type类型
int16 ERROR_VEHICLE_UNFUNCTION = 9  # 小车底盘故障
int16 ERROR_OBSTACLE = 10            # 遇障停车