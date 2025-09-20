# 0x701: Cluster一般信息, 包含cluster的位置和速度,并针对所有检测到的cluster重复发送,先近后远

int8 Cluster_ID
float32 Cluster_DistLong # 纵坐标(x) m
float32 Cluster_DistLat  # 横坐标(y) m
float32 Cluster_VrelLong # 纵向相对速度(x) m/s
int8 Cluster_DynProp  # 动态属性,移动状态
float32 Cluster_VrelLat # 横向相对速度(y) m/s
float32 Cluster_RCS  # 雷达散射截面积

int8 CLUSTER_DYNPROP_MOVE         = 0  # 移动的
int8 CLUSTER_DYNPROP_STAY         = 1  # 静止的
int8 CLUSTER_DYNPROP_COME         = 2  # 来向的
int8 CLUSTER_DYNPROP_MAY_STAY     = 3  # 可能静止的
int8 CLUSTER_DYNPROP_UNKNOWN      = 4  # 未知的
int8 CLUSTER_DYNPROP_THROUGH_STAY = 5  # 横穿静止的
int8 CLUSTER_DYNPROP_THROUGH      = 6  # 横穿的
int8 CLUSTER_DYNPROP_STOP         = 7  # 停止的
