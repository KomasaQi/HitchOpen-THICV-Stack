int8 type  # 是直接给定坐标点,还是给定朝向+距离
float32 target_x  # m
float32 target_y
int8 heading
float32 distance  # m

# 信息类型
int8 TYPE_POSE = 0
int8 TYPE_HEADING = 1

# 朝向
int8 HEADING_FRONT = 0 
int8 HEADING_BACK = 1