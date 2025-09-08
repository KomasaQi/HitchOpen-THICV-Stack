# 控制命令消息
std_msgs/Header header
race_msgs/Longitudinal longitudinal
race_msgs/Lateral lateral
float64 throttle # 自定义的控制内容：油门 （0~1）
float64 brake  # 自定义的控制内容：刹车 （0~1）

# 档位
int8 gear
int8 GEAR_PARK = -2
int8 GEAR_REVERSE = -1
int8 GEAR_NEUTRAL = 0
int8 GEAR_1 = 1
int8 GEAR_2 = 2
int8 GEAR_3 = 3
int8 GEAR_4 = 4
int8 GEAR_5 = 5
int8 GEAR_6 = 6

uint8 control_mode # 控制模式
uint8 THROTTLE_BRAKE_ONLY = 0
uint8 DES_SPEED_ONLY = 1
uint8 DES_ACCEL_ONLY = 2
bool emergency # 紧急制动标志
bool hand_brake # 手刹标志
bool clutch # 离合标志

# 自定义的内容：是否双轴转向
uint8 steering_mode
uint8 FRONT_STEERING_MODE = 1
uint8 DUAL_STEERING_MODE = 2



