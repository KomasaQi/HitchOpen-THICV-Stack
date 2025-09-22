# Race Messages for HitchOpen THICV Stack
This directory contains self defined message definitions used in the HitchOpen THICV Stack.

## Message Definitions
- [Control.msg](msg/Control.msg)
- [Path.msg](msg/Path.msg)
- [PathPoint.msg](msg/PathPoint.msg)
- [Lateral.msg](msg/Lateral.msg)
- [Longitudinal.msg](msg/Longitudinal.msg)
- [VehicleStatus.msg](msg/VehicleStatus.msg)
- [WheelSpeed.msg](msg/WheelSpeed.msg)
- [Euler.msg](msg/Euler.msg)


# Specific Message Definitions

## Control.msg
``` bash
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
```

## Path.msg
``` bash
# 路径消息，与Autoware的Path消息保持一致
std_msgs/Header header

# 路径点序列
race_msgs/PathPoint[] points
```

## PathPoint.msg
``` bash
# 路径点消息
geometry_msgs/Pose pose
# 该点的目标线速度 (m/s)
float64 velocity
# 该点的曲率 (1/m)
float64 curvature
```

## Lateral.msg
``` bash
# 目标转向角 (rad)
float64 steering_angle
# 目标转向角速度 (rad/s)
float64 steering_angle_velocity

# 自定义的内容：后轮转角 （rad）
float64 rear_wheel_angle
# 自定义的内容：后轮转角速度 (rad/s)
float64 rear_wheel_angle_velocity
```

## Longitudinal.msg
``` bash
# 目标线速度 (m/s)
float64 velocity
# 目标加速度 (m/s^2)
float64 acceleration
# 目标 jerk (m/s^3)
float64 jerk
```

## VehicleStatus.msg
``` bash
# 车辆状态消息
std_msgs/Header header # 时间戳和所处坐标系
string child_frame_id # 车辆自车坐标系名称
geometry_msgs/Pose pose # 车辆位置和姿态 包含position(xyz)和orientation(xyzw)
race_msgs/Euler euler # 车辆姿态 包含俯仰角、横滚角、偏航角
geometry_msgs/Twist vel # 平移旋转速度 包含速度、角速度
geometry_msgs/Twist acc # 平移旋转加速度 包含加速度、角加速度
race_msgs/Lateral lateral # 转向情况：包含前轮转角、角速度 后轮转角、角速度
race_msgs/WheelSpeed wheel_speed # 包含四个车轮的角速度 rad/s
int8 gear # 档位信息 -2：驻车 -1：倒车 0：空挡 1~6：前进档 参见Control.msg
uint8 control_mode # 控制模式，0：油门刹车模式，1：速度模式，2：加速度模式 参见Control.msg
bool hand_brake # 手刹状态
bool emergency # 是否紧急状态
bool clutch # 离合器状态
uint8 steering_mode # 转向模式，1：单轴转向，2：双轴转向 参见Control.msg
race_msgs/Tracking tracking # 跟踪信息
```

## WheelSpeed.msg
``` bash
float32 left_front # 单位：rad/s
float32 left_rear # 单位：rad/s
float32 right_front # 单位：rad/s
float32 right_rear # 单位：rad/s
```

## Euler.msg
``` bash
float64 roll # 横滚角
float64 pitch # 俯仰角
float64 yaw # 偏航角
```

## Flag.msg
``` bash
uint8 flag       # 比赛状态标志位
uint8 RED = 0    # 红色旗帜：紧急停止
uint8 GREEN = 1  # 绿色旗帜：正常运行且没有限速
uint8 BLACK = 2  # 黑色旗帜：比赛完成，紧急停止
uint8 G5 = 5     # 绿色旗帜且限速5km/h
uint8 G10 = 10   # 绿色旗帜且限速10km/h
uint8 G15 = 15   # 绿色旗帜且限速15km/h
uint8 G20 = 20   # 绿色旗帜且限速20km/h
uint8 G40 = 40   # 绿色旗帜且限速40km/h
uint8 G60 = 60   # 绿色旗帜且限速60km/h 
uint8 G80 = 80   # 绿色旗帜且限速80km/h
```

## Tracking.msg
``` bash
float64 lateral_tracking_error # 横向跟踪误差, m
float64 heading_angle_error # 航向角跟踪误差, rad
```