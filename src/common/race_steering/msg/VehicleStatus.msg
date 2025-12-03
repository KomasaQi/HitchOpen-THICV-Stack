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
float32 throttle_fb # 油门值 0~1
float32 brake_fb # 刹车值 0~1
race_msgs/TrailerStatus trailer # 挂车状态
race_msgs/LateralLoadTransfer ltr_state # 侧向载荷转移率 包含载荷转移率和变化率




