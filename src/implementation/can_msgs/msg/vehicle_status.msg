Header Header

float32 cur_speed  # 当前速度(m/s)
float32 cur_steer # 当前转向角度(°)

float32 left_wheel_speed
float32 right_wheel_speed


uint8 shift_level # 当前档位 D1 N2 R3
bool  is_brake
uint8 ignition_status # 0:sleep 1:pre-on 2:on
uint8 wheel_direction # 车轮转向方向：0左 1右
uint8 drive_mode # 驾驶模式
float32 total_odometer 
uint8 acc_level # deprecated: 加速度档位
uint8 brake_level # deprecated: 减速度档位

bool turn_left_light
bool turn_right_light
bool hazard_light
bool position_light
bool low_beam
bool high_beam
bool rear_front_light
bool horn
