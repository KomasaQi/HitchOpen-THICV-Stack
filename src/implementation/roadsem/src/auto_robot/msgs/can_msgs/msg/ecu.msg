Header header

float32 motor # 目标速度, m/s, !非负数
float32 throttle # 油门开度0~1.0
float32 steer # 转向度数, 度, 左负右正
float32 steer_rear # 后端转向度数, 度, 左负右正
float32 brake # 刹车力度 0~1.0

uint8 turn_left_light  # 左转灯，1为使能，下同
uint8 turn_right_light
uint8 hazard_light
uint8 position_light
uint8 low_beam
uint8 high_beam
uint8 rear_front_light
uint8 horn

uint8 SHIFT_UNKNOWN = 0 
uint8 SHIFT_D = 1 #前进档位
uint8 SHIFT_N = 2 #停止档位
uint8 SHIFT_R = 3 #后退档位
uint8 shift # 档位

uint8 driver_mode # 速度控制模式 
uint8 SPEED_CONTROL = 0 # 期望速度控制模式
uint8 THROTTLE_CONTROL = 1 # 油门控制模式

uint8 traction_mode # 驱动力控制模式
uint8 TRACTION_4WD = 0 # 全驱动力模式
uint8 TRACTION_RWD = 1 # 后轮驱动力模式
uint8 TRACTION_FWD = 2 # 前轮驱动力模式


uint8 steer_mode # 转向模式
uint8 STEER_FRONT = 0 # 前端转向模式
uint8 STEER_CENTER = 1 # 前后方向相反大小相同中心转向模式
uint8 STEER_WEDGE = 2 # 前后方向大小相同楔形转向模式
uint8 STEER_REAR = 3 # 后端转向模式
uint8 STEER_FREE = 4 # 前后自由控制转角
