Header header

float32 motor # 目标速度, m/s, !非负数
float32 steer # 转向度数, 度, 左负右正
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
