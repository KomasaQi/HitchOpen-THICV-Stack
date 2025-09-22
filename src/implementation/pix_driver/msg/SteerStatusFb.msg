Header header

uint8 SteerEnableStatusDisable = 0
uint8 SteerEnableStatusEnable  = 1
uint8 steer_enable_status

uint8 SteerOverrunWarningNormal = 0
uint8 SteerOverrunWarningOver   = 1
uint8 steer_overrun_warning

uint8 SteerWorkModeMachine = 0
uint8 SteerWorkModeWire    = 1
uint8 SteerWorkModePower   = 2
uint8 steer_work_mode

uint8 SteerModeFrontAckerman      = 0
uint8 SteerModeSameFrontBack      = 1
uint8 SteerModeFrontDifferentBack = 2
uint8 SteerModeBackAckerman       = 3
uint8 SteerModeFrontBack          = 4
uint8 steer_mode

float32 steer_angle_front
float32 steer_angle_rear
float32 steer_angle_speed