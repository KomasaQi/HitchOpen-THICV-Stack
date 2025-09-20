Header header

uint8 DriveEnableStatusDisable = 0
uint8 DriveEnableStatusEnable  = 1
uint8 drive_enable_status

uint8 DriveOverrunWarningNormal = 0
uint8 DriveOverrunWarningOver   = 1
uint8 drive_overrun_warning

uint8 DriveModeSpeedCtrl    = 0
uint8 DriveModeThrottleCtrl = 1
uint8 DriveModeReserve1     = 2
uint8 DriveModeReserve2     = 3
uint8 drive_mode

uint8 GearStatusNoUse = 0
uint8 GearStatusD     = 1
uint8 GearStatusN     = 2
uint8 GearStatusR     = 3
uint8 gear_status

uint8 Drive4WdMode4WD = 0
uint8 Drive4WdModeRwd = 1
uint8 Drive4WdModeFwd = 2
uint8 drive_4wd_mode

float32 speed
float32 throttle
float32 acceleration