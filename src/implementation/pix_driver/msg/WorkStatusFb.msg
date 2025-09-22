# 0x534
Header header

uint8 DriveModeStandby      = 0
uint8 DriveModeSelfDriving  = 1
uint8 DriveModeRemote       = 2
uint8 DriveModeMan          = 3
uint8 drive_mode

uint8 PowerStatusInit   = 0
uint8 PowerStatusOnAcc  = 1
uint8 PowerStatusReady  = 2
uint8 PowerStatusOff    = 3
uint8 power_status

uint8 DCStatusOff     = 0
uint8 DCStatusOn      = 1
uint8 DCStatusStandby = 2
uint8 dc_status

uint8 LimitStatusDefault = 0
uint8 LimitStatusLimit   = 1
uint8 limit_status

uint8 PowerLimitStatusDefault = 0
uint8 PowerLimitStatusLimit   = 1
uint8 power_limit_status

uint8 EcoStatusDefault = 0
uint8 EcoStatusEco     = 1
uint8 EcoStatusSport   = 2
uint8 eco_status

float32 chassis_speed_limit
float32 chassis_low_power_vol

# 0:no
# 1:chassis estop
# 2:remote estop
# 3:chassis err estop
# 4:acu loss estop
# 5:acu estop
# 6:crash estop
uint8 EStopStatusNo          = 0
uint8 EStopStatusChassis     = 1
uint8 EStopStatusRemote      = 2
uint8 EStopStatusErr         = 3
uint8 EStopStatusAcuLoss     = 4
uint8 EStopStatusAcuEstop    = 5
uint8 EStopStatusCrashEStop  = 6
uint8 e_stop_status

uint8 CrashStatusOff = 0
uint8 CrashStatusOn  = 1
uint8 front_crash_status
uint8 back_crash_status
uint8 left_crash_status
uint8 right_crash_status
