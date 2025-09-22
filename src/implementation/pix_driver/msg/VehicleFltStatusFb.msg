Header header

uint8 OverTempStatusNormal   = 0
uint8 OverTempStatusOverTemp = 1
uint8 motor_over_temp_status
uint8 bms_over_temp_status
uint8 brake_over_temp_status
uint8 steer_over_temp_status
uint8 under_volt_status

uint8 FaultLevelNormal = 0
uint8 FaultLevel1      = 1
uint8 FaultLevel2      = 2
uint8 FaultLevel3      = 3
uint8 FaultLevel4      = 4
uint8 sys_fault
uint8 brake_fault
uint8 parking_fault
uint8 steer_front_fault
uint8 steer_back_fault
uint8 motor_lf_fault
uint8 motor_rf_fault
uint8 motor_lr_fault
uint8 motor_rr_fault
uint8 bms_fault
uint8 dc_fault
uint8 fault_reserve1