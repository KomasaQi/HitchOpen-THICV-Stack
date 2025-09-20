Header header

uint8 main_state
uint8 acc_state
uint8 behavior_state
string str_state

# main state
uint8 RUN = 0
uint8 PAUSE = 1
uint8 STOP = 2
# stop分为两种,或者任务结束,或者车辆故障