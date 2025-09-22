int8 cmd

int16 CMD_RUN   = 1  # 解除刹车
int16 CMD_PAUSE = 2  # 刹车
int16 CMD_ABORT_TARGET  = 3  # 停止当前任务, 将清楚当前的target目标信息
int16 CMD_RESET_SUBWAY = 4  # 重置subway_info, 用在更换车厢时