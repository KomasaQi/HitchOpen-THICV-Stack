Header header
uint8 command
uint8 level
uint8 sound_id

uint8 CMD_PLAY = 0
uint8 CMD_CLEAR = 1
uint8 CMD_DISABLE = 2
uint8 CMD_ENABLE = 3

uint8 LEVEL_DEBUG = 0
uint8 LEVEL_INFO = 1
uint8 LEVEL_WARN = 2
uint8 LEVEL_ERROR = 3

uint8 HELLO = 0
uint8 BYE = 1
uint8 TASK_FINISH = 2
uint8 OBSTACLE_BRAKE = 3
uint8 TASK_RECEIVED = 4
uint8 TASK_START = 5
uint8 TASK_PAUSE = 6
uint8 TASK_RESUME = 7
uint8 TASK_ABORT = 8
