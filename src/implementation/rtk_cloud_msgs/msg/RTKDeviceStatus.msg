# 主要是华芯GHSTA的解析

Header header

int16 NavState  
int16 temprature  # Temp 温度
int16 PoseMode    # 定位模式
int16 HeadMode    # 航向模式
int16 StateNum    # 卫星数
float32 HDOP      # 水平位置精度
float32 HeadDop   # 航向精度

# NavState
int16 ERROR        = -1
int16 WARM         = 0
int16 TARGET       = 1
int16 PURE_IMU     = 2
int16 INS_DMI      = 5
int16 INS_GNSS     = 7
int16 INS_GNSS_DMI = 8

# PoseMode
int16 RTK_INVALIDE = 0  # 定位不可用
int16 RTK_SINGLE   = 1  # 单点定位
int16 RTK_FAKE     = 2  # 伪距差分
int16 RTK_FIX      = 4  # 位置已固定,定位最好的状态
int16 RTK_FLOAT    = 5  # 浮点解

# Headmode
int16 HEAD_INVALID = 0  # 航向不可用
int16 HEAD_FIX     = 4  # 航向可靠
int16 HEAD_FLOAT   = 5  # 航向浮动
