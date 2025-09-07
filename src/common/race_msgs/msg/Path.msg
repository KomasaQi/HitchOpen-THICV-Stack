# 路径消息，与Autoware的Path消息保持一致
std_msgs/Header header

# 路径点序列
race_msgs/PathPoint[] points

# 路径是否为临时路径
bool is_avoidance_path