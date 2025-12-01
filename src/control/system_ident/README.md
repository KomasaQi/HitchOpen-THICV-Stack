# System Identification Toolbox in ROS
Komasa Qi 2025
本算法包包含了基于ROS的系统辨识工具包，包含以下功能：
 - 基于EKF的线性/非线性系统状态辨识（Python版,需要手动推导Jacobian）
 - 基于EKF的线性/非线性系统参数辨识（C++版，依靠CasADi自动微分，只需定义系统动态+观测方程）
 - 基于RLS的差分时间序列辨识（Python版、C++版）
 - 在线自相关系数/偏自相关系数/交叉相关系数计算（Python版）
 