#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LTR参数EKF辨识节点：适配race_msgs/VehicleStatus消息
订阅话题：/race/vehicle_state
输出：在线辨识a1/a2/b1参数并打印
"""
import numpy as np
import rospy
# 修正：导入正确的消息类型（VehicleStatus，而非VehicleStatusMsg）
from race_msgs.msg import VehicleStatus
from geometry_msgs.msg import Twist

# -------------------------- 1. EKF配置参数 --------------------------
# Ts 采样步长是根据实际收到的消息频率动态调整的
# 过程噪声协方差Q（5×5：LTR, dLTR, a1, a2, b1）
Q = np.diag([1e-4, 1e-4, 1e-6, 1e-6, 1e-6])  
# 观测噪声协方差R（2×2：LTR测量噪声，dLTR测量噪声）
R = np.diag([1e-4, 1e-6])  
# 初始状态：[LTR, dLTR, a1, a2, b1]（前两维将用首次测量值初始化）
X_est = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  
# 初始协方差矩阵（参数不确定性高于状态）
P_est = np.diag([1e-3, 1e-3, 1.0, 1.0, 1.0])  
last_sample_time = None  # 上次采样时间（用于计算Ts）

# -------------------------- 2. EKF核心函数 --------------------------
def ekf_ltr_identify(X_est, P_est, Z_k, u_k, Ts, Q, R):
    x1, x2, theta1, theta2, theta3 = X_est
    # 预测步
    f = np.array([x2, theta1*x1 + theta2*x2 + theta3*u_k, 0.0, 0.0, 0.0])
    X_pred = X_est + Ts * f
    # 雅克比矩阵F
    F = np.eye(5) + Ts * np.array([
        [0, 1, 0, 0, 0],
        [theta1, theta2, x1, x2, u_k],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ])
    P_pred = F @ P_est @ F.T + Q

    # 更新步
    H = np.array([[1,0,0,0,0], [0,1,0,0,0]])  # 观测矩阵
    H_P_Ht = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(H_P_Ht)
    h_Xpred = np.array([X_pred[0], X_pred[1]])
    residual = Z_k - h_Xpred
    X_new = X_pred + K @ residual
    P_new = (np.eye(5) - K @ H) @ P_pred

    return X_new, P_new

# -------------------------- 3. ROS消息回调函数 --------------------------
is_first_callback = True  # 首次回调标志（避免全局变量声明冗余）
def vehicle_state_callback(msg):
    global X_est, P_est, is_first_callback, last_sample_time

    # 1. 提取观测值 Z_k = [LTR, dLTR]
    ltr = msg.ltr_state.ltr          # 侧向载荷转移率
    ltr_rate = msg.ltr_state.ltr_rate  # LTR变化率（dLTR）
    Z_k = np.array([ltr, ltr_rate])

    # 2. 提取输入 u_k = vx * dtheta（侧向加速度简化值）
    vx = msg.vel.linear.x            # 车辆纵向速度（m/s）
    dtheta = msg.vel.angular.z       # 偏航角速度（rad/s）
    u_k = vx * dtheta

    # 3. 数据有效性校验
    if np.isnan(Z_k).any() or np.isnan(u_k):
        rospy.logwarn("[EKF] 收到NaN数据，跳过本次更新")
        return
    if abs(vx) > 140:  # 合理车速阈值（根据你的车辆调整，如最大50m/s=180km/h）
        rospy.logwarn(f"[EKF] 车速异常：vx={vx:.2f}m/s，跳过本次更新")
        return

    # 4. 首次回调：用真实测量值初始化EKF状态前两维
    if is_first_callback:
        X_est[0] = ltr
        X_est[1] = ltr_rate
        is_first_callback = False
        last_sample_time = msg.header.stamp.to_sec()  # 记录首次时间
        rospy.loginfo("[EKF] 初始化完成，开始参数辨识！")
        return

    # 5. 调用EKF更新参数
    # 计算当前采样步长Ts（根据实际时间间隔动态调整）
    current_time = msg.header.stamp.to_sec()
    Ts = current_time - last_sample_time
    last_sample_time = current_time  # 更新上次采样时间
    X_est, P_est = ekf_ltr_identify(X_est, P_est, Z_k, u_k, Ts, Q, R)

    # 6. 提取并打印结果（优化格式，减少冗余输出）
    a1, a2, b1 = X_est[2], X_est[3], X_est[4]

    rospy.loginfo(
        f"[EKF辨识结果] 时间戳: {msg.header.stamp.to_sec():.3f} | "
        f"a1={a1:.6f} | a2={a2:.6f} | b1={b1:.6f} | "
        f"LTR={ltr:.4f} | dLTR={ltr_rate:.4f} " 
    )

# -------------------------- 4. 主函数 --------------------------
if __name__ == "__main__":
    # 初始化ROS节点（匿名模式避免节点名冲突）
    rospy.init_node("ltr_param_ekf_identify", anonymous=True)

    # 订阅话题：/race/vehicle_state，消息类型race_msgs/VehicleStatus
    rospy.Subscriber(
        name="/race/vehicle_state",
        data_class=VehicleStatus,
        callback=vehicle_state_callback,
        queue_size=20  # 增大队列，避免高频消息丢失
    )

    # 打印启动信息
    rospy.loginfo("="*50)
    rospy.loginfo("LTR参数EKF辨识节点已启动")
    rospy.loginfo(f"订阅话题：/race/vehicle_state")
    rospy.loginfo(f"消息类型：race_msgs/VehicleStatus")
    rospy.loginfo("等待车辆状态消息...")
    rospy.loginfo("="*50)

    # 保持节点运行
    rospy.spin()