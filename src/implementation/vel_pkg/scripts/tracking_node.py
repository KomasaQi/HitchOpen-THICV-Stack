#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import math
from std_msgs.msg import Float32
from PIDController import PIDController as PID
import time
import numpy as np
from MatlabFunctions import interp1
from tf2_msgs.msg import TFMessage
import threading

x_pos = 0.0
y_pos = 0.0
heading = 0.0
ref_head = 0.0  # rad
ref_vel = 0.8   # m/s
pose1_x = 0.0
pose1_y = 0.0
vel_true = 0.0  # m/s 
WHEEL_RADIUS=0.267 #轮子周长

_NEXT_AXIS = [1, 2, 0, 1]
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}
_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)

def euler_from_matrix(matrix, axes='sxyz'):
    firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]
    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy,       M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0
    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
    return euler_from_matrix(quaternion_matrix(quaternion), axes)

def tf_callback(msg):
    global x_pos, y_pos, heading
    for transform in msg.transforms:
        if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
            x_pos = transform.transform.translation.x
            y_pos = transform.transform.translation.y
            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w]
            (_, _, yaw) = euler_from_quaternion(quaternion)
            heading = yaw

def path_callback(path):
    global ref_head, pose1_x, pose1_y
    
    # 确保路径中有足够的点
    if len(path.poses) < 4:
        rospy.logwarn("路径点数量不足: %d", len(path.poses))
        return
    
    # 使用前瞻点来计算参考航向
    lookahead_index = min(10, len(path.poses) - 1)  # 使用第10个点作为前瞻点
    
    pose1_x = path.poses[0].pose.position.x
    pose1_y = path.poses[0].pose.position.y
    pose2_x = path.poses[lookahead_index].pose.position.x
    pose2_y = path.poses[lookahead_index].pose.position.y
    
    ref_head = np.arctan2(pose2_y-pose1_y, pose2_x-pose1_x)
    
    rospy.loginfo("使用路径点: 0 (%.2f, %.2f) 和 %d (%.2f, %.2f) 计算参考航向: %.2f°",
                  pose1_x, pose1_y, lookahead_index, pose2_x, pose2_y, ref_head*180/math.pi)

def calculate_yaw_error(yaw, yaw_ref):
    (thref_x, thref_y) = rotation(np.cos(yaw_ref), np.sin(yaw_ref), -yaw)
    return np.arctan2(-thref_y, thref_x)

def rotation(x0, y0, theta):
    x = x0 * math.cos(theta) - y0 * math.sin(theta)
    y = x0 * math.sin(theta) + y0 * math.cos(theta)
    return x, y

def Steering_and_Speed_Ctrl(x_pos, y_pos):
    global ref_head, heading, pose1_x, pose1_y, vel_true, ref_vel
    lat_err = -(x_pos-pose1_x)*np.sin(ref_head) + (y_pos-pose1_y)*np.cos(ref_head)
    head_err = calculate_yaw_error(heading, ref_head) * 0.5
    global pid_controller, vel_pub
    vel_cmd = Twist()
    vel_cmd.linear.x = ref_vel  # 直接使用参考速度
    angular_z_cmd = pid_controller.update(0, lat_err + head_err)
    vel_cmd.angular.z = round(angular_z_cmd, 2)
    vel_pub.publish(vel_cmd)
    rospy.loginfo("转向角控制= %.2f°, 油门控制= %.2f", vel_cmd.angular.z, vel_cmd.linear.x)
    rospy.logwarn("横向误差= %.3f m", lat_err)
    rospy.logwarn("航向角误差= %.3f m", head_err)
    rospy.loginfo("朝向 %.2f, 轨迹朝向 %.2f, 当前速度 %.2f, 目标速度 %.2f",
                  heading*180/math.pi, ref_head*180/math.pi, vel_true, ref_vel)

def rate_callback(msg):
    global vel_true
    vel_true = (msg.data/360)*WHEEL_RADIUS * 0.4

def control_loop():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        Steering_and_Speed_Ctrl(x_pos, y_pos)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tracking_node")
    path_sub = rospy.Subscriber("/ref_path", Path, path_callback, queue_size=1)
    rate_sub = rospy.Subscriber("/wheel_rate", Float32, rate_callback, queue_size=10, tcp_nodelay=True)
    tf_sub = rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=1)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    delta_limit = 30
    pid_controller = PID(kp=0.44, ki=0.009, kd=0.14,
                         output_min=-delta_limit, output_max=delta_limit,
                         integral_min=-0.2*delta_limit, integral_max=0.2*delta_limit)
    # 移除油门PID控制器
    # 独立线程跑控制循环
    threading.Thread(target=control_loop, daemon=True).start()
    rospy.spin()