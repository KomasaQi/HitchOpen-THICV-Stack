#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
import math


# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
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
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """

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
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)


class LaserScanToDirectCorrectNode:
    def __init__(self):
        rospy.init_node('laser_scan_to_direct_correct_node')
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.tf_subscriber = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.direct_correct_publisher = rospy.Publisher('/direct_correct', Float64, queue_size=10)
        self.tf_data = None
        self.transform=None
        self.heading=0.0
        self.count=0
        self.mbar=0.0
        self.msum=0.0

    def tf_callback(self,msg):
        global heading
        # 获取 base_footprint 到 map 的变换关系
        for transform in msg.transforms:
            if transform.header.frame_id =="map":
                if transform.child_frame_id == "odom":
                    quaternion=[
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w]
                    (roll,pitch,yaw)=euler_from_quaternion(quaternion)
                    self.heading = yaw

    def scan_callback(self, msg):
        

        try:
            startidx = 500
            endidx   = startidx+1000
            # 提取指定范围内的激光点
            selected_ranges = np.array(msg.ranges[startidx:endidx])
            # rospy.loginfo("来啦")
            # # 转换激光点到laser坐标系下的x,y表示
            angles_all = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            angles = angles_all[startidx:endidx]+self.heading
            angles = angles[np.isfinite(selected_ranges)]
            selected_ranges=selected_ranges[np.isfinite(selected_ranges)]
            x_values = (selected_ranges * np.cos(angles))
            y_values = (selected_ranges * np.sin(angles))
            

            # # # 使用最小二乘法进行直线拟合
            A = np.vstack([x_values, np.ones(len(x_values))]).T
            m, c = np.linalg.lstsq(A, y_values, rcond=1)[0]
            # print(A.shape)
            # # # 发布直线的斜率
            self.direct_correct_publisher.publish(m)
            alpha=0.9
            self.msum+=m
            self.count+=1
            self.mbar=self.mbar*alpha+(1-alpha)*m
            rospy.loginfo("直线斜率为：{:.4f},均值为：{:.5f}".format(m,self.mbar))

        except Exception as e:
            rospy.logwarn("Failed to process laser scan data: {}".format(e))


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_to_direct_correct_node = LaserScanToDirectCorrectNode()
        laser_scan_to_direct_correct_node.run()
    except rospy.ROSInterruptException:
        pass
