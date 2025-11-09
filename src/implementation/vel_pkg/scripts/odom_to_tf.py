#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

def odom_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    
    t.transform.rotation = msg.pose.pose.orientation
    
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    rospy.Subscriber("odom", nav_msgs.msg.Odometry, odom_callback)
    rospy.spin()