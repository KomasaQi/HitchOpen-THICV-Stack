#!/usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import String
import csv
import datetime



# 获取当前日期和时间并格式化为特定字符串形式
current_datetime = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

# 构建文件名
csv_file = f'/home/jetson/catkin_ws/src/tracking_pkg/exp_data/truck_exp_data_{current_datetime}.csv'

fieldnames=['time_stamp', 'x_pos','y_pos','heading',
        'vel_true','delta_cmd','acc_cmd','x_acc','y_acc','z_acc','droll','roll',
        'x2_acc','y2_acc','z2_acc','droll2','roll2']


def logger_callback(msg):
        global fieldnames,writer
        data_str = msg.data

        row = dict(zip(fieldnames, data_str.split(',')))
        writer.writerow(row)
        print('一条数据成功记录！')
        # print('log_msg: ' + data_str)


sub = rospy.Subscriber('/log_msg',String,logger_callback,queue_size=1)

rospy.init_node('truck_data_logger_node', anonymous=True)
rospy.loginfo("本车数据记录节点启动啦~")


# 写入CSV文件
with open(csv_file, 'w', newline='') as file:
    writer = csv.DictWriter(file,fieldnames=fieldnames)
    writer.writeheader()  # 写入字段名

    while not rospy.is_shutdown():

        
        # loop_rate.sleep()
        rospy.spin()





        
print('数据已保存到CSV文件:', csv_file)