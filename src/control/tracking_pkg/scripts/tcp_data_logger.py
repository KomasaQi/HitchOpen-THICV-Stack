# -*- coding: UTF-8 -*-
import socket
import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Float32


running_flag = True
desample_count = 0


def logger_callback(msg):
    global connect,desample_count,running_flag
    if desample_count<1:
        desample_count+=1
        return
    else:
        desample_count=0
    # time_stamp = rospy.Time.now()
    # 发送数据给客户端
    connect.sendall((str(msg.data)).encode())
    pc_cmd = connect.recv(1024)
    cmd_msg = pc_cmd.decode()
    print(cmd_msg)
    if cmd_msg == 'quit':
        running_flag = False
    
    

# 创建一个socket套接字，该套接字还没有建立连接
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 绑定监听端口，这里必须填本机的IP192.168.9.30，localhost和127.0.0.1是本机之间的进程通信使用的
server.bind(('192.168.43.18', 9007))
# 开始监听，并设置最大连接数
server.listen(5)

print('waiting for connect...')
# 等待连接，一旦有客户端连接后，返回一个建立了连接后的套接字和连接的客户端的IP和端口元组
connect, (host, port) = server.accept()
print("the client {:s}:{:d} has connected.".format(host, port))

sub = rospy.Subscriber('/log_msg',String,logger_callback,queue_size=1)

rospy.init_node('tcp_data_logger_node', anonymous=True)
# loop_rate = rospy.Rate(20) #Hz

while not rospy.is_shutdown() and running_flag:
    # 接受客户端的数据

    rospy.loginfo("数据tcp发送节点启动啦~")
    
    # loop_rate.sleep()
    rospy.spin()



connect.sendall(('quit').encode())


# 结束socket
server.close()

