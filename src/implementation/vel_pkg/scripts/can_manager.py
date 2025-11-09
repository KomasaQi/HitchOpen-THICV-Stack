#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import MCP2515
from can_msgs.msg import Frame

class CANManager:
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(CANManager, cls).__new__(cls)
                cls._instance._init_can()
            return cls._instance
            
    def _init_can(self):
        self.can = MCP2515.MCP2515()
        self.can.Init()
        self.tx_queue = queue.Queue(maxsize=20)
        self.rx_publisher = rospy.Publisher('/can_rx', Frame, queue_size=50)
        self.rx_thread = threading.Thread(target=self._rx_worker)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        self.tx_thread = threading.Thread(target=self._tx_worker)
        self.tx_thread.daemon = True
        self.tx_thread.start()
        
    def _rx_worker(self):
        """接收线程：处理所有CAN消息并发布到ROS话题"""
        while not rospy.is_shutdown():
            msgs = self.can.Receive()
            for msg in msgs:
                can_id, data = msg
                frame = Frame()
                frame.id = can_id
                frame.dlc = len(data)
                frame.data = bytes(data)
                frame.header.stamp = rospy.Time.now()
                self.rx_publisher.publish(frame)
            rospy.sleep(0.001)  # 适度休眠
    
    def _tx_worker(self):
        """发送线程：从队列取出并发送CAN消息"""
        while not rospy.is_shutdown():
            try:
                frame = self.tx_queue.get(timeout=0.1)
                success = self.can.Send(
                    frame.id, 
                    list(frame.data),
                    frame.dlc,
                    'high' if frame.is_rtr else 'low'
                )
                if not success and rospy.get_param("/debug_mode", False):
                    rospy.logwarn(f"CAN发送失败: ID=0x{frame.id:03X}")
            except queue.Empty:
                pass
    
    def send_frame(self, frame):
        """发送CAN帧（非阻塞）"""
        try:
            self.tx_queue.put_nowait(frame)
            return True
        except queue.Full:
            rospy.logwarn("CAN发送队列已满")
            return False

# 单例访问函数
def get_can_manager():
    return CANManager()