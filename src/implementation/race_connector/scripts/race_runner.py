#!/usr/bin/env python3
# 依赖：paho-mqtt, rospy
# pip install paho-mqtt
import paho.mqtt.client as mqtt
import time
import json
import threading
import rospy

class TMSControllerSlave:
    def __init__(self):
        # 初始化节点
        rospy.init_node('tms_controller_slave', anonymous=False)
        
        # 从参数服务器获取配置，设置默认值
        self.address = rospy.get_param('~mqtt_address', "36.138.92.159")
        self.port = rospy.get_param('~mqtt_port', 1883)
        self.client_id = rospy.get_param('~client_id', "TMS_002")
        self.topic = rospy.get_param('~mqtt_topic', "TMS/CONTROL")
        self.computer_topic = rospy.get_param('~computer_topic', "TMS/COMPUTER")
        self.qos = rospy.get_param('~mqtt_qos', 0)
        self.retain = rospy.get_param('~mqtt_retain', False)
        self.interval = rospy.get_param('~send_interval', 1)
        
        # 看门狗配置
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 5)
        
        # 急停相关配置
        self.estop_topic = rospy.get_param('~estop_topic', "TMS/E-STOP")
        self.estop_qos = rospy.get_param('~estop_qos', 1)
        self.estop_retain = rospy.get_param('~estop_retain', True)
        self.ros_param_red_value = rospy.get_param('~ros_param_red_value', "RED")
        # 绿色触发相关配置
        self.green_topic = rospy.get_param('~green_topic', "TMS/TRIGGERGREEN")
        self.green_qos = rospy.get_param('~green_qos', 1)
        self.green_retain = rospy.get_param('~green_retain', False)
        self.ros_param_green_value = rospy.get_param('~ros_param_green_value', "GREEN")

        # ROS话题相关配置
        self.ros_param_name_=rospy.get_param('~ros_param_name',"/competition_timer") 
        
        # 状态变量
        self.last_rx_time = time.monotonic()
        self.estop_triggered = False
        self.stop_event = threading.Event()
        self.client = None
        
        # 初始化MQTT客户端
        self.init_mqtt_client()
        
        # 启动看门狗线程
        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        
        rospy.loginfo("TMS Controller Slave initialized")
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown)
    
    def init_mqtt_client(self):
        """初始化MQTT客户端并设置回调函数"""
        # 不指定callback_api_version，使用默认版本以兼容旧版本paho-mqtt
        self.client = mqtt.Client(client_id=self.client_id)
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # 连接到MQTT broker
        try:
            self.client.connect(self.address, self.port, keepalive=60)
            self.client.loop_start()
            rospy.loginfo(f"Connected to MQTT broker at {self.address}:{self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to MQTT broker: {e}")
    
    def trigger_estop(self, reason: str):
        """发布急停并标记状态"""
        if self.estop_triggered:
            return
            
        self.estop_triggered = True
        rospy.logwarn(f"[E-STOP] 触发急停，原因：{reason}")
        
        payload = json.dumps({
            "state": "TRIGGERED",
            "reason": reason,
            "from": self.client_id,
            "ts": int(time.time())
        }, ensure_ascii=False)
        
        # 设置ROS参数
        try:
            rospy.set_param(self.ros_param_name_, self.ros_param_red_value)
            rospy.logwarn(f"\033[31mSet ROS parameter {self.ros_param_name_} to {self.ros_param_red_value}\033[0m")
        except Exception as e:
            rospy.logerr(f"Failed to set ROS parameter: {e}")
        

        if self.client and self.client.is_connected():
            self.client.publish(self.estop_topic, payload, qos=self.estop_qos, retain=self.estop_retain)
    
    def trigger_green(self, reason: str):
        """发布绿色触发消息"""
        rospy.loginfo(f"[GREEN] 触发绿色触发，原因：{reason}")
        
        payload = json.dumps({
            "state": "TRIGGERED",
            "reason": reason,
            "from": self.client_id,
            "ts": int(time.time())
        }, ensure_ascii=False)
        
        # 设置ROS参数
        try:
            rospy.set_param(self.ros_param_name_, self.ros_param_green_value)
            rospy.logwarn(f"\033[32mSet ROS parameter {self.ros_param_name_} to {self.ros_param_green_value}\033[0m")
        except Exception as e:
            rospy.logerr(f"Failed to set ROS parameter: {e}")
        


        if self.client and self.client.is_connected():
            self.client.publish(self.green_topic, payload, qos=self.green_qos, retain=self.green_retain)
    
    def watchdog_loop(self):
        """监视一定时间内是否接收过任何消息"""
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            silent_time = time.monotonic() - self.last_rx_time
            if silent_time >= self.watchdog_timeout and not self.estop_triggered:
                self.trigger_estop(reason="no_rx_5s")
            time.sleep(0.2)
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT连接回调 - 使用兼容的参数格式"""
        if rc == 0:
            rospy.loginfo("Connected to MQTT broker.")
            client.subscribe(self.estop_topic, qos=self.estop_qos)
            client.subscribe(self.green_topic, qos=self.green_qos)
            client.subscribe(self.computer_topic, qos=self.qos)
            rospy.loginfo(f"Subscribed to: {self.estop_topic}, {self.green_topic}, {self.computer_topic}")
            self.last_rx_time = time.monotonic()
        else:
            rospy.logerr(f"Connect failed, rc={rc}")
    
    def on_message(self, client, userdata, msg):
        """接收消息回调"""
        try:
            payload = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            payload = str(msg.payload)
        
        if msg.topic == self.estop_topic:
            rospy.loginfo(f"[RX][E-STOP] {payload}")
            # 接收到 E-STOP 消息时，触发急停
            self.trigger_estop(reason="rx_estop")
        elif msg.topic == self.green_topic:
            rospy.loginfo(f"[RX][GREEN] {payload}")
            # 接收到 GREEN 消息时，重置倒计时并恢复发送心跳
            self.estop_triggered = False
            self.last_rx_time = time.monotonic()
            self.trigger_green(reason="rx_green")
        elif msg.topic == self.computer_topic:
            rospy.loginfo(f"[RX][COMPUTER] {payload}")
            self.last_rx_time = time.monotonic()  # 更新倒计时
        else:
            rospy.loginfo(f"[RX] topic={msg.topic} payload={payload}")
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT断开连接回调 - 使用兼容的参数格式"""
        if rc != 0:
            rospy.logwarn(f"MQTT断开连接，rc={rc}（将自动尝试重连）")
        else:
            rospy.loginfo("MQTT正常断开连接")
    
    def run(self):
        """主循环：持续发送消息"""
        rospy.loginfo(f"开始循环向 '{self.topic}' 发布 'TMS'（每 {self.interval}s）")
        
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            if self.estop_triggered:
                rospy.logdebug("[TX] 已触发 E-STOP，停止业务发送。")
            else:
                if self.client and self.client.is_connected():
                    self.client.publish(self.topic, "TMS", qos=self.qos, retain=self.retain)
                else:
                    rospy.logwarn("MQTT未连接，暂不发送消息")
            
            time.sleep(self.interval)
    
    def shutdown(self):
        """关闭节点时的清理工作"""
        rospy.loginfo("TMS Controller Slave开始关闭...")
        self.stop_event.set()
        
        if self.client:
            self.client.loop_stop()
            try:
                self.client.disconnect()
            except Exception as e:
                rospy.logerr(f"断开MQTT连接时出错：{e}")
        
        if hasattr(self, 'watchdog_thread') and self.watchdog_thread.is_alive():
            self.watchdog_thread.join(timeout=1)
        
        rospy.loginfo("TMS Controller Slave关闭完成")

if __name__ == "__main__":
    try:
        controller = TMSControllerSlave()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TMS Controller Slave被ROS中断（如Ctrl+C）")
    except Exception as e:
        rospy.logerr(f"TMS Controller Slave运行出错：{str(e)}", exc_info=True)

