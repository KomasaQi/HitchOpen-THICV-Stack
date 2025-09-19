#!/usr/bin/env python3
# 依赖：paho-mqtt, rospy
# pip install paho-mqtt
import paho.mqtt.client as mqtt
import time
import json
import threading
import rospy

class RaceConnector:
    def __init__(self):
        # 初始化节点
        rospy.init_node('race_connector', anonymous=False)
        
        # 从参数服务器获取配置，设置默认值
        self.address = rospy.get_param('~mqtt_address', "36.138.92.159")
        self.port = rospy.get_param('~mqtt_port', 1883)
        self.client_id = rospy.get_param('~client_id', "TMS_001")
        self.topic = rospy.get_param('~mqtt_topic', "TMS")
        self.qos = rospy.get_param('~mqtt_qos', 0)
        self.retain = rospy.get_param('~mqtt_retain', False)
        self.interval = rospy.get_param('~send_interval', 1)
        self.use_mqtt_v5 = rospy.get_param('~use_mqtt_v5', True)
        
        # 关键参数：断联时间阈值和要设置的ROS参数名称
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 5)
        self.silent_warn_threshold = 1  # 1秒未收到消息触发警告
        self.warn_interval = 0.5  # 每0.5秒打印一次警告
        self.ros_param_name = rospy.get_param('~ros_param_name', "/competition_timer")
        self.ros_param_value = rospy.get_param('~ros_param_value', "RED")
        self.ros_param_reset_value = "GREEN"  # 重连后建议设置的flag值
        
        # 急停相关配置
        self.estop_topic = rospy.get_param('~estop_topic', f"{self.topic}/E-STOP")
        self.estop_qos = rospy.get_param('~estop_qos', 1)
        self.estop_retain = rospy.get_param('~estop_retain', True)
        
        # 状态变量
        self.last_rx_time = time.monotonic()
        self.last_warn_time = 0  # 记录上一次警告时间
        self.estop_triggered = False
        self.stop_event = threading.Event()
        self.client = None
        
        # 初始化MQTT客户端
        self.init_mqtt_client()
        
        # 启动看门狗线程
        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        
        rospy.loginfo("Race Connector initialized")
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown)
    
    def init_mqtt_client(self):
        """初始化MQTT客户端并设置回调函数"""
        # 根据MQTT版本选择协议，兼容回调API版本
        if self.use_mqtt_v5:
            self.client = mqtt.Client(
                client_id=self.client_id, 
                protocol=mqtt.MQTTv5,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2
            )
            self.client.on_connect = self.on_connect_v5
        else:
            self.client = mqtt.Client(
                client_id=self.client_id, 
                protocol=mqtt.MQTTv311,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2
            )
            self.client.on_connect = self.on_connect_v311
        
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish
        self.client.on_disconnect = self.on_disconnect
        
        # 设置Last Will消息
        self.client.will_set(
            self.estop_topic,
            payload=json.dumps({
                "state": "TRIGGERED",
                "reason": "client_lost",
                "from": self.client_id,
                "ts": int(time.time())
            }, ensure_ascii=False),
            qos=self.estop_qos,
            retain=self.estop_retain
        )
        
        # 连接到MQTT broker
        try:
            self.client.connect(self.address, self.port, keepalive=60)
            self.client.loop_start()
            rospy.loginfo(f"Connected to MQTT broker at {self.address}:{self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to MQTT broker: {e}")
    
    def trigger_estop(self, reason: str):
        """发布急停并设置ROS参数"""
        if self.estop_triggered:
            return
            
        self.estop_triggered = True
        rospy.logwarn(f"[E-STOP] 未在{self.watchdog_timeout}秒内接收消息，触发急停！")
        
        # 设置ROS参数
        try:
            rospy.set_param(self.ros_param_name, self.ros_param_value)
            rospy.loginfo(f"Set ROS parameter {self.ros_param_name} to {self.ros_param_value}")
        except Exception as e:
            rospy.logerr(f"Failed to set ROS parameter: {e}")
        
        # 发布急停消息
        payload = json.dumps({
            "state": "TRIGGERED",
            "reason": reason,
            "from": self.client_id,
            "ts": int(time.time())
        }, ensure_ascii=False)
        self.client.publish(self.estop_topic, payload, qos=self.estop_qos, retain=self.estop_retain)
    
    def watchdog_loop(self):
        """监视消息接收状态，周期性打印警告"""
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            current_time = time.monotonic()
            silent_time = current_time - self.last_rx_time
            
            # 周期性打印未收到消息的警告
            if silent_time >= self.silent_warn_threshold:
                if current_time - self.last_warn_time >= self.warn_interval:
                    rospy.logwarn(f"[静默警告] 已{silent_time:.1f}秒未收到MQTT消息（主题：{self.topic}）")
                    self.last_warn_time = current_time
            
            # 触发急停（仅在未触发且超时的情况下）
            if (silent_time >= self.watchdog_timeout) and (not self.estop_triggered):
                self.trigger_estop(reason=f"no_rx_{self.watchdog_timeout}s")
            
            time.sleep(0.1)
    
    # ------------------------------ MQTT回调函数（修复参数不匹配问题） ------------------------------
    def on_connect_v5(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            rospy.loginfo("Connected to MQTT broker (MQTT v5).")
            try:
                client.subscribe(
                    self.topic, 
                    options=mqtt.SubscribeOptions(qos=self.qos, noLocal=True)
                )
                rospy.loginfo(f"Subscribed to: {self.topic} (noLocal=True)")
            except Exception as e:
                rospy.logwarn(f"Subscribe with noLocal failed, fallback to plain subscribe: {e}")
                client.subscribe(self.topic, qos=self.qos)
                rospy.loginfo(f"Subscribed to: {self.topic}")

            # client.subscribe(self.estop_topic, qos=self.estop_qos)
            # rospy.loginfo(f"Subscribed to: {self.estop_topic}")

            # 重置静默时间
            self.last_rx_time = time.monotonic()
            self.last_warn_time = time.monotonic()
        else:
            rospy.logerr(f"MQTT v5 connect failed, reason_code={reason_code}")
    
    def on_connect_v311(self, client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("Connected to MQTT broker (MQTT 3.1.1).")
            client.subscribe(self.topic, qos=self.qos)
            rospy.loginfo(f"Subscribed to: {self.topic}  [注意：会收到自己发布的消息]")
            # client.subscribe(self.estop_topic, qos=self.estop_qos)
            # rospy.loginfo(f"Subscribed to: {self.estop_topic}")
            self.last_rx_time = time.monotonic()
            self.last_warn_time = time.monotonic()
        else:
            rospy.logerr(f"MQTT 3.1.1 connect failed, rc={rc}")
    
    def on_message(self, client, userdata, msg):
        """收到消息时重置静默时间，急停后重连提示"""
        current_time = time.monotonic()
        self.last_rx_time = current_time
        
        # 急停后重新收到消息的提示
        if self.estop_triggered:
            rospy.loginfo("\033[32m[MQTT重连提示] 已重新收到MQTT消息！请手动设置比赛flag：rosparam set %s %s\033[0m" 
                          % (self.ros_param_name, self.ros_param_reset_value))
            self.estop_triggered = False
        
        # 打印消息内容
        try:
            payload = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            payload = str(msg.payload)

        if msg.topic == self.estop_topic:
            rospy.loginfo(f"[RX][E-STOP] {payload}")
        else:
            rospy.loginfo(f"[RX] topic={msg.topic} qos={msg.qos} payload={payload}")
    
    def on_publish(self, client, userdata, mid, reason_code, properties):
        """修复参数不匹配：添加reason_code参数"""
        rospy.logdebug(f"[TX-ACK] 消息发布成功，mid={mid}, reason_code={reason_code}")
    
    def on_disconnect(self, client, userdata, rc, reason_code, properties):
        """修复参数不匹配：添加reason_code参数"""
        if rc != 0 or reason_code != 0:
            rospy.logwarn(f"MQTT断开连接，rc={rc}, reason_code={reason_code}（将自动尝试重连）")
        else:
            rospy.loginfo("MQTT正常断开连接")
    
    # ------------------------------ 主循环与清理 ------------------------------
    def run(self):
        """主循环：持续发送消息"""
        rospy.loginfo(f"开始循环向 '{self.topic}' 发布消息（发送周期：{self.interval}s）")
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            if self.estop_triggered:
                time.sleep(0.5)
                continue
            
            if self.client and self.client.is_connected():
                self.client.publish(self.topic, "TMS", qos=self.qos, retain=self.retain)
            else:
                rospy.logwarn("MQTT未连接，暂不发送消息")
            
            time.sleep(self.interval)
    
    def shutdown(self):
        """关闭节点时的清理工作"""
        rospy.loginfo("Race Connector开始关闭...")
        self.stop_event.set()
        if self.client:
            self.client.loop_stop()
            try:
                self.client.disconnect()
            except Exception as e:
                rospy.logerr(f"断开MQTT连接时出错：{e}")
        if self.watchdog_thread.is_alive():
            self.watchdog_thread.join(timeout=1)
        rospy.loginfo("Race Connector关闭完成")

if __name__ == "__main__":
    try:
        connector = RaceConnector()
        connector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Race Connector被ROS中断（如Ctrl+C）")
    except Exception as e:
        rospy.logerr(f"Race Connector运行出错：{str(e)}", exc_info=True)
