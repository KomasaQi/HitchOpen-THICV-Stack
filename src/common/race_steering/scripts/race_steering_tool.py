#!/usr/bin/env python3
import rospy
import sys
import tty
import termios
import signal
from race_msgs.msg import Control, Longitudinal, Lateral
from std_msgs.msg import Header

class RaceSteeringTool:
    def __init__(self, topic_name):
        # 初始化节点
        rospy.init_node('race_steering_tool', anonymous=True)
        
        # 创建发布者
        self.pub = rospy.Publisher(topic_name, Control, queue_size=10)
        
        # 初始化控制消息
        self.control_msg = Control()
        self.control_msg.header = Header()
        self.control_msg.longitudinal = Longitudinal()
        self.control_msg.lateral = Lateral()
        
        # 设置初始值
        self.control_msg.longitudinal.velocity = 0.0
        self.control_msg.longitudinal.acceleration = 0.0
        self.control_msg.longitudinal.jerk = 0.0
        self.control_msg.lateral.steering_angle = 0.0
        self.control_msg.lateral.steering_angle_velocity = 0.0
        self.control_msg.lateral.rear_wheel_angle = 0.0
        self.control_msg.lateral.rear_wheel_angle_velocity = 0.0
        self.control_msg.throttle = 0.0
        self.control_msg.brake = 0.0
        self.control_msg.gear = self.control_msg.GEAR_NEUTRAL
        self.control_msg.emergency = False
        self.control_msg.steering_mode = self.control_msg.FRONT_STEERING_MODE
        
        # 步长设置
        self.velocity_step = 0.1
        self.steering_step = 0.05
        self.throttle_step = 0.05
        self.brake_step = 0.05
        
        # 发布频率
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 控制循环标志
        self.running = True
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 显示帮助信息
        self.print_help()

    def print_help(self):
        print("\nRace Steering Tool - 命令行控制工具")
        print("-----------------------------------")
        print("方向键或WASD: 控制速度和转向")
        print("  W: 增加速度   S: 减少速度")
        print("  A: 左转       D: 右转")
        print("  Q: 急停")
        print("\n油门和刹车控制:")
        print("  上箭头: 增加油门   下箭头: 增加刹车")
        print("  [ : 减少油门       ] : 减少刹车")
        print("\n档位控制:")
        print("  N: 空挡   D: 前进档   R: 倒档   P: 停车档")
        print("\n转向模式:")
        print("  1: 前轮转向   2: 双轴转向")
        print("\n其他命令:")
        print("  H: 显示帮助   X: 退出程序")
        print("-----------------------------------")

    def get_key(self):
        """读取单个键盘输入"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def update_control(self, key):
        """根据按键更新控制消息"""
        if key == 'w':  # 增加速度
            self.control_msg.longitudinal.velocity += self.velocity_step
            self.control_msg.gear = self.control_msg.GEAR_1  # 前进档
        elif key == 's':  # 减少速度
            self.control_msg.longitudinal.velocity -= self.velocity_step
        elif key == 'a':  # 左转
            self.control_msg.lateral.steering_angle += self.steering_step
        elif key == 'd':  # 右转
            self.control_msg.lateral.steering_angle -= self.steering_step
        elif key == 'q':  # 急停
            self.control_msg.emergency = True
            self.control_msg.longitudinal.velocity = 0.0
            self.control_msg.throttle = 0.0
            self.control_msg.brake = 1.0
        elif key == 'x':  # 退出
            self.running = False
        elif key == 'h':  # 显示帮助
            self.print_help()
        elif key == '\x1b[A':  # 上箭头 - 增加油门
            self.control_msg.throttle = min(1.0, self.control_msg.throttle + self.throttle_step)
            self.control_msg.brake = 0.0
        elif key == '\x1b[B':  # 下箭头 - 增加刹车
            self.control_msg.brake = min(1.0, self.control_msg.brake + self.brake_step)
            self.control_msg.throttle = 0.0
        elif key == '[':  # 减少油门
            self.control_msg.throttle = max(0.0, self.control_msg.throttle - self.throttle_step)
        elif key == ']':  # 减少刹车
            self.control_msg.brake = max(0.0, self.control_msg.brake - self.brake_step)
        elif key == 'n':  # 空挡
            self.control_msg.gear = self.control_msg.GEAR_NEUTRAL
        elif key == 'r':  # 倒档
            self.control_msg.gear = self.control_msg.GEAR_REVERSE
        elif key == 'p':  # 停车档
            self.control_msg.gear = self.control_msg.GEAR_PARK
        elif key == '1':  # 前轮转向
            self.control_msg.steering_mode = self.control_msg.FRONT_STEERING_MODE
        elif key == '2':  # 双轴转向
            self.control_msg.steering_mode = self.control_msg.DUAL_STEERING_MODE
        
        # 限制参数范围
        self.control_msg.longitudinal.velocity = max(-5.0, min(10.0, self.control_msg.longitudinal.velocity))
        self.control_msg.lateral.steering_angle = max(-1.0, min(1.0, self.control_msg.lateral.steering_angle))
        
        # 如果不是急停状态，重置急停标志
        if key not in ['q'] and self.control_msg.emergency:
            self.control_msg.emergency = False

    def publish_control(self):
        """发布控制消息"""
        self.control_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.control_msg)
        
        # 打印当前状态
        sys.stdout.write("\r")
        sys.stdout.write(f"速度: {self.control_msg.longitudinal.velocity:.2f}m/s | "
                        f"转向角: {self.control_msg.lateral.steering_angle:.2f}rad | "
                        f"油门: {self.control_msg.throttle:.2f} | "
                        f"刹车: {self.control_msg.brake:.2f} | "
                        f"档位: {self.get_gear_name()} | "
                        f"转向模式: {self.get_steering_mode_name()} | "
                        f"急停: {'是' if self.control_msg.emergency else '否'}")
        sys.stdout.flush()

    def get_gear_name(self):
        """获取档位名称"""
        if self.control_msg.gear == self.control_msg.GEAR_PARK:
            return "P"
        elif self.control_msg.gear == self.control_msg.GEAR_REVERSE:
            return "R"
        elif self.control_msg.gear == self.control_msg.GEAR_NEUTRAL:
            return "N"
        elif self.control_msg.gear >= self.control_msg.GEAR_1:
            return f"D{self.control_msg.gear}"
        return "未知"

    def get_steering_mode_name(self):
        """获取转向模式名称"""
        if self.control_msg.steering_mode == self.control_msg.FRONT_STEERING_MODE:
            return "前轮"
        elif self.control_msg.steering_mode == self.control_msg.DUAL_STEERING_MODE:
            return "双轴"
        return "未知"

    def signal_handler(self, sig, frame):
        """处理中断信号"""
        print("\n收到退出信号，正在停止...")
        self.running = False

    def run(self):
        """主循环"""
        while self.running and not rospy.is_shutdown():
            # 读取键盘输入
            key = self.get_key()
            # 处理特殊按键（如箭头键）
            if key == '\x1b':  # 转义字符，可能是箭头键
                key += sys.stdin.read(2)
            # 更新控制消息
            self.update_control(key)
            # 发布消息
            self.publish_control()
            # 控制频率
            self.rate.sleep()
        
        # 退出前发布零值消息
        self.control_msg.longitudinal.velocity = 0.0
        self.control_msg.lateral.steering_angle = 0.0
        self.control_msg.throttle = 0.0
        self.control_msg.brake = 0.0
        self.control_msg.gear = self.control_msg.GEAR_NEUTRAL
        self.publish_control()
        print("\n程序已退出")

if __name__ == '__main__':
    try:
        # 处理话题名称参数，默认为"/race/control"
        topic_name = "/race/control"
        if len(sys.argv) > 1:
            topic_name = sys.argv[1]
        
        tool = RaceSteeringTool(topic_name)
        tool.run()
    except rospy.ROSInterruptException:
        pass
