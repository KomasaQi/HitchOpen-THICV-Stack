#!/usr/bin/env python3
import rospy
import sys
import tty
import termios
import signal
import time
import select
from race_msgs.msg import Control, Longitudinal, Lateral
from std_msgs.msg import Header


class RaceSteeringTool:
    def __init__(self, topic_name=None):
        # 初始化节点
        rospy.init_node('race_steering_tool', anonymous=True)
        rospy.loginfo("Starting Race Steering Tool...")
        
        # 加载参数
        self.load_parameters()
        rospy.loginfo("Parameters loaded successfully")
        
        # 确定发布话题 (命令行参数优先于配置文件)
        if topic_name and not topic_name.startswith('__name:='):  # 过滤节点名称参数:
            self.topic_name = topic_name
        else:
            self.topic_name = self.default_topic
        
        
        # 创建发布者
        self.pub = rospy.Publisher(self.topic_name, Control, queue_size=10)
        
        # 初始化控制消息
        self.control_msg = Control()
        self.control_msg.header = Header()
        self.control_msg.longitudinal = Longitudinal()
        self.control_msg.lateral = Lateral()
        
        # 设置初始值
        self.reset_control_values()
        
        # 发布频率和周期
        self.publish_rate = rospy.Rate(self.publish_freq)
        self.period = 1.0 / self.publish_freq  # 计算周期(秒)
        self.last_control_time = time.time()  # 用于计算控制时间差
        
        # 记录上一次更新时间，用于衰减计算
        self.last_time = time.time()
        
        # 控制循环标志
        self.running = True
        
        # 记录按键状态（用于衰减机制）
        self.key_pressed = False
        self.w_pressed = False
        self.s_pressed = False
        self.a_pressed = False
        self.d_pressed = False
        
        # 注册信号处理，允许Ctrl+C直接退出
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 显示帮助信息
        self.print_help()
        rospy.loginfo(f"Publishing to topic: {self.topic_name}")
        
        # 新增：等待发布者就绪（关键修复）
        rospy.sleep(0.5)  # 等待0.5秒，确保publisher创建完成

    def load_parameters(self):
        """从参数服务器加载配置参数"""
        # 原有参数加载逻辑保留，新增以下5行：
        self.throttle_control_rate = rospy.get_param('~throttle_control_rate', 2.5)
        self.brake_control_rate = rospy.get_param('~brake_control_rate', 3.5)
        self.steering_control_rate = rospy.get_param('~steering_control_rate', 3.5)
        self.velocity_control_rate = rospy.get_param('~velocity_control_rate', 2.5)
        self.acceleration_control_rate = rospy.get_param('~acceleration_control_rate', 1.5) 
            
        # 新增：衰减延迟阈值（松开按键后，延迟多久开始衰减，单位：秒）
        self.decay_delay = rospy.get_param('~decay_delay', 0.1)  # 关键新增参数
        
        # 限制值参数
        self.max_velocity = rospy.get_param('~max_velocity', 10.0)
        self.min_velocity = rospy.get_param('~min_velocity', -5.0)
        self.max_acceleration = rospy.get_param('~max_acceleration', 2.0)
        self.min_acceleration = rospy.get_param('~min_acceleration', -2.0)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 1.0)
        self.min_steering_angle = rospy.get_param('~min_steering_angle', -1.0)
        
        # 衰减参数 (线性衰减，单位：单位值/秒)
        self.throttle_decay_rate = rospy.get_param('~throttle_decay_rate', 1.0)
        self.brake_decay_rate = rospy.get_param('~brake_decay_rate', 1.5)
        self.steering_decay_rate = rospy.get_param('~steering_decay_rate', 2.0)
        self.decay_threshold = rospy.get_param('~decay_threshold', 0.01)
        
        # 其他参数
        self.publish_freq = rospy.get_param('~publish_rate', 10.0)
        self.default_topic = rospy.get_param('~default_topic', "/race/control")
        self.default_control_mode = rospy.get_param('~default_control_mode', 0)

    def reset_control_values(self):
        """重置所有控制值为默认状态"""
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
        self.control_msg.hand_brake = False
        self.control_msg.clutch = False
        self.control_msg.steering_mode = self.control_msg.FRONT_STEERING_MODE
        self.control_msg.control_mode = self.default_control_mode  # 使用配置的默认模式

    def print_help(self):
        # 清除当前行并打印帮助信息
        print("\033c", end="")  # 清屏
        print("Race Steering Tool - 命令行控制工具")
        print("-----------------------------------")
        print(f"当前配置: 发布话题={self.topic_name}, 频率={self.publish_freq}Hz")
        print("纵向控制模式 (JKL键):")
        print("  J: 0 - 油门刹车控制 (THROTTLE_BRAKE_ONLY)")
        print("  K: 1 - 目标速度控制 (DES_SPEED_ONLY)")
        print("  L: 2 - 目标加速度控制 (DES_ACCEL_ONLY)")
        print("\n转向模式 (IO键):")
        print("  I: 前轮转向模式")
        print("  O: 双轴转向模式")
        print("\n方向控制 (WASD或方向键):")
        print("  W/上箭头: 增加速度/加速度   S/下箭头: 减少速度/加速度")
        print("  A/左箭头: 左转       D/右箭头: 右转")
        print("  Q: 急停  E: 手刹  C: 离合")

        print("\n档位控制:")
        print("  N: 空挡   1-6: 前进档1-6   R: 倒档   P: 停车档")
        print("\n其他命令:")
        print("  H: 显示帮助   X: 退出程序   Ctrl+C: 直接退出")
        print("-----------------------------------\n")


    def get_key(self):
        """快速非阻塞读取（普通键 + 方向键），支持长按，不反复切换终端模式"""
        # 首次初始化：进入 cbreak/raw，并注册退出恢复
        if not hasattr(self, "_kb_inited") or not self._kb_inited:
            self._kb_inited = True
            self._kb_fd = sys.stdin.fileno()
            try:
                self._kb_old = termios.tcgetattr(self._kb_fd)
                # 用 cbreak 比 raw 更宽容（保留部分处理，如 Ctrl+C）
                tty.setcbreak(self._kb_fd)
                import atexit
                def _restore():
                    try:
                        termios.tcsetattr(self._kb_fd, termios.TCSADRAIN, self._kb_old)
                    except Exception:
                        pass
                atexit.register(_restore)
            except Exception:
                # 若不是 TTY 或无法设置，则直接放弃读取
                return None

        # 非阻塞检测
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None

        key = sys.stdin.read(1)
        # 方向键：ESC [ A/B/C/D，共 3 字节
        if key == '\x1b':
            seq = ''
            if select.select([sys.stdin], [], [], 0)[0]:
                seq += sys.stdin.read(1)
                if select.select([sys.stdin], [], [], 0)[0]:
                    seq += sys.stdin.read(1)
            return key + seq
        return key



    def update_control(self, key, dt):
        """根据按键更新控制消息"""
        if not key:
            self.key_pressed = False
            return
            
        self.key_pressed = True
        current_mode = self.control_msg.control_mode
        
        # 纵向控制模式切换 (JKL键)
        if key == 'j':
            self.control_msg.control_mode = self.control_msg.THROTTLE_BRAKE_ONLY
            print("\033c", end="")  # 清屏
            self.print_help()       # 重打帮助
            print(f"\n已切换到纵向控制模式: 0 - 油门刹车控制")
        elif key == 'k':
            self.control_msg.control_mode = self.control_msg.DES_SPEED_ONLY
            # 切换模式时重置相关参数
            self.control_msg.throttle = 0.0
            self.control_msg.brake = 0.0
            print("\033c", end="")  # 清屏
            self.print_help()       # 重打帮助
            print(f"\n已切换到纵向控制模式: 1 - 目标速度控制")
        elif key == 'l':
            self.control_msg.control_mode = self.control_msg.DES_ACCEL_ONLY
            # 切换模式时重置相关参数
            self.control_msg.throttle = 0.0
            self.control_msg.brake = 0.0
            self.control_msg.longitudinal.velocity = 0.0
            print("\033c", end="")  # 清屏
            self.print_help()       # 重打帮助
            print(f"\n已切换到纵向控制模式: 2 - 目标加速度控制")
        
        # 转向模式切换 (IO键)
        elif key == 'i':
            self.control_msg.steering_mode = self.control_msg.FRONT_STEERING_MODE
            print("\033c", end="")  # 清屏
            self.print_help()       # 重打帮助
            print(f"\n已切换到前轮转向模式")
        elif key == 'o':
            self.control_msg.steering_mode = self.control_msg.DUAL_STEERING_MODE
            print("\033c", end="")  # 清屏
            self.print_help()       # 重打帮助
            print(f"\n已切换到双轴转向模式")
        
        # 档位控制 (N/1-6/R/P)
        elif key == 'n':
            self.control_msg.gear = self.control_msg.GEAR_NEUTRAL
        elif key in ['1', '2', '3', '4', '5', '6']:
            gear = int(key)
            self.control_msg.gear = gear
        elif key == 'r':
            self.control_msg.gear = self.control_msg.GEAR_REVERSE
        elif key == 'p':
            self.control_msg.gear = self.control_msg.GEAR_PARK
        
        # 根据当前模式处理控制指令
        if current_mode == self.control_msg.THROTTLE_BRAKE_ONLY:
            self.handle_mode_0(key, dt)
        elif current_mode == self.control_msg.DES_SPEED_ONLY:
            self.handle_mode_1(key, dt)
        elif current_mode == self.control_msg.DES_ACCEL_ONLY:
            self.handle_mode_2(key, dt)
        
        # 通用控制指令
        # 左转 (A/左箭头)
        if key == 'a' or key == '\x1b[D':
            self.control_msg.lateral.steering_angle += self.steering_control_rate * dt

                
        # 右转 (D/右箭头)
        elif key == 'd' or key == '\x1b[C':
            self.control_msg.lateral.steering_angle -= self.steering_control_rate * dt

        # 急停
        elif key == 'q':
            self.control_msg.emergency = True
            self.emergency_stop_only_clear_values()
            self.control_msg.emergency = True
            
        # 手刹
        elif key == 'e':
            self.control_msg.hand_brake = not self.control_msg.hand_brake
            
        # 离合
        elif key == 'c':
            self.control_msg.clutch = not self.control_msg.clutch
            
            
        # 退出
        elif key == 'x':
            self.running = False
            
        # 显示帮助
        elif key == 'h':
            self.print_help()
        
            

        # 限制参数范围
        self.limit_parameters()
        
        # 如果不是急停状态，重置急停标志
        if key not in ['q'] and self.control_msg.emergency:
            self.control_msg.emergency = False

    def handle_mode_0(self, key, dt):
        """处理模式0: 油门刹车控制"""
        # 确保其他模式参数为0
        self.control_msg.longitudinal.velocity = 0.0
        self.control_msg.longitudinal.acceleration = 0.0
        
        # 增加油门 (W/上箭头)
        if key == 'w' or key == '\x1b[A':
            self.control_msg.throttle = min(1.0, self.control_msg.throttle + self.throttle_control_rate * dt)
            self.control_msg.brake = 0.0
            # 如果当前是空挡或停车档，自动切换到1档
            if self.control_msg.gear in [self.control_msg.GEAR_NEUTRAL, self.control_msg.GEAR_PARK]:
                self.control_msg.gear = self.control_msg.GEAR_1
        # 增加刹车 (S/下箭头)
        elif key == 's' or key == '\x1b[B':
            self.control_msg.brake = min(1.0, self.control_msg.brake + self.brake_control_rate * dt)
            self.control_msg.throttle = 0.0

    def handle_mode_1(self, key, dt):
        """处理模式1: 目标速度控制"""
        # 确保其他模式参数为0
        self.control_msg.throttle = 0.0
        self.control_msg.brake = 0.0
        self.control_msg.longitudinal.acceleration = 0.0
        
        # 增加速度 (W/上箭头)
        if key == 'w' or key == '\x1b[A':
            self.control_msg.longitudinal.velocity += self.velocity_control_rate * dt
            # 如果当前是空挡或停车档，自动切换到1档
            if self.control_msg.gear in [self.control_msg.GEAR_NEUTRAL, self.control_msg.GEAR_PARK]:
                self.control_msg.gear = self.control_msg.GEAR_1
        # 减少速度 (S/下箭头)
        elif key == 's' or key == '\x1b[B':
            self.control_msg.longitudinal.velocity -= self.velocity_control_rate * dt

    def handle_mode_2(self, key, dt):
        """处理模式2: 目标加速度控制"""
        # 确保其他模式参数为0
        self.control_msg.throttle = 0.0
        self.control_msg.brake = 0.0
        
        # 增加加速度 (W/上箭头)
        if key == 'w' or key == '\x1b[A':
            self.control_msg.longitudinal.acceleration += self.acceleration_control_rate * dt
            # 如果当前是空挡或停车档，自动切换到1档
            if self.control_msg.gear in [self.control_msg.GEAR_NEUTRAL, self.control_msg.GEAR_PARK]:
                self.control_msg.gear = self.control_msg.GEAR_1
        # 减少加速度 (S/下箭头)
        elif key == 's' or key == '\x1b[B':
            self.control_msg.longitudinal.acceleration -= self.acceleration_control_rate * dt

    def emergency_stop_only_clear_values(self):
        """仅清零控制指令，保留模式（急停专用）"""
        self.control_msg.longitudinal.velocity = 0.0
        self.control_msg.longitudinal.acceleration = 0.0
        self.control_msg.lateral.steering_angle = 0.0
        self.control_msg.lateral.rear_wheel_angle = 0.0
        self.control_msg.throttle = 0.0
        self.control_msg.brake = 1.0  # 急停时刹车拉满
        self.control_msg.emergency = True  # 保留急停标志
        self.control_msg.clutch = False  # 急停时离合 released
        self.control_msg.gear = self.control_msg.GEAR_NEUTRAL

    def apply_decay(self, dt):
        """应用线性衰减机制（基于时间的衰减）"""
        # 新增：计算松开按键后的时间差
        time_since_release = time.time() - self.key_release_time
        time_since_release_w = time.time() - self.key_release_time_w
        time_since_release_s = time.time() - self.key_release_time_s
        time_since_release_a = time.time() - self.key_release_time_a
        time_since_release_d = time.time() - self.key_release_time_d
        # 修改：衰减触发条件（无按键 + 松开时间超过延迟阈值）
        if not self.w_pressed and time_since_release_w > self.decay_delay:
            # 油门衰减（保留原有逻辑）
            self.control_msg.throttle = max(0.0, self.control_msg.throttle - self.throttle_decay_rate * dt)
            if self.control_msg.throttle < self.decay_threshold:
                self.control_msg.throttle = 0.0
                
        if not self.s_pressed and time_since_release_s > self.decay_delay and not self.control_msg.emergency:
            # 刹车衰减（保留原有逻辑）
            self.control_msg.brake = max(0.0, self.control_msg.brake - self.brake_decay_rate * dt)
            if self.control_msg.brake < self.decay_threshold:
                self.control_msg.brake = 0.0
        # 方向衰减 a d 一个负衰减一个正衰减 
        # 方向衰减 (A/左箭头) 或者 (D/右箭头) 都没有按下，保持原有衰减逻辑
        if not self.a_pressed and not self.d_pressed and time_since_release_a > self.decay_delay and time_since_release_d > self.decay_delay:        
            # 转向衰减（保留原有逻辑）
            if abs(self.control_msg.lateral.steering_angle) > self.decay_threshold:
                decay_amount = self.steering_decay_rate * dt
                if self.control_msg.lateral.steering_angle > 0:
                    self.control_msg.lateral.steering_angle = max(0.0, self.control_msg.lateral.steering_angle - decay_amount)
                else:
                    self.control_msg.lateral.steering_angle = min(0.0, self.control_msg.lateral.steering_angle + decay_amount)
            else:
                self.control_msg.lateral.steering_angle = 0.0
        # 正方向衰减 (A/左箭头) 按下，如果方向此时为负，仍然存在这个衰减
        if self.a_pressed and time_since_release_a > self.decay_delay:
            if self.control_msg.lateral.steering_angle < 0:
                self.control_msg.lateral.steering_angle = max(0.0, self.control_msg.lateral.steering_angle + decay_amount)
        # 负方向衰减 (D/右箭头) 按下，如果方向此时为正，仍然存在这个衰减
        if self.d_pressed and time_since_release_d > self.decay_delay:
            if self.control_msg.lateral.steering_angle > 0:
                self.control_msg.lateral.steering_angle = min(0.0, self.control_msg.lateral.steering_angle - decay_amount)

        # 处理双轴转向，后轴与前轴反向等大
        if self.control_msg.steering_mode == self.control_msg.DUAL_STEERING_MODE:
            self.control_msg.lateral.rear_wheel_angle = -self.control_msg.lateral.steering_angle
            self.control_msg.lateral.rear_wheel_angle_velocity = -self.control_msg.lateral.steering_angle_velocity

        else: # 否则就将后轴转向指令置零
            self.control_msg.lateral.rear_wheel_angle = 0.0
            self.control_msg.lateral.rear_wheel_angle_velocity = 0.0
            

    def limit_parameters(self):
        """限制参数在合理范围内"""
        # 速度限制
        self.control_msg.longitudinal.velocity = max(
            self.min_velocity, min(self.max_velocity, self.control_msg.longitudinal.velocity)
        )
        
        # 加速度限制
        self.control_msg.longitudinal.acceleration = max(
            self.min_acceleration, min(self.max_acceleration, self.control_msg.longitudinal.acceleration)
        )
        
        # 转向角限制
        self.control_msg.lateral.steering_angle = max(
            self.min_steering_angle, min(self.max_steering_angle, self.control_msg.lateral.steering_angle)
        )
        
        # 油门和刹车限制（0-1）
        self.control_msg.throttle = max(0.0, min(1.0, self.control_msg.throttle))
        self.control_msg.brake = max(0.0, min(1.0, self.control_msg.brake))

    # def publish_control(self):
    #     """发布控制消息"""
    #     self.control_msg.header.stamp = rospy.Time.now()
    #     # 新增：打印发布日志（确认是否真的发布）
    #     rospy.logdebug(f"发布消息到{self.topic_name}：油门={self.control_msg.throttle:.2f}，转向角={self.control_msg.lateral.steering_angle:.2f}")
    #     self.pub.publish(self.control_msg)
        
        
    #     # 打印当前状态
    #     sys.stdout.write("\r")
    #     mode_str = {
    #         self.control_msg.THROTTLE_BRAKE_ONLY: "油门刹车",
    #         self.control_msg.DES_SPEED_ONLY: "目标速度",
    #         self.control_msg.DES_ACCEL_ONLY: "目标加速度"
    #     }.get(self.control_msg.control_mode, "未知")
        
    #     steering_mode_str = "前轮" if self.control_msg.steering_mode == self.control_msg.FRONT_STEERING_MODE else "双轴"
        
    #     # 根据当前模式显示不同的状态信息
    #     if self.control_msg.control_mode == self.control_msg.THROTTLE_BRAKE_ONLY:
    #         mode_values = f"油门: {self.control_msg.throttle:.2f} | 刹车: {self.control_msg.brake:.2f}"
    #     elif self.control_msg.control_mode == self.control_msg.DES_SPEED_ONLY:
    #         mode_values = f"目标速度: {self.control_msg.longitudinal.velocity:.2f}m/s"
    #     else:  # DES_ACCEL_ONLY
    #         mode_values = f"目标加速度: {self.control_msg.longitudinal.acceleration:.2f}m/s²"
        
    #     status_line = (f"模式: {mode_str} | 转向: {steering_mode_str} | "
    #                   f"手刹：{ '是' if self.control_msg.hand_brake else '否'} | 离合：{ '是' if self.control_msg.clutch else '否'} "
    #                   f"急停: {'是' if self.control_msg.emergency else '否'}\n"
    #                   f"转向角: {self.control_msg.lateral.steering_angle:.2f}rad | " 
    #                   f"{mode_values} | 档位: {self.get_gear_name()}")

        
    #     sys.stdout.write(status_line)
    #     sys.stdout.flush()
    def publish_control(self):
        """发布控制消息"""
        self.control_msg.header.stamp = rospy.Time.now()
        rospy.logdebug(f"发布消息到{self.topic_name}：油门={self.control_msg.throttle:.2f}，转向角={self.control_msg.lateral.steering_angle:.2f}")
        self.pub.publish(self.control_msg)
        
        # 打印当前状态（固定两行，不刷屏）
        mode_str = {
            self.control_msg.THROTTLE_BRAKE_ONLY: "油门刹车",
            self.control_msg.DES_SPEED_ONLY: "目标速度",
            self.control_msg.DES_ACCEL_ONLY: "目标加速度"
        }.get(self.control_msg.control_mode, "未知")
        
        steering_mode_str = "前轮" if self.control_msg.steering_mode == self.control_msg.FRONT_STEERING_MODE else "双轴"
        
        if self.control_msg.control_mode == self.control_msg.THROTTLE_BRAKE_ONLY:
            mode_values = f"油门: {self.control_msg.throttle:.2f} | 刹车: {self.control_msg.brake:.2f}"
        elif self.control_msg.control_mode == self.control_msg.DES_SPEED_ONLY:
            mode_values = f"目标速度: {self.control_msg.longitudinal.velocity:.2f}m/s"
        else:
            mode_values = f"目标加速度: {self.control_msg.longitudinal.acceleration:.2f}m/s²"
        
        # 构造两行状态信息（不包含\n）
        line1 = (f"模式: {mode_str} | 转向: {steering_mode_str} | "
                f"手刹：{ '是' if self.control_msg.hand_brake else '否'} | 离合：{ '是' if self.control_msg.clutch else '否'} "
                f"急停: {'是' if self.control_msg.emergency else '否'}")
        line2 = (f"转向角: {self.control_msg.lateral.steering_angle:.2f}rad | " 
                f"{mode_values} | 档位: {self.get_gear_name()}")
        
        # ANSI 转义序列：
        # \033[F  光标上移1行
        # \033[K  清除从光标到行尾的内容
        # 先清除上两行的旧内容，再打印新内容
        sys.stdout.write(f"\033[F\033[K{line1}\n\033[K{line2}\r")
        sys.stdout.flush()

    def get_gear_name(self):
        """获取档位名称"""
        if self.control_msg.gear == self.control_msg.GEAR_PARK:
            return "P"
        elif self.control_msg.gear == self.control_msg.GEAR_REVERSE:
            return "R"
        elif self.control_msg.gear == self.control_msg.GEAR_NEUTRAL:
            return "N"
        elif self.control_msg.GEAR_1 <= self.control_msg.gear <= self.control_msg.GEAR_6:
            return f"D{self.control_msg.gear}"
        return "未知"

    def signal_handler(self, sig, frame):
        """强制终止所有流程"""
        print("\n收到Ctrl+C，退出中...")
        self.running = False  # 停止主循环
        
        # 发布零值消息
        self.reset_control_values()
        self.publish_control()
        
        # 恢复终端设置
        if hasattr(self, "_kb_inited") and self._kb_inited:
            try:
                termios.tcsetattr(self._kb_fd, termios.TCSADRAIN, self._kb_old)
            except Exception:
                pass
        
        # 关闭ROS节点
        rospy.signal_shutdown("用户通过Ctrl+C退出")


    def run(self):
        """主循环"""
        rospy.loginfo("Entering main loop...")
        # 新增：记录松开按键的时间（用于衰减延迟）
        self.key_release_time = time.time()  
        self.key_release_time_w = time.time()
        self.key_release_time_s = time.time()
        self.key_release_time_a = time.time()
        self.key_release_time_d = time.time()
        try:
            while self.running and not rospy.is_shutdown():

                # 新增：计算时间差（控制速率用）
                current_time = time.time()
                dt = current_time - self.last_control_time
                self.last_control_time = current_time
                
                # 读取键盘输入
                key = self.get_key()
                    
                # 3. 修改：按键状态判断（核心修复）
                if key:
                    self.key_pressed = True  # 有按键：标记为按下
                    self.key_release_time = current_time  # 重置松开时间
                else:
                    self.key_pressed = False  # 无按键：标记为松开
                
                if key in ['w', '\x1b[A']:
                    self.w_pressed = True
                    self.key_release_time_w = current_time
                else:
                    self.w_pressed = False
                
                if key in ['s', '\x1b[B']:
                    self.s_pressed = True
                    self.key_release_time_s = current_time
                else:
                    self.s_pressed = False
                
                if key in ['a', '\x1b[D']:
                    self.a_pressed = True
                    self.key_release_time_a = current_time
                else:
                    self.a_pressed = False
                
                if key in ['d', '\x1b[C']:
                    self.d_pressed = True
                    self.key_release_time_d = current_time
                else:
                    self.d_pressed = False
                
                self.update_control(key,dt)
                
                # 应用衰减机制
                self.apply_decay(dt)
                
                # 发布消息
                self.publish_control()
                

                # 控制频率
                self.publish_rate.sleep()
        except Exception as e:
            rospy.logerr(f"程序出错: {str(e)}")
            # 出错时发布零值消息
            self.reset_control_values()
            self.publish_control()

if __name__ == '__main__':
    try:
        # 处理话题名称参数，命令行参数优先于配置文件
        topic_name = None
        if len(sys.argv) > 1:
            topic_name = sys.argv[1]
        
        tool = RaceSteeringTool(topic_name)
        tool.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"初始化失败: {str(e)}")
