#!/usr/bin/env python3
import socket
import argparse
import time
from datetime import datetime

# ---------------------- 协议定义 ----------------------
CAN_ID_FAULT = 0x207    # 故障报文ID
CAN_ID_STATUS = 0x208   # 状态报文ID
FRAME_HEADER = 0x20     # 帧信息字节（标准帧 + UDP模式）

# ---------------------- 工具函数 ----------------------
def build_can_packet(can_id, data_bytes):
    """构造符合GCAN-202协议的13字节CAN报文"""
    packet = bytearray(13)

    # 帧信息（1字节）：FF=0(标准帧), RTR=0, UDP=1, DLC=数据长度
    packet[0] = FRAME_HEADER | (len(data_bytes) & 0x0F)

    # 帧ID（4字节大端）
    packet[1:5] = can_id.to_bytes(4, byteorder='big')

    # 数据段（填充剩余字节）
    packet[5:5+len(data_bytes)] = data_bytes
    return bytes(packet)

# ---------------------- 报文生成器 ----------------------
def generate_fault_packet(encoder_fault=0, motor_fault=0, tool_fault=0):
    """生成ID 207故障报文"""
    data = bytearray(3)
    data[0] = encoder_fault & 0x0F  # 编码器故障（低4位）
    data[1] = motor_fault & 0x0F    # 电机故障
    data[2] = 1 if tool_fault else 0 # 机具故障
    return build_can_packet(CAN_ID_FAULT, data)

def generate_status_packet(enable=1, speed=0, angle=0, mode=1, estop=0):
    """生成ID 208状态报文"""
    data = bytearray(8)
    data[0] = enable & 0x01          # 使能状态
    data[1] = speed & 0xFF           # 速度（有符号字节）
    data[2:4] = angle.to_bytes(2, byteorder='big', signed=True) # 角度（大端有符号）
    data[4] = mode & 0x0F            # 模式控制
    data[7] = estop & 0x01           # 急停使能
    return build_can_packet(CAN_ID_STATUS, data)

# ---------------------- 主程序 ----------------------
def main():
    parser = argparse.ArgumentParser(description='CAN协议调试工具')
    parser.add_argument('-p', '--port', type=int, default=4002, help='监听端口')
    parser.add_argument('-t', '--target', default='127.0.0.1:8002',
                        help='发送目标地址，格式IP:端口')
    parser.add_argument('-s', '--send', choices=['207', '208'], help='发送测试报文')
    args = parser.parse_args()

    # 初始化UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port))
    target = (args.target.split(':')[0], int(args.target.split(':')[1]))

    print(f"监听端口: {args.port} | 发送目标: {target[0]}:{target[1]}")

    while True:
        # 接收线程
        try:
            data, addr = sock.recvfrom(1024)
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            hex_str = ' '.join(f'{b:02X}' for b in data)
            print(f"[{timestamp}] 来自 {addr} | {hex_str}")
        except BlockingIOError:
            pass

        # 发送测试报文
        if args.send:
            if args.send == '207':
                # 发送故障报文（示例：左前编码器故障 + 右后电机故障）
                packet = generate_fault_packet(encoder_fault=0b0001, motor_fault=0b0100)
            elif args.send == '208':
                # 发送状态报文（示例：使能 + 速度30km/h + 左转15度）
                packet = generate_status_packet(
                    enable=1,
                    speed=int(30*10),  # 速度*10
                    angle=int(-15*10)  # 角度*10（左负右正）
                )
            sock.sendto(packet, target)
            print(f"已发送 {args.send} 报文 -> {target} {' '.join(f'{b:02X}' for b in packet)}")
            time.sleep(0.5)

if __name__ == '__main__':
    main()