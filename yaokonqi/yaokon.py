#!/usr/bin/env python3
import serial
import struct
import uinput
import time

# 创建虚拟手柄设备
device = uinput.Device([
    uinput.ABS_X + (-32768, 32767, 0, 0),  # 左摇杆横向
    uinput.ABS_Y + (-32768, 32767, 0, 0),  # 左摇杆纵向
    uinput.ABS_RX + (-32768, 32767, 0, 0), # 右摇杆横向
    uinput.ABS_RY + (-32768, 32767, 0, 0), # 右摇杆纵向
    uinput.BTN_A,  # SWA
    uinput.BTN_B,  # SWB
    uinput.BTN_X,  # SWC
    uinput.BTN_Y,  # SWD
    uinput.BTN_TL, # 按键A
    uinput.BTN_TR, # 按键B
])

# 重新打开串口的函数
def reopen_serial():
    global ser
    try:
        ser.close()
    except:
        pass
    time.sleep(1)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

# 打开串口（非阻塞模式，避免 timeout 问题）
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

def parse_ibus(data):
    """解析 i-BUS 数据包，返回 14 个通道的数值"""
    if len(data) < 32:
        return None
    if data[0] != 0x20 or data[1] != 0x40:
        return None
    return struct.unpack('<14H', data[2:30])

def map_axis(value, deadzone=10):
    """将 i-BUS 数值 [1000,2000] 映射到 [-32768,32767]"""
    deviation = value - 1500
    if abs(deviation) < deadzone:
        return 0
    norm = deviation / 500.0
    return int(norm * 32767)

def map_button(value, threshold=1750):
    """将 i-BUS 数值映射到按键状态"""
    return 1 if value > threshold else 0

buffer = b""  # 维护数据缓冲区，防止数据包错位

while True:
    try:
        # 读取所有可用数据
        buffer += ser.read(ser.in_waiting)

        # 确保至少有 32 字节数据
        while len(buffer) >= 32:
            # 查找数据头 0x20 0x40
            start_idx = buffer.find(b'\x20\x40')
            if start_idx == -1:
                buffer = buffer[-31:]  # 保留最近的 31 字节，避免数据完全丢失
                break  # 退出本次循环，等待更多数据
            
            # 取 32 字节的完整数据包
            if start_idx + 32 <= len(buffer):
                packet = buffer[start_idx:start_idx + 32]
                buffer = buffer[start_idx + 32:]  # 移除已处理的数据
                channels = parse_ibus(packet)
                
                if channels:
                    # 解析摇杆数据
                    right_x = map_axis(channels[0])  # 通道1
                    right_y = -map_axis(channels[1]) # 通道2（取反）
                    left_y  = -map_axis(channels[2]) # 通道3（取反）
                    left_x  = map_axis(channels[3])  # 通道4
                    
                    device.emit(uinput.ABS_RX, right_x, syn=False)
                    device.emit(uinput.ABS_RY, right_y, syn=False)
                    device.emit(uinput.ABS_X, left_x, syn=False)
                    device.emit(uinput.ABS_Y, left_y, syn=True)
                    
                    # 解析按键数据
                    device.emit(uinput.BTN_A, map_button(channels[4]), syn=False)  # SWA
                    device.emit(uinput.BTN_B, map_button(channels[5]), syn=False)  # SWB
                    device.emit(uinput.BTN_X, map_button(channels[6]), syn=False)  # SWC
                    device.emit(uinput.BTN_Y, map_button(channels[7]), syn=False)  # SWD
                    device.emit(uinput.BTN_TL, map_button(channels[8]), syn=False) # 按键A
                    device.emit(uinput.BTN_TR, map_button(channels[9]), syn=True)  # 按键B

    except serial.SerialException:
        print("⚠️ 串口错误，正在重启...")
        reopen_serial()

    except Exception as e:
        print(f"⚠️ 发生错误: {e}")

    time.sleep(0.005)  # 5ms 低延迟更新
