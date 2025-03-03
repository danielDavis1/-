# import serial

# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# while True:
#     data = ser.read(32)  # 读取 32 字节数据
#     if data:
#         print("接收到数据: ", data.hex())



# import serial
# import struct

# # 连接 i-BUS 设备
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# def parse_ibus(data):
#     """ 解析 i-BUS 数据，返回 14 个通道值 """
#     if len(data) < 32:
#         return None
#     if data[0] != 0x20 or data[1] != 0x40:  # 检查 i-BUS 头部
#         return None
#     channels = struct.unpack('<14H', data[2:30])  # 解析 14 个通道数据
#     return channels

# while True:
#     data = ser.read(32)
#     if len(data) == 32:
#         channels = parse_ibus(data)
#         if channels:
#             print(f"通道数据: {channels}")
