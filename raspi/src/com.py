# import serial
# import struct 
# def Port_Write(ser, char): 
# 	PackA=struct.pack('c',char)
#     Bytesnum=ser.write(PackA)
    
# Port_Write(ser, b'a')
# def serialinit():
#     try:
#         ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)
#     except:
#         ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=0.5)
#     ser.bytesize = 8
#     print(ser)
#     return ser
# def Port_receive(ser):
# 	return ser.readline()
# ser.write(b'helloworld') 
# #或者 
# ser.write('helloworld'.encode('utf-8'))

import serial
import time
import struct 
import get_theta
import struct
def Port_Write(ser, char): 
    PackA=struct.pack('c',char)
    Bytesnum=ser.write(PackA) 
        
# Port_Write(ser, b'a')
def serialinit():
    try:
        ser = serial.Serial(
    port='/dev/ttyUSB0',  # 树莓派的UART接口
    baudrate=115200,        # 波特率
    bytesize=serial.EIGHTBITS,  # 数据位
    parity=serial.PARITY_NONE,  # 无校验位
    stopbits=serial.STOPBITS_ONE,  # 停止位
    timeout=0.5                  # 超时时间
)
        print("usb0")
    except:
        ser = serial.Serial(
    port='/dev/ttyUSB1',  # 树莓派的UART接口
    baudrate=115200,        # 波特率
    bytesize=serial.EIGHTBITS,  # 数据位
    parity=serial.PARITY_NONE,  # 无校验位
    stopbits=serial.STOPBITS_ONE,  # 停止位
    timeout=1                   # 超时时间
)   
        file = open("example.txt", "a")

# 向文件中写入内容
        file.write("usb1!")

# 关闭文件
        file.close()
    
    return ser
def Port_receive(ser):
	return ser.readline()

ser=serialinit()
#ser.write(b'\x66\x66\x02')
cnt=1

ser.write(b'\x66\x66\x06')
print("init")

# try:
    
#     # 向单片机发送数据
#     ser.write(b'Hello, STM32!')
#     print('Data sent to STM32.')

#     # 延时等待单片机响应
#     time.sleep(1)

#     # 从单片机接收数据
#     data = ser.readline()
#     print('Received from STM32: ', data)

# finally:
#     ser.close()  # 关闭串口

# import serial
# import struct

# def Port_Write(ser, char, angle):
#     if 10 <= angle <= 90:
#         # 发送两个字节的0x66
#         PackA = struct.pack('BB', 0x66, 0x66)
#         Bytesnum = ser.write(PackA)
        
#         # 发送angle值
#         PackB = struct.pack('B', angle)
#         Bytesnum = ser.write(PackB)
#     else:
#         print("Angle must be between 10 and 90")

# # 打开串行端口
# ser = serial.Serial('/dev/ttyUSB0', 9600)

# # 调用Port_Write函数发送数据
# Port_Write(ser, 'a', 50)

# # 关闭串行端口
# ser.close()
