import pygame
import serial
from pygame.locals import *

COMNAME = 'COM4'

ser = serial.Serial(COMNAME,115200)

if not ser.isOpen():
    print(f"未打开串口{COMNAME}!")
    exit(0)


def play(para):
    global ser
    if para == 'MOVE_AHEAD':
        data_to_send = b'\x66\x66\x02\x00\x00\x30'
    if para == 'MOVE_BACK':
        data_to_send = b'\x66\x66\x02\x00\xb4\x30'
    if para == 'MOVE_LEFT':
        data_to_send = b'\x66\x66\x02\x00\x5a\x30'
    if para == 'MOVE_RIGHT':
        data_to_send = b'\x66\x66\x02\x01\x0e\x30'
    if para == 'MOVE_CW':
        data_to_send = b'\x66\x66\x03\x00\x5a'
    if para == 'MOVE_CCW':
        data_to_send = b'\x66\x66\x03\x01\x0e'
    if para == 'JIAQU':
        data_to_send = b'\x66\x66\x05'
    if para == 'TING':
        data_to_send = b'\x66\x66\x07'
    print(f"send:{data_to_send}")
    ser.write(data_to_send)

pygame.init()

# 设置屏幕大小
screen = pygame.display.set_mode((640, 480))

keylist = [K_UP,K_DOWN,K_LEFT,K_RIGHT,K_SPACE,K_q,K_e]
stats = {}
playstr = {K_UP:'MOVE_AHEAD',K_DOWN:'MOVE_BACK',K_LEFT:'MOVE_LEFT',K_RIGHT:'MOVE_RIGHT',
           K_SPACE:'JIAQU',K_q:'MOVE_CW',K_e:'MOVE_CCW'}
for each in keylist:
    stats[each] = 0

# 游戏循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    # 获取按键状态
    keys = pygame.key.get_pressed()

    for each in keylist:
        if (not keys[each]) and stats[each]:
            play('TING')
        if keys[each] and not stats[each]:
            play(playstr[each])
        stats[each] = keys[each]
        
    # 检测按键映射
    '''
    if keys[K_UP]:
        print("上键被按下")
    if keys[K_DOWN]:
        print("下键被按下")
    if keys[K_LEFT]:
        print("左键被按下")
    if keys[K_RIGHT]:
        print("右键被按下")
    if keys[K_SPACE]:
        print("空格键被按下")
    if keys[K_e]:
        pass
    if keys[K_q]:
        pass   
    '''
    # 更新屏幕
    pygame.display.update()

# 退出pygame
pygame.quit()
    
