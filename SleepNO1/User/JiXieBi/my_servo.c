#include "my_servo.h"

//Author: 1xx55
//os: 我发现一个事情,暂时只需要实现一条指令的发送功能就可以了。可以用电脑串口助手测试
//先使用uart1试试

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 将高地八位合成为十六位

// 数据帧格式: 
//     帧头     ID号   数据长度   指令      参数        校验和
//  0x55 0x55   ID     Length    cmd   prm1...prmN   Checksum
// 具体说明见 02+总线舵机通信协议.pdf
uint8_t buf[32];
void SERVOCMD_MOVE_TIME_WRITE(uint8_t servo_id,uint16_t arg_para,uint16_t time_para){
    // 传入舵机id编号(0-255), 角度参数(0-1000)->0-240°, 时间参数(0-30000)ms
    // 作用: 编号为id的舵机花费time_para ms时间移动到arg_para位置

    // 填充传输数据
    // 帧头
    buf[0] = SERIAL_SERVO_FRAME_HEADER; 
    buf[1] = SERIAL_SERVO_FRAME_HEADER;
    // ID号
    buf[2] = servo_id;
    // 有4个参数, 待传输数据长度为7
    buf[3] = 7;
    // 指令
    buf[4] = SERIAL_SERVO_MOVE_TIME_WRITE;
    // 参数, 依次为时间低、高8位, 角度低、高8位
    buf[5] = GET_LOW_BYTE(arg_para);
    buf[6] = GET_HIGH_BYTE(arg_para);
    buf[7] = GET_LOW_BYTE(time_para);
    buf[8] = GET_HIGH_BYTE(time_para);
    // 校验和
    buf[9] = serial_servo_checksum(buf);

    //传送
    HAL_UART_Transmit(&huart2, buf, buf[3]+3, HAL_MAX_DELAY);
    //HAL_UART_Transmit_IT(&huart2, buf, buf[3]+3);
}
